//---------------------------------------------------------------------------
// stereo_fusion_top.v  --  v3 (targets DRM<=35 / APM<=70 on PGL50H)
//
//   clk    = pclk4x (148.5 MHz), pix_en = one clock pulse per pixel (ph0)
//   input  : 720p scan timing + YUV444, stereo pair in the BOTTOM half
//            (hc 0..639 = physical right eye, hc 640..1279 = physical left eye)
//   output : the fused 670 x 292 image, tagged with its own hc/vc.
//
// Line schedule (t = hc*4 + ph, clocks from the start of the line):
//   t     0 ..  639 : shared mapgen burst LEFT  eye -> map buffer L
//   t   640 ..  648 : drain gap
//   t   656 .. 1295 : shared mapgen burst RIGHT eye -> map buffer R
//   t  1328 ..      : 4-point read / remap / blend for canvas cols 0..669
//                     (pixel slot 332 .. 1001, one pixel per 4 clocks)
//
// The 332-pixel offset buys two things:
//   * the whole row's map is ready before the first read,
//   * the current row (vc) is written far enough ahead that, for the right eye,
//     every needed source column is already in the line cache.
//
// Output row mapping: foy = vc - ROI_VC - DLY, produced during line vc and
// displayed during line vc+1 at rows 393..684.
//---------------------------------------------------------------------------
`include "params.vh"
`include "map_params.vh"

module stereo_fusion_top (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        pix_en,
    input  wire        de,
    input  wire        hs,
    input  wire        vs,
    input  wire [10:0] hc,
    input  wire [9:0]  vc,
    input  wire [23:0] yuv_in,
    output wire        de_out,
    output wire [15:0] rgb565_out,
    output wire [23:0] rgb888_out,
    output wire        out_valid,
    output wire [10:0] out_hc,
    output wire [9:0]  out_vc
);
    // ================= phase counter ========================
    // v4: 时钟降到 74.25MHz（pclk x2），每个像素 2 个相位
    reg ph_r;
    always @(posedge clk) begin
        if (!rst_n)      ph_r <= 1'b0;
        else if (pix_en) ph_r <= 1'b0;
        else             ph_r <= 1'b1;
    end
    wire [11:0] t = {hc, ph_r};

    // ================= 1. YUV444 -> RGB565 =================
    //   v4 时序修复：行缓存读出(DRAM Tco 2.3ns + 走线 1.7ns) 与乘法器/减法链/
    //   饱和打包串在一拍实测 10.5ns，超 clk_74_25M 的 13.468ns。
    //   u_csc 内部改成两级 pix_en 流水，两级都用 pix_en 使能 => 每级正好
    //   1 个像素 slot(4 拍)，总延迟 2 slot，pix_en 的 4 拍网格不变，
    //   因此下游 line_bank / remap / blend / compositor 的使能相位都不用动。
    wire        de_c;
    wire [15:0] rgb_c;
    yuv444_to_rgb565 u_csc (
        .clk(clk), .rst_n(rst_n), .pix_en(pix_en),
        .de_in(de), .yuv_in(yuv_in),
        .de_out(de_c), .rgb565_out(rgb_c));

    // ================= 2. ROI gate =========================
    //   坐标必须与像素严格配对：u_csc 延后 2 个 slot，这里同样用 2 级
    //   pix_en 延时（每级 = 1 slot）。原来是 1 级自由跑(1 拍)，与 u_csc 的
    //   1 级流水对齐；现在两者都改成 2 级 pix_en。
    reg [10:0] hc_d, hc_d2;
    reg [9:0]  vc_d, vc_d2;
    always @(posedge clk) begin
        if (!rst_n) begin
            hc_d  <= 11'd0;  hc_d2 <= 11'd0;
            vc_d  <= 10'd0;  vc_d2 <= 10'd0;
        end else if (pix_en) begin
            hc_d  <= hc;     vc_d  <= vc;
            hc_d2 <= hc_d;   vc_d2 <= vc_d;
        end
    end

    wire        r_valid, l_valid;
    wire [9:0]  sx;
    wire [8:0]  sy;
    wire [15:0] rgb_roi;
    stereo_roi_gate #(.W(`IMG_W), .H(`IMG_H), .ROI_VC(`ROI_VC), .SPLIT_HC(`SPLIT_HC))
        u_gate (
        .de(de_c), .hc(hc_d2), .vc(vc_d2), .rgb(rgb_c),
        .r_valid(r_valid), .l_valid(l_valid), .sx(sx), .sy(sy), .rgb_out(rgb_roi));

    // ================= 3. output row of this line ===========
    wire [9:0]  foy     = vc - (`ROI_VC + `DLY);
    wire        prod_ok = (vc >= (`ROI_VC + `DLY + `FOY_LO)) &&
                          (vc <= (`ROI_VC + `DLY + `FOY_LO + `OUT_H - 1));

    // ================= 4. map engines (just-in-time, no row buffer) =========
    // Both engines are fed 3 pixel slots ahead of the read so that their
    // output is ready exactly at the ph0 of the pixel being read.
    //
    // v5 (2026-10) 时序修复（输入侧）：
    //   hc 是 clk_37_125M 域寄存器（line_buf 输出），直接串进 74.25M 域的乘法器
    //   会形成跨域长路径，因此 ox / en 各打一拍。
    //
    // v6: 时钟降到 74.25MHz（pclk x2）后，每像素只有 2 拍，
    //   mapgen 流水 11 拍 = 5.5 个像素，所以 feed 必须提前 6 个像素：
    //   读 slot S 用 fox = S-332，feed 时 ox = s-326  =>  s = S-6  (提前 6 像素)
    //   输出在 s+11 拍(=s+5.5 像素) 就绪，比读点 S 早 0.5 像素，采样安全。
    wire        en_map = pix_en && (hc >= 11'd326) && (hc <= 11'd995);
    wire [9:0]  mox    = hc[9:0] - 10'd326;
    wire [9:0]  mox_r  = (hc[9:0] >= 10'd356) ? (hc[9:0] - 10'd356) : 10'd0;

    reg  [9:0]  mox_q, mox_r_q;
    reg         en_map_q;
    always @(posedge clk) begin
        if (pix_en) begin
            mox_q   <= mox;
            mox_r_q <= mox_r;
        end
        en_map_q <= en_map;          // 1 拍脉冲，延迟一拍后送 mapgen
    end

    wire        dv_l, dv_r;
    wire [17:0] gmx_l, gmy_l, gmx_r, gmy_r;

    mapgen u_map_l (
        .clk(clk), .rst_n(rst_n), .en(en_map_q), .oy(foy[8:0]), .ox(mox_q),
        .c_invfx(`MAP_L_INVFX), .c_invfy(`MAP_L_INVFY),
        .c_offx(`MAP_L_OFFX),   .c_offy(`MAP_L_OFFY),
        .c_mfx(`MAP_L_MFX), .c_mfy(`MAP_L_MFY), .c_mcx(`MAP_L_MCX), .c_mcy(`MAP_L_MCY),
        .c_k1(`MAP_L_K1), .c_k2(`MAP_L_K2), .c_p1(`MAP_L_P1), .c_p2(`MAP_L_P2),
        .c_r00(`MAP_L_R00), .c_r01(`MAP_L_R01), .c_r02(`MAP_L_R02),
        .c_r10(`MAP_L_R10), .c_r11(`MAP_L_R11), .c_r12(`MAP_L_R12),
        .c_r20(`MAP_L_R20), .c_r21(`MAP_L_R21), .c_r22(`MAP_L_R22),
        .dvalid(dv_l), .mapx(gmx_l), .mapy(gmy_l));

    mapgen u_map_r (
        .clk(clk), .rst_n(rst_n), .en(en_map_q), .oy(foy[8:0]), .ox(mox_r_q),
        .c_invfx(`MAP_R_INVFX), .c_invfy(`MAP_R_INVFY),
        .c_offx(`MAP_R_OFFX),   .c_offy(`MAP_R_OFFY),
        .c_mfx(`MAP_R_MFX), .c_mfy(`MAP_R_MFY), .c_mcx(`MAP_R_MCX), .c_mcy(`MAP_R_MCY),
        .c_k1(`MAP_R_K1), .c_k2(`MAP_R_K2), .c_p1(`MAP_R_P1), .c_p2(`MAP_R_P2),
        .c_r00(`MAP_R_R00), .c_r01(`MAP_R_R01), .c_r02(`MAP_R_R02),
        .c_r10(`MAP_R_R10), .c_r11(`MAP_R_R11), .c_r12(`MAP_R_R12),
        .c_r20(`MAP_R_R20), .c_r21(`MAP_R_R21), .c_r22(`MAP_R_R22),
        .dvalid(dv_r), .mapx(gmx_r), .mapy(gmy_r));

    reg [17:0] mx_l, my_l, mx_r, my_r;
    always @(posedge clk) begin
        if (pix_en) begin
            mx_l <= gmx_l; my_l <= gmy_l;
            mx_r <= gmx_r; my_r <= gmy_r;
        end
    end
    // ================= 5. line banks (K=24) ================
    wire        qv_l, qv_r, qok_l, qok_r;
    wire [15:0] l00, l01, l10, l11;
    wire [15:0] r00, r01, r10, r11;

    wire        pinv_l = (mx_l == 18'h3FFFF) || (my_l == 18'h3FFFF);
    wire        pinv_r = (mx_r == 18'h3FFFF) || (my_r == 18'h3FFFF);
    wire [9:0]  lx0 = pinv_l ? 10'd0 : mx_l[17:8];
    wire [8:0]  ly0 = pinv_l ? 9'd0  : my_l[17:8];
    wire [9:0]  rx0 = pinv_r ? 10'd0 : mx_r[17:8];
    wire [8:0]  ry0 = pinv_r ? 9'd0  : my_r[17:8];

    wire        rd_valid = de && prod_ok &&
                           (hc >= `READ_SLOT) &&
                           (hc <  (`READ_SLOT + `OUT_W));

    line_bank #(.W(`IMG_W), .K(`LINE_DEPTH)) u_bank_l (
        .clk(clk), .rst_n(rst_n), .pix_en(pix_en),
        .wr_valid(l_valid), .wr_sy(sy), .wr_sx(sx), .wr_data(rgb_roi),
        .in_valid(rd_valid), .ry0(ly0), .rx0(lx0), .src_ok(~pinv_l),
        .q_valid(qv_l), .q00(l00), .q01(l01), .q10(l10), .q11(l11),
        .q_src_ok(qok_l));

    line_bank #(.W(`IMG_W), .K(`LINE_DEPTH)) u_bank_r (
        .clk(clk), .rst_n(rst_n), .pix_en(pix_en),
        .wr_valid(r_valid), .wr_sy(sy), .wr_sx(sx), .wr_data(rgb_roi),
        .in_valid(rd_valid), .ry0(ry0), .rx0(rx0), .src_ok(~pinv_r),
        .q_valid(qv_r), .q00(r00), .q01(r01), .q10(r10), .q11(r11),
        .q_src_ok(qok_r));

    // ================= 6. bilinear remap ===================
    wire        vr_o, vl_o;
    wire [15:0] pl, pr;

    remap_core u_rm_l (
        .clk(clk), .rst_n(rst_n),
        .valid_in(qv_l), .q00(l00), .q01(l01), .q10(l10), .q11(l11),
        .fx(mx_l[7:0]), .fy(my_l[7:0]),
        .valid_in_src(qok_l), .valid_out(vl_o), .rgb_out(pl));

    remap_core u_rm_r (
        .clk(clk), .rst_n(rst_n),
        .valid_in(qv_r), .q00(r00), .q01(r01), .q10(r10), .q11(r11),
        .fx(mx_r[7:0]), .fy(my_r[7:0]),
        .valid_in_src(qok_r), .valid_out(vr_o), .rgb_out(pr));

    // ================= 7. blend ============================
    reg [9:0]  wcnt;            // canvas column = index of the pixel being blended
    wire       blend_v;
    wire [15:0] blend_rgb;

    stitch_blend u_blend (
        .clk(clk), .rst_n(rst_n),
        .valid(vl_o), .ox(wcnt),
        .pix_left(pl), .pix_right(pr),
        .left_ok(wcnt < `IMG_W),
        .right_ok(wcnt >= `STITCH_SHIFT),
        .valid_out(blend_v), .pix_out(blend_rgb));

    always @(posedge clk) begin
        if (!rst_n)                           wcnt <= 10'd0;
        else if (pix_en && (hc == 11'd0))     wcnt <= 10'd0;
        else if (blend_v && (wcnt < `OUT_W))  wcnt <= wcnt + 10'd1;
    end

    // display-row index of the row being produced (parity must match the
    // read side, which selects with (vc - VC_MIN)[0])
    wire [9:0] foy_rel = foy - `FOY_LO;

    // ================= 8. fused row buffer + read-out ======
    output_compositor u_comp (
        .clk(clk), .rst_n(rst_n), .pix_en(pix_en),
        .wr_en(blend_v && prod_ok),
        .wr_addr(wcnt),
        .wr_data(blend_rgb),
        // buffer parity must use the DISPLAY row index (foy - FOY_LO), not foy:
        // the read side selects with (vc - VC_MIN)[0], and those two differ
        // whenever FOY_LO is odd.
        .wr_sel(foy_rel[0]),
        .de(de), .hc(hc), .vc(vc),
        .de_out(de_out),
        .out_valid(out_valid), .out_hc(out_hc), .out_vc(out_vc),
        .rgb565_out(rgb565_out), .rgb888_out(rgb888_out));

endmodule
