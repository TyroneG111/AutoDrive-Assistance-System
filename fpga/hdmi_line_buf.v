`timescale 1ns/1ps
//=============================================================================
// hdmi_line_buf —— 4 页“就绪页队列”显示行缓存
//
// 作用：把读出器产生的 256bit 字流（一显示行 = 80 个 256bit 字 = 1280 像素）
//       变成 DMT 时序驱动的 16bit/clk 像素流。
//
// 【为什么用“N 页队列”而不是 2 页乒乓】
//   2 页乒乓在“垂直消隐期换页”上存在歧义（消隐期不消费页，但读出器又必须
//   在下一帧第一行之前把数据备好），相位很难自洽。改成 4 页“就绪页队列”后：
//     · 读出器只管把一页写满 -> 标记 ready -> 顺次占用下一页；最多领先 3 行
//     · 显示只在“有效行”开始时顺次取用下一页
//     · 消隐期读出器可提前把下一帧前 3 行备好，天然吸收帧边界
//   没有任何“帧边界特例”，逻辑自洽且便于验证。
//
// 【不黑屏保证】
//   显示永远只读“写满了一整行”的页；若下一行还没 ready，就重复当前页
//   （1 行冻结），绝不输出半行、绝不输出空白。
//
// 【读出延迟对齐】
//   读口打 1 拍：本拍给地址 mem[rp*80 + (hc>>4)]，下拍数据可用，用 hc_d[3:0]
//   选 16bit 切片。因此输出较 hc/de/hs/vs 延迟 1 拍，本模块内部同时把
//   de/hs/vs 也延迟 1 拍，保证像素与同步严格对齐。
//
// 【页切换时刻】在每行最后一个 pclk（hc == H_TOTAL-1）决定下一有效行读哪一页，
//   这样下一行 hc=0 那拍给出的读地址就已经是新页（读地址是组合的）。
//=============================================================================
module hdmi_line_buf #(
    parameter NPIX    = 1280,               // 一显示行像素数
    parameter WPCNT   = 160,                // 一显示行的 256bit 字数（= NPIX/8）
    parameter NPAGE   = 4,                  // 页数
    parameter H_TOTAL = 1650,
    parameter V_ACT   = 720,
    parameter V_TOTAL = 750
)(
    input  wire         pclk,
    input  wire         rst_n,
    // ---- 写口：来自 rd_line_gen（一显示行 80 个字） ----
    input  wire         wr_vld,
    input  wire [255:0] wr_word,
    input  wire         line_end,           // 读出器本显示行写完（1 拍，已含缩放器尾拍）
    // ---- 时序：来自 dmt_tmg_gen ----
    input  wire [10:0]  hc,
    input  wire [9:0]   vc,
    input  wire         de,
    input  wire         hs,
    input  wire         vs,
    // ---- 输出（较 hc/de/hs/vs 延迟 1 拍） ----
    output wire         o_de,
    output wire         o_hs,
    output wire         o_vs,
    output wire [23:0]  o_px,
    output wire [10:0]  o_hc,               // 与 o_px 同拍的行内像素号（= hc 延 1 拍）
    output wire [9:0]   o_vc,               // 与 o_px 同拍的行号（= vc 延 1 拍）
    // ---- 与读出器握手 ----
    output wire         line_ok,            // 电平：读出器可以开始写下一显示行
    output wire         line_rdy,           // 至少有一页已就绪（供顶层生成 run）
    // ---- 调试 ----
    output wire [1:0]   wp_dbg,
    output wire [1:0]   rp_dbg,
    output wire [3:0]   rdy_dbg,
    output wire [6:0]   cnt_dbg
);
    localparam DEPTH = NPAGE*WPCNT;         // 640 个 256bit 字

    reg [255:0] mem [0:DEPTH-1];

    reg [1:0]   wp;                         // 写页（读出器正在写）
    reg [1:0]   rp;                         // 读页（显示正在读）
    reg [3:0]   rdy;                        // 各页“已写满、待显示”
    reg [7:0]   cnt;                        // 当前写页已写入字数 0..WPCNT
    reg         le_pend;                    // 本页已收到字、但 line_end 还没到
    reg [10:0]  hc_d;
    reg [9:0]   vc_d;
    reg         de_d, hs_d, vs_d;
    reg [255:0] rdata_w;

    wire [1:0]  wp_nxt = wp + 2'd1;
    wire [1:0]  rp_nxt = rp + 2'd1;

    // 页基址（常数 0/160/320/480，避免乘法器）
    //   ★必须 ≥10bit：DEPTH=640，页 3 的 480+cnt(>=0) 需 10bit
    wire [9:0]  wbase = (wp == 2'd0) ? 10'd0   :
                        (wp == 2'd1) ? 10'd160 :
                        (wp == 2'd2) ? 10'd320 : 10'd480;
    wire [9:0]  rbase = (rp == 2'd0) ? 10'd0   :
                        (rp == 2'd1) ? 10'd160 :
                        (rp == 2'd2) ? 10'd320 : 10'd480;

    wire [7:0]  woff  = (hc < NPIX[10:0]) ? hc[10:3] : 8'd0;
    wire        wr_ok = ~rdy[wp] & (wp != rp);

    //-------------------------------------------------------------------------
    // 页管理
    //-------------------------------------------------------------------------
    // “下一行是有效行” -> 本行末需要取用下一页
    wire last_cyc    = (hc == H_TOTAL[10:0] - 11'd1);
    wire next_active = (vc == V_TOTAL[9:0] - 10'd1) | ({1'b0, vc} < (V_ACT[9:0] - 10'd1));
    wire take        = last_cyc & next_active & rdy[rp_nxt];

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            wp  <= 2'd0;
            rp  <= 2'd3;                    // 使 rp_nxt = 0，与首个写页对齐
            rdy <= 4'd0;
            cnt <= 8'd0;
            le_pend <= 1'b0;
        end else begin
            // ---- 写一侧：只计数；【页边界由 line_end 决定】 ----
            //   ★若按“字数到 WPCNT”完成，一旦有半页残留（丢字/抢拍），wp 会在行中途跳页，
            //     后续字会写进可能正被显示占用（wp==rp）的页 -> 被 wr_ok 丢掉一半。
            if (wr_vld & wr_ok) begin
                cnt     <= cnt + 8'd1;
                le_pend <= 1'b1;
            end
            if (line_end & wr_ok) begin
                rdy[wp] <= 1'b1;
                wp      <= wp_nxt;
                cnt     <= 8'd0;
                le_pend <= 1'b0;
            end
            // ---- 读一侧：有效行开始处顺次消费下一页 ----
            if (take) begin
                rdy[rp_nxt] <= 1'b0;
                rp          <= rp_nxt;
            end
        end
    end

    always @(posedge pclk) begin
        if (wr_vld & wr_ok) mem[wbase + {2'b00, cnt}] <= wr_word;
    end
    //-------------------------------------------------------------------------
    // 读口（1 拍延迟）+ 同步信号对齐
    //-------------------------------------------------------------------------
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            hc_d    <= 11'd0;
            vc_d    <= 10'd0;
            de_d    <= 1'b0;
            hs_d    <= 1'b0;
            vs_d    <= 1'b0;
            rdata_w <= 256'd0;
        end else begin
            hc_d    <= hc;
            vc_d    <= vc;
            de_d    <= de;
            hs_d    <= hs;
            vs_d    <= vs;
            rdata_w <= mem[rbase + {1'b0, woff}];
        end
    end

    assign o_de = de_d;
    assign o_hs = hs_d;
    assign o_vs = vs_d;
    assign o_hc = hc_d;
    assign o_vc = vc_d;
    // 像素 k（k = hc_d[2:0]）处于 256bit 字的第 k 个 32bit 通道，取低 24bit
    assign o_px = rdata_w[{hc_d[2:0], 5'b00000} +: 24];

    // 上页未完成之前不允许开新行（也防读出器提前插进下一行写同一页）
    assign line_ok  = wr_ok & ~le_pend;
    assign line_rdy = |rdy;

    assign wp_dbg  = wp;
    assign rp_dbg  = rp;
    assign rdy_dbg = rdy;
    assign cnt_dbg = cnt;

endmodule
