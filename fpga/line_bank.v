//---------------------------------------------------------------------------
// line_bank.v  --  one eye, K=24 ring line cache, TRUE-DUAL-PORT, 2 phases
//
//   K=24 -> 12 even slot rows in mem_e, 12 odd slot rows in mem_o
//        12 x 640 = 7680 words each -> ~7 DRM18K per array
//        -> ~14 DRM per eye -> ~28 for both eyes (same as the single-array ver)
//
// Each array is a true dual-port RAM: port A = read (one address per clock),
// port B = write (one pixel per pix_en). Because the two arrays work in
// parallel, TWO bilinear points are fetched per clock, so the whole 2x2
// neighbourhood is collected in TWO phases instead of four:
//
//   ph0 : issue (x0 -> mem_e) and (x0 -> mem_o)   // y0 row and y0+1 row
//   ph1 : issue (x1 -> mem_e) and (x1 -> mem_o)   // x0+1 column
//
// RAM latency is 1 clock:
//   ph0 issues x0  -> data valid in ph1 -> latched into d0/d1
//   ph1 issues x1  -> data valid in ph0 -> read straight off rdE/rdO
// so at the ph0 clock edge all four points (q00,q10,q01,q11) are formed at
// once and q_valid pulses there.  Total latency = exactly 1 pixel (2 clocks
// at 74.25 MHz = 1 pix_en slot), which is what the caller expects.
//---------------------------------------------------------------------------
module line_bank #(
    parameter W = 640,
    parameter K = 24
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         pix_en,
    // write port (input stream, one pixel per pix_en)
    input  wire         wr_valid,
    input  wire [8:0]   wr_sy,
    input  wire [9:0]   wr_sx,
    input  wire [15:0]  wr_data,
    // read request, valid during ph0
    input  wire         in_valid,
    input  wire [8:0]   ry0,
    input  wire [9:0]   rx0,
    input  wire         src_ok,
    // results
    output reg          q_valid,
    output reg  [15:0]  q00, q01, q10, q11,
    output reg          q_src_ok
);
    localparam integer HALFK = K/2;            // 12 行/数组
    localparam integer DEPTH = HALFK * W;      // 12*640 = 7680

    // v4: 真双口 + 奇偶行拆分 —— 每像素只用 2 个时钟相位（配合 74.25MHz）
    //   偶数 slot 行 -> mem_e，奇数 slot 行 -> mem_o，各 12 行 x 640
    //   每个数组用真双口：A 口读（每拍 1 个地址）、B 口写（每像素 1 次）
    //   两个数组并行 => 每拍读 2 个点 => 2 拍读完双线性的 4 点
    reg ph;
    always @(posedge clk) begin
        if (!rst_n)      ph <= 1'b0;
        else if (pix_en) ph <= 1'b0;
        else             ph <= 1'b1;
    end

    reg [15:0] mem_e [0:DEPTH-1];
    reg [15:0] mem_o [0:DEPTH-1];

    // ---------------- addresses ----------------
    wire [4:0]  slot0 = ry0 % K;
    wire [4:0]  slot1 = (slot0 == (K[4:0]-5'd1)) ? 5'd0 : (slot0 + 5'd1);
    wire [9:0]  rx1c  = (rx0 >= (W-1)) ? (W-1) : (rx0 + 10'd1);
    wire        s0_even = ~slot0[0];               // y0 落在 mem_e

    // slot0/slot1 一偶一奇，>>1 即数组内行号
    wire [4:0]  e_slot = s0_even ? slot0 : slot1;
    wire [4:0]  o_slot = s0_even ? slot1 : slot0;
    wire [12:0] ea0 = ((e_slot >> 1) * W) + rx0;
    wire [12:0] oa0 = ((o_slot >> 1) * W) + rx0;
    wire [12:0] ea1 = ((e_slot >> 1) * W) + rx1c;
    wire [12:0] oa1 = ((o_slot >> 1) * W) + rx1c;

    wire [4:0]  wslot = wr_sy % K;
    wire [12:0] waddr = ((wslot >> 1) * W) + wr_sx;
    wire        wsel  = wslot[0];                  // 0->mem_e, 1->mem_o

    // ---------------- RAM：两个真双口，A 口读 / B 口写 ----------------
    reg  [12:0] raE, raO;
    reg  [15:0] rdE, rdO;

    always @(posedge clk) begin
        if (!ph) begin raE <= ea0; raO <= oa0; end   // ph0: x0 两点
        else     begin raE <= ea1; raO <= oa1; end   // ph1: x1 两点
    end

    always @(posedge clk) begin
        rdE <= mem_e[raE];
        rdO <= mem_o[raO];
    end

    always @(posedge clk) begin
        if (wr_valid && pix_en) begin
            if (!wsel) mem_e[waddr] <= wr_data;
            else       mem_o[waddr] <= wr_data;
        end
    end

    // ---------------- data capture + output ----------------
    // 相位时序（RAM 读延迟 1 拍）:
    //   ph0 发 x0 两点  -> ph1 回来      (rdE/rdO)
    //   ph1 发 x1 两点  -> 下一像素 ph0 回来
    // => 在下一像素的 ph0 拍形成 q00..q11，延迟正好 1 个像素
    reg [15:0] d0, d1;
    reg        vdel, okdel, evdel;

    always @(posedge clk) begin
        if (!rst_n) begin
            q_valid  <= 1'b0;
            q_src_ok <= 1'b0;
            q00 <= 0; q01 <= 0; q10 <= 0; q11 <= 0;
            d0 <= 0; d1 <= 0;
            vdel <= 1'b0; okdel <= 1'b0; evdel <= 1'b0;
        end else begin
            if (ph) begin
                // ph1: rdE/rdO 是 (y0,x0) / (y0+1,x0)
                if (s0_even) begin d0 <= rdE; d1 <= rdO; end
                else         begin d0 <= rdO; d1 <= rdE; end
                evdel <= s0_even;
                vdel  <= in_valid;
                okdel <= src_ok;
            end else begin
                // ph0: rdE/rdO 是 (y0,x1) / (y0+1,x1)
                q00 <= d0;
                q10 <= d1;
                if (evdel) begin q01 <= rdE; q11 <= rdO; end
                else       begin q01 <= rdO; q11 <= rdE; end
                q_src_ok <= okdel;
                q_valid  <= vdel;
            end

            if (ph) q_valid <= 1'b0;      // q_valid 单拍脉冲
        end
    end
endmodule
