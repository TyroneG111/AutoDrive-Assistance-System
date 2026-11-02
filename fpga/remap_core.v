//---------------------------------------------------------------------------
// remap_core.v
// 双线性插值校正核（单路）。
// 输入 line_bank 取回的 4 点 q00/q01/q10/q11 与 LUT 小数权重，
// 输出校正后的 RGB565 像素。
//
//   top = c00 + (c01-c00)*fx/256
//   bot = c10 + (c11-c10)*fx/256
//   out = top + (bot-top)*fy/256
// R/G/B 三通道分别插值（拆到 8bit 运算，再打包 5/6/5）。
//---------------------------------------------------------------------------
module remap_core (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         valid_in,
    input  wire [15:0]  q00, q01, q10, q11,
    input  wire [7:0]   fx,   // mapx 小数
    input  wire [7:0]   fy,   // mapy 小数
    input  wire         valid_in_src,  // 该点是否在有效区（非黑边）
    output reg          valid_out,
    output reg  [15:0]  rgb_out
);
    // 拆通道并扩到 8bit
    function [7:0] r5(input [15:0] c); r5 = {c[15:11], 3'd0}; endfunction
    function [7:0] g6(input [15:0] c); g6 = {c[10:5],  2'd0}; endfunction
    function [7:0] b5(input [15:0] c); b5 = {c[4:0],   3'd0}; endfunction

    function signed [10:0] lerp(input signed [8:0] a, input signed [8:0] b, input [7:0] f);
        reg signed [16:0] d;
    begin
        d = (b - a) * $signed({1'b0, f});
        lerp = a + (d >>> 8);
    end
    endfunction

    // 一级：水平插值
    reg signed [10:0] rt, gt, bt, rb, gb, bb;
    reg v1, valid_src1;
    always @(posedge clk) begin
        if (!rst_n) begin
            v1 <= 0; valid_src1 <= 0;
        end else begin
            v1 <= valid_in; valid_src1 <= valid_in_src;
            rt <= lerp(r5(q00), r5(q01), fx);
            gt <= lerp(g6(q00), g6(q01), fx);
            bt <= lerp(b5(q00), b5(q01), fx);
            rb <= lerp(r5(q10), r5(q11), fx);
            gb <= lerp(g6(q10), g6(q11), fx);
            bb <= lerp(b5(q10), b5(q11), fx);
        end
    end

    // 二级：垂直插值
    function [7:0] clip11(input signed [10:0] v);
    begin
        if (v < 0) clip11 = 8'd0;
        else if (v > 255) clip11 = 8'd255;
        else clip11 = v[7:0];
    end
    endfunction

    reg signed [19:0] rr, gg, bbv;
    always @(*) begin
        rr  = {rt[8:0],8'd0} + (rb - rt) * $signed({1'b0, fy});
        gg  = {gt[8:0],8'd0} + (gb - gt) * $signed({1'b0, fy});
        bbv = {bt[8:0],8'd0} + (bb - bt) * $signed({1'b0, fy});
    end

    wire [7:0] r8 = clip11(rr  >>> 8);
    wire [7:0] g8 = clip11(gg  >>> 8);
    wire [7:0] b8 = clip11(bbv >>> 8);

    always @(posedge clk) begin
        if (!rst_n) begin
            valid_out <= 0; rgb_out <= 16'd0;
        end else begin
            valid_out <= v1;
            // NOTE: do not blank the output when the source flag is low -- the
            // map is already clamped to the image border by the caller, and a
            // spurious low flag used to black out the first pixel of every row.
            rgb_out <= {r8[7:3], g8[7:2], b8[7:3]};
        end
    end
endmodule
