`timescale 1ns/1ps
//=============================================================================
// scale_2x2_avg —— 2x2 网格平均缩小（1280x720 -> 640x360，定点，无浮点/乘法/除法）
//
// 原理（用户确认）：
//   输入输出尺寸严格 2:1，坐标一一对应：输出像素 (x,y) 对应输入的 2x2 网格
//       (2x, 2y) (2x+1, 2y) (2x, 2y+1) (2x+1, 2y+1)
//   四个像素相加后舍去末两位（>>2）即为输出像素。系数恒定，
//   故只需「加法 + 移位」，无浮点运算、无乘法器/除法器。
//
// 数据组织（2026-10 YUV444 改造后）：
//   · 输入 256bit 字 = 8 个 32bit 通道，每个通道 = 1 个 24bit 像素（高 8bit 补零），
//     即 1 个 256bit 字 = 8 个连续像素
//   · 输入行按字顺序流入（每行 ROW_WORDS 个字 = 8*ROW_WORDS 像素）
//   · 水平 2:1 在【同一个字内】相邻两个 32bit 通道之间完成（得 4 个部分和）；
//     每 2 个输入字（16 像素）→ 1 个输出字（8 像素）：前 4 个输出像素来自第1字，
//     后 4 个来自第2字；垂直方向再与行缓存中的同一部分和相加，最后 >>2
//
// 两级流水：
//   第 1 拍（字对第 1 个字）：发出行缓存读地址（同步读需提前 1 拍）
//   第 2 拍（字对第 2 个字，pair_last）：算出本对字的 8 个水平部分和
//                         行A（row_b=0）-> 写入行缓存
//   第 3 拍（pair_last_d）  ：行A 部分和（行缓存输出）与行B 部分和相加
//                            -> >>2 -> 输出字（out_vld 延迟 2 拍）
//
// 行缓存：NPAIR 个 (8 x 25bit) 条目
//=============================================================================
module scale_2x2_avg #(
    parameter DW        = 256,
    parameter ROW_WORDS = 160,          // 输入行 256bit 字数（必须为偶数）
    parameter LB_DEPTH  = 128           // 行缓存深度（>= ROW_WORDS/2）
)(
    input  wire             clk,
    input  wire             rst_n,
    // ---- 输入字流（行A 或 行B 顺序流入） ----
    input  wire             word_vld,   // 输入字有效
    input  wire [DW-1:0]    word,       // 输入字
    input  wire             row_b,      // 0 = 行A（写行缓存）  1 = 行B（与行缓存合并输出）
    input  wire             job_start,  // 【与本次 job 第 1 个有效字同拍】（不是提前一拍）
    // ---- 输出字流（仅 row_b=1 时有效） ----
    output reg              out_vld,
    output reg  [DW-1:0]    out_data
);
    localparam LANES = DW/32;           // 8 个 32bit 通道
    localparam NPIX  = DW/32;           // 8 个像素/字（每 32bit 通道 1 个 24bit 像素）
    localparam NPAIR = ROW_WORDS/2;     // 80 对/行
    localparam LWID  = 27;              // 水平部分和位宽 = 3 个 8bit 场各 9bit

    //-------------------------------------------------------------------------
    // 输入字配对：每 2 个输入字为一对
    //   pair_last 那一拍：w_prev = 对中第 1 个字，word = 对中第 2 个字
    //-------------------------------------------------------------------------
    reg [DW-1:0] w_prev;
    reg [7:0]    wc;                    // 行内字计数
    reg [7:0]    pc;                    // 行内对计数（= 行缓存地址）

    wire [DW-1:0] pw0 = w_prev;
    wire [DW-1:0] pw1 = word;
    wire          pair_last = word_vld & wc[0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)        w_prev <= {DW{1'b0}};
        else if (word_vld) w_prev <= word;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          wc <= 8'd0;
        else if (job_start)  wc <= word_vld ? 8'd1 : 8'd0;   // 本拍即第 1 个字 -> 之后为 1
        else if (word_vld)   wc <= wc + 8'd1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          pc <= 8'd0;
        else if (job_start)  pc <= 8'd0;
        else if (pair_last)  pc <= pc + 8'd1;
    end

    //-------------------------------------------------------------------------
    // 水平 2:1：每个 256bit 字内相邻两个 32bit 通道的像素相加（像素对在同一字内）
    //   每字 8 像素 -> 4 个水平部分和
    //   ★Y/Cb/Cr 三个 8bit 场【各自独立相加】——不能把 24bit 当整数相加，
    //     否则最大值 4*255=1020 会进位串扰到相邻场。
    //-------------------------------------------------------------------------
    function [LWID-1:0] hsum2;          // {Y(9b), Cb(9b), Cr(9b)}
        input [23:0] a;
        input [23:0] b;
        begin
            hsum2 = { ({1'b0, a[23:16]} + {1'b0, b[23:16]}),
                      ({1'b0, a[15:8]}  + {1'b0, b[15:8]} ),
                      ({1'b0, a[7:0]}   + {1'b0, b[7:0]}  ) };
        end
    endfunction

    wire [LWID-1:0] hs0 [0:LANES/2-1];
    wire [LWID-1:0] hs1 [0:LANES/2-1];

    genvar i;
    generate
        for (i = 0; i < LANES/2; i = i + 1) begin : g_hsum
            assign hs0[i] = hsum2(pw0[32*(2*i)      +: 24],
                                  pw0[32*(2*i) + 32 +: 24]);
            assign hs1[i] = hsum2(pw1[32*(2*i)      +: 24],
                                  pw1[32*(2*i) + 32 +: 24]);
        end
    endgenerate

    // 8 个水平部分和按输出像素序打包：j=0..3 来自对中第 1 个字，j=4..7 来自第 2 个字
    wire [NPIX*LWID-1:0] s_cur;
    generate
        for (i = 0; i < NPIX; i = i + 1) begin : g_spack
            if (i < LANES/2) assign s_cur[LWID*i +: LWID] = hs0[i];
            else             assign s_cur[LWID*i +: LWID] = hs1[i-LANES/2];
        end
    endgenerate

    //-------------------------------------------------------------------------
    // 行缓存（行 A 的水平部分和）：同步写 / 同步读（推断为简单双口 RAM）
    //-------------------------------------------------------------------------
    reg [NPIX*LWID-1:0] lb [0:LB_DEPTH-1];
    reg [NPIX*LWID-1:0] lb_rdata;
    reg [7:0]           lb_ra;

    always @(posedge clk) begin
        if (pair_last & ~row_b) lb[pc] <= s_cur;      // 行A：写（pair_last 那拍）
    end

    always @(posedge clk) begin
        lb_rdata <= lb[lb_ra];
    end

    // 【关键】读地址必须在字对【第 1 个字】那拍就发出：
    //   同步读有 1 拍延迟，若等到 pair_last 才发地址，数据会对不上同拍的部分和
    //   job_start 与首字同拍，故此处需把地址直接清 0（此时 pc 还是上个 job 的尾值）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                             lb_ra <= 8'd0;
        else if (job_start)                     lb_ra <= 8'd0;
        else if (word_vld & ~wc[0] & row_b)     lb_ra <= pc;
    end

    //-------------------------------------------------------------------------
    // 第 2 拍：垂直相加（行A 部分和 + 行B 部分和）-> >>2 -> 输出像素
    //-------------------------------------------------------------------------
    reg [NPIX*LWID-1:0] s_b;

    always @(posedge clk) if (pair_last) s_b <= s_cur;

    reg pair_last_d, row_b_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin pair_last_d <= 1'b0; row_b_d <= 1'b0; end
        else        begin pair_last_d <= pair_last; row_b_d <= row_b; end
    end

    wire [9:0] ys  [0:NPIX-1];
    wire [9:0] cbs [0:NPIX-1];
    wire [9:0] crs [0:NPIX-1];
    generate
        for (i = 0; i < NPIX; i = i + 1) begin : g_vsum
            assign ys [i] = lb_rdata[LWID*i+26 : LWID*i+18] + s_b[LWID*i+26 : LWID*i+18];
            assign cbs[i] = lb_rdata[LWID*i+17 : LWID*i+9 ] + s_b[LWID*i+17 : LWID*i+9 ];
            assign crs[i] = lb_rdata[LWID*i+8  : LWID*i+0 ] + s_b[LWID*i+8  : LWID*i+0 ];
        end
    endgenerate

    wire [DW-1:0] out_nxt;
    generate
        for (i = 0; i < LANES; i = i + 1) begin : g_opack
            // 每个 32bit 通道 = 1 个 24bit 结果像素（三场 >>2），高 8bit 补零
            // 打包顺序必须与全链路约定一致：{Y[23:16], Cb[15:8], Cr[7:0]}
            assign out_nxt[32*i + 16 +: 8] = ys [i][9:2];
            assign out_nxt[32*i +  8 +: 8] = cbs[i][9:2];
            assign out_nxt[32*i      +: 8] = crs[i][9:2];
            assign out_nxt[32*i + 24 +: 8] = 8'h00;
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_vld  <= 1'b0;
            out_data <= {DW{1'b0}};
        end else begin
            out_vld  <= pair_last_d & row_b_d;
            out_data <= out_nxt;
        end
    end

endmodule
