`timescale 1ns/1ps
//=============================================================================
// dmt_tmg_gen —— 固定 DMT 720p30 视频时序发生器（37.125MHz 像素域）
//
// 时序（消隐周期数与 720p60 相同，仅像素时钟降一半 -> 帧率 30fps）：
//   H: 有效 1280 | 前肩 110 | 同步 40 | 后肩 220   -> 总计 1650
//   V: 有效  720 | 前肩   5 | 同步  5 | 后肩  20   -> 总计  750
//   pclk = 37.125MHz -> 行 44.4444us，帧 33.3333ms（30.000fps），HS/VS 正极性
//
// 【设计要点】
//   · 不做自适应消隐（用户决策 4），行/帧总长恒定。
//   · !run 时计数器停在“垂直消隐起点”，HS/VS/DE 全低 —— 上电期间监视器
//     看到的是“无信号”，而不是黑屏/花屏；run 由顶层在行缓存备好首行后置起。
//   · V_ACT 之前先跑 30 行垂直消隐，给行缓存留出预取时间。
//   · de/hs/vs 与 hc/vc 同拍组合输出；行缓存读口打 1 拍，内部把这三者
//     也延迟 1 拍，保证像素与同步严格对齐。
//=============================================================================
module dmt_tmg_gen #(
    parameter H_ACT   = 1280,
    parameter H_FP    = 110,
    parameter H_SYNC  = 40,
    parameter H_BP    = 220,
    parameter V_ACT   = 720,
    parameter V_FP    = 5,
    parameter V_SYNC  = 5,
    parameter V_BP    = 20
)(
    input  wire        pclk,
    input  wire        rst_n,
    input  wire        run,             // 1 = 开始出图
    output wire        de,
    output wire        hs,
    output wire        vs,
    output wire [10:0] hc,              // 0 .. H_TOTAL-1
    output wire [9:0]  vc               // 0 .. V_TOTAL-1
);
    localparam H_TOTAL = H_ACT + H_FP + H_SYNC + H_BP;
    localparam V_TOTAL = V_ACT + V_FP + V_SYNC + V_BP;

    reg [10:0] hc_r;
    reg [9:0]  vc_r;

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            hc_r <= 11'd0;
            vc_r <= V_ACT[9:0];             // 停在垂直消隐起点
        end else if (!run) begin
            hc_r <= 11'd0;
            vc_r <= V_ACT[9:0];
        end else if (hc_r == H_TOTAL[10:0] - 11'd1) begin
            hc_r <= 11'd0;
            vc_r <= (vc_r == V_TOTAL[9:0] - 10'd1) ? 10'd0 : (vc_r + 10'd1);
        end else begin
            hc_r <= hc_r + 11'd1;
        end
    end

    assign hc = hc_r;
    assign vc = vc_r;

    assign de = run & (hc_r < H_ACT[10:0]) & (vc_r < V_ACT[9:0]);

    //================= HS / VS 极性 =================
    //   720p60 标准是【正极性】（同步脉冲期间为高），本模块按正极性输出。
    //   若 MS7210 寄存器里配的输入极性与 FPGA 送的不一致，芯片会把同步反相后
    //   嵌进 TMDS，显示器就锁不上模式（表现为：有信号但背光不亮/黑屏）。
    //   此时把下面两条 assign 的同步条件用 ~(...) 整段取反即可。
    //   同步区间：水平 [H_ACT+H_FP, +H_SYNC)，垂直 [V_ACT+V_FP, +V_SYNC)。
    //===============================================
    assign hs = run & (hc_r >= (H_ACT + H_FP)) &
                      (hc_r <  (H_ACT + H_FP + H_SYNC));
    assign vs = run & (vc_r >= (V_ACT + V_FP)) &
                      (vc_r <  (V_ACT + V_FP + V_SYNC));

endmodule
