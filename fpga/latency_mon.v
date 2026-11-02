`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: latency_mon
// Description: 端到端时延统计（输入帧起点 -> UDP 整帧发送完成）
//
//   【测量定义】
//     四路拼接模式 (mode[0]=1)：
//        起点 = cam1(CH0) 的场同步上升沿（开始接收这一帧的第一个像素）
//        终点 = UDP 把「含这一帧的拼接图」最后一个报文最后一个字节发完
//        时延 = 两者之间的时间差
//     单路模式 (mode[1..4] 之一)：
//        起点 = 被选中那一路的场同步上升沿
//        终点 = UDP 把这一帧的最后一个字节发完
//
//   【为什么不是"取最近一次 vs"】
//     输入帧经 DDR 帧缓存后才输出，在途帧数恒为 1~2 帧(约 33~67ms)。
//     若直接把"最近一次 vs 时间戳"与"UDP 帧完成"相减，会把在途帧数抹掉。
//     这里用一个 8 深的帧时间戳 FIFO：
//        入队 = 输入帧起点（按模式选中的那一路 vs 上升沿）
//        出队 = UDP 一帧发送完成
//     输入与输出是 1:1 关系，FIFO 自然把"同一帧"的两端配对起来，
//     无需知道缓存深度，且 inflight 输出可直接观察在途帧数是否合理(1~2)。
//
//   【时钟域】
//     · 本模块时钟 = clk（接 clk_37_125M），与 o_mode_frame 同域，无需跨域
//     · ch_vs 来自各路像素时钟域（74.25MHz），内部两级同步
//     · udp_frame_tgl 来自 rgmii_clk(125MHz) 域的电平翻转，两级同步 + 边沿检测
//       （不用脉冲是因为 8ns 脉冲在 37.125MHz 域会丢）
//
//   【时基】
//     1 tick = 1/clk ≈ 26.936ns（clk_37_125M）
//     换算：ns = tick * 26.936 ；us = tick / 37.125 ；ms = tick / 37125
//
//   【64bit 调试总线中的位置】(见 proj_top 的 dbg_bus 组装)
//     [31:0]  lat_tick      端到端时延（tick，26.936ns/tick）
//     [36:32] mode_frame    当前输出帧模式（5bit 独热）
//     [40:37] inflight      在途帧数（0..8）
//     [57]    lat_valid     至少完成过一次有效测量
//
//////////////////////////////////////////////////////////////////////////////////
module latency_mon #(
    parameter FIFO_AW = 3                  // 帧时间戳 FIFO 深度 = 2**FIFO_AW = 8
)(
    input  wire        clk,                // 37.125MHz 像素域（与 mode 同域）
    input  wire        rst_n,

    // 4 路输入场同步（异步，内部两级同步）
    input  wire [3:0]  ch_vs,

    // 当前输出帧模式（与 clk 同域，帧边界更新）
    //   bit0 = 四路拼接，bit1..bit4 = 单路 CH0..CH3
    input  wire [4:0]  mode,

    // UDP 一帧发送完成（异步电平翻转，来自 rgmii_clk 域）
    input  wire        udp_frame_tgl,

    // ---------------- 结果 ----------------
    output reg  [31:0] lat_tick,           // 端到端时延（1 tick = 26.936ns）
    output reg         lat_valid,          // 至少测到过一次
    output reg  [FIFO_AW:0] inflight,      // 在途帧数
    output reg  [15:0] frame_cnt,          // 已完成的帧数（低 16bit）
    output reg         pair_ovf            // FIFO 溢出（输入比输出快太多）
);

    localparam integer      DEPTH   = (1 << FIFO_AW);
    localparam [FIFO_AW:0]  DEPTH_V = DEPTH;          // 满档位（宽度与 used 一致）

    //================================================================
    // 时基：自由运行计数器
    //================================================================
    reg [31:0] ts_cnt;
    always @(posedge clk) begin
        if(!rst_n) ts_cnt <= 32'd0;
        else       ts_cnt <= ts_cnt + 32'd1;
    end

    //================================================================
    // 4 路 vs 两级同步
    //   vs 是 5 个源时钟周期宽（74.25MHz 下约 67ns），远大于本模块周期，可靠
    //================================================================
    reg [3:0] vs_s1, vs_s2;
    always @(posedge clk) begin
        if(!rst_n) begin
            vs_s1 <= 4'd0;
            vs_s2 <= 4'd0;
        end else begin
            vs_s1 <= ch_vs;
            vs_s2 <= vs_s1;
        end
    end

    //================================================================
    // 起点通道选择
    //   拼接模式(mode[0])：取 cam1 = CH0 作为参考
    //   单路模式：取被选中的那一路
    //================================================================
    wire [1:0] sel_ch = mode[4] ? 2'd3 :
                        mode[3] ? 2'd2 :
                        mode[2] ? 2'd1 : 2'd0;   // mode[0]/mode[1] 都取 CH0

    reg        sel_vs_d;
    wire       sel_vs  = vs_s2[sel_ch];
    always @(posedge clk) begin
        if(!rst_n) sel_vs_d <= 1'b0;
        else       sel_vs_d <= sel_vs;
    end
    wire sel_vs_rise = sel_vs & ~sel_vs_d;

    // ---- vs 间隔保护：抑制毛刺 ----
    //   正常帧间隔约 33ms = 1.24M tick，门槛只取 2000 tick(≈54us)，远大于行周期
    //   之外的任何干扰，绝不会漏掉真实帧。复位值给满档，使上电第一帧即可生效。
    reg [19:0] gap_cnt;
    always @(posedge clk) begin
        if(!rst_n)                          gap_cnt <= 20'hFFFFF;
        else if(sel_vs_rise)                gap_cnt <= 20'd0;
        else if(gap_cnt != 20'hFFFFF)       gap_cnt <= gap_cnt + 20'd1;
    end
    wire in_frame_start = sel_vs_rise & (gap_cnt > 20'd2000);

    //================================================================
    // UDP 帧完成事件：toggle 两级同步 + 边沿检测
    //================================================================
    reg tgl_s1, tgl_s2, tgl_s3;
    always @(posedge clk) begin
        if(!rst_n) begin
            tgl_s1 <= 1'b0;
            tgl_s2 <= 1'b0;
            tgl_s3 <= 1'b0;
        end else begin
            tgl_s1 <= udp_frame_tgl;
            tgl_s2 <= tgl_s1;
            tgl_s3 <= tgl_s2;
        end
    end
    wire out_frame_done = tgl_s2 ^ tgl_s3;

    //================================================================
    // 模式变化检测：跨模式配对没有意义，清空 FIFO
    //================================================================
    reg  [4:0] mode_d;
    always @(posedge clk) begin
        if(!rst_n) mode_d <= 5'd0;
        else       mode_d <= mode;
    end
    wire mode_chg = (mode != mode_d);

    //================================================================
    // 帧时间戳 FIFO（同一时钟域，无需格雷码）
    //================================================================
    reg [31:0] ts_fifo [0:DEPTH-1];
    reg [FIFO_AW-1:0] wp, rp;
    wire [FIFO_AW:0]  used = {1'b0, wp} - {1'b0, rp};

    wire fifo_full  = (used == DEPTH_V);

    // ---- 入队：输入帧起点 ----
    always @(posedge clk) begin
        if(!rst_n) begin
            wp       <= {FIFO_AW{1'b0}};
            pair_ovf <= 1'b0;
        end else if(mode_chg) begin
            pair_ovf <= 1'b0;
        end else if(in_frame_start) begin
            if(!fifo_full) begin
                ts_fifo[wp] <= ts_cnt;
                wp          <= wp + 1'b1;
            end else begin
                pair_ovf    <= 1'b1;   // 输入帧比输出帧快太多，丢弃
            end
        end
    end

    // ---- 出队：UDP 帧完成，算差值 ----
    always @(posedge clk) begin
        if(!rst_n) begin
            rp        <= {FIFO_AW{1'b0}};
            lat_tick  <= 32'd0;
            lat_valid <= 1'b0;
            frame_cnt <= 16'd0;
        end else if(mode_chg) begin
            rp        <= wp;              // 清空：丢下去
            lat_valid <= 1'b0;
            frame_cnt <= 16'd0;
        end else if(out_frame_done) begin
            frame_cnt <= frame_cnt + 16'd1;
            if(used != {(FIFO_AW+1){1'b0}}) begin
                lat_tick  <= ts_cnt - ts_fifo[rp];
                rp        <= rp + 1'b1;
                lat_valid <= 1'b1;
            end
        end
    end

    // ---- 在途帧数（综合成常数级逻辑） ----
    always @(posedge clk) begin
        if(!rst_n) inflight <= {(FIFO_AW+1){1'b0}};
        else       inflight <= used;
    end

endmodule
