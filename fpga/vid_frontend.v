`timescale 1ns/1ps
//=============================================================================
// vid_frontend —— 单路视频输入前端（方案 ③）
//
// 输入按用户已确认的时序：
//   · 24bit/像素（YUV444），de 为高期间有效；一行 DE 有效拍数 = H_DE_PULSES(=1280)
//   · vs 是高有效同步脉冲（仅 v=0..4 为高）→ 只检测上升沿，不判断高电平宽度
//   · 74.25MHz 标称（本模块不假设具体频率，全部按 pclk 计数）
//
// 输出：
//   px_vld/px_data  —— 每个 DE 拍（=1 个像素）有效
//   frame_start     —— 有效帧起始（VS 上升沿）1 拍
//   frame_done      —— 一帧完整结束（行/像素数都对）1 拍
//   frame_err       —— 上一帧不完整（个数不符）1 拍
//   stale           —— 长时间无 VS（视频停摆）
//=============================================================================
module vid_frontend #(
    parameter H_DE_PULSES = 1280,           // 每行 DE 有效拍数 = 像素数
    parameter V_ACTIVE    = 720,            // 有效行数
    parameter H_TOTAL     = 3300,           // 一行总 pclk（仅用于去毛刺/超时）
    parameter V_TOTAL     = 750,            // 一帧总行数
    parameter FRAME_BITS  = 26              // 帧周期计数位宽（H_TOTAL*V_TOTAL*4）
)(
    input  wire        pclk,
    input  wire        rst_n,
    // 视频输入
    input  wire        vs,
    input  wire        de,
    input  wire [23:0] pix,
    // 输出
    output reg         px_vld,
    output reg  [23:0] px_data,
    output reg  [11:0] ln_cnt,          // 当前帧已完成的完整行数
    output reg         frame_start,
    output reg         frame_done,
    output reg         frame_err,
    output reg         stale,
    output reg  [31:0] err_cnt          // 不完整帧累计
);
    localparam VS_GAP_MIN = H_TOTAL;                        // 两次 VS 最小间隔
    localparam STALE_MAX  = (H_TOTAL * V_TOTAL) * 4;        // 4 个标称帧周期

    // ---- VS 同步与边沿检测 ----
    reg vs_d0, vs_d1, vs_d2;
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            vs_d0 <= 1'b0; vs_d1 <= 1'b0; vs_d2 <= 1'b0;
        end else begin
            vs_d0 <= vs;
            vs_d1 <= vs_d0;
            vs_d2 <= vs_d1;
        end
    end
    wire vs_rise = vs_d1 & ~vs_d2;

    // ---- 距上次 VS 的间隔计数（去毛刺 + stale 判定） ----
    reg [FRAME_BITS-1:0] gap_cnt;
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n)          gap_cnt <= {FRAME_BITS{1'b0}};
        else if (vs_rise)    gap_cnt <= {FRAME_BITS{1'b0}};
        else                 gap_cnt <= gap_cnt + 1'b1;
    end

    wire vs_ok = vs_rise & (gap_cnt > VS_GAP_MIN[FRAME_BITS-1:0]);

    // ---- 行/像素计数 ----
    reg [11:0] px_cnt;                  // 当前行内 DE 拍数
    reg        frame_active;            // 正在接收有效区

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            px_cnt       <= 12'd0;
            ln_cnt       <= 12'd0;
            frame_active <= 1'b0;
            px_vld       <= 1'b0;
            px_data      <= 24'd0;
            frame_start  <= 1'b0;
            frame_done   <= 1'b0;
            frame_err    <= 1'b0;
            err_cnt      <= 32'd0;
        end else begin
            px_vld      <= 1'b0;
            frame_start <= 1'b0;
            frame_done  <= 1'b0;
            frame_err   <= 1'b0;

            if (vs_ok) begin
                // 上一帧是否完整？
                if (frame_active && (ln_cnt != V_ACTIVE[11:0])) begin
                    frame_err <= 1'b1;
                    err_cnt   <= err_cnt + 32'd1;
                end
                frame_active <= 1'b1;
                frame_start  <= 1'b1;
                ln_cnt       <= 12'd0;
                px_cnt       <= 12'd0;
            end else if (de) begin
                px_vld  <= 1'b1;
                px_data <= pix;
                if (px_cnt == (H_DE_PULSES[11:0] - 12'd1)) begin
                    px_cnt <= 12'd0;
                    if (ln_cnt == (V_ACTIVE[11:0] - 12'd1)) begin
                        ln_cnt       <= V_ACTIVE[11:0];
                        frame_active <= 1'b0;
                        frame_done   <= 1'b1;      // 一帧收齐
                    end else begin
                        ln_cnt <= ln_cnt + 12'd1;
                    end
                end else begin
                    px_cnt <= px_cnt + 12'd1;
                end
            end
        end
    end

    // ---- 视频停摆判定 ----
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n)
            stale <= 1'b1;
        else if (vs_rise)
            stale <= 1'b0;
        else if (gap_cnt >= STALE_MAX[FRAME_BITS-1:0])
            stale <= 1'b1;
    end

endmodule
