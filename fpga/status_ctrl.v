`timescale 1ns/1ps
//=============================================================================
// status_ctrl —— 按键消抖 + LED 指示（用户确认分配）
//   led[0] = ddr/系统初始化完成（init_done）
//   led[1] = 心跳（约 1Hz）
//   led[2] = link（PLL 锁定 + lanel 复位完成 + 4 路都有帧）
//   led[3] = 错误（溢出丢帧 / 源帧异常 / 接收丢帧）
//=============================================================================
module status_ctrl #(
    parameter CLK_FREQ   = 125_000_000,     // core_clk 频率
    parameter DEBOUNCE_N = 1_250_000        // 10ms
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  key,
    input  wire        init_done,
    input  wire        link_ok,
    input  wire        err_flag,
    output wire 	   led
);
    // ---------------- 按键消抖 ----------------
    reg [1:0] key_d0, key_d1;
    reg [20:0] db_cnt;
    reg [1:0]  key_stable;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_d0 <= 2'b00; key_d1 <= 2'b00;
            db_cnt <= 21'd0;
            key_stable <= 2'b00;
        end else begin
            key_d0 <= key;
            key_d1 <= key_d0;
            if (key_d1 != key_stable) begin
                if (db_cnt >= DEBOUNCE_N[20:0])
                    key_stable <= key_d1;
                else
                    db_cnt <= db_cnt + 21'd1;
            end else begin
                db_cnt <= 21'd0;
            end
        end
    end

    // ---------------- 心跳 ----------------
    localparam HB_CNT = CLK_FREQ / 2;
    reg [26:0] hb_cnt;
    reg        hb;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hb_cnt <= 27'd0;
            hb     <= 1'b0;
        end else if (hb_cnt >= HB_CNT[26:0]) begin
            hb_cnt <= 27'd0;
            hb     <= ~hb;
        end else begin
            hb_cnt <= hb_cnt + 27'd1;
        end
    end

    assign led = err_flag;

endmodule
