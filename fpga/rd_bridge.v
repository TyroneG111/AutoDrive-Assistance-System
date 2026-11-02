`timescale 1ns/1ps
//=============================================================================
// rd_bridge —— DDR 读口跨时钟桥（pclk 37.125MHz  <->  core 100MHz）
//
// 作用：把 axi_burst_master 的“内部读突发接口”搬到 hdmi_pclk 域，使读出器
//       （rd_line_gen）完全运行在像素域，与 AXI/DDR 域彻底解耦。
//
// 结构：
//   · 请求 FIFO（pclk 写 / core 读）  ：深度 2^REQ_AW，承载 25bit 字地址
//   · 数据 FIFO（core 写 / pclk 读）  ：深度 2^DAT_AW，承载 256bit 读回数据（FWFT）
//   · core 侧串行执行请求（axi_burst_master 只支持单突发在途），每完成一个突发
//     回一个 rd_finish；pclk 侧靠“每突发固定 16 字”自行计数，无需边带标记。
//
// 两个 FIFO 均由纯 RTL 的 dcram_fifo 例化（PDS 推断为简单双口 DRAM），
// 因此不引入任何 PDS IP，也不需要在仿真里加 IP 行为模型。
//=============================================================================
module rd_bridge #(
    parameter ADDR_W  = 25,
    parameter LEN_W   = 10,
    parameter DW      = 256,
    parameter BURST_W = 16,
    parameter REQ_AW  = 2,          // 请求 FIFO 深度 = 2^REQ_AW
    parameter DAT_AW  = 7           // 数据 FIFO 深度 = 2^DAT_AW（单位：256bit 字）
)(
    // ---------------- pclk 侧（读出器） ----------------
    input  wire              pclk,
    input  wire              pclk_rst_n,
    input  wire              req_vld,       // 1 拍：下发一个突发请求
    input  wire [ADDR_W-1:0] req_addr,
    output wire              req_rdy,       // 请求 FIFO 未满
    output wire              rd_vld,        // FWFT：为高时 rd_data 即当前字
    output wire [DW-1:0]     rd_data,
    input  wire              rd_en,         // 取走当前字
    // ---------------- core 侧（AXI 主控读口） ----------------
    input  wire              core_clk,
    input  wire              core_rst_n,
    output reg               axi_rd_req,
    output reg  [ADDR_W-1:0] axi_rd_addr,
    output wire [LEN_W-1:0]  axi_rd_len,
    input  wire              axi_rd_data_valid,
    input  wire [DW-1:0]     axi_rd_data,
    input  wire              axi_rd_finish
);
    //=========================================================
    // 请求 FIFO：pclk -> core
    //=========================================================
    wire [ADDR_W-1:0] rf_data;
    wire              rf_empty, rf_full;
    reg               rf_pop;

    dcram_fifo #(.DW(ADDR_W), .AW(REQ_AW)) u_req_fifo (
        .wr_clk      (pclk),
        .wr_rst_n    (pclk_rst_n),
        .wr_en       (req_vld),
        .wr_data     (req_addr),
        .wr_full     (rf_full),
        .wr_alm_full (),
        .wr_usedw    (),
        .rd_clk      (core_clk),
        .rd_rst_n    (core_rst_n),
        .rd_en       (rf_pop),
        .rd_data     (rf_data),
        .rd_empty    (rf_empty),
        .rd_alm_empty(),
        .rd_usedw    ()
    );

    assign req_rdy = ~rf_full;

    //=========================================================
    // 数据 FIFO：core -> pclk（FWFT）
    //=========================================================
    wire [DW-1:0] df_data;
    wire          df_empty, df_full;

    dcram_fifo #(.DW(DW), .AW(DAT_AW)) u_dat_fifo (
        .wr_clk      (core_clk),
        .wr_rst_n    (core_rst_n),
        .wr_en       (axi_rd_data_valid),
        .wr_data     (axi_rd_data),
        .wr_full     (df_full),
        .wr_alm_full (),
        .wr_usedw    (),
        .rd_clk      (pclk),
        .rd_rst_n    (pclk_rst_n),
        .rd_en       (rd_en),
        .rd_data     (df_data),
        .rd_empty    (df_empty),
        .rd_alm_empty(),
        .rd_usedw    ()
    );

    assign rd_vld  = ~df_empty;
    assign rd_data = df_data;

    //=========================================================
    // core 侧：串行执行请求（每突发固定 BURST_W 字）
    //=========================================================
    assign axi_rd_len = BURST_W[LEN_W-1:0];

    localparam C_IDLE = 2'd0, C_RUN = 2'd1, C_END = 2'd2;
    reg [1:0] cst;

    always @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
            cst         <= C_IDLE;
            axi_rd_req  <= 1'b0;
            axi_rd_addr <= {ADDR_W{1'b0}};
            rf_pop      <= 1'b0;
        end else begin
            rf_pop <= 1'b0;
            case (cst)
                C_IDLE: begin
                    if (!rf_empty) begin
                        axi_rd_addr <= rf_data;
                        rf_pop      <= 1'b1;      // 弹出该请求
                        axi_rd_req  <= 1'b1;      // 单拍脉冲，主控在 RS_IDLE 收到
                        cst         <= C_RUN;
                    end
                end
                C_RUN: begin
                    axi_rd_req <= 1'b0;
                    if (axi_rd_finish) cst <= C_END;
                end
                C_END: cst <= C_IDLE;             // 等主控回到 RS_IDLE
                default: cst <= C_IDLE;
            endcase
        end
    end

endmodule
