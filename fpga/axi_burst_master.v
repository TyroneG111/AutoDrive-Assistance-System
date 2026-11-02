`timescale 1ns/1ps
//=============================================================================
// axi_burst_master —— 内部突发接口 <-> HMIC_S 的 Simplified AXI4（方案 ⑦ 前端）
//
// 依据 UG022003（Logos HMIC_S IP）：
//   · 数据位宽 = DQ_WIDTH*8 = 256bit（x32 颗粒）
//   · axi_awlen/axi_arlen 只有 4bit -> 单次最长 16 beat = 512B
//   · 没有写响应通道 B，也没有 AWSIZE/AWBURST/ARLOCK/AWCACHE
//   · 地址/长度：本模块对外用“256bit 字”为单位，内部转成字节地址
//        字节地址 = {字地址, 5'b00000}；AWLEN = 字数 - 1
//
// 策略：单次只有一个突发在途（写/读各一套状态机），写不等 B（IP 无此通道），
//       写完 WLAST 即认为完成并产生 wr_finish。
//=============================================================================
module axi_burst_master #(
    parameter CTRL_ADDR_W = 28,     // AXI 字节地址宽度（= ROW+BANK+COL）
    parameter DW          = 256,
    parameter ADDR_W      = 25,     // 内部地址宽度（单位：256bit 字）
    parameter LEN_W       = 10      // 内部长度宽度（单位：256bit 字）
)(
    input  wire                     clk,
    input  wire                     rst_n,
    // ---------------- AXI（Simplified AXI4） ----------------
    output wire [3:0]               axi_awuser_id,
    output wire                     axi_awuser_ap,
    output wire [CTRL_ADDR_W-1:0]   axi_awaddr,
    output wire [3:0]               axi_awlen,
    output wire                     axi_awvalid,
    input  wire                     axi_awready,
    output wire [DW-1:0]            axi_wdata,
    output wire [DW/8-1:0]          axi_wstrb,
    output wire [3:0]               axi_wusero_id,
    output wire                     axi_wusero_last,
    output wire                     axi_wvalid,
    input  wire                     axi_wready,
    output wire [3:0]               axi_aruser_id,
    output wire                     axi_aruser_ap,
    output wire [CTRL_ADDR_W-1:0]   axi_araddr,
    output wire [3:0]               axi_arlen,
    output wire                     axi_arvalid,
    input  wire                     axi_arready,
    input  wire [DW-1:0]            axi_rdata,
    input  wire                     axi_rvalid,
    input  wire                     axi_rlast,
    input  wire [3:0]               axi_rid,
    // ---------------- 内部突发接口：写 ----------------
    input  wire                     wr_req,
    input  wire [LEN_W-1:0]         wr_len,
    input  wire [ADDR_W-1:0]        wr_addr,
    output wire                     wr_data_req,
    input  wire [DW-1:0]            wr_data,
    output wire                     wr_finish,
    // ---------------- 内部突发接口：读 ----------------
    input  wire                     rd_req,
    input  wire [LEN_W-1:0]         rd_len,
    input  wire [ADDR_W-1:0]        rd_addr,
    output wire                     rd_data_valid,
    output wire [DW-1:0]            rd_data,
    output wire                     rd_finish
);
    //=========================================================
    // 写通道
    //=========================================================
    localparam WS_IDLE = 2'd0, WS_AW = 2'd1, WS_W = 2'd2, WS_DONE = 2'd3;

    reg [1:0]       wst;
    reg [ADDR_W-1:0] w_addr_r;
    reg [4:0]       w_beat;        // 剩余 beat 数（len = beat）

    wire [4:0] w_len_b = (wr_len == 0) ? 5'd1 :
                         (wr_len > 16) ? 5'd16 : wr_len[4:0];
    wire       w_is_last = (w_beat == 5'd1);      // 当前即为最后一个 beat

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wst    <= WS_IDLE;
            w_addr_r <= {ADDR_W{1'b0}};
            w_beat <= 5'd0;
        end else begin
            case (wst)
                WS_IDLE: begin
                    if (wr_req) begin
                        w_addr_r <= wr_addr;
                        w_beat   <= w_len_b;
                        wst      <= WS_AW;
                    end
                end
                WS_AW: begin
                    if (axi_awready) wst <= WS_W;
                end
                WS_W: begin
                    if (axi_wvalid & axi_wready) begin
                        if (w_is_last) wst <= WS_DONE;
                        else           w_beat <= w_beat - 5'd1;
                    end
                end
                WS_DONE: wst <= WS_IDLE;
                default: wst <= WS_IDLE;
            endcase
        end
    end

    assign axi_awaddr     = {w_addr_r, 5'b00000};
    assign axi_awlen      = (w_len_b - 5'd1);       // beat 数 - 1
    assign axi_awvalid    = (wst == WS_AW);
    assign axi_awuser_id  = 4'h0;
    assign axi_awuser_ap  = 1'b1;                   // 自动 precharge（同 demo）

    assign axi_wdata      = wr_data;
    assign axi_wstrb      = {DW/8{1'b1}};
    assign axi_wusero_id  = 4'h0;
    assign axi_wusero_last= w_is_last;
    assign axi_wvalid     = (wst == WS_W);
    // 注意：每个 beat（含最后一个）都要从上游 FIFO 弹出一次，否则块边界会错位
    assign wr_data_req    = (wst == WS_W) & axi_wready;
    assign wr_finish      = (wst == WS_DONE);

    //=========================================================
    // 读通道
    //=========================================================
    localparam RS_IDLE = 2'd0, RS_AR = 2'd1, RS_R = 2'd2, RS_DONE = 2'd3;

    reg [1:0]        rst_st;
    reg [ADDR_W-1:0] r_addr_r;
    reg [4:0]        r_beat;
    reg              r_last;

    wire [4:0] r_len_b = (rd_len == 0) ? 5'd1 :
                         (rd_len > 16) ? 5'd16 : rd_len[4:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rst_st   <= RS_IDLE;
            r_addr_r <= {ADDR_W{1'b0}};
            r_beat   <= 5'd0;
            r_last   <= 1'b0;
        end else begin
            case (rst_st)
                RS_IDLE: begin
                    if (rd_req) begin
                        r_addr_r <= rd_addr;
                        r_beat   <= r_len_b;
                        r_last   <= 1'b0;
                        rst_st   <= RS_AR;
                    end
                end
                RS_AR: begin
                    if (axi_arready) rst_st <= RS_R;
                end
                RS_R: begin
                    if (axi_rvalid) begin
                        if (r_beat <= 5'd1) begin
                            r_last <= 1'b1;
                            rst_st <= RS_DONE;
                        end else begin
                            r_beat <= r_beat - 5'd1;
                        end
                    end
                end
                RS_DONE: rst_st <= RS_IDLE;
                default: rst_st <= RS_IDLE;
            endcase
        end
    end

    assign axi_araddr     = {r_addr_r, 5'b00000};
    assign axi_arlen      = (r_len_b - 5'd1);
    assign axi_arvalid    = (rst_st == RS_AR);
    assign axi_aruser_id  = 4'h0;
    assign axi_aruser_ap  = 1'b1;

    assign rd_data        = axi_rdata;
    assign rd_data_valid  = (rst_st == RS_R) & axi_rvalid;
    assign rd_finish      = (rst_st == RS_DONE);

endmodule
