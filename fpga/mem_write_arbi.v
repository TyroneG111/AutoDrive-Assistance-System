`timescale 1ns/1ps
//=============================================================================
// mem_write_arbi —— 4 通道写突发仲裁（方案 ⑥）
//   · 严格轮询（Round-Robin），同一时刻只有一个通道占用总线
//   · 8000 周期看门狗：异常挂死时强制回 IDLE，避免整条通路僵死
//   · 内部突发接口约定（与 demo / aq_axi_master 一致）：
//       长度单位 = 256bit 字；地址单位 = 256bit 字
//   · wr_data 为纯组合直通，与 wr_data_req 同拍有效（vid_wr_path 的打包 FIFO
//     取数时序即按此约定设计）；若在此多打一拍，会造成每个突发的首字重复、
//     末字丢失。
//=============================================================================
module mem_write_arbi #(
    parameter NCH    = 4,
    parameter DW     = 256,
    parameter ADDR_W = 25,
    parameter LEN_W  = 10,
    parameter WD_MAX = 16'd8000
)(
    input  wire                     rst_n,
    input  wire                     clk,
    // ---- 各通道请求 ----
    input  wire [NCH-1:0]           ch_req,
    input  wire [NCH*LEN_W-1:0]     ch_len,
    input  wire [NCH*ADDR_W-1:0]    ch_addr,
    output wire [NCH-1:0]           ch_data_req,
    input  wire [NCH*DW-1:0]        ch_data,
    output wire [NCH-1:0]           ch_finish,
    // ---- 到存储器 ----
    output reg                      wr_req,
    output reg  [LEN_W-1:0]         wr_len,
    output reg  [ADDR_W-1:0]        wr_addr,
    input  wire                     wr_data_req,
    output wire [DW-1:0]            wr_data,
    input  wire                     wr_finish
);
    localparam SEL_W = (NCH <= 2) ? 1 : (NCH <= 4) ? 2 : 3;

    localparam S_IDLE = 2'd0, S_BEG = 2'd1, S_RUN = 2'd2, S_END = 2'd3;

    reg [SEL_W-1:0] sel;
    reg [SEL_W-1:0] sel_nxt;
    reg [1:0]       st;
    reg [15:0]      wd_cnt;
    reg [NCH-1:0]   fin_dec;

    // 选定通道的译码（用于把 data_req / finish 路由回该通道）
    wire [NCH-1:0] sel_dec = ({{(NCH-1){1'b0}}, 1'b1} << sel);

    assign ch_data_req = sel_dec & {NCH{wr_data_req}};
    assign ch_finish   = fin_dec;

    // 写数据：由被选中通道的 FIFO 直接组合给出（与 wr_data_req 同拍有效）
    assign wr_data = ch_data[sel*DW +: DW];

    // ---- 轮询：从 sel 开始找第一个有请求的通道 ----
    integer k;
    always @(*) begin
        sel_nxt = sel;
        for (k = 0; k < NCH; k = k + 1) begin
            if (ch_req[(sel + k) % NCH])
                sel_nxt = (sel + k) % NCH;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st      <= S_IDLE;
            sel     <= {SEL_W{1'b0}};
            wr_req  <= 1'b0;
            wr_len  <= {LEN_W{1'b0}};
            wr_addr <= {ADDR_W{1'b0}};
            fin_dec <= {NCH{1'b0}};
            wd_cnt  <= 16'd0;
        end else begin
            fin_dec <= {NCH{1'b0}};

            case (st)
                S_IDLE: begin
                    wr_req <= 1'b0;
                    wd_cnt <= 16'd0;
                    if (ch_req[sel_nxt]) begin
                        sel <= sel_nxt;
                        st  <= S_BEG;
                    end
                end
                S_BEG: begin
                    wr_len  <= ch_len[sel*LEN_W +: LEN_W];
                    wr_addr <= ch_addr[sel*ADDR_W +: ADDR_W];
                    wr_req  <= 1'b1;
                    st      <= S_RUN;
                end
                S_RUN: begin
                    if (wr_finish) begin
                        wr_req  <= 1'b0;
                        fin_dec[sel] <= 1'b1;
                        st      <= S_END;
                    end
                end
                S_END: begin
                    sel <= (sel == NCH-1) ? {SEL_W{1'b0}} : (sel + 1'b1);
                    st  <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase

            // 看门狗
            if (st == S_IDLE)
                wd_cnt <= 16'd0;
            else if (wd_cnt >= WD_MAX)
                st <= S_IDLE;
            else
                wd_cnt <= wd_cnt + 16'd1;
        end
    end

endmodule
