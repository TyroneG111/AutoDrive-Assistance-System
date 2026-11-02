`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: udp_tx
// Description: UDP 发送层
//   - 生成 UDP 首部 (源端口 / 目的端口 / 长度 / 校验和, 校验和置0不使用)
//   - 采用"拉取式(pull)"接口从上层(视频/用户数据控制模块)读取用户数据:
//       * udp_tx 在每一个载荷字节周期输出 app_rd_en = 1 脉冲
//       * 上层据此把自己的载荷地址指针 +1, 并在 app_data_in 给出"当前"字节
//     上层推荐写法:
//         always @(posedge clk) if(app_rd_en) addr <= addr + 1'b1; else addr <= 0;
//         assign app_data_in = payload_ram[addr];   // 组合输出
//   - 与 IP 层的握手沿用原 demo 的 ready/request/ack 机制(时序完全保持)
//////////////////////////////////////////////////////////////////////////////////

module udp_tx #(
    parameter LOCAL_PORT_NUM = 16'hf000    // 源端口号
)(
    input  wire        clk,                // 时钟 (rgmii_clk, 125MHz)
    input  wire        rstn,               // 复位, 低电平有效

    // ---- 上层应用(载荷)接口 : pull ----
    input  wire [15:0] app_data_length,    // 本包用户数据长度(字节, 含4字节自定义头)
    input  wire        app_data_request,   // 发送请求(电平): 有效期间连续发送
    output wire        app_rd_en,          // 载荷字节读脉冲(组合, 高=SEND_PACKET期间)
    input  wire [7:0]  app_data_in,        // 当前载荷字节(由上层根据内部地址给出)
    input  wire [15:0] udp_dest_port,      // 目的端口号

    // ---- 与 IP 层握手 ----
    output wire        udp_send_ready,
    output wire        udp_send_ack,
    input  wire        ip_send_ready,
    input  wire        ip_send_ack,

    // ---- 发往 IP 层 ----
    output wire        udp_send_request,
    output reg         udp_data_out_valid,
    output reg  [7:0]  udp_data_out,
    output reg  [15:0] udp_packet_length
);

    localparam CHECKSUM = 16'h0000;        // 不使用 UDP 校验和时, 校验和字段全0

    localparam IDLE        = 3'd0;
    localparam WAIT_ACK    = 3'd1;
    localparam WAIT_HDR    = 3'd2;   // 对齐拍: 使 UDP 首部正好与 IP 层载荷起点对齐
    localparam SEND_HEADER = 3'd3;
    localparam SEND_PACKET = 3'd4;

    reg  [2:0]  state, state_n;
    reg  [3:0]  cnt;          // UDP 首部字节索引 0..7
    reg  [15:0] pkt_len;      // 本包用户数据长度(发送开始时锁存)
    reg  [15:0] pay_cnt;      // 载荷字节计数

    assign udp_send_ready   = (state != IDLE);
    assign udp_send_request  = app_data_request;
    assign udp_send_ack      = ip_send_ready;
    // 组合读脉冲: 在 SEND_PACKET 期间持续有效, 上层据此同拍推进其载荷地址
    assign app_rd_en         = (state == SEND_PACKET);

    // ---- UDP 报文长度 = 用户数据长度 + 8(UDP首部) ----
    always @(posedge clk) begin
        if(!rstn) udp_packet_length <= 16'h0008;
        else      udp_packet_length <= app_data_length + 16'h0008;
    end

    // ---- 主状态机 ----
    always @(posedge clk) begin
        if(!rstn) state <= IDLE;
        else      state <= state_n;
    end

    always @(*) begin
        case(state)
            IDLE        : state_n = (app_data_request) ? WAIT_ACK : IDLE;
            WAIT_ACK    : state_n = (ip_send_ready)    ? WAIT_HDR : WAIT_ACK;
            WAIT_HDR    : state_n = SEND_HEADER;
            SEND_HEADER : state_n = (cnt == 4'd7)      ? SEND_PACKET : SEND_HEADER;
            SEND_PACKET : state_n = (pay_cnt == pkt_len - 16'd1) ? IDLE : SEND_PACKET;
            default     : state_n = IDLE;
        endcase
    end

    // ---- 首部字节计数 ----
    always @(posedge clk) begin
        if(!rstn)                     cnt <= 4'd0;
        else if(state == SEND_HEADER) cnt <= cnt + 1'b1;
        else if(state == IDLE)        cnt <= 4'd0;
    end

    // ---- 锁存本包载荷长度 ----
    always @(posedge clk) begin
        if(!rstn) pkt_len <= 16'd0;
        else if(state == WAIT_ACK && ip_send_ready) pkt_len <= app_data_length;    end

    // ---- 载荷字节计数 ----
    always @(posedge clk) begin
        if(!rstn)                     pay_cnt <= 16'd0;
        else if(state == SEND_PACKET) pay_cnt <= pay_cnt + 1'b1;
        else                          pay_cnt <= 16'd0;
    end

    // ---- 输出数据 / 有效 ----
    always @(posedge clk) begin
        if(!rstn) begin
            udp_data_out       <= 8'd0;
            udp_data_out_valid <= 1'b0;
        end
        else begin
            udp_data_out_valid <= 1'b0;
            if(state == SEND_HEADER) begin
                udp_data_out_valid <= 1'b1;
                case(cnt)
                    4'd0: udp_data_out <= LOCAL_PORT_NUM[15:8];
                    4'd1: udp_data_out <= LOCAL_PORT_NUM[7:0];
                    4'd2: udp_data_out <= udp_dest_port[15:8];
                    4'd3: udp_data_out <= udp_dest_port[7:0];
                    4'd4: udp_data_out <= udp_packet_length[15:8];
                    4'd5: udp_data_out <= udp_packet_length[7:0];
                    4'd6: udp_data_out <= CHECKSUM[15:8];
                    4'd7: udp_data_out <= CHECKSUM[7:0];
                    default: udp_data_out <= 8'd0;
                endcase
            end
            else if(state == SEND_PACKET) begin
                udp_data_out_valid <= 1'b1;
                udp_data_out       <= app_data_in;
            end
        end
    end

endmodule
