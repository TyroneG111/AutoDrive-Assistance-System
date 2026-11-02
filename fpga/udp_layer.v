`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: udp_layer
// Description: UDP 层封装: 例化 udp_tx(发送) 与 udp_rx(接收)
//////////////////////////////////////////////////////////////////////////////////

module udp_layer #(
    parameter LOCAL_PORT_NUM = 16'hf000    // 源端口号
)(
    input  wire        clk,                // 时钟 (rgmii_clk)
    input  wire        rstn,               // 复位, 低有效

    // ---- 上层应用(载荷)接口 : pull (接 udp_tx) ----
    input  wire [15:0] app_data_length,
    input  wire        app_data_request,
    output wire        app_rd_en,
    input  wire [7:0]  app_data_in,
    input  wire [15:0] udp_dest_port,

    // ---- 与 IP 层握手 ----
    output wire        udp_send_ready,
    output wire        udp_send_ack,
    input  wire        ip_send_ready,
    input  wire        ip_send_ack,
    output wire        udp_send_request,
    output wire        udp_data_out_valid,
    output wire [7:0]  udp_data_out,
    output wire [15:0] udp_packet_length,

    // ---- 接收 ----
    input  wire [7:0]  udp_rx_data,
    input  wire        udp_rx_req,
    input  wire        ip_checksum_error,
    input  wire        ip_addr_check_error,
    output wire [7:0]  udp_rec_rdata,
    output wire [15:0] udp_rec_data_length,
    output wire        udp_rec_data_valid
);

    udp_tx #(
        .LOCAL_PORT_NUM (LOCAL_PORT_NUM)
    ) udp_tx_inst (
        .clk                (clk),
        .rstn               (rstn),
        .app_data_length    (app_data_length),
        .app_data_request   (app_data_request),
        .app_rd_en          (app_rd_en),
        .app_data_in        (app_data_in),
        .udp_dest_port      (udp_dest_port),
        .udp_send_ready     (udp_send_ready),
        .udp_send_ack       (udp_send_ack),
        .ip_send_ready      (ip_send_ready),
        .ip_send_ack        (ip_send_ack),
        .udp_send_request   (udp_send_request),
        .udp_data_out_valid (udp_data_out_valid),
        .udp_data_out       (udp_data_out),
        .udp_packet_length  (udp_packet_length)
    );

    udp_rx #(
        .LOCAL_PORT (LOCAL_PORT_NUM)
    ) udp_rx_inst (
        .clk                 (clk),
        .rstn                (rstn),
        .udp_rx_data         (udp_rx_data),
        .udp_rx_req          (udp_rx_req),
        .ip_checksum_error   (ip_checksum_error),
        .ip_addr_check_error (ip_addr_check_error),
        .udp_rec_rdata       (udp_rec_rdata),
        .udp_rec_data_length (udp_rec_data_length),
        .udp_rec_data_valid  (udp_rec_data_valid)
    );

endmodule
