`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: udp_stack
// Description: 标准 UDP 协议栈顶层
//   层次:  UDP层(udp_layer) -> IP层(ip_layer) -> MAC层(mac_layer)
//   本工程目的 MAC/IP 由参数写死, 不使用 ARP/ICMP
//   TX 应用接口为"拉取式"(见 udp_tx 说明), RX 直接输出解析后的用户数据
//////////////////////////////////////////////////////////////////////////////////

module udp_stack #(
    parameter LOCAL_MAC  = 48'h3C_2B_1A_09_4D_5E,   // 本机 MAC
    parameter LOCAL_IP   = 32'hC0_A8_01_6E,         // 本机 IP 192.168.1.110
    parameter LOCAL_PORT = 16'hF0F0,                // 本机端口
    parameter DEST_MAC   = 48'hFF_FF_FF_FF_FF_FF,   // 目的 MAC (写死)
    parameter DEST_IP    = 32'hC0_A8_01_69,         // 目的 IP  192.168.1.105
    parameter DEST_PORT  = 16'hA0A0                 // 目的端口
)(
    input  wire        clk,                 // 125MHz (rgmii_clk)
    input  wire        rstn,                // 复位, 低有效

    // ---- TX 应用接口 (pull) ----
    input  wire [15:0] app_data_length,     // 本包用户数据长度(含4字节头)
    input  wire        app_data_request,    // 发送请求(电平)
    output wire        app_rd_en,           // 载荷字节读脉冲
    input  wire [7:0]  app_data_in,         // 当前载荷字节

    // ---- RX 应用接口 ----
    output wire [7:0]  udp_rec_rdata,       // 接收到的用户数据
    output wire [15:0] udp_rec_data_length, // 用户数据长度
    output wire        udp_rec_data_valid,  // 用户数据有效

    // ---- MAC 接口 (接 rgmii_interface) ----
    output wire        mac_data_valid,      // GMII 发送有效
    output wire [7:0]  mac_tx_data,         // GMII 发送数据
    input  wire        rx_en,               // GMII 接收有效
    input  wire [7:0]  mac_rx_datain        // GMII 接收数据
);

    // ---- UDP <-> IP ----
    wire        udp_send_ack;      // (未用, 供 app 握手, 本工程 pull 接口不需要)
    wire        udp_tx_req;        // udp -> ip  发送请求
    wire        udp_tx_ready;      // udp -> ip  发送就绪
    wire [7:0]  udp_tx_data;       // udp -> ip  数据
    wire [15:0] udp_pkt_len;       // udp -> ip  UDP 报文长度(含首部)
    wire        udp_tx_ack;        // ip  -> udp 发送应答
    wire        upper_data_req;    // ip  -> udp 请求上层数据(=ip_send_ready)
    wire        udp_rx_req;        // ip  -> udp 接收请求

    // ---- IP <-> MAC ----
    wire        ip_tx_req, ip_tx_ready, ip_tx_end, ip_tx_ack;
    wire [7:0]  ip_tx_data;
    wire        ip_rx_req, ip_rx_end;
    wire        mac_data_req, mac_send_end, mac_rec_error;
    wire [7:0]  mac_rx_dataout;
    wire [47:0] mac_rx_dest_mac_addr, mac_rx_sour_mac_addr;
    wire        ip_checksum_error;
    wire        ip_addr_check_error;
    wire [15:0] upper_layer_data_length;
    wire [7:0]  net_protocol;
    wire [31:0] ip_rec_source_addr, ip_rec_dest_addr;

    //--------------------------------------------------------------
    // UDP 层
    //--------------------------------------------------------------
    udp_layer #(
        .LOCAL_PORT_NUM (LOCAL_PORT)
    ) udp_layer_inst (
        .clk                 (clk),
        .rstn                (rstn),
        .app_data_length     (app_data_length),
        .app_data_request    (app_data_request),
        .app_rd_en           (app_rd_en),
        .app_data_in         (app_data_in),
        .udp_dest_port       (DEST_PORT),
        .udp_send_ready      (udp_tx_ready),
        .udp_send_ack        (udp_send_ack),
        .ip_send_ready       (upper_data_req),
        .ip_send_ack         (udp_tx_ack),
        .udp_send_request    (udp_tx_req),
        .udp_data_out_valid  (),
        .udp_data_out        (udp_tx_data),
        .udp_packet_length   (udp_pkt_len),
        .udp_rx_data         (mac_rx_dataout),
        .udp_rx_req          (udp_rx_req),
        .ip_checksum_error   (ip_checksum_error),
        .ip_addr_check_error (ip_addr_check_error),
        .udp_rec_rdata       (udp_rec_rdata),
        .udp_rec_data_length (udp_rec_data_length),
        .udp_rec_data_valid  (udp_rec_data_valid)
    );

    //--------------------------------------------------------------
    // IP 层
    //--------------------------------------------------------------
    ip_layer #(
        .LOCAL_IP  (LOCAL_IP),
        .LOCAL_MAC (LOCAL_MAC)
    ) ip_layer_inst (
        .clk                     (clk),
        .rstn                    (rstn),
        // MAC TX
        .mac_tx_ack              (ip_tx_ack),
        .mac_send_end            (mac_send_end),
        .mac_data_req            (mac_data_req),
        .ip_tx_req               (ip_tx_req),
        .ip_tx_ready             (ip_tx_ready),
        .ip_tx_data              (ip_tx_data),
        .ip_tx_end               (ip_tx_end),
        // MAC RX
        .ip_rx_data              (mac_rx_dataout),
        .ip_rx_req               (ip_rx_req),
        .mac_rx_dest_mac_addr    (mac_rx_dest_mac_addr),
        .ip_rx_end               (ip_rx_end),
        .mac_rec_err             (mac_rec_error),
        // UPPER TX
        .dest_mac_addr           (DEST_MAC),
        .dest_ip_addr            (DEST_IP),
        .udp_tx_req              (udp_tx_req),
        .udp_tx_ready            (udp_tx_ready),
        .udp_tx_data             (udp_tx_data),
        .udp_send_data_length    (udp_pkt_len),
        .udp_tx_ack              (udp_tx_ack),
        .upper_data_req          (upper_data_req),
        // UPPER RX
        .udp_rx_req              (udp_rx_req),
        .ip_addr_check_error     (ip_addr_check_error),
        .net_protocol            (net_protocol),
        .ip_rec_source_addr      (ip_rec_source_addr),
        .ip_rec_dest_addr        (ip_rec_dest_addr),
        .upper_layer_data_length (upper_layer_data_length),
        .ip_checksum_error       (ip_checksum_error)
    );

    //--------------------------------------------------------------
    // MAC 层 (ARP 相关信号置0)
    //--------------------------------------------------------------
    mac_layer mac_layer_inst (
        .clk                  (clk),
        .rstn                 (rstn),
        .arp_tx_req           (1'b0),
        .arp_tx_ready         (1'b0),
        .arp_tx_data          (8'd0),
        .arp_tx_end           (1'b0),
        .arp_tx_ack           (),
        .ip_tx_req            (ip_tx_req),
        .ip_tx_ready          (ip_tx_ready),
        .ip_tx_data           (ip_tx_data),
        .ip_tx_end            (ip_tx_end),
        .ip_tx_ack            (ip_tx_ack),
        .mac_data_req         (mac_data_req),
        .mac_send_end         (mac_send_end),
        .mac_tx_data          (mac_tx_data),
        .mac_data_valid       (mac_data_valid),
        .rx_en                (rx_en),
        .mac_rx_datain        (mac_rx_datain),
        .checksum_err         (ip_checksum_error),
        .ip_rx_end            (ip_rx_end),
        .ip_rx_req            (ip_rx_req),
        .arp_rx_end           (1'b0),
        .arp_rx_req           (),
        .mac_rx_dataout       (mac_rx_dataout),
        .mac_rec_error        (mac_rec_error),
        .mac_rx_dest_mac_addr (mac_rx_dest_mac_addr),
        .mac_rx_sour_mac_addr (mac_rx_sour_mac_addr)
    );

endmodule
