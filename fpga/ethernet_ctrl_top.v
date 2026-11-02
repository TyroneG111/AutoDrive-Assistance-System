`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: ethernet_ctrl_top
// Description: 千兆以太网 UDP 视频流上传 + 用户数据收发 顶层
//
//   结构:
//     ethernet_ctrl_top
//       |-- pll_phase            (IP: 由 rgmii_rxc 生成相位时钟)
//       |-- rgmii_interface      (RGMII <-> GMII, GTP_OSERDES/ISERDES)
//       |-- udp_stack            (标准 UDP 协议栈: UDP/IP/MAC, 无 ARP/ICMP)
//       |      |-- udp_layer(udp_tx,udp_rx)
//       |      |-- ip_layer (ip_tx_mode,ip_tx,ip_rx,icmp)
//       |      |-- mac_layer(mac_tx_mode,mac_tx,mac_rx) + crc32_gen
//       |-- video_udp_ctrl       (视频缓存/打包/用户数据/RX 输出)
//       |-- vid_line_dram        (IP: 真双口行缓存 24bit x 8192)
//
//   时钟:
//     rgmii_clk  = 125MHz (由 PHY 提供的 rgmii_rxc 经 pll_phase + 缓冲得到)
//     vid_pclk   = 37.125MHz (HDMI 像素时钟, 外部输入)
//////////////////////////////////////////////////////////////////////////////////

module ethernet_ctrl_top #(
    parameter LOCAL_MAC  = 48'h3C_2B_1A_09_4D_5E,   // 本机 MAC
    parameter LOCAL_IP   = 32'hC0_A8_00_6E,         // 本机 IP 192.168.0.110 (与PC同网段)
    parameter LOCAL_PORT = 16'hF0F0,                // 本机端口
    parameter DEST_MAC   = 48'hC4_C6_E6_C9_A2_E1,   // 目的 MAC (写死)
    parameter DEST_IP    = 32'hC0_A8_00_03,         // 目的 IP  192.168.0.3
    parameter DEST_PORT  = 16'hA0A0                 // 目的端口
)(
    //---------- 系统 ----------
//    input         clk_50m,        // 系统 50MHz(预留, 本设计暂未使用)
    input         rst_n,          // 全局复位, 低有效(按键/上电)
    output        phy_rstn,       // PHY 复位, 低有效

    //---------- RGMII (接 RTL8211) ----------
    input         rgmii_rxc,
    input         rgmii_rx_ctl,
    input  [3:0]  rgmii_rxd,
    output        rgmii_txc,
    output        rgmii_tx_ctl,
    output [3:0]  rgmii_txd,

    //---------- HDMI 视频输入 (RGB888) ----------
    input         vid_pclk,       // 像素时钟 37.125MHz
    input         vid_hs,         // 行同步(正极性)
    input         vid_vs,         // 场同步(正极性)
    input         vid_de,         // 数据有效, 高有效
    input  [10:0] vid_hc,         // 行内像素号 0..1279
    input  [9:0]  vid_vc,         // 行号 0..719
    input  [23:0] vid_rgb,        // {R[7:0],G[7:0],B[7:0]}

    //---------- 双目融合图像 (video_top.stereo_*, fuse_clk 域) ----------
    input         fuse_clk,       // 74.25MHz (pclk 2x)
    input  [15:0] fuse_px,        // RGB565
    input         fuse_de,        // 像素有效 (每像素约 2 拍高)
    input  [10:0] fuse_hc,        // 融合画布列号 0..669
    input  [9:0]  fuse_vc,        // 融合行号 0..291

    //---------- 用户自定义数据 ----------
    input  [15:0] user_data_in,   // 16bit, 电平常有效

    //---------- RX 用户数据输出 ----------
    output        rx_data_valid,
    output [7:0]  rx_data,
    output [15:0] rx_data_length,

    //---------- 时延统计：一帧发送完成事件 (rgmii_clk 域电平翻转) ----------
    //   每发完一帧（到最后一个报文最后一个字节）翻转一次。
    //   用翻转而不是脉冲，以便安全跨到 clk_37_125M 域（8ns 脉冲会丢）。
    output        frame_done_tgl
);

    //---------------------------------------------------------------
    // 时钟与复位
    //---------------------------------------------------------------
    wire rgmii_clk;               // 125MHz 主逻辑时钟
    wire rgmii_rxc_phase;         // 相位时钟 (RX 采样居中)
    wire rgmii_rxc_phase_d;       // 相位时钟2 (TX 时钟)
    wire pll_lock;

    pll_phase pll_phase_inst (
        .clkin1   (rgmii_rxc),
        .clkout0  (rgmii_rxc_phase),
        .clkout1  (rgmii_rxc_phase_d),
        .pll_lock (pll_lock)
    );

    // 复位同步 (网络域)
    reg [2:0] rst_sync_n;
    always @(posedge rgmii_clk) rst_sync_n <= {rst_sync_n[1:0], rst_n};
    wire rstn = rst_sync_n[2];

    // 复位同步 (融合域 74.25MHz)
    reg [2:0] frst_sync_n;
    always @(posedge fuse_clk) frst_sync_n <= {frst_sync_n[1:0], rst_n};
    wire fuse_rstn = frst_sync_n[2];

    //---------------------------------------------------------------
    // 视频源选择
    //   VID_TEST_EN=1: 内部图案发生器(rgmii_clk 域)
    //   VID_TEST_EN=0: 外部 HDMI (vid_pclk 域)
    //---------------------------------------------------------------
    wire        vclk, vrstn, vhs, vvs, vde;
    wire [10:0] vhc;
    wire [9:0]  vvc;
    wire [23:0] vrgb;

    generate
      begin : g_vid_ext
        reg [2:0] vrst_sync_n;
        always @(posedge vid_pclk) vrst_sync_n <= {vrst_sync_n[1:0], rst_n};
        assign vclk  = vid_pclk;
        assign vrstn = vrst_sync_n[2];
        assign vhs   = vid_hs;
        assign vvs   = vid_vs;
        assign vde   = vid_de;
        assign vhc   = vid_hc;
        assign vvc   = vid_vc;
        assign vrgb  = vid_rgb;
      end
    endgenerate

    //---------------------------------------------------------------
    // RGMII 接口
    //---------------------------------------------------------------
    wire        mac_rx_data_valid;
    wire [7:0]  mac_rx_data;
    wire        mac_tx_data_valid;
    wire [7:0]  mac_tx_data;

    // GMII 侧打一拍(与原 demo 一致)
    reg         gmii_rx_dv_r;
    reg  [7:0]  gmii_rxd_r;
    reg         gmii_tx_en_r;
    reg  [7:0]  gmii_txd_r;
    always @(posedge rgmii_clk) begin
        gmii_rx_dv_r <= mac_rx_data_valid;
        gmii_rxd_r   <= mac_rx_data;
        gmii_tx_en_r <= mac_tx_data_valid;
        gmii_txd_r   <= mac_tx_data;
    end

    rgmii_interface rgmii_interface_inst (
        .rst                 (~rst_n),
        .rgmii_clk           (rgmii_clk),
        .rgmii_clk_90p       (1'b0),
        .mac_tx_data_valid   (gmii_tx_en_r),
        .mac_tx_data         (gmii_txd_r),
        .mac_rx_error        (),
        .mac_rx_data_valid   (mac_rx_data_valid),
        .mac_rx_data         (mac_rx_data),
        .rgmii_rxc           (rgmii_rxc_phase),
        .rgmii_rx_ctl        (rgmii_rx_ctl),
        .rgmii_rxd           (rgmii_rxd),
        .rgmii_rxc_phase_d   (rgmii_rxc_phase_d),
        .rgmii_txc           (rgmii_txc),
        .rgmii_tx_ctl        (rgmii_tx_ctl),
        .rgmii_txd           (rgmii_txd)
    );

    //---------------------------------------------------------------
    // 协议栈 <-> 视频控制 互联信号
    //---------------------------------------------------------------
    wire [15:0] app_data_length;
    wire        app_data_request;
    wire        app_rd_en;
    wire [7:0]  app_data_in;

    wire [7:0]  udp_rec_rdata;
    wire [15:0] udp_rec_data_length;
    wire        udp_rec_data_valid;

    wire        dram_wr_en;
    wire [12:0] dram_wr_addr;
    wire [23:0] dram_wr_data;
    wire [12:0] dram_rd_addr;
    wire [23:0] dram_rd_data;

    //---------------------------------------------------------------
    // 标准 UDP 协议栈
    //---------------------------------------------------------------
    udp_stack #(
        .LOCAL_MAC  (LOCAL_MAC),
        .LOCAL_IP   (LOCAL_IP),
        .LOCAL_PORT (LOCAL_PORT),
        .DEST_MAC   (DEST_MAC),
        .DEST_IP    (DEST_IP),
        .DEST_PORT  (DEST_PORT)
    ) udp_stack_inst (
        .clk                 (rgmii_clk),
        .rstn                (rstn),
        .app_data_length     (app_data_length),
        .app_data_request    (app_data_request),
        .app_rd_en           (app_rd_en),
        .app_data_in         (app_data_in),
        .udp_rec_rdata       (udp_rec_rdata),
        .udp_rec_data_length (udp_rec_data_length),
        .udp_rec_data_valid  (udp_rec_data_valid),
        .mac_data_valid      (mac_tx_data_valid),
        .mac_tx_data         (mac_tx_data),
        .rx_en               (gmii_rx_dv_r),
        .mac_rx_datain       (gmii_rxd_r)
    );

    //---------------------------------------------------------------
    // 视频流及用户数据控制
    //---------------------------------------------------------------
    video_udp_ctrl video_udp_ctrl_inst (
        .vid_pclk        (vclk),
        .vid_rstn        (vrstn),
        .vid_hs          (vhs),
        .vid_vs          (vvs),
        .vid_de          (vde),
        .vid_hc          (vhc),
        .vid_vc          (vvc),
        .vid_rgb         (vrgb),

        .fuse_clk        (fuse_clk    ),
        .fuse_rstn       (fuse_rstn   ),
        .fuse_px         (fuse_px     ),
        .fuse_de         (fuse_de     ),
        .fuse_hc         (fuse_hc     ),
        .fuse_vc         (fuse_vc     ),

        .clk             (rgmii_clk),
        .rstn            (rstn),

        .app_data_length (app_data_length),
        .app_data_request(app_data_request),
        .app_rd_en       (app_rd_en),
        .app_data_in     (app_data_in),

        .user_data_in    (user_data_in),

        .udp_rec_rdata       (udp_rec_rdata),
        .udp_rec_data_length (udp_rec_data_length),
        .udp_rec_data_valid  (udp_rec_data_valid),
        .rx_data_valid       (rx_data_valid),
        .rx_data             (rx_data),
        .rx_data_length      (rx_data_length),

        .dram_wr_en   (dram_wr_en),
        .dram_wr_addr (dram_wr_addr),
        .dram_wr_data (dram_wr_data),
        .dram_rd_addr (dram_rd_addr),
        .dram_rd_data (dram_rd_data),

        .frame_done_tgl (frame_done_tgl)
    );

    //---------------------------------------------------------------
    // 行缓存 真双口 RAM
    //---------------------------------------------------------------
    vid_line_dram #(
        .AW (13),
        .DW (24)
    ) vid_line_dram_inst (
        .wr_clk  (vclk),
        .wr_en   (dram_wr_en),
        .wr_addr (dram_wr_addr),
        .wr_data (dram_wr_data),
        .rd_clk  (rgmii_clk),
        .rd_addr (dram_rd_addr),
        .rd_data (dram_rd_data)
    );

    //---------------------------------------------------------------
    assign phy_rstn = rst_n;

endmodule
