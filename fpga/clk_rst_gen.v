`timescale 1ns/1ps
//=============================================================================
// clk_rst_gen —— 复位生成与分配（方案 ①）
//
// · 本工程不产生任何像素时钟：4 路 PCLK 由用户采集工程提供（各自独立时钟域）
// · TX/RX Fabric 时钟由 HSST IP 输出 o_p_clk2core_tx/rx 回环提供，见 §3
// · 本模块只负责：把“原始复位 + 各 IP 就绪标志”同步到每个时钟域，
//   并保证 core / tx 域在 DDR 初始化和 PLL 锁定之后才释放复位
//=============================================================================
module clk_rst_gen #(
    parameter NCH = 4
)(
    input  wire        arst_n,          // 原始复位（按键/外部，低有效）
    // ---- 各域时钟 ----
    input  wire [NCH-1:0] i_pix_clk,    // 4 路像素时钟
    input  wire        core_clk,        // HMIC_S 给出的 AXI/系统时钟
    input  wire        tx_clk,          // HSST TX Fabric 时钟（125MHz）
    input  wire        rx_clk,          // HSST RX Fabric 时钟
    input  wire        cfg_clk,         // 50MHz
    input  wire        hdmi_pclk,       // 74.25MHz 读出/显示像素域（外部 PLL）
    // ---- IP 就绪 ----
    input  wire        ddr_init_done,   // HMIC_S 初始化完成（core_clk 域）
    input  wire        pll_done,        // HSST PLL0 LOCK（cfg_clk 域）
    input  wire        txlane_done,     // HSST LANE2 TX 复位完成（cfg_clk 域）
    // ---- 各域复位（低有效） ----
    output wire [NCH-1:0] o_pix_rst_n,
    output wire        o_core_rst_n,
    output wire        o_tx_rst_n,
    output wire        o_rx_rst_n,
    output wire        o_cfg_rst_n,
    output wire        o_hdmi_rst_n,     // 74.25MHz 读出域复位（低有效）
    output wire        o_init_done      // 全局就绪（core 域）
);
    wire ip_ok = ddr_init_done & pll_done & txlane_done;
    wire arst_all = arst_n & ip_ok;

    // ---- 像素域：像素侧不需要等 DDR（写侧有自己的 FIFO/握手，就绪前不会发写请求） ----
    genvar gi;
    generate
        for (gi = 0; gi < NCH; gi = gi + 1) begin : g_pix_rst
            rst_sync u_rst_sync (
                .clk   (i_pix_clk[gi]),
                .arst_n(arst_n),
                .rst_n (o_pix_rst_n[gi])
            );
        end
    endgenerate

    // ---- 系统 / 发送 / 接收 / 配置 / 读出域：等 IP 全部就绪 ----
    rst_sync u_core_rst (.clk(core_clk ), .arst_n(arst_all), .rst_n(o_core_rst_n));
    rst_sync u_tx_rst   (.clk(tx_clk   ), .arst_n(arst_all), .rst_n(o_tx_rst_n  ));
    rst_sync u_rx_rst   (.clk(rx_clk   ), .arst_n(arst_all), .rst_n(o_rx_rst_n  ));
    rst_sync u_cfg_rst  (.clk(cfg_clk  ), .arst_n(arst_n  ), .rst_n(o_cfg_rst_n ));
    // 读出域：arst_all 之后释放；顶层会再与“被选通道首块就绪”相与后才放行读出器
    rst_sync u_hdmi_rst (.clk(hdmi_pclk), .arst_n(arst_all), .rst_n(o_hdmi_rst_n));

    assign o_init_done = ip_ok;

endmodule
