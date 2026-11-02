`timescale 1ns/1ps
//=============================================================================
// hmic_wrap —— HMIC_S（DDR3）IP 例化封装（方案 ⑦ 后端）
//
// 依据 UG022003 + demo/hdmi_ddr_ov5640_top.v 的接法：
//   · MEM_DQ_WIDTH = 32  -> axi_wdata = 256bit，axi_awlen/arlen = 4bit
//   · Simplified AXI4：无 AWSIZE/AWBURST/ARLOCK/AWCACHE，无写响应通道 B
//   · 参数名采用 demo 中的写法（MEM_ROW_WIDTH/MEM_COLUMN_WIDTH/...）
//   · ddr_init_done 作为整个 core 域的复位释放条件
//
// 【待对接】模块名（demo 中为 IP 生成名）与 PHY 侧端口（dll_lock/init_slip_step/
//           force_read_clk_ctrl/ddrphy_gate_update_en/update_com_val_err_flag/
//           rd_fake_stop 等）请以 IPC 实际生成文件的端口名为准。
//           IP 参数见方案文档 §12.1
//=============================================================================
module hmic_wrap #(
    parameter MEM_ROW_ADDR_WIDTH = 15,
    parameter MEM_COL_ADDR_WIDTH = 10,
    parameter MEM_BADDR_WIDTH    = 3,
    parameter MEM_DQ_WIDTH       = 32,
    parameter CTRL_ADDR_WIDTH    = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH
)(
    // ---- 时钟/复位（端口名与 IPC 生成的 hmic_s_top 一致） ----
    input  wire                     ref_clk,        // 50MHz 输入时钟
    input  wire                     resetn,         // 低有效
    output wire                     core_clk,       // = ddrphy_clkin（AXI/MC 工作时钟）
    output wire                     ddr_init_done,
    output wire                     pll_lock,
    // ---- PHY 调试（可接 ILA） ----
    output wire [34*MEM_DQ_WIDTH/8-1:0] debug_data,
    output wire [13*MEM_DQ_WIDTH/8-1:0] debug_slice_state,
    output wire [21:0]              debug_calib_ctrl,
    output wire [7:0]               ck_dly_set_bin,
    output wire [7:0]               dll_step,
    output wire                     dll_lock,
    output wire [MEM_DQ_WIDTH/8-1:0] update_com_val_err_flag,
    // ---- DDR3 引脚 ----
    output wire                     mem_rst_n,
    output wire                     mem_ck,
    output wire                     mem_ck_n,
    output wire                     mem_cke,
    output wire                     mem_cs_n,
    output wire                     mem_ras_n,
    output wire                     mem_cas_n,
    output wire                     mem_we_n,
    output wire                     mem_odt,
    output wire [MEM_ROW_ADDR_WIDTH-1:0] mem_a,
    output wire [MEM_BADDR_WIDTH-1:0]    mem_ba,
    inout  wire [MEM_DQ_WIDTH-1:0]       mem_dq,
    inout  wire [MEM_DQ_WIDTH/8-1:0]     mem_dqs,
    inout  wire [MEM_DQ_WIDTH/8-1:0]     mem_dqs_n,
    output wire [MEM_DQ_WIDTH/8-1:0]     mem_dm,
    // ---- AXI（Simplified AXI4，接 axi_burst_master） ----
    input  wire [CTRL_ADDR_WIDTH-1:0]    axi_awaddr,
    input  wire                          axi_awuser_ap,
    input  wire [3:0]                    axi_awuser_id,
    input  wire [3:0]                    axi_awlen,
    input  wire                          axi_awvalid,
    output wire                          axi_awready,
    input  wire [MEM_DQ_WIDTH*8-1:0]     axi_wdata,
    input  wire [MEM_DQ_WIDTH-1:0]       axi_wstrb,
    input  wire                          axi_wusero_last,
    input  wire [3:0]                    axi_wusero_id,
    input  wire                          axi_wvalid,
    output wire                          axi_wready,
    input  wire [CTRL_ADDR_WIDTH-1:0]    axi_araddr,
    input  wire                          axi_aruser_ap,
    input  wire [3:0]                    axi_aruser_id,
    input  wire [3:0]                    axi_arlen,
    input  wire                          axi_arvalid,
    output wire                          axi_arready,
    output wire [MEM_DQ_WIDTH*8-1:0]     axi_rdata,
    output wire                          axi_rvalid,
    output wire                          axi_rlast,
    output wire [3:0]                    axi_rid
);

    //=========================================================
    // IP 例化（端口名与 ipcore/hmic_s_top/hmic_s_top.v 完全一致）
    //=========================================================
    hmic_s_top #(
        .MEM_ROW_WIDTH    (MEM_ROW_ADDR_WIDTH),
        .MEM_COLUMN_WIDTH (MEM_COL_ADDR_WIDTH),
        .MEM_BANK_WIDTH   (MEM_BADDR_WIDTH   ),
        .MEM_DQ_WIDTH     (MEM_DQ_WIDTH      ),
        .MEM_DM_WIDTH     (MEM_DQ_WIDTH/8    ),
        .MEM_DQS_WIDTH    (MEM_DQ_WIDTH/8    ),
        .CTRL_ADDR_WIDTH  (CTRL_ADDR_WIDTH   )
    ) u_hmic_s_top (
        .ref_clk                 (ref_clk),
        .resetn                  (resetn),
        .ddr_init_done           (ddr_init_done),
        .ddrphy_clkin            (core_clk),
        .pll_lock                (pll_lock),
        // ---- AXI ----
        .axi_awaddr              (axi_awaddr),
        .axi_awuser_ap           (axi_awuser_ap),
        .axi_awuser_id           (axi_awuser_id),
        .axi_awlen               (axi_awlen),
        .axi_awready             (axi_awready),
        .axi_awvalid             (axi_awvalid),
        .axi_wdata               (axi_wdata),
        .axi_wstrb               (axi_wstrb),
        .axi_wready              (axi_wready),
        .axi_wusero_id           (axi_wusero_id),
        .axi_wusero_last         (axi_wusero_last),
        .axi_araddr              (axi_araddr),
        .axi_aruser_ap           (axi_aruser_ap),
        .axi_aruser_id           (axi_aruser_id),
        .axi_arlen               (axi_arlen),
        .axi_arready             (axi_arready),
        .axi_arvalid             (axi_arvalid),
        .axi_rdata               (axi_rdata),
        .axi_rid                 (axi_rid),
        .axi_rlast               (axi_rlast),
        .axi_rvalid              (axi_rvalid),
        // ---- APB（未使用，tie off） ----
        .apb_clk                 (ref_clk),
        .apb_rst_n               (resetn),
        .apb_sel                 (1'b0),
        .apb_enable              (1'b0),
        .apb_addr                (8'd0),
        .apb_write               (1'b0),
        .apb_wdata               (16'd0),
        .apb_ready               (),
        .apb_rdata               (),
        .apb_int                 (),
        // ---- PHY 调试（输入 tie 0，输出供 ILA） ----
        .debug_data              (debug_data),
        .debug_slice_state       (debug_slice_state),
        .debug_calib_ctrl        (debug_calib_ctrl),
        .ck_dly_set_bin          (ck_dly_set_bin),
        .force_ck_dly_en         (1'b0),
        .force_ck_dly_set_bin    (8'd0),
        .dll_step                (dll_step),
        .dll_lock                (dll_lock),
        .init_read_clk_ctrl      (2'b0),
        .init_slip_step          (4'b0),
        .force_read_clk_ctrl     (1'b0),
        .ddrphy_gate_update_en   (1'b0),
        .update_com_val_err_flag (update_com_val_err_flag),
        .rd_fake_stop            (1'b0),
        // ---- DDR3 物理接口 ----
        .mem_rst_n               (mem_rst_n),
        .mem_ck                  (mem_ck),
        .mem_ck_n                (mem_ck_n),
        .mem_cke                 (mem_cke),
        .mem_cs_n                (mem_cs_n),
        .mem_ras_n               (mem_ras_n),
        .mem_cas_n               (mem_cas_n),
        .mem_we_n                (mem_we_n),
        .mem_odt                 (mem_odt),
        .mem_a                   (mem_a),
        .mem_ba                  (mem_ba),
        .mem_dqs                 (mem_dqs),
        .mem_dqs_n               (mem_dqs_n),
        .mem_dq                  (mem_dq),
        .mem_dm                  (mem_dm)
    );

endmodule
