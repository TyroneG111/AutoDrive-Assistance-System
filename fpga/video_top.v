`timescale 1ns/1ps
//=============================================================================
// video_top —— 4 路标准 24bit YUV444 视频流 -> DDR3(4 块/路轮转) -> 光栅序拼接 -> HDMI
//   平台：MES50HP / PGL50H-6IFBG484      设计文档：v1.3
//
//   【2026-10 改造】
//     · 输入由 16bit/像素(YUV422) 改为 24bit/像素(YUV444)；DDR 每 32bit 通道存 1 个
//       24bit 像素（高 8bit 补零）-> 256bit 字 = 8 像素；一帧 = 115200 字，一行 = 160 字
//     · 输出由 DMT 720p60 改为 720p30（消隐周期数不变，pclk 74.25->37.125MHz）；
//       取消“每源帧显示两遍”的复用逻辑，每个显示帧即一个源帧。
//     · 因输入已是 444，dmt_pix_proc 内不再做 422->444 转换。
//
//   需例化的第三方 IP（参数见方案文档 §12）：
//     · hmic_s_top          —— HMIC_S DDR3 IP（Simplified AXI4，256bit）
//     · afifo_32i_256o_4096 —— DRM Based FIFO（32bit -> 256bit，写侧打包）×4
//
//   输入约定（用户采集工程提供，四方一致）：
//     pclk = 74.25MHz；每行 2560 pclk，其中 de 隔拍有效（1280 拍 = 1280 像素）；
//     vs 为 5 拍高有效同步脉冲；每帧 720 行；24bit/像素（YUV444）
//     【时序兼容性】接收侧（vid_frontend）只依赖“每行 de 为高的拍数 = H_DE_PULSES(1280)”，
//     不依赖 de 窗口宽度与交替相位。故把行时序改为 H_ACTIVE=1280、de 恒高、
//     H_FRONT=1500（仍 3300 pclk/行，V_* 不变）同样正常工作，本模块无需任何修改。
//
//   本工程只做“4 路图像数据 -> DDR3 -> 光栅序拼接 -> HDMI”，不做采集/缩放/格式转换
//
//   【读出模式 v2】顶层新增 i_mode[4:0] 五位独热输入（无需约束）：
//     bit0 = 拼接模式：4 路各自 2x2 网格平均缩小到 640x360，拼成 1280x720
//     bit1..bit4 = 单独输出 CH0/CH1/CH2/CH3（1280x720 原始分辨率）
//   五种模式的输出几何统一为 1280x720@30fps。
//   模式在“每帧读出起始”锁存，整帧同一模式 -> 严格无撕裂。
//=============================================================================
module video_top #(
    // ---------------- 视频时序 ----------------
    parameter H_TOTAL       = 3300,
    parameter V_TOTAL       = 750,
    parameter H_DE_PULSES   = 1280,
    parameter V_ACTIVE      = 720,
    // ---------------- DDR3 存储规划 ----------------
    parameter ADDR_W        = 25,           // 内部地址宽度（单位：256bit 字）
    parameter LEN_W         = 10,
    parameter BURST_WORDS   = 16,           // 512B/突发
    parameter BLK_WORDS     = 115200,       // 一帧 = 1280*720/8 字（每字 8 像素）
    parameter BLK_ALIGN     = 17,           // 块按 2^17 字（4MB）对齐（> 115200）
    parameter CTRL_ADDR_W   = 28,           // AXI 字节地址宽度
    // ---------------- 显示读出（DMT 720p30 @37.125MHz） ----------------
    parameter H_ACT         = 1280,         // 水平有效
    parameter H_FP          = 110,          // 水平前肩
    parameter H_SYN         = 40,           // 水平同步
    parameter H_BP          = 220,          // 水平后肩  -> 总计 1650
    parameter V_FP          = 5,            // 垂直前肩
    parameter V_SYN         = 5,            // 垂直同步
    parameter V_BP          = 20,           // 垂直后肩  -> 总计 750
    // 读出器在途突发上限 / 帧起点看门狗（core 100MHz 拍数，约 200ms）
    parameter RD_IFLIGHT    = 4,
    parameter WD_CYCLES     = 25_000_000,

    // DMT 像素域图像处理级增益（RGB565 三通道增益，256 = 恒等/旁路）
    parameter [8:0] PROC_GAIN = 9'd256,
    // ---------------- 光栅读出几何（每显示行 = 160 个 256bit 字） ----------------
    parameter ROW_WORDS     = H_DE_PULSES/8,                // 每显示行 = 160 个 256bit 字
    parameter ROW_BURSTS    = (H_DE_PULSES/8)/BURST_WORDS   // 每显示行突发数 = 10
)(
    // ---------------- 系统 ----------------
    input  wire         sys_clk,            // 50MHz 板载
    input  wire         clk_74_25M,         // 74.25MHz 融合/像素域 (27M x 11/4)
    input  wire         rst_n,              // 低有效复位
    output wire         all_init_done,
    // ---------------- DDR3 引脚（MES50HP 硬件手册表 2-4） ----------------
    output wire         mem_rst_n,
    output wire         mem_ck,
    output wire         mem_ck_n,
    output wire         mem_cke,
    output wire         mem_cs_n,
    output wire         mem_ras_n,
    output wire         mem_cas_n,
    output wire         mem_we_n,
    output wire         mem_odt,
    output wire [14:0]  mem_a,
    output wire [2:0]   mem_ba,
    inout  wire [31:0]  mem_dq,
    inout  wire [3:0]   mem_dqs,
    inout  wire [3:0]   mem_dqs_n,
    output wire [3:0]   mem_dm,
    output wire         ddr_init_done,
    // ---------------- 4 路视频输入 ----------------
    input  wire [3:0]   i_vid_pclk,
    input  wire [3:0]   i_vid_vs,
    input  wire [3:0]   i_vid_de,
    input  wire [95:0]  i_vid_data,         // 每路 24bit YUV444，{ch3,ch2,ch1,ch0}
    // ---------------- 读出模式（五位独热，帧边界生效；纯逻辑输入，无需约束） ----------------
    input  wire [4:0]   i_mode,             // bit0=拼接缩小  bit1..4=单路 CH0..CH3
    // ---------------- 按键 / LED ----------------
    input  wire [1:0]   key,                // key[0] = 复位（低有效）
    output wire         led,
    // ---------------- HDMI 输出（37.125MHz DMT 720p30；仅数据 + 同步，不含配置） ----------------
    input  wire         i_hdmi_pclk,        // 37.125MHz 像素时钟（模块外部 PLL 提供）
    output wire         o_hdmi_de,
    output wire         o_hdmi_hs,
    output wire         o_hdmi_vs,
    output wire [23:0]  o_hdmi_data,
	output wire [10:0]	o_hdmi_hc,
	output wire [9:0]	o_hdmi_vc,
    // ---------------- 双目拼接视图输出 -----------------
    //   stereo_hc = 融合画布列号 0..669 (11bit)，stereo_vc = 融合行号 0..291 (10bit)
    //   注意位宽与 stereo_fusion_top 的 out_hc[10:0]/out_vc[9:0] 严格对应
    output wire [15:0]  stereo_px,
    output wire         stereo_de,  
    output wire [9:0]   stereo_vc,
    output wire [10:0]  stereo_hc,
    // ---------------- 当前输出帧的读出模式（五位独热，帧边界更新） ----------------
    //   与 i_mode 的区别：i_mode 是按键随时给出的请求；本输出是“当前正在输出的
    //   这一帧实际采用的模式”，在帧起点锁存、一帧之内恒定，与画面严格对应。
    output wire [4:0]   o_mode_frame,
    // ---------------- [调试] DebugCore 采样总线 ----------------
    //   只在 proj_top 里定义 USE_DEBUG_CORE 时才会被使用；否则本信号无负载，
    //   会被综合优化掉，零资源开销。bit 定义见文件末尾的 assign。
    output wire [63:0]  o_dbg_bus
);
    //=========================================================
    // 复位 / 时钟
    //=========================================================
    wire rst_n_all = rst_n & key[0];

    //=========================================================
    // 像素域时钟别名（外部 PLL 的 i_hdmi_pclk，直连，不经过任何逻辑）
    //   720p30 时为 37.125MHz。
    //   ★必须在首次使用之前显式声明：否则 hdmi_pclk 会变成隐式 1bit 悬空网，
    //     导致整个像素域没有时钟。
    //=========================================================
    wire hdmi_pclk = i_hdmi_pclk;

    wire core_clk;
    wire [3:0] pix_rst_n;
    wire core_rst_n, cfg_rst_n;
    // clk_rst_gen 的 tx/rx fabric 时钟输入已无下游消费者，用 core_clk 占位
    //（其内部 rst_sync 因输出悬空会被综合移除）。就绪标志固定为 1，
    // 使 ip_ok 只依赖 ddr_init_done，像素域无需等待 PLL 即可释放复位。
    wire tx_clk = core_clk;
    wire rx_clk = core_clk;
    wire tx_rst_n, rx_rst_n;
    wire phy_pll_done    = 1'b1;
    wire phy_txlane_done = 1'b1;
    wire phy_rxlane_done = 1'b1;

    clk_rst_gen u_clk_rst (
        .arst_n        (rst_n_all),
        .i_pix_clk     (i_vid_pclk),
        .core_clk      (core_clk),
        .tx_clk        (tx_clk),
        .rx_clk        (rx_clk),
        .cfg_clk       (sys_clk),
        .hdmi_pclk     (i_hdmi_pclk),
        .ddr_init_done (ddr_init_done),
        .pll_done      (phy_pll_done),
        .txlane_done   (phy_txlane_done),
        .o_pix_rst_n   (pix_rst_n),
        .o_core_rst_n  (core_rst_n),
        .o_tx_rst_n    (tx_rst_n),
        .o_rx_rst_n    (rx_rst_n),
        .o_cfg_rst_n   (cfg_rst_n),
        .o_hdmi_rst_n  (hdmi_rst_n),
        .o_init_done   (all_init_done)
    );


    //=========================================================
    // DDR3（HMIC_S）
    //=========================================================
    wire [CTRL_ADDR_W-1:0] axi_awaddr, axi_araddr;
    wire                   axi_awuser_ap, axi_awvalid, axi_awready;
    wire [3:0]             axi_awuser_id, axi_awlen;
    wire [255:0]           axi_wdata, axi_rdata;
    wire [31:0]            axi_wstrb;
    wire                   axi_wusero_last, axi_wvalid, axi_wready;
    wire [3:0]             axi_wusero_id, axi_aruser_id, axi_arlen, axi_rid;
    wire                   axi_aruser_ap, axi_arvalid, axi_arready;
    wire                   axi_rvalid, axi_rlast;
    wire                   ddr_pll_lock, ddr_dll_lock;

    hmic_wrap #(
        .MEM_ROW_ADDR_WIDTH (15),
        .MEM_COL_ADDR_WIDTH (10),
        .MEM_BADDR_WIDTH    (3),
        .MEM_DQ_WIDTH       (32),
        .CTRL_ADDR_WIDTH    (CTRL_ADDR_W)
    ) u_hmic_wrap (
        .ref_clk         (sys_clk),
        .resetn          (rst_n_all),
        .core_clk        (core_clk),
        .ddr_init_done   (ddr_init_done),
        .pll_lock        (ddr_pll_lock),
        .debug_data      (),
        .debug_slice_state(),
        .debug_calib_ctrl(),
        .ck_dly_set_bin  (),
        .dll_step        (),
        .dll_lock        (ddr_dll_lock),
        .update_com_val_err_flag(),
        .mem_rst_n       (mem_rst_n),
        .mem_ck          (mem_ck),
        .mem_ck_n        (mem_ck_n),
        .mem_cke         (mem_cke),
        .mem_cs_n        (mem_cs_n),
        .mem_ras_n       (mem_ras_n),
        .mem_cas_n       (mem_cas_n),
        .mem_we_n        (mem_we_n),
        .mem_odt         (mem_odt),
        .mem_a           (mem_a),
        .mem_ba          (mem_ba),
        .mem_dq          (mem_dq),
        .mem_dqs         (mem_dqs),
        .mem_dqs_n       (mem_dqs_n),
        .mem_dm          (mem_dm),
        .axi_awaddr      (axi_awaddr),
        .axi_awuser_ap   (axi_awuser_ap),
        .axi_awuser_id   (axi_awuser_id),
        .axi_awlen       (axi_awlen),
        .axi_awvalid     (axi_awvalid),
        .axi_awready     (axi_awready),
        .axi_wdata       (axi_wdata),
        .axi_wstrb       (axi_wstrb),
        .axi_wusero_last (axi_wusero_last),
        .axi_wusero_id   (axi_wusero_id),
        .axi_wvalid      (axi_wvalid),
        .axi_wready      (axi_wready),
        .axi_araddr      (axi_araddr),
        .axi_aruser_ap   (axi_aruser_ap),
        .axi_aruser_id   (axi_aruser_id),
        .axi_arlen       (axi_arlen),
        .axi_arvalid     (axi_arvalid),
        .axi_arready     (axi_arready),
        .axi_rdata       (axi_rdata),
        .axi_rvalid      (axi_rvalid),
        .axi_rlast       (axi_rlast),
        .axi_rid         (axi_rid)
    );

    //=========================================================
    // AXI 突发主控（Simplified AXI4）
    //=========================================================
    wire              wr_arb_req, wr_arb_dreq, wr_arb_fin;
    wire [LEN_W-1:0]  wr_arb_len;
    wire [ADDR_W-1:0] wr_arb_addr;
    wire [255:0]      wr_arb_data;
    wire              axi_rd_req, axi_rd_data_valid, axi_rd_finish;
    wire [LEN_W-1:0]  axi_rd_len;
    wire [ADDR_W-1:0] axi_rd_addr;
    wire [255:0]      axi_rd_data;

    axi_burst_master #(
        .CTRL_ADDR_W (CTRL_ADDR_W),
        .DW          (256),
        .ADDR_W      (ADDR_W),
        .LEN_W       (LEN_W)
    ) u_axi_mst (
        .clk             (core_clk),
        .rst_n           (core_rst_n),
        // AXI
        .axi_awuser_id   (axi_awuser_id),
        .axi_awuser_ap   (axi_awuser_ap),
        .axi_awaddr      (axi_awaddr),
        .axi_awlen       (axi_awlen),
        .axi_awvalid     (axi_awvalid),
        .axi_awready     (axi_awready),
        .axi_wdata       (axi_wdata),
        .axi_wstrb       (axi_wstrb),
        .axi_wusero_id   (axi_wusero_id),
        .axi_wusero_last (axi_wusero_last),
        .axi_wvalid      (axi_wvalid),
        .axi_wready      (axi_wready),
        .axi_aruser_id   (axi_aruser_id),
        .axi_aruser_ap   (axi_aruser_ap),
        .axi_araddr      (axi_araddr),
        .axi_arlen       (axi_arlen),
        .axi_arvalid     (axi_arvalid),
        .axi_arready     (axi_arready),
        .axi_rdata       (axi_rdata),
        .axi_rvalid      (axi_rvalid),
        .axi_rlast       (axi_rlast),
        .axi_rid         (axi_rid),
        // 内部写接口（经仲裁器）
        .wr_req          (wr_arb_req),
        .wr_len          (wr_arb_len),
        .wr_addr         (wr_arb_addr),
        .wr_data_req     (wr_arb_dreq),
        .wr_data         (wr_arb_data),
        .wr_finish       (wr_arb_fin),
        // 内部读接口（直连光栅读出器：只有一个读主）
        .rd_req          (axi_rd_req),
        .rd_len          (axi_rd_len),
        .rd_addr         (axi_rd_addr),
        .rd_data_valid   (axi_rd_data_valid),
        .rd_data         (axi_rd_data),
        .rd_finish       (axi_rd_finish)
    );

    //=========================================================
    // 4 路写通路 + 写仲裁
    //=========================================================
    wire [3:0]          ch_wr_req, ch_wr_dreq, ch_wr_fin;
    wire [4*LEN_W-1:0]  ch_wr_len;
    wire [4*ADDR_W-1:0] ch_wr_addr;
    wire [4*256-1:0]    ch_wr_data;
    wire [7:0]          ch_blk_wr;
    wire [3:0]          ch_newframe, ch_first, ch_stale, ch_ferr;
    wire [4*32-1:0]     ch_ovf, ch_err, ch_fovf;

    // 读出块号（像素域）同步回本域，供写侧避让正在被读的块
    reg  [7:0]          blkrd_c1, blkrd_c2;

    genvar gi;
    generate
        for (gi = 0; gi < 4; gi = gi + 1) begin : g_ch
            vid_wr_path #(
                .CH_ID       (gi),
                .DW          (256),
                .ADDR_W      (ADDR_W),
                .LEN_W       (LEN_W),
                .BURST_WORDS (BURST_WORDS),
                .BLK_WORDS   (BLK_WORDS),
                .BLK_ALIGN   (BLK_ALIGN),
                .FIFO_AW     (9),
                .H_DE_PULSES (H_DE_PULSES),
                .V_ACTIVE    (V_ACTIVE),
                .H_TOTAL     (H_TOTAL),
                .V_TOTAL     (V_TOTAL)
            ) u_wr_path (
                .pclk         (i_vid_pclk[gi]),
                .pclk_rst_n   (pix_rst_n[gi]),
                .vs           (i_vid_vs[gi]),
                .de           (i_vid_de[gi]),
                .pix          (i_vid_data[gi*24 +: 24]),
                .core_clk     (core_clk),
                .core_rst_n   (core_rst_n),
                .wr_req       (ch_wr_req[gi]),
                .wr_len       (ch_wr_len[gi*LEN_W +: LEN_W]),
                .wr_addr      (ch_wr_addr[gi*ADDR_W +: ADDR_W]),
                .wr_data_req  (ch_wr_dreq[gi]),
                .wr_data      (ch_wr_data[gi*256 +: 256]),
                .wr_finish    (ch_wr_fin[gi]),
                .blk_wr       (ch_blk_wr[gi*2 +: 2]),
                .blk_rd       (blkrd_c2[gi*2 +: 2]),
                .new_frame    (ch_newframe[gi]),
                .first_frame  (ch_first[gi]),
                .ovf_cnt      (ch_ovf[gi*32 +: 32]),
                .err_cnt      (ch_err[gi*32 +: 32]),
                .fifo_ovf_cnt (ch_fovf[gi*32 +: 32]),
                .stale        (ch_stale[gi]),
                .frame_err    (ch_ferr[gi]),
                .ln_cnt_dbg   ()
            );
        end
    endgenerate

    mem_write_arbi #(
        .NCH    (4),
        .DW     (256),
        .ADDR_W (ADDR_W),
        .LEN_W  (LEN_W)
    ) u_wr_arb (
        .rst_n        (core_rst_n),
        .clk          (core_clk),
        .ch_req       (ch_wr_req),
        .ch_len       (ch_wr_len),
        .ch_addr      (ch_wr_addr),
        .ch_data_req  (ch_wr_dreq),
        .ch_data      (ch_wr_data),
        .ch_finish    (ch_wr_fin),
        .wr_req       (wr_arb_req),
        .wr_len       (wr_arb_len),
        .wr_addr      (wr_arb_addr),
        .wr_data_req  (wr_arb_dreq),
        .wr_data      (wr_arb_data),
        .wr_finish    (wr_arb_fin)
    );


    //=========================================================
    // 读出模式同步（像素域）+ 写侧块标志同步
    //   模式只在“每帧起点”被 rd_line_gen 锁存 -> 整帧同一模式，严格无撕裂
    //=========================================================
    reg  [4:0] mode_h1, mode_h2, mode_h3;
    reg  [4:0] mode_cur;

    wire mode_h_ok = (mode_h3 == mode_h2) && (|mode_h3) &&
                     ((mode_h3 & (mode_h3 - 5'd1)) == 5'd0);

    always @(posedge hdmi_pclk or negedge hdmi_rst_n) begin
        if (!hdmi_rst_n) begin
            mode_h1 <= 5'b00001; mode_h2 <= 5'b00001; mode_h3 <= 5'b00001;
            mode_cur <= 5'b00001;
        end else begin
            mode_h1 <= i_mode;
            mode_h2 <= mode_h1;
            mode_h3 <= mode_h2;
            if (mode_h_ok) mode_cur <= mode_h3;
        end
    end

    // 写侧“最新完整块 / 首帧就绪”标志（core -> hdmi_pclk，帧级稳定，三级同步）
    reg [7:0] blkwr_s1, blkwr_s2, blkwr_s3;
    reg [3:0] chrdy_s1, chrdy_s2, chrdy_s3;
    always @(posedge hdmi_pclk or negedge hdmi_rst_n) begin
        if (!hdmi_rst_n) begin
            blkwr_s1 <= 8'd0; blkwr_s2 <= 8'd0; blkwr_s3 <= 8'd0;
            chrdy_s1 <= 4'd0; chrdy_s2 <= 4'd0; chrdy_s3 <= 4'd0;
        end else begin
            blkwr_s1 <= ch_blk_wr;
            blkwr_s2 <= blkwr_s1;
            blkwr_s3 <= blkwr_s2;
            chrdy_s1 <= ch_first;
            chrdy_s2 <= chrdy_s1;
            chrdy_s3 <= chrdy_s2;
        end
    end

    // 被选通道首块就绪（单路只等该路，拼接等 4 路）；rd_start 释放读出器
    wire [3:0] ch_en_h  = mode_cur[0] ? 4'b1111 : mode_cur[4:1];
    wire       rd_start = &(chrdy_s3 | ~ch_en_h);

    //=========================================================
    // DMT 720p30 时序（37.125MHz）+ 行缓存 + 按行读出器 + DDR 读桥
    //   四个模块全部在同一像素域，行/帧结构与消隐完全确定
    //=========================================================
    wire         dmt_de, dmt_hs, dmt_vs;
    wire [10:0]  dmt_hc;
    wire [9:0]   dmt_vc;
    wire         dmt_run;

    dmt_tmg_gen #(
        .H_ACT  (H_ACT),
        .H_FP   (H_FP),
        .H_SYNC (H_SYN),
        .H_BP   (H_BP),
        .V_ACT  (V_ACTIVE),
        .V_FP   (V_FP),
        .V_SYNC (V_SYN),
        .V_BP   (V_BP)
    ) u_dmt (
        .pclk  (hdmi_pclk),
        .rst_n (hdmi_rst_n),
        .run   (dmt_run),
        .de    (dmt_de),
        .hs    (dmt_hs),
        .vs    (dmt_vs),
        .hc    (dmt_hc),
        .vc    (dmt_vc)
    );

    wire         lb_wr_vld;
    wire [255:0] lb_wr_word;
    wire         lb_line_ok;
    wire         lb_line_rdy;
    wire         lb_line_end;
    wire         lb_o_de, lb_o_hs, lb_o_vs;
    wire [23:0]  lb_o_px;
    wire [10:0]  lb_o_hc;
    wire [9:0]   lb_o_vc;

    hdmi_line_buf #(
        .NPIX    (H_ACT),
        .WPCNT   (ROW_WORDS),
        .NPAGE   (4),
        .H_TOTAL (H_ACT + H_FP + H_SYN + H_BP),
        .V_ACT   (V_ACTIVE),
        .V_TOTAL (V_ACTIVE + V_FP + V_SYN + V_BP)
    ) u_line_buf (
        .pclk     (hdmi_pclk),
        .rst_n    (hdmi_rst_n),
        .wr_vld   (lb_wr_vld),
        .wr_word  (lb_wr_word),
        .line_end (lb_line_end),
        .hc       (dmt_hc),
        .vc       (dmt_vc),
        .de       (dmt_de),
        .hs       (dmt_hs),
        .vs       (dmt_vs),
        .o_de     (lb_o_de),
        .o_hs     (lb_o_hs),
        .o_vs     (lb_o_vs),
        .o_px     (lb_o_px),
        .o_hc     (lb_o_hc),
        .o_vc     (lb_o_vc),
        .line_ok  (lb_line_ok),
        .line_rdy (lb_line_rdy),
        .wp_dbg   (),
        .rp_dbg   (),
        .rdy_dbg  (),
        .cnt_dbg  ()
    );

    //=========================================================
    // 【图像处理级】DMT 像素域：24bit/像素，严格 1 pixel/cycle
    //   位置：行缓存读口之后 —— 用显示自己的节拍处理，零额外时间开销。
    //   正常：24bit YUV444 -> CLAHE -> yuv2rgb -> RGB565 -> RGB888
    //   旁路（+define+BYPASS_CLAHE）：整级直通，out_px == in_px
    //=========================================================
    wire        proc_de, proc_hs, proc_vs;
    wire [23:0] proc_px;
    wire [10:0] proc_hc;
    wire [9:0]  proc_vc;

    //=========================================================
    // 【CLAHE 插值模式】只负责"选模式"，不决定"何时切换"
    //   i_mode = 5'b00001（bit0=拼接缩小，即 4 路拼成一张图）
    //        -> interp_mode = 1：4 个 640x360 区域各自独立插值（每区 4x4 tile）
    //   其余模式（单路 CH0..CH3 / 其他）
    //        -> interp_mode = 0：按整幅 8x8 tile 网格正常双线性插值
    //   ★切换时机：真正的锁存发生在 bil_inter 内部 —— 只在 VS（帧起始、
    //     垂直消隐期内）更新，保证一帧之内模式恒定、画面无撕裂。
    //=========================================================
    wire dmt_interp_mode = (mode_cur == 5'b00001);

    dmt_pix_proc #(
        .GAIN (PROC_GAIN)
    ) u_dmt_proc (
        .clk    (hdmi_pclk),
        .rst_n  (hdmi_rst_n),
        .in_de  (lb_o_de),
        .in_hs  (lb_o_hs),
        .in_vs  (lb_o_vs),
        .in_px  (lb_o_px),
        .in_hc  (lb_o_hc),
        .in_vc  (lb_o_vc),              // 本级内部与 hc 一起打拍，保证 (hc,vc) 配对
        .in_interp_mode (dmt_interp_mode),
        .out_de (proc_de),
        .out_hs (proc_hs),
        .out_vs (proc_vs),
        .out_px (proc_px),
        .out_hc (proc_hc),
        .out_vc (proc_vc)
    );

    // PLL 同时输出 37.125 MHz(pclk) 和 74.25 MHz(clk)，同源同相（严格 2:1）
    reg hdmi_pclk_d;
    always @(posedge clk_74_25M) hdmi_pclk_d <= hdmi_pclk;
    wire pix_en = hdmi_pclk & ~hdmi_pclk_d;            // pclk 的上升沿，天然每 2 拍一次

    stereo_fusion_top u_stereo_fusion_top(
        .clk        (clk_74_25M ),
        .rst_n      (hdmi_rst_n ),
        .pix_en     (pix_en     ),
        .de         (lb_o_de    ),
        .hs         (lb_o_hs    ),
        .vs         (lb_o_vs    ),
        .hc         (lb_o_hc    ),
        .vc         (lb_o_vc    ),
        .yuv_in     (lb_o_px    ),
        .de_out     (de_out     ),
        .rgb565_out (stereo_px  ),
        .rgb888_out (           ),
        .out_valid  (stereo_de  ),
        .out_hc     (stereo_hc  ),
        .out_vc     (stereo_vc  )
    );

    assign o_hdmi_de   = proc_de;
    assign o_hdmi_hs   = proc_hs;
    assign o_hdmi_vs   = proc_vs;
    // dmt_pix_proc 直接输出 24bit（正常 = RGB888；BYPASS_CLAHE = 24bit 直通）
    assign o_hdmi_data = proc_px;
	assign o_hdmi_hc   = proc_hc;
	assign o_hdmi_vc   = proc_vc;

    // 首行备好后才放行 DMT：上电期间监视器看到的是“无信号”而不是黑屏/花屏
    reg dmt_run_r;
    always @(posedge hdmi_pclk or negedge hdmi_rst_n) begin
        if (!hdmi_rst_n)      dmt_run_r <= 1'b0;
        else if (lb_line_rdy) dmt_run_r <= 1'b1;
    end
    assign dmt_run = dmt_run_r;

    // 读出器复位：arst 释放 + 被选通道首块就绪
    reg rd_rst_n_r;
    always @(posedge hdmi_pclk or negedge hdmi_rst_n) begin
        if (!hdmi_rst_n)    rd_rst_n_r <= 1'b0;
        else if (rd_start)  rd_rst_n_r <= 1'b1;
    end
    wire rd_rst_n = rd_rst_n_r;

    // ---- 按显示行读出（五种模式，本域） ----
    wire [7:0]        ch_blk_rd;
    wire [255:0]      bdg_rd_data;
    wire              bdg_rd_vld;
    wire              rdl_req_vld;
    wire [ADDR_W-1:0] rdl_req_addr;
    wire [LEN_W-1:0]  rdl_req_len;
    wire              rdl_req_rdy;
    wire              rdl_rd_en;
    wire [31:0]       rd_skip_cnt, rd_stall_cnt;

    // 读出块号回灌 core 域（帧级稳定，两级同步；偶发采样错只会多一帧延迟，良性）
    always @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
            blkrd_c1 <= 8'd0;
            blkrd_c2 <= 8'd0;
        end else begin
            blkrd_c1 <= ch_blk_rd;
            blkrd_c2 <= blkrd_c1;
        end
    end

    rd_line_gen #(
        .NCH        (4),
        .ADDR_W     (ADDR_W),
        .LEN_W      (LEN_W),
        .BURST_W    (BURST_WORDS),
        .ROW_WORDS  (ROW_WORDS),
        .ROW_BURSTS (ROW_BURSTS),
        .V_ACTIVE   (V_ACTIVE),
        .BLK_ALIGN  (BLK_ALIGN),
        .DW         (256),
        .IFLIGHT    (RD_IFLIGHT)
    ) u_rd_line (
        .pclk         (hdmi_pclk),
        .rst_n        (rd_rst_n),
        .start        (rd_start),
        .line_ok      (lb_line_ok),
        .wr_vld       (lb_wr_vld),
        .wr_word      (lb_wr_word),
        .req_vld      (rdl_req_vld),
        .req_addr     (rdl_req_addr),
        .req_len      (rdl_req_len),
        .req_rdy      (rdl_req_rdy),
        .rd_en        (rdl_rd_en),
        .rd_vld       (bdg_rd_vld),
        .rd_data      (bdg_rd_data),
        .ch_blk_wr    (blkwr_s3),
        .ch_rdy       (chrdy_s3),
        .ch_blk_rd    (ch_blk_rd),
        .mode         (mode_cur),
        .o_mode       (o_mode_frame),
        .skip_cnt     (rd_skip_cnt),
        .stall_cnt    (rd_stall_cnt),
        .rd_line_dbg  (),
        .line_cnt     (),
        .line_end     (lb_line_end)
    );

    // ---- DDR 读桥（pclk <-> core，含请求/数据两个纯 RTL 双时钟 FIFO） ----
    rd_bridge #(
        .ADDR_W  (ADDR_W),
        .LEN_W   (LEN_W),
        .DW      (256),
        .BURST_W (BURST_WORDS),
        .REQ_AW  (2),
        .DAT_AW  (7)
    ) u_rd_bridge (
        .pclk              (hdmi_pclk),
        .pclk_rst_n        (rd_rst_n),
        .req_vld           (rdl_req_vld),
        .req_addr          (rdl_req_addr),
        .req_rdy           (rdl_req_rdy),
        .rd_vld            (bdg_rd_vld),
        .rd_data           (bdg_rd_data),
        .rd_en             (rdl_rd_en),
        .core_clk          (core_clk),
        .core_rst_n        (core_rst_n),
        .axi_rd_req        (axi_rd_req),
        .axi_rd_addr       (axi_rd_addr),
        .axi_rd_len        (axi_rd_len),
        .axi_rd_data_valid (axi_rd_data_valid),
        .axi_rd_data       (axi_rd_data),
        .axi_rd_finish     (axi_rd_finish)
    );

    // 错误标志：4 路视频通道各自的溢出 / 丢块 / 停摆
    wire err_flag = (|ch_ovf) | (|ch_err) | (|ch_stale);
    // 链路就绪：4 路都收到过首帧
    wire link_ok  = (&ch_first);

    status_ctrl #(
        .CLK_FREQ   (125_000_000),
        .DEBOUNCE_N (1_250_000)
    ) u_status (
        .clk       (core_clk),
        .rst_n     (core_rst_n),
        .key       (key),
        .init_done (all_init_done),      // 全链路初始化完成（DDR 初始化 + PLL 锁定）
        .link_ok   (link_ok),
        .err_flag  (err_flag),
        .led       (led)
    );

    //=========================================================
    // [调试] DebugCore 采样总线（64bit）
    //   给 proj_top 里的 DebugCore IP 用；proj_top 未定义 USE_DEBUG_CORE 时
    //   本信号无负载，整段会被综合优化掉。
    //   bit 定义:
    //     [23:0]  proc_px[23:0]     送往 HDMI 的像素数据（24bit）
    //     [47:24] lb_o_px[23:0]     行缓存输出像素 = 处理级输入（24bit）
    //     [63:48] rd_stall_cnt[15:0] 读出停滞计数（DDR 供不上会涨）
    //=========================================================
    assign o_dbg_bus = {
        rd_stall_cnt[15:0],     // [63:48]
        lb_o_px[23:0],          // [47:24]
        proc_px[23:0]           // [23:0]
    };

endmodule
