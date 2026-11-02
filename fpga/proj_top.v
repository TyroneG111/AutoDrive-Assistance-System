module proj_top(
   input                      sys_clk,
   input                      sys_clk_27M,
   input                      rstn_async,
//mode select
   input                      key_in,
//key
   input					  key_3,		//直行
   input					  key_4, 		//左转
   input					  key_5, 		//右转
   input					  key_6,		//强制变道
   input					  key_7,		//停车
   input					  key_8,		//双目融合图像输出开关
//cmos1
   inout                      cmos1_scl,
   inout                      cmos1_sda,
   input                      cmos1_vsync_i,
   input                      cmos1_href,
   input                      cmos1_pclk,
   input             [7:0]    cmos1_data,
   output					  cmos1_reset,
//cmos2
   inout                      cmos2_scl,
   inout                      cmos2_sda,
   input                      cmos2_vsync_i,
   input                      cmos2_href,
   input                      cmos2_pclk,
   input             [7:0]    cmos2_data,
   output					  cmos2_reset,
//hdmi_rx
   output                     hdmi_rx_scl,
   inout                      hdmi_rx_sda,
   input                      hdmi_rx_pclk,
   input                      hdmi_rx_vs_in,
   input                      hdmi_rx_hs_in,
   input                      hdmi_rx_de_in,
   input             [7:0]    hdmi_rx_r_in,
   input             [7:0]    hdmi_rx_g_in,
   input             [7:0]    hdmi_rx_b_in,
//ddr3
   output                     mem_rst_n,
   output                     mem_ck,
   output                     mem_ck_n,
   output                     mem_cke,
   output                     mem_cs_n,
   output                     mem_ras_n,
   output                     mem_cas_n,
   output                     mem_we_n,
   output                     mem_odt,
   output            [14:0]   mem_a,
   output             [2:0]   mem_ba,
   inout             [31:0]   mem_dq,
   inout              [3:0]   mem_dqs,
   inout              [3:0]   mem_dqs_n,
   output             [3:0]   mem_dm,
//hdmi_tx
   output                     hdmi_tx_scl,
   inout                      hdmi_tx_sda,
   output                     hdmi_tx_pclk,
   output                     hdmi_tx_vs_out,
   output                     hdmi_tx_hs_out,
   output                     hdmi_tx_de_out,
   output            [23:0]   hdmi_tx_image_out,
   output 					  hdmi_tx_rstn,
//led
   output                     led_error,		//（溢出丢帧 / 源帧异常 / 接收丢帧）
   output                     led_image_ok,		//图像输入就绪
   output            [4:0]    led_dri,			//驾驶员行为
   output					  led_fusion,		//双目融合模式指示
//ethernet
   output        phy_rstn,       // PHY 复位, 低有效
   //---------- RGMII (接 RTL8211) ----------
   input         rgmii_rxc,
   input         rgmii_rx_ctl,
   input  [3:0]  rgmii_rxd,
   output        rgmii_txc,
   output        rgmii_tx_ctl,
   output [3:0]  rgmii_txd
);

//ethernet parameter
parameter local_mac  = 48'h3C_2B_1A_09_4D_5E;   // 本机 MAC
parameter local_ip   = 32'hC0_A8_00_6E;         // 本机 IP 
parameter local_port = 16'hF0F0;                // 本机端口
parameter dest_mac   = 48'hF6_83_AA_6C_89_85;   // 目的 MAC 
//parameter dest_mac   = 48'hC4_C6_E6_C9_A2_E1;   // 目的 MAC 
parameter dest_ip    = 32'hC0_A8_00_03;         // 目的 IP  
parameter dest_port  = 16'd8080;                // 目的端口
//end

reg		rst_n;
reg		rstn_r;

always@(posedge sys_clk or negedge rstn_async)
	if(!rstn_async)begin
		rst_n	<=	1'b0;
		rstn_r	<=	1'b0;
	end else begin
		rstn_r	<=	1'b1;
		rst_n	<=	rstn_r;
	end

assign led_image_ok = hdmi_rx_init_done && cmos_init_done && hdmi_tx_init_done;
assign led_dri 		= {staus[0], staus[1], staus[2], staus[3], staus[4]};
assign led_fusion 	= stereo_out_en;

//pll inst
wire		clk_25M;
wire		clk_10M;
wire        clk_37_125M;
wire		clk_74_25M;
wire        locked;

pll_1 u_pll_1(
   .clkin1      (sys_clk),
   .clkout0     (clk_25M),
   .clkout1     (clk_10M),
   .pll_lock    (locked)
);

pll_2 u_pll_2(
   .clkin1      (sys_clk_27M),
   .clkout0     (clk_37_125M),
   .clkout1		(clk_74_25M  ),
   .pll_lock    (           )
);

//cam_top inst

wire        cmos_init_done;
wire        cmos1_vsync;
wire        cmos1_de;
wire [23:0] cmos1_image;
wire        cmos2_vsync;
wire        cmos2_de;
wire [23:0] cmos2_image;

//`define cmos_genx2

`ifdef cmos_genx2

cmos_genx2 u_cmos_genx2(
    .clk_74_25M     (clk_74_25M     ),
    .cmos_init_done (cmos_init_done ),
    .cmos1_vsync    (cmos1_vsync    ),
    .cmos1_de       (cmos1_de       ),
    .cmos1_image    (cmos1_image    ),
    .cmos2_vsync    (cmos2_vsync    ),
    .cmos2_de       (cmos2_de       ),
    .cmos2_image    (cmos2_image    )
);

`else

cam_top u_cam_top(
    .sys_clk        (sys_clk        ),
    .clk_25M        (clk_25M        ),
    .cmos_init_done (cmos_init_done ),
    .cmos1_scl      (cmos1_scl      ),
    .cmos1_sda      (cmos1_sda      ),
    .cmos1_vsync_i  (cmos1_vsync_i  ),
    .cmos1_href     (cmos1_href     ),
    .cmos1_pclk     (cmos1_pclk     ),
    .cmos1_data     (cmos1_data     ),
    .cmos1_reset    (cmos1_reset     ),
    .cmos1_vsync    (cmos1_vsync    ),
    .cmos1_de       (cmos1_de       ),
    .cmos1_image    (cmos1_image    ),
    .cmos2_scl      (cmos2_scl      ),
    .cmos2_sda      (cmos2_sda      ),
    .cmos2_vsync_i  (cmos2_vsync_i  ),
    .cmos2_href     (cmos2_href     ),
    .cmos2_pclk     (cmos2_pclk     ),
    .cmos2_data     (cmos2_data     ),
    .cmos2_reset    (cmos2_reset    ),
    .cmos2_vsync    (cmos2_vsync    ),
    .cmos2_de       (cmos2_de       ),
    .cmos2_image    (cmos2_image    )
);

`endif

//hdmi inst
wire        hdmi_rx_init_done;
wire        hdmi_vs0;
wire        hdmi_hs0;
wire        hdmi_de0;
wire [23:0] hdmi_image0;
wire        hdmi_vs1;
wire        hdmi_hs1;
wire        hdmi_de1;
wire [23:0] hdmi_image1;

hdmi_rx u_hdmi_rx(
    .cfg_clk    (clk_10M            ),
    .locked     (locked             ),
    .rst_n      (rst_n              ),
    .rstn_out1  (                   ),
    .iic_rx_scl (hdmi_rx_scl        ),
    .iic_rx_sda (hdmi_rx_sda        ),
    .init_over  (hdmi_rx_init_done  ),
    .pclk       (hdmi_rx_pclk       ),
    .vs_in      (hdmi_rx_vs_in      ),
    .hs_in      (hdmi_rx_hs_in      ),
    .de_in      (hdmi_rx_de_in      ),
    .r_in       (hdmi_rx_r_in       ),
    .g_in       (hdmi_rx_g_in       ),
    .b_in       (hdmi_rx_b_in       ),
    .vs_0       (hdmi_vs0           ),
    .hs_0       (hdmi_hs0           ),
    .de_0       (hdmi_de0           ),
    .image_0    (hdmi_image0        ),
    .vs_1       (hdmi_vs1           ),
    .hs_1       (hdmi_hs1           ),
    .de_1       (hdmi_de1           ),
    .image_1    (hdmi_image1        ),
    .led_int    (                   )
);

//ms7210_ctrl inst

// PLL 锁定后延时产生配置复位

reg  [15:0] rstn_1ms;
wire        rstn_out1;

always @(posedge clk_10M)
    begin
        if(!locked)
            rstn_1ms <= 16'd0;
        else if(rstn_1ms == 16'h4000)
            rstn_1ms <= rstn_1ms;
        else
            rstn_1ms <= rstn_1ms + 1'b1;
    end

    assign rstn_out1 = (rstn_1ms == 16'h4000);

assign hdmi_tx_rstn = rstn_out1;

//================= MS7210 发送像素时钟 =================
//   数据(de/hs/vs/D[23:0])在 clk_37_125M 上升沿寄存输出，此处 PCLK 与数据同相。
//   相位注意: MS72xx 系列约定「时钟下降沿换数据、上升沿采样」。若实测出现
//   「显示器有时钟但锁不上模式 / 背光不亮」，说明芯片在数据跳变瞬间采样，
//   此时应让 PCLK 相对数据寄存时钟反相 180°——推荐在 IPC 里重新生成 pll_2，
//   打开 CLKOUT1 = 74.25MHz / 相位 180°，再把下面改成 pll_2_clkout1。
//   该赋值会让时钟网与端口网合并改名，故 50k_proj.fdc 里对应的生成时钟
//     约束使用 PLL 输出【引脚】而不是网名。
assign hdmi_tx_pclk = clk_37_125M;

wire        hdmi_tx_init_done;

ms7210_ctrl u_ms7210_ctrl(
    .clk       (clk_10M           ),
    .rst_n     (rstn_out1         ),
    .init_over (hdmi_tx_init_done ),
    .iic_scl   (hdmi_tx_scl       ),
    .iic_sda   (hdmi_tx_sda       )
);

//waiting for init done

wire init_done = cmos_init_done & hdmi_rx_init_done & buf_tx_init_done & hdmi_tx_init_done;

//================= ch0/ch1 (CMOS 路) 的采样时钟 =================
//   ch0/ch1 的采样时钟必须与该路数据的产生时钟同源，两种前端不同:
//     · cmos_genx2: 内部图案生成器，数据在 clk_74_25M 上产生
//     · cam_top   : 真实相机，数据与排针输入的 cmos1_pclk/cmos2_pclk 同步
//   该时钟同时用于本模块的视频采样与 video_top.i_vid_pclk。
`ifdef cmos_genx2
    wire ch0_pclk = clk_74_25M;
    wire ch1_pclk = clk_74_25M;
`else
    wire ch0_pclk = cmos1_pclk;
    wire ch1_pclk = cmos2_pclk;
`endif

reg          ch0_vs;
reg          ch0_de;
reg  [23:0]  ch0_image;
reg          ch1_vs;
reg          ch1_de;
reg  [23:0]  ch1_image;
reg          ch2_vs;
reg          ch2_de;
reg  [23:0]  ch2_image;
reg          ch3_vs;
reg          ch3_de;
reg  [23:0]  ch3_image;

always@(posedge ch0_pclk or negedge rst_n) begin
    if(!rst_n) begin
        ch0_vs    <= 1'b0;
        ch0_de    <= 1'b0;
        ch0_image <= 24'd0;
    end
    else if(init_done) begin
        ch0_vs    <= cmos1_vsync;
        ch0_de    <= cmos1_de;
        ch0_image <= cmos1_image;
    end
end

always@(posedge ch1_pclk or negedge rst_n) begin
    if(!rst_n) begin
        ch1_vs    <= 1'b0;
        ch1_de    <= 1'b0;
        ch1_image <= 24'd0;
    end
    else if(init_done) begin
        ch1_vs    <= cmos2_vsync;
        ch1_de    <= cmos2_de;
        ch1_image <= cmos2_image;
    end
end

always@(posedge hdmi_rx_pclk or negedge rst_n) begin
    if(!rst_n) begin
        ch2_vs    <= 1'b0;
        ch2_de    <= 1'b0;
        ch2_image <= 24'd0;
        ch3_vs    <= 1'b0;
        ch3_de    <= 1'b0;
        ch3_image <= 24'd0;
    end
    else if(init_done) begin
        ch2_vs    <= hdmi_vs0;
        ch2_de    <= hdmi_de0;
        ch2_image <= hdmi_image0;
        ch3_vs    <= hdmi_vs1;
        ch3_de    <= hdmi_de1;
        ch3_image <= hdmi_image1;
    end
end

//mode select
wire        key_press;
wire [4:0]  mode;

key_debounce u_key_debounce(
    .clk        (sys_clk    ),
    .rst_n      (rst_n      ),
    .key_in     (key_in     ),
    .key_stable (           ),
    .key_press  (key_press  )
);

mode_ctrl u_mode_ctrl(
    .clk        (sys_clk    ),
    .rst_n      (rst_n      ),
    .key_pulse  (key_press  ),
    .onehot_out (mode       )
);

//key_8消抖，双目输出控制
//   消抖时钟选 sys_clk(50MHz) —— 与 key_debounce 默认 CLK_FREQ 一致，也是按键所处的时钟域
//   key_8 未按下为高电平；上电默认不输出融合图，每按一次翻转一次
wire		key8_press;
reg			stereo_out_en;

key_debounce u_key8_debounce(
    .clk        (sys_clk    ),
    .rst_n      (rst_n      ),
    .key_in     (key_8      ),
    .key_stable (           ),
    .key_press  (key8_press )
);

// 按键脉冲 -> 翻转开关 (sys_clk 域)
always @(posedge sys_clk or negedge rst_n) begin
    if(!rst_n)              stereo_out_en <= 1'b0;
    else if(key8_press)     stereo_out_en <= ~stereo_out_en;
end

//frame buffer + video_top 例化

wire	buf_tx_init_done;
wire	ddr_init_done;
// [调试] 送给 DebugCore 的 64bit 采样总线（PC 侧通过 JTAG 读）
//   内容 = 端到端时延统计（详见 latency_mon.v），具体的位分配见本文件
//   latency_mon 例化之后的 assign dbg_bus。
wire [63:0] dbg_bus;
wire [4:0]	frame_mode;
wire [10:0]	hdmi_tx_hc_out;
wire [9:0]	hdmi_tx_vc_out;
wire [15:0] stereo_px;
wire        stereo_de;  
wire [10:0] stereo_hc;   // 融合画布列号 0..669
wire [9:0]  stereo_vc;   // 融合行号 0..291

//---------- 融合图输出开关：跨时钟域 + 行边界锁存 ----------
//   开关在 sys_clk(50MHz) 域，融合像素在 clk_74_25M 域 -> 两级同步
reg  [1:0]  stereo_en_sync;
reg         stereo_en_line;

always @(posedge clk_74_25M or negedge rst_n) begin
    if(!rst_n) stereo_en_sync <= 2'b00;
    else       stereo_en_sync <= {stereo_en_sync[0], stereo_out_en};
end

// 只在融合行首(stereo_hc==0)采样开关：切换必然落在整行边界，
// 避免关断/打开瞬间留下半行数据被当成有效行发出去
always @(posedge clk_74_25M or negedge rst_n) begin
    if(!rst_n)                                 stereo_en_line <= 1'b0;
    else if(stereo_de && (stereo_hc == 11'd0)) stereo_en_line <= stereo_en_sync[1];
end

// 融合像素有效 & 开关打开 -> 才送给以太网打包（关闭时 fuse_de 恒低，不产生融合包）
wire        fuse_de_g = stereo_de & stereo_en_line;

video_top u_video_top(
    .sys_clk         (sys_clk         ),
    .clk_74_25M      (clk_74_25M      ),
    .rst_n           (rst_n           ),
    .mem_rst_n       (mem_rst_n       ),
    .mem_ck          (mem_ck          ),
    .mem_ck_n        (mem_ck_n        ),
    .mem_cke         (mem_cke         ),
    .mem_cs_n        (mem_cs_n        ),
    .mem_ras_n       (mem_ras_n       ),
    .mem_cas_n       (mem_cas_n       ),
    .mem_we_n        (mem_we_n        ),
    .mem_odt         (mem_odt         ),
    .mem_a           (mem_a           ),
    .mem_ba          (mem_ba          ),
    .mem_dq          (mem_dq          ),
    .mem_dqs         (mem_dqs         ),
    .mem_dqs_n       (mem_dqs_n       ),
    .mem_dm          (mem_dm          ),
    .ddr_init_done   (ddr_init_done   ),
    .i_vid_pclk      ({ch0_pclk, ch1_pclk, hdmi_rx_pclk, hdmi_rx_pclk}),
    .i_vid_vs        ({ch0_vs, ch1_vs, ch2_vs, ch3_vs}),
    .i_vid_de        ({ch0_de, ch1_de, ch2_de, ch3_de}),
    .i_vid_data      ({ch0_image, ch1_image, ch2_image, ch3_image}),
    .i_mode          (mode             ),
    .key             (2'b01            ),
    .led             (led_error        ),
	.all_init_done	 (buf_tx_init_done ),
    .i_hdmi_pclk     (clk_37_125M       ),
    .o_hdmi_de       (hdmi_tx_de_out   ),
    .o_hdmi_hs       (hdmi_tx_hs_out   ),
    .o_hdmi_vs       (hdmi_tx_vs_out   ),
	.o_hdmi_hc		 (hdmi_tx_hc_out   ),
	.o_hdmi_vc		 (hdmi_tx_vc_out   ),
    .o_hdmi_data     (hdmi_tx_image_out),
    .stereo_px       (stereo_px        ),
    .stereo_de       (stereo_de        ),
    .stereo_vc       (stereo_vc        ),
    .stereo_hc       (stereo_hc        ),
	.o_mode_frame	 (frame_mode	   ),
    .o_dbg_bus       (                 )   // 旧调试总线(像素/停摆计数)，已被时延统计取代
);

//================= 端到端时延统计 =================
//   四路拼接模式：cam1(CH0) 场同步 -> UDP 发完含该帧的拼接图
//   单路模式    ：被选中那一路的场同步 -> UDP 发完该帧
//   时基 = clk_37_125M，1 tick ≈ 26.936ns；ns = tick*26.936
wire        eth_frame_done_tgl;
wire [31:0] lat_tick;
wire        lat_valid;
wire [3:0]  lat_inflight;
wire [15:0] lat_frame_cnt;
wire        lat_pair_ovf;

latency_mon u_latency_mon (
    .clk            (clk_37_125M        ),
    .rst_n          (rst_n              ),
    .ch_vs          ({ch3_vs, ch2_vs, ch1_vs, ch0_vs}),
    .mode           (frame_mode         ),
    .udp_frame_tgl  (eth_frame_done_tgl ),
    .lat_tick       (lat_tick           ),
    .lat_valid      (lat_valid          ),
    .inflight       (lat_inflight       ),
    .frame_cnt      (lat_frame_cnt      ),
    .pair_ovf       (lat_pair_ovf       )
);

//---- 64bit 调试总线组装（PC 侧 JTAG 读）----
//   [31:0]  lat_tick     端到端时延，单位 = clk_37_125M 周期 ≈ 26.936ns
//                        ns = tick*26.936 / us = tick/37.125 / ms = tick/37125
//   [36:32] mode_frame   当前输出帧模式（独热；bit0=四路拼接 bit1..4=单路CH0..CH3）
//   [40:37] inflight     在途帧数（正常恒定 1~2；持续增大说明输入帧快于输出帧）
//   [56:41] frame_cnt    已完成的 UDP 帧计数（低 16bit）
//   [57]    lat_valid    至少完成过一次有效测量（先看这一位）
//   [58]    pair_ovf     帧配对 FIFO 溢出（测量不可信）
//   [63:59] 保留 0
assign dbg_bus = {
    5'd0,               // [63:59]
    lat_pair_ovf,       // [58]
    lat_valid,          // [57]
    lat_frame_cnt,      // [56:41]
    lat_inflight,       // [40:37]
    frame_mode,         // [36:32]
    lat_tick            // [31:0]
};

//================= [调试] DebugCore 波形采集 =================
//   启用方法（三步）:
//     1) PDS 的 IPC 里生成 System/Tools/Debug/Jtag_Hub 和 System/Tools/Debug/DebugCore
//        两个 IP（参数见文档），把生成的文件加入工程；
//     2) 把两份 IP 在 IPC 里的实例名分别命名为 dbg_hub / dbg_core
//        （生成文件是 <实例名>.v，模块名跟实例名一致）；
//     3) 定义宏 USE_DEBUG_CORE（在这里取消注释，或在工程 Verilog 宏里加）。
//   注意: 未生成 IP 之前千万不要打开这个宏，否则找不到模块会报错。
//   若生成的端口名与下面不同（尤其是采样时钟端口），把 <实例名>.v 的 module
//     端口列表改一下即可，下面这一整段都在 ifdef 里，不影响正常构建。
`define USE_DEBUG_CORE

`ifdef USE_DEBUG_CORE
    // ---- Jtag_Hub <-> DebugCore 互连 ----
    //   Jtag_Hub.conf_sel[14:0] / hub_tdo[14:0] 是 15 根“每核一根”的总线，
    //   我们只用第 0 根接第一个 DebugCore，其余接 0。
    wire        dbg_hub_tdi;
    wire        dbg_capt;
    wire        dbg_shift;
    wire        dbg_drck;
    wire [4:0]  dbg_id;
    wire [14:0] dbg_conf_sel_bus;
    wire [14:0] dbg_hub_tdo_bus;

    assign dbg_hub_tdo_bus[14:1] = 14'd0;

    dbg_hub u_dbg_hub (
        .resetn_i     (rst_n            ),
        .drck_o       (dbg_drck         ),   // -> DebugCore.drck_in (JTAG clk)
        .hub_tdi      (dbg_hub_tdi      ),
        .capt_o       (dbg_capt         ),
        .shift_o      (dbg_shift        ),
        .conf_sel     (dbg_conf_sel_bus ),   // -> DebugCore.conf_sel
        .id_o         (dbg_id           ),   // -> DebugCore.id_i
        .hub_tdo      (dbg_hub_tdo_bus  )    // <- DebugCore.hub_tdo
    );

    dbg_core u_dbg_core (
        .clk          (clk_37_125M            ),  // 采样时钟 = DMT 像素域 74.25MHz
        .resetn_i     (rst_n                 ),
        .hub_tdi      (dbg_hub_tdi           ),
        .hub_tdo      (dbg_hub_tdo_bus[0]    ),
        .id_i         (dbg_id                ),
        .capt_i       (dbg_capt              ),
        .shift_i      (dbg_shift             ),
        .conf_sel     (dbg_conf_sel_bus[0]   ),
        .drck_in      (dbg_drck              ),
        .data_i       (dbg_bus               ),  // 64bit: 端到端时延 + 输出模式
        .trig0_i      (dbg_bus[57]           )   // 1bit 触发 = lat_valid（已测到有效时延）
    );
`endif  // USE_DEBUG_CORE

//dri_staus

wire [4:0]  staus;
wire        rx_data_valid;
wire [7:0]  rx_data;
wire [15:0] rx_data_length;      

dri_staus u_dri_staus(
    .clk   (rgmii_rxc),
    .rst_n (rst_n ),
    .key_1 (key_3 ),
    .key_2 (key_4 ),
    .key_3 (key_5 ),
    .key_4 (key_6 ),
    .key_5 (key_7 ),
    .staus (staus )
);

ethernet_ctrl_top#(
	.LOCAL_MAC		(local_mac		),
	.LOCAL_IP		(local_ip		),
	.LOCAL_PORT		(local_port		),
	.DEST_MAC		(dest_mac		),
	.DEST_IP		(dest_ip		),
	.DEST_PORT		(dest_port		)
) u_ethernet_ctrl_top(
    .rst_n          (rst_n          ),
    .phy_rstn       (phy_rstn       ),
    .rgmii_rxc      (rgmii_rxc      ),
    .rgmii_rx_ctl   (rgmii_rx_ctl   ),
    .rgmii_rxd      (rgmii_rxd      ),
    .rgmii_txc      (rgmii_txc      ),
    .rgmii_tx_ctl   (rgmii_tx_ctl   ),
    .rgmii_txd      (rgmii_txd      ),
    .vid_pclk       (clk_37_125M	),
    .vid_hs         (hdmi_tx_hs_out ),
    .vid_vs         (hdmi_tx_vs_out ),
    .vid_de         (hdmi_tx_de_out ),
    .vid_hc         (hdmi_tx_hc_out ),
    .vid_vc         (hdmi_tx_vc_out ),
    .vid_rgb        (hdmi_tx_image_out),
    //---------- 双目融合图像 (video_top.stereo_*, clk_74_25M 域) ----------
    .fuse_clk       (clk_74_25M     ),
    .fuse_px        (stereo_px      ),
    .fuse_de        (fuse_de_g      ),
    .fuse_hc        (stereo_hc      ),
    .fuse_vc        (stereo_vc      ),
    .user_data_in   ({6'b000_000, frame_mode, staus}),
    .rx_data_valid  (rx_data_valid  ),
    .rx_data        (rx_data        ),
    .rx_data_length (rx_data_length ),
    .frame_done_tgl (eth_frame_done_tgl)
);

endmodule