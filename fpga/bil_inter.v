//==============================================================================
//  bil_inter.v
//  CLAHE 双线性插值输出模块（1280x720 / 8x8 tile，tile = 160x90）
//------------------------------------------------------------------------------
//  功能概述
//    1) 由当前像素的 tile 索引算出四个角相邻 tile 的索引
//    2) 向 cdf_lut 的 4 个映射表读口请求映射后的灰度值
//    3) 双线性插值得到增强后的 Y 分量
//    4) 边缘 tile 退化为一位线性插值，角落 tile 直接取本 tile 映射值
//    5) u/v 分量与 vs/de 同步打拍，与 Y 的处理延迟对齐
//
//  ── 两种插值模式（由 interp_mode 选择）────────────────────────────
//    interp_mode = 0（默认，与原有工程一致）：
//        以整幅 1280x720 的 8x8=64 个 tile 为一张映射网格做双线性插值，
//        最右列(tile_x_index==7) / 最下行(tile_y_index==7) 为边界。
//    interp_mode = 1：
//        把整幅划分成 4 个 640x360 区域（左/右 × 上/下），每区 4x4=16 个
//        tile，各自独立做双线性插值 —— 等价于对每个区域单独跑一遍 16 tile
//        的 CLAHE 插值：区域内最右/最下退化为一维线性，区域角块取自身映射值。
//        tile 尺寸不变(160x90)，权重完全复用，仅“边界”改为区域内
//        （tile_x_index[1:0]==3 / tile_y_index[1:0]==3）。
//    ★ 实际参与运算的是锁存后的 interp_mode_r（见第 1.5 节）：
//      模式只在 VS 消隐期内切换，保证一帧之内模式恒定、画面不撕裂。
//
//  ── 边缘 / 角落的处理方式（沿用原设计技巧）────────────────────────────
//    wx_eff = 最右列 ? 0 : tile 内 x 权重
//    wy_eff = 最下行 ? 0 : tile 内 y 权重
//    代入标准双线性公式后自动退化为一位/直接取值。
//
//  ── 权重的定点化（分母统一取 256，全部移位加法，无除法）──────────────
//    水平：tile 宽 160，精确权重 = tile_x_pos*256/160 = tile_x_pos*1.6
//          wq_x = (tile_x_pos * 205) >> 7      （205/128 = 1.6016，误差 < 0.1%）
//          205 = 128+64+8+4+1
//          wx_cur = 256 - wq_x （恒互补，不引入增益误差）
//          水平插值 = (wx_cur*P00 + wq_x*P01) >> 8
//    垂直：tile 高 90，精确权重 = tile_y_pos*256/90 = tile_y_pos*2.8444
//          wq_y = (tile_y_pos * 728) >> 8      （728/256 = 2.84375，误差 < 0.05%）
//          728 = 512+128+64+16+8
//          wy_cur = 256 - wq_y
//          垂直插值 = (wy_cur*h0 + wq_y*h1) >> 8
//
//  ── 流水线（总延迟 3 拍）─────────────────────────────────────────────
//    第 0 级：算 tile 索引 / 有效位 / 权重，驱动 4 个映射表读口
//    第 1 级：lut_data 到达（比地址滞后 1 拍）→ 水平插值
//    第 2 级：垂直插值
//    第 3 级：寄存输出；u/v/vs/de 经 2 级延迟后与 Y 对齐
//
//  ── 接口假设 ──────────────────────────────────────────────────────────
//    (1) i_image 按 {Y[7:0], U[7:0], V[7:0]} 打包
//    (2) 四个读口的读地址必须相同（同一个像素的原始灰度值）
//==============================================================================

module bil_inter(
	input					pclk,
	input					rst_n,

//输入视频流
	input					i_vs,
	input					i_de,
	input			[23:0]	i_image,

//像素位置信息
	input			[10:0]	x_pos,			//本模块未使用，保留以匹配上层接口
	input			[9:0]	y_pos,			//本模块未使用，保留以匹配上层接口
	input			[2:0]	tile_x_index,	//横向 tile 序号 0~7
	input			[2:0]	tile_y_index,	//纵向 tile 序号 0~7
	input			[7:0]	tile_x_pos,		//tile 内横坐标 0~159
	input			[6:0]	tile_y_pos,		//tile 内纵坐标 0~89
	input			[5:0]	tile_index,		//当前像素的 tile 索引 0~63
	input					interp_mode,	//0=整幅 8x8 双线性(默认)；1=4 区(640x360)各 4x4

//向 cdf_lut 请求映射后的灰度值（4 路并行）
	output	wire	[5:0]	lut_tile_index_1,	//组合输出（DRM 输入侧寄存，总延迟仍 1 拍）
	output	wire	[7:0]	lut_addr_1,
	output	reg			lut_index_val_1,	//保持寄存（有效标志，见第 4 节）
	input			[7:0]	lut_data_1,
	output	wire	[5:0]	lut_tile_index_2,
	output	wire	[7:0]	lut_addr_2,
	output	reg			lut_index_val_2,
	input			[7:0]	lut_data_2,
	output	wire	[5:0]	lut_tile_index_3,
	output	wire	[7:0]	lut_addr_3,
	output	reg			lut_index_val_3,
	input			[7:0]	lut_data_3,
	output	wire	[5:0]	lut_tile_index_4,
	output	wire	[7:0]	lut_addr_4,
	output	reg				lut_index_val_4,
	input			[7:0]	lut_data_4,

//输出视频流
	output	reg		[23:0]	o_image,
	output	reg				o_vs,
	output	reg				o_de,

	input					i_hs,
	input			 [10:0]	i_hc,
	input			 [9:0]	i_vc,
	output	reg		 		o_hs,
	output	reg		 [10:0]	o_hc,
	output	reg		 [9:0]	o_vc
   );

//==============================================================================
// 1. 输入图像的分量拆分（按 {Y,U,V} 打包）
//==============================================================================
	wire	[7:0]	I_Y = i_image[23:16];
	wire	[7:0]	I_U = i_image[15:8];
	wire	[7:0]	I_V = i_image[7:0];

//==============================================================================
// 1.5 插值模式锁存 —— 模式的切换只发生在"插值算法不工作"的时候
//------------------------------------------------------------------------------
//  interp_mode 决定的是"边界"语义：它同时影响 4 个映射表读口的地址生成
//  （idx01 / idx10 / idx11 的钳位）以及权重的退化方式。
//  若在行内或帧内任意时刻直接使用它，同一帧里前后像素就会落在不同的 tile
//  边界上 -> 画面撕裂 + 区域交界处插值错乱。
//
//  因此本模块内部【一律不使用输入 interp_mode】，只用 interp_mode_r。
//  interp_mode_r 仅在 VS 有效期间更新：
//    · DMT 的 VS（高有效，仅 v=0..4）完全落在垂直消隐区内，此时插值算法
//      没有任何活动像素在计算，正是"算法不工作"的窗口；
//    · 3 级流水在这段消隐里早已排空，模式切换不会影响在途像素；
//    · 结果：一帧之内模式恒定，切换永远发生在帧边界。
//  与 rd_line_gen 在每帧起点锁存读出模式的做法保持一致。
//==============================================================================
	reg		interp_mode_r;

	always @(posedge pclk or negedge rst_n) begin
		if (!rst_n)      interp_mode_r <= 1'b0;			//复位后默认：整幅插值
		else if (i_vs)   interp_mode_r <= interp_mode;	//只在 VS 消隐期内切换
	end

//==============================================================================
// 2. 四个角相邻 tile 的索引与有效位
//------------------------------------------------------------------------------
//    P00 = 本 tile ；P01 = 本 tile+1 ；P10 = 本 tile+8 ；P11 = 本 tile+9
//    ? 用 7 位中间量：tile 63 时 "+9" 超出 6 位（虽然该路 index_val=0）
//
//    ---- 边界随 interp_mode 切换 ----
//    interp_mode=0：整幅 8x8 网格 → 边界为 tile_x_index==7 / tile_y_index==7
//    interp_mode=1：4 个 640x360 区域，每区 4x4 → 边界为区域内最右/最下
//                   （tile_x_index[1:0]==3 / tile_y_index[1:0]==3）
//    其余（邻块索引、权重、边缘退化处理）完全复用，无需改动。
//==============================================================================
	wire			last_x_full = (tile_x_index == 3'd7);	//整幅最右列
	wire			last_y_full = (tile_y_index == 3'd7);	//整幅最下行
	wire			last_x_rgn  = (tile_x_index[1:0] == 2'd3);//区域内最右列（每 4 列一区）
	wire			last_y_rgn  = (tile_y_index[1:0] == 2'd3);//区域内最下行（每 4 行一区）

	wire			last_x = interp_mode_r ? last_x_rgn : last_x_full;	//最右列（随模式切换）
	wire			last_y = interp_mode_r ? last_y_rgn : last_y_full;	//最下行（随模式切换）

	wire	[6:0]	t7 = {1'b0, tile_index};

	wire	[6:0]	idx00_7 = t7;
	wire	[6:0]	idx01_7 = last_x            ? t7 : (t7 + 7'd1);
	wire	[6:0]	idx10_7 = last_y            ? t7 : (t7 + 7'd8);
	wire	[6:0]	idx11_7 = (last_x | last_y) ? t7 : (t7 + 7'd9);

	wire	[5:0]	idx00 = idx00_7[5:0];
	wire	[5:0]	idx01 = idx01_7[5:0];
	wire	[5:0]	idx10 = idx10_7[5:0];
	wire	[5:0]	idx11 = idx11_7[5:0];

//==============================================================================
// 3. 插值权重（分母 256，全部移位加法）
//==============================================================================
	//---- 水平权重：wq_x = (tile_x_pos*205)>>7 ----
	wire	[8:0]	xp9    = {1'b0, tile_x_pos};
	wire	[15:0]	xp_mul = (xp9 << 7) + (xp9 << 6) + (xp9 << 3) + (xp9 << 2) + xp9;
	wire	[8:0]	wq_x   = xp_mul[15:7];
	wire	[8:0]	wx     = last_x ? 9'd0 : wq_x;	//最右列权重强制 0

	//---- 垂直权重：wq_y = (tile_y_pos*728)>>8 ----
	wire	[6:0]	ly_eff = last_y ? 7'd0 : tile_y_pos;
	wire	[16:0]	ly_e   = {10'd0, ly_eff};
	wire	[16:0]	ly_mul = (ly_e << 9) + (ly_e << 7) + (ly_e << 6) + (ly_e << 4) + (ly_e << 3);
	wire	[8:0]	wy_nxt = ly_mul[16:8];

//==============================================================================
// 4. 第 0 级：驱动 cdf_lut 的 4 个映射表读口
//------------------------------------------------------------------------------
//  ★ 存储改造后（cdf_lut 改用 DRM，同步读、自带 1 拍读延迟）：
//      - 地址（lut_tile_index_* / lut_addr_*）改为【组合输出】，
//        这一级寄存器被挪到 DRM 的输入侧；从像素到 lut_data 仍是 1 拍；
//      - lut_index_val_* 仍【保持寄存】（它只是有效标志，不必与地址同拍），
//        cdf_lut 内部直接用它门控输出，不再打拍。
//    这样本模块第 1 级起的 _s1 / _s2 流水线一行都不用动，
//    1 pclk/像素 与 2 pclk/像素 两种时序都成立。
//==============================================================================
	assign lut_tile_index_1 = idx00;
	assign lut_addr_1       = I_Y;
	assign lut_tile_index_2 = idx01;
	assign lut_addr_2       = I_Y;
	assign lut_tile_index_3 = idx10;
	assign lut_addr_3       = I_Y;
	assign lut_tile_index_4 = idx11;
	assign lut_addr_4       = I_Y;

	always @(posedge pclk or negedge rst_n) begin
		if (!rst_n) begin
			lut_index_val_1 <= 1'b0;
			lut_index_val_2 <= 1'b0;
			lut_index_val_3 <= 1'b0;
			lut_index_val_4 <= 1'b0;
		end
		else begin
			lut_index_val_1 <= i_de;							//本 tile 恒有效
			lut_index_val_2 <= i_de & ~last_x;					//最右列：右邻不存在
			lut_index_val_3 <= i_de & ~last_y;					//最下行：下邻不存在
			lut_index_val_4 <= i_de & ~last_x & ~last_y;		//右下角：右下邻不存在
		end
	end

//==============================================================================
// 5. 第 0 级 → 第 1 级打拍
//==============================================================================
	reg		[8:0]	wx_s1;
	reg		[8:0]	wy_nxt_s1;
	reg		[7:0]	u_s1, v_s1;
	reg				vs_s1, de_s1;
	reg				hs_s1;
	reg		[10:0]	hc_s1;
	reg		[9:0]	vc_s1;

	always @(posedge pclk or negedge rst_n) begin
		if (!rst_n) begin
			wx_s1		<= 9'd0;
			wy_nxt_s1	<= 9'd0;
			u_s1		<= 8'd0;
			v_s1		<= 8'd0;
			vs_s1		<= 1'b0;
			de_s1		<= 1'b0;
			hs_s1		<= 1'b0;
			hc_s1		<= 11'd0;
			vc_s1		<= 10'd0;
		end
		else begin
			wx_s1		<= wx;
			wy_nxt_s1	<= wy_nxt;
			u_s1		<= I_U;
			v_s1		<= I_V;
			vs_s1		<= i_vs;
			de_s1		<= i_de;
			hs_s1		<= i_hs;
			hc_s1		<= i_hc;
			vc_s1		<= i_vc;
		end
	end

//==============================================================================
// 6. 第 1 级：水平插值（分母 256）
//        h0 = ((256-wx)*P00 + wx*P01) >> 8   （本行）
//        h1 = ((256-wx)*P10 + wx*P11) >> 8   （下行）
//==============================================================================
	wire	[8:0]	wx_cur = 9'd256 - wx_s1;

	wire	[16:0]	h0_sum = (wx_cur * lut_data_1) + (wx_s1 * lut_data_2);
	wire	[16:0]	h1_sum = (wx_cur * lut_data_3) + (wx_s1 * lut_data_4);

	wire	[7:0]	h0 = h0_sum[15:8];					//>>8
	wire	[7:0]	h1 = h1_sum[15:8];

	reg		[7:0]	h0_s2, h1_s2;
	reg		[8:0]	wy_nxt_s2;
	reg		[7:0]	u_s2, v_s2;
	reg				vs_s2, de_s2;
	reg				hs_s2;
	reg		[10:0]	hc_s2;
	reg		[9:0]	vc_s2;

	always @(posedge pclk or negedge rst_n) begin
		if (!rst_n) begin
			h0_s2		<= 8'd0;
			h1_s2		<= 8'd0;
			wy_nxt_s2	<= 9'd0;
			u_s2		<= 8'd0;
			v_s2		<= 8'd0;
			vs_s2		<= 1'b0;
			de_s2		<= 1'b0;
			hs_s2		<= 1'b0;
			hc_s2		<= 11'd0;
			vc_s2		<= 10'd0;
		end
		else begin
			h0_s2		<= h0;
			h1_s2		<= h1;
			wy_nxt_s2	<= wy_nxt_s1;
			u_s2		<= u_s1;
			v_s2		<= v_s1;
			vs_s2		<= vs_s1;
			de_s2		<= de_s1;
			hs_s2		<= hs_s1;
			hc_s2		<= hc_s1;
			vc_s2		<= vc_s1;
		end
	end

//==============================================================================
// 7. 第 2 级：垂直插值（分母 256）
//        y = ((256-wy)*h0 + wy*h1) >> 8
//==============================================================================
	wire	[8:0]	wy_cur = 9'd256 - wy_nxt_s2;

	wire	[16:0]	v_sum = (wy_cur * h0_s2) + (wy_nxt_s2 * h1_s2);
	wire	[7:0]	o_y   = v_sum[15:8];				//>>8

//==============================================================================
// 8. 第 3 级：输出寄存（整体延迟 = 3 拍）
//==============================================================================
	always @(posedge pclk or negedge rst_n) begin
		if (!rst_n) begin
			o_image	<= 24'd0;
			o_vs	<= 1'b0;
			o_de	<= 1'b0;
			o_hs	<= 1'b0;
			o_hc	<= 11'd0;
			o_vc	<= 10'd0;
		end
		else begin
			o_image	<= {o_y, u_s2, v_s2};			//只替换 Y，U/V 原样透传
			o_vs	<= vs_s2;
			o_de	<= de_s2;
			o_hs	<= hs_s2;
			o_hc	<= hc_s2;
			o_vc	<= vc_s2;
		end
	end

endmodule
