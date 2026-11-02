//==============================================================================
//  hist_ram_ctrl.v  —— CLAHE 直方图存储与处理控制器（1280x720 / 8x8 tile 版）
//------------------------------------------------------------------------------
//  图像参数
//    1280 x 720，8 列 x 8 行 = 64 个 tile，每个 tile 160 x 90 = 14400 像素
//    tile 编号：tile_index = tile_y_index*8 + tile_x_index （0 ~ 63）
//
//  存储架构（BRAM 复用，本次关键改动）
//    * 只例化 8 个直方图 RAM，第 c 个 RAM 负责“第 c 列”的 8 个 tile。
//      RAM 地址 = {tile_y_index[2:0], bin[7:0]}，高 3 位区分是哪一行的 tile，
//      低 8 位是灰度 bin。深度 2048，宽度 16bit（单 tile 最多 14400 计数）。
//      => 每块 2048*16 = 32Kbit < 36Kbit，8 块各占一块 BRAM，无浪费。
//    * LUT 映射表（64 块 256x8bit 分布式 RAM）在 cdf_lut.v 中例化。
//
//  并行策略（本次关键改动）
//    * 截断 / CDF 阶段：8 个引擎并行，引擎 c 固定对应“第 c 列”的 RAM。
//      一轮同时处理“一行 8 个 tile”（一列一块 RAM，8 个引擎各访问不同 RAM，
//      天然无冲突），共 8 轮（8 行）。
//    * 引擎与 RAM 一一对应，CDF 的 8 个读口与 8 块 RAM 直连，无需交叉开关。
//
//  各阶段
//    S_CLEAR : 8 块 RAM 并行清零 2048 个地址（一遍，共 2048 拍）
//    S_ACCUM : 逐像素把 i_y 累加到 RAM[tile_x_index][{tile_y_index,i_y}]
//    S_CLIP  : 8 引擎并行“截断 + 重分配”，每引擎负责 8 个 tile（列 c 的 8 行）
//    S_DONE  : 置 redistri_done，等 CDF 算完并写映射表后回送 cdf_done
//
//  截断阈值
//    阈值 = 14400 * cliplimit / 256 = 14400 * (cliplimitx8/8) / 256
//         = cliplimitx8 * 225 / 32 = (cliplimitx8*225) >> 5
//    225 = 128+64+32+1，纯移位加法，无除法、无乘法器。
//    cliplimitx8 = 16（cliplimit = 2.0）时阈值 = 112。
//
//  RAM 读时序：rd_data 比 rd_addr 滞后 1 拍。凡“按读回数据判断归属”的逻辑
//              都必须使用延迟一拍的控制信号，代码中用 ? 标注。
//==============================================================================

module hist_ram_ctrl(
	input					pclk,
	input					rst_n,
	input			[4:0]	cliplimitx8,
//直方图统计阶段信号
	input			[7:0]	i_y,
	input					i_vs,			//场同步，场消隐期间为高
	input					i_de,
	input			[5:0]	i_tile_index,	//tile序号 range:0~63
	input			[10:0]	i_x_pos,		//像素横坐标 range:0~1279
	input			[9:0]	i_y_pos,		//像素行坐标   range:0~719
	output	reg				redistri_done,	//重分配完成标志，高有效
	input					cdf_done,		//cdf完成标志，高有效
//cdf计算信号（8 路，第 k 路对应第 k-1 列）
	input			[5:0]	rd_tile_index_1,
	input			[7:0]	rd_addr_1,
	output	reg		[15:0]	rd_data_1,
	input			[5:0]	rd_tile_index_2,
	input			[7:0]	rd_addr_2,
	output	reg		[15:0]	rd_data_2,
	input			[5:0]	rd_tile_index_3,
	input			[7:0]	rd_addr_3,
	output	reg		[15:0]	rd_data_3,
	input			[5:0]	rd_tile_index_4,
	input			[7:0]	rd_addr_4,
	output	reg		[15:0]	rd_data_4,
	input			[5:0]	rd_tile_index_5,
	input			[7:0]	rd_addr_5,
	output	reg		[15:0]	rd_data_5,
	input			[5:0]	rd_tile_index_6,
	input			[7:0]	rd_addr_6,
	output	reg		[15:0]	rd_data_6,
	input			[5:0]	rd_tile_index_7,
	input			[7:0]	rd_addr_7,
	output	reg		[15:0]	rd_data_7,
	input			[5:0]	rd_tile_index_8,
	input			[7:0]	rd_addr_8,
	output	reg		[15:0]	rd_data_8
   );

//==============================================================================
// 1. 常量定义
//==============================================================================
	localparam	TILE_NUM	= 64;			//tile 总数 8x8
	localparam	COL_NUM		= 8;			//列数 = RAM 数 = 引擎数
	localparam	ROW_NUM		= 8;			//行数
	localparam	TILE_PIX	= 14400;		//每个 tile 的像素数 160*90
	localparam	BIN_NUM		= 256;			//直方图 bin 数
	localparam	RAM_DEPTH	= 2048;			//每块 RAM 深度 = ROW_NUM * BIN_NUM
	localparam	ENG_NUM		= 8;			//并行处理引擎数（= 列数）
	localparam	CDF_PORT_N	= 8;			//CDF 模块逻辑读口数（= 引擎数）
	localparam	TILE_PER_ENG= 8;			//每个引擎负责的 tile 数 = 64/8

	//帧末像素坐标，必须与实际输入分辨率一致
	localparam	[10:0]	LAST_X	= 11'd1279;
	localparam	[9:0]	LAST_Y	= 10'd719;

	//主状态机
	localparam	[2:0]	S_CLEAR	= 3'd0;
	localparam	[2:0]	S_ACCUM	= 3'd1;
	localparam	[2:0]	S_CLIP	= 3'd2;
	localparam	[2:0]	S_DONE	= 3'd3;

	//引擎状态机
	localparam	[1:0]	E_IDLE	= 2'd0;
	localparam	[1:0]	E_SCAN	= 2'd1;		//第 1 遍：统计溢出总量
	localparam	[1:0]	E_REDIST= 2'd2;		//第 2 遍：削峰 + 均摊回写

//==============================================================================
// 2. 主状态机与帧边界控制
//==============================================================================
	reg		[2:0]	state;

	//--- 清零计数器（8 块 RAM 并行，一遍清完 2048 个地址）------------------
	reg		[10:0]	clr_addr;

	//--- 帧末像素检测 → 触发截断 -------------------------------------------
	wire			last_pix = i_de & (i_x_pos == LAST_X) & (i_y_pos == LAST_Y);

	reg				last_pix_d1, last_pix_d2;
	wire			clip_start = last_pix_d2;

	always @(posedge pclk or negedge rst_n) begin
		if (!rst_n) begin
			last_pix_d1 <= 1'b0;
			last_pix_d2 <= 1'b0;
		end
		else begin
			last_pix_d1 <= last_pix & (state == S_ACCUM);
			last_pix_d2 <= last_pix_d1;
		end
	end

	//--- 截断阈值寄存器 -----------------------------------------------------
	//阈值 = cliplimitx8 * 225 / 32 ；225 = 128+64+32+1（移位加法，无乘法器）
	reg		[15:0]	clip_limit_r;
	wire	[15:0]	cl_c       = {11'd0, cliplimitx8};
	wire	[15:0]	clip_lim_n = ((cl_c << 7) + (cl_c << 6) + (cl_c << 5) + cl_c) >> 5;

	always @(posedge pclk or negedge rst_n) begin
		if (!rst_n) clip_limit_r <= 16'd112;			//cliplimitx8=16 -> 2.0
		else        clip_limit_r <= clip_lim_n;
	end

	//--- 引擎空闲汇总 -------------------------------------------------------
	wire	[ENG_NUM-1:0]	eng_idle;
	wire					all_eng_idle = &eng_idle;

	//--- 清零触发条件 -------------------------------------------------------
	wire			clear_now = cdf_done & i_vs;

	//--- 主状态机 -----------------------------------------------------------
	always @(posedge pclk or negedge rst_n) begin
		if (!rst_n) begin
			state			<= S_CLEAR;
			clr_addr		<= 11'd0;
			redistri_done	<= 1'b0;
		end
		else begin
			case (state)
			//--------------------------------------------------------------
			// 清零：8 块 RAM 并行清 2048 个地址
			//--------------------------------------------------------------
			S_CLEAR: begin
				if (clr_addr == RAM_DEPTH - 1) begin
					clr_addr <= 11'd0;
					state    <= S_ACCUM;
				end
				else begin
					clr_addr <= clr_addr + 11'd1;
				end
			end
			//--------------------------------------------------------------
			// 统计：等帧末像素写回后立刻进入截断
			//--------------------------------------------------------------
			S_ACCUM: begin
				if (clip_start) state <= S_CLIP;
			end
			//--------------------------------------------------------------
			// 截断重分配：8 个引擎全部回到 IDLE 才算完成
			//--------------------------------------------------------------
			S_CLIP: begin
				if (all_eng_idle) begin
					state			<= S_DONE;
					redistri_done	<= 1'b1;
				end
			end
			//--------------------------------------------------------------
			// 完成：保持 redistri_done 高，等 cdf_done + 场消隐 → 清零
			//--------------------------------------------------------------
			S_DONE: begin
				if (clear_now) begin
					state			<= S_CLEAR;
					clr_addr		<= 11'd0;
					redistri_done	<= 1'b0;
				end
			end
			//--------------------------------------------------------------
			default: state <= S_CLEAR;
			endcase
		end
	end

//==============================================================================
// 3. RAM 接口信号（8 块，每块 1 写口 + 1 读口，三阶段分时复用）
//==============================================================================
	wire	[10:0]	ram_wr_addr	[0:COL_NUM-1];
	wire	[15:0]	ram_wr_data	[0:COL_NUM-1];
	wire			ram_wr_en	[0:COL_NUM-1];
	wire	[10:0]	ram_rd_addr	[0:COL_NUM-1];
	wire	[15:0]	ram_rd_data	[0:COL_NUM-1];

//==============================================================================
// 4. 引擎对外接口（由第 7 节 generate 驱动）
//==============================================================================
	wire	[2:0]	eng_idx		[0:ENG_NUM-1];	//引擎当前处理的行号 0~7
	wire	[8:0]	eng_cnt		[0:ENG_NUM-1];	//引擎计数器 0~256（低 8 位 = bin）
	wire	[15:0]	eng_rdata	[0:ENG_NUM-1];	//引擎读回的计数
	wire			eng_wen		[0:ENG_NUM-1];
	wire	[7:0]	eng_wbin	[0:ENG_NUM-1];	//引擎写 bin
	wire	[15:0]	eng_wdata	[0:ENG_NUM-1];

//==============================================================================
// 5. CDF 读口地址（第 k 路 → 第 k-1 块 RAM，地址={行,bin}）
//==============================================================================
	wire	[10:0]	cdf_raddr	[0:COL_NUM-1];
	assign cdf_raddr[0] = {rd_tile_index_1[5:3], rd_addr_1};
	assign cdf_raddr[1] = {rd_tile_index_2[5:3], rd_addr_2};
	assign cdf_raddr[2] = {rd_tile_index_3[5:3], rd_addr_3};
	assign cdf_raddr[3] = {rd_tile_index_4[5:3], rd_addr_4};
	assign cdf_raddr[4] = {rd_tile_index_5[5:3], rd_addr_5};
	assign cdf_raddr[5] = {rd_tile_index_6[5:3], rd_addr_6};
	assign cdf_raddr[6] = {rd_tile_index_7[5:3], rd_addr_7};
	assign cdf_raddr[7] = {rd_tile_index_8[5:3], rd_addr_8};

	//引擎与 RAM 一一对应，读数据直连（不再需要按 tile 索引做交叉选择）
	always @(*) begin
		rd_data_1 = ram_rd_data[0];
		rd_data_2 = ram_rd_data[1];
		rd_data_3 = ram_rd_data[2];
		rd_data_4 = ram_rd_data[3];
		rd_data_5 = ram_rd_data[4];
		rd_data_6 = ram_rd_data[5];
		rd_data_7 = ram_rd_data[6];
		rd_data_8 = ram_rd_data[7];
	end

//==============================================================================
// 6. 累加阶段的流水线
//------------------------------------------------------------------------------
//  第 1 级：驱动读地址，并锁存 (列,行,bin) 供下一级使用
//  第 2 级：读回旧计数 +1 后写回
//  读改写冲突用“旁路寄存器”绕开：若当前 bin 与上一拍刚写回的 (列,行,bin) 相同，
//  则直接用上一拍的写数据 +1。
//==============================================================================
	reg		[2:0]	s1_col;
	reg		[2:0]	s1_row;
	reg		[7:0]	s1_bin;
	reg				s1_de;

	always @(posedge pclk or negedge rst_n) begin
		if (!rst_n) begin
			s1_col	<= 3'd0;
			s1_row	<= 3'd0;
			s1_bin	<= 8'd0;
			s1_de	<= 1'b0;
		end
		else if (state == S_ACCUM) begin
			s1_col	<= i_tile_index[2:0];
			s1_row	<= i_tile_index[5:3];
			s1_bin	<= i_y;
			s1_de	<= i_de;
		end
		else begin
			s1_de	<= 1'b0;
		end
	end

	wire	[15:0]	accum_cand	[0:COL_NUM-1];
	genvar gm;
	generate
		for (gm = 0; gm < COL_NUM; gm = gm + 1) begin : gen_accum_mux
			localparam [2:0] GC = gm;
			assign accum_cand[gm] = (s1_col == GC) ? ram_rd_data[gm] : 16'd0;
		end
	endgenerate

	reg		[15:0]	accum_rd_data;
	integer	im;
	always @(*) begin
		accum_rd_data = 16'd0;
		for (im = 0; im < COL_NUM; im = im + 1)
			accum_rd_data = accum_rd_data | accum_cand[im];
	end

	reg				prev_wr_en;
	reg		[2:0]	prev_wr_col;
	reg		[2:0]	prev_wr_row;
	reg		[7:0]	prev_wr_bin;
	reg		[15:0]	prev_wr_cnt;

	wire	same_bin = prev_wr_en & (prev_wr_col == s1_col)
						 & (prev_wr_row == s1_row) & (prev_wr_bin == s1_bin);

	wire	[15:0]	accum_new_cnt = same_bin ? (prev_wr_cnt + 16'd1)
											: (accum_rd_data + 16'd1);

	always @(posedge pclk or negedge rst_n) begin
		if (!rst_n) begin
			prev_wr_en		<= 1'b0;
			prev_wr_col		<= 3'd0;
			prev_wr_row		<= 3'd0;
			prev_wr_bin		<= 8'd0;
			prev_wr_cnt		<= 16'd0;
		end
		else if (state == S_ACCUM) begin
			prev_wr_en		<= s1_de;
			prev_wr_col		<= s1_col;
			prev_wr_row		<= s1_row;
			prev_wr_bin		<= s1_bin;
			prev_wr_cnt		<= accum_new_cnt;
		end
		else begin
			prev_wr_en		<= 1'b0;
		end
	end

//==============================================================================
// 7. 截断 / 重分配引擎 × 8
//------------------------------------------------------------------------------
//  引擎 c 固定负责“第 c 列”的 8 个 tile（行 0~7），直接读写第 c 块 RAM。
//  每个 tile 两遍扫描，每遍 257 拍：
//    E_SCAN  (e_cnt=0~256) : 读 bin 0~255，累加超出阈值的部分 -> excess
//                             结束时 step = excess>>8（每个 bin 均摊），
//                             rem = excess & 255（余数分给前 rem 个 bin）
//    E_REDIST(e_cnt=0~256) : 写回 min(h,clip) + step + (bin<rem)
//  地址 = {e_idx(行), e_cnt[7:0]}；写地址滞后读地址 1 拍。
//==============================================================================
	genvar ge;
	generate
		for (ge = 0; ge < ENG_NUM; ge = ge + 1) begin : gen_engine

			localparam [2:0] GC       = ge;			//本引擎 = 第 GC 列
			localparam [2:0] LAST_IDX = 3'd7;

			reg		[1:0]	e_state;
			reg		[2:0]	e_idx;				//行号 0~7
			reg		[8:0]	e_cnt;				//计数器 0~256
			reg		[15:0]	e_excess;
			reg		[15:0]	e_step;
			reg		[7:0]	e_rem;

			//引擎与 RAM 一一对应：直连第 GC 块 RAM 的读数据
			wire	[15:0]	rd_mux = ram_rd_data[GC];

			assign eng_rdata [ge] = rd_mux;
			assign eng_idx   [ge] = e_idx;
			assign eng_cnt   [ge] = e_cnt;
			assign eng_idle  [ge] = (e_state == E_IDLE);

			//有效窗口：e_cnt = 1~256 期间读回的数据依次对应 bin 0~255
			wire	valid_win = (e_cnt >= 9'd1) & (e_cnt <= 9'd256);

			//第 1 遍：累加溢出量
			wire	[15:0]	over = ((e_state == E_SCAN) & valid_win
									& (rd_mux > clip_limit_r))
								   ? (rd_mux - clip_limit_r) : 16'd0;
			wire	[15:0]	excess_nxt = e_excess + over;

			//第 2 遍：削峰 + 均摊；本拍要写回的 bin = e_cnt-1（自然回绕）
			wire	[7:0]	wr_bin = e_cnt[7:0] - 8'd1;
			wire	[15:0]	h_base = (rd_mux > clip_limit_r) ? clip_limit_r : rd_mux;
			wire	[15:0]	redist_sum = h_base + e_step
									   + ((wr_bin < e_rem) ? 16'd1 : 16'd0);

			assign eng_wen  [ge] = (e_state == E_REDIST) & valid_win;
			assign eng_wbin [ge] = wr_bin;
			assign eng_wdata[ge] = redist_sum;

			always @(posedge pclk or negedge rst_n) begin
				if (!rst_n) begin
					e_state		<= E_IDLE;
					e_idx		<= 3'd0;
					e_cnt		<= 9'd0;
					e_excess	<= 16'd0;
					e_step		<= 16'd0;
					e_rem		<= 8'd0;
				end
				else begin
					case (e_state)
					//------------------------------------------------------
					E_IDLE: begin
						if (clip_start) begin
							e_idx		<= 3'd0;
							e_cnt		<= 9'd0;
							e_excess	<= 16'd0;
							e_state		<= E_SCAN;
						end
					end
					//------------------------------------------------------
					E_SCAN: begin
						e_cnt		<= e_cnt + 9'd1;
						e_excess	<= excess_nxt;

						if (e_cnt == 9'd256) begin
							e_step	<= {8'd0, excess_nxt[15:8]};	//excess>>8
							e_rem	<= excess_nxt[7:0];
							e_cnt	<= 9'd0;
							e_state	<= E_REDIST;
						end
					end
					//------------------------------------------------------
					E_REDIST: begin
						e_cnt <= e_cnt + 9'd1;

						if (e_cnt == 9'd256) begin
							e_cnt <= 9'd0;
							if (e_idx == LAST_IDX) begin
								e_state <= E_IDLE;
							end
							else begin
								e_idx		<= e_idx + 3'd1;
								e_excess	<= 16'd0;
								e_state		<= E_SCAN;
							end
						end
					end
					//------------------------------------------------------
					default: e_state <= E_IDLE;
					endcase
				end
			end

		end
	endgenerate

//==============================================================================
// 8. 每块 RAM 的读写通路多路选择
//------------------------------------------------------------------------------
//  写口两来源：清零 / 累加 / 重分配
//  读口三来源：累加 / 重分配引擎 / CDF
//==============================================================================
	genvar gp;
	generate
		for (gp = 0; gp < COL_NUM; gp = gp + 1) begin : gen_ram_path

			localparam [2:0] GC = gp;

			//---- 读地址 ----
			//累加期：只有 i_tile_index 命中本列的那块 RAM 被读；
			//        地址在整 2 拍像素周期内稳定，0/1 拍读延迟都正确
			wire	[10:0]	rd_addr_accum = (i_tile_index[2:0] == GC)
										? {i_tile_index[5:3], i_y} : 11'd0;
			//截断期：由本列引擎驱动（地址 = {行, bin}）
			wire	[10:0]	rd_addr_clip  = {eng_idx[GC], eng_cnt[GC][7:0]};
			//CDF 期：由对应逻辑口驱动
			wire	[10:0]	rd_addr_cdf   = cdf_raddr[GC];

			assign ram_rd_addr[gp] = (state == S_ACCUM) ? rd_addr_accum :
									 (state == S_CLIP ) ? rd_addr_clip  :
									 (state == S_DONE ) ? rd_addr_cdf   : 11'd0;

			//---- 写通路 ----
			wire	clr_sel     = (state == S_CLEAR);
			wire	wr_en_accum = (state == S_ACCUM) & (s1_col == GC) & s1_de;
			wire	wr_en_clip  = (state == S_CLIP ) & eng_wen[GC];

			assign ram_wr_en[gp] = clr_sel ? 1'b1 : (wr_en_accum | wr_en_clip);

			assign ram_wr_addr[gp] = clr_sel            ? clr_addr :
									 (state == S_ACCUM) ? {s1_row, s1_bin} :
									 (state == S_CLIP ) ? {eng_idx[GC], eng_wbin[GC]} : 11'd0;

			assign ram_wr_data[gp] = clr_sel            ? 16'd0 :
									 (state == S_ACCUM) ? accum_new_cnt :
									 (state == S_CLIP ) ? eng_wdata[GC] : 16'd0;

		end
	endgenerate

//==============================================================================
// 9. 8 块直方图 RAM 例化
//    每块 2048 x 16bit：地址 = {行[2:0], bin[7:0]}，对应一列 8 个 tile
//    复位接 ~rst_n（IP 复位为“输出数据复位值”，高有效、异步）
//    注意：该复位只复位输出寄存器，不动存储阵列，故必须依赖 S_CLEAR 清零。
//==============================================================================
	genvar gr;
	generate
		for (gr = 0; gr < COL_NUM; gr = gr + 1) begin : gen_hist_ram
			ram_2048x16bit u_hist_ram (
				.wr_clk		(pclk),
				.wr_rst		(~rst_n),
				.wr_en		(ram_wr_en  [gr]),
				.wr_addr	(ram_wr_addr[gr]),
				.wr_data	(ram_wr_data[gr]),
				.rd_clk		(pclk),
				.rd_rst		(~rst_n),
				.rd_addr	(ram_rd_addr[gr]),
				.rd_data	(ram_rd_data[gr])
			);
		end
	endgenerate

endmodule
