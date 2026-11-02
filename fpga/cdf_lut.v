//==============================================================================
//  cdf_lut.v  —— CLAHE 灰度映射表（CDF + 归一化）生成与查询控制器
//------------------------------------------------------------------------------
//  功能概述
//    1) 收到 hist_ram_ctrl 的 redistri_done 后启动，8 个引擎并行
//    2) 引擎 e 固定负责“第 e 列”的 8 个 tile（行 0~7），从 hist_ram_ctrl 的
//       第 e 路读口读回“已完成截断重分配”的直方图，边读边做 CDF 累积
//    3) 归一化得到 0~255 映射值，按 DRM 存入 8 块 512x32bit DRM（方案 A）
//       —— 一个字 = 一个 2x2 tile 组（4 个 tile）在同一 bin 的映射值
//          该 DRM 的 byte-write 只支持 8bit 位宽，不能用字节使能，
//          故写入采用【读出 → 改字节 → 整字写回】(RMW) 流水线
//    4) 64 个 tile 全部完成后拉高 cdf_done，通知 hist_ram_ctrl 可以清零
//    5) 对外提供 4 路映射表读口，供双线性插值使用（读延迟仍为 1 拍）
//
//  ── 存储改造（64 块分布式 RAM → 8 块 DRM，见 cdf_lut_DRM_refactor_spec.md）──
//    旧的 64 块 256x8bit 分布式 RAM 占 4096 个 ram32x1dp，导致 Place-0006
//    布局失败与 setup 违例；改造后分布式 RAM 占用为 0，DRM +8 块。
//    对外接口、流水线延迟、CDF 数学运算【全部不变】。
//    代价：bil_inter 的 4 组地址输出必须由“寄存输出”改为“组合输出”
//          （把这一级寄存器挪到 DRM 的输入侧），总延迟仍为 1 拍。
//
//  ── 与 hist_ram_ctrl 的连接（关键简化）─────────────────────────────────
//    引擎 e ↔ 直方图 RAM e ↔ 读口 e 一一对应，读数据直连，
//    不再需要按 tile 索引做 60:1 交叉选择。tile 号仍随读口送出，
//    hist_ram_ctrl 用 tile[5:3] 作为行地址。
//
//  ── 数学运算的化简（无除法、无大乘法器）──────────────────────────────
//    (1) 归一化：lut = cdf * 255 / 14400 （Npix = 160*90 = 14400）
//        255/14400 = 17/960，而 960 = 64*15，于是
//            lut = floor( ((cdf*17) >> 6) / 15 )
//        （floor(floor(x/64)/15) = floor(x/960)，正数下恒成立）
//        cdf*17 = (cdf<<4)+cdf —— 移位+加法，无乘法器。
//        除以 15 用魔数乘：floor(n/15) = (n*4370)>>16
//            4370*15 = 65550，仅比 2^16 大 14，对 n<=3825 恒精确
//            4370 = 4096+256+16+2 —— 仍是移位加法
//        综合： lut = ((cdf*17)>>6 * 4370) >> 16
//        端点校验：cdf=14400 -> 244800>>6=3825 -> 3825*4370>>16=255 ✓
//    (2) 饱和：正常范围 lut 必然 <=255；仍保留判断防止 cdf 异常。
//    (3) tile 号 = e + 8*e_idx ；8 即移位。
//
//  ── 流水线切分（与原设计一致）──────────────────────────────────────────
//    第 1 级：读直方图 → CDF 累加 → 寄存 [p_cdf / p_bin / p_valid]
//    第 2 级：p_cdf → 归一化 → 写映射表 RAM
//    代价是每个 tile 多一拍排空（E_DRAIN），整帧可忽略。
//
//  ── RAM 读时序：rd_data 比 rd_addr 滞后 1 拍 ──────────────────────────
//==============================================================================

module cdf_lut(
	input					pclk,
	input					rst_n,

//与 hist_ram_ctrl 的握手
	input					redistri_done,
	output	reg				cdf_done,

//向 hist_ram_ctrl 请求重分配后的直方图（8 路并行，读地址即原始灰度值）
	output	reg		[5:0]	rd_tile_index_1,
	output	reg		[7:0]	rd_addr_1,
	input			[15:0]	rd_data_1,
	output	reg		[5:0]	rd_tile_index_2,
	output	reg		[7:0]	rd_addr_2,
	input			[15:0]	rd_data_2,
	output	reg		[5:0]	rd_tile_index_3,
	output	reg		[7:0]	rd_addr_3,
	input			[15:0]	rd_data_3,
	output	reg		[5:0]	rd_tile_index_4,
	output	reg		[7:0]	rd_addr_4,
	input			[15:0]	rd_data_4,
	output	reg		[5:0]	rd_tile_index_5,
	output	reg		[7:0]	rd_addr_5,
	input			[15:0]	rd_data_5,
	output	reg		[5:0]	rd_tile_index_6,
	output	reg		[7:0]	rd_addr_6,
	input			[15:0]	rd_data_6,
	output	reg		[5:0]	rd_tile_index_7,
	output	reg		[7:0]	rd_addr_7,
	input			[15:0]	rd_data_7,
	output	reg		[5:0]	rd_tile_index_8,
	output	reg		[7:0]	rd_addr_8,
	input			[15:0]	rd_data_8,

//映射表读口（4 路并行，供双线性插值使用）
	input			[5:0]	lut_tile_index_1,
	input			[7:0]	lut_addr_1,
	input					lut_index_val_1,
	output	reg		[7:0]	lut_data_1,
	input			[5:0]	lut_tile_index_2,
	input			[7:0]	lut_addr_2,
	input					lut_index_val_2,
	output	reg		[7:0]	lut_data_2,
	input			[5:0]	lut_tile_index_3,
	input			[7:0]	lut_addr_3,
	input					lut_index_val_3,
	output	reg		[7:0]	lut_data_3,
	input			[5:0]	lut_tile_index_4,
	input			[7:0]	lut_addr_4,
	input					lut_index_val_4,
	output	reg		[7:0]	lut_data_4
   );

//==============================================================================
// 1. 常量定义
//==============================================================================
	localparam	TILE_NUM	= 64;			//tile 总数
	localparam	ENG_NUM		= 8;			//并行运算引擎数（= 列数）
	localparam	LUT_PORT_N	= 4;			//映射表读口数
	localparam	BIN_NUM		= 256;			//bin 数 = LUT 深度
	localparam	TILE_PIX	= 14400;		//每 tile 像素数 160*90

	//---- 映射表 DRM 参数（方案 A：64 块分布式 RAM → 8 块 DRM）----
	localparam	DRM_NUM		= 8;			//DRM 块数
	localparam	DRM_AW		= 9;			//字地址位宽：{gy[1], bin[7:0]} -> 512 字
	localparam	DRM_BE_N	= 4;			//字节使能数（每字 4 个 tile）

	//引擎状态机
	localparam	[1:0]	E_IDLE	= 2'd0;
	localparam	[1:0]	E_RUN	= 2'd1;		//读直方图 + CDF 累积（第 1 级）
	localparam	[1:0]	E_DRAIN	= 2'd2;		//排空流水线：写最后一个 bin

//==============================================================================
// 2. 与 hist_ram_ctrl 的握手
//    redistri_done 保持高直到本模块回送 cdf_done、主控清零后才拉低，
//    因此启动必须用“上升沿”。
//==============================================================================
	reg				redistri_d1;
	wire			redistri_rise = redistri_done & ~redistri_d1;

	always @(posedge pclk or negedge rst_n) begin
		if (!rst_n) redistri_d1 <= 1'b0;
		else        redistri_d1 <= redistri_done;
	end

	wire	[1:0]	eng_state	[0:ENG_NUM-1];
	wire	[2:0]	eng_idx		[0:ENG_NUM-1];	//行号 0~7
	wire	[8:0]	eng_cnt		[0:ENG_NUM-1];	//计数器 0~256
	wire	[7:0]	eng_lut_val	[0:ENG_NUM-1];
	wire	[7:0]	eng_lut_waddr[0:ENG_NUM-1];
	wire			eng_lut_wen	[0:ENG_NUM-1];

	wire	[ENG_NUM-1:0]	eng_idle;
	wire					all_eng_idle = &eng_idle;

	reg				run_active;

	always @(posedge pclk or negedge rst_n) begin
		if (!rst_n) begin
			run_active <= 1'b0;
			cdf_done   <= 1'b0;
		end
		else if (!redistri_done) begin
			run_active <= 1'b0;
			cdf_done   <= 1'b0;
		end
		else if (redistri_rise) begin
			run_active <= 1'b1;
		end
		else if (run_active & all_eng_idle) begin
			run_active <= 1'b0;
			cdf_done   <= 1'b1;
		end
	end

//==============================================================================
// 3. 引擎与直方图读口的连接（一一对应，直连）
//==============================================================================
	wire	[5:0]	eng_tile	[0:ENG_NUM-1];
	wire	[7:0]	eng_raddr	[0:ENG_NUM-1];
	reg		[15:0]	eng_rdata	[0:ENG_NUM-1];

	always @(*) begin
		rd_tile_index_1  = eng_tile[0];		rd_addr_1  = eng_raddr[0];
		rd_tile_index_2  = eng_tile[1];		rd_addr_2  = eng_raddr[1];
		rd_tile_index_3  = eng_tile[2];		rd_addr_3  = eng_raddr[2];
		rd_tile_index_4  = eng_tile[3];		rd_addr_4  = eng_raddr[3];
		rd_tile_index_5  = eng_tile[4];		rd_addr_5  = eng_raddr[4];
		rd_tile_index_6  = eng_tile[5];		rd_addr_6  = eng_raddr[5];
		rd_tile_index_7  = eng_tile[6];		rd_addr_7  = eng_raddr[6];
		rd_tile_index_8  = eng_tile[7];		rd_addr_8  = eng_raddr[7];
	end

	always @(*) begin
		eng_rdata[0] = rd_data_1;
		eng_rdata[1] = rd_data_2;
		eng_rdata[2] = rd_data_3;
		eng_rdata[3] = rd_data_4;
		eng_rdata[4] = rd_data_5;
		eng_rdata[5] = rd_data_6;
		eng_rdata[6] = rd_data_7;
		eng_rdata[7] = rd_data_8;
	end

//==============================================================================
// 4. 映射表存储接口（DRM 版）
//------------------------------------------------------------------------------
//  存储思想（规格书 §1/§3）：双线性插值的 4 个 tap 读的是【同一个 bin】
//  【不同 tile】，所以把 tile 放进 DRM 的【字宽】而不是【地址】：
//     一个字 = 32 bit = 同一个 2×2 tile 组里 4 个 tile 在同一 bin 的映射值
//     8×8 个 tile 切成 4×4 = 16 个不重叠的 2×2 组
//     ⇒ 16 组 × 256 bin × 32 bit = 128 Kb，只需 8 块 DRM（每块 512 字）
//
//  位域映射（全部是位切片，无算术；与规格书 §3.3 一致）：
//     tx = t[2:0]   ty = t[5:3]
//     DRM 号  d    = {t[4], t[2], t[1]}   = {gy[0], gx[1], gx[0]}   (3 bit)
//     字地址  hsel = t[5]                 = gy[1]
//     字地址       = {t[5], bin[7:0]}                              (9 bit)
//     字节通道 be  = {t[3], t[0]}        = {ty[0], tx[0]}          (2 bit)
//   该映射同时满足：
//     ① 同一时刻 4 个引擎列(gx=0..3)×同一行 → 落 4 块不同 DRM → 写不冲突
//     ② 任意 4 个相邻组(gx..gx+1, gy..gy+1) → 落 4 块不同 DRM → 读可并发
//==============================================================================
	//---- DRM 写口（8 块，整字写：IP 无字节使能，改用 RMW）----
	wire				drm_wr_en	[0:DRM_NUM-1];
	wire	[DRM_AW-1:0]	drm_wr_addr	[0:DRM_NUM-1];
	wire	[31:0]		drm_wr_data	[0:DRM_NUM-1];

	//---- DRM 读口（8 块；写回期间被 RMW 复用，其余时间用于插值查表）----
	wire	[DRM_AW-1:0]	drm_rd_addr	[0:DRM_NUM-1];
	wire	[31:0]		drm_rd_data	[0:DRM_NUM-1];

//==============================================================================
// 5. CDF 运算引擎 × 8（两级流水）
//------------------------------------------------------------------------------
//  时序（每 tile 258 拍）
//    第 1 级 E_RUN，e_cnt = 0~256
//        e_cnt=0        : 发读地址 0，数据未回，不累加
//        e_cnt=1~256    : 读回数据依次对应 bin 0~255，cdf_nxt = e_cdf + h[i]
//                         将 cdf_nxt / bin 号 / 有效标志打入流水寄存器
//    第 2 级：lut = ((cdf*17)>>6 * 4370)>>16 ，写回 p_bin 指向的地址
//    E_DRAIN（1 拍）：排空最后一级，写出 bin 255
//==============================================================================
	genvar ge;
	generate
		for (ge = 0; ge < ENG_NUM; ge = ge + 1) begin : gen_cdf_engine

			localparam [2:0] GC = ge;			//本引擎 = 第 GC 列

			reg		[1:0]	e_state;
			reg		[2:0]	e_idx;				//行号 0~7
			reg		[8:0]	e_cnt;				//计数器 0~256
			reg		[14:0]	e_cdf;				//CDF 累积值，最大 14400

			//---- 流水线寄存器（第 1 级 → 第 2 级）----
			reg		[14:0]	p_cdf;
			reg		[7:0]	p_bin;
			reg				p_valid;

			//tile 号 = e_idx*8 + GC （*8 即移位）
			wire	[5:0]	e_tile = {e_idx, 3'd0} + {3'd0, GC};

			assign eng_tile [ge] = e_tile;
			assign eng_raddr[ge] = e_cnt[7:0];
			assign eng_state[ge] = e_state;
			assign eng_idx  [ge] = e_idx;
			assign eng_cnt  [ge] = e_cnt;
			assign eng_idle [ge] = (e_state == E_IDLE);

			//有效窗口：e_cnt=1~256（e_cnt 在 E_RUN 内最多到 256，故化简为 |e_cnt）
			wire	valid_win = |e_cnt;

			//第 1 级：读直方图 + CDF 累积
			wire	[14:0]	cdf_nxt = e_cdf + eng_rdata[ge];

			always @(posedge pclk or negedge rst_n) begin
				if (!rst_n) begin
					p_cdf	<= 15'd0;
					p_bin	<= 8'd0;
					p_valid	<= 1'b0;
					e_cdf	<= 15'd0;
				end
				else if (e_state == E_RUN) begin
					p_cdf	<= cdf_nxt;
					//? 写地址比读地址滞后 1 拍，与读回数据的 bin 号对齐
					p_bin	<= e_cnt[7:0] - 8'd1;
					p_valid	<= valid_win;
					e_cdf	<= valid_win ? cdf_nxt : e_cdf;
				end
				else if (e_state == E_DRAIN) begin
					p_valid	<= 1'b0;
					e_cdf	<= 15'd0;			//换 tile，累积值清零
				end
				else begin
					p_valid	<= 1'b0;
				end
			end

			//第 2 级：归一化 lut = floor(cdf*255/14400) = ((cdf*17)>>6 * 4370)>>16
			wire	[19:0]	p_cdf_x17 = {p_cdf, 4'b0000} + {5'd0, p_cdf};	//*17
			wire	[13:0]	p_n6      = p_cdf_x17 >> 6;						//>>6
			wire	[26:0]	p_n6_e    = {13'd0, p_n6};
			wire	[26:0]	p_m       = (p_n6_e << 12) + (p_n6_e << 8)
								  + (p_n6_e << 4)  + (p_n6_e << 1);		//*4370
			wire	[10:0]	p_lutq    = p_m >> 16;
			wire	[7:0]	lut_val   = (p_lutq > 11'd255) ? 8'hFF : p_lutq[7:0];

			assign eng_lut_wen  [ge] = p_valid & (e_state != E_IDLE);
			assign eng_lut_waddr[ge] = p_bin;
			assign eng_lut_val  [ge] = lut_val;

			always @(posedge pclk or negedge rst_n) begin
				if (!rst_n) begin
					e_state	<= E_IDLE;
					e_idx	<= 3'd0;
					e_cnt	<= 9'd0;
				end
				else begin
					case (e_state)
					//------------------------------------------------------
					E_IDLE: begin
						if (redistri_rise) begin
							e_idx	<= 3'd0;
							e_cnt	<= 9'd0;
							e_state	<= E_RUN;
						end
					end
					//------------------------------------------------------
					E_RUN: begin
						if (e_cnt == 9'd256) begin
							e_state <= E_DRAIN;
						end
						else begin
							e_cnt <= e_cnt + 9'd1;
						end
					end
					//------------------------------------------------------
					// 排空：p_valid 仍为 1、p_bin=255、p_cdf 为最终累积值，
					//       e_idx 尚未递增，写路由仍指向本 tile，正好写最后一个 bin
					//------------------------------------------------------
					E_DRAIN: begin
						e_cnt <= 9'd0;
						if (e_idx == 3'd7) begin
							e_state <= E_IDLE;
						end
						else begin
							e_idx	<= e_idx + 3'd1;
							e_state	<= E_RUN;
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
// 6. 映射表写通道（读出 → 改字节 → 整字写回，RMW）
//------------------------------------------------------------------------------
//  ★ 该 DRM IP 的 byte-write 只支持 8bit 位宽，不能用字节使能，
//    因此写 1 个字节也要写整字，必须做 read-modify-write：
//       第 0 拍：把目标字地址送 DRM 读口，并锁存 {写使能,字地址,字节使能,字节数据}
//       第 1 拍：DRM 数据回来 → 用掩码把新字节并进去 → 整字写回同一地址
//    吞吐不变（每块 DRM 每拍仍只提交 1 次整字写），只多 1 拍流水延迟。
//
//  写请求来源：64 个 tile 各自算出 (d, hsel, be_sel)，汇聚到 8 块 DRM。
//  tile t 由引擎 (t%8) 处理，行 = t/8；保留原有的“本引擎正在处理本行”门控。
//  同一 DRM 每拍只会收到【同一字地址】的 2 个字节通道（tx 奇偶配对的两个引擎，
//  同一行同一 bin），所以读一次字、并 2 个字节、写一次整字即可，无冲突。
//==============================================================================
	integer	wt, wd, sp;
	reg		[5:0]	tw;					//tile 号的 6 位副本（避免对 integer 做位选）
	reg		[2:0]	w_eng, w_jidx, w_d;
	reg		[1:0]	w_be;
	reg				w_wen;

	//---- 第 0 拍：写请求（组合，来自 64 个 tile）----
	reg				req_en		[0:DRM_NUM-1];
	reg		[DRM_AW-1:0]	req_addr	[0:DRM_NUM-1];
	reg		[31:0]		req_dat		[0:DRM_NUM-1];	//新字节已放到对应字节位
	reg		[DRM_BE_N-1:0]	req_be		[0:DRM_NUM-1];

	always @(*) begin
		for (wd = 0; wd < DRM_NUM; wd = wd + 1) begin
			req_en	[wd] = 1'b0;
			req_addr[wd] = {DRM_AW{1'b0}};
			req_dat [wd] = 32'd0;
			req_be  [wd] = {DRM_BE_N{1'b0}};
		end
		for (wt = 0; wt < TILE_NUM; wt = wt + 1) begin
			tw     = wt;
			w_eng  = tw[2:0];
			w_jidx = tw[5:3];
			w_wen  = eng_lut_wen[w_eng] & (eng_idx[w_eng] == w_jidx);
			if (w_wen) begin
				w_d  = {tw[4], tw[2], tw[1]};			//目标 DRM
				w_be = {tw[3], tw[0]};					//字节通道
				req_en  [w_d]              = 1'b1;
				req_addr[w_d]              = {tw[5], eng_lut_waddr[w_eng]};
				req_be  [w_d][w_be]        = 1'b1;
				req_dat [w_d][w_be*8 +: 8] = eng_lut_val[w_eng];
			end
		end
	end

	// 任一 DRM 有写请求 → 本拍读口让给 RMW（此时无像素在取映射表）
	wire	cdf_wr_act = |{req_en[DRM_NUM-1], req_en[DRM_NUM-2], req_en[DRM_NUM-3],
					   req_en[DRM_NUM-4], req_en[DRM_NUM-5], req_en[DRM_NUM-6],
					   req_en[DRM_NUM-7], req_en[DRM_NUM-8]};

	//---- 第 1 拍：锁存写请求；DRM 读回目标字 ----
	reg				s1_en		[0:DRM_NUM-1];
	reg		[DRM_AW-1:0]	s1_addr		[0:DRM_NUM-1];
	reg		[DRM_BE_N-1:0]	s1_be		[0:DRM_NUM-1];
	reg		[31:0]		s1_dat		[0:DRM_NUM-1];

	//---- 旁路：上一拍刚写回的字（若本拍读地址与它相同，读回数据会漏掉那笔写）----
	reg				s2_en		[0:DRM_NUM-1];
	reg		[DRM_AW-1:0]	s2_addr		[0:DRM_NUM-1];
	reg		[31:0]		s2_dat		[0:DRM_NUM-1];

	//---- 字节掩码合并 ----
	wire	[31:0]	rbase	[0:DRM_NUM-1];
	wire	[31:0]	rmask	[0:DRM_NUM-1];
	wire	[31:0]	rmerge	[0:DRM_NUM-1];

	genvar gs;
	generate
		for (gs = 0; gs < DRM_NUM; gs = gs + 1) begin : gen_lut_rmw
			wire hit_prev = s2_en[gs] & (s1_addr[gs] == s2_addr[gs]);
			assign rbase [gs] = hit_prev ? s2_dat[gs] : drm_rd_data[gs];
			assign rmask [gs] = {{8{s1_be[gs][3]}}, {8{s1_be[gs][2]}},
								 {8{s1_be[gs][1]}}, {8{s1_be[gs][0]}}};
			assign rmerge[gs] = (rbase[gs] & ~rmask[gs]) | (s1_dat[gs] & rmask[gs]);

			assign drm_wr_en  [gs] = s1_en  [gs];
			assign drm_wr_addr[gs] = s1_addr[gs];
			assign drm_wr_data[gs] = rmerge  [gs];
		end
	endgenerate

	always @(posedge pclk or negedge rst_n) begin
		if (!rst_n) begin
			for (sp = 0; sp < DRM_NUM; sp = sp + 1) begin
				s1_en  [sp] <= 1'b0;
				s1_addr[sp] <= {DRM_AW{1'b0}};
				s1_be  [sp] <= {DRM_BE_N{1'b0}};
				s1_dat [sp] <= 32'd0;
				s2_en  [sp] <= 1'b0;
				s2_addr[sp] <= {DRM_AW{1'b0}};
				s2_dat [sp] <= 32'd0;
			end
		end
		else begin
			for (sp = 0; sp < DRM_NUM; sp = sp + 1) begin
				s1_en  [sp] <= req_en  [sp];
				s1_addr[sp] <= req_addr[sp];
				s1_be  [sp] <= req_be  [sp];
				s1_dat [sp] <= req_dat [sp];
				s2_en  [sp] <= s1_en  [sp];
				s2_addr[sp] <= s1_addr[sp];
				s2_dat [sp] <= rmerge [sp];
			end
		end
	end

//==============================================================================
// 7. 映射表读通道（4 个逻辑口 → 8 块 DRM 地址分发 → 字节优先数据回收）
//------------------------------------------------------------------------------
//  每个端口由 tile 号 idx 与 bin 值 a 算出 3 个量（规格书 §4.1）：
//      drm_sel  = {idx[4], idx[2], idx[1]}   （3 bit，目标 DRM）
//      word_a   = {idx[5], a[7:0]}           （9 bit，字地址）
//      byte_sel = {idx[3], idx[0]}           （2 bit，组内 4 字节之一）
//
//  地址分发（4 → 8）：每块 DRM 取“第一个指向本 DRM 的端口”的字地址。
//    正确性：4 个 tap 的组集合 gx∈{tx>>1,(tx+1)>>1}、gy∈{ty>>1,(ty+1)>>1}，
//    而 gy 与 gy+1 的 gy[0] 必不同，故两个组不会落在同一块 DRM；
//    又因为 4 个端口的 lut_addr_n 都是同一个 I_Y，
//    所以指向同一 DRM 的端口其 word_a 必然相同 → 取任一个都正确。
//
//  读延迟 1 拍（DRM 带输出寄存器）：drm_sel / byte_sel 打一拍后与数据对齐。
//    ★ 门控用【当前】的 lut_index_val_n，不再打拍：
//      bil_inter 已把 index_val 寄存了一拍，而地址是组合输出，
//      “组合地址 + DRM 1 拍同步读”后数据出现在 t+1 拍，
//      正好与 t+1 拍的 lut_index_val_n(=i_de(t)) 同拍。
//      若再打一拍会整体后移一拍，2 pclk/像素时序下会全部门控失效。
//==============================================================================
	wire	[2:0]	dsel	[0:LUT_PORT_N-1];	//目标 DRM
	wire	[DRM_AW-1:0]	wadp	[0:LUT_PORT_N-1];	//字地址
	wire	[1:0]	bsel	[0:LUT_PORT_N-1];	//字节通道

	assign dsel[0] = {lut_tile_index_1[4], lut_tile_index_1[2], lut_tile_index_1[1]};
	assign wadp[0] = {lut_tile_index_1[5], lut_addr_1};
	assign bsel[0] = {lut_tile_index_1[3], lut_tile_index_1[0]};

	assign dsel[1] = {lut_tile_index_2[4], lut_tile_index_2[2], lut_tile_index_2[1]};
	assign wadp[1] = {lut_tile_index_2[5], lut_addr_2};
	assign bsel[1] = {lut_tile_index_2[3], lut_tile_index_2[0]};

	assign dsel[2] = {lut_tile_index_3[4], lut_tile_index_3[2], lut_tile_index_3[1]};
	assign wadp[2] = {lut_tile_index_3[5], lut_addr_3};
	assign bsel[2] = {lut_tile_index_3[3], lut_tile_index_3[0]};

	assign dsel[3] = {lut_tile_index_4[4], lut_tile_index_4[2], lut_tile_index_4[1]};
	assign wadp[3] = {lut_tile_index_4[5], lut_addr_4};
	assign bsel[3] = {lut_tile_index_4[3], lut_tile_index_4[0]};

	//---- 地址分发：4 → 8（每块 DRM 一个 4:1 mux）----
	//     写回期间（cdf_wr_act）读口让给 RMW，用该 DRM 自己的写请求地址；
	//     其余时间用于插值查表。
	genvar gda;
	generate
		for (gda = 0; gda < DRM_NUM; gda = gda + 1) begin : gen_lut_rd_addr
			localparam [2:0] GD = gda;
			wire [DRM_AW-1:0] itp_a = (dsel[0] == GD) ? wadp[0] :
									  (dsel[1] == GD) ? wadp[1] :
									  (dsel[2] == GD) ? wadp[2] :
									  (dsel[3] == GD) ? wadp[3] : {DRM_AW{1'b0}};
			assign drm_rd_addr[gda] = cdf_wr_act ? req_addr[gda] : itp_a;
		end
	endgenerate

	//---- 选择信息打一拍（与 DRM 的 1 拍读延迟对齐）----
	reg		[2:0]	dsel_d	[0:LUT_PORT_N-1];
	reg		[1:0]	bsel_d	[0:LUT_PORT_N-1];

	integer	lp;
	always @(posedge pclk or negedge rst_n) begin
		if (!rst_n) begin
			for (lp = 0; lp < LUT_PORT_N; lp = lp + 1) begin
				dsel_d[lp] <= 3'd0;
				bsel_d[lp] <= 2'd0;
			end
		end
		else begin
			for (lp = 0; lp < LUT_PORT_N; lp = lp + 1) begin
				dsel_d[lp] <= dsel[lp];
				bsel_d[lp] <= bsel[lp];
			end
		end
	end

	//---- 数据回收：先做 4:1 选字节，再做 8:1 选 DRM ----
	wire	[7:0]	byte_cand	[0:LUT_PORT_N-1][0:DRM_NUM-1];

	genvar gk, gd2;
	generate
		for (gk = 0; gk < LUT_PORT_N; gk = gk + 1) begin : gen_lut_byte
			for (gd2 = 0; gd2 < DRM_NUM; gd2 = gd2 + 1) begin : gen_byte
				assign byte_cand[gk][gd2] = drm_rd_data[gd2][bsel_d[gk]*8 +: 8];
			end
		end
	endgenerate

	always @(*) begin
		lut_data_1 = lut_index_val_1 ? byte_cand[0][dsel_d[0]] : 8'd0;
		lut_data_2 = lut_index_val_2 ? byte_cand[1][dsel_d[1]] : 8'd0;
		lut_data_3 = lut_index_val_3 ? byte_cand[2][dsel_d[2]] : 8'd0;
		lut_data_4 = lut_index_val_4 ? byte_cand[3][dsel_d[3]] : 8'd0;
	end

//==============================================================================
// 8. 8 块映射表 DRM 例化（512 字 × 32bit，SDP，整字写，输出寄存）
//------------------------------------------------------------------------------
//  IP 由工程师用 Pango IP Compiler 生成：
//      深度 512 / 位宽 32 / SDP / OUTPUT_REG=1 / 复位 ASYNC / 初始化 NONE / PGL50H
//      ★ 不开 byte-write（该 IP 的字节使能只支持 8bit 位宽）
//  wrapper 模块名：ram_512x32bit（端口见下方例化）
//==============================================================================
	genvar gr;
	generate
		for (gr = 0; gr < DRM_NUM; gr = gr + 1) begin : gen_lut_drm
			ram_512x32bit u_lut_drm (
				.wr_clk		(pclk),
				.wr_rst		(~rst_n),
				.wr_en		(drm_wr_en  [gr]),
				.wr_addr	(drm_wr_addr[gr]),
				.wr_data	(drm_wr_data[gr]),
				.rd_clk		(pclk),
				.rd_rst		(~rst_n),
				.rd_addr	(drm_rd_addr[gr]),
				.rd_data	(drm_rd_data[gr])
			);
		end
	endgenerate

endmodule
