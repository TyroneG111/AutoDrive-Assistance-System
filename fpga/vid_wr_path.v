`timescale 1ns/1ps
//=============================================================================
// vid_wr_path —— 单路“像素 -> DDR”写通路（方案 ③④⑤ 合体）
//
// 结构：
//   像素域：vid_frontend + afifo_pack(16bit -> 256bit)
//   AXI 域：突发生成 + 每帧选择目标块（4 块轮转）+ 溢出/异常保护
//
// 地址约定（单位：256bit 字，见方案文档 §4⑤）：
//   块基址 = {CH_ID[1:0], blk[1:0], {BLK_ALIGN{1'b0}}}   每块 2^BLK_ALIGN 字对齐
//   块内偏移 blk_off = 0 .. BLK_WORDS-1（一帧 = 115200 字 = 7200 个 512B 突发）
//
//   【2026-10 YUV444 改造】输入由 16bit/像素 改为 24bit/像素（YUV444）；
//   打包 FIFO 由 16i_256o 改为 32i_256o，每个 256bit 字 = 8 个 32bit 通道，
//   每个 32bit 通道存放 1 个 24bit 像素（高 8bit 补零）-> 一字 = 8 像素。
//
// “0 丢帧”策略：
//   · 主路径：每个 VS 边界选一次目标块，写满 BLK_WORDS 个字即完成该块
//   · 若目标块正好是被读的块 → 本帧整帧丢弃并计数（ovf_cnt），不覆盖正在读的块
//   · 若在写帧过程中又出现 VS 边界（源帧不完整）→ 丢弃该块并排空 FIFO（err_cnt）
//   —— 两种异常都不会静默丢数据，都有计数器
//=============================================================================
module vid_wr_path #(
    parameter CH_ID       = 0,
    parameter DW          = 256,
    parameter ADDR_W      = 25,
    parameter LEN_W       = 10,
    parameter BURST_WORDS = 16,             // 每次突发 16 个 256bit 字 = 512B
    parameter BLK_WORDS   = 115200,         // 一帧 = 1280*720/8 字
    parameter BLK_ALIGN   = 17,             // 块对齐位数（2^17 字 = 4MB，> 115200）
    parameter FIFO_AW     = 9,              // 打包 FIFO 深度 = 2^9 个 256bit 字
    // 视频时序
    parameter H_DE_PULSES = 1280,
    parameter V_ACTIVE    = 720,
    parameter H_TOTAL     = 3300,
    parameter V_TOTAL     = 750
)(
    // ---- 像素域 ----
    input  wire             pclk,
    input  wire             pclk_rst_n,
    input  wire             vs,
    input  wire             de,
    input  wire [23:0]      pix,
    // ---- 到写仲裁器（core 域） ----
    input  wire             core_clk,
    input  wire             core_rst_n,
    output wire             wr_req,
    output wire [LEN_W-1:0] wr_len,
    output wire [ADDR_W-1:0] wr_addr,
    input  wire             wr_data_req,
    output wire [DW-1:0]    wr_data,
    input  wire             wr_finish,
    // ---- 与读侧（均在 core 域） ----
    output reg  [1:0]       blk_wr,         // 最新完整块
    input  wire [1:0]       blk_rd,         // 正在被读的块
    output wire             new_frame,      // 有未被读过的新帧
    output wire             first_frame,    // 至少完成过一帧
    // ---- 状态/计数 ----
    output wire [31:0]      ovf_cnt,        // 因缓冲满丢帧
    output wire [31:0]      err_cnt,        // 源帧异常导致丢块
    output wire [31:0]      fifo_ovf_cnt,   // 打包 FIFO 溢出丢字
    output wire             stale,
    output wire             frame_err,
    output wire [11:0]      ln_cnt_dbg
);
    //=========================================================
    // 像素域：前端解析 + 打包 FIFO
    //=========================================================
    wire        px_vld;
    wire [23:0] px_data;
    wire        frame_start, frame_done;
    wire [31:0] fe_err_cnt;

    vid_frontend #(
        .H_DE_PULSES (H_DE_PULSES),
        .V_ACTIVE    (V_ACTIVE),
        .H_TOTAL     (H_TOTAL),
        .V_TOTAL     (V_TOTAL)
    ) u_frontend (
        .pclk        (pclk),
        .rst_n       (pclk_rst_n),
        .vs          (vs),
        .de          (de),
        .pix         (pix),
        .px_vld      (px_vld),
        .px_data     (px_data),
        .ln_cnt      (ln_cnt_dbg),
        .frame_start (frame_start),
        .frame_done  (frame_done),
        .frame_err   (frame_err),
        .stale       (stale),
        .err_cnt     (fe_err_cnt)
    );

    // 帧边界 toggle（跨到 core 域用）
    reg fs_tog;
    always @(posedge pclk or negedge pclk_rst_n) begin
        if (!pclk_rst_n)    fs_tog <= 1'b0;
        else if (frame_start) fs_tog <= ~fs_tog;
    end

    // 【帧使能】只有收到过帧边界之后才允许写入打包 FIFO：
    // 否则上电到首个 VS 之间的“残留像素”会把 8192 深的 FIFO 灌满并溢出。
    reg frame_seen;
    always @(posedge pclk or negedge pclk_rst_n) begin
        if (!pclk_rst_n)       frame_seen <= 1'b0;
        else if (frame_start)  frame_seen <= 1'b1;
    end
    wire pack_we = px_vld & frame_seen;

    wire [DW-1:0]  pack_rd_data;
    wire           pack_rd_empty;
    wire [FIFO_AW:0] pack_usedw;             // = rd_water_level，单位：256bit 字
    wire           pack_wr_full;
    wire           fifo_pop;                 // 取数（细节见下方 assign）

    //-------------------------------------------------------------------------
    // 打包 FIFO：DRM Based FIFO IP
    //   实例名 afifo_32i_256o_4096（写 32bit/12 位地址=4096 深，读 256bit/9 位地址=512 字）
    //   属性：ASYN_FIFO、混合位宽、ASYNC 复位（高有效）
    //   位序：窄->宽打包在写域完成，先写入的像素放在宽字低位（与 IP 默认一致）
    //-------------------------------------------------------------------------
    afifo_32i_256o_4096 u_pack (
        .wr_clk         (pclk),
        .wr_rst         (~pclk_rst_n),
        .wr_en          (pack_we),
        .wr_data        ({8'h00, px_data}),
        .wr_full        (pack_wr_full),
        .rd_clk         (core_clk),
        .rd_rst         (~core_rst_n),
        .rd_en          (fifo_pop),
        .rd_data        (pack_rd_data),
        .rd_empty       (pack_rd_empty),
        .rd_water_level (pack_usedw),
		.almost_empty	(),
		.almost_full	()
    );

    // FIFO 写满仍写入 -> 丢字（正常不应发生，作保护）
    // 用 toggle 把事件跨到 core 域计数，避免多比特计数器直接跨域
    reg pack_ovf_tog;
    always @(posedge pclk or negedge pclk_rst_n) begin
        if (!pclk_rst_n)                pack_ovf_tog <= 1'b0;
        else if (pack_we & pack_wr_full) pack_ovf_tog <= ~pack_ovf_tog;
    end

    //=========================================================
    // core 域：帧边界检测
    //=========================================================
    reg fs_tog_s1, fs_tog_s2, fs_tog_s3;
    wire fs_tog_sync = fs_tog_s2;
    always @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
            fs_tog_s1 <= 1'b0;
            fs_tog_s2 <= 1'b0;
            fs_tog_s3 <= 1'b0;
        end else begin
            fs_tog_s1 <= fs_tog;
            fs_tog_s2 <= fs_tog_s1;
            fs_tog_s3 <= fs_tog_s2;
        end
    end
    wire fs_new = (fs_tog_s3 ^ fs_tog_sync);         // 有 1 个新帧边界待处理

    //=========================================================
    // core 域：写状态机
    //=========================================================
    localparam S_IDLE  = 2'd0,      // 等帧边界
               S_WRITE = 2'd1,      // 写满一个块
               S_DRAIN = 2'd2,      // 丢弃：排空 FIFO 到下一帧边界
               S_FLUSH = 2'd3;      // 上电首帧：排空 VS 之前的残留像素后再开写

    reg [1:0]  st;
    reg [1:0]  blk_wr_nxt;
    reg [ADDR_W-1:0] blk_off;       // 当前帧已写字数（256bit 字）
    reg              issue;         // 发出一次突发请求
    reg              burst_busy;
    reg  [LEN_W-1:0] b_len;
    reg  [ADDR_W-1:0] b_addr;
    reg  [31:0]      ovf_cnt_r, err_cnt_r;  // 丢帧 / 丢块 计数
    reg  [31:0]      fifo_ovf_r;            // 打包 FIFO 溢出丢字计数
    reg              first_r;
    reg              discard;

    // pack_ovf_tog（像素域）-> core 域：两级同步 + 边沿检测
    reg  pack_ovf_s1, pack_ovf_s2;
    always @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
            pack_ovf_s1 <= 1'b0;
            pack_ovf_s2 <= 1'b0;
        end else begin
            pack_ovf_s1 <= pack_ovf_tog;
            pack_ovf_s2 <= pack_ovf_s1;
        end
    end
    wire pack_ovf_pulse = pack_ovf_s1 ^ pack_ovf_s2;

    assign wr_req   = issue;
    assign wr_len   = b_len;
    assign wr_addr  = b_addr;
    // 【FIFO 读时序】真实 IP 的 rd_data 在 rd_en 之后一拍才有效，因此：
    //   · 突发发起那一拍先“预取”第一个字（start_burst 触发一次 pop）；
    //   · 之后每个 wr_data_req 取下一个字，但【最后一拍不再取】——否则每个突发会多吞一个字；
    //   · 主控在本拍采样的 pack_rd_data 正是上一拍 pop 出来的字，故直接组合送主控
    //     （FIFO 的选通输出寄存器在两次 pop 之间保持不变，突发内数据稳定）。
    assign wr_data  = pack_rd_data;
    assign new_frame = (blk_wr != blk_rd);   // 有未被读走的新帧（只能指向已写完整的块）
    assign first_frame = first_r;
    assign ovf_cnt  = ovf_cnt_r;
    assign err_cnt  = err_cnt_r;
    assign fifo_ovf_cnt = fifo_ovf_r;

    // 块基址（字地址）：{CH_ID, blk, BLK_ALIGN 个 0}
    wire [ADDR_W-1:0] blk_base = {CH_ID[1:0], blk_wr_nxt, {BLK_ALIGN{1'b0}}};

    // 本帧的【目标块】组合值：必须用它与 blk_rd 比较（见 S_IDLE 注释）
    wire [1:0]        blk_nxt_w = (blk_wr == 2'd3) ? 2'd0 : (blk_wr + 2'd1);

    // 目标块选择：若 `blk_nxt_w` 正被读侧占用，则跳过它、改用下一个空块。
    //   若此时整帧丢弃，读侧要等 blk_wr 变化才会换块 -> 双方互等形成死锁。
    wire [1:0]        blk_nxt2_w = (blk_nxt_w == 2'd3) ? 2'd0 : (blk_nxt_w + 2'd1);
    wire [1:0]        blk_tgt_w  = (blk_nxt_w == blk_rd) ? blk_nxt2_w : blk_nxt_w;
    wire              blk_conf   = (blk_tgt_w == blk_rd);   // 连下个空块也被占（4 块时不可能）

    // FIFO 取数：正常写突发时随 wr_data_req 逐字弹出；丢弃态【连续】排空。
    //   丢弃态若仍用 “>=BURST_WORDS” 门槛，一个 VS 周期内 FIFO 会被写满，
    //   真正发生丢像素并污染 fifo_ovf_cnt，故此处放宽为“非空即排空”。
    wire fifo_have     = (pack_usedw >= BURST_WORDS[FIFO_AW:0]);
    wire fifo_have_any = (pack_usedw != {(FIFO_AW+1){1'b0}});

    // 与 FSM 中“发起突发”的条件一致（单拍）
    wire start_burst = (st == S_WRITE) & ~burst_busy & fifo_have;

    // 突发内节拍计数：用于识别【最后一拍】
    reg [4:0] beat_cnt;
    always @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n)                                  beat_cnt <= 5'd0;
        else if (start_burst)                             beat_cnt <= 5'd0;
        else if ((st == S_WRITE) & burst_busy & wr_data_req) beat_cnt <= beat_cnt + 5'd1;
    end
    wire last_beat = (beat_cnt == BURST_WORDS[4:0] - 5'd1);

    assign fifo_pop = start_burst |
                      ((st == S_WRITE) & burst_busy & wr_data_req & ~last_beat) |
                      ((st == S_DRAIN) & fifo_have_any) |
                      ((st == S_FLUSH) & fifo_have_any);

    always @(posedge core_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
            st         <= S_IDLE;
            blk_wr     <= 2'd0;
            blk_wr_nxt <= 2'd0;
            blk_off    <= {ADDR_W{1'b0}};
            issue      <= 1'b0;
            burst_busy <= 1'b0;
            b_len      <= {LEN_W{1'b0}};
            b_addr     <= {ADDR_W{1'b0}};
            ovf_cnt_r  <= 32'd0;
            err_cnt_r  <= 32'd0;
            fifo_ovf_r <= 32'd0;
            first_r    <= 1'b0;
            discard    <= 1'b0;
        end else begin
            if (pack_ovf_pulse) fifo_ovf_r <= fifo_ovf_r + 32'd1;   // 像素域丢字事件

            case (st)
                //---------------------------------------------
                S_IDLE: begin
                    issue      <= 1'b0;
                    burst_busy <= 1'b0;
                    blk_off    <= {ADDR_W{1'b0}};

                    if (fs_new) begin
                        // 必须用【组合计算的目标块】blk_tgt_w 做比较：碰到正在被读的
                        // 块时跳到下一个空块，而不是整帧丢弃。
                        blk_wr_nxt <= blk_tgt_w;
                        if (blk_conf) begin
                            // 真无空块可写才会走到这里（丢掉这一帧）
                            ovf_cnt_r <= ovf_cnt_r + 32'd1;
                            discard <= 1'b1;
                            st      <= S_DRAIN;
                        end else begin
                            discard <= 1'b0;
                            // 上电首帧：先排空 VS 之前的残留像素，使 FIFO 水位从 0 起步
                            st <= first_r ? S_WRITE : S_FLUSH;
                        end
                    end
                end
                //---------------------------------------------
                S_FLUSH: begin
                    // 上电首帧专用：排空 FIFO 里的残留（属 VS 之前的无效数据），
                    // 排空后立即开始写本帧，保证块内数据与帧边界严格对齐
                    issue      <= 1'b0;
                    burst_busy <= 1'b0;
                    if (!fifo_have_any) st <= S_WRITE;
                end
                //---------------------------------------------
                S_WRITE: begin
                    if (fs_new) begin
                        // 帧未写满就来了新边界 -> 源帧异常，丢块
                        err_cnt_r <= err_cnt_r + 32'd1;
                        discard <= 1'b1;
                        st      <= S_DRAIN;
                        issue   <= 1'b0;
                    end else if (!burst_busy) begin
                        if (fifo_have) begin
                            b_len      <= BURST_WORDS[LEN_W-1:0];
                            b_addr     <= blk_base + blk_off;
                            issue      <= 1'b1;
                            burst_busy <= 1'b1;
                        end
                    end else begin
                        if (wr_finish) begin
                            issue      <= 1'b0;
                            burst_busy <= 1'b0;
                            blk_off    <= blk_off + BURST_WORDS[ADDR_W-1:0];
                            if (blk_off + BURST_WORDS[ADDR_W-1:0] >= BLK_WORDS[ADDR_W-1:0]) begin
                                blk_wr  <= blk_wr_nxt;      // 本块完成
                                first_r <= 1'b1;
                                st      <= S_IDLE;
                            end
                        end
                    end
                end
                //---------------------------------------------
                S_DRAIN: begin
                    issue      <= 1'b0;
                    burst_busy <= 1'b0;
                    // 帧边界 + FIFO 已排空时，就地选好下一块并直接进 S_WRITE。
                    // ★关键：帧边界脉冲已在本状态被消耗，若此时回到 S_IDLE，就必须再等
                    //   【下一个】帧边界，等于每次丢帧要付出 2 帧的代价（期间打包 FIFO
                    //   会灌满溢出，并造成块内容与帧边界错位）。
                    if (fs_new && !fifo_have_any) begin
                        blk_wr_nxt <= blk_tgt_w;
                        blk_off    <= {ADDR_W{1'b0}};
                        if (blk_conf) begin
                            ovf_cnt_r <= ovf_cnt_r + 32'd1;   // 真无空块 -> 继续丢
                            discard   <= 1'b1;
                        end else begin
                            discard <= 1'b0;
                            st      <= S_WRITE;
                        end
                    end
                end
                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
