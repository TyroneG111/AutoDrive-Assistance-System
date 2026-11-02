`timescale 1ns/1ps
//=============================================================================
// rd_line_gen —— 光栅序“按显示行”读出器（像素域 37.125MHz，五种读出模式）
//
// 与旧版 mosaic_rd_gen 的区别：
//   · 时钟域由 core_clk 改为 hdmi_pclk(像素域)，与 DMT 时序/行缓存同域
//   · 触发由“tx_framer 的 req_tog + 水位”改为“行缓存的 line_ok 电平”：
//     行缓存只要还有空页就允许读出器写下一显示行（最多领先 3 行）
//   · 不再按“整帧写入 FIFO”工作，而是每行产出 ROW_WORDS 个 256bit 字（= 1280 像素）
//
// 每显示行产出（统一 ROW_WORDS=160 个 256bit 字 = 1280 像素，每字 8 像素 × 32bit/24bit）：
//   单路模式：1 个 job = ROW_BURSTS 个突发（直通）
//   拼接模式：4 个 job（sub 0..3），仅 sub1/sub3（row_b=1）经 2x2 缩放器输出，
//             共 80 + 80 = 160 个字；sub0/sub2 只写缩放器内部行缓存
//
// 【2026-10 取消复用】原设计每源帧显示两遍（60Hz 读出->30fps 等效），
//   现输出直接 720p30：每个显示帧都锁存最新块，不再有 frame_parity 复用。
//
// 帧级策略（严格无撕裂 + 继不黑屏）：
//   【2026-10】取消复用：每帧都锁存新块；帧起点若没有被选通道的完整新块
//   -> 保持旧块重复显示（不是黑屏）
//   · 模式 mode_lat 只在帧起点锁存，整帧不变
//
// 读握手（配合 rd_bridge）：
//   · 最多 IFLIGHT 个突发在途（请求 FIFO + 数据 FIFO 解耦）
//   · rd_vld 为 FWFT：为高时 rd_data 即当前字；rd_en 取走该字
//=============================================================================
module rd_line_gen #(
    parameter NCH        = 4,
    parameter ADDR_W     = 25,
    parameter LEN_W      = 10,
    parameter BURST_W    = 16,
    parameter ROW_WORDS  = 160,
    parameter ROW_BURSTS = 10,
    parameter V_ACTIVE   = 720,
    parameter BLK_ALIGN  = 17,
    parameter DW         = 256,
    parameter IFLIGHT    = 4
)(
    input  wire             pclk,
    input  wire             rst_n,
    // ---- 启动（被选通道首块就绪）----
    input  wire             start,
    // ---- 行缓存握手 ----
    input  wire             line_ok,        // 电平：行缓存有空页，可写下一显示行
    output reg              wr_vld,
    output reg  [DW-1:0]    wr_word,
    // ---- 读桥 ----
    output reg              req_vld,
    output reg  [ADDR_W-1:0] req_addr,
    output reg  [LEN_W-1:0]  req_len,
    input  wire             req_rdy,
    output wire             rd_en,          // 【组合】与 rd_vld 同拍，否则 FWFT FIFO 会每字计两次
    input  wire             rd_vld,
    input  wire [DW-1:0]    rd_data,
    // ---- 写侧块状态（已同步到本域，帧级稳定）----
    input  wire [2*NCH-1:0] ch_blk_wr,
    input  wire [NCH-1:0]   ch_rdy,
    output reg  [2*NCH-1:0] ch_blk_rd,
    // ---- 模式（已在顶层同步 + 独热校验）----
    input  wire [4:0]       mode,
    // ---- 本显示帧实际生效的模式（帧起点锁存，整帧不变）----
    output wire [4:0]       o_mode,
    // ---- 状态 ----
    output reg  [31:0]      skip_cnt,
    output reg  [31:0]      stall_cnt,
    output wire [9:0]       rd_line_dbg,
    output reg  [31:0]      line_cnt,
    output wire             line_end        // 本显示行写完（1 拍，比最后一字晚 4 拍）
);
    localparam VHALF    = V_ACTIVE/2;
    localparam NJB_MOS  = 4*ROW_BURSTS;         // 拼接：每行突发数 = 40
    localparam NJB_ONE  = ROW_BURSTS;           // 单路：每行突发数 = 10

    reg [9:0]  rd_line;             // 当前读出的显示行 0..V_ACTIVE-1
    reg [4:0]  mode_lat;
    reg [5:0]  njb_r;               // 本行突发总数（拼接 40 / 单路 10）

    // 对外输出本显示帧实际生效的模式：
    //   mode_lat 只在“新一显示帧的第一行启动”时锁存 mode（见下方主流水），
    //   所以 o_mode 随当前输出帧变化，且一帧之内恒定。
    assign o_mode = mode_lat;

    reg [1:0]  ijob,  cjob;         // 下发 / 消费 job 指针
    reg [3:0]  iburst, cburst;      // 下发 / 消费 burst 指针（0..ROW_BURSTS-1=9）
    reg [2:0]  inflight;
    reg [5:0]  irem, crem;          // 剩余待下发 / 待消费突发数
    reg [4:0]  beat;                // 当前突发内已消费字数 0..BURST_W-1
    reg        active;              // 本行正在执行
    reg [3:0]  le_sr;               // line_end 延时链（要盖住缩放器尾拍）

    wire       sc_job_start;        // 【组合】与 job 首字同拍；寄存器化会晚一拍 -> 缩放器字对奇偶错位

    //-------------------------------------------------------------------------
    // 模式解码 / 地址生成（使用 ijob 指针）
    //-------------------------------------------------------------------------
    wire        mode_mos = mode_lat[0];
    wire [3:0]  ch_en    = mode_mos ? 4'b1111 : mode_lat[4:1];
    wire [1:0]  sel_one  = mode_lat[2] ? 2'd1 :
                           mode_lat[3] ? 2'd2 :
                           mode_lat[4] ? 2'd3 : 2'd0;
    wire        rdy_all  = &(ch_rdy | ~ch_en);

    wire        ph_i  = (rd_line >= VHALF[9:0]);
    wire [9:0]  rp_i  = ph_i ? (rd_line - VHALF[9:0]) : rd_line;
    wire [1:0]  jch_i = mode_mos ? {ph_i, ijob[1]} : sel_one;
    wire [9:0]  jrow_i= mode_mos ? ({rp_i[8:0], 1'b0} + {9'd0, ijob[0]}) : rd_line;

    wire [ADDR_W-1:0] base_i = {jch_i, ch_blk_rd[jch_i*2 +: 2], {BLK_ALIGN{1'b0}}};
    wire [ADDR_W-1:0] addr_i = base_i + (jrow_i * ROW_WORDS[ADDR_W-1:0])
                                      + (iburst * BURST_W[ADDR_W-1:0]);

    // 消费侧 row_b（拼接：cjob[0] = 0 为行A，1 为行B）
    wire        crow_b = cjob[0];

    //-------------------------------------------------------------------------
    // 缩放器（拼接模式专用）
    //-------------------------------------------------------------------------
    wire          sc_word_vld = rd_vld & active & mode_mos;
    wire          sc_out_vld;
    wire [DW-1:0] sc_out_data;

    scale_2x2_avg #(
        .DW        (DW),
        .ROW_WORDS (ROW_WORDS),
        .LB_DEPTH  (128)
    ) u_scale (
        .clk       (pclk),
        .rst_n     (rst_n),
        .word_vld  (sc_word_vld),
        .word      (rd_data),
        .row_b     (crow_b),
        .job_start (sc_job_start),
        .out_vld   (sc_out_vld),
        .out_data  (sc_out_data)
    );

    //-------------------------------------------------------------------------
    // 主流水
    //-------------------------------------------------------------------------
    wire [5:0] njb_nxt = (rd_line == 10'd0) ?
                         (mode[0] ? NJB_MOS[5:0] : NJB_ONE[5:0]) : njb_r;

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            rd_line      <= 10'd0;
            mode_lat     <= 5'b00001;
            njb_r        <= NJB_MOS[5:0];
            ijob <= 2'd0; iburst <= 4'd0;
            cjob <= 2'd0; cburst <= 4'd0;
            inflight <= 3'd0; irem <= 6'd0; crem <= 6'd0;
            beat  <= 5'd0;
            active <= 1'b0;
            req_vld <= 1'b0; req_addr <= {ADDR_W{1'b0}}; req_len <= BURST_W[LEN_W-1:0];
            wr_vld <= 1'b0; wr_word <= {DW{1'b0}};
            ch_blk_rd <= {(2*NCH){1'b0}};
            skip_cnt <= 32'd0; stall_cnt <= 32'd0;
            line_cnt <= 32'd0;
            le_sr <= 4'd0;
        end else begin
            req_vld      <= 1'b0;

            //-----------------------------------------------------------------
            // 行启动（行缓存有空页才允许）
            //-----------------------------------------------------------------
            if (!active) begin
                if (start && line_ok) begin
                    if (rd_line == 10'd0) begin
                        mode_lat  <= mode;
                        njb_r     <= mode[0] ? NJB_MOS[5:0] : NJB_ONE[5:0];
                        // 【2026-10 取消复用】每个显示帧都锁存最新块（原为每两帧一次）
                        if (rdy_all) ch_blk_rd <= ch_blk_wr;
                        else         skip_cnt  <= skip_cnt + 32'd1;
                    end
                    ijob     <= 2'd0; iburst <= 4'd0;
                    cjob     <= 2'd0; cburst <= 4'd0;
                    inflight <= 3'd0;
                    irem     <= njb_nxt; crem <= njb_nxt;
                    beat     <= 5'd0;
                    active   <= 1'b1;
                end
            end

            //-----------------------------------------------------------------
            // 下发突发请求（最多 IFLIGHT 个在途）
            //-----------------------------------------------------------------
            if (active && (irem != 6'd0) && (inflight < IFLIGHT[2:0])) begin
                if (req_rdy) begin
                    req_addr <= addr_i;
                    req_len  <= BURST_W[LEN_W-1:0];
                    req_vld  <= 1'b1;
                    irem     <= irem - 6'd1;
                    inflight <= inflight + 3'd1;
                    if (iburst == ROW_BURSTS[3:0] - 4'd1) begin
                        iburst <= 4'd0;
                        ijob   <= ijob + 2'd1;
                    end else begin
                        iburst <= iburst + 4'd1;
                    end
                end else begin
                    stall_cnt <= stall_cnt + 32'd1;
                end
            end

            //-----------------------------------------------------------------
            // 消费读回数据
            //-----------------------------------------------------------------
            if (active && rd_vld) begin
                // job 首字 -> 通知缩放器清行内计数（组合，与首字同拍）

                if (beat == BURST_W[4:0] - 5'd1) begin
                    beat     <= 5'd0;
                    inflight <= inflight - 3'd1;
                    crem     <= crem - 6'd1;
                    if (crem == 6'd1) begin
                        // 本显示行结束
                        active   <= 1'b0;
                        line_cnt <= line_cnt + 32'd1;
                        if (rd_line == V_ACTIVE[9:0] - 10'd1) rd_line <= 10'd0;
                        else                                  rd_line <= rd_line + 10'd1;
                    end else begin
                        if (cburst == ROW_BURSTS[3:0] - 4'd1) begin
                            cburst <= 4'd0;
                            cjob   <= cjob + 2'd1;
                        end else begin
                            cburst <= cburst + 4'd1;
                        end
                    end
                end else begin
                    beat <= beat + 5'd1;
                end
            end

            //-----------------------------------------------------------------
            // 像素字输出（拼接经缩放器，单路直通）
            //   注意：拼接模式【不能】用 active 门控，缩放器最后 2 拍输出
            //   会落在 active 拉低之后，门控掉会少字导致该页永远不 ready
            //-----------------------------------------------------------------
            wr_word <= mode_mos ? sc_out_data : rd_data;
            wr_vld  <= mode_mos ? sc_out_vld  : (rd_vld & active);

            // 本行最后一个像素字被消费 -> 延时 4 拍作为行结束标志。
            //   必须晚于最后一个 wr_vld（单路 +2、拼接经缩放器 +3），否则行缓存会早翻页。
            le_sr <= {le_sr[2:0], (active & rd_vld &
                                   (beat == BURST_W[4:0] - 5'd1) & (crem == 5'd1))};
        end
    end

    assign line_end = le_sr[3];

    assign rd_line_dbg = rd_line;

    // job 首字标志（与首字同拍组合给出）
    assign sc_job_start = active & rd_vld & (cburst == 3'd0) & (beat == 5'd0);

    // 【关键】读桥的数据 FIFO 是 FWFT（rd_data 要等 pop 之后才换头），
    //   因此 rd_en 必须与「计数/消费」同拍组合给出；若寄存器化会晚一拍，
    //   导致每个字被计数两次、一行只读到一半就“完成”，缩放器也会把同一个字当作一对
    assign rd_en = active & rd_vld;

endmodule
