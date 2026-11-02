`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: video_udp_ctrl
// Description: 视频流及用户数据控制模块
//   [写侧 - vid_pclk 域] 把 HDMI(RGB888) 视频按行写入双口 DRAM 行缓存
//       每个行缓存块 = 2048 words, 其中 0..1279 存 1280 个像素(RGB888),
//       地址 2047 存该行的元信息 {frame_code[3:0], line[9:0]}
//   [读侧 - clk(125M) 域]  从 DRAM 读出行数据, 按 UDP 报文打包发给 udp_stack:
//       每行 3840 字节图像 = 3 个报文, 每报文 = 4 字节自定义头 + 1280 字节图像
//       4 字节头后 16bit = { 4bit帧同步码[15:12], 2bit行内分段号[11:10], 10bit行号[9:0] }
//   [用户数据] 16bit 输入(电平常有效). 在每个报文开始时采样寄存,
//       并"写到下一次发送的报文中"(即本包用上一包开始时采样的值).
//   [双目融合图] 独立成包, 绝不与原图包混装:
//       融合行 670 px(RGB565) = 1340 B, 加 4 B 自定义头 = 1344 B 载荷 / 一包一行.
//       16bit 帧尾 = { 4bit帧同步码[15:12], 2bit段号=2'b00[11:10], 10bit行号[9:0] },
//       16bit 用户数据恒为 16'hFC00 (原图为 {6'b0,frame_mode[4:0],staus[4:0]}),
//       接收端靠 user 字段即可区分两类包.
//       插入时机: 每个视频行 3 包发完后、趁该行剩余 ~1500 拍空隙发一包;
//       每个视频行最多插一包, 场消隐期(无行可发 >8000 拍)允许连发补发.
//   [RX] 收到的用户数据直接透传到顶层输出.
//
// 视频时序(VESA DMT 1280x720p@30): pclk=37.125MHz, HS/VS 正极性
//   行 1280 有效 + 110 前肩 + 40 同步 + 220 后肩 = 1650
//   场 720 有效 + 5 前肩 + 5 同步 + 20 后肩 = 750
//////////////////////////////////////////////////////////////////////////////////

module video_udp_ctrl #(
    parameter FRAME_CODE_MOD = 16,         // 帧同步码循环模值(=16 -> 0~15), 相邻两帧不同
    parameter BYTES_PER_PKT  = 16'd1280,   // 每包图像字节数
    parameter PKTS_PER_LINE  = 2'd3,       // 每行报文数
    parameter PAYLOAD_LEN    = 16'd1284,   // 每包用户数据长度 = 4 + 1280
    // ---------- 双目融合图像上传 ----------
    parameter FUSE_W         = 11'd670,    // 融合图像宽(像素)
    parameter FUSE_PAYLOAD   = 16'd1344,   // 融合包用户数据长度 = 4 + 670*2(RGB565)
    parameter FUSE_STRIDE    = 11'd672,    // 融合行在 RAM 中的槽间距(> 670)
    // ---------- 帧结束判定（时延统计用） ----------
    parameter V_ACTIVE       = 10'd720     // 一帧有效行数(行号 0..719)
)(
    //----------- 视频输入 (vid_pclk 域) -----------
    input  wire        vid_pclk,           // 像素时钟 37.125MHz
    input  wire        vid_rstn,           // 视频域复位, 低有效
    input  wire        vid_hs,             // 行同步(正极性) —— 本模块用 de/hc 判定, hs 保留
    input  wire        vid_vs,             // 场同步(正极性)
    input  wire        vid_de,             // 数据有效, 高有效
    input  wire [10:0] vid_hc,             // 行内像素号 0..1279
    input  wire [9:0]  vid_vc,             // 行号 0..719
    input  wire [23:0] vid_rgb,            // RGB888 {R[7:0],G[7:0],B[7:0]}

    //----------- 双目融合图像输入 (fuse_clk = 74.25MHz 域) -----------
    //   stereo_fusion_top 的输出: 每像素在 pix_en 处有效, out_valid 连续高 4 拍,
    //   故用 "hc 变化" 提取像素脉冲(与 pix_en 等效, 且无需额外端口)
    input  wire        fuse_clk,           // 74.25MHz (与 video_top 同源)
    input  wire        fuse_rstn,          // 融合域复位, 低有效
    input  wire [15:0] fuse_px,            // RGB565
    input  wire        fuse_de,            // 像素有效
    input  wire [10:0] fuse_hc,            // 融合画布列号 0..669
    input  wire [9:0]  fuse_vc,            // 融合行号 0..291

    //----------- 网络侧 (clk = rgmii_clk 125MHz) -----------
    input  wire        clk,
    input  wire        rstn,               // 网络域复位, 低有效

    //----------- TX 应用接口 (pull, 接 udp_stack) -----------
    output wire [15:0] app_data_length,    // = PAYLOAD_LEN(原图) / FUSE_PAYLOAD(融合)
    output wire        app_data_request,   // 发送请求电平
    input  wire        app_rd_en,          // 载荷字节读脉冲
    output wire [7:0]  app_data_in,        // 当前载荷字节

    //----------- 用户自定义数据 -----------
    input  wire [15:0] user_data_in,       // 16bit, 电平常有效

    //----------- RX 用户数据输出 -----------
    input  wire [7:0]  udp_rec_rdata,
    input  wire [15:0] udp_rec_data_length,
    input  wire        udp_rec_data_valid,
    output wire        rx_data_valid,
    output wire [7:0]  rx_data,
    output wire [15:0] rx_data_length,

    //----------- DRAM 接口 (真双口 24bit x 8192) -----------
    //   写口时钟 = vid_pclk, 读口时钟 = clk (由顶层 IP 决定)
    output wire        dram_wr_en,
    output wire [12:0] dram_wr_addr,
    output wire [23:0] dram_wr_data,
    output wire [12:0] dram_rd_addr,
    input  wire [23:0] dram_rd_data,

    //----------- 时延统计：一帧发送完成事件 (clk 域) -----------
    //   每发完一帧(最后一行最后一个报文的最后一个字节)翻转一次电平。
    //   用 toggle 而不是脉冲，以便安全跨到低频时钟域(脉冲只有 8ns 会丢)。
    output reg         frame_done_tgl
);

    //================================================================
    // 函数
    //================================================================
    function [3:0] bin2gray;
        input [3:0] b;
        begin bin2gray = b ^ (b >> 1); end
    endfunction

    function [3:0] gray2bin;
        input [3:0] g;
        begin
            gray2bin[3] = g[3];
            gray2bin[2] = g[2] ^ gray2bin[3];
            gray2bin[1] = g[1] ^ gray2bin[2];
            gray2bin[0] = g[0] ^ gray2bin[1];
        end
    endfunction

    //================================================================
    // 写侧 (vid_pclk 域)
    //================================================================
    reg  vid_de_d, vid_vs_d;
    wire de_rise =  vid_de & ~vid_de_d;
    wire de_fall = ~vid_de &  vid_de_d;
    wire vs_rise =  vid_vs & ~vid_vs_d;

    reg [3:0] frame_code;
    always @(posedge vid_pclk) begin
        if(!vid_rstn) begin
            vid_de_d   <= 1'b0;
            vid_vs_d   <= 1'b0;
            frame_code <= 4'd0;
        end else begin
            vid_de_d <= vid_de;
            vid_vs_d <= vid_vs;
            if(vs_rise)
                frame_code <= (frame_code == FRAME_CODE_MOD - 4'd1) ? 4'd0 : (frame_code + 4'd1);
        end
    end

    // 行开始时锁存行号(10bit, 0~719)
    reg [9:0] line_vc;
    always @(posedge vid_pclk) begin
        if(!vid_rstn)    line_vc <= 10'd0;
        else if(de_rise) line_vc <= vid_vc;
    end

    // 行缓冲管理
    reg [1:0] wr_buf;
    reg [3:0] wr_line_bin;
    reg [3:0] wr_line_gray;
    always @(posedge vid_pclk) begin
        if(!vid_rstn) begin
            wr_buf       <= 2'd0;
            wr_line_bin  <= 4'd0;
            wr_line_gray <= 4'd0;
        end else begin
            if(de_fall) begin
                wr_buf       <= wr_buf + 2'd1;
                wr_line_bin  <= wr_line_bin + 4'd1;
                wr_line_gray <= bin2gray(wr_line_bin + 4'd1);
            end
        end
    end

    // 行末(de_fall)把该行元信息写到刚写完的缓冲块 {wr_buf,2047}
    wire meta_wr = de_fall;

    // DRAM 写口: 有效像素写 {wr_buf, hc}; 行末(de_fall)写元信息到 {wr_buf,2047}
    //   元信息(低 14bit) = {frame_code[3:0], line[9:0]}; 行内分段号由读侧逐包填入
    assign dram_wr_en   = vid_de | meta_wr;
    assign dram_wr_addr = meta_wr ? {wr_buf, 11'd2047} : {wr_buf, vid_hc};
    assign dram_wr_data = meta_wr ? {10'd0, frame_code, line_vc} : vid_rgb;

    //================================================================
    // 行完成计数同步 (vid_pclk -> clk), 格雷码 + 两级同步
    //================================================================
    reg [3:0] wr_gray_s1, wr_gray_s2;
    always @(posedge clk) begin
        if(!rstn) begin
            wr_gray_s1 <= 4'd0;
            wr_gray_s2 <= 4'd0;
        end else begin
            wr_gray_s1 <= wr_line_gray;
            wr_gray_s2 <= wr_gray_s1;
        end
    end
    wire [3:0] wr_bin_s = gray2bin(wr_gray_s2);

    //================================================================
    // 读侧 (clk 域)
    //================================================================
    reg  [1:0]  rd_buf;
    reg [3:0]  rd_line_cnt;               // 已完整读出的行数
    wire        line_avail = (wr_bin_s != rd_line_cnt);

    localparam RS_IDLE  = 3'd0;
    localparam RS_META  = 3'd1;            // 读行元信息
    localparam RS_PREP  = 3'd2;            // 锁存元信息/首部, 准备发送
    localparam RS_SEND  = 3'd3;            // 发送中
    localparam RS_FPREP = 3'd4;            // 锁存融合行号/首部, 准备发送
    localparam RS_FSEND = 3'd5;            // 融合包发送中

    reg  [2:0]  rstate;
    reg  [1:0]  pkt;                       // 行内包号 0..2
    reg  [11:0] pkt_byte;                  // 包内字节 0..1283
    reg  [10:0] pix_addr;                  // 行内像素地址 0..1279
    reg  [1:0]  byte_sel;                  // 像素内字节 0..2 (0:R,1:G,2:B)
    reg         sending;
    reg  [13:0] meta_rd;                   // {frame_code[3:0],line[9:0]}
    reg  [31:0] hdr_reg;                   // {user[15:0],frame[3:0],pkt[1:0],line[9:0]}
    reg  [15:0] user_q;                    // 用户数据待用寄存器
    // 报尾 16bit = {frame_code[3:0], pkt[1:0], line[9:0]}
    wire [1:0]  pkt_next   = (pkt == PKTS_PER_LINE - 2'd1) ? 2'd0 : (pkt + 2'd1);
    wire [15:0] meta_first = {dram_rd_data[13:10], 2'd0,      dram_rd_data[9:0]}; // 本行首包(pkt=0)
    wire [15:0] meta_bound = {meta_rd[13:10],       pkt_next, meta_rd[9:0]};       // 后续报文的包

    //================================================================
    // 双目融合行缓冲 (三槽轮转: 融合域写 / 网络域读)
    //   RAM: 1 片 ram_2048x16bit 真双口 (3 x 672 = 2016 < 2048)
    //   ★ 为什么用 3 槽而不是 2 槽 (2026-10 修正 "每帧丢一包"):
    //     融合行 N 的行首时刻，与"融合包 N-1 发完"的时刻几乎重合
    //     (实测相差仅 ~100ns)，加上读指针跨时钟域同步的 ~30ns，
    //     2 槽时 fuse_full = (差>=2) 会在此刻误判满 -> 丢掉整行。
    //     丢掉一行还会使 fwr_slot 与 frd_slot 错开，导致后半帧所有行
    //     取错槽(行号与数据整体错位)。改成 3 槽后：
    //       · 允许 2 行在途 (fuse_full 阈值 = 3)，吸收上述相位余量
    //       · 槽号用独立的 0->1->2 循环计数器，不再依赖 fwr_row[0]，
    //         与行计数解耦，即使偶发丢行也不会把整体对应关系打乱
    //   行号由 fuse_slot_line[] 在整行写入期间保持稳定(远早于行指针翻转),
    //   属于双时钟 FIFO 的"数据先稳、指针后同步"手法
    //================================================================
    reg  [10:0] fuse_hc_d;                 // 融合域: 上一拍列号(取像素脉冲用)
    reg  [3:0]  fwr_row;                   // 融合域: 已写入行数(mod 16, 仅用于水位比较)
    reg  [3:0]  fwr_row_gray;
    reg  [1:0]  fwr_slot;                  // 融合域: 当前写槽 0/1/2 循环
    reg         fwr_skip;                  // 融合域: 本行丢弃(缓冲已满)
    reg  [9:0]  fuse_slot_line [0:2];      // 各槽对应的融合行号(0..291)
    reg  [3:0]  frd_gray_f1, frd_gray_f2;  // 读指针 -> 融合域 同步
    reg  [3:0]  fwr_gray_c1, fwr_gray_c2;  // 写指针 -> 网络域 同步

    wire [3:0]  frd_row_f  = gray2bin(frd_gray_f2);   // 融合域看到的已读行数
    wire [3:0]  fwr_row_c  = gray2bin(fwr_gray_c2);   // 网络域看到的已写行数
    wire        fuse_full  = ((fwr_row - frd_row_f) >= 4'd3);   // 3 槽都已待发
    wire        fuse_row_start = fuse_de && (fuse_hc != fuse_hc_d) && (fuse_hc == 11'd0);
    wire        fuse_px_we = fuse_de && (fuse_hc != fuse_hc_d) && (fuse_hc < FUSE_W);
    wire        skip_now   = fuse_row_start ? fuse_full : fwr_skip;  // 行首当拍即判定, 避免首像素漏写
    wire        fuse_wr_en = fuse_px_we && !skip_now;

    wire [1:0]  fwr_slot_nxt = (fwr_slot == 2'd2) ? 2'd0 : (fwr_slot + 2'd1);
    wire [10:0] fwr_slot_off = (fwr_slot == 2'd0) ? 11'd0 :
                               (fwr_slot == 2'd1) ? FUSE_STRIDE : (FUSE_STRIDE << 1);
    wire [10:0] fuse_wr_addr = fwr_slot_off + fuse_hc;

    reg  [3:0]  frd_row;                   // 网络域: 已发送行数(mod 16)
    reg  [3:0]  frd_row_gray;
    reg  [1:0]  frd_slot;                  // 网络域: 当前读槽 0/1/2 循环
    wire [1:0]  frd_slot_nxt = (frd_slot == 2'd2) ? 2'd0 : (frd_slot + 2'd1);
    wire [10:0] frd_slot_off = (frd_slot == 2'd0) ? 11'd0 :
                               (frd_slot == 2'd1) ? FUSE_STRIDE : (FUSE_STRIDE << 1);
    wire [10:0] fuse_base   = frd_slot_off;
    wire        fuse_pending = (fwr_row_c != frd_row);

    reg         fuse_mode;                 // 当前发送的是融合包
    reg  [10:0] fbit;                      // 融合包内字节计数 0..1343
    reg  [9:0]  fcol;                      // 融合像素列 0..669
    reg         fsel;                      // 0:高字节 1:低字节
    reg  [9:0]  fuse_line_q;               // 本包融合行号
    reg  [3:0]  fuse_code_q;               // 本包帧同步码
    reg         fuse_done;                 // 本视频行已插过一包
    reg  [15:0] idle_cnt;                  // "无行可发"持续拍数(判消隐期)
    wire        vbi_mode = (idle_cnt > 16'd8000);
    wire        fuse_budget_ok = (!fuse_done) || vbi_mode;

    wire        fuse_consuming = app_rd_en && (fbit >= 11'd4);
    wire [10:0] fcol_pref = {1'b0, fcol} + ((fuse_consuming && fsel) ? 11'd1 : 11'd0);
    wire [10:0] fuse_rd_addr = fuse_base + fcol_pref;
    wire [15:0] fuse_rd_data;
    // 融合包 4 字节头: {user=16'hFC00}, {4bit帧同步码, 2'b00, 10bit行号}
    wire [31:0] fuse_hdr      = {16'hFC00, fuse_code_q, 2'b00, fuse_line_q};
    wire [7:0]  fuse_hdr_byte = fuse_hdr[(3 - fbit[1:0]) * 8 +: 8];
    wire [7:0]  fuse_pix_byte = fsel ? fuse_rd_data[7:0] : fuse_rd_data[15:8];

    always @(posedge clk) begin
        if(!rstn) begin
            rstate     <= RS_IDLE;
            sending    <= 1'b0;
            pkt        <= 2'd0;
            pkt_byte   <= 12'd0;
            pix_addr   <= 11'd0;
            byte_sel   <= 2'd0;
            rd_buf     <= 2'd0;
            rd_line_cnt<= 4'd0;
            meta_rd    <= 14'd0;
            hdr_reg    <= 32'd0;
            user_q     <= 16'd0;
            fuse_mode  <= 1'b0;
            fbit       <= 11'd0;
            fcol       <= 10'd0;
            fsel       <= 1'b0;
            fuse_line_q<= 10'd0;
            fuse_code_q<= 4'd0;
            fuse_done  <= 1'b0;
            frd_row    <= 4'd0;
            frd_row_gray <= 4'd0;
            frd_slot   <= 2'd0;
        end else begin
            case(rstate)
            RS_IDLE: begin
                sending <= 1'b0;
                // 原图行优先; 空隙里(且预算允许)插入一包融合图
                if(line_avail)
                    rstate <= RS_META;
                else if(fuse_pending && fuse_budget_ok)
                    rstate <= RS_FPREP;
            end
            RS_META: begin
                fuse_done <= 1'b0;      // 新的视频行 -> 重置本行融合插包预算
                rstate <= RS_PREP;      // 读口地址已切到 {rd_buf,2047}
            end
            RS_PREP: begin
                meta_rd  <= dram_rd_data[13:0];                  // {frame_code[3:0],line[9:0]}
                hdr_reg  <= {user_data_in, meta_first};          // 本行首包(pkt=0), 用当前用户数据
                user_q   <= user_data_in;
                pkt      <= 2'd0;
                pkt_byte <= 12'd0;
                pix_addr <= 11'd0;
                byte_sel <= 2'd0;
                fuse_mode<= 1'b0;
                sending  <= 1'b1;
                rstate   <= RS_SEND;
            end
            RS_SEND: begin
                if(app_rd_en) begin
                    if(pkt_byte < 12'd4) begin
                        // ---- 4 字节自定义首部 ----
                        pkt_byte <= pkt_byte + 1'b1;
                    end else begin
                        // ---- 图像字节: 推进像素指针 ----
                        if(byte_sel == 2'd2) begin
                            byte_sel <= 2'd0;
                            pix_addr <= pix_addr + 1'b1;
                        end else begin
                            byte_sel <= byte_sel + 1'b1;
                        end
                        if(pkt_byte == PAYLOAD_LEN - 1) begin
                            // ---- 包边界 ----
                            pkt_byte <= 12'd0;
                            hdr_reg  <= {user_q, meta_bound}; // 下一包首部(填入下一行内分段号)
                            user_q   <= user_data_in;        // 采样, 供"下一次发送的报文"
                            if(pkt == PKTS_PER_LINE - 1) begin
                                // ---- 一行结束 ----
                                pkt         <= 2'd0;
                                sending     <= 1'b0;
                                rd_buf      <= rd_buf + 1'b1;
                                rd_line_cnt <= rd_line_cnt + 1'b1;
                                rstate      <= RS_IDLE;
                            end else begin
                                pkt <= pkt + 1'b1;
                            end
                        end else begin
                            pkt_byte <= pkt_byte + 1'b1;
                        end
                    end
                end
            end
            //--------------------------------------------------------
            // 融合包: 4 字节头 + 670 像素 x 2 字节(RGB565), 一包正好一行
            //--------------------------------------------------------
            RS_FPREP: begin
                fuse_mode   <= 1'b1;
                fuse_line_q <= fuse_slot_line[frd_slot];     // 本槽对应的融合行号
                fuse_code_q <= meta_rd[13:10];              // 跟当前视频行的帧同步码
                fbit        <= 11'd0;
                fcol        <= 10'd0;
                fsel        <= 1'b0;
                sending     <= 1'b1;
                rstate      <= RS_FSEND;
            end
            RS_FSEND: begin
                if(app_rd_en) begin
                    if(fbit < 11'd4) begin
                        fbit <= fbit + 1'b1;                // 4 字节自定义首部
                    end else begin
                        if(fsel) begin
                            fsel <= 1'b0;
                            fcol <= fcol + 1'b1;            // 一像素(2字节)发完
                        end else begin
                            fsel <= 1'b1;
                        end
                        if(fbit == FUSE_PAYLOAD - 1) begin
                            fbit         <= 11'd0;
                            sending      <= 1'b0;
                            fuse_mode    <= 1'b0;
                            fuse_done    <= 1'b1;
                            frd_row      <= frd_row + 4'd1;
                            frd_row_gray <= bin2gray(frd_row + 4'd1);
                            frd_slot     <= frd_slot_nxt;
                            rstate       <= RS_IDLE;
                        end else begin
                            fbit <= fbit + 1'b1;
                        end
                    end
                end
            end
            default: rstate <= RS_IDLE;
            endcase
        end
    end

    //----------------------------------------------------------------
    // 时延统计: 一帧发送完成事件
    //   判定 = 发送中 + 读到载荷最后一个字节 + 本行最后一个报文 + 本行是最后一行
    //   meta_rd[9:0] 是 RS_PREP 锁存的行号, 整行期间有效
    //----------------------------------------------------------------
    wire line_last_byte  = (rstate == RS_SEND) && app_rd_en &&
                           (pkt_byte == PAYLOAD_LEN - 1'b1) &&
                           (pkt == PKTS_PER_LINE - 2'd1);
    wire frame_last_byte = line_last_byte && (meta_rd[9:0] == (V_ACTIVE - 10'd1));

    always @(posedge clk) begin
        if(!rstn)                    frame_done_tgl <= 1'b0;
        else if(frame_last_byte)     frame_done_tgl <= ~frame_done_tgl;
    end

    //----------------------------------------------------------------
    // 融合域: 行缓冲写侧 (每行 670 像素顺序写入当前槽)
    //----------------------------------------------------------------
    always @(posedge fuse_clk) begin
        if(!fuse_rstn) begin
            fuse_hc_d    <= 11'd0;
            fwr_row      <= 4'd0;
            fwr_row_gray <= 4'd0;
            fwr_slot     <= 2'd0;
            fwr_skip     <= 1'b0;
            frd_gray_f1  <= 4'd0;
            frd_gray_f2  <= 4'd0;
            fuse_slot_line[0] <= 10'd0;
            fuse_slot_line[1] <= 10'd0;
            fuse_slot_line[2] <= 10'd0;
        end else begin
            fuse_hc_d   <= fuse_hc;
            frd_gray_f1 <= frd_row_gray;
            frd_gray_f2 <= frd_gray_f1;
            if(fuse_row_start) begin
                fwr_skip <= fuse_full;                       // 行首判满
                if(!fuse_full) fuse_slot_line[fwr_slot] <= fuse_vc;
            end
            if(fuse_px_we && !fwr_skip && (fuse_hc == FUSE_W - 11'd1)) begin
                fwr_row      <= fwr_row + 4'd1;
                fwr_row_gray <= bin2gray(fwr_row + 4'd1);
                fwr_slot     <= fwr_slot_nxt;
            end
        end
    end

    // 融合行缓冲 (真双口: 写 = fuse_clk, 读 = clk), 读延迟 1 拍
    ram_2048x16bit u_fuse_dram (
        .wr_clk  (fuse_clk    ),
        .wr_rst  (1'b0        ),
        .wr_en   (fuse_wr_en  ),
        .wr_addr (fuse_wr_addr),
        .wr_data (fuse_px     ),
        .rd_clk  (clk         ),
        .rd_rst  (1'b0        ),
        .rd_addr (fuse_rd_addr),
        .rd_data (fuse_rd_data)
    );

    // 写指针(格雷码) -> 网络域
    always @(posedge clk) begin
        if(!rstn) begin
            fwr_gray_c1 <= 4'd0;
            fwr_gray_c2 <= 4'd0;
        end else begin
            fwr_gray_c1 <= fwr_row_gray;
            fwr_gray_c2 <= fwr_gray_c1;
        end
    end

    // 无行可发的持续拍数: >1 行(5556 拍) 则认定为场消隐期, 允许融合包连发补发
    always @(posedge clk) begin
        if(!rstn)                     idle_cnt <= 16'd0;
        else if(line_avail)           idle_cnt <= 16'd0;
        else if(idle_cnt != 16'hFFFF) idle_cnt <= idle_cnt + 1'b1;
    end

    // DRAM 读口地址: RS_META/RS_PREP 读元信息; 读像素时需提前给出下一像素地址
    // (块 RAM 地址打拍后数据再出一拍, 故 byte_sel==2 时预取 pix_addr+1)
    wire        pix_consuming = app_rd_en && (pkt_byte >= 12'd4);   // 仅在消费图像字节时
    wire [10:0] pix_pref = pix_addr + ((pix_consuming && (byte_sel == 2'd2)) ? 11'd1 : 11'd0);
    assign dram_rd_addr = (rstate == RS_META || rstate == RS_PREP)
                          ? {rd_buf, 11'd2047}
                          : {rd_buf, pix_pref};

    // 首部/图像字节选择
    wire [1:0] pb2      = pkt_byte[1:0];
    wire [7:0] hdr_byte = hdr_reg[(3 - pb2) * 8 +: 8];
    wire [7:0] pix_byte = dram_rd_data[(2 - byte_sel) * 8 +: 8];

    // 融合包 = 4 字节头 + 图像字节; 原图包 = 原逻辑
    wire [7:0] fuse_out_byte = (fbit < 11'd4) ? fuse_hdr_byte : fuse_pix_byte;

    assign app_data_in     = fuse_mode ? fuse_out_byte
                                       : ((pkt_byte < 12'd4) ? hdr_byte : pix_byte);
    assign app_data_length = fuse_mode ? FUSE_PAYLOAD : PAYLOAD_LEN;
    assign app_data_request= sending;

    //================================================================
    // RX 用户数据透传
    //================================================================
    assign rx_data_valid  = udp_rec_data_valid;
    assign rx_data        = udp_rec_rdata;
    assign rx_data_length = udp_rec_data_length;

endmodule
