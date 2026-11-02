`timescale 1ns/1ps
//=============================================================================
// dmt_pix_proc —— DMT 像素域图像处理级
//
// 【接口约定（严格遵守）】
//   · 时钟：hdmi_pclk（720p30 时为 37.125MHz）
//   · 每拍 1 个像素，24bit YUV444（Y=px[23:16], Cb=px[15:8], Cr=px[7:0]）
//   · 时序信号 de/hs/vs + 行内像素号 hc + 行号 vc 全部按 DMT 栅格走
//   · 本模块【恒定延迟】：out_* 与 in_* 波形形状不变，仅相位差固定若干拍
//     -> 显示侧读到的是“严格 DMT 时序下的 1 pixel/cycle 流”，
//        且因为处理级跟在行缓存之后，用的是显示自己的节拍，**零额外时间开销**
//
// 【要换算法，只改下面 “算法区” 即可】
//   · 2D 算法(3x3/5x5/中值/缩放)：在本级内自建 (N-1) 行行缓存即可 ——
//     hc 给出列号(0..H_ACT-1)、vc 给出行号，in_de 高即有效像素
//
// 【2026-10 YUV444 改造】输入即为 24bit YUV444，不再需要 YUV422->444 转换。
//   · 正常路径：24bit YUV444 -> CLAHE（Y 增强）-> yuv2rgb -> RGB565 -> RGB888
//   · 仿真旁路：+define+BYPASS_CLAHE -> 整级直通，out_px == in_px（逐像素相等）
//=============================================================================
module dmt_pix_proc #(
    parameter [8:0] GAIN = 9'd256,          // 9bit 增益，256 = x1.0（恒等）
                                            // ★注意: 当前算法区已换成 CLAHE，本参数未被使用
    // CLAHE 裁剪限幅：cliplimitx8 >> 3 = cliplimit。
    //   hist_ram_ctrl 里 阈值 = (cliplimitx8 * 225) >> 5；
    //   cliplimitx8 = 16 -> cliplimit = 2.0，阈值 = 112
    //   （与 hist_ram_ctrl 的复位默认值 16'd112 一致）。
    //   ★该值必须非 0，否则直方图所有 bin 被裁光 -> CDF = 0 -> 映射表全 0
    //    -> 输出全黑（de/hs/vs 仍正常透传）。
    parameter [4:0] CLIPLIMITX8 = 5'd16
)(
    input  wire         clk,                // hdmi_pclk (720p30 时为 37.125MHz)
    input  wire         rst_n,
    // ---- 输入：DMT 像素域（1 px/clk，24bit YUV444：Y[23:16] Cb[15:8] Cr[7:0]） ----
    input  wire         in_de,
    input  wire         in_hs,
    input  wire         in_vs,
    input  wire [23:0]  in_px,
    input  wire [10:0]  in_hc,
    input  wire [9:0]   in_vc,
    // ---- CLAHE 插值模式（由显示模式决定，切换时机在 bil_inter 内部锁存） ----
    input  wire         in_interp_mode,     // 0=整幅双线性；1=四区(640x360)分别插值
    output wire         out_de,
    output wire         out_hs,
    output wire         out_vs,
    output wire [23:0]  out_px,             // 24bit RGB888（正常）或 24bit 直通（BYPASS_CLAHE）
    output wire [10:0]  out_hc,
    output wire [9:0]   out_vc
);

//=============================================================================
// 【2026-10】输入已经是 24bit YUV444，无需再做 YUV422->444 转换，故删除原
//   yuv422to444 实例，直接把 24bit 像素送 CLAHE（clahe 只用 Y=[23:16]）。
//   正常路径：24bit YUV444 -> CLAHE（Y 增强 + 双线性）-> yuv2rgb -> RGB565
//             -> 扩展为 RGB888(24bit) 输出
//   仿真旁路：+define+BYPASS_CLAHE -> 整级直通，out_px == in_px（逐像素相等）
//=============================================================================
`ifdef BYPASS_CLAHE
    // ---- 旁路：整级直通（1 拍寄存器），输出 24bit == 输入 24bit ----
    reg [23:0] bp_px;
    reg        bp_de, bp_hs, bp_vs;
    reg [10:0] bp_hc;
    reg [9:0]  bp_vc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bp_px <= 24'd0;
            bp_de <= 1'b0; bp_hs <= 1'b0; bp_vs <= 1'b0;
            bp_hc <= 11'd0; bp_vc <= 10'd0;
        end else begin
            bp_px <= in_px;
            bp_de <= in_de; bp_hs <= in_hs; bp_vs <= in_vs;
            bp_hc <= in_hc; bp_vc <= in_vc;
        end
    end

    assign out_px = bp_px;
    assign out_de = bp_de; assign out_hs = bp_hs; assign out_vs = bp_vs;
    assign out_hc = bp_hc; assign out_vc = bp_vc;
`else

    clahe_top u_clahe_top(
        .pclk        (clk        ),
        .rst_n       (rst_n       ),
        .cliplimitx8 (CLIPLIMITX8 ),
        .interp_mode (in_interp_mode),
        .i_image     (in_px      ),     // 24bit YUV444（Y[23:16], Cb[15:8], Cr[7:0]）
        .i_vs        (in_vs      ),
        .i_hs        (in_hs      ),
        .i_de        (in_de      ),
        .i_hc        (in_hc      ),
        .i_vc        (in_vc      ),
        .o_image     (out_px     ),
        .o_vs        (out_vs     ),
        .o_hs        (out_hs     ),
        .o_hc        (out_hc     ),
        .o_vc        (out_vc     ),
        .o_de        (out_de     )
    );

`endif

endmodule
