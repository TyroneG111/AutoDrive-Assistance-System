//---------------------------------------------------------------------------
// stereo_roi_gate.v
// 从 720p 扫描时序中识别下排双目，生成左右有效标志和内部坐标。
//
// 画面布局（2x2）：
//   左下 hc=0~639   物理右目（先到）
//   右下 hc=640~1279 物理左目（后到）
//   双目行 vc=360~719
//
// 输出：
//   r_valid : 物理右目像素（本拍写右目行缓存）
//   l_valid : 物理左目像素（本拍写左目行缓存 + 触发融合）
//   sx/sy   : 双目内部坐标
//---------------------------------------------------------------------------
module stereo_roi_gate #(
    parameter W = 640,
    parameter H = 360,
    parameter ROI_VC = 360,
    parameter SPLIT_HC = 640
)(
    input  wire        de,
    input  wire [10:0] hc,
    input  wire [9:0]  vc,
    input  wire [15:0] rgb,
    output wire        r_valid,
    output wire        l_valid,
    output wire [9:0]  sx,
    output wire [8:0]  sy,
    output wire [15:0] rgb_out
);
    wire is_stereo = (vc >= ROI_VC) && (vc < ROI_VC + H);
    wire is_right  = is_stereo && (hc <  SPLIT_HC);
    wire is_left   = is_stereo && (hc >= SPLIT_HC) && (hc < SPLIT_HC + W);

    assign r_valid = de && is_right;
    assign l_valid = de && is_left;

    // 右目 hc=0..639；左目 hc=640..1279 -> sx=0..639
    // v2: 用完整位宽计算（v1 里的 hc[9:0]/vc[8:0] 在 hc>=1024、vc>=512 时会回绕）
    assign sx = is_left ? (hc - SPLIT_HC) : hc;
    assign sy = vc - ROI_VC;
    assign rgb_out = rgb;
endmodule
