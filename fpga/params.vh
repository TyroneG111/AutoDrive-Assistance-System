// ===========================================================================
// params.vh  --  geometry / timing parameters (v3: color, DRM <= 35, APM <= 70)
//
// 720p@30 : H total 1650, V total 750, pclk = 37.125MHz, clk = pclk4x = 148.5MHz
//
// v3 budget: DRM <= 35, APM <= 70
//   line cache : K=24 -> 12 rows/bank -> 7680 px -> padded to 8192
//                -> 7 DRM/bank x 4 banks = 28 DRM
//   map buffer : single buffer per table per eye  = 4 DRM
//   out buffer : 2 DRM
//                total = 34 DRM
// ===========================================================================
`ifndef PARAMS_VH
`define PARAMS_VH

// ---- source image (one camera) ----
`define IMG_W        640
`define IMG_H        360

// ---- stitching ----
`define STITCH_SHIFT 30            // physical right eye shifted right by 30 px (= -tx)
`define OUT_W        670           // fused canvas width  = 640 + 30
`define OVERLAP      610           // overlap width       = 640 - 30

// ---- vertical row window ----
`define LINE_DEPTH   24            // must be EVEN; K=24 is the last "cheap" DRM step
`define DLY          11            // output row delay
`define FOY_LO       36            // first produced output row (keeps 36..327, shifted down)
`define OUT_H        292           // produced rows = 36..327

// ---- fixed point ----
`define LUT_FRAC     8             // mapx/mapy output format: Q8.8

// ---- ROI inside the 720p frame ----
`define ROI_VC       360           // stereo pair starts at this line
`define SPLIT_HC     640           // left/right camera split column

// ---- video timing ----
`define H_ACT        1280
`define V_ACT        720
`define H_TOTAL      1650
`define V_TOTAL      750

// ---- map engine schedule, in clocks from the start of the line (t = hc*4 + ph) ----
//   t    0 ..  639 : mapgen burst LEFT  eye, 640 values -> map buffer L
//   t  640 ..  648 : drain gap (last samples flush with the old constants)
//   t  656 .. 1295 : mapgen burst RIGHT eye, 640 values -> map buffer R
//   t 1328 ..      : read / remap / blend for canvas columns 0..669 (1 per 4 clocks)
`define BURST_LEN    640
`define BURST_L_LO   0
`define BURST_R_LO   656
`define CONST_SW     649           // constants switch L -> R at this clock
`define READ_SLOT    332           // read starts at pixel slot 332 (= clock 1328)

`endif
