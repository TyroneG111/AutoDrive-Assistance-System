//---------------------------------------------------------------------------
// stitch_blend.v  --  linear cross-fade in the overlap region (v2)
//
// canvas column ox = 0 .. OUT_W-1 (670):
//   ox <  SHIFT(30)     : left eye only
//   SHIFT <= ox < 640   : left*w + right*(1-w)
//   ox >= 640           : right eye only
//
// v2: the weight is computed, not stored.  blend_wleft.mem is a straight line
//     (constant decrement 107 = 65536/610), so
//        w = 65536 - (ox-30)*27503/256      (27503/256 = 65536/610)
//     which removes the weight ROM (1 BRAM) and the v1 misalignment between
//     the weight index and the pixel (the caller now passes the *aligned* ox).
//---------------------------------------------------------------------------
`include "params.vh"

module stitch_blend (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid,
    input  wire [9:0]  ox,          // canvas column, already aligned with the pixels
    input  wire [15:0] pix_left,
    input  wire [15:0] pix_right,
    input  wire        left_ok,     // ox <  IMG_W
    input  wire        right_ok,    // ox >= SHIFT
    output reg         valid_out,
    output reg  [15:0] pix_out
);
    // ---- weight (Q0.16) ----
    // NOTE: computed with module-level wires instead of a function-local reg --
    // PDS reports "net wcalc.p does not have a driver, tie it to 0" for the
    // function-local variable, which would silently zero the weight.
    //   w = 65536 - (ox-30)*27503/256   (27503/256 ~= 65536/610)
    wire [9:0]  wsub = (ox > `STITCH_SHIFT) ? (ox - `STITCH_SHIFT) : 10'd0;
    wire [24:0] wmul = wsub * 17'd27503;
    wire [16:0] wlin = 17'd65536 - wmul[24:8];
    wire [16:0] wval = ((ox < `STITCH_SHIFT) || (ox >= `IMG_W)) ? 17'd65536 : wlin;

    // ---- stage 1 ----
    reg [15:0] pl1, pr1;
    reg [16:0] w1;
    reg        v1, lok1, rok1;
    always @(posedge clk) begin
        v1   <= valid;
        lok1 <= left_ok;
        rok1 <= right_ok;
        w1   <= wval;
        pl1  <= pix_left;
        pr1  <= pix_right;
    end

    // ---- stage 2 ----
    wire [7:0] lr = {pl1[15:11], 3'd0};
    wire [7:0] lg = {pl1[10:5],  2'd0};
    wire [7:0] lb = {pl1[4:0],   3'd0};
    wire [7:0] rr = {pr1[15:11], 3'd0};
    wire [7:0] rg = {pr1[10:5],  2'd0};
    wire [7:0] rb = {pr1[4:0],   3'd0};
    wire [16:0] wc = 17'd65536 - w1;

    wire [24:0] sr = lr*w1 + rr*wc;
    wire [24:0] sg = lg*w1 + rg*wc;
    wire [24:0] sb = lb*w1 + rb*wc;

    always @(posedge clk) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            pix_out   <= 16'd0;
        end else begin
            valid_out <= v1;
            if (!lok1 && !rok1)      pix_out <= 16'h0000;
            else if (!lok1)          pix_out <= pr1;
            else if (!rok1)          pix_out <= pl1;
            else                     pix_out <= {sr[23:19], sg[23:18], sb[23:19]};
        end
    end
endmodule
