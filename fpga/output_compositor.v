//---------------------------------------------------------------------------
// output_compositor.v  --  fused-row line buffer + aligned read-out (v2)
//
// The fused pixels of output row `foy` are produced during input line
// vc = ROI_VC + DLY + foy, i.e. they emerge while the line is still being
// scanned -> they cannot be displayed in the same line.  They are therefore
// written into a ping-pong row buffer (670 x 16, 2 BRAM18K total) and read out
// during the NEXT line, which makes the read-out exactly hc-aligned (v1 fed the
// current hc/vc straight into the compositor, shifting the image 2~3 px).
//
// output frame: rows 375..719 (345 rows), columns 0..669, everything else black
//---------------------------------------------------------------------------
`include "params.vh"

module output_compositor (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        pix_en,
    // write side (fused pixels of the row being produced)
    input  wire        wr_en,
    input  wire [9:0]  wr_addr,
    input  wire [15:0] wr_data,
    input  wire        wr_sel,
    // read side (current scan position)
    input  wire        de,
    input  wire [10:0] hc,
    input  wire [9:0]  vc,
    output reg         de_out,
    output reg         out_valid,
    output reg  [10:0] out_hc,
    output reg  [9:0]  out_vc,
    output reg  [15:0] rgb565_out,
    output reg  [23:0] rgb888_out
);
    reg [15:0] buf0 [0:`OUT_W-1];
    reg [15:0] buf1 [0:`OUT_W-1];

    always @(posedge clk) begin
        if (wr_en && (wr_addr < `OUT_W)) begin
            if (!wr_sel) buf0[wr_addr] <= wr_data;
            else         buf1[wr_addr] <= wr_data;
        end
    end

    // v3: only rows  FOY_LO .. FOY_LO+OUT_H-1  are produced/displayed.
    //     vc window = 360 + DLY + 1 + FOY_LO  ..  + OUT_H - 1
    localparam [10:0] VC_MIN = `ROI_VC + `DLY + 1 + `FOY_LO;
    localparam [10:0] VC_MAX = `ROI_VC + `DLY + 1 + `FOY_LO + `OUT_H - 1;

    wire        emit_area = (vc >= VC_MIN) && (vc <= VC_MAX) && (hc < `OUT_W);
    wire [9:0]  emit_row  = vc - (`ROI_VC + `DLY + 1);
    wire        emit_sel  = emit_row[0];

    // ---- stage 1: latch the scan position ----
    reg         e1;
    reg  [10:0] h1;
    reg  [9:0]  v1_;
    reg         s1;
    always @(posedge clk) begin
        if (pix_en) begin
            e1  <= emit_area;   // window already implies active video
            h1  <= hc;
            v1_ <= vc;
            s1  <= emit_sel;
        end
    end

    // ---- stage 2: registered read-out ----
    // out_vc is rebased so the fused image starts at row 0 (0..OUT_H-1)
    wire [10:0] vrel = {1'b0, v1_} - VC_MIN;
    wire [15:0] rd = s1 ? buf1[h1[9:0]] : buf0[h1[9:0]];

    always @(posedge clk) begin
        if (!rst_n) begin
            de_out     <= 1'b0;
            out_valid  <= 1'b0;
            out_hc     <= 11'd0;
            out_vc     <= 10'd0;
            rgb565_out <= 16'd0;
            rgb888_out <= 24'd0;
        end else begin
            de_out    <= e1;
            out_valid <= e1;
            out_hc    <= h1;
            out_vc    <= vrel[9:0];
            rgb565_out<= e1 ? rd : 16'd0;
            // RGB565 -> RGB888 with bit replication
            // RGB565 -> RGB888 with bit replication (v = v<<n | v>>(w-n))
            //   r5 = rd[15:11] -> {r5, r5[4:2]} = {rd[15:11], rd[15:13]}
            //   g6 = rd[10:5]  -> {g6, g6[5:4]} = {rd[10:5],  rd[10:9] }
            //   b5 = rd[4:0]   -> {b5, b5[4:2]} = {rd[4:0],   rd[4:2]  }
            rgb888_out<= e1 ? {rd[15:11], rd[15:13],
                               rd[10:5],  rd[10:9],
                               rd[4:0],   rd[4:2]} : 24'd0;
        end
    end
endmodule
