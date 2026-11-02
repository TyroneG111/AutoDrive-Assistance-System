//---------------------------------------------------------------------------
// mapgen.v  --  analytic remap coordinate generator (shared, no ROM, no divider)
//
// v3: ONE engine serves both eyes (the caller time-multiplexes it and switches
//     the 21 constants between bursts), and every constant fits 18 bit signed
//     with Q1.16 / Q27 / Q8.8 scales, so each multiply is a single 18x18 APM.
//
//   x  = ((ox*INVFX) >> 12) - OFFX                  Q2.15
//   [X,Y,Z] = R^T * [x, y, 1]                       (R in Q1.16)
//   a = X/Z , b = Y/Z , r2 = a^2+b^2
//   rad = 1 + r2*(k1 + k2*r2)                       (k3 = 0, Horner)
//   xd  = a*rad + 2*p1*a*b + p2*(r2+2a^2)
//   yd  = b*rad + p1*(r2+2b^2) + 2*p2*a*b
//   mapx = ((MFX*xd) >> 15) + MCX                   Q8.8
//
// 1/Z without a divider (Z is within [0.964,1.033] -> e = Z-1 is tiny):
//   1/(1+e) ~= 1 - e + e^2   (e^3 dropped: it is one APM per engine and
//
// 9 pipeline stages. oy/ox must be held while en is high (one point per clock).
// Out-of-range points output the sentinel 18'h3FFFF.
//---------------------------------------------------------------------------
`include "params.vh"

module mapgen (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en,
    input  wire [8:0]  oy,
    input  wire [9:0]  ox,
    // ---- calibration constants (signed 18 bit; scales in map_params.vh) ----
    input  wire signed [17:0] c_invfx,
    input  wire signed [17:0] c_invfy,
    input  wire signed [17:0] c_offx,
    input  wire signed [17:0] c_offy,
    input  wire signed [17:0] c_mfx,
    input  wire signed [17:0] c_mfy,
    input  wire signed [17:0] c_mcx,
    input  wire signed [17:0] c_mcy,
    input  wire signed [17:0] c_k1,
    input  wire signed [17:0] c_k2,
    input  wire signed [17:0] c_p1,
    input  wire signed [17:0] c_p2,
    input  wire signed [17:0] c_r00,
    input  wire signed [17:0] c_r01,
    input  wire signed [17:0] c_r02,
    input  wire signed [17:0] c_r10,
    input  wire signed [17:0] c_r11,
    input  wire signed [17:0] c_r12,
    input  wire signed [17:0] c_r20,
    input  wire signed [17:0] c_r21,
    input  wire signed [17:0] c_r22,
    output reg                dvalid,
    output reg  [17:0]        mapx,
    output reg  [17:0]        mapy
);
    localparam signed [17:0] ONE15 = 18'sd32768;        // 1.0 in Q2.15

    // rounded right shift: (p + half) >>> s  (keeps the fixed-point bias low)
    function signed [35:0] rsh(input signed [35:0] p, input integer s);
        begin
            rsh = (p + (36'sd1 <<< (s-1))) >>> s;
        end
    endfunction

    // ---------------- valid pipe ----------------
    reg v1,v2,v3,v4,v5,v6,v7,v8,v9;
    always @(posedge clk) begin
        if (!rst_n) begin
            v1<=0; v2<=0; v3<=0; v4<=0; v5<=0; v6<=0; v7<=0; v8<=0; v9<=0;
        end else begin
            v1<=en; v2<=v1; v3<=v2; v4<=v3; v5<=v4;
            v6<=v5; v7<=v6; v8<=v7; v9<=v8;
        end
    end

    // ---------------- s1 : normalize -> Q2.15 ----------------
    // x = ((ox*INVFX) >> 11) - OFFX : INVFX is Q26, offx is Q15
    // v4 时序修复：乘法器后面原来串了「取整加法 + 减偏移」两级加法，
    //   改写成一次加法（常量偏移由综合器折叠）：
    //     rsh(px,11) - c_offx  ==>  (px + (1024 - c_offx*2048)) >>> 11
    //   数学卜严格等价（c_offx*2048 是 2048 的整数倍），省掉一级加法。
    wire signed [35:0] px = $signed({{18{1'b0}}, ox}) * c_invfx;
    wire signed [35:0] py = $signed({{18{1'b0}}, oy}) * c_invfy;
    wire signed [35:0] kx1 = 36'sd1024 - ($signed({{18{c_offx[17]}}, c_offx}) <<< 11);
    wire signed [35:0] ky1 = 36'sd1024 - ($signed({{18{c_offy[17]}}, c_offy}) <<< 11);
    reg  signed [17:0] xq1, yq1;
    always @(posedge clk) begin
        if (en) begin
            xq1 <= (px + kx1) >>> 11;
            yq1 <= (py + ky1) >>> 11;
        end
    end

    // ---------------- s2 : X,Y,Z = R^T * [x,y,1] ----------------
    wire signed [35:0] p00 = c_r00 * xq1;
    wire signed [35:0] p10 = c_r10 * yq1;
    wire signed [35:0] p01 = c_r01 * xq1;
    wire signed [35:0] p11 = c_r11 * yq1;
    wire signed [35:0] p02 = c_r02 * xq1;
    wire signed [35:0] p12 = c_r12 * yq1;
    // v4 时序修复：乘法器后原来串了两次 36bit 取整加法 + 一次求和（实测 7.9ns），
    //   合并成一次加法树（差 <= 1 LSB）：
    //     rsh(p00,16)+rsh(p10,16)+C  ==>  (p00 + p10 + 65536 + C*65536) >>> 16
    wire signed [35:0] kX2 = ($signed({{18{c_r20[17]}}, c_r20}) >>> 1) <<< 16;
    wire signed [35:0] kY2 = ($signed({{18{c_r21[17]}}, c_r21}) >>> 1) <<< 16;
    wire signed [35:0] kZ2 = ($signed({{18{c_r22[17]}}, c_r22}) >>> 1) <<< 16;
    wire signed [36:0] sX2 = {p00[35],p00} + {p10[35],p10} + 37'sd65536 + {kX2[35],kX2};
    wire signed [36:0] sY2 = {p01[35],p01} + {p11[35],p11} + 37'sd65536 + {kY2[35],kY2};
    wire signed [36:0] sZ2 = {p02[35],p02} + {p12[35],p12} + 37'sd65536 + {kZ2[35],kZ2};
    reg signed [17:0] Xq, Yq, Zq;
    always @(posedge clk) begin
        Xq <= sX2[33:16];
        Yq <= sY2[33:16];
        Zq <= sZ2[33:16];
    end

    // ---------------- s3 : invZ = 1 - e + e^2 - e^3 ----------------
    // v5 时序修复：乘法器后面原来是「36bit 取整加法 + 两次 18bit 求和」，实测 8.7ns。
    //   e^2 项本身只有 1.3e-3 量级（|e| <= 0.036），取整带来的 <=0.5 LSB
    //   （Q15 下 3e-5，折算到 mapx 约 0.008px）可忽略，故直接截断：
    //     rsh(pe2,15)  ==>  pe2 >>> 15
    //   省掉 36bit 取整加法，乘法器后只剩一次 18bit 求和。
    wire signed [17:0] e15 = Zq - ONE15;
    wire signed [35:0] pe2 = e15 * e15;
    wire signed [17:0] e2  = pe2 >>> 15;
    reg  signed [17:0] invZq;
    always @(posedge clk) begin
        invZq <= ONE15 - e15 + e2;        // 2nd order: |e^3| <= 4.5e-5 = 0.016 px
    end

    // ---------------- s4 : a = X*invZ , b = Y*invZ ----------------
    wire signed [35:0] pXZ = Xq * invZq;
    wire signed [35:0] pYZ = Yq * invZq;
    reg signed [17:0] aq, bq;
    always @(posedge clk) begin
        aq <= rsh(pXZ,15);
        bq <= rsh(pYZ,15);
    end

    // ---------------- s5 : a2,b2,ab,r2 ----------------
    wire signed [35:0] pAA = aq * aq;
    wire signed [35:0] pBB = bq * bq;
    wire signed [35:0] pAB = aq * bq;
    reg signed [17:0] a2q, b2q, abq, r2q;
    reg signed [17:0] aqd, bqd;
    // v4 时序修复：r2q 原来是 rsh(pAA,15) + rsh(pBB,15)（两个 36bit 取整加法 + 一个
    //   求和串在组合乘法器之后，实测 7.93ns，WNS -2.42ns），合并成一次加法树：
    //     rsh(pAA,15)+rsh(pBB,15)  ==>  (pAA + pBB + 32768) >>> 15   （差 <= 1 LSB）
    wire signed [36:0] pRR = {pAA[35],pAA} + {pBB[35],pBB} + 37'sd32768;
    always @(posedge clk) begin
        a2q <= rsh(pAA,15);
        b2q <= rsh(pBB,15);
        abq <= rsh(pAB,15);
        r2q <= pRR[32:15];
        aqd <= aq;
        bqd <= bq;
    end

    // ---------------- s6a : kb = k1/2 + (k2*r2 >> 16) ----------------
    // NOTE: constants are Q1.16 (16 frac bits), variables Q2.15 -> the product
    // has 31 fraction bits, so it must be shifted by 16 to come back to Q15.
    //
    // v4 (2026-10) 时序修复：
    //   原 s6 把「乘法器1 -> 36bit 取整加法 -> 乘法器2 -> 取整加法」全部串在
    //   一拍内，综合后实测组合深度 10.6ns，而 clk_74_25M 周期只有 13.468ns
    //   （WNS -5.1ns / 632 个端点违例）。这里拆成 s6a / s6b 两级：
    //     s6a: c_k2*r2q 乘法 + 取整加法 -> kbq  （并与 a5..r25 同级）
    //     s6b: r25*kbq  乘法 + 取整加法 -> radq （并与 a6..r26 同级）
    //   总深度 9 拍 -> 10 拍。调用方在 feed 之后 3 个像素 slot(=12 拍) 采
    //   样 map 结果，输出有效窗口 [4s+10, 4s+14) 仍覆盖采样点 4s+12，
    //   故 en_map / READ_SLOT 的对齐**无需改动**（余量由 3 拍降为 2 拍）。
    wire signed [35:0] pR2 = c_k2 * r2q;
    wire signed [17:0] kb  = (c_k1 >>> 1) + rsh(pR2,16);   // k1(Q16->Q15) + k2*r2
    reg  signed [17:0] kbq;
    reg  signed [17:0] a5, b5, a25, b25, ab5, r25;         // s7 旁路数据, 与 kbq 同级
    always @(posedge clk) begin
        kbq <= kb;
        a5 <= aqd; b5 <= bqd; a25 <= a2q; b25 <= b2q; ab5 <= abq; r25 <= r2q;
    end

    // ---------------- s6b : rad = 1 + (r2*kb >> 15) ----------------
    wire signed [35:0] pRD = r25 * kbq;
    reg  signed [17:0] radq;
    reg  signed [17:0] a6, b6, a26, b26, ab6, r26;
    always @(posedge clk) begin
        radq <= ONE15 + rsh(pRD,15);
        a6 <= a5; b6 <= b5; a26 <= a25; b26 <= b25; ab6 <= ab5; r26 <= r25;
    end

    // ---------------- s7 : ar, br, tangential ----------------
    wire signed [35:0] pAR = a6 * radq;
    wire signed [35:0] pBR = b6 * radq;
    wire signed [35:0] pU1 = c_p1 * ab6;
    wire signed [35:0] pU2 = c_p2 * ab6;
    wire signed [17:0] inx = r26 + (a26 <<< 1);
    wire signed [17:0] iny = r26 + (b26 <<< 1);
    wire signed [35:0] pV1 = c_p2 * inx;
    wire signed [35:0] pV2 = c_p1 * iny;
    reg signed [17:0] arq, brq, u1q, u2q, v1q, v2q;
    always @(posedge clk) begin
        arq <= rsh(pAR,15);
        brq <= rsh(pBR,15);
        u1q <= pU1 >>> 16;      // p1 (Q1.16) * ab (Q15) -> Q15
        u2q <= pU2 >>> 16;
        v1q <= pV1 >>> 16;
        v2q <= pV2 >>> 16;
    end

    // ---------------- s8 : xd, yd ----------------
    reg signed [17:0] xdq, ydq;
    always @(posedge clk) begin
        xdq <= arq + (u1q <<< 1) + v1q;
        ydq <= brq + (u2q <<< 1) + v2q;
    end

    // ---------------- s9 : mapx, mapy (Q8.8) + range check ----------------
    wire signed [35:0] pMX = c_mfx * xdq;
    wire signed [35:0] pMY = c_mfy * ydq;
    wire signed [23:0] mapx_q8 = rsh(pMX,14) + c_mcx;   // MFX is Q8.7
    wire signed [23:0] mapy_q8 = rsh(pMY,14) + c_mcy;
    wire ok_x = (mapx_q8 >= 0) && (mapx_q8 < (`IMG_W*256));
    wire ok_y = (mapy_q8 >= 0) && (mapy_q8 < (`IMG_H*256));

    always @(posedge clk) begin
        dvalid <= v8;   // v4: 流水深度 9 -> 10，有效标志同步后移一级
        if (ok_x && ok_y) begin
            mapx <= mapx_q8[17:0];
            mapy <= mapy_q8[17:0];
        end else begin
            mapx <= 18'h3FFFF;
            mapy <= 18'h3FFFF;
        end
    end
endmodule
