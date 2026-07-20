// ============================================================
// tq_lut_rom.v
//
// TurboQuant 4bit Lloyd-Max Centroid / Boundary LUT ROM
//
// NOTE:
//   This ROM now exposes the original (unscaled) Lloyd-Max centroids.
//   The outputs are fixed original Lloyd-Max centroids and boundaries.
//   No d_sel input is needed anymore.
//
// centroid_out   : Q8.15 fixed-point (24bit signed) - 원본 Lloyd-Max centroid
//                  Centroid Sorter(양자화측) / 24bit 역양자화 경로용
// centroid16_out : Q7.8 fixed-point (16bit signed, INT16) - 원본 Lloyd-Max centroid
//                  Centroid Transform 모듈이 16bit 비교를 쓰기 때문에 추가함
//                  표현 범위: [-128.0, +127.9961], 해상도 1/256 ≈ 0.0039
//
// boundary_out : 15개 boundary (Sorter용, 24bit만 존재)
// ============================================================

module tq_lut_rom_imp (
    output reg signed [23:0] centroid_out_0,
    output reg signed [23:0] centroid_out_1,
    output reg signed [23:0] centroid_out_2,
    output reg signed [23:0] centroid_out_3,
    output reg signed [23:0] centroid_out_4,
    output reg signed [23:0] centroid_out_5,
    output reg signed [23:0] centroid_out_6,
    output reg signed [23:0] centroid_out_7,
    output reg signed [23:0] centroid_out_8,
    output reg signed [23:0] centroid_out_9,
    output reg signed [23:0] centroid_out_10,
    output reg signed [23:0] centroid_out_11,
    output reg signed [23:0] centroid_out_12,
    output reg signed [23:0] centroid_out_13,
    output reg signed [23:0] centroid_out_14,
    output reg signed [23:0] centroid_out_15,

    // 16bit INT16 Q7.8 centroid (Centroid Transform 모듈용, 24bit centroid와
    // 동일한 값을 다른 fixed-point 포맷으로 나란히 제공)
    output reg signed [15:0] centroid16_out_0,
    output reg signed [15:0] centroid16_out_1,
    output reg signed [15:0] centroid16_out_2,
    output reg signed [15:0] centroid16_out_3,
    output reg signed [15:0] centroid16_out_4,
    output reg signed [15:0] centroid16_out_5,
    output reg signed [15:0] centroid16_out_6,
    output reg signed [15:0] centroid16_out_7,
    output reg signed [15:0] centroid16_out_8,
    output reg signed [15:0] centroid16_out_9,
    output reg signed [15:0] centroid16_out_10,
    output reg signed [15:0] centroid16_out_11,
    output reg signed [15:0] centroid16_out_12,
    output reg signed [15:0] centroid16_out_13,
    output reg signed [15:0] centroid16_out_14,
    output reg signed [15:0] centroid16_out_15,

    output reg signed [23:0] boundary_out_0,
    output reg signed [23:0] boundary_out_1,
    output reg signed [23:0] boundary_out_2,
    output reg signed [23:0] boundary_out_3,
    output reg signed [23:0] boundary_out_4,
    output reg signed [23:0] boundary_out_5,
    output reg signed [23:0] boundary_out_6,
    output reg signed [23:0] boundary_out_7,
    output reg signed [23:0] boundary_out_8,
    output reg signed [23:0] boundary_out_9,
    output reg signed [23:0] boundary_out_10,
    output reg signed [23:0] boundary_out_11,
    output reg signed [23:0] boundary_out_12,
    output reg signed [23:0] boundary_out_13,
    output reg signed [23:0] boundary_out_14
);

    // -------------------------------------------------------
    // Fixed original centroid table
    // -------------------------------------------------------
    initial begin
        centroid_out_0  = 24'shFEA23A; // -2.732590
        centroid_out_1  = 24'shFEF72A; // -2.069017
        centroid_out_2  = 24'shFF30E4; // -1.618046
        centroid_out_3  = 24'shFF5F34; // -1.256231
        centroid_out_4  = 24'shFF8761; // -0.942340
        centroid_out_5  = 24'shFFABEF; // -0.656759
        centroid_out_6  = 24'shFFCE54; // -0.388048
        centroid_out_7  = 24'shFFEF91; // -0.128395
        centroid_out_8  = 24'sh00106F; // +0.128395
        centroid_out_9  = 24'sh0031AC; // +0.388048
        centroid_out_10 = 24'sh005411; // +0.656759
        centroid_out_11 = 24'sh00789F; // +0.942340
        centroid_out_12 = 24'sh00A0CC; // +1.256231
        centroid_out_13 = 24'sh00CF1C; // +1.618046
        centroid_out_14 = 24'sh0108D6; // +2.069017
        centroid_out_15 = 24'sh015DC6; // +2.732590

        centroid16_out_0  = 16'shFD44; // -2.732590
        centroid16_out_1  = 16'shFDEE; // -2.069017
        centroid16_out_2  = 16'shFE62; // -1.618046
        centroid16_out_3  = 16'shFEBE; // -1.256231
        centroid16_out_4  = 16'shFF0F; // -0.942340
        centroid16_out_5  = 16'shFF58; // -0.656759
        centroid16_out_6  = 16'shFF9D; // -0.388048
        centroid16_out_7  = 16'shFFDF; // -0.128395
        centroid16_out_8  = 16'sh0021; // +0.128395
        centroid16_out_9  = 16'sh0063; // +0.388048
        centroid16_out_10 = 16'sh00A8; // +0.656759
        centroid16_out_11 = 16'sh00F1; // +0.942340
        centroid16_out_12 = 16'sh0142; // +1.256231
        centroid16_out_13 = 16'sh019E; // +1.618046
        centroid16_out_14 = 16'sh0212; // +2.069017
        centroid16_out_15 = 16'sh02BC; // +2.732590

        boundary_out_0  = 24'shFECCB2; // -2.4008035
        boundary_out_1  = 24'shFF1407; // -1.8435315
        boundary_out_2  = 24'shFF480C; // -1.4371385
        boundary_out_3  = 24'shFF734B; // -1.0992855
        boundary_out_4  = 24'shFF99A8; // -0.7995495
        boundary_out_5  = 24'shFFBD22; // -0.5224035
        boundary_out_6  = 24'shFFDEF3; // -0.2582215
        boundary_out_7  = 24'sh000000; //  0.0000000
        boundary_out_8  = 24'sh00210D; // +0.2582215
        boundary_out_9  = 24'sh0042DE; // +0.5224035
        boundary_out_10 = 24'sh006658; // +0.7995495
        boundary_out_11 = 24'sh008CB5; // +1.0992855
        boundary_out_12 = 24'sh00B7F4; // +1.4371385
        boundary_out_13 = 24'sh00EBF9; // +1.8435315
        boundary_out_14 = 24'sh01334E; // +2.4008035
    end

endmodule
