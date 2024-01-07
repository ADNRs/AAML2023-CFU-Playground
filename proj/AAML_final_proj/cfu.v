// Copyright 2023-2024 Chung-Yi Chen
// Implementation of CFU

`define ADDR_BITS 12

(*use_dsp = "no"*) module Cfu (
    input               cmd_valid,
    output              cmd_ready,
    input      [9:0]    cmd_payload_function_id,
    input      [31:0]   cmd_payload_inputs_0,
    input      [31:0]   cmd_payload_inputs_1,
    output reg          rsp_valid,
    input               rsp_ready,
    output reg [31:0]   rsp_payload_outputs_0,
    input               reset,
    input               clk
);

// Wires/regs for systolic array and buffers

reg [9:0]               M, N, K;
reg [8:0]               offset;
reg                     in_valid;
wire                    busy;

wire                    A_wr_en, A_wr_en_sa;
wire [`ADDR_BITS-1:0]   A_index, A_index_sa;
reg [127:0]             A_data_in;
wire [127:0]            A_data_out;

wire                    B_wr_en, B_wr_en_sa;
wire [`ADDR_BITS-1:0]   B_index, B_index_sa;
reg [127:0]             B_data_in;
wire [127:0]            B_data_out;

wire                    C_wr_en, C_wr_en_sa;
wire [`ADDR_BITS-1:0]   C_index, C_index_sa;
wire [511:0]            C_data_in;
wire [511:0]            C_data_out;

reg                     A_wr_en_cfu;
reg [`ADDR_BITS-1:0]    A_index_cfu;

reg                     B_wr_en_cfu;
reg [`ADDR_BITS-1:0]    B_index_cfu;

reg                     C_wr_en_cfu, C_acc_en;
reg [`ADDR_BITS-1:0]    C_index_cfu;

// Set up connection

assign A_wr_en = busy ? A_wr_en_sa : A_wr_en_cfu;
assign A_index = busy ? A_index_sa : A_index_cfu;

assign B_wr_en = busy ? B_wr_en_sa : B_wr_en_cfu;
assign B_index = busy ? B_index_sa : B_index_cfu;

assign C_wr_en = busy ? C_wr_en_sa : C_wr_en_cfu;
assign C_index = busy ? C_index_sa : C_index_cfu;

// Wires/regs for commands

reg         writeinA_state;  // state of matrixA writein
reg         writeinB_state;  // state of matrixB writein
reg [3:0]   readout_state;   // state of matrixC readout
reg [63:0]  A_data_in_temp;
reg [63:0]  B_data_in_temp;
reg [31:0]  readout_value;

// Instantiate systolic array

SystolicArray #(.ADDR_BITS(`ADDR_BITS)) sa(
    .clk(clk), .M(M), .N(N), .K(K), .offset(offset), .in_valid(in_valid),
    .acc_en(C_acc_en), .busy(busy), .A_wr_en(A_wr_en_sa), .A_index(A_index_sa),
    .A_data_in(), .A_data_out(A_data_out), .B_wr_en(B_wr_en_sa),
    .B_index(B_index_sa), .B_data_in(), .B_data_out(B_data_out),
    .C_wr_en(C_wr_en_sa), .C_index(C_index_sa), .C_data_in(C_data_in),
    .C_data_out(C_data_out));

// Instantiate buffers

ReadBuffer #(.ADDR_BITS(`ADDR_BITS), .DATA_BITS(128)) gbuff_A(
    .clk(clk), .wr_en(A_wr_en), .index(A_index),
    .data_in(A_data_in), .data_out(A_data_out));
ReadBuffer #(.ADDR_BITS(`ADDR_BITS), .DATA_BITS(128)) gbuff_B(
    .clk(clk), .wr_en(B_wr_en), .index(B_index),
    .data_in(B_data_in), .data_out(B_data_out));
AccumulationBuffer #(.ADDR_BITS(`ADDR_BITS), .DATA_BITS(512)) gbuff_C(
    .clk(clk), .wr_en(C_wr_en), .index(C_index),
    .data_in(C_data_in), .data_out(C_data_out));

// Implement SRDHM logic

reg         overflow;
reg [63:0]  ab_64;
wire [31:0] nudge;
wire [63:0] ab_64_nudge;
wire [31:0] srdhm;

assign nudge = ab_64[63] ? 32'hc0000001 : 32'h40000000;
assign ab_64_nudge = $signed(ab_64) + $signed(nudge);
assign srdhm = overflow ? 32'h7fffffff :
    ab_64_nudge[63] ? -(-ab_64_nudge >> 31) : ab_64_nudge >> 31;

// Implement RDBPOT logic

wire signed [31:0] mask;
wire signed [31:0] remainder;
wire signed [31:0] threshold;
wire signed [31:0] rdbpot;

assign mask = (1 << cmd_payload_inputs_1) - 1;
assign remainder = cmd_payload_inputs_0 & mask;
assign threshold = (mask >>> 1) + cmd_payload_inputs_0[31];
assign rdbpot = $signed($signed(cmd_payload_inputs_0) >>> cmd_payload_inputs_1) +
                ($signed(remainder) > $signed(threshold));

// Implement off_minmax logic

wire [31:0] add_off;
wire [31:0] clamp_max;
wire [31:0] clamp_min;

assign add_off = cmd_payload_inputs_0 + cmd_payload_inputs_1;
assign clamp_max = $signed(add_off) > $signed(-128) ? add_off : $signed(-128);
assign clamp_min = $signed(clamp_max) < $signed(127) ? clamp_max : $signed(127);

// Implement post selection logic to improve timing

wire [31:0] post_output;

assign post_output = cmd_payload_function_id[7:3] == 6 ? srdhm :
                     cmd_payload_function_id[7:3] == 7 ? rdbpot : clamp_min;

// Only not ready for a command when we have a response.
assign cmd_ready = ~rsp_valid;

always @(posedge clk) begin
    in_valid <= 0;

    if (reset) begin
        rsp_valid <= 1'b0;
        A_wr_en_cfu <= 0; A_index_cfu <= ~0;
        B_wr_en_cfu <= 0; B_index_cfu <= ~0;
        C_wr_en_cfu <= 0; C_index_cfu <= 0;
        writeinA_state <= 0;
        writeinB_state <= 0;
        readout_state <= 0;
    end else if (rsp_valid) begin
        rsp_valid <= ~rsp_ready;
    end else if (cmd_valid) begin
        case(cmd_payload_function_id[9:3])
            0: begin  // set_matrixA: cfu_op0(0, uval, lval)
                case(writeinA_state)
                    0: begin
                        A_wr_en_cfu <= 0;
                        A_data_in_temp <= {cmd_payload_inputs_0,
                                           cmd_payload_inputs_1};
                    end
                    1: begin
                        A_wr_en_cfu <= 1;
                        A_index_cfu <= A_index_cfu + 1;
                        A_data_in <= {A_data_in_temp,
                                      cmd_payload_inputs_0,
                                      cmd_payload_inputs_1};
                    end
                endcase

                writeinA_state <= writeinA_state + 1;
            end
            1: begin  // set_matrixB: cfu_op0(1, uval, lval)
                case(writeinB_state)
                    0: begin
                        B_wr_en_cfu <= 0;
                        B_data_in_temp <= {cmd_payload_inputs_0,
                                           cmd_payload_inputs_1};
                    end
                    1: begin
                        B_wr_en_cfu <= 1;
                        B_index_cfu <= B_index_cfu + 1;
                        B_data_in <= {B_data_in_temp,
                                      cmd_payload_inputs_0,
                                      cmd_payload_inputs_1};
                    end
                endcase

                writeinB_state <= writeinB_state + 1;
            end
            2: begin  // start_GEMM: cfu_op0(2, A<<30 | M<<20 | N<<10 | K, off)
                B_wr_en_cfu <= 0;
                A_index_cfu <= ~0; B_index_cfu <= ~0; C_index_cfu <= 0;
                in_valid <= 1;
                C_acc_en <= cmd_payload_inputs_0[30];
                M <= cmd_payload_inputs_0[29:20];
                N <= cmd_payload_inputs_0[19:10];
                K <= cmd_payload_inputs_0[9:0];
                offset <= cmd_payload_inputs_1[8:0];
            end
            3: begin  // check_GEMM: cfu_op0(3, 0, 0)
                rsp_payload_outputs_0 <= busy;
            end
            4: begin  // get_matrixC: cfu_op0(4, 0, 0)
                case(readout_state)
                    0:  readout_value <= C_data_out[479:448];
                    1:  readout_value <= C_data_out[447:416];
                    2:  readout_value <= C_data_out[415:384];
                    3:  readout_value <= C_data_out[383:352];
                    4:  readout_value <= C_data_out[351:320];
                    5:  readout_value <= C_data_out[319:288];
                    6:  readout_value <= C_data_out[287:256];
                    7:  readout_value <= C_data_out[255:224];
                    8:  readout_value <= C_data_out[223:192];
                    9:  readout_value <= C_data_out[191:160];
                    10: readout_value <= C_data_out[159:128];
                    11: readout_value <= C_data_out[127:96];
                    12: readout_value <= C_data_out[95:64];
                    13: readout_value <= C_data_out[63:32];
                    14: readout_value <= C_data_out[31:0];
                    15: C_index_cfu <= C_index_cfu + 1;
                endcase

                rsp_payload_outputs_0 <= readout_state == 0
                    ? C_data_out[511:480] : readout_value;
                readout_state <= readout_state + 1;
            end
            5: begin  // post_set_SRDHM: cfu_op0(5, a, b)
                overflow <= (cmd_payload_inputs_0 == 32'h80000000) &&
                            (cmd_payload_inputs_1 == 32'h80000000);
                ab_64 <= $signed(cmd_payload_inputs_0) *
                         $signed(cmd_payload_inputs_1);
            end
            6: begin  // post_get_SRDHM: cfu_op0(6, 0, 0)
                rsp_payload_outputs_0 <= post_output;
            end
            7: begin  // post_RDBPOT: cfu_op0(7, x, exp)
                rsp_payload_outputs_0 <= post_output;
            end
            8: begin  // post_off_minmax: cfu_op0(8, val, off)
                rsp_payload_outputs_0 <= post_output;
            end
        endcase  // decode

        rsp_valid <= 1;
    end  // cmd_valid
end  // always

endmodule

module SystolicArray #(parameter ADDR_BITS=8)(
    input                       clk,
    input [9:0]                 M, N, K,
    input [8:0]                 offset,
    input                       in_valid,
    input                       acc_en,
    output reg                  busy,
    output reg                  A_wr_en,
    output reg [ADDR_BITS-1:0]  A_index,
    output reg [127:0]          A_data_in,
    input [127:0]               A_data_out,
    output reg                  B_wr_en,
    output reg [ADDR_BITS-1:0]  B_index,
    output reg [127:0]          B_data_in,
    input [127:0]               B_data_out,
    output reg                  C_wr_en,
    output reg [ADDR_BITS-1:0]  C_index,
    output reg [511:0]          C_data_in,
    input [511:0]               C_data_out
);

// Backup regs

reg [9:0]   M_r, N_r, K_r;
reg [8:0]   offset_r;
reg         acc_en_r;

// Wires/Regs for systolic array

wire [9:0]      cnt_M, cnt_N;  // loop induction variables
reg [9:0]       m, n, k;       // loop induction variables
reg [119:0]     AA[0:15];      // array from A
reg [119:0]     BB[0:15];      // array from B
wire [511:0]    CC[0:15];      // array to C
reg [8:0]       a[0:15];       // input of systolic array
reg [7:0]       b[0:15];       // input of systolic array
reg [65:0]      state;         // state of (16xk)x(kx16) matmul
reg             pe_rst;        // PE reset

wire [8:0]  h_000_001, h_001_002, h_002_003, h_003_004, h_004_005, h_005_006, h_006_007, h_007_008;
wire [8:0]  h_008_009, h_009_010, h_010_011, h_011_012, h_012_013, h_013_014, h_014_015, h_016_017;
wire [8:0]  h_017_018, h_018_019, h_019_020, h_020_021, h_021_022, h_022_023, h_023_024, h_024_025;
wire [8:0]  h_025_026, h_026_027, h_027_028, h_028_029, h_029_030, h_030_031, h_032_033, h_033_034;
wire [8:0]  h_034_035, h_035_036, h_036_037, h_037_038, h_038_039, h_039_040, h_040_041, h_041_042;
wire [8:0]  h_042_043, h_043_044, h_044_045, h_045_046, h_046_047, h_048_049, h_049_050, h_050_051;
wire [8:0]  h_051_052, h_052_053, h_053_054, h_054_055, h_055_056, h_056_057, h_057_058, h_058_059;
wire [8:0]  h_059_060, h_060_061, h_061_062, h_062_063, h_064_065, h_065_066, h_066_067, h_067_068;
wire [8:0]  h_068_069, h_069_070, h_070_071, h_071_072, h_072_073, h_073_074, h_074_075, h_075_076;
wire [8:0]  h_076_077, h_077_078, h_078_079, h_080_081, h_081_082, h_082_083, h_083_084, h_084_085;
wire [8:0]  h_085_086, h_086_087, h_087_088, h_088_089, h_089_090, h_090_091, h_091_092, h_092_093;
wire [8:0]  h_093_094, h_094_095, h_096_097, h_097_098, h_098_099, h_099_100, h_100_101, h_101_102;
wire [8:0]  h_102_103, h_103_104, h_104_105, h_105_106, h_106_107, h_107_108, h_108_109, h_109_110;
wire [8:0]  h_110_111, h_112_113, h_113_114, h_114_115, h_115_116, h_116_117, h_117_118, h_118_119;
wire [8:0]  h_119_120, h_120_121, h_121_122, h_122_123, h_123_124, h_124_125, h_125_126, h_126_127;
wire [8:0]  h_128_129, h_129_130, h_130_131, h_131_132, h_132_133, h_133_134, h_134_135, h_135_136;
wire [8:0]  h_136_137, h_137_138, h_138_139, h_139_140, h_140_141, h_141_142, h_142_143, h_144_145;
wire [8:0]  h_145_146, h_146_147, h_147_148, h_148_149, h_149_150, h_150_151, h_151_152, h_152_153;
wire [8:0]  h_153_154, h_154_155, h_155_156, h_156_157, h_157_158, h_158_159, h_160_161, h_161_162;
wire [8:0]  h_162_163, h_163_164, h_164_165, h_165_166, h_166_167, h_167_168, h_168_169, h_169_170;
wire [8:0]  h_170_171, h_171_172, h_172_173, h_173_174, h_174_175, h_176_177, h_177_178, h_178_179;
wire [8:0]  h_179_180, h_180_181, h_181_182, h_182_183, h_183_184, h_184_185, h_185_186, h_186_187;
wire [8:0]  h_187_188, h_188_189, h_189_190, h_190_191, h_192_193, h_193_194, h_194_195, h_195_196;
wire [8:0]  h_196_197, h_197_198, h_198_199, h_199_200, h_200_201, h_201_202, h_202_203, h_203_204;
wire [8:0]  h_204_205, h_205_206, h_206_207, h_208_209, h_209_210, h_210_211, h_211_212, h_212_213;
wire [8:0]  h_213_214, h_214_215, h_215_216, h_216_217, h_217_218, h_218_219, h_219_220, h_220_221;
wire [8:0]  h_221_222, h_222_223, h_224_225, h_225_226, h_226_227, h_227_228, h_228_229, h_229_230;
wire [8:0]  h_230_231, h_231_232, h_232_233, h_233_234, h_234_235, h_235_236, h_236_237, h_237_238;
wire [8:0]  h_238_239, h_240_241, h_241_242, h_242_243, h_243_244, h_244_245, h_245_246, h_246_247;
wire [8:0]  h_247_248, h_248_249, h_249_250, h_250_251, h_251_252, h_252_253, h_253_254, h_254_255;

wire [7:0]  v_000_016, v_001_017, v_002_018, v_003_019, v_004_020, v_005_021, v_006_022, v_007_023;
wire [7:0]  v_008_024, v_009_025, v_010_026, v_011_027, v_012_028, v_013_029, v_014_030, v_015_031;
wire [7:0]  v_016_032, v_017_033, v_018_034, v_019_035, v_020_036, v_021_037, v_022_038, v_023_039;
wire [7:0]  v_024_040, v_025_041, v_026_042, v_027_043, v_028_044, v_029_045, v_030_046, v_031_047;
wire [7:0]  v_032_048, v_033_049, v_034_050, v_035_051, v_036_052, v_037_053, v_038_054, v_039_055;
wire [7:0]  v_040_056, v_041_057, v_042_058, v_043_059, v_044_060, v_045_061, v_046_062, v_047_063;
wire [7:0]  v_048_064, v_049_065, v_050_066, v_051_067, v_052_068, v_053_069, v_054_070, v_055_071;
wire [7:0]  v_056_072, v_057_073, v_058_074, v_059_075, v_060_076, v_061_077, v_062_078, v_063_079;
wire [7:0]  v_064_080, v_065_081, v_066_082, v_067_083, v_068_084, v_069_085, v_070_086, v_071_087;
wire [7:0]  v_072_088, v_073_089, v_074_090, v_075_091, v_076_092, v_077_093, v_078_094, v_079_095;
wire [7:0]  v_080_096, v_081_097, v_082_098, v_083_099, v_084_100, v_085_101, v_086_102, v_087_103;
wire [7:0]  v_088_104, v_089_105, v_090_106, v_091_107, v_092_108, v_093_109, v_094_110, v_095_111;
wire [7:0]  v_096_112, v_097_113, v_098_114, v_099_115, v_100_116, v_101_117, v_102_118, v_103_119;
wire [7:0]  v_104_120, v_105_121, v_106_122, v_107_123, v_108_124, v_109_125, v_110_126, v_111_127;
wire [7:0]  v_112_128, v_113_129, v_114_130, v_115_131, v_116_132, v_117_133, v_118_134, v_119_135;
wire [7:0]  v_120_136, v_121_137, v_122_138, v_123_139, v_124_140, v_125_141, v_126_142, v_127_143;
wire [7:0]  v_128_144, v_129_145, v_130_146, v_131_147, v_132_148, v_133_149, v_134_150, v_135_151;
wire [7:0]  v_136_152, v_137_153, v_138_154, v_139_155, v_140_156, v_141_157, v_142_158, v_143_159;
wire [7:0]  v_144_160, v_145_161, v_146_162, v_147_163, v_148_164, v_149_165, v_150_166, v_151_167;
wire [7:0]  v_152_168, v_153_169, v_154_170, v_155_171, v_156_172, v_157_173, v_158_174, v_159_175;
wire [7:0]  v_160_176, v_161_177, v_162_178, v_163_179, v_164_180, v_165_181, v_166_182, v_167_183;
wire [7:0]  v_168_184, v_169_185, v_170_186, v_171_187, v_172_188, v_173_189, v_174_190, v_175_191;
wire [7:0]  v_176_192, v_177_193, v_178_194, v_179_195, v_180_196, v_181_197, v_182_198, v_183_199;
wire [7:0]  v_184_200, v_185_201, v_186_202, v_187_203, v_188_204, v_189_205, v_190_206, v_191_207;
wire [7:0]  v_192_208, v_193_209, v_194_210, v_195_211, v_196_212, v_197_213, v_198_214, v_199_215;
wire [7:0]  v_200_216, v_201_217, v_202_218, v_203_219, v_204_220, v_205_221, v_206_222, v_207_223;
wire [7:0]  v_208_224, v_209_225, v_210_226, v_211_227, v_212_228, v_213_229, v_214_230, v_215_231;
wire [7:0]  v_216_232, v_217_233, v_218_234, v_219_235, v_220_236, v_221_237, v_222_238, v_223_239;
wire [7:0]  v_224_240, v_225_241, v_226_242, v_227_243, v_228_244, v_229_245, v_230_246, v_231_247;
wire [7:0]  v_232_248, v_233_249, v_234_250, v_235_251, v_236_252, v_237_253, v_238_254, v_239_255;

// Implement systolic array

ProcessingElement P000(.clk(clk), .rst(pe_rst), .left(a[0]),
    .top(b[0]), .right(h_000_001), .down(v_000_016), .acc(CC[0][511:480]));
ProcessingElement P001(.clk(clk), .rst(pe_rst), .left(h_000_001),
    .top(b[1]), .right(h_001_002), .down(v_001_017), .acc(CC[0][479:448]));
ProcessingElement P002(.clk(clk), .rst(pe_rst), .left(h_001_002),
    .top(b[2]), .right(h_002_003), .down(v_002_018), .acc(CC[0][447:416]));
ProcessingElement P003(.clk(clk), .rst(pe_rst), .left(h_002_003),
    .top(b[3]), .right(h_003_004), .down(v_003_019), .acc(CC[0][415:384]));
ProcessingElement P004(.clk(clk), .rst(pe_rst), .left(h_003_004),
    .top(b[4]), .right(h_004_005), .down(v_004_020), .acc(CC[0][383:352]));
ProcessingElement P005(.clk(clk), .rst(pe_rst), .left(h_004_005),
    .top(b[5]), .right(h_005_006), .down(v_005_021), .acc(CC[0][351:320]));
ProcessingElement P006(.clk(clk), .rst(pe_rst), .left(h_005_006),
    .top(b[6]), .right(h_006_007), .down(v_006_022), .acc(CC[0][319:288]));
ProcessingElement P007(.clk(clk), .rst(pe_rst), .left(h_006_007),
    .top(b[7]), .right(h_007_008), .down(v_007_023), .acc(CC[0][287:256]));
ProcessingElement P008(.clk(clk), .rst(pe_rst), .left(h_007_008),
    .top(b[8]), .right(h_008_009), .down(v_008_024), .acc(CC[0][255:224]));
ProcessingElement P009(.clk(clk), .rst(pe_rst), .left(h_008_009),
    .top(b[9]), .right(h_009_010), .down(v_009_025), .acc(CC[0][223:192]));
ProcessingElement P010(.clk(clk), .rst(pe_rst), .left(h_009_010),
    .top(b[10]), .right(h_010_011), .down(v_010_026), .acc(CC[0][191:160]));
ProcessingElement P011(.clk(clk), .rst(pe_rst), .left(h_010_011),
    .top(b[11]), .right(h_011_012), .down(v_011_027), .acc(CC[0][159:128]));
ProcessingElement P012(.clk(clk), .rst(pe_rst), .left(h_011_012),
    .top(b[12]), .right(h_012_013), .down(v_012_028), .acc(CC[0][127:96]));
ProcessingElement P013(.clk(clk), .rst(pe_rst), .left(h_012_013),
    .top(b[13]), .right(h_013_014), .down(v_013_029), .acc(CC[0][95:64]));
ProcessingElement P014(.clk(clk), .rst(pe_rst), .left(h_013_014),
    .top(b[14]), .right(h_014_015), .down(v_014_030), .acc(CC[0][63:32]));
ProcessingElement P015(.clk(clk), .rst(pe_rst), .left(h_014_015),
    .top(b[15]), .right(), .down(v_015_031), .acc(CC[0][31:0]));

ProcessingElement P016(.clk(clk), .rst(pe_rst), .left(a[1]),
    .top(v_000_016), .right(h_016_017), .down(v_016_032), .acc(CC[1][511:480]));
ProcessingElement P017(.clk(clk), .rst(pe_rst), .left(h_016_017),
    .top(v_001_017), .right(h_017_018), .down(v_017_033), .acc(CC[1][479:448]));
ProcessingElement P018(.clk(clk), .rst(pe_rst), .left(h_017_018),
    .top(v_002_018), .right(h_018_019), .down(v_018_034), .acc(CC[1][447:416]));
ProcessingElement P019(.clk(clk), .rst(pe_rst), .left(h_018_019),
    .top(v_003_019), .right(h_019_020), .down(v_019_035), .acc(CC[1][415:384]));
ProcessingElement P020(.clk(clk), .rst(pe_rst), .left(h_019_020),
    .top(v_004_020), .right(h_020_021), .down(v_020_036), .acc(CC[1][383:352]));
ProcessingElement P021(.clk(clk), .rst(pe_rst), .left(h_020_021),
    .top(v_005_021), .right(h_021_022), .down(v_021_037), .acc(CC[1][351:320]));
ProcessingElement P022(.clk(clk), .rst(pe_rst), .left(h_021_022),
    .top(v_006_022), .right(h_022_023), .down(v_022_038), .acc(CC[1][319:288]));
ProcessingElement P023(.clk(clk), .rst(pe_rst), .left(h_022_023),
    .top(v_007_023), .right(h_023_024), .down(v_023_039), .acc(CC[1][287:256]));
ProcessingElement P024(.clk(clk), .rst(pe_rst), .left(h_023_024),
    .top(v_008_024), .right(h_024_025), .down(v_024_040), .acc(CC[1][255:224]));
ProcessingElement P025(.clk(clk), .rst(pe_rst), .left(h_024_025),
    .top(v_009_025), .right(h_025_026), .down(v_025_041), .acc(CC[1][223:192]));
ProcessingElement P026(.clk(clk), .rst(pe_rst), .left(h_025_026),
    .top(v_010_026), .right(h_026_027), .down(v_026_042), .acc(CC[1][191:160]));
ProcessingElement P027(.clk(clk), .rst(pe_rst), .left(h_026_027),
    .top(v_011_027), .right(h_027_028), .down(v_027_043), .acc(CC[1][159:128]));
ProcessingElement P028(.clk(clk), .rst(pe_rst), .left(h_027_028),
    .top(v_012_028), .right(h_028_029), .down(v_028_044), .acc(CC[1][127:96]));
ProcessingElement P029(.clk(clk), .rst(pe_rst), .left(h_028_029),
    .top(v_013_029), .right(h_029_030), .down(v_029_045), .acc(CC[1][95:64]));
ProcessingElement P030(.clk(clk), .rst(pe_rst), .left(h_029_030),
    .top(v_014_030), .right(h_030_031), .down(v_030_046), .acc(CC[1][63:32]));
ProcessingElement P031(.clk(clk), .rst(pe_rst), .left(h_030_031),
    .top(v_015_031), .right(), .down(v_031_047), .acc(CC[1][31:0]));

ProcessingElement P032(.clk(clk), .rst(pe_rst), .left(a[2]),
    .top(v_016_032), .right(h_032_033), .down(v_032_048), .acc(CC[2][511:480]));
ProcessingElement P033(.clk(clk), .rst(pe_rst), .left(h_032_033),
    .top(v_017_033), .right(h_033_034), .down(v_033_049), .acc(CC[2][479:448]));
ProcessingElement P034(.clk(clk), .rst(pe_rst), .left(h_033_034),
    .top(v_018_034), .right(h_034_035), .down(v_034_050), .acc(CC[2][447:416]));
ProcessingElement P035(.clk(clk), .rst(pe_rst), .left(h_034_035),
    .top(v_019_035), .right(h_035_036), .down(v_035_051), .acc(CC[2][415:384]));
ProcessingElement P036(.clk(clk), .rst(pe_rst), .left(h_035_036),
    .top(v_020_036), .right(h_036_037), .down(v_036_052), .acc(CC[2][383:352]));
ProcessingElement P037(.clk(clk), .rst(pe_rst), .left(h_036_037),
    .top(v_021_037), .right(h_037_038), .down(v_037_053), .acc(CC[2][351:320]));
ProcessingElement P038(.clk(clk), .rst(pe_rst), .left(h_037_038),
    .top(v_022_038), .right(h_038_039), .down(v_038_054), .acc(CC[2][319:288]));
ProcessingElement P039(.clk(clk), .rst(pe_rst), .left(h_038_039),
    .top(v_023_039), .right(h_039_040), .down(v_039_055), .acc(CC[2][287:256]));
ProcessingElement P040(.clk(clk), .rst(pe_rst), .left(h_039_040),
    .top(v_024_040), .right(h_040_041), .down(v_040_056), .acc(CC[2][255:224]));
ProcessingElement P041(.clk(clk), .rst(pe_rst), .left(h_040_041),
    .top(v_025_041), .right(h_041_042), .down(v_041_057), .acc(CC[2][223:192]));
ProcessingElement P042(.clk(clk), .rst(pe_rst), .left(h_041_042),
    .top(v_026_042), .right(h_042_043), .down(v_042_058), .acc(CC[2][191:160]));
ProcessingElement P043(.clk(clk), .rst(pe_rst), .left(h_042_043),
    .top(v_027_043), .right(h_043_044), .down(v_043_059), .acc(CC[2][159:128]));
ProcessingElement P044(.clk(clk), .rst(pe_rst), .left(h_043_044),
    .top(v_028_044), .right(h_044_045), .down(v_044_060), .acc(CC[2][127:96]));
ProcessingElement P045(.clk(clk), .rst(pe_rst), .left(h_044_045),
    .top(v_029_045), .right(h_045_046), .down(v_045_061), .acc(CC[2][95:64]));
ProcessingElement P046(.clk(clk), .rst(pe_rst), .left(h_045_046),
    .top(v_030_046), .right(h_046_047), .down(v_046_062), .acc(CC[2][63:32]));
ProcessingElement P047(.clk(clk), .rst(pe_rst), .left(h_046_047),
    .top(v_031_047), .right(), .down(v_047_063), .acc(CC[2][31:0]));

ProcessingElement P048(.clk(clk), .rst(pe_rst), .left(a[3]),
    .top(v_032_048), .right(h_048_049), .down(v_048_064), .acc(CC[3][511:480]));
ProcessingElement P049(.clk(clk), .rst(pe_rst), .left(h_048_049),
    .top(v_033_049), .right(h_049_050), .down(v_049_065), .acc(CC[3][479:448]));
ProcessingElement P050(.clk(clk), .rst(pe_rst), .left(h_049_050),
    .top(v_034_050), .right(h_050_051), .down(v_050_066), .acc(CC[3][447:416]));
ProcessingElement P051(.clk(clk), .rst(pe_rst), .left(h_050_051),
    .top(v_035_051), .right(h_051_052), .down(v_051_067), .acc(CC[3][415:384]));
ProcessingElement P052(.clk(clk), .rst(pe_rst), .left(h_051_052),
    .top(v_036_052), .right(h_052_053), .down(v_052_068), .acc(CC[3][383:352]));
ProcessingElement P053(.clk(clk), .rst(pe_rst), .left(h_052_053),
    .top(v_037_053), .right(h_053_054), .down(v_053_069), .acc(CC[3][351:320]));
ProcessingElement P054(.clk(clk), .rst(pe_rst), .left(h_053_054),
    .top(v_038_054), .right(h_054_055), .down(v_054_070), .acc(CC[3][319:288]));
ProcessingElement P055(.clk(clk), .rst(pe_rst), .left(h_054_055),
    .top(v_039_055), .right(h_055_056), .down(v_055_071), .acc(CC[3][287:256]));
ProcessingElement P056(.clk(clk), .rst(pe_rst), .left(h_055_056),
    .top(v_040_056), .right(h_056_057), .down(v_056_072), .acc(CC[3][255:224]));
ProcessingElement P057(.clk(clk), .rst(pe_rst), .left(h_056_057),
    .top(v_041_057), .right(h_057_058), .down(v_057_073), .acc(CC[3][223:192]));
ProcessingElement P058(.clk(clk), .rst(pe_rst), .left(h_057_058),
    .top(v_042_058), .right(h_058_059), .down(v_058_074), .acc(CC[3][191:160]));
ProcessingElement P059(.clk(clk), .rst(pe_rst), .left(h_058_059),
    .top(v_043_059), .right(h_059_060), .down(v_059_075), .acc(CC[3][159:128]));
ProcessingElement P060(.clk(clk), .rst(pe_rst), .left(h_059_060),
    .top(v_044_060), .right(h_060_061), .down(v_060_076), .acc(CC[3][127:96]));
ProcessingElement P061(.clk(clk), .rst(pe_rst), .left(h_060_061),
    .top(v_045_061), .right(h_061_062), .down(v_061_077), .acc(CC[3][95:64]));
ProcessingElement P062(.clk(clk), .rst(pe_rst), .left(h_061_062),
    .top(v_046_062), .right(h_062_063), .down(v_062_078), .acc(CC[3][63:32]));
ProcessingElement P063(.clk(clk), .rst(pe_rst), .left(h_062_063),
    .top(v_047_063), .right(), .down(v_063_079), .acc(CC[3][31:0]));

ProcessingElement P064(.clk(clk), .rst(pe_rst), .left(a[4]),
    .top(v_048_064), .right(h_064_065), .down(v_064_080), .acc(CC[4][511:480]));
ProcessingElement P065(.clk(clk), .rst(pe_rst), .left(h_064_065),
    .top(v_049_065), .right(h_065_066), .down(v_065_081), .acc(CC[4][479:448]));
ProcessingElement P066(.clk(clk), .rst(pe_rst), .left(h_065_066),
    .top(v_050_066), .right(h_066_067), .down(v_066_082), .acc(CC[4][447:416]));
ProcessingElement P067(.clk(clk), .rst(pe_rst), .left(h_066_067),
    .top(v_051_067), .right(h_067_068), .down(v_067_083), .acc(CC[4][415:384]));
ProcessingElement P068(.clk(clk), .rst(pe_rst), .left(h_067_068),
    .top(v_052_068), .right(h_068_069), .down(v_068_084), .acc(CC[4][383:352]));
ProcessingElement P069(.clk(clk), .rst(pe_rst), .left(h_068_069),
    .top(v_053_069), .right(h_069_070), .down(v_069_085), .acc(CC[4][351:320]));
ProcessingElement P070(.clk(clk), .rst(pe_rst), .left(h_069_070),
    .top(v_054_070), .right(h_070_071), .down(v_070_086), .acc(CC[4][319:288]));
ProcessingElement P071(.clk(clk), .rst(pe_rst), .left(h_070_071),
    .top(v_055_071), .right(h_071_072), .down(v_071_087), .acc(CC[4][287:256]));
ProcessingElement P072(.clk(clk), .rst(pe_rst), .left(h_071_072),
    .top(v_056_072), .right(h_072_073), .down(v_072_088), .acc(CC[4][255:224]));
ProcessingElement P073(.clk(clk), .rst(pe_rst), .left(h_072_073),
    .top(v_057_073), .right(h_073_074), .down(v_073_089), .acc(CC[4][223:192]));
ProcessingElement P074(.clk(clk), .rst(pe_rst), .left(h_073_074),
    .top(v_058_074), .right(h_074_075), .down(v_074_090), .acc(CC[4][191:160]));
ProcessingElement P075(.clk(clk), .rst(pe_rst), .left(h_074_075),
    .top(v_059_075), .right(h_075_076), .down(v_075_091), .acc(CC[4][159:128]));
ProcessingElement P076(.clk(clk), .rst(pe_rst), .left(h_075_076),
    .top(v_060_076), .right(h_076_077), .down(v_076_092), .acc(CC[4][127:96]));
ProcessingElement P077(.clk(clk), .rst(pe_rst), .left(h_076_077),
    .top(v_061_077), .right(h_077_078), .down(v_077_093), .acc(CC[4][95:64]));
ProcessingElement P078(.clk(clk), .rst(pe_rst), .left(h_077_078),
    .top(v_062_078), .right(h_078_079), .down(v_078_094), .acc(CC[4][63:32]));
ProcessingElement P079(.clk(clk), .rst(pe_rst), .left(h_078_079),
    .top(v_063_079), .right(), .down(v_079_095), .acc(CC[4][31:0]));

ProcessingElement P080(.clk(clk), .rst(pe_rst), .left(a[5]),
    .top(v_064_080), .right(h_080_081), .down(v_080_096), .acc(CC[5][511:480]));
ProcessingElement P081(.clk(clk), .rst(pe_rst), .left(h_080_081),
    .top(v_065_081), .right(h_081_082), .down(v_081_097), .acc(CC[5][479:448]));
ProcessingElement P082(.clk(clk), .rst(pe_rst), .left(h_081_082),
    .top(v_066_082), .right(h_082_083), .down(v_082_098), .acc(CC[5][447:416]));
ProcessingElement P083(.clk(clk), .rst(pe_rst), .left(h_082_083),
    .top(v_067_083), .right(h_083_084), .down(v_083_099), .acc(CC[5][415:384]));
ProcessingElement P084(.clk(clk), .rst(pe_rst), .left(h_083_084),
    .top(v_068_084), .right(h_084_085), .down(v_084_100), .acc(CC[5][383:352]));
ProcessingElement P085(.clk(clk), .rst(pe_rst), .left(h_084_085),
    .top(v_069_085), .right(h_085_086), .down(v_085_101), .acc(CC[5][351:320]));
ProcessingElement P086(.clk(clk), .rst(pe_rst), .left(h_085_086),
    .top(v_070_086), .right(h_086_087), .down(v_086_102), .acc(CC[5][319:288]));
ProcessingElement P087(.clk(clk), .rst(pe_rst), .left(h_086_087),
    .top(v_071_087), .right(h_087_088), .down(v_087_103), .acc(CC[5][287:256]));
ProcessingElement P088(.clk(clk), .rst(pe_rst), .left(h_087_088),
    .top(v_072_088), .right(h_088_089), .down(v_088_104), .acc(CC[5][255:224]));
ProcessingElement P089(.clk(clk), .rst(pe_rst), .left(h_088_089),
    .top(v_073_089), .right(h_089_090), .down(v_089_105), .acc(CC[5][223:192]));
ProcessingElement P090(.clk(clk), .rst(pe_rst), .left(h_089_090),
    .top(v_074_090), .right(h_090_091), .down(v_090_106), .acc(CC[5][191:160]));
ProcessingElement P091(.clk(clk), .rst(pe_rst), .left(h_090_091),
    .top(v_075_091), .right(h_091_092), .down(v_091_107), .acc(CC[5][159:128]));
ProcessingElement P092(.clk(clk), .rst(pe_rst), .left(h_091_092),
    .top(v_076_092), .right(h_092_093), .down(v_092_108), .acc(CC[5][127:96]));
ProcessingElement P093(.clk(clk), .rst(pe_rst), .left(h_092_093),
    .top(v_077_093), .right(h_093_094), .down(v_093_109), .acc(CC[5][95:64]));
ProcessingElement P094(.clk(clk), .rst(pe_rst), .left(h_093_094),
    .top(v_078_094), .right(h_094_095), .down(v_094_110), .acc(CC[5][63:32]));
ProcessingElement P095(.clk(clk), .rst(pe_rst), .left(h_094_095),
    .top(v_079_095), .right(), .down(v_095_111), .acc(CC[5][31:0]));

ProcessingElement P096(.clk(clk), .rst(pe_rst), .left(a[6]),
    .top(v_080_096), .right(h_096_097), .down(v_096_112), .acc(CC[6][511:480]));
ProcessingElement P097(.clk(clk), .rst(pe_rst), .left(h_096_097),
    .top(v_081_097), .right(h_097_098), .down(v_097_113), .acc(CC[6][479:448]));
ProcessingElement P098(.clk(clk), .rst(pe_rst), .left(h_097_098),
    .top(v_082_098), .right(h_098_099), .down(v_098_114), .acc(CC[6][447:416]));
ProcessingElement P099(.clk(clk), .rst(pe_rst), .left(h_098_099),
    .top(v_083_099), .right(h_099_100), .down(v_099_115), .acc(CC[6][415:384]));
ProcessingElement P100(.clk(clk), .rst(pe_rst), .left(h_099_100),
    .top(v_084_100), .right(h_100_101), .down(v_100_116), .acc(CC[6][383:352]));
ProcessingElement P101(.clk(clk), .rst(pe_rst), .left(h_100_101),
    .top(v_085_101), .right(h_101_102), .down(v_101_117), .acc(CC[6][351:320]));
ProcessingElement P102(.clk(clk), .rst(pe_rst), .left(h_101_102),
    .top(v_086_102), .right(h_102_103), .down(v_102_118), .acc(CC[6][319:288]));
ProcessingElement P103(.clk(clk), .rst(pe_rst), .left(h_102_103),
    .top(v_087_103), .right(h_103_104), .down(v_103_119), .acc(CC[6][287:256]));
ProcessingElement P104(.clk(clk), .rst(pe_rst), .left(h_103_104),
    .top(v_088_104), .right(h_104_105), .down(v_104_120), .acc(CC[6][255:224]));
ProcessingElement P105(.clk(clk), .rst(pe_rst), .left(h_104_105),
    .top(v_089_105), .right(h_105_106), .down(v_105_121), .acc(CC[6][223:192]));
ProcessingElement P106(.clk(clk), .rst(pe_rst), .left(h_105_106),
    .top(v_090_106), .right(h_106_107), .down(v_106_122), .acc(CC[6][191:160]));
ProcessingElement P107(.clk(clk), .rst(pe_rst), .left(h_106_107),
    .top(v_091_107), .right(h_107_108), .down(v_107_123), .acc(CC[6][159:128]));
ProcessingElement P108(.clk(clk), .rst(pe_rst), .left(h_107_108),
    .top(v_092_108), .right(h_108_109), .down(v_108_124), .acc(CC[6][127:96]));
ProcessingElement P109(.clk(clk), .rst(pe_rst), .left(h_108_109),
    .top(v_093_109), .right(h_109_110), .down(v_109_125), .acc(CC[6][95:64]));
ProcessingElement P110(.clk(clk), .rst(pe_rst), .left(h_109_110),
    .top(v_094_110), .right(h_110_111), .down(v_110_126), .acc(CC[6][63:32]));
ProcessingElement P111(.clk(clk), .rst(pe_rst), .left(h_110_111),
    .top(v_095_111), .right(), .down(v_111_127), .acc(CC[6][31:0]));

ProcessingElement P112(.clk(clk), .rst(pe_rst), .left(a[7]),
    .top(v_096_112), .right(h_112_113), .down(v_112_128), .acc(CC[7][511:480]));
ProcessingElement P113(.clk(clk), .rst(pe_rst), .left(h_112_113),
    .top(v_097_113), .right(h_113_114), .down(v_113_129), .acc(CC[7][479:448]));
ProcessingElement P114(.clk(clk), .rst(pe_rst), .left(h_113_114),
    .top(v_098_114), .right(h_114_115), .down(v_114_130), .acc(CC[7][447:416]));
ProcessingElement P115(.clk(clk), .rst(pe_rst), .left(h_114_115),
    .top(v_099_115), .right(h_115_116), .down(v_115_131), .acc(CC[7][415:384]));
ProcessingElement P116(.clk(clk), .rst(pe_rst), .left(h_115_116),
    .top(v_100_116), .right(h_116_117), .down(v_116_132), .acc(CC[7][383:352]));
ProcessingElement P117(.clk(clk), .rst(pe_rst), .left(h_116_117),
    .top(v_101_117), .right(h_117_118), .down(v_117_133), .acc(CC[7][351:320]));
ProcessingElement P118(.clk(clk), .rst(pe_rst), .left(h_117_118),
    .top(v_102_118), .right(h_118_119), .down(v_118_134), .acc(CC[7][319:288]));
ProcessingElement P119(.clk(clk), .rst(pe_rst), .left(h_118_119),
    .top(v_103_119), .right(h_119_120), .down(v_119_135), .acc(CC[7][287:256]));
ProcessingElement P120(.clk(clk), .rst(pe_rst), .left(h_119_120),
    .top(v_104_120), .right(h_120_121), .down(v_120_136), .acc(CC[7][255:224]));
ProcessingElement P121(.clk(clk), .rst(pe_rst), .left(h_120_121),
    .top(v_105_121), .right(h_121_122), .down(v_121_137), .acc(CC[7][223:192]));
ProcessingElement P122(.clk(clk), .rst(pe_rst), .left(h_121_122),
    .top(v_106_122), .right(h_122_123), .down(v_122_138), .acc(CC[7][191:160]));
ProcessingElement P123(.clk(clk), .rst(pe_rst), .left(h_122_123),
    .top(v_107_123), .right(h_123_124), .down(v_123_139), .acc(CC[7][159:128]));
ProcessingElement P124(.clk(clk), .rst(pe_rst), .left(h_123_124),
    .top(v_108_124), .right(h_124_125), .down(v_124_140), .acc(CC[7][127:96]));
ProcessingElement P125(.clk(clk), .rst(pe_rst), .left(h_124_125),
    .top(v_109_125), .right(h_125_126), .down(v_125_141), .acc(CC[7][95:64]));
ProcessingElement P126(.clk(clk), .rst(pe_rst), .left(h_125_126),
    .top(v_110_126), .right(h_126_127), .down(v_126_142), .acc(CC[7][63:32]));
ProcessingElement P127(.clk(clk), .rst(pe_rst), .left(h_126_127),
    .top(v_111_127), .right(), .down(v_127_143), .acc(CC[7][31:0]));

ProcessingElement P128(.clk(clk), .rst(pe_rst), .left(a[8]),
    .top(v_112_128), .right(h_128_129), .down(v_128_144), .acc(CC[8][511:480]));
ProcessingElement P129(.clk(clk), .rst(pe_rst), .left(h_128_129),
    .top(v_113_129), .right(h_129_130), .down(v_129_145), .acc(CC[8][479:448]));
ProcessingElement P130(.clk(clk), .rst(pe_rst), .left(h_129_130),
    .top(v_114_130), .right(h_130_131), .down(v_130_146), .acc(CC[8][447:416]));
ProcessingElement P131(.clk(clk), .rst(pe_rst), .left(h_130_131),
    .top(v_115_131), .right(h_131_132), .down(v_131_147), .acc(CC[8][415:384]));
ProcessingElement P132(.clk(clk), .rst(pe_rst), .left(h_131_132),
    .top(v_116_132), .right(h_132_133), .down(v_132_148), .acc(CC[8][383:352]));
ProcessingElement P133(.clk(clk), .rst(pe_rst), .left(h_132_133),
    .top(v_117_133), .right(h_133_134), .down(v_133_149), .acc(CC[8][351:320]));
ProcessingElement P134(.clk(clk), .rst(pe_rst), .left(h_133_134),
    .top(v_118_134), .right(h_134_135), .down(v_134_150), .acc(CC[8][319:288]));
ProcessingElement P135(.clk(clk), .rst(pe_rst), .left(h_134_135),
    .top(v_119_135), .right(h_135_136), .down(v_135_151), .acc(CC[8][287:256]));
ProcessingElement P136(.clk(clk), .rst(pe_rst), .left(h_135_136),
    .top(v_120_136), .right(h_136_137), .down(v_136_152), .acc(CC[8][255:224]));
ProcessingElement P137(.clk(clk), .rst(pe_rst), .left(h_136_137),
    .top(v_121_137), .right(h_137_138), .down(v_137_153), .acc(CC[8][223:192]));
ProcessingElement P138(.clk(clk), .rst(pe_rst), .left(h_137_138),
    .top(v_122_138), .right(h_138_139), .down(v_138_154), .acc(CC[8][191:160]));
ProcessingElement P139(.clk(clk), .rst(pe_rst), .left(h_138_139),
    .top(v_123_139), .right(h_139_140), .down(v_139_155), .acc(CC[8][159:128]));
ProcessingElement P140(.clk(clk), .rst(pe_rst), .left(h_139_140),
    .top(v_124_140), .right(h_140_141), .down(v_140_156), .acc(CC[8][127:96]));
ProcessingElement P141(.clk(clk), .rst(pe_rst), .left(h_140_141),
    .top(v_125_141), .right(h_141_142), .down(v_141_157), .acc(CC[8][95:64]));
ProcessingElement P142(.clk(clk), .rst(pe_rst), .left(h_141_142),
    .top(v_126_142), .right(h_142_143), .down(v_142_158), .acc(CC[8][63:32]));
ProcessingElement P143(.clk(clk), .rst(pe_rst), .left(h_142_143),
    .top(v_127_143), .right(), .down(v_143_159), .acc(CC[8][31:0]));

ProcessingElement P144(.clk(clk), .rst(pe_rst), .left(a[9]),
    .top(v_128_144), .right(h_144_145), .down(v_144_160), .acc(CC[9][511:480]));
ProcessingElement P145(.clk(clk), .rst(pe_rst), .left(h_144_145),
    .top(v_129_145), .right(h_145_146), .down(v_145_161), .acc(CC[9][479:448]));
ProcessingElement P146(.clk(clk), .rst(pe_rst), .left(h_145_146),
    .top(v_130_146), .right(h_146_147), .down(v_146_162), .acc(CC[9][447:416]));
ProcessingElement P147(.clk(clk), .rst(pe_rst), .left(h_146_147),
    .top(v_131_147), .right(h_147_148), .down(v_147_163), .acc(CC[9][415:384]));
ProcessingElement P148(.clk(clk), .rst(pe_rst), .left(h_147_148),
    .top(v_132_148), .right(h_148_149), .down(v_148_164), .acc(CC[9][383:352]));
ProcessingElement P149(.clk(clk), .rst(pe_rst), .left(h_148_149),
    .top(v_133_149), .right(h_149_150), .down(v_149_165), .acc(CC[9][351:320]));
ProcessingElement P150(.clk(clk), .rst(pe_rst), .left(h_149_150),
    .top(v_134_150), .right(h_150_151), .down(v_150_166), .acc(CC[9][319:288]));
ProcessingElement P151(.clk(clk), .rst(pe_rst), .left(h_150_151),
    .top(v_135_151), .right(h_151_152), .down(v_151_167), .acc(CC[9][287:256]));
ProcessingElement P152(.clk(clk), .rst(pe_rst), .left(h_151_152),
    .top(v_136_152), .right(h_152_153), .down(v_152_168), .acc(CC[9][255:224]));
ProcessingElement P153(.clk(clk), .rst(pe_rst), .left(h_152_153),
    .top(v_137_153), .right(h_153_154), .down(v_153_169), .acc(CC[9][223:192]));
ProcessingElement P154(.clk(clk), .rst(pe_rst), .left(h_153_154),
    .top(v_138_154), .right(h_154_155), .down(v_154_170), .acc(CC[9][191:160]));
ProcessingElement P155(.clk(clk), .rst(pe_rst), .left(h_154_155),
    .top(v_139_155), .right(h_155_156), .down(v_155_171), .acc(CC[9][159:128]));
ProcessingElement P156(.clk(clk), .rst(pe_rst), .left(h_155_156),
    .top(v_140_156), .right(h_156_157), .down(v_156_172), .acc(CC[9][127:96]));
ProcessingElement P157(.clk(clk), .rst(pe_rst), .left(h_156_157),
    .top(v_141_157), .right(h_157_158), .down(v_157_173), .acc(CC[9][95:64]));
ProcessingElement P158(.clk(clk), .rst(pe_rst), .left(h_157_158),
    .top(v_142_158), .right(h_158_159), .down(v_158_174), .acc(CC[9][63:32]));
ProcessingElement P159(.clk(clk), .rst(pe_rst), .left(h_158_159),
    .top(v_143_159), .right(), .down(v_159_175), .acc(CC[9][31:0]));

ProcessingElement P160(.clk(clk), .rst(pe_rst), .left(a[10]),
    .top(v_144_160), .right(h_160_161), .down(v_160_176), .acc(CC[10][511:480]));
ProcessingElement P161(.clk(clk), .rst(pe_rst), .left(h_160_161),
    .top(v_145_161), .right(h_161_162), .down(v_161_177), .acc(CC[10][479:448]));
ProcessingElement P162(.clk(clk), .rst(pe_rst), .left(h_161_162),
    .top(v_146_162), .right(h_162_163), .down(v_162_178), .acc(CC[10][447:416]));
ProcessingElement P163(.clk(clk), .rst(pe_rst), .left(h_162_163),
    .top(v_147_163), .right(h_163_164), .down(v_163_179), .acc(CC[10][415:384]));
ProcessingElement P164(.clk(clk), .rst(pe_rst), .left(h_163_164),
    .top(v_148_164), .right(h_164_165), .down(v_164_180), .acc(CC[10][383:352]));
ProcessingElement P165(.clk(clk), .rst(pe_rst), .left(h_164_165),
    .top(v_149_165), .right(h_165_166), .down(v_165_181), .acc(CC[10][351:320]));
ProcessingElement P166(.clk(clk), .rst(pe_rst), .left(h_165_166),
    .top(v_150_166), .right(h_166_167), .down(v_166_182), .acc(CC[10][319:288]));
ProcessingElement P167(.clk(clk), .rst(pe_rst), .left(h_166_167),
    .top(v_151_167), .right(h_167_168), .down(v_167_183), .acc(CC[10][287:256]));
ProcessingElement P168(.clk(clk), .rst(pe_rst), .left(h_167_168),
    .top(v_152_168), .right(h_168_169), .down(v_168_184), .acc(CC[10][255:224]));
ProcessingElement P169(.clk(clk), .rst(pe_rst), .left(h_168_169),
    .top(v_153_169), .right(h_169_170), .down(v_169_185), .acc(CC[10][223:192]));
ProcessingElement P170(.clk(clk), .rst(pe_rst), .left(h_169_170),
    .top(v_154_170), .right(h_170_171), .down(v_170_186), .acc(CC[10][191:160]));
ProcessingElement P171(.clk(clk), .rst(pe_rst), .left(h_170_171),
    .top(v_155_171), .right(h_171_172), .down(v_171_187), .acc(CC[10][159:128]));
ProcessingElement P172(.clk(clk), .rst(pe_rst), .left(h_171_172),
    .top(v_156_172), .right(h_172_173), .down(v_172_188), .acc(CC[10][127:96]));
ProcessingElement P173(.clk(clk), .rst(pe_rst), .left(h_172_173),
    .top(v_157_173), .right(h_173_174), .down(v_173_189), .acc(CC[10][95:64]));
ProcessingElement P174(.clk(clk), .rst(pe_rst), .left(h_173_174),
    .top(v_158_174), .right(h_174_175), .down(v_174_190), .acc(CC[10][63:32]));
ProcessingElement P175(.clk(clk), .rst(pe_rst), .left(h_174_175),
    .top(v_159_175), .right(), .down(v_175_191), .acc(CC[10][31:0]));

ProcessingElement P176(.clk(clk), .rst(pe_rst), .left(a[11]),
    .top(v_160_176), .right(h_176_177), .down(v_176_192), .acc(CC[11][511:480]));
ProcessingElement P177(.clk(clk), .rst(pe_rst), .left(h_176_177),
    .top(v_161_177), .right(h_177_178), .down(v_177_193), .acc(CC[11][479:448]));
ProcessingElement P178(.clk(clk), .rst(pe_rst), .left(h_177_178),
    .top(v_162_178), .right(h_178_179), .down(v_178_194), .acc(CC[11][447:416]));
ProcessingElement P179(.clk(clk), .rst(pe_rst), .left(h_178_179),
    .top(v_163_179), .right(h_179_180), .down(v_179_195), .acc(CC[11][415:384]));
ProcessingElement P180(.clk(clk), .rst(pe_rst), .left(h_179_180),
    .top(v_164_180), .right(h_180_181), .down(v_180_196), .acc(CC[11][383:352]));
ProcessingElement P181(.clk(clk), .rst(pe_rst), .left(h_180_181),
    .top(v_165_181), .right(h_181_182), .down(v_181_197), .acc(CC[11][351:320]));
ProcessingElement P182(.clk(clk), .rst(pe_rst), .left(h_181_182),
    .top(v_166_182), .right(h_182_183), .down(v_182_198), .acc(CC[11][319:288]));
ProcessingElement P183(.clk(clk), .rst(pe_rst), .left(h_182_183),
    .top(v_167_183), .right(h_183_184), .down(v_183_199), .acc(CC[11][287:256]));
ProcessingElement P184(.clk(clk), .rst(pe_rst), .left(h_183_184),
    .top(v_168_184), .right(h_184_185), .down(v_184_200), .acc(CC[11][255:224]));
ProcessingElement P185(.clk(clk), .rst(pe_rst), .left(h_184_185),
    .top(v_169_185), .right(h_185_186), .down(v_185_201), .acc(CC[11][223:192]));
ProcessingElement P186(.clk(clk), .rst(pe_rst), .left(h_185_186),
    .top(v_170_186), .right(h_186_187), .down(v_186_202), .acc(CC[11][191:160]));
ProcessingElement P187(.clk(clk), .rst(pe_rst), .left(h_186_187),
    .top(v_171_187), .right(h_187_188), .down(v_187_203), .acc(CC[11][159:128]));
ProcessingElement P188(.clk(clk), .rst(pe_rst), .left(h_187_188),
    .top(v_172_188), .right(h_188_189), .down(v_188_204), .acc(CC[11][127:96]));
ProcessingElement P189(.clk(clk), .rst(pe_rst), .left(h_188_189),
    .top(v_173_189), .right(h_189_190), .down(v_189_205), .acc(CC[11][95:64]));
ProcessingElement P190(.clk(clk), .rst(pe_rst), .left(h_189_190),
    .top(v_174_190), .right(h_190_191), .down(v_190_206), .acc(CC[11][63:32]));
ProcessingElement P191(.clk(clk), .rst(pe_rst), .left(h_190_191),
    .top(v_175_191), .right(), .down(v_191_207), .acc(CC[11][31:0]));

ProcessingElement P192(.clk(clk), .rst(pe_rst), .left(a[12]),
    .top(v_176_192), .right(h_192_193), .down(v_192_208), .acc(CC[12][511:480]));
ProcessingElement P193(.clk(clk), .rst(pe_rst), .left(h_192_193),
    .top(v_177_193), .right(h_193_194), .down(v_193_209), .acc(CC[12][479:448]));
ProcessingElement P194(.clk(clk), .rst(pe_rst), .left(h_193_194),
    .top(v_178_194), .right(h_194_195), .down(v_194_210), .acc(CC[12][447:416]));
ProcessingElement P195(.clk(clk), .rst(pe_rst), .left(h_194_195),
    .top(v_179_195), .right(h_195_196), .down(v_195_211), .acc(CC[12][415:384]));
ProcessingElement P196(.clk(clk), .rst(pe_rst), .left(h_195_196),
    .top(v_180_196), .right(h_196_197), .down(v_196_212), .acc(CC[12][383:352]));
ProcessingElement P197(.clk(clk), .rst(pe_rst), .left(h_196_197),
    .top(v_181_197), .right(h_197_198), .down(v_197_213), .acc(CC[12][351:320]));
ProcessingElement P198(.clk(clk), .rst(pe_rst), .left(h_197_198),
    .top(v_182_198), .right(h_198_199), .down(v_198_214), .acc(CC[12][319:288]));
ProcessingElement P199(.clk(clk), .rst(pe_rst), .left(h_198_199),
    .top(v_183_199), .right(h_199_200), .down(v_199_215), .acc(CC[12][287:256]));
ProcessingElement P200(.clk(clk), .rst(pe_rst), .left(h_199_200),
    .top(v_184_200), .right(h_200_201), .down(v_200_216), .acc(CC[12][255:224]));
ProcessingElement P201(.clk(clk), .rst(pe_rst), .left(h_200_201),
    .top(v_185_201), .right(h_201_202), .down(v_201_217), .acc(CC[12][223:192]));
ProcessingElement P202(.clk(clk), .rst(pe_rst), .left(h_201_202),
    .top(v_186_202), .right(h_202_203), .down(v_202_218), .acc(CC[12][191:160]));
ProcessingElement P203(.clk(clk), .rst(pe_rst), .left(h_202_203),
    .top(v_187_203), .right(h_203_204), .down(v_203_219), .acc(CC[12][159:128]));
ProcessingElement P204(.clk(clk), .rst(pe_rst), .left(h_203_204),
    .top(v_188_204), .right(h_204_205), .down(v_204_220), .acc(CC[12][127:96]));
ProcessingElement P205(.clk(clk), .rst(pe_rst), .left(h_204_205),
    .top(v_189_205), .right(h_205_206), .down(v_205_221), .acc(CC[12][95:64]));
ProcessingElement P206(.clk(clk), .rst(pe_rst), .left(h_205_206),
    .top(v_190_206), .right(h_206_207), .down(v_206_222), .acc(CC[12][63:32]));
ProcessingElement P207(.clk(clk), .rst(pe_rst), .left(h_206_207),
    .top(v_191_207), .right(), .down(v_207_223), .acc(CC[12][31:0]));

ProcessingElement P208(.clk(clk), .rst(pe_rst), .left(a[13]),
    .top(v_192_208), .right(h_208_209), .down(v_208_224), .acc(CC[13][511:480]));
ProcessingElement P209(.clk(clk), .rst(pe_rst), .left(h_208_209),
    .top(v_193_209), .right(h_209_210), .down(v_209_225), .acc(CC[13][479:448]));
ProcessingElement P210(.clk(clk), .rst(pe_rst), .left(h_209_210),
    .top(v_194_210), .right(h_210_211), .down(v_210_226), .acc(CC[13][447:416]));
ProcessingElement P211(.clk(clk), .rst(pe_rst), .left(h_210_211),
    .top(v_195_211), .right(h_211_212), .down(v_211_227), .acc(CC[13][415:384]));
ProcessingElement P212(.clk(clk), .rst(pe_rst), .left(h_211_212),
    .top(v_196_212), .right(h_212_213), .down(v_212_228), .acc(CC[13][383:352]));
ProcessingElement P213(.clk(clk), .rst(pe_rst), .left(h_212_213),
    .top(v_197_213), .right(h_213_214), .down(v_213_229), .acc(CC[13][351:320]));
ProcessingElement P214(.clk(clk), .rst(pe_rst), .left(h_213_214),
    .top(v_198_214), .right(h_214_215), .down(v_214_230), .acc(CC[13][319:288]));
ProcessingElement P215(.clk(clk), .rst(pe_rst), .left(h_214_215),
    .top(v_199_215), .right(h_215_216), .down(v_215_231), .acc(CC[13][287:256]));
ProcessingElement P216(.clk(clk), .rst(pe_rst), .left(h_215_216),
    .top(v_200_216), .right(h_216_217), .down(v_216_232), .acc(CC[13][255:224]));
ProcessingElement P217(.clk(clk), .rst(pe_rst), .left(h_216_217),
    .top(v_201_217), .right(h_217_218), .down(v_217_233), .acc(CC[13][223:192]));
ProcessingElement P218(.clk(clk), .rst(pe_rst), .left(h_217_218),
    .top(v_202_218), .right(h_218_219), .down(v_218_234), .acc(CC[13][191:160]));
ProcessingElement P219(.clk(clk), .rst(pe_rst), .left(h_218_219),
    .top(v_203_219), .right(h_219_220), .down(v_219_235), .acc(CC[13][159:128]));
ProcessingElement P220(.clk(clk), .rst(pe_rst), .left(h_219_220),
    .top(v_204_220), .right(h_220_221), .down(v_220_236), .acc(CC[13][127:96]));
ProcessingElement P221(.clk(clk), .rst(pe_rst), .left(h_220_221),
    .top(v_205_221), .right(h_221_222), .down(v_221_237), .acc(CC[13][95:64]));
ProcessingElement P222(.clk(clk), .rst(pe_rst), .left(h_221_222),
    .top(v_206_222), .right(h_222_223), .down(v_222_238), .acc(CC[13][63:32]));
ProcessingElement P223(.clk(clk), .rst(pe_rst), .left(h_222_223),
    .top(v_207_223), .right(), .down(v_223_239), .acc(CC[13][31:0]));

ProcessingElement P224(.clk(clk), .rst(pe_rst), .left(a[14]),
    .top(v_208_224), .right(h_224_225), .down(v_224_240), .acc(CC[14][511:480]));
ProcessingElement P225(.clk(clk), .rst(pe_rst), .left(h_224_225),
    .top(v_209_225), .right(h_225_226), .down(v_225_241), .acc(CC[14][479:448]));
ProcessingElement P226(.clk(clk), .rst(pe_rst), .left(h_225_226),
    .top(v_210_226), .right(h_226_227), .down(v_226_242), .acc(CC[14][447:416]));
ProcessingElement P227(.clk(clk), .rst(pe_rst), .left(h_226_227),
    .top(v_211_227), .right(h_227_228), .down(v_227_243), .acc(CC[14][415:384]));
ProcessingElement P228(.clk(clk), .rst(pe_rst), .left(h_227_228),
    .top(v_212_228), .right(h_228_229), .down(v_228_244), .acc(CC[14][383:352]));
ProcessingElement P229(.clk(clk), .rst(pe_rst), .left(h_228_229),
    .top(v_213_229), .right(h_229_230), .down(v_229_245), .acc(CC[14][351:320]));
ProcessingElement P230(.clk(clk), .rst(pe_rst), .left(h_229_230),
    .top(v_214_230), .right(h_230_231), .down(v_230_246), .acc(CC[14][319:288]));
ProcessingElement P231(.clk(clk), .rst(pe_rst), .left(h_230_231),
    .top(v_215_231), .right(h_231_232), .down(v_231_247), .acc(CC[14][287:256]));
ProcessingElement P232(.clk(clk), .rst(pe_rst), .left(h_231_232),
    .top(v_216_232), .right(h_232_233), .down(v_232_248), .acc(CC[14][255:224]));
ProcessingElement P233(.clk(clk), .rst(pe_rst), .left(h_232_233),
    .top(v_217_233), .right(h_233_234), .down(v_233_249), .acc(CC[14][223:192]));
ProcessingElement P234(.clk(clk), .rst(pe_rst), .left(h_233_234),
    .top(v_218_234), .right(h_234_235), .down(v_234_250), .acc(CC[14][191:160]));
ProcessingElement P235(.clk(clk), .rst(pe_rst), .left(h_234_235),
    .top(v_219_235), .right(h_235_236), .down(v_235_251), .acc(CC[14][159:128]));
ProcessingElement P236(.clk(clk), .rst(pe_rst), .left(h_235_236),
    .top(v_220_236), .right(h_236_237), .down(v_236_252), .acc(CC[14][127:96]));
ProcessingElement P237(.clk(clk), .rst(pe_rst), .left(h_236_237),
    .top(v_221_237), .right(h_237_238), .down(v_237_253), .acc(CC[14][95:64]));
ProcessingElement P238(.clk(clk), .rst(pe_rst), .left(h_237_238),
    .top(v_222_238), .right(h_238_239), .down(v_238_254), .acc(CC[14][63:32]));
ProcessingElement P239(.clk(clk), .rst(pe_rst), .left(h_238_239),
    .top(v_223_239), .right(), .down(v_239_255), .acc(CC[14][31:0]));

ProcessingElement P240(.clk(clk), .rst(pe_rst), .left(a[15]),
    .top(v_224_240), .right(h_240_241), .down(), .acc(CC[15][511:480]));
ProcessingElement P241(.clk(clk), .rst(pe_rst), .left(h_240_241),
    .top(v_225_241), .right(h_241_242), .down(), .acc(CC[15][479:448]));
ProcessingElement P242(.clk(clk), .rst(pe_rst), .left(h_241_242),
    .top(v_226_242), .right(h_242_243), .down(), .acc(CC[15][447:416]));
ProcessingElement P243(.clk(clk), .rst(pe_rst), .left(h_242_243),
    .top(v_227_243), .right(h_243_244), .down(), .acc(CC[15][415:384]));
ProcessingElement P244(.clk(clk), .rst(pe_rst), .left(h_243_244),
    .top(v_228_244), .right(h_244_245), .down(), .acc(CC[15][383:352]));
ProcessingElement P245(.clk(clk), .rst(pe_rst), .left(h_244_245),
    .top(v_229_245), .right(h_245_246), .down(), .acc(CC[15][351:320]));
ProcessingElement P246(.clk(clk), .rst(pe_rst), .left(h_245_246),
    .top(v_230_246), .right(h_246_247), .down(), .acc(CC[15][319:288]));
ProcessingElement P247(.clk(clk), .rst(pe_rst), .left(h_246_247),
    .top(v_231_247), .right(h_247_248), .down(), .acc(CC[15][287:256]));
ProcessingElement P248(.clk(clk), .rst(pe_rst), .left(h_247_248),
    .top(v_232_248), .right(h_248_249), .down(), .acc(CC[15][255:224]));
ProcessingElement P249(.clk(clk), .rst(pe_rst), .left(h_248_249),
    .top(v_233_249), .right(h_249_250), .down(), .acc(CC[15][223:192]));
ProcessingElement P250(.clk(clk), .rst(pe_rst), .left(h_249_250),
    .top(v_234_250), .right(h_250_251), .down(), .acc(CC[15][191:160]));
ProcessingElement P251(.clk(clk), .rst(pe_rst), .left(h_250_251),
    .top(v_235_251), .right(h_251_252), .down(), .acc(CC[15][159:128]));
ProcessingElement P252(.clk(clk), .rst(pe_rst), .left(h_251_252),
    .top(v_236_252), .right(h_252_253), .down(), .acc(CC[15][127:96]));
ProcessingElement P253(.clk(clk), .rst(pe_rst), .left(h_252_253),
    .top(v_237_253), .right(h_253_254), .down(), .acc(CC[15][95:64]));
ProcessingElement P254(.clk(clk), .rst(pe_rst), .left(h_253_254),
    .top(v_238_254), .right(h_254_255), .down(), .acc(CC[15][63:32]));
ProcessingElement P255(.clk(clk), .rst(pe_rst), .left(h_254_255),
    .top(v_239_255), .right(), .down(), .acc(CC[15][31:0]));

// Implement control logic

assign cnt_M = (M_r + 15) >> 4;
assign cnt_N = (N_r + 15) >> 4;

always @(posedge clk) begin
    if (in_valid) begin
        busy <= 1;
        M_r <= M; K_r <= K; N_r <= N; offset_r <= offset;
        pe_rst <= 1; acc_en_r <= acc_en;

        BB[0]  <= 0; BB[1]  <= 0; BB[2]  <= 0; BB[3]  <= 0;
        BB[4]  <= 0; BB[5]  <= 0; BB[6]  <= 0; BB[7]  <= 0;
        BB[8]  <= 0; BB[9]  <= 0; BB[10] <= 0; BB[11] <= 0;
        BB[12] <= 0; BB[13] <= 0; BB[14] <= 0; BB[15] <= 0;

        A_index <= 0; B_index <= 0;
        m <= 0; k <= 0; n <= 0;

        state <= 66'b000000000100100011010001010110011110001001101010111100110111101111;
    end else if (busy) begin
        case(state[65:64])
            2'b00: begin
                pe_rst <= 0;
                C_wr_en <= 0;

                AA[state[63:60]] <= k < K_r ? A_data_out[119:0] : 0;
                BB[state[63:60]] <= k < K_r ? B_data_out[119:0] : 0;

                a[0]  <= k < K_r ? $signed(A_data_out[127:120]) + $signed(offset_r) : 0;
                a[1]  <= $signed(AA[state[3:0]][119:112]) + $signed(offset_r);
                a[2]  <= $signed(AA[state[7:4]][111:104]) + $signed(offset_r);
                a[3]  <= $signed(AA[state[11:8]][103:96]) + $signed(offset_r);
                a[4]  <= $signed(AA[state[15:12]][95:88]) + $signed(offset_r);
                a[5]  <= $signed(AA[state[19:16]][87:80]) + $signed(offset_r);
                a[6]  <= $signed(AA[state[23:20]][79:72]) + $signed(offset_r);
                a[7]  <= $signed(AA[state[27:24]][71:64]) + $signed(offset_r);
                a[8]  <= $signed(AA[state[31:28]][63:56]) + $signed(offset_r);
                a[9]  <= $signed(AA[state[35:32]][55:48]) + $signed(offset_r);
                a[10] <= $signed(AA[state[39:36]][47:40]) + $signed(offset_r);
                a[11] <= $signed(AA[state[43:40]][39:32]) + $signed(offset_r);
                a[12] <= $signed(AA[state[47:44]][31:24]) + $signed(offset_r);
                a[13] <= $signed(AA[state[51:48]][23:16]) + $signed(offset_r);
                a[14] <= $signed(AA[state[55:52]][15:8]) + $signed(offset_r);
                a[15] <= $signed(AA[state[59:56]][7:0]) + $signed(offset_r);

                b[0]  <= k < K_r ? B_data_out[127:120] : 0;
                b[1]  <= BB[state[3:0]][119:112];
                b[2]  <= BB[state[7:4]][111:104];
                b[3]  <= BB[state[11:8]][103:96];
                b[4]  <= BB[state[15:12]][95:88];
                b[5]  <= BB[state[19:16]][87:80];
                b[6]  <= BB[state[23:20]][79:72];
                b[7]  <= BB[state[27:24]][71:64];
                b[8]  <= BB[state[31:28]][63:56];
                b[9]  <= BB[state[35:32]][55:48];
                b[10] <= BB[state[39:36]][47:40];
                b[11] <= BB[state[43:40]][39:32];
                b[12] <= BB[state[47:44]][31:24];
                b[13] <= BB[state[51:48]][23:16];
                b[14] <= BB[state[55:52]][15:8];
                b[15] <= BB[state[59:56]][7:0];

                A_index <= m*K_r + k + 1;
                B_index <= n*K_r + k + 1;
                k <= k + 1;

                if (k + 1 < K_r + 16) begin
                    state <= {2'b00, state[59:0], state[63:60]};
                end else begin
                    state <= 66'b010000000100100011010001010110011110001001101010111100110111101111;
                    C_index <= n*M_r + m*16;
                end
            end
            2'b01: begin
                C_wr_en <= 1;
                C_index <= n*M_r + m*16 + state[63:60] + 1;

                C_data_in[511:480] <= CC[state[63:60]][511:480] +
                    (acc_en_r ? C_data_out[511:480] : 0);
                C_data_in[479:448] <= CC[state[63:60]][479:448] +
                    (acc_en_r ? C_data_out[479:448] : 0);
                C_data_in[447:416] <= CC[state[63:60]][447:416] +
                    (acc_en_r ? C_data_out[447:416] : 0);
                C_data_in[415:384] <= CC[state[63:60]][415:384] +
                    (acc_en_r ? C_data_out[415:384] : 0);
                C_data_in[383:352] <= CC[state[63:60]][383:352] +
                    (acc_en_r ? C_data_out[383:352] : 0);
                C_data_in[351:320] <= CC[state[63:60]][351:320] +
                    (acc_en_r ? C_data_out[351:320] : 0);
                C_data_in[319:288] <= CC[state[63:60]][319:288] +
                    (acc_en_r ? C_data_out[319:288] : 0);
                C_data_in[287:256] <= CC[state[63:60]][287:256] +
                    (acc_en_r ? C_data_out[287:256] : 0);
                C_data_in[255:224] <= CC[state[63:60]][255:224] +
                    (acc_en_r ? C_data_out[255:224] : 0);
                C_data_in[223:192] <= CC[state[63:60]][223:192] +
                    (acc_en_r ? C_data_out[223:192] : 0);
                C_data_in[191:160] <= CC[state[63:60]][191:160] +
                    (acc_en_r ? C_data_out[191:160] : 0);
                C_data_in[159:128] <= CC[state[63:60]][159:128] +
                    (acc_en_r ? C_data_out[159:128] : 0);
                C_data_in[127:96] <= CC[state[63:60]][127:96] +
                    (acc_en_r ? C_data_out[127:96] : 0);
                C_data_in[95:64] <= CC[state[63:60]][95:64] +
                    (acc_en_r ? C_data_out[95:64] : 0);
                C_data_in[63:32] <= CC[state[63:60]][63:32] +
                    (acc_en_r ? C_data_out[63:32] : 0);
                C_data_in[31:0] <= CC[state[63:60]][31:0] +
                    (acc_en_r ? C_data_out[31:0] : 0);

                if (m*16 + state[59:56] >= M_r || state[63:60] == 4'b1111) begin
                    pe_rst <= 1;

                    A_index <= n + 1 < cnt_N ? m * K_r : (m + 1) * K_r;
                    B_index <= n + 1 < cnt_N ? (n + 1) * K_r : 0;
                    k <= 0;
                    n <= n + 1 < cnt_N ? n + 1 : 0;
                    m <= n + 1 < cnt_N ? m : m + 1;

                    state <= n + 1 == cnt_N && m + 1 == cnt_M
                        ? 66'b110000000000000000000000000000000000000000000000000000000000000000
                        : 66'b000000000100100011010001010110011110001001101010111100110111101111;
                end else begin
                    state <= {2'b01, state[59:0], state[63:60]};
                end
            end
            2'b11: begin
                C_wr_en <= 0;
                C_index <= 0;
                busy <= 0;
            end
        endcase
    end
end

endmodule

module ProcessingElement(
    input               clk,
    input               rst,
    input [8:0]         left,
    input [7:0]         top,
    output reg [8:0]    right,
    output reg [7:0]    down,
    output reg [31:0]   acc
);

always @(posedge clk) begin
    if (rst) begin
        right <= 0; down <= 0; acc <= 0;
    end else begin
        acc <= $signed(acc) + $signed(left)*$signed(top);
        right <= left;
        down <= top;
    end
end

endmodule

module ReadBuffer #(parameter ADDR_BITS=8, parameter DATA_BITS=8)(
    input                       clk,
    input                       wr_en,
    input      [ADDR_BITS-1:0]  index,
    input      [DATA_BITS-1:0]  data_in,
    output reg [DATA_BITS-1:0]  data_out
);

parameter DEPTH = 2 ** ADDR_BITS;

reg [DATA_BITS-1:0] gbuff [DEPTH-1:0];

always @(negedge clk) begin
    if (wr_en) begin
        gbuff[index] <= data_in;
    end else begin
        data_out <= gbuff[index];
    end
end

endmodule

module AccumulationBuffer #(parameter ADDR_BITS=8, parameter DATA_BITS=8)(
    input                       clk,
    input                       wr_en,
    input      [ADDR_BITS-1:0]  index,
    input      [DATA_BITS-1:0]  data_in,
    output reg [DATA_BITS-1:0]  data_out
);

parameter DEPTH = 2 ** ADDR_BITS;

reg [DATA_BITS-1:0] gbuff [DEPTH-1:0];

always @(negedge clk) begin
    if (wr_en) begin
        gbuff[index-1] <= data_in;
    end

    data_out <= gbuff[index];
end

endmodule
