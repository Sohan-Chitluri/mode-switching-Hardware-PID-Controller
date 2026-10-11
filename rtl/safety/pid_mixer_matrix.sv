//==============================================================================
// Module: pid_mixer_matrix.sv
// Description: Synthesizable Configurable Multi-Rotor Airframe Mixer Matrix.
//              Converts 4-axis control demands (collective thrust, roll, pitch, yaw)
//              into up to 8 individual motor throttle commands for:
//              - Mode 0: Quad-X (4 motors)
//              - Mode 1: Quad-+ (4 motors)
//              - Mode 2: Hexa-X (6 motors)
//              - Mode 3: Hexa-+ (6 motors)
//              - Mode 4: Octo-X (8 motors)
//
// Compliant with Betaflight & PX4 standard airframe mixing conventions.
// All motors clamped to [0.0, 1.0] in fixed-point format with saturation flags.
//==============================================================================

`ifndef PID_MIXER_MATRIX_SV
`define PID_MIXER_MATRIX_SV

`include "pid_pkg.sv"
import pid_pkg::*;

module pid_mixer_matrix #(
    parameter int DATA_W  = 32,
    parameter int FRACT_W = 16,
    parameter int ACCUM_W = 48,
    parameter int NUM_MOTORS = 8
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     sample_valid,

    // Airframe Selection Register
    // 0 = Quad-X, 1 = Quad-+, 2 = Hexa-X, 3 = Hexa-+, 4 = Octo-X
    input  logic [2:0]               airframe_type,

    // Controller Demands (Q16.16 format)
    input  logic signed [DATA_W-1:0] u_roll,    // Roll torque demand  [-1.0, +1.0]
    input  logic signed [DATA_W-1:0] u_pitch,   // Pitch torque demand [-1.0, +1.0]
    input  logic signed [DATA_W-1:0] u_yaw,     // Yaw torque demand   [-1.0, +1.0]
    input  logic signed [DATA_W-1:0] u_thrust,  // Collective thrust   [ 0.0, +1.0]

    // Actuator Motor Outputs (Q16.16 format, saturated to [0.0, 1.0])
    output logic                     motor_valid,
    output logic signed [DATA_W-1:0] motors [NUM_MOTORS],

    // Telemetry Status
    output logic                     saturated_flag
);

    localparam signed [ACCUM_W-1:0] CLAMP_MAX = {{ACCUM_W-DATA_W{1'b0}}, FIX_ONE};
    localparam signed [ACCUM_W-1:0] CLAMP_MIN = '0;

    // Sign extension helpers
    wire signed [ACCUM_W-1:0] ext_thrust = {{ACCUM_W-DATA_W{u_thrust[DATA_W-1]}}, u_thrust};
    wire signed [ACCUM_W-1:0] ext_pitch  = {{ACCUM_W-DATA_W{u_pitch[DATA_W-1]}},  u_pitch};
    wire signed [ACCUM_W-1:0] ext_roll   = {{ACCUM_W-DATA_W{u_roll[DATA_W-1]}},   u_roll};
    wire signed [ACCUM_W-1:0] ext_yaw    = {{ACCUM_W-DATA_W{u_yaw[DATA_W-1]}},    u_yaw};

    // Scaled trigonometrics for Hexa (sin 30 = 0.5, cos 30 = 0.866)
    // 0.866 in Q16.16 = 56755 (0x0000_DDDB)
    localparam signed [DATA_W-1:0] COS30_Q16 = 32'd56755;
    wire signed [ACCUM_W-1:0] ext_roll_half  = ext_roll >>> 1;
    wire signed [ACCUM_W-1:0] ext_pitch_half = ext_pitch >>> 1;

    // Combinational motor sum generation
    logic signed [ACCUM_W-1:0] raw_sums [NUM_MOTORS];
    logic any_sat;

    always_comb begin
        for (int i = 0; i < NUM_MOTORS; i++) begin
            raw_sums[i] = '0;
        end

        case (airframe_type)
            3'b000: begin // Quad-X
                raw_sums[0] = ext_thrust - ext_pitch + ext_roll - ext_yaw; // M1: RL
                raw_sums[1] = ext_thrust - ext_pitch - ext_roll + ext_yaw; // M2: RR
                raw_sums[2] = ext_thrust + ext_pitch - ext_roll - ext_yaw; // M3: FR
                raw_sums[3] = ext_thrust + ext_pitch + ext_roll + ext_yaw; // M4: FL
            end

            3'b001: begin // Quad-+
                raw_sums[0] = ext_thrust + ext_pitch - ext_yaw; // Front
                raw_sums[1] = ext_thrust - ext_roll  + ext_yaw; // Right
                raw_sums[2] = ext_thrust - ext_pitch - ext_yaw; // Rear
                raw_sums[3] = ext_thrust + ext_roll  + ext_yaw; // Left
            end

            3'b010: begin // Hexa-X (6 motors)
                raw_sums[0] = ext_thrust - ext_pitch + ext_roll_half - ext_yaw; // Rear-Left
                raw_sums[1] = ext_thrust - ext_pitch - ext_roll_half + ext_yaw; // Rear-Right
                raw_sums[2] = ext_thrust - ext_roll  - ext_yaw;                 // Mid-Right
                raw_sums[3] = ext_thrust + ext_pitch - ext_roll_half + ext_yaw; // Front-Right
                raw_sums[4] = ext_thrust + ext_pitch + ext_roll_half - ext_yaw; // Front-Left
                raw_sums[5] = ext_thrust + ext_roll  + ext_yaw;                 // Mid-Left
            end

            3'b011: begin // Hexa-+ (6 motors)
                raw_sums[0] = ext_thrust + ext_pitch - ext_yaw;
                raw_sums[1] = ext_thrust + ext_pitch_half - ext_roll + ext_yaw;
                raw_sums[2] = ext_thrust - ext_pitch_half - ext_roll - ext_yaw;
                raw_sums[3] = ext_thrust - ext_pitch + ext_yaw;
                raw_sums[4] = ext_thrust - ext_pitch_half + ext_roll - ext_yaw;
                raw_sums[5] = ext_thrust + ext_pitch_half + ext_roll + ext_yaw;
            end

            3'b100: begin // Octo-X (8 motors)
                raw_sums[0] = ext_thrust - ext_pitch + ext_roll - ext_yaw; // M1
                raw_sums[1] = ext_thrust - ext_pitch - ext_roll + ext_yaw; // M2
                raw_sums[2] = ext_thrust + ext_pitch - ext_roll - ext_yaw; // M3
                raw_sums[3] = ext_thrust + ext_pitch + ext_roll + ext_yaw; // M4
                raw_sums[4] = ext_thrust + ext_pitch - ext_yaw;            // M5 Front
                raw_sums[5] = ext_thrust - ext_roll  + ext_yaw;            // M6 Right
                raw_sums[6] = ext_thrust - ext_pitch - ext_yaw;            // M7 Rear
                raw_sums[7] = ext_thrust + ext_roll  + ext_yaw;            // M8 Left
            end

            default: begin // Default to Quad-X
                raw_sums[0] = ext_thrust - ext_pitch + ext_roll - ext_yaw;
                raw_sums[1] = ext_thrust - ext_pitch - ext_roll + ext_yaw;
                raw_sums[2] = ext_thrust + ext_pitch - ext_roll - ext_yaw;
                raw_sums[3] = ext_thrust + ext_pitch + ext_roll + ext_yaw;
            end
        endcase
    end

    function automatic logic signed [DATA_W-1:0] clamp_m(input logic signed [ACCUM_W-1:0] s);
        if (s > CLAMP_MAX)
            return FIX_ONE;
        else if (s < CLAMP_MIN)
            return FIX_ZERO;
        else
            return s[DATA_W-1:0];
    endfunction

    // Check saturation flag
    always_comb begin
        any_sat = 1'b0;
        for (int i = 0; i < NUM_MOTORS; i++) begin
            if (raw_sums[i] > CLAMP_MAX || raw_sums[i] < CLAMP_MIN)
                any_sat = 1'b1;
        end
    end

    // Registered output stage
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_MOTORS; i++) begin
                motors[i] <= FIX_ZERO;
            end
            motor_valid    <= 1'b0;
            saturated_flag <= 1'b0;
        end else if (sample_valid) begin
            for (int i = 0; i < NUM_MOTORS; i++) begin
                motors[i] <= clamp_m(raw_sums[i]);
            end
            motor_valid    <= 1'b1;
            saturated_flag <= any_sat;
        end else begin
            motor_valid <= 1'b0;
        end
    end

endmodule : pid_mixer_matrix

`endif // PID_MIXER_MATRIX_SV
