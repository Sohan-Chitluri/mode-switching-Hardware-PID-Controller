//==============================================================================
// Module: pid_mixer.sv
// Description: Quadrotor Quad-X Motor Mixer.
//              Converts 4-axis control demands (collective thrust, roll torque,
//              pitch torque, yaw torque) into 4 individual motor thrust commands.
//
// Motor Layout (Quad-X configuration, viewed from top):
//
//              Front (+X)
//       M4 (FL)    ^    M3 (FR)
//        [CCW]     |     [CW]
//             \    |    /
//              \   |   /
//  Left (-Y) <---+---   > Right (+Y)
//              /   |   \
//             /    |    \
//        [CW]      |     [CCW]
//       M1 (RL)    v    M2 (RR)
//              Rear (-X)
//
// Mixing Equations (Q16.16 fixed-point):
//   M1 (Rear-Left,   CW ) = clamp(thrust - pitch + roll - yaw, 0.0, 1.0)
//   M2 (Rear-Right,  CCW) = clamp(thrust - pitch - roll + yaw, 0.0, 1.0)
//   M3 (Front-Right, CW ) = clamp(thrust + pitch - roll - yaw, 0.0, 1.0)
//   M4 (Front-Left,  CCW) = clamp(thrust + pitch + roll + yaw, 0.0, 1.0)
//
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_MIXER_SV
`define PID_MIXER_SV

`include "pid_pkg.sv"
import pid_pkg::*;

module pid_mixer #(
    parameter int DATA_W  = 32,
    parameter int FRACT_W = 16,
    parameter int ACCUM_W = 48
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     sample_valid,

    // Controller Demands (Q16.16 format)
    input  logic signed [DATA_W-1:0] u_roll,    // Roll torque demand  [-1.0, +1.0]
    input  logic signed [DATA_W-1:0] u_pitch,   // Pitch torque demand [-1.0, +1.0]
    input  logic signed [DATA_W-1:0] u_yaw,     // Yaw torque demand   [-1.0, +1.0]
    input  logic signed [DATA_W-1:0] u_thrust,  // Collective thrust   [ 0.0, +1.0]

    // Actuator Motor Outputs (Q16.16 format, saturated to [0.0, 1.0])
    output logic                     motor_valid,
    output logic signed [DATA_W-1:0] motor_1,   // Rear-Left  (RL)
    output logic signed [DATA_W-1:0] motor_2,   // Rear-Right (RR)
    output logic signed [DATA_W-1:0] motor_3,   // Front-Right (FR)
    output logic signed [DATA_W-1:0] motor_4,   // Front-Left (FL)

    // Telemetry Status
    output logic                     saturated_flag
);

    // 48-bit guarded intermediate sums to prevent overflow prior to saturation clamping
    logic signed [ACCUM_W-1:0] sum_m1;
    logic signed [ACCUM_W-1:0] sum_m2;
    logic signed [ACCUM_W-1:0] sum_m3;
    logic signed [ACCUM_W-1:0] sum_m4;

    // Sign extension helpers
    wire signed [ACCUM_W-1:0] ext_thrust = {{ACCUM_W-DATA_W{u_thrust[DATA_W-1]}}, u_thrust};
    wire signed [ACCUM_W-1:0] ext_pitch  = {{ACCUM_W-DATA_W{u_pitch[DATA_W-1]}},  u_pitch};
    wire signed [ACCUM_W-1:0] ext_roll   = {{ACCUM_W-DATA_W{u_roll[DATA_W-1]}},   u_roll};
    wire signed [ACCUM_W-1:0] ext_yaw    = {{ACCUM_W-DATA_W{u_yaw[DATA_W-1]}},    u_yaw};

    // Quad-X Orthogonal Mixing Logic
    assign sum_m1 = ext_thrust - ext_pitch + ext_roll - ext_yaw; // M1: RL
    assign sum_m2 = ext_thrust - ext_pitch - ext_roll + ext_yaw; // M2: RR
    assign sum_m3 = ext_thrust + ext_pitch - ext_roll - ext_yaw; // M3: FR
    assign sum_m4 = ext_thrust + ext_pitch + ext_roll + ext_yaw; // M4: FL

    // Upper and lower saturation clamp bounds (Q16.16)
    localparam signed [ACCUM_W-1:0] CLAMP_MAX = {{ACCUM_W-DATA_W{1'b0}}, FIX_ONE};  // 1.0 (0x0001_0000)
    localparam signed [ACCUM_W-1:0] CLAMP_MIN = '0;                                   // 0.0 (0x0000_0000)

    // Combinational saturation clamping functions
    function automatic logic signed [DATA_W-1:0] clamp_motor(input logic signed [ACCUM_W-1:0] raw_sum);
        if (raw_sum > CLAMP_MAX)
            clamp_motor = FIX_ONE;
        else if (raw_sum < CLAMP_MIN)
            clamp_motor = FIX_ZERO;
        else
            clamp_motor = raw_sum[DATA_W-1:0];
    endfunction

    function automatic logic is_saturated(input logic signed [ACCUM_W-1:0] raw_sum);
        is_saturated = (raw_sum > CLAMP_MAX) || (raw_sum < CLAMP_MIN);
    endfunction

    wire sat_m1 = is_saturated(sum_m1);
    wire sat_m2 = is_saturated(sum_m2);
    wire sat_m3 = is_saturated(sum_m3);
    wire sat_m4 = is_saturated(sum_m4);

    // Synchronous Registered Output Stage
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            motor_1        <= FIX_ZERO;
            motor_2        <= FIX_ZERO;
            motor_3        <= FIX_ZERO;
            motor_4        <= FIX_ZERO;
            motor_valid    <= 1'b0;
            saturated_flag <= 1'b0;
        end else if (sample_valid) begin
            motor_1        <= clamp_motor(sum_m1);
            motor_2        <= clamp_motor(sum_m2);
            motor_3        <= clamp_motor(sum_m3);
            motor_4        <= clamp_motor(sum_m4);
            motor_valid    <= 1'b1;
            saturated_flag <= sat_m1 | sat_m2 | sat_m3 | sat_m4;
        end else begin
            motor_valid    <= 1'b0;
        end
    end

endmodule : pid_mixer

`endif // PID_MIXER_SV
