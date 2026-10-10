//==============================================================================
// Module: pid_tilt_comp.sv
// Description: Thrust tilt compensation engine for multirotor flight.
//              Prevents altitude sag during pitch and roll maneuvers by scaling
//              base vertical thrust: T_comp = T_base / (cos phi * cos theta).
//              Uses a hardware-friendly small-angle Taylor expansion:
//                1 / (cos phi * cos theta) ≈ 1 + 0.5 * phi^2 + 0.5 * theta^2
//              Capped at 1.5x (approx. 48 deg tilt) to prevent motor saturation.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_TILT_COMP_SV
`define PID_TILT_COMP_SV

`include "pid_pkg.sv"
`include "pid_mult.sv"

import pid_pkg::*;

module pid_tilt_comp #(
    parameter int DATA_W  = 32,
    parameter int FRACT_W = 16
) (
    input  logic signed [DATA_W-1:0] base_thrust, // Uncompensated thrust (Q16.16)
    input  logic signed [DATA_W-1:0] phi,         // Roll attitude angle (Q16.16 rad)
    input  logic signed [DATA_W-1:0] theta,       // Pitch attitude angle (Q16.16 rad)

    output logic signed [DATA_W-1:0] comp_thrust  // Tilt-compensated thrust (Q16.16)
);

    // Maximum tilt scale factor: 1.5 in Q16.16 (0x0001_8000)
    localparam logic signed [DATA_W-1:0] MAX_TILT_SCALE = 32'sh0001_8000;

    // Squared angle terms: phi^2 and theta^2
    logic signed [DATA_W-1:0] phi_sq;
    logic signed [DATA_W-1:0] theta_sq;

    pid_mult #(.DATA_W(DATA_W), .FRACT_W(FRACT_W)) u_phi_sq (
        .a(phi),
        .b(phi),
        .p(phi_sq)
    );

    pid_mult #(.DATA_W(DATA_W), .FRACT_W(FRACT_W)) u_theta_sq (
        .a(theta),
        .b(theta),
        .p(theta_sq)
    );

    // Summation: 1.0 + 0.5*phi^2 + 0.5*theta^2
    logic signed [DATA_W-1:0] raw_scale;
    logic signed [DATA_W-1:0] clamped_scale;

    always_comb begin
        raw_scale = FIX_ONE + (phi_sq >>> 1) + (theta_sq >>> 1);

        // Clamp scale factor to [1.0, 1.5]
        if (raw_scale > MAX_TILT_SCALE)
            clamped_scale = MAX_TILT_SCALE;
        else if (raw_scale < FIX_ONE)
            clamped_scale = FIX_ONE;
        else
            clamped_scale = raw_scale;
    end

    // Multiplier applying tilt compensation: base_thrust * clamped_scale
    pid_mult #(.DATA_W(DATA_W), .FRACT_W(FRACT_W)) u_apply_tilt (
        .a(base_thrust),
        .b(clamped_scale),
        .p(comp_thrust)
    );

endmodule : pid_tilt_comp

`endif // PID_TILT_COMP_SV
