//==============================================================================
// Module: pid_integ_unit.sv
// Description: Integrator accumulator with conditional integration (anti-windup
//              clamping), independent saturation bounds, and instantaneous
//              bumpless transfer re-seeding.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_INTEG_UNIT_SV
`define PID_INTEG_UNIT_SV

`include "pid_pkg.sv"

import pid_pkg::*;

module pid_integ_unit #(
    parameter int DATA_W  = 16,
    parameter int FRACT_W = 12
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     sample_valid,

    // Error and Parameters
    input  logic signed [DATA_W-1:0] error,         // Setpoint - Measurement (Q16.16)
    input  logic signed [DATA_W-1:0] ki_eff,        // Effective integral gain Ki * Ts (Q16.16)
    input  logic signed [DATA_W-1:0] i_min,         // Accumulator lower bound (Q16.16)
    input  logic signed [DATA_W-1:0] i_max,         // Accumulator upper bound (Q16.16)

    // Anti-Windup Clamping Feedback (from output stage)
    input  logic                     clamp_high,    // Actuator output is saturated high
    input  logic                     clamp_low,     // Actuator output is saturated low

    // Bumpless Transfer Re-seeding Port
    input  logic                     reseed_valid,  // Assert to overwrite integrator state
    input  logic signed [DATA_W-1:0] reseed_val,    // Back-calculated initialization value

    // Output
    output logic signed [DATA_W-1:0] i_term         // Active integral term I[n] (Q16.16)
);

    // Integrator State Register
    logic signed [DATA_W-1:0] integ_reg;

    // Delta integral multiplier output
    logic signed [DATA_W-1:0] delta_i;
    logic signed [DATA_W-1:0] integ_sum;
    logic signed [DATA_W-1:0] integ_next;
    logic                     freeze;

    // Evaluate Ki * error
    pid_mult #(
        .DATA_W(DATA_W),
        .FRACT_W(FRACT_W)
    ) u_ki_mult (
        .a(ki_eff),
        .b(error),
        .p(delta_i)
    );

    // Anti-windup conditional integration check:
    // Freeze integration if output is saturated and error pushes further into saturation
    assign freeze = (clamp_high && ($signed(error) > $signed({DATA_W{1'b0}}))) ||
                    (clamp_low  && ($signed(error) < $signed({DATA_W{1'b0}})));

    // Clamping helper for 32-bit values
    function automatic logic signed [DATA_W-1:0] clamp_i (
        input logic signed [DATA_W-1:0] val,
        input logic signed [DATA_W-1:0] v_min,
        input logic signed [DATA_W-1:0] v_max
    );
        begin
            if (val > v_max)
                clamp_i = v_max;
            else if (val < v_min)
                clamp_i = v_min;
            else
                clamp_i = val;
        end
    endfunction

    // Guarded 33-bit summation to mathematically prevent two's-complement overflow
    logic signed [DATA_W:0] integ_sum_ext;
    logic signed [DATA_W:0] imax_ext;
    logic signed [DATA_W:0] imin_ext;

    assign integ_sum_ext = $signed({integ_reg[DATA_W-1], integ_reg}) + $signed({delta_i[DATA_W-1], delta_i});
    assign imax_ext      = $signed({i_max[DATA_W-1], i_max});
    assign imin_ext      = $signed({i_min[DATA_W-1], i_min});

    // Next-state evaluation
    always_comb begin
        if (reseed_valid) begin
            // Instantaneous bumpless back-calculation re-seed
            integ_next = clamp_i(reseed_val, i_min, i_max);
        end else if (freeze) begin
            // Hold current accumulator value (anti-windup active)
            integ_next = integ_reg;
        end else begin
            // Normal accumulation with guarded saturation clamping
            if (integ_sum_ext > imax_ext)
                integ_next = i_max;
            else if (integ_sum_ext < imin_ext)
                integ_next = i_min;
            else
                integ_next = integ_sum_ext[DATA_W-1:0];
        end
    end

    // Sequential State Update
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            integ_reg <= FIX_ZERO;
        end else if (sample_valid) begin
            integ_reg <= integ_next;
        end
    end

    // Output the active integral term for the current cycle (Backward Euler)
    assign i_term = integ_next;

`ifdef FORMAL
    always_ff @(posedge clk) begin
        if (rst_n && !reseed_valid) begin
            if (clamp_high && ($signed(error) > $signed({DATA_W{1'b0}}))) begin
                assert(integ_next == integ_reg);
            end
            if (clamp_low && ($signed(error) < $signed({DATA_W{1'b0}}))) begin
                assert(integ_next == integ_reg);
            end
        end
    end
`endif

endmodule : pid_integ_unit

`endif // PID_INTEG_UNIT_SV
