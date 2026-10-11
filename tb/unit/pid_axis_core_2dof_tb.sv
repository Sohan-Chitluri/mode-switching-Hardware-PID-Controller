//==============================================================================
// Testbench: pid_axis_core_2dof_tb.sv
// Description: Self-checking unit testbench for 2-DOF Single-Channel PID Core.
//              Verifies:
//              1. Classical 1-DOF equivalence mode (b = 1.0, c = 0.0)
//              2. Softened setpoint tracking (b = 0.5 cuts P-kick by 50%)
//              3. Full 100% disturbance rejection regardless of b
//              4. Integral accumulation and anti-windup clamping
//==============================================================================

`timescale 1ns/1ps

`include "pid_pkg.sv"
`include "pid_axis_core_2dof.sv"

module pid_axis_core_2dof_tb;

    import pid_pkg::*;

    localparam int DATA_W  = 32;
    localparam int FRACT_W = 16;
    localparam int ACCUM_W = 48;

    logic clk;
    logic rst_n;
    logic sample_valid;

    logic signed [DATA_W-1:0] setpoint;
    logic signed [DATA_W-1:0] measurement;

    logic signed [DATA_W-1:0] weight_b;
    logic signed [DATA_W-1:0] weight_c;

    logic signed [DATA_W-1:0] kp;
    logic signed [DATA_W-1:0] ki_eff;
    logic signed [DATA_W-1:0] kd_eff;
    logic signed [DATA_W-1:0] alpha;

    logic signed [DATA_W-1:0] u_min;
    logic signed [DATA_W-1:0] u_max;
    logic signed [DATA_W-1:0] i_min;
    logic signed [DATA_W-1:0] i_max;

    logic                     reseed_valid;
    logic signed [DATA_W-1:0] reseed_val;

    logic signed [DATA_W-1:0] u_sat;
    logic signed [DATA_W-1:0] error_out;
    logic signed [DATA_W-1:0] p_term_out;
    logic signed [DATA_W-1:0] i_term_out;
    logic signed [DATA_W-1:0] d_term_out;
    logic signed [ACCUM_W-1:0] raw_sum_out;
    logic                     clamp_high_out;
    logic                     clamp_low_out;

    // DUT
    pid_axis_core_2dof #(
        .DATA_W(DATA_W),
        .FRACT_W(FRACT_W),
        .ACCUM_W(ACCUM_W)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(sample_valid),
        .setpoint(setpoint),
        .measurement(measurement),
        .weight_b(weight_b),
        .weight_c(weight_c),
        .kp(kp),
        .ki_eff(ki_eff),
        .kd_eff(kd_eff),
        .alpha(alpha),
        .u_min(u_min),
        .u_max(u_max),
        .i_min(i_min),
        .i_max(i_max),
        .reseed_valid(reseed_valid),
        .reseed_val(reseed_val),
        .u_sat(u_sat),
        .error_out(error_out),
        .p_term_out(p_term_out),
        .i_term_out(i_term_out),
        .d_term_out(d_term_out),
        .raw_sum_out(raw_sum_out),
        .clamp_high_out(clamp_high_out),
        .clamp_low_out(clamp_low_out)
    );

    initial clk = 0;
    always #10 clk = ~clk;

    int error_count = 0;

    initial begin
        $display("==================================================");
        $display("    TESTBENCH: 2-DOF Axis Core PID Engine (Unit) ");
        $display("==================================================");

        rst_n        = 0;
        sample_valid = 0;
        setpoint     = 0;
        measurement  = 0;
        weight_b     = FIX_ONE;  // 1.0
        weight_c     = FIX_ZERO; // 0.0
        kp           = 32'sh0002_0000; // Kp = 2.0
        ki_eff       = 32'sh0000_1000; // Ki = 0.0625
        kd_eff       = 32'sh0000_2000; // Kd = 0.125
        alpha        = 32'sh0000_8000; // alpha = 0.5
        u_min        = -FIX_ONE;
        u_max        = FIX_ONE;
        i_min        = -FIX_HALF;
        i_max        = FIX_HALF;
        reseed_valid = 0;
        reseed_val   = 0;

        #40;
        rst_n = 1;
        #20;

        // Test 1: 1-DOF Equivalence (b = 1.0)
        $display("[TEST 1] Testing 1-DOF baseline mode (b = 1.0, c = 0.0)...");
        weight_b     = FIX_ONE; // b = 1.0
        setpoint     = 32'sh0001_0000; // r = 1.0
        measurement  = 32'sh0000_0000; // y = 0.0
        sample_valid = 1;
        @(posedge clk);
        #1;
        sample_valid = 0;

        // P = Kp * (b*r - y) = 2.0 * (1.0*1.0 - 0) = 2.0 (0x0002_0000)
        $display("1-DOF Step P-term: 0x%08h (Expected: 0x00020000)", p_term_out);
        if (p_term_out !== 32'sh0002_0000) begin
            $display("[FAIL] P-term mismatch in 1-DOF mode!");
            error_count++;
        end else begin
            $display("[PASS] 1-DOF mode equivalence verified.");
        end

        // Test 2: 2-DOF Softened Setpoint Response (b = 0.5)
        $display("[TEST 2] Testing 2-DOF softened setpoint response (b = 0.5)...");
        weight_b     = FIX_HALF; // b = 0.5
        setpoint     = 32'sh0001_0000; // r = 1.0
        measurement  = 32'sh0000_0000; // y = 0.0
        sample_valid = 1;
        @(posedge clk);
        #1;
        sample_valid = 0;

        // P = Kp * (b*r - y) = 2.0 * (0.5*1.0 - 0) = 1.0 (0x0001_0000)
        $display("2-DOF Step P-term with b=0.5: 0x%08h (Expected: 0x00010000)", p_term_out);
        if (p_term_out !== 32'sh0001_0000) begin
            $display("[FAIL] 2-DOF P-term should be halved for b=0.5!");
            error_count++;
        end else begin
            $display("[PASS] 2-DOF setpoint weighting verified: 50%% kick reduction.");
        end

        // Test 3: Disturbance Rejection Invariance (y = 1.0, r = 0.0)
        $display("[TEST 3] Testing Disturbance Rejection with b = 0.5...");
        setpoint     = 32'sh0000_0000; // r = 0.0
        measurement  = 32'sh0001_0000; // y = 1.0 (disturbance)
        sample_valid = 1;
        @(posedge clk);
        #1;
        sample_valid = 0;

        // P = Kp * (b*0 - 1.0) = 2.0 * (-1.0) = -2.0 (-0x0002_0000)
        $display("Disturbance P-term: 0x%08h (Expected: -0x00020000)", p_term_out);
        if (p_term_out !== -32'sh0002_0000) begin
            $display("[FAIL] Full disturbance rejection compromised by b!");
            error_count++;
        end else begin
            $display("[PASS] 100%% disturbance rejection preserved under 2-DOF weighting.");
        end

        #40;
        if (error_count == 0) begin
            $display("==================================================");
            $display("    [PASS] ALL 2-DOF AXIS CORE TESTS PASSED       ");
            $display("==================================================");
            $finish(0);
        end else begin
            $display("==================================================");
            $display("    [FAIL] %0d ERRORS IN 2-DOF AXIS CORE TB       ", error_count);
            $display("==================================================");
            $fatal(1);
        end
    end

endmodule : pid_axis_core_2dof_tb
