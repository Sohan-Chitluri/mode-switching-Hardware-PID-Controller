//==============================================================================
// Testbench: pid_fixed_point_tb.sv
// Description: Targeted fixed-point arithmetic boundary and vulnerability testbench.
//              Systematically verifies:
//              1. Multiplier Extremes (MAX_POS, MIN_NEG, dynamic limits, overflow inversion).
//              2. Four-Quadrant Signedness Combinations & Algebraic Symmetry.
//              3. Truncation Behavior & Asymmetric Floor Bias in Arithmetic Right Shift.
//              4. Flight Controller Gain Bank Envelope Margin & Overflow Thresholds.
//              5. Guarded 48-Bit Summation & Anti-Windup Clamping Integrity.
//              6. Integrator 33-Bit Guarded Addition Under Extreme Steps.
//              7. Consumer Module Edge Cases:
//                 - pid_deriv_filter negation asymmetry (-MIN_NEG)
//                 - pid_outer_loop angle error extrema & gain scaling
//                 - pid_tilt_comp angle extrema & saturation protection
//                 - pid_gain_bank MMIO unvalidated parameter writes
//              8. Pseudo-Random Reference Model Verification & Truncation Profiling.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`timescale 1ns/1ps

`include "pid_pkg.sv"
`include "pid_mult.sv"
`include "pid_deriv_filter.sv"
`include "pid_integ_unit.sv"
`include "pid_sat_unit.sv"
`include "pid_outer_loop.sv"
`include "pid_gain_bank.sv"
`include "pid_tilt_comp.sv"

import pid_pkg::*;

module pid_fixed_point_tb;

    // Signals for pid_mult
    logic signed [31:0] mult_a, mult_b;
    logic signed [31:0] mult_p;

    // Signals for pid_sat_unit
    logic clk;
    logic rst_n;
    logic sample_valid;
    logic signed [31:0] p_term, i_term, d_term;
    logic signed [31:0] u_min, u_max;
    logic signed [31:0] u_sat;
    logic signed [47:0] raw_sum;
    logic               clamp_high, clamp_low;

    // Signals for pid_integ_unit
    logic signed [31:0] integ_err, ki_eff, i_min, i_max;
    logic               integ_ch, integ_cl, reseed_v;
    logic signed [31:0] reseed_val, i_term_out;

    // Signals for pid_deriv_filter
    logic signed [31:0] deriv_meas, deriv_alpha, deriv_kd;
    logic               deriv_reseed;
    logic signed [31:0] deriv_d_term, deriv_filter_state;

    // Signals for pid_outer_loop
    logic [1:0]         outer_mode;
    logic signed [31:0] outer_rc_roll, outer_rc_pitch, outer_rc_yaw, outer_rc_thrust;
    logic signed [31:0] outer_phi, outer_theta, outer_alt_z;
    logic               outer_alt_valid;
    logic signed [31:0] outer_kp_roll, outer_kp_pitch, outer_kp_alt;
    logic signed [31:0] outer_sp_roll, outer_sp_pitch, outer_sp_yaw, outer_sp_vz, outer_base_thrust;

    // Signals for pid_tilt_comp
    logic signed [31:0] tilt_base, tilt_phi, tilt_theta, tilt_comp_out;

    // Signals for pid_gain_bank
    logic [1:0]         bank_mode;
    logic               bank_we;
    logic [7:0]         bank_addr;
    logic [31:0]        bank_wdata, bank_rdata;
    logic signed [31:0] bank_roll_kp, bank_roll_ki, bank_roll_kd, bank_roll_alpha;
    logic signed [31:0] bank_roll_u_min, bank_roll_u_max, bank_roll_i_min, bank_roll_i_max;

    int total_tests = 0;
    int passed_tests = 0;
    int failed_tests = 0;

    // -------------------------------------------------------------------------
    // DUT Instantiations
    // -------------------------------------------------------------------------
    pid_mult #(
        .DATA_W(32),
        .FRACT_W(16)
    ) dut_mult (
        .a(mult_a),
        .b(mult_b),
        .p(mult_p)
    );

    pid_sat_unit #(
        .DATA_W(32),
        .ACCUM_W(48)
    ) dut_sat (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(sample_valid),
        .p_term(p_term),
        .i_term(i_term),
        .d_term(d_term),
        .u_min(u_min),
        .u_max(u_max),
        .u_sat(u_sat),
        .raw_sum(raw_sum),
        .clamp_high(clamp_high),
        .clamp_low(clamp_low)
    );

    pid_integ_unit #(
        .DATA_W(32),
        .FRACT_W(16)
    ) dut_integ (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(sample_valid),
        .error(integ_err),
        .ki_eff(ki_eff),
        .i_min(i_min),
        .i_max(i_max),
        .clamp_high(integ_ch),
        .clamp_low(integ_cl),
        .reseed_valid(reseed_v),
        .reseed_val(reseed_val),
        .i_term(i_term_out)
    );

    pid_deriv_filter #(
        .DATA_W(32),
        .FRACT_W(16)
    ) dut_deriv (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(sample_valid),
        .meas(deriv_meas),
        .alpha(deriv_alpha),
        .kd_eff(deriv_kd),
        .reseed_valid(deriv_reseed),
        .d_term(deriv_d_term),
        .filter_state(deriv_filter_state)
    );

    pid_outer_loop #(
        .DATA_W(32),
        .FRACT_W(16)
    ) dut_outer (
        .active_mode(outer_mode),
        .rc_sp_roll(outer_rc_roll),
        .rc_sp_pitch(outer_rc_pitch),
        .rc_sp_yaw(outer_rc_yaw),
        .rc_sp_thrust(outer_rc_thrust),
        .imu_phi(outer_phi),
        .imu_theta(outer_theta),
        .alt_z(outer_alt_z),
        .alt_valid(outer_alt_valid),
        .kp_att_roll(outer_kp_roll),
        .kp_att_pitch(outer_kp_pitch),
        .kp_alt_pos(outer_kp_alt),
        .inner_sp_roll(outer_sp_roll),
        .inner_sp_pitch(outer_sp_pitch),
        .inner_sp_yaw(outer_sp_yaw),
        .inner_sp_vz(outer_sp_vz),
        .base_thrust(outer_base_thrust)
    );

    pid_tilt_comp #(
        .DATA_W(32),
        .FRACT_W(16)
    ) dut_tilt (
        .base_thrust(tilt_base),
        .phi(tilt_phi),
        .theta(tilt_theta),
        .comp_thrust(tilt_comp_out)
    );

    pid_gain_bank #(
        .DATA_W(32)
    ) dut_bank (
        .clk(clk),
        .rst_n(rst_n),
        .active_mode(bank_mode),
        .cfg_we(bank_we),
        .cfg_addr(bank_addr),
        .cfg_wdata(bank_wdata),
        .cfg_rdata(bank_rdata),
        .kp_att_roll(), .kp_att_pitch(), .kp_alt_pos(), .hover_thrust(), .failsafe_thrust(),
        .roll_kp(bank_roll_kp), .roll_ki_eff(bank_roll_ki), .roll_kd_eff(bank_roll_kd), .roll_alpha(bank_roll_alpha),
        .roll_u_min(bank_roll_u_min), .roll_u_max(bank_roll_u_max), .roll_i_min(bank_roll_i_min), .roll_i_max(bank_roll_i_max),
        .pitch_kp(), .pitch_ki_eff(), .pitch_kd_eff(), .pitch_alpha(),
        .pitch_u_min(), .pitch_u_max(), .pitch_i_min(), .pitch_i_max(),
        .yaw_kp(), .yaw_ki_eff(), .yaw_kd_eff(), .yaw_alpha(),
        .yaw_u_min(), .yaw_u_max(), .yaw_i_min(), .yaw_i_max(),
        .thrust_kp(), .thrust_ki_eff(), .thrust_kd_eff(), .thrust_alpha(),
        .thrust_u_min(), .thrust_u_max(), .thrust_i_min(), .thrust_i_max()
    );

    always #10 clk = ~clk;

    // Helper task to check multiplier output
    task automatic check_mult(
        input string test_name,
        input logic signed [31:0] a,
        input logic signed [31:0] b,
        input logic signed [31:0] exp_p,
        input bit expect_match
    );
        total_tests++;
        mult_a = a;
        mult_b = b;
        #1;
        if (expect_match) begin
            if (mult_p === exp_p) begin
                $display("  [PASS] %s: a=%08h, b=%08h -> p=%08h", test_name, a, b, mult_p);
                passed_tests++;
            end else begin
                $display("  [FAIL] %s: a=%08h, b=%08h -> Got %08h, Exp %08h", test_name, a, b, mult_p, exp_p);
                failed_tests++;
            end
        end else begin
            // Test demonstrating known vulnerability or discrepancy
            $display("  [VULN PROBE] %s: a=%08h (%0.2f), b=%08h (%0.2f) -> p=%08h (%0.2f) vs ideal %08h",
                     test_name, a, real'(a)/65536.0, b, real'(b)/65536.0,
                     mult_p, real'(mult_p)/65536.0, exp_p);
            passed_tests++;
        end
    endtask

    initial begin
        clk          = 0;
        rst_n        = 0;
        sample_valid = 0;
        p_term       = 0;
        i_term       = 0;
        d_term       = 0;
        u_min        = -FIX_ONE;
        u_max        = FIX_ONE;
        integ_err    = 0;
        ki_eff       = 0;
        i_min        = -FIX_HALF;
        i_max        = FIX_HALF;
        integ_ch     = 0;
        integ_cl     = 0;
        reseed_v     = 0;
        reseed_val   = 0;

        deriv_meas   = 0;
        deriv_alpha  = FIX_HALF;
        deriv_kd     = FIX_ONE;
        deriv_reseed = 0;

        outer_mode      = MODE_STABILIZE;
        outer_rc_roll   = 0;
        outer_rc_pitch  = 0;
        outer_rc_yaw    = 0;
        outer_rc_thrust = FIX_HALF;
        outer_phi       = 0;
        outer_theta     = 0;
        outer_alt_z     = 0;
        outer_alt_valid = 1;
        outer_kp_roll   = 32'sh0004_8000; // 4.5
        outer_kp_pitch  = 32'sh0004_8000; // 4.5
        outer_kp_alt    = 32'sh0001_3333; // 1.2

        tilt_base  = FIX_HALF;
        tilt_phi   = 0;
        tilt_theta = 0;

        bank_mode  = MODE_ACRO;
        bank_we    = 0;
        bank_addr  = 0;
        bank_wdata = 0;

        #20 rst_n = 1;
        #20;

        $display("======================================================================");
        $display("  Mode-Switching Hardware PID Fixed-Point Boundary Investigation");
        $display("======================================================================");

        // ---------------------------------------------------------------------
        // SECTION 1: Multiplier Identities & Dynamic Extremes
        // ---------------------------------------------------------------------
        $display("\n>>> [SECTION 1] Multiplier Identities & Dynamic Extremes");

        // 1. Identity: X * 1.0 = X
        check_mult("Identity MAX_POS * 1.0", MAX_POS, FIX_ONE, MAX_POS, 1);
        check_mult("Identity MIN_NEG * 1.0", MIN_NEG, FIX_ONE, MIN_NEG, 1);
        check_mult("Identity 0 * 1.0", FIX_ZERO, FIX_ONE, FIX_ZERO, 1);
        check_mult("Identity 0.5 * 1.0", FIX_HALF, FIX_ONE, FIX_HALF, 1);

        // 2. Negation: X * -1.0
        check_mult("Negation MAX_POS * -1.0", MAX_POS, -FIX_ONE, -MAX_POS, 1);
        // Saturating boundary: MIN_NEG (-32768.0) * -1.0 clamps to MAX_POS (+32767.99998)
        check_mult("Saturating Boundary MIN_NEG * -1.0 -> MAX_POS", MIN_NEG, -FIX_ONE, MAX_POS, 1);

        // 3. Zero property
        check_mult("Zero MAX_POS * 0", MAX_POS, FIX_ZERO, FIX_ZERO, 1);
        check_mult("Zero MIN_NEG * 0", MIN_NEG, FIX_ZERO, FIX_ZERO, 1);

        // ---------------------------------------------------------------------
        // SECTION 2: Signedness Quadrants & Algebraic Symmetry
        // ---------------------------------------------------------------------
        $display("\n>>> [SECTION 2] Signedness Quadrants & Algebraic Symmetry");
        begin
            logic signed [31:0] val_a, val_b;
            logic signed [31:0] p_pp, p_pn, p_np, p_nn;

            val_a = 32'sh0002_8000; // +2.5
            val_b = 32'sh0003_4000; // +3.25 -> prod = +8.125 (0x0008_2000)

            mult_a = val_a;  mult_b = val_b;  #1; p_pp = mult_p;
            mult_a = val_a;  mult_b = -val_b; #1; p_pn = mult_p;
            mult_a = -val_a; mult_b = val_b;  #1; p_np = mult_p;
            mult_a = -val_a; mult_b = -val_b; #1; p_nn = mult_p;

            total_tests++;
            if (p_pp == 32'sh0008_2000 && p_nn == p_pp && p_pn == -p_pp && p_np == p_pn) begin
                $display("  [PASS] 4-Quadrant Symmetry: (+a)(+b)=(%08h), (-a)(-b)=(%08h), (+a)(-b)=(%08h), (-a)(+b)=(%08h)",
                         p_pp, p_nn, p_pn, p_np);
                passed_tests++;
            end else begin
                $display("  [FAIL] Quadrant symmetry violated! p_pp=%08h, p_nn=%08h, p_pn=%08h, p_np=%08h",
                         p_pp, p_nn, p_pn, p_np);
                failed_tests++;
            end
        end

        // ---------------------------------------------------------------------
        // SECTION 3: Sub-LSB Truncation & Asymmetric Floor Bias
        // ---------------------------------------------------------------------
        $display("\n>>> [SECTION 3] Sub-LSB Truncation & Quantization Quantifiers");

        // Smallest non-zero product: 1 LSB * 1.0 = 1 LSB
        check_mult("1 LSB * 1.0", 32'sh0000_0001, FIX_ONE, 32'sh0000_0001, 1);

        // Sub-LSB quantization: 1 LSB * 0.5 = 0.5 LSB -> truncates to 0
        check_mult("Sub-LSB Truncation (0.5 LSB -> 0)", 32'sh0000_0001, FIX_HALF, 32'sh0000_0000, 1);

        // Asymmetric floor bias in two's complement right shift:
        // +1.5 LSB (0x0001 * 1.5 = 0x0001 in integer after shift)
        // -1.5 LSB shifts to -2 LSB because arithmetic right shift rounds towards -infinity
        begin
            logic signed [31:0] p_pos_half, p_neg_half;
            mult_a = 32'sh0000_0003; mult_b = FIX_HALF; #1; p_pos_half = mult_p; // +3 * 0.5 = +1.5 -> +1
            mult_a = -32'sh0000_0003; mult_b = FIX_HALF; #1; p_neg_half = mult_p; // -3 * 0.5 = -1.5 -> -2

            total_tests++;
            $display("  [INFO] Floor Bias Test: +1.5 LSB truncates to %0d LSB, -1.5 LSB truncates to %0d LSB",
                     p_pos_half, p_neg_half);
            if (p_pos_half == 1 && p_neg_half == -2) begin
                $display("  [PASS] Asymmetric floor truncation confirmed (standard Verilog >>> behavior).");
                passed_tests++;
            end else begin
                $display("  [FAIL] Unexpected truncation: pos=%0d, neg=%0d", p_pos_half, p_neg_half);
                failed_tests++;
            end
        end

        // ---------------------------------------------------------------------
        // SECTION 4: Flight Controller Gain Envelope Margins & Overflow Vulnerabilities
        // ---------------------------------------------------------------------
        $display("\n>>> [SECTION 4] Flight Controller Gain Envelope Margins");

        // Case A: Maximum Legal Flight Envelope Operation
        // Attitude error = 180 deg (pi rad = 3.14159 rad = 0x0003_243F)
        // Max pitch/roll Kp = 10.0 (0x000A_0000)
        // Expected P-term = 31.4159 rad/s = 0x001F_6A76
        check_mult("Max Normal Operational P-term (pi rad * Kp=10.0)",
                   32'sh0003_243F, 32'sh000A_0000, 32'sh001F_6A76, 1);

        // Case B: Extreme Tuned Gain Envelope
        // Error = 10.0 rad/s (0x000A_0000), Kp = 50.0 (0x0032_0000)
        // Expected product = 500.0 (0x01F4_0000 = 32,768,000 counts)
        check_mult("Extreme Tuned Envelope (10.0 rad/s * Kp=50.0)",
                   32'sh000A_0000, 32'sh0032_0000, 32'sh01F4_0000, 1);

        // Case C: Headroom to Multiplier Overflow in Operational Envelope
        // Q16.16 max representable without overflow is 32767.99998.
        // Operating at 500.0 leaves a margin of 32767.99998 / 500.0 = 65.5x (36.3 dB margin)!
        $display("  [ANALYSIS] Operational Headroom: Max operational term 500.0 vs 32767.99998 limit = 36.3 dB margin.");

        // Case D: Full-precision Signed Product Overflow & Clamping Verification
        // Clamps positive overflow strictly to MAX_POS (32'sh7FFF_FFFF = +32767.99998)
        check_mult("Positive Overflow Saturation (+1024.0 * +32.0)",
                   32'sh0400_0000, 32'sh0020_0000, MAX_POS, 1);
        check_mult("Negative Overflow Saturation (-1024.0 * +32.0)",
                   -32'sh0400_0000, 32'sh0020_0000, MIN_NEG, 1);
        check_mult("Positive Overflow Saturation (-1024.0 * -32.0)",
                   -32'sh0400_0000, -32'sh0020_0000, MAX_POS, 1);
        check_mult("Full-Scale Positive Over-Range (MAX_POS * MAX_POS)",
                   MAX_POS, MAX_POS, MAX_POS, 1);
        check_mult("Full-Scale Negative Over-Range (MIN_NEG * MAX_POS)",
                   MIN_NEG, MAX_POS, MIN_NEG, 1);
        check_mult("Full-Scale Negative Square (MIN_NEG * MIN_NEG)",
                   MIN_NEG, MIN_NEG, MAX_POS, 1);

        // Exact boundaries just within representable range
        check_mult("Exact In-Range Boundary (+32767.99998 * 1.0)",
                   MAX_POS, FIX_ONE, MAX_POS, 1);
        check_mult("Exact In-Range Boundary (-32768.0 * 1.0)",
                   MIN_NEG, FIX_ONE, MIN_NEG, 1);

        // ---------------------------------------------------------------------
        // SECTION 5: Guarded 48-Bit Summation & Anti-Windup Clamping
        // ---------------------------------------------------------------------
        $display("\n>>> [SECTION 5] Guarded 48-Bit Summation & Saturation Clamping");
        begin
            // Feed individual terms that exceed 1.0 (e.g. 500.0 each)
            p_term = 32'sh01F4_0000; // +500.0
            i_term = 32'sh01F4_0000; // +500.0
            d_term = 32'sh01F4_0000; // +500.0
            u_min  = -FIX_ONE;
            u_max  = FIX_ONE;
            #1;

            total_tests++;
            if (raw_sum == 48'sh0000_05DC_0000 && u_sat == FIX_ONE && clamp_high == 1'b1 && clamp_low == 1'b0) begin
                $display("  [PASS] Guarded Sum: raw_sum=1500.0 (0x%012h), clamped to +1.0, clamp_high=1", raw_sum);
                passed_tests++;
            end else begin
                $display("  [FAIL] Guarded sum mismatch: raw_sum=%012h, u_sat=%08h, ch=%b", raw_sum, u_sat, clamp_high);
                failed_tests++;
            end

            // Symmetrical negative test (-500.0 each)
            p_term = -32'sh01F4_0000;
            i_term = -32'sh01F4_0000;
            d_term = -32'sh01F4_0000;
            #1;

            total_tests++;
            if (raw_sum == -48'sh0000_05DC_0000 && u_sat == -FIX_ONE && clamp_high == 1'b0 && clamp_low == 1'b1) begin
                $display("  [PASS] Guarded Sum: raw_sum=-1500.0, clamped to -1.0, clamp_low=1");
                passed_tests++;
            end else begin
                $display("  [FAIL] Guarded sum negative mismatch: raw_sum=%012h, u_sat=%08h, cl=%b", raw_sum, u_sat, clamp_low);
                failed_tests++;
            end
        end

        // ---------------------------------------------------------------------
        // SECTION 6: Integrator 33-Bit Guarded Addition
        // ---------------------------------------------------------------------
        $display("\n>>> [SECTION 6] Integrator 33-Bit Guarded Accumulation");
        begin
            // Test that huge delta_i does not cause two's complement sign flip
            integ_err = 32'sh7FFF_FFFF; // Max error
            ki_eff    = 32'sh0001_0000; // Ki = 1.0 -> delta_i = MAX_POS
            i_min     = -FIX_HALF;
            i_max     = FIX_HALF;
            integ_ch  = 0;
            integ_cl  = 0;

            sample_valid = 1;
            @(posedge clk);
            #1;
            sample_valid = 0;

            total_tests++;
            if (i_term_out == FIX_HALF) begin
                $display("  [PASS] Integrator 33-bit guard clamped massive step cleanly to i_max (+0.50).");
                passed_tests++;
            end else begin
                $display("  [FAIL] Integrator sign flipped or unclamped! Got %08h, Exp %08h", i_term_out, FIX_HALF);
                failed_tests++;
            end
        end

        // ---------------------------------------------------------------------
        // SECTION 7: Consumer Module Arithmetic Contracts & Vulnerabilities
        // ---------------------------------------------------------------------
        $display("\n>>> [SECTION 7] Consumer Module Arithmetic Contracts & Vulnerabilities");

        // 7A: pid_deriv_filter Saturating Negation & Dynamic Range
        begin
            total_tests++;
            $display(">>> [7A] Derivative Filter Negation Boundary (-MIN_NEG)");
            deriv_alpha  = FIX_ONE;          // Immediate pass-through
            deriv_kd     = FIX_ONE;          // Kd = 1.0
            deriv_reseed = 0;

            // 1. MIN_NEG (-32768.0) saturates cleanly to MAX_POS (+32767.99998)
            deriv_meas   = MIN_NEG;
            sample_valid = 1; #1;
            if (deriv_d_term == MAX_POS) begin
                $display("  [PASS] Saturating Negation: -(MIN_NEG) correctly clamped to MAX_POS (%08h vs %08h).",
                         deriv_d_term, MAX_POS);
                passed_tests++;
            end else begin
                $display("  [FAIL] Saturating Negation failed: Got %08h, Expected MAX_POS (%08h)",
                         deriv_d_term, MAX_POS);
                failed_tests++;
            end
            @(posedge clk); #1; sample_valid = 0;

            // 2. Normal range: MAX_POS -> -MAX_POS
            total_tests++;
            deriv_reseed = 1; deriv_meas = 0; sample_valid = 1; @(posedge clk); #1; deriv_reseed = 0;
            deriv_meas = MAX_POS; #1;
            if (deriv_d_term == -MAX_POS) begin
                $display("  [PASS] Normal Negation: -(MAX_POS) = -MAX_POS (%08h).", deriv_d_term);
                passed_tests++;
            end else begin
                $display("  [FAIL] Normal Negation failed for MAX_POS: Got %08h (exp %08h)", deriv_d_term, -MAX_POS);
                failed_tests++;
            end
            @(posedge clk); #1; sample_valid = 0;

            // 3. Zero: 0.0 -> 0.0
            total_tests++;
            deriv_reseed = 1; deriv_meas = 0; sample_valid = 1; @(posedge clk); #1; deriv_reseed = 0;
            deriv_meas = FIX_ZERO; #1;
            if (deriv_d_term == FIX_ZERO) begin
                $display("  [PASS] Zero Invariance: -(0.0) = 0.0 (%08h).", deriv_d_term);
                passed_tests++;
            end else begin
                $display("  [FAIL] Zero Invariance failed: Got %08h", deriv_d_term);
                failed_tests++;
            end
            @(posedge clk); #1; sample_valid = 0;

            // 4. Nominal Scale: +1.0 -> -1.0
            total_tests++;
            deriv_reseed = 1; deriv_meas = 0; sample_valid = 1; @(posedge clk); #1; deriv_reseed = 0;
            deriv_meas = FIX_ONE; #1;
            if (deriv_d_term == -FIX_ONE) begin
                $display("  [PASS] Nominal Scale: -(+1.0) = -1.0 (%08h).", deriv_d_term);
                passed_tests++;
            end else begin
                $display("  [FAIL] Nominal Scale failed: Got %08h", deriv_d_term);
                failed_tests++;
            end
            @(posedge clk); #1; sample_valid = 0;

            // 5. Nominal Scale: -1.0 -> +1.0
            total_tests++;
            deriv_reseed = 1; deriv_meas = 0; sample_valid = 1; @(posedge clk); #1; deriv_reseed = 0;
            deriv_meas = -FIX_ONE; #1;
            if (deriv_d_term == FIX_ONE) begin
                $display("  [PASS] Nominal Scale: -(-1.0) = +1.0 (%08h).", deriv_d_term);
                passed_tests++;
            end else begin
                $display("  [FAIL] Nominal Scale failed: Got %08h", deriv_d_term);
                failed_tests++;
            end
            @(posedge clk); #1; sample_valid = 0;
        end

        // 7B: pid_outer_loop Attitude Angle Extrema & Multiplier Dynamic Range
        begin
            total_tests++;
            $display("\n>>> [7B] Outer Loop Attitude Error Extrema");
            outer_mode      = MODE_STABILIZE;
            outer_rc_roll   = 32'sh0003_243F; // +pi rad (180 deg)
            outer_phi       = -32'sh0003_243F; // -pi rad (-180 deg) -> error = 2*pi (6.28318 rad = 0x0006_487E)
            outer_kp_roll   = 32'sh0004_8000; // Kp = 4.5
            #1;

            $display("  Error=2*pi (%0.4f rad), Kp=4.5 -> inner_sp_roll = %08h (%0.4f rad/s)",
                     real'(outer_rc_roll - outer_phi)/65536.0,
                     outer_sp_roll, real'(outer_sp_roll)/65536.0);

            // Expected rate setpoint: 6.28318 * 4.5 = 28.2743 rad/s (0x001C_4639)
            if (outer_sp_roll > 32'sh001C_0000 && outer_sp_roll < 32'sh001D_0000) begin
                $display("  [PASS] Outer attitude loop scales extreme 360-deg angle error without premature overflow.");
                passed_tests++;
            end else begin
                $display("  [FAIL] Outer attitude rate setpoint calculation error: Got %08h", outer_sp_roll);
                failed_tests++;
            end
        end

        // 7C: pid_tilt_comp Extreme Angles & Clamping Protection
        begin
            total_tests++;
            $display("\n>>> [7C] Tilt Compensation Angle Extrema");
            tilt_base  = FIX_HALF; // 0.50 base thrust

            // Nominal angle: 45 deg (0.7854 rad)
            tilt_phi   = 32'sh0000_C90F;
            tilt_theta = 32'sd0;
            #1;
            $display("  phi=45 deg (0.785 rad): comp_thrust=%08h (%0.4f)",
                     tilt_comp_out, real'(tilt_comp_out)/65536.0);

            // Extreme angle: 90 deg (1.5708 rad = 0x0001_9220) -> should clamp to 1.5x * base = 0.75
            tilt_phi   = 32'sh0001_9220;
            #1;
            $display("  phi=90 deg (1.571 rad): comp_thrust=%08h (%0.4f)",
                     tilt_comp_out, real'(tilt_comp_out)/65536.0);

            // Extreme inverted angle: 180 deg (3.1416 rad) -> should still clamp to 1.5x = 0.75
            tilt_phi   = 32'sh0003_243F;
            #1;
            $display("  phi=180 deg (3.142 rad): comp_thrust=%08h (%0.4f)",
                     tilt_comp_out, real'(tilt_comp_out)/65536.0);

            // 0.75 = 0x0000_C000 (49152 counts)
            if (tilt_comp_out == 32'sh0000_C000) begin
                $display("  [PASS] Tilt compensation clamped cleanly to 1.5x maximum (0.750) under extreme tilt angles.");
                passed_tests++;
            end else begin
                $display("  [FAIL] Tilt compensation scale clamp failed: Got %08h, Exp 0000C000", tilt_comp_out);
                failed_tests++;
            end
        end

        // 7D: pid_gain_bank MMIO Unvalidated Configuration Writes
        begin
            total_tests++;
            $display("\n>>> [7D] Gain Bank MMIO Unvalidated Configuration Writes");
            bank_mode = MODE_ACRO;

            // Write negative gain Kp = -10.0 (0xFFF6_0000) to Bank 0 roll_kp (addr 5)
            bank_we    = 1'b1;
            bank_addr  = 8'd5;
            bank_wdata = 32'shFFF6_0000;
            @(posedge clk); #1;
            bank_we    = 1'b0;

            $display("  Wrote negative Kp (-10.0) to Bank 0 -> Active roll_kp output = %08h (%0.2f)",
                     bank_roll_kp, real'(bank_roll_kp)/65536.0);

            if (bank_roll_kp == 32'shFFF6_0000) begin
                $display("  [VULN PROBE] Confirmed: Negative gain accepted without rejection/clamping (software trust model).");
                passed_tests++;
            end else begin
                $display("  [FAIL] Unexpected gain clamping: Got %08h", bank_roll_kp);
                failed_tests++;
            end

            // Write huge gain Kp = 2000.0 (0x07D0_0000) to Bank 0 roll_kp (addr 5)
            total_tests++;
            bank_we    = 1'b1;
            bank_addr  = 8'd5;
            bank_wdata = 32'sh07D0_0000;
            @(posedge clk); #1;
            bank_we    = 1'b0;

            $display("  Wrote large Kp (+2000.0) to Bank 0 -> Active roll_kp output = %08h (%0.2f)",
                     bank_roll_kp, real'(bank_roll_kp)/65536.0);
            if (bank_roll_kp == 32'sh07D0_0000) begin
                $display("  [VULN PROBE] Confirmed: Large gain accepted without headroom restriction.");
                passed_tests++;
            end else begin
                $display("  [FAIL] Large gain unexpectedly modified: Got %08h", bank_roll_kp);
                failed_tests++;
            end

            // Inverted saturation bounds: u_min (+1.0) > u_max (+0.5)
            // Hardware clamper logic in pid_gain_bank ensures roll_u_min <= roll_u_max
            total_tests++;
            bank_we    = 1'b1;
            bank_addr  = 8'd9;  // roll_u_min
            bank_wdata = 32'sh0001_0000; // +1.0
            @(posedge clk); #1;
            bank_addr  = 8'd10; // roll_u_max
            bank_wdata = 32'sh0000_8000; // +0.5
            @(posedge clk); #1;
            bank_we    = 1'b0;
            #1;

            if (bank_roll_u_min == 32'sh0000_8000 && bank_roll_u_max == 32'sh0000_8000) begin
                $display("  [PASS] Inverted Saturation Bounds Clamped: u_min automatically clamped to u_max (+0.50).");
                passed_tests++;
            end else begin
                $display("  [FAIL] Inverted saturation bounds not clamped: u_min=%08h, u_max=%08h",
                         bank_roll_u_min, bank_roll_u_max);
                failed_tests++;
            end
            // Restore normal bounds
            bank_we = 1'b1; bank_addr = 8'd9; bank_wdata = -32'sh0001_0000; @(posedge clk); #1;
            bank_addr = 8'd10; bank_wdata = 32'sh0001_0000; @(posedge clk); #1; bank_we = 1'b0;

            // Invalid / Unmapped MMIO Addresses
            total_tests++;
            bank_we    = 1'b1;
            bank_addr  = 8'd200; // Beyond mapped range (max 160)
            bank_wdata = 32'hDEAD_BEEF;
            @(posedge clk); #1;
            bank_we    = 1'b0;
            #1;
            if (bank_rdata == 32'd0) begin
                $display("  [PASS] Unmapped Address Rejection: Reading addr 200 returned 0 (write ignored).");
                passed_tests++;
            end else begin
                $display("  [FAIL] Unmapped address accepted: Read %08h", bank_rdata);
                failed_tests++;
            end

            // Mode Bank Isolation: Modifying Bank 0 does NOT affect Banks 1, 2, or 3
            total_tests++;
            bank_we    = 1'b1;
            bank_addr  = 8'd5; // Bank 0 roll_kp
            bank_wdata = 32'sh0005_0000; // 5.0
            @(posedge clk); #1;
            bank_we    = 1'b0;
            // Switch mode to Stabilize (Bank 1)
            bank_mode  = MODE_STABILIZE;
            #1;
            // Bank 1 roll_kp default is 2.0 (0x0002_0000)
            if (bank_roll_kp == 32'sh0002_0000) begin
                $display("  [PASS] Mode Bank Isolation: Bank 0 write isolated, Bank 1 roll_kp unaffected (0x0002_0000).");
                passed_tests++;
            end else begin
                $display("  [FAIL] Mode Bank Isolation violated: Bank 1 roll_kp = %08h (expected 00020000)", bank_roll_kp);
                failed_tests++;
            end
            // Restore safe gain in Bank 0
            bank_mode = MODE_ACRO;
            bank_we   = 1'b1; bank_addr = 8'd5; bank_wdata = 32'sh0000_2666; @(posedge clk); #1; bank_we = 1'b0;
        end

        // ---------------------------------------------------------------------
        // SECTION 8: Randomized Reference Model Verification & Quantization Error
        // ---------------------------------------------------------------------
        $display("\n>>> [SECTION 8] Randomized Reference Model Verification & Truncation Checks");
        begin
            logic signed [31:0] rand_a, rand_b;
            logic signed [63:0] ref_prod64;
            logic signed [31:0] ref_p;
            automatic int match_count = 0;
            automatic int mismatch_count = 0;

            // 8A: 100 trials within normal operational non-overflow range (|val| <= 180.0)
            for (int k = 0; k < 100; k++) begin
                rand_a = $signed($urandom_range(0, 32'h00B4_0000)) - 32'sh005A_0000;
                rand_b = $signed($urandom_range(0, 32'h0032_0000)) - 32'sh0019_0000;

                mult_a = rand_a;
                mult_b = rand_b;
                #1;

                ref_prod64 = $signed(rand_a) * $signed(rand_b);
                ref_p      = ref_prod64[47:16];

                if (mult_p === ref_p) begin
                    match_count++;
                end else begin
                    mismatch_count++;
                    $display("  [REF MISMATCH] a=%08h, b=%08h -> RTL=%08h, REF=%08h",
                             rand_a, rand_b, mult_p, ref_p);
                end
            end

            total_tests++;
            $display("  Randomized Operational Range Checks: %0d / 100 bit-exact matches against reference model.",
                     match_count);
            if (mismatch_count == 0) begin
                $display("  [PASS] 100%% bit-exact agreement with independent reference model across operational range.");
                passed_tests++;
            end else begin
                $display("  [FAIL] %0d mismatches detected!", mismatch_count);
                failed_tests++;
            end

            // 8B: 100 trials across full 32-bit signed range (including extreme overflows)
            begin
                automatic int full_match_count = 0;
                automatic int full_mismatch_count = 0;
                logic signed [31:0] full_rand_a, full_rand_b;
                logic signed [31:0] exp_sat_p;
                logic signed [63:0] full_prod64;
                logic signed [63:0] full_shifted;

                for (int m = 0; m < 100; m++) begin
                    full_rand_a = $signed($urandom());
                    full_rand_b = $signed($urandom());

                    mult_a = full_rand_a;
                    mult_b = full_rand_b;
                    #1;

                    full_prod64 = $signed(full_rand_a) * $signed(full_rand_b);
                    full_shifted = full_prod64 >>> 16;
                    if (full_shifted > 64'sh0000_0000_7FFF_FFFF)
                        exp_sat_p = MAX_POS;
                    else if (full_shifted < -64'sh0000_0000_8000_0000)
                        exp_sat_p = MIN_NEG;
                    else
                        exp_sat_p = full_shifted[31:0];

                    if (mult_p === exp_sat_p) begin
                        full_match_count++;
                    end else begin
                        full_mismatch_count++;
                        $display("  [FULL SAT MISMATCH] a=%08h, b=%08h -> RTL=%08h, EXP=%08h",
                                 full_rand_a, full_rand_b, mult_p, exp_sat_p);
                    end
                end

                total_tests++;
                $display("  Randomized Full-Range Overflow Checks: %0d / 100 matches against saturating reference model.",
                         full_match_count);
                if (full_mismatch_count == 0) begin
                    $display("  [PASS] 100%% saturating overflow agreement with reference model across full 32-bit domain.");
                    passed_tests++;
                end else begin
                    $display("  [FAIL] %0d full-range mismatches detected!", full_mismatch_count);
                    failed_tests++;
                end
            end
        end

        // =====================================================================
        // Final Summary
        // =====================================================================
        $display("\n======================================================================");
        $display("  Fixed-Point Boundary Verification Summary");
        $display("  Total Probes & Checks : %0d", total_tests);
        $display("  Passed Checks         : %0d", passed_tests);
        $display("  Failed Checks         : %0d", failed_tests);
        if (failed_tests == 0) begin
            $display("  STATUS                : 100%% CLEAN VERIFICATION (ALL PASS)");
        end else begin
            $display("  STATUS                : DISCREPANCIES DETECTED (FAIL)");
        end
        $display("======================================================================");

        if (failed_tests > 0)
            $fatal(1, "Fixed-point boundary checks failed!");
        else
            $finish;
    end

endmodule : pid_fixed_point_tb
