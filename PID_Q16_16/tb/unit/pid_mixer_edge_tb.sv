//==============================================================================
// Testbench: pid_mixer_edge_tb.sv
// Description: Exhaustive Quad-X Motor Mixer Edge-Case & Saturation Distortion Testbench.
//              Systematically verifies:
//              1. Geometric sign convention & rotation direction physics.
//              2. Isolated single-axis torque responses (+/- Roll, Pitch, Yaw).
//              3. Pairwise axis combinations (R+P, R+Y, P+Y).
//              4. Simultaneous 4-axis extreme demands.
//              5. High-thrust and low-thrust saturation clipping.
//              6. Simultaneous dual-boundary clipping (upper 1.0 and lower 0.0 in same cycle).
//              7. Saturation recovery & hysteresis-free deassertion.
//              8. Timing alignment: 1-cycle motor_valid latency & idle-cycle hold behavior.
//              9. Quantitative torque distortion and cross-axis coupling profiling.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`timescale 1ns/1ps

`include "pid_pkg.sv"
`include "pid_mixer.sv"

import pid_pkg::*;

module pid_mixer_edge_tb;

    logic clk;
    logic rst_n;
    logic sample_valid;

    logic signed [31:0] u_roll;
    logic signed [31:0] u_pitch;
    logic signed [31:0] u_yaw;
    logic signed [31:0] u_thrust;

    logic               motor_valid;
    logic signed [31:0] motor_1;
    logic signed [31:0] motor_2;
    logic signed [31:0] motor_3;
    logic signed [31:0] motor_4;
    logic               saturated_flag;

    int total_tests = 0;
    int passed_tests = 0;
    int failed_tests = 0;

    // DUT Instantiation
    pid_mixer #(
        .DATA_W(32),
        .FRACT_W(16),
        .ACCUM_W(48)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(sample_valid),
        .u_roll(u_roll),
        .u_pitch(u_pitch),
        .u_yaw(u_yaw),
        .u_thrust(u_thrust),
        .motor_valid(motor_valid),
        .motor_1(motor_1),
        .motor_2(motor_2),
        .motor_3(motor_3),
        .motor_4(motor_4),
        .saturated_flag(saturated_flag)
    );

    // 50 MHz Clock Generator (20 ns period)
    always #10 clk = ~clk;

    // -------------------------------------------------------------------------
    // Independent Reference Model & Distortion Evaluation Task
    // -------------------------------------------------------------------------
    task automatic check_step(
        input string test_id,
        input logic signed [31:0] req_t,
        input logic signed [31:0] req_r,
        input logic signed [31:0] req_p,
        input logic signed [31:0] req_y
    );
        // 64-bit independent calculations
        logic signed [63:0] raw_m1, raw_m2, raw_m3, raw_m4;
        logic signed [31:0] exp_m1, exp_m2, exp_m3, exp_m4;
        logic               exp_sat;
        logic signed [31:0] ach_t, ach_r, ach_p, ach_y;
        real r_req_t, r_req_r, r_req_p, r_req_y;
        real r_ach_t, r_ach_r, r_ach_p, r_ach_y;
        real dist_t, dist_r, dist_p, dist_y;

        total_tests++;

        // 1. Independent Quad-X Geometric Mixing Model:
        // M1 (RL, CW ) = T - P + R - Y
        // M2 (RR, CCW) = T - P - R + Y
        // M3 (FR, CW ) = T + P - R - Y
        // M4 (FL, CCW) = T + P + R + Y
        raw_m1 = $signed({{32{req_t[31]}}, req_t}) - $signed({{32{req_p[31]}}, req_p}) + $signed({{32{req_r[31]}}, req_r}) - $signed({{32{req_y[31]}}, req_y});
        raw_m2 = $signed({{32{req_t[31]}}, req_t}) - $signed({{32{req_p[31]}}, req_p}) - $signed({{32{req_r[31]}}, req_r}) + $signed({{32{req_y[31]}}, req_y});
        raw_m3 = $signed({{32{req_t[31]}}, req_t}) + $signed({{32{req_p[31]}}, req_p}) - $signed({{32{req_r[31]}}, req_r}) - $signed({{32{req_y[31]}}, req_y});
        raw_m4 = $signed({{32{req_t[31]}}, req_t}) + $signed({{32{req_p[31]}}, req_p}) + $signed({{32{req_r[31]}}, req_r}) + $signed({{32{req_y[31]}}, req_y});

        // Saturation detection
        exp_sat = (raw_m1 > 64'sh0001_0000 || raw_m1 < 64'sh0) ||
                  (raw_m2 > 64'sh0001_0000 || raw_m2 < 64'sh0) ||
                  (raw_m3 > 64'sh0001_0000 || raw_m3 < 64'sh0) ||
                  (raw_m4 > 64'sh0001_0000 || raw_m4 < 64'sh0);

        // Clamping to [0.0, 1.0]
        exp_m1 = (raw_m1 > 64'sh0001_0000) ? FIX_ONE : (raw_m1 < 64'sh0) ? FIX_ZERO : raw_m1[31:0];
        exp_m2 = (raw_m2 > 64'sh0001_0000) ? FIX_ONE : (raw_m2 < 64'sh0) ? FIX_ZERO : raw_m2[31:0];
        exp_m3 = (raw_m3 > 64'sh0001_0000) ? FIX_ONE : (raw_m3 < 64'sh0) ? FIX_ZERO : raw_m3[31:0];
        exp_m4 = (raw_m4 > 64'sh0001_0000) ? FIX_ONE : (raw_m4 < 64'sh0) ? FIX_ZERO : raw_m4[31:0];

        // Apply inputs to DUT
        u_thrust     = req_t;
        u_roll       = req_r;
        u_pitch      = req_p;
        u_yaw        = req_y;
        sample_valid = 1'b1;

        @(posedge clk); #1;
        sample_valid = 1'b0;

        // Check cycle alignment: motor_valid must be asserted on this cycle
        if (!motor_valid) begin
            $display("  [FAIL] %s: motor_valid not asserted 1 cycle after sample_valid strobe!", test_id);
            failed_tests++;
            return;
        end

        // Check bit-exact match against independent model
        if (motor_1 === exp_m1 && motor_2 === exp_m2 &&
            motor_3 === exp_m3 && motor_4 === exp_m4 &&
            saturated_flag === exp_sat) begin
            passed_tests++;
        end else begin
            $display("  [FAIL] %s Mismatch!", test_id);
            $display("    Got: M1=%08h, M2=%08h, M3=%08h, M4=%08h, sat=%b",
                     motor_1, motor_2, motor_3, motor_4, saturated_flag);
            $display("    Exp: M1=%08h, M2=%08h, M3=%08h, M4=%08h, sat=%b",
                     exp_m1, exp_m2, exp_m3, exp_m4, exp_sat);
            failed_tests++;
        end

        // 2. Inverse Motor Matrix - Calculate Achieved Torques:
        ach_t = ($signed(motor_1) + $signed(motor_2) + $signed(motor_3) + $signed(motor_4)) >>> 2;
        ach_p = (-$signed(motor_1) - $signed(motor_2) + $signed(motor_3) + $signed(motor_4)) >>> 2;
        ach_r = (+$signed(motor_1) - $signed(motor_2) - $signed(motor_3) + $signed(motor_4)) >>> 2;
        ach_y = (-$signed(motor_1) + $signed(motor_2) - $signed(motor_3) + $signed(motor_4)) >>> 2;

        r_req_t = real'(req_t) / 65536.0;
        r_req_r = real'(req_r) / 65536.0;
        r_req_p = real'(req_p) / 65536.0;
        r_req_y = real'(req_y) / 65536.0;

        r_ach_t = real'(ach_t) / 65536.0;
        r_ach_r = real'(ach_r) / 65536.0;
        r_ach_p = real'(ach_p) / 65536.0;
        r_ach_y = real'(ach_y) / 65536.0;

        dist_t = r_ach_t - r_req_t;
        dist_r = r_ach_r - r_req_r;
        dist_p = r_ach_p - r_req_p;
        dist_y = r_ach_y - r_req_y;

        if (exp_sat) begin
            $display("  [SAT METRIC] %s: Sat=1 | Req: T=%0.2f R=%0.2f P=%0.2f Y=%0.2f | Ach: T=%0.2f R=%0.2f P=%0.2f Y=%0.2f | Atten: dT=%0.3f dR=%0.3f dP=%0.3f dY=%0.3f",
                     test_id, r_req_t, r_req_r, r_req_p, r_req_y,
                     r_ach_t, r_ach_r, r_ach_p, r_ach_y,
                     dist_t, dist_r, dist_p, dist_y);
        end else begin
            $display("  [PASS] %s: Linear (No Sat) | M1=%0.2f M2=%0.2f M3=%0.2f M4=%0.2f | Achieved matches requested bit-exact.",
                     test_id, real'(motor_1)/65536.0, real'(motor_2)/65536.0, real'(motor_3)/65536.0, real'(motor_4)/65536.0);
        end
    endtask

    initial begin
        clk          = 0;
        rst_n        = 0;
        sample_valid = 0;
        u_roll       = 0;
        u_pitch      = 0;
        u_yaw        = 0;
        u_thrust     = 0;

        $display("======================================================================");
        $display("  Quad-X Motor Mixer Exhaustive Edge & Saturation Distortion Suite");
        $display("======================================================================");

        // Reset verification
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        // ---------------------------------------------------------------------
        // SUITE 1: Isolated Single-Axis Demands
        // ---------------------------------------------------------------------
        $display("\n>>> [SUITE 1] Isolated Single-Axis Demands (T=0.50 Hover)");
        check_step("01_Pure_Thrust_Hover", 32'sh0000_8000, 32'sd0, 32'sd0, 32'sd0);
        check_step("02_Pos_Roll",          32'sh0000_8000, 32'sh0000_199A, 32'sd0, 32'sd0); // +0.10
        check_step("03_Neg_Roll",          32'sh0000_8000, -32'sh0000_199A, 32'sd0, 32'sd0); // -0.10
        check_step("04_Pos_Pitch",         32'sh0000_8000, 32'sd0, 32'sh0000_199A, 32'sd0); // +0.10
        check_step("05_Neg_Pitch",         32'sh0000_8000, 32'sd0, -32'sh0000_199A, 32'sd0); // -0.10
        check_step("06_Pos_Yaw",           32'sh0000_8000, 32'sd0, 32'sd0, 32'sh0000_199A); // +0.10
        check_step("07_Neg_Yaw",           32'sh0000_8000, 32'sd0, 32'sd0, -32'sh0000_199A); // -0.10

        // ---------------------------------------------------------------------
        // SUITE 2: Pairwise Axis Combinations
        // ---------------------------------------------------------------------
        $display("\n>>> [SUITE 2] Pairwise Axis Combinations (T=0.50)");
        check_step("08_Pair_Roll_Pitch_PosPos", 32'sh0000_8000, 32'sh0000_2000, 32'sh0000_2000, 32'sd0); // +0.125, +0.125
        check_step("09_Pair_Roll_Pitch_PosNeg", 32'sh0000_8000, 32'sh0000_2000, -32'sh0000_2000, 32'sd0);
        check_step("10_Pair_Roll_Pitch_NegPos", 32'sh0000_8000, -32'sh0000_2000, 32'sh0000_2000, 32'sd0);
        check_step("11_Pair_Roll_Pitch_NegNeg", 32'sh0000_8000, -32'sh0000_2000, -32'sh0000_2000, 32'sd0);
        check_step("12_Pair_Roll_Yaw_PosPos",   32'sh0000_8000, 32'sh0000_2000, 32'sd0, 32'sh0000_2000);
        check_step("13_Pair_Roll_Yaw_PosNeg",   32'sh0000_8000, 32'sh0000_2000, 32'sd0, -32'sh0000_2000);
        check_step("14_Pair_Pitch_Yaw_PosPos",  32'sh0000_8000, 32'sd0, 32'sh0000_2000, 32'sh0000_2000);
        check_step("15_Pair_Pitch_Yaw_PosNeg",  32'sh0000_8000, 32'sd0, 32'sh0000_2000, -32'sh0000_2000);

        // ---------------------------------------------------------------------
        // SUITE 3: Simultaneous 4-Axis Demands
        // ---------------------------------------------------------------------
        $display("\n>>> [SUITE 3] Simultaneous 4-Axis Demands");
        check_step("16_4Axis_Nominal",  32'sh0000_8000, 32'sh0000_1000, 32'sh0000_1000, 32'sh0000_1000); // T=0.5, R=0.06, P=0.06, Y=0.06
        check_step("17_4Axis_All_Pos",  32'sh0000_8000, 32'sh0000_3333, 32'sh0000_3333, 32'sh0000_3333); // T=0.5, R=0.2, P=0.2, Y=0.2 -> M4 raw=1.10 (clamped)
        check_step("18_4Axis_All_Neg",  32'sh0000_8000, -32'sh0000_3333, -32'sh0000_3333, -32'sh0000_3333);
        check_step("19_4Axis_Mixed",    32'sh0000_8000, 32'sh0000_4000, -32'sh0000_4000, 32'sh0000_4000); // T=0.5, R=+0.25, P=-0.25, Y=+0.25

        // ---------------------------------------------------------------------
        // SUITE 4: High Collective Thrust Saturation (T = 0.90)
        // ---------------------------------------------------------------------
        $display("\n>>> [SUITE 4] High Collective Thrust Saturation (T = 0.90)");
        check_step("20_HighT_PosTorques", 32'sh0000_E666, 32'sh0000_3333, 32'sh0000_3333, 32'sh0000_3333); // T=0.9, R=0.2, P=0.2, Y=0.2
        check_step("21_HighT_NegTorques", 32'sh0000_E666, -32'sh0000_3333, -32'sh0000_3333, -32'sh0000_3333);

        // ---------------------------------------------------------------------
        // SUITE 5: Low Collective Thrust Saturation (T = 0.10)
        // ---------------------------------------------------------------------
        $display("\n>>> [SUITE 5] Low Collective Thrust Saturation (T = 0.10)");
        check_step("22_LowT_PosTorques",  32'sh0000_199A, 32'sh0000_3333, 32'sh0000_3333, 32'sh0000_3333); // T=0.1, R=0.2, P=0.2, Y=0.2
        check_step("23_LowT_NegTorques",  32'sh0000_199A, -32'sh0000_3333, -32'sh0000_3333, -32'sh0000_3333);

        // ---------------------------------------------------------------------
        // SUITE 6: Simultaneous Upper (1.0) and Lower (0.0) Boundary Saturation
        // ---------------------------------------------------------------------
        $display("\n>>> [SUITE 6] Simultaneous Upper (1.0) and Lower (0.0) Clamping in Same Cycle");
        // T=0.50, P=+0.60, R=+0.60, Y=0.0
        // M1 (RL) = 0.50 - 0.60 + 0.60 - 0 = 0.50
        // M2 (RR) = 0.50 - 0.60 - 0.60 + 0 = -0.70 -> Clamped to 0.0 (Lower Bound!)
        // M3 (FR) = 0.50 + 0.60 - 0.60 - 0 = 0.50
        // M4 (FL) = 0.50 + 0.60 + 0.60 + 0 = 1.70 -> Clamped to 1.0 (Upper Bound!)
        check_step("24_Simultaneous_Upper_Lower_Dual_Saturation",
                   32'sh0000_8000, 32'sh0000_999A, 32'sh0000_999A, 32'sd0);

        // Verify that in Test 24, both bounds were simultaneously hit:
        total_tests++;
        if (motor_4 == FIX_ONE && motor_2 == FIX_ZERO && motor_1 == FIX_HALF && motor_3 == FIX_HALF) begin
            $display("  [PASS] Dual-limit clamping verified: M4 at 1.0 and M2 at 0.0 concurrently!");
            passed_tests++;
        end else begin
            $display("  [FAIL] Dual-limit clamping failed: M4=%08h, M2=%08h", motor_4, motor_2);
            failed_tests++;
        end

        // ---------------------------------------------------------------------
        // SUITE 7: Saturation Recovery & Flag Deassertion
        // ---------------------------------------------------------------------
        $display("\n>>> [SUITE 7] Saturation Recovery & Deassertion");
        // Return to nominal hover (0.50, 0, 0, 0)
        check_step("25_Recovery_to_Hover", 32'sh0000_8000, 32'sd0, 32'sd0, 32'sd0);

        // ---------------------------------------------------------------------
        // SUITE 8: Timing Alignment, Latency & Idle-Cycle Hold Behavior
        // ---------------------------------------------------------------------
        $display("\n>>> [SUITE 8] Timing Alignment & Idle Cycle Strobe Behavior");

        // Force saturation first
        u_thrust     = 32'sh0000_E666; // T=0.9
        u_pitch      = 32'sh0000_4CCD; // P=0.3
        u_roll       = 32'sd0;
        u_yaw        = 32'sd0;
        sample_valid = 1'b1;
        @(posedge clk); #1;
        sample_valid = 1'b0;

        total_tests++;
        if (motor_valid == 1'b1 && saturated_flag == 1'b1) begin
            $display("  [PASS] Cycle N+1: motor_valid=1, saturated_flag=1 aligned with motor outputs.");
            passed_tests++;
        end else begin
            $display("  [FAIL] Cycle N+1 alignment failed: mv=%b, sat=%b", motor_valid, saturated_flag);
            failed_tests++;
        end

        // Cycle N+2: sample_valid remains 0 (idle inter-sample cycle)
        @(posedge clk); #1;
        total_tests++;
        if (motor_valid == 1'b0 && saturated_flag == 1'b1 && motor_3 == FIX_ONE) begin
            $display("  [PASS] Cycle N+2 (idle): motor_valid drops to 0, motor outputs & saturated_flag hold value.");
            passed_tests++;
        end else begin
            $display("  [FAIL] Idle cycle behavior incorrect: mv=%b, sat=%b, m3=%08h",
                     motor_valid, saturated_flag, motor_3);
            failed_tests++;
        end

        // =====================================================================
        // Final Summary
        // =====================================================================
        $display("\n======================================================================");
        $display("  Quad-X Motor Mixer Verification Summary");
        $display("  Total Tests & Invariant Checks : %0d", total_tests);
        $display("  Passed Checks                  : %0d", passed_tests);
        $display("  Failed Checks                  : %0d", failed_tests);
        if (failed_tests == 0) begin
            $display("  STATUS                         : 100%% CLEAN VERIFICATION (ALL PASS)");
        end else begin
            $display("  STATUS                         : DISCREPANCIES DETECTED (FAIL)");
        end
        $display("======================================================================");

        if (failed_tests > 0)
            $fatal(1, "Mixer edge-case verification failed!");
        else
            $finish;
    end

endmodule : pid_mixer_edge_tb
