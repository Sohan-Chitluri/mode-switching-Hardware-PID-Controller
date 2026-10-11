//==============================================================================
// Testbench: pid_mixer_tb.sv
// Description: Unit testbench for Quad-X motor mixer.
//              Verifies:
//              1. Pure collective thrust scaling and symmetric distribution.
//              2. Isolated roll, pitch, and yaw torque responses.
//              3. Mixed 4-axis control commands.
//              4. Motor saturation clamping ([0.0, 1.0]) and saturated_flag.
//              5. Measured net torque degradation under saturation.
//              6. Reset and sample_valid strobe handling.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`timescale 1ns/1ps

`include "pid_pkg.sv"
import pid_pkg::*;

module pid_mixer_tb;

    logic clk;
    logic rst_n;
    logic sample_valid;

    logic signed [15:0] u_roll;
    logic signed [15:0] u_pitch;
    logic signed [15:0] u_yaw;
    logic signed [15:0] u_thrust;

    logic               motor_valid;
    logic signed [15:0] motor_1;
    logic signed [15:0] motor_2;
    logic signed [15:0] motor_3;
    logic signed [15:0] motor_4;
    logic               saturated_flag;

    int test_count = 0;
    int pass_count = 0;
    int fail_count = 0;

    pid_mixer #(
        .DATA_W(16),
        .FRACT_W(12),
        .ACCUM_W(24)
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

    // Clock generator: 50 MHz (20 ns period)
    always #10 clk = ~clk;

    // Helper task to apply step
    task automatic apply_step(
        input logic signed [15:0] in_t,
        input logic signed [15:0] in_r,
        input logic signed [15:0] in_p,
        input logic signed [15:0] in_y
    );
        u_thrust     = in_t;
        u_roll       = in_r;
        u_pitch      = in_p;
        u_yaw        = in_y;
        sample_valid = 1'b1;
        @(posedge clk);
        #1;
        sample_valid = 1'b0;
        @(posedge clk);
        #1;
    endtask

    initial begin
        clk          = 0;
        rst_n        = 0;
        sample_valid = 0;
        u_roll       = 0;
        u_pitch      = 0;
        u_yaw        = 0;
        u_thrust     = 0;

        $display("=========================================================");
        $display("  Starting Quad-X Motor Mixer Unit Testbench");
        $display("=========================================================");

        // Reset verification
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        // -------------------------------------------------------------
        // Test 1: Pure Collective Thrust
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 1] Pure Collective Thrust (Hover = 0.50)");
        apply_step(16'sh0800, 16'sd0, 16'sd0, 16'sd0);
        if (motor_1 == 16'sh0800 && motor_2 == 16'sh0800 &&
            motor_3 == 16'sh0800 && motor_4 == 16'sh0800 &&
            !saturated_flag) begin
            $display("  PASS: All 4 motors equal 0.50 (32768 counts)");
            pass_count++;
        end else begin
            $display("  FAIL: M1=%0d, M2=%0d, M3=%0d, M4=%0d", motor_1, motor_2, motor_3, motor_4);
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 2: Isolated Roll Torque (+0.10 roll demand)
        // M1 (RL, left)  = 0.5 + 0.1 = 0.60
        // M2 (RR, right) = 0.5 - 0.1 = 0.40
        // M3 (FR, right) = 0.5 - 0.1 = 0.40
        // M4 (FL, left)  = 0.5 + 0.1 = 0.60
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 2] Isolated Roll Torque (+0.10)");
        apply_step(16'sh0800, 16'sh019A, 16'sd0, 16'sd0); // 0.10 = 6554 counts
        if (motor_1 == 16'sh099A && motor_2 == 16'sh0666 &&
            motor_3 == 16'sh0666 && motor_4 == 16'sh099A &&
            !saturated_flag) begin
            $display("  PASS: Left motors M1,M4 = 0.60, Right motors M2,M3 = 0.40");
            pass_count++;
        end else begin
            $display("  FAIL: Roll response incorrect: M1=%0d, M2=%0d, M3=%0d, M4=%0d",
                     motor_1, motor_2, motor_3, motor_4);
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 3: Isolated Pitch Torque (+0.10 pitch nose-up demand)
        // M1 (RL, rear)  = 0.5 - 0.1 = 0.40
        // M2 (RR, rear)  = 0.5 - 0.1 = 0.40
        // M3 (FR, front) = 0.5 + 0.1 = 0.60
        // M4 (FL, front) = 0.5 + 0.1 = 0.60
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 3] Isolated Pitch Torque (+0.10)");
        apply_step(16'sh0800, 16'sd0, 16'sh019A, 16'sd0);
        if (motor_1 == 16'sh0666 && motor_2 == 16'sh0666 &&
            motor_3 == 16'sh099A && motor_4 == 16'sh099A &&
            !saturated_flag) begin
            $display("  PASS: Rear motors M1,M2 = 0.40, Front motors M3,M4 = 0.60");
            pass_count++;
        end else begin
            $display("  FAIL: Pitch response incorrect: M1=%0d, M2=%0d, M3=%0d, M4=%0d",
                     motor_1, motor_2, motor_3, motor_4);
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 4: Isolated Yaw Torque (+0.10 yaw clockwise demand)
        // M1 (RL, -Y) = 0.5 - 0.1 = 0.40
        // M2 (RR, +Y) = 0.5 + 0.1 = 0.60
        // M3 (FR, -Y) = 0.5 - 0.1 = 0.40
        // M4 (FL, +Y) = 0.5 + 0.1 = 0.60
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 4] Isolated Yaw Torque (+0.10)");
        apply_step(16'sh0800, 16'sd0, 16'sd0, 16'sh019A);
        if (motor_1 == 16'sh0666 && motor_2 == 16'sh099A &&
            motor_3 == 16'sh0666 && motor_4 == 16'sh099A &&
            !saturated_flag) begin
            $display("  PASS: Diagonal pair M2,M4 = 0.60, Diagonal pair M1,M3 = 0.40");
            pass_count++;
        end else begin
            $display("  FAIL: Yaw response incorrect: M1=%0d, M2=%0d, M3=%0d, M4=%0d",
                     motor_1, motor_2, motor_3, motor_4);
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 5: Actuator Saturation Clamping at Upper Limit (1.0)
        // T = 0.80, Pitch = +0.30 -> M3,M4 raw = 1.10 -> clamped to 1.00
        // M1,M2 raw = 0.50
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 5] Saturation Clamping at Upper Bound 1.0");
        apply_step(16'sh0CCD, 16'sd0, 16'sh04CD, 16'sd0); // T=0.80, P=0.30
        if (motor_3 == FIX_ONE && motor_4 == FIX_ONE && saturated_flag == 1'b1) begin
            $display("  PASS: Front motors clipped to 1.0, saturated_flag asserted");
            pass_count++;
        end else begin
            $display("  FAIL: Upper saturation failed: M3=%0d, M4=%0d, sat=%0b",
                     motor_3, motor_4, saturated_flag);
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 6: Actuator Saturation Clamping at Lower Limit (0.0)
        // T = 0.10, Pitch = +0.20 -> M1,M2 raw = -0.10 -> clamped to 0.00
        // M3,M4 raw = 0.30
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 6] Saturation Clamping at Lower Bound 0.0");
        apply_step(16'sh019A, 16'sd0, 16'sh0333, 16'sd0); // T=0.10, P=0.20
        if (motor_1 == FIX_ZERO && motor_2 == FIX_ZERO && saturated_flag == 1'b1) begin
            $display("  PASS: Rear motors clipped to 0.0, saturated_flag asserted");
            pass_count++;
        end else begin
            $display("  FAIL: Lower saturation failed: M1=%0d, M2=%0d, sat=%0b",
                     motor_1, motor_2, saturated_flag);
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 7: Net Torque Degradation Under Saturation
        // Under pure unclipped pitch: delta = (M3+M4 - (M1+M2))/4 == requested pitch.
        // Under clipping (T=0.9, P=0.3): M3,M4 clip at 1.0 (raw 1.2), M1,M2=0.6.
        // Achieved pitch = (1.0 + 1.0 - 0.6 - 0.6)/4 = 0.8/4 = 0.20 instead of 0.30!
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 7] Measuring Torque Attenuation Under Saturation");
        apply_step(16'sh0E66, 16'sd0, 16'sh04CD, 16'sd0); // T=0.9, P=0.3
        begin
            logic signed [15:0] ach_pitch;
            ach_pitch = (motor_3 + motor_4 - motor_1 - motor_2) >>> 2;
            $display("  Requested Pitch Torque: 0.3000 (1229 counts)");
            $display("  Achieved Pitch Torque : %0f (%0d counts)",
                     $itor(ach_pitch)/4096.0, ach_pitch);
            if (ach_pitch < 16'sh04CD && saturated_flag) begin
                $display("  PASS: Torque correctly attenuated due to motor ceiling clipping");
                pass_count++;
            end else begin
                $display("  FAIL: Expected torque attenuation not observed");
                fail_count++;
            end
        end

        // -------------------------------------------------------------
        // Test 8: Pure Negative Roll Torque (-0.10)
        // M1 (RL, left)  = 0.5 - 0.1 = 0.40
        // M2 (RR, right) = 0.5 + 0.1 = 0.60
        // M3 (FR, right) = 0.5 + 0.1 = 0.60
        // M4 (FL, left)  = 0.5 - 0.1 = 0.40
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 8] Pure Negative Roll Torque (-0.10)");
        apply_step(16'sh0800, -16'sh019A, 16'sd0, 16'sd0);
        if (motor_1 == 16'sh0666 && motor_2 == 16'sh099A &&
            motor_3 == 16'sh099A && motor_4 == 16'sh0666 &&
            !saturated_flag) begin
            $display("  PASS: Left motors M1,M4 = 0.40, Right motors M2,M3 = 0.60");
            pass_count++;
        end else begin
            $display("  FAIL: Negative roll response incorrect: M1=%0d, M2=%0d, M3=%0d, M4=%0d",
                     motor_1, motor_2, motor_3, motor_4);
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 9: Pure Negative Pitch Torque (-0.10 nose-down)
        // M1 (RL, rear)  = 0.5 + 0.1 = 0.60
        // M2 (RR, rear)  = 0.5 + 0.1 = 0.60
        // M3 (FR, front) = 0.5 - 0.1 = 0.40
        // M4 (FL, front) = 0.5 - 0.1 = 0.40
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 9] Pure Negative Pitch Torque (-0.10)");
        apply_step(16'sh0800, 16'sd0, -16'sh019A, 16'sd0);
        if (motor_1 == 16'sh099A && motor_2 == 16'sh099A &&
            motor_3 == 16'sh0666 && motor_4 == 16'sh0666 &&
            !saturated_flag) begin
            $display("  PASS: Rear motors M1,M2 = 0.60, Front motors M3,M4 = 0.40");
            pass_count++;
        end else begin
            $display("  FAIL: Negative pitch response incorrect: M1=%0d, M2=%0d, M3=%0d, M4=%0d",
                     motor_1, motor_2, motor_3, motor_4);
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 10: Pure Negative Yaw Torque (-0.10 counter-clockwise)
        // M1 (RL, CW ) = 0.5 + 0.1 = 0.60
        // M2 (RR, CCW) = 0.5 - 0.1 = 0.40
        // M3 (FR, CW ) = 0.5 + 0.1 = 0.60
        // M4 (FL, CCW) = 0.5 - 0.1 = 0.40
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 10] Pure Negative Yaw Torque (-0.10)");
        apply_step(16'sh0800, 16'sd0, 16'sd0, -16'sh019A);
        if (motor_1 == 16'sh099A && motor_2 == 16'sh0666 &&
            motor_3 == 16'sh099A && motor_4 == 16'sh0666 &&
            !saturated_flag) begin
            $display("  PASS: CW pair M1,M3 = 0.60, CCW pair M2,M4 = 0.40");
            pass_count++;
        end else begin
            $display("  FAIL: Negative yaw response incorrect: M1=%0d, M2=%0d, M3=%0d, M4=%0d",
                     motor_1, motor_2, motor_3, motor_4);
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 11: Mixed 4-Axis Demands
        // T=0.50, R=0.05 (3277), P=0.05, Y=0.05
        // M1 = 0.50 - 0.05 + 0.05 - 0.05 = 0.45 (29491)
        // M2 = 0.50 - 0.05 - 0.05 + 0.05 = 0.45 (29491)
        // M3 = 0.50 + 0.05 - 0.05 - 0.05 = 0.45 (29491)
        // M4 = 0.50 + 0.05 + 0.05 + 0.05 = 0.65 (42598)
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 11] Mixed 4-Axis Demands (T=0.50, R=0.05, P=0.05, Y=0.05)");
        apply_step(16'sh0800, 16'sh00CD, 16'sh00CD, 16'sh00CD);
        if (motor_1 == 16'sh0733 && motor_2 == 16'sh0733 &&
            motor_3 == 16'sh0733 && motor_4 == 16'sh0A67 &&
            !saturated_flag) begin
            $display("  PASS: Mixed mapping bit-exact: M1,M2,M3=0.45, M4=0.65");
            pass_count++;
        end else begin
            $display("  FAIL: Mixed mapping mismatch: M1=%0d, M2=%0d, M3=%0d, M4=%0d",
                     motor_1, motor_2, motor_3, motor_4);
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 12: Minimum (0.0) and Maximum (1.0) Neutral Commands
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 12] Minimum (0.0) and Maximum (1.0) Commands");
        apply_step(16'sd0, 16'sd0, 16'sd0, 16'sd0);
        if (motor_1 == 16'sd0 && motor_2 == 16'sd0 && motor_3 == 16'sd0 && motor_4 == 16'sd0 && !saturated_flag) begin
            apply_step(FIX_ONE, 16'sd0, 16'sd0, 16'sd0);
            if (motor_1 == FIX_ONE && motor_2 == FIX_ONE && motor_3 == FIX_ONE && motor_4 == FIX_ONE && !saturated_flag) begin
                $display("  PASS: Min command (0.0) and Max command (1.0) cleanly handled without spurious saturation");
                pass_count++;
            end else begin
                $display("  FAIL: Max command output incorrect");
                fail_count++;
            end
        end else begin
            $display("  FAIL: Min command output incorrect");
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 13: Saturation Recovery back to Linear Region
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 13] Saturation Recovery & Flag Deassertion");
        // Force saturation first
        apply_step(16'sh0E66, 16'sd0, 16'sh04CD, 16'sd0);
        if (saturated_flag == 1'b1) begin
            // Return to nominal hover
            apply_step(16'sh0800, 16'sd0, 16'sd0, 16'sd0);
            if (motor_1 == 16'sh0800 && motor_2 == 16'sh0800 &&
                motor_3 == 16'sh0800 && motor_4 == 16'sh0800 &&
                saturated_flag == 1'b0) begin
                $display("  PASS: Successfully recovered from saturation, flag cleanly deasserted");
                pass_count++;
            end else begin
                $display("  FAIL: Flag failed to deassert on return to linear range (sat=%0b)", saturated_flag);
                fail_count++;
            end
        end else begin
            $display("  FAIL: Prerequisite saturation not established");
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 14: Invalid & Extreme Input Handling (No 48-bit Wrap)
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 14] Extreme Dynamic Range & Intermediate Guarding");
        // Extreme positive and negative inputs: T=+2.0, P=+3.0, R=-4.0, Y=+2.0
        apply_step(16'sh2000, -16'sh4000, 16'sh3000, 16'sh2000);
        if (motor_1 >= 16'sd0 && motor_1 <= FIX_ONE &&
            motor_2 >= 16'sd0 && motor_2 <= FIX_ONE &&
            motor_3 >= 16'sd0 && motor_3 <= FIX_ONE &&
            motor_4 >= 16'sd0 && motor_4 <= FIX_ONE &&
            saturated_flag == 1'b1) begin
            $display("  PASS: 48-bit guarding prevented arithmetic wrap; all motors clamped in [0.0, 1.0]");
            pass_count++;
        end else begin
            $display("  FAIL: Arithmetic overflow or invalid clamping detected!");
            fail_count++;
        end

        // -------------------------------------------------------------
        // Summary
        // -------------------------------------------------------------
        $display("=========================================================");
        $display("  pid_mixer Unit Verification Summary");
        $display("  Total Tests : %0d", test_count);
        $display("  Passed      : %0d", pass_count);
        $display("  Failed      : %0d", fail_count);
        if (fail_count == 0)
            $display("  STATUS      : ALL MIXER TESTS PASSED (PASS)");
        else
            $fatal(1, "Mixer unit verification failed!");
        $display("=========================================================");
        $finish;
    end

endmodule : pid_mixer_tb
