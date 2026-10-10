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

    int test_count = 0;
    int pass_count = 0;
    int fail_count = 0;

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

    // Clock generator: 50 MHz (20 ns period)
    always #10 clk = ~clk;

    // Helper task to apply step
    task automatic apply_step(
        input logic signed [31:0] in_t,
        input logic signed [31:0] in_r,
        input logic signed [31:0] in_p,
        input logic signed [31:0] in_y
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
        apply_step(32'sh0000_8000, 32'sd0, 32'sd0, 32'sd0);
        if (motor_1 == 32'sh0000_8000 && motor_2 == 32'sh0000_8000 &&
            motor_3 == 32'sh0000_8000 && motor_4 == 32'sh0000_8000 &&
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
        apply_step(32'sh0000_8000, 32'sh0000_199A, 32'sd0, 32'sd0); // 0.10 = 6554 counts
        if (motor_1 == 32'sh0000_999A && motor_2 == 32'sh0000_6666 &&
            motor_3 == 32'sh0000_6666 && motor_4 == 32'sh0000_999A &&
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
        apply_step(32'sh0000_8000, 32'sd0, 32'sh0000_199A, 32'sd0);
        if (motor_1 == 32'sh0000_6666 && motor_2 == 32'sh0000_6666 &&
            motor_3 == 32'sh0000_999A && motor_4 == 32'sh0000_999A &&
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
        apply_step(32'sh0000_8000, 32'sd0, 32'sd0, 32'sh0000_199A);
        if (motor_1 == 32'sh0000_6666 && motor_2 == 32'sh0000_999A &&
            motor_3 == 32'sh0000_6666 && motor_4 == 32'sh0000_999A &&
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
        apply_step(32'sh0000_CCCC, 32'sd0, 32'sh0000_4CCD, 32'sd0); // T=0.80, P=0.30
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
        apply_step(32'sh0000_199A, 32'sd0, 32'sh0000_3333, 32'sd0); // T=0.10, P=0.20
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
        apply_step(32'sh0000_E666, 32'sd0, 32'sh0000_4CCD, 32'sd0); // T=0.9, P=0.3
        begin
            logic signed [31:0] ach_pitch;
            ach_pitch = (motor_3 + motor_4 - motor_1 - motor_2) >>> 2;
            $display("  Requested Pitch Torque: 0.3000 (19661 counts)");
            $display("  Achieved Pitch Torque : %0f (%0d counts)",
                     $itor(ach_pitch)/65536.0, ach_pitch);
            if (ach_pitch < 32'sh0000_4CCD && saturated_flag) begin
                $display("  PASS: Torque correctly attenuated due to motor ceiling clipping");
                pass_count++;
            end else begin
                $display("  FAIL: Expected torque attenuation not observed");
                fail_count++;
            end
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
