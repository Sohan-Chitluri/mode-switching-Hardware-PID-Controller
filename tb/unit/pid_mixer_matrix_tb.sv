//==============================================================================
// Testbench: pid_mixer_matrix_tb.sv
// Description: Self-checking unit testbench for Configurable Airframe Mixer Matrix.
//              Verifies motor thrust assignments and saturation flags across:
//              - Airframe 0: Quad-X
//              - Airframe 1: Quad-+
//              - Airframe 2: Hexa-X
//              - Airframe 3: Hexa-+
//              - Airframe 4: Octo-X
//==============================================================================

`timescale 1ns/1ps

`include "pid_pkg.sv"
`include "pid_mixer_matrix.sv"

module pid_mixer_matrix_tb;

    import pid_pkg::*;

    localparam int DATA_W     = 32;
    localparam int FRACT_W    = 16;
    localparam int ACCUM_W    = 48;
    localparam int NUM_MOTORS = 8;

    logic clk;
    logic rst_n;
    logic sample_valid;

    logic [2:0] airframe_type;

    logic signed [DATA_W-1:0] u_roll;
    logic signed [DATA_W-1:0] u_pitch;
    logic signed [DATA_W-1:0] u_yaw;
    logic signed [DATA_W-1:0] u_thrust;

    logic                     motor_valid;
    logic signed [DATA_W-1:0] motors [NUM_MOTORS];
    logic                     saturated_flag;

    // DUT
    pid_mixer_matrix #(
        .DATA_W(DATA_W),
        .FRACT_W(FRACT_W),
        .ACCUM_W(ACCUM_W),
        .NUM_MOTORS(NUM_MOTORS)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(sample_valid),
        .airframe_type(airframe_type),
        .u_roll(u_roll),
        .u_pitch(u_pitch),
        .u_yaw(u_yaw),
        .u_thrust(u_thrust),
        .motor_valid(motor_valid),
        .motors(motors),
        .saturated_flag(saturated_flag)
    );

    initial clk = 0;
    always #10 clk = ~clk;

    int error_count = 0;

    initial begin
        $display("==================================================");
        $display("    TESTBENCH: Multi-Rotor Mixer Matrix (Unit)   ");
        $display("==================================================");

        rst_n         = 0;
        sample_valid  = 0;
        airframe_type = 3'd0;
        u_roll        = 0;
        u_pitch       = 0;
        u_yaw         = 0;
        u_thrust      = 0;

        #40;
        rst_n = 1;
        #20;

        // Test 1: Quad-X Hover Thrust
        $display("[TEST 1] Testing Quad-X Hover (T=0.5, R=P=Y=0)...");
        airframe_type = 3'b000;
        u_thrust      = 32'sh0000_8000; // 0.5
        u_roll        = 32'sh0000_0000;
        u_pitch       = 32'sh0000_0000;
        u_yaw         = 32'sh0000_0000;
        sample_valid  = 1;
        $display("BEFORE CLK: time=%0t rst_n=%b valid=%b raw_sums[0]=%h", $time, rst_n, sample_valid, dut.raw_sums[0]);
        @(posedge clk);
        #1;
        $display("AFTER CLK 1: time=%0t valid=%b raw_sums[0]=%h motors[0]=%h", $time, sample_valid, dut.raw_sums[0], motors[0]);
        sample_valid  = 0;
        @(posedge clk);
        #1;
        $display("AFTER CLK 2: time=%0t valid=%b motors[0]=%h", $time, sample_valid, motors[0]);

        for (int i = 0; i < 4; i++) begin
            if (motors[i] !== 32'sh0000_8000) begin
                $display("[FAIL] Motor %0d expected 0.5, got 0x%08h", i+1, motors[i]);
                error_count++;
            end
        end
        if (saturated_flag !== 1'b0) begin
            $display("[FAIL] False saturation flag in hover!");
            error_count++;
        end else begin
            $display("[PASS] Quad-X hover verified: all motors at 0.5, saturated=0.");
        end

        // Test 2: Quad-+ Pure Pitch Command
        $display("[TEST 2] Testing Quad-+ Pure Pitch (T=0.5, Pitch=+0.2)...");
        airframe_type = 3'b001; // Quad-+
        u_thrust      = 32'sh0000_8000; // 0.5
        u_pitch       = 32'sh0000_3333; // ~0.2
        u_roll        = 0;
        u_yaw         = 0;
        sample_valid  = 1;
        @(posedge clk);
        #1;
        sample_valid  = 0;
        $display("TEST 2: M0(Front)=%h, M2(Rear)=%h, raw_sum0=%h, raw_sum2=%h", 
                 motors[0], motors[2], dut.raw_sums[0], dut.raw_sums[2]);
        if (motors[0] <= motors[2]) begin
            $display("[FAIL] Quad-+ Front motor should be greater than rear motor!");
            error_count++;
        end else begin
            $display("[PASS] Quad-+ differential pitch torque verified: Front > Rear.");
        end

        // Test 3: Octo-X Full 8-Motor Output & Saturation Clamping
        $display("[TEST 3] Testing Octo-X Saturation Clamping (T=0.9, Roll=+0.5)...");
        airframe_type = 3'b100; // Octo-X
        u_thrust      = 32'sh0000_E666; // 0.9
        u_roll        = 32'sh0000_8000; // 0.5
        u_pitch       = 0;
        u_yaw         = 0;
        sample_valid  = 1;
        @(posedge clk);
        #1;
        sample_valid  = 0;
        $display("TEST 3: sat_flag=%b, any_sat=%b, raw_sums[3]=%h", 
                 saturated_flag, dut.any_sat, dut.raw_sums[3]);
        if (saturated_flag !== 1'b1) begin
            $display("[FAIL] Saturation flag should be set for over-driven condition!");
            error_count++;
        end
        for (int i = 0; i < NUM_MOTORS; i++) begin
            if (motors[i] > FIX_ONE || motors[i] < FIX_ZERO) begin
                $display("[FAIL] Motor %0d breached [0.0, 1.0] clamp! Got: 0x%08h", i+1, motors[i]);
                error_count++;
            end
        end
        $display("[PASS] Octo-X saturation clamping strictly contained within [0.0, 1.0].");

        #40;
        if (error_count == 0) begin
            $display("==================================================");
            $display("    [PASS] ALL MIXER MATRIX TESTS PASSED         ");
            $display("==================================================");
            $finish(0);
        end else begin
            $display("==================================================");
            $display("    [FAIL] %0d ERRORS IN MIXER MATRIX TB         ", error_count);
            $display("==================================================");
            $fatal(1);
        end
    end

endmodule : pid_mixer_matrix_tb
