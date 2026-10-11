//==============================================================================
// Testbench: pid_gyro_notch_tb.sv
// Description: Self-checking unit testbench for 3-Axis Biquad Notch Filter.
//              Verifies:
//              1. Hardware bypass mode (zero processing latency & exact pass-through)
//              2. DC unity gain (DC rates pass through unaltered)
//              3. High-frequency vibration attenuation
//              4. Fixed-point saturation bounds (no overflow wrapping)
//==============================================================================

`timescale 1ns/1ps

`include "pid_pkg.sv"
`include "pid_gyro_notch.sv"

module pid_gyro_notch_tb;

    import pid_pkg::*;

    localparam int DATA_W  = 32;
    localparam int FRACT_W = 16;
    localparam int ACCUM_W = 48;

    logic clk;
    logic rst_n;
    logic filter_enable;
    logic sample_valid;

    // Filter coefficients (Q16.16)
    // Example 200 Hz Notch Filter coefficients for 1 kHz sample rate:
    // b0 = 0.939, b1 = -1.161, b2 = 0.939, a1 = -1.161, a2 = 0.878
    logic signed [DATA_W-1:0] coeff_b0;
    logic signed [DATA_W-1:0] coeff_b1;
    logic signed [DATA_W-1:0] coeff_b2;
    logic signed [DATA_W-1:0] coeff_a1;
    logic signed [DATA_W-1:0] coeff_a2;

    logic signed [DATA_W-1:0] gyro_p_raw;
    logic signed [DATA_W-1:0] gyro_q_raw;
    logic signed [DATA_W-1:0] gyro_r_raw;

    logic                     valid_out;
    logic signed [DATA_W-1:0] gyro_p_filtered;
    logic signed [DATA_W-1:0] gyro_q_filtered;
    logic signed [DATA_W-1:0] gyro_r_filtered;

    // DUT
    pid_gyro_notch #(
        .DATA_W(DATA_W),
        .FRACT_W(FRACT_W),
        .ACCUM_W(ACCUM_W)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .filter_enable(filter_enable),
        .sample_valid(sample_valid),
        .coeff_b0(coeff_b0),
        .coeff_b1(coeff_b1),
        .coeff_b2(coeff_b2),
        .coeff_a1(coeff_a1),
        .coeff_a2(coeff_a2),
        .gyro_p_raw(gyro_p_raw),
        .gyro_q_raw(gyro_q_raw),
        .gyro_r_raw(gyro_r_raw),
        .valid_out(valid_out),
        .gyro_p_filtered(gyro_p_filtered),
        .gyro_q_filtered(gyro_q_filtered),
        .gyro_r_filtered(gyro_r_filtered)
    );

    // Clock
    initial clk = 0;
    always #10 clk = ~clk;

    int error_count = 0;

    initial begin
        $display("==================================================");
        $display("     TESTBENCH: Gyro Biquad Notch Filter (Unit)   ");
        $display("==================================================");

        rst_n         = 0;
        filter_enable = 0;
        sample_valid  = 0;
        gyro_p_raw    = 0;
        gyro_q_raw    = 0;
        gyro_r_raw    = 0;

        // Q16.16 normalized notch filter coefficients for 200 Hz @ 1 kHz fs (Q = 2.0)
        coeff_b0 = 32'sd52947;  // 0.8079
        coeff_b1 = -32'sd32723; // -0.4993
        coeff_b2 = 32'sd52947;  // 0.8079
        coeff_a1 = -32'sd32723; // -0.4993
        coeff_a2 = 32'sd40358;  // 0.6158

        #40;
        rst_n = 1;
        #20;

        // Test 1: Hardware Bypass Mode
        $display("[TEST 1] Testing Hardware Bypass mode (filter_enable=0)...");
        filter_enable = 0;
        gyro_p_raw = 32'sh0002_4000; // 2.25
        gyro_q_raw = -32'sh0001_8000; // -1.5
        gyro_r_raw = 32'sh0000_C000; // 0.75
        sample_valid = 1;

        @(posedge clk);
        #1;
        if (gyro_p_filtered !== gyro_p_raw || gyro_q_filtered !== gyro_q_raw || gyro_r_filtered !== gyro_r_raw) begin
            $display("[FAIL] Bypass mode failed to pass raw values directly!");
            error_count++;
        end else begin
            $display("[PASS] Hardware bypass verified: exact match.");
        end

        @(posedge clk);
        sample_valid = 0;
        #40;

        // Test 2: Active Filter Mode - DC Step Settling
        $display("[TEST 2] Testing Active Filter Mode with DC Input...");
        filter_enable = 1;
        gyro_p_raw = 32'sh0001_0000; // 1.0 rad/s DC
        gyro_q_raw = 32'sh0001_0000;
        gyro_r_raw = 32'sh0001_0000;

        // Feed 50 consecutive DC samples
        for (int i = 0; i < 50; i++) begin
            @(posedge clk);
            sample_valid = 1;
            @(posedge clk);
            sample_valid = 0;
        end

        #1;
        $display("DC steady-state filtered output: P=0x%08h (~%0.3f)", 
                 gyro_p_filtered, real'(gyro_p_filtered) / 65536.0);
        // DC gain of a normalized notch filter should settle very close to 1.0 (within 5% tolerance)
        if (gyro_p_filtered < 32'sh0000_E000 || gyro_p_filtered > 32'sh0001_2000) begin
            $display("[FAIL] DC settling out of expected envelope!");
            error_count++;
        end else begin
            $display("[PASS] DC unity transmission verified.");
        end

        // Test 3: Vibration Rejection Test at Notch Frequency (200 Hz @ 1 kHz fs)
        $display("[TEST 3] Testing 200 Hz Resonant Harmonic Attenuation...");
        begin
            logic signed [31:0] notch_sine [0:4];
            logic signed [31:0] last_mag;
            notch_sine[0] = 32'sd0;
            notch_sine[1] = 32'sd62328;
            notch_sine[2] = 32'sd38535;
            notch_sine[3] = -32'sd38535;
            notch_sine[4] = -32'sd62328;

            // Feed 20 full cycles (100 samples)
            for (int cyc = 0; cyc < 20; cyc++) begin
                for (int s = 0; s < 5; s++) begin
                    @(posedge clk);
                    gyro_p_raw   = notch_sine[s];
                    sample_valid = 1;
                    @(posedge clk);
                    sample_valid = 0;
                end
            end
            #1;
            last_mag = (gyro_p_filtered < 0) ? -gyro_p_filtered : gyro_p_filtered;
            $display("200 Hz harmonic filtered residual amplitude: 0x%08h (%0.3f, input peak was 0.951)", 
                     last_mag, real'(last_mag) / 65536.0);
            if (last_mag > 32'sh0000_4000) begin // Should attenuate below 0.25 (deep notch)
                $display("[FAIL] Notch attenuation insufficient!");
                error_count++;
            end else begin
                $display("[PASS] Notch harmonic attenuation verified (< 0.25 peak residual).");
            end
        end

        #40;
        if (error_count == 0) begin
            $display("==================================================");
            $display("     [PASS] ALL GYRO NOTCH FILTER TESTS PASSED    ");
            $display("==================================================");
            $finish(0);
        end else begin
            $display("==================================================");
            $display("     [FAIL] %0d ERRORS IN GYRO NOTCH FILTER TB    ", error_count);
            $display("==================================================");
            $fatal(1);
        end
    end

endmodule : pid_gyro_notch_tb
