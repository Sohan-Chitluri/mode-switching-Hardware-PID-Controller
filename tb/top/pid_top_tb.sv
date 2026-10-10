//==============================================================================
// Testbench: pid_top_tb.sv
// Description: Full-chip integrated self-checking testbench for pid_top.
//              Verifies:
//              1. 10,000 randomized and directed vectors from top_random_10k.hex.
//              2. Bumpless mode transition transient bounds (|delta_U| <= 5%).
//              3. Watchdog timeout determinism and sticky fault latching.
//              4. Emergency hardware failsafe override and pilot re-arm handshake.
// Tool: Verilator (--binary)
//==============================================================================

`timescale 1ns/1ps

`include "pid_pkg.sv"

module pid_top_tb;

    import pid_pkg::*;

    // Clock and Reset
    logic clk;
    logic rst_n;

    // Sensor Sample Interface
    logic                     sample_valid;
    logic                     sample_ready;
    logic signed [31:0]       imu_p;
    logic signed [31:0]       imu_q;
    logic signed [31:0]       imu_r;
    logic signed [31:0]       imu_phi;
    logic signed [31:0]       imu_theta;
    logic signed [31:0]       alt_z;
    logic signed [31:0]       alt_vz;
    logic                     alt_valid;

    // Pilot Commands
    logic [1:0]               mode_cmd;
    logic                     rearm_cmd;
    logic                     failsafe_ext_n;
    logic signed [31:0]       rc_sp_roll;
    logic signed [31:0]       rc_sp_pitch;
    logic signed [31:0]       rc_sp_yaw;
    logic signed [31:0]       rc_sp_thrust;

    // Configuration Interface
    logic                     cfg_we;
    logic [7:0]               cfg_addr;
    logic [31:0]              cfg_wdata;
    logic [31:0]              cfg_rdata;

    // Actuator Outputs
    logic                     output_valid;
    logic signed [31:0]       u_roll;
    logic signed [31:0]       u_pitch;
    logic signed [31:0]       u_yaw;
    logic signed [31:0]       u_thrust;

    // Status and Telemetry
    logic [1:0]               active_mode;
    logic [3:0]               fault_code;
    logic                     watchdog_alarm;
    logic                     saturated_flag;

    // Instantiate Top DUT
    pid_top #(
        .DATA_W(32),
        .FRACT_W(16),
        .ACCUM_W(48),
        .TIMEOUT_CYC(100) // Fast 100-cycle timeout for testbench verification
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(sample_valid),
        .sample_ready(sample_ready),
        .imu_p(imu_p),
        .imu_q(imu_q),
        .imu_r(imu_r),
        .imu_phi(imu_phi),
        .imu_theta(imu_theta),
        .alt_z(alt_z),
        .alt_vz(alt_vz),
        .alt_valid(alt_valid),
        .mode_cmd(mode_cmd),
        .rearm_cmd(rearm_cmd),
        .failsafe_ext_n(failsafe_ext_n),
        .rc_sp_roll(rc_sp_roll),
        .rc_sp_pitch(rc_sp_pitch),
        .rc_sp_yaw(rc_sp_yaw),
        .rc_sp_thrust(rc_sp_thrust),
        .cfg_we(cfg_we),
        .cfg_addr(cfg_addr),
        .cfg_wdata(cfg_wdata),
        .cfg_rdata(cfg_rdata),
        .output_valid(output_valid),
        .u_roll(u_roll),
        .u_pitch(u_pitch),
        .u_yaw(u_yaw),
        .u_thrust(u_thrust),
        .active_mode(active_mode),
        .fault_code(fault_code),
        .watchdog_alarm(watchdog_alarm),
        .saturated_flag(saturated_flag)
    );

    // 50 MHz clock generator
    always #10 clk = ~clk;

    int total_vectors = 0;
    int mismatches = 0;

    // Test task: Run full random vector file
    task automatic run_top_vectors(input string filename);
        int fd;
        string line;
        int status;

        logic        v_sv;
        logic [31:0] v_p, v_q, v_r, v_phi, v_theta, v_z, v_vz;
        logic        v_alt_v;
        logic [1:0]  v_mode;
        logic        v_rearm;
        logic        v_fs_n;
        logic [31:0] v_spr, v_spp, v_spy, v_spt;
        logic        exp_v;
        logic [1:0]  exp_mode;
        logic [31:0] exp_ur, exp_up, exp_uy, exp_ut;
        logic        exp_alarm;
        logic [3:0]  exp_fault;

        logic [31:0] pipe_ur, pipe_up, pipe_uy, pipe_ut;
        logic [1:0]  pipe_mode;
        logic        pipe_alarm;
        logic [3:0]  pipe_fault;
        logic        pipe_out_valid;
        logic        pipe_valid;

        $display("--- Opening Top Vector File: %s ---", filename);
        fd = $fopen(filename, "r");
        if (fd == 0) begin
            $fatal(1, "ERROR: Could not open %s", filename);
        end

        pipe_valid = 1'b0;

        while ($fgets(line, fd) != 0) begin
            if (line.len() == 0 || line[0] == 8'h23 || line[0] == 8'h2F)
                continue;

            status = $sscanf(line, "%b %h %h %h %h %h %h %h %b %d %b %b %h %h %h %h | %b %d %h %h %h %h %b %d",
                             v_sv, v_p, v_q, v_r, v_phi, v_theta, v_z, v_vz, v_alt_v, v_mode, v_rearm, v_fs_n,
                             v_spr, v_spp, v_spy, v_spt,
                             exp_v, exp_mode, exp_ur, exp_up, exp_uy, exp_ut, exp_alarm, exp_fault);

            if (status == 24) begin
                @(posedge clk);
                #1;
                // Check pipelined output registered from PREVIOUS cycle
                if (pipe_valid) begin
                    total_vectors++;

                    if (output_valid !== pipe_out_valid) begin
                        $display("FAIL [Vector %0d]: output_valid mismatch! Got=%0b, Exp=%0b", total_vectors, output_valid, pipe_out_valid);
                        mismatches++;
                    end
                    if (active_mode !== pipe_mode) begin
                        $display("FAIL [Vector %0d]: active_mode mismatch! Got=%0d, Exp=%0d", total_vectors, active_mode, pipe_mode);
                        mismatches++;
                    end

                    if (u_roll !== pipe_ur) begin
                        $display("FAIL [Vector %0d]: u_roll mismatch! Got=%08h, Exp=%08h", total_vectors, u_roll, pipe_ur);
                        mismatches++;
                    end
                    if (u_pitch !== pipe_up) begin
                        $display("FAIL [Vector %0d]: u_pitch mismatch! Got=%08h, Exp=%08h", total_vectors, u_pitch, pipe_up);
                        mismatches++;
                    end
                    if (u_yaw !== pipe_uy) begin
                        $display("FAIL [Vector %0d]: u_yaw mismatch! Got=%08h, Exp=%08h", total_vectors, u_yaw, pipe_uy);
                        mismatches++;
                    end
                    if (u_thrust !== pipe_ut) begin
                        $display("FAIL [Vector %0d]: u_thrust mismatch! Got=%08h, Exp=%08h", total_vectors, u_thrust, pipe_ut);
                        $display("DEBUG [Vector %0d]: active_mode=%0d, mode_next_w=%0d, final_pre_sat=%08h, tilt_comp=%08h, base_thrust=%08h, core_u_thrust=%08h, hover_thrust=%08h",
                                 total_vectors, active_mode, dut.mode_next_w, dut.final_pre_sat_thrust, dut.tilt_comp_thrust, dut.base_thrust, dut.core_u_thrust, dut.hover_thrust);
                        mismatches++;
                    end
                    if (watchdog_alarm !== pipe_alarm) begin
                        $display("FAIL [Vector %0d]: watchdog_alarm mismatch! Got=%0b, Exp=%0b", total_vectors, watchdog_alarm, pipe_alarm);
                        mismatches++;
                    end
                    if (fault_code !== pipe_fault) begin
                        $display("FAIL [Vector %0d]: fault_code mismatch! Got=%0h, Exp=%0h", total_vectors, fault_code, pipe_fault);
                        mismatches++;
                    end

                    if (mismatches > 15) begin
                        $fatal(1, "ERROR: Exceeded mismatch threshold in top regression!");
                    end
                end

                // Drive new vector inputs
                sample_valid   = v_sv;
                imu_p          = v_p;
                imu_q          = v_q;
                imu_r          = v_r;
                imu_phi        = v_phi;
                imu_theta      = v_theta;
                alt_z          = v_z;
                alt_vz         = v_vz;
                alt_valid      = v_alt_v;
                mode_cmd       = v_mode;
                rearm_cmd      = v_rearm;
                failsafe_ext_n = v_fs_n;
                rc_sp_roll     = v_spr;
                rc_sp_pitch    = v_spp;
                rc_sp_yaw      = v_spy;
                rc_sp_thrust   = v_spt;

                // Latch expected outputs for check on next cycle
                pipe_out_valid = exp_v;
                pipe_ur    = exp_ur;
                pipe_up    = exp_up;
                pipe_uy    = exp_uy;
                pipe_ut    = exp_ut;
                pipe_mode  = exp_mode;
                pipe_alarm = exp_alarm;
                pipe_fault = exp_fault;
                pipe_valid = 1'b1;
            end
        end

        // Check final pipelined vector
        if (pipe_valid) begin
            @(posedge clk);
            #1;
            total_vectors++;
            if (output_valid !== pipe_out_valid) begin
                $display("FAIL [Vector %0d]: output_valid mismatch! Got=%0b, Exp=%0b", total_vectors, output_valid, pipe_out_valid);
                mismatches++;
            end
            if (active_mode !== pipe_mode) begin
                $display("FAIL [Vector %0d]: active_mode mismatch! Got=%0d, Exp=%0d", total_vectors, active_mode, pipe_mode);
                mismatches++;
            end
            if (u_roll !== pipe_ur) begin
                $display("FAIL [Vector %0d]: u_roll mismatch! Got=%08h, Exp=%08h", total_vectors, u_roll, pipe_ur);
                mismatches++;
            end
            if (u_pitch !== pipe_up) begin
                $display("FAIL [Vector %0d]: u_pitch mismatch! Got=%08h, Exp=%08h", total_vectors, u_pitch, pipe_up);
                mismatches++;
            end
            if (u_yaw !== pipe_uy) begin
                $display("FAIL [Vector %0d]: u_yaw mismatch! Got=%08h, Exp=%08h", total_vectors, u_yaw, pipe_uy);
                mismatches++;
            end
            if (u_thrust !== pipe_ut) begin
                $display("FAIL [Vector %0d]: u_thrust mismatch! Got=%08h, Exp=%08h", total_vectors, u_thrust, pipe_ut);
                mismatches++;
            end
            if (watchdog_alarm !== pipe_alarm) begin
                $display("FAIL [Vector %0d]: watchdog_alarm mismatch! Got=%0b, Exp=%0b", total_vectors, watchdog_alarm, pipe_alarm);
                mismatches++;
            end
            if (fault_code !== pipe_fault) begin
                $display("FAIL [Vector %0d]: fault_code mismatch! Got=%0h, Exp=%0h", total_vectors, fault_code, pipe_fault);
                mismatches++;
            end
        end

        $fclose(fd);
        $display("Completed %0d top-level vectors.", total_vectors);
    endtask

    // Directed bumpless transfer verification task
    task automatic test_bumpless_transition();
        logic signed [31:0] u_before;
        logic signed [31:0] u_after;
        logic signed [31:0] diff;
        $display("=== [DIRECTED TEST] Verifying Bumpless Mode Handover ===");

        // Step 1: Fly in Acro mode with stick holding trim attitude
        mode_cmd = MODE_ACRO;
        rc_sp_roll = 32'sh0000_3000; // Stick holding current attitude: 0.1875 rad
        imu_p = 32'sh0000_3000;      // Gyro rate: 0.1875 rad/s
        imu_phi = 32'sh0000_3000;    // Attitude angle: 0.1875 rad
        sample_valid = 1;
        repeat (20) @(posedge clk);
        #1;
        u_before = u_roll;

        // Step 2: Flip mode switch to Stabilize while aircraft is in flight
        @(posedge clk);
        #1;
        mode_cmd = MODE_STABILIZE;

        // Transition strobe fires immediately; wait 1 cycle for output stage register
        @(posedge clk);
        #1;
        u_after = u_roll;

        diff = (u_after > u_before) ? (u_after - u_before) : (u_before - u_after);
        $display("  Bumpless Step: U_before=%08h, U_after=%08h, Diff=%08h counts", u_before, u_after, diff);

        // Spec gate: Diff must be <= 5% of full scale (0.05 * 65536 = 3276 counts = 0xCCD)
        if (diff <= 3276) begin
            $display("  Bumpless Transfer Result: PASS (|delta_U| <= 5%%)");
        end else begin
            $fatal(1, "FAIL: Bumpless transfer exceeded 5%% transient limit! Diff=%0d counts", diff);
        end
    endtask

    // Directed watchdog timeout verification task
    task automatic test_watchdog_timeout();
        $display("=== [DIRECTED TEST] Verifying Watchdog Sample Freshness Timeout ===");
        sample_valid = 1;
        repeat (5) @(posedge clk);

        // Drop sensor samples (stale sensor condition)
        sample_valid = 0;
        $display("  Ceased sample_valid strobes. Waiting for timeout (configured for 100 cycles)...");

        repeat (105) @(posedge clk);
        #1;

        if (watchdog_alarm && (active_mode == MODE_FAILSAFE) && (u_roll == 32'sd0)) begin
            $display("  Watchdog Result: PASS (Alarm raised, Failsafe entered, Outputs zeroed)");
        end else begin
            $fatal(1, "FAIL: Watchdog failed to trigger Failsafe! alarm=%b, mode=%0d", watchdog_alarm, active_mode);
        end

        // Test Re-Arm Recovery Handshake
        $display("  Testing re-arm handshake recovery...");
        sample_valid = 1;
        repeat (5) @(posedge clk);
        rearm_cmd = 1;
        mode_cmd = MODE_STABILIZE;
        @(posedge clk);
        #1;
        rearm_cmd = 0;
        repeat (5) @(posedge clk);
        #1;

        if (active_mode == MODE_STABILIZE) begin
            $display("  Re-Arm Handshake Result: PASS (Recovered from Failsafe to Stabilize)");
        end else begin
            $fatal(1, "FAIL: Re-arm failed to restore mode! active_mode=%0d", active_mode);
        end
    endtask

    // Directed synchronous reset verification task
    task automatic test_synchronous_reset();
        $display("=== [DIRECTED TEST] Verifying Synchronous Reset Behavior ===");
        // Drive non-zero state in flight
        sample_valid = 1;
        mode_cmd = MODE_STABILIZE;
        rc_sp_roll = 32'sh0000_4000;
        imu_p = 32'sh0000_1000;
        imu_phi = 32'sh0000_2000;
        repeat (10) @(posedge clk);
        #1;
        if (u_roll == 32'sd0) begin
            $fatal(1, "FAIL: Expected active u_roll before reset, got 0");
        end

        // Assert synchronous reset
        rst_n = 0;
        @(posedge clk);
        #1;
        // Verify immediate synchronous clear
        if (u_roll !== 32'sd0 || u_pitch !== 32'sd0 || u_yaw !== 32'sd0 || u_thrust !== 32'sd0 ||
            output_valid !== 1'b0 || active_mode !== MODE_ACRO || fault_code !== 4'd0) begin
            $fatal(1, "FAIL: Synchronous reset failed to clear DUT state! u_roll=%h, valid=%b, mode=%d",
                   u_roll, output_valid, active_mode);
        end
        $display("  Synchronous Reset Result: PASS (All states cleared to default on clock edge)");

        // Release reset
        rst_n = 1;
        sample_valid = 0;
        repeat (5) @(posedge clk);
    endtask

    // Directed mode-dependent gain selection verification task
    task automatic test_mode_gain_selection();
        logic signed [31:0] u_acro, u_stab;
        $display("=== [DIRECTED TEST] Verifying Mode-Dependent Gain Bank Selection ===");

        // Verify configuration memory read of gain banks
        // Bank 0 (Acro): addr 5 = roll_kp = 0x0000_4000
        cfg_addr = 8'd5;
        #1;
        if (cfg_rdata !== 32'h0000_4000)
            $fatal(1, "FAIL: Bank 0 roll_kp read mismatch! Got=%08h, Exp=00004000", cfg_rdata);

        // Bank 1 (Stabilize): addr 32 + 5 = 37 = roll_kp = 0x0001_6666
        cfg_addr = 8'd37;
        #1;
        if (cfg_rdata !== 32'h0001_6666)
            $fatal(1, "FAIL: Bank 1 roll_kp read mismatch! Got=%08h, Exp=00016666", cfg_rdata);

        // Bank 2 (AltHold): addr 64 + 29 = 93 = thrust_kp = 0x0000_6666
        cfg_addr = 8'd93;
        #1;
        if (cfg_rdata !== 32'h0000_6666)
            $fatal(1, "FAIL: Bank 2 thrust_kp read mismatch! Got=%08h, Exp=00006666", cfg_rdata);

        // Bank 3 (Failsafe): addr 96 + 5 = 101 = roll_kp = 0x0000_0000
        cfg_addr = 8'd101;
        #1;
        if (cfg_rdata !== 32'h0000_0000)
            $fatal(1, "FAIL: Bank 3 roll_kp read mismatch! Got=%08h, Exp=00000000", cfg_rdata);

        $display("  Gain Bank Memory Read: PASS (All 4 banks verified distinct via cfg interface)");

        // Test behavioral response: Apply identical stimulus in Acro vs Stabilize
        rc_sp_roll = 32'sh0000_4000;
        imu_p      = 32'sh0000_0000;
        imu_phi    = 32'sh0000_0000;
        sample_valid = 1;

        // Run in Acro
        mode_cmd = MODE_ACRO;
        repeat (5) @(posedge clk);
        #1;
        u_acro = u_roll;

        // Run in Stabilize with identical input
        mode_cmd = MODE_STABILIZE;
        repeat (5) @(posedge clk);
        #1;
        u_stab = u_roll;

        // Run in Failsafe
        mode_cmd = MODE_FAILSAFE;
        repeat (5) @(posedge clk);
        #1;

        if (u_roll !== 32'sd0)
            $fatal(1, "FAIL: Failsafe mode output not zero! Got=%08h", u_roll);

        if (u_acro == u_stab)
            $fatal(1, "FAIL: Acro and Stabilize produced identical output! Mode gain bank not switching.");

        $display("  Behavioral Mode Response: PASS (Acro u_roll=%08h != Stab u_roll=%08h, Failsafe=0)", u_acro, u_stab);

        // Reset inputs
        sample_valid = 0;
        mode_cmd = MODE_ACRO;
        rc_sp_roll = 0;
        repeat (5) @(posedge clk);
    endtask

    // Directed external failsafe kill verification task
    task automatic test_external_failsafe_kill();
        $display("=== [DIRECTED TEST] Verifying External Failsafe Kill Latency and Override ===");
        sample_valid = 1;
        mode_cmd = MODE_STABILIZE;
        rc_sp_roll = 32'sh0000_4000;
        imu_phi = 32'sh0000_2000;
        failsafe_ext_n = 1;
        repeat (5) @(posedge clk);
        #1;
        if (u_roll == 32'sd0)
            $fatal(1, "FAIL: Controller not producing active torque before kill assertion");

        // Assert asynchronous external kill pin
        failsafe_ext_n = 0;

        // Cycle 1: Captured by fs_sync_q1
        @(posedge clk);
        #1;

        // Cycle 2: Captured by fs_sync_q2 (failsafe_ext_sync_n drops, next_mode becomes FAILSAFE)
        @(posedge clk);
        #1;

        // Cycle 3: Registered by output_stage (u_roll becomes 0)
        @(posedge clk);
        #1;

        if (active_mode !== MODE_FAILSAFE || u_roll !== 32'sd0 || u_pitch !== 32'sd0 || u_yaw !== 32'sd0 || (fault_code & FAULT_EXT_FAILSAFE) == 0) begin
            $fatal(1, "FAIL: External failsafe kill failed to clamp outputs! mode=%d, u_roll=%h, fault=%h",
                   active_mode, u_roll, fault_code);
        end
        $display("  External Kill Result: PASS (Outputs clamped within 3 clock cycles, fault code latched)");

        // Test recovery
        failsafe_ext_n = 1;
        repeat (2) @(posedge clk);
        rearm_cmd = 1;
        mode_cmd = MODE_STABILIZE;
        @(posedge clk);
        #1;
        rearm_cmd = 0;
        repeat (5) @(posedge clk);
        #1;
        if (active_mode !== MODE_STABILIZE)
            $fatal(1, "FAIL: Re-arm failed after external kill recovery! active_mode=%d", active_mode);

        $display("  External Kill Recovery: PASS (Re-armed back to Stabilize)");
        sample_valid = 0;
        repeat (5) @(posedge clk);
    endtask

    // Main Test Sequence
    initial begin
        clk            = 0;
        rst_n          = 0;
        sample_valid   = 0;
        imu_p          = 0;
        imu_q          = 0;
        imu_r          = 0;
        imu_phi        = 0;
        imu_theta      = 0;
        alt_z          = 0;
        alt_vz         = 0;
        alt_valid      = 0;
        mode_cmd       = MODE_ACRO;
        rearm_cmd      = 0;
        failsafe_ext_n = 1;
        rc_sp_roll     = 0;
        rc_sp_pitch    = 0;
        rc_sp_yaw      = 0;
        rc_sp_thrust   = 0;
        cfg_we         = 0;
        cfg_addr       = 0;
        cfg_wdata      = 0;

        $display("=========================================================");
        $display("  Starting pid_top Full-Chip Hardware Verification");
        $display("=========================================================");

        // Power-on Reset
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        // Run Directed Tests First
        test_synchronous_reset();
        test_mode_gain_selection();
        test_bumpless_transition();
        test_watchdog_timeout();
        test_external_failsafe_kill();

        // Power-on Reset before Top Regression Suite
        rst_n = 0;
        sample_valid = 0;
        mode_cmd = MODE_ACRO;
        repeat (10) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        // Run Top Regression Vectors
        run_top_vectors("vectors/random/top_random_10k.hex");

        $display("=========================================================");
        $display("  pid_top Full-Chip Verification Summary");
        $display("  Total Vectors Processed : %0d", total_vectors);
        $display("  Total Bit Mismatches    : %0d", mismatches);
        if (mismatches == 0) begin
            $display("  STATUS                  : 100%% BIT-EXACT PASS");
        end else begin
            $fatal(1, "Hardware verification failed with mismatches!");
        end
        $display("=========================================================");

        $finish;
    end

endmodule : pid_top_tb
