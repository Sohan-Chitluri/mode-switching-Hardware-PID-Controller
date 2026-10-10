//==============================================================================
// Testbench: pid_soc_top_tb.sv
// Description: Verification testbench for pid_soc_top MMIO bus wrapper.
//              Verifies:
//              1. MMIO register reset values and read/write semantics.
//              2. Unsupported address handling (returns 0x00000000).
//              3. Software-driven closed-loop step:
//                 - Writing setpoints and IMU measurements.
//                 - Pulsing sample_valid strobe via REG_CTRL[0].
//                 - Reading back controller commands and motor commands M1..M4.
//              4. Pass-through configuration bus access to gain bank (0x100..0x19F).
//              5. Mode switching and status readback through MMIO.
//              6. External hardware emergency kill override.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`timescale 1ns/1ps

`include "pid_pkg.sv"
import pid_pkg::*;

module pid_soc_top_tb;

    logic clk;
    logic rst_n;
    logic failsafe_ext_n;

    logic        bus_we;
    logic [8:0]  bus_addr;
    logic [31:0] bus_wdata;
    logic [31:0] bus_rdata;
    logic        bus_rvalid;

    logic        motor_valid;
    logic [31:0] motor_1;
    logic [31:0] motor_2;
    logic [31:0] motor_3;
    logic [31:0] motor_4;

    logic        controller_valid;
    logic [31:0] u_roll;
    logic [31:0] u_pitch;
    logic [31:0] u_yaw;
    logic [31:0] u_thrust;

    logic [1:0]  active_mode;
    logic [3:0]  fault_code;
    logic        watchdog_alarm;
    logic        controller_sat_flag;
    logic        mixer_sat_flag;

    int test_count = 0;
    int pass_count = 0;
    int fail_count = 0;

    pid_soc_top #(
        .DATA_W(32),
        .FRACT_W(16),
        .ACCUM_W(48),
        .TIMEOUT_CYC(100)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .failsafe_ext_n(failsafe_ext_n),
        .bus_we(bus_we),
        .bus_addr(bus_addr),
        .bus_wdata(bus_wdata),
        .bus_rdata(bus_rdata),
        .bus_rvalid(bus_rvalid),
        .motor_valid(motor_valid),
        .motor_1(motor_1),
        .motor_2(motor_2),
        .motor_3(motor_3),
        .motor_4(motor_4),
        .controller_valid(controller_valid),
        .sample_ready(),
        .u_roll(u_roll),
        .u_pitch(u_pitch),
        .u_yaw(u_yaw),
        .u_thrust(u_thrust),
        .active_mode(active_mode),
        .fault_code(fault_code),
        .watchdog_alarm(watchdog_alarm),
        .controller_sat_flag(controller_sat_flag),
        .mixer_sat_flag(mixer_sat_flag)
    );

    // 50 MHz Clock
    always #10 clk = ~clk;

    // Helper task to write MMIO register
    task automatic mmio_write(input [8:0] addr, input [31:0] data);
        bus_we    = 1'b1;
        bus_addr  = addr;
        bus_wdata = data;
        @(posedge clk);
        #1;
        bus_we    = 1'b0;
        bus_addr  = 9'd0;
        bus_wdata = 32'd0;
        @(posedge clk);
        #1;
    endtask

    // Helper task to read MMIO register
    task automatic mmio_read(input [8:0] addr, output [31:0] data);
        bus_we   = 1'b0;
        bus_addr = addr;
        @(posedge clk);
        #1;
        data = bus_rdata;
    endtask

    initial begin
        clk            = 0;
        rst_n          = 0;
        failsafe_ext_n = 1;
        bus_we         = 0;
        bus_addr       = 0;
        bus_wdata      = 0;

        $display("=========================================================");
        $display("  Starting pid_soc_top MMIO Integration Testbench");
        $display("=========================================================");

        // Power-on Reset
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        // -------------------------------------------------------------
        // Test 1: Reset Values of MMIO Registers
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 1] Verifying Reset Values of MMIO Registers");
        begin
            logic [31:0] rval;
            mmio_read(9'h00, rval); // REG_CTRL
            if (rval == 32'd0) begin
                mmio_read(9'h01, rval); // REG_MODE_CMD
                if (rval == 32'd0) begin
                    $display("  PASS: Reset values of REG_CTRL and REG_MODE_CMD are 0x0");
                    pass_count++;
                end else begin
                    $display("  FAIL: REG_MODE_CMD reset value: %h", rval);
                    fail_count++;
                end
            end else begin
                $display("  FAIL: REG_CTRL reset value: %h", rval);
                fail_count++;
            end
        end

        // -------------------------------------------------------------
        // Test 2: Unsupported Address Handling
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 2] Reading Unsupported Addresses (0x020, 0x0FF)");
        begin
            logic [31:0] rval;
            mmio_read(9'h020, rval);
            if (rval == 32'd0) begin
                mmio_read(9'h0FF, rval);
                if (rval == 32'd0) begin
                    $display("  PASS: Unsupported addresses return 0x00000000");
                    pass_count++;
                end else begin
                    $display("  FAIL: 0x0FF returned %h", rval);
                    fail_count++;
                end
            end else begin
                $display("  FAIL: 0x020 returned %h", rval);
                fail_count++;
            end
        end

        // -------------------------------------------------------------
        // Test 3: Setpoint and Sensor Programming via MMIO
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 3] Programming Setpoints & IMU Values via MMIO");
        begin
            logic [31:0] rval;
            // Write Setpoint Thrust = 0.50 (32768 counts)
            mmio_write(9'h06, 32'sh0000_8000);
            // Write Setpoint Pitch = 0.0156 (1024 counts)
            mmio_write(9'h04, 32'sh0000_0400);
            // Verify readback
            mmio_read(9'h06, rval);
            if (rval == 32'sh0000_8000) begin
                mmio_read(9'h04, rval);
                if (rval == 32'sh0000_0400) begin
                    $display("  PASS: Setpoints successfully programmed and read back");
                    pass_count++;
                end else begin
                    $display("  FAIL: SP_PITCH readback: %h", rval);
                    fail_count++;
                end
            end else begin
                $display("  FAIL: SP_THRUST readback: %h", rval);
                fail_count++;
            end
        end

        // -------------------------------------------------------------
        // Test 4: Operational Step via MMIO Strobe & Motor Command Readback
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 4] Triggering Sample via REG_CTRL[0] & Motor Readback");
        begin
            logic [31:0] m1, m2, m3, m4;
            // Set mode = MODE_STABILIZE
            mmio_write(9'h01, 32'd1);
            // Set level pitch setpoint for steady hover
            mmio_write(9'h04, 32'sd0);
            // Pulse sample_valid strobe (Bit 0 of REG_CTRL)
            mmio_write(9'h00, 32'h0000_0001);

            // Wait 3 clock cycles for controller registration + mixer registration
            repeat (3) @(posedge clk);

            // Readback motor commands
            mmio_read(9'h12, m1);
            mmio_read(9'h13, m2);
            mmio_read(9'h14, m3);
            mmio_read(9'h15, m4);

            $display("  Observed Taps  : Mode=%0d, U_t=%0d, U_p=%0d, U_r=%0d, U_y=%0d",
                     active_mode, u_thrust, u_pitch, u_roll, u_yaw);
            $display("  Observed Motors: M1=%0d, M2=%0d, M3=%0d, M4=%0d", m1, m2, m3, m4);
            // All motors must be active and non-zero
            if (m1 > 0 && m2 > 0 && m3 > 0 && m4 > 0) begin
                $display("  PASS: Motors actively updated through MMIO operational strobe");
                pass_count++;
            end else begin
                $display("  FAIL: Motors not updated: M1=%0d, M2=%0d, M3=%0d, M4=%0d", m1, m2, m3, m4);
                fail_count++;
            end
        end

        // -------------------------------------------------------------
        // Test 5: Gain Bank Forwarding via 0x100..0x19F
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 5] Forwarding Gain Bank Writes/Reads (Addr 0x100 + 13)");
        begin
            logic [31:0] rval;
            // Write Bank 0 pitch_kp (addr 13 -> bus_addr = 0x10D)
            mmio_write(9'h10D, 32'sh0003_5555); // Kp = 3.333
            mmio_read(9'h10D, rval);
            if (rval == 32'sh0003_5555) begin
                $display("  PASS: Gain bank pass-through read/write verified (0x10D = %h)", rval);
                pass_count++;
            end else begin
                $display("  FAIL: Gain bank pass-through: expected 0x00035555, got %h", rval);
                fail_count++;
            end
        end

        // -------------------------------------------------------------
        // Test 6: External Hardware Emergency Kill Pin Override
        // -------------------------------------------------------------
        test_count++;
        $display(">>> [Test 6] External Emergency Kill Override on SoC Actuators");
        begin
            logic [31:0] status_val;
            failsafe_ext_n = 1'b0; // Assert hardware kill
            repeat (5) @(posedge clk);
            mmio_read(9'h02, status_val); // REG_STATUS
            $display("  Status Register after kill: %h (active_mode=%0d, fault_code=%0d)",
                     status_val, status_val[1:0], status_val[5:2]);
            if (status_val[1:0] == MODE_FAILSAFE && ((status_val[5:2] & FAULT_EXT_FAILSAFE) != 4'd0)) begin
                $display("  PASS: Hardware emergency kill asserted Failsafe mode and fault status");
                pass_count++;
            end else begin
                $display("  FAIL: Status did not reflect kill: %h", status_val);
                fail_count++;
            end
            failsafe_ext_n = 1'b1; // De-assert kill
            repeat (5) @(posedge clk);
        end

        // -------------------------------------------------------------
        // Summary
        // -------------------------------------------------------------
        $display("=========================================================");
        $display("  pid_soc_top Integration Verification Summary");
        $display("  Total Tests : %0d", test_count);
        $display("  Passed      : %0d", pass_count);
        $display("  Failed      : %0d", fail_count);
        if (fail_count == 0)
            $display("  STATUS      : ALL SOC TOP TESTS PASSED (PASS)");
        else
            $fatal(1, "SoC top integration test failed!");
        $display("=========================================================");
        $finish;
    end

endmodule : pid_soc_top_tb
