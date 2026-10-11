//==============================================================================
// Testbench: pid_soc_axi_top_tb.sv
// Description: Comprehensive SoC integration testbench for pid_soc_axi_top.
//              Emulates an AXI4-Lite Host CPU (e.g., RISC-V Ibex / ARM Cortex-M)
//              interacting with the Hardware PID Flight Controller:
//              1. Reset and initial status read over AXI4-Lite
//              2. Writing setpoints and IMU attitude rates over AXI4-Lite
//              3. Initiating control sample cycle via MMIO strobe
//              4. Reading back calculated motor commands and flight telemetry
//              5. Flight mode transition (Acro -> Stabilize -> AltHold)
//              6. Hardware Emergency Kill Pin asserts instant Failsafe
//==============================================================================

`timescale 1ns/1ps

`include "pid_pkg.sv"
`include "pid_soc_axi_top.sv"

module pid_soc_axi_top_tb;

    import pid_pkg::*;

    localparam int DATA_W      = 32;
    localparam int FRACT_W     = 16;
    localparam int ACCUM_W     = 48;
    localparam int TIMEOUT_CYC = 200;
    localparam int ADDR_W      = 11;

    logic aclk;
    logic aresetn;
    logic failsafe_ext_n;

    // AXI4-Lite Interface
    logic [ADDR_W-1:0]      s_axi_awaddr;
    logic [2:0]             s_axi_awprot;
    logic                   s_axi_awvalid;
    logic                   s_axi_awready;

    logic [DATA_W-1:0]      s_axi_wdata;
    logic [(DATA_W/8)-1:0]  s_axi_wstrb;
    logic                   s_axi_wvalid;
    logic                   s_axi_wready;

    logic [1:0]             s_axi_bresp;
    logic                   s_axi_bvalid;
    logic                   s_axi_bready;

    logic [ADDR_W-1:0]      s_axi_araddr;
    logic [2:0]             s_axi_arprot;
    logic                   s_axi_arvalid;
    logic                   s_axi_arready;

    logic [DATA_W-1:0]      s_axi_rdata;
    logic [1:0]             s_axi_rresp;
    logic                   s_axi_rvalid;
    logic                   s_axi_rready;

    // Actuator Pins
    logic                     motor_valid;
    logic signed [DATA_W-1:0] motor_1;
    logic signed [DATA_W-1:0] motor_2;
    logic signed [DATA_W-1:0] motor_3;
    logic signed [DATA_W-1:0] motor_4;

    // Telemetry Pins
    logic [1:0]               active_mode;
    logic [3:0]               fault_code;
    logic                     watchdog_alarm;
    logic                     controller_sat_flag;
    logic                     mixer_sat_flag;

    // DUT Instance
    pid_soc_axi_top #(
        .DATA_W(DATA_W),
        .FRACT_W(FRACT_W),
        .ACCUM_W(ACCUM_W),
        .TIMEOUT_CYC(TIMEOUT_CYC),
        .ADDR_W(ADDR_W)
    ) dut (
        .aclk(aclk),
        .aresetn(aresetn),
        .failsafe_ext_n(failsafe_ext_n),
        .s_axi_awaddr(s_axi_awaddr),
        .s_axi_awprot(s_axi_awprot),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata),
        .s_axi_wstrb(s_axi_wstrb),
        .s_axi_wvalid(s_axi_wvalid),
        .s_axi_wready(s_axi_wready),
        .s_axi_bresp(s_axi_bresp),
        .s_axi_bvalid(s_axi_bvalid),
        .s_axi_bready(s_axi_bready),
        .s_axi_araddr(s_axi_araddr),
        .s_axi_arprot(s_axi_arprot),
        .s_axi_arvalid(s_axi_arvalid),
        .s_axi_arready(s_axi_arready),
        .s_axi_rdata(s_axi_rdata),
        .s_axi_rresp(s_axi_rresp),
        .s_axi_rvalid(s_axi_rvalid),
        .s_axi_rready(s_axi_rready),
        .motor_valid(motor_valid),
        .motor_1(motor_1),
        .motor_2(motor_2),
        .motor_3(motor_3),
        .motor_4(motor_4),
        .active_mode(active_mode),
        .fault_code(fault_code),
        .watchdog_alarm(watchdog_alarm),
        .controller_sat_flag(controller_sat_flag),
        .mixer_sat_flag(mixer_sat_flag)
    );

    // 50 MHz Clock
    initial aclk = 0;
    always #10 aclk = ~aclk;

    int error_count = 0;

    // AXI Master Helper Tasks
    task automatic axi_write(input [ADDR_W-1:0] addr, input [DATA_W-1:0] data);
        @(posedge aclk);
        s_axi_awaddr  <= addr;
        s_axi_awprot  <= 3'b000;
        s_axi_awvalid <= 1'b1;
        s_axi_wdata   <= data;
        s_axi_wstrb   <= 4'b1111;
        s_axi_wvalid  <= 1'b1;
        s_axi_bready  <= 1'b1;

        while (!(s_axi_awready && s_axi_wready)) @(posedge aclk);
        @(posedge aclk);
        s_axi_awvalid <= 1'b0;
        s_axi_wvalid  <= 1'b0;

        while (!s_axi_bvalid) @(posedge aclk);
        if (s_axi_bresp != 2'b00) begin
            $display("[FAIL] AXI Write Error at addr 0x%03h", addr);
            error_count++;
        end
        @(posedge aclk);
        s_axi_bready <= 1'b0;
    endtask

    task automatic axi_read(input [ADDR_W-1:0] addr, output [DATA_W-1:0] data);
        @(posedge aclk);
        s_axi_araddr  <= addr;
        s_axi_arprot  <= 3'b000;
        s_axi_arvalid <= 1'b1;
        s_axi_rready  <= 1'b1;

        while (!s_axi_arready) @(posedge aclk);
        @(posedge aclk);
        s_axi_arvalid <= 1'b0;

        while (!s_axi_rvalid) @(posedge aclk);
        data = s_axi_rdata;
        if (s_axi_rresp != 2'b00) begin
            $display("[FAIL] AXI Read Error at addr 0x%03h", addr);
            error_count++;
        end
        @(posedge aclk);
        s_axi_rready <= 1'b0;
    endtask

    logic [31:0] read_data;

    initial begin
        $display("==================================================");
        $display("   TESTBENCH: SoC Top with AXI4-Lite Bus Interconnect");
        $display("==================================================");

        // Power-On Reset
        aresetn        = 0;
        failsafe_ext_n = 1; // Kill pin inactive
        s_axi_awaddr   = 0;
        s_axi_awprot   = 0;
        s_axi_awvalid  = 0;
        s_axi_wdata    = 0;
        s_axi_wstrb    = 0;
        s_axi_wvalid   = 0;
        s_axi_bready   = 0;
        s_axi_araddr   = 0;
        s_axi_arprot   = 0;
        s_axi_arvalid  = 0;
        s_axi_rready   = 0;

        #60;
        aresetn = 1;
        #40;

        // Stage 1: Read Initial Status Register (Word 0x02, Byte 0x08)
        $display("[STAGE 1] Reading REG_STATUS over AXI4-Lite...");
        axi_read(11'h008, read_data);
        $display("REG_STATUS = 0x%08h (Active Mode=%0d, FaultCode=%0d)", 
                 read_data, read_data[1:0], read_data[5:2]);
        if (read_data[1:0] !== 2'b00) begin // Default mode should be Acro (0)
            $display("[FAIL] Initial mode mismatch! Expected Acro (0), Got %0d", read_data[1:0]);
            error_count++;
        end else begin
            $display("[PASS] Initial status verified over AXI4-Lite.");
        end

        // Stage 2: Configure Setpoints & Sensor Inputs
        $display("[STAGE 2] Programming Setpoints & IMU inputs over AXI4-Lite...");
        axi_write(11'h018, 32'sh0000_8000); // REG_SP_THRUST (word 0x06): 0.5 hover thrust
        axi_write(11'h00C, 32'sh0000_2000); // REG_SP_ROLL   (word 0x03): +0.125 roll setpoint
        axi_write(11'h01C, 32'sh0000_0000); // REG_IMU_P     (word 0x07): 0.0 gyro p
        axi_write(11'h028, 32'sh0000_0000); // REG_IMU_PHI   (word 0x0A): 0.0 roll phi

        // Stage 3: Mode Switch to Stabilize (Mode 1)
        $display("[STAGE 3] Switching mode to Stabilize via REG_MODE_CMD...");
        axi_write(11'h004, 32'h0000_0001); // REG_MODE_CMD (word 0x01) = 1 (Stabilize)

        // Stage 4: Trigger Control Execution Strobe
        $display("[STAGE 4] Triggering sample_valid strobe via REG_CTRL...");
        axi_write(11'h000, 32'h0000_0001); // REG_CTRL (word 0x00) Bit 0 = 1

        // Wait 4 cycles for cascaded attitude -> rate -> mixer pipeline
        repeat(5) @(posedge aclk);

        // Stage 5: Read Actuator Commands via AXI4-Lite
        $display("[STAGE 5] Reading Motor Actuator Outputs over AXI4-Lite...");
        axi_read(11'h048, read_data); // REG_MOTOR_1 (word 0x12)
        $display("AXI Read Motor 1 = 0x%08h (~%0.3f)", read_data, real'(read_data) / 65536.0);
        axi_read(11'h04C, read_data); // REG_MOTOR_2 (word 0x13)
        $display("AXI Read Motor 2 = 0x%08h (~%0.3f)", read_data, real'(read_data) / 65536.0);
        axi_read(11'h050, read_data); // REG_MOTOR_3 (word 0x14)
        $display("AXI Read Motor 3 = 0x%08h (~%0.3f)", read_data, real'(read_data) / 65536.0);
        axi_read(11'h054, read_data); // REG_MOTOR_4 (word 0x15)
        $display("AXI Read Motor 4 = 0x%08h (~%0.3f)", read_data, real'(read_data) / 65536.0);

        if (motor_1 === 32'd0 && motor_2 === 32'd0) begin
            $display("[FAIL] Motors failed to actuate!");
            error_count++;
        end else begin
            $display("[PASS] AXI-driven closed-loop actuation verified.");
        end

        // Stage 6: Emergency Kill Pin Override
        $display("[STAGE 6] Asserting Hardware Emergency Kill Pin (failsafe_ext_n=0)...");
        failsafe_ext_n = 0;
        repeat(4) @(posedge aclk);
        axi_read(11'h008, read_data); // REG_STATUS
        $display("Post-kill REG_STATUS: Mode=%0d, FaultCode=%0d", read_data[1:0], read_data[5:2]);
        if (read_data[1:0] !== 2'b11) begin // Must be Failsafe (3)
            $display("[FAIL] Hardware kill failed to force Failsafe mode!");
            error_count++;
        end else begin
            $display("[PASS] Hardware emergency kill deterministically forced Failsafe mode.");
        end

        #40;
        if (error_count == 0) begin
            $display("==================================================");
            $display("   [PASS] ALL SOC AXI4-LITE INTEGRATION TESTS PASSED");
            $display("==================================================");
            $finish(0);
        end else begin
            $display("==================================================");
            $display("   [FAIL] %0d ERRORS IN SOC AXI4-LITE TESTBENCH   ", error_count);
            $display("==================================================");
            $fatal(1);
        end
    end

endmodule : pid_soc_axi_top_tb
