//==============================================================================
// Testbench: pid_soc_apb_top_tb.sv
// Description: Comprehensive SoC integration testbench for pid_soc_apb_top.
//              Emulates an APB4 Host CPU (e.g., ARM Cortex-M) interacting with
//              the Hardware PID Flight Controller over AMBA APB4 protocol.
//==============================================================================

`timescale 1ns/1ps

`include "pid_pkg.sv"
`include "pid_soc_apb_top.sv"

module pid_soc_apb_top_tb;

    import pid_pkg::*;

    localparam int DATA_W      = 32;
    localparam int FRACT_W     = 16;
    localparam int ACCUM_W     = 48;
    localparam int TIMEOUT_CYC = 200;
    localparam int ADDR_W      = 11;

    logic pclk;
    logic presetn;
    logic failsafe_ext_n;

    // APB4 Interface
    logic [ADDR_W-1:0]        paddr;
    logic                     psel;
    logic                     penable;
    logic                     pwrite;
    logic [DATA_W-1:0]        pwdata;
    logic [(DATA_W/8)-1:0]    pstrb;
    logic [2:0]               pprot;
    logic [DATA_W-1:0]        prdata;
    logic                     pready;
    logic                     pslverr;

    // Actuator Outputs
    logic                     motor_valid;
    logic signed [DATA_W-1:0] motor_1;
    logic signed [DATA_W-1:0] motor_2;
    logic signed [DATA_W-1:0] motor_3;
    logic signed [DATA_W-1:0] motor_4;
    logic signed [DATA_W-1:0] motor_5;
    logic signed [DATA_W-1:0] motor_6;
    logic signed [DATA_W-1:0] motor_7;
    logic signed [DATA_W-1:0] motor_8;

    // Telemetry Taps
    logic [1:0]               active_mode;
    logic [3:0]               fault_code;
    logic                     watchdog_alarm;
    logic                     controller_sat_flag;
    logic                     mixer_sat_flag;

    // DUT Instance
    pid_soc_apb_top #(
        .DATA_W(DATA_W),
        .FRACT_W(FRACT_W),
        .ACCUM_W(ACCUM_W),
        .TIMEOUT_CYC(TIMEOUT_CYC),
        .ADDR_W(ADDR_W)
    ) dut (
        .pclk(pclk),
        .presetn(presetn),
        .failsafe_ext_n(failsafe_ext_n),
        .paddr(paddr),
        .psel(psel),
        .penable(penable),
        .pwrite(pwrite),
        .pwdata(pwdata),
        .pstrb(pstrb),
        .pprot(pprot),
        .prdata(prdata),
        .pready(pready),
        .pslverr(pslverr),
        .motor_valid(motor_valid),
        .motor_1(motor_1),
        .motor_2(motor_2),
        .motor_3(motor_3),
        .motor_4(motor_4),
        .motor_5(motor_5),
        .motor_6(motor_6),
        .motor_7(motor_7),
        .motor_8(motor_8),
        .active_mode(active_mode),
        .fault_code(fault_code),
        .watchdog_alarm(watchdog_alarm),
        .controller_sat_flag(controller_sat_flag),
        .mixer_sat_flag(mixer_sat_flag)
    );

    // 50 MHz Clock
    initial pclk = 0;
    always #10 pclk = ~pclk;

    int error_count = 0;

    // APB4 Master Helper Tasks
    task automatic apb_write(input [ADDR_W-1:0] addr, input [DATA_W-1:0] data);
        @(posedge pclk);
        paddr   <= addr;
        pwrite  <= 1'b1;
        psel    <= 1'b1;
        penable <= 1'b0;
        pwdata  <= data;
        pstrb   <= 4'b1111;
        pprot   <= 3'b000;

        @(posedge pclk);
        penable <= 1'b1;

        while (!pready) @(posedge pclk);
        @(posedge pclk);
        psel    <= 1'b0;
        penable <= 1'b0;
        pwrite  <= 1'b0;
    endtask

    task automatic apb_read(input [ADDR_W-1:0] addr, output [DATA_W-1:0] data);
        @(posedge pclk);
        paddr   <= addr;
        pwrite  <= 1'b0;
        psel    <= 1'b1;
        penable <= 1'b0;

        @(posedge pclk);
        penable <= 1'b1;

        while (!pready) @(posedge pclk);
        data = prdata;
        @(posedge pclk);
        psel    <= 1'b0;
        penable <= 1'b0;
    endtask

    logic [31:0] read_data;

    initial begin
        $display("==================================================");
        $display("   TESTBENCH: SoC Top with APB4 Bus Interconnect  ");
        $display("==================================================");

        // Power-On Reset
        presetn        = 0;
        failsafe_ext_n = 1;
        paddr          = 0;
        psel           = 0;
        penable        = 0;
        pwrite         = 0;
        pwdata         = 0;
        pstrb          = 0;
        pprot          = 0;

        #60;
        presetn = 1;
        #40;

        // Stage 1: Read Initial Status Register over APB
        $display("[STAGE 1] Reading REG_STATUS over APB4...");
        apb_read(11'h008, read_data); // REG_STATUS (word 0x02)
        $display("REG_STATUS = 0x%08h (Active Mode=%0d, FaultCode=%0d)", 
                 read_data, read_data[1:0], read_data[5:2]);
        if (read_data[1:0] !== 2'b00) begin
            $display("[FAIL] Initial mode mismatch! Expected Acro (0), Got %0d", read_data[1:0]);
            error_count++;
        end else begin
            $display("[PASS] Initial status verified over APB4.");
        end

        // Stage 2: Configure Setpoints & Sensor Inputs
        $display("[STAGE 2] Programming Setpoints & IMU inputs over APB4...");
        apb_write(11'h018, 32'sh0000_8000); // REG_SP_THRUST (word 0x06): 0.5 hover thrust
        apb_write(11'h00C, 32'sh0000_2000); // REG_SP_ROLL   (word 0x03): +0.125 roll setpoint
        apb_write(11'h01C, 32'sh0000_0000); // REG_IMU_P     (word 0x07): 0.0 gyro p
        apb_write(11'h028, 32'sh0000_0000); // REG_IMU_PHI   (word 0x0A): 0.0 roll phi

        // Stage 3: Mode Switch to Stabilize (Mode 1)
        $display("[STAGE 3] Switching mode to Stabilize via REG_MODE_CMD...");
        apb_write(11'h004, 32'h0000_0001); // REG_MODE_CMD = 1 (Stabilize)

        // Stage 4: Trigger Control Execution Strobe
        $display("[STAGE 4] Triggering sample_valid strobe via REG_CTRL...");
        apb_write(11'h000, 32'h0000_0001); // REG_CTRL Bit 0 = 1

        repeat(5) @(posedge pclk);

        // Stage 5: Read Actuator Commands via APB4
        $display("[STAGE 5] Reading Motor Actuator Outputs over APB4...");
        apb_read(11'h048, read_data); // REG_MOTOR_1
        $display("APB Read Motor 1 = 0x%08h (~%0.3f)", read_data, real'(read_data) / 65536.0);
        apb_read(11'h04C, read_data); // REG_MOTOR_2
        $display("APB Read Motor 2 = 0x%08h (~%0.3f)", read_data, real'(read_data) / 65536.0);

        if (motor_1 === 32'd0 && motor_2 === 32'd0) begin
            $display("[FAIL] Motors failed to actuate!");
            error_count++;
        end else begin
            $display("[PASS] APB-driven closed-loop actuation verified.");
        end

        // Stage 6: Emergency Kill Pin Override
        $display("[STAGE 6] Asserting Hardware Emergency Kill Pin (failsafe_ext_n=0)...");
        failsafe_ext_n = 0;
        repeat(4) @(posedge pclk);
        apb_read(11'h008, read_data); // REG_STATUS
        $display("Post-kill REG_STATUS: Mode=%0d, FaultCode=%0d", read_data[1:0], read_data[5:2]);
        if (read_data[1:0] !== 2'b11) begin
            $display("[FAIL] Hardware kill failed to force Failsafe mode!");
            error_count++;
        end else begin
            $display("[PASS] Hardware emergency kill deterministically forced Failsafe mode.");
        end

        #40;
        if (error_count == 0) begin
            $display("==================================================");
            $display("   [PASS] ALL SOC APB4 INTEGRATION TESTS PASSED   ");
            $display("==================================================");
            $finish(0);
        end else begin
            $display("==================================================");
            $display("   [FAIL] %0d ERRORS IN SOC APB4 TESTBENCH        ", error_count);
            $display("==================================================");
            $fatal(1);
        end
    end

endmodule : pid_soc_apb_top_tb
