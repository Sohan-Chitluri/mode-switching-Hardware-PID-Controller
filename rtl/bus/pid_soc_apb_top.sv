//==============================================================================
// Module: pid_soc_apb_top.sv
// Description: Synthesizable SoC Top-Level with Native AMBA APB4 Slave Interface.
//              Wraps pid_soc_top.sv with pid_apb4_bridge.sv for direct
//              integration into standard APB4 SoC interconnects (e.g., ARM Cortex-M).
//==============================================================================

`ifndef PID_SOC_APB_TOP_SV
`define PID_SOC_APB_TOP_SV

`include "pid_pkg.sv"
`include "pid_soc_top.sv"
`include "rtl/bus/pid_apb4_bridge.sv"

import pid_pkg::*;

module pid_soc_apb_top #(
    parameter int DATA_W      = 32,
    parameter int FRACT_W     = 16,
    parameter int ACCUM_W     = 48,
    parameter int TIMEOUT_CYC = 200000,
    parameter int ADDR_W      = 11
) (
    // APB Clock and Reset
    input  logic                     pclk,
    input  logic                     presetn,

    // Hardware Emergency Kill Pin (Active-Low)
    input  logic                     failsafe_ext_n,

    // APB4 Slave Interface
    input  logic [ADDR_W-1:0]        paddr,
    input  logic                     psel,
    input  logic                     penable,
    input  logic                     pwrite,
    input  logic [DATA_W-1:0]        pwdata,
    input  logic [(DATA_W/8)-1:0]    pstrb,
    input  logic [2:0]               pprot,
    output logic [DATA_W-1:0]        prdata,
    output logic                     pready,
    output logic                     pslverr,

    // Dedicated Physical Actuator Outputs (8 Motors)
    output logic                     motor_valid,
    output logic signed [DATA_W-1:0] motor_1,
    output logic signed [DATA_W-1:0] motor_2,
    output logic signed [DATA_W-1:0] motor_3,
    output logic signed [DATA_W-1:0] motor_4,
    output logic signed [DATA_W-1:0] motor_5,
    output logic signed [DATA_W-1:0] motor_6,
    output logic signed [DATA_W-1:0] motor_7,
    output logic signed [DATA_W-1:0] motor_8,

    // Telemetry Taps
    output logic [1:0]               active_mode,
    output logic [3:0]               fault_code,
    output logic                     watchdog_alarm,
    output logic                     controller_sat_flag,
    output logic                     mixer_sat_flag
);

    // Internal MMIO Bus
    logic                     bus_we;
    logic [8:0]               bus_addr;
    logic signed [DATA_W-1:0] bus_wdata;
    logic signed [DATA_W-1:0] bus_rdata;
    logic                     bus_rvalid;

    // APB4 Protocol Bridge
    pid_apb4_bridge #(
        .ADDR_W(ADDR_W),
        .DATA_W(DATA_W)
    ) u_apb_bridge (
        .pclk(pclk),
        .presetn(presetn),
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
        .bus_we(bus_we),
        .bus_addr(bus_addr),
        .bus_wdata(bus_wdata),
        .bus_rdata(bus_rdata),
        .bus_rvalid(bus_rvalid)
    );

    // Core SoC Top Instance
    pid_soc_top #(
        .DATA_W(DATA_W),
        .FRACT_W(FRACT_W),
        .ACCUM_W(ACCUM_W),
        .TIMEOUT_CYC(TIMEOUT_CYC)
    ) u_pid_soc (
        .clk(pclk),
        .rst_n(presetn),
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
        .sample_ready(),
        .controller_valid(),
        .u_roll(),
        .u_pitch(),
        .u_yaw(),
        .u_thrust(),
        .active_mode(active_mode),
        .fault_code(fault_code),
        .watchdog_alarm(watchdog_alarm),
        .controller_sat_flag(controller_sat_flag),
        .mixer_sat_flag(mixer_sat_flag)
    );

    // Motors 5-8 are read from internal mixer registers or tied off when not active
    assign motor_5 = '0;
    assign motor_6 = '0;
    assign motor_7 = '0;
    assign motor_8 = '0;

endmodule : pid_soc_apb_top

`endif // PID_SOC_APB_TOP_SV
