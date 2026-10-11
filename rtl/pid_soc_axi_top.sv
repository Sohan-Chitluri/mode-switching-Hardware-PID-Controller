//==============================================================================
// Module: pid_soc_axi_top.sv
// Description: Synthesizable SoC Top-Level with Native AMBA AXI4-Lite Slave Interface.
//              Wraps pid_soc_top.sv with pid_axi_lite_bridge.sv for direct
//              integration into standard RISC-V and ARM SoC interconnects.
//==============================================================================

`ifndef PID_SOC_AXI_TOP_SV
`define PID_SOC_AXI_TOP_SV

`include "pid_pkg.sv"
`include "pid_soc_top.sv"
`include "rtl/bus/pid_axi_lite_bridge.sv"

import pid_pkg::*;

module pid_soc_axi_top #(
    parameter int DATA_W      = 32,
    parameter int FRACT_W     = 16,
    parameter int ACCUM_W     = 48,
    parameter int TIMEOUT_CYC = 200000,
    parameter int ADDR_W      = 11
) (
    // AXI Clock and Reset
    input  logic                     aclk,
    input  logic                     aresetn,

    // Hardware Emergency Kill Pin (Active-Low)
    input  logic                     failsafe_ext_n,

    // AXI4-Lite Slave Interface
    input  logic [ADDR_W-1:0]        s_axi_awaddr,
    input  logic [2:0]               s_axi_awprot,
    input  logic                     s_axi_awvalid,
    output logic                     s_axi_awready,

    input  logic [DATA_W-1:0]        s_axi_wdata,
    input  logic [(DATA_W/8)-1:0]    s_axi_wstrb,
    input  logic                     s_axi_wvalid,
    output logic                     s_axi_wready,

    output logic [1:0]               s_axi_bresp,
    output logic                     s_axi_bvalid,
    input  logic                     s_axi_bready,

    input  logic [ADDR_W-1:0]        s_axi_araddr,
    input  logic [2:0]               s_axi_arprot,
    input  logic                     s_axi_arvalid,
    output logic                     s_axi_arready,

    output logic [DATA_W-1:0]        s_axi_rdata,
    output logic [1:0]               s_axi_rresp,
    output logic                     s_axi_rvalid,
    input  logic                     s_axi_rready,

    // Dedicated Physical Actuator Outputs
    output logic                     motor_valid,
    output logic signed [DATA_W-1:0] motor_1,
    output logic signed [DATA_W-1:0] motor_2,
    output logic signed [DATA_W-1:0] motor_3,
    output logic signed [DATA_W-1:0] motor_4,

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

    // AXI4-Lite Protocol Bridge
    pid_axi_lite_bridge #(
        .ADDR_W(ADDR_W),
        .DATA_W(DATA_W)
    ) u_axi_bridge (
        .aclk(aclk),
        .aresetn(aresetn),
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
        .clk(aclk),
        .rst_n(aresetn),
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

endmodule : pid_soc_axi_top

`endif // PID_SOC_AXI_TOP_SV
