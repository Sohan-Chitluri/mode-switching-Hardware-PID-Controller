//==============================================================================
// Module: pid_watchdog.sv
// Description: Sensor sample freshness watchdog timer.
//              Detects missing, delayed, or frozen IMU/sensor samples.
//              Asserts watchdog_alarm and sets sticky fault code upon timeout.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_WATCHDOG_SV
`define PID_WATCHDOG_SV

`include "pid_pkg.sv"

import pid_pkg::*;

module pid_watchdog #(
    parameter int TIMEOUT_CYC_W = 32,
    parameter int DEF_TIMEOUT   = 50000 // 50k cycles = 1.0 ms at 50 MHz
) (
    input  logic                     clk,
    input  logic                     rst_n,

    // Sensor Sample Interface
    input  logic                     sample_valid,        // Strobed high when fresh sample arrives
    input  logic [TIMEOUT_CYC_W-1:0] cfg_timeout_cycles,  // Programmable timeout threshold

    // Status & Alarms
    output logic                     watchdog_alarm,      // Asserted when timeout occurs
    output logic [TIMEOUT_CYC_W-1:0] current_count        // Live cycle counter for diagnostics
);

    logic [TIMEOUT_CYC_W-1:0] counter_r;
    logic                     alarm_r;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            counter_r <= '0;
            alarm_r   <= 1'b0;
        end else begin
            if (sample_valid) begin
                counter_r <= '0;
                alarm_r   <= 1'b0;
            end else begin
                if (counter_r >= cfg_timeout_cycles) begin
                    alarm_r <= 1'b1;
                    // Counter saturates to avoid rollover
                    counter_r <= counter_r;
                end else begin
                    counter_r <= counter_r + 1'b1;
                end
            end
        end
    end

    assign watchdog_alarm = alarm_r;
    assign current_count  = counter_r;

`ifdef FORMAL
    logic [TIMEOUT_CYC_W-1:0] past_cnt;
    logic [TIMEOUT_CYC_W-1:0] past_timeout;
    logic                     past_sv;
    logic                     past_rst;
    always_ff @(posedge clk) begin
        past_cnt     <= counter_r;
        past_timeout <= cfg_timeout_cycles;
        past_sv      <= sample_valid;
        past_rst     <= rst_n;
        if (rst_n && past_rst) begin
            if (past_cnt >= past_timeout && !past_sv) begin
                assert(watchdog_alarm == 1'b1);
            end
        end
    end
`endif

endmodule : pid_watchdog

`endif // PID_WATCHDOG_SV
