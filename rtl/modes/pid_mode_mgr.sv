//==============================================================================
// Module: pid_mode_mgr.sv
// Description: Operating Mode Manager & Transition FSM.
//              Manages state across Acro, Stabilize, AltHold, and Failsafe.
//              Generates single-cycle transition_strobe for bumpless transfer re-seeding.
//              Enforces deterministic single-cycle jump to Failsafe on any fault.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_MODE_MGR_SV
`define PID_MODE_MGR_SV

import pid_pkg::*;

module pid_mode_mgr (
    input  logic       clk,
    input  logic       rst_n,

    input  logic       sample_valid,    // IMU sensor sample valid strobe
    input  logic [1:0] mode_cmd,        // 00=Acro, 01=Stab, 10=AltHold, 11=Failsafe
    input  logic       rearm_cmd,       // Strobe to clear sticky fault and exit failsafe

    // Hardware Safety Fault Strobes
    input  logic       watchdog_alarm,  // Stale IMU sample timeout
    input  logic       failsafe_ext_n,  // Active-low emergency kill switch
    input  logic       alt_valid,       // Altitude sensor valid flag

    // Mode Status and Handover Strobes
    output logic [1:0] active_mode,       // Registered operating mode
    output logic [1:0] next_mode,         // Combinational target mode for current cycle
    output logic [1:0] prev_mode,         // Previous mode before latest transition
    output logic       transition_strobe, // 1-cycle pulse asserted on mode boundary
    output logic [3:0] fault_code         // Diagnostic sticky fault status
);

    logic [1:0] state_r;
    logic [1:0] prev_state_r;
    logic [1:0] next_state;
    logic [3:0] fault_r;

    logic [3:0] active_fault;

    // Instantaneous fault resolution (rearm clears sticky fault combinationally if conditions are safe and sensor sample is valid)
    assign active_fault = (rearm_cmd && sample_valid && !watchdog_alarm && failsafe_ext_n) ? FAULT_NONE :
                          (fault_r | (watchdog_alarm ? FAULT_WATCHDOG_STALE : 4'd0) | (!failsafe_ext_n ? FAULT_EXT_FAILSAFE : 4'd0));

    // Fault latching logic
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            fault_r <= FAULT_NONE;
        end else begin
            if (rearm_cmd && sample_valid && !watchdog_alarm && failsafe_ext_n) begin
                fault_r <= FAULT_NONE; // Clear sticky fault on re-arm
            end else begin
                if (watchdog_alarm)
                    fault_r <= fault_r | FAULT_WATCHDOG_STALE;
                if (!failsafe_ext_n)
                    fault_r <= fault_r | FAULT_EXT_FAILSAFE;
            end
        end
    end

    logic [1:0] req_mode;

    // Next State Logic
    always_comb begin
        req_mode = MODE_FAILSAFE;
        if (active_fault != FAULT_NONE) begin
            // Safety invariant: Any active fault locks controller into Failsafe immediately
            next_state = MODE_FAILSAFE;
        end else if (!sample_valid) begin
            // Hold current operating mode between sample valid strobes
            next_state = state_r;
        end else begin
            // Normal mode transitions evaluated synchronously on sample boundaries
            // Sticky Failsafe lock until explicit re-arm with valid sensor data
            if (state_r == MODE_FAILSAFE) begin
                req_mode = (rearm_cmd) ? mode_cmd : MODE_FAILSAFE;
            end else begin
                req_mode = mode_cmd;
            end

            case (req_mode)
                MODE_ACRO:      next_state = MODE_ACRO;
                MODE_STABILIZE: next_state = MODE_STABILIZE;
                MODE_ALTHOLD: begin
                    // Graceful degradation: if altitude sensor drops, stay in Stabilize
                    if (alt_valid)
                        next_state = MODE_ALTHOLD;
                    else
                        next_state = MODE_STABILIZE;
                end
                MODE_FAILSAFE:  next_state = MODE_FAILSAFE;
                default:        next_state = MODE_ACRO;
            endcase
        end
    end

    // Sequential State Tracking
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state_r      <= MODE_ACRO;
            prev_state_r <= MODE_ACRO;
        end else begin
            prev_state_r <= state_r;
            state_r      <= next_state;
        end
    end

    assign active_mode       = state_r;
    assign next_mode         = next_state;
    assign prev_mode         = prev_state_r;
    assign transition_strobe = (next_state != state_r);
    assign fault_code        = fault_r;

`ifdef FORMAL
    always_comb begin
        if (!failsafe_ext_n || watchdog_alarm) begin
            assert(next_mode == MODE_FAILSAFE);
        end
    end
`endif

endmodule : pid_mode_mgr

`endif // PID_MODE_MGR_SV
