//==============================================================================
// Package: pid_pkg.sv
// Description: Global types, constants, fixed-point parameters, and modes for
//              the 16-bit hardware flight-controller PID core (Q4.12 precision).
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_PKG_SV
`define PID_PKG_SV

package pid_pkg;

    // Fixed-Point Precision Parameters (Q4.12)
    parameter int DATA_W      = 16;
    parameter int FRACT_W     = 12;
    parameter int ACCUM_W     = 24; // 16 bits + 8 guard bits against overflow
    parameter int PROD_W      = 32; // 16 x 16 full product

    // Dynamic Range Extremes (Signed 16-bit Two's Complement)
    parameter logic signed [DATA_W-1:0] MAX_POS = 16'sh7FFF; // +7.99975
    parameter logic signed [DATA_W-1:0] MIN_NEG = 16'sh8000; // -8.00000

    // Fixed-point 1.0 and 0.5 constants (Q4.12 format: 1.0 = 0x1000 = 4096)
    parameter logic signed [DATA_W-1:0] FIX_ONE  = 16'sh1000; // 1.0 in Q4.12
    parameter logic signed [DATA_W-1:0] FIX_HALF = 16'sh0800; // 0.5 in Q4.12
    parameter logic signed [DATA_W-1:0] FIX_ZERO = 16'sh0000; // 0.0 in Q4.12

    // Operating Mode Encodings
    typedef enum logic [1:0] {
        MODE_ACRO      = 2'b00, // Direct rate feedback loop
        MODE_STABILIZE = 2'b01, // Cascaded attitude angle -> rate loop
        MODE_ALTHOLD   = 2'b10, // Altitude position/velocity -> thrust loop
        MODE_FAILSAFE  = 2'b11  // Deterministic safe output override
    } mode_t;

    // Diagnostic Fault Codes (One-Hot or Bitmask)
    parameter logic [3:0] FAULT_NONE          = 4'b0000;
    parameter logic [3:0] FAULT_WATCHDOG_STALE= 4'b0001; // Stale IMU sample timeout
    parameter logic [3:0] FAULT_EXT_FAILSAFE  = 4'b0010; // Hardware kill-switch pin
    parameter logic [3:0] FAULT_ALT_INVALID   = 4'b0100; // Altitude sensor dropped
    parameter logic [3:0] FAULT_ARITH_OVERFLOW= 4'b1000; // Internal overflow clamp

    // Default Saturation Limits
    parameter logic signed [DATA_W-1:0] DEF_U_MAX = 16'sh1000; // +1.0
    parameter logic signed [DATA_W-1:0] DEF_U_MIN = -16'sh1000;// -1.0
    parameter logic signed [DATA_W-1:0] DEF_I_MAX = 16'sh0800; // +0.5
    parameter logic signed [DATA_W-1:0] DEF_I_MIN = -16'sh0800;// -0.5

endpackage : pid_pkg

`endif // PID_PKG_SV
