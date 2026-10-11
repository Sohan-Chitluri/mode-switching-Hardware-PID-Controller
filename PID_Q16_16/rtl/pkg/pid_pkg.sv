//==============================================================================
// Package: pid_pkg.sv
// Description: Global types, constants, fixed-point parameters, and modes for
//              the hardware flight-controller PID core.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_PKG_SV
`define PID_PKG_SV

package pid_pkg;

    // Fixed-Point Precision Parameters (Q16.16)
    parameter int DATA_W      = 32;
    parameter int FRACT_W     = 16;
    parameter int ACCUM_W     = 48; // 32 bits + 16 guard bits against overflow
    parameter int PROD_W      = 64; // 32 x 32 full product

    // Dynamic Range Extremes (Signed 32-bit Two's Complement)
    parameter logic signed [DATA_W-1:0] MAX_POS = 32'sh7FFF_FFFF; // +32767.99998
    parameter logic signed [DATA_W-1:0] MIN_NEG = 32'sh8000_0000; // -32768.0

    // Fixed-point 1.0 and 0.5 constants
    parameter logic signed [DATA_W-1:0] FIX_ONE  = 32'sh0001_0000; // 1.0 in Q16.16
    parameter logic signed [DATA_W-1:0] FIX_HALF = 32'sh0000_8000; // 0.5 in Q16.16
    parameter logic signed [DATA_W-1:0] FIX_ZERO = 32'sh0000_0000;

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
    parameter logic signed [DATA_W-1:0] DEF_U_MAX = 32'sh0001_0000; // +1.0
    parameter logic signed [DATA_W-1:0] DEF_U_MIN = -32'sh0001_0000;// -1.0
    parameter logic signed [DATA_W-1:0] DEF_I_MAX = 32'sh0000_8000; // +0.5
    parameter logic signed [DATA_W-1:0] DEF_I_MIN = -32'sh0000_8000;// -0.5

endpackage : pid_pkg

`endif // PID_PKG_SV
