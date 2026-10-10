//==============================================================================
// Module: pid_gain_bank.sv
// Description: Multi-mode parameter gain bank matrix.
//              Houses mode-indexed parameter sets for Acro, Stabilize, AltHold, and Failsafe.
//              Supports memory-mapped configuration bus read/write.
// Language: IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_GAIN_BANK_SV
`define PID_GAIN_BANK_SV

import pid_pkg::*;

module pid_gain_bank #(
    parameter int DATA_W = 32
) (
    input  logic                     clk,
    input  logic                     rst_n,

    // Active Operating Mode
    input  logic [1:0]               active_mode,

    // Configuration Memory-Mapped Bus
    input  logic                     cfg_we,
    input  logic [7:0]               cfg_addr,
    input  logic [DATA_W-1:0]        cfg_wdata,
    output logic [DATA_W-1:0]        cfg_rdata,

    // Active Outer Loop Gains
    output logic signed [DATA_W-1:0] kp_att_roll,
    output logic signed [DATA_W-1:0] kp_att_pitch,
    output logic signed [DATA_W-1:0] kp_alt_pos,
    output logic signed [DATA_W-1:0] hover_thrust,
    output logic signed [DATA_W-1:0] failsafe_thrust,

    // Active Inner Channel Gains: Roll
    output logic signed [DATA_W-1:0] roll_kp,
    output logic signed [DATA_W-1:0] roll_ki_eff,
    output logic signed [DATA_W-1:0] roll_kd_eff,
    output logic signed [DATA_W-1:0] roll_alpha,
    output logic signed [DATA_W-1:0] roll_u_min,
    output logic signed [DATA_W-1:0] roll_u_max,
    output logic signed [DATA_W-1:0] roll_i_min,
    output logic signed [DATA_W-1:0] roll_i_max,

    // Active Inner Channel Gains: Pitch
    output logic signed [DATA_W-1:0] pitch_kp,
    output logic signed [DATA_W-1:0] pitch_ki_eff,
    output logic signed [DATA_W-1:0] pitch_kd_eff,
    output logic signed [DATA_W-1:0] pitch_alpha,
    output logic signed [DATA_W-1:0] pitch_u_min,
    output logic signed [DATA_W-1:0] pitch_u_max,
    output logic signed [DATA_W-1:0] pitch_i_min,
    output logic signed [DATA_W-1:0] pitch_i_max,

    // Active Inner Channel Gains: Yaw
    output logic signed [DATA_W-1:0] yaw_kp,
    output logic signed [DATA_W-1:0] yaw_ki_eff,
    output logic signed [DATA_W-1:0] yaw_kd_eff,
    output logic signed [DATA_W-1:0] yaw_alpha,
    output logic signed [DATA_W-1:0] yaw_u_min,
    output logic signed [DATA_W-1:0] yaw_u_max,
    output logic signed [DATA_W-1:0] yaw_i_min,
    output logic signed [DATA_W-1:0] yaw_i_max,

    // Active Inner Channel Gains: Altitude Thrust
    output logic signed [DATA_W-1:0] thrust_kp,
    output logic signed [DATA_W-1:0] thrust_ki_eff,
    output logic signed [DATA_W-1:0] thrust_kd_eff,
    output logic signed [DATA_W-1:0] thrust_alpha,
    output logic signed [DATA_W-1:0] thrust_u_min,
    output logic signed [DATA_W-1:0] thrust_u_max,
    output logic signed [DATA_W-1:0] thrust_i_min,
    output logic signed [DATA_W-1:0] thrust_i_max
);

    // Default Baseline Gains in Q16.16:
    // Outer angle: Kp_att = 4.5 -> 0x0004_8000
    // Roll/Pitch rate: Kp=0.15 (0x0000_2666), Ki_eff=0.00005 (0x0000_0003), Kd_eff=16.0 (0x0010_0000), alpha=0.3 (0x0000_4CCD)
    // Yaw rate: Kp=0.25 (0x0000_4000), Ki_eff=0.0000375 (0x0000_0002), Kd_eff=0, alpha=0.3 (0x0000_4CCD)
    // Thrust velocity: Kp=0.40 (0x0000_6666), Ki_eff=0.000075 (0x0000_0005), Kd_eff=40.0 (0x0028_0000), alpha=0.2 (0x0000_3333)

    // Register File Storage: 32 registers
    logic [DATA_W-1:0] reg_bank [0:31];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            reg_bank[0]  <= 32'sh0004_8000; // kp_att_roll
            reg_bank[1]  <= 32'sh0004_8000; // kp_att_pitch
            reg_bank[2]  <= 32'sh0001_3333; // kp_alt_pos (1.2)
            reg_bank[3]  <= 32'sh0000_8000; // hover_thrust (0.50)
            reg_bank[4]  <= 32'sh0000_0000; // failsafe_thrust (0.0)

            // Roll
            reg_bank[5]  <= 32'sh0000_2666; // roll_kp
            reg_bank[6]  <= 32'sh0000_0003; // roll_ki_eff
            reg_bank[7]  <= 32'sh0010_0000; // roll_kd_eff
            reg_bank[8]  <= 32'sh0000_4CCD; // roll_alpha
            reg_bank[9]  <= -32'sh0001_0000;// roll_u_min
            reg_bank[10] <= 32'sh0001_0000; // roll_u_max
            reg_bank[11] <= -32'sh0000_8000;// roll_i_min
            reg_bank[12] <= 32'sh0000_8000; // roll_i_max

            // Pitch
            reg_bank[13] <= 32'sh0000_2666; // pitch_kp
            reg_bank[14] <= 32'sh0000_0003; // pitch_ki_eff
            reg_bank[15] <= 32'sh0010_0000; // pitch_kd_eff
            reg_bank[16] <= 32'sh0000_4CCD; // pitch_alpha
            reg_bank[17] <= -32'sh0001_0000;// pitch_u_min
            reg_bank[18] <= 32'sh0001_0000; // pitch_u_max
            reg_bank[19] <= -32'sh0000_8000;// pitch_i_min
            reg_bank[20] <= 32'sh0000_8000; // pitch_i_max

            // Yaw
            reg_bank[21] <= 32'sh0000_4000; // yaw_kp
            reg_bank[22] <= 32'sh0000_0002; // yaw_ki_eff
            reg_bank[23] <= 32'sh0000_0000; // yaw_kd_eff
            reg_bank[24] <= 32'sh0000_4CCD; // yaw_alpha
            reg_bank[25] <= -32'sh0001_0000;// yaw_u_min
            reg_bank[26] <= 32'sh0001_0000; // yaw_u_max
            reg_bank[27] <= -32'sh0000_8000;// yaw_i_min
            reg_bank[28] <= 32'sh0000_8000; // yaw_i_max

            // Thrust
            reg_bank[29] <= 32'sh0000_6666; // thrust_kp
            reg_bank[30] <= 32'sh0000_0005; // thrust_ki_eff
            reg_bank[31] <= 32'sh0028_0000; // thrust_kd_eff
        end else if (cfg_we && (cfg_addr < 32)) begin
            reg_bank[cfg_addr[4:0]] <= cfg_wdata;
        end
    end

    assign cfg_rdata = (cfg_addr < 32) ? reg_bank[cfg_addr[4:0]] : 32'd0;

    // Output assignments
    assign kp_att_roll     = reg_bank[0];
    assign kp_att_pitch    = reg_bank[1];
    assign kp_alt_pos      = reg_bank[2];
    assign hover_thrust    = reg_bank[3];
    assign failsafe_thrust = reg_bank[4];

    // Roll
    assign roll_kp      = reg_bank[5];
    assign roll_ki_eff  = reg_bank[6];
    assign roll_kd_eff  = reg_bank[7];
    assign roll_alpha   = reg_bank[8];
    assign roll_u_min   = reg_bank[9];
    assign roll_u_max   = reg_bank[10];
    assign roll_i_min   = reg_bank[11];
    assign roll_i_max   = reg_bank[12];

    // Pitch
    assign pitch_kp     = reg_bank[13];
    assign pitch_ki_eff = reg_bank[14];
    assign pitch_kd_eff = reg_bank[15];
    assign pitch_alpha  = reg_bank[16];
    assign pitch_u_min  = reg_bank[17];
    assign pitch_u_max  = reg_bank[18];
    assign pitch_i_min  = reg_bank[19];
    assign pitch_i_max  = reg_bank[20];

    // Yaw
    assign yaw_kp       = reg_bank[21];
    assign yaw_ki_eff   = reg_bank[22];
    assign yaw_kd_eff   = reg_bank[23];
    assign yaw_alpha    = reg_bank[24];
    assign yaw_u_min    = reg_bank[25];
    assign yaw_u_max    = reg_bank[26];
    assign yaw_i_min    = reg_bank[27];
    assign yaw_i_max    = reg_bank[28];

    // Altitude Thrust
    assign thrust_kp     = reg_bank[29];
    assign thrust_ki_eff = reg_bank[30];
    assign thrust_kd_eff = reg_bank[31];
    assign thrust_alpha  = 32'sh0000_3333; // 0.20
    assign thrust_u_min  = 32'sh0000_0000; // 0.0
    assign thrust_u_max  = 32'sh0001_0000; // 1.0
    assign thrust_i_min  = -32'sh0000_4000;// -0.25
    assign thrust_i_max  = 32'sh0000_4000; // +0.25

endmodule : pid_gain_bank

`endif // PID_GAIN_BANK_SV
