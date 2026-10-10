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

    // 4 Mode Banks of 32 registers each:
    // Bank 0 (2'b00): MODE_ACRO
    // Bank 1 (2'b01): MODE_STABILIZE
    // Bank 2 (2'b10): MODE_ALTHOLD
    // Bank 3 (2'b11): MODE_FAILSAFE
    logic [DATA_W-1:0] reg_bank [0:3][0:31];
    // Mode-specific thrust parameter bank: [mode 0..3][param 0..4]
    // 0: thrust_alpha, 1: thrust_u_min, 2: thrust_u_max, 3: thrust_i_min, 4: thrust_i_max
    logic [DATA_W-1:0] thrust_bank [0:3][0:4];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            // -------------------------------------------------------------
            // Bank 0: MODE_ACRO (Rate feedback loop)
            // -------------------------------------------------------------
            reg_bank[0][0]  <= 32'sh0004_8000; // kp_att_roll
            reg_bank[0][1]  <= 32'sh0004_8000; // kp_att_pitch
            reg_bank[0][2]  <= 32'sh0001_3333; // kp_alt_pos (1.2)
            reg_bank[0][3]  <= 32'sh0000_8000; // hover_thrust (0.50)
            reg_bank[0][4]  <= 32'sh0000_0000; // failsafe_thrust (0.0)

            // Roll rate (agile rate control)
            reg_bank[0][5]  <= 32'sh0000_4000; // roll_kp = 0.25
            reg_bank[0][6]  <= 32'sh0000_0007; // roll_ki_eff = 0.0001 (Ki=0.40)
            reg_bank[0][7]  <= 32'sh0014_0000; // roll_kd_eff = 20.0 (Kd=0.005)
            reg_bank[0][8]  <= 32'sh0000_4CCD; // roll_alpha = 0.3
            reg_bank[0][9]  <= -32'sh0001_0000;// roll_u_min = -1.0
            reg_bank[0][10] <= 32'sh0001_0000; // roll_u_max = +1.0
            reg_bank[0][11] <= -32'sh0000_8000;// roll_i_min = -0.5
            reg_bank[0][12] <= 32'sh0000_8000; // roll_i_max = +0.5

            // Pitch rate (agile rate control)
            reg_bank[0][13] <= 32'sh0000_4000; // pitch_kp = 0.25
            reg_bank[0][14] <= 32'sh0000_0007; // pitch_ki_eff = 0.0001
            reg_bank[0][15] <= 32'sh0014_0000; // pitch_kd_eff = 20.0
            reg_bank[0][16] <= 32'sh0000_4CCD; // pitch_alpha = 0.3
            reg_bank[0][17] <= -32'sh0001_0000;// pitch_u_min = -1.0
            reg_bank[0][18] <= 32'sh0001_0000; // pitch_u_max = +1.0
            reg_bank[0][19] <= -32'sh0000_8000;// pitch_i_min = -0.5
            reg_bank[0][20] <= 32'sh0000_8000; // pitch_i_max = +0.5

            // Yaw rate
            reg_bank[0][21] <= 32'sh0000_4000; // yaw_kp = 0.25
            reg_bank[0][22] <= 32'sh0000_0002; // yaw_ki_eff = 0.0000375
            reg_bank[0][23] <= 32'sh0000_0000; // yaw_kd_eff = 0
            reg_bank[0][24] <= 32'sh0000_4CCD; // yaw_alpha = 0.3
            reg_bank[0][25] <= -32'sh0001_0000;// yaw_u_min = -1.0
            reg_bank[0][26] <= 32'sh0001_0000; // yaw_u_max = +1.0
            reg_bank[0][27] <= -32'sh0000_8000;// yaw_i_min = -0.5
            reg_bank[0][28] <= 32'sh0000_8000; // yaw_i_max = +0.5

            // Thrust
            reg_bank[0][29] <= 32'sh0000_6666; // thrust_kp = 0.40
            reg_bank[0][30] <= 32'sh0000_0005; // thrust_ki_eff = 0.000075
            reg_bank[0][31] <= 32'sh0028_0000; // thrust_kd_eff = 40.0

            // -------------------------------------------------------------
            // Bank 1: MODE_STABILIZE (Cascaded attitude angle -> rate loop)
            // Tuned for quadrotor physical stabilization (Phase 1 tuned baseline)
            // -------------------------------------------------------------
            reg_bank[1][0]  <= 32'sh0023_0000; // kp_att_roll = 35.0
            reg_bank[1][1]  <= 32'sh0023_0000; // kp_att_pitch = 35.0
            reg_bank[1][2]  <= 32'sh0001_3333; // kp_alt_pos = 1.2
            reg_bank[1][3]  <= 32'sh0000_8000; // hover_thrust = 0.50
            reg_bank[1][4]  <= 32'sh0000_0000; // failsafe_thrust = 0.0

            // Roll rate (stabilizing damping)
            reg_bank[1][5]  <= 32'sh0002_0000; // roll_kp = 2.0
            reg_bank[1][6]  <= 32'sh0000_019A; // roll_ki_eff = 0.00625 (Ki=25.0)
            reg_bank[1][7]  <= 32'sh0050_0000; // roll_kd_eff = 80.0 (Kd=0.020)
            reg_bank[1][8]  <= 32'sh0000_4CCD; // roll_alpha = 0.3
            reg_bank[1][9]  <= -32'sh0001_0000;// roll_u_min = -1.0
            reg_bank[1][10] <= 32'sh0001_0000; // roll_u_max = +1.0
            reg_bank[1][11] <= -32'sh0000_8000;// roll_i_min = -0.5
            reg_bank[1][12] <= 32'sh0000_8000; // roll_i_max = +0.5

            // Pitch rate (stabilizing damping)
            reg_bank[1][13] <= 32'sh0002_0000; // pitch_kp = 2.0
            reg_bank[1][14] <= 32'sh0000_019A; // pitch_ki_eff = 0.00625 (Ki=25.0)
            reg_bank[1][15] <= 32'sh0050_0000; // pitch_kd_eff = 80.0 (Kd=0.020)
            reg_bank[1][16] <= 32'sh0000_4CCD; // pitch_alpha = 0.3
            reg_bank[1][17] <= -32'sh0001_0000;// pitch_u_min = -1.0
            reg_bank[1][18] <= 32'sh0001_0000; // pitch_u_max = +1.0
            reg_bank[1][19] <= -32'sh0000_8000;// pitch_i_min = -0.5
            reg_bank[1][20] <= 32'sh0000_8000; // pitch_i_max = +0.5

            // Yaw rate
            reg_bank[1][21] <= 32'sh0000_4000; // yaw_kp = 0.25
            reg_bank[1][22] <= 32'sh0000_0002; // yaw_ki_eff = 0.0000375
            reg_bank[1][23] <= 32'sh0000_0000; // yaw_kd_eff = 0
            reg_bank[1][24] <= 32'sh0000_4CCD; // yaw_alpha = 0.3
            reg_bank[1][25] <= -32'sh0001_0000;
            reg_bank[1][26] <= 32'sh0001_0000;
            reg_bank[1][27] <= -32'sh0000_8000;
            reg_bank[1][28] <= 32'sh0000_8000;

            // Thrust
            reg_bank[1][29] <= 32'sh0000_6666; // thrust_kp = 0.40
            reg_bank[1][30] <= 32'sh0000_0005; // thrust_ki_eff = 0.000075
            reg_bank[1][31] <= 32'sh0028_0000; // thrust_kd_eff = 40.0

            // -------------------------------------------------------------
            // Bank 2: MODE_ALTHOLD (Attitude + Altitude position loop)
            // -------------------------------------------------------------
            reg_bank[2][0]  <= 32'sh0023_0000; // kp_att_roll = 35.0
            reg_bank[2][1]  <= 32'sh0023_0000; // kp_att_pitch = 35.0
            reg_bank[2][2]  <= 32'sh0001_3333; // kp_alt_pos = 1.2
            reg_bank[2][3]  <= 32'sh0000_8000; // hover_thrust = 0.50
            reg_bank[2][4]  <= 32'sh0000_0000; // failsafe_thrust = 0.0

            // Roll rate
            reg_bank[2][5]  <= 32'sh0002_0000; // roll_kp = 2.0
            reg_bank[2][6]  <= 32'sh0000_019A; // roll_ki_eff = 0.00625
            reg_bank[2][7]  <= 32'sh0050_0000; // roll_kd_eff = 80.0
            reg_bank[2][8]  <= 32'sh0000_4CCD; // roll_alpha = 0.3
            reg_bank[2][9]  <= -32'sh0001_0000;
            reg_bank[2][10] <= 32'sh0001_0000;
            reg_bank[2][11] <= -32'sh0000_8000;
            reg_bank[2][12] <= 32'sh0000_8000;

            // Pitch rate
            reg_bank[2][13] <= 32'sh0002_0000; // pitch_kp = 2.0
            reg_bank[2][14] <= 32'sh0000_019A; // pitch_ki_eff = 0.00625
            reg_bank[2][15] <= 32'sh0050_0000; // pitch_kd_eff = 80.0
            reg_bank[2][16] <= 32'sh0000_4CCD; // pitch_alpha = 0.3
            reg_bank[2][17] <= -32'sh0001_0000;
            reg_bank[2][18] <= 32'sh0001_0000;
            reg_bank[2][19] <= -32'sh0000_8000;
            reg_bank[2][20] <= 32'sh0000_8000;

            // Yaw rate
            reg_bank[2][21] <= 32'sh0000_4000;
            reg_bank[2][22] <= 32'sh0000_0002;
            reg_bank[2][23] <= 32'sh0000_0000;
            reg_bank[2][24] <= 32'sh0000_4CCD;
            reg_bank[2][25] <= -32'sh0001_0000;
            reg_bank[2][26] <= 32'sh0001_0000;
            reg_bank[2][27] <= -32'sh0000_8000;
            reg_bank[2][28] <= 32'sh0000_8000;

            // Altitude Thrust climb velocity loop
            reg_bank[2][29] <= 32'sh0000_6666; // thrust_kp = 0.40
            reg_bank[2][30] <= 32'sh0000_0005; // thrust_ki_eff = 0.000075
            reg_bank[2][31] <= 32'sh0028_0000; // thrust_kd_eff = 40.0

            // -------------------------------------------------------------
            // Bank 3: MODE_FAILSAFE (Deterministic Safe Bounds)
            // -------------------------------------------------------------
            reg_bank[3][0]  <= 32'sh0000_0000;
            reg_bank[3][1]  <= 32'sh0000_0000;
            reg_bank[3][2]  <= 32'sh0000_0000;
            reg_bank[3][3]  <= 32'sh0000_0000;
            reg_bank[3][4]  <= 32'sh0000_0000; // failsafe_thrust = 0.0

            reg_bank[3][5]  <= 32'sh0000_0000;
            reg_bank[3][6]  <= 32'sh0000_0000;
            reg_bank[3][7]  <= 32'sh0000_0000;
            reg_bank[3][8]  <= 32'sh0000_4CCD;
            reg_bank[3][9]  <= -32'sh0001_0000;
            reg_bank[3][10] <= 32'sh0001_0000;
            reg_bank[3][11] <= -32'sh0000_8000;
            reg_bank[3][12] <= 32'sh0000_8000;

            reg_bank[3][13] <= 32'sh0000_0000;
            reg_bank[3][14] <= 32'sh0000_0000;
            reg_bank[3][15] <= 32'sh0000_0000;
            reg_bank[3][16] <= 32'sh0000_4CCD;
            reg_bank[3][17] <= -32'sh0001_0000;
            reg_bank[3][18] <= 32'sh0001_0000;
            reg_bank[3][19] <= -32'sh0000_8000;
            reg_bank[3][20] <= 32'sh0000_8000;

            reg_bank[3][21] <= 32'sh0000_0000;
            reg_bank[3][22] <= 32'sh0000_0000;
            reg_bank[3][23] <= 32'sh0000_0000;
            reg_bank[3][24] <= 32'sh0000_4CCD;
            reg_bank[3][25] <= -32'sh0001_0000;
            reg_bank[3][26] <= 32'sh0001_0000;
            reg_bank[3][27] <= -32'sh0000_8000;
            reg_bank[3][28] <= 32'sh0000_8000;

            reg_bank[3][29] <= 32'sh0000_0000;
            reg_bank[3][30] <= 32'sh0000_0000;
            reg_bank[3][31] <= 32'sh0000_0000;

            // -------------------------------------------------------------
            // Thrust Parameter Banks per Mode:
            // [0]: thrust_alpha, [1]: thrust_u_min, [2]: thrust_u_max,
            // [3]: thrust_i_min, [4]: thrust_i_max
            // -------------------------------------------------------------
            // Bank 0: Acro
            thrust_bank[0][0] <= 32'sh0000_3333; // alpha = 0.20
            thrust_bank[0][1] <= 32'sh0000_0000; // u_min = 0.0
            thrust_bank[0][2] <= 32'sh0001_0000; // u_max = 1.0
            thrust_bank[0][3] <= -32'sh0000_4000;// i_min = -0.25
            thrust_bank[0][4] <= 32'sh0000_4000; // i_max = +0.25

            // Bank 1: Stabilize
            thrust_bank[1][0] <= 32'sh0000_3333; // alpha = 0.20
            thrust_bank[1][1] <= 32'sh0000_0000; // u_min = 0.0
            thrust_bank[1][2] <= 32'sh0001_0000; // u_max = 1.0
            thrust_bank[1][3] <= -32'sh0000_4000;// i_min = -0.25
            thrust_bank[1][4] <= 32'sh0000_4000; // i_max = +0.25

            // Bank 2: AltHold (velocity control delta around hover thrust)
            thrust_bank[2][0] <= 32'sh0000_3333; // alpha = 0.20
            thrust_bank[2][1] <= -32'sh0000_8000;// u_min = -0.50 (allows descent delta)
            thrust_bank[2][2] <= 32'sh0000_8000; // u_max = +0.50 (allows climb delta)
            thrust_bank[2][3] <= -32'sh0000_4000;// i_min = -0.25
            thrust_bank[2][4] <= 32'sh0000_4000; // i_max = +0.25

            // Bank 3: Failsafe
            thrust_bank[3][0] <= 32'sh0000_3333; // alpha = 0.20
            thrust_bank[3][1] <= 32'sh0000_0000; // u_min = 0.0
            thrust_bank[3][2] <= 32'sh0001_0000; // u_max = 1.0
            thrust_bank[3][3] <= -32'sh0000_4000;// i_min = -0.25
            thrust_bank[3][4] <= 32'sh0000_4000; // i_max = +0.25

        end else if (cfg_we) begin
            if (cfg_addr < 128) begin
                reg_bank[cfg_addr[6:5]][cfg_addr[4:0]] <= cfg_wdata;
            end else if (cfg_addr >= 128 && cfg_addr < 160 && cfg_addr[2:0] < 5) begin
                thrust_bank[cfg_addr[4:3]][cfg_addr[2:0]] <= cfg_wdata;
            end
        end
    end

    // Configuration bus read data multiplexing
    always_comb begin
        if (cfg_addr < 128) begin
            cfg_rdata = reg_bank[cfg_addr[6:5]][cfg_addr[4:0]];
        end else if (cfg_addr >= 128 && cfg_addr < 160 && cfg_addr[2:0] < 5) begin
            cfg_rdata = thrust_bank[cfg_addr[4:3]][cfg_addr[2:0]];
        end else begin
            cfg_rdata = 32'd0;
        end
    end

    // Active Mode Multiplexed Output Assignments
    assign kp_att_roll     = reg_bank[active_mode][0];
    assign kp_att_pitch    = reg_bank[active_mode][1];
    assign kp_alt_pos      = reg_bank[active_mode][2];
    assign hover_thrust    = reg_bank[active_mode][3];
    assign failsafe_thrust = reg_bank[active_mode][4];

    // Roll Active Channel
    assign roll_kp      = reg_bank[active_mode][5];
    assign roll_ki_eff  = reg_bank[active_mode][6];
    assign roll_kd_eff  = reg_bank[active_mode][7];
    assign roll_alpha   = reg_bank[active_mode][8];
    assign roll_u_min   = reg_bank[active_mode][9];
    assign roll_u_max   = reg_bank[active_mode][10];
    assign roll_i_min   = reg_bank[active_mode][11];
    assign roll_i_max   = reg_bank[active_mode][12];

    // Pitch Active Channel
    assign pitch_kp     = reg_bank[active_mode][13];
    assign pitch_ki_eff = reg_bank[active_mode][14];
    assign pitch_kd_eff = reg_bank[active_mode][15];
    assign pitch_alpha  = reg_bank[active_mode][16];
    assign pitch_u_min  = reg_bank[active_mode][17];
    assign pitch_u_max  = reg_bank[active_mode][18];
    assign pitch_i_min  = reg_bank[active_mode][19];
    assign pitch_i_max  = reg_bank[active_mode][20];

    // Yaw Active Channel
    assign yaw_kp       = reg_bank[active_mode][21];
    assign yaw_ki_eff   = reg_bank[active_mode][22];
    assign yaw_kd_eff   = reg_bank[active_mode][23];
    assign yaw_alpha    = reg_bank[active_mode][24];
    assign yaw_u_min    = reg_bank[active_mode][25];
    assign yaw_u_max    = reg_bank[active_mode][26];
    assign yaw_i_min    = reg_bank[active_mode][27];
    assign yaw_i_max    = reg_bank[active_mode][28];

    // Altitude Thrust Active Channel (Mode-specific parameter bank routing)
    assign thrust_kp     = reg_bank[active_mode][29];
    assign thrust_ki_eff = reg_bank[active_mode][30];
    assign thrust_kd_eff = reg_bank[active_mode][31];

    logic signed [DATA_W-1:0] raw_thrust_alpha;
    logic signed [DATA_W-1:0] raw_thrust_u_min;
    logic signed [DATA_W-1:0] raw_thrust_u_max;
    logic signed [DATA_W-1:0] raw_thrust_i_min;
    logic signed [DATA_W-1:0] raw_thrust_i_max;

    assign raw_thrust_alpha = thrust_bank[active_mode][0];
    assign raw_thrust_u_min = thrust_bank[active_mode][1];
    assign raw_thrust_u_max = thrust_bank[active_mode][2];
    assign raw_thrust_i_min = thrust_bank[active_mode][3];
    assign raw_thrust_i_max = thrust_bank[active_mode][4];

    // Safe boundary handling for invalid configurations (e.g. u_min > u_max or i_min > i_max):
    assign thrust_alpha = raw_thrust_alpha;
    assign thrust_u_max = raw_thrust_u_max;
    assign thrust_u_min = (raw_thrust_u_min <= raw_thrust_u_max) ? raw_thrust_u_min : raw_thrust_u_max;
    assign thrust_i_max = raw_thrust_i_max;
    assign thrust_i_min = (raw_thrust_i_min <= raw_thrust_i_max) ? raw_thrust_i_min : raw_thrust_i_max;

endmodule : pid_gain_bank

`endif // PID_GAIN_BANK_SV
