//==============================================================================
// Module: pid_axis_engine_shared.sv
// Description: Time-shared 4-axis PID feedback engine (Roll, Pitch, Yaw, Altitude).
//              Reuses a single 4-multiplier arithmetic datapath across 4 clock cycles,
//              achieving ~75% reduction in multiplier area compared to fully parallel.
// Architecture: Time-Shared Multiplexed Datapath (4 multipliers for all 4 axes)
// Throughput  : 4 clock cycles per sample (80 ns at 50 MHz; max 12.5 MSamples/s)
// Language    : IEEE 1800-2012 Synthesizable SystemVerilog
//==============================================================================

`ifndef PID_AXIS_ENGINE_SHARED_SV
`define PID_AXIS_ENGINE_SHARED_SV

`include "pid_pkg.sv"
`include "pid_mult.sv"
`include "pid_sat_unit.sv"

import pid_pkg::*;

module pid_axis_engine_shared #(
    parameter int DATA_W  = 16,
    parameter int FRACT_W = 12,
    parameter int ACCUM_W = 24
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     sample_valid,

    // Reseed inputs (bumpless transfer)
    input  logic                     reseed_valid,
    input  logic signed [DATA_W-1:0] reseed_i_roll,
    input  logic signed [DATA_W-1:0] reseed_i_pitch,
    input  logic signed [DATA_W-1:0] reseed_i_yaw,
    input  logic signed [DATA_W-1:0] reseed_i_thrust,

    // Setpoints (Q4.12)
    input  logic signed [DATA_W-1:0] inner_sp_roll,
    input  logic signed [DATA_W-1:0] inner_sp_pitch,
    input  logic signed [DATA_W-1:0] inner_sp_yaw,
    input  logic signed [DATA_W-1:0] inner_sp_vz,

    // Measurements (Q4.12)
    input  logic signed [DATA_W-1:0] imu_p,
    input  logic signed [DATA_W-1:0] imu_q,
    input  logic signed [DATA_W-1:0] imu_r,
    input  logic signed [DATA_W-1:0] alt_vz,

    // Channel Parameters: Roll
    input  logic signed [DATA_W-1:0] roll_kp,
    input  logic signed [DATA_W-1:0] roll_ki_eff,
    input  logic signed [DATA_W-1:0] roll_kd_eff,
    input  logic signed [DATA_W-1:0] roll_alpha,
    input  logic signed [DATA_W-1:0] roll_u_min,
    input  logic signed [DATA_W-1:0] roll_u_max,
    input  logic signed [DATA_W-1:0] roll_i_min,
    input  logic signed [DATA_W-1:0] roll_i_max,

    // Channel Parameters: Pitch
    input  logic signed [DATA_W-1:0] pitch_kp,
    input  logic signed [DATA_W-1:0] pitch_ki_eff,
    input  logic signed [DATA_W-1:0] pitch_kd_eff,
    input  logic signed [DATA_W-1:0] pitch_alpha,
    input  logic signed [DATA_W-1:0] pitch_u_min,
    input  logic signed [DATA_W-1:0] pitch_u_max,
    input  logic signed [DATA_W-1:0] pitch_i_min,
    input  logic signed [DATA_W-1:0] pitch_i_max,

    // Channel Parameters: Yaw
    input  logic signed [DATA_W-1:0] yaw_kp,
    input  logic signed [DATA_W-1:0] yaw_ki_eff,
    input  logic signed [DATA_W-1:0] yaw_kd_eff,
    input  logic signed [DATA_W-1:0] yaw_alpha,
    input  logic signed [DATA_W-1:0] yaw_u_min,
    input  logic signed [DATA_W-1:0] yaw_u_max,
    input  logic signed [DATA_W-1:0] yaw_i_min,
    input  logic signed [DATA_W-1:0] yaw_i_max,

    // Channel Parameters: Thrust
    input  logic signed [DATA_W-1:0] thrust_kp,
    input  logic signed [DATA_W-1:0] thrust_ki_eff,
    input  logic signed [DATA_W-1:0] thrust_kd_eff,
    input  logic signed [DATA_W-1:0] thrust_alpha,
    input  logic signed [DATA_W-1:0] thrust_u_min,
    input  logic signed [DATA_W-1:0] thrust_u_max,
    input  logic signed [DATA_W-1:0] thrust_i_min,
    input  logic signed [DATA_W-1:0] thrust_i_max,

    // Output Commands
    output logic                     engine_valid,
    output logic signed [DATA_W-1:0] core_u_roll,
    output logic signed [DATA_W-1:0] core_u_pitch,
    output logic signed [DATA_W-1:0] core_u_yaw,
    output logic signed [DATA_W-1:0] core_u_thrust,

    // Anti-windup clamp status flags
    output logic                     ch_r, cl_r,
    output logic                     ch_p, cl_p,
    output logic                     ch_y, cl_y,
    output logic                     ch_t, cl_t
);

    // -------------------------------------------------------------------------
    // Sequencer FSM
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        SEQ_IDLE   = 3'd0,
        SEQ_ROLL   = 3'd1,
        SEQ_PITCH  = 3'd2,
        SEQ_YAW    = 3'd3,
        SEQ_THRUST = 3'd4,
        SEQ_DONE   = 3'd5
    } seq_state_t;

    seq_state_t state_r, state_next;
    logic [1:0] active_axis; // 0=Roll, 1=Pitch, 2=Yaw, 3=Thrust

    // Stored axis internal state registers
    logic signed [DATA_W-1:0] yf_state      [0:3];
    logic signed [DATA_W-1:0] integ_state   [0:3];
    logic                     clamp_h_state [0:3];
    logic                     clamp_l_state [0:3];

    // Latched input buffers to protect against changes during 4-cycle execution
    logic                     buf_reseed_v;
    logic signed [DATA_W-1:0] buf_reseed_val [0:3];
    logic signed [DATA_W-1:0] buf_sp         [0:3];
    logic signed [DATA_W-1:0] buf_meas       [0:3];
    logic signed [DATA_W-1:0] buf_kp         [0:3];
    logic signed [DATA_W-1:0] buf_ki_eff     [0:3];
    logic signed [DATA_W-1:0] buf_kd_eff     [0:3];
    logic signed [DATA_W-1:0] buf_alpha      [0:3];
    logic signed [DATA_W-1:0] buf_u_min      [0:3];
    logic signed [DATA_W-1:0] buf_u_max      [0:3];
    logic signed [DATA_W-1:0] buf_i_min      [0:3];
    logic signed [DATA_W-1:0] buf_i_max      [0:3];

    // -------------------------------------------------------------------------
    // Multiplexers: Route selected axis operands to the shared datapath
    // -------------------------------------------------------------------------
    logic signed [DATA_W-1:0] mux_sp;
    logic signed [DATA_W-1:0] mux_meas;
    logic signed [DATA_W-1:0] mux_kp;
    logic signed [DATA_W-1:0] mux_ki_eff;
    logic signed [DATA_W-1:0] mux_kd_eff;
    logic signed [DATA_W-1:0] mux_alpha;
    logic signed [DATA_W-1:0] mux_u_min;
    logic signed [DATA_W-1:0] mux_u_max;
    logic signed [DATA_W-1:0] mux_i_min;
    logic signed [DATA_W-1:0] mux_i_max;
    logic signed [DATA_W-1:0] mux_reseed_val;
    logic signed [DATA_W-1:0] mux_yf_cur;
    logic signed [DATA_W-1:0] mux_integ_cur;
    logic                     mux_ch_cur;
    logic                     mux_cl_cur;

    always_comb begin
        // When sample_valid arrives in IDLE, sample inputs directly for Roll (cycle 0)
        if (state_r == SEQ_IDLE) begin
            active_axis    = 2'd0; // Roll
            mux_sp         = inner_sp_roll;
            mux_meas       = imu_p;
            mux_kp         = roll_kp;
            mux_ki_eff     = roll_ki_eff;
            mux_kd_eff     = roll_kd_eff;
            mux_alpha      = roll_alpha;
            mux_u_min      = roll_u_min;
            mux_u_max      = roll_u_max;
            mux_i_min      = roll_i_min;
            mux_i_max      = roll_i_max;
            mux_reseed_val = reseed_i_roll;
            mux_yf_cur     = yf_state[0];
            mux_integ_cur  = integ_state[0];
            mux_ch_cur     = clamp_h_state[0];
            mux_cl_cur     = clamp_l_state[0];
        end else begin
            active_axis    = (state_r == SEQ_ROLL)   ? 2'd0 :
                             (state_r == SEQ_PITCH)  ? 2'd1 :
                             (state_r == SEQ_YAW)    ? 2'd2 : 2'd3;
            mux_sp         = buf_sp[active_axis];
            mux_meas       = buf_meas[active_axis];
            mux_kp         = buf_kp[active_axis];
            mux_ki_eff     = buf_ki_eff[active_axis];
            mux_kd_eff     = buf_kd_eff[active_axis];
            mux_alpha      = buf_alpha[active_axis];
            mux_u_min      = buf_u_min[active_axis];
            mux_u_max      = buf_u_max[active_axis];
            mux_i_min      = buf_i_min[active_axis];
            mux_i_max      = buf_i_max[active_axis];
            mux_reseed_val = buf_reseed_val[active_axis];
            mux_yf_cur     = yf_state[active_axis];
            mux_integ_cur  = integ_state[active_axis];
            mux_ch_cur     = clamp_h_state[active_axis];
            mux_cl_cur     = clamp_l_state[active_axis];
        end
    end

    // -------------------------------------------------------------------------
    // Shared Combinational PID Datapath (Instantiated Once: 4 Multipliers)
    // -------------------------------------------------------------------------
    logic signed [DATA_W-1:0] dp_error;
    logic signed [DATA_W-1:0] dp_p_term;
    logic signed [DATA_W-1:0] dp_delta_y;
    logic signed [DATA_W-1:0] dp_delta_filt;
    logic signed [DATA_W-1:0] dp_yf_next;
    logic signed [DATA_W-1:0] dp_delta_yf;
    logic signed [DATA_W-1:0] dp_d_mult_out;
    logic signed [DATA_W-1:0] dp_d_term;

    logic signed [DATA_W-1:0] dp_delta_i;
    logic                     dp_freeze;
    logic signed [DATA_W:0]   dp_integ_sum_ext;
    logic signed [DATA_W:0]   dp_imax_ext;
    logic signed [DATA_W:0]   dp_imin_ext;
    logic signed [DATA_W-1:0] dp_integ_next;
    logic signed [DATA_W-1:0] dp_i_term;

    logic signed [DATA_W-1:0]  dp_u_sat;
    logic signed [ACCUM_W-1:0] dp_raw_sum;
    logic                      dp_clamp_h;
    logic                      dp_clamp_l;

    // Error
    assign dp_error = mux_sp - mux_meas;

    // 1. Proportional Multiplier
    pid_mult #(.DATA_W(DATA_W), .FRACT_W(FRACT_W)) u_mult_p (
        .a(mux_kp), .b(dp_error), .p(dp_p_term)
    );

    // 2. Derivative Filter Multipliers
    assign dp_delta_y = mux_meas - mux_yf_cur;

    pid_mult #(.DATA_W(DATA_W), .FRACT_W(FRACT_W)) u_mult_alpha (
        .a(mux_alpha), .b(dp_delta_y), .p(dp_delta_filt)
    );

    wire cur_reseed_v = (state_r == SEQ_IDLE) ? reseed_valid : buf_reseed_v;

    always_comb begin
        if (cur_reseed_v) begin
            dp_yf_next  = mux_meas;
            dp_delta_yf = '0;
        end else begin
            dp_yf_next  = mux_yf_cur + dp_delta_filt;
            dp_delta_yf = dp_yf_next - mux_yf_cur;
        end
    end

    pid_mult #(.DATA_W(DATA_W), .FRACT_W(FRACT_W)) u_mult_kd (
        .a(mux_kd_eff), .b(dp_delta_yf), .p(dp_d_mult_out)
    );
    assign dp_d_term = -dp_d_mult_out;

    // 3. Integrator Unit Multiplier & Anti-Windup
    pid_mult #(.DATA_W(DATA_W), .FRACT_W(FRACT_W)) u_mult_ki (
        .a(mux_ki_eff), .b(dp_error), .p(dp_delta_i)
    );

    assign dp_freeze = (mux_ch_cur && ($signed(dp_error) > 16'sd0)) ||
                       (mux_cl_cur && ($signed(dp_error) < 16'sd0));

    assign dp_integ_sum_ext = $signed({mux_integ_cur[DATA_W-1], mux_integ_cur}) + $signed({dp_delta_i[DATA_W-1], dp_delta_i});
    assign dp_imax_ext      = $signed({mux_i_max[DATA_W-1], mux_i_max});
    assign dp_imin_ext      = $signed({mux_i_min[DATA_W-1], mux_i_min});

    always_comb begin
        if (cur_reseed_v) begin
            if (mux_reseed_val > mux_i_max)
                dp_integ_next = mux_i_max;
            else if (mux_reseed_val < mux_i_min)
                dp_integ_next = mux_i_min;
            else
                dp_integ_next = mux_reseed_val;
        end else if (dp_freeze) begin
            dp_integ_next = mux_integ_cur;
        end else begin
            if (dp_integ_sum_ext > dp_imax_ext)
                dp_integ_next = mux_i_max;
            else if (dp_integ_sum_ext < dp_imin_ext)
                dp_integ_next = mux_i_min;
            else
                dp_integ_next = dp_integ_sum_ext[DATA_W-1:0];
        end
    end
    assign dp_i_term = dp_integ_next;

    // 4. Saturation Unit
    pid_sat_unit #(
        .DATA_W(DATA_W),
        .ACCUM_W(ACCUM_W)
    ) u_sat_unit (
        .clk(clk),
        .rst_n(rst_n),
        .sample_valid(1'b1),
        .p_term(dp_p_term),
        .i_term(dp_i_term),
        .d_term(dp_d_term),
        .u_min(mux_u_min),
        .u_max(mux_u_max),
        .u_sat(dp_u_sat),
        .raw_sum(dp_raw_sum),
        .clamp_high(dp_clamp_h),
        .clamp_low(dp_clamp_l)
    );

    // -------------------------------------------------------------------------
    // Sequential State Update & Output Staging
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state_r        <= SEQ_IDLE;
            engine_valid   <= 1'b0;
            core_u_roll    <= '0;
            core_u_pitch   <= '0;
            core_u_yaw     <= '0;
            core_u_thrust  <= '0;
            buf_reseed_v   <= 1'b0;

            for (int i = 0; i < 4; i++) begin
                yf_state[i]      <= FIX_ZERO;
                integ_state[i]   <= FIX_ZERO;
                clamp_h_state[i] <= 1'b0;
                clamp_l_state[i] <= 1'b0;
                buf_reseed_val[i]<= FIX_ZERO;
                buf_sp[i]        <= FIX_ZERO;
                buf_meas[i]      <= FIX_ZERO;
                buf_kp[i]        <= FIX_ZERO;
                buf_ki_eff[i]    <= FIX_ZERO;
                buf_kd_eff[i]    <= FIX_ZERO;
                buf_alpha[i]     <= FIX_ZERO;
                buf_u_min[i]     <= FIX_ZERO;
                buf_u_max[i]     <= FIX_ZERO;
                buf_i_min[i]     <= FIX_ZERO;
                buf_i_max[i]     <= FIX_ZERO;
            end
        end else begin
            case (state_r)
                SEQ_IDLE: begin
                    engine_valid <= 1'b0;
                    if (sample_valid) begin
                        // Latch input parameters for Pitch, Yaw, Thrust
                        buf_reseed_v       <= reseed_valid;
                        buf_reseed_val[0]  <= reseed_i_roll;
                        buf_reseed_val[1]  <= reseed_i_pitch;
                        buf_reseed_val[2]  <= reseed_i_yaw;
                        buf_reseed_val[3]  <= reseed_i_thrust;

                        buf_sp[0]          <= inner_sp_roll;
                        buf_sp[1]          <= inner_sp_pitch;
                        buf_sp[2]          <= inner_sp_yaw;
                        buf_sp[3]          <= inner_sp_vz;

                        buf_meas[0]        <= imu_p;
                        buf_meas[1]        <= imu_q;
                        buf_meas[2]        <= imu_r;
                        buf_meas[3]        <= alt_vz;

                        buf_kp[0]          <= roll_kp;
                        buf_kp[1]          <= pitch_kp;
                        buf_kp[2]          <= yaw_kp;
                        buf_kp[3]          <= thrust_kp;

                        buf_ki_eff[0]      <= roll_ki_eff;
                        buf_ki_eff[1]      <= pitch_ki_eff;
                        buf_ki_eff[2]      <= yaw_ki_eff;
                        buf_ki_eff[3]      <= thrust_ki_eff;

                        buf_kd_eff[0]      <= roll_kd_eff;
                        buf_kd_eff[1]      <= pitch_kd_eff;
                        buf_kd_eff[2]      <= yaw_kd_eff;
                        buf_kd_eff[3]      <= thrust_kd_eff;

                        buf_alpha[0]       <= roll_alpha;
                        buf_alpha[1]       <= pitch_alpha;
                        buf_alpha[2]       <= yaw_alpha;
                        buf_alpha[3]       <= thrust_alpha;

                        buf_u_min[0]       <= roll_u_min;
                        buf_u_min[1]       <= pitch_u_min;
                        buf_u_min[2]       <= yaw_u_min;
                        buf_u_min[3]       <= thrust_u_min;

                        buf_u_max[0]       <= roll_u_max;
                        buf_u_max[1]       <= pitch_u_max;
                        buf_u_max[2]       <= yaw_u_max;
                        buf_u_max[3]       <= thrust_u_max;

                        buf_i_min[0]       <= roll_i_min;
                        buf_i_min[1]       <= pitch_i_min;
                        buf_i_min[2]       <= yaw_i_min;
                        buf_i_min[3]       <= thrust_i_min;

                        buf_i_max[0]       <= roll_i_max;
                        buf_i_max[1]       <= pitch_i_max;
                        buf_i_max[2]       <= yaw_i_max;
                        buf_i_max[3]       <= thrust_i_max;

                        // Cycle 0 evaluated Roll combinational result -> register into Roll outputs
                        core_u_roll        <= dp_u_sat;
                        yf_state[0]        <= dp_yf_next;
                        integ_state[0]     <= dp_integ_next;
                        clamp_h_state[0]   <= dp_clamp_h;
                        clamp_l_state[0]   <= dp_clamp_l;

                        state_r            <= SEQ_PITCH;
                    end
                end

                SEQ_PITCH: begin
                    // Cycle 1 evaluated Pitch -> register into Pitch outputs
                    core_u_pitch       <= dp_u_sat;
                    yf_state[1]        <= dp_yf_next;
                    integ_state[1]     <= dp_integ_next;
                    clamp_h_state[1]   <= dp_clamp_h;
                    clamp_l_state[1]   <= dp_clamp_l;

                    state_r            <= SEQ_YAW;
                end

                SEQ_YAW: begin
                    // Cycle 2 evaluated Yaw -> register into Yaw outputs
                    core_u_yaw         <= dp_u_sat;
                    yf_state[2]        <= dp_yf_next;
                    integ_state[2]     <= dp_integ_next;
                    clamp_h_state[2]   <= dp_clamp_h;
                    clamp_l_state[2]   <= dp_clamp_l;

                    state_r            <= SEQ_THRUST;
                end

                SEQ_THRUST: begin
                    // Cycle 3 evaluated Thrust -> register into Thrust outputs
                    core_u_thrust      <= dp_u_sat;
                    yf_state[3]        <= dp_yf_next;
                    integ_state[3]     <= dp_integ_next;
                    clamp_h_state[3]   <= dp_clamp_h;
                    clamp_l_state[3]   <= dp_clamp_l;

                    // Strobe done on cycle 4
                    engine_valid       <= 1'b1;
                    state_r            <= SEQ_DONE;
                end

                SEQ_DONE: begin
                    engine_valid       <= 1'b0;
                    state_r            <= SEQ_IDLE;
                end

                default: state_r       <= SEQ_IDLE;
            endcase
        end
    end

    // Clamp flag assignments
    assign ch_r = clamp_h_state[0];
    assign cl_r = clamp_l_state[0];
    assign ch_p = clamp_h_state[1];
    assign cl_p = clamp_l_state[1];
    assign ch_y = clamp_h_state[2];
    assign cl_y = clamp_l_state[2];
    assign ch_t = clamp_h_state[3];
    assign cl_t = clamp_l_state[3];

endmodule : pid_axis_engine_shared

`endif // PID_AXIS_ENGINE_SHARED_SV
