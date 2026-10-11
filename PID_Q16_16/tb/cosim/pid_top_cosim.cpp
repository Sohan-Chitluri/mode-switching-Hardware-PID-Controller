//==============================================================================
// File: tb/cosim/pid_top_cosim.cpp
// Description: C API wrapper around Verilated Vpid_top for Python ctypes co-simulation.
//              Provides deterministic clock advancing, reset sequencing, port driving,
//              and output reading without inter-process IPC overhead.
//==============================================================================

#include "Vpid_top.h"
#include "verilated.h"
#include <cstdint>
#include <cstdlib>
#include <cstring>

extern "C" {

struct CosimState {
    Vpid_top* top;
    VerilatedContext* context;
    uint64_t total_cycles;
    int32_t output_bias_pitch; // Test hook for injecting intentional torque perturbations
};

struct CosimInputs {
    int32_t sample_valid;
    int32_t imu_p;
    int32_t imu_q;
    int32_t imu_r;
    int32_t imu_phi;
    int32_t imu_theta;
    int32_t alt_z;
    int32_t alt_vz;
    int32_t alt_valid;
    int32_t mode_cmd;
    int32_t rearm_cmd;
    int32_t failsafe_ext_n;
    int32_t rc_sp_roll;
    int32_t rc_sp_pitch;
    int32_t rc_sp_yaw;
    int32_t rc_sp_thrust;
    int32_t cfg_we;
    int32_t cfg_addr;
    int32_t cfg_wdata;
    int32_t cfg_failsafe_hold_last;
};

struct CosimOutputs {
    int32_t output_valid;
    int32_t u_roll;
    int32_t u_pitch;
    int32_t u_yaw;
    int32_t u_thrust;
    int32_t active_mode;
    int32_t fault_code;
    int32_t watchdog_alarm;
    int32_t saturated_flag;
    int32_t cfg_rdata;
};

CosimState* cosim_init() {
    CosimState* s = new CosimState();
    s->context = new VerilatedContext();
    s->top = new Vpid_top(s->context);
    s->total_cycles = 0;
    s->output_bias_pitch = 0;

    s->top->clk = 0;
    s->top->rst_n = 0;
    s->top->sample_valid = 0;
    s->top->alt_valid = 0;
    s->top->mode_cmd = 0;
    s->top->rearm_cmd = 0;
    s->top->failsafe_ext_n = 1;
    s->top->rc_sp_roll = 0;
    s->top->rc_sp_pitch = 0;
    s->top->rc_sp_yaw = 0;
    s->top->rc_sp_thrust = 0;
    s->top->imu_p = 0;
    s->top->imu_q = 0;
    s->top->imu_r = 0;
    s->top->imu_phi = 0;
    s->top->imu_theta = 0;
    s->top->alt_z = 0;
    s->top->alt_vz = 0;
    s->top->cfg_we = 0;
    s->top->cfg_addr = 0;
    s->top->cfg_wdata = 0;
    s->top->cfg_failsafe_hold_last = 0;

    s->top->eval();
    return s;
}

void cosim_reset(CosimState* s, int cycles) {
    if (!s || !s->top) return;

    s->top->rst_n = 0;
    s->top->sample_valid = 0;
    for (int i = 0; i < cycles; ++i) {
        s->top->clk = 0; s->top->eval();
        s->top->clk = 1; s->top->eval();
        s->total_cycles++;
    }

    // Release reset
    s->top->rst_n = 1;
    s->top->clk = 0; s->top->eval();
    s->top->clk = 1; s->top->eval();
    s->total_cycles++;
    s->top->clk = 0; s->top->eval();
}

void cosim_step(
    CosimState* s,
    const CosimInputs* in,
    CosimOutputs* out,
    int clock_cycles
) {
    if (!s || !s->top || !in || !out) return;

    if (clock_cycles < 1) clock_cycles = 1;

    // Apply inputs on clock low
    s->top->sample_valid   = in->sample_valid;
    s->top->imu_p          = in->imu_p;
    s->top->imu_q          = in->imu_q;
    s->top->imu_r          = in->imu_r;
    s->top->imu_phi        = in->imu_phi;
    s->top->imu_theta      = in->imu_theta;
    s->top->alt_z          = in->alt_z;
    s->top->alt_vz         = in->alt_vz;
    s->top->alt_valid      = in->alt_valid;
    s->top->mode_cmd       = in->mode_cmd;
    s->top->rearm_cmd      = in->rearm_cmd;
    s->top->failsafe_ext_n = in->failsafe_ext_n;
    s->top->rc_sp_roll     = in->rc_sp_roll;
    s->top->rc_sp_pitch    = in->rc_sp_pitch;
    s->top->rc_sp_yaw      = in->rc_sp_yaw;
    s->top->rc_sp_thrust   = in->rc_sp_thrust;
    s->top->cfg_we         = in->cfg_we;
    s->top->cfg_addr       = in->cfg_addr;
    s->top->cfg_wdata      = in->cfg_wdata;
    s->top->cfg_failsafe_hold_last = in->cfg_failsafe_hold_last;

    // Cycle 1: Sample strobe evaluated and registered on posedge
    s->top->clk = 0; s->top->eval();
    s->top->clk = 1; s->top->eval();
    s->total_cycles++;

    // Latch registered outputs from DUT
    out->output_valid   = s->top->output_valid;
    out->u_roll         = (int32_t)s->top->u_roll;
    out->u_pitch        = (int32_t)s->top->u_pitch + s->output_bias_pitch;
    out->u_yaw          = (int32_t)s->top->u_yaw;
    out->u_thrust       = (int32_t)s->top->u_thrust;
    out->active_mode    = s->top->active_mode;
    out->fault_code     = s->top->fault_code;
    out->watchdog_alarm = s->top->watchdog_alarm;
    out->saturated_flag = s->top->saturated_flag;
    out->cfg_rdata      = (int32_t)s->top->cfg_rdata;

    // If multi-cycle clock step requested, deassert sample_valid and advance clock
    if (clock_cycles > 1) {
        s->top->sample_valid = 0;
        s->top->cfg_we = 0;
        for (int i = 1; i < clock_cycles; ++i) {
            s->top->clk = 0; s->top->eval();
            s->top->clk = 1; s->top->eval();
            s->total_cycles++;
        }
    }

    s->top->clk = 0; s->top->eval();
}

void cosim_set_bias_pitch(CosimState* s, int32_t bias_q16) {
    if (s) {
        s->output_bias_pitch = bias_q16;
    }
}

uint64_t cosim_get_total_cycles(CosimState* s) {
    return s ? s->total_cycles : 0;
}

void cosim_free(CosimState* s) {
    if (s) {
        if (s->top) {
            s->top->final();
            delete s->top;
        }
        if (s->context) {
            delete s->context;
        }
        delete s;
    }
}

}
