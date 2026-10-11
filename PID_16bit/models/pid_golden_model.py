#!/usr/bin/env python3
"""
models/pid_golden_model.py
Bit-accurate fixed-point reference model for hardware PID flight controller (16-bit Q4.12).
Matches synthesizable SystemVerilog datapath:
- Q4.12 signed two's complement arithmetic (DATA_W=16, FRACT_W=12)
- 32-bit multiplier full product with shift-by-12 truncation
- 24-bit guarded accumulation (ACCUM_W=24, 8 guard bits prevent intermediate overflow)
- Measurement-based derivative with first-order IIR LPF
- Conditional integration anti-windup clamping
- Bumpless mode-transition integrator back-calculation
- Cycle-accurate pipeline matching
"""

import sys
import math
import json

# ==============================================================================
# FIXED-POINT ARITHMETIC PRIMITIVES (Q4.12)
# ==============================================================================

WIDTH = 16
FRACT_W = 12
ACCUM_W = 24
SCALE = 1 << FRACT_W  # 4096

MAX_INT16 = (1 << 15) - 1   #  32767
MIN_INT16 = -(1 << 15)      # -32768

MASK16 = (1 << 16) - 1
MASK24 = (1 << 24) - 1
MASK32 = (1 << 16) - 1  # For 16-bit compatibility in hex formatters


def to_signed16(val: int) -> int:
    """Clamp/wrap to signed 16-bit integer."""
    val = val & MASK16
    if val & (1 << 15):
        return val - (1 << 16)
    return val


def to_signed24(val: int) -> int:
    """Sign-extend / wrap to signed 24-bit accumulator."""
    val = val & MASK24
    if val & (1 << 23):
        return val - (1 << 24)
    return val


def float_to_q12(val: float) -> int:
    """Convert floating-point number to Q4.12 signed 16-bit integer."""
    scaled = int(round(val * SCALE))
    if scaled > MAX_INT16:
        return MAX_INT16
    if scaled < MIN_INT16:
        return MIN_INT16
    return scaled


def q12_to_float(val: int) -> float:
    """Convert Q4.12 signed integer to float."""
    return val / float(SCALE)


def clamp(val: int, low: int, high: int) -> int:
    """Saturate integer val to [low, high]."""
    if val > high:
        return high
    if val < low:
        return low
    return val


def mult_q12(a: int, b: int) -> int:
    """
    Synthesizable signed 16x16 -> 32-bit product, arithmetic shifted right by 12.
    Matches RTL: signed'(a) * signed'(b) >>> 12.
    """
    a_s = to_signed16(a)
    b_s = to_signed16(b)
    prod32 = a_s * b_s
    res = prod32 >> FRACT_W
    if res > MAX_INT16:
        return MAX_INT16
    if res < MIN_INT16:
        return MIN_INT16
    return res


# Aliases for backwards compatibility with test harnesses
float_to_q16 = float_to_q12
q16_to_float = q12_to_float
mult_q16 = mult_q12
to_signed32 = to_signed16
to_signed48 = to_signed24


# ==============================================================================
# SINGLE-CHANNEL PID CORE GOLDEN MODEL (pid_axis_core)
# ==============================================================================

class PidAxisCoreGolden:
    """
    Bit-accurate model of a single-axis PID channel in 16-bit Q4.12.
    Reused across Roll, Pitch, Yaw, and Altitude thrust channels.
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        alpha: float = 0.5,
        ts: float = 0.00025,  # 4 kHz sample rate (250 microseconds)
        u_min: float = -1.0,
        u_max: float = +1.0,
        i_min: float = -0.5,
        i_max: float = +0.5,
        kp_q12: int = None,
        ki_eff_q12: int = None,
        kd_eff_q12: int = None,
        alpha_q12: int = None,
        u_min_q12: int = None,
        u_max_q12: int = None,
        i_min_q12: int = None,
        i_max_q12: int = None,
    ):
        self.ts = ts
        self.kp_q16 = to_signed16(kp_q12) if kp_q12 is not None else float_to_q12(kp)
        self.ki_eff_q16 = to_signed16(ki_eff_q12) if ki_eff_q12 is not None else float_to_q12(ki * ts)
        self.kd_eff_q16 = to_signed16(kd_eff_q12) if kd_eff_q12 is not None else (float_to_q12(kd / ts) if ts > 0 else 0)
        self.alpha_q16 = to_signed16(alpha_q12) if alpha_q12 is not None else float_to_q12(alpha)

        self.u_min_q16 = to_signed16(u_min_q12) if u_min_q12 is not None else float_to_q12(u_min)
        self.u_max_q16 = to_signed16(u_max_q12) if u_max_q12 is not None else float_to_q12(u_max)
        self.i_min_q16 = to_signed16(i_min_q12) if i_min_q12 is not None else float_to_q12(i_min)
        self.i_max_q16 = to_signed16(i_max_q12) if i_max_q12 is not None else float_to_q12(i_max)

        # Internal State Registers
        self.reset()

    def reset(self):
        """Clear all internal registers to deterministic zero state."""
        self.integ_reg = 0         # 16-bit signed accumulator
        self.deriv_filter_reg = 0  # 16-bit signed filtered measurement
        self.clamp_high_reg = False
        self.clamp_low_reg = False
        self.u_prev = 0

    def set_gains(self, kp: float, ki: float, kd: float, alpha: float):
        """Update active gains (e.g., on mode change)."""
        self.kp_q16 = float_to_q12(kp)
        self.ki_eff_q16 = float_to_q12(ki * self.ts)
        self.kd_eff_q16 = float_to_q12(kd / self.ts) if self.ts > 0 else 0
        self.alpha_q16 = float_to_q12(alpha)

    def set_gains_q16(self, kp_q16: int, ki_eff_q16: int, kd_eff_q16: int, alpha_q16: int):
        """Directly set fixed-point gains."""
        self.kp_q16 = to_signed16(kp_q16)
        self.ki_eff_q16 = to_signed16(ki_eff_q16)
        self.kd_eff_q16 = to_signed16(kd_eff_q16)
        self.alpha_q16 = to_signed16(alpha_q16)

    set_gains_q12 = set_gains_q16

    def set_limits_q16(self, u_min_q16: int, u_max_q16: int, i_min_q16: int, i_max_q16: int):
        """Set saturation limits with safety clamping against invalid inversion."""
        self.u_max_q16 = to_signed16(u_max_q16)
        self.u_min_q16 = to_signed16(u_min_q16) if to_signed16(u_min_q16) <= self.u_max_q16 else self.u_max_q16
        self.i_max_q16 = to_signed16(i_max_q16)
        self.i_min_q16 = to_signed16(i_min_q16) if to_signed16(i_min_q16) <= self.i_max_q16 else self.i_max_q16

    set_limits_q12 = set_limits_q16

    def step(
        self,
        sp_q16: int,
        meas_q16: int,
        reseed_valid: bool = False,
        reseed_val_q16: int = 0,
    ) -> dict:
        """
        Execute one cycle of the PID channel datapath.
        Returns dictionary of all intermediate and output signals.
        """
        sp_q16 = to_signed16(sp_q16)
        meas_q16 = to_signed16(meas_q16)
        reseed_val_q16 = to_signed16(reseed_val_q16)

        # ----------------------------------------------------------------------
        # Stage 1: Derivative Filter (Measurement-Based IIR LPF)
        # ----------------------------------------------------------------------
        if reseed_valid:
            # Bumpless transfer: pre-seed filter with current measurement -> delta = 0
            self.deriv_filter_reg = meas_q16
            delta_yf = 0
        else:
            delta_y = to_signed16(meas_q16 - self.deriv_filter_reg)
            delta_filt = mult_q12(self.alpha_q16, delta_y)
            next_filter = to_signed16(self.deriv_filter_reg + delta_filt)
            delta_yf = to_signed16(next_filter - self.deriv_filter_reg)
            self.deriv_filter_reg = next_filter

        # D-term = -Kd_eff * delta_yf
        d_term = to_signed16(-mult_q12(self.kd_eff_q16, delta_yf))

        # ----------------------------------------------------------------------
        # Stage 2: Proportional Term
        # ----------------------------------------------------------------------
        error = to_signed16(sp_q16 - meas_q16)
        p_term = mult_q12(self.kp_q16, error)

        # ----------------------------------------------------------------------
        # Stage 3: Integrator Unit with Anti-Windup Clamping
        # ----------------------------------------------------------------------
        if reseed_valid:
            # Instantaneous bumpless back-calculation re-seeding
            self.integ_reg = clamp(reseed_val_q16, self.i_min_q16, self.i_max_q16)
        else:
            # Conditional integration clamping:
            # Freeze if saturated high and error > 0, or saturated low and error < 0
            freeze = (self.clamp_high_reg and (error > 0)) or (self.clamp_low_reg and (error < 0))
            if not freeze:
                delta_i = mult_q12(self.ki_eff_q16, error)
                next_i = to_signed16(self.integ_reg + delta_i)
                self.integ_reg = clamp(next_i, self.i_min_q16, self.i_max_q16)

        i_term = self.integ_reg

        # ----------------------------------------------------------------------
        # Stage 4: Summation in 24-bit Guarded Accumulator & Saturation
        # ----------------------------------------------------------------------
        raw_sum = to_signed24(p_term + i_term + d_term)

        # Evaluate saturation bounds
        clamp_high = raw_sum >= self.u_max_q16
        clamp_low = raw_sum <= self.u_min_q16
        u_sat = clamp(raw_sum, self.u_min_q16, self.u_max_q16)

        # Register clamp flags for next cycle's anti-windup
        self.clamp_high_reg = clamp_high
        self.clamp_low_reg = clamp_low
        self.u_prev = u_sat

        return {
            "error": error,
            "p_term": p_term,
            "i_term": i_term,
            "d_term": d_term,
            "raw_sum": raw_sum,
            "u_sat": u_sat,
            "clamp_high": clamp_high,
            "clamp_low": clamp_low,
        }


# ==============================================================================
# TOP-LEVEL CONTROLLER GOLDEN MODEL (pid_top)
# ==============================================================================

class PidTopGolden:
    """
    Full top-level golden model coordinating:
    - 4 parallel channels: Roll, Pitch, Yaw, Altitude
    - Outer attitude (Roll/Pitch angle -> desired body rate)
    - Outer altitude (Position -> climb rate)
    - Small-angle thrust tilt-compensation
    - Autonomous Mode Management FSM
    - Bumpless mode transitions
    - Watchdog sample freshness timer & deterministic Failsafe override
    """

    MODE_ACRO = 0
    MODE_STABILIZE = 1
    MODE_ALTHOLD = 2
    MODE_FAILSAFE = 3

    def __init__(self, timeout_cycles: int = 50000, ts: float = 0.00025, cfg_failsafe_hold_last: bool = False):
        self.timeout_cycles = timeout_cycles
        self.ts = ts
        self.cfg_failsafe_hold_last = cfg_failsafe_hold_last

        # Core PID Channels (Parallel Hardware)
        self.core_roll = PidAxisCoreGolden(kp=0.25, ki=0.40, kd=0.005, alpha=0.3, ts=ts, u_min=-1.0, u_max=+1.0)
        self.core_pitch = PidAxisCoreGolden(kp=0.25, ki=0.40, kd=0.005, alpha=0.3, ts=ts, u_min=-1.0, u_max=+1.0)
        self.core_yaw = PidAxisCoreGolden(kp=0.25, ki=0.15, kd=0.000, alpha=0.3, ts=ts, u_min=-1.0, u_max=+1.0)
        self.core_thrust = PidAxisCoreGolden(kp=0.40, ki=0.30, kd=0.010, alpha=0.2, ts=ts, u_min=0.0, u_max=+1.0)

        # Outer Attitude & Altitude Gains
        self.kp_att_roll = 0x4800   # 4.5
        self.kp_att_pitch = 0x4800
        self.kp_alt_pos = 0x1333    # 1.2
        self.hover_thrust_q16 = 0x0800 # 50% baseline hover throttle
        self.failsafe_thrust_q16 = 0x0000 # Motor disarm / 0 throttle

        # 2-Stage CDC Synchronizer for failsafe_ext_n (active-low, default True = inactive)
        self.fs_stage1 = True
        self.fs_stage2 = True

        # State Registers
        self.active_mode = self.MODE_ACRO
        self.watchdog_counter = 0
        self.watchdog_alarm = False
        self.sticky_fault = 0

        self.u_roll_prev = 0
        self.u_pitch_prev = 0
        self.u_yaw_prev = 0
        self.u_thrust_prev = 0

        self.last_valid_roll = 0
        self.last_valid_pitch = 0
        self.last_valid_yaw = 0
        self.last_valid_thrust = 0
        self.has_valid_sample = False

        self._apply_mode_gains(self.active_mode)

    def _apply_mode_gains(self, mode: int):
        """Dynamically apply gain bank matrix matching pid_gain_bank.sv in Q4.12."""
        if mode == self.MODE_ACRO:
            self.kp_att_roll = 0x4800
            self.kp_att_pitch = 0x4800
            self.kp_alt_pos = 0x1333
            self.hover_thrust_q16 = 0x0800
            self.failsafe_thrust_q16 = 0x0000
            # Acro mode agile rate gains
            self.core_roll.set_gains_q16(kp_q16=0x0400, ki_eff_q16=0x0004, kd_eff_q16=0x2000, alpha_q16=0x04CD)
            self.core_pitch.set_gains_q16(kp_q16=0x0400, ki_eff_q16=0x0004, kd_eff_q16=0x2000, alpha_q16=0x04CD)
            self.core_yaw.set_gains_q16(kp_q16=0x0400, ki_eff_q16=0x0001, kd_eff_q16=0x0000, alpha_q16=0x04CD)
            self.core_thrust.set_gains_q16(kp_q16=0x0666, ki_eff_q16=0x0003, kd_eff_q16=0x3000, alpha_q16=0x0333)
            self.core_thrust.set_limits_q16(u_min_q16=0x0000, u_max_q16=0x1000, i_min_q16=-0x0400, i_max_q16=0x0400)
        elif mode == self.MODE_STABILIZE:
            self.kp_att_roll = 0x4000
            self.kp_att_pitch = 0x4000
            self.kp_alt_pos = 0x1333
            self.hover_thrust_q16 = 0x0800
            self.failsafe_thrust_q16 = 0x0000
            self.core_roll.set_gains_q16(kp_q16=0x2000, ki_eff_q16=0x001A, kd_eff_q16=0x4000, alpha_q16=0x04CD)
            self.core_pitch.set_gains_q16(kp_q16=0x2000, ki_eff_q16=0x001A, kd_eff_q16=0x4000, alpha_q16=0x04CD)
            self.core_yaw.set_gains_q16(kp_q16=0x0400, ki_eff_q16=0x0001, kd_eff_q16=0x0000, alpha_q16=0x04CD)
            self.core_thrust.set_gains_q16(kp_q16=0x0666, ki_eff_q16=0x0003, kd_eff_q16=0x3000, alpha_q16=0x0333)
            self.core_thrust.set_limits_q16(u_min_q16=0x0000, u_max_q16=0x1000, i_min_q16=-0x0400, i_max_q16=0x0400)
        elif mode == self.MODE_ALTHOLD:
            self.kp_att_roll = 0x4000
            self.kp_att_pitch = 0x4000
            self.kp_alt_pos = 0x1333
            self.hover_thrust_q16 = 0x0800
            self.failsafe_thrust_q16 = 0x0000
            self.core_roll.set_gains_q16(kp_q16=0x2000, ki_eff_q16=0x001A, kd_eff_q16=0x4000, alpha_q16=0x04CD)
            self.core_pitch.set_gains_q16(kp_q16=0x2000, ki_eff_q16=0x001A, kd_eff_q16=0x4000, alpha_q16=0x04CD)
            self.core_yaw.set_gains_q16(kp_q16=0x0400, ki_eff_q16=0x0001, kd_eff_q16=0x0000, alpha_q16=0x04CD)
            self.core_thrust.set_gains_q16(kp_q16=0x0666, ki_eff_q16=0x0003, kd_eff_q16=0x3000, alpha_q16=0x0333)
            # AltHold delta limits: [-0.5, +0.5] allows climb and descent control
            self.core_thrust.set_limits_q16(u_min_q16=-0x0800, u_max_q16=0x0800, i_min_q16=-0x0400, i_max_q16=0x0400)
        else: # MODE_FAILSAFE
            self.kp_att_roll = 0
            self.kp_att_pitch = 0
            self.kp_alt_pos = 0
            self.hover_thrust_q16 = 0
            self.failsafe_thrust_q16 = 0
            self.core_roll.set_gains_q16(kp_q16=0, ki_eff_q16=0, kd_eff_q16=0, alpha_q16=0x04CD)
            self.core_pitch.set_gains_q16(kp_q16=0, ki_eff_q16=0, kd_eff_q16=0, alpha_q16=0x04CD)
            self.core_yaw.set_gains_q16(kp_q16=0, ki_eff_q16=0, kd_eff_q16=0, alpha_q16=0x04CD)
            self.core_thrust.set_gains_q16(kp_q16=0, ki_eff_q16=0, kd_eff_q16=0, alpha_q16=0x0333)
            self.core_thrust.set_limits_q16(u_min_q16=0x0000, u_max_q16=0x1000, i_min_q16=-0x0400, i_max_q16=0x0400)

    def reset(self):
        """System reset."""
        self.core_roll.reset()
        self.core_pitch.reset()
        self.core_yaw.reset()
        self.core_thrust.reset()
        self.active_mode = self.MODE_ACRO
        self.watchdog_counter = 0
        self.watchdog_alarm = False
        self.sticky_fault = 0
        self.fs_stage1 = True
        self.fs_stage2 = True
        self.u_roll_prev = 0
        self.u_pitch_prev = 0
        self.u_yaw_prev = 0
        self.u_thrust_prev = 0
        self.last_valid_roll = 0
        self.last_valid_pitch = 0
        self.last_valid_yaw = 0
        self.last_valid_thrust = 0
        self.has_valid_sample = False
        self._apply_mode_gains(self.active_mode)

    def step(
        self,
        sample_valid: bool,
        imu_p: int,       # Roll rate (Q4.12 rad/s)
        imu_q: int,       # Pitch rate (Q4.12 rad/s)
        imu_r: int,       # Yaw rate (Q4.12 rad/s)
        imu_phi: int,     # Roll angle (Q4.12 rad)
        imu_theta: int,   # Pitch angle (Q4.12 rad)
        alt_z: int,       # Altitude (Q4.12 m)
        alt_vz: int,      # Vertical climb rate (Q4.12 m/s)
        alt_valid: bool,
        mode_cmd: int,
        failsafe_ext_n: bool,
        sp_roll: int,
        sp_pitch: int,
        sp_yaw: int,
        sp_thrust: int,
        rearm_cmd: bool = False,
    ) -> dict:
        """
        Execute one clock/sample cycle of the integrated controller.
        """
        # 2-Stage CDC Synchronizer for asynchronous external kill switch
        fs_sync_n = self.fs_stage2
        self.fs_stage2 = self.fs_stage1
        self.fs_stage1 = failsafe_ext_n

        # Watchdog logic
        if sample_valid:
            self.watchdog_counter = 0
        else:
            self.watchdog_counter += 1

        if self.watchdog_counter >= self.timeout_cycles:
            self.watchdog_alarm = True
            self.sticky_fault |= 0x1  # Bit 0: Watchdog timeout

        if not fs_sync_n:
            self.sticky_fault |= 0x2  # Bit 1: External failsafe trigger

        # Clear sticky faults on valid pilot re-arm handshake with valid sensor samples
        if rearm_cmd and sample_valid and (not self.watchdog_alarm) and fs_sync_n:
            self.sticky_fault = 0

        # Mode Transition Logic
        target_mode = mode_cmd
        if self.sticky_fault != 0:
            target_mode = self.MODE_FAILSAFE
        elif not sample_valid:
            target_mode = self.active_mode
        else:
            if self.active_mode == self.MODE_FAILSAFE:
                if rearm_cmd and (self.sticky_fault == 0):
                    target_mode = mode_cmd  # Recovered
                else:
                    target_mode = self.MODE_FAILSAFE  # Sticky lock

            # If in AltHold and alt_valid drops, degrade gracefully to Stabilize
            if (target_mode == self.MODE_ALTHOLD) and (not alt_valid):
                target_mode = self.MODE_STABILIZE

        is_transition = (target_mode != self.active_mode)
        prev_mode = self.active_mode
        self.active_mode = target_mode
        self._apply_mode_gains(self.active_mode)

        # Failsafe Output Override logic with priority hierarchy:
        # Priority 1: Critical fault (Kill / Watchdog) -> Immediate safe disarm
        # Priority 2: Failsafe mode with hold-last enabled and initialized -> Hold last valid
        # Priority 3: Failsafe mode with hold-last disabled or uninitialized -> Configured safe output
        if self.active_mode == self.MODE_FAILSAFE:
            is_critical_fault = (not fs_sync_n) or self.watchdog_alarm or bool(self.sticky_fault & 0x3)
            if is_critical_fault or (not self.cfg_failsafe_hold_last) or (not self.has_valid_sample):
                u_r = 0
                u_p = 0
                u_y = 0
                u_t = self.failsafe_thrust_q16
            else:
                u_r = self.last_valid_roll
                u_p = self.last_valid_pitch
                u_y = self.last_valid_yaw
                u_t = self.last_valid_thrust
            self.u_roll_prev = 0
            self.u_pitch_prev = 0
            self.u_yaw_prev = 0
            self.u_thrust_prev = 0
            return {
                "output_valid": True,
                "active_mode": self.active_mode,
                "u_roll": u_r,
                "u_pitch": u_p,
                "u_yaw": u_y,
                "u_thrust": u_t,
                "watchdog_alarm": self.watchdog_alarm,
                "fault_code": self.sticky_fault,
            }

        # If not sample_valid in normal modes, pipeline holds previous outputs
        if not sample_valid:
            return {
                "output_valid": False,
                "active_mode": self.active_mode,
                "u_roll": self.u_roll_prev,
                "u_pitch": self.u_pitch_prev,
                "u_yaw": self.u_yaw_prev,
                "u_thrust": self.u_thrust_prev,
                "watchdog_alarm": self.watchdog_alarm,
                "fault_code": self.sticky_fault,
            }

        # ----------------------------------------------------------------------
        # Outer Loop: Attitude and Altitude Position
        # ----------------------------------------------------------------------
        thrust_meas = 0
        inner_sp_thrust = 0
        if self.active_mode == self.MODE_ACRO:
            inner_sp_roll = sp_roll
            inner_sp_pitch = sp_pitch
            inner_sp_yaw = sp_yaw
            base_thrust = sp_thrust
        elif self.active_mode in (self.MODE_STABILIZE, self.MODE_ALTHOLD):
            err_phi = to_signed16(sp_roll - imu_phi)
            err_theta = to_signed16(sp_pitch - imu_theta)
            inner_sp_roll = mult_q12(self.kp_att_roll, err_phi)
            inner_sp_pitch = mult_q12(self.kp_att_pitch, err_theta)
            inner_sp_yaw = sp_yaw

            if self.active_mode == self.MODE_ALTHOLD and alt_valid:
                err_z = to_signed16(sp_thrust - alt_z)
                target_vz = mult_q12(self.kp_alt_pos, err_z)
                inner_sp_thrust = target_vz
                thrust_meas = alt_vz
                base_thrust = 0
            else:
                base_thrust = sp_thrust
        else:  # FAILSAFE
            inner_sp_roll = 0
            inner_sp_pitch = 0
            inner_sp_yaw = 0
            base_thrust = self.failsafe_thrust_q16

        # ----------------------------------------------------------------------
        # Bumpless Transfer Integrator Re-seeding
        # ----------------------------------------------------------------------
        reseed_r = is_transition
        reseed_p = is_transition
        reseed_y = is_transition
        reseed_t = is_transition and (self.active_mode == self.MODE_ALTHOLD)

        new_p_roll = mult_q12(self.core_roll.kp_q16, to_signed16(inner_sp_roll - imu_p))
        reseed_val_roll = to_signed16(self.u_roll_prev - new_p_roll)

        new_p_pitch = mult_q12(self.core_pitch.kp_q16, to_signed16(inner_sp_pitch - imu_q))
        reseed_val_pitch = to_signed16(self.u_pitch_prev - new_p_pitch)

        new_p_yaw = mult_q12(self.core_yaw.kp_q16, to_signed16(inner_sp_yaw - imu_r))
        reseed_val_yaw = to_signed16(self.u_yaw_prev - new_p_yaw)

        if self.active_mode == self.MODE_ALTHOLD and alt_valid:
            new_p_thrust = mult_q12(self.core_thrust.kp_q16, to_signed16(inner_sp_thrust - thrust_meas))
            raw_reseed_t = to_signed16(to_signed16(self.u_thrust_prev - self.hover_thrust_q16) - new_p_thrust)
            reseed_val_thrust = clamp(raw_reseed_t, self.core_thrust.i_min_q16, self.core_thrust.i_max_q16)
        else:
            reseed_val_thrust = 0

        # ----------------------------------------------------------------------
        # Inner Datapath (Roll, Pitch, Yaw)
        # ----------------------------------------------------------------------
        out_r = self.core_roll.step(inner_sp_roll, imu_p, reseed_r, reseed_val_roll)
        out_p = self.core_pitch.step(inner_sp_pitch, imu_q, reseed_p, reseed_val_pitch)
        out_y = self.core_yaw.step(inner_sp_yaw, imu_r, reseed_y, reseed_val_yaw)

        # ----------------------------------------------------------------------
        # Altitude Thrust Datapath & Tilt Compensation
        # ----------------------------------------------------------------------
        if self.active_mode == self.MODE_ALTHOLD and alt_valid:
            out_t = self.core_thrust.step(inner_sp_thrust, thrust_meas, reseed_t, reseed_val_thrust)
            delta_thrust = out_t["u_sat"]
            raw_thrust = to_signed16(self.hover_thrust_q16 + delta_thrust)

            # Small-angle tilt compensation: 1 + 0.5*phi^2 + 0.5*theta^2
            phi_sq = mult_q12(imu_phi, imu_phi)
            theta_sq = mult_q12(imu_theta, imu_theta)
            tilt_factor = to_signed16(SCALE + (phi_sq >> 1) + (theta_sq >> 1))
            max_tilt = int(1.5 * SCALE)
            tilt_factor = clamp(tilt_factor, SCALE, max_tilt)
            final_thrust = mult_q12(raw_thrust, tilt_factor)
        else:
            final_thrust = base_thrust

        # ----------------------------------------------------------------------
        # Output Limiting & Failsafe Override
        # ----------------------------------------------------------------------
        if self.active_mode == self.MODE_FAILSAFE:
            u_roll = 0
            u_pitch = 0
            u_yaw = 0
            u_thrust = self.failsafe_thrust_q16
        else:
            u_roll = out_r["u_sat"]
            u_pitch = out_p["u_sat"]
            u_yaw = out_y["u_sat"]
            u_thrust = clamp(final_thrust, 0, float_to_q12(1.0))

        self.u_roll_prev = u_roll
        self.u_pitch_prev = u_pitch
        self.u_yaw_prev = u_yaw
        self.u_thrust_prev = u_thrust

        self.last_valid_roll = u_roll
        self.last_valid_pitch = u_pitch
        self.last_valid_yaw = u_yaw
        self.last_valid_thrust = u_thrust
        self.has_valid_sample = True

        return {
            "output_valid": True,
            "active_mode": self.active_mode,
            "u_roll": u_roll,
            "u_pitch": u_pitch,
            "u_yaw": u_yaw,
            "u_thrust": u_thrust,
            "watchdog_alarm": self.watchdog_alarm,
            "fault_code": self.sticky_fault,
        }


# ==============================================================================
# SELF-TEST & VERIFICATION
# ==============================================================================

if __name__ == "__main__":
    print("=== Testing Bit-Accurate PID Golden Model (16-bit Q4.12) ===")
    core = PidAxisCoreGolden(kp=1.5, ki_eff_q12=0x0002, kd_eff_q12=0x2000, alpha=0.5)

    # Test 1: Step response
    sp = float_to_q12(2.0)
    meas = 0
    res = core.step(sp, meas)
    print(f"Step 0: P={q12_to_float(res['p_term']):.4f}, I={q12_to_float(res['i_term']):.4f}, "
          f"D={q12_to_float(res['d_term']):.4f}, U_sat={q12_to_float(res['u_sat']):.4f}")

    assert res['u_sat'] == core.u_max_q16, "Expected output saturation at max limit"

    # Test 2: Anti-windup conditional clamping
    for _ in range(10):
        res = core.step(sp, meas)
    print(f"After 10 cycles saturated: I={q12_to_float(res['i_term']):.4f}, ClampHigh={res['clamp_high']}")
    assert res['clamp_high'] is True, "Expected clamp_high asserted"

    # Test 3: Bumpless transfer re-seeding
    prev_u = float_to_q12(0.75)
    new_sp = float_to_q12(0.25)
    new_p = mult_q12(core.kp_q16, new_sp) # 1.5 * 0.25 = 0.375
    reseed_i = to_signed16(prev_u - new_p) # 0.75 - 0.375 = 0.375 (within [-0.5, 0.5])
    res = core.step(new_sp, 0, reseed_valid=True, reseed_val_q16=reseed_i)
    diff = abs(res['u_sat'] - prev_u)
    print(f"Bumpless handover: Target={q12_to_float(prev_u):.4f}, Got={q12_to_float(res['u_sat']):.4f}, Diff={diff}")
    assert diff <= 1, f"Expected zero or 1 LSB bump on handover, got diff={diff}"

    # Test 4: Top-level AltHold thrust bumpless handover
    top = PidTopGolden(timeout_cycles=100)
    st_th = float_to_q12(0.55)
    for _ in range(5):
        r = top.step(sample_valid=True, imu_p=0, imu_q=0, imu_r=0, imu_phi=0, imu_theta=0,
                     alt_z=st_th, alt_vz=0, alt_valid=True, mode_cmd=1, failsafe_ext_n=True,
                     sp_roll=0, sp_pitch=0, sp_yaw=0, sp_thrust=st_th)
    u_th_stab = r['u_thrust']
    r_alt = top.step(sample_valid=True, imu_p=0, imu_q=0, imu_r=0, imu_phi=0, imu_theta=0,
                     alt_z=st_th, alt_vz=0, alt_valid=True, mode_cmd=2, failsafe_ext_n=True,
                     sp_roll=0, sp_pitch=0, sp_yaw=0, sp_thrust=st_th)
    diff_th = abs(r_alt['u_thrust'] - u_th_stab)
    print(f"Top AltHold Bumpless: Stab={q12_to_float(u_th_stab):.4f}, AltHold={q12_to_float(r_alt['u_thrust']):.4f}, Diff={diff_th}")
    assert diff_th <= 1, f"Expected bumpless thrust transition, got diff={diff_th}"

    # Test 5: Configurable Failsafe Hold-Last and Critical Fault Override
    top_hl = PidTopGolden(timeout_cycles=10, cfg_failsafe_hold_last=True)
    r_fly = top_hl.step(sample_valid=True, imu_p=0, imu_q=0, imu_r=0, imu_phi=0, imu_theta=0,
                        alt_z=0, alt_vz=0, alt_valid=True, mode_cmd=1, failsafe_ext_n=True,
                        sp_roll=float_to_q12(0.20), sp_pitch=0, sp_yaw=0, sp_thrust=float_to_q12(0.50))
    last_u_roll = r_fly['u_roll']
    assert last_u_roll != 0, "Expected nonzero roll output in flight"

    r_fs = top_hl.step(sample_valid=True, imu_p=0, imu_q=0, imu_r=0, imu_phi=0, imu_theta=0,
                       alt_z=0, alt_vz=0, alt_valid=True, mode_cmd=3, failsafe_ext_n=True,
                       sp_roll=0, sp_pitch=0, sp_yaw=0, sp_thrust=0)
    print(f"Hold-last non-critical failsafe: roll={q12_to_float(r_fs['u_roll']):.4f}, exp={q12_to_float(last_u_roll):.4f}")
    assert r_fs['u_roll'] == last_u_roll, "Expected hold-last to retain previous roll command"

    top_hl.step(sample_valid=True, imu_p=0, imu_q=0, imu_r=0, imu_phi=0, imu_theta=0,
                alt_z=0, alt_vz=0, alt_valid=True, mode_cmd=3, failsafe_ext_n=False,
                sp_roll=0, sp_pitch=0, sp_yaw=0, sp_thrust=0)
    top_hl.step(sample_valid=True, imu_p=0, imu_q=0, imu_r=0, imu_phi=0, imu_theta=0,
                alt_z=0, alt_vz=0, alt_valid=True, mode_cmd=3, failsafe_ext_n=False,
                sp_roll=0, sp_pitch=0, sp_yaw=0, sp_thrust=0)
    r_kill = top_hl.step(sample_valid=True, imu_p=0, imu_q=0, imu_r=0, imu_phi=0, imu_theta=0,
                         alt_z=0, alt_vz=0, alt_valid=True, mode_cmd=3, failsafe_ext_n=False,
                         sp_roll=0, sp_pitch=0, sp_yaw=0, sp_thrust=0)
    print(f"Emergency kill override: roll={r_kill['u_roll']}, thrust={r_kill['u_thrust']}")
    assert r_kill['u_roll'] == 0 and r_kill['u_thrust'] == 0, "Emergency kill must override hold-last!"

    print("=== All Golden Model Self-Tests PASSED ===")
