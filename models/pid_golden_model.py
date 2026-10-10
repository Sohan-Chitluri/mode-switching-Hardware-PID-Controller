#!/usr/bin/env python3
"""
models/pid_golden_model.py
Bit-accurate fixed-point reference model for hardware PID flight controller.
Matches synthesizable SystemVerilog datapath:
- Q16.16 signed two's complement arithmetic
- 64-bit multiplier full product with shift-by-16 truncation
- 48-bit guarded accumulation (16 guard bits prevent intermediate overflow)
- Measurement-based derivative with first-order IIR LPF
- Conditional integration anti-windup clamping
- Bumpless mode-transition integrator back-calculation
- Cycle-accurate pipeline matching
"""

import sys
import math
import json

# ==============================================================================
# FIXED-POINT ARITHMETIC PRIMITIVES (Q16.16)
# ==============================================================================

WIDTH = 32
FRACT_W = 16
SCALE = 1 << FRACT_W  # 65536

MAX_INT32 = (1 << 31) - 1   #  2147483647
MIN_INT32 = -(1 << 31)      # -2147483648

MASK32 = (1 << 32) - 1
MASK64 = (1 << 64) - 1


def to_signed32(val: int) -> int:
    """Clamp/wrap to signed 32-bit integer."""
    val = val & MASK32
    if val & (1 << 31):
        return val - (1 << 32)
    return val


def to_signed48(val: int) -> int:
    """Sign-extend / wrap to signed 48-bit accumulator."""
    mask48 = (1 << 48) - 1
    val = val & mask48
    if val & (1 << 47):
        return val - (1 << 48)
    return val


def float_to_q16(val: float) -> int:
    """Convert floating-point number to Q16.16 signed 32-bit integer."""
    scaled = int(round(val * SCALE))
    if scaled > MAX_INT32:
        return MAX_INT32
    if scaled < MIN_INT32:
        return MIN_INT32
    return scaled


def q16_to_float(val: int) -> float:
    """Convert Q16.16 signed integer to float."""
    return val / float(SCALE)


def clamp(val: int, low: int, high: int) -> int:
    """Saturate integer val to [low, high]."""
    if val > high:
        return high
    if val < low:
        return low
    return val


def mult_q16(a: int, b: int) -> int:
    """
    Synthesizable signed 32x32 -> 64-bit product, arithmetic shifted right by 16.
    Matches RTL: signed'(a) * signed'(b) >>> 16.
    """
    prod64 = a * b
    res = prod64 >> FRACT_W
    return to_signed32(res)


# ==============================================================================
# SINGLE-CHANNEL PID CORE GOLDEN MODEL (pid_axis_core)
# ==============================================================================

class PidAxisCoreGolden:
    """
    Bit-accurate model of a single-axis PID channel.
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
    ):
        self.ts = ts
        # Pre-scale gains as done in digital flight controllers
        self.kp_q16 = float_to_q16(kp)
        self.ki_eff_q16 = float_to_q16(ki * ts)
        self.kd_eff_q16 = float_to_q16(kd / ts) if ts > 0 else 0
        self.alpha_q16 = float_to_q16(alpha)

        self.u_min_q16 = float_to_q16(u_min)
        self.u_max_q16 = float_to_q16(u_max)
        self.i_min_q16 = float_to_q16(i_min)
        self.i_max_q16 = float_to_q16(i_max)

        # Internal State Registers
        self.reset()

    def reset(self):
        """Clear all internal registers to deterministic zero state."""
        self.integ_reg = 0         # 32-bit signed accumulator
        self.deriv_filter_reg = 0  # 32-bit signed filtered measurement
        self.clamp_high_reg = False
        self.clamp_low_reg = False
        self.u_prev = 0

    def set_gains(self, kp: float, ki: float, kd: float, alpha: float):
        """Update active gains (e.g., on mode change)."""
        self.kp_q16 = float_to_q16(kp)
        self.ki_eff_q16 = float_to_q16(ki * self.ts)
        self.kd_eff_q16 = float_to_q16(kd / self.ts) if self.ts > 0 else 0
        self.alpha_q16 = float_to_q16(alpha)

    def set_gains_q16(self, kp_q16: int, ki_eff_q16: int, kd_eff_q16: int, alpha_q16: int):
        """Directly set fixed-point gains."""
        self.kp_q16 = kp_q16
        self.ki_eff_q16 = ki_eff_q16
        self.kd_eff_q16 = kd_eff_q16
        self.alpha_q16 = alpha_q16

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
        # ----------------------------------------------------------------------
        # Stage 1: Derivative Filter (Measurement-Based IIR LPF)
        # ----------------------------------------------------------------------
        if reseed_valid:
            # Bumpless transfer: pre-seed filter with current measurement -> delta = 0
            self.deriv_filter_reg = meas_q16
            delta_yf = 0
        else:
            delta_y = to_signed32(meas_q16 - self.deriv_filter_reg)
            delta_filt = mult_q16(self.alpha_q16, delta_y)
            next_filter = to_signed32(self.deriv_filter_reg + delta_filt)
            delta_yf = to_signed32(next_filter - self.deriv_filter_reg)
            self.deriv_filter_reg = next_filter

        # D-term = -Kd_eff * delta_yf
        d_term = to_signed32(-mult_q16(self.kd_eff_q16, delta_yf))

        # ----------------------------------------------------------------------
        # Stage 2: Proportional Term
        # ----------------------------------------------------------------------
        error = to_signed32(sp_q16 - meas_q16)
        p_term = mult_q16(self.kp_q16, error)

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
                delta_i = mult_q16(self.ki_eff_q16, error)
                next_i = to_signed32(self.integ_reg + delta_i)
                self.integ_reg = clamp(next_i, self.i_min_q16, self.i_max_q16)

        i_term = self.integ_reg

        # ----------------------------------------------------------------------
        # Stage 4: Summation in 48-bit Guarded Accumulator & Saturation
        # ----------------------------------------------------------------------
        raw_sum = to_signed48(p_term + i_term + d_term)

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

    def __init__(self, timeout_cycles: int = 50000, ts: float = 0.00025):
        self.timeout_cycles = timeout_cycles
        self.ts = ts

        # Core PID Channels (Parallel Hardware)
        self.core_roll = PidAxisCoreGolden(kp=0.15, ki=0.20, kd=0.004, alpha=0.3, ts=ts, u_min=-1.0, u_max=+1.0)
        self.core_pitch = PidAxisCoreGolden(kp=0.15, ki=0.20, kd=0.004, alpha=0.3, ts=ts, u_min=-1.0, u_max=+1.0)
        self.core_yaw = PidAxisCoreGolden(kp=0.25, ki=0.15, kd=0.000, alpha=0.3, ts=ts, u_min=-1.0, u_max=+1.0)
        self.core_thrust = PidAxisCoreGolden(kp=0.40, ki=0.30, kd=0.010, alpha=0.2, ts=ts, u_min=0.0, u_max=+1.0)

        # Outer Attitude & Altitude Gains
        self.kp_att_roll = float_to_q16(4.5)   # Outer angle P-gain (rad/s per rad)
        self.kp_att_pitch = float_to_q16(4.5)
        self.kp_alt_pos = float_to_q16(1.2)    # Outer altitude P-gain ((m/s) per m)
        self.hover_thrust_q16 = float_to_q16(0.50) # 50% baseline hover throttle
        self.failsafe_thrust_q16 = float_to_q16(0.0) # Motor disarm / 0 throttle

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

        self._apply_mode_gains(self.active_mode)

    def _apply_mode_gains(self, mode: int):
        """Dynamically apply gain bank matrix matching pid_gain_bank.sv."""
        if mode == self.MODE_ACRO:
            self.kp_att_roll = 0x00048000
            self.kp_att_pitch = 0x00048000
            self.kp_alt_pos = 0x00013333
            self.hover_thrust_q16 = 0x00008000
            self.failsafe_thrust_q16 = 0x00000000
            # Acro mode agile rate gains
            self.core_roll.set_gains_q16(kp_q16=0x00004000, ki_eff_q16=0x00000007, kd_eff_q16=0x00140000, alpha_q16=0x00004CCD)
            self.core_pitch.set_gains_q16(kp_q16=0x00004000, ki_eff_q16=0x00000007, kd_eff_q16=0x00140000, alpha_q16=0x00004CCD)
            self.core_yaw.set_gains_q16(kp_q16=0x00004000, ki_eff_q16=0x00000002, kd_eff_q16=0x00000000, alpha_q16=0x00004CCD)
            self.core_thrust.set_gains_q16(kp_q16=0x00006666, ki_eff_q16=0x00000005, kd_eff_q16=0x00280000, alpha_q16=0x00003333)
        elif mode in (self.MODE_STABILIZE, self.MODE_ALTHOLD):
            self.kp_att_roll = 0x000A0000 # 10.0
            self.kp_att_pitch = 0x000A0000 # 10.0
            self.kp_alt_pos = 0x00013333 # 1.2
            self.hover_thrust_q16 = 0x00008000
            self.failsafe_thrust_q16 = 0x00000000
            # Stabilize / AltHold tuned rate gains (kp=1.4, ki=30.0, kd=0.038)
            self.core_roll.set_gains_q16(kp_q16=0x00016666, ki_eff_q16=0x000001EC, kd_eff_q16=0x00980000, alpha_q16=0x00004CCD)
            self.core_pitch.set_gains_q16(kp_q16=0x00016666, ki_eff_q16=0x000001EC, kd_eff_q16=0x00980000, alpha_q16=0x00004CCD)
            self.core_yaw.set_gains_q16(kp_q16=0x00004000, ki_eff_q16=0x00000002, kd_eff_q16=0x00000000, alpha_q16=0x00004CCD)
            self.core_thrust.set_gains_q16(kp_q16=0x00006666, ki_eff_q16=0x00000005, kd_eff_q16=0x00280000, alpha_q16=0x00003333)
        else: # MODE_FAILSAFE
            self.kp_att_roll = 0
            self.kp_att_pitch = 0
            self.kp_alt_pos = 0
            self.hover_thrust_q16 = 0
            self.failsafe_thrust_q16 = 0
            self.core_roll.set_gains_q16(kp_q16=0, ki_eff_q16=0, kd_eff_q16=0, alpha_q16=0x00004CCD)
            self.core_pitch.set_gains_q16(kp_q16=0, ki_eff_q16=0, kd_eff_q16=0, alpha_q16=0x00004CCD)
            self.core_yaw.set_gains_q16(kp_q16=0, ki_eff_q16=0, kd_eff_q16=0, alpha_q16=0x00004CCD)
            self.core_thrust.set_gains_q16(kp_q16=0, ki_eff_q16=0, kd_eff_q16=0, alpha_q16=0x00003333)

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
        self._apply_mode_gains(self.active_mode)

    def step(
        self,
        sample_valid: bool,
        imu_p: int,       # Roll rate (Q16.16 rad/s)
        imu_q: int,       # Pitch rate (Q16.16 rad/s)
        imu_r: int,       # Yaw rate (Q16.16 rad/s)
        imu_phi: int,     # Roll angle (Q16.16 rad)
        imu_theta: int,   # Pitch angle (Q16.16 rad)
        alt_z: int,       # Altitude (Q16.16 m)
        alt_vz: int,      # Vertical climb rate (Q16.16 m/s)
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

        # Immediate deterministic failsafe override (even when sensor samples stop)
        if self.active_mode == self.MODE_FAILSAFE:
            self.u_roll_prev = 0
            self.u_pitch_prev = 0
            self.u_yaw_prev = 0
            self.u_thrust_prev = self.failsafe_thrust_q16
            return {
                "output_valid": True,
                "active_mode": self.active_mode,
                "u_roll": 0,
                "u_pitch": 0,
                "u_yaw": 0,
                "u_thrust": self.failsafe_thrust_q16,
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
        if self.active_mode == self.MODE_ACRO:
            # Direct rate setpoints from pilot
            inner_sp_roll = sp_roll
            inner_sp_pitch = sp_pitch
            inner_sp_yaw = sp_yaw
            base_thrust = sp_thrust
        elif self.active_mode in (self.MODE_STABILIZE, self.MODE_ALTHOLD):
            # Outer attitude proportional loop
            err_phi = to_signed32(sp_roll - imu_phi)
            err_theta = to_signed32(sp_pitch - imu_theta)
            inner_sp_roll = mult_q16(self.kp_att_roll, err_phi)
            inner_sp_pitch = mult_q16(self.kp_att_pitch, err_theta)
            inner_sp_yaw = sp_yaw  # Yaw remains rate control

            if self.active_mode == self.MODE_ALTHOLD and alt_valid:
                err_z = to_signed32(sp_thrust - alt_z)
                target_vz = mult_q16(self.kp_alt_pos, err_z)
                # Altitude channel operates on velocity error
                inner_sp_thrust = target_vz
                thrust_meas = alt_vz
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

        # Back-calculate: I_new = U_prev - P_new
        new_p_roll = mult_q16(self.core_roll.kp_q16, to_signed32(inner_sp_roll - imu_p))
        reseed_val_roll = to_signed32(self.u_roll_prev - new_p_roll)

        new_p_pitch = mult_q16(self.core_pitch.kp_q16, to_signed32(inner_sp_pitch - imu_q))
        reseed_val_pitch = to_signed32(self.u_pitch_prev - new_p_pitch)

        new_p_yaw = mult_q16(self.core_yaw.kp_q16, to_signed32(inner_sp_yaw - imu_r))
        reseed_val_yaw = to_signed32(self.u_yaw_prev - new_p_yaw)

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
            out_t = self.core_thrust.step(inner_sp_thrust, thrust_meas, reseed_t, 0)
            delta_thrust = out_t["u_sat"]
            raw_thrust = to_signed32(self.hover_thrust_q16 + delta_thrust)

            # Small-angle tilt compensation: T / (cos phi * cos theta) ≈ T * (1 + 0.5*phi^2 + 0.5*theta^2)
            phi_sq = mult_q16(imu_phi, imu_phi)
            theta_sq = mult_q16(imu_theta, imu_theta)
            tilt_factor = to_signed32(SCALE + (phi_sq >> 1) + (theta_sq >> 1))
            # Clamp tilt factor to max 1.5x (approx 48 deg tilt)
            max_tilt = int(1.5 * SCALE)
            tilt_factor = clamp(tilt_factor, SCALE, max_tilt)
            final_thrust = mult_q16(raw_thrust, tilt_factor)
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
            u_thrust = clamp(final_thrust, 0, float_to_q16(1.0))

        self.u_roll_prev = u_roll
        self.u_pitch_prev = u_pitch
        self.u_yaw_prev = u_yaw
        self.u_thrust_prev = u_thrust

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
    print("=== Testing Bit-Accurate PID Golden Model ===")
    core = PidAxisCoreGolden(kp=1.5, ki=2.0, kd=0.05, alpha=0.5, ts=0.00025)

    # Test 1: Step response
    sp = float_to_q16(10.0)
    meas = 0
    res = core.step(sp, meas)
    print(f"Step 0: P={q16_to_float(res['p_term']):.4f}, I={q16_to_float(res['i_term']):.4f}, "
          f"D={q16_to_float(res['d_term']):.4f}, U_sat={q16_to_float(res['u_sat']):.4f}")

    assert res['u_sat'] == core.u_max_q16, "Expected output saturation at max limit"

    # Test 2: Anti-windup conditional clamping
    for _ in range(10):
        res = core.step(sp, meas)
    print(f"After 10 cycles saturated: I={q16_to_float(res['i_term']):.4f}, ClampHigh={res['clamp_high']}")
    assert res['clamp_high'] is True, "Expected clamp_high asserted"

    # Test 3: Bumpless transfer re-seeding
    prev_u = float_to_q16(0.75)
    new_sp = float_to_q16(0.25)
    new_p = mult_q16(core.kp_q16, new_sp) # 1.5 * 0.25 = 0.375
    reseed_i = to_signed32(prev_u - new_p) # 0.75 - 0.375 = 0.375 (within [-0.5, 0.5])
    res = core.step(new_sp, 0, reseed_valid=True, reseed_val_q16=reseed_i)
    diff = abs(res['u_sat'] - prev_u)
    print(f"Bumpless handover: Target={q16_to_float(prev_u):.4f}, Got={q16_to_float(res['u_sat']):.4f}, Diff={diff}")
    assert diff <= 1, f"Expected zero or 1 LSB bump on handover, got diff={diff}"

    print("=== All Golden Model Self-Tests PASSED ===")
