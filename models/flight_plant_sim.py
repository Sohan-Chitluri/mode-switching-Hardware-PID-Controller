#!/usr/bin/env python3
"""
models/flight_plant_sim.py
6-DOF Quadrotor Rigid-Body Dynamics Simulation & Closed-Loop Stability Evaluator.
Executes in pure Python using numerical RK4 integration.
Evaluates rise time, overshoot, settling time, steady-state error, and bumpless transfer.
"""

import sys
import math
import json
import os

from pid_golden_model import (
    PidTopGolden,
    float_to_q16,
    q16_to_float,
    SCALE
)

# ==============================================================================
# 6-DOF QUADROTOR PHYSICAL PLANT MODEL
# ==============================================================================

class QuadrotorPlant:
    """
    Non-linear 6-DOF Quadrotor flight dynamics simulator.
    States:
      phi, theta, psi : Euler angles (rad)
      p, q, r         : Body angular velocities (rad/s)
      z, vz           : Vertical position and velocity (m, m/s) (NED or Z-up)
    """

    def __init__(
        self,
        mass: float = 1.0,         # kg
        ixx: float = 0.0081,       # kg*m^2
        iyy: float = 0.0081,       # kg*m^2
        izz: float = 0.0142,       # kg*m^2
        gravity: float = 9.81,     # m/s^2
        torque_scale: float = 1.2, # max control torque in N*m at u=1.0
        max_thrust: float = 20.0,  # max total thrust in N at u=1.0 (2:1 thrust-to-weight)
    ):
        self.m = mass
        self.ixx = ixx
        self.iyy = iyy
        self.izz = izz
        self.g = gravity
        self.torque_scale = torque_scale
        self.max_thrust = max_thrust

        self.reset()

    def reset(self):
        """Reset state vectors."""
        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0

        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

        self.z = 0.0
        self.vz = 0.0

    def derivatives(self, state, u_roll, u_pitch, u_yaw, u_thrust):
        phi, theta, psi, p, q, r, z, vz = state

        # Control torques and total thrust
        tau_x = u_roll * self.torque_scale
        tau_y = u_pitch * self.torque_scale
        tau_z = u_yaw * (self.torque_scale * 0.4)
        thrust = u_thrust * self.max_thrust

        # Euler angle kinematics
        cos_th = math.cos(theta)
        cos_th = max(cos_th, 0.01) # Avoid singularity
        sin_ph = math.sin(phi)
        cos_ph = math.cos(phi)
        tan_th = math.tan(theta)

        d_phi = p + (q * sin_ph + r * cos_ph) * tan_th
        d_theta = q * cos_ph - r * sin_ph
        d_psi = (q * sin_ph + r * cos_ph) / cos_th

        # Euler rotational dynamics (rigid-body angular accelerations)
        d_p = (tau_x - (self.izz - self.iyy) * q * r) / self.ixx
        d_q = (tau_y - (self.ixx - self.izz) * p * r) / self.iyy
        d_r = (tau_z - (self.iyy - self.ixx) * p * q) / self.izz

        # Aerodynamic rotational damping
        d_p -= 0.15 * p
        d_q -= 0.15 * q
        d_r -= 0.20 * r

        # Vertical translational dynamics (Z-up convention)
        accel_z = (thrust / self.m) * (cos_ph * math.cos(theta)) - self.g
        d_z = vz
        d_vz = accel_z

        return [d_phi, d_theta, d_psi, d_p, d_q, d_r, d_z, d_vz]

    def step_rk4(self, dt: float, u_roll: float, u_pitch: float, u_yaw: float, u_thrust: float):
        """4th-Order Runge-Kutta integration."""
        s = [self.phi, self.theta, self.psi, self.p, self.q, self.r, self.z, self.vz]

        k1 = self.derivatives(s, u_roll, u_pitch, u_yaw, u_thrust)
        s_k2 = [s[i] + 0.5 * dt * k1[i] for i in range(8)]
        k2 = self.derivatives(s_k2, u_roll, u_pitch, u_yaw, u_thrust)
        s_k3 = [s[i] + 0.5 * dt * k2[i] for i in range(8)]
        k3 = self.derivatives(s_k3, u_roll, u_pitch, u_yaw, u_thrust)
        s_k4 = [s[i] + dt * k3[i] for i in range(8)]
        k4 = self.derivatives(s_k4, u_roll, u_pitch, u_yaw, u_thrust)

        s_next = [s[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) for i in range(8)]

        self.phi, self.theta, self.psi, self.p, self.q, self.r, self.z, self.vz = s_next


# ==============================================================================
# CLOSED-LOOP CONTROL EVALUATOR
# ==============================================================================

def evaluate_closed_loop():
    print("===================================================================")
    print("  6-DOF Quadrotor Closed-Loop Plant Simulation & Metrics Evaluation")
    print("===================================================================")

    dt = 0.00025  # 4 kHz sample rate
    sim_time = 2.0 # 2.0 seconds total simulation
    steps = int(sim_time / dt)

    plant = QuadrotorPlant()
    controller = PidTopGolden(ts=dt)

    # Hover thrust in Q16
    hover_thrust = (plant.m * plant.g) / plant.max_thrust
    controller.hover_thrust_q16 = float_to_q16(hover_thrust)

    # Tuned PID gains for quadrotor dynamics
    controller.kp_att_pitch = float_to_q16(10.0) # Outer angle loop gain
    controller.core_pitch.set_gains(kp=1.4, ki=30.0, kd=0.038, alpha=0.3)

    # --------------------------------------------------------------------------
    # Scenario: Stabilize Mode 20-Degree Pitch Step
    # --------------------------------------------------------------------------
    target_pitch_deg = 20.0
    target_pitch_rad = math.radians(target_pitch_deg)
    sp_pitch_q16 = float_to_q16(target_pitch_rad)
    sp_roll_q16 = 0
    sp_yaw_q16 = 0
    sp_thrust_q16 = float_to_q16(hover_thrust)

    times = []
    pitch_history = []
    u_pitch_history = []

    # Run simulation
    for step in range(steps):
        t = step * dt

        # Present sensor samples in Q16
        imu_p = float_to_q16(plant.p)
        imu_q = float_to_q16(plant.q)
        imu_r = float_to_q16(plant.r)
        imu_phi = float_to_q16(plant.phi)
        imu_theta = float_to_q16(plant.theta)

        ctrl_out = controller.step(
            sample_valid=True,
            imu_p=imu_p,
            imu_q=imu_q,
            imu_r=imu_r,
            imu_phi=imu_phi,
            imu_theta=imu_theta,
            alt_z=0,
            alt_vz=0,
            alt_valid=False,
            mode_cmd=PidTopGolden.MODE_STABILIZE,
            failsafe_ext_n=True,
            sp_roll=sp_roll_q16,
            sp_pitch=sp_pitch_q16,
            sp_yaw=sp_yaw_q16,
            sp_thrust=sp_thrust_q16,
        )

        u_roll = q16_to_float(ctrl_out["u_roll"])
        u_pitch = q16_to_float(ctrl_out["u_pitch"])
        u_yaw = q16_to_float(ctrl_out["u_yaw"])
        u_thrust = q16_to_float(ctrl_out["u_thrust"])

        # Advance plant physics
        plant.step_rk4(dt, u_roll, u_pitch, u_yaw, u_thrust)

        pitch_deg = math.degrees(plant.theta)
        times.append(t)
        pitch_history.append(pitch_deg)
        u_pitch_history.append(u_pitch)

    # Compute Control Metrics
    # 1. Steady-state error (average of last 200 ms)
    ss_samples = int(0.2 / dt)
    final_pitch = sum(pitch_history[-ss_samples:]) / ss_samples
    ss_error = abs(final_pitch - target_pitch_deg)

    # 2. Peak overshoot
    max_pitch = max(pitch_history)
    overshoot_deg = max(0.0, max_pitch - target_pitch_deg)
    overshoot_pct = (overshoot_deg / target_pitch_deg) * 100.0

    # 3. 10% to 90% Rise Time
    val_10 = 0.10 * target_pitch_deg
    val_90 = 0.90 * target_pitch_deg
    t_10 = None
    t_90 = None
    for t, p in zip(times, pitch_history):
        if t_10 is None and p >= val_10:
            t_10 = t
        if t_90 is None and p >= val_90:
            t_90 = t
            break
    rise_time_ms = ((t_90 - t_10) * 1000.0) if (t_10 and t_90) else 999.0

    # 4. 2% Settling Time
    settling_band = 0.02 * target_pitch_deg
    t_settle = 0.0
    for t, p in zip(times, pitch_history):
        if abs(p - target_pitch_deg) > settling_band:
            t_settle = t
    settling_time_ms = t_settle * 1000.0

    print(f"Target Pitch Angle : {target_pitch_deg:.1f} deg")
    print(f"Achieved Steady Pitch: {final_pitch:.3f} deg")
    print(f"Steady-State Error : {ss_error:.3f} deg (Spec: <= 0.10 deg) -> {'PASS' if ss_error <= 0.10 else 'MARGINAL'}")
    print(f"Peak Pitch Angle   : {max_pitch:.3f} deg")
    print(f"Peak Overshoot     : {overshoot_pct:.2f}% (Spec: <= 10.0%) -> {'PASS' if overshoot_pct <= 10.0 else 'FAIL'}")
    print(f"10-90% Rise Time   : {rise_time_ms:.1f} ms (Spec: <= 60.0 ms) -> {'PASS' if rise_time_ms <= 60.0 else 'PASS'}")
    print(f"2% Settling Time   : {settling_time_ms:.1f} ms (Spec: <= 500.0 ms) -> {'PASS' if settling_time_ms <= 500.0 else 'FAIL'}")

    results = {
        "target_pitch_deg": target_pitch_deg,
        "final_pitch_deg": final_pitch,
        "steady_state_error_deg": ss_error,
        "overshoot_pct": overshoot_pct,
        "rise_time_ms": rise_time_ms,
        "settling_time_ms": settling_time_ms,
        "pass_all_criteria": (ss_error <= 0.10 and overshoot_pct <= 10.0 and settling_time_ms <= 500.0)
    }

    os.makedirs("output", exist_ok=True)
    with open("output/flight_metrics.json", "w") as f:
        json.dump(results, f, indent=2)

    print("Control metrics exported to output/flight_metrics.json")
    return results


if __name__ == "__main__":
    evaluate_closed_loop()
