#!/usr/bin/env python3
"""
models/generate_vectors.py
Generates deterministic, reproducible test vector files for Verilator testbenches (16-bit Q4.12):
1. vectors/directed/unit_core_directed.hex:
   Targeted tests for arithmetic bounds, anti-windup, derivative kick, and bumpless transfer.
2. vectors/random/unit_core_random_10k.hex:
   10,000 pseudo-random stimulus vectors and expected outputs for pid_axis_core_tb.
3. vectors/random/top_random_10k.hex:
   10,000 multi-axis vectors for pid_top_tb covering all operating modes and transitions.
"""

import sys
import os
import random
from pid_golden_model import (
    PidAxisCoreGolden,
    PidTopGolden,
    float_to_q12,
    q12_to_float,
    mult_q12,
    to_signed16,
    SCALE
)

os.makedirs("vectors/directed", exist_ok=True)
os.makedirs("vectors/random", exist_ok=True)


def to_hex16(val: int) -> str:
    """Format signed 16-bit int as 4-character lowercase hex."""
    return f"{val & 0xFFFF:04x}"


def generate_unit_core_directed():
    print("Generating vectors/directed/unit_core_directed.hex ...")
    core = PidAxisCoreGolden(kp_q12=0x1800, ki_eff_q12=0x0002, kd_eff_q12=0x2000, alpha_q12=0x0800)

    vectors = []
    # Vector format:
    # sp_hex meas_hex reseed_v reseed_val_hex | exp_u_hex exp_p_hex exp_i_hex exp_d_hex exp_clamp_h exp_clamp_l

    # Phase 1: Zero inputs (idle)
    for _ in range(10):
        res = core.step(0, 0)
        vectors.append((0, 0, 0, 0, res))

    # Phase 2: Moderate step error (linear region)
    sp = float_to_q12(0.20)
    for _ in range(20):
        res = core.step(sp, 0)
        vectors.append((sp, 0, 0, 0, res))

    # Phase 3: Extreme positive step forcing output saturation and anti-windup
    sp_huge = float_to_q12(2.0)
    for _ in range(30):
        res = core.step(sp_huge, 0)
        vectors.append((sp_huge, 0, 0, 0, res))

    # Phase 4: Extreme negative step forcing negative saturation
    sp_neg = float_to_q12(-2.0)
    for _ in range(30):
        res = core.step(sp_neg, 0)
        vectors.append((sp_neg, 0, 0, 0, res))

    # Phase 5: Bumpless transfer re-seeding
    prev_u = float_to_q12(0.60)
    new_sp = float_to_q12(0.20)
    new_p = mult_q12(core.kp_q16, new_sp)
    reseed_i = to_signed16(prev_u - new_p)
    res = core.step(new_sp, 0, reseed_valid=True, reseed_val_q16=reseed_i)
    vectors.append((new_sp, 0, 1, reseed_i, res))

    for _ in range(20):
        res = core.step(new_sp, 0)
        vectors.append((new_sp, 0, 0, 0, res))

    # Write output file
    with open("vectors/directed/unit_core_directed.hex", "w") as f:
        for sp_v, meas_v, rv, r_val, r in vectors:
            line = (
                f"{to_hex16(sp_v)} {to_hex16(meas_v)} {rv} {to_hex16(r_val)} "
                f"{to_hex16(r['u_sat'])} {to_hex16(r['p_term'])} {to_hex16(r['i_term'])} {to_hex16(r['d_term'])} "
                f"{1 if r['clamp_high'] else 0} {1 if r['clamp_low'] else 0}\n"
            )
            f.write(line)
    print(f"Generated {len(vectors)} directed vectors.")


def generate_unit_core_random(num_vectors=10000, seed=12345):
    print(f"Generating vectors/random/unit_core_random_10k.hex ({num_vectors} vectors, seed={seed}) ...")
    random.seed(seed)
    core = PidAxisCoreGolden(kp_q12=0x1800, ki_eff_q12=0x0002, kd_eff_q12=0x2000, alpha_q12=0x0800)

    with open("vectors/random/unit_core_random_10k.hex", "w") as f:
        for i in range(num_vectors):
            r = random.random()
            if r < 0.25:
                # Large saturating step to thoroughly exercise anti-windup clamping
                sp = int(random.uniform(-4.0, 4.0) * SCALE)
                meas = int(random.uniform(-4.0, 4.0) * SCALE)
            elif r < 0.35:
                # Zero or sub-count micro-steps
                sp = random.choice([0, 1, -1, 10, -10])
                meas = random.choice([0, 1, -1, 10, -10])
            else:
                # Standard flight control dynamic operating range
                sp = int(random.uniform(-1.5, 1.5) * SCALE)
                meas = int(random.uniform(-1.5, 1.5) * SCALE)

            reseed_v = 1 if (random.random() < 0.02) else 0
            reseed_val = int(random.uniform(-0.5, 0.5) * SCALE) if reseed_v else 0

            res = core.step(sp, meas, reseed_valid=bool(reseed_v), reseed_val_q16=reseed_val)

            line = (
                f"{to_hex16(sp)} {to_hex16(meas)} {reseed_v} {to_hex16(reseed_val)} "
                f"{to_hex16(res['u_sat'])} {to_hex16(res['p_term'])} {to_hex16(res['i_term'])} {to_hex16(res['d_term'])} "
                f"{1 if res['clamp_high'] else 0} {1 if res['clamp_low'] else 0}\n"
            )
            f.write(line)
    print("Completed unit_core_random_10k.hex.")


def generate_top_random(num_vectors=10000, seed=54321):
    print(f"Generating vectors/random/top_random_10k.hex ({num_vectors} vectors, seed={seed}) ...")
    random.seed(seed)
    top = PidTopGolden(timeout_cycles=50000, ts=0.00025)

    with open("vectors/random/top_random_10k.hex", "w") as f:
        current_mode = 0
        rearm_cmd = 0
        for i in range(num_vectors):
            sample_valid = 0 if (random.random() < 0.01) else 1 # 1% sample drops for watchdog
            fs_ext_n = 0 if (random.random() < 0.005) else 1    # Rare emergency kill
            alt_valid = 0 if (random.random() < 0.05) else 1

            # If in Failsafe, attempt pilot re-arm handshake with 15% probability when kill switch released
            if top.active_mode == PidTopGolden.MODE_FAILSAFE:
                if fs_ext_n == 1 and random.random() < 0.15:
                    rearm_cmd = 1
                    current_mode = random.choice([0, 1, 2])
                else:
                    rearm_cmd = 0
            else:
                rearm_cmd = 0
                if random.random() < 0.05:
                    current_mode = random.choice([0, 1, 2, 3])

            imu_p = int(random.uniform(-3.0, 3.0) * SCALE)
            imu_q = int(random.uniform(-3.0, 3.0) * SCALE)
            imu_r = int(random.uniform(-2.0, 2.0) * SCALE)
            imu_phi = int(random.uniform(-0.4, 0.4) * SCALE)
            imu_theta = int(random.uniform(-0.4, 0.4) * SCALE)
            alt_z = int(random.uniform(0.0, 5.0) * SCALE)
            alt_vz = int(random.uniform(-1.5, 1.5) * SCALE)

            sp_r = int(random.uniform(-0.8, 0.8) * SCALE)
            sp_p = int(random.uniform(-0.8, 0.8) * SCALE)
            sp_y = int(random.uniform(-0.8, 0.8) * SCALE)
            sp_t = int(random.uniform(0.2, 0.8) * SCALE)

            res = top.step(
                sample_valid=bool(sample_valid),
                imu_p=imu_p, imu_q=imu_q, imu_r=imu_r,
                imu_phi=imu_phi, imu_theta=imu_theta,
                alt_z=alt_z, alt_vz=alt_vz, alt_valid=bool(alt_valid),
                mode_cmd=current_mode, failsafe_ext_n=bool(fs_ext_n),
                sp_roll=sp_r, sp_pitch=sp_p, sp_yaw=sp_y, sp_thrust=sp_t,
                rearm_cmd=bool(rearm_cmd)
            )

            line = (
                f"{sample_valid} {to_hex16(imu_p)} {to_hex16(imu_q)} {to_hex16(imu_r)} "
                f"{to_hex16(imu_phi)} {to_hex16(imu_theta)} {to_hex16(alt_z)} {to_hex16(alt_vz)} {alt_valid} "
                f"{current_mode} {rearm_cmd} {fs_ext_n} {to_hex16(sp_r)} {to_hex16(sp_p)} {to_hex16(sp_y)} {to_hex16(sp_t)} | "
                f"{1 if res['output_valid'] else 0} {res['active_mode']} {to_hex16(res['u_roll'])} {to_hex16(res['u_pitch'])} "
                f"{to_hex16(res['u_yaw'])} {to_hex16(res['u_thrust'])} {1 if res['watchdog_alarm'] else 0} {res['fault_code']}\n"
            )
            f.write(line)
    print("Completed top_random_10k.hex.")


if __name__ == "__main__":
    generate_unit_core_directed()
    generate_unit_core_random(10000)
    generate_top_random(10000)
    print("All vector generation tasks PASSED.")
