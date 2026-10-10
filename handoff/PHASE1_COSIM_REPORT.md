# Phase 1 Report: Closed-Loop RTL-to-Plant Co-Simulation & PID Tuning

**Project:** Mode-Switching Hardware PID Flight Controller (`pid_top`)  
**Repository:** `/home/peskybird/Projects/PID`  
**Starting Branch:** `fix/audit-remediation`  
**Working Branch:** `feat/phase1-cosim-tuning`  
**Baseline Commit:** `1ab1732` (Parent: `e647704`)  
**Date:** October 2026  
**Status:** Verification Signed-Off (True RTL Closed-Loop PASS, Specifications Met)

---

## 1. Executive Summary

In Phase 1, a true closed-loop RTL-to-flight-plant co-simulation was constructed and validated. The compiled Verilated synthesizable `pid_top` SystemVerilog design directly controls the 4-DOF Quadrotor physical flight plant through a high-performance, deterministic C API without inter-process IPC overhead.

Before tuning, the co-simulation harness was formally proven to drive the plant through seven distinct authenticity checks (including dynamic input responsiveness, plant acceleration propagation, closed feedback loop coupling, bit-exact golden equivalence, trajectory sensitivity to perturbed RTL actuation, and mismatch detection).

Following harness validation, Bank 1 (`MODE_STABILIZE`) and Bank 2 (`MODE_ALTHOLD`) PID parameters were systematically tuned in both RTL and the Python golden model. The closed-loop plant was brought into full compliance with all acceptance requirements:
- **10–90% Rise Time:** **57.2 ms** (Requirement: $\le 60.0\text{ ms}$) — **PASS** (improved from un-tuned 160.8 ms)
- **Peak Overshoot:** **0.78%** (Requirement: $\le 10.0\%$) — **PASS**
- **Steady-State Error:** **0.003°** (Requirement: $\le 0.10^\circ$) — **PASS**
- **2% Settling Time:** **93.8 ms** (Requirement: $\le 500.0\text{ ms}$) — **PASS**
- **RTL / Golden Equivalence:** **0 mismatches** across all 32,000 evaluated channels in 8,000 co-simulation steps (100% bit-exact) — **PASS**

---

## 2. Co-Simulation Architecture & Build Instructions

### 2.1 Architecture
The co-simulation interface couples the Verilator C++ simulation model (`Vpid_top`) directly to Python using `ctypes` through a dedicated C wrapper (`tb/cosim/pid_top_cosim.cpp`):

```
+---------------------------------------------------------------------------------+
|                               Python Runtime                                    |
|  models/flight_plant_cosim.py                                                  |
|   - Step plant state -> Sample sensors (rad, rad/s) -> Quantize to Q16.16       |
|   - Parallel Golden Model (PidTopGolden) step-by-step verification              |
|   - Dynamic step response evaluator (compute_control_metrics)                   |
+---------------------------------------------------------------------------------+
                                      |
                            ctypes in-memory C API
                                      v
+---------------------------------------------------------------------------------+
|                      Shared Library: libpid_top_cosim.so                        |
|  tb/cosim/pid_top_cosim.cpp                                                     |
|   - cosim_init(), cosim_reset(), cosim_step(), cosim_set_bias_pitch()           |
|   - Clock orchestration: clk low -> eval() -> clk high -> eval() -> latch out   |
+---------------------------------------------------------------------------------+
                                      |
                              Direct C++ Calls
                                      v
+---------------------------------------------------------------------------------+
|                           Verilated RTL Model                                   |
|  output/obj_cosim/Vpid_top.cpp                                                  |
|   - Synthesizable SystemVerilog top-level (pid_top.sv) and 14 submodules        |
|   - Fixed-point multipliers, IIR derivative filter, anti-windup integrator,     |
|     cascaded outer loops, mode manager FSM, and registered output stage         |
+---------------------------------------------------------------------------------+
                                      |
                     Actuator Torques u_roll, u_pitch, ...
                                      v
+---------------------------------------------------------------------------------+
|                         4-DOF Quadrotor Physical Plant                          |
|  flight_plant_sim.QuadrotorPlant                                                |
|   - 4th-Order Runge-Kutta (RK4) numerical integration at dt = 0.00025 s (4 kHz) |
|   - States: Euler angles (phi, theta, psi), angular rates (p, q, r), z, vz      |
+---------------------------------------------------------------------------------+
```

### 2.2 Build & Execution Instructions
Within the Nix development environment (`nix develop`):
1. **Build shared library:**
   ```bash
   make cosim-lib
   ```
2. **Execute 7-point harness authenticity verification:**
   ```bash
   PYTHONPATH=models python3 models/flight_plant_cosim.py --verify-harness
   ```
3. **Run 2.0 s closed-loop RTL co-simulation step response:**
   ```bash
   PYTHONPATH=models python3 models/flight_plant_cosim.py
   ```
4. **Run full 7-stage end-to-end regression:**
   ```bash
   bash scripts/run_regression.sh
   ```

---

## 3. Evidence That the Compiled RTL Drives the Plant

Before tuning, the harness was subjected to 7 verification proofs (`--verify-harness`), all of which passed:

| Proof # | Test Name | Method & Criterion | Result |
| :---: | :--- | :--- | :---: |
| **Proof 1** | **RTL Responsiveness to Inputs** | Applied positive pitch setpoint ($10.0^\circ$) to reset RTL; verified `u_pitch` command asserts positive torque ($+65,536$ counts = $+1.0$). | **PASS** |
| **Proof 2** | **RTL Output Propagation** | Applied $+0.5$ torque command into physical plant; verified plant angular acceleration $\Delta q = +0.0185\text{ rad/s}$ via RK4 integration. | **PASS** |
| **Proof 3** | **Closed Feedback Loop Coupler** | Closed loop around plant for 30 cycles; verified plant pitch evolved ($\theta = 0.0421^\circ$) and RTL dynamically modulated torque through 30 unique non-saturated values. | **PASS** |
| **Proof 4** | **RTL vs Golden Equivalence** | Stepped RTL and golden model in lockstep over 500 sinusoidal stimulus steps; verified $0$ bit mismatches across all 2,000 channel evaluations. | **PASS** |
| **Proof 5** | **Actuation Sensitivity** | Injected intentional $+0.1$ Q16 torque bias via `cosim_set_bias_pitch`; verified plant trajectory diverged by $0.2407^\circ$ relative to unperturbed baseline. | **PASS** |
| **Proof 6** | **Discrepancy Detection** | Injected deliberate 1-count difference; verified mismatch detector triggered and halted cleanly. | **PASS** |
| **Proof 7** | **Deterministic Reproducibility** | Executed identical closed-loop simulation runs from reset; verified 100% bit-exact trajectory repeatability across runs. | **PASS** |

---

## 4. Plant Model Assumptions, Units & Coordinate System

- **Degrees of Freedom:** Exactly **4-DOF**:
  - 3 Rotational DOFs: Roll $\phi$, Pitch $\theta$, Yaw $\psi$ (Euler angles in radians; body rates $p, q, r$ in rad/s).
  - 1 Translational DOF: Vertical altitude $z$ and vertical velocity $v_z$ (meters, m/s; Z-up convention).
  - *Explicit Limitation:* Horizontal planar translations ($x, y$) are decoupled and not modeled.
- **Physical Parameters:**
  - Mass: $m = 1.0\text{ kg}$
  - Moments of Inertia: $I_{xx} = 0.0081\text{ kg}\cdot\text{m}^2$, $I_{yy} = 0.0081\text{ kg}\cdot\text{m}^2$, $I_{zz} = 0.0142\text{ kg}\cdot\text{m}^2$
  - Max Control Torque: $\tau_{max} = 1.2\text{ N}\cdot\text{m}$ (scaled by normalized command $u \in [-1.0, 1.0]$)
  - Max Total Thrust: $T_{max} = 20.0\text{ N}$ (2:1 thrust-to-weight ratio, scaled by $u_{thrust} \in [0.0, 1.0]$)
  - Rotational Aerodynamic Damping: $0.15\cdot p$, $0.15\cdot q$, $0.20\cdot r$
- **Integration Scheme:** 4th-order Runge-Kutta (RK4) integration with uniform step size $dt = 0.00025\text{ s}$ ($4\text{ kHz}$).
- **Controller Interface:** Synchronous strobe `sample_valid` at $4\text{ kHz}$ ($250\ \mu\text{s}$ interval); 1-cycle registered output latency.

---

## 5. Tuning Procedure & Parameter History

### 5.1 Parameter Meaning and Arithmetic Implementation
In the cascaded flight controller:
1. **Outer Attitude Loop:** Computes rate setpoint from attitude angle error:
   $$q_{sp} = K_{p,att} \cdot (\theta_{sp} - \theta_{meas})$$
2. **Inner Rate Loop:** Computes torque command from rate error:
   $$u = \text{sat}\left( K_p \cdot (q_{sp} - q) + \text{Accum} + K_{d,eff} \cdot \Delta q_{filt} \right)$$
   where:
   - $K_{i,eff} = K_i \cdot T_s$ (integrated each cycle via 33-bit guarded accumulator with anti-windup clamping).
   - $K_{d,eff} = K_d / T_s$ (applied to 1st-order IIR filtered measurement derivative).
   - $\alpha = 0.3$ (IIR low-pass filter cutoff weighting).

### 5.2 Tested Configurations & Measured Dynamic Responses

| Iteration | Description | $K_{p,att}$ | $K_p$ | $K_i$ ($K_{i,eff}$) | $K_d$ ($K_{d,eff}$) | $\alpha$ | Rise Time | Overshoot | SS Error | Settling Time | Status |
| :---: | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **0** | Un-tuned Baseline | 10.0 | 1.4 | 30.0 (0.0075) | 0.038 (152.0) | 0.3 | 160.8 ms | 0.00% | 0.023° | 441.5 ms | **FAIL (Slow Rise)** |
| **1** | Candidate 1 | 35.0 | 2.0 | 25.0 (0.00625) | 0.018 (72.0) | 0.3 | 57.8 ms | 0.03% | 0.003° | 96.5 ms | **PASS** |
| **2 (Final)** | **Candidate 2 (Adopted)** | **35.0** | **2.0** | **25.0 (0.00625)** | **0.020 (80.0)** | **0.3** | **57.2 ms** | **0.78%** | **0.003°** | **93.8 ms** | **PASS (All Criteria)** |
| **3** | Candidate 3 | 36.0 | 2.0 | 25.0 (0.00625) | 0.020 (80.0) | 0.3 | 56.2 ms | 1.09% | 0.003° | 91.8 ms | **PASS** |
| **4** | Candidate 4 | 38.0 | 2.2 | 25.0 (0.00625) | 0.020 (80.0) | 0.3 | 54.7 ms | 1.25% | 0.003° | 89.0 ms | **PASS** |

Candidate 2 was selected for final RTL implementation because it provides robust stability margin, zero control chatter, gentle actuator demands ($\max |u| \le 1.0$, smooth braking), and comfortably beats the 60.0 ms rise time limit without aggressive overshoot.

### 5.3 Encodings Applied to RTL (`pid_gain_bank.sv`) & Golden Model (`pid_golden_model.py`)
- `kp_att_roll` = `32'sh0023_0000` ($35.0 \times 65536 = 2,293,760$)
- `kp_att_pitch` = `32'sh0023_0000` ($35.0 \times 65536 = 2,293,760$)
- `roll_kp` / `pitch_kp` = `32'sh0002_0000` ($2.0 \times 65536 = 131,072$)
- `roll_ki_eff` / `pitch_ki_eff` = `32'sh0000_019A` ($0.00625 \times 65536 \approx 410$)
- `roll_kd_eff` / `pitch_kd_eff` = `32'sh0050_0000` ($80.0 \times 65536 = 5,242,880$)
- `roll_alpha` / `pitch_alpha` = `32'sh0000_4CCD` ($0.3 \times 65536 = 19,661$)

---

## 6. Final Performance Metrics (Closed-Loop RTL Co-Simulation)

Measured from the 8,000-step co-simulation trajectory using `compute_control_metrics()`:

| Performance Metric | Documented Specification | Measured RTL Co-Sim Value | Status |
| :--- | :---: | :---: | :---: |
| **10–90% Rise Time** | $\le 60.0\text{ ms}$ | **57.2 ms** | **PASS** |
| **Peak Overshoot** | $\le 10.0\%$ | **0.78%** ($20.157^\circ$ peak on $20.0^\circ$ cmd) | **PASS** |
| **Steady-State Error** | $\le 0.10^\circ$ | **0.003°** ($19.997^\circ$ achieved) | **PASS** |
| **2% Settling Time** | $\le 500.0\text{ ms}$ | **93.8 ms** | **PASS** |
| **Actuator Command Saturation** | Safe clamping in $[-1.0, 1.0]$ | Saturated cleanly during transient; zero steady chatter | **PASS** |
| **RTL / Golden Equivalence** | $0$ mismatches | **0 mismatches / 32,000 checks** | **PASS** |

---

## 7. Full Regression & Verification Consistency

| Verification Stage | Command Executed | Return Code | Results Observed |
| :--- | :--- | :---: | :--- |
| **1. RTL Synthesizability Lint** | `make lint` | `0` | **PASS** — Zero errors or warnings across 15 Verilog modules. |
| **2. Structural Elaboration Check** | `yosys -s scripts/prelim_synth.tcl` | `0` | **PASS** — Top `pid_top` elaborated, 171,215 cells synthesized, `check -assert` clean with 0 latches/loops. |
| **3. Unit Core Regression** | `make test-unit` | `0` | **PASS** — 10,111 vectors verified bit-exact against reference model. |
| **4. Full-Chip Top Testbench** | `make test-top` | `0` | **PASS** — 10,000 random vectors + 5 directed tests verified bit-exact. |
| **5. Formal Invariant Suite** | `sby -f tb/formal/pid_formal.sby` | `0` | **PASS** — Properties P1–P5 proven for BMC depth 10 via Z3 SMT solver. |
| **6. RTL Co-Sim Harness Proofs** | `python3 models/flight_plant_cosim.py --verify-harness` | `0` | **PASS** — All 7 authenticity and coupling proofs passed. |
| **7. Closed-Loop RTL Co-Simulation** | `python3 models/flight_plant_cosim.py` | `0` | **PASS** — 8,000 steps executed; rise time 57.2 ms; 0 mismatches. |
| **Metric Evaluator Unit Tests** | `python3 models/flight_plant_sim.py --test-metrics` | `0` | **PASS** — 5/5 boundary condition tests passed; correctly rejects slow/non-finite data. |
| **Manifest Checksum Catalog** | Python SHA-256 verification | `0` | **PASS** — All 28 files in `handoff/MANIFEST.md` match 100% on disk. |

---

## 8. Remaining Limitations & Non-Blockers

1. **Horizontal Planar Dynamics:** The quadrotor plant models 4-DOF ($p, q, r, z$); lateral translation ($x, y$) is decoupled.
2. **Industrial ASIC PDK Paths:** Genus and Innovus synthesis scripts require customer-specific foundry `.lib` and `.lef` technology files. The RTL and SDC constraints are fully synthesizable and port-consistent.

---

## 9. Acceptance Criteria Decision

| Acceptance Criterion | Decision | Notes |
| :--- | :---: | :--- |
| **Genuine RTL-to-Plant Co-Simulation** | **PASS** | Compiled Verilated RTL directly drives plant feedback loop. |
| **Authenticity Proof of RTL Driving Plant** | **PASS** | 7/7 verification proofs passed cleanly. |
| **RTL vs Golden Model Equivalence** | **PASS** | 0 mismatches across 32,000 channel evaluations in co-sim loop. |
| **10–90% Rise Time $\le 60.0\text{ ms}$** | **PASS** | Measured 57.2 ms. |
| **Peak Overshoot $\le 10.0\%$** | **PASS** | Measured 0.78%. |
| **Steady-State Error $\le 0.10^\circ$** | **PASS** | Measured 0.003°. |
| **2% Settling Time $\le 500.0\text{ ms}$** | **PASS** | Measured 93.8 ms. |
| **Regression & Elaboration Cleanliness** | **PASS** | All 7 regression stages clean. |
| **Manifest Checksum Integrity** | **PASS** | 28/28 files match `MANIFEST.md` bit-for-bit. |

---

## 10. Proposed Commit Message

```text
feat(cosim): implement closed-loop RTL-to-plant co-simulation and tune PID gains

- Add tb/cosim/pid_top_cosim.cpp providing C API wrapper around Verilated Vpid_top.
- Add models/flight_plant_cosim.py implementing true closed-loop RTL-to-plant
  co-simulation with 7-point harness authenticity verification suite.
- Tune Bank 1 (Stabilize) and Bank 2 (AltHold) gains in rtl/modes/pid_gain_bank.sv
  and models/pid_golden_model.py (kp_att=35.0, kp=2.0, ki=25.0, kd=0.020, alpha=0.3).
- Achieve 57.2 ms rise time (spec <= 60 ms), 0.78% overshoot (spec <= 10%),
  0.003 deg steady-state error (spec <= 0.10 deg), and 93.8 ms settling time.
- Update tb/top/pid_top_tb.sv for tuned Bank 1 roll_kp cfg_rdata expectation.
- Regenerate vectors/random/top_random_10k.hex with tuned parameters.
- Update Makefile with cosim-lib and test-cosim targets.
- Update scripts/run_regression.sh to 7 stages including co-simulation suite.
- Update handoff/MANIFEST.md with all 28 file checksums.
- Generate handoff/PHASE1_COSIM_REPORT.md.
```
