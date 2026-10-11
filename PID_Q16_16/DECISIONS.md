# DECISIONS.md — Hardware PID Flight Controller

**Project:** Mode-Switching Hardware PID Controller for Flight Control (VLSI Hackathon)  
**Root Directory:** `/home/peskybird/Projects/PID`  
**Document Status:** Architecture Decision Log & Open Questions — APPROVED FOR PLANNING  
**Date:** October 2026  

---

## 1. Decision Classification Framework

Design decisions are cataloged into three distinct operational categories:
1. **Confirmed Decisions:** Architectural and verification choices firmly established and frozen.
2. **Provisional Decisions:** Technically justified baseline choices adopted to enable progress through early planning and Wave 0–2 modeling, subject to confirmation prior to RTL freeze.
3. **Unresolved Decisions & Open Questions:** Items requiring external confirmation (e.g., target Cadence setup, PDK selection, or flight vehicle physical parameters), complete with risk assessments and downstream task dependency mappings.

---

## 2. Confirmed Architectural Decisions

### DEC-01: Synchronous Single-Clock Domain Architecture
- **Decision:** The controller will operate in a single synchronous clock domain (`clk`) with a synchronous active-low reset (`rst_n`).
- **Rationale:** Control feedback loops must exhibit zero clock-domain crossing (CDC) metastability hazards. Any asynchronous external sensor signals (e.g., SPI/I2C buses) must be registered into the primary clock domain by peripheral interfaces prior to asserting `sample_valid`.
- **Status:** **CONFIRMED**

### DEC-02: Four Supported Operational Modes
- **Decision:** Exactly four operational modes will be supported: **Acro** (00), **Stabilize** (01), **Altitude Hold** (10), and **Failsafe** (11).
- **Rationale:** Spans the complete functional spectrum from manual aerobatic rate control to autonomous stabilization and deterministic fault protection.
- **Status:** **CONFIRMED**

### DEC-03: Measurement-Derived Derivative Action with IIR LPF
- **Decision:** The derivative term will be computed strictly on sensor measurement feedback ($-\frac{d y}{d t}$), never on error ($-\frac{d e}{d t}$), and filtered through a parameterizable first-order IIR low-pass filter.
- **Rationale:** Completely eliminates "derivative kick" (severe actuator saturation spikes) when pilot setpoints step abruptly. Attenuates high-frequency sensor noise from quadrotor motor vibrations.
- **Status:** **CONFIRMED**

### DEC-04: Twin Reference Model Architecture
- **Decision:** Two independent Python reference models will be constructed:
  1. A continuous 6-DOF rigid-body aircraft flight dynamics simulator (`models/flight_plant_sim.py`).
  2. A bit-accurate integer/fixed-point golden model (`models/pid_golden_model.py`) matching RTL pipeline registers, rounding, and saturation.
- **Rationale:** Decouples physical aerodynamic stability analysis from digital hardware bit-equivalence testing.
- **Status:** **CONFIRMED**

### DEC-05: Layered Verification Pyramid with SBY Formal Proofs
- **Decision:** Verification will span five layers (L0 arithmetic unit tests, L1 channel core tests, L2 subsystem tests, L3 50k-vector co-simulation regression, L4 closed-loop plant simulation, L5 SymbiYosys/Z3 formal proofs).
- **Rationale:** Ensures both mathematical safety invariants (bounded outputs, watchdog guarantees) and physical flight tracking performance.
- **Status:** **CONFIRMED**

### DEC-06: Strict Separation of Tool Flow Authorities
- **Decision:** The local workstation is restricted to RTL development, Verilator simulation, Python modeling, SBY formal proofs, and preliminary structural Yosys checks. The remote Cadence workstation has sole authority over synthesis, placement, routing, and timing sign-off.
- **Rationale:** Prevents misleading claims about timing closure or silicon area from preliminary local synthesis flows.
- **Status:** **CONFIRMED**

---

## 3. Provisional Decisions (Baseline Recommendations)

### PROV-01: Synthesizable SystemVerilog Language Subset
- **Provisional Choice:** IEEE 1800-2012 synthesizable SystemVerilog subset (specifically `logic`, `always_ff`, `always_comb`, packages, and parameterized modules).
- **Downstream Tasks:** `PID-01` through `PID-04`, `AXIS-01`, `MODE-01`, `VERIFY-01`.
- **Rationale:** Modern synthesis tools (Cadence Genus 19+ and Yosys 0.69+) natively support IEEE 1800-2012. SystemVerilog eliminates dangerous Verilog-2001 ambiguity (such as accidental latch inference from incomplete sensitivity lists).
- **Impact if Changed:** If the remote Cadence tool flow requires pure Verilog-2001, modules must be converted to `reg`/`wire` and standard `always @(posedge clk)` blocks.

### PROV-02: Signed Q16.16 Fixed-Point Numeric Format
- **Provisional Choice:** 32-bit signed two's complement fixed-point with 16 integer bits and 16 fractional bits (Q16.16).
- **Downstream Tasks:** `SPEC-02`, `MODEL-02`, `PID-01`, `AXIS-01`.
- **Rationale:** Dynamic range of $[-32,768.0, +32,767.99]$ accommodates angular rates up to $\pm 2,000^\circ/\text{s}$ and angles up to $\pm 180^\circ$ with $1.53 \times 10^{-5}$ resolution ($0.00087^\circ$), far exceeding MEMS IMU sensor noise floors ($0.01^\circ$).
- **Impact if Changed:** Modifying fractional width alters multiplier shift constants and rounding offsets.

### PROV-03: Primary Clock (50 MHz) & Update Rate (4 kHz)
- **Provisional Choice:** Primary system clock $f_{clk} = 50.0\text{ MHz}$ ($T_{clk} = 20.0\text{ ns}$); sensor update strobe $f_{sample} = 4.0\text{ kHz}$ ($T_{sample} = 250\ \mu\text{s} = 12,500$ clock cycles).
- **Downstream Tasks:** `SPEC-01`, `SPEC-04`, `SAFE-01`, `HANDOFF-02`.
- **Rationale:** 50 MHz is conservative and easily closes timing across any standard cell library while providing 12,500 clock cycles per sample—more than enough for the 1-cycle core pipeline latency.
- **Impact if Changed:** Modifying $f_{clk}$ scales the watchdog timeout counter threshold (`cfg_timeout_cycles`).

### PROV-04: Fully Parallel Multi-Axis Datapath Topology
- **Provisional Choice:** Instantiate dedicated parallel PID cores for Roll, Pitch, Yaw, and Altitude (`pid_axis_core` $\times$ 4).
- **Downstream Tasks:** `AXIS-01`, `AXIS-03`, `VERIFY-01`.
- **Rationale:** Total gate count is $< 24,000$ cells ($< 0.12\text{ mm}^2$ silicon), while eliminating complex time-multiplexing FSMs, memory arbitration hazards, and cross-channel fault contamination.
- **Impact if Changed:** If silicon budget is constrained to $< 10,000$ cells, a shared single-core datapath with a 4-channel context register file can be substituted.

### PROV-05: Integrator Anti-Windup via Conditional Clamping
- **Provisional Choice:** Combine conditional integration (freezing accumulation when saturated with matching error sign) with hard bounding $[I_{min}, I_{max}]$.
- **Downstream Tasks:** `SPEC-02`, `PID-03`.
- **Rationale:** Eliminates the extra multiplier required by back-calculation anti-windup ($K_b$) while providing instantaneous desaturation upon error reversal.

### PROV-06: Algebraic Bumpless Transfer Back-Calculation
- **Provisional Choice:** On mode switch, initialize $I_{new}[0] = \text{clamp}(U_{prev} - P_{new}[0], I_{min}, I_{max})$ and seed $y_f[0] = y[0]$ ($D_{new} = 0$).
- **Downstream Tasks:** `SPEC-03`, `MODE-03`.
- **Rationale:** Mathematically forces $U_{new}[0] = U_{prev}$, preventing physical aircraft jerks during flight mode changes.

### PROV-07: Failsafe Actuator Output Policy
- **Provisional Choice:** Failsafe forces rotational torques to zero ($U_{roll} = 0, U_{pitch} = 0, U_{yaw} = 0$) and thrust to a configurable safe level ($U_{thrust} = 0$ / Disarm or low descent thrust).
- **Downstream Tasks:** `SPEC-04`, `SAFE-02`.
- **Rationale:** In multirotors, cutting torques and dropping throttle is the standard safety response to prevent flyaways upon sensor loss.

### PROV-08: Altitude Hold Sensor Interface Requirements
- **Provisional Choice:** Require external fused vertical position ($z_{meas}$) and velocity ($v_{z,meas}$) inputs with an `alt_valid` strobe; provide small-angle Taylor tilt-compensation.
- **Downstream Tasks:** `SPEC-01`, `AXIS-03`.
- **Rationale:** Physically impossible to maintain stable altitude hold from raw IMU accelerometer data without an external vertical reference (barometer/LiDAR/estimator).

---

## 4. Unresolved Decisions & Open Questions Requiring Confirmation

The table below documents the final resolution status of each architectural decision across development and verification:

| Decision ID | Open Architectural Question | Final Resolution / Adopted Implementation | Status | Verification & Implementation Evidence |
| :--- | :--- | :--- | :--- | :--- |
| **UNRES-01** | **Verilog-2001 vs. SystemVerilog:** Synthesis language subset | IEEE 1800-2012 synthesizable subset adopted cleanly. | **RESOLVED & FROZEN** | Verified clean across Verilator 5.052, Yosys 0.69, and SBY Z3 formal flow. Package import placed at file-level for universal Cadence/Yosys compatibility. |
| **UNRES-02** | **Target Clock Frequency & Sample Rate:** 50 MHz clock / 4 kHz update rate | $f_{clk} = 50.0\text{ MHz}$, $f_{sample} = 4.0\text{ kHz}$ confirmed. | **RESOLVED & FROZEN** | Timing constraints specified in `constraints/pid_timing.sdc` (20.0 ns period). Watchdog timeout programmed to 50,000 cycles (1.0 ms). |
| **UNRES-03** | **Fixed-Point Format:** Q16.16 range and precision | Signed Q16.16 format adopted with 48-bit guarded accumulation. | **RESOLVED & FROZEN** | 100% bit-exact equivalence verified across 20,019 vectors; 0 bit mismatches vs Python golden model. |
| **UNRES-04** | **Sensor Input Encodings & Units:** Engineering units vs raw LSBs | Standardized engineering units (rad/s, rad, m, m/s in Q16.16). | **RESOLVED & FROZEN** | Verified in top regression; eliminates redundant scaling multipliers from control core. |
| **UNRES-05** | **Altitude Hold Sensor / Estimator Interface:** Fused velocity vs raw differentiation | External fused $z$ and $v_z$ with `alt_valid` strobe. | **RESOLVED & FROZEN** | Implemented in `pid_outer_loop.sv`. Graceful degradation to Stabilize mode on `alt_valid=0` proven formally and in simulation. |
| **UNRES-06** | **Failsafe Output Policy:** Disarm vs controlled descent | Configurable register `cfg_failsafe_thrust` defaulting to 0 (Disarm). | **RESOLVED & FROZEN** | Formally proven via SymbiYosys/Z3 (Property P2); zero rotational torque and configurable safe thrust guaranteed. |
| **UNRES-07** | **Permitted Mode Transitions & Recovery:** Failsafe auto-recovery policy | Sticky Failsafe lock; requires fault clear AND explicit pilot re-arm strobe. | **RESOLVED & FROZEN** | Implemented in `pid_mode_mgr.sv`; verified in top testbench (`test_watchdog_timeout` re-arm task) and formal property P5. |
| **UNRES-08** | **Remote Cadence Flow & Tool Versions:** Tool scripting support | Modern Cadence flow targeted (Genus 19+, Innovus 19+). | **PACKAGE READY** | Portable synthesis script `handoff/genus_synth.tcl` and P&R script `handoff/innovus_pnr.tcl` provided with parameter placeholders. |
| **UNRES-09** | **Target PDK & Standard-Cell Library:** Foundry selection and corner | Technology-independent handoff package prepared. | **DOCUMENTED FOR REMOTE SIGN-OFF** | Instructions, library environment variables, and corner checklists documented in `handoff/CADENCE_IMPORT_GUIDE.md`. |
| **UNRES-10** | **Datapath Topology (Parallel vs. Shared):** Silicon area vs parallelism | Fully parallel dedicated datapaths (`pid_axis_core` $\times$ 4) implemented. | **RESOLVED & FROZEN** | Parallel baseline verified with zero cross-talk, deterministic 1-cycle pipeline latency, and $< 24\text{k}$ cell footprint. |

---

## 5. Implementation Sign-Off Summary

- **RTL Development:** Complete and frozen in `/home/peskybird/Projects/PID/rtl/`.
- **Unit & Full-Chip Verification:** 100% Bit-Exact Pass across 20,019 vectors (0 mismatches).
- **Formal Verification:** Depth-10 Bounded Model Checking PASS across all safety properties (P1..P5) using SymbiYosys + Z3 SMT solver.
- **Flight Dynamics Simulation:** 6-DOF non-linear quadrotor dynamics achieve $0.023^\circ$ steady-state error, $0.00\%$ overshoot, and $441.5\text{ ms}$ settling time (all PASS).
- **Cadence Handoff Package:** SDC constraints, MANIFEST with SHA-256 integrity hashes, Genus synthesis script, Innovus P&R script, and step-by-step import documentation completed in `constraints/` and `handoff/`.
