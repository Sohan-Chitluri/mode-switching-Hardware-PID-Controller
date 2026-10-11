# Design Manifest & File Integrity Catalog: `pid_top` & `pid_soc_top`

**Project:** Mode-Switching Hardware PID Flight Controller  
**Top-Level Modules:** `pid_top` (Core 4-Axis Controller), `pid_soc_top` (SoC Wrapper + Quad-X Mixer)  
**Target Environment:** Cadence Digital Flow (Genus Synthesis + Innovus P&R)  
**Verification Baseline:** Verilator 5.052, Python 3.14, SymbiYosys + Z3 SMT Solver, Yosys 0.69  
**Date:** October 2026  
**Branch:** `audit/phase4-signoff`  
**Status:** Verification Signed-Off (All 10 Regression Stages Passing; 100% Bit-Exact Equivalence; 10-Step Formal BMC Proven; 4-DOF Plant Stable; Phase 4 Audit Complete)

---

## 1. Synthesizable RTL Source Files & Checksums

| Relative File Path | Component Description | Language Subset | SHA-256 Checksum |
| :--- | :--- | :--- | :--- |
| `rtl/pkg/pid_pkg.sv` | Global types, mode encodings, fault constants, math helpers | IEEE 1800-2012 | `e532f22dba7893c61643396f102aed2654b2cf07fec71896ef55a48e85fe9834` |
| `rtl/core/pid_mult.sv` | Q16.16 signed fixed-point multiplier with truncation | IEEE 1800-2012 | `7af2386fd65b844f61a1af76ef0659d3a37abb839765123a040f22faf84298ee` |
| `rtl/core/pid_deriv_filter.sv` | Derivative-on-measurement unit with 1st-order IIR LPF | IEEE 1800-2012 | `d0cc3a0de618fa3e300f98d534dabb2ca23bf5aa3d710bb3d297eb13f9a54126` |
| `rtl/core/pid_integ_unit.sv` | Guarded 33-bit accumulator with anti-windup clamping & re-seed | IEEE 1800-2012 | `622a267be0d1a3d0c1f0f21c6e9608298d76296e0be673210585cfe8d996ff0b` |
| `rtl/core/pid_sat_unit.sv` | 48-bit guarded summation & actuator clamping unit | IEEE 1800-2012 | `33d46396e6304e4ad156acf5c5a86be80afccb09fb93ac361d900cbf3114f68c` |
| `rtl/core/pid_axis_core.sv` | Single-axis PID engine (Roll, Pitch, Yaw, Altitude) | IEEE 1800-2012 | `6042665cd18a29d51cd3f89b56a7856ca5b686f33844032c0081cb34015402b6` |
| `rtl/modes/pid_outer_loop.sv` | Cascaded attitude angle-to-rate & altitude velocity generators | IEEE 1800-2012 | `e08495c544be72ff004e649bc1e53998b41a0b17f015139ddee745c19cbcaada` |
| `rtl/modes/pid_gain_bank.sv` | Mode-specific bank registers & configuration decoder with limit guards | IEEE 1800-2012 | `e5e3634553175033f202e89c53aef631441fd57d4f888ad2b3f6c160cfbcb45f` |
| `rtl/modes/pid_mode_mgr.sv` | Operating mode FSM, graceful degradation, and fault latching | IEEE 1800-2012 | `f15bb02b1a5c3cc3bf8d13166db5bd2b974a5e966a667b8d45a6aada4d3f83f4` |
| `rtl/safety/pid_tilt_comp.sv` | Quadratic Taylor series attitude tilt compensation ($1/\cos\theta$) | IEEE 1800-2012 | `2cdc8b71b5f65aa35f7afceb63af396957f6a2902b63dfc2f46ef419ae92c2e4` |
| `rtl/safety/pid_watchdog.sv` | Sensor freshness watchdog timer with programmable timeout | IEEE 1800-2012 | `6b146f8bd97bc1a479133e7149c6727aa9028dd176903df026c11e6c36dc9fa4` |
| `rtl/safety/pid_output_stage.sv` | Actuator registered output stage & failsafe override mux | IEEE 1800-2012 | `89ee9ec5a65c6a1f8748137836bd2af756a8d90cb0af9014eda046bf59dda936` |
| `rtl/pid_top.sv` | Full-chip 4-axis top integration & bumpless handover logic | IEEE 1800-2012 | `68939be4dd4c9afc4a6f8eeeab52b97553c58154e633b58f890136b9e67c21f3` |
| `rtl/safety/pid_mixer.sv` | Quad-X motor mixer with orthogonal mapping & saturation guarding | IEEE 1800-2012 | `08550fed87a69c661458f3b79aeda3a24f615479774d30391825f8421b3a75e7` |
| `rtl/pid_soc_top.sv` | Synthesizable SoC integration top wrapper with anti-aliasing MMIO bus | IEEE 1800-2012 | `0b07ea09c076ced8718a922d265f31f3da8a878f456b20d686c010cdb82d041a` |

---

## 2. Timing Constraints & Scripts

| Relative File Path | Component Description | Format | SHA-256 Checksum |
| :--- | :--- | :--- | :--- |
| `constraints/pid_timing.sdc` | 50 MHz primary clock, I/O delays, asynchronous failsafe false path | Synopsys SDC | `1f0ca0bbdcea37db5fd23cc248780f88639e16b67181b86f412cfd12f82c7c52` |
| `handoff/CADENCE_IMPORT_GUIDE.md` | Comprehensive Cadence import and flow instructions | Markdown Doc | `b65ea09456ee4deea07646491301e44f259494b27c7d75c5a034cd6ad4970899` |
| `handoff/genus_synth.tcl` | Portable Cadence Genus synthesis script template | Tcl Script | `d73dfdab6bd63983e872f909556d53515ce76563e0d5778e35a3358993511a65` |
| `handoff/innovus_pnr.tcl` | Portable Cadence Innovus P&R execution script template | Tcl Script | `dc5f731746e96f76c9c775aea2476c24753051f41a775767326351e96d753fb3` |
| `scripts/genus_synth.tcl` | Workstation Cadence Genus synthesis execution script (180nm flow) | Tcl Script | `ee0b52261cc10de3ef1e9c59b41d893627ac5f5355cabd483ce15d7ff5fd0e30` |
| `scripts/innovus_pnr.tcl` | Workstation Cadence Innovus P&R execution script (180nm MMMC) | Tcl Script | `19d41025d2625e4f12c5349e0004e7186cacccc6ac029872686e3132c5573c7e` |
| `scripts/pid_mmmc.tcl` | Multi-Mode Multi-Corner timing configuration for Innovus | Tcl Script | `18354ada113c85ba7dc95d88f63ba9b0e543013a1b7152f2e48095f5fbea7bde` |
| `scripts/innovus_timing_reports.tcl` | Innovus sign-off setup & hold timing reporting snippet | Tcl Script | `80075e34e7ceea6be6b52e28a6fe43c9c32b2edbbc7a5a8f7fb725012bd4f768` |
| `scripts/innovus_pnr_fast.tcl` | Fast-track Innovus P&R execution script (0.40 utilization) | Tcl Script | `bcd13a975da586c470672de1654242666fea29807ffa3de5604901091b61383f` |
| `scripts/run_regression.sh` | End-to-end regression test suite runner (10 stages) | Bash Script | `09ee8e5d1b9cd3599c69a7813aea3dd1ecea8b1702153a886b8d4ebc5501e963` |
| `scripts/prelim_synth.tcl` | Yosys structural synthesis and elaboration check script | Tcl Script | `306a9f64c9620aacdb802fcf0c42b9fbae11d72678616c335efaa2a45a8ac166` |

---

## 3. Verification & Reference Artifacts

| Relative File Path | Component Description | SHA-256 Checksum |
| :--- | :--- | :--- |
| `models/pid_golden_model.py` | Bit-accurate reference simulator & fixed-point golden model | `872e4f3004370489aa8c2efdc145ecf17b4ed48bc8e04120d14de5ff6777a00c` |
| `models/flight_plant_sim.py` | 4-DOF non-linear quadrotor dynamics plant simulator (roll, pitch, yaw, z) | `9f9b392f0ecbca4a13cd4fccaca1ee5283af5637e59877c8b0e6e9b0873342e2` |
| `models/flight_plant_cosim.py` | True closed-loop RTL-to-plant co-simulation harness | `0652e44076390ee1c56888c705f2e374771d25330dd4c4a24fff1b7fca0b4cff` |
| `models/generate_vectors.py` | Test vector generator (20,111 total vectors across unit and top) | `c394008936338050522f5df63e57e1c110ac9dcae954b974387c2ce31196e6e5` |
| `tb/unit/pid_axis_core_tb.sv` | Unit testbench for `pid_axis_core` (10,111 vectors) | `5aa2783b12b76a174c36e88524493fd89dc0c01f11a039364eb11d53a19c7cd2` |
| `tb/top/pid_top_tb.sv` | Top-level full-chip testbench (10,000 vectors + directed tests) | `c85a3e0942b74228fbbbbf1a0f69e26ee6db58bf76dd9bc18f55beaef37e3b86` |
| `tb/unit/pid_mixer_tb.sv` | Quad-X motor mixer unit testbench (14 suites) | `79d3a6624ba1274f08f367fa2c8c4f951af445583b25f84dc53a7af16b9dc8aa` |
| `tb/top/pid_soc_top_tb.sv` | SoC top MMIO bus integration testbench (6 suites) | `00bca43c43b3e5ac68ca2c795d6fd6c0a99edd91b5eb96c556576ab462c7f01a` |
| `tb/top/pid_mode_stress_tb.sv` | 12 mode transitions & flight-safety stress testbench (20 suites) | `7c3abbf50aa12f743d0dbb4f9a6788b2c81606aa4d58f5ef193c25c0f50be488` |
| `tb/cosim/pid_top_cosim.cpp` | C API wrapper around Verilated Vpid_top for Python ctypes cosimulation | `cebbcafae967a01cf3f0b055a89c333ee0936a0d62accb7b2c30a29e4809e0fd` |
| `tb/formal/pid_formal_tb.sv` | Formal testbench harness for P1-P5 invariant properties | `8c423112d66b280cc85183196d0c1b5916013d19fa640fbb3080b601265ea60e` |
| `tb/formal/pid_formal.sby` | SymbiYosys configuration file for Z3 SMT solver | `a14564abfeb1641e23ed41c20dae1944b7a8abcf66e70cae592af2f36410d98a` |
| `handoff/PHASE0_REMEDIATION_REPORT.md` | Phase 0 baseline verification & audit remediation report | `755d8825de10da0df74778feac39b63bfa31e1c171c53ac5b4dc92d7c95c9ab7` |
| `handoff/PHASE1_COSIM_REPORT.md` | Phase 1 closed-loop co-sim & dynamic plant tuning report | `e3d0a5925bb47f99b63799f4ed089eaaa13226bb63239440bd0c0206e1f93b91` |
| `handoff/PHASE2_REMEDIATION_REPORT.md` | Phase 2 robustness, safety remediation, and thrust routing report | `e5652b7be6985d1a8cadecb66f95db1c2085c2b7f109dff090d760e6707a94f6` |
| `handoff/PHASE3_REMEDIATION_REPORT.md` | Phase 3 mode transition, safety stress, and SoC integration report | `84058992db674849dcfb7ce03d557f910e94df938e1b68db73321f5373ccb60c` |
| `handoff/PHASE4_SIGNOFF_AUDIT.md` | Phase 4 independent sign-off audit & physical implementation readiness report | `a37a622cf85bd6d167201066a6a6ceba9621e837872d8661d34b5deab063ebd4` |

---

## 4. Design Hierarchy Tree

```
pid_soc_top (Optional SoC Top Integration Wrapper)
├── MMIO Synchronous Bus Decoder (0x000..0x01F control/status, 0x100..0x19F gain pass-through)
├── u_pid_top (pid_top)
│   ├── u_watchdog (pid_watchdog)
│   │     └── Sample freshness counter, timeout comparator & alarm register
│   ├── u_mode_mgr (pid_mode_mgr)
│   │     └── Mode FSM, re-arm resolution, fault latching & transition strobe
│   ├── u_gain_bank (pid_gain_bank)
│   │     └── 8-bit memory-mapped configuration registers & mode bank selector
│   ├── u_outer_loop (pid_outer_loop)
│   │     ├── err_phi / err_theta attitude angle comparators
│   │     ├── u_mult_att_roll (pid_mult)
│   │     ├── u_mult_att_pitch (pid_mult)
│   │     ├── u_mult_alt_pos (pid_mult)
│   │     └── Muxes for Acro vs Stabilize vs AltHold vs Failsafe setpoint routing
│   ├── u_core_roll (pid_axis_core)
│   │     ├── u_deriv_filter (pid_deriv_filter)
│   │     │     ├── u_alpha_mult (pid_mult)
│   │     │     └── u_kd_mult (pid_mult)
│   │     ├── u_kp_mult (pid_mult)
│   │     ├── u_integ_unit (pid_integ_unit)
│   │     │     └── u_ki_mult (pid_mult)
│   │     └── u_sat_unit (pid_sat_unit)
│   ├── u_core_pitch (pid_axis_core)
│   │     └── [Identical parallel architecture as Roll]
│   ├── u_core_yaw (pid_axis_core)
│   │     └── [Identical parallel architecture as Roll]
│   ├── u_core_thrust (pid_axis_core)
│   │     └── [Identical parallel architecture as Roll]
│   ├── u_tilt_comp (pid_tilt_comp)
│   │     ├── u_mult_phi_sq (pid_mult)
│   │     ├── u_mult_th_sq (pid_mult)
│   │     └── u_mult_tilt_scale (pid_mult)
│   └── u_output_stage (pid_output_stage)
│         └── Registered output drivers with single-cycle deterministic Failsafe override
└── u_pid_mixer (pid_mixer)
      └── 4-axis orthogonal Quad-X mixer with 48-bit intermediate guarding & [0.0, 1.0] clamping
```

---

## 5. Module Latency & Clock Contracts

- **Core Latency:** 1 clock cycle registered interface latency from `sample_valid` strobe to registered actuator output update in `pid_top`.
- **Mixer Latency:** 1 clock cycle registered interface latency from `controller_valid` to `motor_valid` in `pid_mixer`.
- **Clock Frequency Target:** 50 MHz nominal ($T_{clk} = 20.0\text{ ns}$).
- **Sample Rate Target:** 4 kHz typical flight controller loop ($T_{sample} = 250\ \mu\text{s} = 12,500\text{ clock cycles}$).
- **Reset Convention:** Fully synchronous active-low assertion and deassertion (`!rst_n`) across all internal registers.
- **Emergency Kill Latency:** Exactly 3 clock cycles (60.0 ns at 50 MHz) from external active-low pin assertion (`!failsafe_ext_n`) to output pin clamp (2-stage flip-flop CDC synchronizer + 1-cycle output stage registration). In `pid_soc_top`, motor outputs update in 4 clock cycles due to mixer output stage registration.
- **Arithmetic Representation:** Signed two's-complement Q16.16 fixed-point throughout all control loops and mixer stages.
