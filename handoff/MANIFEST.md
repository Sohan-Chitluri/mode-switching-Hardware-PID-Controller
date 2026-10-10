# Design Manifest & File Integrity Catalog: `pid_top` & `pid_soc_top`

**Project:** Mode-Switching Hardware PID Flight Controller  
**Top-Level Modules:** `pid_top` (Core 4-Axis Controller), `pid_soc_top` (SoC Wrapper + Quad-X Mixer)  
**Target Environment:** Cadence Digital Flow (Genus Synthesis + Innovus P&R)  
**Verification Baseline:** Verilator 5.052, Python 3.14, SymbiYosys + Z3 SMT Solver, Yosys 0.69  
**Date:** October 2026  
**Branch:** `feat/phase3-verification-soc`  
**Status:** Verification Signed-Off (All 10 Regression Stages Passing; 100% Bit-Exact Equivalence; 10-Step Formal BMC Proven; 4-DOF Plant Stable; Phase 3 Complete)

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
| `rtl/modes/pid_gain_bank.sv` | Mode-specific bank registers & configuration decoder | IEEE 1800-2012 | `b764f6e262c6f177bd27fb7f7538e90ef142c6f1067828ee12865b5002b263e3` |
| `rtl/modes/pid_mode_mgr.sv` | Operating mode FSM, graceful degradation, and fault latching | IEEE 1800-2012 | `f15bb02b1a5c3cc3bf8d13166db5bd2b974a5e966a667b8d45a6aada4d3f83f4` |
| `rtl/safety/pid_tilt_comp.sv` | Quadratic Taylor series attitude tilt compensation ($1/\cos\theta$) | IEEE 1800-2012 | `2cdc8b71b5f65aa35f7afceb63af396957f6a2902b63dfc2f46ef419ae92c2e4` |
| `rtl/safety/pid_watchdog.sv` | Sensor freshness watchdog timer with programmable timeout | IEEE 1800-2012 | `6b146f8bd97bc1a479133e7149c6727aa9028dd176903df026c11e6c36dc9fa4` |
| `rtl/safety/pid_output_stage.sv` | Actuator registered output stage & failsafe override mux | IEEE 1800-2012 | `89ee9ec5a65c6a1f8748137836bd2af756a8d90cb0af9014eda046bf59dda936` |
| `rtl/pid_top.sv` | Full-chip 4-axis top integration & bumpless handover logic | IEEE 1800-2012 | `68939be4dd4c9afc4a6f8eeeab52b97553c58154e633b58f890136b9e67c21f3` |
| `rtl/safety/pid_mixer.sv` | Quad-X motor mixer with orthogonal mapping & saturation guarding | IEEE 1800-2012 | `08550fed87a69c661458f3b79aeda3a24f615479774d30391825f8421b3a75e7` |
| `rtl/pid_soc_top.sv` | Synthesizable SoC integration top wrapper with MMIO register bus | IEEE 1800-2012 | `d48e3ce629992299c9abc94baf79f60baba435079b473c16e631a0913106ca90` |

---

## 2. Timing Constraints & Scripts

| Relative File Path | Component Description | Format | SHA-256 Checksum |
| :--- | :--- | :--- | :--- |
| `constraints/pid_timing.sdc` | 50 MHz primary clock, I/O delays, false paths | Synopsys SDC | `ebf28f3a4254aa01bd2b12bd4dd197e2e1aae4066742e32ff4dcb30b5681231d` |
| `handoff/CADENCE_IMPORT_GUIDE.md` | Comprehensive Cadence import and flow instructions | Markdown Doc | `a192d9af9a0c35951c89501ff80fd55826ad51bbe8a795d727d58c4152d0936b` |
| `handoff/genus_synth.tcl` | Cadence Genus synthesis execution script | Tcl Script | `4b6e5255bbbcf54b00094bd9c8ce37f217dfc241e733e2b03f16e7a800d2a702` |
| `handoff/innovus_pnr.tcl` | Cadence Innovus place-and-route script template | Tcl Script | `8f4247c9c691ed46017e77975c87de7c65b6f4efbf271fb088e092305d64f786` |
| `scripts/run_regression.sh` | End-to-end regression test suite runner (10 stages) | Bash Script | `ea5b00f93efbd0f308ff27c82880a95716fac7ce86cc58bafd2911fb50942b15` |
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
| `tb/unit/pid_mixer_tb.sv` | Quad-X motor mixer unit testbench (7 suites) | `c27ae2a0cba83abd1aff7b612a92ba614e07de188128485bd563a000b2587cf8` |
| `tb/top/pid_soc_top_tb.sv` | SoC top MMIO bus integration testbench (6 suites) | `4c47bc6d4d941a4816059f7047816ee2d314df2b5ea8efa30cdf7fce7ee29cb0` |
| `tb/top/pid_mode_stress_tb.sv` | 12 mode transitions & flight-safety stress testbench (20 suites) | `065ff65aa0fdac8b11862c0f9e2e6a73c55e2799965cb1c9e4218be22a052f62` |
| `tb/cosim/pid_top_cosim.cpp` | C API wrapper around Verilated Vpid_top for Python ctypes cosimulation | `cebbcafae967a01cf3f0b055a89c333ee0936a0d62accb7b2c30a29e4809e0fd` |
| `tb/formal/pid_formal_tb.sv` | Formal testbench harness for P1-P5 invariant properties | `8c423112d66b280cc85183196d0c1b5916013d19fa640fbb3080b601265ea60e` |
| `tb/formal/pid_formal.sby` | SymbiYosys configuration file for Z3 SMT solver | `a14564abfeb1641e23ed41c20dae1944b7a8abcf66e70cae592af2f36410d98a` |
| `handoff/PHASE1_COSIM_REPORT.md` | Phase 1 closed-loop co-sim & dynamic plant tuning report | `e3d0a5925bb47f99b63799f4ed089eaaa13226bb63239440bd0c0206e1f93b91` |
| `handoff/PHASE2_REMEDIATION_REPORT.md` | Phase 2 robustness, safety remediation, and thrust routing report | `e5652b7be6985d1a8cadecb66f95db1c2085c2b7f109dff090d760e6707a94f6` |
| `handoff/PHASE3_REMEDIATION_REPORT.md` | Phase 3 mode transition, safety stress, and SoC integration report | `84058992db674849dcfb7ce03d557f910e94df938e1b68db73321f5373ccb60c` |

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
- **Emergency Kill Latency:** Exactly 3 clock cycles (60.0 ns at 50 MHz) from external active-low pin assertion (`!failsafe_ext_n`) to output pin clamp (2-stage flip-flop CDC synchronizer + 1-cycle output stage registration).
- **Arithmetic Representation:** Signed two's-complement Q16.16 fixed-point throughout all control loops and mixer stages.
