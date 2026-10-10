# Design Manifest & File Integrity Catalog: `pid_top`

**Project:** Mode-Switching Hardware PID Flight Controller  
**Top-Level Module:** `pid_top`  
**Target Environment:** Cadence Digital Flow (Genus Synthesis + Innovus P&R)  
**Verification Baseline:** Verilator 5.050, Python 3.14, SymbiYosys + Z3 SMT Solver, Yosys 0.68  
**Date:** October 2026  
**Branch:** `fix/audit-remediation`  
**Status:** Verification Signed-Off (100% Bit-Exact Equivalence across 20,111 vectors; 10-Step Formal BMC Proven; 4-DOF Plant Stable)

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
| `rtl/modes/pid_gain_bank.sv` | Mode-specific bank registers & configuration decoder | IEEE 1800-2012 | `a7f26d03e673c5d0ad07eb77e6d631d374cfcf8135e1495f8dedd428c75c4fc9` |
| `rtl/modes/pid_mode_mgr.sv` | Operating mode FSM, graceful degradation, and fault latching | IEEE 1800-2012 | `f15bb02b1a5c3cc3bf8d13166db5bd2b974a5e966a667b8d45a6aada4d3f83f4` |
| `rtl/safety/pid_tilt_comp.sv` | Quadratic Taylor series attitude tilt compensation ($1/\cos\theta$) | IEEE 1800-2012 | `2cdc8b71b5f65aa35f7afceb63af396957f6a2902b63dfc2f46ef419ae92c2e4` |
| `rtl/safety/pid_watchdog.sv` | Sensor freshness watchdog timer with programmable timeout | IEEE 1800-2012 | `6b146f8bd97bc1a479133e7149c6727aa9028dd176903df026c11e6c36dc9fa4` |
| `rtl/safety/pid_output_stage.sv` | Actuator registered output stage & failsafe override mux | IEEE 1800-2012 | `ca0eef5a1403206332ef7c52bd20c46655c900e2d9cb2c3a60baaf3cb43eeefc` |
| `rtl/pid_top.sv` | Full-chip 4-axis top integration & bumpless handover logic | IEEE 1800-2012 | `0166c288eaed1ebd4e8de7152980c3aa1dd0f7d99714f3aa12ed6ee270846237` |

---

## 2. Timing Constraints & Scripts

| Relative File Path | Component Description | Format | SHA-256 Checksum |
| :--- | :--- | :--- | :--- |
| `constraints/pid_timing.sdc` | 50 MHz primary clock, I/O delays, false paths | Synopsys SDC | `ebf28f3a4254aa01bd2b12bd4dd197e2e1aae4066742e32ff4dcb30b5681231d` |
| `handoff/CADENCE_IMPORT_GUIDE.md` | Comprehensive Cadence import and flow instructions | Markdown Doc | `a192d9af9a0c35951c89501ff80fd55826ad51bbe8a795d727d58c4152d0936b` |
| `handoff/genus_synth.tcl` | Cadence Genus synthesis execution script | Tcl Script | `4b6e5255bbbcf54b00094bd9c8ce37f217dfc241e733e2b03f16e7a800d2a702` |
| `handoff/innovus_pnr.tcl` | Cadence Innovus place-and-route script template | Tcl Script | `8f4247c9c691ed46017e77975c87de7c65b6f4efbf271fb088e092305d64f786` |
| `scripts/run_regression.sh` | End-to-end regression test suite runner (6 stages) | Bash Script | `3d81896017f881c5f2c69741e57bb87b673639eb1ee26839b57ad78a6c80eaa3` |
| `scripts/prelim_synth.tcl` | Yosys structural synthesis and elaboration check script | Tcl Script | `03266d2d212c3de3a254b6d34f266731659eafe6eeea421b7e342924a1bb04d2` |

---

## 3. Verification & Reference Artifacts

| Relative File Path | Component Description | SHA-256 Checksum |
| :--- | :--- | :--- |
| `models/pid_golden_model.py` | Bit-accurate reference simulator & fixed-point golden model | `5c4a7a625f32fb5916acb700bc817459c734011e8c83fda1524f1e67ca26a1c5` |
| `models/flight_plant_sim.py` | 4-DOF non-linear quadrotor dynamics plant simulator (roll, pitch, yaw, z) | `95aa6576d3d93bcef04ed0915798f54e627f54eece8467f4f6d81107e8711c20` |
| `models/generate_vectors.py` | Test vector generator (20,111 total vectors across unit and top) | `c394008936338050522f5df63e57e1c110ac9dcae954b974387c2ce31196e6e5` |
| `tb/unit/pid_axis_core_tb.sv` | Unit testbench for `pid_axis_core` (10,111 vectors) | `5aa2783b12b76a174c36e88524493fd89dc0c01f11a039364eb11d53a19c7cd2` |
| `tb/top/pid_top_tb.sv` | Top-level full-chip testbench (10,000 vectors + directed tests) | `87befff81ae773bfa6a06de3069ae7dbed5b3afb517929b0ea3177e9b7a6573e` |
| `tb/formal/pid_formal_tb.sv` | Formal testbench harness for P1-P5 invariant properties | `e6e6b49148a52d8b0df6ddce8f1d2c07e861ee1783989dd2d4920d886cb6378d` |
| `tb/formal/pid_formal.sby` | SymbiYosys configuration file for Z3 SMT solver | `a14564abfeb1641e23ed41c20dae1944b7a8abcf66e70cae592af2f36410d98a` |

---

## 4. Design Hierarchy Tree

```
pid_top (Top Module)
├── u_watchdog (pid_watchdog)
│     └── Sample freshness counter, timeout comparator & alarm register
├── u_mode_mgr (pid_mode_mgr)
│     └── Mode FSM, re-arm resolution, fault latching & transition strobe
├── u_gain_bank (pid_gain_bank)
│     └── 8-bit memory-mapped configuration registers & mode bank selector
├── u_outer_loop (pid_outer_loop)
│     ├── err_phi / err_theta attitude angle comparators
│     ├── u_mult_att_roll (pid_mult)
│     ├── u_mult_att_pitch (pid_mult)
│     ├── u_mult_alt_pos (pid_mult)
│     └── Muxes for Acro vs Stabilize vs AltHold vs Failsafe setpoint routing
├── u_core_roll (pid_axis_core)
│     ├── u_deriv_filter (pid_deriv_filter)
│     │     ├── u_alpha_mult (pid_mult)
│     │     └── u_kd_mult (pid_mult)
│     ├── u_kp_mult (pid_mult)
│     ├── u_integ_unit (pid_integ_unit)
│     │     └── u_ki_mult (pid_mult)
│     └── u_sat_unit (pid_sat_unit)
├── u_core_pitch (pid_axis_core)
│     └── [Identical parallel architecture as Roll]
├── u_core_yaw (pid_axis_core)
│     └── [Identical parallel architecture as Roll]
├── u_core_thrust (pid_axis_core)
│     └── [Identical parallel architecture as Roll]
├── u_tilt_comp (pid_tilt_comp)
│     ├── u_mult_phi_sq (pid_mult)
│     ├── u_mult_th_sq (pid_mult)
│     └── u_mult_tilt_scale (pid_mult)
└── u_output_stage (pid_output_stage)
      └── Registered output drivers with single-cycle deterministic Failsafe override
```

---

## 5. Module Latency & Clock Contracts

- **Core Latency:** 1 clock cycle registered interface latency from `sample_valid` strobe to registered actuator output update.
- **Clock Frequency Target:** 50 MHz nominal ($T_{clk} = 20.0\text{ ns}$).
- **Sample Rate Target:** 4 kHz typical flight controller loop ($T_{sample} = 250\ \mu\text{s} = 12,500\text{ clock cycles}$).
- **Reset Convention:** Fully synchronous active-low assertion and deassertion (`!rst_n`) across all internal registers.
- **Emergency Kill Latency:** 3 clock cycles (60.0 ns at 50 MHz) from external active-low pin assertion (`!failsafe_ext_n`) to output pin clamp (2-stage flip-flop CDC synchronizer + 1-cycle output stage registration).
- **Arithmetic Representation:** Signed two's-complement Q16.16 fixed-point throughout all control loops.
