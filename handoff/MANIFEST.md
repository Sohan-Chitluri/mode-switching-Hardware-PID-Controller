# Design Manifest & File Integrity Catalog: `pid_top`

**Project:** Mode-Switching Hardware PID Flight Controller  
**Top-Level Module:** `pid_top`  
**Target Environment:** Cadence Digital Flow (Genus Synthesis + Innovus P&R)  
**Verification Baseline:** Verilator 5.052, Python 3.14, SymbiYosys + Z3 SMT Solver  
**Date:** 2026-10-10  
**Git Commit SHA:** `f777b2c`  
**Status:** Verification Signed-Off (100% Bit-Exact Equivalence across 20,019 vectors; 10-Step Formal BMC Proven)

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
| `rtl/modes/pid_gain_bank.sv` | Mode-specific bank registers & configuration decoder | IEEE 1800-2012 | `09bd0dbe5d1177c00b41d044413d49e1f53773923c0c0fb68feaacbdf4d4e794` |
| `rtl/modes/pid_mode_mgr.sv` | Operating mode FSM, graceful degradation, and fault latching | IEEE 1800-2012 | `561758f7840a8e27978f93d03142522bd66784bf90e56d5f025c4dd981083eae` |
| `rtl/safety/pid_tilt_comp.sv` | Quadratic Taylor series attitude tilt compensation ($1/\cos\theta$) | IEEE 1800-2012 | `2cdc8b71b5f65aa35f7afceb63af396957f6a2902b63dfc2f46ef419ae92c2e4` |
| `rtl/safety/pid_watchdog.sv` | Sensor freshness watchdog timer with programmable timeout | IEEE 1800-2012 | `6b146f8bd97bc1a479133e7149c6727aa9028dd176903df026c11e6c36dc9fa4` |
| `rtl/safety/pid_output_stage.sv` | Actuator registered output stage & failsafe override mux | IEEE 1800-2012 | `ca0eef5a1403206332ef7c52bd20c46655c900e2d9cb2c3a60baaf3cb43eeefc` |
| `rtl/pid_top.sv` | Full-chip 4-axis top integration & bumpless handover logic | IEEE 1800-2012 | `8b71285f8ff658a467b2cf0a02d06af9b68ee5ca82ceb877f49a59f19fcdf494` |

---

## 2. Timing Constraints & Scripts

| Relative File Path | Component Description | Format | SHA-256 Checksum |
| :--- | :--- | :--- | :--- |
| `constraints/pid_timing.sdc` | 50 MHz primary clock, I/O delays, false paths | Synopsys SDC | `d901806c6d92dfc83bd6ff5cddbfe476c13ae9862b76f8f344c88c7777b07695` |
| `handoff/CADENCE_IMPORT_GUIDE.md` | Comprehensive Cadence import and flow instructions | Markdown Doc | `a87aaef18fd2ef2676402fe7c1932f211993d7db9b4f15e42572b2ad42a959e3` |
| `handoff/genus_synth.tcl` | Cadence Genus synthesis execution script | Tcl Script | `4e54f068f17a366a76f46544df90a3cc21c71db28b5c413c99cef4dae11bf3b5` |
| `handoff/innovus_pnr.tcl` | Cadence Innovus place-and-route script template | Tcl Script | `c6aa7fee81b4631f1a8d0c4f4f0fa88944d679b3be6939f297e655954e2975d2` |

---

## 3. Verification & Reference Artifacts

| Relative File Path | Component Description | SHA-256 Checksum |
| :--- | :--- | :--- |
| `models/pid_golden_model.py` | Bit-accurate reference simulator & fixed-point golden model | `6f1450dcb00e686d1949d1285924eac7d3ffdca389be7fc1cdbbfde2946c44cf` |
| `models/flight_plant_sim.py` | 6-DOF non-linear quadrotor dynamics plant simulator | `f20dfa98c5d175c87fd824955de458a3d465aad0b4e50c20ec40473a5d3c4d2f` |
| `models/generate_vectors.py` | Comprehensive test vector generator (20,019 vectors) | `c394008936338050522f5df63e57e1c110ac9dcae954b974387c2ce31196e6e5` |
| `tb/unit/pid_axis_core_tb.sv` | Unit testbench for `pid_axis_core` (10,111 vectors) | `5aa2783b12b76a174c36e88524493fd89dc0c01f11a039364eb11d53a19c7cd2` |
| `tb/top/pid_top_tb.sv` | Top-level full-chip testbench (9,908 vectors) | `8060c384f43d4faf92594e1a4db697dc1fadc682f1a75bff612cb166696a8c47` |
| `tb/formal/pid_formal_tb.sv` | Formal testbench harness for P1-P5 invariant properties | `c30e51f8b913eaf97b0fb6f5c049477cf8aa877ebc202b00b5b557ee6131b262` |
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

- **Core Latency:** 1 clock cycle pipeline latency from `sample_valid` strobe to registered actuator output update.
- **Clock Frequency Target:** 50 MHz nominal ($T_{clk} = 20.0\text{ ns}$).
- **Sample Rate Target:** 4 kHz typical flight controller loop ($T_{sample} = 250\ \mu\text{s} = 12,500\text{ clock cycles}$).
- **Reset Convention:** Asynchronous active-low assertion (`!rst_n`), synchronous deassertion.
- **Arithmetic Representation:** Signed two's-complement Q16.16 fixed-point throughout all control loops.
