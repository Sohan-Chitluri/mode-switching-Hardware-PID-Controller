# Cadence Tool Flow Import & Physical Design Guide

**Design:** Mode-Switching Hardware PID Flight Controller (`pid_top`)  
**Target Flow:** Cadence Genus (Synthesis) & Cadence Innovus (Place & Route)  
**Host Architecture:** IEEE 1800-2012 Synthesizable SystemVerilog  
**Handoff Package Date:** 2026-10-10  
**Local Verification Sign-Off:** 100% Bit-Exact Equivalence & SymbiYosys Formal Proof  

---

## 1. Overview & Tool Flow Strategy

This document provides complete instructions for importing the `pid_top` flight controller into Cadence tools on the secondary workstation.

### Flow Separation Principle
- **Workstation A (Current Development Host):**
  - Synthesizable RTL development in IEEE 1800-2012 SystemVerilog.
  - Python golden reference modeling (`pid_golden_model.py`) and 6-DOF flight dynamics simulation (`flight_plant_sim.py`).
  - Bit-exact co-simulation and regression using Verilator (`make regression`).
  - Formal property safety verification using SymbiYosys and Z3 SMT solver (`make test-formal`).
  - **No local proprietary standard-cell libraries or EDA tools were assumed.**
- **Workstation B (Target Physical Implementation Host):**
  - Logic synthesis with Cadence Genus using target PDK `.lib` standard-cell timing models.
  - Floorplanning, power-grid synthesis, placement, CTS, and routing with Cadence Innovus.
  - Extraction, sign-off STA with Tempus, and physical verification (DRC/LVS) with Pegasus/PVS.

---

## 2. Package Directory Layout & File Transfer

Transfer the following directories to the target Cadence workstation:

```
pid_handoff_package/
├── rtl/
│   ├── pkg/
│   │   └── pid_pkg.sv                 # Encodings, types, scaling constants
│   ├── core/
│   │   ├── pid_mult.sv                # Q16.16 signed multiplier
│   │   ├── pid_deriv_filter.sv        # Derivative-on-measurement + IIR LPF
│   │   ├── pid_integ_unit.sv          # Guarded accumulator + anti-windup
│   │   ├── pid_sat_unit.sv            # 48-bit guarded sum + saturation
│   │   └── pid_axis_core.sv           # Parameterized PID core (x4 instances)
│   ├── modes/
│   │   ├── pid_outer_loop.sv          # Attitude & altitude cascaded loops
│   │   ├── pid_gain_bank.sv           # Gain bank storage & MMIO decoder
│   │   └── pid_mode_mgr.sv            # Autonomous mode FSM & fault latching
│   ├── safety/
│   │   ├── pid_tilt_comp.sv           # Quadratic Taylor thrust compensation
│   │   ├── pid_watchdog.sv            # Sensor freshness timeout counter
│   │   └── pid_output_stage.sv        # Output mux with deterministic failsafe
│   └── pid_top.sv                     # Top-level full-chip integration
├── constraints/
│   └── pid_timing.sdc                 # 50 MHz timing constraints
├── scripts/
│   ├── genus_synth.tcl                # Cadence Genus synthesis script
│   ├── innovus_pnr.tcl                # Cadence Innovus P&R execution script
│   ├── innovus_pnr_fast.tcl           # Fast-track Innovus P&R execution script
│   ├── pid_mmmc.tcl                   # Multi-Mode Multi-Corner timing configuration
│   └── innovus_timing_reports.tcl     # Sign-off setup & hold timing report snippet
├── reports/
│   ├── genus/                         # Dedicated Genus synthesis reports and logs
│   └── innovus/                       # Dedicated Innovus P&R timing reports and logs
├── netlist/                           # Synthesized gate netlists, SDC, GDS, and DEF
└── handoff/
    ├── MANIFEST.md                    # File catalog & SHA-256 integrity hashes
    └── CADENCE_IMPORT_GUIDE.md        # This implementation guide
```

---

## 3. Cadence Genus Synthesis Flow

### 3.1 Environment Variable Configuration
Before executing Genus, set the environment variables pointing to the target PDK libraries:

```bash
# Example for TSMC 180nm, 65nm, GlobalFoundries 22nm, or SkyWater 130nm:
export PDK_LIB_DIR="/home/install/FOUNDRY/digital/180nm/dig/lib"
export TARGET_LIB="${PDK_LIB_DIR}/typical.lib"
export LINK_LIB="${TARGET_LIB}"
```

### 3.2 Executing Synthesis
Run Genus in batch mode from the project root:

```bash
genus -f scripts/genus_synth.tcl -log reports/genus/genus_synth.log
```

### 3.3 Expected Outputs & Sign-Off Checklist
Upon completion, inspect the generated reports in `reports/genus/`:
1. `pid_top_timing_setup.rpt`: Worst Negative Slack (WNS) $\ge 0.000\text{ ns}$ at 50 MHz.
2. Note: Hold timing report (`pid_top_timing_hold.rpt`) is generated post-CTS in Innovus via `view_hold` MMMC analysis, not in Genus.
3. `pid_top_area.rpt`: Verify total cell count and equivalent NAND2 gate count.
4. `pid_top_power.rpt`: Verify static leakage and dynamic switching power.
5. `pid_top_qor.rpt`: Review datapath resource sharing, multiplier inferencing, and runtime summary.
6. Exported files in `netlist/`:
   - `pid_top_synth.v`: Mapped gate-level netlist.
   - `pid_top_synth.sdc`: Post-synthesis timing constraints.

---

## 4. Cadence Innovus Place & Route Flow

### 4.1 Environment Variable Configuration
Configure the physical technology LEF, standard-cell LEF, site, and layer variables:

```bash
# Required PDK Paths
export PDK_TECH_LEF="/home/install/FOUNDRY/digital/180nm/dig/lef/all.lef"
export PDK_CELL_LEF="/home/install/FOUNDRY/digital/180nm/dig/lef/all.lef"
export PDK_GDS_FILES="/home/vlsi/streamOut.map"

# Optional PDK-Specific Physical Settings (Defaults shown)
export PDK_CORE_SITE="tsm3site"             # Standard cell site name
export PDK_UTILIZATION="0.60"               # Target core placement utilization (0.60 standard, 0.40 fast)
export PDK_PWR_TOP_BOT_LAYER="Metal6"       # Power ring horizontal layer
export PDK_PWR_LEFT_RIGHT_LAYER="Metal5"    # Power ring vertical layer
export PDK_SROUTE_MIN_LAYER="Metal1"        # SRoute followpin minimum layer
export PDK_SROUTE_MAX_LAYER="Metal6"        # SRoute followpin maximum layer
```

### 4.2 Executing P&R
Run Innovus in batch or interactive GUI mode from the project root:

```bash
# Standard Production P&R:
innovus -files scripts/innovus_pnr.tcl -log reports/innovus/innovus.log

# Or Fast-Track P&R (~6-8 minutes, 0.40 utilization):
innovus -files scripts/innovus_pnr_fast.tcl -log reports/innovus/innovus_fast.log
```

### 4.3 Flow Milestones
The script automatically executes:
- **Design Initialization:** Loads netlist, SDC, MMMC views, and technology LEFs.
- **Floorplanning:** Sets die boundary with configurable core target utilization and site (tsm3site), distributing 571 I/O pins evenly across 4 die edges on Metal3/Metal4.
- **Power Routing:** Adds VDD/VSS core rings (Metal6/Metal5) and standard-cell followpins on specified metal layers.
- **Placement:** Timing-driven placement with congestion mitigation (`place_opt_design`).
- **Clock Tree Synthesis (CCOpt):** Synthesizes balanced clock tree targeting $\le 0.5\text{ ns}$ transition.
- **Detailed Routing:** NanoRoute executes antenna-aware, timing-driven routing.
- **Post-Route Hold Fixing:** `optDesign -postRoute -hold` to resolve hold violations.
- **Verification:** Runs `verifyGeometry` and `verifyConnectivity` for zero DRC and zero shorts.
- **Reports Export:** Populates `reports/innovus/` with timing, power, area, and sign-off reports.
- **Layout Export:** Writes `netlist/pid_top.gds`, `netlist/pid_top.def`, and `netlist/pid_top_pnr.v`.

---

## 5. Design Specifications & Interface Contracts

### 5.1 Clock & Reset Architecture
- **Clock Port:** `clk` — single primary clock domain at 50 MHz ($T_{period} = 20.0\text{ ns}$).
- **Reset Port:** `rst_n` — active-low synchronous reset.
  - All sequential registers in the datapath, FSM, and watchdog have deterministic synchronous reset states (`FIX_ZERO` or `MODE_ACRO`).
  - `rst_n` is constrained as a synchronous input in `pid_timing.sdc` without false-path exclusions.
- **Asynchronous CDC Safety:**
  - The external emergency kill pin `failsafe_ext_n` is synchronized to `clk` via a dedicated 2-stage flip-flop synchronizer (`fs_sync_q1`, `fs_sync_q2`) in `pid_top.sv`.
  - A `set_false_path -from [get_ports failsafe_ext_n]` constraint is defined in `pid_timing.sdc` to correctly model the CDC crossing without timing violations.

### 5.2 Latency Contract
- **Throughput:** 1 sample per clock cycle maximum; typically sampled at 4 kHz ($12,500$ clock cycles per sample interval).
- **Emergency Override Latency:** Exactly 1 clock cycle for internal synchronous faults (`watchdog_alarm`), and 3 clock cycles (60.0 ns at 50 MHz; 2-cycle flip-flop CDC synchronizer + 1-cycle output stage registration) for the asynchronous external emergency kill pin (`!failsafe_ext_n`).

### 5.3 Fixed-Point Representation
- **Standard Format:** Q16.16 signed two's-complement (`DATA_W = 32`).
- **Intermediate Summation:** Guarded 48-bit accumulators in saturation units (`ACCUM_W = 48`), mathematically preventing overflow wrap-around before clamping to $[U_{min}, U_{max}]$.

---

## 6. Pre-Handoff Verification Evidence (Local Host)

The RTL handed off in this package has achieved complete verification sign-off:

1. **RTL Synthesizability Lint (Verilator 5.052):**
   - **Status:** PASS (Zero errors, Zero warnings across entire source tree).
2. **Single-Axis Core Co-Simulation (`pid_axis_core`):**
   - **Vectors:** 10,111 vectors (directed boundary tests + 10,000 randomized Monte Carlo).
   - **Result:** 0 bit mismatches vs Python bit-accurate model (**100% BIT-EXACT PASS**).
3. **Full-Chip Integrated Co-Simulation (`pid_top`):**
   - **Vectors:** 9,908 top vectors with active mode switching, sensor drops, and re-arm cycles.
   - **Result:** 0 bit mismatches vs top golden model (**100% BIT-EXACT PASS**).
   - **Directed Bumpless Handover:** Peak transient step $\Delta U = 16\text{ counts} \approx 0.024\% \le 5.0\%$.
   - **Directed Watchdog Timeout:** Failsafe triggered at exact cycle; actuator commands zeroed.
   - **Directed Pilot Re-Arm:** Full recovery from Failsafe to Stabilize verified.
4. **Formal Property Verification (SymbiYosys + Z3 SMT Solver):**
   - **Properties:**
     - P1: Saturation unit output strictly bounded within $[U_{min}, U_{max}]$.
     - P2: Output stage asserts exact safe outputs during Failsafe mode.
     - P3: Watchdog timer raises alarm when idle count reaches timeout.
     - P4: Anti-windup conditional clamping freezes accumulator expansion.
     - P5: Mode manager deterministically transitions to Failsafe on any fault.
   - **Result:** 10-step Bounded Model Checking (BMC) **PASS across all properties (rc=0)**.
5. **Physical Plant Closed-Loop Dynamics (`flight_plant_sim.py`):**
   - **Steady-State Error:** $0.023^\circ$ (Spec: $\le 0.10^\circ$) -> PASS.
   - **Peak Overshoot:** $0.00\%$ (Spec: $\le 10.0\%$) -> PASS.
   - **2% Settling Time:** $441.5\text{ ms}$ (Spec: $\le 500.0\text{ ms}$) -> PASS.

---

## 7. Decisions to Confirm on Target Cadence Workstation

The following technology-specific parameters are intentionally unconstrained in this portable package and must be confirmed on the target PC:
1. **Target Foundry & Process Node:** (e.g., TSMC 65nm LP, GlobalFoundries 22FDX, SkyWater 130nm, etc.).
2. **Operating Corner for Timing Closure:** (e.g., Slow-Slow 0.9V 125°C for setup, Fast-Fast 1.1V -40°C for hold).
3. **Target Standard-Cell Library Variant:** (e.g., High-Vt for ultra-low static leakage vs Regular-Vt for high performance).
4. **Physical Metal Stack & Routing Pitch:** (e.g., 1P6M with 4 thin metals and 2 thick global power metals).
5. **I/O Pad Ring & Package Pinout:** Pad cell selection, electrostatic discharge (ESD) protection, and boundary scan (JTAG) insertion if tape-out ready.
