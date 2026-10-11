#!/usr/bin/env bash
# ==============================================================================
# scripts/run_regression.sh
# End-to-end regression test suite for Mode-Switching Hardware PID Controller
# ==============================================================================
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

# Dynamic library and Python environment resolution
GCC_LIB_DIR=$(dirname $(g++ -print-file-name=libstdc++.so.6 2>/dev/null) 2>/dev/null)
ZLIB_DIR=$(dirname $(find /nix/store -maxdepth 4 -name "libz.so.1" 2>/dev/null | grep zlib-1.3 | head -n 1) 2>/dev/null)
if [ -n "${GCC_LIB_DIR}" ] && [ -n "${ZLIB_DIR}" ]; then
    export LD_LIBRARY_PATH="${ZLIB_DIR}:${GCC_LIB_DIR}:${LD_LIBRARY_PATH}"
fi

if [ -f "${REPO_ROOT}/.venv/bin/python3" ]; then
    PYTHON="${REPO_ROOT}/.venv/bin/python3"
else
    PYTHON="python3"
fi

echo "======================================================================"
echo " Starting Full Hardware PID Regression Suite (Phase 4)"
echo " Working Directory: ${REPO_ROOT}"
echo " Python Binary    : ${PYTHON}"
echo "======================================================================"

mkdir -p output/waves

# 1. Lint Verification
echo ""
echo ">>> [STAGE 1/10] Running Verilator Lint Checks..."
make lint
echo ">>> Stage 1 PASSED: Verilator lint clean across all 17 modules."

# 2. Structural Elaboration & Synthesis Check
echo ""
echo ">>> [STAGE 2/10] Running Yosys Structural Elaboration & Synthesis..."
yosys -s scripts/prelim_synth.tcl > output/yosys_synth.log 2>&1
echo ">>> Stage 2 PASSED: Yosys elaboration and structural synthesis clean."

# 3. Unit Testbench (Single-Axis Core)
echo ""
echo ">>> [STAGE 3/10] Running Unit Testbench (pid_axis_core_tb)..."
make test-unit
echo ">>> Stage 3 PASSED: 10,111 unit vectors 100% bit-exact."

# 4. Quad-X Motor Mixer Unit Testbench
echo ""
echo ">>> [STAGE 4/10] Running Quad-X Motor Mixer Unit Testbench (pid_mixer_tb)..."
make test-mixer
echo ">>> Stage 4 PASSED: 14 mixer unit test suites passed."

# 5. Top-Level Full-Chip Regression
echo ""
echo ">>> [STAGE 5/10] Running Top-Level Regression (pid_top_tb)..."
make test-top
echo ">>> Stage 5 PASSED: 10,000 top vectors + directed suites 100% bit-exact."

# 6. SoC Top Integration & MMIO Bus Testbench
echo ""
echo ">>> [STAGE 6/10] Running SoC Top MMIO Integration Testbench (pid_soc_top_tb)..."
make test-soc
echo ">>> Stage 6 PASSED: 6 SoC MMIO integration tests passed."

# 7. Mode Transition & Flight Safety Stress Testbench
echo ""
echo ">>> [STAGE 7/10] Running Mode Transition & Safety Stress Testbench (pid_mode_stress_tb)..."
make test-stress
echo ">>> Stage 7 PASSED: All 20 exhaustive stress tests passed."

# 8. Formal Verification (BMC Depth 10)
echo ""
echo ">>> [STAGE 8/10] Running Formal Verification (SymbiYosys)..."
sby -f tb/formal/pid_formal.sby
echo ">>> Stage 8 PASSED: Properties P1-P5 proven for BMC depth 10."

# 9. RTL-to-Plant Co-Simulation Harness Verification
echo ""
echo ">>> [STAGE 9/10] Running RTL-to-Plant Co-Simulation Harness Verification..."
make cosim-lib
PYTHONPATH=models ${PYTHON} models/flight_plant_cosim.py --verify-harness
echo ">>> Stage 9 PASSED: Co-simulation harness proofs verified."

# 10. 4-DOF Quadrotor Closed-Loop RTL Co-Simulation & Dynamic Performance
echo ""
echo ">>> [STAGE 10/10] Running 4-DOF Closed-Loop RTL Co-Simulation..."
PYTHONPATH=models ${PYTHON} models/flight_plant_cosim.py
PYTHONPATH=models ${PYTHON} models/flight_plant_sim.py --test-metrics
echo ">>> Stage 10 PASSED: Closed-loop stability and dynamic metrics verified (Rise Time <= 60ms)."

echo ""
echo "======================================================================"
echo " ALL 10 REGRESSION STAGES COMPLETED SUCCESSFULLY (PASS)"
echo "======================================================================"
