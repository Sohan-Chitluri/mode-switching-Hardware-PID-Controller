#!/usr/bin/env bash
# ==============================================================================
# verify_all.sh — Master 1-Command Verification Script
# PS-02 Mode-Switching Hardware PID Flight Controller
# Sudarshana Semiconductors • VLSI Hackathon 2026
# Designers: Sohan Chitluri (24BVD1104) & Prasiddh Saha (24BVD1103)
# ==============================================================================
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${REPO_ROOT}"

# Dynamic library resolution for Verilated C++ runtime
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
echo " PS-02 Mode-Switching Hardware PID Flight Controller"
echo " Comprehensive Automated Verification Suite"
echo " Evaluator Quick-Check: 100% Reproducible Functional Verification"
echo " Designers: Sohan Chitluri (24BVD1104) & Prasiddh Saha (24BVD1103)"
echo " Working Directory: ${REPO_ROOT}"
echo "======================================================================"

PASS_COUNT=0
TOTAL_STAGES=6

# Stage 1: Verilator RTL Synthesizability Lint
echo ""
echo ">>> [STAGE 1/6] Running Verilator Synthesizability Lint..."
make lint
echo ">>> [PASS] Stage 1: Zero lint errors across all SystemVerilog modules."
PASS_COUNT=$((PASS_COUNT + 1))

# Stage 2: 32-bit Q16.16 Core & Top-Level Testbenches
echo ""
echo ">>> [STAGE 2/6] Running 32-bit Q16.16 Testbenches (Unit, Top, Mixer, SoC, Stress)..."
make test-unit test-top test-mixer test-soc test-stress
echo ">>> [PASS] Stage 2: All 32-bit Q16.16 verification targets passed bit-exact."
PASS_COUNT=$((PASS_COUNT + 1))

# Stage 3: Fixed-Point Boundaries & Safety Edge Cases
echo ""
echo ">>> [STAGE 3/6] Running Fixed-Point Boundaries & Safety Edge Cases..."
make test-edge-all
echo ">>> [PASS] Stage 3: All 19 boundary edge-cases & fault latching tests passed."
PASS_COUNT=$((PASS_COUNT + 1))

# Stage 4: 16-bit Q4.12 Parallel & Time-Shared Architectures
echo ""
echo ">>> [STAGE 4/6] Running 16-bit Q4.12 Parallel & Time-Shared Datapaths..."
make test-16bit
echo ">>> [PASS] Stage 4: Both 16-bit Parallel & Time-Shared variants passed bit-exact."
PASS_COUNT=$((PASS_COUNT + 1))

# Stage 5: Closed-Loop Plant Simulation Stability Metrics
echo ""
echo ">>> [STAGE 5/6] Verifying Closed-Loop Quadrotor Dynamic Stability Metrics..."
PYTHONPATH=models ${PYTHON} models/flight_plant_sim.py --test-metrics
echo ">>> [PASS] Stage 5: Plant dynamic metrics verified (Rise Time <= 60ms, Overshoot < 10%)."
PASS_COUNT=$((PASS_COUNT + 1))

# Stage 6: RTL-to-Plant Co-Simulation Harness & 4-DOF Quadrotor Dynamics
echo ""
echo ">>> [STAGE 6/6] Running True RTL-to-Plant Closed-Loop Co-Simulation..."
make cosim-lib
PYTHONPATH=models ${PYTHON} models/flight_plant_cosim.py --verify-harness
PYTHONPATH=models ${PYTHON} models/flight_plant_cosim.py
echo ">>> [PASS] Stage 6: Full 4-DOF quadrotor closed-loop flight co-simulation verified."
PASS_COUNT=$((PASS_COUNT + 1))

echo ""
echo "======================================================================"
echo " MASTER VERIFICATION SUMMARY"
echo " Total Verification Stages : ${TOTAL_STAGES}"
echo " Passed Stages             : ${PASS_COUNT}"
echo " Failed Stages             : 0"
echo " STATUS                    : 100% BIT-EXACT PASS (ALL TARGETS VERIFIED)"
echo "======================================================================"
