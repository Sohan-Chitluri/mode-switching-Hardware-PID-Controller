#!/usr/bin/env bash
# ==============================================================================
# scripts/run_regression.sh
# End-to-end regression test suite for Mode-Switching Hardware PID Controller
# ==============================================================================
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

echo "======================================================================"
echo " Starting Full Hardware PID Regression Suite"
echo " Working Directory: ${REPO_ROOT}"
echo "======================================================================"

# 1. Lint Verification
echo ""
echo ">>> [STAGE 1/6] Running Verilator Lint Checks..."
make lint
echo ">>> Stage 1 PASSED: Verilator lint clean."

# 2. Structural Elaboration Check
echo ""
echo ">>> [STAGE 2/6] Running Yosys Structural Elaboration..."
yosys -s scripts/prelim_synth.tcl > output/yosys_synth.log 2>&1
echo ">>> Stage 2 PASSED: Yosys elaboration and check clean."

# 3. Unit Testbench (10,111 vectors)
echo ""
echo ">>> [STAGE 3/6] Running Unit Testbench (pid_axis_core_tb)..."
make test-unit
echo ">>> Stage 3 PASSED: 10,111 unit vectors 100% bit-exact."

# 4. Top-Level Full-Chip Regression (10,000 vectors)
echo ""
echo ">>> [STAGE 4/6] Running Top-Level Regression (pid_top_tb)..."
make test-top
echo ">>> Stage 4 PASSED: 10,000 top vectors 100% bit-exact."

# 5. Formal Verification (BMC Depth 10)
echo ""
echo ">>> [STAGE 5/6] Running Formal Verification (SymbiYosys)..."
sby -f tb/formal/pid_formal.sby
echo ">>> Stage 5 PASSED: Properties P1-P5 proven for BMC depth 10."

# 6. RTL-to-Plant Co-Simulation Harness Verification
echo ""
echo ">>> [STAGE 6/7] Running RTL-to-Plant Co-Simulation Harness Verification..."
make cosim-lib
PYTHONPATH=models python3 models/flight_plant_cosim.py --verify-harness
echo ">>> Stage 6 PASSED: Co-simulation harness proofs verified."

# 7. 4-DOF Quadrotor Closed-Loop RTL Co-Simulation & Dynamic Performance
echo ""
echo ">>> [STAGE 7/7] Running 4-DOF Closed-Loop RTL Co-Simulation..."
PYTHONPATH=models python3 models/flight_plant_cosim.py
PYTHONPATH=models python3 models/flight_plant_sim.py --test-metrics
echo ">>> Stage 7 PASSED: Closed-loop stability and dynamic metrics verified (Rise Time <= 60ms)."

echo ""
echo "======================================================================"
echo " ALL REGRESSION STAGES COMPLETED SUCCESSFULLY (PASS)"
echo "======================================================================"
