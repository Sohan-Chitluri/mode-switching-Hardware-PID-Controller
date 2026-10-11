#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================================================="
echo "  STARTING COMPLETE P&R EXECUTION: 16-BIT BOTH CASES + 32-BIT             "
echo "=========================================================================="

echo ">>> [1/2] Running 16-bit Both Cases (pid_top and pid_top_shared) ..."
"${SCRIPT_DIR}/PID_16bit/scripts/run_pnr_both_cases.sh"

echo ""
echo ">>> [2/2] Running 32-bit Q16.16 Case (pid_top) ..."
"${SCRIPT_DIR}/PID_Q16_16/scripts/run_cadence_flow.sh" parallel full

echo "=========================================================================="
echo "  ALL P&R CASES COMPLETED SUCCESSFULLY!                                   "
echo "=========================================================================="
