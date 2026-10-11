#!/usr/bin/env bash
set -euo pipefail

export CDS_LIC_FILE="5280@cadence"
export LM_LICENSE_FILE="29000@172.16.77.98:5280@cadence"
export PATH="/home/install/INNOVUS211/bin:/home/install/GENUS211/bin:$PATH"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${PROJECT_ROOT}"
mkdir -p reports/innovus

echo "=========================================================================="
echo "  [1/2] Running Innovus P&R for Case 1: pid_top (Fully Parallel)           "
echo "=========================================================================="
export DESIGN_NAME="pid_top"
innovus -batch -no_gui -files scripts/innovus_pnr.tcl -log reports/innovus/innovus_pid_top.log

echo "=========================================================================="
echo "  [2/2] Running Innovus P&R for Case 2: pid_top_shared (Time-Shared)       "
echo "=========================================================================="
export DESIGN_NAME="pid_top_shared"
innovus -batch -no_gui -files scripts/innovus_pnr.tcl -log reports/innovus/innovus_pid_top_shared.log

echo "=========================================================================="
echo "  Both 16-bit P&R runs completed successfully!                            "
echo "=========================================================================="
