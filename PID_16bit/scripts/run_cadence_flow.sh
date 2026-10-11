#!/usr/bin/env bash
################################################################################
# Master Sequential Cadence Flow Runner
# Target Design: Mode-Switching Hardware PID Flight Controller
#
# Usage:
#   ./scripts/run_cadence_flow.sh [parallel|shared] [full|fast]
#
# Examples:
#   ./scripts/run_cadence_flow.sh                   # Parallel pid_top, full P&R
#   ./scripts/run_cadence_flow.sh shared            # Time-shared pid_top_shared, full P&R
#   ./scripts/run_cadence_flow.sh shared fast       # Time-shared pid_top_shared, fast P&R
#   ./scripts/run_cadence_flow.sh parallel fast     # Parallel pid_top, fast P&R
################################################################################

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${PROJECT_ROOT}"

# Parse architecture selection
ARCH="${1:-parallel}"
PNR_MODE="${2:-full}"

case "${ARCH}" in
    shared|pid_top_shared)
        export DESIGN_NAME="pid_top_shared"
        ARCH_DESC="Time-Shared Datapath (10 multipliers, 4-cycle sequenced)"
        ;;
    parallel|pid_top)
        export DESIGN_NAME="pid_top"
        ARCH_DESC="Fully Parallel Datapath (22 multipliers, 1-cycle latency)"
        ;;
    soc_shared|pid_soc_top_shared)
        export DESIGN_NAME="pid_soc_top_shared"
        ARCH_DESC="Time-Shared SoC Top with Mixer"
        ;;
    soc|pid_soc_top)
        export DESIGN_NAME="pid_soc_top"
        ARCH_DESC="Fully Parallel SoC Top with Mixer"
        ;;
    *)
        export DESIGN_NAME="${ARCH}"
        ARCH_DESC="Custom design: ${ARCH}"
        ;;
esac

case "${PNR_MODE}" in
    fast)
        INNOVUS_SCRIPT="scripts/innovus_pnr_fast.tcl"
        ;;
    full|*)
        INNOVUS_SCRIPT="scripts/innovus_pnr.tcl"
        ;;
esac

echo "=========================================================================="
echo "  Cadence ASIC Implementation Flow (Sequential Execution)                 "
echo "  Design Target : ${DESIGN_NAME}"
echo "  Architecture  : ${ARCH_DESC}"
echo "  P&R Mode      : ${PNR_MODE} (${INNOVUS_SCRIPT})"
echo "=========================================================================="

# Check tool binaries
GENUS_CMD="${GENUS_BIN:-genus}"
INNOVUS_CMD="${INNOVUS_CMD:-innovus}"

# Search standard university/lab paths if not in PATH
if ! command -v "${GENUS_CMD}" >/dev/null 2>&1; then
    for candidate in /home/install/GENUS211/bin/genus /cad/cadence/GENUS*/bin/genus /eda/cadence/genus/bin/genus; do
        if [ -x "${candidate}" ]; then
            GENUS_CMD="${candidate}"
            export PATH="$(dirname "${candidate}"):${PATH}"
            break
        fi
    done
fi

if ! command -v "${INNOVUS_CMD}" >/dev/null 2>&1; then
    for candidate in /home/install/INNOVUS211/bin/innovus /cad/cadence/INNOVUS*/bin/innovus /eda/cadence/innovus/bin/innovus; do
        if [ -x "${candidate}" ]; then
            INNOVUS_CMD="${candidate}"
            export PATH="$(dirname "${candidate}"):${PATH}"
            break
        fi
    done
fi

if ! command -v "${GENUS_CMD}" >/dev/null 2>&1; then
    echo "[-] ERROR: Cadence Genus ('${GENUS_CMD}') not found in PATH."
    echo "    Please run this script on your EDA workstation or export PATH:"
    echo "    export PATH=/path/to/genus/bin:\$PATH"
    exit 1
fi

if ! command -v "${INNOVUS_CMD}" >/dev/null 2>&1; then
    echo "[-] ERROR: Cadence Innovus ('${INNOVUS_CMD}') not found in PATH."
    echo "    Please run this script on your EDA workstation or export PATH:"
    echo "    export PATH=/path/to/innovus/bin:\$PATH"
    exit 1
fi

mkdir -p reports/genus reports/innovus netlist

# ==============================================================================
# STAGE 1: SEQUENTIAL SYNTHESIS (Cadence Genus)
# ==============================================================================
echo ""
echo "--------------------------------------------------------------------------"
echo ">>> [STAGE 1/2] Launching Cadence Genus Synthesis for ${DESIGN_NAME}..."
echo "--------------------------------------------------------------------------"

# Remove stale completion markers
rm -f "netlist/${DESIGN_NAME}_synth.done" "netlist/${DESIGN_NAME}_pnr.done"

GENUS_LOG="reports/genus/genus_${DESIGN_NAME}.log"
START_SYNTH=$(date +%s)

"${GENUS_CMD}" -batch -no_gui \
    -execute "set DESIGN_NAME \"${DESIGN_NAME}\"; source scripts/genus_synth.tcl" \
    -log "${GENUS_LOG}"

SYNTH_EXIT_CODE=$?
END_SYNTH=$(date +%s)
SYNTH_ELAPSED=$((END_SYNTH - START_SYNTH))

if [ ${SYNTH_EXIT_CODE} -ne 0 ] || [ ! -f "netlist/${DESIGN_NAME}_synth.v" ]; then
    echo "[-] ERROR: Genus synthesis failed with exit code ${SYNTH_EXIT_CODE}!"
    echo "    Review the synthesis log: ${GENUS_LOG}"
    if [ -f "${GENUS_LOG}" ]; then
        echo "    --- Last 25 lines of ${GENUS_LOG} ---"
        tail -n 25 "${GENUS_LOG}"
    fi
    exit ${SYNTH_EXIT_CODE}
fi

echo "[+] STAGE 1 COMPLETE: Synthesis succeeded in ${SYNTH_ELAPSED}s."
echo "    Gate Netlist: netlist/${DESIGN_NAME}_synth.v"
echo "    SDC Output  : netlist/${DESIGN_NAME}_synth.sdc"

# ==============================================================================
# STAGE 2: SEQUENTIAL IMPLEMENTATION (Cadence Innovus Place & Route)
# ==============================================================================
echo ""
echo "--------------------------------------------------------------------------"
echo ">>> [STAGE 2/2] Launching Cadence Innovus P&R for ${DESIGN_NAME}..."
echo "--------------------------------------------------------------------------"

INNOVUS_LOG="reports/innovus/innovus_${DESIGN_NAME}.log"
START_PNR=$(date +%s)

"${INNOVUS_CMD}" -batch -no_gui \
    -execute "set DESIGN_NAME \"${DESIGN_NAME}\"; source ${INNOVUS_SCRIPT}" \
    -log "${INNOVUS_LOG}"

PNR_EXIT_CODE=$?
END_PNR=$(date +%s)
PNR_ELAPSED=$((END_PNR - START_PNR))

if [ ${PNR_EXIT_CODE} -ne 0 ] || [ ! -f "netlist/${DESIGN_NAME}_pnr.v" ]; then
    echo "[-] ERROR: Innovus P&R failed with exit code ${PNR_EXIT_CODE}!"
    echo "    Review the P&R log: ${INNOVUS_LOG}"
    if [ -f "${INNOVUS_LOG}" ]; then
        echo "    --- Last 25 lines of ${INNOVUS_LOG} ---"
        tail -n 25 "${INNOVUS_LOG}"
    fi
    exit ${PNR_EXIT_CODE}
fi

echo "[+] STAGE 2 COMPLETE: Innovus P&R succeeded in ${PNR_ELAPSED}s."
echo "    Post-Route Netlist: netlist/${DESIGN_NAME}_pnr.v"
echo "    DEF Layout        : netlist/${DESIGN_NAME}.def"
echo "    GDSII Layout      : netlist/${DESIGN_NAME}.gds"

# ==============================================================================
# STAGE 3: SIGNOFF QUALITY OF RESULTS (QoR) SUMMARY
# ==============================================================================
echo ""
echo "=========================================================================="
echo "                CADENCE SEQUENTIAL FLOW SIGNOFF SUMMARY                   "
echo "=========================================================================="
echo "  Design Module      : ${DESIGN_NAME}"
echo "  Architecture       : ${ARCH_DESC}"
echo "  Total Flow Runtime : $((SYNTH_ELAPSED + PNR_ELAPSED))s (Synthesis: ${SYNTH_ELAPSED}s, P&R: ${PNR_ELAPSED}s)"
echo ""

# Extract Area & Gates from Genus report if available
AREA_RPT="reports/genus/${DESIGN_NAME}_area.rpt"
if [ -f "${AREA_RPT}" ]; then
    echo "  [Genus Synthesis Area Summary]:"
    grep -E -i "total area|cell count|gates|hierarchical" "${AREA_RPT}" | head -n 8 | sed 's/^/    /' || true
fi

# Extract Post-Route Timing Slack from Innovus reports if available
SETUP_RPT="reports/innovus/${DESIGN_NAME}_timing_setup.rpt"
HOLD_RPT="reports/innovus/${DESIGN_NAME}_timing_hold.rpt"

if [ -f "${SETUP_RPT}" ]; then
    echo ""
    echo "  [Innovus Post-Route Setup Timing]:"
    grep -E -i "slack|endpoint|data path" "${SETUP_RPT}" | head -n 4 | sed 's/^/    /' || true
fi

if [ -f "${HOLD_RPT}" ]; then
    echo ""
    echo "  [Innovus Post-Route Hold Timing]:"
    grep -E -i "slack|endpoint|data path" "${HOLD_RPT}" | head -n 4 | sed 's/^/    /' || true
fi

echo ""
echo "  Exported Artifacts:"
echo "    - Netlist (Synthesized) : netlist/${DESIGN_NAME}_synth.v"
echo "    - SDC (Synthesized)     : netlist/${DESIGN_NAME}_synth.sdc"
echo "    - Netlist (Post-Route)  : netlist/${DESIGN_NAME}_pnr.v"
echo "    - DEF Layout            : netlist/${DESIGN_NAME}.def"
echo "    - GDSII Layout          : netlist/${DESIGN_NAME}.gds"
echo "    - Reports Directory     : reports/genus/ & reports/innovus/"
echo "=========================================================================="
echo "Sequential Cadence Flow executed successfully end-to-end!"
