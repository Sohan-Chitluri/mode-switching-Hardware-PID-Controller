#!/usr/bin/env bash
export DISPLAY="${DISPLAY:-:0}"
export CDS_LIC_FILE="5280@cadence"
export LM_LICENSE_FILE="29000@172.16.77.98:5280@cadence"
export IUSHOME="/home/install/INCISIVE152"
export PATH="$IUSHOME/tools/bin:$IUSHOME/tools.lnx86/bin:$PATH"
export CDS_AUTO_64BIT="ALL"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
echo "Starting Cadence NCLaunch..."
exec nclaunch -cdslib cds.lib -hdlvar hdl.var "$@"
