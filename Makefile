#==============================================================================
# Makefile for Mode-Switching Hardware PID Flight Controller
# Architecture: Fully Parallel Multi-Axis Datapath (Roll, Pitch, Yaw, Altitude)
#==============================================================================

SHELL := /usr/bin/env bash

# Directories
RTL_DIR        := rtl
TB_DIR         := tb
MODELS_DIR     := models
VECTORS_DIR    := vectors
SCRIPTS_DIR    := scripts
OUTPUT_DIR     := output
ABS_OUTPUT     := $(abspath $(OUTPUT_DIR))

# Python virtual environment
VENV_PYTHON    := $(CURDIR)/.venv/bin/python3
PYTHON         := $(if $(wildcard $(VENV_PYTHON)),$(VENV_PYTHON),python3)

# Tool paths
VERILATOR      := verilator
YOSYS          := yosys
SBY            := sby
GTKWAVE        := gtkwave

VERILATOR_INCLUDES  := -I$(RTL_DIR)/pkg -I$(RTL_DIR)/core -I$(RTL_DIR)/modes -I$(RTL_DIR)/safety

# Verilator Flags
VERILATOR_LINT_FLAGS := --lint-only -Wall -Wno-UNUSED -Wno-DECLFILENAME \
                        -Wno-PINCONNECTEMPTY -Wno-VARHIDDEN $(VERILATOR_INCLUDES) --sv

VERILATOR_SIM_FLAGS  := --binary -j 0 -O3 -CFLAGS "-O3 -std=c++17" \
                        --trace --trace-structs \
                        -Wno-VARHIDDEN -Wno-TIMESCALEMOD \
                        $(VERILATOR_INCLUDES) \
                        --sv --timing --assert

# RTL Source Collections
RTL_PKG        := $(RTL_DIR)/pkg/pid_pkg.sv

RTL_CORE_SRCS  := \
	$(RTL_PKG) \
	$(RTL_DIR)/core/pid_mult.sv \
	$(RTL_DIR)/core/pid_deriv_filter.sv \
	$(RTL_DIR)/core/pid_integ_unit.sv \
	$(RTL_DIR)/core/pid_sat_unit.sv \
	$(RTL_DIR)/core/pid_axis_core.sv

RTL_TOP_SRCS   := \
	$(RTL_CORE_SRCS) \
	$(RTL_DIR)/modes/pid_outer_loop.sv \
	$(RTL_DIR)/modes/pid_gain_bank.sv \
	$(RTL_DIR)/modes/pid_mode_mgr.sv \
	$(RTL_DIR)/safety/pid_tilt_comp.sv \
	$(RTL_DIR)/safety/pid_watchdog.sv \
	$(RTL_DIR)/safety/pid_output_stage.sv \
	$(RTL_DIR)/pid_top.sv

.PHONY: all help clean lint test-unit test-formal gen-vectors sim-plant regression

all: lint test-unit

help:
	@echo "Hardware PID Controller Build System"
	@echo "Targets:"
	@echo "  lint         : Lint all synthesizable RTL files with Verilator"
	@echo "  test-unit    : Build and run single-axis unit tests in Verilator"
	@echo "  gen-vectors  : Generate reproducible reference vectors from bit-accurate model"
	@echo "  sim-plant    : Run 6-DOF quadrotor plant simulation in Python"
	@echo "  regression   : Run full 50,000-vector co-simulation regression"
	@echo "  test-formal  : Run formal verification properties via SymbiYosys/Z3"
	@echo "  clean        : Remove build outputs, waveforms, and simulation binaries"

$(OUTPUT_DIR):
	@mkdir -p $(OUTPUT_DIR)/waves

lint: $(OUTPUT_DIR)
	@echo "=== [LINT] Verilator RTL Synthesizability Lint ==="
	$(VERILATOR) $(VERILATOR_LINT_FLAGS) $(RTL_TOP_SRCS)
	@echo "=== [LINT] PASS: Zero lint errors ==="

test-unit: $(OUTPUT_DIR)
	@echo "=== [TEST-UNIT] Running pid_axis_core unit testbench ==="
	$(VERILATOR) $(VERILATOR_SIM_FLAGS) \
		--top-module pid_axis_core_tb \
		$(RTL_CORE_SRCS) \
		$(TB_DIR)/unit/pid_axis_core_tb.sv \
		-o $(ABS_OUTPUT)/Vpid_axis_core_tb
	$(ABS_OUTPUT)/Vpid_axis_core_tb

test-top: $(OUTPUT_DIR)
	@echo "=== [TEST-TOP] Running pid_top integrated testbench ==="
	$(VERILATOR) $(VERILATOR_SIM_FLAGS) \
		--top-module pid_top_tb \
		$(RTL_TOP_SRCS) \
		$(TB_DIR)/top/pid_top_tb.sv \
		-o $(ABS_OUTPUT)/Vpid_top_tb
	$(ABS_OUTPUT)/Vpid_top_tb

gen-vectors:
	@echo "=== [VECTORS] Generating test vectors from bit-accurate model ==="
	$(PYTHON) $(MODELS_DIR)/generate_vectors.py

sim-plant:
	@echo "=== [PLANT] Running 6-DOF Quadrotor Plant Simulation ==="
	$(PYTHON) $(MODELS_DIR)/flight_plant_sim.py

regression: gen-vectors test-top
	@echo "=== [REGRESSION] Full Regression Complete ==="

test-formal:
	@echo "=== [FORMAL] Running SymbiYosys Formal Property Suite ==="
	$(SBY) -f $(TB_DIR)/formal/pid_formal.sby

clean:
	@echo "=== [CLEAN] Cleaning artifacts ==="
	@rm -rf $(OUTPUT_DIR) obj_dir/ pid_formal_*/ *.vcd *.fst *.log __pycache__
	@echo "=== [CLEAN] Done ==="
