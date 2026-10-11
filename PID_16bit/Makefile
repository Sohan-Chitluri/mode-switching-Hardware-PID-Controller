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

VERILATOR_INCLUDES  := -I$(RTL_DIR) -I$(RTL_DIR)/pkg -I$(RTL_DIR)/core -I$(RTL_DIR)/modes -I$(RTL_DIR)/safety

# Verilator Flags
VERILATOR_LINT_FLAGS := --lint-only -Wall -Wno-UNUSED -Wno-DECLFILENAME \
                        -Wno-PINCONNECTEMPTY -Wno-VARHIDDEN -Wno-IMPORTSTAR $(VERILATOR_INCLUDES) --sv

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

RTL_SOC_SRCS   := \
	$(RTL_TOP_SRCS) \
	$(RTL_DIR)/safety/pid_mixer.sv \
	$(RTL_DIR)/pid_soc_top.sv

RTL_SHARED_SRCS := \
	$(RTL_PKG) \
	$(RTL_DIR)/core/pid_mult.sv \
	$(RTL_DIR)/core/pid_deriv_filter.sv \
	$(RTL_DIR)/core/pid_integ_unit.sv \
	$(RTL_DIR)/core/pid_sat_unit.sv \
	$(RTL_DIR)/modes/pid_outer_loop.sv \
	$(RTL_DIR)/modes/pid_gain_bank.sv \
	$(RTL_DIR)/modes/pid_mode_mgr.sv \
	$(RTL_DIR)/safety/pid_tilt_comp.sv \
	$(RTL_DIR)/safety/pid_watchdog.sv \
	$(RTL_DIR)/safety/pid_output_stage.sv \
	$(RTL_DIR)/shared/pid_axis_engine_shared.sv \
	$(RTL_DIR)/shared/pid_top_shared.sv

RTL_SOC_SHARED_SRCS := \
	$(RTL_SHARED_SRCS) \
	$(RTL_DIR)/safety/pid_mixer.sv \
	$(RTL_DIR)/shared/pid_soc_top_shared.sv

.PHONY: all help clean lint test-unit test-top test-shared test-mixer test-soc test-stress test-formal synth-shared gen-vectors sim-plant regression cadence-flow cadence-flow-shared cadence-flow-fast cadence-flow-shared-fast

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
	$(VERILATOR) $(VERILATOR_LINT_FLAGS) $(RTL_SOC_SRCS)
	@echo "=== [LINT] PASS: Zero lint errors ==="

test-unit: $(OUTPUT_DIR)
	@echo "=== [TEST-UNIT] Running pid_axis_core unit testbench ==="
	$(VERILATOR) $(VERILATOR_SIM_FLAGS) \
		--top-module pid_axis_core_tb \
		--Mdir $(ABS_OUTPUT)/obj_unit \
		$(RTL_CORE_SRCS) \
		$(TB_DIR)/unit/pid_axis_core_tb.sv \
		-o $(ABS_OUTPUT)/Vpid_axis_core_tb
	$(ABS_OUTPUT)/Vpid_axis_core_tb

test-top: $(OUTPUT_DIR)
	@echo "=== [TEST-TOP] Running pid_top integrated testbench ==="
	$(VERILATOR) $(VERILATOR_SIM_FLAGS) \
		--top-module pid_top_tb \
		--Mdir $(ABS_OUTPUT)/obj_top \
		$(RTL_TOP_SRCS) \
		$(TB_DIR)/top/pid_top_tb.sv \
		-o $(ABS_OUTPUT)/Vpid_top_tb
	$(ABS_OUTPUT)/Vpid_top_tb

test-shared: $(OUTPUT_DIR)
	@echo "=== [TEST-SHARED] Running pid_top_shared_tb time-shared testbench ==="
	$(VERILATOR) $(VERILATOR_SIM_FLAGS) -I$(RTL_DIR)/shared \
		--top-module pid_top_shared_tb \
		--Mdir $(ABS_OUTPUT)/obj_shared \
		$(RTL_SHARED_SRCS) \
		$(TB_DIR)/top/pid_top_shared_tb.sv \
		-o $(ABS_OUTPUT)/Vpid_top_shared_tb
	$(ABS_OUTPUT)/Vpid_top_shared_tb

synth-shared:
	@echo "=== [SYNTH-SHARED] Running Yosys synthesis for time-shared SoC ==="
	$(YOSYS) -s $(SCRIPTS_DIR)/prelim_synth_shared.tcl

test-mixer: $(OUTPUT_DIR)
	@echo "=== [TEST-MIXER] Running pid_mixer unit testbench ==="
	$(VERILATOR) $(VERILATOR_SIM_FLAGS) \
		--top-module pid_mixer_tb \
		--Mdir $(ABS_OUTPUT)/obj_mixer \
		$(RTL_PKG) $(RTL_DIR)/safety/pid_mixer.sv \
		$(TB_DIR)/unit/pid_mixer_tb.sv \
		-o $(ABS_OUTPUT)/Vpid_mixer_tb
	$(ABS_OUTPUT)/Vpid_mixer_tb

test-soc: $(OUTPUT_DIR)
	@echo "=== [TEST-SOC] Running pid_soc_top integration testbench ==="
	$(VERILATOR) $(VERILATOR_SIM_FLAGS) \
		--top-module pid_soc_top_tb \
		--Mdir $(ABS_OUTPUT)/obj_soc \
		$(RTL_TOP_SRCS) $(RTL_DIR)/safety/pid_mixer.sv $(RTL_DIR)/pid_soc_top.sv \
		$(TB_DIR)/top/pid_soc_top_tb.sv \
		-o $(ABS_OUTPUT)/Vpid_soc_top_tb
	$(ABS_OUTPUT)/Vpid_soc_top_tb

test-stress: $(OUTPUT_DIR)
	@echo "=== [TEST-STRESS] Running pid_mode_stress_tb mode transition stress testbench ==="
	$(VERILATOR) $(VERILATOR_SIM_FLAGS) \
		--top-module pid_mode_stress_tb \
		--Mdir $(ABS_OUTPUT)/obj_stress \
		$(RTL_TOP_SRCS) \
		$(TB_DIR)/top/pid_mode_stress_tb.sv \
		-o $(ABS_OUTPUT)/Vpid_mode_stress_tb
	$(ABS_OUTPUT)/Vpid_mode_stress_tb

gen-vectors:
	@echo "=== [VECTORS] Generating test vectors from bit-accurate model ==="
	$(PYTHON) $(MODELS_DIR)/generate_vectors.py

sim-plant:
	@echo "=== [PLANT] Running 6-DOF Quadrotor Plant Simulation ==="
	$(PYTHON) $(MODELS_DIR)/flight_plant_sim.py

cosim-lib: $(OUTPUT_DIR)
	@echo "=== [COSIM] Building Verilated Shared Library libpid_top_cosim.so ==="
	@mkdir -p $(ABS_OUTPUT)/obj_cosim
	$(VERILATOR) --cc -O3 -CFLAGS "-O3 -fPIC" -Wno-VARHIDDEN -Wno-TIMESCALEMOD \
		$(VERILATOR_INCLUDES) --sv \
		--top-module pid_top \
		--Mdir $(ABS_OUTPUT)/obj_cosim \
		$(RTL_TOP_SRCS)
	g++ -O3 -fPIC -shared \
		-I$(ABS_OUTPUT)/obj_cosim \
		-I$(shell verilator -V | grep -m1 VERILATOR_ROOT | awk '{print $$3}')/include \
		-I$(shell verilator -V | grep -m1 VERILATOR_ROOT | awk '{print $$3}')/include/vltstd \
		$(shell verilator -V | grep -m1 VERILATOR_ROOT | awk '{print $$3}')/include/verilated.cpp \
		$(shell verilator -V | grep -m1 VERILATOR_ROOT | awk '{print $$3}')/include/verilated_threads.cpp \
		$(ABS_OUTPUT)/obj_cosim/Vpid_top*.cpp \
		$(TB_DIR)/cosim/pid_top_cosim.cpp \
		-o $(ABS_OUTPUT)/libpid_top_cosim.so
	@echo "=== [COSIM] libpid_top_cosim.so built successfully ==="

test-cosim: cosim-lib
	@echo "=== [COSIM] Running RTL-to-Plant Co-Simulation ==="
	PYTHONPATH=$(MODELS_DIR) $(PYTHON) $(MODELS_DIR)/flight_plant_cosim.py

regression: gen-vectors test-top
	@echo "=== [REGRESSION] Full Regression Complete ==="

test-formal:
	@echo "=== [FORMAL] Running SymbiYosys Formal Property Suite ==="
	$(SBY) -f $(TB_DIR)/formal/pid_formal.sby

cadence-flow:
	@echo "=== [CADENCE-FLOW] Running Sequential Cadence Flow for Parallel pid_top ==="
	./scripts/run_cadence_flow.sh parallel full

cadence-flow-shared:
	@echo "=== [CADENCE-FLOW] Running Sequential Cadence Flow for Time-Shared pid_top_shared ==="
	./scripts/run_cadence_flow.sh shared full

cadence-flow-fast:
	@echo "=== [CADENCE-FLOW] Running Fast Sequential Cadence Flow for Parallel pid_top ==="
	./scripts/run_cadence_flow.sh parallel fast

cadence-flow-shared-fast:
	@echo "=== [CADENCE-FLOW] Running Fast Sequential Cadence Flow for Time-Shared pid_top_shared ==="
	./scripts/run_cadence_flow.sh shared fast

clean:
	@echo "=== [CLEAN] Cleaning artifacts ==="
	@rm -rf $(OUTPUT_DIR) obj_dir/ pid_formal_*/ *.vcd *.fst *.log __pycache__
	@echo "=== [CLEAN] Done ==="
