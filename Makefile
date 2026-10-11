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

.PHONY: all help clean lint test-unit test-top test-mixer test-soc test-stress test-fixed-point test-mixer-edge test-safety-edge test-edge-all test-axi test-apb test-notch test-matrix test-2dof test-advanced test-formal gen-vectors sim-plant regression

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

test-fixed-point: $(OUTPUT_DIR)
	@echo "=== [TEST-FIXED-POINT] Running pid_fixed_point_tb boundary and consumer testbench ==="
	$(VERILATOR) $(VERILATOR_SIM_FLAGS) \
		--top-module pid_fixed_point_tb \
		--Mdir $(ABS_OUTPUT)/obj_fp \
		$(RTL_PKG) \
		$(RTL_DIR)/core/pid_mult.sv \
		$(RTL_DIR)/core/pid_deriv_filter.sv \
		$(RTL_DIR)/core/pid_integ_unit.sv \
		$(RTL_DIR)/core/pid_sat_unit.sv \
		$(RTL_DIR)/modes/pid_outer_loop.sv \
		$(RTL_DIR)/modes/pid_gain_bank.sv \
		$(RTL_DIR)/safety/pid_tilt_comp.sv \
		$(TB_DIR)/unit/pid_fixed_point_tb.sv \
		-o $(ABS_OUTPUT)/Vpid_fixed_point_tb
	$(ABS_OUTPUT)/Vpid_fixed_point_tb

test-mixer-edge: $(OUTPUT_DIR)
	@echo "=== [TEST-MIXER-EDGE] Running pid_mixer_edge_tb edge and distortion testbench ==="
	$(VERILATOR) $(VERILATOR_SIM_FLAGS) \
		--top-module pid_mixer_edge_tb \
		--Mdir $(ABS_OUTPUT)/obj_mixer_edge \
		$(RTL_PKG) $(RTL_DIR)/safety/pid_mixer.sv \
		$(TB_DIR)/unit/pid_mixer_edge_tb.sv \
		-o $(ABS_OUTPUT)/Vpid_mixer_edge_tb
	$(ABS_OUTPUT)/Vpid_mixer_edge_tb

test-safety-edge: $(OUTPUT_DIR)
	@echo "=== [TEST-SAFETY-EDGE] Running pid_safety_edge_tb safety and mode transition testbench ==="
	$(VERILATOR) $(VERILATOR_SIM_FLAGS) \
		--top-module pid_safety_edge_tb \
		--Mdir $(ABS_OUTPUT)/obj_safety_edge \
		$(RTL_SOC_SRCS) \
		$(TB_DIR)/top/pid_safety_edge_tb.sv \
		-o $(ABS_OUTPUT)/Vpid_safety_edge_tb
	$(ABS_OUTPUT)/Vpid_safety_edge_tb

test-edge-all: test-fixed-point test-mixer-edge test-safety-edge
	@echo "=== [EDGE-ALL] All edge-case verification testbenches passed ==="

test-axi: $(OUTPUT_DIR)
	@echo "=== [TEST-AXI] Running pid_soc_axi_top_tb AXI4-Lite integration testbench ==="
	$(VERILATOR) $(VERILATOR_SIM_FLAGS) -I$(RTL_DIR)/bus \
		--top-module pid_soc_axi_top_tb \
		--Mdir $(ABS_OUTPUT)/obj_axi \
		$(RTL_SOC_SRCS) $(RTL_DIR)/bus/pid_axi_lite_bridge.sv $(RTL_DIR)/pid_soc_axi_top.sv \
		$(TB_DIR)/top/pid_soc_axi_top_tb.sv \
		-o $(ABS_OUTPUT)/Vpid_soc_axi_top_tb
	$(ABS_OUTPUT)/Vpid_soc_axi_top_tb

test-apb: $(OUTPUT_DIR)
	@echo "=== [TEST-APB] Running pid_soc_apb_top_tb APB4 integration testbench ==="
	$(VERILATOR) $(VERILATOR_SIM_FLAGS) -I$(RTL_DIR)/bus \
		--top-module pid_soc_apb_top_tb \
		--Mdir $(ABS_OUTPUT)/obj_apb \
		$(RTL_SOC_SRCS) $(RTL_DIR)/bus/pid_apb4_bridge.sv $(RTL_DIR)/bus/pid_soc_apb_top.sv \
		$(TB_DIR)/top/pid_soc_apb_top_tb.sv \
		-o $(ABS_OUTPUT)/Vpid_soc_apb_top_tb
	$(ABS_OUTPUT)/Vpid_soc_apb_top_tb

test-notch: $(OUTPUT_DIR)
	@echo "=== [TEST-NOTCH] Running pid_gyro_notch_tb Biquad filter testbench ==="
	$(VERILATOR) $(VERILATOR_SIM_FLAGS) -I$(RTL_DIR)/dsp \
		--top-module pid_gyro_notch_tb \
		--Mdir $(ABS_OUTPUT)/obj_notch \
		$(RTL_PKG) $(RTL_DIR)/dsp/pid_gyro_notch.sv \
		$(TB_DIR)/unit/pid_gyro_notch_tb.sv \
		-o $(ABS_OUTPUT)/Vpid_gyro_notch_tb
	$(ABS_OUTPUT)/Vpid_gyro_notch_tb

test-matrix: $(OUTPUT_DIR)
	@echo "=== [TEST-MATRIX] Running pid_mixer_matrix_tb multi-rotor mixer testbench ==="
	$(VERILATOR) $(VERILATOR_SIM_FLAGS) -I$(RTL_DIR)/safety \
		--top-module pid_mixer_matrix_tb \
		--Mdir $(ABS_OUTPUT)/obj_matrix \
		$(RTL_PKG) $(RTL_DIR)/safety/pid_mixer_matrix.sv \
		$(TB_DIR)/unit/pid_mixer_matrix_tb.sv \
		-o $(ABS_OUTPUT)/Vpid_mixer_matrix_tb
	$(ABS_OUTPUT)/Vpid_mixer_matrix_tb

test-2dof: $(OUTPUT_DIR)
	@echo "=== [TEST-2DOF] Running pid_axis_core_2dof_tb 2-DOF PID engine testbench ==="
	$(VERILATOR) $(VERILATOR_SIM_FLAGS) \
		--top-module pid_axis_core_2dof_tb \
		--Mdir $(ABS_OUTPUT)/obj_2dof \
		$(RTL_PKG) $(RTL_DIR)/core/pid_mult.sv $(RTL_DIR)/core/pid_deriv_filter.sv $(RTL_DIR)/core/pid_integ_unit.sv $(RTL_DIR)/core/pid_sat_unit.sv $(RTL_DIR)/core/pid_axis_core_2dof.sv \
		$(TB_DIR)/unit/pid_axis_core_2dof_tb.sv \
		-o $(ABS_OUTPUT)/Vpid_axis_core_2dof_tb
	$(ABS_OUTPUT)/Vpid_axis_core_2dof_tb

test-advanced: test-axi test-apb test-notch test-matrix test-2dof
	@echo "=== [ADVANCED-ALL] All post-hackathon advanced features verified 100% PASS ==="


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

# Multi-Variant & Cadence Integration Targets
.PHONY: test-shared test-16bit synth-shared cadence-all cadence-16bit cadence-32bit test-all

test-shared:
	@echo "=== [TEST-SHARED] Running 16-bit time-shared datapath testbench ==="
	$(MAKE) -C PID_16bit test-shared

test-16bit:
	@echo "=== [TEST-16BIT] Running all 16-bit verification targets ==="
	$(MAKE) -C PID_16bit test-unit
	$(MAKE) -C PID_16bit test-top
	$(MAKE) -C PID_16bit test-shared
	$(MAKE) -C PID_16bit test-mixer
	$(MAKE) -C PID_16bit test-soc
	$(MAKE) -C PID_16bit test-stress
	@echo "=== [TEST-16BIT] All 16-bit targets PASSED! ==="

synth-shared:
	@echo "=== [SYNTH-SHARED] Running Yosys synthesis for time-shared SoC ==="
	$(MAKE) -C PID_16bit synth-shared

cadence-all:
	@echo "=== [CADENCE-ALL] Running all Cadence P&R runs ==="
	./run_all_cases.sh

cadence-16bit:
	@echo "=== [CADENCE-16BIT] Running both 16-bit P&R runs ==="
	./PID_16bit/scripts/run_pnr_both_cases.sh

cadence-32bit:
	@echo "=== [CADENCE-32BIT] Running 32-bit Q16.16 Cadence flow ==="
	./PID_Q16_16/scripts/run_cadence_flow.sh parallel full

test-all: test-unit test-top test-mixer test-soc test-stress test-edge-all test-16bit
	@echo "=== [TEST-ALL] All 32-bit and 16-bit verification targets PASSED! ==="

clean:
	@echo "=== [CLEAN] Cleaning artifacts ==="
	@rm -rf $(OUTPUT_DIR) obj_dir/ pid_formal_*/ *.vcd *.fst *.log __pycache__
	$(MAKE) -C PID_Q16_16 clean
	$(MAKE) -C PID_16bit clean
	@echo "=== [CLEAN] Done ==="
