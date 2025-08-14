# Project constants
PROJECT_NAME := core
BUILD_DIR := build

# Project files
TOP_VERILOG_SRC_FILE := verilog/core.sv
VERILOG_SRC_FILES := \
	verilog/core.sv \
	verilog/program_counter.sv \
	verilog/memory.sv \
	verilog/instruction_decode.sv \
	verilog/register_file.sv \
	verilog/alu.sv \
	verilog/adder4.sv \
	verilog/adder32.sv \
	verilog/shifter.sv \
	verilog/cells/mux2.sv \
	verilog/cells/mux2_2.sv \
	verilog/cells/mux2_3.sv \
	verilog/cells/mux2_4.sv \
	verilog/cells/mux2_stages.sv \
	verilog/cells/dff.sv \
	verilog/cells/dff_2.sv \
	verilog/cells/dff_3.sv \
	verilog/cells/dff_8.sv \
	verilog/cells/dff_stages.sv \
	verilog/csr.sv \
	verilog/fsm.sv \
	verilog/stages.sv

vpath %.sv verilog

$(BUILD_DIR):
	@mkdir -p $@

all: build_sim

test: sby run_sim compliance

CHILD_VERILOG_SRC_FILES = $(filter-out $(TOP_VERILOG_SRC_FILE),$(VERILOG_SRC_FILES))
pngs: $(foreach png,$(CHILD_VERILOG_SRC_FILES),$(BUILD_DIR)/$(notdir $(png:.sv=.png)))

clean:
	@echo "Removing build folder"
	@rm -rf $(BUILD_DIR)

.PHONY: all clean
.DEFAULT_GOAL := all

include make/env.mk
include make/synth.mk
include make/sim.mk
include make/sby.mk
