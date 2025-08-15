# Project constants
PROJECT_NAME := core
BUILD_DIR := build

# Project files
TOP_VERILOG_SRC_FILE := verilog/core.sv
VERILOG_SRC_FILES := \
	$(wildcard verilog/*.sv) \
	$(wildcard verilog/cells/*.sv)

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
