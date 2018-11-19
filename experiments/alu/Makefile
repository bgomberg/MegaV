REPO_ROOT := $(shell git rev-parse --show-toplevel)
PROJECT_NAME := alu
BUILD_DIR := build

VERILOG_SRC_FILES := alu.sv
SIM_SRC_FILES := $(abspath sim_main.cpp)
SIM_HEADER_FILES := $(abspath $(REPO_ROOT)/lib/verilator/core_wrapper.hpp)

include $(REPO_ROOT)/utils/make/directories.mk
include $(REPO_ROOT)/utils/make/dot.mk
include $(REPO_ROOT)/utils/make/verilator.mk
include $(REPO_ROOT)/utils/make/yosys.mk

GATES_PNG_OUT := $(OUT_DIR)/$(PROJECT_NAME).png
GATES_JSON_OUT := $(OUT_DIR)/$(PROJECT_NAME).json
SIM_EXE_OUT := $(OUT_DIR)/$(PROJECT_NAME).exe

$(GATES_PNG_OUT): $(OBJ_DIR)/$(PROJECT_NAME).gates.png
	@echo "Copying output $(notdir $@)"
	@cp $< $@

gates_png: directories $(GATES_PNG_OUT)

$(GATES_JSON_OUT): $(OBJ_DIR)/$(PROJECT_NAME).gates.json
	@echo "Copying output $(notdir $@)"
	@cp $< $@

gates_json: directories $(GATES_JSON_OUT)

$(SIM_EXE_OUT): $(OBJ_DIR)/Vmodule.exe
	@echo "Copying output $(notdir $@)"
	@cp $< $@

build_sim: directories $(SIM_EXE_OUT)

run_sim: build_sim
	@$(SIM_EXE_OUT)

sby:
	@sby -d $(OBJ_DIR)/sby -f verify.sby

all: gates_png gates_json build_sim

clean:
	@echo "Removing build folder"
	@rm -rf $(BUILD_DIR)

.PHONY: gates_png gates_json build_sim run_sim sby all clean
.DEFAULT_GOAL := all
