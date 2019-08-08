# Project constants
PROJECT_NAME := core
BUILD_DIR := build

# Makefile configuration
SBY_MODULE ?= core

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
	verilog/csr.sv \
	verilog/fsm.sv
SIM_SRC_FILES := \
	sim/sim_main.cpp \
	sim/decode.cpp \
	sim/mem.cpp
SIM_HEADER_FILES := \
	sim/decode.h \
	sim/mem.h
SIM_TEST_PROG_SRC_FILE := sim/test_prog.S
SIM_COMPLIANCE_SRC_DIR := sim/compliance/src
SIM_COMPLIANCE_INC_DIR := sim/compliance/inc
SIM_COMPLIANCE_REFERNCES_DIR := sim/compliance/references
SBY_CONFIG_FILE := scripts/verify.sby
YOSYS_SYNTH_SCRIPT_FILE = scripts/synth.ys
YOSYS_GATES_SCRIPT_FILE = scripts/gates.ys

# Generated variables
REPO_ROOT := $(shell git rev-parse --show-toplevel)
OBJ_DIR := $(BUILD_DIR)/obj
OUT_DIR := $(BUILD_DIR)/out
RESULT_DIR := $(BUILD_DIR)/result
GATES_PNG_OUT := $(OUT_DIR)/$(PROJECT_NAME).png
GATES_JSON_OUT := $(OUT_DIR)/$(PROJECT_NAME).json
SIM_EXE_OUT := $(OUT_DIR)/$(PROJECT_NAME).exe
SIM_TEST_PROG_OUT := $(OUT_DIR)/test_prog.bin

# Project configuration
VERILATOR_MODULE_NAME := Vmodule
VERILATOR_ARGS := \
	--exe \
	-CFLAGS -Wno-attributes \
	$(addprefix -CFLAGS -I,$(dir $(abspath $(SIM_HEADER_FILES)))) \
	--top-module $(PROJECT_NAME) \
	--cc \
	-CFLAGS -g3 \
	-Mdir $(OBJ_DIR) \
	-Wall \
	--prefix $(VERILATOR_MODULE_NAME) \
	$(addprefix -I,$(dir $(VERILOG_SRC_FILES)))

# Executables
DOT := dot
VERILATOR := verilator
SBY := sby
YOSYS := yosys
RISCV_CC := riscv-none-embed-gcc
RISCV_LD := riscv-none-embed-ld
RISCV_OBJCOPY := riscv-none-embed-objcopy

directories: $(OBJ_DIR) $(OUT_DIR) $(RESULT_DIR)
$(OBJ_DIR):
	@mkdir -p $@
$(OUT_DIR):
	@mkdir -p $@
$(RESULT_DIR):
	@mkdir -p $@

gates_json: directories $(GATES_JSON_OUT)
$(GATES_JSON_OUT): $(OBJ_DIR)/$(PROJECT_NAME).gates.json
	@echo "Copying output $(notdir $@)"
	@cp $< $@
%.json: %.ilang
	@echo "Building $(notdir $@)"
	@$(YOSYS) -q -f ilang -b json -o $@ $^
%.gates.ilang: %.synth.ilang
	@echo "Building $(notdir $@)"
	@$(YOSYS) -q -f ilang -b ilang -s $(YOSYS_GATES_SCRIPT_FILE) -o $@ $^
%.synth.ilang: $(TOP_VERILOG_SRC_FILE)
	@echo "Building $(notdir $@)"
	@$(YOSYS) -q -f verilog -b ilang -s $(YOSYS_SYNTH_SCRIPT_FILE) -o $@ $^
gates_png: directories $(GATES_PNG_OUT)
$(GATES_PNG_OUT): $(OBJ_DIR)/$(PROJECT_NAME).gates.png
	@echo "Copying output $(notdir $@)"
	@cp $< $@
%.png: %.dot
	@echo "Building $(notdir $@)"
	@$(DOT) -Tpng $(shell cygpath -w $<) -o $(shell cygpath -w $@)
%.dot: %.ilang
	@echo "Building $(notdir $@)"
	@$(YOSYS) -q -f ilang -p "show -width -notitle -format dot -prefix $(basename $@)" $^

build_sim: directories $(SIM_EXE_OUT) $(SIM_TEST_PROG_OUT)
run_sim: build_sim
	@$(SIM_EXE_OUT) $(SIM_TEST_PROG_OUT)
$(SIM_EXE_OUT): $(OBJ_DIR)/$(VERILATOR_MODULE_NAME).exe
	@echo "Copying output $(notdir $@)"
	@cp $< $@
$(OBJ_DIR)/$(VERILATOR_MODULE_NAME).exe: $(VERILOG_SRC_FILES) $(SIM_SRC_FILES) $(SIM_HEADER_FILES)
	@echo "Building $(notdir $@)"
	@$(VERILATOR) $(VERILATOR_ARGS) $(TOP_VERILOG_SRC_FILE) $(abspath $(SIM_SRC_FILES))
	@$(MAKE) -s -C $(OBJ_DIR)/ -f $(VERILATOR_MODULE_NAME).mk
$(SIM_TEST_PROG_OUT): $(OBJ_DIR)/test_prog.elf
	@echo "Copying $(notdir $@)"
	@$(RISCV_OBJCOPY) -O binary $< $@
$(OBJ_DIR)/test_prog.elf: $(OBJ_DIR)/test_prog.o sim/link.ld
	@echo "Linking $(notdir $@)"
	@$(RISCV_LD) -T sim/link.ld -nostartfiles $< -o $@
$(OBJ_DIR)/test_prog.o: $(SIM_TEST_PROG_SRC_FILE)
	@echo "Compiling $(notdir $@)"
	@$(RISCV_CC) -march=rv32e -c -mabi=ilp32e -nostdlib -nodefaultlibs -nostartfiles $^ -o $@

COMPLIANCE_SRCS := $(wildcard $(SIM_COMPLIANCE_SRC_DIR)/*)
COMPLIANCE_BINS := $(addprefix $(OBJ_DIR)/,$(notdir $(COMPLIANCE_SRCS:.S=.bin)))
COMPLIANCE_RESULTS := $(addprefix $(RESULT_DIR)/,$(notdir $(COMPLIANCE_SRCS:.S=.txt)))
compliance: directories $(SIM_EXE_OUT) $(COMPLIANCE_RESULTS)
$(RESULT_DIR)/%.txt: $(OBJ_DIR)/%.bin $(SIM_EXE_OUT)
	@echo "Running test: $(notdir $(<:.bin=.S))"
	@$(SIM_EXE_OUT) $< $(addprefix $(SIM_COMPLIANCE_REFERNCES_DIR)/,$(notdir $(<:.bin=.reference_output))) > $@
$(OBJ_DIR)/%.bin: $(OBJ_DIR)/%.elf
	@echo "Copying $(notdir $@)"
	@$(RISCV_OBJCOPY) -O binary $< $@
$(OBJ_DIR)/%.elf: $(OBJ_DIR)/%.o sim/link.ld
	@echo "Linking $(notdir $@)"
	@$(RISCV_LD) -T sim/link.ld -nostartfiles $< -o $@
$(OBJ_DIR)/%.o: $(SIM_COMPLIANCE_SRC_DIR)/%.S $(wildcard $(SIM_COMPLIANCE_INC_DIR)/*)
	@echo "Compiling $(notdir $@)"
	@$(RISCV_CC) -march=rv32e -c -mabi=ilp32e -nostdlib -nodefaultlibs -nostartfiles -I$(SIM_COMPLIANCE_INC_DIR) $< -o $@

sby:
	@$(SBY) -d $(OBJ_DIR)/sby -f $(SBY_CONFIG_FILE) $(SBY_MODULE)

all: build_sim gates_json

test: sby run_sim

clean:
	@echo "Removing build folder"
	@rm -rf $(BUILD_DIR)

.PRECIOUS: %.synth.ilang %.gates.ilang %.dot %.json $(OBJ_DIR)/%.o $(OBJ_DIR)/%.elf $(OBJ_DIR)/%.bin
.PHONY: gates_png gates_json build_sim run_sim sby all clean directories
.DEFAULT_GOAL := all
