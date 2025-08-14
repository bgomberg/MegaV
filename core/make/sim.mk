
SIM_SRC_FILES := \
	sim/sim_main.cpp \
	sim/decode.cpp \
	sim/mem.cpp
SIM_HEADER_FILES := \
	sim/decode.h \
	sim/mem.h
SIM_TEST_PROG_SRC_FILE := sim/test_prog.S
SIM_COMPLIANCE_INC_DIR := sim/compliance/inc
SIM_COMPLIANCE_REFERNCES_DIR := sim/compliance/references
SIM_OUT_DIR := $(BUILD_DIR)/sim/out
SIM_OBJ_DIR := $(BUILD_DIR)/sim/obj
RESULT_DIR := $(BUILD_DIR)/sim/result
SIM_EXE_OUT := $(SIM_OUT_DIR)/$(PROJECT_NAME)
SIM_TEST_PROG_OUT := $(SIM_OUT_DIR)/test_prog.bin

VERILATOR_MODULE_NAME := Vmodule
VERILATOR_ARGS := \
	--exe \
	-CFLAGS -Wno-attributes \
	$(addprefix -CFLAGS -I,$(dir $(abspath $(SIM_HEADER_FILES)))) \
	--top-module $(PROJECT_NAME) \
	--cc \
	-CFLAGS -g3 \
	-Mdir $(SIM_OBJ_DIR) \
	-Wall \
	--prefix $(VERILATOR_MODULE_NAME) \
	-Iverilog \
	-Iverilog/cells

COMPLIANCE_SRC_DIR := sim/compliance/src
COMPLIANCE_SRCS := $(wildcard $(COMPLIANCE_SRC_DIR)/*)
COMPLIANCE_BINS := $(addprefix $(SIM_OBJ_DIR)/,$(notdir $(COMPLIANCE_SRCS:.S=.bin)))
COMPLIANCE_RESULTS := $(addprefix $(RESULT_DIR)/,$(notdir $(COMPLIANCE_SRCS:.S=.txt)))

$(SIM_OUT_DIR):
	@mkdir -p $@
$(SIM_OBJ_DIR):
	@mkdir -p $@
$(RESULT_DIR):
	@mkdir -p $@

build_sim: $(SIM_EXE_OUT) $(SIM_TEST_PROG_OUT)
run_sim: build_sim
	@$(SIM_EXE_OUT) $(SIM_TEST_PROG_OUT)
$(SIM_EXE_OUT): $(SIM_OBJ_DIR)/$(VERILATOR_MODULE_NAME) | $(SIM_OUT_DIR)
	@echo "Copying output $(notdir $@)"
	@cp $< $@
$(SIM_OBJ_DIR)/$(VERILATOR_MODULE_NAME): $(VERILOG_SRC_FILES) $(SIM_SRC_FILES) $(SIM_HEADER_FILES) | $(SIM_OBJ_DIR)
	@echo "Building $(notdir $@)"
	@$(VERILATOR) $(VERILATOR_ARGS) $(TOP_VERILOG_SRC_FILE) $(abspath $(SIM_SRC_FILES))
	@$(MAKE) -s -C $(SIM_OBJ_DIR)/ -f $(VERILATOR_MODULE_NAME).mk
$(SIM_TEST_PROG_OUT): $(SIM_OBJ_DIR)/test_prog.elf
	@echo "Copying $(notdir $@)"
	@$(RISCV_OBJCOPY) -O binary $< $@
$(SIM_OBJ_DIR)/test_prog.elf: $(SIM_OBJ_DIR)/test_prog.o sim/link.ld
	@echo "Linking $(notdir $@)"
	@$(RISCV_LD) -T sim/link.ld $< -o $@
$(SIM_OBJ_DIR)/test_prog.o: $(SIM_TEST_PROG_SRC_FILE)
	@echo "Compiling $(notdir $@)"
	@$(RISCV_CC) -march=rv32e_zicsr_zifencei -c -mabi=ilp32e -nostdlib -nodefaultlibs -nostartfiles $^ -o $@

compliance: $(RESULT_DIR) $(SIM_EXE_OUT) $(COMPLIANCE_RESULTS)
$(RESULT_DIR)/%.txt: $(SIM_OBJ_DIR)/%.bin $(SIM_EXE_OUT)
	@echo "Running test: $(notdir $(<:.bin=.S))"
	@$(SIM_EXE_OUT) $< $(addprefix $(SIM_COMPLIANCE_REFERNCES_DIR)/,$(notdir $(<:.bin=.reference_output))) > $@ || { cat $@; rm $@; }
$(SIM_OBJ_DIR)/%.bin: $(SIM_OBJ_DIR)/%.elf
	@echo "Copying $(notdir $@)"
	@$(RISCV_OBJCOPY) -O binary $< $@
$(SIM_OBJ_DIR)/%.elf: $(SIM_OBJ_DIR)/%.o sim/link.ld
	@echo "Linking $(notdir $@)"
	@$(RISCV_LD) -T sim/link.ld $< -o $@
$(SIM_OBJ_DIR)/%.o: $(COMPLIANCE_SRC_DIR)/%.S $(wildcard $(SIM_COMPLIANCE_INC_DIR)/*)
	@echo "Compiling $(notdir $@)"
	@$(RISCV_CC) -march=rv32e_zicsr_zifencei -c -mabi=ilp32e -nostdlib -nodefaultlibs -nostartfiles -I$(SIM_COMPLIANCE_INC_DIR) $< -o $@

.PRECIOUS: $(SIM_OBJ_DIR)/%.o $(SIM_OBJ_DIR)/%.elf $(SIM_OBJ_DIR)/%.bin
.PHONY: build_sim run_sim compliance
