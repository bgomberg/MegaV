VERILATOR := verilator

VERILATOR_MODULE_NAME := Vmodule

ifndef VERILOG_SRC_FILES
$(error "VERILOG_SRC_FILES is not set!")
endif
ifndef SIM_SRC_FILES
$(error "SIM_SRC_FILES is not set!")
endif

$(OBJ_DIR)/$(VERILATOR_MODULE_NAME).exe: $(VERILOG_SRC_FILES) $(SIM_SRC_FILES) $(SIM_HEADER_FILES)
	@echo "Building $(notdir $@)"
	@$(VERILATOR) --top-module $(PROJECT_NAME) --cc -Mdir $(OBJ_DIR) -Wall --prefix $(VERILATOR_MODULE_NAME) -CFLAGS -Wno-attributes $(addprefix -CFLAGS -I,$(dir $(filter %.hpp,$^))) $(filter %.sv,$^) --exe $(filter %.cpp,$^)
	@$(MAKE) -s -C $(OBJ_DIR)/ -f $(VERILATOR_MODULE_NAME).mk
