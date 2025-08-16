
SBY_MODULES := \
	program_counter \
	register_file \
	alu \
	csr \
	memory \
	instruction_decode \
	fsm \
	adder4 \
	adder32 \
	shifter \
	and2 \
	or2 \
	nor4 \
	mux2 \
	dff \
	dffe \
	xor2
SBY_OBJ_DIR = $(BUILD_DIR)/sby
SBY_RESULTS := $(addprefix $(SBY_OBJ_DIR)/,$(SBY_MODULES:=.pass))
SBY_CONFIG_FILE := scripts/verify.sby

$(SBY_OBJ_DIR):
	@mkdir -p $@

$(SBY_OBJ_DIR)/%.pass: verilog/cells/%.sv | $(SBY_OBJ_DIR)
	@$(SBY) -d $(SBY_OBJ_DIR)/$(basename $(notdir $@)) -f $(SBY_CONFIG_FILE) $(basename $(notdir $@))
	@cp $(SBY_OBJ_DIR)/$(basename $(notdir $@))/PASS $@

$(SBY_OBJ_DIR)/%.pass: verilog/%.sv | $(SBY_OBJ_DIR)
	@$(SBY) -d $(SBY_OBJ_DIR)/$(basename $(notdir $@)) -f $(SBY_CONFIG_FILE) $(basename $(notdir $@))
	@cp $(SBY_OBJ_DIR)/$(basename $(notdir $@))/PASS $@

sby: $(SBY_RESULTS)

.PHONY: sby
