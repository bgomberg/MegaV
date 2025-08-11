YOSYS_SYNTH_SCRIPT_FILE = scripts/synth.ys
SYNTH_DIR := $(BUILD_DIR)/synth

$(SYNTH_DIR):
	@mkdir -p $@

$(SYNTH_DIR)/%.synth.ys: %.sv | $(SYNTH_DIR)
	@echo "Creating $(notdir $@)"
	@echo "read_verilog -sv $<" > $@
	@cat $(YOSYS_SYNTH_SCRIPT_FILE) >> $@

$(SYNTH_DIR)/%.rtlil: %.sv $(SYNTH_DIR)/%.synth.ys
	@echo "Building $(notdir $@)"
	@$(YOSYS) -q -f verilog -b rtlil -s $(SYNTH_DIR)/${basename $(notdir $<)}.synth.ys -o $@

$(SYNTH_DIR)/%.dot: $(SYNTH_DIR)/%.rtlil | $(SYNTH_DIR)
	@echo "Building $(notdir $@)"
	@$(YOSYS) -q -f rtlil -p "prep -top $(basename $(notdir $<)); show -stretch -colors 1 -width -notitle -format dot -prefix $(basename $@)" $^
	@python scripts/dot_helper.py $@

$(BUILD_DIR)/%.png: $(SYNTH_DIR)/%.dot | $(BUILD_DIR)
	@echo "Building $(notdir $@)"
	@$(DOT) -Tpng $< -o $@ >/dev/null

.PRECIOUS: $(SYNTH_DIR)/%.rtlil
