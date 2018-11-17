YOSYS := yosys

ifeq ($(REPO_ROOT),)
$(error "REPO_ROOT is not set!")
endif

%.synth.ilang: $(SRC_FILES)
	@echo "Building $(notdir $@)"
	@$(YOSYS) -q -f verilog -b ilang -s $(REPO_ROOT)/utils/yosys/synth.ys -o $@ $^

%.gates.ilang: %.synth.ilang
	@echo "Building $(notdir $@)"
	@$(YOSYS) -q -f ilang -b ilang -s $(REPO_ROOT)/utils/yosys/gates.ys -o $@ $^

%.dot: %.ilang
	@echo "Building $(notdir $@)"
	@$(YOSYS) -q -f ilang -p "show -width -notitle -format dot -prefix $(basename $@)" $^

%.json: %.ilang
	@echo "Building $(notdir $@)"
	@$(YOSYS) -q -f ilang -b json -o $@ $^

.PRECIOUS: %.synth.ilang %.gates.ilang %.dot %.json
