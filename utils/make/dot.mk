DOT := dot

%.png: %.dot
	@echo "Building $(notdir $@)"
	@$(DOT) -Tpng $(shell cygpath -w $<) -o $(shell cygpath -w $@)
