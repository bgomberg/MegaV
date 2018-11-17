REPO_ROOT := $(shell git rev-parse --show-toplevel)
PROJECT_NAME := basic
BUILD_DIR := build
OBJ_DIR := $(BUILD_DIR)/obj
OUT_DIR := $(BUILD_DIR)/out

SRC_FILES := basic.sv

include $(REPO_ROOT)/utils/make/yosys.mk
include $(REPO_ROOT)/utils/make/dot.mk

$(OUT_DIR)/$(PROJECT_NAME).png: $(OBJ_DIR)/$(PROJECT_NAME).gates.png
	@echo "Copying to $(notdir $@)"
	@cp $< $@

$(OUT_DIR)/$(PROJECT_NAME).json: $(OBJ_DIR)/$(PROJECT_NAME).gates.json
	@echo "Copying to $(notdir $@)"
	@cp $< $@

$(OBJ_DIR):
	@mkdir -p $@

$(OUT_DIR):
	@mkdir -p $@

directories: $(OBJ_DIR) $(OUT_DIR)

all: directories $(OUT_DIR)/$(PROJECT_NAME).png $(OUT_DIR)/$(PROJECT_NAME).json

clean:
	@rm -rf $(BUILD_DIR)

.PHONY: directories all clean
.DEFAULT_GOAL := all
