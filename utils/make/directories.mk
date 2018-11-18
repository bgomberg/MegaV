ifndef BUILD_DIR
$(error "BUILD_DIR is not set!")
endif

OBJ_DIR := $(BUILD_DIR)/obj
OUT_DIR := $(BUILD_DIR)/out

$(OBJ_DIR):
	@mkdir -p $@

$(OUT_DIR):
	@mkdir -p $@

directories: $(OBJ_DIR) $(OUT_DIR)

.PHONY: directories
