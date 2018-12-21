REPO_ROOT := $(shell git rev-parse --show-toplevel)
PROJECT_NAME := memory

VERILOG_SRC_FILES := memory.sv
SIM_SRC_FILES := $(abspath sim_main.cpp)
SIM_HEADER_FILES := $(abspath $(REPO_ROOT)/lib/verilator/core_wrapper.hpp)

include $(REPO_ROOT)/utils/make/project_template.mk
