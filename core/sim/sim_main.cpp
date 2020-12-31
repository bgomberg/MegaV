#include "decode.h"
#include "mem.h"

#include <Vmodule.h>
#include <Vmodule_core.h>
#include <Vmodule_memory.h>
#include <Vmodule_register_file.h>
#include <Vmodule_fsm.h>

#include <verilated.h>

#include <stdio.h>


#define STAGE_CONTROL 0
#define STAGE_FETCH 1
#define STAGE_DECODE 2
#define STAGE_READ 3
#define STAGE_EXECUTE 4
#define STAGE_MEMORY 5
#define STAGE_WRITE_BACK 6
#define STAGE_UPDATE_PC 7
#define NUM_STAGES 8


static const char* const STAGE_STRS[NUM_STAGES] = {
  "STAGE_CONTROL",
  "STAGE_FETCH",
  "STAGE_DECODE",
  "STAGE_READ",
  "STAGE_EXECUTE",
  "STAGE_MEMORY",
  "STAGE_WRITE_BACK",
  "STAGE_UPDATE_PC",
};


class Core {
public:
  Core(bool compliance_mode) : module_(new Vmodule()), compliance_mode_(compliance_mode) {
    module_->clk = 0;
    module_->reset_n = 1;
    module_->ext_int = 0;
  }

  ~Core() {
    delete module_;
    module_ = NULL;
  }

  Vmodule_core* operator->() {
    return module_->core;
  }

  void reset() {
    printf("Resetting core\n");
    eval();
    module_->reset_n = 0;
    tick();
    module_->reset_n = 1;
  }

  void set_ext_int(bool value) {
    module_->ext_int = value;
  }

  void microstep() {
    // Print info on the stage we're about to execute
    const int prev_stage_num = get_stage_num();
    if (!compliance_mode_) {
      printf("Executing %s\n", STAGE_STRS[prev_stage_num]);
      printf("  Inputs:\n");
      switch (prev_stage_num) {
      case STAGE_CONTROL:
        printf("    fault_num: %d\n", (*this)->fault_num);
        break;
      case STAGE_FETCH:
        printf("    control_op: 0x%x\n", (*this)->control_op);
        printf("    pc: 0x%x\n", (*this)->pc_pc);
        break;
      case STAGE_DECODE:
        printf("    instr: 0x%x [", (*this)->instr);
        print_decoded_instr((*this)->instr);
        printf("]\n");
        break;
      case STAGE_READ:
        break;
      case STAGE_EXECUTE:
        printf("    csr_addr_exception: 0x%x\n", (*this)->csr_addr_exception);
        printf("    csr_in: 0x%x\n", (*this)->csr_in);
        printf("    csr_read_value: 0x%x\n", (*this)->csr_read_value);
        printf("    next_pc: 0x%x\n", (*this)->pc_next_pc);
        break;
      case STAGE_MEMORY:
        break;
      case STAGE_WRITE_BACK:
        printf("    write_data: 0x%x\n", (*this)->rf_write_data);
        break;
      case STAGE_UPDATE_PC:
        printf("    next_pc: 0x%x\n", (*this)->pc_next_pc);
        break;
      default:
        fprintf(stderr, "Unknown stage!!!\n");
        exit(1);
        break;
      }
    }

    // Move to the next stage
    while (get_stage_num() == prev_stage_num) {
      tick();
    }

    if (!compliance_mode_) {
      printf("  Outputs:\n");
      switch (prev_stage_num) {
      case STAGE_CONTROL:
        break;
      case STAGE_FETCH:
        break;
      case STAGE_DECODE:
        printf("    imm: 0x%x\n", (*this)->decode_imm);
        printf("    alu_a_mux_position: 0x%x\n", (*this)->decode_alu_a_mux_position);
        printf("    alu_b_mux_position: 0x%x\n", (*this)->decode_alu_b_mux_position);
        printf("    csr_mux_position: 0x%x\n", (*this)->decode_csr_mux_position);
        printf("    wb_pc_mux_position: 0x%x\n", (*this)->decode_wb_pc_mux_position);
        printf("    wb_mux_position: 0x%x\n", (*this)->decode_wb_mux_position);
        printf("    rd_rf_microcode: 0x%x\n", (*this)->decode_rd_rf_microcode);
        printf("    ex_alu_microcode: 0x%x\n", (*this)->decode_ex_alu_microcode);
        printf("    ex_csr_microcode: 0x%x\n", (*this)->decode_ex_csr_microcode);
        printf("    ma_mem_microcode: 0x%x\n", (*this)->decode_ma_mem_microcode);
        printf("    wb_rf_microcode : 0x%x\n", (*this)->decode_wb_rf_microcode);
        printf("    next_pc: 0x%x\n", (*this)->pc_next_pc);
        break;
      case STAGE_READ:
        printf("    read_data_a: 0x%x\n", (*this)->rf_read_data_a);
        printf("    read_data_b: 0x%x\n", (*this)->rf_read_data_b);
        break;
      case STAGE_EXECUTE:
        printf("    alu_out: 0x%x\n", (*this)->alu_out);
        printf("    csr_read_value: 0x%x\n", (*this)->csr_read_value);
        break;
      case STAGE_MEMORY:
        printf("    out: 0x%x\n", (*this)->mem_out);
        printf("    mem_op_fault: 0x%x\n", (*this)->mem_op_fault);
        printf("    mem_addr_fault: 0x%x\n", (*this)->mem_addr_fault);
        printf("    mem_access_fault: 0x%x\n", (*this)->mem_access_fault);
        break;
      case STAGE_WRITE_BACK:
        break;
      case STAGE_UPDATE_PC:
        break;
      default:
        fprintf(stderr, "Unknown stage!!!\n");
        exit(1);
        break;
      }
    }
  }

  void step() {
    do {
      microstep();
    } while ((*this)->stage_active != (1 << 0));
    if (!compliance_mode_) {
      printf("\n");
    }
  }

private:
  Vmodule* module_;
  bool compliance_mode_;

  void tick() {
    module_->clk = 1;
    eval();
    module_->clk = 0;
    eval();
  }

  void eval() {
    module_->eval();
  }

  int get_stage_num() {
    const uint32_t stage = (*this)->stage_active;
    int stage_num = -1;
    for (int i = 0; i < NUM_STAGES; i++) {
      if (stage == (1 << i)) {
        stage_num = i;
        break;
      }
    }
    if (stage_num == -1) {
      fprintf(stderr, "Invalid stage: 0x%x\n", stage);
      exit(1);
    }
    return stage_num;
  }
};


int main(int argc, char** argv) {
  const bool compliance_mode = argc == 3;
  if (argc != 2 && !compliance_mode) {
    fprintf(stderr, "Usage: %s <test_prog.bin> [-c]\n", argv[0]);
    return -1;
  }

  // Open the test program file
  FILE* f = fopen(argv[1], "r");

  // Get the size of the test program
  fseek(f, 0L, SEEK_END);
  const size_t file_size = ftell(f);
  rewind(f);

  // Read the contents of the test program
  uint32_t* program = (uint32_t*)malloc(file_size);
  if (fread(program, file_size, 1, f) != 1) {
    fclose(f);
    fprintf(stderr, "Error reading file\n");
    return -1;
  }

  // Close the file
  fclose(f);

  // Create our core object, reset it, and copy in the program
  Core core(compliance_mode && false);
  mem_init(program, file_size);
  core.reset();
  printf("\n");

  // run until we enter an infinite loop
  uint32_t prev_pc = UINT32_MAX;
  bool same_pc_flag = false;
  int watchdog_instr_left = 1000;
  bool triggered_ext_int = compliance_mode;
  while (true) {
    if (watchdog_instr_left-- <= 0) {
      fprintf(stderr, "Watchdog!!!\n");
      exit(1);
    }
    same_pc_flag = core->pc_pc == prev_pc;
    prev_pc = core->pc_pc;
    core.step();
    if (core->pc_pc == prev_pc && same_pc_flag) {
      if (triggered_ext_int) {
        break;
      }
      triggered_ext_int = true;
      same_pc_flag = false;
      printf(">>>>> Setting external interrupt high\n\n");
      core.set_ext_int(true);
      core.step();
      core.set_ext_int(false);
      printf(">>>>> Setting external interrupt low\n\n");
    }
  }

  printf("pc = 0x%x\n", core->pc_pc);

  // Dump the non-zero registers and memory
  printf("Registers:\n");
  for (uint8_t i = 1; i < 16; i++) {
    uint32_t value = core->rf_module->registers[i-1];
    if (value) {
      printf("  %s (x%d) = 0x%x\n", get_register_name(i), i, value);
    }
  }
  mem_dump_ram();

  if (compliance_mode) {
    // check the result
    const uint32_t result = core->rf_module->registers[2];
    if (result != 1) {
      fprintf(stderr, "Test failed with result %u\n", result);
      exit(1);
    }
    // Open the reference output file
    FILE* ref_file = fopen(argv[2], "r");
    unsigned int value;
    uint32_t mem_addr = core->rf_module->registers[1];
    while (fscanf(ref_file, "%x\n", &value) != EOF) {
      const uint32_t mem_value = mem_read(mem_addr);
      if (mem_value != value) {
        fclose(ref_file);
        fprintf(stderr, "Expected 0x%x, got 0x%x at addr 0x%x\n", value, mem_value, mem_addr);
        exit(1);
      }
      mem_addr += sizeof(uint32_t);
    }
    fclose(ref_file);
  }

  return 0;
}
