#include <Vmodule.h>
#include <Vmodule_core.h>
#include <Vmodule_memory.h>
#include <Vmodule_register_file.h>

#include <verilated.h>

#include <stdio.h>


class SimulatedCore {
public:
	SimulatedCore() : module_(new Vmodule()) {
		module_->clk = 0;
		module_->reset = 0;
		eval();
	}

	~SimulatedCore() {
		delete module_;
		module_ = NULL;
	}

	Vmodule_core* operator->() {
		return module_->core;
	}

	void reset() {
		printf("Resetting core\n");
		module_->reset = 1;
		eval();
		tick();
		module_->reset = 0;
		eval();
	}

	void step() {
		// Move forward one clock cycle
		tick();
		if ((*this)->stage == (1 << 0)) {
			printf("Executing STAGE_FETCH\n");
			printf("  pc: 0x%x\n", (*this)->pc_pc);
			printf("  instr: 0x%x\n", (*this)->mem_out);
		} else if ((*this)->stage == (1 << 1)) {
			printf("Executing STAGE_DECODE\n");
			printf("  imm: 0x%x\n", (*this)->decode_imm);
			printf("  alu_a_mux_position: 0x%x\n", (*this)->decode_alu_a_mux_position);
			printf("  alu_b_mux_position: 0x%x\n", (*this)->decode_alu_b_mux_position);
			printf("  wb_mux_position: 0x%x\n", (*this)->decode_wb_mux_position);
			printf("  rd_rf_microcode: 0x%x\n", (*this)->decode_rd_rf_microcode);
			printf("  ex_alu_microcode: 0x%x\n", (*this)->decode_ex_alu_microcode);
			printf("  ma_mem_microcode: 0x%x\n", (*this)->decode_ma_mem_microcode);
			printf("  wb_rf_microcode : 0x%x\n", (*this)->decode_wb_rf_microcode);
			printf("  wb_pc_microcode: 0x%x\n", (*this)->decode_wb_pc_microcode);
		} else if ((*this)->stage == (1 << 2)) {
			printf("Executing STAGE_READ\n");
			printf("  read_data_a: 0x%x\n", (*this)->rf_read_data_a);
			printf("  read_data_b: 0x%x\n", (*this)->rf_read_data_b);
		} else if ((*this)->stage == (1 << 3)) {
			printf("Executing STAGE_EXECUTE\n");
			printf("  out: 0x%x\n", (*this)->alu_out);
			printf("  next_pc: 0x%x\n", (*this)->pc_next_pc);
		} else if ((*this)->stage == (1 << 4)) {
			printf("Executing STAGE_MEMORY\n");
		} else if ((*this)->stage == (1 << 5)) {
			printf("Executing STAGE_WRITE_BACK\n");
			printf("  write_data: 0x%x\n", (*this)->rf_write_data);
			printf("  next_pc: 0x%x\n", (*this)->pc_next_pc);
		} else {
			printf("!!! UNKNOWN STAGE\n");
		}

		// Check for faults
		if (module_->fault) {
			printf("!!! FAULT\n");
			exit(1);
		}
	}

private:
	Vmodule* module_;

	void tick() {
		module_->clk = 1;
		eval();
		module_->clk = 0;
		eval();
	}

	void eval() {
		module_->eval();
		if (module_->fault) {
			printf("!!! FAULT\n");
			exit(1);
		}
	}
};

#define _BIT(VAL, POS) (((VAL) >> POS) & 0x1)
#define _BITS(VAL, UPPER, LOWER) (((VAL) >> LOWER) & ((1 << ((UPPER) - (LOWER) + 1)) - 1))
#define _R_TYPE(FUNCT7, RS2, RS1, FUNCT3, RD, OPCODE) \
	((FUNCT7 << 25) | (RS2 << 20) | (RS1 << 15) | (FUNCT3 << 12) | (RD << 7) | (OPCODE))
#define _I_TYPE(IMM, RS1, FUNCT3, RD, OPCODE) \
	((IMM << 20) | (RS1 << 15) | (FUNCT3 << 12) | (RD << 7) | (OPCODE))
#define _S_TYPE(IMM, RS2, RS1, FUNCT3, OPCODE) \
	(((IMM >> 5) << 25) | (RS2 << 20) | (RS1 << 15) | (FUNCT3 << 12) | ((IMM & 0x1f) << 7) | (OPCODE))
#define _J_TYPE(IMM, RD, OPCODE) \
	((_BIT(IMM, 20) << 31) | (_BITS(IMM, 10, 1) << 21) | (_BIT(IMM, 11) << 20) | (_BITS(IMM, 19, 12) << 12) | (RD << 7) | (OPCODE))

#define JAL(RD, OFFSET) _J_TYPE(UINT32_C(OFFSET), RD, 0x6f)
#define SW(RS2, RS1, OFFSET) _S_TYPE(UINT32_C(OFFSET), RS2, RS1, 0x2, 0x23)
#define LW(RD, RS1, OFFSET) _I_TYPE(UINT32_C(OFFSET), RS1, 0x2, RD, 0x03)
#define ADDI(RD, RS1, IMM) _I_TYPE(UINT32_C(IMM), RS1, 0x0, RD, 0x13)
#define ADD(RD, RS1, RS2) _R_TYPE(0x00, RS2, RS1, 0x0, RD, 0x33)

#define ARRAY_LENGTH(ARR) ((sizeof(ARR)) / (sizeof(*ARR)))

static const uint32_t TEST_PROGRAM[] = {
	ADDI(1, 0, 0x11), // ADDI r1,r0,0x11
	ADDI(2, 0, 0x31), // ADDI r2,r0,0x31
	ADD(2, 1, 2), // ADD r2,r1,r2
	ADD(3, 1, 2), // ADD r3,r1,r2
	SW(3, 0, 0x80), // SW r3,r0,0x80
	LW(4, 0, 0x80), // LW r4,r0,0x80
	ADDI(5, 0, 0x100), // ADDI r5,r0,0x100
	JAL(10, 8), // JAL r10,8
	JAL(0, 0), // JAL r0,0
	ADDI(5, 5, 0x01), // ADDI r5,r5,0x01
	ADDI(5, 5, 0x01), // ADDI r5,r5,0x01
	JAL(11, -12), // JAL r11,-12
};


int main(int argc, char** argv) {
	Verilated::commandArgs(argc, argv);
	printf("\n");

	// Create our core object, reset it, and copy in the program
	SimulatedCore core;
	core.reset();
	memcpy(core->mem_module->fake_memory, TEST_PROGRAM, sizeof(TEST_PROGRAM));
	printf("\n");

	// run until we enter an infinite loop
	uint32_t prev_pc = UINT32_MAX;
	while (core->pc_pc != prev_pc) {
		prev_pc = core->pc_pc;
		core.step();
		while (core->stage != (1 << 5)) {
			core.step();
		}
		printf("\n");
	}

	printf("Registers:\n");
	for (int i = 1; i < 16; i++) {
		uint32_t value = core->rf_module->registers[i-1];
		if (value) {
			printf("  r%d = 0x%x\n", i, value);
		}
	}
	printf("Memory:\n");
	for (int i = 0x80; i < 0x100; i += 4) {
		uint32_t value;
		memcpy(&value, &core->mem_module->fake_memory[i], sizeof(value));
		if (value) {
			printf("  [0x%x] = 0x%x\n", i, value);
		}
	}

	return 0;
}
