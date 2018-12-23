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
		module_->eval();
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
		module_->eval();
		tick();
		module_->reset = 0;
		tick();
	}

	void step() {
		// Move forward one clock cycle
		if ((*this)->stage == (1 << 0)) {
			printf("Executing STAGE_FETCH\n");
			tick();
			printf("  pc: 0x%x\n", (*this)->pc_pc);
			printf("  instr: 0x%x\n", (*this)->mem_out);
		} else if ((*this)->stage == (1 << 1)) {
			printf("Executing STAGE_DECODE\n");
			tick();
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
			tick();
			printf("  read_data_a: 0x%x\n", (*this)->rf_read_data_a);
			printf("  read_data_b: 0x%x\n", (*this)->rf_read_data_b);
		} else if ((*this)->stage == (1 << 3)) {
			printf("Executing STAGE_EXECUTE\n");
			printf("  out: 0x%x\n", (*this)->alu_out);
			tick();
		} else if ((*this)->stage == (1 << 4)) {
			printf("Executing STAGE_MEMORY\n");
			tick();
		} else if ((*this)->stage == (1 << 5)) {
			printf("Executing STAGE_WRITE_BACK\n");
			tick();
			printf("  write_data: 0x%x\n", (*this)->rf_write_data);
		} else {
			printf("!!! UNKNOWN STAGE\n");
		}

		// Check for faults
		if (module_->fault) {
			bool found = false;
			if ((*this)->pc_fault) {
				printf("!!! PC FAULT\n");
				found = true;
			}
			if ((*this)->mem_fault) {
				printf("!!! MEM FAULT\n");
				found = true;
			}
			if ((*this)->decode_fault) {
				printf("!!! DECODE FAULT\n");
				found = true;
			}
			if ((*this)->alu_fault) {
				printf("!!! ALU FAULT\n");
				found = true;
			}
			if (!found) {
				printf("!!! UNKNOWN FAULT\n");
			}
			exit(1);
		}
	}

private:
	Vmodule* module_;

	void tick() {
		module_->clk = 1;
		module_->eval();
		module_->clk = 0;
		module_->eval();
	}
};


#define _R_TYPE(FUNCT7, RS2, RS1, FUNCT3, RD, OPCODE) \
	((FUNCT7 << 25) | (RS2 << 20) | (RS1 << 15) | (FUNCT3 << 12) | (RD << 7) | (OPCODE))
#define _I_TYPE(IMM, RS1, FUNCT3, RD, OPCODE) \
	((IMM << 20) | (RS1 << 15) | (FUNCT3 << 12) | (RD << 7) | (OPCODE))
#define _S_TYPE(IMM, RS2, RS1, FUNCT3, OPCODE) \
	(((IMM >> 5) << 25) | (RS2 << 20) | (RS1 << 15) | (FUNCT3 << 12) | ((IMM & 0x1f) << 7) | (OPCODE))

#define ADD(RD, RS1, RS2) _R_TYPE(0x00, RS2, RS1, 0x0, RD, 0x33)
#define ADDI(RD, RS1, IMM) _I_TYPE(IMM, RS1, 0x0, RD, 0x13)
#define SW(RS2, RS1, OFFSET) _S_TYPE(OFFSET, RS2, RS1, 0x2, 0x23)
#define LW(RD, RS1, OFFSET) _I_TYPE(OFFSET, RS1, 0x2, RD, 0x03)

#define ARRAY_LENGTH(ARR) ((sizeof(ARR)) / (sizeof(*ARR)))

static const uint32_t TEST_PROGRAM[] = {
	ADDI(1, 0, 0x11), // ADDI r1,r0,0x11
	ADDI(2, 0, 0x31), // ADDI r2,r0,0x31
	ADD(2, 1, 2), // ADD r2,r1,r2
	ADD(3, 1, 2), // ADD r3,r1,r2
	SW(3, 0, 0x80), // SW r3,r0,0x80
	LW(4, 0, 0x80), // LW r4,r0,0x80
};


int main(int argc, char** argv) {
	Verilated::commandArgs(argc, argv);
	printf("\n");

	// Create our core object, reset it, and copy in the program
	SimulatedCore core;
	core.reset();
	memcpy(core->mem_module->fake_memory, TEST_PROGRAM, sizeof(TEST_PROGRAM));
	printf("\n");

	for (int i = 0; i < ARRAY_LENGTH(TEST_PROGRAM); i++) {
		core.step();
		core.step();
		core.step();
		core.step();
		core.step();
		core.step();
		// Step the CPU past the fetch stage and then until we reach the fetch stage again
		// core.step();
		// while (core->stage != (1 << 0)) core.step();
		printf("\n");
	}

	printf("Registers:\n");
	for (int i = 1; i < 16; i++) {
		uint32_t value = core->rf_module->registers[i-1];
		if (value) {
			printf("  R%d = 0x%x\n", i, value);
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
