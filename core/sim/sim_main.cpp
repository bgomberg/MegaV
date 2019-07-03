#include "mem.h"

#include <Vmodule.h>
#include <Vmodule_core.h>
#include <Vmodule_memory.h>
#include <Vmodule_register_file.h>

#include <verilated.h>

#include <stdio.h>


#define STAGE_CONTROL 0
#define STAGE_FETCH 1
#define STAGE_DECODE 2
#define STAGE_READ 3
#define STAGE_EXECUTE 4
#define STAGE_MEMORY 5
#define STAGE_WRITE_BACK 6
#define NUM_STAGES 7

#define BIT(VAL, POS) \
	(((VAL) >> (POS)) & 1)
#define BITS(VAL, TOP, BOTTOM) \
	(((VAL) >> (BOTTOM)) & ((1 << ((TOP) - (BOTTOM) + 1)) - 1))


static const char * const REGISTER_NAMES[] = {
	"zero",
	"ra",
	"sp",
	"gp",
	"tp",
	"t0",
	"t1",
	"t2",
	"s0/fp",
	"s1",
	"a0",
	"a1",
	"a2",
	"a3",
	"a4",
	"a5",
};


static int32_t sign_extend(uint32_t value, uint8_t value_num_bits) {
	uint32_t sign = BIT(value, value_num_bits - 1);
	for (; value_num_bits < 32; value_num_bits++) {
		value = (value & ~(1 << value_num_bits)) | (sign << value_num_bits);
	}
	return value;
}

static void print_decoded_instr(uint32_t instr) {
	const uint8_t opcode = BITS(instr, 6, 0);
	const uint8_t funct3 = BITS(instr, 14, 12);
	const uint8_t funct7 = BITS(instr, 31, 25);
	const uint8_t rd = BITS(instr, 11, 7) & 0xf;
	const uint8_t rs1 = BITS(instr, 19, 15) & 0xf;
	const uint8_t rs2 = BITS(instr, 24, 20) & 0xf;
	const uint8_t csr = BITS(instr, 31, 20);
	const int32_t i_type_imm = sign_extend(BITS(instr, 31, 20), 12);
	const int32_t s_type_imm = sign_extend((BITS(instr, 31, 25) << 5) | BITS(instr, 11, 7), 12);
	const int32_t b_type_imm = sign_extend((BIT(instr, 31) << 12) | (BIT(instr, 7) << 11) | (BITS(instr, 30, 25) << 5) | (BITS(instr, 11, 8) << 1), 13);
	const int32_t u_type_imm = sign_extend(BITS(instr, 31, 12), 20);
	const int32_t j_type_imm = sign_extend((BIT(instr, 31) << 20) | (BITS(instr, 19, 12) << 12) | BIT(instr, 20) << 11 | BITS(instr, 30, 21) << 1, 21);
	switch (opcode) {
	case 0x37:
		printf("lui %s, %d", REGISTER_NAMES[rd], u_type_imm);
		break;
	case 0x17:
		printf("auipc %s, %d", REGISTER_NAMES[rd], u_type_imm);
		break;
	case 0x6f:
		printf("jal %d", j_type_imm);
		break;
	case 0x67:
		printf("jalr %s, %s, %d", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], i_type_imm);
		break;
	case 0x63:
		switch (funct3) {
		case 0x0:
			printf("beq %s, %s, %d", REGISTER_NAMES[rs1], REGISTER_NAMES[rs2], b_type_imm);
			break;
		case 0x1:
			printf("bne %s, %s, %d", REGISTER_NAMES[rs1], REGISTER_NAMES[rs2], b_type_imm);
			break;
		case 0x4:
			printf("blt %s, %s, %d", REGISTER_NAMES[rs1], REGISTER_NAMES[rs2], b_type_imm);
			break;
		case 0x5:
			printf("bge %s, %s, %d", REGISTER_NAMES[rs1], REGISTER_NAMES[rs2], b_type_imm);
			break;
		case 0x6:
			printf("bltu %s, %s, %d", REGISTER_NAMES[rs1], REGISTER_NAMES[rs2], b_type_imm);
			break;
		case 0x7:
			printf("bgeu %s, %s, %d", REGISTER_NAMES[rs1], REGISTER_NAMES[rs2], b_type_imm);
			break;
		default:
			printf("UNKNOWN_BRANCH_FUNCT3");
			break;
		}
		break;
	case 0x03:
		switch (funct3) {
		case 0x0:
			printf("lb %s, %d(%s)", REGISTER_NAMES[rs1], i_type_imm, REGISTER_NAMES[rd]);
			break;
		case 0x1:
			printf("lh %s, %d(%s)", REGISTER_NAMES[rs1], i_type_imm, REGISTER_NAMES[rd]);
			break;
		case 0x2:
			printf("lw %s, %d(%s)", REGISTER_NAMES[rs1], i_type_imm, REGISTER_NAMES[rd]);
			break;
		case 0x4:
			printf("lbu %s, %d(%s)", REGISTER_NAMES[rs1], i_type_imm, REGISTER_NAMES[rd]);
			break;
		case 0x5:
			printf("lhu %s, %d(%s)", REGISTER_NAMES[rs1], i_type_imm, REGISTER_NAMES[rd]);
			break;
		default:
			printf("UNKNOWN_LOAD_FUNCT3");
			break;
		}
		break;
	case 0x23:
		switch (funct3) {
		case 0x0:
			printf("sb %s, %d(%s)", REGISTER_NAMES[rs2], s_type_imm, REGISTER_NAMES[rs1]);
			break;
		case 0x1:
			printf("sh %s, %d(%s)", REGISTER_NAMES[rs2], s_type_imm, REGISTER_NAMES[rs1]);
			break;
		case 0x2:
			printf("sw %s, %d(%s)", REGISTER_NAMES[rs2], s_type_imm, REGISTER_NAMES[rs1]);
			break;
		default:
			printf("UNKNOWN_STORE_FUNCT3");
			break;
		}
		break;
	case 0x13:
		switch (funct3) {
		case 0x0:
			printf("addi %s, %s, %d", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], i_type_imm);
			break;
		case 0x1:
			printf("slli %s, %s, %d", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], i_type_imm);
			break;
		case 0x2:
			printf("slti %s, %s, %d", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], i_type_imm);
			break;
		case 0x3:
			printf("sltiu %s, %s, %d", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], i_type_imm);
			break;
		case 0x4:
			printf("xori %s, %s, %d", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], i_type_imm);
			break;
		case 0x5:
			if (funct7 == 0x20) {
				printf("srai %s, %s, %d", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], i_type_imm);
			} else {
				printf("srli %s, %s, %d", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], i_type_imm);
			}
			break;
		case 0x6:
			printf("ori %s, %s, %d", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], i_type_imm);
			break;
		case 0x7:
			printf("andi %s, %s, %d", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], i_type_imm);
			break;
		default:
			printf("UNKNOWN_OPIMM_FUNCT3");
			break;
		}
		break;
	case 0x33:
		switch (funct3) {
		case 0x0:
			if (funct7 == 0x20) {
				printf("sub %s, %s, %s", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], REGISTER_NAMES[rs2]);
			} else {
				printf("add %s, %s, %s", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], REGISTER_NAMES[rs2]);
			}
			break;
		case 0x1:
			printf("sll %s, %s, %s", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], REGISTER_NAMES[rs2]);
			break;
		case 0x2:
			printf("slt %s, %s, %s", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], REGISTER_NAMES[rs2]);
			break;
		case 0x3:
			printf("sltu %s, %s, %s", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], REGISTER_NAMES[rs2]);
			break;
		case 0x4:
			printf("xor %s, %s, %s", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], REGISTER_NAMES[rs2]);
			break;
		case 0x5:
			if (funct7 == 0x20) {
				printf("sra %s, %s, %s", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], REGISTER_NAMES[rs2]);
			} else {
				printf("srl %s, %s, %s", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], REGISTER_NAMES[rs2]);
			}
			break;
		case 0x6:
			printf("or %s, %s, %s", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], REGISTER_NAMES[rs2]);
			break;
		case 0x7:
			printf("and %s, %s, %s", REGISTER_NAMES[rd], REGISTER_NAMES[rs1], REGISTER_NAMES[rs2]);
			break;
		default:
			printf("UNKNOWN_OP_FUNCT3");
			break;
		}
		break;
	case 0x0f:
		switch (funct3) {
		case 0x0:
			printf("fence");
			break;
		case 0x1:
			printf("fence.i");
			break;
		default:
			printf("UNKNOWN_FENCE_FUNCT3");
			break;
		}
		break;
	case 0x73:
		switch (funct3) {
		case 0x0:
			switch (i_type_imm) {
			case 0x000:
				printf("ecall");
				break;
			case 0x001:
				printf("ebreak");
				break;
			case 0x302:
				printf("mret");
				break;
			default:
				printf("UNKNOWN_SYSTEM_IMM(0x%x)", i_type_imm);
				break;
			}
			break;
		case 0x1:
			printf("csrrw %s, 0x%x, %s", REGISTER_NAMES[rd], csr, REGISTER_NAMES[rs1]);
			break;
		case 0x2:
			printf("csrrs %s, 0x%x, %s", REGISTER_NAMES[rd], csr, REGISTER_NAMES[rs1]);
			break;
		case 0x3:
			printf("csrrc %s, 0x%x, %s", REGISTER_NAMES[rd], csr, REGISTER_NAMES[rs1]);
			break;
		case 0x5:
			printf("csrrwi %s, 0x%x, 0x%x", REGISTER_NAMES[rd], csr, rs1);
			break;
		case 0x6:
			printf("csrrsi %s, 0x%x, 0x%x", REGISTER_NAMES[rd], csr, rs1);
			break;
		case 0x7:
			printf("csrrci %s, 0x%x, 0x%x", REGISTER_NAMES[rd], csr, rs1);
			break;
		default:
			printf("UNKNOWN_OP_FUNCT3");
			break;
		}
		break;
	default:
		printf("UNKNOWN_OPCODE(0x%x)", opcode);
		break;
	}
}

class Core {
public:
	Core() : module_(new Vmodule()) {
		module_->clk = 0;
		module_->reset = 0;
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
		module_->reset = 1;
		tick();
		module_->reset = 0;
	}

	void step() {
		// Print info no the stage we're about to execute
		const int prev_stage_num = get_stage_num();
		switch (prev_stage_num) {
		case STAGE_CONTROL:
			printf("Executing STAGE_CONTROL\n");
			printf("  Inputs:\n");
			printf("    fault_num: %d\n", (*this)->fault_num);
			break;
		case STAGE_FETCH:
			printf("Executing STAGE_FETCH\n");
			printf("  Inputs:\n");
			printf("    control_op: 0x%x\n", (*this)->control_op);
			printf("    pc: 0x%x\n", (*this)->pc_pc);
			break;
		case STAGE_DECODE:
			printf("Executing STAGE_DECODE\n");
			printf("  Inputs:\n");
			printf("    instr: 0x%x [", (*this)->instr);
			print_decoded_instr((*this)->instr);
			printf("]\n");
			break;
		case STAGE_READ:
			printf("Executing STAGE_READ\n");
			printf("  Inputs:\n");
			break;
		case STAGE_EXECUTE:
			printf("Executing STAGE_EXECUTE\n");
			printf("  Inputs:\n");
			printf("    csr_addr_exception: 0x%x\n", (*this)->csr_addr_exception);
			printf("    csr_in: 0x%x\n", (*this)->csr_in);
			printf("    csr_read_value: 0x%x\n", (*this)->csr_read_value);
			printf("    next_pc: 0x%x\n", (*this)->pc_next_pc);
			break;
		case STAGE_MEMORY:
			printf("Executing STAGE_MEMORY\n");
			printf("  Inputs:\n");
			break;
		case STAGE_WRITE_BACK:
			printf("Executing STAGE_WRITE_BACK\n");
			printf("  Inputs:\n");
			printf("    write_data: 0x%x\n", (*this)->rf_write_data);
			printf("    next_pc: 0x%x\n", (*this)->pc_next_pc);
			break;
		default:
			fprintf(stderr, "Unknown stage!!!\n");
			exit(1);
			break;
		}

		// Move to the next stage
		while (get_stage_num() == prev_stage_num) {
			tick();
		}

		switch (prev_stage_num) {
		case STAGE_CONTROL:
			printf("  Outputs:\n");
			break;
		case STAGE_FETCH:
			printf("  Outputs:\n");
			break;
		case STAGE_DECODE:
			printf("  Outputs:\n");
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
			printf("  Outputs:\n");
			printf("    read_data_a: 0x%x\n", (*this)->rf_read_data_a);
			printf("    read_data_b: 0x%x\n", (*this)->rf_read_data_b);
			break;
		case STAGE_EXECUTE:
			printf("  Outputs:\n");
			printf("    alu_out: 0x%x\n", (*this)->alu_out);
			printf("    csr_read_value: 0x%x\n", (*this)->csr_read_value);
			break;
		case STAGE_MEMORY:
			printf("  Outputs:\n");
			printf("    out: 0x%x\n", (*this)->mem_out);
			break;
		case STAGE_WRITE_BACK:
			printf("  Outputs:\n");
			break;
		default:
			fprintf(stderr, "Unknown stage!!!\n");
			exit(1);
			break;
		}
	}

private:
	Vmodule* module_;

	void tick() {
		module_->clk = 1;
		eval();
		module_->clk = 0;
		eval();
		// printf(".\n");
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
	if (argc != 2) {
		fprintf(stderr, "Usage: %s <test_prog.bin>\n", argv[0]);
		return -1;
	}

	// Open the test program file
	FILE *f = fopen(argv[1], "r");

	// Get the size of the test program
	fseek(f, 0L, SEEK_END);
	const size_t file_size = ftell(f);
	rewind(f);

	// Read the contents of the test program
	uint32_t *program = (uint32_t *)malloc(file_size);
	if (fread(program, file_size, 1, f) != 1) {
		fclose(f);
		fprintf(stderr, "Error reading file\n");
		return -1;
	}

	// Close the file
	fclose(f);

	// Create our core object, reset it, and copy in the program
	Core core;
	mem_init(core.operator->(), program, file_size);
	core.reset();
	printf("\n");

	// run until we enter an infinite loop
	uint32_t prev_pc = UINT32_MAX;
	bool same_pc_flag = false;
	int watchdog_instr_left = 100;
	while (core->pc_pc != prev_pc || !same_pc_flag) {
		if (watchdog_instr_left-- <= 0) {
			fprintf(stderr, "Watchdog!!!\n");
			exit(1);
		}
		same_pc_flag = core->pc_pc == prev_pc;
		prev_pc = core->pc_pc;
		core.step();
		while (core->stage_active != (1 << 0)) {
			core.step();
		}
		printf("\n");
	}

	// Dump the non-zero registers and memory
	printf("Registers:\n");
	for (int i = 1; i < 16; i++) {
		uint32_t value = core->rf_module->registers[i-1];
		if (value) {
			printf("  %s (x%d) = 0x%x\n", REGISTER_NAMES[i], i, value);
		}
	}
	mem_dump_ram();

	return 0;
}
