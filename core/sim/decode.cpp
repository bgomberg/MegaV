#include "decode.h"

#include <stdio.h>


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

void print_decoded_instr(uint32_t instr) {
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

const char* get_register_name(uint8_t num) {
  return REGISTER_NAMES[num];
}
