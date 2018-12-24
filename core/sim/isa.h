#pragma once

#define _BIT(VAL, POS) (((VAL) >> POS) & 0x1)
#define _BITS(VAL, UPPER, LOWER) (((VAL) >> LOWER) & ((1 << ((UPPER) - (LOWER) + 1)) - 1))
#define _R_TYPE(FUNCT7, RS2, RS1, FUNCT3, RD, OPCODE) \
	((FUNCT7 << 25) | (RS2 << 20) | (RS1 << 15) | (FUNCT3 << 12) | (RD << 7) | (OPCODE))
#define _I_TYPE(IMM, RS1, FUNCT3, RD, OPCODE) \
	((IMM << 20) | (RS1 << 15) | (FUNCT3 << 12) | (RD << 7) | (OPCODE))
#define _S_TYPE(IMM, RS2, RS1, FUNCT3, OPCODE) \
	(((IMM >> 5) << 25) | (RS2 << 20) | (RS1 << 15) | (FUNCT3 << 12) | ((IMM & 0x1f) << 7) | (OPCODE))
#define _B_TYPE(IMM, RS2, RS1, FUNCT3, OPCODE) \
	((_BIT(IMM, 12) << 31) | (_BITS(IMM, 10, 5) << 25) | (RS2 << 20) | (RS1 << 15) | (FUNCT3 << 12) | (_BITS(IMM, 4, 0) << 7) | (OPCODE))
#define _J_TYPE(IMM, RD, OPCODE) \
	((_BIT(IMM, 20) << 31) | (_BITS(IMM, 10, 1) << 21) | (_BIT(IMM, 11) << 20) | (_BITS(IMM, 19, 12) << 12) | (RD << 7) | (OPCODE))

#define JAL(RD, OFFSET) _J_TYPE(UINT32_C(OFFSET), RD, 0x6f)
#define BEQ(RS1, RS2, OFFSET) _B_TYPE(UINT32_C(OFFSET), RS2, RS1, 0x0, 0x63)
#define SW(RS2, RS1, OFFSET) _S_TYPE(UINT32_C(OFFSET), RS2, RS1, 0x2, 0x23)
#define LW(RD, RS1, OFFSET) _I_TYPE(UINT32_C(OFFSET), RS1, 0x2, RD, 0x03)
#define ADDI(RD, RS1, IMM) _I_TYPE(UINT32_C(IMM), RS1, 0x0, RD, 0x13)
#define ADD(RD, RS1, RS2) _R_TYPE(0x00, RS2, RS1, 0x0, RD, 0x33)
