`ifndef __TYPES_SV__
`define __TYPES_SV__

typedef enum bit [2:0] {
    ALU_OUTPUT_SELECT_ADD_SUB = 3'b000,
    ALU_OUTPUT_SELECT_SLL = 3'b001,
    ALU_OUTPUT_SELECT_SLT_BRANCH = 3'b010,
    ALU_OUTPUT_SELECT_SLTU = 3'b011,
    ALU_OUTPUT_SELECT_XOR = 3'b100,
    ALU_OUTPUT_SELECT_SRL_SRA = 3'b101,
    ALU_OUTPUT_SELECT_OR = 3'b110,
    ALU_OUTPUT_SELECT_AND = 3'b111
} alu_output_select_t;

typedef enum bit [1:0] {
    SHIFTER_OP_SLL = 2'b00,
    SHIFTER_OP_SRL = 2'b01,
    SHIFTER_OP_SRA = 2'b11
} shifter_op_t;

typedef enum bit [2:0] {
    CSR_MAPPED_ADDR_MSTATUS = 3'b000,
    CSR_MAPPED_ADDR_MIE = 3'b011,
    CSR_MAPPED_ADDR_MEPC = 3'b101,
    CSR_MAPPED_ADDR_MCAUSE = 3'b110,
    CSR_MAPPED_ADDR_MIP = 3'b111
} csr_mapped_addr_t;

`define CSR_EXCEPTION_INT 3'b011

typedef enum bit [2:0] {
    CSR_OP_EXCEPTION = 3'b000,
    CSR_OP_MRET = 3'b001,
    CSR_OP_NO_OP = 3'b011,
    CSR_OP_CSRRW = 3'b101,
    CSR_OP_CSRRS = 3'b110,
    CSR_OP_CSRRC = 3'b111
} csr_op_t;

typedef enum bit [1:0] {
    MEM_OP_SIZE_BYTE = 2'b00,
    MEM_OP_SIZE_HALF_WORD = 2'b01,
    MEM_OP_SIZE_WORD = 2'b10
} mem_op_size_t;

typedef enum bit [2:0] {
    FAULT_NUM_INSTR_ADDR_MISALIGNED = 3'b000,
    FAULT_NUM_INSTR_ACCESS_FAULT = 3'b001,
    FAULT_NUM_ILLEGAL_INSTR = 3'b010,
    FAULT_NUM_LOAD_ADDR_MISALIGNED = 3'b100,
    FAULT_NUM_LOAD_ACCESS_FAULT = 3'b101,
    FAULT_NUM_STORE_ADDR_MISALIGNED = 3'b110,
    FAULT_NUM_STORE_ACCESS_FAULT = 3'b111
} fault_num_t;

typedef struct packed {
    bit [6:0] funct7;
    bit [4:0] rs2;
    bit [4:0] rs1;
    bit [2:0] funct3;
    bit [4:0] rd;
    bit [6:0] opcode;
} instruction_t;

typedef struct packed {
    bit alu_in_a; // (0=rs1, 1=pc)
    bit alu_in_b; // (0=rs2, 1=imm)
    bit [1:0] csr_in; // (00=rs1, 01=imm, 10=next_pc)
    bit [1:0] wb_rf_in; // (00=alu_res, 01=imm, 10=mem_data, 11=next_pc)
    bit [1:0] wb_pc; // (00=next_pc, 01=I[31:1], 10=pc+offset, 11=Y?I+offset:next_pc)
} decode_mux_position_t;

typedef struct packed {
    bit enable;
    bit [3:0] addr_a;
    bit [3:0] addr_b;
} decode_rd_rf_microcode_t;

typedef struct packed {
    bit enable;
    bit [3:0] addr;
} decode_wb_rf_microcode_t;

typedef struct packed {
    bit is_beq_bne; // Is BEQ / BNE {Outputs 010 / 011}
    bit is_bge_bgeu_bne; // Is BGE / BGEU / BNE instead of BLT / BLTU / BEQ {Outputs 010 / 011}
    bit is_sltu_bltu_bgeu; // Is SLTU / BLTU / BGEU instead of SLT / BLT / BGE {outputs 010 / 011}
    bit is_add_n; // Is ADD (active low) {outputs 000 / 010 / 011}
    shifter_op_t shifter_op; // Shifter operation
    alu_output_select_t output_select; // Output select
} alu_microcode_t;

typedef struct packed {
    bit enable;
    alu_microcode_t alu;
} decode_ex_alu_microcode_t;

typedef struct packed {
    bit is_write; // Is write
    bit is_unsigned; // Unsigned read
    mem_op_size_t op_size; // Operation size
} mem_microcode_t;

typedef struct packed {
    bit enable;
    mem_microcode_t mem;
} decode_ma_mem_microcode_t;

typedef struct packed {
    bit enable;
    bit [3:0] addr_exception; // Mapped register address / exception value
    csr_op_t op; // Operation
} decode_ex_csr_microcode_t;

`endif
