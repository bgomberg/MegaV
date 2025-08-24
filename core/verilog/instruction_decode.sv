`include "cells/and2.sv"
`include "cells/and4.sv"
`include "cells/dffe.sv"
`include "cells/mux2.sv"
`include "cells/mux4.sv"
`include "cells/nand3.sv"
`include "cells/nor2.sv"
`include "cells/nor4.sv"
`include "cells/or4.sv"
`include "cells/xor3.sv"
`include "include/alu_microcode.sv"
`include "include/types.sv"

/*
 * A decoder for the RV32I instruction set.
 */
module instruction_decode(
    input logic clk, // Clock signal
    input logic reset_n, // Reset signal (active low)
    input logic enable_n, // Enable (active low)
    input instruction_t instr, // Instruction to decode
    output logic [31:0] imm, // Immediate value
    output decode_mux_position_t mux_position,
    output decode_rd_rf_microcode_t rd_rf_microcode, // Read stage register file microcode
    output decode_ex_alu_microcode_t ex_alu_microcode, // Execute stage ALU microcode
    output decode_ma_mem_microcode_t ma_mem_microcode, // Memory access stage memory microcode
    output decode_ex_csr_microcode_t ex_csr_microcode, // Execute stage CSR microcode
    output decode_wb_rf_microcode_t wb_rf_microcode, // Write back stage register file microcode
    output logic fault // Fault condition (i.e. invalid instruction)
);

    /* Basic Decode */
    wire opcode_is_arith_reg = ~instr.opcode[6] & instr.opcode[5] & instr.opcode[4] & ~instr.opcode[2]; // ADD / SUB / SLL / SLT / SLTU / XOR / SRL / SRA / OR / AND
    wire opcode_is_arith_imm = ~instr.opcode[5] & instr.opcode[4] & ~instr.opcode[2]; // ADDI / SLTI / SLTIU / XORI / ORI / ANDI
    wire opcode_is_branch = instr.opcode[6] & ~instr.opcode[4] & ~instr.opcode[2]; // BEQ / BNE / BLT / BGE / BLTU / BGEU
    wire opcode_is_load = ~instr.opcode[5] & ~instr.opcode[4] & ~instr.opcode[3]; // LB / LH / LW / LBU / LHU
    wire opcode_is_store = ~instr.opcode[6] & instr.opcode[5] & ~instr.opcode[4]; // SB / SH / SW
    wire opcode_is_auipc = ~instr.opcode[5] & ~instr.opcode[3] & instr.opcode[2]; // AUIPC
    wire opcode_is_jal = instr.opcode[5] & instr.opcode[3]; // JAL
    wire opcode_is_jalr = ~instr.opcode[4] & ~instr.opcode[3] & instr.opcode[2]; // JALR
    wire opcode_is_lui = ~instr.opcode[6] & instr.opcode[5] & instr.opcode[2]; // LUI
    wire opcode_is_fence = ~instr.opcode[6] & instr.opcode[3]; // FENCE / FENCE.I
    wire opcode_is_system = instr.opcode[6] & instr.opcode[4]; // ECALL / EBREAK / CSRRW / CSRRS / CSRRC / CSRRWI / CSRRSI / CSRRCI
    wire invalid_opcode = ~instr.opcode[0] | ~instr.opcode[1] |
        (instr.opcode[3] & ~instr.opcode[2]) |
        (instr.opcode[4] & instr.opcode[3]) |
        (instr.opcode[6] & ~instr.opcode[5]) |
        (~instr.opcode[6] & instr.opcode[5] & instr.opcode[3]) |
        (instr.opcode[6] & instr.opcode[4] & instr.opcode[2]) |
        (~instr.opcode[6] & ~instr.opcode[4] & ~instr.opcode[3] & instr.opcode[2]);
    wire instr_is_system_e_mret = opcode_is_system & ~instr.funct3[2] & ~instr.funct3[1] & ~instr.funct3[0];
    wire instr_is_system_mret = instr_is_system_e_mret & instr.funct7[4];
    wire instr_is_system_e = instr_is_system_e_mret & ~instr_is_system_mret;
    wire instr_is_system_csr = opcode_is_system & ~instr_is_system_e_mret;
    wire invalid_funct3 =
        (opcode_is_jalr & (instr.funct3[0] | instr.funct3[1] | instr.funct3[2])) |
        (opcode_is_store & (instr.funct3[2] | (instr.funct3[1] & instr.funct3[0]))) |
        (opcode_is_load & ((instr.funct3[1] & instr.funct3[0]) | (instr.funct3[2] & instr.funct3[1]))) |
        (opcode_is_branch & ~instr.funct3[2] & instr.funct3[1]) |
        (opcode_is_fence & (instr.funct3[2] | instr.funct3[1])) |
        (opcode_is_system & instr.funct3[2] & ~instr.funct3[1] & ~instr.funct3[0]);
    wire funct7_invalid_arith_bit_set = instr.funct7[6] | instr.funct7[4] | instr.funct7[3] | instr.funct7[2] | instr.funct7[1] | instr.funct7[0];
    wire invalid_funct7 =
        (opcode_is_arith_reg & (funct7_invalid_arith_bit_set | (instr.funct3[1] & instr.funct7[5]) | (~instr.funct3[2] & instr.funct3[0] & instr.funct7[5]) | (instr.funct3[2] & ~instr.funct3[0] & instr.funct7[5]))) |
        (opcode_is_arith_imm & ((~instr.funct3[1] & instr.funct3[0] & funct7_invalid_arith_bit_set) | (~instr.funct3[2] & ~instr.funct3[1] & instr.funct3[0] & instr.funct7[5]))) |
        (opcode_is_fence & (instr.funct7[6] | instr.funct7[5] | instr.funct7[4] | instr.funct7[3] | (instr.funct3[0] & (instr.funct7[2] | instr.funct7[1] | instr.funct7[0])))) |
        (instr_is_system_e & (instr.funct7[6] | instr.funct7[5] | instr.funct7[4] | instr.funct7[3] | instr.funct7[2] | instr.funct7[1] | instr.funct7[0])) |
        (instr_is_system_mret & (instr.funct7[6] | instr.funct7[5] | ~instr.funct7[4] | ~instr.funct7[3] | instr.funct7[2] | instr.funct7[1] | instr.funct7[0]));

    /* Read Stage */
    wire rd_rf_enable = ~(opcode_is_lui | opcode_is_auipc | opcode_is_jal | opcode_is_fence | opcode_is_system) |
        (instr_is_system_csr & ~instr.funct3[2]);
    wire rs1_is_zero = ~instr.rs1[4] & ~instr.rs1[3] & ~instr.rs1[2] & ~instr.rs1[1] & ~instr.rs1[0];
    wire invalid_rs = (rd_rf_enable & (instr.rs1[4] | ((opcode_is_arith_reg | opcode_is_store | opcode_is_branch) & instr.rs2[4]))) |
        (opcode_is_fence & (~rs1_is_zero | (instr.funct3[0] & (instr.rs2 != 5'b0)))) |
        (instr_is_system_e & (~rs1_is_zero | instr.rs2[4] | instr.rs2[3] | instr.rs2[2] | instr.rs2[1])) |
        (instr_is_system_mret & (~rs1_is_zero | instr.rs2[4] | instr.rs2[3] | instr.rs2[2] | ~instr.rs2[1] | instr.rs2[0]));
    decode_rd_rf_microcode_t next_rd_rf_microcode;
    assign next_rd_rf_microcode.enable = ~invalid_instr & rd_rf_enable;
    assign next_rd_rf_microcode.addr_a = instr.rs1[3:0];
    assign next_rd_rf_microcode.addr_b = instr.rs2[3:0];
    dffe #(.BITS($bits(rd_rf_microcode))) rd_rf_microcode_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_rd_rf_microcode),
        .out(rd_rf_microcode)
    );

    /* Execute Stage  - ALU */
    decode_ex_alu_microcode_t next_ex_alu_microcode;
    wire ex_alu_microcode_is_arith = opcode_is_arith_imm | opcode_is_arith_reg;
    wire ex_alu_microcode_is_arith_branch = ex_alu_microcode_is_arith | opcode_is_branch;
    assign next_ex_alu_microcode.enable = ~invalid_instr &
        (ex_alu_microcode_is_arith_branch | opcode_is_load | opcode_is_store | opcode_is_auipc | opcode_is_jalr);
    wire ex_alu_temp = instr.funct7[5] & ~instr.funct3[1] & ex_alu_microcode_is_arith & (opcode_is_arith_reg | instr.funct3[2]) &
        (instr.funct3[2] | ~instr.funct3[0]) & (~instr.funct3[2] | instr.funct3[0]);
    assign next_ex_alu_microcode.alu.is_beq_bne = opcode_is_branch & ~instr.funct3[2];
    assign next_ex_alu_microcode.alu.is_bge_bgeu_bne = opcode_is_branch & instr.funct3[0];
    assign next_ex_alu_microcode.alu.is_sltu_bltu_bgeu = (opcode_is_branch & instr.funct3[1]) | (~opcode_is_branch & instr.funct3[1] & instr.funct3[0]);
    assign next_ex_alu_microcode.alu.shifter_op = {instr.funct3[2] & ex_alu_temp, instr.funct3[2]};
    wire [2:0] ex_alu_microcode_output;
    assign next_ex_alu_microcode.alu.output_select = ex_alu_microcode_output;
    mux4 #(.BITS($bits(ex_alu_microcode_output))) ex_alu_microcode_output_mux(
        .d1(3'b000),
        .d2(3'b010),
        .d3(instr.funct3),
        .d4(3'b010),
        .select({ex_alu_microcode_is_arith, opcode_is_branch}),
        .out(ex_alu_microcode_output)
    );
    or4 ex_alu_microcode_is_add_n_or(
        .a(opcode_is_branch),
        .b(ex_alu_temp),
        .c(ex_alu_microcode_output[1]),
        .d(ex_alu_microcode_output[0]),
        .out(next_ex_alu_microcode.alu.is_add_n)
    );
    dffe #(.BITS($bits(ex_alu_microcode))) ex_alu_microcode_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_ex_alu_microcode),
        .out(ex_alu_microcode)
    );

    /* Execute Stage  - CSR */
    decode_ex_csr_microcode_t next_ex_csr_microcode;
    assign next_ex_csr_microcode.enable = ~invalid_instr & opcode_is_system;
    wire [11:0] csr_addr = {instr.funct7, instr.rs2};
    wire csr_addr_zero_check1;
    nor4 csr_addr_zero_check1_nor(
        .a(csr_addr[11]),
        .b(csr_addr[10]),
        .c(csr_addr[7]),
        .d(csr_addr[5]),
        .out(csr_addr_zero_check1)
    );
    wire csr_addr_zero_check2;
    nor2 csr_addr_zero_check2_nor(
        .a(csr_addr[4]),
        .b(csr_addr[3]),
        .out(csr_addr_zero_check2)
    );
    wire csr_addr_is_valid_unused_bits;
    and4 csr_addr_is_valid_unused_bits_and(
        .a(csr_addr_zero_check1),
        .b(csr_addr_zero_check2),
        .c(csr_addr[9]),
        .d(csr_addr[8]),
        .out(csr_addr_is_valid_unused_bits)
    );
    wire csr_addr_bits_6_1_0_zero_check;
    nor4 csr_addr_bits_6_1_0_zero_check_nor(
        .a(csr_addr[6]),
        .b(csr_addr[1]),
        .c(csr_addr[0]),
        .d(1'b0),
        .out(csr_addr_bits_6_1_0_zero_check)
    );
    wire csr_addr_lower_3_odd_parity_check;
    xor3 csr_addr_lower_3_odd_parity_check_xor(
        .a(csr_addr[2]),
        .b(csr_addr[1]),
        .c(csr_addr[0]),
        .out(csr_addr_lower_3_odd_parity_check)
    );
    wire csr_addr_lower_3_not_all_one_check;
    nand3 csr_addr_lower_3_not_all_one_check_nand(
        .a(csr_addr[2]),
        .b(csr_addr[1]),
        .c(csr_addr[0]),
        .out(csr_addr_lower_3_not_all_one_check)
    );
    wire csr_addr_is_valid = csr_addr_is_valid_unused_bits & (
        csr_addr_bits_6_1_0_zero_check |
        (csr_addr[6] & csr_addr_lower_3_odd_parity_check & csr_addr_lower_3_not_all_one_check)
    );
    wire [2:0] ex_csr_op = {instr_is_system_csr, instr.funct3[1], instr.funct3[0] | instr_is_system_mret};
    assign next_ex_csr_microcode.op = ex_csr_op;
    wire invalid_ex_csr_op =
        (ex_csr_op[2] | ex_csr_op[1] | ex_csr_op[0]) &
        ~(~ex_csr_op[2] & ~ex_csr_op[1] & ex_csr_op[0]) &
        ~(~ex_csr_op[2] & ex_csr_op[1] & ex_csr_op[0]) &
        ~(ex_csr_op[2] & (ex_csr_op[1] | ex_csr_op[0]));
    wire invalid_ex_csr = instr_is_system_csr & (instr.funct3[1] | instr.funct3[0]) & (~csr_addr_is_valid | invalid_ex_csr_op);
    wire [2:0] csr_addr_mapped = {csr_addr[6], csr_addr[2] | csr_addr[1], csr_addr[2] | csr_addr[0]};
    mux2 #(.BITS($bits(next_ex_csr_microcode.addr_exception))) next_ex_csr_microcode_addr_exception_mux(
        .a({1'b1, csr_addr_mapped}),
        .b({~instr.rs2[0], `CSR_EXCEPTION_INT}),
        .select(instr_is_system_e),
        .out(next_ex_csr_microcode.addr_exception)
    );
    dffe #(.BITS($bits(ex_csr_microcode))) ex_csr_microcode_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_ex_csr_microcode),
        .out(ex_csr_microcode)
    );

    /* Memory Stage */
    decode_ma_mem_microcode_t next_ma_mem_microcode;
    assign next_ma_mem_microcode.enable = ~invalid_instr & (opcode_is_load | opcode_is_store);
    assign next_ma_mem_microcode.mem.is_write = opcode_is_store;
    assign next_ma_mem_microcode.mem.is_unsigned = instr.funct3[2];
    assign next_ma_mem_microcode.mem.op_size = instr.funct3[1:0];
    dffe #(.BITS($bits(next_ma_mem_microcode))) ma_mem_microcode_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_ma_mem_microcode),
        .out(ma_mem_microcode)
    );

    /* Write Back Stage */
    wire wb_rf_enable = ~(opcode_is_store | opcode_is_branch | opcode_is_fence | instr_is_system_e_mret);
    wire rd_is_zero = ~instr.rd[4] & ~instr.rd[3] & ~instr.rd[2] & ~instr.rd[1] & ~instr.rd[0];
    wire invalid_rd = (wb_rf_enable & instr.rd[4]) | ((opcode_is_fence | instr_is_system_e_mret) & ~rd_is_zero);
    decode_wb_rf_microcode_t next_wb_rf_microcode;
    assign next_wb_rf_microcode.enable = ~invalid_instr & wb_rf_enable;
    assign next_wb_rf_microcode.addr = instr.rd[3:0];
    dffe #(.BITS($bits(next_wb_rf_microcode))) wb_rf_microcode_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_wb_rf_microcode),
        .out(wb_rf_microcode)
    );

    /* Immediate Signals */
    wire [6:0] imm_lower_non_jal_middle;
    mux2 #(.BITS($bits(imm_lower_non_jal_middle))) imm_lower_non_jal_middle_mux(
        .a(instr.funct7),
        .b({instr.rd[0], instr.funct7[5:0]}),
        .select(opcode_is_branch),
        .out(imm_lower_non_jal_middle)
    );
    wire [4:0] imm_lower_non_jal_lower;
    wire [4:0] imm_lower_non_jal_lower_intermediate;
    and2 #(.BITS($bits(imm_lower_non_jal_lower_intermediate))) imm_lower_non_jal_lower_intermediate_and(
        .a(instr.rd),
        .b({4'b1111, ~opcode_is_branch}),
        .out(imm_lower_non_jal_lower_intermediate)
    );
    mux2 #(.BITS($bits(imm_lower_non_jal_lower))) imm_lower_non_jal_lower_mux(
        .a(instr.rs2),
        .b(imm_lower_non_jal_lower_intermediate),
        .select(opcode_is_branch | opcode_is_store),
        .out(imm_lower_non_jal_lower)
    );
    wire [19:0] imm_lower_non_jal = {
        {8{instr.funct7[6]}},
        imm_lower_non_jal_middle,
        imm_lower_non_jal_lower
    };
    wire [11:0] imm_upper;
    mux2 #(.BITS($bits(imm_upper))) imm_upper_mux(
        .a({12{~opcode_is_system & instr.funct7[6]}}),
        .b({instr.funct7, instr.rs2}),
        .select(opcode_is_lui | opcode_is_auipc),
        .out(imm_upper)
    );
    wire [4:0] imm_lower_non_system_intermediate;
    and2 #(.BITS($bits(imm_lower_non_system_intermediate))) imm_lower_non_system_intermediate_and(
        .a(instr.rs2),
        .b(5'b11110),
        .out(imm_lower_non_system_intermediate)
    );
    wire [19:0] imm_lower_non_system;
    mux2 #(.BITS($bits(imm_lower_non_system))) imm_lower_non_system_mux(
        .a(imm_lower_non_jal),
        .b({instr.rs1, instr.funct3, instr.rs2[0], instr.funct7[5:0], imm_lower_non_system_intermediate}),
        .select(opcode_is_jal),
        .out(imm_lower_non_system)
    );
    wire [19:0] imm_lower_non_lui_auipc;
    mux2 #(.BITS($bits(imm_lower_non_lui_auipc))) imm_lower_non_lui_auipc_mux(
        .a(imm_lower_non_system),
        .b({15'b0, instr.rs1}),
        .select(opcode_is_system),
        .out(imm_lower_non_lui_auipc)
    );
    wire [19:0] imm_lower;
    mux2 #(.BITS($bits(imm_lower))) imm_lower_mux(
        .a(imm_lower_non_lui_auipc),
        .b({instr.rs1, instr.funct3, 12'b0}),
        .select(opcode_is_lui | opcode_is_auipc),
        .out(imm_lower)
    );
    wire [31:0] next_imm = {imm_upper, imm_lower};
    dffe #(.BITS($bits(imm))) imm_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_imm),
        .out(imm)
    );

    /* Mux Position Signals */
    wire next_alu_a_mux_position = instr_is_system_e | opcode_is_auipc;
    dffe alu_a_mux_position_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_alu_a_mux_position),
        .out(mux_position.alu_in_a)
    );
    wire next_alu_b_mux_position = ~opcode_is_branch & ~opcode_is_arith_reg;
    dffe alu_b_mux_position_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_alu_b_mux_position),
        .out(mux_position.alu_in_b)
    );
    wire [1:0] next_csr_mux_position = {instr_is_system_e, instr_is_system_csr & instr.funct3[2]};
    dffe #(.BITS($bits(mux_position.csr_in))) csr_mux_position_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_csr_mux_position),
        .out(mux_position.csr_in)
    );
    wire [1:0] next_wb_mux_position = {
        (opcode_is_load | opcode_is_jalr | opcode_is_jal),
        (opcode_is_lui | opcode_is_jalr | opcode_is_jal)
    };
    dffe #(.BITS($bits(mux_position.wb_rf_in))) wb_mux_position_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_wb_mux_position),
        .out(mux_position.wb_rf_in)
    );
    wire [1:0] next_wb_pc_mux_position = {
        (opcode_is_branch | opcode_is_jal),
        (opcode_is_jalr | opcode_is_branch | instr_is_system_e_mret)
    };
    dffe #(.BITS($bits(mux_position.wb_pc))) wb_pc_mux_position_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_wb_pc_mux_position),
        .out(mux_position.wb_pc)
    );

    /* Fault */
    wire invalid_instr = invalid_opcode | invalid_rs | invalid_funct7 | invalid_funct3 | invalid_rd | invalid_ex_csr;
    dffe next_fault_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(invalid_instr),
        .out(fault)
    );

`ifdef FORMAL
    initial assume(~reset_n);
    logic f_past_valid;
    initial f_past_valid = 0;
    always_ff @(posedge clk) begin
        f_past_valid = 1;
    end

    /* Validate Logic */
    always_comb begin
        if (!invalid_opcode) begin
            assert((opcode_is_arith_reg + opcode_is_arith_imm + opcode_is_load + opcode_is_store + opcode_is_auipc +
                opcode_is_branch + opcode_is_jal + opcode_is_jalr + opcode_is_lui + opcode_is_fence +
                opcode_is_system) == 1);
        end
    end
    wire has_rd_stage = rd_rf_microcode.enable;
    wire has_ex_stage = ex_alu_microcode.enable | ex_csr_microcode.enable;
    wire has_ma_stage = ma_mem_microcode.enable;
    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            if (!$past(reset_n)) begin
                assert(!fault);
            end else begin
                assume($past(!enable_n));
                case ($past(instr.opcode))
                    7'b0110111: begin // LUI
                        assert($past(opcode_is_lui));
                        assert(!$past(invalid_opcode) && !$past(invalid_funct7) && !$past(invalid_funct3) && !$past(invalid_rs));
                        assert($past(invalid_rd) == $past(instr.rd[4]));
                        assert(fault == $past(invalid_rd));
                        if (!fault) begin
                            assert(!has_rd_stage && !has_ex_stage && !has_ma_stage);
                            assert(imm == {$past(instr[31:12]), 12'b0});
                            assert(mux_position.wb_rf_in == 2'b01);
                            assert(!ex_alu_microcode.enable);
                            assert(!ex_csr_microcode.enable);
                            assert(!ma_mem_microcode.enable);
                            assert(wb_rf_microcode.enable);
                            assert(wb_rf_microcode.addr == $past(instr.rd));
                            assert(mux_position.wb_pc == 2'b00);
                        end
                    end
                    7'b0010111: begin // AUIPC
                        assert($past(opcode_is_auipc));
                        assert(!$past(invalid_opcode) && !$past(invalid_funct7) && !$past(invalid_funct3) && !$past(invalid_rs));
                        assert($past(invalid_rd) == $past(instr.rd[4]));
                        assert(fault == $past(invalid_rd));
                        if (!fault) begin
                            assert(!has_rd_stage && has_ex_stage && !has_ma_stage);
                            assert(imm == {$past(instr[31:12]), 12'b0});
                            assert(mux_position.alu_in_a == 1'b1);
                            assert(mux_position.alu_in_b == 1'b1);
                            assert(mux_position.wb_rf_in == 2'b00);
                            assert(ex_alu_microcode.enable);
                            assert(!ex_alu_microcode.alu.is_add_n);
                            assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_ADD_SUB);
                            assert(!ex_csr_microcode.enable);
                            assert(!ma_mem_microcode.enable);
                            assert(wb_rf_microcode.enable);
                            assert(wb_rf_microcode.addr == $past(instr.rd));
                            assert(mux_position.wb_pc == 2'b00);
                        end
                    end
                    7'b1101111: begin // JAL
                        assert($past(opcode_is_jal));
                        assert(!$past(invalid_opcode) && !$past(invalid_funct7) && !$past(invalid_funct3) && !$past(invalid_rs));
                        assert($past(invalid_rd) == $past(instr.rd[4]));
                        assert(fault == $past(invalid_rd));
                        if (!fault) begin
                            assert(!has_rd_stage && !has_ex_stage && !has_ma_stage);
                            assert(imm == {{12{$past(instr[31])}}, $past(instr[19:12]), $past(instr[20]), $past(instr[30:21]), 1'b0});
                            assert(mux_position.wb_rf_in == 2'b11);
                            assert(!ex_alu_microcode.enable);
                            assert(!ex_csr_microcode.enable);
                            assert(!ma_mem_microcode.enable);
                            assert(wb_rf_microcode.enable);
                            assert(wb_rf_microcode.addr == $past(instr.rd));
                            assert(mux_position.wb_pc == 2'b10);
                        end
                    end
                    7'b1100111: begin // JALR
                        assert($past(opcode_is_jalr));
                        assert(!$past(invalid_opcode) && !$past(invalid_funct7));
                        assert($past(invalid_rs) == $past(instr.rs1[4]));
                        assert($past(invalid_rd) == $past(instr.rd[4]));
                        assert($past(invalid_funct3) == ($past(instr.funct3) != 3'b000));
                        assert(fault == ($past(invalid_rs) || $past(invalid_rd) || $past(invalid_funct3)));
                        if (!fault) begin
                            assert(has_rd_stage && has_ex_stage && !has_ma_stage);
                            assert(imm == {{20{$past(instr[31])}}, $past(instr[31:20])});
                            assert(mux_position.alu_in_a == 1'b0);
                            assert(mux_position.alu_in_b == 1'b1);
                            assert(mux_position.wb_rf_in == 2'b11);
                            assert(rd_rf_microcode.addr_a == $past(instr.rs1));
                            assert(ex_alu_microcode.enable);
                            assert(!ex_alu_microcode.alu.is_add_n);
                            assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_ADD_SUB);
                            assert(!ex_csr_microcode.enable);
                            assert(!ma_mem_microcode.enable);
                            assert(wb_rf_microcode.enable);
                            assert(wb_rf_microcode.addr == $past(instr.rd));
                            assert(mux_position.wb_pc == 2'b01);
                        end
                    end
                    7'b1100011: begin // BRANCH
                        assert($past(opcode_is_branch));
                        assert(!$past(invalid_opcode) && !$past(invalid_funct7) && !$past(invalid_rd));
                        assert($past(invalid_rs) == ($past(instr.rs2[4]) || $past(instr.rs1[4])));
                        assert($past(invalid_funct3) == ($past(instr.funct3[2:1]) == 2'b01));
                        assert(fault == ($past(invalid_rs) || $past(invalid_funct3)));
                        if (!fault) begin
                            assert(has_rd_stage && has_ex_stage && !has_ma_stage);
                            assert(imm == {{20{$past(instr[31])}}, $past(instr[7]), $past(instr[30:25]), $past(instr[11:8]), 1'b0});
                            assert(mux_position.alu_in_a == 1'b0);
                            assert(mux_position.alu_in_b == 1'b0);
                            assert(rd_rf_microcode.addr_a == $past(instr.rs1));
                            assert(rd_rf_microcode.addr_b == $past(instr.rs2));
                            assert(ex_alu_microcode.enable);
                            case ($past(instr.funct3))
                                3'b000: begin // BEQ
                                    assert(ex_alu_microcode.alu.is_beq_bne);
                                    assert(!ex_alu_microcode.alu.is_bge_bgeu_bne);
                                    assert(!ex_alu_microcode.alu.is_sltu_bltu_bgeu);
                                    assert(ex_alu_microcode.alu.is_add_n);
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_SLT_BRANCH);
                                end
                                3'b001: begin // BNE
                                    assert(ex_alu_microcode.alu.is_beq_bne);
                                    assert(ex_alu_microcode.alu.is_bge_bgeu_bne);
                                    assert(!ex_alu_microcode.alu.is_sltu_bltu_bgeu);
                                    assert(ex_alu_microcode.alu.is_add_n);
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_SLT_BRANCH);
                                end
                                3'b100: begin // BLT
                                    assert(!ex_alu_microcode.alu.is_beq_bne);
                                    assert(!ex_alu_microcode.alu.is_bge_bgeu_bne);
                                    assert(!ex_alu_microcode.alu.is_sltu_bltu_bgeu);
                                    assert(ex_alu_microcode.alu.is_add_n);
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_SLT_BRANCH);
                                end
                                3'b101: begin // BGE
                                    assert(!ex_alu_microcode.alu.is_beq_bne);
                                    assert(ex_alu_microcode.alu.is_bge_bgeu_bne);
                                    assert(!ex_alu_microcode.alu.is_sltu_bltu_bgeu);
                                    assert(ex_alu_microcode.alu.is_add_n);
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_SLT_BRANCH);
                                end
                                3'b110: begin // BLTU
                                    assert(!ex_alu_microcode.alu.is_beq_bne);
                                    assert(!ex_alu_microcode.alu.is_bge_bgeu_bne);
                                    assert(ex_alu_microcode.alu.is_sltu_bltu_bgeu);
                                    assert(ex_alu_microcode.alu.is_add_n);
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_SLT_BRANCH);
                                end
                                3'b111: begin // BGEU
                                    assert(!ex_alu_microcode.alu.is_beq_bne);
                                    assert(ex_alu_microcode.alu.is_bge_bgeu_bne);
                                    assert(ex_alu_microcode.alu.is_sltu_bltu_bgeu);
                                    assert(ex_alu_microcode.alu.is_add_n);
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_SLT_BRANCH);
                                end
                                default: assert(0);
                            endcase
                            assert(!ex_csr_microcode.enable);
                            assert(!ma_mem_microcode.enable);
                            assert(!wb_rf_microcode.enable);
                            assert(mux_position.wb_pc == 2'b11);
                        end
                    end
                    7'b0000011: begin // LOAD
                        assert($past(opcode_is_load));
                        assert(!$past(invalid_opcode) && !$past(invalid_funct7));
                        assert($past(invalid_rs) == $past(instr.rs1[4]));
                        assert($past(invalid_rd) == $past(instr.rd[4]));
                        assert($past(invalid_funct3) == (($past(instr.funct3) == 3'b011) || ($past(instr.funct3[2:1]) == 2'b11)));
                        assert(fault == ($past(invalid_rs) || $past(invalid_rd) || $past(invalid_funct3)));
                        if (!fault) begin
                            assert(has_rd_stage && has_ex_stage && has_ma_stage);
                            assert(imm == {{20{$past(instr[31])}}, $past(instr[31:20])});
                            assert(mux_position.alu_in_a == 1'b0);
                            assert(mux_position.alu_in_b == 1'b1);
                            assert(mux_position.wb_rf_in == 2'b10);
                            assert(rd_rf_microcode.addr_a == $past(instr.rs1));
                            assert(ex_alu_microcode.enable);
                            assert(!ex_alu_microcode.alu.is_add_n);
                            assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_ADD_SUB);
                            assert(!ex_csr_microcode.enable);
                            assert(ma_mem_microcode.enable);
                            assert(!ma_mem_microcode.mem.is_write);
                            assert(ma_mem_microcode.mem.is_unsigned == $past(instr.funct3[2]));
                            assert(ma_mem_microcode.mem.op_size == $past(instr.funct3[1:0]));
                            assert(wb_rf_microcode.enable);
                            assert(wb_rf_microcode.addr == $past(instr.rd));
                            assert(mux_position.wb_pc == 2'b00);
                        end
                    end
                    7'b0100011: begin // STORE
                        assert($past(opcode_is_store));
                        assert(!$past(invalid_opcode) && !$past(invalid_funct7) && !$past(invalid_rd));
                        assert($past(invalid_rs) == ($past(instr.rs2[4]) || $past(instr.rs1[4])));
                        assert($past(invalid_funct3) == ($past(instr.funct3[2]) || ($past(instr.funct3[1:0]) == 2'b11)));
                        assert(fault == $past(invalid_rs) || $past(invalid_funct3));
                        if (!fault) begin
                            assert(has_rd_stage && has_ex_stage && has_ma_stage);
                            assert(imm == {{20{$past(instr[31])}}, $past(instr.funct7), $past(instr.rd)});
                            assert(mux_position.alu_in_a == 1'b0);
                            assert(mux_position.alu_in_b == 1'b1);
                            assert(rd_rf_microcode.addr_a == $past(instr.rs1));
                            assert(rd_rf_microcode.addr_b == $past(instr.rs2));
                            assert(ex_alu_microcode.enable);
                            assert(!ex_alu_microcode.alu.is_add_n);
                            assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_ADD_SUB);
                            assert(ma_mem_microcode.enable);
                            assert(ma_mem_microcode.mem.is_write);
                            assert(ma_mem_microcode.mem.is_unsigned == $past(instr.funct3[2]));
                            assert(ma_mem_microcode.mem.op_size == $past(instr.funct3[1:0]));
                            assert(!wb_rf_microcode.enable);
                            assert(mux_position.wb_pc == 2'b00);
                        end
                    end
                    7'b0010011: begin // Arith-Imm
                        assert($past(opcode_is_arith_imm));
                        assert(!$past(invalid_opcode) && !$past(invalid_funct3));
                        assert($past(invalid_funct7) == (($past(instr.funct3) == 3'b001) && ($past(instr.funct7) != 7'b0000000) ||
                            (($past(instr.funct3) == 3'b101) && (($past(instr[31]) != 1'b0) || ($past(instr[29:25]) != 5'b00000)))));
                        assert($past(invalid_rs) == $past(instr.rs1[4]));
                        assert($past(invalid_rd) == $past(instr.rd[4]));
                        assert(fault == ($past(invalid_funct7) || $past(invalid_rs) || $past(invalid_rd)));
                        if (!fault) begin
                            assert(has_rd_stage && has_ex_stage && !has_ma_stage);
                            assert(imm == {{20{$past(instr[31])}}, $past(instr[31:20])});
                            assert(mux_position.alu_in_a == 1'b0);
                            assert(mux_position.alu_in_b == 1'b1);
                            assert(mux_position.wb_rf_in == 2'b00);
                            assert(rd_rf_microcode.addr_a == $past(instr.rs1));
                            assert(ex_alu_microcode.enable);
                            case ($past(instr.funct3))
                                3'b000: begin // ADDI
                                    assert(!ex_alu_microcode.alu.is_add_n);
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_ADD_SUB);
                                end
                                3'b001: begin // SLLI
                                    assert(ex_alu_microcode.alu.shifter_op == SHIFTER_OP_SLL);
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_SLL);
                                end
                                3'b010: begin // SLTI
                                    assert(!ex_alu_microcode.alu.is_beq_bne);
                                    assert(!ex_alu_microcode.alu.is_bge_bgeu_bne);
                                    assert(!ex_alu_microcode.alu.is_sltu_bltu_bgeu);
                                    assert(ex_alu_microcode.alu.is_add_n);
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_SLT_BRANCH);
                                end
                                3'b011: begin // SLTIU
                                    assert(!ex_alu_microcode.alu.is_beq_bne);
                                    assert(!ex_alu_microcode.alu.is_bge_bgeu_bne);
                                    assert(ex_alu_microcode.alu.is_sltu_bltu_bgeu);
                                    assert(ex_alu_microcode.alu.is_add_n);
                                end
                                3'b100: begin // XORI
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_XOR);
                                end
                                3'b101: begin
                                    if ($past(instr.funct7[5])) // SRAI
                                        assert(ex_alu_microcode.alu.shifter_op == SHIFTER_OP_SRA);
                                    else // SRLI
                                        assert(ex_alu_microcode.alu.shifter_op == SHIFTER_OP_SRL);
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_SRL_SRA);
                                end
                                3'b110: begin // ORI
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_OR);
                                end
                                3'b111: begin // ANDI
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_AND);
                                end
                                default: assert(0);
                            endcase
                            assert(!ex_csr_microcode.enable);
                            assert(!ma_mem_microcode.enable);
                            assert(wb_rf_microcode.enable);
                            assert(wb_rf_microcode.addr == $past(instr.rd));
                            assert(mux_position.wb_pc == 2'b00);
                        end
                    end
                    7'b0110011: begin // Arith-Reg
                        assert($past(opcode_is_arith_reg));
                        assert(!$past(invalid_opcode) && !$past(invalid_funct3));
                        assert($past(invalid_funct7) == ((($past(instr[31]) != 1'b0) || ($past(instr[29:25]) != 5'b00000)) ||
                            ($past(instr[30]) && ($past(instr.funct3) != 3'b000) && ($past(instr.funct3) != 3'b101))));
                        assert($past(invalid_rs) == ($past(instr.rs1[4]) || $past(instr.rs2[4])));
                        assert($past(invalid_rd) == $past(instr.rd[4]));
                        assert(fault == ($past(invalid_funct7) || $past(invalid_rs) || $past(invalid_rd)));
                        if (!fault) begin
                            assert(has_rd_stage && has_ex_stage && !has_ma_stage);
                            assert(mux_position.alu_in_a == 1'b0);
                            assert(mux_position.alu_in_b == 1'b0);
                            assert(mux_position.wb_rf_in == 2'b00);
                            assert(rd_rf_microcode.addr_a == $past(instr.rs1));
                            assert(rd_rf_microcode.addr_b == $past(instr[23:20]));
                            assert(ex_alu_microcode.enable);
                            case ($past(instr.funct3))
                                3'b000: begin
                                    if ($past(instr[30])) // SUB
                                        assert(ex_alu_microcode.alu.is_add_n);
                                    else // ADD
                                        assert(!ex_alu_microcode.alu.is_add_n);
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_ADD_SUB);
                                end
                                3'b001: begin // SLL
                                    assert(ex_alu_microcode.alu.shifter_op == SHIFTER_OP_SLL);
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_SLL);
                                end
                                3'b010: begin // SLT
                                    assert(!ex_alu_microcode.alu.is_beq_bne);
                                    assert(!ex_alu_microcode.alu.is_bge_bgeu_bne);
                                    assert(!ex_alu_microcode.alu.is_sltu_bltu_bgeu);
                                    assert(ex_alu_microcode.alu.is_add_n);
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_SLT_BRANCH);
                                end
                                3'b011: begin // SLTU
                                    assert(!ex_alu_microcode.alu.is_beq_bne);
                                    assert(!ex_alu_microcode.alu.is_bge_bgeu_bne);
                                    assert(ex_alu_microcode.alu.is_sltu_bltu_bgeu);
                                    assert(ex_alu_microcode.alu.is_add_n);
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_SLTU);
                                end
                                3'b100: begin // XOR
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_XOR);
                                end
                                3'b101: begin
                                    if ($past(instr.funct7[5])) // SRA
                                        assert(ex_alu_microcode.alu.shifter_op == SHIFTER_OP_SRA);
                                    else // SRL
                                        assert(ex_alu_microcode.alu.shifter_op == SHIFTER_OP_SRL);
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_SRL_SRA);
                                end
                                3'b110: begin // OR
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_OR);
                                end
                                3'b111: begin // AND
                                    assert(ex_alu_microcode.alu.output_select == ALU_OUTPUT_SELECT_AND);
                                end
                                default: assert(0);
                            endcase
                            assert(!ex_csr_microcode.enable);
                            assert(!ma_mem_microcode.enable);
                            assert(wb_rf_microcode.enable);
                            assert(wb_rf_microcode.addr == $past(instr.rd));
                            assert(mux_position.wb_pc == 2'b00);
                        end
                    end
                    7'b0001111: begin // FENCE
                        assert($past(opcode_is_fence));
                        assert(!$past(invalid_opcode));
                        assert($past(invalid_funct3) == ($past(instr.funct3[2:1]) != 2'b0));
                        if (!$past(instr.funct3[0])) begin
                            // FENCE
                            assert($past(invalid_funct7) == ($past(instr[31:28]) != 4'b0));
                            assert($past(invalid_rs) == ($past(instr.rs1) != 5'b0));
                        end else begin
                            // FENCE.I
                            assert($past(invalid_funct7) == ($past(instr.funct7) != 7'b0));
                            assert($past(invalid_rs) == ($past({instr.rs1, instr.rs2}) != 10'b0));
                        end
                        assert($past(invalid_rd) == ($past(instr.rd) != 5'b0));
                        assert(fault == ($past(invalid_funct7) || $past(invalid_funct3) || $past(invalid_rs) || $past(invalid_rd)));
                        if (!fault) begin
                            assert(!has_rd_stage && !has_ex_stage && !has_ma_stage);
                            assert(!ex_alu_microcode.enable);
                            assert(!ex_csr_microcode.enable);
                            assert(!ma_mem_microcode.enable);
                            assert(!wb_rf_microcode.enable);
                            assert(mux_position.wb_pc == 2'b00);
                        end
                    end
                    7'b1110011: begin // SYSTEM
                        assert($past(opcode_is_system));
                        assert(!$past(invalid_opcode));
                        if (($past(instr[31:7]) == 25'b0000000000000000000000000) || $past(instr[31:7]) == 25'b0000000000010000000000000) begin
                            // ECALL / EBREAK
                            assert($past(instr_is_system_e_mret));
                            assert($past(instr_is_system_e));
                            assert(!$past(instr_is_system_mret));
                            assert(!$past(instr_is_system_csr));
                            assert(!$past(invalid_funct7) && !$past(invalid_funct3) && !$past(invalid_funct7) &&
                                !$past(invalid_rs) && !$past(invalid_rd));
                            assert(!fault);
                            assert(!has_rd_stage && has_ex_stage && !has_ma_stage);
                            assert(!ex_alu_microcode.enable);
                            assert(ex_csr_microcode == {1'b1, ~$past(instr[20]), `CSR_EXCEPTION_INT, 3'b000});
                            assert(!ma_mem_microcode.enable);
                            assert(!wb_rf_microcode.enable);
                            assert(mux_position.wb_pc == 2'b01);
                            assert(mux_position.csr_in == 2'b10);
                        end else if ($past(instr[31:7]) == 25'b0011000000100000000000000) begin
                            // MRET
                            assert($past(instr_is_system_e_mret));
                            assert(!$past(instr_is_system_e));
                            assert($past(instr_is_system_mret));
                            assert(!$past(instr_is_system_csr));
                            assert(!$past(invalid_funct7) && !$past(invalid_funct3) && !$past(invalid_funct7) &&
                                !$past(invalid_rs) && !$past(invalid_rd));
                            assert(!fault);
                            assert(!has_rd_stage && has_ex_stage && !has_ma_stage);
                            assert(!ex_alu_microcode.enable);
                            assert(ex_csr_microcode.enable);
                            assert(ex_csr_microcode.op == CSR_OP_MRET);
                            assert(!ma_mem_microcode.enable);
                            assert(!wb_rf_microcode.enable);
                            assert(mux_position.wb_pc == 2'b01);
                        end else if ($past(instr.funct3[1:0]) != 2'b00) begin
                            // CSR
                            assert(!$past(invalid_funct3));
                            assert(!$past(invalid_funct7));
                            if ($past(instr.funct3[2])) begin
                                // immediate
                                assert(!$past(invalid_rs));
                            end else begin
                                // register
                                assert($past(invalid_rs) == $past(instr.rs1[4]));
                            end
                            assert($past(invalid_rd) == ($past(instr.rd[4]) != 1'b0));
                            if ($past(invalid_rs | invalid_rd)) begin
                                assert(fault);
                            end else begin
                                case ($past(instr[31:20]))
                                    12'h300: begin // mstatus
                                        assert(!fault);
                                        assert(ex_csr_microcode.enable);
                                        assert(ex_csr_microcode == {1'b1, 1'b1, CSR_MAPPED_ADDR_MSTATUS, 1'b1, $past(instr.funct3[1:0])});
                                    end
                                    12'h304: begin // mie
                                        assert(!fault);
                                        assert(ex_csr_microcode.enable);
                                        assert(ex_csr_microcode == {1'b1, 1'b1, CSR_MAPPED_ADDR_MIE, 1'b1, $past(instr.funct3[1:0])});
                                    end
                                    12'h341: begin // mepc
                                        assert(!fault);
                                        assert(ex_csr_microcode.enable);
                                        assert(ex_csr_microcode == {1'b1, 1'b1, CSR_MAPPED_ADDR_MEPC, 1'b1, $past(instr.funct3[1:0])});
                                    end
                                    12'h342: begin // mcause
                                        assert(!fault);
                                        assert(ex_csr_microcode.enable);
                                        assert(ex_csr_microcode == {1'b1, 1'b1, CSR_MAPPED_ADDR_MCAUSE, 1'b1, $past(instr.funct3[1:0])});
                                    end
                                    12'h344: begin // mip
                                        assert(!fault);
                                        assert(ex_csr_microcode.enable);
                                        assert(ex_csr_microcode == {1'b1, 1'b1, CSR_MAPPED_ADDR_MIP, 1'b1, $past(instr.funct3[1:0])});
                                    end
                                    default: begin
                                        assert(fault);
                                    end
                                endcase
                                if (!fault) begin
                                    assert(!$past(instr_is_system_e_mret));
                                    assert(!$past(instr_is_system_e));
                                    assert(!$past(instr_is_system_mret));
                                    assert($past(instr_is_system_csr));
                                    assert(has_ex_stage);
                                    assert(!has_ma_stage);
                                    assert(mux_position.alu_in_a == 1'b0);
                                    if ($past(instr.funct3[2])) begin
                                        // immediate
                                        assert(!has_rd_stage);
                                        assert(mux_position.csr_in == 2'b01);
                                    end else begin
                                        // register
                                        assert(has_rd_stage);
                                        assert(rd_rf_microcode.addr_a == $past(instr.rs1));
                                        assert(mux_position.csr_in == 2'b00);
                                    end
                                    assert(!ex_alu_microcode.enable);
                                    assert(!ma_mem_microcode.enable);
                                    assert(wb_rf_microcode.enable);
                                    assert(wb_rf_microcode.addr == $past(instr.rd));
                                    assert(mux_position.wb_pc == 2'b00);
                                    assert(mux_position.wb_rf_in == 2'b00);
                                end
                            end
                        end else begin
                            assert(fault);
                        end
                    end
                    default: begin
                        assert($past(invalid_opcode));
                    end
                endcase
            end
        end
    end
`endif

endmodule
