`include "cells/and2.sv"
`include "cells/dffe.sv"
`include "cells/mux2.sv"

/*
 * A decoder for the RV32I instruction set.
 */
module instruction_decode(
    input logic clk, // Clock signal
    input logic reset_n, // Reset signal (active low)
    input logic enable_n, // Enable (active low)
    input logic [31:0] instr, // Instruction to decode
    output logic [31:0] imm, // Immediate value
    output logic alu_a_mux_position, // Position of the ALU A mux (0=rs1, 1=pc)
    output logic alu_b_mux_position, // Position of the ALU B mux (0=rs2, 1=imm)
    output logic [1:0] csr_mux_position, // Position of the CSR mux (0=rs1, 1=imm, 2=npc, 3=UNUSED)
    output logic [1:0] wb_mux_position, // Position of the write back mux (0=alu_res, 1=imm, 2=mem_data, 3=npc)
    /* verilator lint_off UNOPTFLAT */
    output logic [8:0] rd_rf_microcode, // Read stage register file microcode
    /* verilator lint_on UNOPTFLAT */
    output logic [5:0] ex_alu_microcode, // Execute stage ALU microcode
    output logic [4:0] ma_mem_microcode, // Memory access stage memory microcode
    output logic [15:0] ex_csr_microcode, // Execute stage CSR microcode
    output logic [4:0] wb_rf_microcode, // Write back stage register file microcode
    output logic [1:0] wb_pc_mux_position, // Write back stage program counter mux position (00=npc, 01=I[31:1], 10=pc+offset, 11=Y?I+offset:npc)
    output logic fault // Fault condition (i.e. invalid instruction)
);

    /* Basic Decode */
    wire [6:0] opcode = instr[6:0];
    wire [4:0] rd = instr[11:7];
    wire [2:0] funct3 = instr[14:12];
    wire [4:0] rs1 = instr[19:15];
    wire [4:0] rs2 = instr[24:20];
    wire [6:0] funct7 = instr[31:25];
    wire opcode_is_op = ~opcode[6] & opcode[5] & opcode[4] & ~opcode[2];
    wire opcode_is_opimm = ~opcode[5] & opcode[4] & ~opcode[2];
    wire opcode_is_load = ~opcode[5] & ~opcode[4] & ~opcode[3];
    wire opcode_is_store = ~opcode[6] & opcode[5] & ~opcode[4];
    wire opcode_is_auipc = ~opcode[5] & ~opcode[3] & opcode[2];
    wire opcode_is_branch = opcode[6] & ~opcode[4] & ~opcode[2];
    wire opcode_is_jal = opcode[5] & opcode[3];
    wire opcode_is_jalr = ~opcode[4] & ~opcode[3] & opcode[2];
    wire opcode_is_lui = ~opcode[6] & opcode[5] & opcode[2];
    wire opcode_is_fence = ~opcode[6] & opcode[3];
    wire opcode_is_system = opcode[6] & opcode[4];
    wire invalid_opcode = ~opcode[0] | ~opcode[1] |
        (opcode[3] & ~opcode[2]) |
        (opcode[4] & opcode[3]) |
        (opcode[6] & ~opcode[5]) |
        (~opcode[6] & opcode[5] & opcode[3]) |
        (opcode[6] & opcode[4] & opcode[2]) |
        (~opcode[6] & ~opcode[4] & ~opcode[3] & opcode[2]);
    wire instr_is_system_e_mret = opcode_is_system & ~funct3[2] & ~funct3[1] & ~funct3[0];
    wire instr_is_system_mret = instr_is_system_e_mret & funct7[4];
    wire instr_is_system_e = instr_is_system_e_mret & ~instr_is_system_mret;
    wire instr_is_system_csr = opcode_is_system & ~instr_is_system_e_mret;

    /* Read Stage */
    wire rd_rf_enable = ~(opcode_is_lui | opcode_is_auipc | opcode_is_jal | opcode_is_fence | opcode_is_system) |
        (instr_is_system_csr & ~funct3[2]);
    wire rs1_is_zero = ~rs1[4] & ~rs1[3] & ~rs1[2] & ~rs1[1] & ~rs1[0];
    wire invalid_rs = (rd_rf_enable & (rs1[4] | ((opcode_is_op | opcode_is_store | opcode_is_branch) & rs2[4]))) |
        (opcode_is_fence & (~rs1_is_zero | (funct3[0] & (rs2 != 5'b0)))) |
        (instr_is_system_e & (~rs1_is_zero | rs2[4] | rs2[3] | rs2[2] | rs2[1])) |
        (instr_is_system_mret & (~rs1_is_zero | rs2[4] | rs2[3] | rs2[2] | ~rs2[1] | rs2[0]));
    wire [8:0] next_rd_rf_microcode = {
        ~invalid_instr & rd_rf_enable,
        rs1[3:0],
        rs2[3:0]
    };
    dffe #(.BITS(9)) rd_rf_microcode_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_rd_rf_microcode),
        .out(rd_rf_microcode)
    );

    /* Execute Stage */
    wire invalid_funct3 = (opcode_is_store & funct3[2]) |
        (opcode_is_jalr & funct3[0]) |
        (opcode_is_jalr & funct3[1]) |
        (opcode_is_jalr & funct3[2]) |
        (opcode_is_store & funct3[1] & funct3[0]) |
        (opcode_is_load & funct3[1] & funct3[0]) |
        (opcode_is_load & funct3[2] & funct3[1]) |
        (opcode_is_branch & ~funct3[2] & funct3[1]) |
        (opcode_is_fence & (funct3[2] | funct3[1])) |
        (opcode_is_system & funct3[2] & ~funct3[1] & ~funct3[0]);
    wire funct7_invalid_op_opimm_bit_set = funct7[6] | funct7[4] | funct7[3] | funct7[2] | funct7[1] | funct7[0];
    wire invalid_funct7 = (opcode_is_op & funct7_invalid_op_opimm_bit_set) |
        (opcode_is_op & funct3[1] & funct7[5]) |
        (opcode_is_opimm & ~funct3[1] & funct3[0] & funct7_invalid_op_opimm_bit_set) |
        (opcode_is_op & ~funct3[2] & funct3[0] & funct7[5]) |
        (opcode_is_op & funct3[2] & ~funct3[0] & funct7[5]) |
        (opcode_is_opimm & ~funct3[2] & ~funct3[1] & funct3[0] & funct7[5]) |
        (opcode_is_fence & (funct7[6] | funct7[5] | funct7[4] | funct7[3])) |
        (opcode_is_fence & funct3[0] & (funct7[2] | funct7[1] | funct7[0])) |
        (instr_is_system_e & (funct7[6] | funct7[5] | funct7[4] | funct7[3] | funct7[2] | funct7[1] | funct7[0])) |
        (instr_is_system_mret & (funct7[6] | funct7[5] | ~funct7[4] | ~funct7[3] | funct7[2] | funct7[1] | funct7[0]));
    wire [5:0] next_ex_alu_microcode = {
        ~invalid_instr & (opcode_is_op | opcode_is_opimm | opcode_is_branch | opcode_is_load | opcode_is_store | opcode_is_auipc |
            opcode_is_jalr),
        opcode_is_branch,
        funct7[5] & ~funct3[1] & (opcode_is_op | opcode_is_opimm) & (opcode_is_op | funct3[2]) &
            (funct3[2] | ~funct3[0]) & (~funct3[2] | funct3[0]),
        ({3{opcode_is_op | opcode_is_opimm | opcode_is_branch}} & funct3)
    };
    dffe #(.BITS(6)) ex_alu_microcode_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_ex_alu_microcode),
        .out(ex_alu_microcode)
    );
    wire [11:0] next_ex_csr_microcode_part;
    mux2 #(.BITS(12)) next_ex_csr_microcode_part_mux(
        .a({funct7, rs2}),
        .b({8'b0, ~rs2[0], 3'b011}),
        .select(instr_is_system_e),
        .out(next_ex_csr_microcode_part)
    );
    wire [15:0] next_ex_csr_microcode = {
        ~invalid_instr & opcode_is_system,
        next_ex_csr_microcode_part,
        instr_is_system_csr,
        funct3[1],
        funct3[0] | instr_is_system_mret
    };
    dffe #(.BITS(16)) ex_csr_microcode_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_ex_csr_microcode),
        .out(ex_csr_microcode)
    );

    /* Memory Stage */
    wire [4:0] next_ma_mem_microcode = {
        ~invalid_instr & (opcode_is_load | opcode_is_store),
        opcode_is_store,
        funct3
    };
    dffe #(.BITS(5)) ma_mem_microcode_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_ma_mem_microcode),
        .out(ma_mem_microcode)
    );

    /* Write Back Stage */
    wire wb_rf_enable = ~(opcode_is_store | opcode_is_branch | opcode_is_fence | instr_is_system_e_mret);
    wire rd_is_zero = ~rd[4] & ~rd[3] & ~rd[2] & ~rd[1] & ~rd[0];
    wire invalid_rd = (wb_rf_enable & rd[4]) | ((opcode_is_fence | instr_is_system_e_mret) & ~rd_is_zero);
    wire [4:0] next_wb_rf_microcode = {
        ~invalid_instr & wb_rf_enable,
        rd[3:0]
    };
    dffe #(.BITS(5)) wb_rf_microcode_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_wb_rf_microcode),
        .out(wb_rf_microcode)
    );

    /* Immediate Signals */
    wire [6:0] imm_lower_non_jal_middle;
    mux2 #(.BITS(7)) imm_lower_non_jal_middle_mux(
        .a(funct7),
        .b({rd[0], funct7[5:0]}),
        .select(opcode_is_branch),
        .out(imm_lower_non_jal_middle)
    );
    wire [4:0] imm_lower_non_jal_lower;
    wire [4:0] imm_lower_non_jal_lower_intermediate;
    and2 #(.BITS(5)) imm_lower_non_jal_lower_intermediate_and(
        .a(rd),
        .b({4'b1111, ~opcode_is_branch}),
        .out(imm_lower_non_jal_lower_intermediate)
    );
    mux2 #(.BITS(5)) imm_lower_non_jal_lower_mux(
        .a(rs2),
        .b(imm_lower_non_jal_lower_intermediate),
        .select(opcode_is_branch | opcode_is_store),
        .out(imm_lower_non_jal_lower)
    );
    wire [19:0] imm_lower_non_jal = {
        {8{funct7[6]}},
        imm_lower_non_jal_middle,
        imm_lower_non_jal_lower
    };
    wire [11:0] imm_upper;
    mux2 #(.BITS(12)) imm_upper_mux(
        .a({12{~opcode_is_system & funct7[6]}}),
        .b({funct7, rs2}),
        .select(opcode_is_lui | opcode_is_auipc),
        .out(imm_upper)
    );
    wire [4:0] imm_lower_non_system_intermediate;
    and2 #(.BITS(5)) imm_lower_non_system_intermediate_and(
        .a(rs2),
        .b(5'b11110),
        .out(imm_lower_non_system_intermediate)
    );
    wire [19:0] imm_lower_non_system;
    mux2 #(.BITS(20)) imm_lower_non_system_mux(
        .a(imm_lower_non_jal),
        .b({rs1, funct3, rs2[0], funct7[5:0], imm_lower_non_system_intermediate}),
        .select(opcode_is_jal),
        .out(imm_lower_non_system)
    );
    wire [19:0] imm_lower_non_lua_auipc;
    mux2 #(.BITS(20)) imm_lower_non_lua_auipc_mux(
        .a(imm_lower_non_system),
        .b({15'b0, rs1}),
        .select(opcode_is_system),
        .out(imm_lower_non_lua_auipc)
    );
    wire [19:0] imm_lower;
    mux2 #(.BITS(20)) imm_lower_mux(
        .a(imm_lower_non_lua_auipc),
        .b({rs1, funct3, 12'b0}),
        .select(opcode_is_lui | opcode_is_auipc),
        .out(imm_lower)
    );
    wire [31:0] next_imm = {imm_upper, imm_lower};
    dffe #(.BITS(32)) imm_dffe(
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
        .out(alu_a_mux_position)
    );
    wire next_alu_b_mux_position = ~opcode_is_branch & ~opcode_is_op;
    dffe alu_b_mux_position_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_alu_b_mux_position),
        .out(alu_b_mux_position)
    );
    wire [1:0] next_csr_mux_position = {instr_is_system_e, instr_is_system_csr & funct3[2]};
    dffe #(.BITS(2)) csr_mux_position_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_csr_mux_position),
        .out(csr_mux_position)
    );
    wire [1:0] next_wb_mux_position = {
        (opcode_is_load | opcode_is_jalr | opcode_is_jal),
        (opcode_is_lui | opcode_is_jalr | opcode_is_jal)
    };
    dffe #(.BITS(2)) wb_mux_position_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_wb_mux_position),
        .out(wb_mux_position)
    );
    wire [1:0] next_wb_pc_mux_position = {
        (opcode_is_branch | opcode_is_jal),
        (opcode_is_jalr | opcode_is_branch | instr_is_system_e_mret)
    };
    dffe #(.BITS(2)) wb_pc_mux_position_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_wb_pc_mux_position),
        .out(wb_pc_mux_position)
    );

    /* Fault */
    wire invalid_instr = invalid_opcode | invalid_rs | invalid_funct7 | invalid_funct3 | invalid_rd;
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
            assert((opcode_is_op + opcode_is_opimm + opcode_is_load + opcode_is_store + opcode_is_auipc +
                opcode_is_branch + opcode_is_jal + opcode_is_jalr + opcode_is_lui + opcode_is_fence +
                opcode_is_system) == 1);
        end
    end
    wire has_rd_stage = rd_rf_microcode[8];
    wire has_ex_stage = ex_alu_microcode[5] | ex_csr_microcode[15];
    wire has_ma_stage = ma_mem_microcode[4];
    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            if (!$past(reset_n)) begin
                assert(!fault);
            end else begin
                assume($past(~enable_n));
                case ($past(instr[6:0]))
                    7'b0110111: begin // LUI
                        assert($past(opcode_is_lui));
                        assert(!$past(invalid_opcode) && !$past(invalid_funct7) && !$past(invalid_funct3) && !$past(invalid_rs));
                        assert($past(invalid_rd) == $past(instr[11]));
                        assert(fault == $past(invalid_rd));
                        if (!fault) begin
                            assert(!has_rd_stage && !has_ex_stage && !has_ma_stage);
                            assert(imm == {$past(instr[31:12]), 12'b0});
                            assert(wb_mux_position == 2'b01);
                            assert(ex_alu_microcode[5] == 1'b0);
                            assert(ex_csr_microcode[15] == 1'b0);
                            assert(ma_mem_microcode[4] == 1'b0);
                            assert(wb_rf_microcode == {1'b1, $past(instr[10:7])});
                            assert(wb_pc_mux_position == 2'b00);
                        end
                    end
                    7'b0010111: begin // AUIPC
                        assert($past(opcode_is_auipc));
                        assert(!$past(invalid_opcode) && !$past(invalid_funct7) && !$past(invalid_funct3) && !$past(invalid_rs));
                        assert($past(invalid_rd) == $past(instr[11]));
                        assert(fault == $past(invalid_rd));
                        if (!fault) begin
                            assert(!has_rd_stage && has_ex_stage && !has_ma_stage);
                            assert(imm == {$past(instr[31:12]), 12'b0});
                            assert(alu_a_mux_position == 1'b1);
                            assert(alu_b_mux_position == 1'b1);
                            assert(wb_mux_position == 2'b00);
                            assert(ex_alu_microcode == 6'b100000);
                            assert(ex_csr_microcode[15] == 1'b0);
                            assert(ma_mem_microcode[4] == 1'b0);
                            assert(wb_rf_microcode == {1'b1, $past(instr[10:7])});
                            assert(wb_pc_mux_position == 2'b00);
                        end
                    end
                    7'b1101111: begin // JAL
                        assert($past(opcode_is_jal));
                        assert(!$past(invalid_opcode) && !$past(invalid_funct7) && !$past(invalid_funct3) && !$past(invalid_rs));
                        assert($past(invalid_rd) == $past(instr[11]));
                        assert(fault == $past(invalid_rd));
                        if (!fault) begin
                            assert(!has_rd_stage && !has_ex_stage && !has_ma_stage);
                            assert(imm == {{12{$past(instr[31])}}, $past(instr[19:12]), $past(instr[20]), $past(instr[30:21]), 1'b0});
                            assert(wb_mux_position == 2'b11);
                            assert(ex_alu_microcode[5] == 1'b0);
                            assert(ex_csr_microcode[15] == 1'b0);
                            assert(ma_mem_microcode[4] == 1'b0);
                            assert(wb_rf_microcode == {1'b1, $past(instr[10:7])});
                            assert(wb_pc_mux_position == 2'b10);
                        end
                    end
                    7'b1100111: begin // JALR
                        assert($past(opcode_is_jalr));
                        assert(!$past(invalid_opcode) && !$past(invalid_funct7));
                        assert($past(invalid_rs) == $past(instr[19]));
                        assert($past(invalid_rd) == $past(instr[11]));
                        assert($past(invalid_funct3) == ($past(instr[14:12]) != 3'b000));
                        assert(fault == ($past(invalid_rs) || $past(invalid_rd) || $past(invalid_funct3)));
                        if (!fault) begin
                            assert(has_rd_stage && has_ex_stage && !has_ma_stage);
                            assert(imm == {{20{$past(instr[31])}}, $past(instr[31:20])});
                            assert(alu_a_mux_position == 1'b0);
                            assert(alu_b_mux_position == 1'b1);
                            assert(wb_mux_position == 2'b11);
                            assert(rd_rf_microcode[7:4] == $past(instr[18:15]));
                            assert(ex_alu_microcode == 6'b100000);
                            assert(ex_csr_microcode[15] == 1'b0);
                            assert(ma_mem_microcode[4] == 1'b0);
                            assert(wb_rf_microcode == {1'b1, $past(instr[10:7])});
                            assert(wb_pc_mux_position == 2'b01);
                        end
                    end
                    7'b1100011: begin // BRANCH
                        assert($past(opcode_is_branch));
                        assert(!$past(invalid_opcode) && !$past(invalid_funct7) && !$past(invalid_rd));
                        assert($past(invalid_rs) == ($past(instr[24]) || $past(instr[19])));
                        assert($past(invalid_funct3) == ($past(instr[14:13]) == 2'b01));
                        assert(fault == ($past(invalid_rs) || $past(invalid_funct3)));
                        if (!fault) begin
                            assert(has_rd_stage && has_ex_stage && !has_ma_stage);
                            assert(imm == {{20{$past(instr[31])}}, $past(instr[7]), $past(instr[30:25]), $past(instr[11:8]), 1'b0});
                            assert(alu_a_mux_position == 1'b0);
                            assert(alu_b_mux_position == 1'b0);
                            assert(rd_rf_microcode[7:0] == {$past(instr[18:15]), $past(instr[23:20])});
                            assert(ex_alu_microcode == {3'b110, $past(instr[14:12])});
                            assert(ex_csr_microcode[15] == 1'b0);
                            assert(ma_mem_microcode[4] == 1'b0);
                            assert(wb_rf_microcode[4] == 1'b0);
                            assert(wb_pc_mux_position == 2'b11);
                        end
                    end
                    7'b0000011: begin // LOAD
                        assert($past(opcode_is_load));
                        assert(!$past(invalid_opcode) && !$past(invalid_funct7));
                        assert($past(invalid_rs) == $past(instr[19]));
                        assert($past(invalid_rd) == $past(instr[11]));
                        assert($past(invalid_funct3) == (($past(instr[14:12]) == 3'b011) || ($past(instr[14:13]) == 2'b11)));
                        assert(fault == ($past(invalid_rs) || $past(invalid_rd) || $past(invalid_funct3)));
                        if (!fault) begin
                            assert(has_rd_stage && has_ex_stage && has_ma_stage);
                            assert(imm == {{20{$past(instr[31])}}, $past(instr[31:20])});
                            assert(alu_a_mux_position == 1'b0);
                            assert(alu_b_mux_position == 1'b1);
                            assert(wb_mux_position == 2'b10);
                            assert(rd_rf_microcode[7:4] == $past(instr[18:15]));
                            assert(ex_alu_microcode == 6'b100000);
                            assert(ex_csr_microcode[15] == 1'b0);
                            assert(ma_mem_microcode == {2'b10, $past(instr[14:12])});
                            assert(wb_rf_microcode == {1'b1, $past(instr[10:7])});
                            assert(wb_pc_mux_position == 2'b00);
                        end
                    end
                    7'b0100011: begin // STORE
                        assert($past(opcode_is_store));
                        assert(!$past(invalid_opcode) && !$past(invalid_funct7) && !$past(invalid_rd));
                        assert($past(invalid_rs) == ($past(instr[24]) || $past(instr[19])));
                        assert($past(invalid_funct3) == ($past(instr[14]) || ($past(instr[13:12]) == 2'b11)));
                        assert(fault == $past(invalid_rs) || $past(invalid_funct3));
                        if (!fault) begin
                            assert(has_rd_stage && has_ex_stage && has_ma_stage);
                            assert(imm == {{20{$past(instr[31])}}, $past(instr[31:25]), $past(instr[11:7])});
                            assert(alu_a_mux_position == 1'b0);
                            assert(alu_b_mux_position == 1'b1);
                            assert(rd_rf_microcode[7:0] == {$past(instr[18:15]), $past(instr[23:20])});
                            assert(ex_alu_microcode == 6'b100000);
                            assert(ex_csr_microcode[15] == 1'b0);
                            assert(ma_mem_microcode == {2'b11, $past(instr[14:12])});
                            assert(wb_rf_microcode[4] == 1'b0);
                            assert(wb_pc_mux_position == 2'b00);
                        end
                    end
                    7'b0010011: begin // OP-IMM
                        assert($past(opcode_is_opimm));
                        assert(!$past(invalid_opcode) && !$past(invalid_funct3));
                        assert($past(invalid_funct7) == (($past(instr[14:12]) == 3'b001) && ($past(instr[31:25]) != 7'b0000000) ||
                            (($past(instr[14:12]) == 3'b101) && (($past(instr[31]) != 1'b0) || ($past(instr[29:25]) != 5'b00000)))));
                        assert($past(invalid_rs) == $past(instr[19]));
                        assert($past(invalid_rd) == $past(instr[11]));
                        assert(fault == ($past(invalid_funct7) || $past(invalid_rs) || $past(invalid_rd)));
                        if (!fault) begin
                            assert(has_rd_stage && has_ex_stage && !has_ma_stage);
                            assert(imm == {{20{$past(instr[31])}}, $past(instr[31:20])});
                            assert(alu_a_mux_position == 1'b0);
                            assert(alu_b_mux_position == 1'b1);
                            assert(wb_mux_position == 2'b00);
                            assert(rd_rf_microcode[7:4] == $past(instr[18:15]));
                            if ($past(instr[14:12]) == 3'b101) begin
                                assert(ex_alu_microcode == {2'b10, $past(instr[30]), $past(instr[14:12])});
                            end else begin
                                assert(ex_alu_microcode == {3'b100, $past(instr[14:12])});
                            end
                            assert(ex_csr_microcode[15] == 1'b0);
                            assert(ma_mem_microcode[4] == 1'b0);
                            assert(wb_rf_microcode == {1'b1, $past(instr[10:7])});
                            assert(wb_pc_mux_position == 2'b00);
                        end
                    end
                    7'b0110011: begin // OP
                        assert($past(opcode_is_op));
                        assert(!$past(invalid_opcode) && !$past(invalid_funct3));
                        assert($past(invalid_funct7) == ((($past(instr[31]) != 1'b0) || ($past(instr[29:25]) != 5'b00000)) ||
                            ($past(instr[30]) && ($past(instr[14:12]) != 3'b000) && ($past(instr[14:12]) != 3'b101))));
                        assert($past(invalid_rs) == ($past(instr[19]) || $past(instr[24])));
                        assert($past(invalid_rd) == $past(instr[11]));
                        assert(fault == ($past(invalid_funct7) || $past(invalid_rs) || $past(invalid_rd)));
                        if (!fault) begin
                            assert(has_rd_stage && has_ex_stage && !has_ma_stage);
                            assert(alu_a_mux_position == 1'b0);
                            assert(alu_b_mux_position == 1'b0);
                            assert(wb_mux_position == 2'b00);
                            assert(rd_rf_microcode[7:0] == {$past(instr[18:15]), $past(instr[23:20])});
                            assert(ex_alu_microcode == {2'b10, $past(instr[30]), $past(instr[14:12])});
                            assert(ex_csr_microcode[15] == 1'b0);
                            assert(ma_mem_microcode[4] == 1'b0);
                            assert(wb_rf_microcode == {1'b1, $past(instr[10:7])});
                            assert(wb_pc_mux_position == 2'b00);
                        end
                    end
                    7'b0001111: begin // FENCE
                        assert($past(opcode_is_fence));
                        assert(!$past(invalid_opcode));
                        assert($past(invalid_funct3) == ($past(instr[14:13]) != 2'b0));
                        if (~$past(instr[12])) begin
                            // FENCE
                            assert($past(invalid_funct7) == ($past(instr[31:28]) != 4'b0));
                            assert($past(invalid_rs) == ($past(instr[19:15]) != 5'b0));
                        end else begin
                            // FENCE.I
                            assert($past(invalid_funct7) == ($past(instr[31:25]) != 7'b0));
                            assert($past(invalid_rs) == ($past(instr[24:15]) != 10'b0));
                        end
                        assert($past(invalid_rd) == ($past(instr[11:7]) != 5'b0));
                        assert(fault == ($past(invalid_funct7) || $past(invalid_funct3) || $past(invalid_rs) || $past(invalid_rd)));
                        if (!fault) begin
                            assert(!has_rd_stage && !has_ex_stage && !has_ma_stage);
                            assert(ex_alu_microcode[5] == 1'b0);
                            assert(ex_csr_microcode[15] == 1'b0);
                            assert(ma_mem_microcode[4] == 1'b0);
                            assert(wb_rf_microcode[4] == 1'b0);
                            assert(wb_pc_mux_position == 2'b00);
                        end
                    end
                    7'b1110011: begin // SYSTEM
                        assert($past(opcode_is_system));
                        assert(!$past(invalid_opcode));
                        if (($past(instr) == 32'b00000000000000000000000001110011) || $past(instr) == 32'b00000000000100000000000001110011) begin
                            // ECALL / EBREAK
                            assert($past(instr_is_system_e_mret));
                            assert($past(instr_is_system_e));
                            assert(!$past(instr_is_system_mret));
                            assert(!$past(instr_is_system_csr));
                            assert(!$past(invalid_funct7) && !$past(invalid_funct3) && !$past(invalid_funct7) &&
                                !$past(invalid_rs) && !$past(invalid_rd));
                            assert(!fault);
                            assert(!has_rd_stage && has_ex_stage && !has_ma_stage);
                            assert(ex_alu_microcode[5] == 1'b0);
                            assert(ex_csr_microcode == {1'b1, 8'b00000000, ~$past(instr[20]), 3'b011, 3'b000});
                            assert(ma_mem_microcode[4] == 1'b0);
                            assert(wb_rf_microcode[4] == 1'b0);
                            assert(wb_pc_mux_position == 2'b01);
                            assert(csr_mux_position == 2'b10);
                        end else if ($past(instr) == 32'b00110000001000000000000001110011) begin
                            // MRET
                            assert($past(instr_is_system_e_mret));
                            assert(!$past(instr_is_system_e));
                            assert($past(instr_is_system_mret));
                            assert(!$past(instr_is_system_csr));
                            assert(!$past(invalid_funct7) && !$past(invalid_funct3) && !$past(invalid_funct7) &&
                                !$past(invalid_rs) && !$past(invalid_rd));
                            assert(!fault);
                            assert(!has_rd_stage && has_ex_stage && !has_ma_stage);
                            assert(ex_alu_microcode[5] == 1'b0);
                            assert(ex_csr_microcode[15] == 1'b1);
                            assert(ex_csr_microcode[2:0] == 3'b001);
                            assert(ma_mem_microcode[4] == 1'b0);
                            assert(wb_rf_microcode[4] == 1'b0);
                            assert(wb_pc_mux_position == 2'b01);
                        end else if ($past(instr[14:12]) != 3'b000) begin
                            // CSR
                            assert($past(invalid_funct3) == ($past(instr[13:12]) == 2'b0));
                            assert(!$past(invalid_funct7));
                            if ($past(instr[14])) begin
                                // immediate
                                assert(!$past(invalid_rs));
                            end else begin
                                // register
                                assert($past(invalid_rs) == $past(instr[19]));
                            end
                            assert($past(invalid_rd) == ($past(instr[11]) != 1'b0));
                            assert(fault == ($past(invalid_funct7) || $past(invalid_funct3) || $past(invalid_rs) || $past(invalid_rd)));
                            if (!fault) begin
                                assert(!$past(instr_is_system_e_mret));
                                assert(!$past(instr_is_system_e));
                                assert(!$past(instr_is_system_mret));
                                assert($past(instr_is_system_csr));
                                assert(has_ex_stage);
                                assert(!has_ma_stage);
                                assert(alu_a_mux_position == 1'b0);
                                if ($past(instr[14])) begin
                                    // immediate
                                    assert(!has_rd_stage);
                                    assert(csr_mux_position == 2'b01);
                                end else begin
                                    // register
                                    assert(has_rd_stage);
                                    assert(rd_rf_microcode[7:4] == $past(instr[18:15]));
                                    assert(csr_mux_position == 2'b00);
                                end
                                assert(ex_alu_microcode[5] == 1'b0);
                                assert(ex_csr_microcode == {1'b1, $past(instr[31:20]), 1'b1, $past(instr[13:12])});
                                assert(ma_mem_microcode[4] == 1'b0);
                                assert(wb_rf_microcode == {1'b1, $past(instr[10:7])});
                                assert(wb_pc_mux_position == 2'b00);
                                assert(wb_mux_position == 2'b00);
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
