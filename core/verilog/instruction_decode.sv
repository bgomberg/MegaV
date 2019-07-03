`define STATE_IDLE          2'b00
`define STATE_IN_PROGRESS   2'b01
`define STATE_DONE          2'b10

/*
 * A decoder for the RV32I instruction set.
 */
module instruction_decode(
    input clk, // Clock signal
    input reset, // Reset signal
    input available, // Operation available
    input [31:0] instr, // Instruction to decode
    output [31:0] imm, // Immediate value
    output alu_a_mux_position, // Position of the ALU A mux (0=rs1, 1=pc)
    output alu_b_mux_position, // Position of the ALU B mux (0=rs2, 1=imm)
    output [1:0] csr_mux_position, // Position of the CSR mux (0=rs1, 1=imm, 2=npc, 3=UNUSED)
    output [1:0] wb_mux_position, // Position of the write back mux (0=alu_res, 1=imm, 2=mem_data, 3=npc)
    /* verilator lint_off UNOPTFLAT */
    output [8:0] rd_rf_microcode, // Read stage register file microcode
    /* verilator lint_on UNOPTFLAT */
    output [5:0] ex_alu_microcode, // Execute stage ALU microcode
    output [4:0] ma_mem_microcode, // Memory access stage memory microcode
    output [15:0] ex_csr_microcode, // Execute stage CSR microcode
    output [9:0] wb_rf_microcode, // Write back stage register file microcode
    output [1:0] wb_pc_mux_position, // Write back stage program counter microcode (00=npc, 01=I[31:1], 10=pc+offset, 11=Y?I+offset:npc)
    output busy, // Operation busy
    output fault // Fault condition (i.e. invalid instruction)
);

    /* Outputs */
    reg [31:0] imm;
    reg alu_a_mux_position;
    reg alu_b_mux_position;
    reg [1:0] csr_mux_position;
    reg [1:0] wb_mux_position;
    reg [8:0] rd_rf_microcode;
    reg [5:0] ex_alu_microcode;
    reg [4:0] ma_mem_microcode;
    reg [15:0] ex_csr_microcode;
    reg [9:0] wb_rf_microcode;
    reg [1:0] wb_pc_mux_position;
    reg busy;
    reg fault;

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

    /* Read Stage Decode */
    wire rd_rf_enable = ~(opcode_is_lui | opcode_is_auipc | opcode_is_jal | opcode_is_fence | opcode_is_system) |
        (instr_is_system_csr & ~funct3[2]);
    wire rs1_is_zero = ~rs1[4] & ~rs1[3] & ~rs1[2] & ~rs1[1] & ~rs1[0];
    wire invalid_rs = (rd_rf_enable & (rs1[4] | ((opcode_is_op | opcode_is_store | opcode_is_branch) & rs2[4]))) |
        (opcode_is_fence & (~rs1_is_zero | (funct3[0] & (rs2 != 5'b0)))) |
        (instr_is_system_e & (~rs1_is_zero | rs2[4] | rs2[3] | rs2[2] | rs2[1])) |
        (instr_is_system_mret & (~rs1_is_zero | rs2[4] | rs2[3] | rs2[2] | ~rs2[1] | rs2[0]));

    /* Execute Stage Decode */
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

    /* Write Back Stage Decode */
    wire wb_rf_enable = ~(opcode_is_store | opcode_is_branch | opcode_is_fence | instr_is_system_e_mret);
    wire rd_is_zero = ~rd[4] & ~rd[3] & ~rd[2] & ~rd[1] & ~rd[0];
    wire invalid_rd = (wb_rf_enable & rd[4]) | ((opcode_is_fence | instr_is_system_e_mret) & ~rd_is_zero);

    /* Immediate Signals Decode */
    wire [19:0] imm_lower_jal = {rs1, funct3, rs2[0], funct7[5:0], rs2 & 5'b11110};
    wire [19:0] imm_lower_non_jal = {
        {8{funct7[6]}},
        opcode_is_branch ? {rd[0], funct7[5:0]} : funct7,
        (opcode_is_branch | opcode_is_store) ? rd & {4'b1111, ~opcode_is_branch} : rs2
    };

    reg [1:0] state;
    wire invalid_instr = invalid_opcode | invalid_rs | invalid_funct7 | invalid_funct3 | invalid_rd;
    always @(posedge clk) begin
        if (reset) begin
            busy <= 1'b0;
            state <= `STATE_IDLE;
            fault <= 1'b0;
        end else begin
            case (state)
                `STATE_IDLE: begin
                    if (available) begin
                        // Starting a new operation
                        state <= `STATE_IN_PROGRESS;
                        busy <= 1'b1;
                    end else begin
                        fault <= 1'b0;
                        busy <= 1'b0;
                    end
                end
                `STATE_IN_PROGRESS: begin
                    /* Read Stage */
                    rd_rf_microcode <= {
                        ~invalid_instr & rd_rf_enable,
                        rs1[3:0],
                        rs2[3:0]
                    };

                    /* Execute Stage */
                    ex_alu_microcode <= {
                        ~invalid_instr & (opcode_is_op | opcode_is_opimm | opcode_is_branch | opcode_is_load | opcode_is_store | opcode_is_auipc |
                            opcode_is_jalr),
                        opcode_is_branch,
                        funct7[5] & ~funct3[1] & (opcode_is_op | opcode_is_opimm) & (opcode_is_op | funct3[2]) &
                            (funct3[2] | ~funct3[0]) & (~funct3[2] | funct3[0]),
                        ({3{opcode_is_op | opcode_is_opimm | opcode_is_branch}} & funct3)
                    };
                    ex_csr_microcode <= {
                        ~invalid_instr & opcode_is_system,
                        instr_is_system_e ? {8'b0, ~rs2[0], 3'b011} : {funct7, rs2},
                        instr_is_system_csr,
                        funct3[1],
                        funct3[0] | instr_is_system_mret
                    };

                    /* Memory Stage */
                    ma_mem_microcode <= {
                        ~invalid_instr & (opcode_is_load | opcode_is_store),
                        opcode_is_store,
                        funct3
                    };

                    /* Write Back Stage */
                    wb_rf_microcode <= {
                        ~invalid_instr & wb_rf_enable,
                        1'b1,
                        rd[3:0],
                        4'b0
                    };

                    /* Immediate Signals */
                    if (opcode_is_lui | opcode_is_auipc) begin
                        imm <= {funct7, rs2, rs1, funct3, 12'b0};
                    end else if (opcode_is_system) begin
                        imm <= {27'b0, rs1};
                    end else begin
                        imm <= {{12{funct7[6]}}, opcode_is_jal ? imm_lower_jal : imm_lower_non_jal};
                    end

                    /* Fault Signal */
                    fault <= invalid_instr;

                    /* Mux Position Signals */
                    alu_a_mux_position <= instr_is_system_e | opcode_is_auipc;
                    alu_b_mux_position <= ~opcode_is_branch & ~opcode_is_op;
                    csr_mux_position <= {instr_is_system_e, instr_is_system_csr & funct3[2]};
                    wb_mux_position <= {
                        (opcode_is_load | opcode_is_jalr | opcode_is_jal),
                        (opcode_is_lui | opcode_is_jalr | opcode_is_jal)
                    };
                    wb_pc_mux_position <= {
                        (opcode_is_branch | opcode_is_jal),
                        (opcode_is_jalr | opcode_is_branch | instr_is_system_e_mret)
                    };

                    // operation is complete
                    busy <= 1'b0;
                    state <= `STATE_DONE;
                end
                `STATE_DONE: begin
                    if (!available) begin
                        state <= `STATE_IDLE;
                    end
                end
                default: begin
                    // should never get here
                end
            endcase
        end
    end

`ifdef FORMAL
    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) begin
        f_past_valid = 1;
    end

    /* Validate Logic */
    always @(*) begin
        if (!invalid_opcode) begin
            assert((opcode_is_op + opcode_is_opimm + opcode_is_load + opcode_is_store + opcode_is_auipc +
                opcode_is_branch + opcode_is_jal + opcode_is_jalr + opcode_is_lui + opcode_is_fence +
                opcode_is_system) == 1);
        end
    end
    wire has_rd_stage = rd_rf_microcode[8];
    wire has_ex_stage = ex_alu_microcode[5] | ex_csr_microcode[15];
    wire has_ma_stage = ma_mem_microcode[4];
    always @(posedge clk) begin
        if (f_past_valid) begin
            assume(state != 2'b11);
            if ($past(reset)) begin
                assert(!busy);
                assert(!fault);
            end else if (state == `STATE_DONE && $past(state) == `STATE_IN_PROGRESS) begin
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
                            assert(wb_rf_microcode == {2'b11, $past(instr[10:7]), 4'b0});
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
                            assert(wb_rf_microcode == {2'b11, $past(instr[10:7]), 4'b0});
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
                            assert(wb_rf_microcode == {2'b11, $past(instr[10:7]), 4'b0});
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
                            assert(wb_rf_microcode == {2'b11, $past(instr[10:7]), 4'b0});
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
                            assert(wb_rf_microcode[9] == 1'b0);
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
                            assert(wb_rf_microcode == {2'b11, $past(instr[10:7]), 4'b0});
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
                            assert(wb_rf_microcode[9] == 1'b0);
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
                            assert(wb_rf_microcode == {2'b11, $past(instr[10:7]), 4'b0});
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
                            assert(wb_rf_microcode == {2'b11, $past(instr[10:7]), 4'b0});
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
                            assert(wb_rf_microcode[9] == 1'b0);
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
                            assert(wb_rf_microcode[9] == 1'b0);
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
                            assert(wb_rf_microcode[9] == 1'b0);
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
                                assert(wb_rf_microcode == {2'b11, $past(instr[10:7]), 4'b0});
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
