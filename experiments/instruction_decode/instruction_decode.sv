/*
 * A decoder for the RV32I instruction set (FENCE/ECALL/EBREAK/CSR* instructions are excluded).
 */
module instruction_decode #(
)(
    input clk, // Clock signal
    input [31:0] instr, // Instruction to decode
    output [31:0] imm, // Immediate value
    output [1:0] alu_mux_position, // Positions of ALU muxes (bit0=A, bit1=B; 0=reg, 1=other)
    output [9:0] rd_rf_microcode, // Read stage register file microcode
    output [5:0] ex_alu_microcode, // Execute stage ALU microcode
    output [4:0] ma_mem_microcode, // Memory access stage memory microcode
    output [9:0] wb_rf_microcode, // Write back stage register file microcode
    output [1:0] wb_pc_microcode, // Write back stage program counter microcode
    output fault // Fault condition (i.e. invalid instruction)
);

    /* Outputs */
    reg [31:0] imm;
    reg [9:0] rd_rf_microcode;
    reg [5:0] ex_alu_microcode;
    reg [4:0] ma_mem_microcode;
    reg [9:0] wb_rf_microcode;
    reg [1:0] wb_pc_microcode;
    reg [1:0] alu_mux_position;
    reg fault;

    /* Basic Decode */
    wire [6:0] opcode = instr[6:0];
    wire [4:0] rd = instr[11:7];
    wire [2:0] funct3 = instr[14:12];
    wire [4:0] rs1 = instr[19:15];
    wire [4:0] rs2 = instr[24:20];
    wire [6:0] funct7 = instr[31:25];
    wire opcode_is_op = opcode[5] & opcode[4] & ~opcode[2];
    wire opcode_is_opimm = ~opcode[5] & opcode[4] & ~opcode[2];
    wire opcode_is_load = ~opcode[5] & ~opcode[4];
    wire opcode_is_store = ~opcode[6] & opcode[5] & ~opcode[4];
    wire opcode_is_auipc = ~opcode[5] & opcode[2];
    wire opcode_is_branch = opcode[6] & ~opcode[2];
    wire opcode_is_jal = opcode[3];
    wire opcode_is_jalr = ~opcode[4] & ~opcode[3] & opcode[2];
    wire opcode_is_lui = ~opcode[6] & opcode[5] & opcode[2];
    wire invalid_opcode = ~opcode[0] | ~opcode[1] |
        (~opcode[6] & opcode[3]) |
        (opcode[3] & ~opcode[2]) |
        (opcode[6] & ~opcode[5]) |
        (opcode[6] & opcode[4]) |
        (~opcode[6] & ~opcode[4] & opcode[2]);

    /* Read Stage */
    wire rd_rf_enable = ~(opcode_is_lui | opcode_is_auipc | opcode_is_jal);
    wire invalid_rs = rd_rf_enable & (rs1[4] | ((opcode_is_op | opcode_is_store | opcode_is_branch) & rs2[4]));
    always @(posedge clk) begin
        rd_rf_microcode <= {
            rd_rf_enable,
            1'b0,
            rs1[3:0],
            rs2[3:0]
        };
    end

    /* Execute Stage */
    // op_is_jalr, op_is_branch, op_is_load, op_is_store, funct3[2], funct3[1], funct3[0]
    wire invalid_funct3 = (opcode_is_store & funct3[2]) |
        (opcode_is_jalr & funct3[0]) |
        (opcode_is_jalr & funct3[1]) |
        (opcode_is_jalr & funct3[2]) |
        (opcode_is_store & funct3[1] & funct3[0]) |
        (opcode_is_load & funct3[1] & funct3[0]) |
        (opcode_is_load & funct3[2] & funct3[1]) |
        (opcode_is_branch & ~funct3[2] & funct3[1]);
    wire funct7_non_funct_bit_set = funct7[6] | funct7[4] | funct7[3] | funct7[2] | funct7[1] | funct7[0];
    wire invalid_funct7 = (opcode_is_op & funct7_non_funct_bit_set) |
        (opcode_is_op & funct3[1] & funct7[5]) |
        (opcode_is_opimm & ~funct3[1] & funct3[0] & funct7_non_funct_bit_set) |
        (opcode_is_op & ~funct3[2] & funct3[0] & funct7[5]) |
        (opcode_is_op & funct3[2] & ~funct3[0] & funct7[5]) |
        (opcode_is_opimm & ~funct3[2] & ~funct3[1] & funct3[0] & funct7[5]);
    always @(posedge clk) begin
        ex_alu_microcode <= {
            (opcode_is_op | opcode_is_opimm | opcode_is_branch | opcode_is_load | opcode_is_store | opcode_is_auipc |
                opcode_is_jalr),
            opcode_is_branch,
            funct7[5] & ~funct3[1] & (opcode_is_op | opcode_is_opimm) & (opcode_is_op | funct3[2]) &
                (funct3[2] | ~funct3[0]) & (~funct3[2] | funct3[0]),
            ({3{opcode_is_op | opcode_is_opimm | opcode_is_branch}} & funct3)
        };
    end

    /* Memory Stage */
    wire ma_mem_enable = opcode_is_load | opcode_is_store;
    always @(posedge clk) begin
        ma_mem_microcode <= {
            ma_mem_enable,
            opcode_is_store,
            funct3
        };
    end

    /* Write Back Stage */
    wire wb_rf_enable = ~(opcode_is_store | opcode_is_branch);
    wire invalid_rd = wb_rf_enable & rd[4];
    always @(posedge clk) begin
        wb_rf_microcode <= {
            wb_rf_enable,
            1'b1,
            rd[3:0],
            4'b0
        };
        wb_pc_microcode <= {
            (opcode_is_branch | opcode_is_jal),
            (opcode_is_jalr | opcode_is_branch)
        };
    end

    /* Immediate and Fault Signals */
    wire [19:0] imm_lower_jal = {rs1, funct3, rs2[0], funct7[5:0], rs2 & 5'b11110};
    wire [19:0] imm_lower_non_jal = {
        {8{funct7[6]}},
        opcode_is_branch ? {rd[0], funct7[5:0]} : funct7,
        (opcode_is_branch | opcode_is_store) ? rd & {4'b1111, ~opcode_is_branch} : rs2
    };
    always @(posedge clk) begin
        fault <= invalid_opcode | invalid_rs | invalid_funct7 | invalid_funct3 | invalid_rd;
        if (opcode_is_lui | opcode_is_auipc)
            imm <= {funct7, rs2, rs1, funct3, 12'b0};
        else
            imm <= {{12{funct7[6]}}, opcode_is_jal ? imm_lower_jal : imm_lower_non_jal};
        alu_mux_position <= 2'b00;
    end

/*
    PC opcodes
    00: pc <- npc
    01: pc <- I[31:1]
    10: pc <- pc + offset
    11: pc <- Y ? I + offset : npc
*/
/*
    00000 // 1 - LOAD
    00100 // 1 - OP-IMM
    00101 // 1 - AUIPC
    01000 // 1 - STORE
    01100 // 1 - OP
    01101 // 0 - LUI
    11000 // 1 - BRANCH
    11001 // 1 - JALR
    11011 // 0 - JAL
*/
/*
    7'b0110111 // LUI
    7'b0010111 // AUIPC
    7'b1101111 // JAL
    7'b1100111 // JALR
    7'b1100011 // BRANCH
    7'b0000011 // LOAD
    7'b0100011 // STORE
    7'b0010011 // OP-IMM
    7'b0110011 // OP
*/

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
                opcode_is_branch + opcode_is_jal + opcode_is_jalr + opcode_is_lui) == 1);
        end
    end
    always @(posedge clk) begin
    	if (f_past_valid) begin
            case ($past(instr[6:0]))
                7'b0110111: begin // LUI
                    assert($past(opcode_is_lui));
                    assert(!$past(invalid_opcode) && !$past(invalid_funct7) && !$past(invalid_funct3) && !$past(invalid_rs));
                    assert($past(invalid_rd) == $past(instr[11]));
                    assert(fault == $past(invalid_rd));
                    if (!fault) begin
                        assert(imm == {$past(instr[31:12]), 12'b0});
                        assert(rd_rf_microcode[9] == 1'b0);
                        assert(ex_alu_microcode[5] == 1'b0);
                        assert(ma_mem_microcode[4] == 1'b0);
                        assert(wb_rf_microcode == {2'b11, $past(instr[10:7]), 4'b0});
                        assert(wb_pc_microcode == 2'b00);
                    end
                end
                7'b0010111: begin // AUIPC
                    assert($past(opcode_is_auipc));
                    assert(!$past(invalid_opcode) && !$past(invalid_funct7) && !$past(invalid_funct3) && !$past(invalid_rs));
                    assert($past(invalid_rd) == $past(instr[11]));
                    assert(fault == $past(invalid_rd));
                    if (!fault) begin
                        assert(imm == {$past(instr[31:12]), 12'b0});
                        assert(rd_rf_microcode[9] == 1'b0);
                        assert(ex_alu_microcode == 6'b100000);
                        assert(ma_mem_microcode[4] == 1'b0);
                        assert(wb_rf_microcode == {2'b11, $past(instr[10:7]), 4'b0});
                        assert(wb_pc_microcode == 2'b00);
                    end
                end
                7'b1101111: begin // JAL
                    assert($past(opcode_is_jal));
                    assert(!$past(invalid_opcode) && !$past(invalid_funct7) && !$past(invalid_funct3) && !$past(invalid_rs));
                    assert($past(invalid_rd) == $past(instr[11]));
                    assert(fault == $past(invalid_rd));
                    if (!fault) begin
                        assert(imm == {{12{$past(instr[31])}}, $past(instr[19:12]), $past(instr[20]), $past(instr[30:21]), 1'b0});
                        assert(rd_rf_microcode[9] == 1'b0);
                        assert(ex_alu_microcode[5] == 1'b0);
                        assert(ma_mem_microcode[4] == 1'b0);
                        assert(wb_rf_microcode == {2'b11, $past(instr[10:7]), 4'b0});
                        assert(wb_pc_microcode == 2'b10);
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
                        assert(imm == {{20{$past(instr[31])}}, $past(instr[31:20])});
                        assert(rd_rf_microcode[9:4] == {2'b10, $past(instr[18:15])});
                        assert(ex_alu_microcode == 6'b100000);
                        assert(ma_mem_microcode[4] == 1'b0);
                        assert(wb_rf_microcode == {2'b11, $past(instr[10:7]), 4'b0});
                        assert(wb_pc_microcode == 2'b01);
                    end
                end
                7'b1100011: begin // BRANCH
                    assert($past(opcode_is_branch));
                    assert(!$past(invalid_opcode) && !$past(invalid_funct7) && !$past(invalid_rd));
                    assert($past(invalid_rs) == ($past(instr[24]) || $past(instr[19])));
                    assert($past(invalid_funct3) == ($past(instr[14:13]) == 2'b01));
                    assert(fault == ($past(invalid_rs) || $past(invalid_funct3)));
                    if (!fault) begin
                        assert(imm == {{20{$past(instr[31])}}, $past(instr[7]), $past(instr[30:25]), $past(instr[11:8]), 1'b0});
                        assert(rd_rf_microcode == {2'b10, $past(instr[18:15]), $past(instr[23:20])});
                        assert(ex_alu_microcode == {3'b110, $past(instr[14:12])});
                        assert(ma_mem_microcode[4] == 1'b0);
                        assert(wb_rf_microcode[9] == 1'b0);
                        assert(wb_pc_microcode == 2'b11);
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
                        assert(imm == {{20{$past(instr[31])}}, $past(instr[31:20])});
                        assert(rd_rf_microcode[9:4] == {2'b10, $past(instr[18:15])});
                        assert(ex_alu_microcode == 6'b100000);
                        assert(ma_mem_microcode == {2'b10, $past(instr[14:12])});
                        assert(wb_rf_microcode == {2'b11, $past(instr[10:7]), 4'b0});
                        assert(wb_pc_microcode == 2'b00);
                    end
                end
                7'b0100011: begin // STORE
                    assert($past(opcode_is_store));
                    assert(!$past(invalid_opcode) && !$past(invalid_funct7) && !$past(invalid_rd));
                    assert($past(invalid_rs) == ($past(instr[24]) || $past(instr[19])));
                    assert($past(invalid_funct3) == ($past(instr[14]) || ($past(instr[13:12]) == 2'b11)));
                    assert(fault == $past(invalid_rs) || $past(invalid_funct3));
                    if (!fault) begin
                        assert(imm == {{20{$past(instr[31])}}, $past(instr[31:25]), $past(instr[11:7])});
                        assert(rd_rf_microcode == {2'b10, $past(instr[18:15]), $past(instr[23:20])});
                        assert(ex_alu_microcode == 6'b100000);
                        assert(ma_mem_microcode == {2'b11, $past(instr[14:12])});
                        assert(wb_rf_microcode[9] == 1'b0);
                        assert(wb_pc_microcode == 2'b00);
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
                        assert(imm == {{20{$past(instr[31])}}, $past(instr[31:20])});
                        assert(rd_rf_microcode[9:4] == {2'b10, $past(instr[18:15])});
                        if ($past(instr[14:12]) == 3'b101) begin
                            assert(ex_alu_microcode == {2'b10, $past(instr[30]), $past(instr[14:12])});
                        end else begin
                            assert(ex_alu_microcode == {3'b100, $past(instr[14:12])});
                        end
                        assert(ma_mem_microcode[4] == 1'b0);
                        assert(wb_rf_microcode == {2'b11, $past(instr[10:7]), 4'b0});
                        assert(wb_pc_microcode == 2'b00);
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
                        assert(rd_rf_microcode == {2'b10, $past(instr[18:15]), $past(instr[23:20])});
                        assert(ex_alu_microcode == {2'b10, $past(instr[30]), $past(instr[14:12])});
                        assert(ma_mem_microcode[4] == 1'b0);
                        assert(wb_rf_microcode == {2'b11, $past(instr[10:7]), 4'b0});
                        assert(wb_pc_microcode == 2'b00);
                    end
                end
                default: begin
                    assert($past(invalid_opcode));
                end
            endcase
        end
    end
`endif

endmodule
