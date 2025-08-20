`include "adder32.sv"
`include "shifter.sv"
`include "constants/alu_microcode.sv"
`include "cells/and2.sv"
`include "cells/nand8.sv"
`include "cells/dffe.sv"
`include "cells/or2.sv"
`include "cells/or4.sv"
`include "cells/mux2.sv"
`include "cells/mux8.sv"
`include "cells/nor4.sv"
`include "cells/xor2.sv"

/*
 * An ALU which can perform all the math / branch operations (opcodes 0b0010011, 0b0110011, and 1100011) of the RV31I/E instruction set.
 */
module alu(
    input [8:0] microcode, // Microcode
    input [31:0] in_a, // Input data (bus A)
    input [31:0] in_b, // Input data (bus B)
    output [31:0] out // Output data
);

    /* Bitwise operations */
    wire [31:0] xor_result;
    xor2 #(.BITS(32)) xor_result_xor(
        .a(in_a),
        .b(in_b),
        .out(xor_result)
    );
    wire [31:0] or_result;
    or2 #(.BITS(32)) or_result_or(
        .a(in_a),
        .b(in_b),
        .out(or_result)
    );
    wire [31:0] and_result;
    and2 #(.BITS(32)) and_result_and(
        .a(in_a),
        .b(in_b),
        .out(and_result)
    );

    /* Adder */
    wire [31:0] adder_in_b;
    xor2 #(.BITS(32)) adder_in_b_xor(
        .a(in_b),
        .b({32{microcode[5]}}),
        .out(adder_in_b)
    );
    wire [31:0] adder_sum;
    adder32 adder_module(
        .a(in_a),
        .b(adder_in_b),
        .carry_in(microcode[5]),
        .sum(adder_sum)
    );

    /* Shifter */
    wire [31:0] shift_result;
    shifter shifter_module(
        .in(in_a),
        .amount(in_b[4:0]),
        .op(microcode[4:3]),
        .out(shift_result)
    );

    /* Equality */
    wire [7:0] xor_result_nibble_is_zero;
    nor4 #(.BITS(8)) xor_result_nibble_is_zero_nor(
        .a(xor_result[7:0]),
        .b(xor_result[15:8]),
        .c(xor_result[23:16]),
        .d(xor_result[31:24]),
        .out(xor_result_nibble_is_zero)
    );
    wire eq_result_n;
    nand8 eq_result_nand(
        .a(xor_result_nibble_is_zero[0]),
        .b(xor_result_nibble_is_zero[1]),
        .c(xor_result_nibble_is_zero[2]),
        .d(xor_result_nibble_is_zero[3]),
        .e(xor_result_nibble_is_zero[4]),
        .f(xor_result_nibble_is_zero[5]),
        .g(xor_result_nibble_is_zero[6]),
        .h(xor_result_nibble_is_zero[7]),
        .out(eq_result_n)
    );

    /* Branch / SLT / SLTU */
    wire lt_ltu_result = ((in_a[31] ^ microcode[6]) & ~(in_b[31] ^ microcode[6])) | (~(in_a[31] ^ in_b[31]) & adder_sum[31] & eq_result_n);
    wire eq_lt_ltu_result;
    mux2 eq_lt_ltu_result_mux(
        .a(lt_ltu_result),
        .b(~eq_result_n),
        .select(microcode[8]),
        .out(eq_lt_ltu_result)
    );
    wire [31:0] branch_slt_sltu_result = {31'b0, eq_lt_ltu_result ^ microcode[7]};

    /* Output */
    mux8 #(.BITS(32)) out_mux(
        .d1(adder_sum),
        .d2(shift_result),
        .d3(branch_slt_sltu_result),
        .d4(branch_slt_sltu_result),
        .d5(xor_result),
        .d6(shift_result),
        .d7(or_result),
        .d8(and_result),
        .select(microcode[2:0]),
        .out(out)
    );

`ifdef FORMAL
    /* Validate logic */
    always_comb begin
        case (microcode)
            `ALU_MICROCODE_ADD:
                assert(out == (in_a + in_b));
            `ALU_MICROCODE_SLL:
                assert(out == (in_a << in_b[4:0]));
            `ALU_MICROCODE_SLT:
                assert(out == (($signed(in_a) < $signed(in_b)) ? 1 : 0));
            `ALU_MICROCODE_SLTU:
                assert(out == ((in_a < in_b) ? 1 : 0));
            `ALU_MICROCODE_XOR:
                assert(out == (in_a ^ in_b));
            `ALU_MICROCODE_SRL:
                assert(out == (in_a >> in_b[4:0]));
            `ALU_MICROCODE_OR:
                assert(out == (in_a | in_b));
            `ALU_MICROCODE_AND:
                assert(out == (in_a & in_b));
            `ALU_MICROCODE_SUB:
                assert(out == (in_a - in_b));
            `ALU_MICROCODE_SRA:
                assert($signed(out) == ($signed(in_a) >>> in_b[4:0]));
            `ALU_MICROCODE_BEQ:
                assert(out == ((in_a == in_b) ? 1'b1 : 1'b0));
            `ALU_MICROCODE_BNE:
                assert(out == ((in_a != in_b) ? 1'b1 : 1'b0));
            `ALU_MICROCODE_BLT:
                assert(out == (($signed(in_a) < $signed(in_b)) ? 1'b1 : 1'b0));
            `ALU_MICROCODE_BGE:
                assert(out == (($signed(in_a) >= $signed(in_b)) ? 1'b1 : 1'b0));
            `ALU_MICROCODE_BLTU:
                assert(out == ((in_a < in_b) ? 1'b1 : 1'b0));
            `ALU_MICROCODE_BGEU:
                assert(out == ((in_a >= in_b) ? 1'b1 : 1'b0));
        endcase
    end
`endif

endmodule
