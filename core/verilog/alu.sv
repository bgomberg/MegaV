`include "adder32.sv"
`include "shifter.sv"
`include "cells/and2.sv"
`include "cells/dffe.sv"
`include "cells/or2.sv"
`include "cells/nor4.sv"
`include "cells/xor2.sv"

/*
 * An ALU which can perform all the math operations (opcodes 0b0010011 and 0b0110011) of the RV31I/E instruction set.
 */
module alu(
    input logic clk, // Clock signal
    input logic reset_n, // Reset signal (active low)
    input logic enable_n, // Enable (active low)
    input logic [4:0] op, // Operation to be performed
    input logic [31:0] in_a, // Input data (bus A)
    input logic [31:0] in_b, // Input data (bus B)
    output logic [31:0] out, // Output data
    output logic fault // Invalid operation
);

    /* Op decoding */
    wire op_is_add = (op == 5'b00000);
    wire op_shift_is_right = op[2];
    wire op_shift_is_right_arithmetic = op_shift_is_right & op[3];
    wire op_is_branch_eq_ne = op[4] & ~op[2];
    wire op_is_branch_gt = op[4] & op[0];
    wire op_branch_slt_is_unsigned = (~op[4] & op[0]) | (op[2] & op[1]);
    wire op_is_arith = ~op[4] & ~op[2] & ~op[1] & ~op[0];
    wire op_is_xor = ~op[4] & op[2] & ~op[1] & ~op[0];
    wire op_is_or = ~op[4] & op[2] & op[1] & ~op[0];
    wire op_is_and = ~op[4] & op[2] & op[1] & op[0];
    wire op_is_shift = ~op[4] & ~op[1] & op[0];
    wire is_invalid_op = (op[3] & op[1]) | (op[4] & op[3]) | (op[3] & ~op[2] & op[0]) | (op[3] & op[2] & ~op[0]) | (op[4] & ~op[2] & op[1]);

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
        .b({32{~op_is_add}}),
        .out(adder_in_b)
    );
    wire adder_carry_in = ~op_is_add;
    wire [31:0] adder_sum;
    adder32 adder_module(
        in_a,
        adder_in_b,
        adder_carry_in,
        adder_sum
    );

    /* Shifter */
    wire [31:0] shift_result;
    shifter shifter_module(
        in_a,
        in_b[4:0],
        op_shift_is_right,
        op_shift_is_right_arithmetic,
        shift_result
    );

    /* Logic */
    wire [7:0] xor_result_nibble_is_zero;
    nor4 #(.BITS(8)) xor_result_nibble_is_zero_nor(
        .a(xor_result[7:0]),
        .b(xor_result[15:8]),
        .c(xor_result[23:16]),
        .d(xor_result[31:24]),
        .out(xor_result_nibble_is_zero)
    );
    wire eq_result = (xor_result_nibble_is_zero == 8'b11111111);
    wire sign_cmp_result = (in_a[31] ^ op_branch_slt_is_unsigned) & ~(in_b[31] ^ op_branch_slt_is_unsigned);
    wire lt_ltu_result = sign_cmp_result | (~(in_a[31] ^ in_b[31]) & adder_sum[31] & ~eq_result);
    wire branch_slt_sltu_result = (op_is_branch_eq_ne ? eq_result : lt_ltu_result) ^ op_is_branch_gt;

    /* Result */
    wire op_is_arith_xor = op_is_arith | op_is_xor;
    wire [31:0] result_arith_xor;
    mux2 #(.BITS(32)) result_arith_xor_mux(
        .a(adder_sum),
        .b(xor_result),
        .select(op_is_xor),
        .out(result_arith_xor)
    );
    wire op_is_and_or = op_is_and | op_is_or;
    wire [31:0] result_and_or;
    mux2 #(.BITS(32)) result_and_or_mux(
        .a(and_result),
        .b(or_result),
        .select(op_is_or),
        .out(result_and_or)
    );
    wire op_is_arith_xor_and_or = op_is_arith_xor | op_is_and_or;
    wire [31:0] result_arith_xor_and_or;
    mux2 #(.BITS(32)) result_arith_xor_and_or_mux(
        .a(result_arith_xor),
        .b(result_and_or),
        .select(op_is_and_or),
        .out(result_arith_xor_and_or)
    );
    wire [31:0] result_shift_cmp;
    mux2 #(.BITS(32)) result_shift_cmp_mux(
        .a({31'b0, branch_slt_sltu_result}),
        .b(shift_result),
        .select(op_is_shift),
        .out(result_shift_cmp)
    );
    wire [31:0] result;
    mux2 #(.BITS(32)) result_mux(
        .a(result_shift_cmp),
        .b(result_arith_xor_and_or),
        .select(op_is_arith_xor_and_or),
        .out(result)
    );
    dffe #(.BITS(32)) out_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(result),
        .out(out)
    );
    dffe fault_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(is_invalid_op),
        .out(fault)
    );

`ifdef FORMAL
    initial assume(~reset_n);
    logic f_past_valid;
    initial f_past_valid = 0;
    always_ff @(posedge clk) begin
        f_past_valid = 1;
    end

    /* Validate logic */
    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            if ($past(~reset_n)) begin
                assert(!fault);
            end else begin
                assume($past(~enable_n));
                case ($past(op))
                    5'b0000: begin // ADD
                        assert(!fault);
                        assert(out == ($past(in_a) + $past(in_b)));
                    end
                    5'b00001: begin // SLL
                        assert(!fault);
                        assert(out == ($past(in_a) << $past(in_b[4:0])));
                    end
                    5'b00010: begin // SLT
                        assert(!fault);
                        assert(out == (($signed($past(in_a)) < $signed($past(in_b))) ? 1 : 0));
                    end
                    5'b00011: begin // SLTU
                        assert(!fault);
                        assert(out == (($past(in_a) < $past(in_b)) ? 1 : 0));
                    end
                    5'b00100: begin // XOR
                        assert(!fault);
                        assert(out == ($past(in_a) ^ $past(in_b)));
                    end
                    5'b00101: begin // SRL
                        assert(!fault);
                        assert(out == ($past(in_a) >> $past(in_b[4:0])));
                    end
                    5'b00110: begin // OR
                        assert(!fault);
                        assert(out == ($past(in_a) | $past(in_b)));
                    end
                    5'b00111: begin // AND
                        assert(!fault);
                        assert(out == ($past(in_a) & $past(in_b)));
                    end
                    5'b01000: begin // SUB
                        assert(!fault);
                        assert(out == ($past(in_a) - $past(in_b)));
                    end
                    5'b01101: begin // SRA
                        assert(!fault);
                        assert($signed(out) == ($signed($past(in_a)) >>> $past(in_b[4:0])));
                    end
                    5'b10000: begin // BEQ
                        assert(!fault);
                        assert(out == (($past(in_a) == $past(in_b)) ? 1'b1 : 1'b0));
                    end
                    5'b10001: begin // BNE
                        assert(!fault);
                        assert(out == (($past(in_a) != $past(in_b)) ? 1'b1 : 1'b0));
                    end
                    5'b10100: begin // BLT
                        assert(!fault);
                        assert(out == (($signed($past(in_a)) < $signed($past(in_b))) ? 1'b1 : 1'b0));
                    end
                    5'b10101: begin // BGE
                        assert(!fault);
                        assert(out == (($signed($past(in_a)) >= $signed($past(in_b))) ? 1'b1 : 1'b0));
                    end
                    5'b10110: begin // BLTU
                        assert(!fault);
                        assert(out == (($past(in_a) < $past(in_b)) ? 1'b1 : 1'b0));
                    end
                    5'b10111: begin // BGEU
                        assert(!fault);
                        assert(out == (($past(in_a) >= $past(in_b)) ? 1'b1 : 1'b0));
                    end
                    default: begin
                        assert(fault);
                    end
                endcase
            end
        end
    end
`endif

endmodule
