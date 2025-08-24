`include "cells/and2.sv"
`include "cells/mux2.sv"
`include "cells/xor2.sv"

/*
 * A 4 bit full adder.
 */
module adder4(
    input [3:0] a, // Input A
    input [3:0] b, // Input B
    input carry_in, // Carry in
    output [3:0] sum, // Result
    output carry_out // Carry out
);

    wire [3:0] prop;
    xor2 #(.BITS($bits(prop))) prop_xor(
        .a(a),
        .b(b),
        .out(prop)
    );
    wire [3:0] gen;
    and2 #(.BITS($bits(gen))) gen_and(
        .a(a),
        .b(b),
        .out(gen)
    );

    wire all_prop = prop[0] & prop[1] & prop[2] & prop[3];
    wire carry_0 = (prop[0] & carry_in) | gen[0];
    wire carry_1 = (prop[1] & carry_0) | gen[1];
    wire carry_2 = (prop[2] & carry_1) | gen[2];
    mux2 carry_out_mux(
        .a((prop[3] & carry_2) | gen[3]),
        .b(carry_in),
        .select(all_prop),
        .out(carry_out)
    );

    xor2 #(.BITS($bits(sum))) sum_xor(
        .a(prop),
        .b({carry_2, carry_1, carry_0, carry_in}),
        .out(sum)
    );

`ifdef FORMAL
    /* Validate logic */
    always_comb begin
        assert(sum == (a + b + carry_in));
    end
`endif

endmodule
