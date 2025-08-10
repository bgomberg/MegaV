`ifndef __ADDER32_SV__
`define __ADDER32_SV__

`include "adder4.sv"

/*
 * A 32 bit full adder.
 */
module adder32(
    input [31:0] prop, // Propagate signal (a ^ b)
    input [31:0] gen, // Generate signal (a & b)
    input carry_in, // Carry in
    output [31:0] sum // Result
);

    wire carry_0;
    wire carry_1;
    wire carry_2;
    wire carry_3;
    wire carry_4;
    wire carry_5;
    wire carry_6;
    /* verilator lint_off UNUSED */
    wire carry_7;
    /* verilator lint_on UNUSED */
    adder4 adder0(prop[3:0], gen[3:0], carry_in, sum[3:0], carry_0);
    adder4 adder1(prop[7:4], gen[7:4], carry_0, sum[7:4], carry_1);
    adder4 adder2(prop[11:8], gen[11:8], carry_1, sum[11:8], carry_2);
    adder4 adder3(prop[15:12], gen[15:12], carry_2, sum[15:12], carry_3);
    adder4 adder4(prop[19:16], gen[19:16], carry_3, sum[19:16], carry_4);
    adder4 adder5(prop[23:20], gen[23:20], carry_4, sum[23:20], carry_5);
    adder4 adder6(prop[27:24], gen[27:24], carry_5, sum[27:24], carry_6);
    adder4 adder7(prop[31:28], gen[31:28], carry_6, sum[31:28], carry_7);

`ifdef FORMAL
    /* Validate logic */
    (* anyconst *) wire [3:0] f_a;
    (* anyconst *) wire [3:0] f_b;
    always_comb begin
        assume(prop == (f_a ^ f_b));
        assume(gen == (f_a & f_b));
        assert(sum == (f_a + f_b + carry_in));
    end
`endif

endmodule

`endif
