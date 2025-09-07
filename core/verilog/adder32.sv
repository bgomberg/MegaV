`ifndef __ADDER32_SV__
`define __ADDER32_SV__

`include "adder4.sv"

/*
 * A 32 bit full adder.
 */
module adder32(
    input [31:0] a, // Input A
    input [31:0] b, // Input B
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
    adder4 adder0(
        .a(a[3:0]),
        .b(b[3:0]),
        .carry_in(carry_in),
        .sum(sum[3:0]),
        .carry_out(carry_0)
    );
    adder4 adder1(
        .a(a[7:4]),
        .b(b[7:4]),
        .carry_in(carry_0),
        .sum(sum[7:4]),
        .carry_out(carry_1)
    );
    adder4 adder2(
        .a(a[11:8]),
        .b(b[11:8]),
        .carry_in(carry_1),
        .sum(sum[11:8]),
        .carry_out(carry_2)
    );
    adder4 adder3(
        .a(a[15:12]),
        .b(b[15:12]),
        .carry_in(carry_2),
        .sum(sum[15:12]),
        .carry_out(carry_3)
    );
    adder4 adder4(
        .a(a[19:16]),
        .b(b[19:16]),
        .carry_in(carry_3),
        .sum(sum[19:16]),
        .carry_out(carry_4)
    );
    adder4 adder5(
        .a(a[23:20]),
        .b(b[23:20]),
        .carry_in(carry_4),
        .sum(sum[23:20]),
        .carry_out(carry_5)
    );
    adder4 adder6(
        .a(a[27:24]),
        .b(b[27:24]),
        .carry_in(carry_5),
        .sum(sum[27:24]),
        .carry_out(carry_6)
    );
    adder4 adder7(
        .a(a[31:28]),
        .b(b[31:28]),
        .carry_in(carry_6),
        .sum(sum[31:28]),
        .carry_out(carry_7)
    );

`ifdef FORMAL
    /* Validate logic */
    always_comb begin
        assert(sum == (a + b + carry_in));
    end
`endif

endmodule

`endif
