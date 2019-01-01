`ifndef __ADDER32_SV__
`define __ADDER32_SV__

`include "adder4.sv"

/*
 * A 32 bit full adder.
 */
module adder32(
    input [31:0] a, // Input data
    input [31:0] b, // Input data
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
    adder4 adder0(a[3:0], b[3:0], carry_in, sum[3:0], carry_0);
    adder4 adder1(a[7:4], b[7:4], carry_0, sum[7:4], carry_1);
    adder4 adder2(a[11:8], b[11:8], carry_1, sum[11:8], carry_2);
    adder4 adder3(a[15:12], b[15:12], carry_2, sum[15:12], carry_3);
    adder4 adder4(a[19:16], b[19:16], carry_3, sum[19:16], carry_4);
    adder4 adder5(a[23:20], b[23:20], carry_4, sum[23:20], carry_5);
    adder4 adder6(a[27:24], b[27:24], carry_5, sum[27:24], carry_6);
    adder4 adder7(a[31:28], b[31:28], carry_6, sum[31:28], carry_7);

`ifdef FORMAL
    /* Validate logic */
    always @(*) begin
        assert(sum == (a + b + carry_in));
    end
`endif

endmodule

`endif
