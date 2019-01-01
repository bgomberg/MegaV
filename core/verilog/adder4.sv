`ifndef __ADDER4_SV__
`define __ADDER4_SV__

/*
 * A 4 bit full adder.
 */
module adder4(
    input [3:0] a, // Input data
    input [3:0] b, // Input data
    input carry_in, // Carry in
    output [3:0] sum, // Result
    output carry_out // Carry out
);

    wire [3:0] prop = a ^ b;
    wire [3:0] gen = a & b;
    wire all_prop = prop[0] & prop[1] & prop[2] & prop[3];

    wire carry_0 = (prop[0] & carry_in) | (gen[0]);
    wire carry_1 = (prop[1] & carry_0) | (gen[1]);
    wire carry_2 = (prop[2] & carry_1) | (gen[2]);
    assign carry_out = all_prop ? carry_in : ((prop[3] & carry_2) | (gen[3]));

    assign sum = {
        prop[3] ^ carry_2,
        prop[2] ^ carry_1,
        prop[1] ^ carry_0,
        prop[0] ^ carry_in
    };

`ifdef FORMAL
    /* Validate logic */
    always @(*) begin
        assert(sum == (a + b + carry_in));
    end
`endif

endmodule

`endif
