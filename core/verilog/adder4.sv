/*
 * A 4 bit full adder.
 */
module adder4(
    input [3:0] prop, // Propagate signal (a ^ b)
    input [3:0] gen, // Generate signal (a & b)
    input carry_in, // Carry in
    output [3:0] sum, // Result
    output carry_out // Carry out
);

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
    (* anyconst *) wire [3:0] f_a;
    (* anyconst *) wire [3:0] f_b;
    always_comb begin
        assume(prop == (f_a ^ f_b));
        assume(gen == (f_a & f_b));
        assert(sum == (f_a + f_b + carry_in));
    end
`endif

endmodule
