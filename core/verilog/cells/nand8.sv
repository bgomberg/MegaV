`ifndef __NAND8_SV__
`define __NAND8_SV__

/*
 * 8:1 NAND Gate
 */
module nand8 #(
    parameter BITS = 1
) (
    input [BITS-1:0] a, // Input A
    input [BITS-1:0] b, // Input B
    input [BITS-1:0] c, // Input C
    input [BITS-1:0] d, // Input D
    input [BITS-1:0] e, // Input E
    input [BITS-1:0] f, // Input F
    input [BITS-1:0] g, // Input G
    input [BITS-1:0] h, // Input H
    output [BITS-1:0] out // Output
);

    assign out = ~(a & b & c & d & e & f & g & h);

`ifdef FORMAL
    /* Validate logic */
    always_comb begin
        assert(out == ~(a & b & c & d & e & f & g & h));
    end
`endif

endmodule

`endif
