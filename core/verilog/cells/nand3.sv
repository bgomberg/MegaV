`ifndef __NAND3_SV__
`define __NAND3_SV__

/*
 * 3:1 NAND Gate
 */
module nand3 #(
    parameter BITS = 1
) (
    input [BITS-1:0] a, // Input A
    input [BITS-1:0] b, // Input B
    input [BITS-1:0] c, // Input C
    output [BITS-1:0] out // Output
);

    assign out = ~(a & b & c);

`ifdef FORMAL
    /* Validate logic */
    always_comb begin
        assert(out == ~(a & b & c));
    end
`endif

endmodule

`endif
