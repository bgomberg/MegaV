`ifndef __OR4_SV__
`define __OR4_SV__

/*
 * 4:1 OR Gate
 */
module or4 #(
    parameter BITS = 1
) (
    input [BITS-1:0] a, // Input A
    input [BITS-1:0] b, // Input B
    input [BITS-1:0] c, // Input C
    input [BITS-1:0] d, // Input D
    output [BITS-1:0] out // Output
);

    assign out = a | b | c | d;

`ifdef FORMAL
    /* Validate logic */
    always_comb begin
        assert(out == (a | b | c | d));
    end
`endif

endmodule

`endif
