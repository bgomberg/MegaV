`ifndef __XOR2_SV__
`define __XOR2_SV__

/*
 * 2:1 XOR Gate
 */
module xor2 #(
    parameter BITS = 1
) (
    input [BITS-1:0] a, // Input A
    input [BITS-1:0] b, // Input B
    output [BITS-1:0] out // Output
);

    assign out = a ^ b;

`ifdef FORMAL
    /* Validate logic */
    always_comb begin
        assert(out == (a ^ b));
    end
`endif

endmodule

`endif
