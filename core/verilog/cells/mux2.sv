`ifndef __MUX2_SV__
`define __MUX2_SV__

/*
 * 2:1 MUX
 */
module mux2 #(
    parameter BITS = 1
) (
    input [BITS-1:0] a, // Input A
    input [BITS-1:0] b, // Input B
    input select, // Select
    output [BITS-1:0] out // Output
);

    assign out = select ? b : a;

`ifdef FORMAL
    /* Validate logic */
    always_comb begin
        if (select) begin
            assert(out == b);
        end else begin
            assert(out == a);
        end
    end
`endif

endmodule

`endif
