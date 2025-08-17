`ifndef __MUX4_SV__
`define __MUX4_SV__

/*
 * 4:1 MUX
 */
module mux4 #(
    parameter BITS = 1
) (
    input [BITS-1:0] d1, // Input D1
    input [BITS-1:0] d2, // Input D2
    input [BITS-1:0] d3, // Input D3
    input [BITS-1:0] d4, // Input D4
    input [1:0] select, // Select
    output [BITS-1:0] out // Output
);

    assign out = select == 0 ? d1 :
        (select == 1 ? d2 :
            (select == 2 ? d3 : d4)
        );

`ifdef FORMAL
    /* Validate logic */
    always_comb begin
        if (select == 0)
            assert(out == d1);
        else if (select == 1)
            assert(out == d2);
        else if (select == 2)
            assert(out == d3);
        else
            assert(out == d4);
    end
`endif

endmodule

`endif
