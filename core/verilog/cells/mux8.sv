`ifndef __MUX8_SV__
`define __MUX8_SV__

/*
 * 8:3 MUX
 */
module mux8 #(
    parameter BITS = 1
) (
    input [BITS-1:0] d1, // Input D1
    input [BITS-1:0] d2, // Input D2
    input [BITS-1:0] d3, // Input D3
    input [BITS-1:0] d4, // Input D4
    input [BITS-1:0] d5, // Input D5
    input [BITS-1:0] d6, // Input D6
    input [BITS-1:0] d7, // Input D7
    input [BITS-1:0] d8, // Input D8
    input [2:0] select, // Select
    output [BITS-1:0] out // Output
);

    assign out = select == 0 ? d1 :
        (select == 1 ? d2 :
            (select == 2 ? d3 :
                (select == 3 ? d4 :
                    (select == 4 ? d5 :
                        (select == 5 ? d6 :
                            (select == 6 ? d7 : d8)
                        )
                    )
                )
            )
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
        else if (select == 3)
            assert(out == d4);
        else if (select == 4)
            assert(out == d5);
        else if (select == 5)
            assert(out == d6);
        else if (select == 6)
            assert(out == d7);
        else
            assert(out == d8);
    end
`endif

endmodule

`endif
