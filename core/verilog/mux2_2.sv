`ifndef __MUX2_2_SV__
`define __MUX2_2_SV__


`include "mux2.sv"

module mux2_2(
    input [1:0] a, // Input A
    input [1:0] b, // Input B
    input select, // Select
    output [1:0] out // Output
);

    mux2 #(.DATA_WIDTH(2)) mux(
        .a(a),
        .b(b),
        .select(select),
        .out(out)
    );

endmodule

`endif
