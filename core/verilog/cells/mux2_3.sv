`ifndef __MUX2_3_SV__
`define __MUX2_3_SV__

`include "mux2.sv"

module mux2_3(
    input [2:0] a, // Input A
    input [2:0] b, // Input B
    input select, // Select
    output [2:0] out // Output
);

    mux2 #(.DATA_WIDTH(3)) mux(
        .a(a),
        .b(b),
        .select(select),
        .out(out)
    );

endmodule

`endif
