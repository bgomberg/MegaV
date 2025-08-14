`ifndef __MUX2_4_SV__
`define __MUX2_4_SV__

`include "mux2.sv"

module mux2_4(
    input [3:0] a, // Input A
    input [3:0] b, // Input B
    input select, // Select
    output [3:0] out // Output
);

    mux2 #(.DATA_WIDTH(4)) mux(
        .a(a),
        .b(b),
        .select(select),
        .out(out)
    );

endmodule

`endif
