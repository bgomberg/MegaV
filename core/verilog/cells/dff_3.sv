`ifndef __DFF_3_SV__
`define __DFF_3_SV__

`include "dff.sv"

module dff_3 #(
    parameter RESET_VALUE = 0
) (
    input logic clk, // Clock
    input logic clear_n, // Clear (active-low)
    input logic [2:0] in, // Input
    output logic [2:0] out // Output
);
    dff #(.DATA_WIDTH(3), .RESET_VALUE(RESET_VALUE)) dff(
        .clk(clk),
        .clear_n(clear_n),
        .in(in),
        .out(out)
    );

endmodule

`endif
