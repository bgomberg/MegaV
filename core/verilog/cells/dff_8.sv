`ifndef __DFF_8_SV__
`define __DFF_8_SV__

`include "dff.sv"

module dff_8 #(
    parameter RESET_VALUE = 0
) (
    input logic clk, // Clock
    input logic clear_n, // Clear (active-low)
    input logic [7:0] in, // Input
    output logic [7:0] out // Output
);
    dff #(.DATA_WIDTH(8), .RESET_VALUE(RESET_VALUE)) dff(
        .clk(clk),
        .clear_n(clear_n),
        .in(in),
        .out(out)
    );

endmodule

`endif
