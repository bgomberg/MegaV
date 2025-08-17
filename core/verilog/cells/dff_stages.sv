`ifndef __DFF_STAGES_SV__
`define __DFF_STAGES_SV__

`include "dff.sv"
`include "../stages.sv"

module dff_stages (
    input logic clk, // Clock
    input logic clear_n, // Clear (active low)
    input logic [`NUM_STAGES-1:0] in, // Input
    output logic [`NUM_STAGES-1:0] out // Output
);
    dff #(.BITS(`NUM_STAGES), .RESET_VALUE(~`DEFAULT_STAGE_ACTIVE)) dff(
        .clk(clk),
        .clear_n(clear_n),
        .in(in),
        .out(out)
    );

endmodule

`endif
