`include "cells/dffe.sv"

/*
 * Program counter.
 */
module program_counter(
    input logic clk, // Clock signal
    input logic reset_n, // Reset signal (active low)
    input logic enable_n, // Enable (active low)
    input logic [31:0] in, // Input data
    output logic [31:0] pc // Current PC
);

    dffe #(.BITS(32)) pc_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(in),
        .out(pc)
    );

`ifdef FORMAL
    initial assume(~reset_n);
    logic f_past_valid;
    initial f_past_valid = 0;
    always_ff @(posedge clk) begin
        f_past_valid = 1;
    end

    /* Validate logic */
    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            if ($past(~reset_n)) begin
                assert(pc == 0);
            end else begin
                assume($past(~enable_n));
                assert(pc == $past(in));
            end
        end
    end
`endif

endmodule
