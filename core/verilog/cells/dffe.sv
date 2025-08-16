`ifndef __DFFE_SV__
`define __DFFE_SV__

/*
 * A D-Type Flip-Flop with Enable (74ALVCH16823DGG).
 */
module dffe #(
    parameter BITS = 1
) (
    input logic clk, // Clock
    input logic clear_n, // Clear (active low)
    input logic enable_n, // Enable (active low)
    input logic [BITS-1:0] in, // Input
    output logic [BITS-1:0] out // Output
);

    always_ff @(posedge clk) begin
        if (~clear_n) begin
            out <= 0;
        end else if (~enable_n) begin
            out <= in;
        end
    end

`ifdef FORMAL
    logic f_past_valid;
    initial f_past_valid = 0;
    always_ff @(posedge clk) begin
        f_past_valid = 1;
    end

    /* Validate logic */
    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            if (~$past(clear_n)) begin
                assert(out == 0);
            end else if (~$past(enable_n)) begin
                assert(out == $past(in));
            end else begin
                assert(out == $past(out));
            end
        end
    end
`endif

endmodule

`endif
