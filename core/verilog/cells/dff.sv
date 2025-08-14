`ifndef __DFF_SV__
`define __DFF_SV__

/*
 * A D-Type Flip-Flop.
 */
module dff #(
    parameter DATA_WIDTH = 1,
    parameter RESET_VALUE = 0
) (
    input logic clk, // Clock
    input logic clear_n, // Clear (active-low)
    input logic [DATA_WIDTH-1:0] in, // Input
    output logic [DATA_WIDTH-1:0] out // Output
);

    always_ff @(posedge clk) begin
        if (~clear_n) begin
            out <= RESET_VALUE;
        end else begin
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
            if ($past(clear_n)) begin
                assert(out == $past(in));
            end else begin
                assert(out == RESET_VALUE);
            end
        end
    end
`endif

endmodule

`endif
