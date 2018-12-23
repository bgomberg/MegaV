/*
 * A simple adder.
 */
module adder #(
)(
    input clk, // Clock signal
    input [31:0] in_a,
    input [31:0] in_b,
    output [31:0] out
);

    /* Logic */
    reg [31:0] out;
    always @(posedge clk) begin
        out <= in_a + in_b;
    end

`ifdef FORMAL
    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) begin
        f_past_valid = 1;
    end

    /* Validate logic */
    always @(posedge clk) begin
    	if (f_past_valid) begin
            assert(out == ($past(in_a) + $past(in_b)));
        end
    end
`endif
endmodule
