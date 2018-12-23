/*
 * A basic +4 adder.
 */
module plus_4_adder #(
)(
    input clk, // Clock signal
    input [31:0] in, // Reset signal
    output [31:0] out
);

    /* Logic */
    reg [31:0] out;
    always @(posedge clk) begin
        out <= in + 4;
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
            assert(out == in + 4);
        end
    end
`endif
endmodule
