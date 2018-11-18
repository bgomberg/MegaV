/*
 * Basic module which sets the o_result to the sum of i_a and i_b on the rising edge of the clock
 */
module basic(i_clk, i_a, i_b, o_result);

input i_clk;
input [2:0] i_a;
input [2:0] i_b;
output [2:0] o_result;

reg	[2:0] o_result;

initial o_result = 0;
always @(posedge i_clk) begin
	o_result <= i_a + i_b;
end

/*
 * Formal properties
 */
`ifdef FORMAL

initial	f_past_valid = 1'b0;
always @(posedge i_clk) begin
	f_past_valid <= 1'b1;
end

always @(posedge i_clk) begin
	if (f_past_valid) begin
		assert(o_result == $past(i_a) + $past(i_b));
	end
end

`endif

endmodule
