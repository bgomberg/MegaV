module basic(i_clk, i_a, i_b, o_result);

input i_clk;
input [2:0] i_a;
input [2:0] i_b;
output [2:0] o_result;

reg	[2:0] out;

initial out = 0;
always @(posedge i_clk)
	out <= i_a + i_b;

assign o_result = out;

endmodule
