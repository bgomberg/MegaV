`include "adder32.sv"

/*
 * A clocked 32-bit adder.
 */
module adder32_sync(
    input clk, // Clock
    input [31:0] a, // Input data
    input [31:0] b, // Input data
    output [31:0] out // Result
);

    reg [31:0] out;

    wire [31:0] sum;
    adder32 offset_pc_adder_module(
        a ^ b,
        a & b,
        1'b0,
        sum);

    /* Logic */
    always @(posedge clk) begin
        out <= sum;
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
            assert(out == ($past(a) + $past(b)));
        end
    end
`endif
endmodule
