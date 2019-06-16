/*
 * Program counter.
 */
module program_counter(
    input clk, // Clock signal
    input reset, // Reset signal
    input [31:0] in, // Input data
    output [31:0] pc, // Current PC
    output fault // Fault condition
);

    /* Outputs */
    reg [31:0] pc;
    reg fault;

    /* Logic */
    always @(posedge clk) begin
        if (reset) begin
            pc <= 0;
            fault <= 0;
        end else begin
            pc <= in;
            fault <= in[1] | in[0];
        end
    end

`ifdef FORMAL
    initial assume(reset);
    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) begin
        f_past_valid = 1;
    end

    /* Validate logic */
    always @(posedge clk) begin
        if (f_past_valid) begin
            if ($past(reset)) begin
                assert(pc == 0);
                assert(!fault);
            end else begin
                assert(pc == $past(in));
                assert(fault == ($past(in[0]) | $past(in[1])));
            end
        end
    end
`endif

endmodule
