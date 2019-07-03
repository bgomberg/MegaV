/*
 * Program counter.
 */
module program_counter(
    input clk, // Clock signal
    input reset, // Reset signal
    input available, // Operation available
    input [31:0] in, // Input data
    output [31:0] pc, // Current PC
    output busy // Operation busy
);

    /* Outputs */
    reg [31:0] pc;
    reg busy;

    /* Logic */
    reg started;
    always @(posedge clk) begin
        pc <= (~reset & started & busy) ? in : ({32{~reset}} & pc);
        busy <= ~reset & ~started & available;
        started <= ~reset & available;
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
            assume(~started | busy);
            if ($past(reset)) begin
                assert(pc == 0);
            end else if ($past(busy) && !busy) begin
                assert(pc == $past(in));
            end
        end
    end
`endif

endmodule
