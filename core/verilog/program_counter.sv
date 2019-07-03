/*
 * Program counter.
 */
module program_counter(
    input clk, // Clock signal
    input reset_n, // Reset signal (active low)
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
        pc <= (reset_n & started & busy) ? in : ({32{reset_n}} & pc);
        busy <= reset_n & ~started & available;
        started <= reset_n & available;
    end

`ifdef FORMAL
    initial assume(~reset_n);
    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) begin
        f_past_valid = 1;
    end

    /* Validate logic */
    always @(posedge clk) begin
        if (f_past_valid) begin
            assume(~started | busy);
            if ($past(~reset_n)) begin
                assert(pc == 0);
            end else if ($past(busy) && !busy) begin
                assert(pc == $past(in));
            end
        end
    end
`endif

endmodule
