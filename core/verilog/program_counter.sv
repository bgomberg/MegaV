/*
 * Program counter.
 */
/* verilator lint_off UNUSED */
module program_counter(
    input logic clk, // Clock signal
    input logic reset_n, // Reset signal (active low)
    input logic available, // Operation available
    input logic [31:0] in, // Input data
    output logic [31:0] pc // Current PC
);

    /* Logic */
    always_ff @(posedge clk) begin
        if (~reset_n) begin
            // Reset
            pc <= 32'b0;
        end else if (available) begin
            pc <= in;
        end
    end

`ifdef FORMAL
    initial assume(~reset_n);
    logic f_past_valid;
    initial f_past_valid = 0;
    always_ff @(posedge clk) begin
        f_past_valid = 1;
    end

    /* Validate logic */
    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            if ($past(~reset_n)) begin
                assert(pc == 0);
            end else begin
                assume($past(available));
                assert(pc == $past(in));
            end
        end
    end
`endif

endmodule
