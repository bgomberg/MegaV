`define STATE_IDLE          2'b00
`define STATE_IN_PROGRESS   2'b01
`define STATE_DONE          2'b10

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
    reg [1:0] state;
    always @(posedge clk) begin
        if (reset) begin
            pc <= 32'b0;
            busy <= 1'b0;
            state <= `STATE_IDLE;
        end else begin
            case (state)
                `STATE_IDLE: begin
                    if (available) begin
                        // Starting a new operation
                        busy <= 1'b1;
                        state <= `STATE_IN_PROGRESS;
                    end else begin
                        busy <= 1'b0;
                    end
                end
                `STATE_IN_PROGRESS: begin
                    pc <= in;
                    // Operation is complete
                    busy <= 1'b0;
                    state <= `STATE_DONE;
                end
                `STATE_DONE: begin
                    if (!available) begin
                        state <= `STATE_IDLE;
                    end
                end
                default: begin
                    // should never get here
                end
            endcase
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
            assume(state != 2'b11);
            if ($past(reset)) begin
                assert(pc == 0);
            end else if (state == `STATE_DONE && $past(state) == `STATE_IN_PROGRESS) begin
                assert(pc == $past(in));
            end
        end
    end
`endif

endmodule
