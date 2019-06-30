`define STATE_IDLE          2'b00
`define STATE_IN_PROGRESS   2'b01
`define STATE_DONE          2'b10

/*
 * Control
 */
module control(
    input clk, // Clock signal
    input reset, // Reset signal
    input available, // Operation available
    input fault, // Fault signal
    input ext_int, // External interrupt
    input sw_int, // Software interrupt
    output [1:0] op, // Control operation (2'b00=trap, 2'b01=ext_int, 2'b10=sw_int, 2'b11=normal)
    output busy // Operation busy
);

    /* Outputs */
    reg [1:0] op;
    reg busy;

    /* Logic */
    reg [1:0] state;
    always @(posedge clk) begin
        if (reset) begin
            op <= 2'b11;
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
                    if (fault) begin
                        op <= 2'b00;
                    end else if (ext_int) begin
                        op <= 2'b01;
                    end else if (sw_int) begin
                        op <= 2'b10;
                    end else begin
                        op <= 2'b11;
                    end
                    // Operation is complete
                    busy <= 1'b0;
                    state <= `STATE_DONE;
                end
                `STATE_DONE: begin
                    if (!available) begin
                        busy <= 1'b0;
                        state <= `STATE_IDLE;
                    end else begin
                        busy <= 1'b1;
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
                assert(op == 2'b11);
            end else if (state == `STATE_DONE && $past(state) == `STATE_IN_PROGRESS) begin
                if ($past(fault)) begin
                    assert(op == 2'b00);
                end else if ($past(ext_int)) begin
                    assert(op == 2'b01);
                end else if ($past(sw_int)) begin
                    assert(op == 2'b10);
                end else begin
                    assert(op == 2'b11);
                end
            end
        end
    end
`endif

endmodule
