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
    wire [1:0] op_value = {~fault & ~ext_int, ~fault & (ext_int | ~sw_int)};
    reg started;
    always @(posedge clk) begin
        op <= (started & busy) ? op_value : op;
        busy <= ~reset & ~started & available;
        started <= ~reset & (busy | available);
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
            if (busy) begin
                assume(available);
            end
            if (!$past(reset) && !busy && $past(busy)) begin
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
