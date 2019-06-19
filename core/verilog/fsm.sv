/*
 * FSM.
 */

`define STAGE_FETCH 0
`define STAGE_DECODE 1
`define STAGE_READ 2
`define STAGE_EXECUTE 3
`define STAGE_MEMORY 4
`define STAGE_WRITE_BACK 5
`define NUM_STAGES 6

module fsm(
    input clk, // Clock signal
    input reset, // Reset signal
    /* verilator lint_off UNUSED */
    input [`NUM_STAGES-1:0] stage_enabled, // Enabled stages
    /* verilator lint_on UNUSED */
    output [`NUM_STAGES-1:0] stage_active // Current stage
);

    /* Outputs */
    reg [`NUM_STAGES-1:0] stage_active;

    /* Logic */
    reg [`NUM_STAGES-1:0] current_stage;
    wire [`NUM_STAGES-1:0] next_stage = {current_stage[`NUM_STAGES-2:0], current_stage[`NUM_STAGES-1]};
    wire [`NUM_STAGES-1:0] next_next_stage = {next_stage[`NUM_STAGES-2:0], next_stage[`NUM_STAGES-1]};
    always @(posedge clk) begin
        if (reset) begin
            current_stage <= 1 << (`NUM_STAGES - 1);
        end else if ((next_stage & stage_enabled) != `NUM_STAGES'b0) begin
            current_stage <= next_stage;
        end else begin
            current_stage <= next_next_stage;
        end
    end
    assign stage_active = current_stage;

`ifdef FORMAL
    initial assume(reset);
    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) begin
        f_past_valid = 1;
    end

    /* Validate logic */
    always @(posedge clk) begin
        assume($past(stage_enabled) & 6'b101011 == 6'b101011);
        if (f_past_valid) begin
            if ($past(reset)) begin
                assert(stage_active == (1 << (`NUM_STAGES - 1)));
            end else if ($past(current_stage) == (1 << `STAGE_FETCH)) begin
                // fetch -> decode
                assert(stage_active == (1 << `STAGE_DECODE));
            end else if ($past(current_stage) == (1 << `STAGE_DECODE)) begin
                // decode -> read / execute
                if ($past(stage_enabled[`STAGE_READ])) begin
                    assert(stage_active == (1 << `STAGE_READ));
                end else begin
                    assert(stage_active == (1 << `STAGE_EXECUTE));
                end
            end else if ($past(current_stage) == (1 << `STAGE_READ)) begin
                // read -> execute
                assert(stage_active == (1 << `STAGE_EXECUTE));
            end else if ($past(current_stage) == (1 << `STAGE_EXECUTE)) begin
                // execute -> memory / write back
                if ($past(stage_enabled[`STAGE_MEMORY])) begin
                    assert(stage_active == (1 << `STAGE_MEMORY));
                end else begin
                    assert(stage_active == (1 << `STAGE_WRITE_BACK));
                end
            end else if ($past(current_stage) == (1 << `STAGE_MEMORY)) begin
                // memory -> write back
                assert(stage_active == (1 << `STAGE_WRITE_BACK));
            end else if ($past(current_stage) == (1 << `STAGE_WRITE_BACK)) begin
                // write back -> fetch
                assert(stage_active == (1 << `STAGE_FETCH));
            end else begin
                assert(0);
            end
        end
    end
`endif

endmodule
