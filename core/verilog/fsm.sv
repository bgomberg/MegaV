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
    input [`NUM_STAGES-1:0] stage_enabled, // Enabled stages
    input [`NUM_STAGES-1:0] stage_done, // Stage done
    output [`NUM_STAGES-1:0] stage_active // Current stage
);

    /* Outputs */
    reg [`NUM_STAGES-1:0] stage_active;

    /* Logic */
    reg did_stall;
    wire [`NUM_STAGES-1:0] next_stage = {stage_active[`NUM_STAGES-2:0], stage_active[`NUM_STAGES-1]};
    wire [`NUM_STAGES-1:0] next_next_stage = {next_stage[`NUM_STAGES-2:0], next_stage[`NUM_STAGES-1]};
    wire stage_active_busy = (stage_active & stage_done) == 0;
    wire next_stage_enabled = (next_stage & stage_enabled) != 0;
    always @(posedge clk) begin
        if (reset) begin
            stage_active <= 1 << 0;
            did_stall <= 1'b0;
        end else if (stage_active_busy) begin
            // not done with the current stage, so don't change it
            stage_active <= stage_active;
            // this counts as our stall cycle
            did_stall <= 1'b1;
        end else if (!did_stall) begin
            stage_active <= stage_active;
            // need to stall at least one cycle per stage to allow the stage_done bits to be updated
            did_stall <= 1'b1;
        end else if (next_stage_enabled) begin
            stage_active <= next_stage;
            did_stall <= 1'b0;
        end else begin
            stage_active <= next_next_stage;
            did_stall <= 1'b0;
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
        assume($past(stage_enabled) & 6'b101011 == 6'b101011);
        if (f_past_valid) begin
            if ($past(reset)) begin
                assert(stage_active == 1 << 0);
            end else if (!($past(stage_done) & $past(stage_active))) begin
                assert(stage_active == $past(stage_active));
            end else if (!$past(did_stall)) begin
                assert(stage_active == $past(stage_active));
            end else if (!($past(stage_enabled) & {$past(stage_active[`NUM_STAGES-2:0]), $past(stage_active[`NUM_STAGES-1])})) begin
                assert(stage_active == {$past(stage_active[`NUM_STAGES-3:0]), $past(stage_active[`NUM_STAGES-1:`NUM_STAGES-2])});
            end else begin
                assert(stage_active == {$past(stage_active[`NUM_STAGES-2:0]), $past(stage_active[`NUM_STAGES-1])});
            end
        end
    end
`endif

endmodule
