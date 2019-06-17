/*
 * FSM.
 */

`define STAGE_FETCH 0
`define STAGE_DECODE 1
`define STAGE_READ 2
`define STAGE_EXECUTE 3
`define STAGE_MEMORY 4
`define STAGE_WRITE_BACK 5

module fsm(
    input clk, // Clock signal
    input reset, // Reset signal
    input read_stage_enable, // Read stage enable
    input mem_stage_enable, // Memory stage enable
    output stage_is_fetch, // Fetch stage
    output stage_is_decode, // Decode stage
    output stage_is_read, // Read stage
    output stage_is_execute, // Execute stage
    output stage_is_memory, // Memory stage
    output stage_is_write_back // Write back stage
);

    /* Logic */
    reg [5:0] current_stage;
    always @(posedge clk) begin
        if (reset) begin
            current_stage <= 1 << `STAGE_WRITE_BACK;
        end else if (current_stage[`STAGE_DECODE] & ~read_stage_enable) begin
            // no register read so skip to execute
            current_stage <= 1 << `STAGE_EXECUTE;
        end else if (current_stage[`STAGE_EXECUTE] & ~mem_stage_enable) begin
            // no memory access so skip to write back
            current_stage <= 1 << `STAGE_WRITE_BACK;
        end else begin
            current_stage <= {current_stage[4:0], current_stage[5]};
        end
    end
    assign {stage_is_write_back, stage_is_memory, stage_is_execute, stage_is_read, stage_is_decode, stage_is_fetch} = current_stage;

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
                assert(!stage_is_fetch);
                assert(!stage_is_decode);
                assert(!stage_is_read);
                assert(!stage_is_execute);
                assert(!stage_is_memory);
                assert(stage_is_write_back);
            end else if ($past(current_stage) == (1 << `STAGE_FETCH)) begin
                // fetch -> decode
                assert(!stage_is_fetch);
                assert(stage_is_decode);
                assert(!stage_is_read);
                assert(!stage_is_execute);
                assert(!stage_is_memory);
                assert(!stage_is_write_back);
            end else if ($past(current_stage) == (1 << `STAGE_DECODE)) begin
                // decode -> read / execute
                assert(!stage_is_fetch);
                assert(!stage_is_decode);
                assert(!stage_is_memory);
                assert(!stage_is_write_back);
                if ($past(read_stage_enable)) begin
                    assert(stage_is_read);
                    assert(!stage_is_execute);
                end else begin
                    assert(!stage_is_read);
                    assert(stage_is_execute);
                end
            end else if ($past(current_stage) == (1 << `STAGE_READ)) begin
                // read -> execute
                assert(!stage_is_fetch);
                assert(!stage_is_decode);
                assert(!stage_is_read);
                assert(stage_is_execute);
                assert(!stage_is_memory);
                assert(!stage_is_write_back);
            end else if ($past(current_stage) == (1 << `STAGE_EXECUTE)) begin
                // execute -> memory / write back
                assert(!stage_is_fetch);
                assert(!stage_is_decode);
                assert(!stage_is_read);
                assert(!stage_is_execute);
                if ($past(mem_stage_enable)) begin
                    assert(stage_is_memory);
                    assert(!stage_is_write_back);
                end else begin
                    assert(!stage_is_memory);
                    assert(stage_is_write_back);
                end
            end else if ($past(current_stage) == (1 << `STAGE_MEMORY)) begin
                // memory -> write back
                assert(!stage_is_fetch);
                assert(!stage_is_decode);
                assert(!stage_is_read);
                assert(!stage_is_execute);
                assert(!stage_is_memory);
                assert(stage_is_write_back);
            end else if ($past(current_stage) == (1 << `STAGE_WRITE_BACK)) begin
                // write back -> fetch
                assert(stage_is_fetch);
                assert(!stage_is_decode);
                assert(!stage_is_read);
                assert(!stage_is_execute);
                assert(!stage_is_memory);
                assert(!stage_is_write_back);
            end else begin
                assert(0);
            end
        end
    end
`endif

endmodule
