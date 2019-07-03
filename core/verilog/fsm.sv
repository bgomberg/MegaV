`define STAGE_CONTROL 0
`define STAGE_FETCH 1
`define STAGE_DECODE 2
`define STAGE_READ 3
`define STAGE_EXECUTE 4
`define STAGE_MEMORY 5
`define STAGE_WRITE_BACK 6
`define NUM_STAGES 7

/*
 * FSM.
 */
module fsm(
    input clk, // Clock signal
    input reset, // Reset signal
    input [`NUM_STAGES-1:0] stage_done, // Stage done
    input illegal_instr_fault, // Illegal instruction fault
    input mem_addr_fault, // Memory address fault
    input mem_access_fault, // Memory access fault
    input mem_fault_is_store, // Memory fault is related to a store operation
    output [`NUM_STAGES-1:0] stage_active, // Current stage
    output fault, // Fault active
    output [2:0] fault_num // Active fault number
);

    /* Outputs */
    reg [`NUM_STAGES-1:0] stage_active;
    reg fault;
    reg [2:0] fault_num;

    /* Logic */
    reg in_progress; // need to stall at least one cycle per stage to allow the stage_done bits to be updated
    wire current_stage_done = ((stage_active & stage_done) != 0) & in_progress;
    wire [`NUM_STAGES-1:0] next_stage = current_stage_done ? {stage_active[`NUM_STAGES-2:0], stage_active[`NUM_STAGES-1]} : stage_active;
    wire mem_fault = mem_addr_fault | mem_access_fault;
    wire fetch_stage_mem_fault = mem_fault & stage_active[`STAGE_FETCH];
    wire mem_stage_mem_fault = mem_fault & stage_active[`STAGE_MEMORY];
    wire non_control_stage_instr_fault = ~stage_active[`STAGE_CONTROL] & illegal_instr_fault;
    wire active_fault = current_stage_done & (fetch_stage_mem_fault | mem_stage_mem_fault | non_control_stage_instr_fault);
    wire [2:0] active_fault_num = {
        mem_stage_mem_fault & ~non_control_stage_instr_fault,
        (mem_stage_mem_fault & mem_fault_is_store) | non_control_stage_instr_fault,
        ~non_control_stage_instr_fault & ~mem_addr_fault & mem_access_fault
    };
    always @(posedge clk) begin
        fault_num <= active_fault ? active_fault_num : fault_num;
        in_progress <= ~reset & ~current_stage_done;
        fault <= ~reset & (current_stage_done ? active_fault : fault);
        stage_active <= (~reset & ~active_fault) ? next_stage : 1 << 0;
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
            // Only one bit in stage_active should ever be set
            assume($past(stage_active & (stage_active - 1)) == 0);
            if ($past(reset)) begin
                assert(stage_active == 1 << 0);
                assert(!fault);
            end else if ($past(stage_active) != $past(stage_active, 2) || $past(reset, 2)) begin
                // need to stay in each stage for at least 2 clock cycles
                assert(stage_active == $past(stage_active));
                assert(fault == $past(fault));
            end else if (!($past(stage_done) & $past(stage_active))) begin
                // the stage wasn't done, so should still be in it
                assert(stage_active == $past(stage_active));
                assert(fault == $past(fault));
            end else begin
                assert(stage_active == (1 << 0) || stage_active != $past(stage_active));
                if ($past(illegal_instr_fault) && !$past(stage_active[`STAGE_CONTROL])) begin
                    // Illegal instruction
                    assert(fault);
                    assert(stage_active == (1 << 0));
                    assert(fault_num == 3'b010);
                end else if ($past(mem_addr_fault) && $past(stage_active[`STAGE_FETCH])) begin
                    // Instruction address misaligned
                    assert(fault);
                    assert(stage_active == (1 << 0));
                    assert(fault_num == 3'b000);
                end else if ($past(mem_addr_fault) && $past(stage_active[`STAGE_MEMORY])) begin
                    assert(fault);
                    assert(stage_active == (1 << 0));
                    if ($past(mem_fault_is_store)) begin
                        // Store address misaligned
                        assert(fault_num == 3'b110);
                    end else begin
                        // Load address misaligned
                        assert(fault_num == 3'b100);
                    end
                end else if ($past(mem_access_fault) && $past(stage_active[`STAGE_FETCH])) begin
                    // Instruction access fault
                    assert(fault);
                    assert(fault_num == 3'b001);
                end else if ($past(mem_access_fault) && $past(stage_active[`STAGE_MEMORY])) begin
                    assert(fault);
                    assert(stage_active == (1 << 0));
                    if ($past(mem_fault_is_store)) begin
                        // Store access fault
                        assert(fault_num == 3'b111);
                    end else begin
                        // Load access fault
                        assert(fault_num == 3'b101);
                    end
                end else begin
                    // Stage was completed without a fault
                    assert(!fault);
                    assert(stage_active == $past({stage_active[`NUM_STAGES-2:0], stage_active[`NUM_STAGES-1]}));
                end
            end
        end
    end
`endif

endmodule
