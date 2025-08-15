`include "stages.sv"
`include "cells/dff.sv"
`include "cells/dff_stages.sv"
`include "cells/dffe.sv"
`include "cells/mux2.sv"

/*
 * FSM.
 */
module fsm(
    input logic clk, // Clock signal
    input logic reset_n, // Reset signal (active low)
    input logic illegal_instr_fault, // Illegal instruction fault
    input logic mem_addr_fault, // Memory address fault
    input logic mem_access_fault, // Memory access fault
    input logic mem_fault_is_store, // Memory fault is related to a store operation
    input logic ext_int, // External interrupt
    input logic sw_int, // Software interrupt
    output logic [`NUM_STAGES-1:0] stage_active, // Current stage
    output logic [1:0] control_op, // Control operation (2'b00=trap, 2'b01=ext_int, 2'b10=sw_int, 2'b11=normal)
    output logic [2:0] fault_num // Active fault number
);

    /* Next Stage */
    wire [`NUM_STAGES-1:0] next_stage = {stage_active[`NUM_STAGES-2:0], stage_active[`NUM_STAGES-1]};

    /* In Progress */
    logic in_progress;
    dff in_progress_dff(
        .clk(clk),
        .clear_n(reset_n),
        .in(~in_progress),
        .out(in_progress)
    );

    /* Control Op */
    wire [1:0] control_op_value = {~fault_value & ~ext_int, ~fault_value & (ext_int | ~sw_int)};
    wire update_control_op = ~reset_n | (in_progress & (next_stage[`STAGE_CONTROL] | active_fault));
    // TODO: Why doesn't this work?
    // dffe #(.BITS(2)) control_op_dffe(
    //     .clk(clk),
    //     .clear_n(reset_n),
    //     .enable_n(~update_control_op),
    //     .in(control_op_value),
    //     .out(control_op)
    // );
    wire [1:0] next_control_op;
    mux2 #(.BITS(2)) control_op_mux(
        .a(control_op),
        .b(control_op_value),
        .select(update_control_op),
        .out(next_control_op)
    );
    dff #(.BITS(2)) control_op_dff(
        .clk(clk),
        .clear_n(reset_n),
        .in(next_control_op),
        .out(control_op)
    );

    /* Fault / Fault Number */
    logic fault;
    wire mem_fault = mem_addr_fault | mem_access_fault;
    wire fetch_stage_mem_fault = mem_fault & stage_active[`STAGE_FETCH];
    wire mem_stage_mem_fault = mem_fault & stage_active[`STAGE_MEMORY];
    wire non_control_stage_instr_fault = ~stage_active[`STAGE_CONTROL] & illegal_instr_fault;
    wire active_fault = in_progress & (fetch_stage_mem_fault | mem_stage_mem_fault | non_control_stage_instr_fault);
    wire [2:0] active_fault_num = {
        mem_stage_mem_fault & ~non_control_stage_instr_fault,
        (mem_stage_mem_fault & mem_fault_is_store) | non_control_stage_instr_fault,
        ~non_control_stage_instr_fault & ~mem_addr_fault & mem_access_fault
    };
    wire fault_value;
    mux2 fault_value_mux(
        .a(fault),
        .b(active_fault),
        .select(in_progress),
        .out(fault_value)
    );
    dff fault_dff(
        .clk(clk),
        .clear_n(reset_n),
        .in(fault_value),
        .out(fault)
    );
    dffe #(.BITS(3)) fault_num_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(~active_fault),
        .in(active_fault_num),
        .out(fault_num)
    );

    /* Active Stage */
    wire [`NUM_STAGES-1:0] next_stage_active;
    wire [`NUM_STAGES-1:0] next_stage_active_intermediate;
    mux2 #(.BITS(`NUM_STAGES)) next_stage_active_intermediate_mux(
        .a(next_stage),
        .b(`DEFAULT_STAGE_ACTIVE),
        .select(active_fault),
        .out(next_stage_active_intermediate)
    );
    mux2 #(.BITS(`NUM_STAGES)) next_stage_active_mux(
        .a(stage_active),
        .b(next_stage_active_intermediate),
        .select(in_progress),
        .out(next_stage_active)
    );
    dff_stages stage_active_dff(
        .clk(clk),
        .clear_n(reset_n),
        .in(next_stage_active),
        .out(stage_active)
    );

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
            // Only one bit in stage_active should ever be set
            assume($past(stage_active & (stage_active - 1)) == 0);
            if ($past(~reset_n)) begin
                assert(stage_active == 1 << 0);
            end else if ($past(stage_active) != $past(stage_active, 2) || $past(~reset_n, 2)) begin
                // need to stay in each stage for at least 2 clock cycles
                assert(stage_active == $past(stage_active));
            end else begin
                assert(stage_active == (1 << 0) || stage_active != $past(stage_active));
                if ($past(illegal_instr_fault) && !$past(stage_active[`STAGE_CONTROL])) begin
                    // Illegal instruction
                    assert(stage_active == (1 << 0));
                    assert(fault_num == 3'b010);
                end else if ($past(mem_addr_fault) && $past(stage_active[`STAGE_FETCH])) begin
                    // Instruction address misaligned
                    assert(stage_active == (1 << 0));
                    assert(fault_num == 3'b000);
                end else if ($past(mem_addr_fault) && $past(stage_active[`STAGE_MEMORY])) begin
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
                    assert(fault_num == 3'b001);
                end else if ($past(mem_access_fault) && $past(stage_active[`STAGE_MEMORY])) begin
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
                    assert(stage_active == $past({stage_active[`NUM_STAGES-2:0], stage_active[`NUM_STAGES-1]}));
                end
            end
        end
    end
`endif

endmodule
