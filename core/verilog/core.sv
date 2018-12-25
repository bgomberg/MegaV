`include "program_counter.sv"
`include "memory.sv"
`include "instruction_decode.sv"
`include "register_file.sv"
`include "alu.sv"
`include "adder32_sync.sv"

`define STAGE_FETCH 0
`define STAGE_DECODE 1
`define STAGE_READ 2
`define STAGE_EXECUTE 3
`define STAGE_MEMORY 4
`define STAGE_WRITE_BACK 5

/*
 * The core which controls and integrates the individual components of the CPU.
 */
module core #(
)(
    input clk, // Clock signal
    input reset, // Reset signal
    output fault // Fault condition
);

    /* Stage state variables */
    reg [5:0] stage /* verilator public */;
    always @(posedge clk) begin
        if (reset) begin
            stage <= 6'b0;
        end else if (stage == 6'b0) begin
            stage <= 1 << `STAGE_FETCH;
        end else if (stage[`STAGE_DECODE] & ~decode_rd_rf_microcode[9]) begin
            // no register read so skip to execute
            stage <= 1 << `STAGE_EXECUTE;
        end else if (stage[`STAGE_EXECUTE] & ~decode_ma_mem_microcode[4]) begin
            // no memory access so skip to write back
            stage <= 1 << `STAGE_WRITE_BACK;
        end else begin
            stage <= {stage[4:0], stage[5]};
        end
    end

    /* Program counter */
    wire [31:0] pc_pc /* verilator public */;
    reg [31:0] pc_next_pc /* verilator public */;
    wire [31:0] pc_offset_pc /* verilator public */;
    wire pc_fault;
    wire [31:0] pc_in = (decode_wb_pc_microcode == 2'b00) ? pc_next_pc : (
        (decode_wb_pc_microcode == 2'b01) ? alu_out : (
        (decode_wb_pc_microcode == 2'b10) ? pc_offset_pc : (
        (decode_wb_pc_microcode == 2'b11) ? (alu_out[0] ? pc_offset_pc : pc_next_pc) : 32'b0)));
    program_counter pc_module(
        clk & stage[`STAGE_WRITE_BACK],
        reset,
        pc_in,
        pc_pc,
        pc_fault);
    adder32_sync offset_pc_adder_module(
        clk & stage[`STAGE_EXECUTE],
        pc_pc,
        32'd4,
        pc_next_pc);
    adder32_sync next_pc_adder_module(
        clk & stage[`STAGE_EXECUTE],
        pc_pc,
        decode_imm,
        pc_offset_pc);

    /* Memory */
    wire [31:0] mem_out /* verilator public */;
    wire mem_fault;
    memory mem_module(
        clk & (stage[`STAGE_FETCH] | (stage[`STAGE_MEMORY] & decode_ma_mem_microcode[4])),
        // op needs to be set to b010 and stable before fetch stage starts (and remain stable during the fetch stage)
        ((stage == 6'b0) | stage[`STAGE_WRITE_BACK] | stage[`STAGE_FETCH]) ? 3'b010 : {decode_ma_mem_microcode[3], decode_ma_mem_microcode[1:0]},
        // address needs to be set to the PC and stable before fetch stage starts (and remain stable during the fetch stage)
        ((stage == 6'b0) | stage[`STAGE_WRITE_BACK] | stage[`STAGE_FETCH]) ? pc_pc : alu_out,
        rf_read_data_b,
        mem_out,
        mem_fault);

    /* Instruction decode */
    wire [31:0] decode_imm /* verilator public */;
    wire decode_alu_a_mux_position /* verilator public */;
    wire decode_alu_b_mux_position /* verilator public */;
    wire [1:0] decode_wb_mux_position /* verilator public */;
    wire [9:0] decode_rd_rf_microcode /* verilator public */;
    /* verilator lint_off UNOPT */
    wire [5:0] decode_ex_alu_microcode /* verilator public */;
    /* verilator lint_on UNOPT */
    wire [4:0] decode_ma_mem_microcode /* verilator public */;
    wire [9:0] decode_wb_rf_microcode /* verilator public */;
    wire [1:0] decode_wb_pc_microcode /* verilator public */;
    wire decode_fault;
    instruction_decode decode_module(
        clk & stage[`STAGE_DECODE],
        mem_out,
        decode_imm,
        decode_alu_a_mux_position,
        decode_alu_b_mux_position,
        decode_wb_mux_position,
        decode_rd_rf_microcode,
        decode_ex_alu_microcode,
        decode_ma_mem_microcode,
        decode_wb_rf_microcode,
        decode_wb_pc_microcode,
        decode_fault);

    /* Register file */
    wire [31:0] rf_write_data /* verilator public */ = (decode_wb_mux_position == 2'b00) ? alu_out : (
        (decode_wb_mux_position == 2'b01) ? decode_imm : (
        (decode_wb_mux_position == 2'b10) ? mem_out :
        pc_next_pc));
    wire [31:0] rf_read_data_a /* verilator public */;
    wire [31:0] rf_read_data_b /* verilator public */;
    register_file rf_module(
        clk & (((stage[`STAGE_READ] & decode_rd_rf_microcode[9]) | (stage[`STAGE_WRITE_BACK] & decode_wb_rf_microcode[9]))),
        ~stage[`STAGE_READ] & decode_wb_rf_microcode[8],
        decode_wb_rf_microcode[7:4],
        rf_write_data,
        decode_rd_rf_microcode[7:4],
        rf_read_data_a,
        decode_rd_rf_microcode[3:0],
        rf_read_data_b);

    /* ALU */
    wire [31:0] alu_out /* verilator public */;
    wire alu_fault;
    alu alu_module(
        clk & (stage[`STAGE_EXECUTE] & decode_ex_alu_microcode[5]),
        decode_ex_alu_microcode[4:0],
        decode_alu_a_mux_position ? pc_pc : rf_read_data_a,
        decode_alu_b_mux_position ? decode_imm : rf_read_data_b,
        alu_out,
        alu_fault);

    /* Fault signal */
    reg fault;
    always @(*) begin
        fault = pc_fault | mem_fault | decode_fault | alu_fault;
`ifdef VERILATOR
        if (pc_fault)
            $display("!!! PC FAULT");
        if (mem_fault)
            $display("!!! MEM FAULT");
        if (decode_fault)
            $display("!!! DECODE FAULT");
        if (alu_fault)
            $display("!!! ALU FAULT");
`endif
    end

`ifdef FORMAL
    initial	assume(reset);
    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) begin
        f_past_valid = 1;
    end

    /* Validate Logic */
    always @(posedge clk) begin
    	if (f_past_valid) begin
        end
    end
`endif

endmodule
