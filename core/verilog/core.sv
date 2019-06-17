`include "program_counter.sv"
`include "memory.sv"
`include "instruction_decode.sv"
`include "register_file.sv"
`include "alu.sv"
`include "adder32_sync.sv"
`include "csr.sv"
`include "fsm.sv"

/*
 * The core which controls and integrates the individual components of the CPU.
 */
module core(
    input clk, // Clock signal
    input reset, // Reset signal
    output fault // Fault condition
);

    /* FSM */
    wire stage_is_fetch /* verilator public */;
    wire stage_is_decode /* verilator public */;
    wire stage_is_read /* verilator public */;
    wire stage_is_execute /* verilator public */;
    wire stage_is_memory /* verilator public */;
    wire stage_is_write_back /* verilator public */;
    wire decode_rd_rf_enabled;
    wire decode_ma_mem_enabled;
    fsm fsm_module(
        clk,
        reset,
        decode_rd_rf_enabled,
        decode_ma_mem_enabled,
        stage_is_fetch,
        stage_is_decode,
        stage_is_read,
        stage_is_execute,
        stage_is_memory,
        stage_is_write_back);

    /* Program counter */
    wire [31:0] pc_pc /* verilator public */;
    reg [31:0] pc_next_pc /* verilator public */;
    wire [31:0] pc_offset_pc /* verilator public */;
    wire pc_fault;
    wire [31:0] pc_in = (decode_wb_pc_mux_position == 2'b00) ? pc_next_pc : (
        (decode_wb_pc_mux_position == 2'b01) ? ex_out : (
        (decode_wb_pc_mux_position == 2'b10) ? pc_offset_pc : (
        (decode_wb_pc_mux_position == 2'b11) ? (alu_out[0] ? pc_offset_pc : pc_next_pc) : 32'b0)));
    program_counter pc_module(
        clk & (reset | stage_is_write_back),
        reset,
        pc_in,
        pc_pc,
        pc_fault);
    adder32_sync offset_pc_adder_module(
        clk & stage_is_execute,
        pc_pc,
        32'd4,
        pc_next_pc);
    adder32_sync next_pc_adder_module(
        clk & stage_is_execute,
        pc_pc,
        decode_imm,
        pc_offset_pc);

    /* Memory */
    wire [31:0] mem_out /* verilator public */;
    wire mem_fault;
    memory mem_module(
        clk & (stage_is_fetch | stage_is_memory),
        // op needs to be set to b010 and stable before fetch stage starts (and remain stable during the fetch stage)
        (stage_is_write_back | stage_is_fetch) ? 3'b010 : {decode_ma_mem_microcode[3], decode_ma_mem_microcode[1:0]},
        // address needs to be set to the PC and stable before fetch stage starts (and remain stable during the fetch stage)
        (stage_is_write_back | stage_is_fetch) ? pc_pc : alu_out,
        rf_read_data_b,
        mem_out,
        mem_fault);

    /* Instruction decode */
    wire [31:0] decode_imm /* verilator public */;
    wire decode_alu_a_mux_position /* verilator public */;
    wire decode_alu_b_mux_position /* verilator public */;
    wire decode_csr_mux_position /* verilator public */;
    wire [1:0] decode_wb_mux_position /* verilator public */;
    wire [7:0] decode_rd_rf_microcode /* verilator public */;
    /* verilator lint_off UNOPT */
    wire [5:0] decode_ex_alu_microcode /* verilator public */;
    wire [15:0] decode_ex_csr_microcode /* verilator public */;
    /* verilator lint_on UNOPT */
    wire [3:0] decode_ma_mem_microcode /* verilator public */;
    wire [9:0] decode_wb_rf_microcode /* verilator public */;
    wire [1:0] decode_wb_pc_mux_position /* verilator public */;
    wire decode_fault;
    instruction_decode decode_module(
        clk & stage_is_decode,
        mem_out,
        decode_imm,
        decode_alu_a_mux_position,
        decode_alu_b_mux_position,
        decode_csr_mux_position,
        decode_wb_mux_position,
        {decode_rd_rf_enabled, decode_rd_rf_microcode},
        decode_ex_alu_microcode,
        {decode_ma_mem_enabled, decode_ma_mem_microcode},
        decode_ex_csr_microcode,
        decode_wb_rf_microcode,
        decode_wb_pc_mux_position,
        decode_fault);

    /* Register file */
    wire [31:0] ex_out = decode_ex_alu_microcode[5] ? alu_out : csr_read_value;
    wire [31:0] rf_write_data /* verilator public */ = (decode_wb_mux_position == 2'b00) ? ex_out : (
        (decode_wb_mux_position == 2'b01) ? decode_imm : (
        (decode_wb_mux_position == 2'b10) ? mem_out :
        pc_next_pc));
    wire [31:0] rf_read_data_a /* verilator public */;
    wire [31:0] rf_read_data_b /* verilator public */;
    register_file rf_module(
        clk & (((stage_is_read & decode_rd_rf_enabled) | (stage_is_write_back & decode_wb_rf_microcode[9]))),
        ~stage_is_read & decode_wb_rf_microcode[8],
        decode_wb_rf_microcode[7:4],
        rf_write_data,
        decode_rd_rf_microcode[7:4],
        rf_read_data_a,
        decode_rd_rf_microcode[3:0],
        rf_read_data_b);

    /* ALU */
    wire [31:0] alu_out /* verilator public */;
    wire alu_fault;
    wire [31:0] alu_in_a = decode_alu_a_mux_position ? pc_pc : rf_read_data_a;
    alu alu_module(
        clk & (stage_is_execute & decode_ex_alu_microcode[5]),
        decode_ex_alu_microcode[4:0],
        alu_in_a,
        decode_alu_b_mux_position ? decode_imm : rf_read_data_b,
        alu_out,
        alu_fault);

    wire [31:0] csr_read_value /* verilator public */;
    wire csr_fault;
    csr csr_module(
        clk & (reset | (stage_is_execute & decode_ex_csr_microcode[15])),
        reset,
        decode_ex_csr_microcode[2:0],
        decode_ex_csr_microcode[14:3],
        decode_csr_mux_position ? decode_imm : alu_in_a,
        csr_read_value,
        csr_fault
    );

    /* Fault signal */
    reg fault;
    always @(*) begin
        fault = pc_fault | mem_fault | decode_fault | alu_fault | csr_fault;
`ifdef VERILATOR
        if (pc_fault)
            $display("!!! PC FAULT");
        if (mem_fault)
            $display("!!! MEM FAULT");
        if (decode_fault)
            $display("!!! DECODE FAULT");
        if (alu_fault)
            $display("!!! ALU FAULT");
        if (csr_fault)
            $display("!!! CSR FAULT");
`endif
    end

`ifdef FORMAL
    initial assume(reset);
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
