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
    wire [`NUM_STAGES-1:0] stage_done;
    assign stage_done[`STAGE_FETCH] = ~mem_busy;
    assign stage_done[`STAGE_DECODE] = 1'b1;
    assign stage_done[`STAGE_READ] = 1'b1;
    assign stage_done[`STAGE_EXECUTE] = 1'b1;
    assign stage_done[`STAGE_MEMORY] = ~mem_busy;
    assign stage_done[`STAGE_WRITE_BACK] = 1'b1;
    /* verilator lint_off UNOPT */
    wire [`NUM_STAGES-1:0] stage_active /* verilator public */;
    /* verilator lint_on UNOPT */
    wire [`NUM_STAGES-1:0] enabled_stages;
    assign enabled_stages[`STAGE_FETCH] = 1'b1;
    assign enabled_stages[`STAGE_DECODE] = 1'b1;
    assign enabled_stages[`STAGE_EXECUTE] = 1'b1;
    assign enabled_stages[`STAGE_WRITE_BACK] = 1'b1;
    fsm fsm_module(
        clk,
        reset,
        enabled_stages,
        stage_done,
        stage_active);

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
        clk & (reset | stage_active[`STAGE_WRITE_BACK]),
        reset,
        pc_in,
        pc_pc,
        pc_fault);
    adder32_sync next_pc_adder_module(
        clk & stage_active[`STAGE_DECODE],
        pc_pc,
        32'd4,
        pc_next_pc);
    adder32_sync offset_pc_adder_module(
        clk & stage_active[`STAGE_EXECUTE],
        pc_pc,
        decode_imm,
        pc_offset_pc);

    /* Memory */
    wire [31:0] mem_out /* verilator public */;
    wire mem_busy;
    wire mem_fault;
    memory mem_module(
        clk,
        reset,
        (stage_active[`STAGE_FETCH] | stage_active[`STAGE_MEMORY]),
        stage_active[`STAGE_FETCH] ? 1'b0 : decode_ma_mem_microcode[3],
        stage_active[`STAGE_FETCH] ? 1'b0 : decode_ma_mem_microcode[2],
        stage_active[`STAGE_FETCH] ? 2'b10 : decode_ma_mem_microcode[1:0],
        stage_active[`STAGE_FETCH] ? pc_pc : alu_out,
        rf_read_data_b,
        mem_out,
        mem_busy,
        mem_fault);

    /* Instruction decode */
    wire [31:0] decode_imm /* verilator public */;
    wire decode_alu_a_mux_position /* verilator public */;
    wire decode_alu_b_mux_position /* verilator public */;
    wire [1:0] decode_csr_mux_position /* verilator public */;
    wire [1:0] decode_wb_mux_position /* verilator public */;
    wire [7:0] decode_rd_rf_microcode /* verilator public */;
    /* verilator lint_off UNOPT */
    wire [5:0] decode_ex_alu_microcode /* verilator public */;
    wire [15:0] decode_ex_csr_microcode /* verilator public */;
    /* verilator lint_on UNOPT */
    wire [3:0] decode_ma_mem_microcode /* verilator public */;
    /* verilator lint_off UNOPT */
    wire [9:0] decode_wb_rf_microcode /* verilator public */;
    /* verilator lint_on UNOPT */
    wire [1:0] decode_wb_pc_mux_position /* verilator public */;
    wire decode_fault;
    instruction_decode decode_module(
        clk & stage_active[`STAGE_DECODE],
        mem_out,
        decode_imm,
        decode_alu_a_mux_position,
        decode_alu_b_mux_position,
        decode_csr_mux_position,
        decode_wb_mux_position,
        {enabled_stages[`STAGE_READ], decode_rd_rf_microcode},
        decode_ex_alu_microcode,
        {enabled_stages[`STAGE_MEMORY], decode_ma_mem_microcode},
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
        clk & (((stage_active[`STAGE_READ]) | (stage_active[`STAGE_WRITE_BACK] & decode_wb_rf_microcode[9]))),
        ~stage_active[`STAGE_READ] & decode_wb_rf_microcode[8],
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
        clk & (stage_active[`STAGE_EXECUTE] & decode_ex_alu_microcode[5]),
        decode_ex_alu_microcode[4:0],
        alu_in_a,
        decode_alu_b_mux_position ? decode_imm : rf_read_data_b,
        alu_out,
        alu_fault);

    wire [31:0] csr_read_value /* verilator public */;
    wire csr_fault;
    wire [31:0] csr_in /* verilator public */ = (decode_csr_mux_position == 2'b00) ? rf_read_data_a : (
        (decode_csr_mux_position == 2'b01) ? decode_imm : (
        (decode_csr_mux_position == 2'b10) ? pc_next_pc :
        32'b0));
    csr csr_module(
        clk & (reset | (stage_active[`STAGE_EXECUTE] & decode_ex_csr_microcode[15])),
        reset,
        decode_ex_csr_microcode[2:0],
        decode_ex_csr_microcode[14:3],
        csr_in,
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
