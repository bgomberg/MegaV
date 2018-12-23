`include "program_counter.sv"
`include "memory.sv"
`include "instruction_decode.sv"
`include "register_file.sv"
`include "alu.sv"

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
    wire [5:0] stage /* verilator public */;
    always @(posedge clk) begin
        if (reset)
            stage <= 6'b0;
        else
            stage <= (stage == 6'b0) ? (1 << `STAGE_FETCH) : {stage[4:0], stage[5]};
    end

    /* Program counter */
    wire pc_en;
    always @(posedge clk) begin
        pc_en <= stage[`STAGE_WRITE_BACK];
    end
    wire [31:0] pc_pc /* verilator public */;
    wire [31:0] pc_next_pc;
    wire pc_fault /* verilator public */;
    program_counter pc_module(
        clk & pc_en,
        reset,
        decode_wb_pc_microcode,
        decode_imm,
        alu_out,
        pc_pc,
        pc_next_pc,
        pc_fault);

    /* Memory */
    wire mem_en;
    always @(posedge clk) begin
        mem_en <= stage[`STAGE_FETCH] | (stage[`STAGE_MEMORY] & decode_ma_mem_microcode[4]);
    end
    wire [31:0] mem_out /* verilator public */;
    wire mem_fault /* verilator public */;
    memory mem_module(
        clk & mem_en,
        // op needs to be set to b010 and stable before fetch stage starts (and remain stable during fetch stage)
        (stage[`STAGE_WRITE_BACK] | stage[`STAGE_FETCH]) ? 3'b010 : {decode_ma_mem_microcode[3], decode_ma_mem_microcode[1:0]},
        // address needs to be set to the PC and stable before fetch stage starts (and remain stable during fetch stage)
        (stage[`STAGE_WRITE_BACK] | stage[`STAGE_FETCH]) ? pc_pc : alu_out,
        rf_read_data_b,
        mem_out,
        mem_fault);

    /* Instruction decode */
    wire decode_en;
    always @(posedge clk) begin
        decode_en <= stage[`STAGE_DECODE];
    end
    wire [31:0] decode_imm /* verilator public */;
    wire decode_alu_a_mux_position /* verilator public */;
    wire decode_alu_b_mux_position /* verilator public */;
    wire [1:0] decode_wb_mux_position /* verilator public */;
    wire [9:0] decode_rd_rf_microcode /* verilator public */;
    wire [5:0] decode_ex_alu_microcode /* verilator public */;
    wire [4:0] decode_ma_mem_microcode /* verilator public */;
    wire [9:0] decode_wb_rf_microcode /* verilator public */;
    wire [1:0] decode_wb_pc_microcode /* verilator public */;
    wire decode_fault /* verilator public */;
    instruction_decode decode_module(
        clk & decode_en,
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
    wire rf_en;
    always @(posedge clk) begin
        rf_en <= ((stage[`STAGE_READ] & decode_rd_rf_microcode[9]) | (stage[`STAGE_WRITE_BACK] & decode_wb_rf_microcode[9]));
    end
    wire [31:0] rf_write_data /* verilator public */ = (decode_wb_mux_position == 2'b00) ? alu_out : (
        (decode_wb_mux_position == 2'b01) ? decode_imm : (
        (decode_wb_mux_position == 2'b10) ? mem_out :
        pc_next_pc));
    wire [31:0] rf_read_data_a /* verilator public */;
    wire [31:0] rf_read_data_b /* verilator public */;
    register_file rf_module(
        clk & rf_en,
        ~stage[`STAGE_READ] & decode_wb_rf_microcode[8],
        decode_wb_rf_microcode[7:4],
        rf_write_data,
        decode_rd_rf_microcode[7:4],
        rf_read_data_a,
        decode_rd_rf_microcode[3:0],
        rf_read_data_b);

    /* ALU */
    wire alu_en;
    always @(posedge clk) begin
        alu_en <= (stage[`STAGE_EXECUTE] & decode_ex_alu_microcode[5]);
    end
    wire [31:0] alu_out /* verilator public */;
    wire alu_fault /* verilator public */;
    alu alu_module(
        clk & alu_en,
        decode_ex_alu_microcode[3:0],  // FIXME: add branch instructions to ALU
        decode_alu_a_mux_position ? pc_pc : rf_read_data_a,
        decode_alu_b_mux_position ? decode_imm : rf_read_data_b,
        alu_out,
        alu_fault);

    /* Fault signal */
    assign fault = pc_fault | mem_fault | decode_fault | alu_fault;

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
