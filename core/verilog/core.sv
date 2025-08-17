`include "program_counter.sv"
`include "memory.sv"
`include "instruction_decode.sv"
`include "register_file.sv"
`include "alu.sv"
`include "csr.sv"
`include "fsm.sv"
`include "cells/and2.sv"
`include "cells/dffe.sv"
`include "cells/mux2.sv"
`include "cells/mux4.sv"

`define INSTR_INT_ECALL 32'h00000073

/*
 * The core which controls and integrates the individual components of the CPU.
 */
module core(
    input clk, // Clock signal
    input reset_n, // Reset signal (active low)
    input ext_int // External interrupt
);

    /* FSM */
    /* verilator lint_off UNOPT */
    wire [`NUM_STAGES-1:0] stage_active_n /* verilator public */;
    /* verilator lint_on UNOPT */
    wire [1:0] control_op /* verilator public */;
    wire control_op_normal = control_op[1] & control_op[0];
    wire control_op_trap = ~control_op[1] & ~control_op[0];
    wire control_op_ext_int = ~control_op[1] & control_op[0];
    wire control_op_sw_int = control_op[1] & ~control_op[0];
    logic [2:0] fault_num /* verilator public */;
    wire fsm_illegal_instr_fault = decode_fault | alu_fault | csr_fault;
    (* keep *) fsm fsm_module(
        .clk(clk),
        .reset_n(reset_n),
        .illegal_instr_fault(fsm_illegal_instr_fault),
        .mem_fault_num(mem_fault_num),
        .ext_int(csr_ext_int_pending),
        .sw_int(csr_sw_int_pending),
        .stage_active_n(stage_active_n),
        .control_op(control_op),
        .fault_num(fault_num)
    );

    /* Program counter */
    wire [31:0] pc_pc /* verilator public */;
    logic [31:0] pc_next_pc /* verilator public */;
    logic [31:0] pc_offset_pc /* verilator public */;
    wire pc_in_is_ex_result = decode_wb_pc_mux_position[1] & decode_wb_pc_mux_position[0];
    wire [1:0] pc_in_mux_select;
    mux2 #(.BITS(2)) pc_in_mux_select_mux(
        .a(decode_wb_pc_mux_position),
        .b({ex_alu_out[0], 1'b0}),
        .select(pc_in_is_ex_result),
        .out(pc_in_mux_select)
    );
    wire [31:1] pc_in;
    mux4 #(.BITS(31)) pc_in_mux(
        .d1(pc_next_pc[31:1]),
        .d2(ex_out[31:1]),
        .d3(pc_offset_pc[31:1]),
        .d4(31'b0),
        .select(pc_in_mux_select),
        .out(pc_in)
    );
    wire pc_enable_n = stage_active_n[`STAGE_UPDATE_PC];
    (* keep *) program_counter pc_module(
        .clk(clk),
        .reset_n(reset_n),
        .enable_n(pc_enable_n),
        .in({pc_in, 1'b0}),
        .pc(pc_pc)
    );

    /* Memory */
    wire [31:0] mem_out /* verilator public */;
    wire [2:0] mem_fault_num /* verilator public */;
    wire [1:0] mem_op_size;
    mux2 #(.BITS(2)) mem_op_size_mux(
        .a(2'b10),
        .b(decode_ma_mem_microcode[1:0]),
        .select(stage_active_n[`STAGE_FETCH]),
        .out(mem_op_size)
    );
    wire [31:0] mem_addr;
    mux2 #(.BITS(32)) mem_addr_mux(
        .a(pc_pc),
        .b(alu_out),
        .select(stage_active_n[`STAGE_FETCH]),
        .out(mem_addr)
    );
    wire mem_enable_n = (stage_active_n[`STAGE_FETCH] | ~control_op_normal) & (stage_active_n[`STAGE_MEMORY] | ~decode_has_mem_stage);
    wire mem_is_write = ~stage_active_n[`STAGE_MEMORY] & decode_ma_mem_microcode[3];
    wire mem_is_unsigned = ~stage_active_n[`STAGE_MEMORY] & decode_ma_mem_microcode[2];
    (* keep *) memory mem_module(
        .clk(clk),
        .reset_n(reset_n),
        .enable_n(mem_enable_n),
        .is_write(mem_is_write),
        .is_unsigned(mem_is_unsigned),
        .op_size(mem_op_size),
        .addr(mem_addr),
        .in(rf_read_data_b),
        .out(mem_out),
        .fault_num(mem_fault_num)
    );

    /* Instruction decode */
    wire [31:0] decode_imm /* verilator public */;
    wire decode_alu_a_mux_position /* verilator public */;
    wire decode_alu_b_mux_position /* verilator public */;
    wire [1:0] decode_csr_mux_position /* verilator public */;
    wire [1:0] decode_wb_mux_position /* verilator public */;
    wire [7:0] decode_rd_rf_microcode /* verilator public */;
    wire [5:0] decode_ex_alu_microcode /* verilator public */;
    wire [15:0] decode_ex_csr_microcode /* verilator public */;
    wire [3:0] decode_ma_mem_microcode /* verilator public */;
    /* verilator lint_off UNOPT */
    wire [4:0] decode_wb_rf_microcode /* verilator public */;
    /* verilator lint_on UNOPT */
    wire [1:0] decode_wb_pc_mux_position /* verilator public */;
    wire decode_has_mem_stage;
    wire decode_has_read_stage;
    wire decode_fault;
    wire [31:0] instr /* verilator public */;
    mux2 #(.BITS(32)) instr_mux(
        .a(`INSTR_INT_ECALL),
        .b(mem_out),
        .select(control_op_normal),
        .out(instr)
    );
    wire decode_enable_n = stage_active_n[`STAGE_DECODE];
    (* keep *) instruction_decode decode_module(
        .clk(clk),
        .reset_n(reset_n),
        .enable_n(decode_enable_n),
        .instr(instr),
        .imm(decode_imm),
        .alu_a_mux_position(decode_alu_a_mux_position),
        .alu_b_mux_position(decode_alu_b_mux_position),
        .csr_mux_position(decode_csr_mux_position),
        .wb_mux_position(decode_wb_mux_position),
        .rd_rf_microcode({decode_has_read_stage, decode_rd_rf_microcode}),
        .ex_alu_microcode(decode_ex_alu_microcode),
        .ma_mem_microcode({decode_has_mem_stage, decode_ma_mem_microcode}),
        .ex_csr_microcode(decode_ex_csr_microcode),
        .wb_rf_microcode(decode_wb_rf_microcode),
        .wb_pc_mux_position(decode_wb_pc_mux_position),
        .fault(decode_fault)
    );

    /* Register file */
    wire [31:0] ex_out;
    mux2 #(.BITS(32)) ex_out_mux(
        .a(csr_read_value),
        .b(ex_alu_out),
        .select(decode_ex_alu_microcode[5]),
        .out(ex_out)
    );
    wire [31:0] rf_write_data /* verilator public */;
    mux4 #(.BITS(32)) rf_write_data_mux(
        .d1(ex_out),
        .d2(decode_imm),
        .d3(mem_out),
        .d4(pc_next_pc),
        .select(decode_wb_mux_position),
        .out(rf_write_data)
    );
    wire [31:0] rf_read_data_a /* verilator public */;
    wire [31:0] rf_read_data_b /* verilator public */;
    wire rf_is_read_stage = ~stage_active_n[`STAGE_READ] & decode_has_read_stage;
    wire rf_is_wb_stage = ~stage_active_n[`STAGE_WRITE_BACK] & decode_wb_rf_microcode[4];
    wire rf_enable_n = ~rf_is_read_stage & ~rf_is_wb_stage;
    wire rf_write_en = ~rf_is_read_stage;
    (* keep *) register_file rf_module(
        .clk(clk),
        .reset_n(reset_n),
        .enable_n(rf_enable_n),
        .write_en(rf_write_en),
        .write_addr(decode_wb_rf_microcode[3:0]),
        .write_data(rf_write_data),
        .read_addr_a(decode_rd_rf_microcode[7:4]),
        .read_data_a(rf_read_data_a),
        .read_addr_b(decode_rd_rf_microcode[3:0]),
        .read_data_b(rf_read_data_b)
    );

    /* ALU */
    logic [31:0] ex_alu_out;
    wire [31:0] alu_out /* verilator public */;
    wire [31:0] alu_in_a;
    mux2 #(.BITS(32)) alu_in_a_mux(
        .a(rf_read_data_a),
        .b(pc_pc),
        .select(~stage_active_n[`STAGE_FETCH] | ~stage_active_n[`STAGE_WRITE_BACK] | decode_alu_a_mux_position),
        .out(alu_in_a)
    );
    wire [31:0] alu_in_b;
    mux4 #(.BITS(32)) alu_in_b_mux(
        .d1(32'h00000004),
        .d2(32'h00000004),
        .d3(rf_read_data_b),
        .d4(decode_imm),
        .select({stage_active_n[`STAGE_FETCH], ~stage_active_n[`STAGE_WRITE_BACK] | decode_alu_b_mux_position}),
        .out(alu_in_b)
    );
    wire alu_fault;
    wire [4:0] alu_op;
    and2 #(.BITS(5)) alu_op_and(
        .a({5{~stage_active_n[`STAGE_EXECUTE]}}),
        .b(decode_ex_alu_microcode[4:0]),
        .out(alu_op)
    );
    wire alu_enable_n = stage_active_n[`STAGE_FETCH] & stage_active_n[`STAGE_WRITE_BACK] & (stage_active_n[`STAGE_EXECUTE] | ~decode_ex_alu_microcode[5]);
    (* keep *) alu alu_module(
        .clk(clk),
        .reset_n(reset_n),
        .enable_n(alu_enable_n),
        .op(alu_op),
        .in_a(alu_in_a),
        .in_b(alu_in_b),
        .out(alu_out),
        .fault(alu_fault)
    );
    dffe #(.BITS(32)) pc_next_pc_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(stage_active_n[`STAGE_DECODE]),
        .in(alu_out),
        .out(pc_next_pc)
    );
    dffe #(.BITS(32)) ex_alu_out_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(stage_active_n[`STAGE_EXECUTE]),
        .in(alu_out),
        .out(ex_alu_out)
    );
    dffe #(.BITS(32)) pc_offset_pc_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(stage_active_n[`STAGE_WRITE_BACK]),
        .in(alu_out),
        .out(pc_offset_pc)
    );

    /* CSR */
    wire [2:0] csr_op;
    mux2 #(.BITS(3)) csr_op_mux(
        .a(3'b011),
        .b(decode_ex_csr_microcode[2:0]),
        .select((~stage_active_n[`STAGE_EXECUTE] | ~stage_active_n[`STAGE_MEMORY]) & decode_ex_csr_microcode[15]),
        .out(csr_op)
    );
    wire [3:0] csr_trap_num;
    mux2 #(.BITS(4)) csr_trap_num_mux(
        .a({control_op_ext_int, 3'b011}),
        .b({1'b0, fault_num}),
        .select(control_op_trap),
        .out(csr_trap_num)
    );
    wire [11:0] csr_addr_exception /* verilator public */;
    mux2 #(.BITS(12)) csr_addr_exception_mux(
        .a({7'b0, control_op_ext_int | control_op_sw_int, csr_trap_num}),
        .b(decode_ex_csr_microcode[14:3]),
        .select(control_op_normal),
        .out(csr_addr_exception)
    );
    wire [31:0] csr_in_pc;
    mux2 #(.BITS(32)) csr_in_pc_mux(
        .a(pc_pc),
        .b(pc_next_pc),
        .select(control_op_normal),
        .out(csr_in_pc)
    );
    wire [31:0] csr_in /* verilator public */;
    mux4 #(.BITS(32)) csr_in_mux(
        .d1(rf_read_data_a),
        .d2(decode_imm),
        .d3(csr_in_pc),
        .d4(32'b0),
        .select(decode_csr_mux_position),
        .out(csr_in)
    );
    wire [31:0] csr_read_value /* verilator public */;
    wire csr_fault;
    wire csr_ext_int_pending;
    wire csr_sw_int_pending;
    wire csr_enable_n = stage_active_n[`STAGE_EXECUTE] & stage_active_n[`STAGE_MEMORY];
    wire csr_is_write_stage = ~stage_active_n[`STAGE_MEMORY] & decode_ex_csr_microcode[15];
    (* keep *) csr csr_module(
        .clk(clk),
        .reset_n(reset_n),
        .enable_n(csr_enable_n),
        .is_write_stage(csr_is_write_stage),
        .ext_int(ext_int),
        .op(csr_op),
        .addr_exception(csr_addr_exception),
        .write_value(csr_in),
        .read_value(csr_read_value),
        .ext_int_pending(csr_ext_int_pending),
        .sw_int_pending(csr_sw_int_pending),
        .fault(csr_fault)
    );

endmodule
