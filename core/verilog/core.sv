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
    wire [`NUM_STAGES-1:0] stage_active_n /* verilator public */;
    wire [1:0] control_op /* verilator public */;
    wire control_op_normal = control_op[1] & control_op[0];
    wire control_op_trap = ~control_op[1] & ~control_op[0];
    wire control_op_ext_int = ~control_op[1] & control_op[0];
    wire control_op_sw_int = control_op[1] & ~control_op[0];
    logic [2:0] fault_num /* verilator public */;
    (* keep *) fsm fsm_module(
        .clk(clk),
        .reset_n(reset_n),
        .illegal_instr_fault(decode_fault),
        .mem_fault_num(mem_fault_num),
        .ext_int(csr_ext_int_pending),
        .sw_int(csr_sw_int_pending),
        .stage_active_n(stage_active_n),
        .control_op(control_op),
        .fault_num(fault_num)
    );

    /*
        Program Counter:
            [UPDATE_PC]: pc_pc = pc_in
    */
    wire [31:0] pc_pc /* verilator public */;
    logic [31:0] pc_next_pc /* verilator public */;
    logic [31:0] pc_offset_pc /* verilator public */;
    wire pc_in_is_ex_result = decode_mux_position.wb_pc[1] & decode_mux_position.wb_pc[0];
    wire [1:0] pc_in_mux_select;
    mux2 #(.BITS($bits(pc_in_mux_select))) pc_in_mux_select_mux(
        .a(decode_mux_position.wb_pc),
        .b({ex_alu_out[0], 1'b0}),
        .select(pc_in_is_ex_result),
        .out(pc_in_mux_select)
    );
    wire [31:0] pc_in;
    mux4 #(.BITS($bits(pc_in))) pc_in_mux(
        .d1(pc_next_pc),
        .d2({ex_out[31:1], 1'b0}),
        .d3({pc_offset_pc[31:1], 1'b0}),
        .d4(32'b0), // Unused
        .select(pc_in_mux_select),
        .out(pc_in)
    );
    wire pc_enable_n = stage_active_n[`STAGE_UPDATE_PC];
    dffe #(.BITS($bits(pc_pc))) pc_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(pc_enable_n),
        .in(pc_in),
        .out(pc_pc)
    );

    /*
        Memory:
            [FETCH]: instr = mem[pc_pc]
            [MEMORY]: rf_write_data = mem[ex_alu_out] || mem[ex_alu_out] = rf_read_data_b
    */
    wire [31:0] mem_out /* verilator public */;
    wire [2:0] mem_fault_num /* verilator public */;
    mem_microcode_t mem_microcode;
    mux2 #(.BITS($bits(mem_microcode))) mem_microcode_op_size_mux(
        .a({2'b0, MEM_OP_SIZE_WORD}),
        .b(decode_ma_mem_microcode.mem),
        .select(stage_active_n[`STAGE_FETCH]),
        .out(mem_microcode)
    );
    wire [31:0] mem_addr;
    mux2 #(.BITS($bits(mem_addr))) mem_addr_mux(
        .a(pc_pc),
        .b(ex_alu_out),
        .select(stage_active_n[`STAGE_FETCH]),
        .out(mem_addr)
    );
    wire mem_enable_n = (stage_active_n[`STAGE_FETCH] | ~control_op_normal) & (stage_active_n[`STAGE_MEMORY] | ~decode_ma_mem_microcode.enable);
    wire [31:0] mem_in = rf_read_data_b;
    (* keep *) memory mem_module(
        .clk(clk),
        .reset_n(reset_n),
        .enable_n(mem_enable_n),
        .microcode(mem_microcode),
        .addr(mem_addr),
        .in(mem_in),
        .out(mem_out),
        .fault_num(mem_fault_num)
    );

    /*
        Instruction Decode:
            [DECODE]: * = instr
    */
    wire [31:0] decode_imm /* verilator public */;
    decode_mux_position_t decode_mux_position /* verilator public */;
    decode_rd_rf_microcode_t decode_rd_rf_microcode /* verilator public */;
    decode_ex_alu_microcode_t decode_ex_alu_microcode /* verilator public */;
    decode_ex_csr_microcode_t decode_ex_csr_microcode /* verilator public */;
    decode_ma_mem_microcode_t decode_ma_mem_microcode /* verilator public */;
    decode_wb_rf_microcode_t decode_wb_rf_microcode /* verilator public */;
    wire decode_fault;
    wire [31:0] instr /* verilator public */;
    mux2 #(.BITS($bits(instr))) instr_mux(
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
        .mux_position(decode_mux_position),
        .rd_rf_microcode(decode_rd_rf_microcode),
        .ex_alu_microcode(decode_ex_alu_microcode),
        .ma_mem_microcode(decode_ma_mem_microcode),
        .ex_csr_microcode(decode_ex_csr_microcode),
        .wb_rf_microcode(decode_wb_rf_microcode),
        .fault(decode_fault)
    );

    /*
        Register File:
            [READ]: rf_read_data_a = reg[rf_read_addr_a]; rf_read_data_b = reg[rf_read_addr_b]
            [WRITE_BACK]: reg[rf_write_addr] = rf_write_data;
    */
    wire [31:0] ex_out;
    mux2 #(.BITS($bits(ex_out))) ex_out_mux(
        .a(csr_read_value),
        .b(ex_alu_out),
        .select(decode_ex_alu_microcode.enable),
        .out(ex_out)
    );
    wire [31:0] rf_write_data /* verilator public */;
    mux4 #(.BITS($bits(rf_write_data))) rf_write_data_mux(
        .d1(ex_out),
        .d2(decode_imm),
        .d3(mem_out),
        .d4(pc_next_pc),
        .select(decode_mux_position.wb_rf_in),
        .out(rf_write_data)
    );
    wire [31:0] rf_read_data_a /* verilator public */;
    wire [31:0] rf_read_data_b /* verilator public */;
    wire rf_read_en_n = stage_active_n[`STAGE_READ] | ~decode_rd_rf_microcode.enable;
    wire rf_write_en_n = stage_active_n[`STAGE_WRITE_BACK] | ~decode_wb_rf_microcode.enable;
    wire [3:0] rf_write_addr = decode_wb_rf_microcode.addr;
    wire [3:0] rf_read_addr_a = decode_rd_rf_microcode.addr_a;
    wire [3:0] rf_read_addr_b = decode_rd_rf_microcode.addr_b;
    (* keep *) register_file rf_module(
        .clk(clk),
        .reset_n(reset_n),
        .write_en_n(rf_write_en_n),
        .write_addr(rf_write_addr),
        .write_data(rf_write_data),
        .read_en_n(rf_read_en_n),
        .read_addr_a(rf_read_addr_a),
        .read_data_a(rf_read_data_a),
        .read_addr_b(rf_read_addr_b),
        .read_data_b(rf_read_data_b)
    );

    /*
        ALU:
            [FETCH] pc_next_pc = pc_pc + 4
            [EXECUTE] ex_alu_out = *
            [WRITE_BACK] pc_offset_pc = pc_pc + decode_imm
    */
    logic [31:0] ex_alu_out;
    wire [31:0] alu_in_a;
    mux2 #(.BITS($bits(alu_in_a))) alu_in_a_mux(
        .a(rf_read_data_a),
        .b(pc_pc),
        .select(~stage_active_n[`STAGE_FETCH] | ~stage_active_n[`STAGE_WRITE_BACK] | decode_mux_position.alu_in_a),
        .out(alu_in_a)
    );
    wire [31:0] alu_in_b;
    mux4 #(.BITS($bits(alu_in_b))) alu_in_b_mux(
        .d1(32'h00000004),
        .d2(32'h00000004),
        .d3(rf_read_data_b),
        .d4(decode_imm),
        .select({stage_active_n[`STAGE_FETCH], ~stage_active_n[`STAGE_WRITE_BACK] | decode_mux_position.alu_in_b}),
        .out(alu_in_b)
    );
    alu_microcode_t alu_microcode;
    mux2 #(.BITS($bits(alu_microcode))) alu_microcode_mux(
        .a(decode_ex_alu_microcode.alu),
        .b(`ALU_MICROCODE_ADD),
        .select(stage_active_n[`STAGE_EXECUTE]),
        .out(alu_microcode)
    );
    wire [31:0] alu_out /* verilator public */;
    (* keep *) alu alu_module(
        .microcode(alu_microcode),
        .in_a(alu_in_a),
        .in_b(alu_in_b),
        .out(alu_out)
    );
    dffe #(.BITS($bits(pc_next_pc))) pc_next_pc_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(stage_active_n[`STAGE_FETCH]),
        .in(alu_out),
        .out(pc_next_pc)
    );
    dffe #(.BITS($bits(ex_alu_out))) ex_alu_out_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(stage_active_n[`STAGE_EXECUTE]),
        .in(alu_out),
        .out(ex_alu_out)
    );
    dffe #(.BITS($bits(pc_offset_pc))) pc_offset_pc_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(stage_active_n[`STAGE_WRITE_BACK]),
        .in(alu_out),
        .out(pc_offset_pc)
    );

    /*
        CSR:
            [EXECUTE] csr_read_value = CSR[csr_addr_exception], internally evaluate interrupts
            [MEMORY] CSR[csr_addr_exception] = csr_in, internally evaluate interrupts
    */
    wire [2:0] csr_op;
    mux2 #(.BITS($bits(csr_op))) csr_op_mux(
        .a(CSR_OP_NO_OP),
        .b(decode_ex_csr_microcode.op),
        .select(decode_ex_csr_microcode.enable),
        .out(csr_op)
    );
    wire [3:0] csr_trap_num;
    mux2 #(.BITS($bits(csr_trap_num))) csr_trap_num_mux(
        .a({control_op_ext_int, `CSR_EXCEPTION_INT}),
        .b({1'b0, fault_num}),
        .select(control_op_trap),
        .out(csr_trap_num)
    );
    wire [4:0] csr_addr_exception /* verilator public */;
    mux2 #(.BITS($bits(csr_addr_exception))) csr_addr_exception_mux(
        .a({control_op_ext_int | control_op_sw_int, csr_trap_num}),
        .b({1'b0, decode_ex_csr_microcode.addr_exception}),
        .select(control_op_normal),
        .out(csr_addr_exception)
    );
    wire [31:0] csr_in_pc;
    mux2 #(.BITS($bits(csr_in_pc))) csr_in_pc_mux(
        .a(pc_pc),
        .b(pc_next_pc),
        .select(control_op_normal),
        .out(csr_in_pc)
    );
    wire [31:0] csr_in /* verilator public */;
    mux4 #(.BITS($bits(csr_in))) csr_in_mux(
        .d1(rf_read_data_a),
        .d2(decode_imm),
        .d3(csr_in_pc),
        .d4(32'b0), // Unused
        .select(decode_mux_position.csr_in),
        .out(csr_in)
    );
    wire [31:0] csr_read_value /* verilator public */;
    wire csr_ext_int_pending;
    wire csr_sw_int_pending;
    wire csr_write_stage_enable_n = stage_active_n[`STAGE_MEMORY] | ~decode_ex_csr_microcode.enable;
    wire csr_enable_n = stage_active_n[`STAGE_EXECUTE] & csr_write_stage_enable_n;
    (* keep *) csr csr_module(
        .clk(clk),
        .reset_n(reset_n),
        .enable_n(csr_enable_n),
        .is_write_stage(~csr_write_stage_enable_n),
        .ext_int(ext_int),
        .op(csr_op),
        .addr_exception(csr_addr_exception),
        .write_value(csr_in),
        .read_value(csr_read_value),
        .ext_int_pending(csr_ext_int_pending),
        .sw_int_pending(csr_sw_int_pending)
    );

endmodule
