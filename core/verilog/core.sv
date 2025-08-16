`include "program_counter.sv"
`include "memory.sv"
`include "instruction_decode.sv"
`include "register_file.sv"
`include "alu.sv"
`include "csr.sv"
`include "fsm.sv"

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
    wire [`NUM_STAGES-1:0] stage_active /* verilator public */;
    /* verilator lint_on UNOPT */
    wire [1:0] control_op /* verilator public */;
    wire control_op_normal = control_op[1] & control_op[0];
    wire control_op_trap = ~control_op[1] & ~control_op[0];
    wire control_op_ext_int = ~control_op[1] & control_op[0];
    wire control_op_sw_int = control_op[1] & ~control_op[0];
    logic [2:0] fault_num /* verilator public */;
    fsm fsm_module(
        .clk(clk),
        .reset_n(reset_n),
        .illegal_instr_fault(decode_fault | mem_op_fault | alu_fault | csr_fault),
        .mem_addr_fault(mem_addr_fault),
        .mem_access_fault(~mem_access_fault_n),
        .mem_fault_is_store(decode_ma_mem_microcode[3]),
        .ext_int(csr_ext_int_pending),
        .sw_int(csr_sw_int_pending),
        .stage_active(stage_active),
        .control_op(control_op),
        .fault_num(fault_num)
    );

    /* Program counter */
    wire [31:0] pc_pc /* verilator public */;
    logic [31:0] pc_next_pc /* verilator public */;
    logic [31:0] pc_offset_pc /* verilator public */;
    wire [31:1] pc_in = (~decode_wb_pc_mux_position[1] & decode_wb_pc_mux_position[0]) ? ex_out[31:1] : (
        (decode_wb_pc_mux_position[1] & (~decode_wb_pc_mux_position[0] | ex_alu_out[0])) ? pc_offset_pc[31:1] : pc_next_pc[31:1]);
    program_counter pc_module(
        .clk(clk),
        .reset_n(reset_n),
        .enable_n(~stage_active[`STAGE_UPDATE_PC]),
        .in({pc_in, 1'b0}),
        .pc(pc_pc)
    );

    /* Memory */
    wire [31:0] mem_out /* verilator public */;
    wire mem_op_fault /* verilator public */;
    wire mem_addr_fault /* verilator public */;
    wire mem_access_fault_n /* verilator public */;
    memory mem_module(
        .clk(clk),
        .reset_n(reset_n),
        .enable_n(~((stage_active[`STAGE_FETCH] & control_op_normal) | (stage_active[`STAGE_MEMORY] & decode_has_mem_stage))),
        .is_write(stage_active[`STAGE_MEMORY] & decode_ma_mem_microcode[3]),
        .is_unsigned(stage_active[`STAGE_MEMORY] & decode_ma_mem_microcode[2]),
        .op(stage_active[`STAGE_FETCH] ? 2'b10 : decode_ma_mem_microcode[1:0]),
        .addr(stage_active[`STAGE_FETCH] ? pc_pc : alu_out),
        .in(rf_read_data_b),
        .out(mem_out),
        .op_fault(mem_op_fault),
        .addr_fault(mem_addr_fault),
        .access_fault_n(mem_access_fault_n)
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
    wire [9:0] decode_wb_rf_microcode /* verilator public */;
    /* verilator lint_on UNOPT */
    wire [1:0] decode_wb_pc_mux_position /* verilator public */;
    wire decode_has_mem_stage;
    wire decode_has_read_stage;
    wire decode_fault;
    wire [31:0] instr /* verilator public */ = control_op_normal ? mem_out : 32'h00000073; // interrupt using ECALL
    instruction_decode decode_module(
        .clk(clk),
        .reset_n(reset_n),
        .enable_n(~stage_active[`STAGE_DECODE]),
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
    wire [31:0] ex_out = decode_ex_alu_microcode[5] ? ex_alu_out : csr_read_value;
    wire [31:0] rf_write_data /* verilator public */ = (decode_wb_mux_position == 2'b00) ? ex_out : (
        (decode_wb_mux_position == 2'b01) ? decode_imm : (
        (decode_wb_mux_position == 2'b10) ? mem_out :
        pc_next_pc));
    wire [31:0] rf_read_data_a /* verilator public */;
    wire [31:0] rf_read_data_b /* verilator public */;
    register_file rf_module(
        .clk(clk),
        .reset_n(reset_n),
        .enable_n(~((stage_active[`STAGE_READ] & decode_has_read_stage) | (stage_active[`STAGE_WRITE_BACK] & decode_wb_rf_microcode[9]))),
        .write_en(~(stage_active[`STAGE_READ] & decode_has_read_stage) & decode_wb_rf_microcode[8]),
        .write_addr(decode_wb_rf_microcode[7:4]),
        .write_data(rf_write_data),
        .read_addr_a(decode_rd_rf_microcode[7:4]),
        .read_data_a(rf_read_data_a),
        .read_addr_b(decode_rd_rf_microcode[3:0]),
        .read_data_b(rf_read_data_b)
    );

    /* ALU */
    logic [31:0] ex_alu_out;
    wire [31:0] alu_out /* verilator public */;
    wire [31:0] alu_in_a = (stage_active[`STAGE_FETCH] | stage_active[`STAGE_WRITE_BACK] | decode_alu_a_mux_position) ? pc_pc : rf_read_data_a;
    wire [31:0] alu_in_b = stage_active[`STAGE_FETCH] ? 32'h00000004 : ((stage_active[`STAGE_WRITE_BACK] | decode_alu_b_mux_position) ? decode_imm : rf_read_data_b);
    wire alu_fault;
    alu alu_module(
        .clk(clk),
        .reset_n(reset_n),
        .enable_n(~(stage_active[`STAGE_FETCH] | stage_active[`STAGE_WRITE_BACK] | (stage_active[`STAGE_EXECUTE] & decode_ex_alu_microcode[5]))),
        .op({5{stage_active[`STAGE_EXECUTE]}} & decode_ex_alu_microcode[4:0]),
        .in_a(alu_in_a),
        .in_b(alu_in_b),
        .out(alu_out),
        .fault(alu_fault)
    );
    always_ff @(posedge clk) begin
        if (stage_active[`STAGE_DECODE]) begin
            pc_next_pc <= alu_out;
        end else if (stage_active[`STAGE_EXECUTE]) begin
            ex_alu_out <= alu_out;
        end else if (stage_active[`STAGE_WRITE_BACK]) begin
            pc_offset_pc <= alu_out;
        end
    end

    /* CSR */
    wire [2:0] csr_op = ((stage_active[`STAGE_EXECUTE] | stage_active[`STAGE_MEMORY]) & decode_ex_csr_microcode[15]) ? decode_ex_csr_microcode[2:0] : 3'b011;
    wire [3:0] csr_trap_num = control_op_trap ? {1'b0, fault_num} : {control_op_ext_int, 3'b011};
    wire [11:0] csr_addr_exception /* verilator public */ = control_op_normal ? decode_ex_csr_microcode[14:3] :
        {7'b0, control_op_ext_int | control_op_sw_int, csr_trap_num};
    wire [31:0] csr_in /* verilator public */ = (decode_csr_mux_position == 2'b00) ? rf_read_data_a : (
        (decode_csr_mux_position == 2'b01) ? decode_imm : (
        (decode_csr_mux_position == 2'b10) ? (~control_op_normal ? pc_pc : pc_next_pc) :
        32'b0));
    wire [31:0] csr_read_value /* verilator public */;
    wire csr_fault;
    wire csr_ext_int_pending;
    wire csr_sw_int_pending;
    csr csr_module(
        .clk(clk),
        .reset_n(reset_n),
        .enable_n(~(stage_active[`STAGE_EXECUTE] | stage_active[`STAGE_MEMORY])),
        .is_write_stage(stage_active[`STAGE_MEMORY] & decode_ex_csr_microcode[15]),
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
