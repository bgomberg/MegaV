`include "constants/csr_op.sv"
`include "cells/and2.sv"
`include "cells/dff.sv"
`include "cells/dffe.sv"
`include "cells/and4.sv"
`include "cells/mux2.sv"
`include "cells/mux4.sv"
`include "cells/mux8.sv"
`include "cells/nand8.sv"
`include "cells/nor2.sv"
`include "cells/nor4.sv"
`include "cells/or2.sv"
`include "cells/xor3.sv"

`define MSTATUS_BIT_INTS_ENABLED                3
`define MSTATUS_BIT_PRIOR_INTS_ENABLED          7
`define MIE_BIT_EXT_INT_ENABLED                 11
`define MIP_BIT_EXT_INT_PENDING                 11
`define MIE_BIT_SW_INT_ENABLED                  3
`define MIP_BIT_SW_INT_PENDING                  3
`define MCAUSE_BIT_IS_INT                       31

/*
 * RISC-V machine-level CSRs.
 */
module csr #(
    parameter [31:0] IRQ_HANDLER_ADDR = 32'h00000010
) (
    input logic clk, // Clock signal
    input logic reset_n, // Reset signal (active low)
    input logic enable_n, // Enable (active low)
    input logic is_write_stage, // Perform the write portion of the operation
    input logic ext_int, // External interrupt
    input logic [2:0] op, // Operation
    input logic [11:0] addr_exception, // Address / exception value
    input logic [31:0] write_value, // Write value
    output logic [31:0] read_value, // Read value
    output logic ext_int_pending, // External interrupt pending
    output logic sw_int_pending, // Software interrupt pending
    output logic fault // Fault condition
);

    /* Address Decode */
    wire addr_exception_zero_check1;
    nor4 addr_exception_zero_check1_nor(
        .a(addr_exception[11]),
        .b(addr_exception[10]),
        .c(addr_exception[7]),
        .d(addr_exception[5]),
        .out(addr_exception_zero_check1)
    );
    wire addr_exception_zero_check2;
    nor2 addr_exception_zero_check2_nor(
        .a(addr_exception[4]),
        .b(addr_exception[3]),
        .out(addr_exception_zero_check2)
    );
    wire addr_exception_is_valid_unused_bits;
    and4 addr_exception_is_valid_unused_bits_and(
        .a(addr_exception_zero_check1),
        .b(addr_exception_zero_check2),
        .c(addr_exception[9]),
        .d(addr_exception[8]),
        .out(addr_exception_is_valid_unused_bits)
    );
    wire addr_exception_bits_6_1_0_zero_check;
    nor4 addr_exception_bits_6_1_0_zero_check_nor(
        .a(addr_exception[6]),
        .b(addr_exception[1]),
        .c(addr_exception[0]),
        .d(1'b0),
        .out(addr_exception_bits_6_1_0_zero_check)
    );
    wire addr_exception_lower_3_odd_parity_check;
    xor3 addr_exception_lower_3_odd_parity_check_xor(
        .a(addr_exception[2]),
        .b(addr_exception[1]),
        .c(addr_exception[0]),
        .out(addr_exception_lower_3_odd_parity_check)
    );
    wire addr_exception_lower_3_not_all_one_check;
    nand8 addr_exception_lower_3_not_all_one_check_nand(
        .a(addr_exception[2]),
        .b(addr_exception[1]),
        .c(addr_exception[0]),
        .d(1'b1),
        .e(1'b1),
        .f(1'b1),
        .g(1'b1),
        .h(1'b1),
        .out(addr_exception_lower_3_not_all_one_check)
    );
    wire addr_exception_is_valid = addr_exception_is_valid_unused_bits & (
        addr_exception_bits_6_1_0_zero_check |
        (addr_exception[6] & addr_exception_lower_3_odd_parity_check & addr_exception_lower_3_not_all_one_check)
    );
    wire [2:0] mapped_csr_reg_address = {addr_exception[6], addr_exception[2] | addr_exception[1], addr_exception[2] | addr_exception[0]};
    wire addr_exception_is_trap = addr_exception[4];
    wire addr_exception_is_trap_sw_int = addr_exception_is_trap & (addr_exception[3:0] == 4'b0011);
    wire addr_exception_is_trap_ext_int = addr_exception_is_trap & (addr_exception[3:0] == 4'b1011);

    /* Op Decode */
    wire op_is_csr = op[2] & (op[1] | op[0]);
    wire op_is_exception = ~op[2] & ~op[1] & ~op[0];
    wire op_is_mret = ~op[2] & ~op[1] & op[0];
    wire op_is_csr_rw = op_is_csr & ~op[1] & op[0];
    wire op_is_csr_rc = op_is_csr & op[1] & op[0];
    wire op_is_nop = ~op[2] & op[1] & op[0];
    wire op_is_write_mie = is_write_stage & op_is_csr & (mapped_csr_reg_address == 3'b011);
    wire op_is_write_mip = is_write_stage & op_is_csr & (mapped_csr_reg_address == 3'b111);
    wire op_is_write_mepc = is_write_stage & op_is_csr & (mapped_csr_reg_address == 3'b101);
    wire op_is_write_mcause = is_write_stage & op_is_csr & (mapped_csr_reg_address == 3'b110);
    wire op_is_write_mstatus = is_write_stage & op_is_csr & (mapped_csr_reg_address == 3'b000);
    wire op_is_write_exception = is_write_stage & op_is_exception;
    wire op_is_write_mret = is_write_stage & op_is_mret;
    wire op_is_valid_read_write = op_is_exception | op_is_mret | op_is_nop | (op_is_csr & addr_exception_is_valid);

    /* Registers */
    /* verilator lint_off UNUSED */
    logic [31:0] reg_mstatus;
    logic [31:0] reg_mepc;
    logic [31:0] reg_mcause;
    logic [31:0] reg_mie;
    logic [31:0] reg_mip;
    /* verilator lint_on UNUSED */
    // Zero out unused register fields
    assign reg_mip[31:12] = 20'b0;
    assign reg_mip[10:4] = 7'b0;
    assign reg_mip[2:0] = 3'b0;

    logic temp_reg_ints_enabled;

    /* Read Value */
    wire [2:0] next_read_value_mux_select;
    mux4 #(.BITS(3)) next_read_value_mux_select_mux(
        .d1(mapped_csr_reg_address),
        .d2(3'b101),
        .d3(3'b010),
        .d4(3'b0), // Unused
        .select({op_is_exception, op_is_mret}),
        .out(next_read_value_mux_select)
    );
    wire [31:0] next_read_value;
    mux8 #(.BITS(32)) next_read_value_mux(
        .d1(reg_mstatus),
        .d2(32'b0), // Unused
        .d3(IRQ_HANDLER_ADDR),
        .d4(reg_mie),
        .d5(32'b0), // Unused
        .d6(reg_mepc),
        .d7(reg_mcause),
        .d8(reg_mip),
        .select(next_read_value_mux_select),
        .out(next_read_value)
    );
    wire update_read_value = ~is_write_stage & (op_is_exception | op_is_mret | (op_is_csr & addr_exception_is_valid));
    dffe #(.BITS(32)) read_value_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n | ~update_read_value),
        .in(next_read_value),
        .out(read_value)
    );

    /* Read + Write Value */
    wire [31:0] write_value_or_mask;
    and2 #(.BITS(32)) write_value_or_mask_and(
        .a({32{~op_is_csr_rc}}),
        .b(write_value),
        .out(write_value_or_mask)
    );
    wire [31:0] write_value_and_mask;
    nor2 #(.BITS(32)) write_value_and_mask_nor(
        .a({32{op_is_csr_rw}}),
        .b(write_value),
        .out(write_value_and_mask)
    );
    wire [31:0] read_write_value_intermediate;
    and2 #(.BITS(32)) read_write_value_and(
        .a(next_read_value),
        .b(write_value_and_mask),
        .out(read_write_value_intermediate)
    );
    wire [31:0] read_write_value;
    or2 #(.BITS(32)) read_write_value_or(
        .a(read_write_value_intermediate),
        .b(write_value_or_mask),
        .out(read_write_value)
    );

    /* Write mepc Register */
    wire [29:0] next_mepc_upper;
    mux2 #(.BITS(30)) next_mepc_upper_mux(
        .a(write_value[31:2]),
        .b(read_write_value[31:2]),
        .select(op_is_write_mepc),
        .out(next_mepc_upper)
    );
    dffe #(.BITS(32)) mepc_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n | ~(op_is_write_exception | op_is_write_mepc)),
        .in({next_mepc_upper, 2'b0}),
        .out(reg_mepc)
    );

    /*
        Temp Ints Enabled
            [Read Stage, Exception Op]: temp_reg_ints_enabled = mstatus[INTS_ENABLED]
            [Read Stage, MRET Op]: temp_reg_ints_enabled = mstatus[PRIOR_INTS_ENABLED]
    */
    wire next_temp_reg_ints_enabled;
    mux2 next_temp_reg_ints_enabled_mux(
        .a(reg_mstatus[`MSTATUS_BIT_PRIOR_INTS_ENABLED]),
        .b(reg_mstatus[`MSTATUS_BIT_INTS_ENABLED]),
        .select(op_is_exception),
        .out(next_temp_reg_ints_enabled)
    );
    dffe temp_reg_ints_enabled_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n | is_write_stage | ~(op_is_exception | op_is_mret)),
        .in(next_temp_reg_ints_enabled),
        .out(temp_reg_ints_enabled)
    );

    /* Write mstatus Register */
    wire next_ints_enabled;
    mux4 next_ints_enabled_mux(
        .d1(read_write_value[`MSTATUS_BIT_INTS_ENABLED]),
        .d2(temp_reg_ints_enabled),
        .d3(1'b0),
        .d4(1'b0), // Unused
        .select({op_is_write_exception, op_is_write_mret}),
        .out(next_ints_enabled)
    );
    wire next_prior_reg_ints_enabled;
    mux4 next_prior_reg_ints_enabled_mux(
        .d1(temp_reg_ints_enabled),
        .d2(read_write_value[`MSTATUS_BIT_PRIOR_INTS_ENABLED]),
        .d3(reg_mstatus[`MSTATUS_BIT_PRIOR_INTS_ENABLED]),
        .d4(1'b0), // Unused
        .select({op_is_write_mret, op_is_write_mstatus}),
        .out(next_prior_reg_ints_enabled)
    );
    dffe #(.BITS(32)) reg_mstatus_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n | ~(op_is_write_exception | op_is_write_mret | op_is_write_mstatus)),
        .in({24'b0, next_prior_reg_ints_enabled, 3'b0, next_ints_enabled, 3'b0}),
        .out(reg_mstatus)
    );

    /* Write mie Register */
    dffe #(.BITS(32)) reg_mie_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n | ~op_is_write_mie),
        .in({20'b0, read_write_value[`MIE_BIT_EXT_INT_ENABLED], 7'b0, read_write_value[`MIE_BIT_SW_INT_ENABLED], 3'b0}),
        .out(reg_mie)
    );

    /* External Interrupt */
    dff ext_int_pending_dff(
        .clk(clk),
        .clear_n(reset_n),
        .in(reg_mip[`MIP_BIT_EXT_INT_PENDING] & reg_mie[`MIE_BIT_EXT_INT_ENABLED] & reg_mstatus[`MSTATUS_BIT_INTS_ENABLED]),
        .out(ext_int_pending)
    );
    wire clear_ext_int = ~enable_n & op_is_write_exception & addr_exception_is_trap_ext_int;
    wire set_ext_int = ~enable_n & op_is_write_mip;
    wire next_ext_int_pending = ext_int | (set_ext_int & ~clear_ext_int & read_write_value[`MIP_BIT_EXT_INT_PENDING]);
    dffe reg_ext_int_pending_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(~ext_int & ~clear_ext_int & ~set_ext_int),
        .in(next_ext_int_pending),
        .out(reg_mip[`MIP_BIT_EXT_INT_PENDING])
    );

    /* SW Interrupt */
    dff sw_int_pending_dff(
        .clk(clk),
        .clear_n(reset_n),
        .in(reg_mip[`MIP_BIT_SW_INT_PENDING] & reg_mie[`MIE_BIT_SW_INT_ENABLED] & reg_mstatus[`MSTATUS_BIT_INTS_ENABLED]),
        .out(sw_int_pending)
    );
    wire clear_sw_int = ~enable_n & op_is_write_exception & reg_mip[`MIP_BIT_SW_INT_PENDING] & addr_exception_is_trap_sw_int;
    wire set_sw_int = ~enable_n & op_is_write_mip;
    wire next_sw_int_pending = ~clear_sw_int & read_write_value[`MIP_BIT_SW_INT_PENDING];
    dffe reg_sw_int_pending_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(~clear_sw_int & ~set_sw_int),
        .in(next_sw_int_pending),
        .out(reg_mip[`MIP_BIT_SW_INT_PENDING])
    );

    /* Write mcause Register */
    wire [31:0] next_mcause;
    mux2 #(.BITS(32)) next_mcause_mux(
        .a({read_write_value[`MCAUSE_BIT_IS_INT], 27'b0, read_write_value[3:0]}),
        .b({addr_exception[4], 27'b0, addr_exception[3:0]}),
        .select(op_is_write_exception),
        .out(next_mcause)
    );
    dffe #(.BITS(32)) reg_mcause_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n | ~(op_is_write_exception | op_is_write_mcause)),
        .in(next_mcause),
        .out(reg_mcause)
    );

    /* Fault */
    dffe fault_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(~op_is_valid_read_write),
        .out(fault)
    );

`ifdef FORMAL
    initial assume(~reset_n);
    logic f_past_valid;
    initial f_past_valid = 1'b0;
    always_ff @(posedge clk) begin
        f_past_valid = 1;
    end

    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            if (!$past(reset_n)) begin
                assert(!fault);
                assert(!ext_int_pending);
                assert(!sw_int_pending);
                assert(reg_mepc == 32'b0);
                assert(!reg_mstatus[`MSTATUS_BIT_INTS_ENABLED]);
                assert(!reg_mstatus[`MSTATUS_BIT_PRIOR_INTS_ENABLED]);
                assert(!reg_mie[`MIE_BIT_EXT_INT_ENABLED]);
                assert(!reg_mip[`MIP_BIT_EXT_INT_PENDING]);
                assert(!reg_mie[`MIE_BIT_SW_INT_ENABLED]);
                assert(!reg_mip[`MIP_BIT_SW_INT_PENDING]);
                assert(reg_mcause[3:0] == 4'b0);
                assert(!reg_mcause[`MCAUSE_BIT_IS_INT]);
            end else if ($past(reset_n, 2)) begin
                assume($past(~enable_n));
                if ($past(ext_int)) begin
                    // the external interrupt was asserted, so should be pending
                    assert(reg_mip[`MIP_BIT_EXT_INT_PENDING]);
                end
                if ($past(reg_mip[`MIP_BIT_EXT_INT_PENDING]) && $past(reg_mie[`MIE_BIT_EXT_INT_ENABLED]) && $past(reg_mstatus[`MSTATUS_BIT_INTS_ENABLED])) begin
                    assert(ext_int_pending);
                end else begin
                    assert(~ext_int_pending);
                end
                if ($past(reg_mip[`MIP_BIT_SW_INT_PENDING]) && $past(reg_mie[`MIE_BIT_SW_INT_ENABLED]) && $past(reg_mstatus[`MSTATUS_BIT_INTS_ENABLED])) begin
                    assert(sw_int_pending);
                end else begin
                    assert(~sw_int_pending);
                end
                if ($past(is_write_stage)) begin
                    // Write state
                    case ($past(op))
                        `CSR_OP_EXCEPTION: begin
                            assert(!fault);
                            assert(reg_mcause[3:0] == $past(addr_exception[3:0]));
                            assert(reg_mcause[`MCAUSE_BIT_IS_INT] == $past(addr_exception[4]));
                            assert(reg_mepc[31:2] == $past(write_value[31:2]));
                            assert(reg_mepc[1:0] == 2'b0);
                            assert(!reg_mstatus[`MSTATUS_BIT_INTS_ENABLED]);
                            assert(reg_mstatus[`MSTATUS_BIT_PRIOR_INTS_ENABLED] == $past(temp_reg_ints_enabled));
                            if ($past(addr_exception[4:0]) == 5'b10011) begin
                                assert(!reg_mip[`MIP_BIT_SW_INT_PENDING]);
                            end else begin
                                assert(reg_mip[`MIP_BIT_SW_INT_PENDING] == $past(reg_mip[`MIP_BIT_SW_INT_PENDING]));
                            end
                        end
                        `CSR_OP_MRET: begin
                            assert(!fault);
                            assert(reg_mstatus[`MSTATUS_BIT_INTS_ENABLED] == $past(temp_reg_ints_enabled));
                            assert(reg_mepc == $past(reg_mepc));
                        end
                        `CSR_OP_NO_OP: begin
                            assert(!fault);
                        end
                        `CSR_OP_CSRRW: begin
                            case ($past(addr_exception))
                                12'h300: begin // mstatus
                                    assert(!fault);
                                    assert(reg_mstatus[`MSTATUS_BIT_INTS_ENABLED] == $past(write_value[`MSTATUS_BIT_INTS_ENABLED]));
                                    assert(reg_mstatus[`MSTATUS_BIT_PRIOR_INTS_ENABLED] == $past(write_value[`MSTATUS_BIT_PRIOR_INTS_ENABLED]));
                                end
                                12'h304: begin // mie
                                    assert(!fault);
                                    assert(reg_mie[`MIE_BIT_EXT_INT_ENABLED] == $past(write_value[`MIE_BIT_EXT_INT_ENABLED]));
                                    assert(reg_mie[`MIE_BIT_SW_INT_ENABLED] == $past(write_value[`MIE_BIT_SW_INT_ENABLED]));
                                end
                                12'h341: begin // mepc
                                    assert(!fault);
                                    assert(reg_mepc[31:2] == $past(write_value[31:2]));
                                end
                                12'h342: begin // mcause
                                    assert(!fault);
                                    assert(reg_mcause[3:0] == $past(write_value[3:0]));
                                end
                                12'h344: begin // mip
                                    assert(!fault);
                                    assert(reg_mip[`MIP_BIT_SW_INT_PENDING] == $past(write_value[`MIP_BIT_SW_INT_PENDING]));
                                end
                                default:
                                    assert(fault);
                            endcase
                        end
                        `CSR_OP_CSRRS: begin
                            case ($past(addr_exception))
                                12'h300: begin // mstatus
                                    assert(!fault);
                                    assert(reg_mstatus[`MSTATUS_BIT_INTS_ENABLED] == ($past(reg_mstatus[`MSTATUS_BIT_INTS_ENABLED]) | $past(write_value[`MSTATUS_BIT_INTS_ENABLED])));
                                    assert(reg_mstatus[`MSTATUS_BIT_PRIOR_INTS_ENABLED] == ($past(reg_mstatus[`MSTATUS_BIT_PRIOR_INTS_ENABLED]) | $past(write_value[`MSTATUS_BIT_PRIOR_INTS_ENABLED])));
                                end
                                12'h304: begin // mie
                                    assert(!fault);
                                    assert(reg_mie[`MIE_BIT_EXT_INT_ENABLED] == ($past(reg_mie[`MIE_BIT_EXT_INT_ENABLED]) | $past(write_value[`MIE_BIT_EXT_INT_ENABLED])));
                                    assert(reg_mie[`MIE_BIT_SW_INT_ENABLED] == ($past(reg_mie[`MIE_BIT_SW_INT_ENABLED]) | $past(write_value[`MIE_BIT_SW_INT_ENABLED])));
                                end
                                12'h341: begin // mepc
                                    assert(!fault);
                                    assert(reg_mepc[31:2] == ($past(reg_mepc[31:2]) | $past(write_value[31:2])));
                                end
                                12'h342: begin // mcause
                                    assert(!fault);
                                    assert(reg_mcause[3:0] == ($past(reg_mcause[3:0]) | $past(write_value[3:0])));
                                end
                                12'h344: begin // mip
                                    assert(!fault);
                                    assert(reg_mip[`MIP_BIT_SW_INT_PENDING] == ($past(reg_mip[`MIP_BIT_SW_INT_PENDING]) | $past(write_value[`MIP_BIT_SW_INT_PENDING])));
                                end
                                default:
                                    assert(fault);
                            endcase
                        end
                        `CSR_OP_CSRRC: begin
                            case ($past(addr_exception))
                                12'h300: begin // mstatus
                                    assert(!fault);
                                    assert(reg_mstatus[`MSTATUS_BIT_INTS_ENABLED] == ($past(reg_mstatus[`MSTATUS_BIT_INTS_ENABLED]) & ~$past(write_value[`MSTATUS_BIT_INTS_ENABLED])));
                                    assert(reg_mstatus[`MSTATUS_BIT_PRIOR_INTS_ENABLED] == ($past(reg_mstatus[`MSTATUS_BIT_PRIOR_INTS_ENABLED]) & ~$past(write_value[`MSTATUS_BIT_PRIOR_INTS_ENABLED])));
                                end
                                12'h304: begin // mie
                                    assert(!fault);
                                    assert(reg_mie[`MIE_BIT_EXT_INT_ENABLED] == ($past(reg_mie[`MIE_BIT_EXT_INT_ENABLED]) & ~$past(write_value[`MIE_BIT_EXT_INT_ENABLED])));
                                    assert(reg_mie[`MIE_BIT_SW_INT_ENABLED] == ($past(reg_mie[`MIE_BIT_SW_INT_ENABLED]) & ~$past(write_value[`MIE_BIT_SW_INT_ENABLED])));
                                end
                                12'h341: begin // mepc
                                    assert(!fault);
                                    assert(reg_mepc[31:2] == ($past(reg_mepc[31:2]) & ~$past(write_value[31:2])));
                                end
                                12'h342: begin // mcause
                                    assert(!fault);
                                    assert(reg_mcause[3:0] == ($past(reg_mcause[3:0]) & ~$past(write_value[3:0])));
                                end
                                12'h344: begin // mip
                                    assert(!fault);
                                    assert(reg_mip[`MIP_BIT_SW_INT_PENDING] == ($past(reg_mip[`MIP_BIT_SW_INT_PENDING]) & ~$past(write_value[`MIP_BIT_SW_INT_PENDING])));
                                end
                                default:
                                    assert(fault);
                            endcase
                        end
                        default:
                            assert(fault);
                    endcase
                end else begin
                    // Read stage
                    case ($past(op))
                        `CSR_OP_EXCEPTION: begin
                            assert(!fault);
                            assert(read_value[31:2] == IRQ_HANDLER_ADDR[31:2]);
                            assert(temp_reg_ints_enabled == $past(reg_mstatus[`MSTATUS_BIT_INTS_ENABLED]));
                        end
                        `CSR_OP_MRET: begin
                            assert(!fault);
                            assert(read_value == $past(reg_mepc));
                            assert(temp_reg_ints_enabled == $past(reg_mstatus[`MSTATUS_BIT_PRIOR_INTS_ENABLED]));
                        end
                        `CSR_OP_NO_OP: begin
                            assert(!fault);
                        end
                        `CSR_OP_CSRRW, `CSR_OP_CSRRS, `CSR_OP_CSRRC: begin
                            case ($past(addr_exception))
                                12'h300: begin // mstatus
                                    assert(!fault);
                                    // TODO: Figure out why this fails
                                    // assert(read_value[31:8] == 24'b0);
                                    // assert(read_value[6:4] == 3'b0);
                                    // assert(read_value[2:0] == 3'b0);
                                    assert(read_value == $past(reg_mstatus));
                                end
                                12'h304: begin // mie
                                    assert(!fault);
                                    // TODO: Figure out why this fails
                                    // assert(read_value[31:12] == 20'b0);
                                    // assert(read_value[10:4] == 7'b0);
                                    // assert(read_value[2:0] == 3'b0);
                                    assert(read_value == $past(reg_mie));
                                end
                                12'h341: begin // mepc
                                    assert(!fault);
                                    assert(read_value == $past(reg_mepc));
                                end
                                12'h342: begin // mcause
                                    assert(!fault);
                                    // TODO: Figure out why this fails
                                    // assert(read_value[30:4] == 27'b0);
                                    assert(read_value == $past(reg_mcause));
                                end
                                12'h344: begin // mip
                                    assert(!fault);
                                    // TODO: Figure out why this fails
                                    // assert(read_value[31:12] == 20'b0);
                                    // assert(read_value[10:4] == 7'b0);
                                    // assert(read_value[2:0] == 3'b0);
                                    assert(read_value == $past(reg_mip));
                                end
                                default:
                                    assert(fault);
                            endcase
                        end
                        default:
                            assert(fault);
                    endcase
                end
            end
        end
    end
`endif

endmodule
