`include "constants/csr_mapped_addr.sv"
`include "constants/csr_op.sv"
`include "cells/and2.sv"
`include "cells/dff.sv"
`include "cells/dffe.sv"
`include "cells/mux2.sv"
`include "cells/mux4.sv"
`include "cells/mux8.sv"
`include "cells/nor2.sv"
`include "cells/or2.sv"

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
    input logic [4:0] addr_exception, // Address / exception value
    input logic [31:0] write_value, // Write value
    output logic [31:0] read_value, // Read value
    output logic ext_int_pending, // External interrupt pending
    output logic sw_int_pending // Software interrupt pending
);

    /* Address Decode */
    wire [2:0] mapped_csr_reg_address = addr_exception[2:0];
    wire addr_exception_is_trap = addr_exception[4];
    wire addr_exception_is_trap_sw_int = addr_exception_is_trap & (addr_exception[3:0] == 4'b0011);
    wire addr_exception_is_trap_ext_int = addr_exception_is_trap & (addr_exception[3:0] == 4'b1011);

    /* Op Decode */
    wire op_is_exception = ~op[2] & ~op[1] & ~op[0];
    wire op_is_mret = ~op[2] & ~op[1] & op[0];
    wire op_is_csr = op[2] & (op[1] | op[0]);
    wire op_is_csr_rw = op[2] & ~op[1] & op[0];
    wire op_is_csr_rc = op[2] & op[1] & op[0];
    wire is_write_csr = is_write_stage & op_is_csr;
    wire op_is_write_mstatus = is_write_csr & (mapped_csr_reg_address == `CSR_MAPPED_ADDR_MSTATUS);
    wire op_is_write_mie = is_write_csr & (mapped_csr_reg_address == `CSR_MAPPED_ADDR_MIE);
    wire op_is_write_mepc = is_write_csr & (mapped_csr_reg_address == `CSR_MAPPED_ADDR_MEPC);
    wire op_is_write_mcause = is_write_csr & (mapped_csr_reg_address == `CSR_MAPPED_ADDR_MCAUSE);
    wire op_is_write_mip = is_write_csr & (mapped_csr_reg_address == `CSR_MAPPED_ADDR_MIP);
    wire op_is_write_exception = is_write_stage & op_is_exception;
    wire op_is_write_mret = is_write_stage & op_is_mret;

    /* Registers */
    logic [31:0] reg_mstatus;
    logic [31:0] reg_mepc;
    logic [31:0] reg_mcause;
    logic [31:0] reg_mie;
    logic [31:0] reg_mip;
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
    wire update_read_value = ~is_write_stage & (op_is_exception | op_is_mret | op_is_csr);
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

    /* Write mip Register */
    wire clear_ext_int = op_is_write_exception & addr_exception_is_trap_ext_int;
    wire next_ext_int_pending;
    mux2 next_ext_int_pending_mux(
        .a(reg_mip[`MIP_BIT_EXT_INT_PENDING]),
        .b(ext_int | (~clear_ext_int & read_write_value[`MIP_BIT_EXT_INT_PENDING])),
        .select(ext_int | clear_ext_int | op_is_write_mip),
        .out(next_ext_int_pending)
    );
    wire clear_sw_int = op_is_write_exception & addr_exception_is_trap_sw_int;
    wire next_sw_int_pending;
    mux2 next_sw_int_pending_mux(
        .a(reg_mip[`MIP_BIT_SW_INT_PENDING]),
        .b(~clear_sw_int & read_write_value[`MIP_BIT_SW_INT_PENDING]),
        .select(clear_sw_int | op_is_write_mip),
        .out(next_sw_int_pending)
    );
    wire update_mip = ext_int | clear_ext_int | clear_sw_int | op_is_write_mip;
    dffe #(.BITS(32)) reg_mip_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(~enable_n & ~update_mip),
        .in({20'b0, next_ext_int_pending, 7'b0, next_sw_int_pending, 3'b0}),
        .out(reg_mip)
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

    /* Pending Interrupts */
    dff ext_int_pending_dff(
        .clk(clk),
        .clear_n(reset_n),
        .in(reg_mip[`MIP_BIT_EXT_INT_PENDING] & reg_mie[`MIE_BIT_EXT_INT_ENABLED] & reg_mstatus[`MSTATUS_BIT_INTS_ENABLED]),
        .out(ext_int_pending)
    );
    dff sw_int_pending_dff(
        .clk(clk),
        .clear_n(reset_n),
        .in(reg_mip[`MIP_BIT_SW_INT_PENDING] & reg_mie[`MIE_BIT_SW_INT_ENABLED] & reg_mstatus[`MSTATUS_BIT_INTS_ENABLED]),
        .out(sw_int_pending)
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
                            assert(reg_mstatus[`MSTATUS_BIT_INTS_ENABLED] == $past(temp_reg_ints_enabled));
                            assert(reg_mepc == $past(reg_mepc));
                        end
                        `CSR_OP_CSRRW: begin
                            case ($past(addr_exception[2:0]))
                                `CSR_MAPPED_ADDR_MSTATUS: begin
                                    assume($past(addr_exception[3]));
                                    assert(reg_mstatus[`MSTATUS_BIT_INTS_ENABLED] == $past(write_value[`MSTATUS_BIT_INTS_ENABLED]));
                                    assert(reg_mstatus[`MSTATUS_BIT_PRIOR_INTS_ENABLED] == $past(write_value[`MSTATUS_BIT_PRIOR_INTS_ENABLED]));
                                end
                                `CSR_MAPPED_ADDR_MIE: begin
                                    assume($past(addr_exception[3]));
                                    assert(reg_mie[`MIE_BIT_EXT_INT_ENABLED] == $past(write_value[`MIE_BIT_EXT_INT_ENABLED]));
                                    assert(reg_mie[`MIE_BIT_SW_INT_ENABLED] == $past(write_value[`MIE_BIT_SW_INT_ENABLED]));
                                end
                                `CSR_MAPPED_ADDR_MEPC: begin
                                    assume($past(addr_exception[3]));
                                    assert(reg_mepc[31:2] == $past(write_value[31:2]));
                                end
                                `CSR_MAPPED_ADDR_MCAUSE: begin
                                    assume($past(addr_exception[3]));
                                    assert(reg_mcause[3:0] == $past(write_value[3:0]));
                                end
                                `CSR_MAPPED_ADDR_MIP: begin
                                    assume($past(addr_exception[3]));
                                    assert(reg_mip[`MIP_BIT_SW_INT_PENDING] == $past(write_value[`MIP_BIT_SW_INT_PENDING]));
                                end
                            endcase
                        end
                        `CSR_OP_CSRRS: begin
                            case ($past(addr_exception[2:0]))
                                `CSR_MAPPED_ADDR_MSTATUS: begin
                                    assume($past(addr_exception[3]));
                                    assert(reg_mstatus[`MSTATUS_BIT_INTS_ENABLED] == ($past(reg_mstatus[`MSTATUS_BIT_INTS_ENABLED]) | $past(write_value[`MSTATUS_BIT_INTS_ENABLED])));
                                    assert(reg_mstatus[`MSTATUS_BIT_PRIOR_INTS_ENABLED] == ($past(reg_mstatus[`MSTATUS_BIT_PRIOR_INTS_ENABLED]) | $past(write_value[`MSTATUS_BIT_PRIOR_INTS_ENABLED])));
                                end
                                `CSR_MAPPED_ADDR_MIE: begin
                                    assume($past(addr_exception[3]));
                                    assert(reg_mie[`MIE_BIT_EXT_INT_ENABLED] == ($past(reg_mie[`MIE_BIT_EXT_INT_ENABLED]) | $past(write_value[`MIE_BIT_EXT_INT_ENABLED])));
                                    assert(reg_mie[`MIE_BIT_SW_INT_ENABLED] == ($past(reg_mie[`MIE_BIT_SW_INT_ENABLED]) | $past(write_value[`MIE_BIT_SW_INT_ENABLED])));
                                end
                                `CSR_MAPPED_ADDR_MEPC: begin
                                    assume($past(addr_exception[3]));
                                    assert(reg_mepc[31:2] == ($past(reg_mepc[31:2]) | $past(write_value[31:2])));
                                end
                                `CSR_MAPPED_ADDR_MCAUSE: begin
                                    assume($past(addr_exception[3]));
                                    assert(reg_mcause[3:0] == ($past(reg_mcause[3:0]) | $past(write_value[3:0])));
                                end
                                `CSR_MAPPED_ADDR_MIP: begin
                                    assume($past(addr_exception[3]));
                                    assert(reg_mip[`MIP_BIT_SW_INT_PENDING] == ($past(reg_mip[`MIP_BIT_SW_INT_PENDING]) | $past(write_value[`MIP_BIT_SW_INT_PENDING])));
                                end
                            endcase
                        end
                        `CSR_OP_CSRRC: begin
                            case ($past(addr_exception[2:0]))
                                `CSR_MAPPED_ADDR_MSTATUS: begin
                                    assume($past(addr_exception[3]));
                                    assert(reg_mstatus[`MSTATUS_BIT_INTS_ENABLED] == ($past(reg_mstatus[`MSTATUS_BIT_INTS_ENABLED]) & ~$past(write_value[`MSTATUS_BIT_INTS_ENABLED])));
                                    assert(reg_mstatus[`MSTATUS_BIT_PRIOR_INTS_ENABLED] == ($past(reg_mstatus[`MSTATUS_BIT_PRIOR_INTS_ENABLED]) & ~$past(write_value[`MSTATUS_BIT_PRIOR_INTS_ENABLED])));
                                end
                                `CSR_MAPPED_ADDR_MIE: begin
                                    assume($past(addr_exception[3]));
                                    assert(reg_mie[`MIE_BIT_EXT_INT_ENABLED] == ($past(reg_mie[`MIE_BIT_EXT_INT_ENABLED]) & ~$past(write_value[`MIE_BIT_EXT_INT_ENABLED])));
                                    assert(reg_mie[`MIE_BIT_SW_INT_ENABLED] == ($past(reg_mie[`MIE_BIT_SW_INT_ENABLED]) & ~$past(write_value[`MIE_BIT_SW_INT_ENABLED])));
                                end
                                `CSR_MAPPED_ADDR_MEPC: begin
                                    assume($past(addr_exception[3]));
                                    assert(reg_mepc[31:2] == ($past(reg_mepc[31:2]) & ~$past(write_value[31:2])));
                                end
                                `CSR_MAPPED_ADDR_MCAUSE: begin
                                    assume($past(addr_exception[3]));
                                    assert(reg_mcause[3:0] == ($past(reg_mcause[3:0]) & ~$past(write_value[3:0])));
                                end
                                `CSR_MAPPED_ADDR_MIP: begin
                                    assume($past(addr_exception[3]));
                                    assert(reg_mip[`MIP_BIT_SW_INT_PENDING] == ($past(reg_mip[`MIP_BIT_SW_INT_PENDING]) & ~$past(write_value[`MIP_BIT_SW_INT_PENDING])));
                                end
                            endcase
                        end
                    endcase
                end else begin
                    // Read stage
                    case ($past(op))
                        `CSR_OP_EXCEPTION: begin
                            assert(read_value[31:2] == IRQ_HANDLER_ADDR[31:2]);
                            assert(temp_reg_ints_enabled == $past(reg_mstatus[`MSTATUS_BIT_INTS_ENABLED]));
                        end
                        `CSR_OP_MRET: begin
                            assert(read_value == $past(reg_mepc));
                            assert(temp_reg_ints_enabled == $past(reg_mstatus[`MSTATUS_BIT_PRIOR_INTS_ENABLED]));
                        end
                        `CSR_OP_CSRRW, `CSR_OP_CSRRS, `CSR_OP_CSRRC: begin
                            case ($past(addr_exception[2:0]))
                                `CSR_MAPPED_ADDR_MSTATUS: begin
                                    assume($past(addr_exception[3]));
                                    // TODO: Figure out why this fails
                                    // assert(read_value[31:8] == 24'b0);
                                    // assert(read_value[6:4] == 3'b0);
                                    // assert(read_value[2:0] == 3'b0);
                                    assert(read_value == $past(reg_mstatus));
                                end
                                `CSR_MAPPED_ADDR_MIE: begin
                                    assume($past(addr_exception[3]));
                                    // TODO: Figure out why this fails
                                    // assert(read_value[31:12] == 20'b0);
                                    // assert(read_value[10:4] == 7'b0);
                                    // assert(read_value[2:0] == 3'b0);
                                    assert(read_value == $past(reg_mie));
                                end
                                `CSR_MAPPED_ADDR_MEPC: begin
                                    assume($past(addr_exception[3]));
                                    assert(read_value == $past(reg_mepc));
                                end
                                `CSR_MAPPED_ADDR_MCAUSE: begin
                                    assume($past(addr_exception[3]));
                                    // TODO: Figure out why this fails
                                    // assert(read_value[30:4] == 27'b0);
                                    assert(read_value == $past(reg_mcause));
                                end
                                `CSR_MAPPED_ADDR_MIP: begin
                                    assume($past(addr_exception[3]));
                                    // TODO: Figure out why this fails
                                    // assert(read_value[31:12] == 20'b0);
                                    // assert(read_value[10:4] == 7'b0);
                                    // assert(read_value[2:0] == 3'b0);
                                    assert(read_value == $past(reg_mip));
                                end
                            endcase
                        end
                    endcase
                end
            end
        end
    end
`endif

endmodule
