`include "cells/and2.sv"
`include "cells/dff.sv"
`include "cells/dffe.sv"
`include "cells/mux2.sv"
`include "cells/nor2.sv"
`include "cells/or2.sv"

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
    input logic [2:0] op, // (3'b000=Exception, 3'b001=MRET, 3'b011=Nop, 3'b101=CSRRW, 3'b110=CSRRS, 3'b111=CSRRC)
    input logic [11:0] addr_exception, // Address / exception value
    input logic [31:0] write_value, // Write value
    output logic [31:0] read_value, // Read value
    output logic ext_int_pending, // External interrupt pending
    output logic sw_int_pending, // Software interrupt pending
    output logic fault // Fault condition
);

    /* Address Logic */
    wire addr_exception_is_mstatus = addr_exception == 12'h300;
    wire addr_exception_is_mie = addr_exception == 12'h304;
    wire addr_exception_is_mepc = addr_exception == 12'h341;
    wire addr_exception_is_mcause = addr_exception == 12'h342;
    wire addr_exception_is_mip = addr_exception == 12'h344;
    wire addr_exception_is_valid = addr_exception_is_mstatus | addr_exception_is_mie | addr_exception_is_mepc | addr_exception_is_mcause | addr_exception_is_mip;

    /* Op decode */
    wire op_is_csr = op[2] & (op[1] | op[0]);
    wire op_is_exception = ~op[2] & ~op[1] & ~op[0];
    wire op_is_mret = ~op[2] & ~op[1] & op[0];
    wire op_is_csr_rw = op_is_csr & ~op[1] & op[0];
    wire op_is_csr_rc = op_is_csr & op[1] & op[0];
    wire op_is_nop = ~op[2] & op[1] & op[0];
    wire op_is_write_mie = is_write_stage & op_is_csr & addr_exception_is_mie;
    wire op_is_write_mip = is_write_stage & op_is_csr & addr_exception_is_mip;
    wire op_is_write_mepc = is_write_stage & op_is_csr & addr_exception_is_mepc;
    wire op_is_write_mcause = is_write_stage & op_is_csr & addr_exception_is_mcause;
    wire op_is_write_mstatus = is_write_stage & op_is_csr & addr_exception_is_mstatus;
    wire op_is_write_exception = is_write_stage & op_is_exception;
    wire op_is_write_mret = is_write_stage & op_is_mret;
    wire op_is_valid_read_write = op_is_exception | op_is_mret | op_is_nop | (op_is_csr & addr_exception_is_valid);

    /* State */
    logic reg_ints_enabled;
    logic temp_reg_ints_enabled;
    logic prior_reg_ints_enabled;
    logic reg_ext_int_enabled;
    logic reg_ext_int_pending;
    logic reg_sw_int_enabled;
    logic reg_sw_int_pending;
    logic [31:0] exception_pc;
    logic [3:0] reg_exception_code;
    logic reg_trap_is_int;

    /* Write Logic */
    wire [31:0] write_value_or_mask; // = {32{~op_is_csr_rc}} & write_value;
    and2 #(.BITS(32)) write_value_or_mask_and(
        .a({32{~op_is_csr_rc}}),
        .b(write_value),
        .out(write_value_or_mask)
    );
    wire [31:0] write_value_and_mask; // = {32{~op_is_csr_rw}} & ~write_value;
    nor2 #(.BITS(32)) write_value_and_mask_nor(
        .a({32{op_is_csr_rw}}),
        .b(write_value),
        .out(write_value_and_mask)
    );
    wire write_value_ext_int_enabled = write_value_or_mask[11] | (write_value_and_mask[11] & reg_ext_int_enabled);
    wire write_value_ext_int_pending = write_value_or_mask[11] | (write_value_and_mask[11] & reg_ext_int_pending);
    wire write_value_sw_int_enabled = write_value_or_mask[3] | (write_value_and_mask[3] & reg_sw_int_enabled);
    wire write_value_sw_int_pending = write_value_or_mask[3] | (write_value_and_mask[3] & reg_sw_int_pending);
    wire [3:0] write_value_exception_code;
    wire [3:0] write_value_exception_code_intermediate;
    and2 #(.BITS(4)) write_value_exception_code_and(
        .a(write_value_and_mask[3:0]),
        .b(reg_exception_code),
        .out(write_value_exception_code_intermediate)
    );
    or2 #(.BITS(4)) write_value_exception_code_or(
        .a(write_value_or_mask[3:0]),
        .b(write_value_exception_code_intermediate),
        .out(write_value_exception_code)
    );
    wire write_value_trap_is_int = write_value_or_mask[31] | (write_value_and_mask[31] & reg_trap_is_int);
    wire write_value_ints_enabled = write_value_or_mask[3] | (write_value_and_mask[3] & reg_ints_enabled);
    wire write_value_prior_ints_enabled = write_value_or_mask[7] | (write_value_and_mask[7] & prior_reg_ints_enabled);
    wire [29:0] write_value_exception_pc;
    wire [29:0] write_value_exception_pc_intermediate;
    and2 #(.BITS(30)) write_value_exception_pc_and(
        .a(write_value_and_mask[31:2]),
        .b(exception_pc[31:2]),
        .out(write_value_exception_pc_intermediate)
    );
    or2 #(.BITS(30)) write_value_exception_pc_or(
        .a(write_value_or_mask[31:2]),
        .b(write_value_exception_pc_intermediate),
        .out(write_value_exception_pc)
    );

    /* Read Value */
    wire [31:0] csr_read_mstatus_mie;
    mux2 #(.BITS(32)) csr_read_mstatus_mie_mux(
        .a({24'b0, prior_reg_ints_enabled, 3'b0, reg_ints_enabled, 3'b0}),
        .b({20'b0, reg_ext_int_enabled, 7'b0, reg_sw_int_enabled, 3'b0}),
        .select(addr_exception_is_mie),
        .out(csr_read_mstatus_mie)
    );
    wire [31:0] csr_read_mepc_mcause;
    mux2 #(.BITS(32)) csr_read_mepc_mcause_mux(
        .a(exception_pc),
        .b({reg_trap_is_int, 27'b0, reg_exception_code}),
        .select(addr_exception_is_mcause),
        .out(csr_read_mepc_mcause)
    );
    wire [31:0] csr_read_mstatus_mie_mepc_mcause;
    mux2 #(.BITS(32)) csr_read_mstatus_mie_mepc_mcause_mux(
        .a(csr_read_mstatus_mie),
        .b(csr_read_mepc_mcause),
        .select(addr_exception_is_mepc | addr_exception_is_mcause),
        .out(csr_read_mstatus_mie_mepc_mcause)
    );
    wire [31:0] csr_read;
    mux2 #(.BITS(32)) csr_read_mux(
        .a(csr_read_mstatus_mie_mepc_mcause),
        .b({20'b0, reg_ext_int_pending, 7'b0, reg_sw_int_pending, 3'b0}),
        .select(addr_exception_is_mip),
        .out(csr_read)
    );
    wire [31:0] exception_read_value;
    mux2 #(.BITS(32)) exception_read_value_mux(
        .a(IRQ_HANDLER_ADDR),
        .b(exception_pc),
        .select(op_is_mret),
        .out(exception_read_value)
    );
    wire [31:0] next_read_value;
    mux2 #(.BITS(32)) next_read_value_mux(
        .a(exception_read_value),
        .b(csr_read),
        .select(op_is_csr & addr_exception_is_valid),
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

    /* Write Exception PC */
    wire [29:0] next_exception_pc_upper;
    mux2 #(.BITS(30)) next_exception_pc_mux(
        .a(write_value[31:2]),
        .b(write_value_exception_pc),
        .select(op_is_write_mepc),
        .out(next_exception_pc_upper)
    );
    dffe #(.BITS(32)) exception_pc_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n | ~(op_is_write_exception | op_is_write_mepc)),
        .in({next_exception_pc_upper, 2'b0}),
        .out(exception_pc)
    );

    /* Interrupts Enabled */
    wire ints_enabled_mstatus_mret;
    mux2 ints_enabled_mstatus_mret_mux(
        .a(write_value_ints_enabled),
        .b(temp_reg_ints_enabled),
        .select(op_is_write_mret),
        .out(ints_enabled_mstatus_mret)
    );
    dffe reg_ints_enabled_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n | ~(op_is_write_exception | op_is_write_mret | op_is_write_mstatus)),
        .in(~op_is_write_exception & ints_enabled_mstatus_mret),
        .out(reg_ints_enabled)
    );
    wire next_temp_reg_ints_enabled;
    mux2 next_temp_reg_ints_enabled_mux(
        .a(prior_reg_ints_enabled),
        .b(reg_ints_enabled),
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
    wire next_prior_reg_ints_enabled;
    mux2 next_prior_reg_ints_enabled_mux(
        .a(temp_reg_ints_enabled),
        .b(write_value_prior_ints_enabled),
        .select(op_is_write_mstatus),
        .out(next_prior_reg_ints_enabled)
    );
    dffe next_prior_reg_ints_enabled_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n | (~op_is_write_exception & ~op_is_write_mstatus)),
        .in(next_prior_reg_ints_enabled),
        .out(prior_reg_ints_enabled)
    );

    /* External Interrupt */
    dff ext_int_pending_dff(
        .clk(clk),
        .clear_n(reset_n),
        .in(reg_ext_int_pending & reg_ext_int_enabled & reg_ints_enabled),
        .out(ext_int_pending)
    );
    dffe reg_ext_int_enabled_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n | ~op_is_write_mie),
        .in(write_value_ext_int_enabled),
        .out(reg_ext_int_enabled)
    );
    wire clear_ext_int = ~enable_n & op_is_write_exception & (addr_exception[4:0] == 5'b11011);
    wire set_ext_int = ~enable_n & op_is_write_mip;
    dffe reg_ext_int_pending_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(~ext_int & ~clear_ext_int & ~set_ext_int),
        .in(ext_int | (set_ext_int & ~clear_ext_int & write_value_ext_int_pending)),
        .out(reg_ext_int_pending)
    );

    /* SW Interrupt */
    dff sw_int_pending_dff(
        .clk(clk),
        .clear_n(reset_n),
        .in(reg_sw_int_pending & reg_sw_int_enabled & reg_ints_enabled),
        .out(sw_int_pending)
    );
    dffe reg_sw_int_enabled_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n | ~op_is_write_mie),
        .in(write_value_sw_int_enabled),
        .out(reg_sw_int_enabled)
    );
    wire clear_sw_int = ~enable_n & op_is_write_exception & reg_sw_int_pending & (addr_exception[4:0] == 5'b10011);
    wire set_sw_int = ~enable_n & op_is_write_mip;
    dffe reg_sw_int_pending_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(~clear_sw_int & ~set_sw_int),
        .in(~clear_sw_int & write_value_sw_int_pending),
        .out(reg_sw_int_pending)
    );

    /* Exception / Trap */
    wire [3:0] next_reg_exception_code;
    mux2 #(.BITS(4)) next_reg_exception_code_mux(
        .a(write_value_exception_code),
        .b(addr_exception[3:0]),
        .select(op_is_write_exception),
        .out(next_reg_exception_code)
    );
    dffe #(.BITS(4)) reg_exception_code_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n | (~op_is_write_exception & ~op_is_write_mcause)),
        .in(next_reg_exception_code),
        .out(reg_exception_code)
    );
    wire next_reg_trap_is_int;
    mux2 next_reg_trap_is_int_mux(
        .a(write_value_trap_is_int),
        .b(addr_exception[4]),
        .select(op_is_write_exception),
        .out(next_reg_trap_is_int)
    );
    dffe reg_trap_is_int_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n | (~op_is_write_exception & ~op_is_write_mcause)),
        .in(next_reg_trap_is_int),
        .out(reg_trap_is_int)
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
                assert(exception_pc == 32'b0);
                assert(!reg_ints_enabled);
                assert(!prior_reg_ints_enabled);
                assert(!reg_ext_int_enabled);
                assert(!reg_ext_int_pending);
                assert(!reg_sw_int_enabled);
                assert(!reg_sw_int_pending);
                assert(reg_exception_code == 4'b0);
                assert(!reg_trap_is_int);
            end else if ($past(reset_n, 2)) begin
                assume($past(~enable_n));
                if ($past(ext_int)) begin
                    // the external interrupt was asserted, so should be pending
                    assert(reg_ext_int_pending);
                end
                if ($past(reg_ext_int_pending) && $past(reg_ext_int_enabled) && $past(reg_ints_enabled)) begin
                    assert(ext_int_pending);
                end else begin
                    assert(~ext_int_pending);
                end
                if ($past(reg_sw_int_pending) && $past(reg_sw_int_enabled) && $past(reg_ints_enabled)) begin
                    assert(sw_int_pending);
                end else begin
                    assert(~sw_int_pending);
                end
                if ($past(op) == 3'b000) begin
                    // exception
                    assert(!fault);
                    if (!$past(is_write_stage)) begin
                        // read stage
                        assert(read_value[31:2] == IRQ_HANDLER_ADDR[31:2]);
                        assert(temp_reg_ints_enabled == $past(reg_ints_enabled));
                    end else begin
                        // write stage
                        assert(reg_exception_code == $past(addr_exception[3:0]));
                        assert(reg_trap_is_int == $past(addr_exception[4]));
                        assert(exception_pc[31:2] == $past(write_value[31:2]));
                        assert(exception_pc[1:0] == 2'b0);
                        assert(!reg_ints_enabled);
                        assert(prior_reg_ints_enabled == $past(temp_reg_ints_enabled));
                        if ($past(addr_exception[4:0]) == 5'b10011) begin
                            assert(!reg_sw_int_pending);
                        end else begin
                            assert(reg_sw_int_pending == $past(reg_sw_int_pending));
                        end
                    end
                end else if ($past(op) == 3'b001) begin
                    // MRET
                    assert(!fault);
                    if (!$past(is_write_stage)) begin
                        // read stage
                        assert(read_value == $past(exception_pc));
                        assert(temp_reg_ints_enabled == $past(prior_reg_ints_enabled));
                    end else begin
                        // write stage
                        assert(reg_ints_enabled == $past(temp_reg_ints_enabled));
                        assert(exception_pc == $past(exception_pc));
                    end
                end else if ($past(op) == 3'b011) begin
                    // No-Op
                    assert(!fault);
                end else if ($past(op[2]) && ($past(op[1:0]) != 2'b0)) begin
                    // CSR* instruction
                    case ($past(addr_exception))
                        12'h300: begin // mstatus
                            assert(!fault);
                            if (!$past(is_write_stage)) begin
                                // read stage
                                assert(read_value == {28'b0, $past(prior_reg_ints_enabled), 3'b0, $past(reg_ints_enabled), 3'b0});
                            end else begin
                                // write stage
                                if ($past(op) == 3'b101)
                                    assert(reg_ints_enabled == $past(write_value[3]));
                                else if ($past(op) == 3'b110)
                                    assert(reg_ints_enabled == ($past(reg_ints_enabled) | $past(write_value[3])));
                                else if ($past(op) == 3'b111)
                                    assert(reg_ints_enabled == ($past(reg_ints_enabled) & ~$past(write_value[3])));
                                if ($past(op) == 3'b101)
                                    assert(prior_reg_ints_enabled == $past(write_value[7]));
                                else if ($past(op) == 3'b110)
                                    assert(prior_reg_ints_enabled == ($past(prior_reg_ints_enabled) | $past(write_value[7])));
                                else if ($past(op) == 3'b111)
                                    assert(prior_reg_ints_enabled == ($past(prior_reg_ints_enabled) & ~$past(write_value[7])));
                            end
                        end
                        12'h304: begin // mie
                            assert(!fault);
                            if (!$past(is_write_stage)) begin
                                // read stage
                                assert(read_value == {20'b0, $past(reg_ext_int_enabled), 7'b0, $past(reg_sw_int_enabled), 3'b0});
                            end else begin
                                // write stage
                                if ($past(op) == 3'b101)
                                    assert(reg_ext_int_enabled == $past(write_value[11]));
                                else if ($past(op) == 3'b110)
                                    assert(reg_ext_int_enabled == ($past(reg_ext_int_enabled) | $past(write_value[11])));
                                else if ($past(op) == 3'b111)
                                    assert(reg_ext_int_enabled == ($past(reg_ext_int_enabled) & ~$past(write_value[11])));
                                if ($past(op) == 3'b101)
                                    assert(reg_sw_int_enabled == $past(write_value[3]));
                                else if ($past(op) == 3'b110)
                                    assert(reg_sw_int_enabled == ($past(reg_sw_int_enabled) | $past(write_value[3])));
                                else if ($past(op) == 3'b111)
                                    assert(reg_sw_int_enabled == ($past(reg_sw_int_enabled) & ~$past(write_value[3])));
                            end
                        end
                        12'h341: begin // mepc
                            assert(!fault);
                            if (!$past(is_write_stage)) begin
                                // read stage
                                assert(read_value == $past(exception_pc));
                            end else begin
                                // write stage
                                if ($past(op) == 3'b101)
                                    assert(exception_pc[31:2] == $past(write_value[31:2]));
                                else if ($past(op) == 3'b110)
                                    assert(exception_pc[31:2] == ($past(exception_pc[31:2]) | $past(write_value[31:2])));
                                else if ($past(op) == 3'b111)
                                    assert(exception_pc[31:2] == ($past(exception_pc[31:2]) & ~$past(write_value[31:2])));
                            end
                        end
                        12'h342: begin // mcause
                            assert(!fault);
                            if (!$past(is_write_stage)) begin
                                // read stage
                                assert(read_value == {$past(reg_trap_is_int), 27'b0, $past(reg_exception_code)});
                            end else begin
                                // write stage
                                if ($past(op) == 3'b101)
                                    assert(reg_exception_code == $past(write_value[3:0]));
                                else if ($past(op) == 3'b110)
                                    assert(reg_exception_code == ($past(reg_exception_code[3:0]) | $past(write_value[3:0])));
                                else if ($past(op) == 3'b111)
                                    assert(reg_exception_code == ($past(reg_exception_code[3:0]) & ~$past(write_value[3:0])));
                            end
                        end
                        12'h344: begin // mip
                            assert(!fault);
                            if (!$past(is_write_stage)) begin
                                // read stage
                                assert(read_value == {20'b0, $past(reg_ext_int_pending), 7'b0, $past(reg_sw_int_pending), 3'b0});
                            end else begin
                                // write stage
                                if ($past(op) == 3'b101)
                                    assert(reg_sw_int_pending == $past(write_value[3]));
                                else if ($past(op) == 3'b110)
                                    assert(reg_sw_int_pending == ($past(reg_sw_int_pending) | $past(write_value[3])));
                                else if ($past(op) == 3'b111)
                                    assert(reg_sw_int_pending == ($past(reg_sw_int_pending) & ~$past(write_value[3])));
                            end
                        end
                        default: begin
                            assert(fault);
                        end
                    endcase
                end else begin
                    assert(fault);
                end
            end
        end
    end
`endif

endmodule
