/*
 * RISC-V machine-level CSRs.
 */
module csr #(
    parameter [31:0] IRQ_HANDLER_ADDR = 32'h00000010
) (
    input clk, // Clock signal
    input reset_n, // Reset signal (active low)
    input available, // Operation available
    input ext_int, // External interrupt (ignores the available input)
    input [2:0] op, // (3'b000=Exception, 3'b001=MRET, 3'b101=CSRRW, 3'b110=CSRRS, 3'b111=CSRRC)
    input [11:0] addr_exception, // Address / exception value
    input [31:0] write_value, // Write value
    output [31:0] read_value, // Read value
    output ext_int_pending, // External interrupt pending
    output sw_int_pending, // Software interrupt pending
    output busy, // Operation busy
    output fault // Fault condition
);

    /* Outputs */
    reg [31:0] read_value;
    reg ext_int_pending;
    reg sw_int_pending;
    reg busy;
    reg fault;

    /* Op decode */
    wire op_is_csr = op[2];
    wire op_is_exception = ~op_is_csr & ~op[1] & ~op[0];
    wire op_is_mret = ~op_is_csr & ~op[1] & op[0];
    wire op_is_csr_rw = op_is_csr & ~op[1] & op[0];
    wire op_is_csr_rs = op_is_csr & op[1] & ~op[0];
    wire op_is_csr_rc = op_is_csr & op[1] & op[0];
    wire op_is_invalid = (~op_is_csr & op[1]) | (op_is_csr & ~op[1] & ~op[0]);
    wire write_value_non_zero = write_value != 32'b0;
    wire op_is_write = op_is_csr_rw | (op_is_csr_rs & write_value_non_zero) | (op_is_csr_rc & write_value_non_zero);
    wire csr_addr_is_invalid = (addr_exception[11:7] != 5'b00110) | (addr_exception[5:3] != 3'b000) |
        (~addr_exception[6] & (addr_exception[1] | addr_exception[0])) |
        (addr_exception[6] & addr_exception[2] & (addr_exception[1] | addr_exception[0])) |
        (addr_exception[6] & ~addr_exception[2] & ~(addr_exception[1] ^ addr_exception[0]));
    wire csr_is_read_only = addr_exception[6] & ~addr_exception[2];
    wire csr_access_error = csr_addr_is_invalid | (csr_is_read_only & op_is_write);
    wire op_is_valid_csr = (op_is_csr_rw | op_is_csr_rs | op_is_csr_rc) & ~csr_access_error;
    wire op_is_csr_mcause = op_is_valid_csr & addr_exception[6] & addr_exception[1];
    wire op_is_csr_mip_mstatus_mie = op_is_valid_csr & (~addr_exception[6] | addr_exception[2]);
    wire op_is_csr_mip = op_is_csr_mip_mstatus_mie & addr_exception[6];
    wire op_is_csr_mstatus = op_is_csr_mip_mstatus_mie & ~addr_exception[6] & ~addr_exception[2];
    wire op_is_csr_mie = op_is_csr_mip_mstatus_mie & ~addr_exception[6] & addr_exception[2];

    /* State */
    reg interrupts_enabled;
    reg prior_interrupts_enabled;
    reg external_interrupt_enabled;
    reg external_interrupt_pending;
    reg software_interrupt_enabled;
    reg software_interrupt_pending;
    reg [31:0] exception_pc;
    reg [3:0] exception_code;
    reg trap_is_interrupt;

    /* Logic */
    reg ext_int_received;
    reg started;
    wire should_perform_op = busy & started;
    wire [31:0] csr_mip_mstatus_mie_value = {
        20'b0,
        addr_exception[6] ? external_interrupt_pending : (addr_exception[2] & external_interrupt_enabled),
        3'b0,
        ~addr_exception[6] & ~addr_exception[2] & prior_interrupts_enabled,
        3'b0,
        addr_exception[6] ? software_interrupt_pending : (addr_exception[2] ? software_interrupt_enabled : interrupts_enabled),
        3'b0
    };
    wire [31:0] read_value_value = op_is_exception ? IRQ_HANDLER_ADDR : (
        (op_is_csr_mcause) ? {trap_is_interrupt, 27'b0, exception_code} : (
        (op_is_csr_mip_mstatus_mie) ? csr_mip_mstatus_mie_value : exception_pc));
    wire prior_interrupts_enabled_write_value = ((~op_is_csr_rc & write_value[7]) | (~op_is_csr_rw & ~write_value[7] & prior_interrupts_enabled));
    wire interrupts_enabled_write_value = (~op_is_csr_rc & write_value[3]) | (~op_is_csr_rw & ~write_value[3] & interrupts_enabled);
    wire external_interrupt_enabled_value = (~op_is_csr_rc & write_value[11]) | (~op_is_csr_rw & ~write_value[11] & external_interrupt_enabled);
    wire software_interrupt_enabled_value = (~op_is_csr_rc & write_value[3]) | (~op_is_csr_rw & ~write_value[3] & software_interrupt_enabled);
    wire software_interrupt_pending_value = (~op_is_csr_rc & write_value[3]) | (~op_is_csr_rw & ~write_value[3] & software_interrupt_pending);
    wire handling_pending_external_interrupt = reset_n & should_perform_op & op_is_exception & external_interrupt_pending & (addr_exception[4:0] == 5'b11011);
    wire handling_pending_software_interrupt = reset_n & should_perform_op & op_is_exception & software_interrupt_pending & (addr_exception[4:0] == 5'b10011);
    always @(posedge clk) begin
        ext_int_pending <= reset_n & external_interrupt_pending & external_interrupt_enabled & interrupts_enabled;
        sw_int_pending <= reset_n & software_interrupt_pending & software_interrupt_enabled & interrupts_enabled;
        fault <= reset_n & busy & available & (op_is_invalid | (op_is_csr & csr_access_error));
        busy <= reset_n & ~started & available;
        started <= reset_n & (available | (started & busy));
        read_value <= should_perform_op ? read_value_value : read_value;
        prior_interrupts_enabled <= reset_n & ((should_perform_op & op_is_exception) ? interrupts_enabled : (
            (should_perform_op & op_is_csr_mstatus) ? prior_interrupts_enabled_write_value : prior_interrupts_enabled));
        exception_code <= (should_perform_op & op_is_exception) ? addr_exception[3:0] : ({4{reset_n}} & exception_code);
        interrupts_enabled <= reset_n & (should_perform_op ? (~op_is_exception & (op_is_mret ? prior_interrupts_enabled : (op_is_csr_mstatus ? interrupts_enabled_write_value : interrupts_enabled))) : interrupts_enabled);
        external_interrupt_enabled <= reset_n & ((should_perform_op & op_is_csr_mie) ? external_interrupt_enabled_value : external_interrupt_enabled);
        software_interrupt_enabled <= reset_n & ((should_perform_op & op_is_csr_mie) ? software_interrupt_enabled_value : software_interrupt_enabled);
        trap_is_interrupt <= reset_n & ((should_perform_op & op_is_exception) ? addr_exception[4] : trap_is_interrupt);
        ext_int_received <= reset_n & ext_int;
        external_interrupt_pending <= reset_n & ~handling_pending_external_interrupt & ((ext_int & ~ext_int_received) | external_interrupt_pending);
        software_interrupt_pending <= reset_n & ~handling_pending_software_interrupt & ((should_perform_op & op_is_csr_mip) ? software_interrupt_pending_value : software_interrupt_pending);
        exception_pc <= (reset_n & should_perform_op & op_is_exception) ? write_value : ({32{reset_n}} & exception_pc);
    end

`ifdef FORMAL
    initial assume(~reset_n);
    reg f_past_valid;
    initial f_past_valid = 1'b0;
    always @(posedge clk) begin
        f_past_valid = 1;
    end

    always @(posedge clk) begin
        if (f_past_valid) begin
            assume(started | ~busy);
            if (available) begin
                assume($stable(op));
                assume($stable(addr_exception));
                assume($stable(write_value));
            end
            if (busy) begin
                assume(available);
            end
            if ($past(~reset_n)) begin
                assert(!busy);
                assert(!fault);
                assert(!ext_int_pending);
                assert(!sw_int_pending);
            end else begin
                if ($past(reset_n, 2)) begin
                    if ($past(ext_int, 2) && !$past(ext_int, 3) && $past(external_interrupt_enabled) && $past(interrupts_enabled)) begin
                        assert(ext_int_pending);
                    end
                    if ($past(busy, 2) && !$past(busy) && $past(addr_exception, 2) == 12'h344 && $past(op[2], 2) && $past(op[1:0], 2) != 2'b0 && $past(software_interrupt_pending_value, 2) && $past(software_interrupt_enabled) && $past(interrupts_enabled)) begin
                        assert(sw_int_pending);
                    end
                    if ($past(op, 2) == 3'b000 && $past(busy, 2) && !$past(busy)) begin
                        // we executed an exception 2 cycles ago
                        if ($past(ext_int_pending, 2) && $past(addr_exception[4:0], 2) == 5'b11011) begin
                            assert(!ext_int_pending);
                        end
                        if ($past(sw_int_pending, 2) && $past(addr_exception[4:0], 2) == 5'b10011) begin
                            assert(!sw_int_pending);
                        end
                    end
                end
                if ($past(busy) & ~busy) begin
                    if ($past(op) == 3'b000) begin
                        // exception
                        assert(exception_code == $past(addr_exception[3:0]));
                        assert(trap_is_interrupt == $past(addr_exception[4]));
                        assert(exception_pc == $past(write_value));
                        assert(read_value[31:2] == IRQ_HANDLER_ADDR[31:2]);
                        assert(!interrupts_enabled);
                        assert(prior_interrupts_enabled == $past(interrupts_enabled));
                        assert(!fault);
                    end else if ($past(op) == 3'b001) begin
                        // MRET
                        assert(read_value == $past(exception_pc));
                        assert(interrupts_enabled == $past(prior_interrupts_enabled));
                        assert(!fault);
                    end else if ($past(op[2]) && ($past(op[1:0]) != 2'b0)) begin
                        // CSR* instruction
                        case ($past(addr_exception))
                            12'h300: begin // mstatus
                                assert(read_value == {28'b0, $past(prior_interrupts_enabled), 3'b0, $past(interrupts_enabled), 3'b0});
                                if ($past(op) == 3'b101)
                                    assert(interrupts_enabled == $past(write_value[3]));
                                else if ($past(op) == 3'b110)
                                    assert(interrupts_enabled == ($past(interrupts_enabled) | $past(write_value[3])));
                                else if ($past(op) == 3'b111)
                                    assert(interrupts_enabled == ($past(interrupts_enabled) & ~$past(write_value[3])));
                                if ($past(op) == 3'b101)
                                    assert(prior_interrupts_enabled == $past(write_value[7]));
                                else if ($past(op) == 3'b110)
                                    assert(prior_interrupts_enabled == ($past(prior_interrupts_enabled) | $past(write_value[7])));
                                else if ($past(op) == 3'b111)
                                    assert(prior_interrupts_enabled == ($past(prior_interrupts_enabled) & ~$past(write_value[7])));
                                assert(!fault);
                            end
                            12'h304: begin // mie
                                assert(read_value == {20'b0, $past(external_interrupt_enabled), 7'b0, $past(software_interrupt_enabled), 3'b0});
                                if ($past(op) == 3'b101)
                                    assert(external_interrupt_enabled == $past(write_value[11]));
                                else if ($past(op) == 3'b110)
                                    assert(external_interrupt_enabled == ($past(external_interrupt_enabled) | $past(write_value[11])));
                                else if ($past(op) == 3'b111)
                                    assert(external_interrupt_enabled == ($past(external_interrupt_enabled) & ~$past(write_value[11])));
                                if ($past(op) == 3'b101)
                                    assert(software_interrupt_enabled == $past(write_value[3]));
                                else if ($past(op) == 3'b110)
                                    assert(software_interrupt_enabled == ($past(software_interrupt_enabled) | $past(write_value[3])));
                                else if ($past(op) == 3'b111)
                                    assert(software_interrupt_enabled == ($past(software_interrupt_enabled) & ~$past(write_value[3])));
                                assert(!fault);
                            end
                            12'h341: begin // mepc
                                if (!fault) begin
                                    assert(read_value == $past(exception_pc));
                                    assert(exception_pc == $past(exception_pc));
                                end
                                if ($past(op) == 3'b101)
                                    assert(fault);
                                else if ($past(op) == 3'b110)
                                    assert(fault == ($past(write_value) != 0));
                                else if ($past(op) == 3'b111)
                                    assert(fault == ($past(write_value) != 0));
                                else
                                    assert(0);
                            end
                            12'h342: begin // mcause
                                if (!fault) begin
                                    assert(read_value == {$past(trap_is_interrupt), 27'b0, $past(exception_code)});
                                    assert(exception_code == $past(exception_code));
                                    assert(trap_is_interrupt == $past(trap_is_interrupt));
                                end
                                if ($past(op) == 3'b101)
                                    assert(fault);
                                else if ($past(op) == 3'b110)
                                    assert(fault == ($past(write_value) != 0));
                                else if ($past(op) == 3'b111)
                                    assert(fault == ($past(write_value) != 0));
                                else
                                    assert(0);
                            end
                            12'h344: begin // mip
                                assert(read_value == {20'b0, $past(external_interrupt_pending), 7'b0, $past(software_interrupt_pending), 3'b0});
                                if ($past(op) == 3'b101)
                                    assert(software_interrupt_pending == $past(write_value[3]));
                                else if ($past(op) == 3'b110)
                                    assert(software_interrupt_pending == ($past(software_interrupt_pending) | $past(write_value[3])));
                                else if ($past(op) == 3'b111)
                                    assert(software_interrupt_pending == ($past(software_interrupt_pending) & ~$past(write_value[3])));
                                assert(!fault);
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
    end
`endif

endmodule
