`define STATE_IDLE          2'b00
`define STATE_IN_PROGRESS   2'b01
`define STATE_DONE          2'b10

/*
 * RISC-V machine-level CSRs.
 */
module csr #(
    parameter [31:0] IRQ_HANDLER_ADDR = 32'h00000010
) (
    input clk, // Clock signal
    input reset_n, // Reset signal (active low)
    input available, // Operation available
    input [2:0] op, // (3'b000=Exception, 3'b001=MRET, 3'b101=CSRRW, 3'b110=CSRRS, 3'b111=CSRRC)
    input [11:0] addr_exception, // Address / exception value
    input [31:0] write_value, // Write value
    output [31:0] read_value, // Read value
    output busy, // Operation busy
    output fault // Fault condition
);

    /* Outputs */
    reg [31:0] read_value;
    reg busy;
    reg fault;

    /* Op decode */
    wire op_is_exception = ~op[2] & ~op[1] & ~op[0];
    wire op_is_mret = ~op[2] & ~op[1] & op[0];
    wire op_is_rw = op[2] & ~op[1] & op[0];
    wire op_is_rs = op[2] & op[1] & ~op[0];
    wire op_is_rc = op[2] & op[1] & op[0];
    wire op_is_write = op_is_rw | (op_is_rs && (write_value != 32'b0)) | (op_is_rc && (write_value != 32'b0));

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
    reg [1:0] state;
    always @(posedge clk) begin
        if (~reset_n) begin
            interrupts_enabled <= 1'b0;
            prior_interrupts_enabled <= 1'b0;
            external_interrupt_enabled <= 1'b0;
            external_interrupt_pending <= 1'b0;
            software_interrupt_enabled <= 1'b0;
            software_interrupt_pending <= 1'b0;
            exception_pc <= 32'b0;
            exception_code <= 4'b0;
            trap_is_interrupt <= 1'b0;
            read_value <= 32'b0;
            busy <= 1'b0;
            state <= `STATE_IDLE;
            fault <= 1'b0;
        end else begin
            case (state)
                `STATE_IDLE: begin
                    if (available) begin
                        // Starting a new operation
                        busy <= 1'b1;
                        state <= `STATE_IN_PROGRESS;
                    end else begin
                        busy <= 1'b0;
                    end
                end
                `STATE_IN_PROGRESS: begin
                    if (op_is_exception) begin
                        // exception
                        exception_code <= addr_exception[3:0];
                        trap_is_interrupt <= addr_exception[4];
                        exception_pc <= write_value;
                        read_value <= {IRQ_HANDLER_ADDR[31:2], 2'b0};
                        prior_interrupts_enabled <= interrupts_enabled;
                        interrupts_enabled <= 1'b0;
                        fault <= 1'b0;
                    end else if (op_is_mret) begin
                        // MRET
                        read_value <= exception_pc;
                        interrupts_enabled <= prior_interrupts_enabled;
                        fault <= 1'b0;
                    end else if (op_is_rw | op_is_rs | op_is_rc) begin
                        // RW/RS/RC
                        case (addr_exception)
                            12'h300: begin // mstatus
                                read_value <= {
                                    24'b0, // reserved / unsupported
                                    prior_interrupts_enabled, // MPIE (prior interrupt enable)
                                    3'b0, // reserved / unsupported
                                    interrupts_enabled, // MIE (interrupt enable)
                                    3'b0 // reserved / unsupported
                                };
                                interrupts_enabled <= (~op_is_rc & write_value[3]) | (~op_is_rw & ~write_value[3] & interrupts_enabled);
                                prior_interrupts_enabled <= (~op_is_rc & write_value[7]) | (~op_is_rw & ~write_value[7] & prior_interrupts_enabled);
                                fault <= 1'b0;
                            end
                            12'h304: begin // mie
                                read_value <= {
                                    20'b0, // reserved
                                    external_interrupt_enabled,
                                    7'b0, // reserved / unsupported
                                    software_interrupt_enabled,
                                    3'b0 // reserved / unsupported
                                };
                                external_interrupt_enabled <= (~op_is_rc & write_value[11]) | (~op_is_rw & ~write_value[11] & external_interrupt_enabled);
                                software_interrupt_enabled <= (~op_is_rc & write_value[3]) | (~op_is_rw & ~write_value[3] & software_interrupt_enabled);
                                fault <= 1'b0;
                            end
                            12'h305: begin // mtvec
                                read_value <= {
                                    IRQ_HANDLER_ADDR[31:2], // base
                                    2'b0 // mode (direct)
                                };
                                fault <= op_is_write;
                            end
                            12'h341: begin // mepc
                                read_value <= exception_pc;
                                fault <= op_is_write;
                            end
                            12'h342: begin // mcause
                                read_value <= {trap_is_interrupt, 27'b0, exception_code};
                                fault <= op_is_write;
                            end
                            12'h344: begin // mip
                                read_value <= {
                                    20'b0, // reserved
                                    external_interrupt_pending,
                                    7'b0, // reserved / unsupported
                                    software_interrupt_pending,
                                    3'b0 // reserved / unsupported
                                };
                                software_interrupt_pending <= (~op_is_rc & write_value[3]) | (~op_is_rw & ~write_value[3] & software_interrupt_pending);
                                fault <= 1'b0;
                            end
                            default: begin
                                read_value <= 32'b0;
                                fault <= 1'b1;
                            end
                        endcase
                    end else begin
                        fault <= 1'b1;
                    end
                    // Operation is complete
                    busy <= 1'b0;
                    state <= `STATE_DONE;
                end
                `STATE_DONE: begin
                    if (!available) begin
                        busy <= 1'b0;
                        state <= `STATE_IDLE;
                    end else begin
                        busy <= 1'b1;
                    end
                    fault <= 1'b0;
                end
                default: begin
                    // should never get here
                end
            endcase
        end
    end

`ifdef FORMAL
    initial assume(~reset_n);
    initial interrupts_enabled = 1'b0;
    initial prior_interrupts_enabled = 1'b0;
    initial external_interrupt_enabled = 1'b0;
    initial external_interrupt_pending = 1'b0;
    initial software_interrupt_enabled = 1'b0;
    initial software_interrupt_pending = 1'b0;
    initial exception_pc = 32'b0;
    initial exception_code = 4'b0;
    initial trap_is_interrupt = 1'b0;

    reg f_past_valid;
    initial f_past_valid = 1'b0;
    always @(posedge clk) begin
        f_past_valid = 1;
    end

    always @(posedge clk) begin
        if (f_past_valid && !$past(~reset_n) && $past(available) && !busy) begin
            assume(state != 2'b11);
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
                    12'h305: begin // mtvec
                        assert(read_value == {IRQ_HANDLER_ADDR[31:2], 2'b0});
                        if ($past(op) == 3'b101)
                            assert(fault);
                        else if ($past(op) == 3'b110)
                            assert(fault == ($past(write_value) != 0));
                        else if ($past(op) == 3'b111)
                            assert(fault == ($past(write_value) != 0));
                        else
                            assert(0);
                    end
                    12'h341: begin // mepc
                        assert(read_value == $past(exception_pc));
                        assert(exception_pc == $past(exception_pc));
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
                        assert(read_value == {$past(trap_is_interrupt), 27'b0, $past(exception_code)});
                        assert(exception_code == $past(exception_code));
                        assert(trap_is_interrupt == $past(trap_is_interrupt));
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
                        // This bit cannot be change by software
                        assert(external_interrupt_pending == $past(external_interrupt_pending));
                        assert(!fault);
                    end
                    default: begin
                        assert(read_value == 32'b0);
                        assert(fault);
                    end
                endcase
            end else begin
                assert(fault);
            end
        end
    end
`endif

endmodule
