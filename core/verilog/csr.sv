/*
 * RISC-V machine-level CSRs.
 */
module csr #(
    parameter [31:0] IRQ_HANDLER_ADDR = 32'h00000010
) (
    input logic clk_i, // Clock signal
    input logic reset_n_i, // Reset signal (active low)
    input logic available_i, // Operation available
    input logic is_write_stage_i, // Perform the write portion of the operation
    input logic ext_int_i, // External interrupt
    input logic [2:0] op_i, // (3'b000=Exception, 3'b001=MRET, 3'b011=Nop, 3'b101=CSRRW, 3'b110=CSRRS, 3'b111=CSRRC)
    input logic [11:0] addr_exception_i, // Address / exception value
    input logic [31:0] write_value_i, // Write value
    output logic [31:0] read_value_o, // Read value
    output logic ext_int_pending_o, // External interrupt pending
    output logic sw_int_pending_o, // Software interrupt pending
    output logic fault_o // Fault condition
);

    /* Op decode */
    wire op_is_csr = op_i[2] & (op_i[1] | op_i[0]);
    wire op_is_exception = ~op_i[2] & ~op_i[1] & ~op_i[0];
    wire op_is_mret = ~op_i[2] & ~op_i[1] & op_i[0];
    wire op_is_csr_rw = op_is_csr & ~op_i[1] & op_i[0];
    wire op_is_csr_rc = op_is_csr & op_i[1] & op_i[0];
    wire op_is_nop = ~op_i[2] & op_i[1] & op_i[0];

    /* State */
    logic interrupts_enabled;
    logic prior_interrupts_enabled;
    logic external_interrupt_enabled;
    logic external_interrupt_pending;
    logic software_interrupt_enabled;
    logic software_interrupt_pending;
    logic [31:0] exception_pc;
    logic [3:0] exception_code;
    logic trap_is_interrupt;

    /* Logic */
    wire [31:0] write_value_or_mask = {32{~op_is_csr_rc}} & write_value_i;
    wire [31:0] write_value_and_mask = {32{~op_is_csr_rw}} & ~write_value_i;
    logic temp_interrupts_enabled;
    always_ff @(posedge clk_i) begin
        if (~reset_n_i) begin
            ext_int_pending_o <= 0;
            sw_int_pending_o <= 0;
            fault_o <= 0;
            interrupts_enabled <= 0;
            prior_interrupts_enabled <= 0;
            external_interrupt_enabled <= 0;
            external_interrupt_pending <= 0;
            software_interrupt_enabled <= 0;
            software_interrupt_pending <= 0;
            exception_pc <= 32'b0;
            exception_code <= 4'b0;
            trap_is_interrupt <= 0;
        end else begin
            ext_int_pending_o <= external_interrupt_pending & external_interrupt_enabled & interrupts_enabled;
            sw_int_pending_o <= software_interrupt_pending & software_interrupt_enabled & interrupts_enabled;
            fault_o <= 0;
            if (available_i) begin
                if (~is_write_stage_i) begin
                    // Read stage
                    if (op_is_exception) begin
                        // exception
                        read_value_o <= IRQ_HANDLER_ADDR;
                        temp_interrupts_enabled <= interrupts_enabled;
                    end else if (op_is_mret) begin
                        read_value_o <= exception_pc;
                        temp_interrupts_enabled <= prior_interrupts_enabled;
                    end else if (op_is_csr) begin
                        // CSR*
                        if (addr_exception_i == 12'h300) begin
                            // mstatus
                            read_value_o <= {24'b0, prior_interrupts_enabled, 3'b0, interrupts_enabled, 3'b0};
                        end else if (addr_exception_i == 12'h304) begin
                            // mie
                            read_value_o <= {20'b0, external_interrupt_enabled, 7'b0, software_interrupt_enabled, 3'b0};
                        end else if (addr_exception_i == 12'h341) begin
                            // mepc
                            read_value_o <= exception_pc;
                        end else if (addr_exception_i == 12'h342) begin
                            // mcause
                            read_value_o <= {trap_is_interrupt, 27'b0, exception_code};
                        end else if (addr_exception_i == 12'h344) begin
                            // mip
                            read_value_o <= {20'b0, external_interrupt_pending, 7'b0, software_interrupt_pending, 3'b0};
                        end else begin
                            // invalid register
                            fault_o <= 1;
                        end
                    end else if (op_is_nop) begin
                        // do nothing
                    end else begin
                        // invalid op
                        fault_o <= 1;
                    end
                end else begin
                    // Write stage
                    if (op_is_exception) begin
                        // exception
                        exception_code <= addr_exception_i[3:0];
                        prior_interrupts_enabled <= temp_interrupts_enabled;
                        interrupts_enabled <= 0;
                        trap_is_interrupt <= addr_exception_i[4];
                        exception_pc <= {write_value_i[31:2], 2'b0};
                        if (software_interrupt_pending & addr_exception_i[4:0] == 5'b10011) begin
                            // handling the pending software interrupt
                            software_interrupt_pending <= 0;
                        end else if (external_interrupt_pending & addr_exception_i[4:0] == 5'b11011) begin
                            // handling the pending external interrupt
                            external_interrupt_pending <= 0;
                        end
                    end else if (op_is_mret) begin
                        // MRET
                        interrupts_enabled <= temp_interrupts_enabled;
                    end else if (op_is_csr) begin
                        // CSR*
                        if (addr_exception_i == 12'h300) begin
                            // mstatus
                            prior_interrupts_enabled <= write_value_or_mask[7] | (write_value_and_mask[7] & prior_interrupts_enabled);
                            interrupts_enabled <= write_value_or_mask[3] | (write_value_and_mask[3] & interrupts_enabled);
                        end else if (addr_exception_i == 12'h304) begin
                            // mie
                            external_interrupt_enabled <= write_value_or_mask[11] | (write_value_and_mask[11] & external_interrupt_enabled);
                            software_interrupt_enabled <= write_value_or_mask[3] | (write_value_and_mask[3] & software_interrupt_enabled);
                        end else if (addr_exception_i == 12'h341) begin
                            // mepc
                            exception_pc <= {write_value_or_mask[31:2] | (write_value_and_mask[31:2] & exception_pc[31:2]), 2'b0};
                        end else if (addr_exception_i == 12'h342) begin
                            // mcause
                            trap_is_interrupt <= write_value_or_mask[31] | (write_value_and_mask[31] & trap_is_interrupt);
                            exception_code <= write_value_or_mask[3:0] | (write_value_and_mask[3:0] & exception_code);
                        end else if (addr_exception_i == 12'h344) begin
                            // mip
                            external_interrupt_pending <= write_value_or_mask[11] | (write_value_and_mask[11] & external_interrupt_pending);
                            software_interrupt_pending <= write_value_or_mask[3] | (write_value_and_mask[3] & software_interrupt_pending);
                        end else begin
                            // invalid register
                            fault_o <= 1;
                        end
                    end else if (op_is_nop) begin
                        // do nothing
                    end else begin
                        // invalid op
                        fault_o <= 1;
                    end
                end
            end
            if (ext_int_i)
                external_interrupt_pending <= 1;
        end
    end

`ifdef FORMAL
    initial assume(~reset_n_i);
    logic f_past_valid;
    initial f_past_valid = 1'b0;
    always_ff @(posedge clk_i) begin
        f_past_valid = 1;
    end

    always_ff @(posedge clk_i) begin
        if (f_past_valid) begin
            if (!$past(reset_n_i)) begin
                assert(!fault_o);
                assert(!ext_int_pending_o);
                assert(!sw_int_pending_o);
                assert(exception_pc == 32'b0);
                assert(!interrupts_enabled);
                assert(!prior_interrupts_enabled);
                assert(!external_interrupt_enabled);
                assert(!external_interrupt_pending);
                assert(!software_interrupt_enabled);
                assert(!software_interrupt_pending);
                assert(exception_code == 4'b0);
                assert(!trap_is_interrupt);
            end else if ($past(reset_n_i, 2)) begin
                assume($past(available_i));
                if ($past(ext_int_i)) begin
                    // the external interrupt was asserted, so should be pending
                    assert(external_interrupt_pending);
                end
                if ($past(external_interrupt_pending) && $past(external_interrupt_enabled) && $past(interrupts_enabled)) begin
                    assert(ext_int_pending_o);
                end else begin
                    assert(~ext_int_pending_o);
                end
                if ($past(software_interrupt_pending) && $past(software_interrupt_enabled) && $past(interrupts_enabled)) begin
                    assert(sw_int_pending_o);
                end else begin
                    assert(~sw_int_pending_o);
                end
                if ($past(op_i) == 3'b000) begin
                    // exception
                    assert(!fault_o);
                    if (!$past(is_write_stage_i)) begin
                        // read stage
                        assert(read_value_o[31:2] == IRQ_HANDLER_ADDR[31:2]);
                        assert(temp_interrupts_enabled == $past(interrupts_enabled));
                    end else begin
                        // write stage
                        assert(exception_code == $past(addr_exception_i[3:0]));
                        assert(trap_is_interrupt == $past(addr_exception_i[4]));
                        assert(exception_pc[31:2] == $past(write_value_i[31:2]));
                        assert(exception_pc[1:0] == 2'b0);
                        assert(!interrupts_enabled);
                        assert(prior_interrupts_enabled == $past(temp_interrupts_enabled));
                        if ($past(addr_exception_i[4:0]) == 5'b10011) begin
                            assert(!software_interrupt_pending);
                        end else begin
                            assert(software_interrupt_pending == $past(software_interrupt_pending));
                        end
                    end
                end else if ($past(op_i) == 3'b001) begin
                    // MRET
                    assert(!fault_o);
                    if (!$past(is_write_stage_i)) begin
                        // read stage
                        assert(read_value_o == $past(exception_pc));
                        assert(temp_interrupts_enabled == $past(prior_interrupts_enabled));
                    end else begin
                        // write stage
                        assert(interrupts_enabled == $past(temp_interrupts_enabled));
                        assert(exception_pc == $past(exception_pc));
                    end
                end else if ($past(op_i[2]) && ($past(op_i[1:0]) != 2'b0)) begin
                    // CSR* instruction
                    case ($past(addr_exception_i))
                        12'h300: begin // mstatus
                            assert(!fault_o);
                            if (!$past(is_write_stage_i)) begin
                                // read stage
                                assert(read_value_o == {28'b0, $past(prior_interrupts_enabled), 3'b0, $past(interrupts_enabled), 3'b0});
                            end else begin
                                // write stage
                                if ($past(op_i) == 3'b101)
                                    assert(interrupts_enabled == $past(write_value_i[3]));
                                else if ($past(op_i) == 3'b110)
                                    assert(interrupts_enabled == ($past(interrupts_enabled) | $past(write_value_i[3])));
                                else if ($past(op_i) == 3'b111)
                                    assert(interrupts_enabled == ($past(interrupts_enabled) & ~$past(write_value_i[3])));
                                if ($past(op_i) == 3'b101)
                                    assert(prior_interrupts_enabled == $past(write_value_i[7]));
                                else if ($past(op_i) == 3'b110)
                                    assert(prior_interrupts_enabled == ($past(prior_interrupts_enabled) | $past(write_value_i[7])));
                                else if ($past(op_i) == 3'b111)
                                    assert(prior_interrupts_enabled == ($past(prior_interrupts_enabled) & ~$past(write_value_i[7])));
                            end
                        end
                        12'h304: begin // mie
                            assert(!fault_o);
                            if (!$past(is_write_stage_i)) begin
                                // read stage
                                assert(read_value_o == {20'b0, $past(external_interrupt_enabled), 7'b0, $past(software_interrupt_enabled), 3'b0});
                            end else begin
                                // write stage
                                if ($past(op_i) == 3'b101)
                                    assert(external_interrupt_enabled == $past(write_value_i[11]));
                                else if ($past(op_i) == 3'b110)
                                    assert(external_interrupt_enabled == ($past(external_interrupt_enabled) | $past(write_value_i[11])));
                                else if ($past(op_i) == 3'b111)
                                    assert(external_interrupt_enabled == ($past(external_interrupt_enabled) & ~$past(write_value_i[11])));
                                if ($past(op_i) == 3'b101)
                                    assert(software_interrupt_enabled == $past(write_value_i[3]));
                                else if ($past(op_i) == 3'b110)
                                    assert(software_interrupt_enabled == ($past(software_interrupt_enabled) | $past(write_value_i[3])));
                                else if ($past(op_i) == 3'b111)
                                    assert(software_interrupt_enabled == ($past(software_interrupt_enabled) & ~$past(write_value_i[3])));
                            end
                        end
                        12'h341: begin // mepc
                            assert(!fault_o);
                            if (!$past(is_write_stage_i)) begin
                                // read stage
                                assert(read_value_o == $past(exception_pc));
                            end else begin
                                // write stage
                                if ($past(op_i) == 3'b101)
                                    assert(exception_pc[31:2] == $past(write_value_i[31:2]));
                                else if ($past(op_i) == 3'b110)
                                    assert(exception_pc[31:2] == ($past(exception_pc[31:2]) | $past(write_value_i[31:2])));
                                else if ($past(op_i) == 3'b111)
                                    assert(exception_pc[31:2] == ($past(exception_pc[31:2]) & ~$past(write_value_i[31:2])));
                            end
                        end
                        12'h342: begin // mcause
                            assert(!fault_o);
                            if (!$past(is_write_stage_i)) begin
                                // read stage
                                assert(read_value_o == {$past(trap_is_interrupt), 27'b0, $past(exception_code)});
                            end else begin
                                // write stage
                                if ($past(op_i) == 3'b101)
                                    assert(exception_code == $past(write_value_i[3:0]));
                                else if ($past(op_i) == 3'b110)
                                    assert(exception_code == ($past(exception_code[3:0]) | $past(write_value_i[3:0])));
                                else if ($past(op_i) == 3'b111)
                                    assert(exception_code == ($past(exception_code[3:0]) & ~$past(write_value_i[3:0])));
                            end
                        end
                        12'h344: begin // mip
                            assert(!fault_o);
                            if (!$past(is_write_stage_i)) begin
                                // read stage
                                assert(read_value_o == {20'b0, $past(external_interrupt_pending), 7'b0, $past(software_interrupt_pending), 3'b0});
                            end else begin
                                // write stage
                                if ($past(op_i) == 3'b101)
                                    assert(software_interrupt_pending == $past(write_value_i[3]));
                                else if ($past(op_i) == 3'b110)
                                    assert(software_interrupt_pending == ($past(software_interrupt_pending) | $past(write_value_i[3])));
                                else if ($past(op_i) == 3'b111)
                                    assert(software_interrupt_pending == ($past(software_interrupt_pending) & ~$past(write_value_i[3])));
                            end
                        end
                        default: begin
                            assert(fault_o);
                        end
                    endcase
                end else begin
                    assert(fault_o);
                end
            end
        end
    end
`endif

endmodule
