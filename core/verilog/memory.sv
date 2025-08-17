`include "cells/dff.sv"
`include "cells/dffe.sv"
`include "cells/external_memory.sv"
`include "cells/mux2.sv"
`include "cells/or2.sv"
`include "constants/mem_op_size.sv"

/*
Address Space:
    0x00000000-0x1fffffff: FLASH
    0x20000000-0x2fffffff: RAM
    0x30000000-0x3fffffff: PERIPH
    0x40000000-0xffffffff: RESERVED
*/

/*
 * Memory access unit.
 */
module memory(
    input logic clk, // Clock signal
    input logic reset_n, // Reset signal (active low)
    input logic enable_n, // Enable (active low)
    input logic is_write /* verilator public */, // Whether or not the operation is a write
    input logic is_unsigned /* verilator public */, // Whether or not the read byte/half-word value should be zero-extended (vs. sign-extended)
    input logic [1:0] op_size /* verilator public */, // Size of the operation
    input logic [31:0] addr /* verilator public */, // Address to access
    input logic [31:0] in /* verilator public */, // Input data
    output logic [31:0] out, // Output data
    output logic op_fault, // Invalid op fault
    output logic addr_fault, // Misaligned address fault
    output logic access_fault_n // Access fault (active low)
);

    /* Size Decode */
    wire op_size_is_byte = ~op_size[1] & ~op_size[0];
    wire op_size_is_half_word = ~op_size[1] & op_size[0];
    wire op_size_is_word = op_size[1] & ~op_size[0];
    wire op_size_is_invalid = op_size[1] & op_size[0];
    wire addr_is_misaligned = (op_size_is_word & (addr[1] | addr[0])) | (op_size_is_half_word & addr[0]);

    /* External Memory */
    wire active = reset_n & ~enable_n & ~addr_is_misaligned & ~op_size_is_invalid;
    wire [31:0] out_data;
    wire next_access_fault;
    external_memory external_mem(
        .enable(active),
        .is_write(is_write),
        .op_size(op_size),
        .addr(addr),
        .in(in),
        .out(out_data),
        .access_fault(next_access_fault)
    );

    /* Data Output */
    wire sign_bit;
    mux2 sign_bit_mux(
        .a(out_data[7]),
        .b(out_data[15]),
        .select(op_size_is_half_word),
        .out(sign_bit)
    );
    wire [15:0] out_upper_half_word;
    or2 #(.BITS(16)) out_upper_half_word_or(
        .a({16{sign_bit & ~is_unsigned & ~op_size_is_word}}),
        .b(out_data[31:16]),
        .out(out_upper_half_word)
    );
    wire [7:0] out_lower_half_word_upper_byte;
    or2 #(.BITS(8)) out_lower_half_word_upper_byte_or(
        .a({8{sign_bit & ~is_unsigned & op_size_is_byte}}),
        .b(out_data[15:8]),
        .out(out_lower_half_word_upper_byte)
    );
    dffe #(.BITS(32)) out_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in({out_upper_half_word, out_lower_half_word_upper_byte, out_data[7:0]}),
        .out(out)
    );

    /* Fault */
    wire access_fault;
    dffe access_fault_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_access_fault),
        .out(access_fault)
    );
    assign access_fault_n = ~access_fault;
    dff addr_fault_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .in(~enable_n & addr_is_misaligned),
        .out(addr_fault)
    );
    dff op_fault_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .in(~enable_n & op_size_is_invalid),
        .out(op_fault)
    );

`ifdef FORMAL
    initial assume(~reset_n);
    logic f_past_valid;
    initial f_past_valid = 0;
    always_ff @(posedge clk) begin
        f_past_valid <= 1;
    end

    /* Read path */
    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            if (~enable_n) begin
                assume($stable(is_write));
                assume($stable(is_unsigned));
                assume($stable(op_size));
                assume($stable(addr));
                assume($stable(in));
            end
            if ($past(~reset_n)) begin
                assert(~op_fault);
                assert(~addr_fault);
                assert(access_fault_n);
            end else if ($past(~enable_n) & ~enable_n) begin
                case ($past(op_size))
                    `MEM_OP_SIZE_BYTE: begin
                        assert(~op_fault);
                        assert(~addr_fault);
                        assert(access_fault_n);
                    end
                    `MEM_OP_SIZE_HALF_WORD: begin
                        assert(~op_fault);
                        if ($past(addr[0])) begin
                            assert(access_fault_n);
                            assert(addr_fault);
                        end else begin
                            assert(access_fault_n);
                            assert(~addr_fault);
                        end
                    end
                    `MEM_OP_SIZE_WORD: begin
                        assert(~op_fault);
                        if ($past(addr[1:0])) begin
                            assert(access_fault_n);
                            assert(addr_fault);
                        end else begin
                            assert(access_fault_n);
                            assert(~addr_fault);
                        end
                    end
                    default: begin
                        assert(op_fault);
                    end
                endcase
            end
        end
    end
`endif

endmodule
