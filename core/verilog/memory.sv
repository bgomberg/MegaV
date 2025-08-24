`include "cells/and2.sv"
`include "cells/dff.sv"
`include "cells/dffe.sv"
`include "cells/external_memory.sv"
`include "cells/mux2.sv"
`include "cells/or2.sv"
`include "include/types.sv"

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
    input mem_microcode_t microcode, // Microcode
    input logic [31:0] addr /* verilator public */, // Address to access
    input logic [31:0] in /* verilator public */, // Input data
    output logic [31:0] out, // Output data
    output logic [2:0] fault_num // Fault number
);

    /* Decode */
    wire op_size_is_byte = ~microcode.op_size[1] & ~microcode.op_size[0];
    wire op_size_is_half_word = ~microcode.op_size[1] & microcode.op_size[0];
    wire op_size_is_word = microcode.op_size[1] & ~microcode.op_size[0];
    wire op_size_is_invalid = microcode.op_size[1] & microcode.op_size[0];
    wire addr_is_misaligned = (op_size_is_word & (addr[1] | addr[0])) | (op_size_is_half_word & addr[0]);
    wire misaligned_fault = op_size_is_invalid | addr_is_misaligned;

    /* External Memory */
    wire active = reset_n & ~enable_n & ~misaligned_fault;
    wire [31:0] out_data;
    wire next_access_fault;
    external_memory external_mem(
        .enable(active),
        .is_write(microcode.is_write),
        .op_size(microcode.op_size),
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
    or2 #(.BITS($bits(out_upper_half_word))) out_upper_half_word_or(
        .a({16{sign_bit & ~microcode.is_unsigned & ~op_size_is_word}}),
        .b(out_data[31:16]),
        .out(out_upper_half_word)
    );
    wire [7:0] out_lower_half_word_upper_byte;
    or2 #(.BITS($bits(out_lower_half_word_upper_byte))) out_lower_half_word_upper_byte_or(
        .a({8{sign_bit & ~microcode.is_unsigned & op_size_is_byte}}),
        .b(out_data[15:8]),
        .out(out_lower_half_word_upper_byte)
    );
    dffe #(.BITS($bits(out))) out_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in({out_upper_half_word, out_lower_half_word_upper_byte, out_data[7:0]}),
        .out(out)
    );

    /* Fault */
    wire [2:0] next_fault_num = {
        misaligned_fault | next_access_fault,
        microcode.is_write,
        ~misaligned_fault & next_access_fault
    };
    logic [2:0] temp_fault_num;
    dffe #(.BITS($bits(fault_num))) fault_num_dff(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_fault_num),
        .out(temp_fault_num)
    );
    // Need to output a fault number of 000 when not enabled
    mux2 #(.BITS($bits(fault_num))) fault_num_mux(
        .a(temp_fault_num),
        .b(3'b000),
        .select(enable_n),
        .out(fault_num)
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
                assume($stable(microcode.is_write));
                assume($stable(microcode.is_unsigned));
                assume($stable(microcode.op_size));
                assume($stable(addr));
                assume($stable(in));
            end
            if ($past(~reset_n)) begin
                assert(~fault_num[2]);
            end else if ($past(~enable_n) & ~enable_n) begin
                case ($past(microcode.op_size))
                    MEM_OP_SIZE_BYTE: begin
                        assert(~fault_num[2]);
                    end
                    MEM_OP_SIZE_HALF_WORD: begin
                        if ($past(addr[0])) begin
                            if ($past(microcode.is_write))
                                assert(fault_num == FAULT_NUM_STORE_ADDR_MISALIGNED);
                            else
                                assert(fault_num == FAULT_NUM_LOAD_ADDR_MISALIGNED);
                        end else begin
                            assert(~fault_num[2]);
                        end
                    end
                    MEM_OP_SIZE_WORD: begin
                        if ($past(addr[1:0])) begin
                            if ($past(microcode.is_write))
                                assert(fault_num == FAULT_NUM_STORE_ADDR_MISALIGNED);
                            else
                                assert(fault_num == FAULT_NUM_LOAD_ADDR_MISALIGNED);
                        end else begin
                            assert(~fault_num[2]);
                        end
                    end
                endcase
            end
        end
    end
`endif

endmodule
