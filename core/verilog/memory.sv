/*
 * Memory access unit.
 */

`define MEM_OP_SIZE_BYTE 2'b00
`define MEM_OP_SIZE_HALF_WORD 2'b01
`define MEM_OP_SIZE_WORD 2'b10

`ifdef verilator
import "DPI-C" function bit mem_get_fault(int addr, bit op_h, bit op_l, bit is_write);
import "DPI-C" function int mem_get_read_result(int addr, int write_val, bit op_h, bit op_l, bit is_write, bit is_unsigned);
`else
function mem_get_fault;
    mem_get_fault = 1'b0;
endfunction
function mem_get_read_result;
    mem_get_read_result = 32'b0;
endfunction
`endif

/*
Address Space:
    0x00000000-0x1fffffff: FLASH
    0x20000000-0x2fffffff: RAM
    0x30000000-0x3fffffff: PERIPH
    0x40000000-0xffffffff: RESERVED
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

    /* Basic Decode */
    wire op_is_invalid = op_size[1] & op_size[0];
    wire addr_is_misaligned = (op_size[1] & (addr[1] | addr[0])) | (op_size[0] & addr[0]);

    /* Memory Access */
    always_ff @(posedge clk) begin
        if (~reset_n) begin
            out <= 32'b0;
            addr_fault <= 1'b0;
            op_fault <= 1'b0;
            access_fault_n <= 1'b1;
        end else begin
            if (~enable_n) begin
                addr_fault <= addr_is_misaligned;
                op_fault <= op_is_invalid;
                access_fault_n <= ~mem_get_fault(addr, op_size[1], op_size[0], is_write);
                out <= mem_get_read_result(addr, in, op_size[1], op_size[0], is_write, is_unsigned);
            end else begin
                addr_fault <= 1'b0;
                op_fault <= 1'b0;
                access_fault_n <= 1'b1;
            end
        end
    end

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
