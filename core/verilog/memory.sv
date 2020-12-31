/*
 * Memory access unit.
 */

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

module memory(
    input clk, // Clock signal
    input reset_n, // Reset signal (active low)
    input available, // Operation available
    input is_write /* verilator public */, // Whether or not the operation is a write
    input is_unsigned /* verilator public */, // Whether or not the read byte/half-word value should be zero-extended (vs. sign-extended)
    input [1:0] op /* verilator public */, // Size of the operation (2'b00=byte, 2'b01=half-word, 2'b10=word)
    input [31:0] addr /* verilator public */, // Address to access
    input [31:0] in /* verilator public */, // Input data
    output [31:0] out, // Output data
    output op_fault, // Invalid op fault
    output addr_fault, // Misaligned address fault
    output access_fault // Access fault
);

    /* Outputs */
    reg [31:0] out;
    reg op_fault;
    reg addr_fault;
    reg access_fault;

    /* Basic Decode */
    wire op_is_invalid = op[1] & op[0];
    wire addr_is_misaligned = (op[1] & (addr[1] | addr[0])) | (op[0] & addr[0]);

    /* Memory Access */
    always @(posedge clk) begin
        if (~reset_n) begin
            out <= 32'b0;
            addr_fault <= 0;
            op_fault <= 0;
            access_fault <= 0;
        end else begin
            if (available) begin
                addr_fault <= addr_is_misaligned;
                op_fault <= op_is_invalid;
                access_fault <= mem_get_fault(addr, op[1], op[0], is_write);
                out <= mem_get_read_result(addr, in, op[1], op[0], is_write, is_unsigned);
            end else begin
                addr_fault <= 0;
                op_fault <= 0;
                access_fault <= 0;
            end
        end
    end

`ifdef FORMAL
    initial assume(~reset_n);
    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) begin
        f_past_valid <= 1;
    end

    /* Read path */
    always @(posedge clk) begin
        if (f_past_valid) begin
            if (available) begin
                assume($stable(is_write));
                assume($stable(is_unsigned));
                assume($stable(op));
                assume($stable(addr));
                assume($stable(in));
            end
            if ($past(~reset_n)) begin
                assert(!op_fault);
                assert(!addr_fault);
                assert(!access_fault);
            end else if ($past(available) && !available) begin
                case ($past(op))
                    2'b00: begin // byte
                        assert(!op_fault);
                        assert(!addr_fault);
                        assert(!access_fault);
                    end
                    2'b01: begin // half-word
                        assert(!op_fault);
                        if ($past(addr[0])) begin
                            assert(!access_fault);
                            assert(addr_fault);
                        end else begin
                            assert(!access_fault);
                            assert(!addr_fault);
                        end
                    end
                    2'b10: begin // word
                        assert(!op_fault);
                        if ($past(addr[1:0])) begin
                            assert(!access_fault);
                            assert(addr_fault);
                        end else begin
                            assert(!access_fault);
                            assert(!addr_fault);
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
