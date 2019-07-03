/*
 * Memory access unit.
 */

`ifdef verilator
import "DPI-C" function bit mem_op_setup();
import "DPI-C" function bit mem_op_is_busy();
import "DPI-C" function int mem_read_get_result();
`else
function mem_op_setup;
    mem_op_setup = 1'b0;
endfunction
function mem_op_is_busy;
    mem_op_is_busy = 1'b0;
endfunction
function mem_read_get_result;
    mem_read_get_result = 32'b0;
endfunction
`endif

module memory(
    input clk, // Clock signal
    input reset_n, // Reset signal (active low)
    input available /* verilator public */, // Operation available
    input is_write /* verilator public */, // Whether or not the operation is a write
    input is_unsigned /* verilator public */, // Whether or not the read byte/half-word value should be zero-extended (vs. sign-extended)
    input [1:0] op /* verilator public */, // Size of the operation (2'b00=byte, 2'b01=half-word, 2'b10=word)
    input [31:0] addr /* verilator public */, // Address to access
    input [31:0] in /* verilator public */, // Input data
    output [31:0] out, // Output data
    output busy, // Operation busy
    output op_fault, // Invalid op fault
    output addr_fault, // Misaligned address fault
    output access_fault // Access fault
);

    /* Outputs */
    reg [31:0] out;
    reg busy;
    reg op_fault;
    reg addr_fault;
    reg access_fault;

    /* Basic Decode */
    wire op_is_invalid = op[1] & op[0];
    wire addr_is_misaligned = (op[1] & (addr[1] | addr[0])) | (op[0] & addr[0]);

    /* Memory Access */
    reg started /* verilator public */;
    always @(posedge clk) begin
        addr_fault <= reset_n & available & addr_is_misaligned;
        op_fault <= reset_n & available & op_is_invalid;
        access_fault <= reset_n & available & mem_op_setup();
        out <= mem_read_get_result();
        started <= reset_n & ((started & (available | busy)) | (~busy & available & ~mem_op_setup()));
        busy <= reset_n & ((~started & ~busy & available & ~mem_op_setup()) | (busy & mem_op_is_busy()));
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
            assume(~started | busy);
            if (available) begin
                assume($stable(is_write));
                assume($stable(is_unsigned));
                assume($stable(op));
                assume($stable(addr));
                assume($stable(in));
            end
            if (busy) begin
                assume(available);
            end
            if ($past(~reset_n)) begin
                assert(!busy);
                assert(!op_fault);
                assert(!addr_fault);
                assert(!access_fault);
            end else if ($past(busy) && !busy) begin
                case ($past(op))
                    2'b00: begin // byte
                        assert(!op_fault);
                        assert(!addr_fault);
                        assert(!access_fault);
                    end
                    2'b01: begin // half-word
                        assert(!op_fault);
                        if ($past(addr[0])) begin
                            assert(access_fault);
                            assert(addr_fault);
                        end else begin
                            assert(!access_fault);
                            assert(!addr_fault);
                        end
                    end
                    2'b10: begin // word
                        assert(!op_fault);
                        if ($past(addr[1:0])) begin
                            assert(access_fault);
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
