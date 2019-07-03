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
    input reset, // Reset signal
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
        if (reset) begin
            busy <= 1'b0;
            started <= 1'b0;
            op_fault <= 1'b0;
            addr_fault <= 1'b0;
            access_fault <= 1'b0;
        end else begin
            addr_fault <= available & addr_is_misaligned;
            op_fault <= available & op_is_invalid;
            access_fault <= available & (addr_is_misaligned | mem_op_setup());
            out <= mem_read_get_result();
            if (~started & ~busy & available & ~(addr_is_misaligned | mem_op_setup())) begin
                // Starting a new operation
                started <= 1'b1;
                busy <= 1'b1;
            end else if (started & busy & ~mem_op_is_busy()) begin
                // Operation is complete
                busy <= 1'b0;
            end else if (started & ~busy & ~available) begin
                // End of the operation
                started <= 1'b0;
            end
        end
    end

`ifdef FORMAL
    initial assume(reset);
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
            if ($past(reset)) begin
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
