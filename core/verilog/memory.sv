/*
 * Memory access unit.
 */

`ifdef verilator
import "DPI-C" function int mem_read(input int addr, input bit is_unsigned, input bit op_1, input bit op_0);
import "DPI-C" function void mem_write(input int addr, input int data, input bit op_1, input bit op_0);
`endif

module memory(
    input clk, // Clock signal
    input is_write, // Whether or not the operation is a write
    input is_unsigned, // Whether or not the read byte/half-word value should be zero-extended (vs. sign-extended)
    input [1:0] op /* verilator public */, // Size of the operation (2'b00=byte, 2'b01=half-word, 2'b10=word)
    input [31:0] addr, // Address to access
    input [31:0] in, // Input data
    output [31:0] out, // Output data
    output fault // A fault condition has occurred
);

    /* Outputs */
    reg [31:0] out /* verilator public */;
    reg fault;

    /* Basic Decode */
    wire op_is_invalid = op[1] & op[0];
    wire addr_is_misaligned = (op[1] & (addr[1] | addr[0])) | (op[0] & addr[0]);

    /* Memory Access */
    always @(posedge clk) begin
`ifdef verilator
        if (is_write) begin
            mem_write(addr, in, op[1], op[0]);
        end else begin
            out <= mem_read(addr, is_unsigned, op[1], op[0]);
        end
`else
        out <= 0;
`endif
        fault <= op_is_invalid | addr_is_misaligned;
    end

`ifdef FORMAL
    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) begin
        f_past_valid = 1;
    end

    (* anyconst *) wire [7:0] f_read_addr;
    (* anyconst *) wire [7:0] f_write_addr;

    /* Read path */
    always @(posedge clk) begin
        if (f_past_valid && $past(addr[7:0]) == f_read_addr && !$past(is_write)) begin
            case ($past(op))
                2'b00: begin // byte
                    assert(!fault);
                end
                2'b01: begin // half-word
                    if (f_read_addr[0]) begin
                        assert(fault);
                        assert($past(addr_is_misaligned));
                    end else begin
                        assert(!fault);
                    end
                end
                2'b10: begin // word
                    if (f_read_addr[1:0]) begin
                        assert(fault);
                        assert($past(addr_is_misaligned));
                    end else begin
                        assert(!fault);
                    end
                end
                default: begin
                    assert(fault && $past(op_is_invalid));
                end
            endcase
        end
    end

    /* Write path */
    always @(posedge clk) begin
        if (f_past_valid && $past(addr[7:0]) == f_write_addr && $past(is_write)) begin
            case ($past(op))
                2'b00: begin // byte
                    assert(!fault);
                end
                2'b01: begin // half-word
                    if (f_write_addr[0]) begin
                        assert(fault);
                        assert($past(addr_is_misaligned));
                    end else begin
                        assert(!fault);
                    end
                end
                2'b10: begin // word
                    if (f_write_addr[1:0]) begin
                        assert(fault);
                        assert($past(addr_is_misaligned));
                    end else begin
                        assert(!fault);
                    end
                end
                default: begin
                    assert(fault && $past(op_is_invalid));
                end
            endcase
        end
    end
`endif

endmodule
