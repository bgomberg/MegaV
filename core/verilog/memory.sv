/*
 * Memory access unit.
 */

`ifdef verilator
import "DPI-C" function void mem_op_setup(input bit is_write, input bit op_1, input bit op_0, input int addr, input int write_data, input bit read_is_unsigned);
import "DPI-C" function bit mem_op_is_busy();
import "DPI-C" function int mem_read_get_result();
`endif

module memory(
    input clk, // Clock signal
    input reset, // Reset signal
    input available, // Operation available
    input is_write, // Whether or not the operation is a write
    input is_unsigned, // Whether or not the read byte/half-word value should be zero-extended (vs. sign-extended)
    input [1:0] op /* verilator public */, // Size of the operation (2'b00=byte, 2'b01=half-word, 2'b10=word)
    input [31:0] addr, // Address to access
    input [31:0] in, // Input data
    output [31:0] out, // Output data
    output busy, // Operation busy
    output fault // A fault condition has occurred
);

    /* Outputs */
    reg [31:0] out;
    reg busy;
    reg fault;

    /* Basic Decode */
    wire op_is_invalid = op[1] & op[0];
    wire addr_is_misaligned = (op[1] & (addr[1] | addr[0])) | (op[0] & addr[0]);

    /* Memory Access */
    reg started;
    always @(posedge clk) begin
        if (reset) begin
            busy <= 1'b0;
            started <= 1'b0;
            fault <= 1'b0;
        end else begin
            fault <= available & (op_is_invalid | addr_is_misaligned);
            if (!started && available) begin
                // Starting a new operation
`ifdef verilator
                mem_op_setup(is_write, op[1], op[0], addr, in, is_unsigned);
`endif
                busy <= 1'b1;
                started <= 1'b1;
            end else if (started && busy) begin
                // Operation has previously been started
`ifdef verilator
                if (!mem_op_is_busy()) begin
`else
                if (1) begin
`endif
                    // Operation is complete
                    busy <= 1'b0;
                    if (!is_write) begin
`ifdef verilator
                        out <= mem_read_get_result();
`else
                        out <= 32'b0;
`endif
                    end
                end
            end else if (!available) begin
                // Wait for the available signal to fall before allowing a new operation to start
                started <= 1'b0;
            end
        end
    end

`ifdef FORMAL
    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) begin
        f_past_valid = 1;
    end

    /* Read path */
    always @(posedge clk) begin
        if (f_past_valid) begin
            if ($past(reset)) begin
                assert(!busy);
                assert(!fault);
            end else if ($past(available)) begin
                case ($past(op))
                    2'b00: begin // byte
                        assert(!fault);
                    end
                    2'b01: begin // half-word
                        if ($past(addr[0])) begin
                            assert(fault);
                        end else begin
                            assert(!fault);
                        end
                    end
                    2'b10: begin // word
                        if ($past(addr[1:0])) begin
                            assert(fault);
                        end else begin
                            assert(!fault);
                        end
                    end
                    default: begin
                        assert(fault);
                    end
                endcase
            end
        end
    end
`endif

endmodule
