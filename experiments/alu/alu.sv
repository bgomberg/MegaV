/*
 * An ALU which can perform all the math operations (opcodes 0b0010011 and 0b0110011) of the RV31I/E instruction set.
 */
module alu #(
    parameter DATA_WIDTH = 32
    parameter OP_WIDTH = 4
)(
    input clk, // Clock signal
    input [OP_WIDTH-1:0] op, // Operation to be performed
    input [DATA_WIDTH-1:0] in_a, // Input data (bus A)
    input [DATA_WIDTH-1:0] in_b, // Input data (bus B)
    output [DATA_WIDTH-1:0] out, // Output data
    output fault // Invalid operation
);

    /* Internal variables */
    reg [DATA_WIDTH-1:0] out;
    reg fault;

    /* Logic */
    always @(posedge clk) begin
        case (op)
            4'b0000: begin // ADD
                fault <= 0;
                out <= in_a + in_b;
            end
            4'b0001: begin // SLL
                fault <= 0;
                out <= in_a << in_b[4:0];
            end
            4'b0010: begin // SLT
                fault <= 0;
                out <= ($signed(in_a) < $signed(in_b)) ? 1 : 0;
            end
            4'b0011: begin // SLTU
                fault <= 0;
                out <= (in_a < in_b) ? 1 : 0;
            end
            4'b0100: begin // XOR
                fault <= 0;
                out <= in_a ^ in_b;
            end
            4'b0101: begin // SRL
                fault <= 0;
                out <= in_a >> in_b[4:0];
            end
            4'b0110: begin // OR
                fault <= 0;
                out <= in_a | in_b;
            end
            4'b0111: begin // AND
                fault <= 0;
                out <= in_a & in_b;
            end
            4'b1000: begin // SUB
                fault <= 0;
                out <= in_a - in_b;
            end
            4'b1101: begin // SRA
                fault <= 0;
                out <= in_a >>> in_b[4:0];
            end
            default: begin
                fault <= 1;
                out <= 32'bx;
            end
        endcase
    end

`ifdef FORMAL
    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) begin
        f_past_valid = 1;
    end

    /* Validate logic */
    always @(posedge clk) begin
    	if (f_past_valid) begin
            case ($past(op))
                4'b000: begin // ADD
                    assert(!fault);
                    assert(out == ($past(in_a) + $past(in_b)));
                end
                4'b0001: begin // SLL
                    assert(!fault);
                    assert(out == ($past(in_a) << $past(in_b[4:0])));
                end
                4'b0010: begin // SLT
                    assert(!fault);
                    assert(out == (($signed($past(in_a)) < $signed($past(in_b))) ? 1 : 0));
                end
                4'b0011: begin // SLTU
                    assert(!fault);
                    assert(out == (($past(in_a) < $past(in_b)) ? 1 : 0));
                end
                4'b0100: begin // XOR
                    assert(!fault);
                    assert(out == ($past(in_a) ^ $past(in_b)));
                end
                4'b0101: begin // SRL
                    assert(!fault);
                    assert(out == ($past(in_a) >> $past(in_b[4:0])));
                end
                4'b0110: begin // OR
                    assert(!fault);
                    assert(out == ($past(in_a) | $past(in_b)));
                end
                4'b0111: begin // AND
                    assert(!fault);
                    assert(out == ($past(in_a) & $past(in_b)));
                end
                4'b1000: begin // SUB
                    assert(!fault);
                    assert(out == ($past(in_a) - $past(in_b)));
                end
                4'b1101: begin // SRA
                    assert(!fault);
                    assert(out == ($past(in_a) >>> $past(in_b[4:0])));
                end
                default: begin
                    assert(fault);
                    assert(out == 32'b1);
                end
            endcase
        end
    end
`endif

endmodule
