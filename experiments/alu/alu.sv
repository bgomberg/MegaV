/*
 * An ALU which can perform all the math operations (opcodes 0b0010011 and 0b0110011) of the RV31I/E instruction set.
 */
module alu #(
    parameter DATA_WIDTH = 32
)(
    input reset, // Synchronous reset line
    input clk, // Clock signal
    input [3:0] op, // Operation to be performed
    input [DATA_WIDTH-1:0] in_a, // Input data (bus A)
    input [DATA_WIDTH-1:0] in_b, // Input data (bus B)
    output [DATA_WIDTH-1:0] out, // Output data
    output invalid // Invalid operation
);

    /* Internal variables */
    reg [DATA_WIDTH-1:0] out;
    reg invalid;

    /* Logic */
    always @(posedge clk) begin
        if (reset) begin
            out <= 0;
            invalid <= 0;
        end
        else begin
            case (op)
                4'b000: begin // ADD
                    invalid <= 0;
                    out <= in_a + in_b;
                end
                4'b0001: begin // SLL
                    invalid <= 0;
                    out <= in_a << in_b[4:0];
                end
                4'b0010: begin // SLT
                    invalid <= 0;
                    out <= ($signed(in_a) < $signed(in_b)) ? 1 : 0;
                end
                4'b0011: begin // SLTU
                    invalid <= 0;
                    out <= (in_a < in_b) ? 1 : 0;
                end
                4'b0100: begin // XOR
                    invalid <= 0;
                    out <= in_a ^ in_b;
                end
                4'b0101: begin // SRL
                    invalid <= 0;
                    out <= in_a >> in_b[4:0];
                end
                4'b0110: begin // OR
                    invalid <= 0;
                    out <= in_a | in_b;
                end
                4'b0111: begin // AND
                    invalid <= 0;
                    out <= in_a & in_b;
                end
                4'b1000: begin // SUB
                    invalid <= 0;
                    out <= in_a - in_b;
                end
                4'b1101: begin // SRA
                    invalid <= 0;
                    out <= in_a >>> in_b[4:0];
                end
                default: begin
                    invalid <= 1;
                    out <= 32'bx;
                end
            endcase
        end
    end

`ifdef FORMAL
    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) begin
        f_past_valid = 1;
    end

    initial assume(reset);
    always @(*) begin
        if (!f_past_valid) begin
            assume(reset);
        end
    end

    /* Validate logic */
    always @(posedge clk) begin
    	if (f_past_valid && !$past(reset)) begin
            case ($past(op))
                4'b000: begin // ADD
                    assert(!invalid);
                    assert(out == ($past(in_a) + $past(in_b)));
                end
                4'b0001: begin // SLL
                    assert(!invalid);
                    assert(out == ($past(in_a) << $past(in_b[4:0])));
                end
                4'b0010: begin // SLT
                    assert(!invalid);
                    assert(out == (($signed($past(in_a)) < $signed($past(in_b))) ? 1 : 0));
                end
                4'b0011: begin // SLTU
                    assert(!invalid);
                    assert(out == (($past(in_a) < $past(in_b)) ? 1 : 0));
                end
                4'b0100: begin // XOR
                    assert(!invalid);
                    assert(out == ($past(in_a) ^ $past(in_b)));
                end
                4'b0101: begin // SRL
                    assert(!invalid);
                    assert(out == ($past(in_a) >> $past(in_b[4:0])));
                end
                4'b0110: begin // OR
                    assert(!invalid);
                    assert(out == ($past(in_a) | $past(in_b)));
                end
                4'b0111: begin // AND
                    assert(!invalid);
                    assert(out == ($past(in_a) & $past(in_b)));
                end
                4'b1000: begin // SUB
                    assert(!invalid);
                    assert(out == ($past(in_a) - $past(in_b)));
                end
                4'b1101: begin // SRA
                    assert(!invalid);
                    assert(out == ($past(in_a) >>> $past(in_b[4:0])));
                end
                default: begin
                    assert(invalid);
                end
            endcase
        end
    end
`endif

endmodule
