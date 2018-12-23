/*
 * Program counter.
 */
module program_counter #(
)(
    input clk, // Clock signal
    input reset, // Reset signal
    input [1:0] op, // Operation to perform (b01=in[31:1], b10=pc+offset, b11=in[0]?pc+offset:in[31:1])
    input [31:0] offset, // Offset to add to the PC (for op b10 and b11)
    input [31:0] in, // Input data
    output [31:0] pc, // Current PC
    output fault // Fault condition
);

    /* Outputs */
    reg [31:0] pc;
    reg fault;

    /* Decoding and intermediate values */
    wire [31:0] in_pc_value = {in[31:2], 2'b00};
    wire [31:0] pc_offset_sum = pc + {offset[31:2], 2'b00};
    wire op_is_in = (op == 2'b01) | ((op == 2'b11) & ~in[0]);
    wire op_is_pc_offset_sum = (op == 2'b10) | ((op == 2'b11) & in[0]);
    wire in_is_invalid = in[1];
    wire offset_is_invalid = offset[1] | offset[0];

    /* Logic */
    always @(posedge clk) begin
        if (reset) begin
            pc <= 0;
            fault <= 0;
        end else begin
            if (op_is_in) begin
                fault <= in_is_invalid;
                pc <= in_pc_value;
            end else if (op_is_pc_offset_sum) begin
                fault <= offset_is_invalid;
                pc <= pc_offset_sum;
            end else begin
                fault <= 1;
            end
        end
    end

`ifdef FORMAL
    initial	assume(reset);
    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) begin
        f_past_valid = 1;
    end

    /* Validate logic */
    always @(posedge clk) begin
    	if (f_past_valid) begin
            assert(next_pc == pc + 4);
            if ($past(reset)) begin
                assert(pc == 0);
                assert(!fault);
            end else begin
                case ($past(op))
                    2'b00: begin
                        assume(!$past(reset));
                        if ($past(next_pc[1:0])) begin
                            assert(fault);
                        end else begin
                            assert(!fault);
                            assert(pc == $past(next_pc));
                        end
                    end
                    2'b01: begin
                        if ($past(in[1])) begin
                            assert(fault);
                        end else begin
                            assert(!fault);
                            assert(pc == {$past(in[31:2]), 2'b00});
                        end
                    end
                    2'b10: begin
                        if ($past(offset[1:0])) begin
                            assert(fault);
                        end else begin
                            assert(!fault);
                            assert(pc == ($past(pc) + $past(offset)));
                        end
                    end
                    2'b11: begin
                        if ($past(in[0])) begin
                            if ($past(offset[1:0])) begin
                                assert(fault);
                            end else begin
                                assert(!fault);
                                assert(pc == ($past(pc) + $past(offset)));
                            end
                        end else begin
                            assume(!$past(reset));
                            if ($past(next_pc[1:0])) begin
                                assert(fault);
                            end else begin
                                assert(!fault);
                                assert(pc == $past(next_pc));
                            end
                        end
                    end
                endcase
            end
        end
    end
`endif

endmodule
