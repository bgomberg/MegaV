/*
 * Program counter.
 */
module program_counter #(
)(
    input clk, // Clock signal
    input reset, // Reset signal
    input [31:0] in, // Input data
    output [31:0] pc, // Current PC
    output fault // Fault condition
);

    /* Outputs */
    reg [31:0] pc;
    reg fault;

    /* Logic */
    always @(posedge clk) begin
        if (reset) begin
            pc <= 0;
            fault <= 0;
        end else begin
            pc <= in;
            fault <= in[1] | in[0];
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
