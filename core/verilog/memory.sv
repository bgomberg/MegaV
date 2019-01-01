/*
 * Memory access unit.
 */
module memory(
    input clk, // Clock signal
    input [2:0] op, // Operation to perform
    /* verilator lint_off UNUSED */
    input [31:0] addr, // Address to access
    /* verilator lint_on UNUSED */
    input [31:0] in, // Input data
    output [31:0] out, // Output data
    output fault // A fault condition has occurred
);

    /* Outputs */
    reg [31:0] out;
    reg fault;

    /* Fake Memory */
    reg [7:0] fake_memory[0:255] /* verilator public */;
    wire [31:0] fake_addr = {24'b0, addr[7:0]};

    /* Basic Decode */
    wire op_is_store = op[2];
    wire op_is_invalid = op[1] & op[0];
    wire addr_is_misaligned = (op[1] & (addr[1] | addr[0])) | (op[0] & addr[0]);

    /* Memory Access */
    always @(posedge clk) begin
        if (op_is_store) begin
            fake_memory[fake_addr] <= in[7:0];
            if (op[0] | op[1]) begin
                fake_memory[{fake_addr[31:1], 1'b1}] <= in[15:8];
            end
            if (op[1]) begin
                fake_memory[{fake_addr[31:2], 2'b10}] <= in[23:16];
                fake_memory[{fake_addr[31:2], 2'b11}] <= in[31:24];
            end
        end else begin
            out[7:0] <= fake_memory[fake_addr];
            out[15:8] <= (op[0] | op[1]) ? fake_memory[{fake_addr[31:1], 1'b1}] : 8'b0;
            out[23:16] <= op[1] ? fake_memory[{fake_addr[31:2], 2'b10}] : 8'b0;
            out[31:24] <= op[1] ? fake_memory[{fake_addr[31:2], 2'b11}] : 8'b0;
        end
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
    reg [31:0] f_read_data;
    initial f_read_data = {
        fake_memory[f_read_addr + 3],
        fake_memory[f_read_addr + 2],
        fake_memory[f_read_addr + 1],
        fake_memory[f_read_addr]
    };
    always @(*) begin
        assert(fake_memory[f_read_addr] == f_read_data[7:0]);
        assert(fake_memory[f_read_addr + 1] == f_read_data[15:8]);
        assert(fake_memory[f_read_addr + 2] == f_read_data[23:16]);
        assert(fake_memory[f_read_addr + 3] == f_read_data[31:24]);
    end

    /* Load path */
    always @(posedge clk) begin
    	if (f_past_valid && $past(addr[7:0]) == f_read_addr) begin
            case ($past(op))
                3'b000: begin // LB
                    assert(!fault);
                    assert(out == {24'b0, f_read_data[7:0]});
                end
                3'b001: begin // LH
                    if (f_read_addr[0]) begin
                        assert(fault);
                        assert($past(addr_is_misaligned));
                    end else begin
                        assert(!fault);
                        assert(out == {16'b0, f_read_data[15:0]});
                    end
                end
                3'b010: begin // LW
                    if (f_read_addr[1:0]) begin
                        assert(fault);
                        assert($past(addr_is_misaligned));
                    end else begin
                        assert(!fault);
                        assert(out == f_read_data);
                    end
                end
                default: begin
                    assert($past(op[2]) || (fault && $past(op_is_invalid)));
                end
            endcase
        end
    end

    /* Store path */
    always @(posedge clk) begin
    	if (f_past_valid && $past(addr[7:0]) == f_write_addr) begin
            case ($past(op))
                3'b100: begin // SB
                    assert(!fault && fake_memory[f_write_addr] == $past(in[7:0]));
                end
                3'b101: begin // SH
                    if (f_write_addr[0]) begin
                        assert(fault);
                        assert($past(addr_is_misaligned));
                    end else begin
                        assert(!fault);
                        assert({fake_memory[f_write_addr | 2'b01], fake_memory[f_write_addr]} == $past(in[15:0]));
                    end
                end
                3'b110: begin // SW
                    if (f_write_addr[1:0]) begin
                        assert(fault);
                        assert($past(addr_is_misaligned));
                    end else begin
                        assert(!fault);
                        assert({fake_memory[f_write_addr | 2'b11], fake_memory[f_write_addr | 2'b10], fake_memory[f_write_addr | 2'b01], fake_memory[f_write_addr]} == $past(in));
                    end
                end
                default: begin
                    assert(!$past(op[2]) || (fault && $past(op_is_invalid)));
                end
            endcase
        end
    end
`endif

endmodule
