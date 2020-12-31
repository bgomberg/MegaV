`include "adder32.sv"

function [31:0] reverse_bit_order(input [31:0] data);
integer i;
begin
    for (i = 0; i < 32; i = i + 1) begin
        reverse_bit_order[31-i] = data[i];
    end
end
endfunction

/*
 * An ALU which can perform all the math operations (opcodes 0b0010011 and 0b0110011) of the RV31I/E instruction set.
 */
module alu(
    input clk, // Clock signal
    input reset_n, // Reset signal (active low)
    input available, // Operation available
    input [4:0] op, // Operation to be performed
    input [31:0] in_a, // Input data (bus A)
    input [31:0] in_b, // Input data (bus B)
    output [31:0] out, // Output data
    output fault // Invalid operation
);

    /* Internal variables */
    reg [31:0] out;
    reg fault;

    /* Op decoding */
    wire op_is_add = (op == 5'b00000);
    wire op_shift_is_right = op[2];
    wire op_shift_is_right_arithmetic = op_shift_is_right & op[3];
    wire op_is_branch_eq_ne = op[4] & ~op[2];
    wire op_is_branch_gt = op[4] & op[0];
    wire op_branch_slt_is_unsigned = (~op[4] & op[0]) | (op[2] & op[1]);
    wire is_invalid_op = (op[3] & op[1]) | (op[4] & op[3]) | (op[3] & ~op[2] & op[0]) | (op[3] & op[2] & ~op[0]) | (op[4] & ~op[2] & op[1]);

    /* Bitwise operations */
    wire [31:0] xor_result = in_a ^ in_b;
    wire [31:0] or_result = in_a | in_b;
    wire [31:0] and_result = in_a & in_b;

    /* Adder */
    wire [31:0] adder_b = in_b ^ {32{~op_is_add}};
    wire adder_carry_in = ~op_is_add;
    wire [31:0] adder_sum;
    adder32 adder_module(
        in_a ^ adder_b,
        in_a & adder_b,
        adder_carry_in,
        adder_sum);

    /* Shifters */
    wire shift_in_bit = op_shift_is_right_arithmetic & in_a[31];
    wire [31:0] shift_in = op_shift_is_right ? reverse_bit_order(in_a) : in_a;
    wire [31:0] shift_int_1 = in_b[4] ? {shift_in[15:0], {16{shift_in_bit}}} : shift_in;
    wire [31:0] shift_int_2 = in_b[3] ? {shift_int_1[23:0], {8{shift_in_bit}}} : shift_int_1;
    wire [31:0] shift_int_3 = in_b[2] ? {shift_int_2[27:0], {4{shift_in_bit}}} : shift_int_2;
    wire [31:0] shift_int_4 = in_b[1] ? {shift_int_3[29:0], {2{shift_in_bit}}} : shift_int_3;
    wire [31:0] shift_int_5 = in_b[0] ? {shift_int_4[30:0], shift_in_bit} : shift_int_4;
    wire [31:0] shift_result = op_shift_is_right ? reverse_bit_order(shift_int_5) : shift_int_5;

    /* Logic */
    wire eq_result = (xor_result == 32'b0);
    wire sign_cmp_result = (in_a[31] ^ op_branch_slt_is_unsigned) & ~(in_b[31] ^ op_branch_slt_is_unsigned);
    wire lt_ltu_result = sign_cmp_result | (~(in_a[31] ^ in_b[31]) & adder_sum[31] & ~eq_result);
    wire branch_slt_sltu_result = (op_is_branch_eq_ne ? eq_result : lt_ltu_result) ^ op_is_branch_gt;
    always @(posedge clk) begin
        if (~reset_n) begin
            // Reset
            fault <= 0;
        end else begin
            if (available) begin
                fault <= is_invalid_op;
                if (~op[4] & ~op[2] & ~op[1] & ~op[0]) // ADD,SUB
                    out <= adder_sum;
                else if (~op[4] & op[2] & ~op[1] & ~op[0]) // XOR
                    out <= xor_result;
                else if (~op[4] & op[2] & op[1] & ~op[0]) // OR
                    out <= or_result;
                else if (~op[4] & op[2] & op[1] & op[0]) // AND
                    out <= and_result;
                else if (~op[4] & ~op[1] & op[0]) // SLL,SRL,SRA
                    out <= shift_result;
                else // SLT,SLTU,BEQ,BNE,BLT,BGE,BTLU,BGEU
                    out <= {31'b0, branch_slt_sltu_result};
            end else begin
                fault <= 0;
            end
        end
    end

`ifdef FORMAL
    initial assume(~reset_n);
    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) begin
        f_past_valid = 1;
    end

    /* Validate logic */
    always @(posedge clk) begin
        if (f_past_valid) begin
            if ($past(~reset_n)) begin
                assert(!fault);
            end else begin
                assume($past(available));
                case ($past(op))
                    5'b0000: begin // ADD
                        assert(!fault);
                        assert(out == ($past(in_a) + $past(in_b)));
                    end
                    5'b00001: begin // SLL
                        assert(!fault);
                        assert(out == ($past(in_a) << $past(in_b[4:0])));
                    end
                    5'b00010: begin // SLT
                        assert(!fault);
                        assert(out == (($signed($past(in_a)) < $signed($past(in_b))) ? 1 : 0));
                    end
                    5'b00011: begin // SLTU
                        assert(!fault);
                        assert(out == (($past(in_a) < $past(in_b)) ? 1 : 0));
                    end
                    5'b00100: begin // XOR
                        assert(!fault);
                        assert(out == ($past(in_a) ^ $past(in_b)));
                    end
                    5'b00101: begin // SRL
                        assert(!fault);
                        assert(out == ($past(in_a) >> $past(in_b[4:0])));
                    end
                    5'b00110: begin // OR
                        assert(!fault);
                        assert(out == ($past(in_a) | $past(in_b)));
                    end
                    5'b00111: begin // AND
                        assert(!fault);
                        assert(out == ($past(in_a) & $past(in_b)));
                    end
                    5'b01000: begin // SUB
                        assert(!fault);
                        assert(out == ($past(in_a) - $past(in_b)));
                    end
                    5'b01101: begin // SRA
                        assert(!fault);
                        assert(out == ($past(in_a) >>> $past(in_b[4:0])));
                    end
                    5'b10000: begin // BEQ
                        assert(!fault);
                        assert(out == (($past(in_a) == $past(in_b)) ? 1'b1 : 1'b0));
                    end
                    5'b10001: begin // BNE
                        assert(!fault);
                        assert(out == (($past(in_a) != $past(in_b)) ? 1'b1 : 1'b0));
                    end
                    5'b10100: begin // BLT
                        assert(!fault);
                        assert(out == (($signed($past(in_a)) < $signed($past(in_b))) ? 1'b1 : 1'b0));
                    end
                    5'b10101: begin // BGE
                        assert(!fault);
                        assert(out == (($signed($past(in_a)) >= $signed($past(in_b))) ? 1'b1 : 1'b0));
                    end
                    5'b10110: begin // BLTU
                        assert(!fault);
                        assert(out == (($past(in_a) < $past(in_b)) ? 1'b1 : 1'b0));
                    end
                    5'b10111: begin // BGEU
                        assert(!fault);
                        assert(out == (($past(in_a) >= $past(in_b)) ? 1'b1 : 1'b0));
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
