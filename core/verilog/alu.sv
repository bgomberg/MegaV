`include "adder32.sv"
`include "shifter.sv"

/*
 * An ALU which can perform all the math operations (opcodes 0b0010011 and 0b0110011) of the RV31I/E instruction set.
 */
module alu(
    input logic clk, // Clock signal
    input logic reset_n, // Reset signal (active low)
    input logic available, // Operation available
    input logic [4:0] op, // Operation to be performed
    input logic [31:0] in_a, // Input data (bus A)
    input logic [31:0] in_b, // Input data (bus B)
    output logic [31:0] out, // Output data
    output logic fault // Invalid operation
);

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
    wire [31:0] adder_in_b = in_b ^ {32{~op_is_add}};
    wire adder_carry_in = ~op_is_add;
    wire [31:0] adder_sum;
    adder32 adder_module(
        in_a,
        adder_in_b,
        adder_carry_in,
        adder_sum
    );

    /* Shifter */
    wire [31:0] shift_result;
    shifter shifter_module(
        in_a,
        in_b[4:0],
        op_shift_is_right,
        op_shift_is_right_arithmetic,
        shift_result
    );

    /* Logic */
    wire eq_result = (xor_result == 32'b0);
    wire sign_cmp_result = (in_a[31] ^ op_branch_slt_is_unsigned) & ~(in_b[31] ^ op_branch_slt_is_unsigned);
    wire lt_ltu_result = sign_cmp_result | (~(in_a[31] ^ in_b[31]) & adder_sum[31] & ~eq_result);
    wire branch_slt_sltu_result = (op_is_branch_eq_ne ? eq_result : lt_ltu_result) ^ op_is_branch_gt;
    always_ff @(posedge clk) begin
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
    logic f_past_valid;
    initial f_past_valid = 0;
    always_ff @(posedge clk) begin
        f_past_valid = 1;
    end

    /* Validate logic */
    always_ff @(posedge clk) begin
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
                        assert($signed(out) == ($signed($past(in_a)) >>> $past(in_b[4:0])));
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
