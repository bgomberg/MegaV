`ifndef __SHIFTER_SV__
`define __SHIFTER_SV__

`include "cells/mux2.sv"

function [31:0] reverse_bit_order(input [31:0] data);
integer i;
begin
    for (i = 0; i < 32; i = i + 1) begin
        reverse_bit_order[31-i] = data[i];
    end
end
endfunction

/*
 * A shifter.
 */
module shifter(
    input [31:0] in, // Input
    input [4:0] amount, // Amount
    input is_right, // Right shift
    input is_right_arithmetic, // Right arithmetic shift
    output [31:0] out // Result
);

    wire shift_in_bit = is_right_arithmetic & in[31];
    wire [31:0] shift_in;
    mux2 #(.BITS(32)) shift_in_mux(
        .a(in),
        .b(reverse_bit_order(in)),
        .select(is_right),
        .out(shift_in)
    );
    wire [31:0] shift_int_1;
    mux2 #(.BITS(32)) shift_int_1_mux(
        .a(shift_in),
        .b({shift_in[15:0], {16{shift_in_bit}}}),
        .select(amount[4]),
        .out(shift_int_1)
    );
    wire [31:0] shift_int_2;
    mux2 #(.BITS(32)) shift_int_2_mux(
        .a(shift_int_1),
        .b({shift_int_1[23:0], {8{shift_in_bit}}}),
        .select(amount[3]),
        .out(shift_int_2)
    );
    wire [31:0] shift_int_3;
    mux2 #(.BITS(32)) shift_int_3_mux(
        .a(shift_int_2),
        .b({shift_int_2[27:0], {4{shift_in_bit}}}),
        .select(amount[2]),
        .out(shift_int_3)
    );
    wire [31:0] shift_int_4;
    mux2 #(.BITS(32)) shift_int_4_mux(
        .a(shift_int_3),
        .b({shift_int_3[29:0], {2{shift_in_bit}}}),
        .select(amount[1]),
        .out(shift_int_4)
    );
    wire [31:0] shift_int_5;
    mux2 #(.BITS(32)) shift_int_5_mux(
        .a(shift_int_4),
        .b({shift_int_4[30:0], shift_in_bit}),
        .select(amount[0]),
        .out(shift_int_5)
    );
    mux2 #(.BITS(32)) out_mux(
        .a(shift_int_5),
        .b(reverse_bit_order(shift_int_5)),
        .select(is_right),
        .out(out)
    );

`ifdef FORMAL
    /* Validate logic */
    always_comb begin
        if (is_right && is_right_arithmetic) begin
            assert($signed(out) == ($signed(in) >>> amount));
        end else if (is_right) begin
            assert(out == (in >> amount));
        end else begin
            assume(!is_right_arithmetic);
            assert(out == (in << amount));
        end
    end
`endif

endmodule

`endif
