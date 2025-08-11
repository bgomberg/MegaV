(* blackbox *)
module BUF (A, Y);
	input A;
	output Y;
	assign Y = A;
endmodule

(* blackbox *)
module NOT (A, Y);
	input A;
	output Y;
	assign Y = ~A;
endmodule

(* blackbox *)
module MUX2 (A, B, S, Y);
	input A, B, S;
	output Y;
	assign Y = (A & B) | (S & B) | (~S & A);
endmodule

(* blackbox *)
module XOR (A, B, Y);
	input A, B;
	output Y;
	assign Y = A ^ B;
endmodule

(* blackbox *)
module AND (A, B, Y);
	input A, B;
	output Y;
	assign Y = A & B;
endmodule

(* blackbox *)
module OR (A, B, Y);
	input A, B;
	output Y;
	assign Y = A | B;
endmodule

(* blackbox *)
module NAND (A, B, Y);
	input A, B;
	output Y;
	assign Y = ~(A & B);
endmodule

(* blackbox *)
module NOR (A, B, Y);
	input A, B;
	output Y;
	assign Y = ~(A | B);
endmodule

(* blackbox *)
module AND3 (A, B, C, Y);
	input A, B, C;
	output Y;
	assign Y = A & B & C;
endmodule

(* blackbox *)
module OR3 (A, B, C, Y);
	input A, B, C;
	output Y;
	assign Y = A | B | C;
endmodule

(* blackbox *)
module NAND3 (A, B, C, Y);
	input A, B, C;
	output Y;
	assign Y = ~(A & B & C);
endmodule

(* blackbox *)
module NOR3 (A, B, C, Y);
	input A, B, C;
	output Y;
	assign Y = ~(A | B | C);
endmodule

(* blackbox *)
module AND4 (A, B, C, D, Y);
	input A, B, C, D;
	output Y;
	assign Y = A & B & C & D;
endmodule

(* blackbox *)
module OR4 (A, B, C, D, Y);
	input A, B, C, D;
	output Y;
	assign Y = A | B | C | D;
endmodule

(* blackbox *)
module NAND4 (A, B, C, D, Y);
	input A, B, C, D;
	output Y;
	assign Y = ~(A & B & C & D);
endmodule

(* blackbox *)
module NOR4 (A, B, C, D, Y);
	input A, B, C, D;
	output Y;
	assign Y = ~(A | B | C | D);
endmodule

(* blackbox *)
module D_FF (D, CLK, Q);
    input D;
    input CLK;
    output Q;
endmodule

(* blackbox *)
module D_FF_S (D, CLK, PREn, Q);
    input D;
    input CLK;
    input PREn;
    output Q;
endmodule

(* blackbox *)
module D_FF_R (D, CLK, CLRn, Q);
    input D;
    input CLK;
    input CLRn;
    output Q;
endmodule

(* blackbox *)
module D_FF_SR (D, CLK, PREn, CLRn, Q);
    input D;
    input CLK;
    input PREn;
    input CLRn;
    output Q;
endmodule

(* blackbox *)
module JK_FF (J, K, CLK, Q);
    input J;
    input K;
    input CLK;
    output Q;
endmodule

(* blackbox *)
module JK_FF_C (J, K, CLK, CLRn, Q);
    input J;
    input K;
    input CLK;
    input CLRn;
    output Q;
endmodule
