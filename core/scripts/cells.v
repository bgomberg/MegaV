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
