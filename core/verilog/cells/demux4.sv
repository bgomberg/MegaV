`ifndef __DEMUX4_SV__
`define __DEMUX4_SV__

/*
 * 1:4 DEMUX
 */
module demux4 #(
    parameter BITS = 1
) (
    input [BITS-1:0] in_n, // Input (active low)
    input [BITS-1:0] s1, // Select 1
    input [BITS-1:0] s2, // Select 2
    output [BITS-1:0] out1_n, // Output 1 (active low)
    output [BITS-1:0] out2_n, // Output 2 (active low)
    output [BITS-1:0] out3_n, // Output 3 (active low)
    output [BITS-1:0] out4_n // Output 4 (active low)
);

    assign out1_n = in_n | s1 | s2;
    assign out2_n = in_n | ~s1 | s2;
    assign out3_n = in_n | s1 | ~s2;
    assign out4_n = in_n | ~s1 | ~s2;

`ifdef FORMAL
    /* Validate logic */
    always_comb begin
        if (in_n) begin
            assert(out1_n);
            assert(out2_n);
            assert(out3_n);
            assert(out4_n);
        end else if (~s1 & ~s2) begin
            assert(~out1_n);
            assert(out2_n);
            assert(out3_n);
            assert(out4_n);
        end else if (s1 & ~s2) begin
            assert(out1_n);
            assert(~out2_n);
            assert(out3_n);
            assert(out4_n);
        end else if (~s1 & s2) begin
            assert(out1_n);
            assert(out2_n);
            assert(~out3_n);
            assert(out4_n);
        end else if (s1 & s2) begin
            assert(out1_n);
            assert(out2_n);
            assert(out3_n);
            assert(~out4_n);
        end
    end
`endif

endmodule

`endif
