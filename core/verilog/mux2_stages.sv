`ifndef __MUX2_STAGES_SV__
`define __MUX2_STAGES_SV__

`include "stages.sv"
`include "mux2.sv"

module mux2_stages(
    input [`NUM_STAGES-1:0] a, // Input A
    input [`NUM_STAGES-1:0] b, // Input B
    input select, // Select
    output [`NUM_STAGES-1:0] out // Output
);

    mux2 #(.DATA_WIDTH(`NUM_STAGES)) mux(
        .a(a),
        .b(b),
        .select(select),
        .out(out)
    );

endmodule

`endif
