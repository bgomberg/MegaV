`include "cells/demux4.sv"
`include "cells/dffe.sv"
`include "cells/mux2.sv"
`include "cells/mux8.sv"

/*
 * A 16x32 register file with r0 tied to 0 and two read ports and one write port.
 */
module register_file(
    input logic clk, // Clock signal
    input logic reset_n, // Reset signal (active low)
    input logic enable_n, // Enable (active low)
    input logic write_en, // Perform a write operation
    input logic [3:0] write_addr, // Address to write to
    input logic [31:0] write_data, // Data to be written
    input logic [3:0] read_addr_a, // Address to read from (bus A)
    output logic [31:0] read_data_a, // Data which was read (bus A)
    input logic [3:0] read_addr_b, // Address to read from (bus B)
    output logic [31:0] read_data_b // Data which was read (bus B)
);

    /* Registers */
    logic [31:0] r1 /* verilator public */;
    logic [31:0] r2 /* verilator public */;
    logic [31:0] r3 /* verilator public */;
    logic [31:0] r4 /* verilator public */;
    logic [31:0] r5 /* verilator public */;
    logic [31:0] r6 /* verilator public */;
    logic [31:0] r7 /* verilator public */;
    logic [31:0] r8 /* verilator public */;
    logic [31:0] r9 /* verilator public */;
    logic [31:0] r10 /* verilator public */;
    logic [31:0] r11 /* verilator public */;
    logic [31:0] r12 /* verilator public */;
    logic [31:0] r13 /* verilator public */;
    logic [31:0] r14 /* verilator public */;
    logic [31:0] r15 /* verilator public */;

    /* Write Address */
    wire write_addr_is_r0_r3_n;
    wire write_addr_is_r4_r7_n;
    wire write_addr_is_r8_r11_n;
    wire write_addr_is_r12_r15_n;
    demux4 write_addr_upper_demux(
        .in_n(enable_n | ~write_en),
        .s1(write_addr[2]),
        .s2(write_addr[3]),
        .out1_n(write_addr_is_r0_r3_n),
        .out2_n(write_addr_is_r4_r7_n),
        .out3_n(write_addr_is_r8_r11_n),
        .out4_n(write_addr_is_r12_r15_n)
    );

    /* Write r0-r3 */
    /* verilator lint_off UNUSED */
    wire write_addr_is_r0_n;
    /* verilator lint_on UNUSED */
    wire write_addr_is_r1_n;
    wire write_addr_is_r2_n;
    wire write_addr_is_r3_n;
    demux4 write_addr_r0_r3_demux(
        .in_n(write_addr_is_r0_r3_n),
        .s1(write_addr[0]),
        .s2(write_addr[1]),
        .out1_n(write_addr_is_r0_n),
        .out2_n(write_addr_is_r1_n),
        .out3_n(write_addr_is_r2_n),
        .out4_n(write_addr_is_r3_n)
    );
    dffe #(.BITS(32)) r1_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(write_addr_is_r1_n),
        .in(write_data),
        .out(r1)
    );
    dffe #(.BITS(32)) r2_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(write_addr_is_r2_n),
        .in(write_data),
        .out(r2)
    );
    dffe #(.BITS(32)) r3_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(write_addr_is_r3_n),
        .in(write_data),
        .out(r3)
    );

    /* Write r4-r7 */
    wire write_addr_is_r4_n;
    wire write_addr_is_r5_n;
    wire write_addr_is_r6_n;
    wire write_addr_is_r7_n;
    demux4 write_addr_r4_r7_demux(
        .in_n(write_addr_is_r4_r7_n),
        .s1(write_addr[0]),
        .s2(write_addr[1]),
        .out1_n(write_addr_is_r4_n),
        .out2_n(write_addr_is_r5_n),
        .out3_n(write_addr_is_r6_n),
        .out4_n(write_addr_is_r7_n)
    );
    dffe #(.BITS(32)) r4_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(write_addr_is_r4_n),
        .in(write_data),
        .out(r4)
    );
    dffe #(.BITS(32)) r5_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(write_addr_is_r5_n),
        .in(write_data),
        .out(r5)
    );
    dffe #(.BITS(32)) r6_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(write_addr_is_r6_n),
        .in(write_data),
        .out(r6)
    );
    dffe #(.BITS(32)) r7_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(write_addr_is_r7_n),
        .in(write_data),
        .out(r7)
    );

    /* Write r8-r11 */
    wire write_addr_is_r8_n;
    wire write_addr_is_r9_n;
    wire write_addr_is_r10_n;
    wire write_addr_is_r11_n;
    demux4 write_addr_r8_r11_demux(
        .in_n(write_addr_is_r8_r11_n),
        .s1(write_addr[0]),
        .s2(write_addr[1]),
        .out1_n(write_addr_is_r8_n),
        .out2_n(write_addr_is_r9_n),
        .out3_n(write_addr_is_r10_n),
        .out4_n(write_addr_is_r11_n)
    );
    dffe #(.BITS(32)) r8_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(write_addr_is_r8_n),
        .in(write_data),
        .out(r8)
    );
    dffe #(.BITS(32)) r9_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(write_addr_is_r9_n),
        .in(write_data),
        .out(r9)
    );
    dffe #(.BITS(32)) r10_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(write_addr_is_r10_n),
        .in(write_data),
        .out(r10)
    );
    dffe #(.BITS(32)) r11_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(write_addr_is_r11_n),
        .in(write_data),
        .out(r11)
    );

    /* Write r12-r15 */
    wire write_addr_is_r12_n;
    wire write_addr_is_r13_n;
    wire write_addr_is_r14_n;
    wire write_addr_is_r15_n;
    demux4 write_addr_r12_r15_demux(
        .in_n(write_addr_is_r12_r15_n),
        .s1(write_addr[0]),
        .s2(write_addr[1]),
        .out1_n(write_addr_is_r12_n),
        .out2_n(write_addr_is_r13_n),
        .out3_n(write_addr_is_r14_n),
        .out4_n(write_addr_is_r15_n)
    );
    dffe #(.BITS(32)) r12_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(write_addr_is_r12_n),
        .in(write_data),
        .out(r12)
    );
    dffe #(.BITS(32)) r13_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(write_addr_is_r13_n),
        .in(write_data),
        .out(r13)
    );
    dffe #(.BITS(32)) r14_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(write_addr_is_r14_n),
        .in(write_data),
        .out(r14)
    );
    dffe #(.BITS(32)) r15_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(write_addr_is_r15_n),
        .in(write_data),
        .out(r15)
    );

    /* Read A */
    wire [31:0] read_data_a_lower_addr;
    mux8 #(.BITS(32)) read_data_a_lower_addr_mux(
        .d1(32'b0),
        .d2(r1),
        .d3(r2),
        .d4(r3),
        .d5(r4),
        .d6(r5),
        .d7(r6),
        .d8(r7),
        .select(read_addr_a[2:0]),
        .out(read_data_a_lower_addr)
    );
    wire [31:0] read_data_a_upper_addr;
    mux8 #(.BITS(32)) read_data_a_upper_addr_mux(
        .d1(r8),
        .d2(r9),
        .d3(r10),
        .d4(r11),
        .d5(r12),
        .d6(r13),
        .d7(r14),
        .d8(r15),
        .select(read_addr_a[2:0]),
        .out(read_data_a_upper_addr)
    );
    wire [31:0] next_read_data_a;
    mux2 #(.BITS(32)) read_data_a_mux(
        .a(read_data_a_lower_addr),
        .b(read_data_a_upper_addr),
        .select(read_addr_a[3]),
        .out(next_read_data_a)
    );
    dffe #(.BITS(32)) read_data_a_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_read_data_a),
        .out(read_data_a)
    );

    /* Read B */
    wire [31:0] read_data_b_lower_addr;
    mux8 #(.BITS(32)) read_data_b_lower_addr_mux(
        .d1(32'b0),
        .d2(r1),
        .d3(r2),
        .d4(r3),
        .d5(r4),
        .d6(r5),
        .d7(r6),
        .d8(r7),
        .select(read_addr_b[2:0]),
        .out(read_data_b_lower_addr)
    );
    wire [31:0] read_data_b_upper_addr;
    mux8 #(.BITS(32)) read_data_b_upper_addr_mux(
        .d1(r8),
        .d2(r9),
        .d3(r10),
        .d4(r11),
        .d5(r12),
        .d6(r13),
        .d7(r14),
        .d8(r15),
        .select(read_addr_b[2:0]),
        .out(read_data_b_upper_addr)
    );
    wire [31:0] next_read_data_b;
    mux2 #(.BITS(32)) read_data_b_mux(
        .a(read_data_b_lower_addr),
        .b(read_data_b_upper_addr),
        .select(read_addr_b[3]),
        .out(next_read_data_b)
    );
    dffe #(.BITS(32)) read_data_b_dffe(
        .clk(clk),
        .clear_n(reset_n),
        .enable_n(enable_n),
        .in(next_read_data_b),
        .out(read_data_b)
    );

`ifdef FORMAL
    initial assume(~reset_n);
    logic f_past_valid;
    initial f_past_valid = 0;
    always_ff @(posedge clk) begin
        f_past_valid = 1;
    end

    (* anyconst *) wire [3:0] f_read_addr_a;
    (* anyconst *) wire [3:0] f_read_addr_b;
    (* anyconst *) wire [3:0] f_write_addr;
    wire [31:0] registers[1:15];
    assign registers[1] = r1;
    assign registers[2] = r2;
    assign registers[3] = r3;
    assign registers[4] = r4;
    assign registers[5] = r5;
    assign registers[6] = r6;
    assign registers[7] = r7;
    assign registers[8] = r8;
    assign registers[9] = r9;
    assign registers[10] = r10;
    assign registers[11] = r11;
    assign registers[12] = r12;
    assign registers[13] = r13;
    assign registers[14] = r14;
    assign registers[15] = r15;
    logic [31:0] f_read_data_a;
    logic [31:0] f_read_data_b;
    initial f_read_data_a = registers[f_read_addr_a];
    initial f_read_data_b = registers[f_read_addr_b];
    always_comb begin
        if (f_read_addr_a != 0) begin
            assert(registers[f_read_addr_a] == f_read_data_a);
        end
        if (f_read_addr_b != 0) begin
            assert(registers[f_read_addr_b] == f_read_data_b);
        end
    end

    always_ff @(posedge clk) begin
        if (f_past_valid & $past(reset_n)) begin
            assume($past(~enable_n));
            if ($past(write_en)) begin
                /* Write path */
                if (($past(write_addr) != 0) && ($past(write_addr) == f_write_addr)) begin
                    assert(registers[$past(write_addr)] == $past(write_data));
                end
            end else begin
                /* Read path */
                if ($past(read_addr_a) == 0) begin
                    assert(read_data_a == 0);
                end else if ($past(read_addr_a) == f_read_addr_a) begin
                    if ($past(read_addr_a) == $past(write_addr)) begin
                        assert(read_data_a == $past(f_read_data_a));
                    end else begin
                        assert(read_data_a == f_read_data_a);
                    end
                end
                if ($past(read_addr_b) == 0) begin
                    assert(read_data_b == 0);
                end else if ($past(read_addr_b) == f_read_addr_b) begin
                    if ($past(read_addr_b) == $past(write_addr)) begin
                        assert(read_data_b == $past(f_read_data_b));
                    end else begin
                        assert(read_data_b == f_read_data_b);
                    end
                end
            end
        end
    end
`endif

endmodule
