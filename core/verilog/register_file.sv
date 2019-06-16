/*
 * A 16x32 register file with r0 tied to 0 and two read ports and one write port.
 */
module register_file(
    input clk, // Clock signal
    input write_en, // Perform a write operation
    input [3:0] write_addr, // Address to write to
    input [31:0] write_data, // Data to be written
    input [3:0] read_addr_a, // Address to read from (bus A)
    output [31:0] read_data_a, // Data which was read (bus A)
    input [3:0] read_addr_b, // Address to read from (bus B)
    output [31:0] read_data_b // Data which was read (bus B)
);

    /* Internal variables */
    reg [31:0] registers[1:15] /* verilator public */;
    reg [31:0] read_data_a;
    reg [31:0] read_data_b;

    /* Logic */
    always @(posedge clk) begin
        // Handle writes (ignore writes to r0)
        if (write_en && write_addr != 0) begin
            registers[write_addr] <= write_data;
        end

        // Set read_data_a
        if (read_addr_a == 0) begin
            read_data_a <= 0;
        end else begin
            read_data_a <= registers[read_addr_a];
        end

        // Set read_data_b
        if (read_addr_b == 0) begin
            read_data_b <= 0;
        end else begin
            read_data_b <= registers[read_addr_b];
        end
    end

`ifdef FORMAL
    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) begin
        f_past_valid = 1;
    end

    (* anyconst *) wire [3:0] f_read_addr_a;
    (* anyconst *) wire [3:0] f_read_addr_b;
    (* anyconst *) wire [3:0] f_write_addr;
    reg [31:0] f_read_data_a;
    reg [31:0] f_read_data_b;
    initial f_read_data_a = registers[f_read_addr_a];
    initial f_read_data_b = registers[f_read_addr_b];
    always @(*) begin
        if (f_read_addr_a != 0) begin
            assert(registers[f_read_addr_a] == f_read_data_a);
        end
        if (f_read_addr_b != 0) begin
            assert(registers[f_read_addr_b] == f_read_data_b);
        end
    end

    /* Read path */
    always @(posedge clk) begin
        if (f_past_valid) begin
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

    /* Write path */
    always @(posedge clk) begin
        if (f_past_valid && $past(write_en) && ($past(write_addr) != 0) && ($past(write_addr) == f_write_addr)) begin
            assert(registers[$past(write_addr)] == $past(write_data));
        end
    end
`endif

endmodule
