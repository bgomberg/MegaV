/*
 * A 16x32 register file with r0 tied to 0 and two read ports and one write port.
 */
module register_file(
    input clk, // Clock signal
    input reset_n, // Reset signal (active low)
    input available, // Operation available
    input write_en, // Perform a write operation
    input [3:0] write_addr, // Address to write to
    input [31:0] write_data, // Data to be written
    input [3:0] read_addr_a, // Address to read from (bus A)
    output [31:0] read_data_a, // Data which was read (bus A)
    input [3:0] read_addr_b, // Address to read from (bus B)
    output [31:0] read_data_b // Data which was read (bus B)
);

    /* Outputs */
    reg [31:0] read_data_a;
    reg [31:0] read_data_b;

    /* Internal variables */
    reg [31:0] registers[1:15] /* verilator public */;

    /* Logic */
    always @(posedge clk) begin
        if (~reset_n) begin
            // Reset
            registers[1] <= 32'b0;
            registers[2] <= 32'b0;
            registers[3] <= 32'b0;
            registers[4] <= 32'b0;
            registers[5] <= 32'b0;
            registers[6] <= 32'b0;
            registers[7] <= 32'b0;
            registers[8] <= 32'b0;
            registers[9] <= 32'b0;
            registers[10] <= 32'b0;
            registers[11] <= 32'b0;
            registers[12] <= 32'b0;
            registers[13] <= 32'b0;
            registers[14] <= 32'b0;
            registers[15] <= 32'b0;
        end else if (available) begin
            if (write_en & (write_addr != 0)) begin
                registers[write_addr] <= write_data;
            end
            read_data_a <= (read_addr_a == 0) ? 32'b0 : registers[read_addr_a];
            read_data_b <= (read_addr_b == 0) ? 32'b0 : registers[read_addr_b];
        end
    end

`ifdef FORMAL
    initial assume(~reset_n);
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

    always @(posedge clk) begin
        if (f_past_valid & $past(reset_n)) begin
            assume($past(available));
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
