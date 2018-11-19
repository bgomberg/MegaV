/*
 * A 16x32 register file with r0 tied to 0 and two read ports and one write port.
 */
module register_file #(
    parameter ADDR_WIDTH = 4,
    parameter DATA_WIDTH = 32,
    parameter NUM_REGS = 16
)(
    input reset, // Synchronous reset line
    input clk, // Clock signal
    input write, // Perform a write operation
    input [ADDR_WIDTH-1:0] write_addr, // Address to write to
    input [DATA_WIDTH-1:0] write_data, // Data to be written
    input [ADDR_WIDTH-1:0] read_addr_a, // Address to read from (bus A)
    output [DATA_WIDTH-1:0] read_data_a, // Data which was read (bus A)
    input [ADDR_WIDTH-1:0] read_addr_b, // Address to read from (bus B)
    output [DATA_WIDTH-1:0] read_data_b // Data which was read (bus B)
);

    /* Internal variables */
    reg [DATA_WIDTH-1:0] registers[NUM_REGS];
    reg [DATA_WIDTH-1:0] read_data_a;
    reg [DATA_WIDTH-1:0] read_data_b;

    /* Logic */
    integer i;
    always @(posedge clk) begin
        if (reset) begin
            read_data_a <= 0;
            read_data_b <= 0;
            for (i = 0; i < NUM_REGS; i++) begin
                registers[i] <= 0;
            end
        end
        else if (write && write_addr != 0) begin
            registers[write_addr] <= write_data;
        end
        else if (!write) begin
            read_data_a <= (read_addr_a == 0) ? 0 : registers[read_addr_a];
            read_data_b <= (read_addr_b == 0) ? 0 : registers[read_addr_b];
        end
    end

`ifdef FORMAL
    reg f_past_valid;
    initial f_past_valid = 0;
    always @(posedge clk) begin
        f_past_valid = 1;
    end

    initial assume(reset);
    always @(*) begin
        if (!f_past_valid) begin
            assume(reset);
        end
    end

    /* Read path */
    always @(posedge clk) begin
    	if (f_past_valid && !$past(reset) && !$past(write)) begin
            if ($past(read_addr_a) == 0) begin
                assert(read_data_a == 0);
            end
            else begin
                assert(read_data_a == registers[$past(read_addr_a)]);
            end
            if ($past(read_addr_b) == 0) begin
                assert(read_data_b == 0);
            end
            else begin
                assert(read_data_b == registers[$past(read_addr_b)]);
            end
    	end
    end

    /* Write path */
    always @(posedge clk) begin
    	if (f_past_valid && !$past(reset)) begin
            if ($past(write) && $past(write_addr) != 0) begin
                assert(registers[$past(write_addr)] == $past(write_data));
            end
    	end
    end

    /* Register 0 */
    always @(posedge clk) begin
    	if (f_past_valid && !$past(reset)) begin
            assert(registers[0] == 0);
        end
    end
`endif

endmodule
