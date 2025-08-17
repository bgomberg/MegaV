`ifdef verilator
import "DPI-C" function void mem_get_access_fault_dpi_c(
    input bit [31:0] addr,
    input bit is_write,
    output bit result
);
import "DPI-C" function int mem_get_read_result_dpi_c(
    input bit enable,
    input bit [31:0] addr,
    input bit [31:0] write_val,
    input bit [1:0] op_size,
    input bit is_write
);
`endif

function bit mem_get_access_fault(
    input bit[31:0] addr,
    input bit is_write
);
`ifdef verilator
    bit result;
    mem_get_access_fault_dpi_c(addr, is_write, result);
    mem_get_access_fault = result;
`else
    mem_get_access_fault = 3'b0;
`endif
endfunction
function bit[31:0] mem_get_read_result(
    input bit enable,
    input bit[31:0] addr,
    input bit[31:0] write_val,
    input bit[1:0] op_size,
    input bit is_write
);
`ifdef verilator
    mem_get_read_result = mem_get_read_result_dpi_c(enable, addr, write_val, op_size, is_write);
`else
    mem_get_read_result = 32'b0;
`endif
endfunction

/*
 * External memory
 */
module external_memory(
    input enable, // Enable
    input is_write, // Whether or not the operation is a write
    input [1:0] op_size /* verilator public */, // Size of the operation
    input [31:0] addr /* verilator public */, // Address to access
    input [31:0] in, // Input data
    output [31:0] out, // Output data
    output access_fault // Access fault
);

    assign access_fault = mem_get_access_fault(addr, is_write);
    assign out = mem_get_read_result(enable, addr, in, op_size, is_write);
endmodule
