#include <core_wrapper.hpp>

#include <verilated.h>

#include <stdio.h>

int main(int argc, char** argv) {
	Verilated::commandArgs(argc, argv);
	CoreWrapper core;

	// core->write = 0;
	// core->write_addr = 0;
	// core->write_data = 0;
	// core->read_addr_a = 0;
	// core->read_addr_b = 1;
	// core.tick();
	// printf("Reg %d is 0x%x and reg %d is 0x%x\n", core->read_addr_a, core->read_data_a, core->read_addr_b, core->read_data_b);
	//
	// core->write_addr = 1;
	// core->write_data = 0x12345678;
	// core->write = 1;
	// core.tick();
	// printf("Set reg %d to 0x%x\n", core->write_addr, core->write_data);
	//
	// core->write = 0;
	// core->write_addr = 0;
	// core->write_data = 0;
	// core.tick();
	// printf("Reg %d is 0x%x and reg %d is 0x%x\n", core->read_addr_a, core->read_data_a, core->read_addr_b, core->read_data_b);

	return 0;
}
