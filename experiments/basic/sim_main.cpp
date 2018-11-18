#include <core_wrapper.hpp>

#include <verilated.h>

#include <stdio.h>

int main(int argc, char** argv) {
	Verilated::commandArgs(argc, argv);
	CoreWrapper core;

	core->a = 0x2;
	core->b = 0x1;
	core.tick();
	printf("0x%x + 0x%x = 0x%x\n", core->a, core->b, core->result);

	core->a = 0x1;
	core->b = 0x0;
	core.tick();
	printf("0x%x + 0x%x = 0x%x\n", core->a, core->b, core->result);

	return 0;
}
