#include <core_wrapper.hpp>

#include <verilated.h>

#include <stdio.h>

int main(int argc, char** argv) {
	Verilated::commandArgs(argc, argv);
	CoreWrapper core;

	core->i_a = 0x2;
	core->i_b = 0x1;
	core.tick();
	printf("0x%x + 0x%x = 0x%x\n", core->i_a, core->i_b, core->o_result);

	core->i_a = 0x1;
	core->i_b = 0x0;
	core.tick();
	printf("0x%x + 0x%x = 0x%x\n", core->i_a, core->i_b, core->o_result);

	return 0;
}
