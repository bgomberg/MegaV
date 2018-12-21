#include <core_wrapper.hpp>

#include <verilated.h>

#include <stdio.h>

int main(int argc, char** argv) {
	Verilated::commandArgs(argc, argv);
	CoreWrapper core;
	return 0;
}
