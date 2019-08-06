#pragma once

#include <inttypes.h>

void print_decoded_instr(uint32_t instr);

const char* get_register_name(uint8_t num);
