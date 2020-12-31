#pragma once

#include <inttypes.h>
#include <stddef.h>

void mem_init(const void *data, size_t len);

uint32_t mem_read(uint32_t addr);

void mem_dump_ram(void);
