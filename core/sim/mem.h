#pragma once

#include <Vmodule.h>
#include <Vmodule_core.h>

#include <inttypes.h>
#include <stddef.h>

void mem_init(const Vmodule_core* core, const void *data, size_t len);

uint32_t mem_read(uint32_t addr);

void mem_dump_ram(void);
