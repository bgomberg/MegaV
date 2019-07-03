#pragma once

#include <Vmodule.h>
#include <Vmodule_core.h>

#include <stddef.h>

void mem_init(const Vmodule_core* core, const void *data, size_t len);

void mem_dump_ram(void);
