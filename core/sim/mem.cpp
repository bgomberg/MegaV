#include "mem.h"

#include <Vmodule_memory.h>

#include <stdio.h>

#define BIT(VAL, POS) \
	(((VAL) >> (POS)) & 1)

typedef struct {
	bool busy;
	int debug_cycles;
	uint32_t read_result;
} mem_op_t;

static const Vmodule_core* m_core;
static mem_op_t m_mem_op;
static uint8_t m_flash[0xC0];
static uint8_t m_ram[0x40];

void mem_init(const Vmodule_core* core, const void *data, size_t len) {
	m_core = core;
	if (len % sizeof(uint32_t) || len > sizeof(m_flash)) {
		fprintf(stderr, "Invalid flash length (%zu)\n", len);
		exit(1);
	}
	memcpy(m_flash, data, len);
}

void mem_dump_ram(void) {
	printf("Memory:\n");
	for (int i = 0xC0; i < 0x100; i += 4) {
		uint32_t value;
		memcpy(&value, &m_ram[i-0xC0], sizeof(value));
		if (value) {
			printf("  [0x%x] = 0x%x\n", i, value);
		}
	}
}

static uint8_t mem_read_byte(int addr) {
	if (addr >= 0 && addr < 0xC0) {
		return m_flash[addr];
	} else if (addr >= 0xC0 && addr < 0x100) {
		return m_ram[addr-0xC0];
	} else {
		printf("!!! Invalid read addr\n");
		exit(1);
	}
}

static void mem_write_byte(int addr, char data) {
	if (addr >= 0xC0 && addr < 0x100) {
		m_ram[addr-0xC0] = data;
	} else {
		printf("!!! Invalid write addr\n");
		exit(1);
	}
}

svBit mem_op_setup() {
	if (!m_core->mem_module->available || m_core->mem_module->started) {
		return 0;
	}
	const uint32_t addr = m_core->mem_module->addr;
	const uint8_t op = m_core->mem_module->op;
	if ((BIT(op, 1) & (BIT(addr, 1) | BIT(addr, 0))) | (BIT(op, 0) & BIT(addr, 0))) {
		// Address misaligned
		return 1;
	}
	if (addr < 0 || (m_core->mem_module->is_write && addr < 0xC0) || addr >= 0x100) {
		// Access fault
		return 1;
	}
	m_mem_op = (mem_op_t) {
		.busy = true,
		.debug_cycles = 0,
		.read_result = UINT32_MAX,
	};
	return 0;
}

svBit mem_op_is_busy(void) {
	if (m_mem_op.busy && ++m_mem_op.debug_cycles == 5) {
		const uint32_t addr = m_core->mem_module->addr;
		const uint8_t op = m_core->mem_module->op;
		if (m_core->mem_module->is_write) {
			const uint32_t write_data = m_core->mem_module->in;
			mem_write_byte(addr, write_data & 0xff);
			if (BIT(op, 0) || BIT(op, 1)) {
				mem_write_byte(addr + 1, (write_data >> 8) & 0xff);
			}
			if (BIT(op, 1)) {
				mem_write_byte(addr + 2, (write_data >> 16) & 0xff);
				mem_write_byte(addr + 3, (write_data >> 24) & 0xff);
			}
		} else {
			m_mem_op.read_result = mem_read_byte(addr);
			if (BIT(op, 0) || BIT(op, 1)) {
				m_mem_op.read_result |= (mem_read_byte(addr + 1) << 8);
			}
			if (BIT(op, 1)) {
				m_mem_op.read_result |= (mem_read_byte(addr + 2) << 16);
				m_mem_op.read_result |= (mem_read_byte(addr + 3) << 24);
			}
			if (!BIT(op, 1) && !BIT(op, 0)) {
				if (!m_core->mem_module->is_unsigned && (m_mem_op.read_result & 0x80)) {
					m_mem_op.read_result |= 0xFFFFFF00;
				}
			} else if (!BIT(op, 1) && BIT(op, 0)) {
				if (!m_core->mem_module->is_unsigned && (m_mem_op.read_result & 0x8000)) {
					m_mem_op.read_result |= 0xFFFF0000;
				}
			}
		}
		m_mem_op.busy = false;
	}
	return m_mem_op.busy;
}

int mem_read_get_result(void) {
	return m_mem_op.read_result;
}
