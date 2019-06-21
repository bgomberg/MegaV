#include "mem.h"

#include <Vmodule.h>

#include <stdio.h>

typedef struct {
	bool is_write;
	int addr;
	int write_data;
	bool read_is_unsigned;
	bool op_1;
	bool op_0;
	bool busy;
	int debug_cycles;
	uint32_t read_result;
} mem_op_t;

static mem_op_t m_mem_op;
static uint8_t m_flash[0xC0];
static uint8_t m_ram[0x40];

void mem_load_flash(const void *data, size_t len) {
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

void mem_op_setup(svBit is_write, svBit op_1, svBit op_0, int addr, int write_data, svBit read_is_unsigned) {
	if (m_mem_op.busy) {
		printf("!!! Invalid mem op\n");
		exit(1);
	}
	m_mem_op = (mem_op_t) {
		.is_write = is_write,
		.addr = addr,
		.write_data = write_data,
		.read_is_unsigned = read_is_unsigned,
		.op_1 = op_1,
		.op_0 = op_0,
		.busy = true,
		.debug_cycles = 1,
		.read_result = UINT32_MAX,
	};
}

svBit mem_op_is_busy(void) {
	if (m_mem_op.debug_cycles++ == 5) {
		if (m_mem_op.is_write) {
			mem_write_byte(m_mem_op.addr, m_mem_op.write_data & 0xff);
			if (m_mem_op.op_0 || m_mem_op.op_1) {
				mem_write_byte(m_mem_op.addr + 1, (m_mem_op.write_data >> 8) & 0xff);
			}
			if (m_mem_op.op_1) {
				mem_write_byte(m_mem_op.addr + 2, (m_mem_op.write_data >> 16) & 0xff);
				mem_write_byte(m_mem_op.addr + 3, (m_mem_op.write_data >> 24) & 0xff);
			}
		} else {
			m_mem_op.read_result = mem_read_byte(m_mem_op.addr);
			if (m_mem_op.op_0 || m_mem_op.op_1) {
				m_mem_op.read_result |= (mem_read_byte(m_mem_op.addr + 1) << 8);
			}
			if (m_mem_op.op_1) {
				m_mem_op.read_result |= (mem_read_byte(m_mem_op.addr + 2) << 16);
				m_mem_op.read_result |= (mem_read_byte(m_mem_op.addr + 3) << 24);
			}
			if (m_mem_op.op_1 == 0 && m_mem_op.op_0 == 0) {
				if (!m_mem_op.read_is_unsigned && (m_mem_op.read_result & 0x80)) {
					m_mem_op.read_result |= 0xFFFFFF00;
				}
			} else if (m_mem_op.op_1 == 0 && m_mem_op.op_0 == 1) {
				if (!m_mem_op.read_is_unsigned && (m_mem_op.read_result & 0x8000)) {
					m_mem_op.read_result |= 0xFFFF0000;
				}
			}
		}
		m_mem_op.busy = false;
	}
	return m_mem_op.busy;
}

int mem_read_get_result(void) {
	if (m_mem_op.busy || m_mem_op.is_write) {
		printf("!!! Invalid mem result (%d, %d)\n");
		exit(1);
	}
	return m_mem_op.read_result;
}
