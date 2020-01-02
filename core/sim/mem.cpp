#include "mem.h"

#include <Vmodule_memory.h>

#include <stdio.h>

#define FLASH_START   0x00000000
#define FLASH_SIZE    0x00010000
#define RAM_START     0x10000000
#define RAM_SIZE      0x00002000
#define PERIPH_START  0x20000000
#define PERIPH_SIZE   0x00000008

#define ARRAY_LENGTH(ARR) ((sizeof(ARR)) / sizeof(*(ARR)))
#define BIT(VAL, POS) \
	(((VAL) >> (POS)) & 1)

typedef struct {
  uint32_t start_addr;
  uint32_t length;
  uint8_t(*read_byte_func)(uint32_t offset);
  void(*write_byte_func)(uint32_t offset, uint8_t data);
} mem_region_t;

typedef struct {
	bool busy;
	int debug_cycles;
	uint32_t read_result;
} mem_op_t;

static uint8_t flash_read_byte_func(uint32_t offset);
static void flash_write_byte_func(uint32_t offset, uint8_t data);
static uint8_t ram_read_byte_func(uint32_t offset);
static void ram_write_byte_func(uint32_t offset, uint8_t data);
static void periph_write_byte_func(uint32_t offset, uint8_t data);

static const Vmodule_core* m_core;
static mem_op_t m_mem_op;
static uint8_t m_flash[RAM_START];
static uint8_t m_ram[RAM_SIZE];
static const mem_region_t REGIONS[] = {
  {
    .start_addr = FLASH_START,
    .length = FLASH_SIZE,
    .read_byte_func = flash_read_byte_func,
    .write_byte_func = flash_write_byte_func,
  },
  {
    .start_addr = RAM_START,
    .length = RAM_SIZE,
    .read_byte_func = ram_read_byte_func,
    .write_byte_func = ram_write_byte_func,
  },
  {
    .start_addr = PERIPH_START,
    .length = PERIPH_SIZE,
    .read_byte_func = nullptr,
    .write_byte_func = periph_write_byte_func,
  },
};

void mem_init(const Vmodule_core* core, const void *data, size_t len) {
	m_core = core;
	if (len > sizeof(m_flash)) {
		fprintf(stderr, "Invalid flash length (%zu)\n", len);
		exit(1);
	}
	memcpy(m_flash, data, len);
}

void mem_dump_ram(void) {
	printf("Memory:\n");
	for (int i = 0; i < RAM_SIZE; i += 4) {
		uint32_t value;
		memcpy(&value, &m_ram[i], sizeof(value));
		if (value) {
			printf("  [0x%x] = 0x%x\n", i + RAM_START, value);
		}
	}
}

static const mem_region_t* get_region(uint32_t addr) {
  for (size_t i = 0; i < ARRAY_LENGTH(REGIONS); i++) {
    const mem_region_t* region = &REGIONS[i];
    if (addr >= region->start_addr && addr < region->start_addr + region->length) {
      return region;
    }
  }
  return nullptr;
}

static uint8_t mem_read_byte(uint32_t addr) {
  const mem_region_t* region = get_region(addr);
  if (!region || !region->read_byte_func) {
  	printf("!!! Invalid read addr\n");
  	exit(1);
  }
  return region->read_byte_func(addr - region->start_addr);
}

static void mem_write_byte(uint32_t addr, uint8_t data) {
  const mem_region_t* region = get_region(addr);
  if (!region || !region->write_byte_func) {
  	printf("!!! Invalid write addr\n");
  	exit(1);
  }
  region->write_byte_func(addr - region->start_addr, data);
}

static uint8_t flash_read_byte_func(uint32_t offset) {
  return m_flash[offset];
}

static void flash_write_byte_func(uint32_t offset, uint8_t data) {
  m_flash[offset] = data;
}

static uint8_t ram_read_byte_func(uint32_t offset) {
  return m_ram[offset];
}

static void ram_write_byte_func(uint32_t offset, uint8_t data) {
  m_ram[offset] = data;
}

static void periph_write_byte_func(uint32_t offset, uint8_t data) {
  static uint32_t values[2];
  ((uint8_t*)values)[offset] = data;
  if ((offset % sizeof(uint32_t)) == sizeof(uint32_t) - 1) {
    uint32_t index = offset / sizeof(uint32_t);
    if (index == 0) {
      uint32_t addr = values[index];
      char c;
      while ((c = mem_read_byte(addr++)) != '\0') {
        fputc(c, stdout);
      }
    } else if (index == 1) {
      printf("0x%08x", values[index]);
    }
  }
}

svBit mem_op_setup(void) {
	if (!m_core->mem_module->available || m_core->mem_module->started) {
		return 0;
	}
  const bool is_write = m_core->mem_module->is_write;
	const uint32_t addr = m_core->mem_module->addr;
	const uint8_t op = m_core->mem_module->op;
	if ((BIT(op, 1) & (BIT(addr, 1) | BIT(addr, 0))) | (BIT(op, 0) & BIT(addr, 0))) {
		// Address misaligned
		return 1;
	}
  const mem_region_t* region = get_region(addr);
	if (!region || (is_write && !region->write_byte_func) || (!is_write && !region->read_byte_func)) {
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

uint32_t mem_read(uint32_t addr) {
  uint32_t value = 0;
  for (size_t i = 0; i < sizeof(value); i++) {
    value >>= 8;
    value |= mem_read_byte(addr + i) << 24;
  }
  return value;
}
