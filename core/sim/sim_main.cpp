#include <Vmodule.h>
#include <Vmodule_core.h>
#include <Vmodule_memory.h>
#include <Vmodule_register_file.h>

#include <verilated.h>

#include <stdio.h>


static const char * const REGISTER_NAMES[] = {
	"zero",
	"ra",
	"sp",
	"gp",
	"tp",
	"t0",
	"t1",
	"t2",
	"s0/fp",
	"s1",
	"a0",
	"a1",
	"a2",
	"a3",
	"a4",
	"a5",
};
static uint8_t m_flash[0xC0];
static uint8_t m_ram[0x40];


class Core {
public:
	Core() : module_(new Vmodule()) {
		module_->clk = 0;
		module_->reset = 0;
		eval();
	}

	~Core() {
		delete module_;
		module_ = NULL;
	}

	Vmodule_core* operator->() {
		return module_->core;
	}

	void reset() {
		printf("Resetting core\n");
		module_->reset = 1;
		eval();
		tick();
		module_->reset = 0;
		eval();
	}

	void step() {
		// Move forward one clock cycle
		tick();
		if ((*this)->stage_active == (1 << 0)) {
			printf("Executing STAGE_FETCH\n");
			printf("  pc: 0x%x\n", (*this)->pc_pc);
			printf("  instr: 0x%x\n", (*this)->mem_out);
		} else if ((*this)->stage_active == (1 << 1)) {
			printf("Executing STAGE_DECODE\n");
			printf("  imm: 0x%x\n", (*this)->decode_imm);
			printf("  alu_a_mux_position: 0x%x\n", (*this)->decode_alu_a_mux_position);
			printf("  alu_b_mux_position: 0x%x\n", (*this)->decode_alu_b_mux_position);
			printf("  csr_mux_position: 0x%x\n", (*this)->decode_csr_mux_position);
			printf("  wb_pc_mux_position: 0x%x\n", (*this)->decode_wb_pc_mux_position);
			printf("  wb_mux_position: 0x%x\n", (*this)->decode_wb_mux_position);
			printf("  rd_rf_microcode: 0x%x\n", (*this)->decode_rd_rf_microcode);
			printf("  ex_alu_microcode: 0x%x\n", (*this)->decode_ex_alu_microcode);
			printf("  ex_csr_microcode: 0x%x\n", (*this)->decode_ex_csr_microcode);
			printf("  ma_mem_microcode: 0x%x\n", (*this)->decode_ma_mem_microcode);
			printf("  wb_rf_microcode : 0x%x\n", (*this)->decode_wb_rf_microcode);
		} else if ((*this)->stage_active == (1 << 2)) {
			printf("Executing STAGE_READ\n");
			printf("  read_data_a: 0x%x\n", (*this)->rf_read_data_a);
			printf("  read_data_b: 0x%x\n", (*this)->rf_read_data_b);
		} else if ((*this)->stage_active == (1 << 3)) {
			printf("Executing STAGE_EXECUTE\n");
			printf("  out: 0x%x\n", (*this)->alu_out);
			printf("  csr_read_value: 0x%x\n", (*this)->csr_read_value);
			printf("  csr_in: 0x%x\n", (*this)->csr_in);
			printf("  next_pc: 0x%x\n", (*this)->pc_next_pc);
		} else if ((*this)->stage_active == (1 << 4)) {
			printf("Executing STAGE_MEMORY\n");
			printf("  out: 0x%x\n", (*this)->mem_module->out);
		} else if ((*this)->stage_active == (1 << 5)) {
			printf("Executing STAGE_WRITE_BACK\n");
			printf("  write_data: 0x%x\n", (*this)->rf_write_data);
			printf("  next_pc: 0x%x\n", (*this)->pc_next_pc);
		} else {
			printf("!!! UNKNOWN stage\n");
		}

		// Check for faults
		if (module_->fault) {
			printf("!!! FAULT\n");
			exit(1);
		}
	}

private:
	Vmodule* module_;

	void tick() {
		module_->clk = 1;
		eval();
		module_->clk = 0;
		eval();
	}

	void eval() {
		module_->eval();
		if (module_->fault) {
			printf("!!! FAULT\n");
			exit(1);
		}
	}
};


int main(int argc, char** argv) {
	if (argc != 2) {
		fprintf(stderr, "Usage: %s <test_prog.bin>\n", argv[0]);
		return -1;
	}

	// Open the test program file
	FILE *f = fopen(argv[1], "r");

	// Get the size of the test program
	fseek(f, 0L, SEEK_END);
	const size_t file_size = ftell(f);
	rewind(f);
	if (file_size % sizeof(uint32_t) || file_size > sizeof(m_flash)) {
		fclose(f);
		fprintf(stderr, "Invalid test program binary length (%zu)\n", file_size);
		return -1;
	}

	// Read the contents of the test program
	uint32_t *program = (uint32_t *)malloc(file_size);
	if (fread(program, file_size, 1, f) != 1) {
		fclose(f);
		fprintf(stderr, "Error reading file\n");
		return -1;
	}

	// Close the file
	fclose(f);

	// Create our core object, reset it, and copy in the program
	Core core;
	core.reset();
	memcpy(m_flash, program, file_size);
	printf("\n");

	// run until we enter an infinite loop
	uint32_t prev_pc = UINT32_MAX;
	while (core->pc_pc != prev_pc) {
		prev_pc = core->pc_pc;
		core.step();
		while (core->stage_active != (1 << 5)) {
			core.step();
		}
		printf("\n");
	}

	// Dump the non-zero registers and memory
	printf("Registers:\n");
	for (int i = 1; i < 16; i++) {
		uint32_t value = core->rf_module->registers[i-1];
		if (value) {
			printf("  %s (x%d) = 0x%x\n", REGISTER_NAMES[i], i, value);
		}
	}
	printf("Memory:\n");
	for (int i = 0xC0; i < 0x100; i += 4) {
		uint32_t value;
		memcpy(&value, &m_ram[i-0xC0], sizeof(value));
		if (value) {
			printf("  [0x%x] = 0x%x\n", i, value);
		}
	}

	return 0;
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

int mem_read(int addr, svBit is_unsigned, svBit op_1, svBit op_0) {
	uint32_t result = mem_read_byte(addr);
	if (op_0 || op_1) {
		result |= (mem_read_byte(addr + 1) << 8);
	}
	if (op_1) {
		result |= (mem_read_byte(addr + 2) << 16);
		result |= (mem_read_byte(addr + 3) << 24);
	}
	if (op_1 == 0 && op_0 == 0) {
		if (!is_unsigned && (result & 0x80)) {
			result |= 0xFFFFFF00;
		}
	} else if (op_1 == 0 && op_0 == 1) {
		if (!is_unsigned && (result & 0x8000)) {
			result |= 0xFFFF0000;
		}
	}
	return (int)result;
}

void mem_write_byte(int addr, char data) {
	if (addr >= 0xC0 && addr < 0x100) {
		m_ram[addr-0xC0] = data;
	} else {
		printf("!!! Invalid write addr\n");
		exit(1);
	}
}

void mem_write(int addr, int data, svBit op_1, svBit op_0) {
	mem_write_byte(addr, data & 0xff);
	if (op_0 || op_1) {
		mem_write_byte(addr + 1, (data >> 8) & 0xff);
	}
	if (op_1) {
		mem_write_byte(addr + 2, (data >> 16) & 0xff);
		mem_write_byte(addr + 3, (data >> 24) & 0xff);
	}
}
