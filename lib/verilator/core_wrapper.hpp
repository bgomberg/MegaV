#include "Vmodule.h"

class CoreWrapper {
public:
	CoreWrapper() : core_(new Vmodule()) {
		core_->clk = 0;
		core_->eval();
	}

	~CoreWrapper() {
		delete core_;
		core_ = NULL;
	}

	Vmodule* operator->() {
		return core_;
	}

	void tick() {
		core_->clk = 1;
		core_->eval();
		core_->clk = 0;
		core_->eval();
	}

	bool is_done(void) {
		return Verilated::gotFinish();
	}

private:
	Vmodule* core_;
};
