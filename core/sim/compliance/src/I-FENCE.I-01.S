# RISC-V Compliance Test I-FENCE.I-01
#
# Copyright (c) 2017, Codasip Ltd.
# Copyright (c) 2018, Imperas Software Ltd. Additions
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in the
#        documentation and/or other materials provided with the distribution.
#      * Neither the name of the Codasip Ltd., Imperas Software Ltd. nor the
#        names of its contributors may be used to endorse or promote products
#        derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
# IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Codasip Ltd., Imperas Software Ltd.
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Specification: RV32I Base Integer Instruction Set, Version 2.0
# Description: Testing instruction FENCE.I.

#include "compliance_test.h"
#include "compliance_io.h"
#include "test_macros.h"

# Test Virtual Machine (TVM) used by program.
RV_COMPLIANCE_RV32M

# Test code region
RV_COMPLIANCE_CODE_BEGIN

  RVTEST_IO_INIT
  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)
  RVTEST_IO_WRITE_STR(sp, "# Test Begin\n")

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A - test fence\n");

  # Addresses for test data and results
  la x1, test_A_data
  la x2, test_A_res

  # Register initialization
  li x6, 0

  # Load testdata
  lw x4, 0(x1)
  lw x5, 4(x1)

  # Test
  la x10, instr_A_src
  la x11, instr_A_dst
  lw x7, 0(x10)
  sw x7, 0(x11)
  fence.i
instr_A_dst:
  lui x5, 0

  # Store results
  sw x4, 0(x2)
  sw x5, 4(x2)
  sw x6, 8(x2)
  sw x7, 12(x2)

  //
  // Assert
  //
  RVTEST_IO_CHECK()
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000012)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000042)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x00428333)

  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - Complete\n");

  RVTEST_IO_WRITE_STR(sp, "# Test End\n")

 # ---------------------------------------------------------------------------------------------
  # HALT
  RV_COMPLIANCE_HALT

RV_COMPLIANCE_CODE_END

# Input data section.
  .section .rodata
  .align 4

instr_A_src:
  add x6, x5, x4

test_A_data:
  .word 0x30
  .word 0x12

# Output data section.
RV_COMPLIANCE_DATA_BEGIN
  .align 4

test_A_res:
  .fill 4, 4, -1

RV_COMPLIANCE_DATA_END
