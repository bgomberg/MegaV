# RISC-V Compliance Test I-LUI-01
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
# Description: Testing instruction LUI.

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
  RVTEST_IO_WRITE_STR(sp, "# Test part A - general test of LUI\n");

  # Address for test results
  la x2, test_A_res

  # Test
  lui x3, 0x0
  lui x4, 0x1
  lui x5, 0xFFFFF
  lui x6, 0x7FFFF
  lui x7, 0x80000

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)

  //
  // Assert
  //
  RVTEST_IO_CHECK()
  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00001000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0xFFFFF000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x7FFFF000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x80000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part B - test LUI overwrites low bits\n");

  # Addresses for test data and results
  la x1, test_B_data
  la x2, test_B_res

  # Load testdata
  lw x3, 0(x1)
  lw x4, 0(x1)
  lw x5, 0(x1)
  lw x6, 0(x1)
  lw x7, 0(x1)

  # Test
  lui x3, 0x80000
  lui x4, 0x7FFFF
  lui x5, 0
  lui x6, 1
  lui x7, 0xFFFFF

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x7FFFF000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00001000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0xFFFFF000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part C - test loading immediate using lui + addi or li alias\n");

  # Address for results
  la x2, test_C_res

  # Test
  li x3, 0x42726E6F
  li x4, 0x12345678
  li x5, 0x9ABCDEF0

  lui x6, 0x42727
  addi x6, x6, 0xFFFFFE6F
  lui x7, 0x12345
  addi x7, x7, 0x678
  lui x8, 0x9ABCE
  addi x8, x8, 0xFFFFFEF0

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)
  sw x8, 20(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x42726E6F)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x12345678)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x9ABCDEF0)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x42726E6F)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x12345678)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x9ABCDEF0)

  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - Complete\n");

  RVTEST_IO_WRITE_STR(sp, "# Test End\n")

 # ---------------------------------------------------------------------------------------------
  # HALT
  RV_COMPLIANCE_HALT

RV_COMPLIANCE_CODE_END

# Input data section.
  .section .rodata
  .align 4

test_B_data:
  .word 0x11111111


# Output data section.
RV_COMPLIANCE_DATA_BEGIN
  .align 4

test_A_res:
  .fill 5, 4, -1
test_B_res:
  .fill 5, 4, -1
test_C_res:
  .fill 6, 4, -1

RV_COMPLIANCE_DATA_END
