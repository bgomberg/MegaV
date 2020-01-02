# RISC-V Compliance Test I-NOP-01
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
# Description: Testing pseudo instruction NOP.

#include "compliance_test.h"
#include "compliance_io.h"
#include "test_macros.h"

# Test Virtual Machine (TVM) used by program.
RV_COMPLIANCE_RV32M

# Test code region.
RV_COMPLIANCE_CODE_BEGIN

  RVTEST_IO_INIT
  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)
  RVTEST_IO_WRITE_STR(sp, "# Test Begin\n")

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - general test, instruction should not change any register\n");

  # Addresses for test data and results
  la x1, test_A1_res

  # Register initialization
  li x2, 2
  li x3, 3
  li x4, 4
  li x5, 5
  li x6, 6
  li x7, 7
  li x8, 8
  li x9, 9
  li x10, 10
  li x11, 11
  li x12, 12
  li x13, 13
  li x14, 14
  li x15, 15

  #Test
  nop
  nop
  nop
  nop
  nop
  nop

  # Store results
  sw x0, 0(x1)
  sw x2, 4(x1)
  sw x3, 8(x1)
  sw x4, 12(x1)
  sw x5, 16(x1)
  sw x6, 20(x1)
  sw x7, 24(x1)
  sw x8, 28(x1)
  sw x9, 32(x1)
  sw x10, 36(x1)
  sw x11, 40(x1)
  sw x12, 44(x1)
  sw x13, 48(x1)
  sw x14, 52(x1)
  sw x15, 56(x1)

  //
  // Assert
  //
  RVTEST_IO_CHECK()
  RVTEST_IO_ASSERT_GPR_EQ(x1, x0, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(x1, x2, 0x00000002)
  RVTEST_IO_ASSERT_GPR_EQ(x1, x3, 0x00000003)
  RVTEST_IO_ASSERT_GPR_EQ(x1, x4, 0x00000004)
  RVTEST_IO_ASSERT_GPR_EQ(x1, x5, 0x00000005)
  RVTEST_IO_ASSERT_GPR_EQ(x1, x6, 0x00000006)
  RVTEST_IO_ASSERT_GPR_EQ(x1, x7, 0x00000007)
  RVTEST_IO_ASSERT_GPR_EQ(x1, x8, 0x00000008)
  RVTEST_IO_ASSERT_GPR_EQ(x1, x9, 0x00000009)
  RVTEST_IO_ASSERT_GPR_EQ(x1, x10, 0x0000000A)
  RVTEST_IO_ASSERT_GPR_EQ(x1, x11, 0x0000000B)
  RVTEST_IO_ASSERT_GPR_EQ(x1, x12, 0x0000000C)
  RVTEST_IO_ASSERT_GPR_EQ(x1, x13, 0x0000000D)
  RVTEST_IO_ASSERT_GPR_EQ(x1, x14, 0x0000000E)
  RVTEST_IO_ASSERT_GPR_EQ(x1, x15, 0x0000000F)

  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - general test, only 5 cycles are executed\n");

  # Addresses for test data and results
  la x2, test_A2_res

  # Register initialization
  auipc x3, 0

  #Test
  nop
  nop
  nop
  nop
  nop

  # Store results
  auipc x4, 0
  sub x4, x4, x3

  # Store results
  sw x4, 0(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000018)

  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - Complete\n");

  RVTEST_IO_WRITE_STR(sp, "# Test End\n")

 # ---------------------------------------------------------------------------------------------
  # HALT
  RV_COMPLIANCE_HALT

RV_COMPLIANCE_CODE_END

# Input data section.
  .section .rodata
  .align 4

# Output data section.
RV_COMPLIANCE_DATA_BEGIN
  .align 4

test_A1_res:
  .fill 31, 4, -1
test_A2_res:
  .fill 1, 4, -1

RV_COMPLIANCE_DATA_END     # End of test output data region.
