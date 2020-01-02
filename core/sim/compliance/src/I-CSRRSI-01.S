# RISC-V Compliance Test I-CSRRSI-01
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
# Description: Testing instruction CSRRSI.

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
  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - general test of CSRRSI\n");

  # Address for test results
  la x2, test_A1_res

  # Register initialization
  csrrw x0, mepc, x0

  # Test
  csrrsi x3, mepc, 1
  csrrw x3, mepc, x0

  csrrsi x4, mepc, 0
  csrrw x4, mepc, x0

  csrrsi x5, mepc, 0x1F
  csrrw x5, mepc, x0

  csrrsi x6, mepc, 0x10
  csrrw x6, mepc, x0

  csrrsi x7, mepc, 0xF
  csrrw x7, mepc, x0

  # Store results
  sw x0, 0(x2)
  sw x3, 4(x2)
  sw x4, 8(x2)
  sw x5, 12(x2)
  sw x6, 16(x2)
  sw x7, 20(x2)

  //
  // Assert
  //
  RVTEST_IO_CHECK()
  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x0000001C)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000010)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x0000000C)

  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - general test of CSRRSI\n");

  # Address for test results
  la x2, test_A2_res

  # Register initialization
  csrrw x0, mepc, x0

  # Test
  csrrsi x3, mepc, 1
  csrrsi x4, mepc, 0
  csrrsi x5, mepc, 0x1F
  csrrsi x6, mepc, 0x10
  csrrsi x7, mepc, 0xF
  csrrsi x8, mepc, 0

  # Store results
  sw x0, 0(x2)
  sw x3, 4(x2)
  sw x4, 8(x2)
  sw x5, 12(x2)
  sw x6, 16(x2)
  sw x7, 20(x2)
  sw x8, 24(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x0000001C)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x0000001C)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x0000001C)

  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part B - testing writing to x0\n");

  # Address for test results
  la x2, test_B_res

  # Register initialization
  li x3, 0x32165498
  csrrw x0, mepc, x3

  # Test
  csrrsi x0, mepc, 0xF
  csrrw x4, mepc, x3

  # store results
  sw x0, 0(x2)
  sw x4, 4(x2)
  sw x3, 8(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x3216549C)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x32165498)

  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - Complete\n");

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
  .fill 6, 4, -1
test_A2_res:
  .fill 7, 4, -1
test_B_res:
  .fill 3, 4, -1

RV_COMPLIANCE_DATA_END     # End of test output data region.
