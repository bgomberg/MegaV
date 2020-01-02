# RISC-V Compliance Test I-CSRRW-01
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
# Description: Testing instruction CSRRW.

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
  RVTEST_IO_WRITE_STR(sp, "# Test part A - general test of CSRRW\n");

  # Address for test results
  la x2, test_A_res

  # Register initialization
  li x10, 1
  li x11, 0
  li x12, -1
  li x13, 0x7FFFFFFF
  li x14, 0x80000000
  csrrw x0, mepc, x0

  # Test
  csrrw x4, mepc, x10
  csrrw x5, mepc, x11
  csrrw x6, mepc, x12
  csrrw x7, mepc, x13
  csrrw x8, mepc, x14
  csrrw x9, mepc, x0

  # Store results
  sw x4, 0(x2)
  sw x5, 4(x2)
  sw x6, 8(x2)
  sw x7, 12(x2)
  sw x8, 16(x2)
  sw x9, 20(x2)

  //
  // Assert
  //
  RVTEST_IO_CHECK()
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0xFFFFFFFC)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x7FFFFFFC)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x9, 0x80000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part B - testing forwarding between instructions\n");

  # Address for test results
  la x2, test_B_res

  # Register initialization
  li x3, 0x12345678
  li x4, 0x9ABCDEF0

  # Test
  csrrw x0, mepc, x3
  csrrw x5, mepc, x4
  csrrw x6, mepc, x5
  csrrw x7, mepc, x6
  csrrw x8, mepc, x0

  # store results
  sw x5, 0(x2)
  sw x6, 4(x2)
  sw x7, 8(x2)
  sw x8, 12(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x12345678)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x9ABCDEF0)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x12345678)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x9ABCDEF0)

  RVTEST_IO_WRITE_STR(sp, "# Test part B - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part C - testing writing to x0\n");

  # Address for test results
  la x2, test_C_res

  # Register initialization
  li x4, 0x42726E6F

  # Test
  csrrw x0, mepc, x4
  csrrw x0, mepc, x0

  # store results
  sw x0, 0(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part C - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part D - testing forwarding throught x0\n");

  # Address for test results
  la x2, test_D_res

  # Register initialization
  li x3, 0xF7FF8818

  # Test
  csrrw x0, mepc, x3
  csrrw x0, mepc, x0
  csrrw x0, mepc, x0
  csrrw x5, mepc, x0

  # store results
  sw x0, 0(x2)
  sw x5, 4(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part D - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part E - testing csrrw with same dst and src registers\n");

  # Address for test results
  la x2, test_E_res

  # Register initialization
  li x5, 0x32165498
  li x4, 0x14725836
  li x3, 0x96385274

  # Test
  csrrw x0, mepc, x4
  csrrw x3, mepc, x3
  csrrw x5, mepc, x5
  csrrw x6, mepc, x0

  # Store results
  sw x3, 0(x2)
  sw x5, 4(x2)
  sw x6, 8(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x14725834)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x96385274)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x32165498)

  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - Complete\n");

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

test_A_res:
  .fill 6, 4, -1
test_B_res:
  .fill 4, 4, -1
test_C_res:
  .fill 1, 4, -1
test_D_res:
  .fill 2, 4, -1
test_E_res:
  .fill 3, 4, -1

RV_COMPLIANCE_DATA_END     # End of test output data region.
