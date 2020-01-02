# RISC-V Compliance Test I-SB-01
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
# Description: Testing instruction SB.

#include "compliance_test.h"
#include "compliance_io.h"
#include "test_macros.h"

# Test Virtual Machine (TVM) used by program.
RV_COMPLIANCE_RV32M

# Test code region
RV_COMPLIANCE_CODE_BEGIN

  RVTEST_IO_INIT
  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)
  RVTEST_IO_WRITE_STR(sp, "Test Begin\n")

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - test base address + 0\n");

  # Address for test results
  la x2, test_A1_res

  # Set memory
  li x4, 0xAAAABBBB
  sw x4, 0(x2)

  # Test
  li x5, 0x11F1F222
  sb x5, 0(x2)

  //
  // Assert
  //
  RVTEST_IO_CHECK()
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x11F1F222)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0xAAAABBBB)

  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - test base address - 1\n");

  # Address for test results
  la x2, test_A2_res + 1

  # Clear memory
  sw x0, -1(x2)

  # Test
  li x4, 0xF33344F4
  sb x4, 0xFFFFFFFF(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0xF33344F4)

  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - test base address + 1\n");

  # Address for test results
  la x2, test_A3_res - 1

  # Clear memory
  sw x0, 1(x2)

  # Test
  li x4, 0x55F5F666
  sb x4, +1(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x55F5F666)

  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - test base address - 2048\n");

  # Address for test results
  la x2, test_A4_res + 2048

  # Clear memory
  sw x0, -2048(x2)

  # Test
  li x4, 0xF77788F8
  sb x4, 0xFFFFF800(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0xF77788F8)

  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - test base address + 2047\n");

  # Address for test results
  la x2, test_A5_res - 2047

  # Clear memory
  sw x0, 2047(x2)

  # Test
  li x4, 0x99090AAA
  sb x4, 0x7FF(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x99090AAA)

  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part B - test base address + -4, -3, ... , 7\n");

  # Address for test results
  la x2, test_B_res

  # Test
  li x4, 0x1111110C
  li x5, 0x222222CC
  li x6, 0x333333BB
  li x7, 0x4444440B
  li x8, 0x555555EE
  li x9, 0x6666660E
  li x10, 0x7777770D
  li x11, 0x888888DD
  li x12, 0x999999F0
  li x13, 0xAAAAAA00
  li x14, 0xBBBBBBFF
  li x15, 0xCCCCCC0F

  # Store results
  sb x4, -4(x2)
  sb x5, -3(x2)
  sb x6, -2(x2)
  sb x7, -1(x2)
  sb x8, 0(x2)
  sb x9, 1(x2)
  sb x10, 2(x2)
  sb x11, 3(x2)
  sb x12, 4(x2)
  sb x13, 5(x2)
  sb x14, 6(x2)
  sb x15, 7(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x1111110C)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x222222CC)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x333333BB)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x4444440B)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x555555EE)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x9, 0x6666660E)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x10, 0x7777770D)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x11, 0x888888DD)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x12, 0x999999F0)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x13, 0xAAAAAA00)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x14, 0xBBBBBBFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x15, 0xCCCCCC0F)

  RVTEST_IO_WRITE_STR(sp, "# Test part B - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part C - test store x0\n");

  # Address for test results
  la x2, test_C_res

  # Set memory
  li x4, 0x87654321
  sw x4, 0(x2)

  # Test
  li x0, 0x12345678
  sb x0, 0(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x87654321)

  RVTEST_IO_WRITE_STR(sp, "# Test part C - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part D1 - test for forwarding (to address register)\n");

  # Address for test data
  la x1, test_D1_data

  # Clear memory
  lw x4, 0(x1)
  sw x0, 0(x4)

  # Test
  li x6, 0x11223344
  lw x5, 0(x1)
  sb x6, 0(x5)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x11223344)

  RVTEST_IO_WRITE_STR(sp, "# Test part D - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part D2 - test for forwarding (to data register)\n");

  # Address for test data
  la x1, test_D2_data
  la x2, test_D2_res

  # Clear memory
  sw x0, 0(x2)

  # Test
  lw x4, 0(x1)
  sb x4, 0(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x9ABCDEF0)

  RVTEST_IO_WRITE_STR(sp, "# Test part E - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part E1 - test war hazard (data register)\n");

  # Address for test results
  la x2, test_E1_res

  # Clear memory
  sw x0, 0(x2)

  # Test
  li x4, 0x76543210
  sb x4, 0(x2)
  mv x4, x0

  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part E2 - test war hazard (address register)\n");

  # Address for test results
  la x2, test_E2_res

  # Clear memory
  sw x0, 0(x2)

  # Test
  li x4, 0x89ABCDEF
  sb x4, 0(x2)
  addi x2, x2, -4

  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x89ABCDEF)

  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part F - test raw hazard in memory\n");

  # Address for test results
  la x2, test_F_res

  # Clear memory
  sw x0, 0(x2)
  sw x0, 4(x2)

  # Test
  li x4, 0x14725836
  sb x4, 0(x2)
  lw x5, 0(x2)
  sb x5, 4(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x14725836)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000036)

  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part G - test waw hazard in memory\n");

  # Address for test results
  la x1, test_G_res

  # Clear memory
  sw x0, 0(x1)

  # Test
  li x4, 0x96385201
  li x5, 0x25814963
  sb x4, 0(x1)
  sb x5, 0(x1)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x96385201)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x25814963)

  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - Complete\n");

  RVTEST_IO_WRITE_STR(sp, "Test End\n")

# ---------------------------------------------------------------------------------------------
  # HALT
  RV_COMPLIANCE_HALT

RV_COMPLIANCE_CODE_END

# Input data section.
  .section .rodata
  .align 4

test_D1_data:
  .word test_D1_res
test_D2_data:
  .word 0x9ABCDEF0


# Output data section.
RV_COMPLIANCE_DATA_BEGIN
  .align 4

test_A1_res:
  .fill 1, 4, -1
test_A2_res:
  .fill 1, 4, -1
test_A3_res:
  .fill 1, 4, -1
test_A4_res:
  .fill 1, 4, -1
test_A5_res:
  .fill 1, 4, -1
  .fill 1, 4, -1
test_B_res:
  .fill 2, 4, -1
test_C_res:
  .fill 1, 4, -1
test_D1_res:
  .fill 1, 4, -1
test_D2_res:
  .fill 1, 4, -1
test_E1_res:
  .fill 1, 4, -1
test_E2_res:
  .fill 1, 4, -1
test_F_res:
  .fill 2, 4, -1
test_G_res:
  .fill 1, 4, -1

RV_COMPLIANCE_DATA_END
