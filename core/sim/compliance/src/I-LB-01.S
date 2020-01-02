# RISC-V Compliance Test I-LB-01
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
# Description: Testing instruction LB.

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

  # ------------------0x00000000---------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - test base address + 0\n");

  # Addresses for test data and results
  la x1, test_A1_data
  la x2, test_A1_res

  # Test
  lb x3, 0(x1)
  lb x4, 1(x1)
  lb x5, 2(x1)
  lb x6, 3(x1)

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)

  //
  // Assert
  //
  RVTEST_IO_CHECK()
  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x00000022)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0xFFFFFFF2)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0xFFFFFFF1)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000011)

  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - test base address - 1\n");

  # Addresses for test data and results
  la x1, test_A2_data + 1
  la x2, test_A2_res

  # Test
  lb x5, -1(x1)
  lb x6, +0(x1)
  lb x7, +1(x1)
  lb x8, +2(x1)

  # Store results
  sw x5, 0(x2)
  sw x6, 4(x2)
  sw x7, 8(x2)
  sw x8, 12(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0xFFFFFFF4)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000044)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x00000033)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0xFFFFFFF3)

  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - test base address + 1\n");

  # Addresses for test data and results
  la x1, test_A3_data - 1
  la x2, test_A3_res

  # Test
  lb x4, 1(x1)
  lb x5, 2(x1)
  lb x6, 3(x1)
  lb x7, 4(x1)

  # Store results
  sw x4, 0(x2)
  sw x5, 4(x2)
  sw x6, 8(x2)
  sw x7, 12(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000066)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0xFFFFFFF6)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0xFFFFFFF5)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x00000055)

  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - test base address - 2048\n");

  # Addresses for test data and results
  la x1, test_A4_data + 2048
  la x2, test_A4_res

  # Test
  lb x4, 0xFFFFF800(x1)
  lb x5, 0xFFFFF801(x1)
  lb x6, 0xFFFFF802(x1)
  lb x7, 0xFFFFF803(x1)

  # Store results
  sw x4, 0(x2)
  sw x5, 4(x2)
  sw x6, 8(x2)
  sw x7, 12(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0xFFFFFFF8)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0xFFFFFF88)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000077)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0xFFFFFFF7)

  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - test base address + 2047\n");

  # Addresses for test data and results
  la x1, test_A5_data - 2044
  la x2, test_A5_res

  # Test
  lb x3, 0x7FC(x1)
  lb x4, 0x7FD(x1)
  lb x5, 0x7FE(x1)
  lb x6, 0x7FF(x1)

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0xFFFFFFAA)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x0000000A)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000009)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0xFFFFFF99)

  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part B - test base address + -4,-1, ..., 6, 7\n");

  # Addresses for test data and results
  la x1, test_B_data
  la x2, test_B_res

  # Test
  lb x3, -4(x1)
  lb x4, -3(x1)
  lb x5, -2(x1)
  lb x6, -1(x1)
  lb x7, 0(x1)
  lb x8, 1(x1)
  lb x9, 2(x1)
  lb x10, 3(x1)
  lb x11, 4(x1)
  lb x12, 5(x1)
  lb x13, 6(x1)
  lb x14, 7(x1)

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)
  sw x8, 20(x2)
  sw x9, 24(x2)
  sw x10, 28(x2)
  sw x11, 32(x2)
  sw x12, 36(x2)
  sw x13, 40(x2)
  sw x14, 44(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x0000000C)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0xFFFFFFCC)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0xFFFFFFBB)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x0000000B)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0xFFFFFFEE)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x0000000E)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x9, 0x0000000D)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x10, 0xFFFFFFDD)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x11, 0xFFFFFFF0)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x12, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x13, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x14, 0x0000000F)

  RVTEST_IO_WRITE_STR(sp, "# Test part B - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part C - test load to x0\n");

  # Addresses for test data and results
  la x1, test_C_data
  la x2, test_C_res

  # Test
  lb x0, 0(x1)

  # Store results
  sw x0, 0(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part C - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part D - test for forwarding\n");

  # Addresses for test data and results
  la x1, test_D_data
  la x2, test_D_res

  # Test
  lw x3, 0(x1)
  lb x4, 0(x3)
  mv x5, x4

  # Store results
  sw x5, 0(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0xFFFFFFF0)

  RVTEST_IO_WRITE_STR(sp, "# Test part D - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part E1 - test store with same base and destination address\n");

  # Addresses for test data and results
  la x4, test_E1_data
  la x2, test_E1_res

  # Test
  lb x4, 0(x4)

  # Store results
  sw x4, 0(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000010)

  RVTEST_IO_WRITE_STR(sp, "# Test part E - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part E2 - test store with same base and destination address, base with offset\n");

  # Addresses for test data and results
  la x1, test_E2_data + 1
  la x2, test_E2_res

  # Test
  lb x1, -1(x1)

  # Store results
  sw x1, 0(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x1, 0xFFFFFF98)

  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - Complete\n");

  RVTEST_IO_WRITE_STR(sp, "# Test End\n")

 # ---------------------------------------------------------------------------------------------
  # HALT
  RV_COMPLIANCE_HALT

RV_COMPLIANCE_CODE_END

# Input data section.
  .section .rodata
  .align 4

test_A1_data:
  .word 0x11F1F222
test_A2_data:
  .word 0xF33344F4
test_A3_data:
  .word 0x55F5F666
test_A4_data:
  .word 0xF77788F8
test_A5_data:
  .word 0x99090AAA
  .word 0x0BBBCC0C
test_B_data:
  .word 0xDD0D0EEE
  .word 0x0FFF00F0
test_C_data:
  .word 0x12345678
test_D_data:
  .word test_D_data2_label
test_D_data2_label:
  .word 0x9ABCDEF0
test_E1_data:
  .word 0x76543210
test_E2_data:
  .word 0xFEDCBA98


# Output data section.
RV_COMPLIANCE_DATA_BEGIN
  .align 4

test_A1_res:
  .fill 4, 4, -1
test_A2_res:
  .fill 4, 4, -1
test_A3_res:
  .fill 4, 4, -1
test_A4_res:
  .fill 4, 4, -1
test_A5_res:
  .fill 4, 4, -1
test_B_res:
  .fill 12, 4, -1
test_C_res:
  .fill 1, 4, -1
test_D_res:
  .fill 1, 4, -1
test_E1_res:
  .fill 1, 4, -1
test_E2_res:
  .fill 1, 4, -1

RV_COMPLIANCE_DATA_END
