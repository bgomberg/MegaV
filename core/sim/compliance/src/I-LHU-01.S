# RISC-V Compliance Test I-LHU-01
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
# Description: Testing instruction LHU.

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
  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - test base address + 0\n");

  # Addresses for test data and results
  la x1, test_A1_data
  la x2, test_A1_res

  # Test
  lhu x3, 0(x1)
  lhu x4, 2(x1)

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)

  //
  // Assert
  //
  RVTEST_IO_CHECK()
  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x0000F222)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x000011F1)

  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - test base address - 1\n");

  # Addresses for test data and results
  la x1, test_A2_data + 1
  la x2, test_A2_res

  # Test
  lhu x3, -1(x1)
  lhu x4, +1(x1)

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x000044F4)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x0000F333)

  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - test base address + 1\n");

  # Addresses for test data and results
  la x1, test_A3_data - 1
  la x2, test_A3_res

  # Test
  lhu x3, 1(x1)
  lhu x4, 3(x1)

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x0000F666)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x000055F5)

  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - test base address - 2048\n");

  # Addresses for test data and results
  la x1, test_A4_data + 2048
  la x2, test_A4_res

  # Test
  lhu x3, 0xFFFFF800(x1)
  lhu x4, 0xFFFFF802(x1)

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x000088F8)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x0000F777)

  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - test base address + 2047\n");

  # Addresses for test data and results
  la x1, test_A5_data - 2045
  la x2, test_A5_res

  # Test
  lhu x3, 0x7FD(x1)
  lhu x4, 0x7FF(x1)

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x00000AAA)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00009909)

  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part B - test base address + -4,-2, 0, 2, 4, 6\n");

  # Addresses for test data and results
  la x1, test_B_data
  la x2, test_B_res

  # Test
  lhu x3, -4(x1)
  lhu x4, -2(x1)
  lhu x5, 0(x1)
  lhu x6, 2(x1)
  lhu x7, 4(x1)
  lhu x8, 6(x1)

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)
  sw x8, 20(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x0000CC0C)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000BBB)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000EEE)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x0000DD0D)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x000000F0)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x00000FFF)

  RVTEST_IO_WRITE_STR(sp, "# Test part B - Complete\n");

  # # ---------------------------------------------------------------------------------------------
  # RVTEST_IO_WRITE_STR(sp, "# Test part C - test load to x0\n");
  #
  # # Addresses for test data and results
  # la x21, test_C_data
  # la x22, test_C_res
  #
  # # Test
  # lhu x0, 0(x21)
  #
  # # Store results
  # sw x0, 0(x22)
  #
  # RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)
  #
  # RVTEST_IO_WRITE_STR(sp, "# Test part C - Complete\n");
  #
  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part D - test for forwarding\n");

  # Addresses for test data and results
  la x1, test_D_data
  la x2, test_D_res

  # Test
  lw x3, 0(x1)
  lhu x4, 0(x3)
  mv x5, x4

  # Store results
  sw x5, 0(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x0000DEF0)

  RVTEST_IO_WRITE_STR(sp, "# Test part D - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part E1 - test store with same base and destination address\n");

  # Addresses for test data and results
  la x4, test_E1_data
  la x2, test_E1_res

  # Test
  lhu x4, 0(x4)

  # Store results
  sw x4, 0(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00003210)

  RVTEST_IO_WRITE_STR(sp, "# Test part E - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part E2 - test store with same base and destination address, base with offset\n");

  # Addresses for test data and results
  la x4, test_E2_data + 1
  la x2, test_E2_res

  # Test
  lhu x4, -1(x4)

  # Store results
  sw x4, 0(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x0000BA98)

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
  .fill 2, 4, -1
test_A2_res:
  .fill 2, 4, -1
test_A3_res:
  .fill 2, 4, -1
test_A4_res:
  .fill 2, 4, -1
test_A5_res:
  .fill 2, 4, -1
test_B_res:
  .fill 6, 4, -1
test_C_res:
  .fill 1, 4, -1
test_D_res:
  .fill 1, 4, -1
test_E1_res:
  .fill 1, 4, -1
test_E2_res:
  .fill 1, 4, -1

RV_COMPLIANCE_DATA_END
