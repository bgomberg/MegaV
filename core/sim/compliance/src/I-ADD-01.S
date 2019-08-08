# RISC-V Compliance Test I-ADD-01
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
# Description: Testing instruction ADD.

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
  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - general test of value 0 with 0, 1, -1, MIN, MAX register values\n");

  # Addresses for test data and results
  la x4, test_A1_data
  la x5, test_A1_res

  # Load testdata
  lw x6, 0(x4)

  # Register initialization
  li x7, 0
  li x8, 1
  li x9, -1
  li x10, 0x7FFFFFFF
  li x11, 0x80000000

  # Test
  add x7, x6, x7
  add x8, x6, x8
  add x9, x6, x9
  add x10, x6, x10
  add x11, x6, x11

  # Store results
  sw x6, 0(x5)
  sw x7, 4(x5)
  sw x8, 8(x5)
  sw x9, 12(x5)
  sw x10, 16(x5)
  sw x11, 20(x5)

  //
  // Assert
  //
  RVTEST_IO_CHECK()
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x9, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x10, 0x7FFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x11, 0x80000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - general test of value 1 with 0, 1, -1, MIN, MAX register values\n");

  # Addresses for test data and results
  la x14, test_A2_data
  la x15, test_A2_res

  # Load testdata
  lw x8, 0(x14)

  # Register initialization
  li x9, 0
  li x10, 1
  li x11, -1
  li x12, 0x7FFFFFFF
  li x13, 0x80000000

  # Test
  add x9, x8, x9
  add x10, x8, x10
  add x11, x8, x11
  add x12, x8, x12
  add x13, x8, x13

  # Store results
  sw x8, 0(x15)
  sw x9, 4(x15)
  sw x10, 8(x15)
  sw x11, 12(x15)
  sw x12, 16(x15)
  sw x13, 20(x15)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x9, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x10, 0x00000002)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x11, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x12, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x13, 0x80000001)

  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - general test of value -1 with 0, 1, -1, MIN, MAX register values\n");

  # Addresses for test data and results
  la x9, test_A3_data
  la x8, test_A3_res

  # Load testdata
  lw x10, 0(x9)

  # Register initialization
  li x11, 0
  li x12, 1
  li x13, -1
  li x14, 0x7FFFFFFF
  li x15, 0x80000000

  # Test
  add x11, x10, x11
  add x12, x10, x12
  add x13, x10, x13
  add x14, x10, x14
  add x15, x10, x15

  # Store results
  sw x10, 0(x8)
  sw x11, 4(x8)
  sw x12, 8(x8)
  sw x13, 12(x8)
  sw x14, 16(x8)
  sw x15, 20(x8)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x10, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x11, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x12, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x13, 0xFFFFFFFE)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x14, 0x7FFFFFFE)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x15, 0x7FFFFFFF)

  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - general test of value 0x7FFFFFFF with 0, 1, -1, MIN, MAX register values\n");

  # Addresses for test data and results
  la x6, test_A4_data
  la x7, test_A4_res

  # Load testdata
  lw x8, 0(x6)

  # Register initialization
  li x9, 0
  li x10, 1
  li x11, -1
  li x12, 0x7FFFFFFF
  li x13, 0x80000000

  # Test execution
  add x9, x8, x9
  add x10, x8, x10
  add x11, x8, x11
  add x12, x8, x12
  add x13, x8, x13

  # Store results
  sw x8, 0(x7)
  sw x9, 4(x7)
  sw x10, 8(x7)
  sw x11, 12(x7)
  sw x12, 16(x7)
  sw x13, 20(x7)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x7FFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x9, 0x7FFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x10, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x11, 0x7FFFFFFE)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x12, 0xFFFFFFFE)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x13, 0xFFFFFFFF)

  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - general test of value 0x80000000 with 0, 1, -1, MIN, MAX register values\n");

  # Addresses for test data and results
  la x5, test_A5_data
  la x6, test_A5_res

  # Load testdata
  lw x7, 0(x5)

  # Register initialization
  li x8, 0
  li x9, 1
  li x10, -1
  li x11, 0x7FFFFFFF
  li x12, 0x80000000

  # Test
  add x8, x7, x8
  add x9, x7, x9
  add x10, x7, x10
  add x11, x7, x11
  add x12, x7, x12

  # Store results
  sw x7, 0(x6)
  sw x8, 4(x6)
  sw x9, 8(x6)
  sw x10, 12(x6)
  sw x11, 16(x6)
  sw x12, 20(x6)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x9, 0x80000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x10, 0x7FFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x11, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x12, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part B - testing forwarding between instructions\n");

  # Addresses for test data and results
  la x5, test_B_data
  la x6, test_B_res

  # Load testdata
  lw x8, 0(x5)

  # Register initialization
  li x7, 0x1

  # Test
  add x9, x8, x7
  add x10, x9, x7
  add x11, x10, x7
  add x12, x11, x7
  add x13, x12, x7
  add x14, x13, x7

  # store results
  sw x7, 0(x6)
  sw x8, 4(x6)
  sw x9, 8(x6)
  sw x10, 12(x6)
  sw x11, 16(x6)
  sw x12, 20(x6)
  sw x13, 24(x6)
  sw x14, 28(x6)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x0000ABCD)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x9, 0x0000ABCE)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x10, 0x0000ABCF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x11, 0x0000ABD0)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x12, 0x0000ABD1)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x13, 0x0000ABD2)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x14, 0x0000ABD3)

  RVTEST_IO_WRITE_STR(sp, "# Test part B - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part C - testing writing to x0\n");

  # Addresses for test data and results
  la x5, test_C_data
  la x6, test_C_res

  # Load testdata
  lw x8, 0(x5)

  # Register initialization
  li x7, 0xF7FF8818

  # Test
  add x0, x8, x7

  # store results
  sw x0, 0(x6)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part C - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part D - testing forwarding throught x0\n");

  # Addresses for test data and results
  la x5, test_D_data
  la x6, test_D_res

  # Load testdata
  lw x8, 0(x5)

  # Register initialization
  li x7, 0xF7FF8818

  # Test
  add x0, x8, x7
  add x15, x0, x0

  # store results
  sw x0, 0(x6)
  sw x15, 4(x6)


  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x15, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part D  - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part E - testing moving (add with x0)\n");

  # Addresses for test data and results
  la x5, test_E_data
  la x6, test_E_res

  # Load testdata
  lw x7, 0(x5)

  # Test
  add x8, x7, x0
  add x9, x8, x0
  add x10, x0, x9
  add x11, x10, x0
  add x12, x11, x0
  add x13, x12, x0
  add x14, x0, x13
  add x15, x0, x14
  add x4, x15, x0

  # Store results
  sw x8, 0(x6)
  sw x15, 4(x6)
  sw x4, 8(x6)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x8,  0x36925814)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x15, 0x36925814)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x36925814)

  RVTEST_IO_WRITE_STR(sp, "# Test part E  - Complete\n");

  RVTEST_IO_WRITE_STR(sp, "# Test End\n")

  # ---------------------------------------------------------------------------------------------
  # HALT
  RV_COMPLIANCE_HALT

RV_COMPLIANCE_CODE_END

# Input data section.
  .section .rodata
  .align 4

test_A1_data:
  .word 0
test_A2_data:
  .word 1
test_A3_data:
  .word -1
test_A4_data:
  .word 0x7FFFFFFF
test_A5_data:
  .word 0x80000000
test_B_data:
  .word 0x0000ABCD
test_C_data:
  .word 0x12345678
test_D_data:
  .word 0xFEDCBA98
test_E_data:
  .word 0x36925814

# Output data section.
RV_COMPLIANCE_DATA_BEGIN
  .align 4

test_A1_res:
  .fill 6, 4, -1
test_A2_res:
  .fill 6, 4, -1
test_A3_res:
  .fill 6, 4, -1
test_A4_res:
  .fill 6, 4, -1
test_A5_res:
  .fill 6, 4, -1
test_B_res:
  .fill 8, 4, -1
test_C_res:
  .fill 1, 4, -1
test_D_res:
  .fill 2, 4, -1
test_E_res:
  .fill 3, 4, -1

RV_COMPLIANCE_DATA_END
