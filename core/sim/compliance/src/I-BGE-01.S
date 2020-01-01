# RISC-V Compliance Test I-BGE-01
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
# Description: Testing instruction BGE.

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
  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - general test of value 0 with 0, 1, -1, MIN, MAX register values\n");

  # Addresses for test data and results
  la x11, test_A1_data
  la x12, test_A1_res

  # Load testdata
  lw x13, 0(x11)

  # Register initialization
  li x4, 0
  li x5, 1
  li x6, -1
  li x7, 0x7FFFFFFF
  li x8, 0x80000000
  li x15, 0

  # Test
  bge x13, x4, 1f
  ori x15, x15, 0x1
1:
  bge x13, x5, 1f
  ori x15, x15, 0x2
1:
  bge x13, x6, 1f
  ori x15, x15, 0x4
1:
  bge x13, x7, 1f
  ori x15, x15, 0x8
1:
  bge x13, x8, 1f
  ori x15, x15, 0x10
1:

  # Store results
  sw x13, 0(x12)
  sw x4, 4(x12)
  sw x5, 8(x12)
  sw x6, 12(x12)
  sw x7, 16(x12)
  sw x8, 20(x12)
  sw x15, 24(x12)

  //
  // Assert
  //
  RVTEST_IO_CHECK()
  RVTEST_IO_ASSERT_GPR_EQ(sp, x13, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x7FFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x15, 0x0000000A)

  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - general test of value 1 with 0, 1, -1, MIN, MAX register values\n");

  # Addresses for test data and results
  la x4, test_A2_data
  la x5, test_A2_res

  # Load testdata
  lw x6, 0(x4)

  # Register initialization
  li x7, 0
  li x8, 1
  li x9, -1
  li x10, 0x7FFFFFFF
  li x11, 0x80000000
  li x15, 0

  # Test
  bge x6, x7, 1f
  ori x15, x15, 0x1
1:
  bge x6, x8, 1f
  ori x15, x15, 0x2
1:
  bge x6, x9, 1f
  ori x15, x15, 0x4
1:
  bge x6, x10, 1f
  ori x15, x15, 0x8
1:
  bge x6, x11, 1f
  ori x15, x15, 0x10
1:

  # Store results
  sw x6, 0(x5)
  sw x7, 4(x5)
  sw x8, 8(x5)
  sw x9, 12(x5)
  sw x10, 16(x5)
  sw x11, 20(x5)
  sw x15, 24(x5)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x9, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x10, 0x7FFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x11, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x15, 0x00000008)

  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - general test of value -1 with 0, 1, -1, MIN, MAX register values\n");

  # Addresses for test data and results
  la x9, test_A3_data
  la x8, test_A3_res

  # Load testdata
  lw x12, 0(x9)

  # Register initialization
  li x13, 0
  li x14, 1
  li x15, -1
  li x6, 0x7FFFFFFF
  li x7, 0x80000000
  li x10, 0

  # Test
  bge x12, x13, 1f
  ori x10, x10, 0x1
1:
  bge x12, x14, 1f
  ori x10, x10, 0x2
1:
  bge x12, x15, 1f
  ori x10, x10, 0x4
1:
  bge x12, x6, 1f
  ori x10, x10, 0x8
1:
  bge x12, x7, 1f
  ori x10, x10, 0x10
1:

  # Store results
  sw x12, 0(x8)
  sw x13, 4(x8)
  sw x14, 8(x8)
  sw x15, 12(x8)
  sw x6, 16(x8)
  sw x7, 20(x8)
  sw x10, 24(x8)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x12, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x13, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x14, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x15, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x7FFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x10, 0x0000000B)

  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - general test of value 0x7FFFFFFF with 0, 1, -1, MIN, MAX register values\n");

  # Addresses for test data and results
  la x4, test_A4_data
  la x14, test_A4_res

  # Load testdata
  lw x8, 0(x4)

  # Register initialization
  li x9, 0
  li x10, 1
  li x11, -1
  li x12, 0x7FFFFFFF
  li x13, 0x80000000
  li x15, 0

  # Test
  bge x8, x9, 1f
  ori x15, x15, 0x1
1:
  bge x8, x10, 1f
  ori x15, x15, 0x2
1:
  bge x8, x11, 1f
  ori x15, x15, 0x4
1:
  bge x8, x12, 1f
  ori x15, x15, 0x8
1:
  bge x8, x13, 1f
  ori x15, x15, 0x10
1:

  # Store results
  sw x8, 0(x14)
  sw x9, 4(x14)
  sw x10, 8(x14)
  sw x11, 12(x14)
  sw x12, 16(x14)
  sw x13, 20(x14)
  sw x15, 24(x14)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x7FFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x9, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x10, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x11, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x12, 0x7FFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x13, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x15, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - general test of value 0x80000000 with 0, 1, -1, MIN, MAX register values\n");

  # Addresses for test data and results
  la x11, test_A5_data
  la x12, test_A5_res

  # Load testdata
  lw x4, 0(x11)

  # Register initialization
  li x5, 0
  li x6, 1
  li x7, -1
  li x8, 0x7FFFFFFF
  li x9, 0x80000000
  li x15, 0

  # Test
  bge x4, x5, 1f
  ori x15, x15, 0x1
1:
  bge x4, x6, 1f
  ori x15, x15, 0x2
1:
  bge x4, x7, 1f
  ori x15, x15, 0x4
1:
  bge x4, x8, 1f
  ori x15, x15, 0x8
1:
  bge x4, x9, 1f
  ori x15, x15, 0x10
1:

  # Store results
  sw x4, 0(x12)
  sw x5, 4(x12)
  sw x6, 8(x12)
  sw x7, 12(x12)
  sw x8, 16(x12)
  sw x9, 20(x12)
  sw x15, 24(x12)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x7FFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x9, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x15, 0x0000000F)

  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part B - testing compare with x0\n");

  # Address for test results
  la x5, test_B_res

  # Register initialization
  li x11, 0
  li x12, 1
  li x13, -1
  li x14, 0x7FFFFFFF
  li x15, 0x80000000
  li x10, 0

  # Test
  bge x11, x0, 1f
  ori x10, x10, 0x1
1:
  bge x12, x0, 1f
  ori x10, x10, 0x2
1:
  bge x13, x0, 1f
  ori x10, x10, 0x4
1:
  bge x14, x0, 1f
  ori x10, x10, 0x8
1:
  bge x15, x0, 1f
  ori x10, x10, 0x10
1:
  bge x0, x11, 1f
  ori x10, x10, 0x20
1:
  bge x0, x12, 1f
  ori x10, x10, 0x40
1:
  bge x0, x13, 1f
  ori x10, x10, 0x80
1:
  bge x0, x14, 1f
  ori x10, x10, 0x100
1:
  bge x0, x15, 1f
  ori x10, x10, 0x200
1:

  # Store results
  sw x10, 0(x5)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x10, 0x000000154)

  RVTEST_IO_WRITE_STR(sp, "# Test part B - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part C - jumps forward, backward\n");

  # Address for test data and results
  la x4, test_C_data
  la x5, test_C_res

  # Load testdata
  lw x10, 0(x4)

  # Register initialization
  li x12, 0xFFFFFFFF
  li x13, 0xFFFFFFFF
  li x14, 0x0FEDCBA9

  # Test
  bge x10, x0, 2f
  li x12, 0
  li x13, 0
  li x14, 0

1:
  li x13, 0x87654321
  bge x10, x0, 3f
  li x12, 0
  li x13, 0
  li x14, 0

2:
  li x12, 0x9ABCDEF0
  bge x10, x0, 1b
  li x12, 0
  li x13, 0
  li x14, 0

3:

  # Store results
  sw x0, 0(x5)
  sw x12, 4(x5)
  sw x13, 8(x5)
  sw x14, 12(x5)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x12, 0x9ABCDEF0)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x13, 0x87654321)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x14, 0x0FEDCBA9)

  RVTEST_IO_WRITE_STR(sp, "# Test part C - Complete\n");

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
test_C_data:
  .word 1

# Output data section.
RV_COMPLIANCE_DATA_BEGIN
  .align 4

test_A1_res:
  .fill 7, 4, -1
test_A2_res:
  .fill 7, 4, -1
test_A3_res:
  .fill 7, 4, -1
test_A4_res:
  .fill 7, 4, -1
test_A5_res:
  .fill 7, 4, -1
test_B_res:
  .fill 1, 4, -1
test_C_res:
  .fill 4, 4, -1

RV_COMPLIANCE_DATA_END
