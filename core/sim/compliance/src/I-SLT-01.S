# RISC-V Compliance Test I-SLT-01
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
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.SE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Specification: RV32I Base Integer Instruction Set, Version 2.0
# Description: Testing instruction SLT.

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
  la x1, test_A1_data
  la x2, test_A1_res

  # Load testdata
  lw x3, 0(x1)

  # Register initialization
  li x4, 1
  li x5, 0x7FFFFFFF
  li x6, 0xFFFFFFFF
  li x7, 0
  li x8, 0x80000000

  # Test
  slt x4, x3, x4
  slt x5, x3, x5
  slt x6, x3, x6
  slt x7, x3, x7
  slt x8, x3, x8

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)
  sw x8, 20(x2)

  //
  // Assert
  //
  RVTEST_IO_CHECK()
  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - general test of value 1 with 0, 1, -1, MIN, MAX register values\n");

  # Addresses for test data and results
  la x1, test_A2_data
  la x2, test_A2_res

  # Load testdata
  lw x3, 0(x1)

  # Register initialization
  li x4, 1
  li x5, 0x7FFFFFFF
  li x6, 0xFFFFFFFF
  li x7, 0
  li x8, 0x80000000

  # Test
  slt x4, x3, x4
  slt x5, x3, x5
  slt x6, x3, x6
  slt x7, x3, x7
  slt x8, x3, x8

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)
  sw x8, 20(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - general test of value -1 with 0, 1, -1, MIN, MAX register values\n");

  # Addresses for test data and results
  la x1, test_A3_data
  la x2, test_A3_res

  # Load testdata
  lw x3, 0(x1)

  # Register initialization
  li x4, 1
  li x5, 0x7FFFFFFF
  li x6, 0xFFFFFFFF
  li x7, 0
  li x8, 0x80000000

  # Test
  slt x4, x3, x4
  slt x5, x3, x5
  slt x6, x3, x6
  slt x7, x3, x7
  slt x8, x3, x8

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)
  sw x8, 20(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - general test of value 0x7FFFFFFF with 0, 1, -1, MIN, MAX register values\n");

  # Addresses for test data and results
  la x1, test_A4_data
  la x2, test_A4_res

  # Load testdata
  lw x3, 0(x1)

  # Register initialization
  li x4, 1
  li x5, 0x7FFFFFFF
  li x6, 0xFFFFFFFF
  li x7, 0
  li x8, 0x80000000

  # Test
  slt x4, x3, x4
  slt x5, x3, x5
  slt x6, x3, x6
  slt x7, x3, x7
  slt x8, x3, x8

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)
  sw x8, 20(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x7FFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - general test of value 0x80000000 with 0, 1, -1, MIN, MAX register values\n");

  # Addresses for test data and results
  la x1, test_A5_data
  la x2, test_A5_res

  # Load testdata
  lw x3, 0(x1)

  # Register initialization
  li x4, 1
  li x5, 0x7FFFFFFF
  li x6, 0xFFFFFFFF
  li x7, 0
  li x8, 0x80000000

  # Test
  slt x4, x3, x4
  slt x5, x3, x5
  slt x6, x3, x6
  slt x7, x3, x7
  slt x8, x3, x8

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)
  sw x8, 20(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part B - testing writing to x0\n");

  # Addresses for test data and results
  la x1, test_B_data
  la x2, test_B_res

  # Load testdata
  lw x3, 0(x1)

  # Register initialization
  li x4, 1

  # Test
  slt x0, x3, x4

  # Store results
  sw x0, 0(x2)
  sw x3, 4(x2)
  sw x4, 8(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000001)

  RVTEST_IO_WRITE_STR(sp, "# Test part B - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part C - testing compare x0\n");

  # Register initialization
  li x1, 1
  li x2, 0x7FF
  li x3, 0xFFFFFFFF
  li x4, 0
  li x5, 0xFFFFF800

  # Test
  slt x6, x0, x1
  slt x7, x0, x2
  slt x8, x0, x3
  slt x9, x0, x4
  slt x10, x0, x5
  slt x11, x1, x0
  slt x12, x2, x0
  slt x13, x3, x0
  slt x14, x4, x0
  slt x15, x5, x0

  # Addresses for test data and results
  la x1, test_C_res

  # Store results
  sw x6, 0(x1)
  sw x7, 4(x1)
  sw x8, 8(x1)
  sw x9, 12(x1)
  sw x10, 16(x1)
  sw x11, 20(x1)
  sw x12, 24(x1)
  sw x13, 28(x1)
  sw x14, 32(x1)
  sw x15, 36(x1)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x9, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x10, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x11, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x12, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x13, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x14, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x15, 0x00000001)

  RVTEST_IO_WRITE_STR(sp, "# Test part C - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part D - testing forwarding between instructions\n");

  # Addresses for test data and results
  la x2, test_D_res

  # Register initialization
  li x3, 0x1

  # Test
  slt x4, x0, x3
  slt x5, x4, x3
  slt x6, x5, x3
  slt x7, x6, x3
  slt x8, x7, x3

  # Store results
  sw x4, 0(x2)
  sw x5, 4(x2)
  sw x6, 8(x2)
  sw x7, 12(x2)
  sw x8, 16(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x00000001)

  RVTEST_IO_WRITE_STR(sp, "# Test part D - Complete\n");

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
  .word 0xFFFFFFFF

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
  .fill 3, 4, -1
test_C_res:
  .fill 10, 4, -1
test_D_res:
  .fill 5, 4, -1


RV_COMPLIANCE_DATA_END
