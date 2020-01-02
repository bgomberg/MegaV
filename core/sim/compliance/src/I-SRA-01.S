# RISC-V Compliance Test I-SRA-01
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
# Description: Testing instruction SRA.

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
  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - general test of value 0 with 0, 1, -1, MIN, MAX (5bit) register values\n");

  # Addresses for test data and results
  la x1, test_A1_data
  la x2, test_A1_res

  # Load testdata
  lw x3, 0(x1)

  # Register initialization
  li x4, 1
  li x5, 0xF
  li x6, 0x1F
  li x7, 0
  li x8, 0x10

  # Test
  sra x4, x3, x4
  sra x5, x3, x5
  sra x6, x3, x6
  sra x7, x3, x7
  sra x8, x3, x8

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
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - general test of value 1 with 0, 1, -1, MIN, MAX (5bit) register values\n");

  # Addresses for test data and results
  la x1, test_A2_data
  la x2, test_A2_res

  # Load testdata
  lw x3, 0(x1)

  # Register initialization
  li x4, 1
  li x5, 0xF
  li x6, 0x1F
  li x7, 0
  li x8, 0x10

  # Test
  sra x4, x3, x4
  sra x5, x3, x5
  sra x6, x3, x6
  sra x7, x3, x7
  sra x8, x3, x8

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)
  sw x8, 20(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - general test of value -1 with 0, 1, -1, MIN, MAX (5bit) register values\n");

  # Addresses for test data and results
  la x1, test_A3_data
  la x2, test_A3_res

  # Load testdata
  lw x3, 0(x1)

  # Register initialization
  li x4, 1
  li x5, 0xF
  li x6, 0x1F
  li x7, 0
  li x8, 0x10

  # Test
  sra x4, x3, x4
  sra x5, x3, x5
  sra x6, x3, x6
  sra x7, x3, x7
  sra x8, x3, x8

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)
  sw x8, 20(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0xFFFFFFFF)

  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - general test of value 0x7FFFFFFF with 0, 1, -1, MIN, MAX (5bit) register values\n");

  # Addresses for test data and results
  la x1, test_A4_data
  la x2, test_A4_res

  # Load testdata
  lw x3, 0(x1)

  # Register initialization
  li x4, 1
  li x5, 0xF
  li x6, 0x1F
  li x7, 0
  li x8, 0x10

  # Test
  sra x4, x3, x4
  sra x5, x3, x5
  sra x6, x3, x6
  sra x7, x3, x7
  sra x8, x3, x8

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)
  sw x8, 20(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x7FFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x3FFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x0000FFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x7FFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x00007FFF)

  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - general test of value 0x80000000 with 0, 1, -1, MIN, MAX (5bit) register values\n");

  # Addresses for test data and results
  la x1, test_A5_data
  la x2, test_A5_res

  # Load testdata
  lw x3, 0(x1)

  # Register initialization
  li x4, 1
  li x5, 0xF
  li x6, 0x1F
  li x7, 0
  li x8, 0x10

  # Test
  sra x4, x3, x4
  sra x5, x3, x5
  sra x6, x3, x6
  sra x7, x3, x7
  sra x8, x3, x8

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)
  sw x8, 20(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0xC0000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0xFFFF0000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0xFFFF8000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part B - testing forwarding between instructions\n");

  # Addresses for test data and results
  la x1, test_B_data
  la x2, test_B_res

  # Load testdata
  lw x3, 0(x1)

  # Register initialization
  li x10, 0x1

  # Test
  sra x4, x3, x10
  sra x5, x4, x10
  sra x6, x5, x10
  sra x7, x6, x10
  sra x8, x7, x10
  sra x9, x8, x10

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)
  sw x8, 20(x2)
  sw x9, 24(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0xABCDEF10)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0xD5E6F788)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0xEAF37BC4)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0xF579BDE2)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0xFABCDEF1)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0xFD5E6F78)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x9, 0xFEAF37BC)

  RVTEST_IO_WRITE_STR(sp, "# Test part B - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part C - testing writing to x0\n");

  # Addresses for test data and results
  la x1, test_C_data
  la x2, test_C_res

  # Load testdata
  lw x3, 0(x1)

  # Register initialization
  li x4, 1

  # Test
  sra x0, x3, x4

  # Store results
  sw x0, 0(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part C - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part D - testing forwarding throught x0\n");

  # Addresses for test data and results
  la x1, test_D_data
  la x2, test_D_res

  # Load testdata
  lw x3, 0(x1)

  # Register initialization
  li x4, 1

  # Test
  sra x0, x3, x4
  sra x3, x0, x4

  # Store results
  sw x0, 0(x2)
  sw x3, 4(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part D - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part E - testing shift by value greater than 31\n");

  # Addresses for test data and results
  la x1, test_E_data
  la x2, test_E_res

  # Load testdata
  lw x3, 0(x1)

  # Register initialization
  li x4, 0xFFFFFE0
  li x5, 0xFFFFFE1
  li x6, 0xFFFFFEF
  li x7, 0xFFFFFFF

  # Test
  sra x4, x3, x4
  sra x5, x3, x5
  sra x6, x3, x6
  sra x7, x3, x7

  # Store results
  sw x4, 0(x2)
  sw x5, 4(x2)
  sw x6, 8(x2)
  sw x7, 12(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x87654321)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0xC3B2A190)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0xFFFF0ECA)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0xFFFFFFFF)

  RVTEST_IO_WRITE_STR(sp, "# Test part E - Complete\n");

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
  .word 0xABCDEF10
test_C_data:
  .word 0x12345678
test_D_data:
  .word 0xFEDCBA98
test_E_data:
  .word 0x87654321

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
  .fill 7, 4, -1
test_C_res:
  .fill 1, 4, -1
test_D_res:
  .fill 2, 4, -1
test_E_res:
  .fill 4, 4, -1

RV_COMPLIANCE_DATA_END
