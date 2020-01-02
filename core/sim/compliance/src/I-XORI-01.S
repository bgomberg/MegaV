# RISC-V Compliance Test I-XORI-01
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
# Description: Testing instruction XORI.

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
  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - general test of value 0 with 0, 1, -1, MIN, MAX immediate values\n");

  # Addresses for test data and results
  la x1, test_A1_data
  la x2, test_A1_res

  # Load testdata
  lw x3, 0(x1)

  # Test
  xori x4, x3, 1
  xori x5, x3, 0x7FF
  xori x6, x3, 0xFFFFFFFF
  xori x7, x3, 0
  xori x8, x3, 0xFFFFF800

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
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x000007FF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0xFFFFF800)

  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - general test of value 1 with 0, 1, -1, MIN, MAX immediate values\n");

  # Addresses for test data and results
  la x1, test_A2_data
  la x2, test_A2_res

  # Load testdata
  lw x3, 0(x1)

  # Test
  xori x4, x3, 1
  xori x5, x3, 0x7FF
  xori x6, x3, 0xFFFFFFFF
  xori x7, x3, 0
  xori x8, x3, 0xFFFFF800

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)
  sw x8, 20(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x000007FE)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0xFFFFFFFE)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x00000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0xFFFFF801)

  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - general test of value -1 with 0, 1, -1, MIN, MAX immediate values\n");

  # Addresses for test data and results
  la x1, test_A3_data
  la x2, test_A3_res

  # Load testdata
  lw x3, 0(x1)

  # Test
  xori x4, x3, 1
  xori x5, x3, 0x7FF
  xori x6, x3, 0xFFFFFFFF
  xori x7, x3, 0
  xori x8, x3, 0xFFFFF800

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)
  sw x8, 20(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0xFFFFFFFE)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0xFFFFF800)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0xFFFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x000007FF)

  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - general test of value 0x7FFFFFFF with 0, 1, -1, MIN, MAX immediate values\n");

  # Addresses for test data and results
  la x1, test_A4_data
  la x2, test_A4_res

  # Load testdata
  lw x3, 0(x1)

  # Test
  xori x4, x3, 1
  xori x5, x3, 0x7FF
  xori x6, x3, 0xFFFFFFFF
  xori x7, x3, 0
  xori x8, x3, 0xFFFFF800

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)
  sw x8, 20(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x7FFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x7FFFFFFE)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x7FFFF800)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x7FFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x800007FF)

  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - general test of value 0x80000000 with 0, 1, -1, MIN, MAX immediate values\n");

  # Addresses for test data and results
  la x1, test_A5_data
  la x2, test_A5_res

  # Load testdata
  lw x3, 0(x1)

  # Test
  xori x4, x3, 1
  xori x5, x3, 0x7FF
  xori x6, x3, 0xFFFFFFFF
  xori x7, x3, 0
  xori x8, x3, 0xFFFFF800

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)
  sw x8, 20(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x80000001)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x800007FF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x7FFFFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x7FFFF800)

  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part B - testing forwarding between instructions\n");

  # Addresses for test data and results
  la x1, test_B_data
  la x2, test_B_res

  # Load testdata
  lw x3, 0(x1)

  # Test
  xori x4, x3, 0x7F
  xori x5, x4, 0x3F
  xori x6, x5, 0x1F
  xori x7, x6, 0x0F
  xori x8, x7, 0x07
  xori x9, x8, 0x03

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x5, 8(x2)
  sw x6, 12(x2)
  sw x7, 16(x2)
  sw x8, 20(x2)
  sw x9, 24(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0xABCDFFFF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0xABCDFF80)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0xABCDFFBF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0xABCDFFA0)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0xABCDFFAF)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0xABCDFFA8)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x9, 0xABCDFFAB)

  RVTEST_IO_WRITE_STR(sp, "# Test part B - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part C - testing writing to x0\n");

  # Addresses for test data and results
  la x1, test_C_data
  la x2, test_C_res

  # Load testdata
  lw x3, 0(x1)

  # Test
  xori x0, x3, 1

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

  # Test
  xori x0, x3, 1
  xori x3, x0, 1

  # Store results
  sw x0, 0(x2)
  sw x3, 4(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x0, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x00000001)

  RVTEST_IO_WRITE_STR(sp, "# Test part D - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part E - testing moving (xori with 0)\n");

  # Addresses for test data and results
  la x1, test_E_data
  la x2, test_E_res

  # Load testdata
  lw x3, 0(x1)

  # Test
  xori x4, x3, 0
  xori x5, x4, 0
  xori x6, x5, 0
  xori x7, x6, 0
  xori x8, x7, 0
  xori x9, x8, 0
  xori x10, x9, 0
  xori x11, x10, 0
  xori x12, x11, 0

  # Store results
  sw x3, 0(x2)
  sw x4, 4(x2)
  sw x11, 8(x2)
  sw x12, 12(x2)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x36925814)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x36925814)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x11, 0x36925814)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x12, 0x36925814)

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
  .word 0xABCDFFFF
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
  .fill 7, 4, -1
test_C_res:
  .fill 1, 4, -1
test_D_res:
  .fill 2, 4, -1
test_E_res:
  .fill 4, 4, -1

RV_COMPLIANCE_DATA_END
