# RISC-V Compliance Test I-SW-01
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
# Description: Testing instruction SW.

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

  # Address for test results
  la x4, test_A1_res

  # Test
  li x5, 0x11F1F222
  sw x5, 0(x4)

  //
  // Assert
  //
  RVTEST_IO_CHECK()
  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - test base address - 1\n");

  # Address for test results
  la x4, test_A2_res + 1

  # Test
  li x5, 0xF33344F4
  sw x5, 0xFFFFFFFF(x4)

  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - test base address + 1\n");

  # Address for test results
  la x4, test_A3_res - 1

  # Test
  li x5, 0x55F5F666
  sw x5, +1(x4)

  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - test base address - 2048\n");

  # Address for test results
  la x4, test_A4_res + 2048

  # Test
  li x5, 0xF77788F8
  sw x5, 0xFFFFF800(x4)

  RVTEST_IO_WRITE_STR(sp, "# Test part A4 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - test base address + 2047\n");

  # Address for test results
  la x4, test_A5_res - 2047

  # Test
  li x5, 0x99090AAA
  sw x5, 0x7FF(x4)

  RVTEST_IO_WRITE_STR(sp, "# Test part A5 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part B - test base address + -4, 0, 4\n");

  # Address for test results
  la x4, test_B_res

  # Test
  li x5, 0x0BBBCC0C
  li x6, 0xDD0D0EEE
  li x7, 0x0FFF00F0

  # Store results
  sw x5, -4(x4)
  sw x6, 0(x4)
  sw x7, 4(x4)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x0BBBCC0C)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0xDD0D0EEE)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x0FFF00F0)

  RVTEST_IO_WRITE_STR(sp, "# Test part B - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part C - test store x0\n");

  # Address for test results
  la x4, test_C_res

  # Test
  li x0, 0x12345678
  sw x0, 0(x4)

  RVTEST_IO_WRITE_STR(sp, "# Test part C - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part D1 - test for forwarding (to address register)\n");

  # Address for test data
  la x4, test_D1_data

  # Test
  li x5, 0x11223344
  lw x6, 0(x4)
  sw x5, 0(x6)

  RVTEST_IO_WRITE_STR(sp, "# Test part D1 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part D2 - test for forwarding (to data register)\n");

  # Address for test data
  la x4, test_D2_data
  la x5, test_D2_res

  # Test
  lw x6, 0(x4)
  sw x6, 0(x5)

  RVTEST_IO_WRITE_STR(sp, "# Test part D2 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part E1 - test war hazard (data register)\n");

  # Address for test results
  la x4, test_E1_res

  # Test
  li x5, 0x76543210
  sw x5, 0(x4)
  mv x5, x0

  RVTEST_IO_WRITE_STR(sp, "# Test part E1 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part E2 - test war hazard (address register)\n");

  # Address for test results
  la x4, test_E2_res

  # Test
  li x5, 0x89ABCDEF
  sw x5, 0(x4)
  addi x4, x4, -4

  RVTEST_IO_WRITE_STR(sp, "# Test part E2 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part F - test raw hazard in memory\n");

  # Address for test results
  la x5, test_F_res

  # Test
  li x4, 0x14725836
  sw x4, 0(x5)
  lw x6, 0(x5)
  sw x6, 4(x5)

  RVTEST_IO_WRITE_STR(sp, "# Test part F - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part G - test waw hazard in memory\n");

  # Address for test results
  la x4, test_G_res

  # Test
  li x5, 0x96385201
  li x6, 0x25814963
  sw x5, 0(x4)
  sw x6, 0(x4)

  RVTEST_IO_WRITE_STR(sp, "# Test part G - Complete\n");

  RVTEST_IO_WRITE_STR(sp, "# Test End\n")

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
