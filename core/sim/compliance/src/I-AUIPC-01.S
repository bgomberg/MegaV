# RISC-V Compliance Test I-AUIPC-01
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
# Description: Testing instruction AUIPC.

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
  RVTEST_IO_WRITE_STR(sp, "# Test part A - general test of AUIPC\n");

  # Addresses for test data and results
  la x14, test_A_data
  la x15, test_A_res

  # Register initialization
  lw x11, 0(x14)
  li x12, 0x00001004
  li x13, 0xFFFFF008
  li x4, 0x7FFFF00C
  li x5, 0x80000010

  add x12, x11, x12
  add x13, x11, x13
  add x4, x11, x4
  add x5, x11, x5

  # Test
test_A:
  auipc x6, 0x0
  auipc x7, 0x1
  auipc x8, 0xFFFFF
  auipc x9, 0x7FFFF
  auipc x10, 0x80000

  xor x6, x6, x11
  xor x7, x7, x12
  xor x8, x8, x13
  xor x9, x9, x4
  xor x10, x10, x5

  # Store results
  sw x6, 0(x15)
  sw x7, 4(x15)
  sw x8, 8(x15)
  sw x9, 12(x15)
  sw x10, 16(x15)

  //
  // Assert
  //
  RVTEST_IO_CHECK()
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x9, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x10, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part B - test AUIPC overwrites low bits\n");

  # Addresses for test data and results
  la x10, test_B_data
  la x4, test_B_res

  # Register initialization
  li x11, 0x11111111
  li x12, 0x22222222
  li x15, 0x33333333
  li x13, 0x44444444
  li x14, 0x55555555

  lw x9, 0(x10)
  li x10, 0x80000000
  li x5, 0x7FFFF004
  li x6, 0x00000008
  li x7, 0x0000100C
  li x8, 0xFFFFF010

  add x10, x9, x10
  add x5, x9, x5
  add x6, x9, x6
  add x7, x9, x7
  add x8, x9, x8

  # Test
test_B:
  auipc x11, 0x80000
  auipc x12, 0x7FFFF
  auipc x15, 0
  auipc x13, 1
  auipc x14, 0xFFFFF

  xor x11, x11, x10
  xor x12, x12, x5
  xor x15, x15, x6
  xor x13, x13, x7
  xor x14, x14, x8

  # Store results
  sw x11, 0(x4)
  sw x12, 4(x4)
  sw x15, 8(x4)
  sw x13, 12(x4)
  sw x14, 16(x4)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x11, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x12, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x15, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x13, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x14, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A2 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part C - test loading address using auipc + addi or la alias\n");

  # Addresses for test data and results
  la x10, test_C_data
  la x11, test_C_res

test_C1:
  # Register initialization
  lw x7, 0(x10)
  lw x8, 4(x10)
  lw x9, 8(x10)

  # Test
  la x4,  test_C1
  la x15, test_C2
  la x10, test_C3

  # C1
  auipc x5, 0x0
  addi x5, x5, 0xFFFFFFDC

test_C2:
  # C2
  auipc x12, 0x0
  addi x12, x12, 0x0

  # C3
  auipc x6, 0x0
  addi x6, x6, 0x28

  xor x4, x4, x7
  xor x15, x15, x8
  xor x10, x10, x9
  sub x5, x5, x7
  xor x12, x12, x8
  xor x6, x6, x9

  # Store results
  sw x4, 0(x11)
  sw x15, 4(x11)
test_C3:
  sw x10, 8(x11)
  sw x5, 12(x11)
  sw x12, 16(x11)
  sw x6, 20(x11)

  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x15, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x10, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x12, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x00000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - Complete\n");

  RVTEST_IO_WRITE_STR(sp, "# Test End\n")

# ---------------------------------------------------------------------------------------------
  # HALT
  RV_COMPLIANCE_HALT

RV_COMPLIANCE_CODE_END

# Input data section.
  .section .rodata
  .align 4

test_A_data:
  .word test_A
test_B_data:
  .word test_B
test_C_data:
  .word test_C1
  .word test_C2
  .word test_C3


# Output data section.
RV_COMPLIANCE_DATA_BEGIN
  .align 4

test_A_res:
  .fill 5, 4, -1
test_B_res:
  .fill 5, 4, -1
test_C_res:
  .fill 6, 4, -1

RV_COMPLIANCE_DATA_END
