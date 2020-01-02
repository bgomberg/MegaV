# RISC-V Compliance Test I-RF_width-01
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
# Description: Testing width of register file.

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
  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - test x1 - x15 are 32 bits\n");

  # Address for test results
  la x15, test_A1_res

  # Init registers
  li x0, 1
  li x1, 1
  li x2, 1
  li x3, 1
  li x4, 1
  li x5, 1
  li x6, 1
  li x7, 1
  li x8, 1
  li x9, 1
  li x10, 1
  li x11, 1
  li x12, 1
  li x13, 1
  li x14, 1


  # Test
  slli x1, x1, 31
  bltz x1, 1f
  li x1, 0

1:
  slli x2, x2, 31
  bltz x2, 1f
  li x2, 0

1:
  slli x3, x3, 31
  bltz x3, 1f
  li x3, 0

1:
  slli x4, x4, 31
  bltz x4, 1f
  li x4, 0

1:
  slli x5, x5, 31
  bltz x5, 1f
  li x5, 0

1:
  slli x6, x6, 31
  bltz x6, 1f
  li x6, 0

1:
  slli x7, x7, 31
  bltz x7, 1f
  li x7, 0

1:
  slli x8, x8, 31
  bltz x8, 1f
  li x8, 0

1:
  slli x9, x9, 31
  bltz x9, 1f
  li x9, 0

1:
  slli x10, x10, 31
  bltz x10, 1f
  li x10, 0

1:
  slli x11, x11, 31
  bltz x11, 1f
  li x11, 0

1:
  slli x12, x12, 31
  bltz x12, 1f
  li x12, 0

1:
  slli x13, x13, 31
  bltz x13, 1f
  li x13, 0

1:
  slli x14, x14, 31
  bltz x14, 1f
  li x14, 0

1:

  # Store results
  sw x0, 0(x15)
  sw x1, 4(x15)
  sw x2, 8(x15)
  sw x3, 12(x15)
  sw x4, 16(x15)
  sw x5, 20(x15)
  sw x6, 24(x15)
  sw x7, 28(x15)
  sw x8, 32(x15)
  sw x9, 36(x15)
  sw x10, 40(x15)
  sw x11, 44(x15)
  sw x12, 48(x15)
  sw x13, 52(x15)
  sw x14, 56(x15)

  //
  // Assert
  //
  RVTEST_IO_CHECK()
  RVTEST_IO_ASSERT_GPR_EQ(x15, x0, 0x00000000)
  RVTEST_IO_ASSERT_GPR_EQ(x15, x1, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(x15, x2, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x9, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x10, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x11, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x12, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x13, 0x80000000)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x14, 0x80000000)

  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - Complete\n");

  RVTEST_IO_WRITE_STR(sp, "# Test End\n")

 # ---------------------------------------------------------------------------------------------
  # HALT
  RV_COMPLIANCE_HALT

RV_COMPLIANCE_CODE_END

# Input data section.
  .section .rodata
  .align 4

# Output data section.
RV_COMPLIANCE_DATA_BEGIN
  .align 4

test_A1_res:
  .fill 16, 4, -1

RV_COMPLIANCE_DATA_END
