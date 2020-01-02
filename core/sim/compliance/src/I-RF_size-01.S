# RISC-V Compliance Test I-RF_size-01
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
# Description: Testing size of register file.

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
  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - test registers x0 - x15\n");

  # Address for test results
  la x15, test_A1_res

  # Test
  li x0, 0x42726e6f
  li x1, 0x56333249
  li x2, 0x2d562052
  li x3, 0x52697363
  li x4, 0x736b7920
  li x5, 0x73746572
  li x6, 0x6e204e6f
  li x7, 0x4d696c61
  li x8, 0x6f646173
  li x9, 0x6b794063
  li x10, 0x74657273
  li x11, 0x286e6f73
  li x12, 0x656b2048
  li x13, 0x20526164
  li x14, 0x6f6d292c

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
  RVTEST_IO_ASSERT_GPR_EQ(x15, x1, 0x56333249)
  RVTEST_IO_ASSERT_GPR_EQ(x15, x2, 0x2D562052)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x3, 0x52697363)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x4, 0x736B7920)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x5, 0x73746572)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x6, 0x6E204E6F)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x7, 0x4D696C61)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x8, 0x6F646173)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x9, 0x6B794063)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x10, 0x74657273)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x11, 0x286E6F73)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x12, 0x656B2048)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x13, 0x20526164)
  RVTEST_IO_ASSERT_GPR_EQ(sp, x14, 0x6F6D292C)

  RVTEST_IO_WRITE_STR(sp, "# Test part A1 - Complete\n");

  # ---------------------------------------------------------------------------------------------
  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - test registers x0 - x3 again (were not overwritten)\n");

  # Address for test results
  la x4, test_A3_res

  # Test
  sw x0, 0(x4)
  sw x5, 4(x4)
  sw x6, 8(x4)
  sw x7, 12(x4)

  RVTEST_IO_WRITE_STR(sp, "# Test part A3 - Complete\n");

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
  .fill 15, 4, -1
test_A3_res:
  .fill 4, 4, -1

RV_COMPLIANCE_DATA_END
