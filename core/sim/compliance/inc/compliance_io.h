// RISC-V Compliance IO Test Header File

/*
 * Copyright (c) 2005-2018 Imperas Software Ltd., www.imperas.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied.
 *
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */


//
// In general the following registers are reserved
// ra, a0, t0, t1
// x1, x10 x5, x6
// new reserve x31
//

#ifndef _COMPLIANCE_IO_H
#define _COMPLIANCE_IO_H

#define STRINGIFY(x) #x
#define TOSTRING(x)  STRINGIFY(x)

#define RVTEST_IO_INIT

#define RVTEST_IO_WRITE_STR(_SP, _STR)                                \
    la _SP, begin_regstate;                                           \
    sw x10, 0(_SP);                                                   \
    sw x11, 4(_SP);                                                   \
  .section .rodata;                                                   \
  20001:                                                              \
  .string _STR;                                                       \
  .section .text;                                                     \
    la x10, 20001b;                                                   \
    li x11, 0x20000000;                                               \
    sw x10, 0(x11);                                                   \
    lw x11, 4(_SP);                                                   \
    lw x10, 0(_SP);

#define RVTEST_IO_WRITE_GFR(_SP, _R)                                  \
    sw x10, 8(_SP);                                                   \
    li x10, 0x20000004;                                               \
    sw _R, 0(x10);                                                    \
    lw x10, 8(_SP);

#define RVTEST_IO_CHECK()

#define RVTEST_IO_ASSERT_GPR_EQ(_SP, _R, _I)                          \
    la _SP, begin_regstate;                                           \
    beq _SP, _R, 20003f;                                              \
    sw x11, 12(_SP);                                                  \
    add x11, _R, x0;                                                  \
    sw x10, 16(_SP);                                                  \
    li x10, _I;                                                       \
    beq x11, x10, 20002f;                                             \
    RVTEST_IO_WRITE_STR(_SP, "Assertion violation: file ");           \
    RVTEST_IO_WRITE_STR(_SP, __FILE__);                               \
    RVTEST_IO_WRITE_STR(_SP, ", line ");                              \
    RVTEST_IO_WRITE_STR(_SP, TOSTRING(__LINE__));                     \
    RVTEST_IO_WRITE_STR(_SP, ": ");                                   \
    RVTEST_IO_WRITE_STR(_SP, # _R);                                   \
    RVTEST_IO_WRITE_STR(_SP, "(");                                    \
    RVTEST_IO_WRITE_GFR(_SP, x11);                                    \
    RVTEST_IO_WRITE_STR(_SP, ") != ");                                \
    RVTEST_IO_WRITE_STR(_SP, # _I);                                   \
    RVTEST_IO_WRITE_STR(_SP, "\n");                                   \
    li gp, 100;                                                       \
    j .;                                                              \
  20003:                                                              \
    RVTEST_IO_WRITE_STR(_SP, "Invalid register used: file ");         \
    RVTEST_IO_WRITE_STR(_SP, __FILE__);                               \
    RVTEST_IO_WRITE_STR(_SP, ", line ");                              \
    RVTEST_IO_WRITE_STR(_SP, TOSTRING(__LINE__));                     \
    RVTEST_IO_WRITE_STR(_SP, ": ");                                   \
    RVTEST_IO_WRITE_STR(_SP, # _R);                                   \
    RVTEST_IO_WRITE_STR(_SP, " (_SP=");                               \
    RVTEST_IO_WRITE_STR(_SP, # _SP);                                  \
    RVTEST_IO_WRITE_STR(_SP, ")\n");                                  \
    li gp, 177;                                                       \
    j .;                                                              \
  20002:                                                              \
    lw x10, 16(_SP);                                                  \
    lw x11, 12(_SP);

#endif // _COMPLIANCE_IO_H
