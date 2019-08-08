// RISC-V Compliance Test Header File
// Copyright (c) 2017, Codasip Ltd. All Rights Reserved.
// See LICENSE for license details.
//
// Description: Common header file for RV32I tests

#ifndef _COMPLIANCE_TEST_H
#define _COMPLIANCE_TEST_H

// #include "riscv_test.h"

//-----------------------------------------------------------------------
// RV Compliance Macros
//-----------------------------------------------------------------------

#define RV_COMPLIANCE_HALT                                            \
    la sp, test_result_data;                                          \
    li gp, 1;                                                         \
    j .;

#define RV_COMPLIANCE_RV32M

#define RV_COMPLIANCE_CODE_BEGIN                                      \
  .section .prog_entry, "ax";                                         \
  .global prog_entry;                                                 \
  prog_entry:                                                         \
    j main;                                                           \
  .section .irq_handler;                                              \
  trap_vector:                                                        \
    /* test whether the test came from pass/fail */                   \
    csrr x10, mcause;                                                 \
    li x11, 0xb;                                                      \
    beq x10, x11, exception_loop;                                     \
  other_exception:                                                    \
    /* some unhandlable exception occurred */                         \
    ori gp, gp, 1337;                                                 \
  exception_loop:                                                     \
    j .;                                                              \
  1:                                                                  \
  .section .text;                                                     \
  main:

#define RV_COMPLIANCE_CODE_END                                        \
  end_testcode:                                                       \
    unimp

#define RV_COMPLIANCE_DATA_BEGIN                                      \
  .section .data;                                                     \
  test_result_data:

#define RV_COMPLIANCE_DATA_END                                        \
  .align 4; .global end_signature; end_signature:                     \
  .align 8; .global begin_regstate; begin_regstate:                   \
  .word 128;                                                          \
  .align 8; .global end_regstate; end_regstate:                       \
  .word 4;

#endif
