.section .prog_entry, "ax"
.global prog_entry
prog_entry:
  j     main

.section .irq_handler
irq_handler:
  # Handle the interrupt
  csrr  s0, mcause
  li    s1, 0x0000000b
  beq   s0, s1, irq_handler_ecall
  li    s1, 0x00000003
  beq   s0, s1, irq_handler_ebreak
  li    s1, 0x8000000b
  beq   s0, s1, irq_handler_ext
  li    s1, 0x80000003
  beq   s0, s1, irq_handler_sw
  # Default handler
  addi  a5, a5, 0x700
  mret

.section .text
main:
  # Enable interrupts
  li    t0, 0x808
  csrs  mstatus, t0
  li    t0, 0x808
  csrs  mie, t0
  fence
  fence.i
  j     l1
  li    a5, 0x0bd
  j     .

l1:
  csrr  a3, mstatus
  li    t0, 0x11
  li    t1, 0x31
  add   t1, t0, t1
  add   t2, t0, t1
  la    tp, test_bss_var
  sw    t2, 0(tp)
  lw    a0, 0(tp)
  li    a1, 0x100
  jal   l3

l2:
  li    t0, 0
  bne   a5, t0, fail
  li    t0, 0x101
  ecall
  bne   a5, t0, fail
  li    t0, 0x203
  ebreak
  bne   a5, t0, fail
  li    t0, 0x30b
  csrsi mip, 0x08 # Trigger a SW interrupt
  bne   a5, t0, fail
  ori   a5, a5, 0x10
  addi  a5, a5, 0x100
  j     .

l3:
  addi  a2, a2, 0x01
  addi  a2, a2, 0x01
  beqz  a2, l2
  beqz  zero, l4

l4:
  jal   gp, l2

fail:
  j     .

irq_handler_ecall:
  ori   a5, a5, 0x01
  addi  a5, a5, 0x100
  mret

irq_handler_ebreak:
  ori   a5, a5, 0x02
  addi  a5, a5, 0x100
  mret

irq_handler_ext:
  ori   a5, a5, 0x04
  addi  a5, a5, 0x100
  mret

irq_handler_sw:
  ori   a5, a5, 0x08
  addi  a5, a5, 0x100
  mret

.section .bss
test_bss_var:
  .word
