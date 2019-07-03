.section .prog_entry, "ax"
.global prog_entry
prog_entry:
  j     main

.section .irq_handler
irq_handler:
  # Handle the interrupt
  csrr  s0, mcause
  li    s1, 0xb
  beq   s0, s1, irq_handler_ecall
  li    s1, 0x3
  beq   s0, s1, irq_handler_ebreak
  # Default handler
  ori   a5, a5, 0x40
  j     .

irq_handler_ecall:
  mret

irq_handler_ebreak:
  ori   a5, a5, 0x02
  lw    a0, 0x400(x0) # Trigger a memory access fault
  ori   a5, a5, 0xff

.section .text
main:
  # Enable interrupts
  csrsi mstatus,0x08
  fence
  fence.i
  j     l1
  li    a5, 0x0bd
  j     .

l1:
  csrr  a3, mstatus
  csrr  a4, mtvec
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
  ecall
  ebreak

l3:
  addi  a2, a2, 0x01
  addi  a2, a2, 0x01
  beqz  a2, l2
  beqz  zero, l4

l4:
  jal   gp, l2

.section .bss
test_bss_var:
  .word
