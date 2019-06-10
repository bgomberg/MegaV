.section .text
.globl _start
_start:
	csrsi mtvec,0x10
	csrsi mstatus,0x08
	j l1
	li a5,0x0bd
	j .

l1:
	csrr a3,mstatus
	csrr a4,mtvec
	li t0,0x11
	li t1,0x31
	add t1,t0,t1
	add t2,t0,t1
	sw t2,0x80(x0)
	lw a0,0x80(x0)
	li a1,0x100
	jal l3

l2:
	j .

l3:
	addi a2,a2,0x01
	addi a2,a2,0x01
	beqz a2,l2
	beqz zero,l4

l4:
	jal gp,l2
