.global asm_test_start
asm_test_start:
  nop
  jal x1, zeros_all
  lui x2,1
  srai x3,x2,5
  jal x1, test
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop

  beqz x0,8
  nop
  beq x2,zero,8
  nop
  beq x3,zero,8
  nop
  beq x4,zero,8
  nop
  beq x5,zero,8
  nop
  beq x6,zero,8
  nop
  beq x7,zero,8
  nop
  beq x8,zero,8
  nop
  beq x9,zero,8
  nop
  beq x10,zero,8
  nop
  beq x11,zero,8
  nop
  beq x12,zero,8
  nop
  beq x13,zero,8
  nop

test:
  addi x1,x0,1          # x1 = 1
  addi x2,x1,-100       # x2 = 11
  add  x3,x1,x2         # x3 = 12
  add  x3,x1,x3         # x3 = 13
  add  x3,x1,x1         # x3 = 2
  add  x1,x1,x3         # x1 = 3
  add  x3,x1,x1         # x3 = 6
  beqz x2,skok
  beqz x0,skok
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  xor  x1,x2,x3
  and  x1,x2,x3
  or  x1,x2,x3
  xori x1,x2,10
  andi x1,x2,10
  ori x1,x2,10
  addi x1,x0,0
  slt x1,x1,x3
  sltu x1,x1,x3
  jal x3,  skok
  ori x1,x2,10
  nop
  nop
  jal x3, skok2
  nop

skok:
  addi  x2,x1,1
  lui x1, 1
  auipc x2, 1
  addi x1,x0,4
  nop
  nop

skok2:
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  jal x1, skok3
  nop
  nop
  nop
  nop
  jal x1, skok4
  nop
  nop

skok3:
  addi x2,x0,4
  jalr x0,x1,0

skok4:
  nop
  nop
  nop
  nop
  nop
  addi x3,zero,0
  addi x3,x3,5
  lui x2,0xfffff
  addi x2,x2,0xff
  srai x2,x2,8
loop:

  addi x2,x2,1
  bne x3,x2,loop
  nop
  nop
  addi x3,zero,0
  addi x10,x3,-1        # x10 = 0xffffffff (-1 ze znakiem)
  slt x1,x10,x3
  sltu x1,x10,x3
  addi x2,x0,5
  addi x1,x0,3
  addi x10,x10,0
  mul x15,x10,x10
  mulh x15,x10,x10
  mulhsu x15,x10,x10
  mulhu x15,x10,x10
  nop 
  nop
  nop
  nop
  nop
  addi x1,zero,5
  srli x10,x10,8
  addi x10,x10,0
  addi x1,zero,0
  div   x5,x10,x1
  divu  x5,x10,x1
  rem   x5,x10,x1
  remu  x5,x10,x1
  addi x1,zero,-1
  div   x5,x10,x1
  divu  x5,x10,x1
  rem   x5,x10,x1
  remu  x5,x10,x1
  addi  x1,zero,-1
  addi x2,zero,0

test_cos:
  mul x15,x10,x10
  mulh x15,x10,x10
  mulhsu x15,x10,x10
  mulhu x15,x10,x10
  addi x10,x10,1
  addi x2,x2,1
  bge x2,x1, test_cos

  j end_loop            # finish test

zeros_all:
  addi x2,zero,0
  addi x3,zero,0
  addi x4,zero,0
  addi x5,zero,0
  addi x6,zero,0
  addi x7,zero,0
  addi x8,zero,0
  addi x9,zero,0
  addi x10,zero,0
  addi x11,zero,0
  addi x12,zero,0
  addi x13,zero,0
  addi x14,zero,0
  addi x15,zero,0
  addi x16,zero,0
  addi x17,zero,0
  addi x18,zero,0
  addi x19,zero,0
  addi x20,zero,0
  addi x21,zero,0
  addi x22,zero,0
  addi x23,zero,0
  addi x24,zero,0
  addi x25,zero,0
  addi x26,zero,0
  addi x27,zero,0
  addi x28,zero,0
  addi x29,zero,0
  addi x30,zero,0
  addi x31,zero,0

  ret

# dummy main
.global main
main:
  jr ra

