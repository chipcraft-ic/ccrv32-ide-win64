#H*****************************************************************************
#
# Copyright (c) 2019 ChipCraft Sp. z o.o. All rights reserved
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#   - Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   - Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in
#     the documentation and/or other materials provided with the
#     distribution.
#   - Neither the name of ChipCraft Sp. z o.o. nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# ******************************************************************************
# File Name : stackprotect.S
# Author    : Rafal Harabien
# ******************************************************************************
# $Date: 2019-09-11 09:01:01 +0200 (śro, 11 wrz 2019) $
# $Revision: 464 $
#H******************************************************************************

#include <specialreg.h>

.macro inc_var var_name
  la t4, \var_name
  lw t6, (t4)
  add t6, t6, 1
  sw t6, (t4)
.endm

#define MERGE(a, b)  a##b
#define OK_LABEL_(line) MERGE(.ok, line)
#define OK_LABEL OK_LABEL_(__LINE__)

#define ASSERT_ZERO(reg) \
  beqz reg, OK_LABEL; \
  _ASSERT_REST

#define ASSERT_NOT_ZERO(reg) \
  bnez reg, OK_LABEL; \
  _ASSERT_REST

#define ASSERT_EQ(reg, validReg) \
  beq reg, validReg, OK_LABEL; \
  _ASSERT_REST

#define _ASSERT_REST \
  nop; \
  inc_var g_failedTests; \
  la a0, test_failed_str; \
  li a1, __LINE__; \
  jal safe_printf; \
  nop; \
OK_LABEL: \
  inc_var g_totalTests

#define EXPECT_EXCEPTION_BEGIN(num) \
  la t4, expected_exception; \
  li t6, num; \
  sw t6, (t4);

#define EXPECT_EXCEPTION_END \
  la t4, expected_exception; \
  lw t4, (t4); \
  ASSERT_ZERO(t4)

.data

safe_printf_buffer:
  .word 0 # IRQ Controller Status Register
  .word 0 # Stack Pointer
  .word 0 # Resturn Address

test_val:
  .word 0xFFFFFFFF

.text

safe_printf:
  la t5, safe_printf_buffer
  sw sp, 4(t5)                           # save Stack Pointer in buffer
  sw ra, 8(t5)                           # save Return Address in buffer
  csrr t6, mcontrol                      # load control register
  sw t6, 0(t5)                           # save control register in buffer
  csrrci t1, mcontrol, SPROT_EN_MASK     # disable stack protection
  la sp, safe_printf_stack_end
  jal printf                             # do the printing job
  la t5, safe_printf_buffer
  lw t6, 0(t5)                           # read old control register value
  lw sp, 4(t5)                           # restore Stack Pointer
  lw ra, 8(t5)                           # restore Return Address
  csrw mcontrol, t6                      # restore control register
  jr ra

.global testStackProtection
testStackProtection:
  add sp, sp, -16                        # alloc stack
  sw s0, 0(sp)                           # save s0 on stack
  sw s1, 4(sp)                           # save s1 on stack
  sw s2, 8(sp)                           # save s2 on stack
  sw s3, 12(sp)                          # save s3 on stack

  move s0, sp                            # save current Stack Pointer
  move s1, ra                            # save Return Address

  csrrci t1, mcontrol, SPROT_EN_MASK     # disable stack protection
  la sp, stack_end                       # set temporary stack
  la t2, stack_begin
  csrw mstackmin, t2                     # set stack pointer min. value
  csrw mstackmax, sp                     # set stack pointer min. value
  csrrsi t1, mcontrol, SPROT_EN_MASK     # enable stack protection

  addi sp, sp, -4

  # now test Stack Protection

  lw zero, -4(sp)  # ok

  lh zero, -4(sp)  # ok
  lh zero, -2(sp)  # ok

  lb zero, -4(sp)  # ok
  lb zero, -3(sp)  # ok
  lb zero, -2(sp)  # ok
  lb zero, -1(sp)  # ok

  sw zero, -4(sp)  # ok

  sh zero, -4(sp)  # ok
  sh zero, -2(sp)  # ok

  sb zero, -4(sp)  # ok
  sb zero, -3(sp)  # ok
  sb zero, -2(sp)  # ok
  sb zero, -1(sp)  # ok

  sb zero, 0(sp)  # ok
  sb zero, 1(sp)  # ok
  sb zero, 2(sp)  # ok
  sb zero, 3(sp)  # ok
  sb zero, 4(sp)  # ok

  sh zero, 0(sp)  # ok
  sh zero, 2(sp)  # ok
  sh zero, 4(sp)  # ok

  sw zero, 0(sp)  # ok
  sw zero, 4(sp)  # ok

  lb zero, 0(sp)  # ok
  lb zero, 1(sp)  # ok
  lb zero, 2(sp)  # ok
  lb zero, 3(sp)  # ok
  lb zero, 4(sp)  # ok

  lh zero, 0(sp)  # ok
  lh zero, 2(sp)  # ok
  lh zero, 4(sp)  # ok

  lw zero, 0(sp)  # ok
  lw zero, 4(sp)  # ok

  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  sb zero, 5(sp)  # fail
  EXPECT_EXCEPTION_END
  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  sb zero, 6(sp)  # fail
  EXPECT_EXCEPTION_END
  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  sb zero, 7(sp)  # fail
  EXPECT_EXCEPTION_END
  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  sb zero, 8(sp)  # fail
  EXPECT_EXCEPTION_END

  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  sh zero, 6(sp)  # fail
  EXPECT_EXCEPTION_END
  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  sh zero, 8(sp)  # fail
  EXPECT_EXCEPTION_END

  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  sw zero, 8(sp)  # fail
  EXPECT_EXCEPTION_END

  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  lb zero, 5(sp)  # fail
  EXPECT_EXCEPTION_END
  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  lb zero, 6(sp)  # fail
  EXPECT_EXCEPTION_END
  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  lb zero, 7(sp)  # fail
  EXPECT_EXCEPTION_END
  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  lb zero, 8(sp)  # fail
  EXPECT_EXCEPTION_END

  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  lh zero, 6(sp)  # fail
  EXPECT_EXCEPTION_END
  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  lh zero, 8(sp)  # fail
  EXPECT_EXCEPTION_END

  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  lw zero, 8(sp)  # fail
  EXPECT_EXCEPTION_END

  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  sb zero, -5(sp)  # fail
  EXPECT_EXCEPTION_END
  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  sb zero, -6(sp)  # fail
  EXPECT_EXCEPTION_END
  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  sb zero, -7(sp)  # fail
  EXPECT_EXCEPTION_END
  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  sb zero, -8(sp)  # fail
  EXPECT_EXCEPTION_END

  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  sh zero, -6(sp)  # fail
  EXPECT_EXCEPTION_END
  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  sh zero, -8(sp)  # fail
  EXPECT_EXCEPTION_END

  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  sw zero, -8(sp)  # fail
  EXPECT_EXCEPTION_END

  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  lb zero, -5(sp)  # fail
  EXPECT_EXCEPTION_END
  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  lb zero, -6(sp)  # fail
  EXPECT_EXCEPTION_END
  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  lb zero, -7(sp)  # fail
  EXPECT_EXCEPTION_END
  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  lb zero, -8(sp)  # fail
  EXPECT_EXCEPTION_END

  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  lh zero, -6(sp)  # fail
  EXPECT_EXCEPTION_END
  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  lh zero, -8(sp)  # fail
  EXPECT_EXCEPTION_END

  EXPECT_EXCEPTION_BEGIN(IRQ_EXC_ILLEGAL)
  lw zero, -8(sp)  # fail
  EXPECT_EXCEPTION_END

  csrrci s3, mcontrol, SPROT_EN_MASK     # disable stack protection

  move sp, s0                            # restore Stack Pointer
  move ra, s1                            # restore Return Address
  lw s0, 0(sp)                           # restore s0 from stack
  lw s1, 4(sp)                           # restore s1 from stack
  lw s2, 8(sp)                           # restore s2 from stack
  lw s3, 12(sp)                          # restore s3 from stack
  add sp, sp, 16                         # free stack
  jr ra                                  # return

###################
# Exception handler
####################

.global exception_handler
exception_handler:
.common_handler:
  csrr t0, mcause                        # get cause

  #j .exit_exc
  #nop

  # Check if reason is not 0
  beqz t0, .invalid_handler

  j .handler_ok

  # Invalid handler
.invalid_handler:
  la a0, invalid_handler_str
  csrr t0, mepc                          # read RET_PC
  jal safe_printf

.handler_ok:
  # Update expected exception variable
  la t1, expected_exception
  lw t1, (t1)
  beq t0, t1, .exc_rsn_ok                # check if this is expected exception

  # Invalid exception
  la a0, unexp_exc_str
  csrr a1, mepc                          # read RET_PC
  move a2, t1
  move a3, t0
  jal safe_printf
  j .exit_exc

.exc_rsn_ok:
  la t1, expected_exception
  sw zero, (t1)                          # clear expected_exception

.exit_exc:
  csrr t1, mepc                          # read RET_PC
  lh t0, 0(t1)
  andi t0, t0, 3                         # check if compressed
  li t2, 3
  beq t0, t2, incrfour
incrtwo:
  addi t1, t1, 2                         # go to next instruction
  csrw mepc, t1
  mret                                   # end of exception handler
incrfour:
  addi t1, t1, 4                         # go to next instruction
  csrw mepc, t1
  mret                                   # end of exception handler

#ifndef _NO_SPRAM_MEMORY
.section .spram_bss, "aw", @nobits
#else
.data
#endif
  .space 8, 0 # space before
stack_begin:
  .space 8, 0
stack_end:
  .space 8, 0 # space after

safe_printf_stack_begin:
  .space 512, 0
safe_printf_stack_end:

##########
# Strings
##########

.data

test_failed_str:
  .ascii "(" __FILE__ ":%u) Test failed!\n\0"
unexp_exc_str:
  .ascii "Unexpected exception on address %p: expected reason 0x%x, got 0x%x\n\0"
invalid_handler_str:
  .ascii "Invalid handler called (syscall vs exception) from address %p\n\0"

.align 4
expected_exception:
  .word 0
