#H*****************************************************************************
#
# Copyright (c) 2018 ChipCraft Sp. z o.o. All rights reserved
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
# File Name : startup.S
# Author    : Rafal Harabien, Krzysztof Marcinek
# ******************************************************************************
# $Date: 2022-12-05 17:07:41 +0100 (pon, 05 gru 2022) $
# $Revision: 928 $
#H******************************************************************************

# Useful defines:
#
# _STARTUP_STACK_SIZE - size of initial stack
# _STARTUP_STACK_IN_RAM - use RAM instead of SPRAM
# _STARTUP_STACK_IN_RAM_NUM_CORES - maximal number of cores for stack allocation in RAM
#
# _DONT_BREAK_ON_EXCEPTION - do not break on exception
# _DONT_BREAK_ON_BREAK - do not break on break instruction
#
# _EXEC_TIMEOUT - define execution timeout using watchdog
#

#include <specialreg.h>

#ifdef _NO_SPRAM_MEMORY
  #define _STARTUP_STACK_IN_RAM
  #define _STARTUP_STACK_IN_RAM_NUM_CORES _CPU_NUM_CORES
  #ifndef _CPU_NUM_CORES
    #error _CPU_NUM_CORES not defined
  #endif
#endif

#ifndef _STARTUP_STACK_SIZE
  #define _STARTUP_STACK_SIZE 2048
  #ifdef _STARTUP_STACK_IN_RAM
    #warning _STARTUP_STACK_SIZE set to 2048
  #endif
#endif
#ifndef _STARTUP_STACK_IN_RAM_NUM_CORES
  #define _STARTUP_STACK_IN_RAM_NUM_CORES 1
  #ifdef _STARTUP_STACK_IN_RAM
    #error _STARTUP_STACK_IN_RAM_NUM_CORES not defined
  #endif
#endif


.section .boot, "ax", @progbits
.align  6
.global __start
__start:
#ifdef _ASM_TEST_CODE
  j asm_test_start                      # test start code
#endif

  j start                               # start code

.align 2
trap_vector:
  j trap_handler

.global end_loop
end_loop:
  li gp, 1
  sw gp, tohost, t5                     # simulation exit
end_jump:
  j end_jump
  nop

.global start
start:
  nop                                   # feed pipeline with nops (for x-prop ASIC simulation)
  nop
  nop
  nop
  jal common_start                      # common start
  j program                             # jump to rest of code

# Start used for main and additional cores
.global common_start
common_start:

  la t0,   trap_vector                  # set trap vector
  csrw mtval, t0

  or sp,   zero, zero                   # clear register file
  or gp,   zero, zero
  or tp,   zero, zero
  or t0,   zero, zero
  or t1,   zero, zero
  or t2,   zero, zero
  or s0,   zero, zero
  or s1,   zero, zero
  or a0,   zero, zero
  or a1,   zero, zero
  or a2,   zero, zero
  or a3,   zero, zero
  or a4,   zero, zero
  or a5,   zero, zero
  or a6,   zero, zero
  or a7,   zero, zero
  or s2,   zero, zero
  or s3,   zero, zero
  or s4,   zero, zero
  or s5,   zero, zero
  or s6,   zero, zero
  or s7,   zero, zero
  or s8,   zero, zero
  or s9,   zero, zero
  or s10,  zero, zero
  or s11,  zero, zero
  or t3,   zero, zero
  or t4,   zero, zero
  or t5,   zero, zero
  or t6,   zero, zero
  #or ra, zero, zero                    # saved in jal instruction

#ifdef __riscv_float_abi_double

  csrr t0, misa                         # check FPU existance
  li t1, FPU_MASK
  and t1, t1, t0
  beq t1, zero, no_fpu

  fmv.w.x f0, zero
  fmv.w.x f1, zero
  fmv.w.x f2, zero
  fmv.w.x f3, zero
  fmv.w.x f4, zero
  fmv.w.x f5, zero
  fmv.w.x f6, zero
  fmv.w.x f7, zero
  fmv.w.x f8, zero
  fmv.w.x f9, zero
  fmv.w.x f10, zero
  fmv.w.x f11, zero
  fmv.w.x f12, zero
  fmv.w.x f13, zero
  fmv.w.x f14, zero
  fmv.w.x f15, zero
  fmv.w.x f16, zero
  fmv.w.x f17, zero
  fmv.w.x f18, zero
  fmv.w.x f19, zero
  fmv.w.x f20, zero
  fmv.w.x f21, zero
  fmv.w.x f22, zero
  fmv.w.x f23, zero
  fmv.w.x f24, zero
  fmv.w.x f25, zero
  fmv.w.x f26, zero
  fmv.w.x f27, zero
  fmv.w.x f28, zero
  fmv.w.x f29, zero
  fmv.w.x f30, zero
  fmv.w.x f31, zero
no_fpu:

#endif

cache_init:
  li t0, ICACHE_CTRL_BASE               # Instruction Cache Controller address
  li t2, DCACHE_CTRL_BASE               # Data Cache Controller address
  or t1, zero, CACHE_SC_MASK            # cache scrambling enable mask
  sw t1, CACHE_STAT_OFFSET(t0)          # enable instruction cache scrambling
  sw t1, CACHE_STAT_OFFSET(t2)          # enable data cache scrambling
  sw zero, CACHE_FLUSH_OFFSET(t0)       # flush instruction cache
  sw zero, CACHE_FLUSH_OFFSET(t2)       # flush data cache
  or t1, zero, CACHE_EN_MASK            # cache enable mask
  or t1, t1, CACHE_FT_MASK              # cache parity/ecc enable mask
  or t1, t1, CACHE_HE_MASK              # cache hard error enable mask
  sw t1, CACHE_STAT_OFFSET(t0)          # enable instruction cache
  sw t1, CACHE_STAT_OFFSET(t2)          # enable data cache
  .option push
  .option norelax
  la gp, __global_pointer$              # init global pointer
  .option pop
#ifdef _STARTUP_STACK_IN_RAM
  csrr t1, mhartid                      # get coreID
  addi t1, t1, 1                        # increment t1
  li t2, _STARTUP_STACK_SIZE
  mul t1, t2, t1                        # multiply stack size and (coreID + 1)
  la sp, _stack_begin                   # init stack pointer
  add sp, sp, t1                        # add dynamic offset based on coreID
  sub t1, sp, t2                        # subtract stack size
  csrw mstackmin, t1                    # set stack pointer min. value
#else
  la sp, _stack_end                     # init stack pointer
  la t2, _stack_begin                   # init stack pointer
  csrw mstackmin, t2                    # set stack pointer min. value
#endif
  addi t1, sp, -4                       # decrement t1
  csrw mstackmax, t1                    # set stack pointer min. value

  csrw mscratch, ra                     # save ra
  jal hardware_init                     # hardware specific initialization
  csrr ra, mscratch                     # restore ra

  csrrsi t1, mcontrol, SPROT_EN_MASK    # enable stack protection
  csrrsi t1, mstatus, IRQ_EN_MASK       # enable interrupts
  addi t1, zero, 1
  slli t1, t1, IRQ_EXT_EN_SHIFT
  csrw mie, t1                          # enable external interrupts

check_spram_size:
  csrr s1, mconfig0                     # get CPU features register
  srl t0, s1, SPRAM_SIZE_SHIFT
  and t0, t0, SPRAM_SIZE_MASK           # t0 = log2(SPRAM size)
  li s2, 1
  sll s2, s2, t0                        # s2 = SPRAM size
  lui s3, SPRAM_BASE                    # SPRAM memory base address
  add s3, s3, s2                        # s3 = SPRAM memory end
  la t1, _espram_bss                    # SPRAM BSS section end address
  sub t1, s3, t1                        # compare to end of physical SPRAM
  blt t1, zero, no_mem                  # branch if there is not enough SPRAM memory

init_spram_data:
  la t0, _sspram_data                   # data section start address
  la t1, _espram_data                   # data section end address
  la t2, _spram_data_load               # address of data in ROM memory
  beq t0, t1, init_spram_bss            # check if data section is empty
  addi t1, t1, -4                       # last valid address
  addi t0, t0, -4
copy_spram_data:
  add t0, t0, 4                         # increment destination ptr
  lw t3, 0(t2)                          # copy word from ROM memory
  add t2, t2, 4                         # increment source ptr
  sw t3, 0(t0)                          # save word in SPRAM memory
  sub t4, t1, t0                        # last ptr - cur ptr
  bgt t4, zero, copy_spram_data         # loop
init_spram_bss:
  la t0, _sspram_bss                    # SPRAM BSS section start address
  la t1, _espram_bss                    # SPRAM BSS section end address
  beq t0, t1, common_done               # check if SPRAM BSS section is empty
  addi t1, t1, -4                       # last valid address
  add t0, t0, -4
zero_spram_bss:
  add t0, t0, 4                         # increment pointer
  sw zero, 0(t0)                        # zero word in SPRAM BSS
  sub t4, t1, t0                        # last ptr - cur ptr
  bgt t4, zero, zero_spram_bss          # loop
common_done:
  jr ra                                 # return

program:

#ifdef __riscv_float_abi_double
check_hardfloat:
  csrr s1, misa                         # get CPU ISA
  ori t0, zero, FPU_MASK                # load FPU mask
  and t0, s1, t0                        # check if FPU exists
  beq t0, zero, no_hardfloat
#endif

  li s1, FPU_INIT_MASK                  # load FPU initial state mask
  csrs mstatus, s1                      # enable FPU if present

  csrr s1, mconfig0                     # get CPU features register
  srl t0, s1, RAM_SIZE_SHIFT
  and t0, t0, RAM_SIZE_MASK             # t0 = log2(RAM size)
  li s2, 1
  sll s2, s2, t0                        # s2 = RAM size
  lui s3, RAM_BASE                      # RAM memory base address
  add s3, s3, s2                        # s3 = RAM memory end

  srl t1, s1, ROM_SIZE_SHIFT
  and t1, t1, ROM_SIZE_MASK             # t1 = log2(ROM size)
  li s2, 1
  sll s2, s2, t1                        # s2 = ROM size
  lui s1, ROM_BASE                      # ROM memory base address
  add s2, s1, s2                        # s2 = ROM memory end

check_ram_memory_size:
  la t2, _sheap                         # start of unused RAM
  sub t2, s3, t2                        # compare to end of physical RAM
  beq t0, zero, check_rom_memory_size   # no on-chip memory, proceeed
  blt t2, zero, no_mem                  # branch if there is not enough RAM memory

check_rom_memory_size:
  la t2, _sdata                         # data section start address
  la t0, _edata                         # data section end address
  sub t2, t0, t2                        # size of data section
  la t0, _data_load                     # address of data in ROM memory
  add t2, t2, t0                        # end of data section in ROM
  sub t2, s2, t2                        # compare to end of physical ROM
  beq t1, zero, init_data               # no on-chip memory, proceeed
  blt t2, zero, no_mem                  # branch if there is not enough ROM memory

init_data:
  la t0, _sdata                         # data section start address
  la t1, _edata                         # data section end address
  la t2, _data_load                     # address of data in ROM memory
  beq t0, t1, init_bss                  # check if data section is empty
  addi t1, t1, -4                       # last valid address
  add t0, t0, -4
copy_data:
  add t0, t0, 4                         # increment destination ptr
  lw t3, 0(t2)                          # copy word from ROM memory
  add t2, t2, 4                         # increment source ptr
  sw t3, 0(t0)                          # save word in RAM memory
  sub t4, t1, t0                        # last ptr - cur ptr
  bgt t4, zero, copy_data               # loop

init_bss:
  la t0, _sbss                          # BSS section start address
  la t1, _ebss                          # BSS section end address
  beq t0, t1, done                      # check if BSS section is empty
  addi t1, t1, -4                       # last valid address
  add t0, t0, -4
zero_bss:
  add t0, t0, 4                         # increment pointer
  sw zero, 0(t0)                        # zero word in BSS
  sub t4, t1, t0                        # last ptr - cur ptr
  bgt t4, zero, zero_bss                # loop

done:
  addi sp, sp, -16                      # call arguments area

#ifdef _EXEC_TIMEOUT

  li t0, PWD_ADDRESS                    # load power management controller address
  lw t1, RST_RSN_OFFSET(t0)             # read reset reason
  ori t2, zero, WDT_RESET_MASK          # load watchdog reset mask
  beq t1, t2, end_loop                  # end simulation if watchdog reset

  lui t0, AMBA_BASE                     # load amba base address
  lw t1, AMBA_INFO0_OFFSET(t0)          # load AMBA_INFO_0
  lui t2, WDT_MASK                      # load watchdog mask
  and t1, t1, t2                        # check is watchdog is present
  beq zero, t1, end_loop                # end simulation if no watchdog
  li t0, WDT_ADDRESS                    # load watchdog address
  ori t1, zero, WDT_UNLOCK_MASK         # load unlock mask
  lui t2, %hi(_EXEC_TIMEOUT)
  ori t2, t2, %lo(_EXEC_TIMEOUT)        # load period
  ori t3, zero, WDT_ENABLE_MASK         # load enable mask
wdt_pres:
  li t5, 512                            # load prescaler value
  sw t1, WDT_UNLOCK_OFFSET(t0)          # unlok registers
  sw t5, WDT_PRES_OFFSET(t0)            # store period
  lw t4, WDT_PRES_OFFSET(t0)            # load period
  bne t5, t4, wdt_pres                  # check if prescaler stored
wdt_period:
  sw t1, WDT_UNLOCK_OFFSET(t0)          # unlok registers
  sw t2, WDT_PERIOD_OFFSET(t0)          # store period
  lw t4, WDT_PERIOD_OFFSET(t0)          # load period
  bne t2, t4, wdt_period                # check if period stored
wdt_enable:
  sw t1, WDT_UNLOCK_OFFSET(t0)          # unlok registers
  sw t3, WDT_CTRL_OFFSET(t0)            # enable watchdog
  lw t4, WDT_CTRL_OFFSET(t0)            # load control
  bne t3, t4, wdt_enable                # check if watchdog enabled

#endif

#ifdef _LOCK_ROM
  li t3, ROM_UNLOCK_KEY                 # load rom unlock key
  csrw mromunlock, t3                   # lock rom memory
#endif

  jal board_init                        # board specific initialization
  jal __libc_init_array                 # init libc
  jal _ccrv32_init
  jal __libc_impure_alloc               # allocate impure data
  jal __libc_impure_init                # initialize impure data
  jal main
  j end_loop                            # jump to end_loop

#ifdef __riscv_float_abi_double
no_hardfloat:                           # no FPU loop
#ifdef _EXEC_TIMEOUT
  j end_loop
#else
  ebreak
  j no_hardfloat
#endif
  nop
#endif

no_mem:                                 # no mem loop
#ifdef _EXEC_TIMEOUT
  j end_loop
#else
  ebreak
  j no_mem
#endif

.weak mtvec_handler
mtvec_handler:
  j trap_handler

.weak stvec_handler
stvec_handler:
  j trap_handler

.weak trap_handler
trap_handler:
  csrw mscratch, t0                     # save t0
  csrr t0, mcause
  srl t0, t0, IRQ_CAUSE_SHIFT           # check if interrupt or exception
  bne t0, zero, interrupt_handler
  beq zero, zero, exception_handler
  ebreak

.weak interrupt_handler
interrupt_handler:

  csrr t0, mcause                       # check if nmi interrupt
  andi t0, t0, IRQ_CAUSE_NMI
  sll t0, t0, 1
  bne t0, zero, nmi_handler

int_enter:
  addi sp, sp, -128-256-8-36-4            # alloc stack
  csrr t0, mscratch                     # restore t0 from scratch
  sw t0, 36+20(sp)                      # save t0 on stack
  sw t1, 36+24(sp)                      # save t1 on stack
  csrr t0, mepc                         # read mepc
  sw t0, 16(sp)                         # save mepc on stack
  csrr t0, mstatus                      # read mstatus
  sw t0, 24(sp)                         # save mstatus on stack

  csrr t0, mcause                       # check if timer or software interrupt
  slli t0, t0, 1
  srli t0, t0, 1
  sw t0, 28(sp)                         # save mcause on stack
  ori t1, zero, IRQ_CAUSE_MSOFT
  beq t0, t1, int_enter_stack
  ori t1, zero, IRQ_CAUSE_MTIMER
  beq t0, t1, int_enter_stack

  lui t0, PLIC_BASE
  lw t0, PLIC_CLAIM_OFFSET(t0)          # read claim register
  sw t0, 32(sp)                         # save claim on stack

  csrrsi t0, mstatus, IRQ_EN_MASK       # re-enable interrupts

int_enter_stack:
  # save registers on stack (except t0, t1)
  sw ra, 36+0(sp)
  #sw sp, 36+4(sp)
  #sw gp, 36+8(sp)
  sw tp, 36+16(sp)
  #sw t0, 36+20(sp)
  #sw t1, 36+24(sp)
  sw t2, 36+28(sp)
  sw s0, 36+32(sp)
  sw s1, 36+36(sp)
  sw a0, 36+40(sp)
  sw a1, 36+44(sp)
  sw a2, 36+48(sp)
  sw a3, 36+52(sp)
  sw a4, 36+56(sp)
  sw a5, 36+60(sp)
  sw a6, 36+64(sp)
  sw a7, 36+68(sp)
  sw s2, 36+72(sp)
  sw s3, 36+76(sp)
  sw s4, 36+80(sp)
  sw s5, 36+84(sp)
  sw s6, 36+88(sp)
  sw s7, 36+92(sp)
  sw s8, 36+96(sp)
  sw s9, 36+100(sp)
  sw s10, 36+104(sp)
  sw s11, 36+108(sp)
  sw t3, 36+112(sp)
  sw t4, 36+116(sp)
  sw t5, 36+120(sp)
  sw t6, 36+124(sp)

  addi t0, sp, 36+124+12
  srli t0, t0, 3
  slli t0, t0, 3

int_isr_check:
  lw t0, 28(sp)                         # read mcause from stack
  ori t1, zero, IRQ_CAUSE_MSOFT
  beq t0, t1, int_soft                  # jump to software handler
  ori t1, zero, IRQ_CAUSE_MTIMER
  beq t0, t1, int_timer                 # jump to timer handler

int_enter_rest:
  lw t0, 32(sp)                         # read claim from stack
  slli t0, t0, 2                        # claim * 4
  la t1, int_vectors                    # address of IRQ vector table
  add t0, t1, t0                        # address of IRQ handler
  lw t0, (t0)                           # load IRQ handler address
  la ra, int_ret                        # set return address for IRQ handler
  jr t0                                 # jump to IRQ handler

int_soft:
  la t0, isr_soft                       # load software isr address
  la ra, int_ret_stack                  # set return address for IRQ handler
  jr t0                                 # jump to IRQ handler

int_timer:
  la t0, isr_timer                      # load timer isr address
  la ra, int_ret_stack                  # set return address for IRQ handler
  jr t0                                 # jump to IRQ handler

int_ret:
  lw t0, 24(sp)                         # read mstatus from stack
  csrw mstatus, t0                      # write mstatus (interrupt disabled again)
  lw t0, 32(sp)                         # read claim from stack
  lui t1, PLIC_BASE
  sw t0, PLIC_CLAIM_OFFSET(t1)          # complete claim

int_ret_stack:

  addi t0, sp, 36+124+12
  srli t0, t0, 3
  slli t0, t0, 3

  # Restore registers from stack (except t0)
  lw ra, 36+0(sp)
  #lw sp, 36+4(sp)
  #lw gp, 36+8(sp)
  lw tp, 36+16(sp)
  #lw t0, 36+20(sp)
  lw t1, 36+24(sp)
  lw t2, 36+28(sp)
  lw s0, 36+32(sp)
  lw s1, 36+36(sp)
  lw a0, 36+40(sp)
  lw a1, 36+44(sp)
  lw a2, 36+48(sp)
  lw a3, 36+52(sp)
  lw a4, 36+56(sp)
  lw a5, 36+60(sp)
  lw a6, 36+64(sp)
  lw a7, 36+68(sp)
  lw s2, 36+72(sp)
  lw s3, 36+76(sp)
  lw s4, 36+80(sp)
  lw s5, 36+84(sp)
  lw s6, 36+88(sp)
  lw s7, 36+92(sp)
  lw s8, 36+96(sp)
  lw s9, 36+100(sp)
  lw s10, 36+104(sp)
  lw s11, 36+108(sp)
  lw t3, 36+112(sp)
  lw t4, 36+116(sp)
  lw t5, 36+120(sp)
  lw t6, 36+124(sp)

int_ret_rest:
  lw t0, 16(sp)                         # read mepc from stack
  csrw mepc, t0                         # write mepc
  lw t0, 36+20(sp)                      # restore t0
  addi sp, sp, 128+256+8+36+4           # free allocated stack
  mret                                  # interrupt end

.weak exception_handler
exception_handler:
#ifndef _DONT_BREAK_ON_BREAK
  ebreak
  j end_jump
#else
  csrw mscratch, t0                     # save t0
  csrr t0, mepc
  lh t0, 0(t0)                          # check if compressed
  andi t0, t0, 3
  addi t0, t0, -3
  beq t0, zero, incr_four
incr_two:
  csrr t0, mepc
  add t0, t0, 2
  j incr_common
incr_four:
  csrr t0, mepc
  add t0, t0, 4
incr_common:
  csrw mepc, t0                         # write mepc
  csrr t0, mscratch                     # restore t0
  mret
#endif

.weak nmi_handler
nmi_handler:
  csrr t0, mscratch
  mret

# IRQ Vectors
.align 2
int_vectors:
  .word dummy_isr
  .word isr1
  .word isr2
  .word isr3
  .word isr4
  .word isr5
  .word isr6
  .word isr7
  .word isr8
  .word isr9
  .word isr10
  .word isr11
  .word isr12
  .word isr13
  .word isr14
  .word isr15

.weak isr1
isr1:
.weak isr2
isr2:
.weak isr3
isr3:
.weak isr4
isr4:
.weak isr5
isr5:
.weak isr6
isr6:
.weak isr7
isr7:
.weak isr8
isr8:
.weak isr9
isr9:
.weak isr10
isr10:
.weak isr11
isr11:
.weak isr12
isr12:
.weak isr13
isr13:
.weak isr14
isr14:
.weak isr15
isr15:
.weak isr_soft
isr_soft:
.weak isr_timer
isr_timer:
dummy_isr:
  # Dummy ISR
  jr ra

# Dummy board_init
.weak board_init
board_init:
  jr ra

.pushsection .tohost,"aw",@progbits
.align 6; .global tohost; tohost: .dword 0
.popsection

#ifdef _STARTUP_STACK_IN_RAM
.data
#else
.section .spram_bss, "aw", @nobits
#endif
# Stack for startup
.global _stack_begin
.global _stack_end
.align 3 # stack should be double-word aligned
_stack_begin:
#ifdef _STARTUP_STACK_IN_RAM
  .space _STARTUP_STACK_SIZE*_STARTUP_STACK_IN_RAM_NUM_CORES
#else
  .space _STARTUP_STACK_SIZE
#endif
_stack_end:
