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
# File Name : specialreg.h
# Author    : Krzysztof Marcinek
# ******************************************************************************
# $Date: 2021-06-28 11:16:55 +0200 (pon, 28 cze 2021) $
# $Revision: 712 $
#H******************************************************************************

#ifndef _SPECIALREG_H_
#define _SPECIALREG_H_

/*
 * Custom CSR Registers
 */
#define mconfig0                0x07C0       /* Processor Resources Register 0                   */
#define mconfig1                0x07C1       /* Processor Resources Register 1                   */
#define mconfig2                0x07C2       /* Processor Resources Register 2                   */
#define mcontrol                0x07C8       /* Processor Control Register                       */
#define mstackmin               0x07C9       /* Stack Pointer Protection Min. Register           */
#define mstackmax               0x07CA       /* Stack Pointer Protection Max. Register           */
#define mromunlock              0x07CD       /* ROM Unlock Register                              */

/*
 * Exception codes definitions
 */
#define IRQ_EXC_ILLEGAL         0x00000002   /* illegal instruction                              */

/*
 * Cache Controller Utils
 */
#define ICACHE_CTRL_BASE        0xF0070000   /* Instruction Cache Controller Base                */
#define DCACHE_CTRL_BASE        0xF0072000   /* Data Cache Controller Base                       */
#define CACHE_STAT_OFFSET       0x0          /* Cache Controller Status offset                   */
#define CACHE_FLUSH_OFFSET      0x4          /* Cache Controller Flush offset                    */
#define DCACHE_BUSY_MASK        0x4          /* Data Cache Controller busy bit mask              */

/*
 * Processor Startup Utils
 */
#define ROM_SIZE_SHIFT          0            /* ROM size shift                                   */
#define ROM_SIZE_MASK           0x1F         /* ROM size mask                                    */
#define ROM_BASE                0x00000      /* ROM memory upper base address                    */
#define RAM_SIZE_SHIFT          5            /* RAM size shift                                   */
#define RAM_SIZE_MASK           0x1F         /* RAM size mask                                    */
#define RAM_BASE                0x40000      /* RAM memory upper base address                    */
#define SPRAM_SIZE_SHIFT        18           /* SPRAM size shift                                 */
#define SPRAM_SIZE_MASK         0x1F         /* SPRAM size mask                                  */
#define FPU_MASK                0x28         /* FPU mask                                         */
#define SPRAM_BASE              0xD0000      /* SPRAM memory upper base address                  */
#define AMBA_BASE               0xE0000      /* Upper base address of AMBA bus                   */
#define AMBA_INFO0_OFFSET       0x0          /* AMBA_INFO_0 offset                               */
#define PWD_ADDRESS             0xF0020000   /* power management controller address              */
#define WDT_MASK                0x2000       /* AMBA_INFO_0 watchdog mask                        */
#define WDT_ADDRESS             0xE0001500   /* watchdog address on AMBA bus                     */
#define WDT_UNLOCK_MASK         11           /* watchdog unlock mask                             */
#define WDT_ENABLE_MASK         1            /* watchdog enable mask                             */
#define WDT_UNLOCK_OFFSET       0x0          /* watchdog unlock offset                           */
#define WDT_CTRL_OFFSET         0x4          /* watchdog control offset                          */
#define WDT_PERIOD_OFFSET       0xC          /* watchdog period offset                           */
#define WDT_PRES_OFFSET         0x14         /* watchdog prescaler offset                        */
#define WDT_RESET_MASK          0x8          /* watchdog reset mask                              */
#define RST_RSN_OFFSET          0x4          /* power management reset reason offset             */
#define SPROT_EN_MASK           0x8          /* stack protection enable mask                     */
#define IRQ_EN_MASK             0x8          /* interrupt enable mask                            */
#define IRQ_EXT_EN_SHIFT        11           /* external interrupt enable shift                  */
#define IRQ_CAUSE_SHIFT         31           /* interrupt cause shift                            */
#define IRQ_CAUSE_MSOFT         3            /* machine software interrupt                       */
#define IRQ_CAUSE_MTIMER        7            /* machine timer interrupt                          */
#define IRQ_CAUSE_NMI           0x700        /* nmi interrupt                                    */
#define PLIC_BASE               0xF0038      /* plic base address                                */
#define PLIC_CLAIM_OFFSET       140          /* plic claim offset                                */
#define CACHE_EN_MASK           0x1          /* cache enable mask                                */
#define CACHE_FT_MASK           0x8          /* cache parity/ecc enable mask                     */
#define CACHE_SC_MASK           0x20         /* cache scrambling enable mask                     */
#define CACHE_HE_MASK           0x100        /* hard error mask                                  */
#define ROM_UNLOCK_KEY          0xA5000000   /* ROM unlock key                                   */
#define FPU_INIT_MASK           0x4000       /* FPU initial state mask                           */

#endif /* _SPECIALREG_H_ */
