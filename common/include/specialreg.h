/******************************************************************************

  Copyright (c) 2018 ChipCraft Sp. z o.o. All rights reserved

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:
    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    - Neither the name of ChipCraft Sp. z o.o. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

  ******************************************************************************
  File Name : specialreg.h
  Author    : Krzysztof Marcinek
  ******************************************************************************
  $Date: 2025-03-24 10:59:53 +0100 (Mon, 24 Mar 2025) $
  $Revision: 1137 $
 ******************************************************************************/

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

#define miopmpcfg0              0x08A0      /* ioPMP Configuration Register 0                    */
#define miopmpcfg1              0x08A1      /* ioPMP Configuration Register 1                    */
#define miopmpcfg2              0x08A2      /* ioPMP Configuration Register 2                    */
#define miopmpcfg3              0x08A3      /* ioPMP Configuration Register 3                    */

#define miopmpaddr0             0x08B0      /* ioPMP Configuration Register 0                    */
#define miopmpaddr1             0x08B1      /* ioPMP Configuration Register 1                    */
#define miopmpaddr2             0x08B2      /* ioPMP Configuration Register 2                    */
#define miopmpaddr3             0x08B3      /* ioPMP Configuration Register 3                    */
#define miopmpaddr4             0x08B4      /* ioPMP Configuration Register 4                    */
#define miopmpaddr5             0x08B5      /* ioPMP Configuration Register 5                    */
#define miopmpaddr6             0x08B6      /* ioPMP Configuration Register 6                    */
#define miopmpaddr7             0x08B7      /* ioPMP Configuration Register 7                    */
#define miopmpaddr8             0x08B8      /* ioPMP Configuration Register 8                    */
#define miopmpaddr9             0x08B9      /* ioPMP Configuration Register 9                    */
#define miopmpaddr10            0x08BA      /* ioPMP Configuration Register 10                   */
#define miopmpaddr11            0x08BB      /* ioPMP Configuration Register 11                   */
#define miopmpaddr12            0x08BC      /* ioPMP Configuration Register 12                   */
#define miopmpaddr13            0x08BD      /* ioPMP Configuration Register 13                   */
#define miopmpaddr14            0x08BE      /* ioPMP Configuration Register 14                   */
#define miopmpaddr15            0x08BF      /* ioPMP Configuration Register 15                   */

/*
 * Exception codes definitions
 */
#define IRQ_EXC_ILLEGAL         0x00000002   /* illegal instruction                              */

/*
 * Cache Controller Utils
 */
#define ICACHE_CTRL_BASE        0xF0070000   /* Instruction Cache Controller Base                */
#define DCACHE_CTRL_BASE        0xF0072000   /* Data Cache Controller Base                       */
#define CACHE_STCR_OFFSET       0x0          /* Cache Controller Status and Control offset       */
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
#define NMI_DIS_MASK            0x200        /* NMI enable mask                                  */

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
#define CACHE_BM_MASK           0x400        /* store buffer merging mask                        */
#define CACHE_NM_MASK           0x800        /* NMI mask                                         */
#define ROM_UNLOCK_KEY          0xA5000000   /* ROM unlock key                                   */
#define ROM_UNLOCK_VALUE        0x00000001   /* ROM unlock value                                 */
#define FPU_INIT_MASK           0x4000       /* FPU initial state mask                           */

/*
 * PMP definitions
 */
#define PMP_READ                0x1          /* PMP read mask                                    */
#define PMP_WRITE               0x2          /* PMP write mask                                   */
#define PMP_EXEC                0x4          /* PMP execute mask                                 */
#define PMP_OFF                 0x00         /* PMP OFF mask                                     */
#define PMP_TOR                 0x08         /* PMP TOR mask                                     */
#define PMP_NA4                 0x10         /* PMP NA4 mask                                     */
#define PMP_NAPOT               0x18         /* PMP NAPOT mask                                   */
#define PMP_LOCKED              0x80         /* PMP locked mask                                  */

#endif /* _SPECIALREG_H_ */
