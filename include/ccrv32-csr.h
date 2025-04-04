/* ----------------------------------------------------------------------
*
* Copyright (c) 2019 ChipCraft Sp. z o.o. All rights reserved
*
* $Date: 2024-10-15 20:09:44 +0200 (wto, 15 paź 2024) $
* $Revision: 1112 $
*
*  ----------------------------------------------------------------------
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ChipCraft Sp. z o.o. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------- */

/**
 * @file            ccrv32-csr.h
 * @brief           CCRV32 Processor Core Peripheral Access Layer Header File.
 * @author          Krzysztof Marcinek
 *
 * @addtogroup      CCRV32
 * CC Processor core definitions
 * @{
 */

#ifndef __CCRV32_CSR_H
#define __CCRV32_CSR_H

#include <stdint.h>

/************************//**
 * @defgroup csr CSR
 * CSR registers and definitions
 * @{
 *//************************/

/**
 * @name Custom CSR Registers
 * @{
 */

#define mconfig0        0x07C0   /* Processor Resources Register 0                      */
#define mconfig1        0x07C1   /* Processor Resources Register 1                      */
#define mconfig2        0x07C2   /* Processor Resources Register 2                      */
#define mcontrol        0x07C8   /* Processor Control Register                          */
#define mstackmin       0x07C9   /* Stack Pointer Protection Min. Register              */
#define mstackmax       0x07CA   /* Stack Pointer Protection Max. Register              */
#define mdbgbaud        0x07CB   /* On-chip Debugger Baud Rate Register                 */
#define mremap          0x07CC   /* Memory remap                                        */
#define mromunlock      0x07CD   /* ROM unlock                                          */
#define mfterror        0x07CE   /* Error Statistics (FT-only)                          */

#define mbusperf0       0x0810   /* Main Bus Perf. Counter Register (counter  window)   */
#define mbusperf1       0x0811   /* Main Bus Perf. Counter Register (icache   util.)    */
#define mbusperf2       0x0812   /* Main Bus Perf. Counter Register (master0  util.)    */
#define mbusperf3       0x0813   /* Main Bus Perf. Counter Register (master1  util.)    */
#define mbusperf4       0x0814   /* Main Bus Perf. Counter Register (master2  util.)    */
#define mbusperf5       0x0815   /* Main Bus Perf. Counter Register (master3  util.)    */
#define mbusperf6       0x0816   /* Main Bus Perf. Counter Register (master4  util.)    */
#define mbusperf7       0x0817   /* Main Bus Perf. Counter Register (master5  util.)    */
#define mbusperf8       0x0818   /* Main Bus Perf. Counter Register (master6  util.)    */
#define mbusperf9       0x0819   /* Main Bus Perf. Counter Register (master7  util.)    */
#define mbusperf10      0x081A   /* Main Bus Perf. Counter Register (master8  util.)    */
#define mbusperf11      0x081B   /* Main Bus Perf. Counter Register (master9  util.)    */
#define mbusperf12      0x081C   /* Main Bus Perf. Counter Register (master10 util.)    */
#define mbusperf13      0x081D   /* Main Bus Perf. Counter Register (master11 util.)    */
#define mbusperf14      0x081E   /* Main Bus Perf. Counter Register (master12 util.)    */
#define mbusperf15      0x081F   /* Main Bus Perf. Counter Register (master13 util.)    */
#define mbusperf16      0x0820   /* Main Bus Perf. Counter Register (master14 util.)    */
#define mbusperf17      0x0821   /* Main Bus Perf. Counter Register (master15 util.)    */
#define mbusperf18      0x0822   /* Main Bus Perf. Counter Register (master17 util.)    */
#define mbusperf19      0x0823   /* Main Bus Perf. Counter Register (master17 util.)    */
#define mbusperf20      0x0824   /* Main Bus Perf. Counter Register (master18 util.)    */
#define mbusperf21      0x0825   /* Main Bus Perf. Counter Register (master19 util.)    */
#define mbusperf22      0x0826   /* Main Bus Perf. Counter Register (master20 util.)    */
#define mbusperf23      0x0827   /* Main Bus Perf. Counter Register (master21 util.)    */
#define mbusperf24      0x0828   /* Main Bus Perf. Counter Register (master22 util.)    */
#define mbusperf25      0x0829   /* Main Bus Perf. Counter Register (master23 util.)    */
#define mbusperf26      0x082A   /* Main Bus Perf. Counter Register (master24 util.)    */
#define mbusperf27      0x082B   /* Main Bus Perf. Counter Register (master25 util.)    */
#define mbusperf28      0x082C   /* Main Bus Perf. Counter Register (master26 util.)    */
#define mbusperf29      0x082D   /* Main Bus Perf. Counter Register (master27 util.)    */
#define mbusperf30      0x082E   /* Main Bus Perf. Counter Register (master28 util.)    */
#define mbusperf31      0x082F   /* Main Bus Perf. Counter Register (master29 util.)    */

#define iopmpcfg0       0x08A0   /* ioPMP Configuration Register 0                      */
#define iopmpcfg1       0x08A1   /* ioPMP Configuration Register 1                      */
#define iopmpcfg2       0x08A2   /* ioPMP Configuration Register 2                      */
#define iopmpcfg3       0x08A3   /* ioPMP Configuration Register 3                      */

#define iopmpaddr0      0x08B0   /* ioPMP Configuration Register 0                      */
#define iopmpaddr1      0x08B1   /* ioPMP Configuration Register 1                      */
#define iopmpaddr2      0x08B2   /* ioPMP Configuration Register 2                      */
#define iopmpaddr3      0x08B3   /* ioPMP Configuration Register 3                      */
#define iopmpaddr4      0x08B4   /* ioPMP Configuration Register 4                      */
#define iopmpaddr5      0x08B5   /* ioPMP Configuration Register 5                      */
#define iopmpaddr6      0x08B6   /* ioPMP Configuration Register 6                      */
#define iopmpaddr7      0x08B7   /* ioPMP Configuration Register 7                      */
#define iopmpaddr8      0x08B8   /* ioPMP Configuration Register 8                      */
#define iopmpaddr9      0x08B9   /* ioPMP Configuration Register 9                      */
#define iopmpaddr10     0x08BA   /* ioPMP Configuration Register 10                     */
#define iopmpaddr11     0x08BB   /* ioPMP Configuration Register 11                     */
#define iopmpaddr12     0x08BC   /* ioPMP Configuration Register 12                     */
#define iopmpaddr13     0x08BD   /* ioPMP Configuration Register 13                     */
#define iopmpaddr14     0x08BE   /* ioPMP Configuration Register 14                     */
#define iopmpaddr15     0x08BF   /* ioPMP Configuration Register 15                     */

/** @} */

/** CSR MSTATUS Register bits */
enum
{
    MSTAT_UIE  = 1 << 0,    /*!< User Interrupt Enable                 */
    MSTAT_SIE  = 1 << 1,    /*!< Supervisor Interrupt Enable           */
    MSTAT_MIE  = 1 << 3,    /*!< Machine Interrupt Enable              */
    MSTAT_UPIE = 1 << 4,    /*!< Previous User Interrupt Enable        */
    MSTAT_SPIE = 1 << 5,    /*!< Previous Supervisor Interrupt Enable  */
    MSTAT_MPIE = 1 << 7,    /*!< Previous Machine Interrupt Enable     */
};

/** CSR MIE Register bits */
enum
{
    MIE_USIE = 1 << 0,      /*!< User Software Interrupt Enable        */
    MIE_SSIE = 1 << 1,      /*!< Supervisor Software Interrupt Enable  */
    MIE_MSIE = 1 << 3,      /*!< Machine Software Interrupt Enable     */
    MIE_UTIE = 1 << 4,      /*!< User Timer Interrupt Enable           */
    MIE_STIE = 1 << 5,      /*!< Supervisor Timer Interrupt Enable     */
    MIE_MTIE = 1 << 7,      /*!< Machine Timer Interrupt Enable        */
    MIE_UEIE = 1 << 8,      /*!< User External Interrupt Enable        */
    MIE_SEIE = 1 << 9,      /*!< Supervisor External Interrupt Enable  */
    MIE_MEIE = 1 << 11,     /*!< Machine External Interrupt Enable     */
};

/** CSR Config 0 Register bit offsets */
enum
{
    CPU_IMSIZE_SHIFT  = 0,   /*!< Instruction Memory Bus Address Size Offset    */
    CPU_DMSIZE_SHIFT  = 5,   /*!< Data Memory Bus Address Size Offset           */
    CPU_ICSIZE_SHIFT  = 11,  /*!< Instruction Cache Size Offset                 */
    CPU_SPRSIZE_SHIFT = 18,  /*!< Scratch-Pad RAM Size Offset                   */
    CPU_ICWAY_SHIFT   = 29,  /*!< Instruction Cache Ways Offset                 */
};

/** CSR Config 0 Register masks */
enum
{
    CPU_IMSIZE_MASK  = 0x1F << CPU_IMSIZE_SHIFT,   /*!< Instruction Memory Bus Address Size Mask    */
    CPU_DMSIZE_MASK  = 0x1F << CPU_DMSIZE_SHIFT,   /*!< Data Memory Bus Address Size Mask           */
    CPU_ICSIZE_MASK  = 0x1F << CPU_ICSIZE_SHIFT,   /*!< Instruction Cache Size Mask                 */
    CPU_SPRSIZE_MASK = 0x1F << CPU_SPRSIZE_SHIFT,  /*!< Scratch-Pad RAM Size Mask                   */
    CPU_ICWAY_MASK   = 0x07 << CPU_ICWAY_SHIFT,    /*!< Instruction Cache Ways Mask                 */
};

/** CSR Config 0 Register bits */
enum
{
    CPU_ICACHE = 1 << 10,  /*!< Instruction Cache       */
    CPU_MCORE  = 1 << 16,  /*!< Multicore Controller    */
    CPU_SPRAM  = 1 << 17,  /*!< Scratch-Pad RAM         */
    CPU_PWD    = 1 << 23,  /*!< Power Management        */
    CPU_IRQ    = 1 << 24,  /*!< Interrupt Controller    */
    CPU_USER   = 1 << 25,  /*!< User Mode               */
    CPU_MPU    = 1 << 26,  /*!< Memory Protection Unit  */
    CPU_FFT    = 1 << 27,  /*!< FFT Unit                */
    CPU_SPROT  = 1 << 28,  /*!< Stack Protection        */
};

/**
 * @name CSR Config 0 Register helper macros
 * @{
 */
#define CPU_INFO_GET_IMSIZE_LOG(cpu_info0)   ((cpu_info0 & CPU_IMSIZE_MASK)  >> CPU_IMSIZE_SHIFT)   /*!< Instruction Memory Size Logarithm  */
#define CPU_INFO_GET_DMSIZE_LOG(cpu_info0)   ((cpu_info0 & CPU_DMSIZE_MASK)  >> CPU_DMSIZE_SHIFT)   /*!< Data Memory Size Logarithm         */
#define CPU_INFO_GET_ICSIZE_LOG(cpu_info0)   ((cpu_info0 & CPU_ICSIZE_MASK)  >> CPU_ICSIZE_SHIFT)   /*!< Instruction Cache Size Logarithm   */
#define CPU_INFO_GET_SPRSIZE_LOG(cpu_info0)  ((cpu_info0 & CPU_SPRSIZE_MASK) >> CPU_SPRSIZE_SHIFT)  /*!< Scratch-Pad RAM Size Logarithm     */
#define CPU_INFO_GET_ICWAY(cpu_info0)        ((cpu_info0 & CPU_ICWAY_MASK)   >> CPU_ICWAY_SHIFT)    /*!< Instruction Cache Ways             */
#define CPU_INFO_GET_IMSIZE(cpu_info0)       (1 << CPU_INFO_GET_IMSIZE_LOG(cpu_info0))              /*!< Instruction Memory Size            */
#define CPU_INFO_GET_DMSIZE(cpu_info0)       (1 << CPU_INFO_GET_DMSIZE_LOG(cpu_info0))              /*!< Data Memory Size                   */
#define CPU_INFO_GET_ICSIZE(cpu_info0)       (1 << CPU_INFO_GET_ICSIZE_LOG(cpu_info0))              /*!< Instruction Cache Size             */
#define CPU_INFO_GET_SPRSIZE(cpu_info0)      (1 << CPU_INFO_GET_SPRSIZE_LOG(cpu_info0))             /*!< Scratch-Pad RAM Size               */
/** @} */

/** CSR Config 1 Register bit offsets */
enum
{
    CPU_DCSIZE_SHIFT      = 1,    /*!< Data Cache Size Offset   */
    CPU_DCWAY_SHIFT       = 6,    /*!< Data Cache Ways Offset   */
    CPU_FPU_SHIFT         = 12,   /*!< FPU Offset               */
    CPU_MUL_SHIFT         = 17,   /*!< Multiplier Offset        */
    CPU_BPRED_SHIFT       = 21,   /*!< Branch Prediction Offset */
    CPU_GNSS_ISE_SHIFT    = 26,   /*!< GNSS Channels Offset     */
};

/** CSR Config 1 Register masks */
enum
{
    CPU_DCSIZE_MASK      = 0x1F << CPU_DCSIZE_SHIFT,   /*!< Data Cache Size Mask      */
    CPU_DCWAY_MASK       = 0x07 << CPU_DCWAY_SHIFT,    /*!< Data Cache Ways Mask      */
    CPU_FPU_MASK         = 0x0F << CPU_FPU_SHIFT,      /*!< FPU Mask                  */
    CPU_MUL_MASK         = 0x03 << CPU_MUL_SHIFT,      /*!< Multiplier Mask           */
    CPU_BPRED_MASK       = 0x07 << CPU_BPRED_SHIFT,    /*!< Branch Prediction Mask    */
    CPU_GNSS_ISE_MASK    = 0x3F << CPU_GNSS_ISE_SHIFT, /*!< GNSS Channels Number Mask */
};

/** CSR Config 1 Register bits */
enum
{
    CPU_DCACHE     = 1 << 0,  /*!< Data Cache                    */
    CPU_MBIST      = 1 << 10, /*!< Memory BIST Controller        */
    CPU_PERFCNT    = 1 << 11, /*!< Performance Counter           */
    CPU_AROPT      = 1 << 16, /*!< Area Optimization             */
    CPU_TECH       = 1 << 19, /*!< Target Technology             */
    CPU_MULFAST    = 1 << 20, /*!< Fast Multiplication           */
    CPU_COMPR      = 1 << 24, /*!< Compressed ISA Mode Support   */
    CPU_GNSS       = 1 << 25, /*!< GNSS Controller               */
};

/**
 * @name CSR Config 1 Register helper macros
 * @{
 */
#define CPU_INFO_GET_DCSIZE_LOG(cpu_info1)      ((cpu_info1 & CPU_DCSIZE_MASK)      >> CPU_DCSIZE_SHIFT)        /*!< Data Cache Size Logarithm  */
#define CPU_INFO_GET_DCWAY(cpu_info1)           ((cpu_info1 & CPU_DCWAY_MASK)       >> CPU_DCWAY_SHIFT)         /*!< Data Cache Ways            */
#define CPU_INFO_GET_DCSIZE(cpu_info1)          (1 << CPU_INFO_GET_DCSIZE_LOG(cpu_info1))                       /*!< Data Cache Size            */
#define CPU_INFO_GET_FPU(cpu_info1)             ((cpu_info1 & CPU_FPU_MASK)         >> CPU_FPU_SHIFT)           /*!< FPU Number                 */
#define CPU_INFO_GET_GNSS_ISE_NUM(cpu_info1)    ((cpu_info1 & CPU_GNSS_ISE_MASK)    >> CPU_GNSS_ISE_SHIFT)      /*!< GNSS Channels Number       */
/** @} */

/** CSR Config 2 Register bits */
enum
{
    CPU_DCLS    = 1 << 3,  /*!< Dual-core Lockstep Controller   */
    CPU_VITERBI = 1 << 8,  /*!< Viterbi Decoder                 */
    CPU_XPRAM   = 1 << 9,  /*!< Executable Scratch-Pad RAM      */
};

/** CSR Config 2 Register bit offsets */
enum
{
    CPU_ARCH_SHIFT        = 0,    /*!< CPU Architecture Offset   */
    CPU_GNSS_BANKS_SHIFT  = 4,    /*!< GNSS Banks Number Offset  */
    CPU_SYSBUS_SHIFT      = 10,   /*!< System Bus Width Offset   */
    CPU_BTB_SHIFT         = 12,   /*!< BTB Size Offset           */
    CPU_PHT_SHIFT         = 16,   /*!< PHT Size Offset           */
    CPU_DCTRL_SHIFT       = 26,   /*!< Main Bus Masters Offset   */
};

/** CSR Config 2 Register bit masks */
enum
{
    CPU_ARCH_MASK         = 0x7  << CPU_ARCH_SHIFT,       /*!< GNSS Architecture Mask    */
    CPU_GNSS_BANKS_MASK   = 0xF  << CPU_GNSS_BANKS_SHIFT, /*!< GNSS Banks Number Mask    */
    CPU_SYSBUS_MASK       = 0x3  << CPU_SYSBUS_SHIFT,     /*!< System Bus Width Mask     */
    CPU_BTB_MASK          = 0x7  << CPU_BTB_SHIFT,        /*!< BTB Size Mask             */
    CPU_PHT_MASK          = 0x7  << CPU_PHT_SHIFT,        /*!< PHT Size Mask             */
    CPU_DCTRL_MASK        = 0x3F << CPU_DCTRL_SHIFT       /*!< Main Bus Masters Mask     */
};

/**
 * @name CSR Config 2 Register helper macros
 * @{
 */
#define CPU_INFO_GET_GNSS_BANKS_NUM(cpu_info2)  ((cpu_info2 & CPU_GNSS_BANKS_MASK)    >> CPU_GNSS_BANKS_SHIFT)      /*!< GNSS Channels Number       */
/** @} */

/** Debug prescaler helper macro */
#define DBG_UART_PRES(mantisa, fraction) ((mantisa) | ((fraction) << 16))  /*!< Debug prescaler from mantisa and fraction */

#define CSR_REMAP_KEY        0x000000A5    /*!< Bootloader Remap Key      */
#define CSR_ROM_UNLOCK_KEY   0xA5000000    /*!< ROM Unlock Key            */

/** CSR Mcontrol Register bits */
enum
{
    MCTRL_SPROT_EN      = 1 << 3,   /*!< Stack Protection Enable                 */
    MCTRL_DCLS_EN       = 1 << 5,   /*!< Dual-core Lockstep Enabled              */
    MCTRL_BOOT_REMAP    = 1 << 7,   /*!< Bootloader Remap                        */
    MCTRL_PHT_DIS       = 1 << 8,   /*!< PHT Disable                             */
    MCTRL_NMI_DIS       = 1 << 9,   /*!< Non-Maskable Interrupts Disable         */
    MCTRL_BTB_DIS       = 1 << 10,  /*!< BTB Disable                             */
    MCTRL_GHR_DIS       = 1 << 11,  /*!< GHR Disable                             */
    MCTRL_BUS_PERF_EN   = 1 << 12,  /*!< Main Bus Performance Counters Enable    */
    MCTRL_BUS_PERF_UP   = 1 << 13,  /*!< Main Bus Performance Counters Update    */
    MCTRL_BUS_PERF_RD   = 1 << 14,  /*!< Main Bus Performance Counters Ready     */
    MCTRL_BUS_PERF_OV   = 1 << 15,  /*!< Main Bus Performance Counters Overflow  */
};

/** CSR Mcontrol Register bit offsets */
enum
{
    MCTRL_START_SHIFT   = 20,       /*!< Processor Boot Region Shift    */
    MCTRL_COREIDX_SHIFT = 24,       /*!< Processor Core Index Shift     */
};

    /** CSR Mcontrol Register bit masks */
enum
{
    MCTRL_START_MASK    = 0xF  << MCTRL_START_SHIFT,    /*!< Processor Boot Region Mask */
    MCTRL_COREIDX_MASK  = 0xFF << MCTRL_COREIDX_SHIFT,  /*!< Processor Core Index Mask  */
};        

/**
 * @name CSR helper macros
 * @{
 */

#define csr_read_part(reg) ({ uint32_t __tmp; \
    __asm__ __volatile__ ("csrr %0, " #reg : "=r"(__tmp)); \
    __tmp; })

#define csr_write_part(reg, val) ({ \
    if (__builtin_constant_p(val) && (unsigned long)(val) < 32) \
        __asm__ __volatile__ ("csrw " #reg ", %0" :: "i"(val)); \
    else \
        __asm__ __volatile__ ("csrw " #reg ", %0" :: "r"(val)); })

#define csr_read(reg)       csr_read_part(reg)
#define csr_write(reg, val) csr_write_part(reg, val)

/** @} */

/** @} */

#endif /* __CCRV32_CSR_H */
/** @} */

