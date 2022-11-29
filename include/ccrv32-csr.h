/* ----------------------------------------------------------------------
*
* Copyright (c) 2019 ChipCraft Sp. z o.o. All rights reserved
*
* $Date: 2022-11-24 15:32:16 +0100 (czw, 24 lis 2022) $
* $Revision: 916 $
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

/*
 * Originally the CSR register offsets were calculated similarly to:
 * typedef struct
 * {
 *     uint32_t _reserved[1995];
 *     uint32_t DBG_BAUD;
 *     uint32_t MEM_REMAP;
 * } csr_regs_t;
 *
 * Thus:
 * offset of DBG_BAUD = 1996 * sizeof( uint32_t )
 * offset of MEM_REMAP = 1997 * sizeof( uint32_t )
 */

#define CSR_CTRL_BASE ( CSR_BASE )
#define CSR_DBG_BAUD_OFFSET ( 1996 * sizeof( uint32_t ) )
#define CSR_MEM_REMAP_OFFSET ( 1997 * sizeof( uint32_t ) )

/**
 * @name Custom CSR Registers
 * @{
 */
#define mconfig0    0x07C0   /* Processor Resources Register 0                   */
#define mconfig1    0x07C1   /* Processor Resources Register 1                   */
#define mconfig2    0x07C2   /* Processor Resources Register 2                   */
#define mcontrol    0x07C8   /* Processor Control Register                       */
#define mstackmin   0x07C9   /* Stack Pointer Protection Min. Register           */
#define mstackmax   0x07CA   /* Stack Pointer Protection Max. Register           */
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
    CPU_IMSIZE_SHIFT  = 0,   /*!< Instruction Memory Size Offset */
    CPU_DMSIZE_SHIFT  = 5,   /*!< Data Memory Size Offset        */
    CPU_ICSIZE_SHIFT  = 11,  /*!< Instruction Cache Size Offset  */
    CPU_SPRSIZE_SHIFT = 18,  /*!< Scratch-Pad RAM Size Offset    */
    CPU_ICWAY_SHIFT   = 29,  /*!< Instruction Cache Ways Offset  */
};

/** CSR Config 0 Register masks */
enum
{
    CPU_IMSIZE_MASK  = 0x1F << CPU_IMSIZE_SHIFT,   /*!< Instruction Memory Size Mask */
    CPU_DMSIZE_MASK  = 0x1F << CPU_DMSIZE_SHIFT,   /*!< Data Memory Size Mask        */
    CPU_ICSIZE_MASK  = 0x1F << CPU_ICSIZE_SHIFT,   /*!< Instruction Cache Size Mask  */
    CPU_SPRSIZE_MASK = 0x1F << CPU_SPRSIZE_SHIFT,  /*!< Scratch-Pad RAM Size Mask    */
    CPU_ICWAY_MASK   = 0x07 << CPU_ICWAY_SHIFT,    /*!< Instruction Cache Ways Mask  */
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
#define CPU_INFO_GET_DCSIZE_LOG(cpu_info1)   ((cpu_info1 & CPU_DCSIZE_MASK)      >> CPU_DCSIZE_SHIFT)        /*!< Data Cache Size Logarithm  */
#define CPU_INFO_GET_DCWAY(cpu_info1)        ((cpu_info1 & CPU_DCWAY_MASK)       >> CPU_DCWAY_SHIFT)         /*!< Data Cache Ways            */
#define CPU_INFO_GET_DCSIZE(cpu_info1)       (1 << CPU_INFO_GET_DCSIZE_LOG(cpu_info1))                       /*!< Data Cache Size            */
#define CPU_INFO_GET_FPU(cpu_info1)          ((cpu_info1 & CPU_FPU_MASK)         >> CPU_FPU_SHIFT)           /*!< FPU Number                 */
#define CPU_INFO_GET_GNSS_ISE_NUM(cpu_info1) ((cpu_info1 & CPU_GNSS_ISE_MASK)    >> CPU_GNSS_ISE_SHIFT)      /*!< GNSS Channels Number       */
/** @} */

/** CSR Config 2 Register bits */
enum
{
    CPU_VITERBI = 1 << 8,  /*!< Viterbi Decoder             */
    CPU_XPRAM   = 1 << 9,  /*!< Executable Scratch-Pad RAM  */
};

/** CSR Config 2 Register bit offsets */
enum
{
    CPU_ARCH_SHIFT        = 0,    /*!< CPU Architecture Offset   */
    CPU_GNSS_BANKS_SHIFT  = 4,    /*!< GNSS Banks Number Offset  */
};

/** CSR Config 2 Register bit masks */
enum
{
    CPU_ARCH_MASK         = 0x7 << CPU_ARCH_SHIFT,        /*!< GNSS Architecture Mask    */
    CPU_GNSS_BANKS_MASK   = 0xF << CPU_GNSS_BANKS_SHIFT,  /*!< GNSS Banks Number Offset  */
};

/** CSR Config 2 Register bits */
enum
{
    CPU_DCLS = 1 << 3,  /*!< Dual-core Lockstep Controller       */
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
