/* ----------------------------------------------------------------------
*
* Copyright (c) 2019 ChipCraft Sp. z o.o. All rights reserved
*
* $Date: 2024-09-17 20:06:05 +0200 (wto, 17 wrz 2024) $
* $Revision: 1107 $
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
 * @file            ccrv32-icache.h
 * @brief           CCRV32 Processor Core Peripheral Access Layer Header File.
 * @author          Krzysztof Marcinek
 *
 * @addtogroup      CCRV32
 * CC Processor core definitions
 * @{
 */

#ifndef __CCRV32_ICACHE_H
#define __CCRV32_ICACHE_H

#include <stdint.h>

/************************//**
 * @defgroup icache Instruction Cache Controller
 * Instruction Cache Controller registers and definitions
 * @{
 *//************************/

/** Instruction Cache Controller Registers */
typedef struct
{
    uint32_t STCR;            /*!< Status and Control Register      */
    uint32_t FLUSH;           /*!< Flush Tag Memory                 */
    uint32_t INFO;            /*!< Cache Info Register              */
    uint32_t ERR_CNT_0;       /*!< Error Counter 0 (FT-only)        */
    uint32_t ERR_CNT_1;       /*!< Error Counter 1 (FT-only)        */
    uint32_t INJECT_MASK_LO;  /*!< Error Injection Mask (FT-only)   */
    uint32_t INJECT_MASK_HI;  /*!< Error Injection Mask (FT-only)   */
    uint32_t _reserved0[5];
    uint32_t HIT_CNT;         /*!< Hit Counter Register             */
    uint32_t MISS_CNT;        /*!< Miss Counter Register            */
    uint32_t HIT_CNT_BUF;     /*!< Hit Counter Buffer Register      */
    uint32_t MISS_CNT_BUF;    /*!< Miss Counter Buffer Register     */
} icache_regs_t;

static volatile icache_regs_t * const ICACHE_PTR = (icache_regs_t*)ICACHE_BASE;

/** ICC Status Register bits */
enum
{

    ICACHE_STCR_EN              = 1 << 0,  /*!< Instruction Cache Enable                 */
    ICACHE_STCR_PARITY_EN       = 1 << 3,  /*!< Parity Enable (FT-only)                  */
    ICACHE_STCR_ERR_TRIG        = 1 << 4,  /*!< Error Count Trigger (FT-only)            */
    ICACHE_STCR_SCRAMBLE_EN     = 1 << 5,  /*!< Scramble Enable (FT-only)                */
    ICACHE_STCR_MEM_ERR_INJECT  = 1 << 6,  /*!< Memory Error Injection Enable (FT-only)  */
    ICACHE_STCR_TAG_ERR_INJECT  = 1 << 7,  /*!< Tag Error Injection Enable (FT-only)     */
    ICACHE_STCR_HARD_ERR_EN     = 1 << 8,  /*!< Hard error enable (FT-only)              */
    ICACHE_STCR_HARD_ERR_FLAG   = 1 << 9,  /*!< Hard error flag (FT-only)                */

    ICACHE_STCR_PERF_EN         = 1 << 24, /*!< Enable Performance Counters              */
    ICACHE_STCR_PERF_UPDATE     = 1 << 25, /*!< Update Performance Counters              */
    ICACHE_STCR_PERF_READY      = 1 << 26, /*!< Performance Counters Ready               */
    ICACHE_STCR_PERF_OVF        = 1 << 27, /*!< Performance Counters Overflow            */

};

/** ICC Status Register bit offsets */
enum
{
    ICACHE_STCR_RETRY_SHIFT   = 12,  /*!< Fetch Retry Count Offset (FT-only)    */
};

/** ICC Status Register masks */
enum
{
    ICACHE_STCR_RETRY_MASK  = 0x0F << ICACHE_STCR_RETRY_SHIFT,   /*!< Fetch Retry Count Mask (FT-only)  */
};

/** ICC Info Register bit offsets */
enum
{
    ICACHE_ICWAY_SHIFT   = 0,  /*!< Instruction Cache Ways Offset           */
    ICACHE_ICSIZE_SHIFT  = 3,  /*!< Instruction Cache Size Offset           */
    ICACHE_ICLINE_SHIFT  = 8,  /*!< Instruction Cache Line Offset           */
    ICACHE_ICALG_SHIFT   = 13, /*!< Instruction Cache Algorithm Offset      */
    ICACHE_ICTAG_SHIFT   = 15, /*!< Instruction Cache Tag Offset            */
    ICACHE_IMPL_SHIFT    = 28, /*!< Instruction Cache Implementation Offset */
};

/** ICC Info Register masks */
enum
{
    ICACHE_ICWAY_MASK  = 0x07 << ICACHE_ICWAY_SHIFT,   /*!< Instruction Cache Ways Mask           */
    ICACHE_ICSIZE_MASK = 0x1F << ICACHE_ICSIZE_SHIFT,  /*!< Instruction Cache Size Mask           */
    ICACHE_ICLINE_MASK = 0x0F << ICACHE_ICLINE_SHIFT,  /*!< Instruction Cache Line Mask           */
    ICACHE_ICALG_MASK  = 0x03 << ICACHE_ICALG_SHIFT,   /*!< Instruction Cache Algorithm Mask      */
    ICACHE_ICTAG_MASK  = 0x7F << ICACHE_ICTAG_SHIFT,   /*!< Instruction Cache Tag Mask            */
    ICACHE_IMPL_MASK   = 0x0F << ICACHE_IMPL_SHIFT,    /*!< Instruction Cache Implementation Mask */
};

/** ICC Implementation */
enum
{
    ICACHE_IMPL_HP = 0x00, /*!< High-performance instruction cache implementation */
    ICACHE_IMPL_FT = 0x03, /*!< Fault-tolerant instruction cache implementation   */
};

/** @} */

#endif /* __CCRV32_ICACHE_H */
/** @} */
