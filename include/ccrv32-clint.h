/* ----------------------------------------------------------------------
*
* Copyright (c) 2019 ChipCraft Sp. z o.o. All rights reserved
*
* $Date: 2022-01-19 09:38:48 +0100 (śro, 19 sty 2022) $
* $Revision: 814 $
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
 * @file            ccrv32-clint.h
 * @brief           CCRV32 Processor Core Peripheral Access Layer Header File.
 * @author          Krzysztof Marcinek
 *
 * @addtogroup      CCRV32
 * CC Processor core definitions
 * @{
 */

#ifndef __CCRV32_CLINT_H
#define __CCRV32_CLINT_H

#include <stdint.h>

/************************//**
 * @defgroup clint Core Local Interruptor Controller
 * Core Local Interruptor Controller registers and definitions
 * @{
 *//************************/

/** CLINT Registers */
typedef struct
{
    uint32_t MSIP[32];         /*!< Machine Software Interrupt Pending     */
    uint64_t MTIMECMP[32];     /*!< Machine Time Compare Register          */
    uint32_t MTIME_LO;         /*!< Machine Time Register Lo               */
    uint32_t MTIME_HI;         /*!< Machine Time Register Hi               */
    uint32_t MTIMECFG;         /*!< Machine Time Configuration Register    */
} clint_regs_t;

static volatile clint_regs_t * const CLINT_PTR = (clint_regs_t*)CLINT_BASE;

/** Machine Timer Control Register flags */
enum
{
    MTIMECFG_EN       = 0x0001,     /*!< Machine Timer Enable      */
};

/** Machine Timer Control Register bit offsets */
enum
{
    MTIMECFG_SRC_SHIFT   = 4,       /*!< Machine Timer Source Shift    */
    MTIMECFG_BITS_SHIFT  = 16,      /*!< Machine Timer Bits Shift      */
};

/** Machine Timer Control Register masks */
enum
{
    MTIMECFG_SRC_MASK   = 0xF << MTIMECFG_SRC_SHIFT,     /*!< Machine Timer Source Mask     */
    MTIMECFG_BITS_MASK  = 0x7F << MTIMECFG_BITS_SHIFT,   /*!< Machine Timer Bits Mask       */
};

/** Machine Timer Sources */
enum
{
    MTIMECFG_SRC_CORE   = 0x0,  /*!< Processor Clock Source       */
    MTIMECFG_SRC_L1E1   = 0x1,  /*!< GNSS L1/E1 Source            */
    MTIMECFG_SRC_L5E5a  = 0x2,  /*!< GNSS L5/E5a Source           */
    MTIMECFG_SRC_E5b    = 0x3,  /*!< GNSS E5b Source              */
    MTIMECFG_SRC_L2     = 0x4,  /*!< GNSS L2 Source               */
    MTIMECFG_SRC_E6     = 0x5,  /*!< GNSS E6 Source               */
    MTIMECFG_SRC_AUX0   = 0xA,  /*!< GNSS AUX0 Source             */
    MTIMECFG_SRC_AUX1   = 0xB,  /*!< GNSS AUX1 Source             */
    MTIMECFG_SRC_VIRT   = 0xF,  /*!< GNSS Virtual Frontend Source */
};

/**
 * @name Machine Timer Control Register macros
 * @{
 */
#define PERFCNT_STATUS_GET_BITS(status)   ((status & PERFCNTT_STAT_BITS_MASK) >> MTIMECFG_BITS_SHIFT)  /*!< Gets Machine Timer Bits     */
#define PERFCNT_STATUS_GET_SOURCE(status) ((status & PERFCNTT_STAT_SRC_MASK) >> MTIMECFG_SRC_SHIFT)    /*!< Gets Machine Timer Source   */
/** @} */

/** @} */

#endif /* __CCRV32_CLINT_H */
/** @} */
