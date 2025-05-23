/* ----------------------------------------------------------------------
*
* Copyright (c) 2025 ChipCraft Sp. z o.o. All rights reserved
*
* $Date: 2025-05-16 14:07:05 +0200 (Fri, 16 May 2025) $
* $Revision: 1152 $
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
 * @file            ccrv32-amba-apb3.h
 * @brief           CCRV32 Processor AMBA APB3 definitions
 * @author          Krzysztof Marcinek
 *
 * @defgroup        CCAMBA AMBA APB3 Peripherials
 * AMBA Bus Peripherials
 * @{
 */

#ifndef __CCRV32_AMBA_APB3_H
#define __CCRV32_AMBA_APB3_H

#include <stdint.h>
#include "ccrv32.h"

/** 
 * @cond
 * enable/disable static assertions
 */
#if 0
 #define STATIC_ASSERT(expr, msg) _Static_assert(expr, msg)
#else
 #define STATIC_ASSERT(expr, msg)
#endif

/** @endcond */

/** 
 * @cond
 * enable/disable ccrv32 sdk integration
 */
#if 1
 #define CCRV32_SDK
#endif

/** @endcond */

/** AMBA APB3 addresses */
enum
{
    AMBA_APB3_BASE       = AMBA_BASE      + 0x03000000,     /*!< APB 3 Bridge base address                  */
    AMBA_APB3_CLUSTER    = AMBA_APB3_BASE + 0x00000000,     /*!< APB 3 GNSS channels base address           */
    AMBA_APB3_CLINT      = AMBA_APB3_BASE + 0x0001C000,     /*!< APB 3 CLINT base address                   */
    AMBA_APB3_GNSS       = AMBA_APB3_BASE + 0x00020000,     /*!< APB 3 GNSS controller base address         */
    AMBA_APB3_VITERBI    = AMBA_APB3_BASE + 0x00100000,     /*!< APB 3 Viterbi Decodeer base address        */
    AMBA_APB3_ACQENG0    = AMBA_APB3_BASE + 0x00200000,     /*!< APB 3 ACQENG0 base address                 */
    AMBA_APB3_ACQENG1    = AMBA_APB3_BASE + 0x00300000,     /*!< APB 3 ACQENG1 base address                 */
};

/** cluster registers */
typedef struct
{
    uint32_t INFO;          /*!< Information Register           */
    uint32_t CMD;           /*!< Commnad Register               */
    uint32_t OP1;           /*!< op1 Register                   */
    uint32_t OP2;           /*!< op2 Register                   */
    uint32_t RDATA;         /*!< Read Data Register             */
    uint32_t MBIST;         /*!< Mbist Register                 */
} gnss_cluster_t;

static volatile gnss_cluster_t * const GNSS_CLUSTER_PTR = (gnss_cluster_t*)AMBA_APB3_CLUSTER; /*!< GNSS Cluster pointer */


/** GNSS Controller Status Register Flags */
enum
{
    CLUSTER_INFO_MBIST  = 1 << 16, /*!< GNSS Cluster Mbist  */
};

/** GNSS Cluster Info Register bit offsets */
enum
{
    CLUSTER_INFO_BANKS_SHIFT        = 0, /*!< GNSS Cluster Banks Shift          */
    CLUSTER_INFO_CHANNELS_SHIFT     = 8, /*!< GNSS Cluster Channels Shift       */
};

/** GNSS Cluster Info Register masks */
enum
{
    CLUSTER_INFO_BANKS_MASK     = 0xFF << CLUSTER_INFO_BANKS_SHIFT,     /*!< GNSS Cluster Banks Mask     */
    CLUSTER_INFO_CHANNELS_MASK  = 0xFF << CLUSTER_INFO_CHANNELS_SHIFT,  /*!< GNSS Cluster Channels Mask  */
};

#endif /* __CCRV32_AMBA_APB3_H */
/** @} */

