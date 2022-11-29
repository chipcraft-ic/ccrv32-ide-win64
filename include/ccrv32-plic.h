/* ----------------------------------------------------------------------
*
* Copyright (c) 2019 ChipCraft Sp. z o.o. All rights reserved
*
* $Date: 2019-10-03 19:22:07 +0200 (czw, 03 paź 2019) $
* $Revision: 474 $
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
 * @file            ccrv32-plic.h
 * @brief           CCRV32 Processor Core Peripheral Access Layer Header File.
 * @author          Krzysztof Marcinek
 *
 * @addtogroup      CCRV32
 * CC Processor core definitions
 * @{
 */

#ifndef __CCPROC_PLIC_H
#define __CCPROC_PLIC_H

#include <stdint.h>

/************************//**
 * @defgroup plic Platform Level Interrupt Controller
 * Platform Level Interrupt Controller registers and definitions
 * @{
 *//************************/

/** PLIC Registers */
typedef struct
{
    uint32_t PRIOR[16];         /*!< Interrupt Priority                     */
    uint32_t _reserved[16];
    uint32_t PENDING;           /*!< Pending Register                       */
    uint32_t ENABLE;            /*!< Enable Register                        */
    uint32_t THRESHOLD;         /*!< Threshold Register                     */
    uint32_t CLAIM;             /*!< Claim Register                         */
} plic_regs_t;

static volatile plic_regs_t * const PLIC_PTR = (plic_regs_t*)PLIC_BASE;

/** @} */

#endif /* __CCPROC_PLIC_H */
/** @} */
