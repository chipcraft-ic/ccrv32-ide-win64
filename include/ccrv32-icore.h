/* ----------------------------------------------------------------------
*
* Copyright (c) 2021 ChipCraft Sp. z o.o. All rights reserved
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
 * @file            ccrv32-icore.h
 * @brief           CCRV32 Processor Core Peripheral Access Layer Header File.
 * @author          Krzysztof Marcinek
 *
 * @addtogroup      CCRV32
 * CC Processor core definitions
 * @{
 */

#ifndef __CCPROC_ICORE_H
#define __CCPROC_ICORE_H

#include <stdint.h>

/************************//**
 * @defgroup icore Intercore Interrupt Controller
 * Intercore Interrupt Controller registers and definitions
 * @{
 *//************************/

/** ICORE Registers */
typedef struct
{
    uint32_t ICORE_IRQ_MAP;     /*!< Inter-Core IRQ Mapping                 */
    uint32_t ICORE_IRQ_TRIG;    /*!< Inter-Core IRQ Trigger                 */
    uint32_t ICORE_IRQ_FLAG;    /*!< Inter-Core IRQ Flags                   */
} icore_regs_t;

static volatile icore_regs_t * const ICORE_PTR = (icore_regs_t*)ICORE_BASE;

/** @} */

#endif /* __CCPROC_ICORE_H */
/** @} */
