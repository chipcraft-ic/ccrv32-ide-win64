/* ----------------------------------------------------------------------
*
* Copyright (c) 2024 ChipCraft Sp. z o.o. All rights reserved
*
* $Date: 2024-09-04 12:09:10 +0200 (śro, 04 wrz 2024) $
* $Revision: 1104 $
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
 * @file            ccrv32-amba-mbist.h
 * @brief           CCRV32 Processor AMBA Main Memory MBIST Controller definitions
 * @author          Krzysztof Marcinek
 *
 * @addtogroup      CCAMBA
 * AMBA Bus Peripherials
 * @{
 */

#ifndef __CCRV32_AMBA_MBIST_H
#define __CCRV32_AMBA_MBIST_H

#include <stdint.h>

#ifdef CCRV32_SDK
 #include "ccrv32.h"
#endif

/************************//**
 * @defgroup edac Main Memory MBIST Controller
 * Main Memory MBIST Controller registers and definitions
 * @{
 *//************************/

/** Main Memory MBIST Controller Registers */
typedef struct
{
    uint32_t STATUS;            /*!< Status Register                    */
    uint32_t CTRL;              /*!< Control Register                   */
    uint32_t ALGO;              /*!< Algorithm Register                 */
    uint32_t DMASK[5];          /*!< Data Mask Registers                */
    uint32_t PATTERN;           /*!< Pattern Register                   */
    uint32_t START_ADDR;        /*!< Start Address Register             */
    uint32_t END_ADDR;          /*!< End Address Register               */
    uint32_t ERRCNT_PORT;       /*!< Error Count Register               */
    uint32_t _reserved0;
    uint32_t FIRST_ERR_ADDR[4]; /*!< First Error Address Register       */
    uint32_t LAST_ERR_ADDR[4];  /*!< Last Error Address Register        */
    uint32_t _reserved1[8];
    uint32_t FIRST_ERR_IDX[2];  /*!< First Error Index Register         */
    uint32_t LAST_ERR_IDX[2];   /*!< Last Error Index Register          */
    uint32_t _reserved2[4];
    uint32_t INJ_ADDR;          /*!< Error Injection Address Register   */
    uint32_t INJ_CTRL;          /*!< Error Injection Control Register   */
} amba_mbist_t;

/** AMBA MBIST Controller Status Register bits */
enum
{
    AMBA_MBIST_STAT_EN      = 1 << 0,  /*!< MBIST enable bit    */
    AMBA_MBIST_STAT_BUSY    = 1 << 1,  /*!< MBIST busy bit      */
    AMBA_MBIST_STAT_ERR     = 1 << 2,  /*!< MBIST error bit     */
};

/** AMBA MBIST Controller Control Register bits */
enum
{
    AMBA_MBIST_CTRL_EN      = 1 << 0,  /*!< MBIST enable bit    */
    AMBA_MBIST_CTRL_START   = 1 << 1,  /*!< MBIST start bit     */
};

/** AMBA MBIST Controller Algorithms */
enum
{
    AMBA_MBIST_ALG_ZERO_ONE = 0x01, /*!< Zero-one algorithm   */
    AMBA_MBIST_ALG_MATS_P   = 0x04, /*!< MATS+ algorithm      */
    AMBA_MBIST_ALG_MARCH_CM = 0x09, /*!< March C- algorithm   */
};

/** AMBA MBIST Controller Fault Injection Register bits */
enum
{
    AMBA_MBIST_ERR_IDX0_SGL  = 0x00000100, /*!< Single error indicator   */
    AMBA_MBIST_ERR_IDX0_MPL  = 0x00000200, /*!< Multiple error indicator */
    AMBA_MBIST_ERR_IDX1_SGL  = 0x01000000, /*!< Single error indicator   */
    AMBA_MBIST_ERR_IDX1_MPL  = 0x02000000, /*!< Multiple error indicator */
};

/** @} */

#endif /* __CCRV32_AMBA_MBIST_H */
/** @} */
