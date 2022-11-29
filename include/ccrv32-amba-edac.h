/* ----------------------------------------------------------------------
*
* Copyright (c) 2021 ChipCraft Sp. z o.o. All rights reserved
*
* $Date: 2022-08-25 14:11:28 +0200 (czw, 25 sie 2022) $
* $Revision: 885 $
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
 * @file            ccrv32-amba-edac.h
 * @brief           CCRV32 Processor AMBA Main Memory EDAC Controller definitions
 * @author          Krzysztof Marcinek
 *
 * @addtogroup      CCAMBA
 * AMBA Bus Peripherials
 * @{
 */

#ifndef __CCRV32_AMBA_EDAC_H
#define __CCRV32_AMBA_EDAC_H

#include <stdint.h>

#ifdef CCRV32_SDK
 #include "ccrv32.h"
#endif

/************************//**
 * @defgroup edac Main Memory EDAC Controller
 * Main Memory EDAC Controller registers and definitions
 * @{
 *//************************/

/** Main Memory EDAC Controller Registers */
typedef struct
{
    uint32_t CTRL;           /*!< Control Register                      */
    uint32_t _reserved0[1];
    uint32_t STAT;           /*!< Status Register                       */
    uint32_t _reserved1[1];
    uint32_t SERR_CNT;       /*!< Single Errors Count Register          */
    uint32_t DERR_CNT;       /*!< Double Errors Count Register          */
    uint32_t SERR_ADDR;      /*!< Single Errors Address Register        */
    uint32_t DERR_ADDR;      /*!< Double Errors Address Register        */
    uint32_t ERR_INJ0;       /*!< Error Injection Register 0            */
    uint32_t ERR_INJ1;       /*!< Error Injection Register 1            */
    uint32_t SCR_START_ADDR; /*!< Scrubber Start Address Register       */
    uint32_t SCR_END_ADDR;   /*!< Scrubber End Address Register         */
    uint32_t SCR_PERIOD;     /*!< Scrubber Period Register              */
    uint32_t IRQ_MAP;        /*!< Interrupt Mapping Register            */
} amba_edac_t;

/** EDAC CTRL bits */
enum
{
    EDAC_CTRL_IRQ_EN    = 0x01,  /*!< Interrupt Enable                 */
    EDAC_CTRL_SLP_EN    = 0x02,  /*!< Memory Bank Sleep Enable         */
    EDAC_CTRL_ECC_EN    = 0x04,  /*!< Enable ECC Operation             */
    EDAC_CTRL_ERR_EN    = 0x08,  /*!< Enable Error Response            */
    EDAC_CTRL_SBR_EN    = 0x10,  /*!< Enable Scrubber Operation        */
    EDAC_CTRL_INJ_EN    = 0x20,  /*!< Enable Error Injection           */
};

/** @} */

#endif /* __CCRV32_AMBA_EDAC_H */
/** @} */
