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
 * @file            ccrv32-amba-streamer.h
 * @brief           CCRV32 Processor AMBA Data Streamer definitions
 * @author          Sebastian Cieslak
 *
 * @addtogroup      CCAMBA
 * AMBA Bus Peripherials
 * @{
 */

#ifndef __CCRV32_AMBA_STREAMER_H
#define __CCRV32_AMBA_STREAMER_H

#include <stdint.h>

#ifdef CCRV32_SDK
 #include "ccrv32.h"
#endif

/************************//**
 * @defgroup dma STREAMER Controller
 * GNSS Streamer Controller registers and definitions
 * @{
 *//************************/

/** @brief STREAMER Registers */
typedef struct
{
    uint32_t STATUS;       /*!< Data Streamer Status Register                */
    uint32_t CTRL;         /*!< Data Streamer Control Register               */
    uint32_t CONFIG;       /*!< Data Streamer Configuration Register         */
    uint32_t INFO;         /*!< Data Streamer Info Register                  */
    uint32_t ADDRESS;      /*!< Data Streamer Memory Address Register        */
    uint32_t ADDRESSREL;   /*!< Data Streamer Memory Address Reload Register */
    uint32_t COUNTER;      /*!< Data Streamer Sample Counter Register        */
    uint32_t COUNTERREL;   /*!< Data Streamer Sample Counter Reload Register */
    uint32_t IRQMASK;      /*!< Data Streamer Interrupt Mask Register        */
} amba_streamer_t;

/** STREAMER Data Streamer Status Register Flags */
enum
{
    DS_STATUS_BUSY = 1 << 0,  /*!< Streamer Busy Flag              */
    DS_STATUS_SCZ  = 1 << 1,  /*!< Sample Counter Zero Flag        */
    DS_STATUS_SRCZ = 1 << 2,  /*!< Sample Counter Reload Zero Flag */
    DS_STATUS_MAR  = 1 << 3,  /*!< Memory Address Reload Flag      */
    DS_STATUS_MERR = 1 << 4,  /*!< Memory Access Error Flag        */
};

/** STREAMER Data Streamer Status Register Shifts */
enum
{
    DS_STATUS_FIFOU_SHIFT = 5,  /*!< FIFO Usage Shift */
};

/** STREAMER Data Streamer Status Register Masks */
enum
{
    DS_STATUS_FIFOU_MASK = 0x03FF << DS_STATUS_FIFOU_SHIFT,  /*!< FIFO Usage Mask */
};

/** STREAMER Data Streamer Control Register Flags */
enum
{
    DS_CTRL_ACT   = 1 << 0,  /*!< Streamer Active Flag  */
    DS_CTRL_ENA   = 1 << 1,  /*!< Enable Streaming Flag */
    DS_CTRL_FLUSH = 1 << 2,  /*!< Flush Streamer Flag   */
};

/** STREAMER Data Streamer Configuration Register Flags */
enum
{
    DS_CONFIG_RING = 1 << 0,  /*!< Ring Buffer Flag                     */
    DS_CONFIG_SEST = 1 << 5,  /*!< Signed Extended Stream Transfer Flag */
};

/** STREAMER Data Streamer Configuration Register Shifts */
enum
{
    DS_CONFIG_SSS_SHIFT = 1,  /*!< Stream Sample Size Shift */
};

/** STREAMER Data Streamer Configuration Register Masks */
enum
{
    DS_CONFIG_SSS_MASK = 0x0F << DS_CONFIG_SSS_SHIFT,  /*!< Stream Sample Size Mask */
};

/** STREAMER Data Streamer Info Register Shifts */
enum
{
    DS_INFO_SIDW_SHIFT   = 0,   /*!< Stream Interface Data Width Shift */
    DS_INFO_FIFOD_SHIFT  = 10,  /*!< FIFO Depth Shift                  */
    DS_INFO_FIFOWD_SHIFT = 20,  /*!< FIFO Word Width Shift             */
};

/** STREAMER Data Streamer Info Register Masks */
enum
{
    DS_INFO_SIDW_MASK   = 0x03FF << DS_INFO_SIDW_SHIFT,    /*!< Stream Interface Data Width Mask */
    DS_INFO_FIFOD_MASK  = 0x03FF << DS_INFO_FIFOD_SHIFT,   /*!< FIFO Depth Mask                  */
    DS_INFO_FIFOWD_MASK = 0x03FF << DS_INFO_FIFOWD_SHIFT,  /*!< FIFO Word Width Mask             */
};

/** STREAMER Data Streamer Interrupt Mask Register Flags */
enum
{
    DS_IRQMASK_SCZIE  = 1 << 0,  /*!< Sample Counter Zero Interrupt Enable Flag        */
    DS_IRQMASK_SRCZIE = 1 << 1,  /*!< Sample Counter Reload Zero Interrupt Enable Flag */
    DS_IRQMASK_MARIE  = 1 << 2,  /*!< Memory Address Reload Interrupt Enable Flag      */
    DS_IRQMASK_MERRIE = 1 << 3,  /*!< Memory Access Error Interrupt Enable Flag        */
};

/** @} */

#endif /* __CCRV32_AMBA_STREAMER_H */
/** @} */
