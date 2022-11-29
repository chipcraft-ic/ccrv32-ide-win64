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
 * @file            ccrv32-gnss.h
 * @brief           CCRV32 Processor Core Peripheral Access Layer Header File.
 * @author          Krzysztof Marcinek
 *
 * @addtogroup      CCRV32
 * CC Processor core definitions
 * @{
 */

#ifndef __CCRV32_GNSS_H
#define __CCRV32_GNSS_H

#include <stdint.h>
#include "ccrv32.h"

/************************//**
 * @defgroup gnss GNSS Controller
 * GNSS Controller registers and definitions
 * @{
 *//************************/

/** GNSS Controller Registers */
typedef struct
{
    uint32_t STATUS;            /*!< Status Register                                      */
    uint32_t COUNT;             /*!< Count Register                                       */
    uint32_t ADDRI;             /*!< I Array Address                                      */
    uint32_t ADDRQ;             /*!< Q Array Address                                      */
    uint32_t COUNTI;            /*!< Remaining I Count Register                           */
    uint32_t COUNTQ;            /*!< Remaining Q Count Register                           */
    uint32_t CTRL;              /*!< GNSS-ISE Control Register                            */
    uint32_t TICKF;             /*!< GNSS-ISE Code Tick Interrupt Flags Register          */
    uint32_t OVRF;              /*!< GNSS-ISE Code Tick Overrun Interrupt Flags Register  */
    uint32_t ERRF;              /*!< GNSS-ISE Code Error Interrupt Flags Register         */
    uint32_t IRQMAP;            /*!< Interrupt Mapping Register                           */
    uint32_t FIFOCNT;           /*!< FIFO Count Override (for diagnostics only)           */
    uint32_t FIFOWR;            /*!< FIFO Write Override (for diagnostics only)           */
    uint32_t TMSTMP_ACQ_LO;     /*!< Acquisition Timestamp LO                             */
    uint32_t TMSTMP_ACQ_HI;     /*!< Acquisition Timestamp HI                             */
    uint32_t ADCVAL;            /*!< Test ADC input (for diagnostics only)                */
    uint32_t RNG_NCO;           /*!< Range Generation NCO Register                        */
    uint32_t RNG_PERIOD;        /*!< Range Generation Period Register                     */
    uint32_t _reserved0[1];
    uint32_t TMSTMP_RNG_LO;     /*!< Range Generation Timestamp LO                        */
    uint32_t TMSTMP_RNG_HI;     /*!< Range Generation Timestamp HI                        */
    uint32_t FLAGS;             /*!< Flags Register                                       */
    uint32_t PPS_IN_CFG;        /*!< PPS Input Configuration Register                     */
    uint32_t PPS_PERIOD;        /*!< PPS Generation Period Register                       */
    uint32_t PPS_HI;            /*!< PPS Generation Period High Register                  */
    uint32_t PPS_PHASE;         /*!< PPS Output Phase Correction Register                 */
    uint32_t TMSTMP_PPS_IN_LO;  /*!< PPS Input Timestamp LO                               */
    uint32_t TMSTMP_PPS_IN_HI;  /*!< PPS Input Timestamp HI                               */
    uint32_t TMSTMP_PPS_OUT_LO; /*!< PPS Output Timestamp LO                              */
    uint32_t TMSTMP_PPS_OUT_HI; /*!< PPS Output Timestamp HI                              */
    uint32_t CARR_CFG;          /*!< Carrier Removal Configuration Register               */
    uint32_t CARR_NCO;          /*!< Carrier Removal NCO Register                         */
    uint32_t TMSTMP_RTC_CMP_LO; /*!< RTC Compare Timestamp LO Register                    */
    uint32_t TMSTMP_RTC_CMP_HI; /*!< RTC Compare Timestamp HI Register                    */
    uint32_t GNSS_DIAG_FUNCT;   /*!< GNSS-ISE Diagnostic Function Register                */
    uint32_t GNSS_DIAG_DATA1;   /*!< GNSS-ISE Diagnostic Data Register                    */
    uint32_t GNSS_DIAG_DATA2;   /*!< GNSS-ISE Diagnostic Data Register                    */
} gnss_regs_t;

static volatile gnss_regs_t * const GNSS_PTR = (gnss_regs_t*)GNSS_BASE;

#define GNSS_I_FIFO_BASE (GNSS_BASE+0x6000)       /*!< GNSS Real FIFO base address (for diagnostics, access only by core 0)        */
#define GNSS_Q_FIFO_BASE (GNSS_BASE+0x6400)       /*!< GNSS Imaginary FIFO base address (for diagnostics, access only by core 0)   */
#define GNSS_FIFO_SIZE   256                      /*!< GNSS FIFO size in 32-bit words                                              */

/** GNSS Controller Status Register Flags */
enum
{
    GNSS_STAT_L1E1        = 1 << 0,  /*!< GNSS L1/E1 Available                        */
    GNSS_STAT_L5E5        = 1 << 1,  /*!< GNSS L5/E5 Available                        */
    GNSS_STAT_L2E6        = 1 << 2,  /*!< GNSS L2/E6 Available                        */
    GNSS_STAT_L1E1_EN     = 1 << 4,  /*!< Enable L1/E1 FIFO                           */
    GNSS_STAT_L5E5_EN     = 1 << 5,  /*!< Enable L5/E5 FIFO                           */
    GNSS_STAT_L2E6_EN     = 1 << 6,  /*!< Enable L2/E6 FIFO                           */
    GNSS_STAT_MODE        = 1 << 12, /*!< GNSS Sample Mode                            */
    GNSS_STAT_PLAY        = 1 << 13, /*!< GNSS Playback Mode                          */
    GNSS_STAT_BUSY        = 1 << 14, /*!< GNSS Busy                                   */
    GNSS_STAT_START       = 1 << 15, /*!< GNSS Start                                  */
    GNSS_STAT_ERR         = 1 << 16, /*!< GNSS FIFO Error                             */
    GNSS_STAT_OVF_I       = 1 << 17, /*!< GNSS FIFO I Overflow                        */
    GNSS_STAT_OVF_Q       = 1 << 18, /*!< GNSS FIFO Q Overflow                        */
    GNSS_STAT_RTC_RDY     = 1 << 19, /*!< RTC Compare Timestamp Ready                 */
    GNSS_STAT_ACQPLAYIE   = 1 << 20, /*!< Acquisition/Playback Interrupt Enable       */
    GNSS_STAT_ACQPLAYIF   = 1 << 21, /*!< Acquisition/Playback Interrupt Flag         */
    GNSS_STAT_RNGIE       = 1 << 22, /*!< Range Generation Interrupt Enable           */
    GNSS_STAT_RNGIF       = 1 << 23, /*!< Range Generation Interrupt Flag             */
};

/** GNSS Controller Status Register bit offsets */
enum
{
    GNSS_STAT_RFAFE_SHIFT  = 8,   /*!< GNSS RF AFE Shift                        */
    GNSS_STAT_ACQDEC_SHIFT = 24,  /*!< GNSS Acquisition Decimator Shift         */
    GNSS_STAT_ACQSCL_SHIFT = 28,  /*!< GNSS Acquisition Decimator Scale Shift   */
};

/** GNSS Controller Status Register masks */
enum
{
    GNSS_STAT_RFAFE_MASK   = 0x0F << GNSS_STAT_RFAFE_SHIFT,  /*!< GNSS RF AFE Mask                       */
    GNSS_STAT_ACQDEC_MASK  = 0x0F << GNSS_STAT_ACQDEC_SHIFT, /*!< GNSS Acquisition Decimator Mask        */
    GNSS_STAT_ACQSCL_MASK  = 0x0F << GNSS_STAT_ACQSCL_SHIFT, /*!< GNSS Acquisition Decimator Scale Mask  */
};

/** GNSS Controller Status RF AFE */
enum
{
    GNSS_STAT_RFAFE_L1E1   = 0x1,  /*!< GNSS L1/E1 RF AFE         */
    GNSS_STAT_RFAFE_L5E5a  = 0x2,  /*!< GNSS L5/E5a RF AFE        */
    GNSS_STAT_RFAFE_E5b    = 0x3,  /*!< GNSS E5b RF AFE           */
    GNSS_STAT_RFAFE_L2     = 0x4,  /*!< GNSS L2 RF AFE            */
    GNSS_STAT_RFAFE_E6     = 0x5,  /*!< GNSS E6 RF AFE            */
    GNSS_STAT_RFAFE_AUX0   = 0xA,  /*!< GNSS AUX0 RF AFE          */
    GNSS_STAT_RFAFE_AUX1   = 0xB,  /*!< GNSS AUX0 RF AFE          */
};

/** GNSS Controller GNSS-ISE Control Register Flags */
enum
{
    GNSS_CTRL_CODE_TICK_IE   = 0x1,  /*!< GNSS-ISE Code Tick Interrupt Enable            */
    GNSS_CTRL_CODE_OVR_IE    = 0x2,  /*!< GNSS-ISE Code Overrun Interrupt Enable         */
    GNSS_CTRL_CODE_ERR_IE    = 0x4,  /*!< GNSS-ISE Code Error Interrupt Enable           */
};


/** GNSS Controller Flags Register Flags */
enum
{
    GNSS_FLAGS_ACQPLAY  = 1 << 0, /*!< Acquisition/Playback Interrupt Flag         */
    GNSS_FLAGS_RNGIF    = 1 << 1, /*!< Range Generation Interrupt Flag             */
};

/** GNSS Controller PPS Input Configuration Register Flags */
enum
{
    GNSS_PPS_IN_EN  = 1 << 5,   /*!< PPS Input Enable */
};

/** GNSS Controller PPS Input Configuration Register bit offsets */
enum
{
    GNSS_PPS_IN_PIN_SHIFT  = 0,   /*!< PPS Input Pin Selection Shift */
};

/** GNSS Controller PPS Input Configuration Register bit masks */
enum
{
    GNSS_PPS_IN_PIN_MASK   = 0x1F << GNSS_PPS_IN_PIN_SHIFT,  /*!< PPS Input Pin Selection Mask */
};

/** GNSS Controller Carrier Removal Configuration Register Flags */
enum
{
    GNSS_CARR_EN = 1 << 0, /*!< Carrier Removal Enable */
};

/** GNSS Controller Carrier Removal Configuration Register bit offsets */
enum
{
    GNSS_CARR_MODE_SHIFT = 2, /*!< Carrier Removal Mode Shift */
};

/** GNSS Controller Carrier Removal Configuration Register bit masks */
enum
{
    GNSS_CARR_MODE_MASK = 0x3 << GNSS_CARR_MODE_SHIFT, /*!< Carrier Removal Mode Mask */
};

/** GNSS Carrier Removal Modes */
enum
{
    GNSS_CARR_PIMQ_PQPI = 0,
    GNSS_CARR_PIPQ_PQMI = 1,
    GNSS_CARR_PIPQ_MQPI = 2,
    GNSS_CARR_PIMQ_MQMI = 3,
};

/**
 * @name GNSS Controller Status Register macros
 * @{
 */
#define GNSS_STATUS_GET_RFAFE(status)      ((status & GNSS_STAT_RFAFE_MASK) >> GNSS_STAT_RFAFE_SHIFT)      /*!< Gets GNSS RF AFE                     */
#define GNSS_STATUS_BUILD_RFAFE(rfafe)     ((rfafe << GNSS_STAT_RFAFE_SHIFT) & GNSS_STAT_RFAFE_MASK)       /*!< Builds GNSS RF AFE                   */
#define GNSS_STATUS_GET_ACQDEC(status)     ((status & GNSS_STAT_ACQDEC_MASK) >> GNSS_STAT_ACQDEC_SHIFT)    /*!< Gets Acquisition Decimator           */
#define GNSS_STATUS_BUILD_ACQDEC(acqdec)   ((acqdec << GNSS_STAT_ACQDEC_SHIFT) & GNSS_STAT_ACQDEC_MASK)    /*!< Builds Acquisition Decimator         */
#define GNSS_STATUS_GET_ACQSCL(status)     ((status & GNSS_STAT_ACQSCL_MASK) >> GNSS_STAT_ACQSCL_SHIFT)    /*!< Gets Acquisition Decimator Scale     */
#define GNSS_STATUS_BUILD_ACQSCL(acqscl)   ((acqscl << GNSS_STAT_ACQSCL_SHIFT) & GNSS_STAT_ACQSCL_MASK)    /*!< Builds Acquisition Decimator Scale   */
/** @} */

/** @} */

#endif /* __CCRV32_GNSS_H */
/** @} */
