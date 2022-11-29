/* ----------------------------------------------------------------------
*
* Copyright (c) 2021 ChipCraft Sp. z o.o. All rights reserved
*
* $Date: 2022-04-04 11:08:39 +0200 (pon, 04 kwi 2022) $
* $Revision: 853 $
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
 * @file            ccrv32-amba-acqeng.h
 * @brief           CCRV32 Processor AMBA GNSS Acquisition Engine definitions
 * @author          Sebastian Cieslak
 *
 * @addtogroup      CCAMBA
 * AMBA Bus Peripherials
 * @{
 */

#ifndef __CCRV32_AMBA_ACQENG_H
#define __CCRV32_AMBA_ACQENG_H

#include <stdint.h>

#ifdef CCRV32_SDK
 #include "ccrv32.h"
 #include "ccrv32-amba-streamer.h"
#endif

/************************//**
 * @defgroup dma ACQENG Controller
 * GNSS Acquisition Engine Controller registers and definitions
 * @{
 *//************************/

/** @brief ACQENG Registers */
typedef struct
{
    amba_streamer_t DS_I_BASE;  /*!< Data Streamer I Base Address                                       */
    uint32_t _reserved0[65527]; /*!< Reserved for Data Streamer I                                       */
    amba_streamer_t DS_Q_BASE;  /*!< Data Streamer Q Base Address                                       */
    uint32_t _reserved1[65527]; /*!< Reserved for Data Streamer Q                                       */
    uint32_t STATUS;            /*!< Acquisition Engine Core Status Register                            */
    uint32_t CTRL;              /*!< Acquisition Engine Core Control Register                           */
    uint32_t CONFIG;            /*!< Acquisition Engine Core Main Configuration Register                */
    uint32_t CFG_F_IF;          /*!< Acquisition Engine Core Carrier Frequency Configuration Register   */
    uint32_t CFG_F_DOPP;        /*!< Acquisition Engine Core Doppler Frequency Configuration Register   */
    uint32_t SET_F_PHASE;       /*!< Acquisition Engine Core Sine NCO Phase Register                    */
    uint32_t CFG_FINE_F_DOPP;   /*!< Acquisition Engine Core Fine Doppler Search Configuration Register */
    uint32_t CFG_PRN_CODE;      /*!< Acquisition Engine Core PRN Code Number Configuration Register     */
    uint32_t CFG_PRN_FREQ;      /*!< Acquisition Engine Core PRN Frequency Configuration Register       */
    uint32_t CFG_POST;          /*!< Acquisition Engine Core Post-processing Configuration Register     */
    uint32_t CFG_DET;           /*!< Acquisition Engine Core Detection Configuration Register           */
    uint32_t CFG_MIN_F_DOPP;    /*!< Acquisition Engine Core Min Doppler Range Configuration Register   */
    uint32_t CFG_MAX_F_DOPP;    /*!< Acquisition Engine Core Max Doppler Range Configuration Register   */
    uint32_t CFG_PRN_MASK_0;    /*!< Acquisition Engine Core PRN Mask 0 Configuration Register          */
    uint32_t CFG_PRN_MASK_1;    /*!< Acquisition Engine Core PRN Mask 1 Configuration Register          */
    uint32_t INFO;              /*!< Acquisition Engine Core Info Register                              */
    uint32_t INFO_MF;           /*!< Acquisition Engine Core Info Matched Filter Register               */
    uint32_t IRQMASK;           /*!< Acquisition Engine Core Interrupt Mask Register                    */
} amba_acqeng_t;

#ifdef CCRV32_SDK
 static volatile amba_acqeng_t   * const AMBA_ACQENG_PTR      = (amba_acqeng_t*)AMBA_ACQENG_BASE;              /*!< GNSS Acquisition Engine Controller pointer      */
 static volatile amba_streamer_t * const AMBA_ACQENG_DS_I_PTR = (amba_streamer_t*)&AMBA_ACQENG_PTR->DS_I_BASE; /*!< GNSS Acquisition Engine Data Streamer I pointer */
 static volatile amba_streamer_t * const AMBA_ACQENG_DS_Q_PTR = (amba_streamer_t*)&AMBA_ACQENG_PTR->DS_Q_BASE; /*!< GNSS Acquisition Engine Data Streamer Q pointer */
#endif

#define DS_FIFO_SIZE (256*4)                                               /*!< Data Streamer FIFO Size                                                        */
#define AEC_ACCU_SIZE (4096*4)                                             /*!< Acquisition Engine Core Accumulator Size                                       */
#define AEC_PRN_SIZE (8192*4)                                              /*!< Acquisition Engine Core PRN Generator Size                                     */
#define DIAG_ACQENG_BASE (0xC0000)                                         /*!< Diagnostic Access Base Address                                                 */
#define DIAG_AEC_PRN_BASE (AMBA_ACQENG_BASE+DIAG_ACQENG_BASE+AEC_PRN_SIZE) /*!< Diagnostic Access Acquisition Engine Core PRN Generator Base Address           */
#define DIAG_AEC_COH_ACCU_I_BASE (DIAG_AEC_PRN_BASE+AEC_PRN_SIZE)          /*!< Diagnostic Access Acquisition Engine Core Coherent Accumulator I Base Address  */
#define DIAG_AEC_COH_ACCU_Q_BASE (DIAG_AEC_COH_ACCU_I_BASE+AEC_ACCU_SIZE)  /*!< Diagnostic Access Acquisition Engine Core Coherent Accumulator Q Base Address  */
#define DIAG_AEC_NCOH_ACCU_BASE (DIAG_AEC_COH_ACCU_Q_BASE+AEC_ACCU_SIZE)   /*!< Diagnostic Access Acquisition Engine Core Noncoherent Accumulator Base Address */
#define DIAG_DS_I_FIFO_BASE (DIAG_AEC_PRN_BASE-(DS_FIFO_SIZE*2))           /*!< Diagnostic Access Data Streamer I FIFO Base Address                            */
#define DIAG_DS_Q_FIFO_BASE (DIAG_DS_I_FIFO_BASE+DS_FIFO_SIZE)             /*!< Diagnostic Access Data Streamer Q FIFO Base Address                            */

/** ACQENG Acquisition Engine Core Status Register Flags */
enum
{
    AEC_STATUS_BUSY    = 1 << 0,  /*!< Acquisition Engine Core Busy Flag */
    AEC_STATUS_PRNDONE = 1 << 1,  /*!< PRN Generation Done Flag          */
    AEC_STATUS_SATFO   = 1 << 2,  /*!< Satellite Found Flag              */
    AEC_STATUS_DONE    = 1 << 3,  /*!< Acquisition Done Flag             */
    AEC_STATUS_AUTOERR = 1 << 20, /*!< Auto Mode Error Flag              */
};

/** ACQENG Acquisition Engine Core Status Register Shifts */
enum
{
    AEC_STATUS_PRNSHFT_SHIFT = 4, /*!< PRN Shift Shift */
};

/** ACQENG Acquisition Engine Core Status Register Masks */
enum
{
    AEC_STATUS_PRNSHFT_MASK = 0xFFFF << AEC_STATUS_PRNSHFT_SHIFT, /*!< PRN Shift Mask */
};

/** ACQENG Acquisition Engine Core Control Register Flags */
enum
{
    AEC_CTRL_CLKREQ    = 1 << 0,  /*!< Clock Request Flag                       */
    AEC_CTRL_DIAGMBIST = 1 << 1,  /*!< Diagnostic Access MBIST Mode             */
    AEC_CTRL_ACT       = 1 << 2,  /*!< Acquisition Engine Core Active Flag      */
    AEC_CTRL_READY     = 1 << 3,  /*!< AXI Stream Ready Flag                    */
    AEC_CTRL_PRNCL     = 1 << 4,  /*!< PRN Generator Clear Flag                 */
    AEC_CTRL_PRNLO     = 1 << 5,  /*!< Load PRN Code Flag                       */
    AEC_CTRL_CARRCL    = 1 << 6,  /*!< Carrier Mixer Clear Flag                 */
    AEC_CTRL_POSTCL    = 1 << 7,  /*!< Post-processing Clear Flag               */
    AEC_CTRL_DROPSMP   = 1 << 8,  /*!< Drop Next Sample Flag                    */
    AEC_CTRL_REFILLMF  = 1 << 9,  /*!< Force Refilling Matched Filter Data Flag */
    AEC_CTRL_AUTOCL    = 1 << 10, /*!< Auto Mode Clear Flag                     */
};

/** ACQENG Acquisition Engine Core Main Configuration Register Flags */
enum
{
    AEC_CONFIG_AUTO    = 1 << 0,   /*!< Automatic Mode Flag           */
    AEC_CONFIG_GPS     = 1 << 1,   /*!< GPS Enabled Flag              */
    AEC_CONFIG_GALIL   = 1 << 2,   /*!< Galileo Enabled Flag          */
    AEC_CONFIG_GLONA   = 1 << 3,   /*!< GLONASS Enabled Flag          */
    AEC_CONFIG_BEID    = 1 << 4,   /*!< Beidou Enabled Flag           */
    AEC_CONFIG_QZSS    = 1 << 5,   /*!< QZSS Enabled Flag             */
    AEC_CONFIG_NAVIC   = 1 << 6,   /*!< NAVIC Enabled Flag            */
    AEC_CONFIG_L1E1F   = 1 << 7,   /*!< L1/E1 Frequency Enabled Flag  */
    AEC_CONFIG_L2F     = 1 << 8,   /*!< L2 Frequency Enabled Flag     */
    AEC_CONFIG_E6F     = 1 << 9,   /*!< E6 Frequency Enabled Flag     */
    AEC_CONFIG_L5E5AF  = 1 << 10,  /*!< L5/E5a Frequency Enabled Flag */
    AEC_CONFIG_E5BF    = 1 << 11,  /*!< E5b Frequency Enabled Flag    */
    AEC_CONFIG_SF      = 1 << 12,  /*!< S Frequency Enabled Flag      */
    AEC_CONFIG_CF      = 1 << 13,  /*!< C Frequency Enabled Flag      */
    AEC_CONFIG_MIXER   = 1 << 14,  /*!< Carrier Mixer Enabled Flag    */
    AEC_CONFIG_FFT     = 1 << 15,  /*!< FFT Enabled Flag              */
    AEC_CONFIG_BOCMIX  = 1 << 18,  /*!< PRN BOC Mixer Enabled Flag    */
    AEC_CONFIG_CYCLIC  = 1 << 23,  /*!< Cyclic Data Enabled Flag      */
    AEC_CONFIG_SBAS    = 1 << 24,  /*!< SBAS Enabled Flag             */
    AEC_CONFIG_ALWFINE = 1 << 25,  /*!< Always Fine Enabled Flag      */
};

/** ACQENG Acquisition Engine Core Main Configuration Register Shifts */
enum
{
    AEC_CONFIG_CARRM_SHIFT = 16,  /*!< Carrier Mode Choice Shift       */
    AEC_CONFIG_SPCS_SHIFT  = 19,  /*!< Samples per Chip as Shift Shift */
};

/** ACQENG Acquisition Engine Core Main Configuration Register Masks */
enum
{
    AEC_CONFIG_CARRM_MASK = 0x03 << AEC_CONFIG_CARRM_SHIFT,  /*!< Carrier Mode Choice Mask       */
    AEC_CONFIG_SPCS_MASK  = 0x0F << AEC_CONFIG_SPCS_SHIFT,   /*!< Samples per Chip as Shift Mask */
};

/** ACQENG Acquisition Engine Core PRN Code Number Register Shifts */
enum
{
    AEC_CFG_PRN_CODE_PRN_SHIFT   = 0,  /*!< Number of PRN code Shift              */
    AEC_CFG_PRN_CODE_PRNBI_SHIFT = 8,  /*!< Number of PRN code bits to load Shift */
};

/** ACQENG Acquisition Engine Core PRN Code Number Register Masks */
enum
{
    AEC_CFG_PRN_CODE_PRN_MASK   = 0xFF   << AEC_CFG_PRN_CODE_PRN_SHIFT,    /*!< Number of PRN code Mask              */
    AEC_CFG_PRN_CODE_PRNBI_MASK = 0xFFFF << AEC_CFG_PRN_CODE_PRNBI_SHIFT,  /*!< Number of PRN code bits to load Mask */
};

/** ACQENG Acquisition Engine Core Post-processing Configuration Register Shifts */
enum
{
    AEC_CFG_POST_PERIOD_SHIFT   = 0,   /*!< PRN Period Shift                                      */
    AEC_CFG_POST_NPERCOH_SHIFT  = 16,  /*!< Number of PRN Period Coherent Accumulations Shift     */
    AEC_CFG_POST_NPERNCOH_SHIFT = 20,  /*!< Number of PRN Periods Noncoherent Accumulations Shift */
    AEC_CFG_POST_STEP_SHIFT     = 24,  /*!< PRN step Shift                                        */
};

/** ACQENG Acquisition Engine Core Post-processing Configuration Register Mask */
enum
{
    AEC_CFG_POST_PERIOD_MASK   = 0xFFFF << AEC_CFG_POST_PERIOD_SHIFT,    /*!< PRN Period Mask                                      */
    AEC_CFG_POST_NPERCOH_MASK  = 0xF    << AEC_CFG_POST_NPERCOH_SHIFT,   /*!< Number of PRN Periods Coherent Accumulations Mask    */
    AEC_CFG_POST_NPERNCOH_MASK = 0xF    << AEC_CFG_POST_NPERNCOH_SHIFT,  /*!< Number of PRN Periods Noncoherent Accumulations Mask */
    AEC_CFG_POST_STEP_MASK     = 0xFF   << AEC_CFG_POST_STEP_SHIFT,      /*!< PRN step Mask                                        */
};

/** ACQENG Acquisition Engine Core Info Register Flags */
enum
{
    AEC_INFO_HAS_AUTO  = 1 << 8,   /*!< Has Automatic Mode Flag            */
    AEC_INFO_SBAS_L1   = 1 << 19,  /*!< Has SBAS L1 PRN Generator Flag     */
    AEC_INFO_NAVIC_L5  = 1 << 20,  /*!< Has NAVIC L5 PRN Generator Flag    */
    AEC_INFO_QZSS_L1   = 1 << 21,  /*!< Has QZSS L1 PRN Generator Flag     */
    AEC_INFO_BEID      = 1 << 22,  /*!< Has Beidou PRN Generator Flag      */
    AEC_INFO_GLONA_L1  = 1 << 23,  /*!< Has GLONASS L1 PRN Generator Flag  */
    AEC_INFO_GALIL_C   = 1 << 24,  /*!< Has Galileo C PRN Generator Flag   */
    AEC_INFO_GALIL_E5B = 1 << 25,  /*!< Has Galileo E5b PRN Generator Flag */
    AEC_INFO_GALIL_E5A = 1 << 26,  /*!< Has Galileo E5a PRN Generator Flag */
    AEC_INFO_GALIL_E6  = 1 << 27,  /*!< Has Galileo E6 PRN Generator Flag  */
    AEC_INFO_GALIL_E1  = 1 << 28,  /*!< Has Galileo E1 PRN Generator Flag  */
    AEC_INFO_GPS_L5    = 1 << 29,  /*!< Has GPS L5 PRN Generator Flag      */
    AEC_INFO_GPS_L2    = 1 << 30,  /*!< Has GPS L2 PRN Generator Flag      */
    AEC_INFO_GPS_L1    = 1 << 31,  /*!< Has GPS L1 PRN Generator Flag      */
};

/** ACQENG Acquisition Engine Core Info Register Shifts */
enum
{
    AEC_INFO_VERSION_SHIFT = 0,  /*!< Acquisition Engine Version Shift                */
    AEC_INFO_CORRWD_SHIFT  = 9,  /*!< Correlation Width / Accumulator RAM Width Shift */
    AEC_INFO_RSVD_SHIFT    = 14, /*!< Reserved Shift                                  */
};

/** ACQENG Acquisition Engine Core Info Register Masks */
enum
{
    AEC_INFO_VERSION_MASK = 0xFF << AEC_INFO_VERSION_SHIFT, /*!< Acquisition Engine Version Mask                */
    AEC_INFO_CORRWD_MASK  = 0x1F << AEC_INFO_CORRWD_SHIFT,  /*!< Correlation Width / Accumulator RAM Width Mask */
    AEC_INFO_RSVD_MASK    = 0x1F << AEC_INFO_RSVD_SHIFT,    /*!< Reserved Shift                                 */
};

/** ACQENG Acquisition Engine Core Info Matched Filter Register Shifts */
enum
{
    AEC_INFO_MF_MFDATA_SHIFT = 0,   /*!< Matched Filter Data Shift Register Length Shift */
    AEC_INFO_MF_MFCODE_SHIFT = 16,  /*!< Matched Filter Code Shift Register Length Shift */
};

/** ACQENG Acquisition Engine Core Info Matched Filter Register Masks */
enum
{
    AEC_INFO_MF_MFDATA_MASK = 0xFFFF << AEC_INFO_MF_MFDATA_SHIFT,  /*!< Matched Filter Data Shift Register Length Mask */
    AEC_INFO_MF_MFCODE_MASK = 0xFFFF << AEC_INFO_MF_MFCODE_SHIFT,  /*!< Matched Filter Code Shift Register Length Mask */
};

/** ACQENG Acquisition Engine Core Interrupt Mask Register Flags */
enum
{
    AEC_IRQMASK_PRNDONEIE = 1 << 0,  /*!< PRN Generation Done Interrupt Enable */
    AEC_IRQMASK_SATFOIE   = 1 << 1,  /*!< Satellite Found Interrupt Enable     */
    AEC_IRQMASK_DONEIE    = 1 << 2,  /*!< Acquisition Done Interrupt Enable    */
};

/** @} */

#endif /* __CCRV32_AMBA_ACQENG_H */
/** @} */
