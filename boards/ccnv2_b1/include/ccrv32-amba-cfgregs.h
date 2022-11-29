/* ----------------------------------------------------------------------
*
* Copyright (c) 2017 ChipCraft Sp. z o.o. All rights reserved
*
* $Date$
* $Revision$
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
 * @brief           CC Processor AMBA cfg_regs definitions
 * @author          Krzysztof Marcinek
 *
 * @addtogroup      CCAMBA
 * AMBA Bus Peripherials
 * @{
 */

#ifndef __CFG_REGS_H
#define __CFG_REGS_H

#include <stdint.h>

/************************//**
 * @defgroup cfg_regs CFG_REGS Controller
 * cfg_regs registers and definitions
 * @{
 *//************************/

/** @brief cfg_regs Registers */
typedef struct
{
    uint32_t CFGREG_IRQMAP;
    uint32_t CFGREG_UNLOCK;
    uint32_t CFGREG_IRQFLAGS;
    uint32_t CFGREG_MEM_CONF;
    uint32_t CFGREG_RTCSTAT;
    uint32_t CFGREG_RTCCONF;
    uint32_t CFGREG_COREFREQ_CLK;
    uint32_t CFGREG_COREFREQ_PLL;
    uint32_t CFGREG_COREFREQ_STAT;
    uint32_t CFGREG_GNSSAFE_CONF;
    uint32_t CFGREG_PLL1_CONF;
    uint32_t CFGREG_PLL1DCO_CONF;
    uint32_t CFGREG_PLL1_STAT;
    uint32_t CFGREG_PLL1_STAT2;
    uint32_t CFGREG_PLL1DCO_STAT;
    uint32_t CFGREG_PLL25_CONF;
    uint32_t CFGREG_PLL25DCO_CONF;
    uint32_t CFGREG_PLL25_STAT;
    uint32_t CFGREG_PLL25_STAT2;
    uint32_t CFGREG_PLL25DCO_STAT;
    uint32_t CFGREG_IF1_CONF;
    uint32_t CFGREG_IF25_CONF;
    uint32_t CFGREG_PROBE_CONF;
    uint32_t CFGREG_ADC1_CONF;
    uint32_t CFGREG_ADC25_CONF;
    uint32_t CFGREG_SPLIT1_CTRL;
    uint32_t CFGREG_SPLIT1_CFG_GAIN_CHANGE;
    uint32_t CFGREG_SPLIT1_CFG_LIMITER;
    uint32_t CFGREG_SPLIT1_CFG_MANUAL_MODE;
    uint32_t CFGREG_SPLIT1_L1E1_CFG;
    uint32_t CFGREG_SPLIT1_L1E1_CFG_CARR_FREQ;
    uint32_t CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I;
    uint32_t CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q;
    uint32_t CFGREG_SPLIT1_L1E1_CFG_MANUAL_MODE_DIG;
    uint32_t CFGREG_SPLIT25_CTRL;
    uint32_t CFGREG_SPLIT25_CFG_GAIN_CHANGE;
    uint32_t CFGREG_SPLIT25_CFG_LIMITER;
    uint32_t CFGREG_SPLIT25_CFG_MANUAL_MODE;
    uint32_t CFGREG_SPLIT25_L2_CFG;
    uint32_t CFGREG_SPLIT25_L2_CFG_CARR_FREQ;
    uint32_t CFGREG_SPLIT25_L2_CFG_THRESHOLD_I;
    uint32_t CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q;
    uint32_t CFGREG_SPLIT25_L2_CFG_MANUAL_MODE_DIG;
    uint32_t CFGREG_SPLIT25_E6_CFG;
    uint32_t CFGREG_SPLIT25_E6_CFG_CARR_FREQ;
    uint32_t CFGREG_SPLIT25_E6_CFG_THRESHOLD_I;
    uint32_t CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q;
    uint32_t CFGREG_SPLIT25_E6_CFG_MANUAL_MODE_DIG;
    uint32_t CFGREG_SPLIT25_L5E5A_CFG;
    uint32_t CFGREG_SPLIT25_L5E5A_CFG_CARR_FREQ;
    uint32_t CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I;
    uint32_t CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q;
    uint32_t CFGREG_SPLIT25_L5E5A_CFG_MANUAL_MODE_DIG;
    uint32_t CFGREG_SPLIT25_E5B_CFG;
    uint32_t CFGREG_SPLIT25_E5B_CFG_CARR_FREQ;
    uint32_t CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I;
    uint32_t CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q;
    uint32_t CFGREG_SPLIT25_E5B_CFG_MANUAL_MODE_DIG;
    uint32_t CFGREG_LNA125_CONF;
    uint32_t CFGREG_LNA125_TUNE_CONF;
    uint32_t CFGREG_BALUN_MIXER1_CONF;
    uint32_t CFGREG_BALUN_MIXER25_CONF;
    uint32_t CFGREG_PMU_CONF;
    uint32_t CFGREG_PMU_STAT;
    uint32_t CFGREG_PM_STAT;
    uint32_t CFGREG_PM_CONF;
    uint32_t CFGREG_GNSSAFE_TEST_CONF;
    uint32_t CFGREG_SPARE_CONF;
    uint32_t CFGREG_SPARE_STAT;
} cfg_regs_t;

static volatile cfg_regs_t * const CFG_REGS_PTR = (cfg_regs_t*)AMBA_REG_BASE;


/** Registers Default Values */
enum
{
     CFGREG_IRQMAP_DEF                                            = 0x00000000,  /*!<  Interrupt Mapping Register Default Value */
     CFGREG_UNLOCK_DEF                                            = 0x0000A55A,  /*!<  Unlock Register Default Value */
     CFGREG_IRQFLAGS_DEF                                          = 0x00000000,  /*!<  32kHz RTC Clock Interrupt Flags Default Value */
     CFGREG_MEM_CONF_DEF                                          = 0x00000000,  /*!<  Memory Configuration Register Default Value */
     CFGREG_RTCSTAT_DEF                                           = 0x00000000,  /*!<  32kHz RTC Clock Status Default Value */
     CFGREG_RTCCONF_DEF                                           = 0x00000000,  /*!<  32kHz RTC Clock Configuration Default Value */
     CFGREG_COREFREQ_CLK_DEF                                      = 0x00000002,  /*!<  External XTAL Configuration Register Default Value */
     CFGREG_COREFREQ_PLL_DEF                                      = 0x00000020,  /*!<  PLL Configuration Register Default Value */
     CFGREG_COREFREQ_STAT_DEF                                     = 0x00000000,  /*!<  Core Clock Status Register Default Value */
     CFGREG_GNSSAFE_CONF_DEF                                      = 0x00000000,  /*!<  GNSS-AFE Configuration Register Default Value */
     CFGREG_PLL1_CONF_DEF                                         = 0x00C46A7E,  /*!<  PLL1 Configuration Register Default Value */
     CFGREG_PLL1DCO_CONF_DEF                                      = 0x00104000,  /*!<  PLL1 DCO Configuration Register Default Value */
     CFGREG_PLL1_STAT_DEF                                         = 0x00000000,  /*!<  PLL1 Status Register Default Value */
     CFGREG_PLL1_STAT2_DEF                                        = 0x00001000,  /*!<  PLL1 Status Register Default Value */
     CFGREG_PLL1DCO_STAT_DEF                                      = 0x00000000,  /*!<  PLL1 TDC Status Register Default Value */
     CFGREG_PLL25_CONF_DEF                                        = 0x00C46A7E,  /*!<  PLL25 Configuration Register Default Value */
     CFGREG_PLL25DCO_CONF_DEF                                     = 0x00104000,  /*!<  PLL25 DCO Configuration Register Default Value */
     CFGREG_PLL25_STAT_DEF                                        = 0x00000000,  /*!<  PLL25 Status Register Default Value */
     CFGREG_PLL25_STAT2_DEF                                       = 0x00001000,  /*!<  PLL25 Status Register Default Value */
     CFGREG_PLL25DCO_STAT_DEF                                     = 0x00000000,  /*!<  PLL25 TDC Status Register Default Value */
     CFGREG_IF1_CONF_DEF                                          = 0x00000000,  /*!<  IF1 Configuration Register Default Value */
     CFGREG_IF25_CONF_DEF                                         = 0x00000000,  /*!<  IF25 Configuration Register Default Value */
     CFGREG_PROBE_CONF_DEF                                        = 0x00000000,  /*!<  IF25 Configuration Register Default Value */
     CFGREG_ADC1_CONF_DEF                                         = 0x00000000,  /*!<  IF25 Configuration Register Default Value */
     CFGREG_ADC25_CONF_DEF                                        = 0x00000000,  /*!<  IF25 Configuration Register Default Value */
     CFGREG_SPLIT1_CTRL_DEF                                       = 0x00000000,  /*!<  Splitter AFE1 Control Register Default Value */
     CFGREG_SPLIT1_CFG_GAIN_CHANGE_DEF                            = 0x00030D3F,  /*!<  Splitter AFE1 Gain Change Configuration Register Default Value */
     CFGREG_SPLIT1_CFG_LIMITER_DEF                                = 0x0000C350,  /*!<  Splitter AFE1 Limiter Threshold Configuration Register Default Value */
     CFGREG_SPLIT1_CFG_MANUAL_MODE_DEF                            = 0x00000001,  /*!<  Splitter AFE1 Manual Mode Configuration Register Default Value */
     CFGREG_SPLIT1_L1E1_CFG_DEF                                   = 0x0000000E,  /*!<  Splitter AFE1 L1E1 Configuration Register Default Value */
     CFGREG_SPLIT1_L1E1_CFG_CARR_FREQ_DEF                         = 0x00000000,  /*!<  Splitter AFE1 L1E1 Carrier Frequency Configuration Register Default Value */
     CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_DEF                       = 0x0FFFC000,  /*!<  Splitter AFE1 L1E1 Threshold I Configuration Register Default Value */
     CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_DEF                       = 0x0FFFC000,  /*!<  Splitter AFE1 L1E1 Threshold Q Configuration Register Default Value */
     CFGREG_SPLIT1_L1E1_CFG_MANUAL_MODE_DIG_DEF                   = 0x00000000,  /*!<  Splitter AFE1 L1E1 Manual Mode Digital Gain Configuration Register Default Value */
     CFGREG_SPLIT25_CTRL_DEF                                      = 0x00000000,  /*!<  Splitter AFE25 Control Register Default Value */
     CFGREG_SPLIT25_CFG_GAIN_CHANGE_DEF                           = 0x00030D3F,  /*!<  Splitter AFE25 Gain Change Configuration Register Default Value */
     CFGREG_SPLIT25_CFG_LIMITER_DEF                               = 0x0000C350,  /*!<  Splitter AFE25 Limiter Threshold Configuration Register Default Value */
     CFGREG_SPLIT25_CFG_MANUAL_MODE_DEF                           = 0x00000001,  /*!<  Splitter AFE25 Manual Mode Configuration Register Default Value */
     CFGREG_SPLIT25_L2_CFG_DEF                                    = 0x0000000E,  /*!<  Splitter AFE25 L2 Configuration Register Default Value */
     CFGREG_SPLIT25_L2_CFG_CARR_FREQ_DEF                          = 0x00000000,  /*!<  Splitter AFE25 L2 Carrier Frequency Configuration Register Default Value */
     CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_DEF                        = 0x0FFFC000,  /*!<  Splitter AFE25 L2 Threshold I Configuration Register Default Value */
     CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_DEF                        = 0x0FFFC000,  /*!<  Splitter AFE25 L2 Threshold Q Configuration Register Default Value */
     CFGREG_SPLIT25_L2_CFG_MANUAL_MODE_DIG_DEF                    = 0x00000000,  /*!<  Splitter AFE25 L2 Manual Mode Digital Gain Configuration Register Default Value */
     CFGREG_SPLIT25_E6_CFG_DEF                                    = 0x0000000E,  /*!<  Splitter AFE25 E6 Configuration Register Default Value */
     CFGREG_SPLIT25_E6_CFG_CARR_FREQ_DEF                          = 0x00000000,  /*!<  Splitter AFE25 E6 Carrier Frequency Configuration Register Default Value */
     CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_DEF                        = 0x0FFFC000,  /*!<  Splitter AFE25 E6 Threshold I Configuration Register Default Value */
     CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_DEF                        = 0x0FFFC000,  /*!<  Splitter AFE25 E6 Threshold Q Configuration Register Default Value */
     CFGREG_SPLIT25_E6_CFG_MANUAL_MODE_DIG_DEF                    = 0x00000000,  /*!<  Splitter AFE25 E6 Manual Mode Digital Gain Configuration Register Default Value */
     CFGREG_SPLIT25_L5E5A_CFG_DEF                                 = 0x0000000E,  /*!<  Splitter AFE25 L5E5A Configuration Register Default Value */
     CFGREG_SPLIT25_L5E5A_CFG_CARR_FREQ_DEF                       = 0x00000000,  /*!<  Splitter AFE25 L5E5A Carrier Frequency Configuration Register Default Value */
     CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_DEF                     = 0x0FFFC000,  /*!<  Splitter AFE25 L5E5A Threshold I Configuration Register Default Value */
     CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_DEF                     = 0x0FFFC000,  /*!<  Splitter AFE25 L5E5A Threshold Q Configuration Register Default Value */
     CFGREG_SPLIT25_L5E5A_CFG_MANUAL_MODE_DIG_DEF                 = 0x00000000,  /*!<  Splitter AFE25 L5E5A Manual Mode Digital Gain Configuration Register Default Value */
     CFGREG_SPLIT25_E5B_CFG_DEF                                   = 0x0000000E,  /*!<  Splitter AFE25 E5B Configuration Register Default Value */
     CFGREG_SPLIT25_E5B_CFG_CARR_FREQ_DEF                         = 0x00000000,  /*!<  Splitter AFE25 E5B Carrier Frequency Configuration Register Default Value */
     CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_DEF                       = 0x0FFFC000,  /*!<  Splitter AFE25 E5B Threshold I Configuration Register Default Value */
     CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_DEF                       = 0x0FFFC000,  /*!<  Splitter AFE25 E5B Threshold Q Configuration Register Default Value */
     CFGREG_SPLIT25_E5B_CFG_MANUAL_MODE_DIG_DEF                   = 0x00000000,  /*!<  Splitter AFE25 E5B Manual Mode Digital Gain Configuration Register Default Value */
     CFGREG_LNA125_CONF_DEF                                       = 0x00000000,  /*!<  LNA125 Configuration Register Default Value */
     CFGREG_LNA125_TUNE_CONF_DEF                                  = 0x10001000,  /*!<  LNA125 Configuration Register Default Value */
     CFGREG_BALUN_MIXER1_CONF_DEF                                 = 0x00000018,  /*!<  BALUN and MIXER 1 Configuration Register Default Value */
     CFGREG_BALUN_MIXER25_CONF_DEF                                = 0x00000018,  /*!<  BALUN and MIXER 25 Configuration Register Default Value */
     CFGREG_PMU_CONF_DEF                                          = 0x0000301E,  /*!<  PMU Configuration register Default Value */
     CFGREG_PMU_STAT_DEF                                          = 0x00000000,  /*!<  Power Management Unit Status Default Value */
     CFGREG_PM_STAT_DEF                                           = 0x00000001,  /*!<  GNSS-AFE PM Status Register Default Value */
     CFGREG_PM_CONF_DEF                                           = 0x00000820,  /*!<  GNSS-AFE PM Configuration Register Default Value */
     CFGREG_GNSSAFE_TEST_CONF_DEF                                 = 0x00000000,  /*!<  GNSS-AFE PM Configuration Register Default Value */
     CFGREG_SPARE_CONF_DEF                                        = 0x00000000,  /*!<  SPARE Configuration Register Default Value */
     CFGREG_SPARE_STAT_DEF                                        = 0x00000000,  /*!<  SPARE Configuration Register Default Value */
};

/** Registers Masks */
enum
{

     CFGREG_IRQMAP_IRQMAP_MASK                                    = 0x0000FFFE,  /*!<  Interrupt Mapping Register Mask Value */

     CFGREG_UNLOCK_UNLOCK_KEY_MASK                                = 0x0000FFFF,  /*!<  Unlock Register Mask Value */

     CFGREG_IRQFLAGS_PLL_FAIL_MASK                                = 0x00000004,  /*!<  32kHz RTC Clock Interrupt Flags Mask Value */
     CFGREG_IRQFLAGS_XTAL_CORE_FAIL_MASK                          = 0x00000002,  /*!<  32kHz RTC Clock Interrupt Flags Mask Value */
     CFGREG_IRQFLAGS_XTAL_RTC_FAIL_MASK                           = 0x00000001,  /*!<  32kHz RTC Clock Interrupt Flags Mask Value */

     CFGREG_MEM_CONF_TEST1_MASK                                   = 0x00000020,  /*!<  Memory Configuration Register Mask Value */
     CFGREG_MEM_CONF_RME_MASK                                     = 0x00000010,  /*!<  Memory Configuration Register Mask Value */
     CFGREG_MEM_CONF_RM_MASK                                      = 0x0000000F,  /*!<  Memory Configuration Register Mask Value */

     CFGREG_RTCSTAT_RC_RTC_VAL_MASK                               = 0x000001FC,  /*!<  32kHz RTC Clock Status Mask Value */
     CFGREG_RTCSTAT_RC_RTC_RDY_MASK                               = 0x00000002,  /*!<  32kHz RTC Clock Status Mask Value */
     CFGREG_RTCSTAT_CLK_RTC_SEL_MASK                              = 0x00000001,  /*!<  32kHz RTC Clock Status Mask Value */

     CFGREG_RTCCONF_RC_RTC_VAL_SRC_MASK                           = 0x00000400,  /*!<  32kHz RTC Clock Configuration Mask Value */
     CFGREG_RTCCONF_RC_RTC_VAL_MASK                               = 0x000003F8,  /*!<  32kHz RTC Clock Configuration Mask Value */
     CFGREG_RTCCONF_RC_RTC_CAL_MASK                               = 0x00000004,  /*!<  32kHz RTC Clock Configuration Mask Value */
     CFGREG_RTCCONF_XTAL_RTC_SEL_MASK                             = 0x00000002,  /*!<  32kHz RTC Clock Configuration Mask Value */
     CFGREG_RTCCONF_XTAL_RTC_TEST_MASK                            = 0x00000001,  /*!<  32kHz RTC Clock Configuration Mask Value */

     CFGREG_COREFREQ_CLK_TEST_OUT_EN_MASK                         = 0x00000800,  /*!<  External XTAL Configuration Register Mask Value */
     CFGREG_COREFREQ_CLK_RC_CORE_VAL_SRC_MASK                     = 0x00000400,  /*!<  External XTAL Configuration Register Mask Value */
     CFGREG_COREFREQ_CLK_RC_CORE_VAL_MASK                         = 0x000003F0,  /*!<  External XTAL Configuration Register Mask Value */
     CFGREG_COREFREQ_CLK_RC_CORE_CAL_MASK                         = 0x00000008,  /*!<  External XTAL Configuration Register Mask Value */
     CFGREG_COREFREQ_CLK_CORE_SEL_MASK                            = 0x00000006,  /*!<  External XTAL Configuration Register Mask Value */
     CFGREG_COREFREQ_CLK_XTAL_CORE_TEST_MASK                      = 0x00000001,  /*!<  External XTAL Configuration Register Mask Value */

     CFGREG_COREFREQ_PLL_CTRL_FINE_LOAD_MASK                      = 0x7FFE0000,  /*!<  PLL Configuration Register Mask Value */
     CFGREG_COREFREQ_PLL_CTRL_LOAD_MASK                           = 0x00010000,  /*!<  PLL Configuration Register Mask Value */
     CFGREG_COREFREQ_PLL_CTRL_PVT_LOAD_MASK                       = 0x0000FC00,  /*!<  PLL Configuration Register Mask Value */
     CFGREG_COREFREQ_PLL_PLL_VFB_EN_MASK                          = 0x00000200,  /*!<  PLL Configuration Register Mask Value */
     CFGREG_COREFREQ_PLL_TEST_MASK                                = 0x00000100,  /*!<  PLL Configuration Register Mask Value */
     CFGREG_COREFREQ_PLL_N_MASK                                   = 0x000000FC,  /*!<  PLL Configuration Register Mask Value */
     CFGREG_COREFREQ_PLL_REF_SEL_MASK                             = 0x00000002,  /*!<  PLL Configuration Register Mask Value */
     CFGREG_COREFREQ_PLL_EN_MASK                                  = 0x00000001,  /*!<  PLL Configuration Register Mask Value */

     CFGREG_COREFREQ_STAT_CTRL_FINE_MASK                          = 0x7FFE0000,  /*!<  Core Clock Status Register Mask Value */
     CFGREG_COREFREQ_STAT_CTRL_PVT_MASK                           = 0x0001F800,  /*!<  Core Clock Status Register Mask Value */
     CFGREG_COREFREQ_STAT_RC_CORE_VAL_MASK                        = 0x000007E0,  /*!<  Core Clock Status Register Mask Value */
     CFGREG_COREFREQ_STAT_RC_CORE_RDY_MASK                        = 0x00000010,  /*!<  Core Clock Status Register Mask Value */
     CFGREG_COREFREQ_STAT_PLL_LOCK_MASK                           = 0x00000008,  /*!<  Core Clock Status Register Mask Value */
     CFGREG_COREFREQ_STAT_XTAL_CORE_LOCK_MASK                     = 0x00000004,  /*!<  Core Clock Status Register Mask Value */
     CFGREG_COREFREQ_STAT_CLK_CORE_SEL_MASK                       = 0x00000003,  /*!<  Core Clock Status Register Mask Value */

     CFGREG_GNSSAFE_CONF_GNSS_OUT_BAND_MASK                       = 0x00000004,  /*!<  GNSS-AFE Configuration Register Mask Value */
     CFGREG_GNSSAFE_CONF_GNSS_AUX1_EN_MASK                        = 0x00000002,  /*!<  GNSS-AFE Configuration Register Mask Value */
     CFGREG_GNSSAFE_CONF_GNSS_AUX0_EN_MASK                        = 0x00000001,  /*!<  GNSS-AFE Configuration Register Mask Value */

     CFGREG_PLL1_CONF_EN_BOOST_MASK                               = 0x08000000,  /*!<  PLL1 Configuration Register Mask Value */
     CFGREG_PLL1_CONF_ADC_CLK_DIV_MASK                            = 0x04000000,  /*!<  PLL1 Configuration Register Mask Value */
     CFGREG_PLL1_CONF_LOPCB_EN_MASK                               = 0x02000000,  /*!<  PLL1 Configuration Register Mask Value */
     CFGREG_PLL1_CONF_TEST_EN_MASK                                = 0x01000000,  /*!<  PLL1 Configuration Register Mask Value */
     CFGREG_PLL1_CONF_FCW_MASK                                    = 0x00FFFFFE,  /*!<  PLL1 Configuration Register Mask Value */
     CFGREG_PLL1_CONF_EN_MASK                                     = 0x00000001,  /*!<  PLL1 Configuration Register Mask Value */

     CFGREG_PLL1DCO_CONF_AMP_MASK                                 = 0x03C00000,  /*!<  PLL1 DCO Configuration Register Mask Value */
     CFGREG_PLL1DCO_CONF_AMP_LOAD_MASK                            = 0x00200000,  /*!<  PLL1 DCO Configuration Register Mask Value */
     CFGREG_PLL1DCO_CONF_CTRL_PVT_MASK                            = 0x001F0000,  /*!<  PLL1 DCO Configuration Register Mask Value */
     CFGREG_PLL1DCO_CONF_CTRL_FINE_MASK                           = 0x00007FFE,  /*!<  PLL1 DCO Configuration Register Mask Value */
     CFGREG_PLL1DCO_CONF_CTRL_LOAD_MASK                           = 0x00000001,  /*!<  PLL1 DCO Configuration Register Mask Value */

     CFGREG_PLL1_STAT_PH_VAR_MASK                                 = 0x0FE00000,  /*!<  PLL1 Status Register Mask Value */
     CFGREG_PLL1_STAT_PHERR_MASK                                  = 0x001FFFFE,  /*!<  PLL1 Status Register Mask Value */
     CFGREG_PLL1_STAT_LOCK_MASK                                   = 0x00000001,  /*!<  PLL1 Status Register Mask Value */

     CFGREG_PLL1_STAT2_TV_MASK                                    = 0x0000FFFF,  /*!<  PLL1 Status Register Mask Value */

     CFGREG_PLL1DCO_STAT_AMP_MASK                                 = 0x00F00000,  /*!<  PLL1 TDC Status Register Mask Value */
     CFGREG_PLL1DCO_STAT_AMP_LOW_MASK                             = 0x00080000,  /*!<  PLL1 TDC Status Register Mask Value */
     CFGREG_PLL1DCO_STAT_CTRL_PVT_MASK                            = 0x0007C000,  /*!<  PLL1 TDC Status Register Mask Value */
     CFGREG_PLL1DCO_STAT_CTRL_FINE_MASK                           = 0x00003FFF,  /*!<  PLL1 TDC Status Register Mask Value */

     CFGREG_PLL25_CONF_EN_BOOST_MASK                              = 0x08000000,  /*!<  PLL25 Configuration Register Mask Value */
     CFGREG_PLL25_CONF_ADC_CLK_DIV_MASK                           = 0x04000000,  /*!<  PLL25 Configuration Register Mask Value */
     CFGREG_PLL25_CONF_LOPCB_EN_MASK                              = 0x02000000,  /*!<  PLL25 Configuration Register Mask Value */
     CFGREG_PLL25_CONF_TEST_EN_MASK                               = 0x01000000,  /*!<  PLL25 Configuration Register Mask Value */
     CFGREG_PLL25_CONF_FCW_MASK                                   = 0x00FFFFFE,  /*!<  PLL25 Configuration Register Mask Value */
     CFGREG_PLL25_CONF_EN_MASK                                    = 0x00000001,  /*!<  PLL25 Configuration Register Mask Value */

     CFGREG_PLL25DCO_CONF_AMP_MASK                                = 0x07800000,  /*!<  PLL25 DCO Configuration Register Mask Value */
     CFGREG_PLL25DCO_CONF_AMP_LOAD_MASK                           = 0x00400000,  /*!<  PLL25 DCO Configuration Register Mask Value */
     CFGREG_PLL25DCO_CONF_CTRL_PVT_MASK                           = 0x003F0000,  /*!<  PLL25 DCO Configuration Register Mask Value */
     CFGREG_PLL25DCO_CONF_CTRL_FINE_MASK                          = 0x00007FFE,  /*!<  PLL25 DCO Configuration Register Mask Value */
     CFGREG_PLL25DCO_CONF_CTRL_LOAD_MASK                          = 0x00000001,  /*!<  PLL25 DCO Configuration Register Mask Value */

     CFGREG_PLL25_STAT_PH_VAR_MASK                                = 0x0FE00000,  /*!<  PLL25 Status Register Mask Value */
     CFGREG_PLL25_STAT_PHERR_MASK                                 = 0x001FFFFE,  /*!<  PLL25 Status Register Mask Value */
     CFGREG_PLL25_STAT_LOCK_MASK                                  = 0x00000001,  /*!<  PLL25 Status Register Mask Value */

     CFGREG_PLL25_STAT2_TV_MASK                                   = 0x0000FFFF,  /*!<  PLL25 Status Register Mask Value */

     CFGREG_PLL25DCO_STAT_AMP_MASK                                = 0x01E00000,  /*!<  PLL25 TDC Status Register Mask Value */
     CFGREG_PLL25DCO_STAT_AMP_LOW_MASK                            = 0x00100000,  /*!<  PLL25 TDC Status Register Mask Value */
     CFGREG_PLL25DCO_STAT_CTRL_PVT_MASK                           = 0x000FC000,  /*!<  PLL25 TDC Status Register Mask Value */
     CFGREG_PLL25DCO_STAT_CTRL_FINE_MASK                          = 0x00003FFF,  /*!<  PLL25 TDC Status Register Mask Value */

     CFGREG_IF1_CONF_IF_BANDCUT_MASK                              = 0x00004000,  /*!<  IF1 Configuration Register Mask Value */
     CFGREG_IF1_CONF_OFFSET_CAL_Q_MASK                            = 0x00003C00,  /*!<  IF1 Configuration Register Mask Value */
     CFGREG_IF1_CONF_OFFSET_CAL_DIRECTION_Q_MASK                  = 0x00000200,  /*!<  IF1 Configuration Register Mask Value */
     CFGREG_IF1_CONF_OFFSET_CAL_I_MASK                            = 0x000001E0,  /*!<  IF1 Configuration Register Mask Value */
     CFGREG_IF1_CONF_OFFSET_CAL_DIRECTION_I_MASK                  = 0x00000010,  /*!<  IF1 Configuration Register Mask Value */
     CFGREG_IF1_CONF_PGA2_EN_MASK                                 = 0x00000008,  /*!<  IF1 Configuration Register Mask Value */
     CFGREG_IF1_CONF_PGA1_EN_MASK                                 = 0x00000004,  /*!<  IF1 Configuration Register Mask Value */
     CFGREG_IF1_CONF_PREAMP_EN_MASK                               = 0x00000002,  /*!<  IF1 Configuration Register Mask Value */
     CFGREG_IF1_CONF_EN_MASK                                      = 0x00000001,  /*!<  IF1 Configuration Register Mask Value */

     CFGREG_IF25_CONF_IF_BANDCUT_MASK                             = 0x00004000,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_IF25_CONF_OFFSET_CAL_Q_MASK                           = 0x00003C00,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_IF25_CONF_OFFSET_CAL_DIRECTION_Q_MASK                 = 0x00000200,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_IF25_CONF_OFFSET_CAL_I_MASK                           = 0x000001E0,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_IF25_CONF_OFFSET_CAL_DIRECTION_I_MASK                 = 0x00000010,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_IF25_CONF_PGA2_EN_MASK                                = 0x00000008,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_IF25_CONF_PGA1_EN_MASK                                = 0x00000004,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_IF25_CONF_PREAMP_EN_MASK                              = 0x00000002,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_IF25_CONF_EN_MASK                                     = 0x00000001,  /*!<  IF25 Configuration Register Mask Value */

     CFGREG_PROBE_CONF_PLL15FB_SEL_MASK                           = 0x00008000,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_PROBE_CONF_PLL15FB_EN_MASK                            = 0x00004000,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_PROBE_CONF_IF25_PROBEB_SELECT_MASK                    = 0x00003800,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_PROBE_CONF_IF25_PROBEB_EN_MASK                        = 0x00000400,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_PROBE_CONF_IF25_PROBEA_SELECT_MASK                    = 0x00000380,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_PROBE_CONF_IF1_PROBEB_SELECT_MASK                     = 0x00000070,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_PROBE_CONF_IF1_PROBEB_EN_MASK                         = 0x00000008,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_PROBE_CONF_IF1_PROBEA_SELECT_MASK                     = 0x00000007,  /*!<  IF25 Configuration Register Mask Value */

     CFGREG_ADC1_CONF_CLK_SEL_MASK                                = 0x000000C0,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_ADC1_CONF_CLK_CONF_MASK                               = 0x00000030,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_ADC1_CONF_SAH_IBIAS_CTRL_MASK                         = 0x00000008,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_ADC1_CONF_CAL_EN_MASK                                 = 0x00000004,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_ADC1_CONF_SAH_EN_MASK                                 = 0x00000002,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_ADC1_CONF_ADC_EN_MASK                                 = 0x00000001,  /*!<  IF25 Configuration Register Mask Value */

     CFGREG_ADC25_CONF_CLK_SEL_MASK                               = 0x000000C0,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_ADC25_CONF_CLK_CONF_MASK                              = 0x00000030,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_ADC25_CONF_SAH_IBIAS_CTRL_MASK                        = 0x00000008,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_ADC25_CONF_CAL_EN_MASK                                = 0x00000004,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_ADC25_CONF_SAH_EN_MASK                                = 0x00000002,  /*!<  IF25 Configuration Register Mask Value */
     CFGREG_ADC25_CONF_ADC_EN_MASK                                = 0x00000001,  /*!<  IF25 Configuration Register Mask Value */

     CFGREG_SPLIT1_CTRL_CLEAR_MASK                                = 0x00000001,  /*!<  Splitter AFE1 Control Register Mask Value */

     CFGREG_SPLIT1_CFG_GAIN_CHANGE_CNT_MASK                       = 0x000FFFFF,  /*!<  Splitter AFE1 Gain Change Configuration Register Mask Value */

     CFGREG_SPLIT1_CFG_LIMITER_THRESHOLD_MASK                     = 0x000FFFFF,  /*!<  Splitter AFE1 Limiter Threshold Configuration Register Mask Value */

     CFGREG_SPLIT1_CFG_MANUAL_MODE_PGA2_GAIN_MASK                 = 0x00000004,  /*!<  Splitter AFE1 Manual Mode Configuration Register Mask Value */
     CFGREG_SPLIT1_CFG_MANUAL_MODE_PGA1_GAIN_MASK                 = 0x00000002,  /*!<  Splitter AFE1 Manual Mode Configuration Register Mask Value */
     CFGREG_SPLIT1_CFG_MANUAL_MODE_EN_MASK                        = 0x00000001,  /*!<  Splitter AFE1 Manual Mode Configuration Register Mask Value */

     CFGREG_SPLIT1_L1E1_CFG_DEC_BY_THREE_EN_MASK                  = 0x00000400,  /*!<  Splitter AFE1 L1E1 Configuration Register Mask Value */
     CFGREG_SPLIT1_L1E1_CFG_DEC_SAMPLES_SHIFT_MASK                = 0x00000380,  /*!<  Splitter AFE1 L1E1 Configuration Register Mask Value */
     CFGREG_SPLIT1_L1E1_CFG_CARR_MODE_MASK                        = 0x00000060,  /*!<  Splitter AFE1 L1E1 Configuration Register Mask Value */
     CFGREG_SPLIT1_L1E1_CFG_CARR_ENABLE_MASK                      = 0x00000010,  /*!<  Splitter AFE1 L1E1 Configuration Register Mask Value */
     CFGREG_SPLIT1_L1E1_CFG_LPF_K_PARAM_MASK                      = 0x0000000E,  /*!<  Splitter AFE1 L1E1 Configuration Register Mask Value */
     CFGREG_SPLIT1_L1E1_CFG_IQ_SAME_GAIN_MASK                     = 0x00000001,  /*!<  Splitter AFE1 L1E1 Configuration Register Mask Value */

     CFGREG_SPLIT1_L1E1_CFG_CARR_FREQ_STEP_MASK                   = 0xFFFFFFFF,  /*!<  Splitter AFE1 L1E1 Carrier Frequency Configuration Register Mask Value */

     CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_MAX_MASK                  = 0x0FFFC000,  /*!<  Splitter AFE1 L1E1 Threshold I Configuration Register Mask Value */
     CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_MIN_MASK                  = 0x00003FFF,  /*!<  Splitter AFE1 L1E1 Threshold I Configuration Register Mask Value */

     CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_MAX_MASK                  = 0x0FFFC000,  /*!<  Splitter AFE1 L1E1 Threshold Q Configuration Register Mask Value */
     CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_MIN_MASK                  = 0x00003FFF,  /*!<  Splitter AFE1 L1E1 Threshold Q Configuration Register Mask Value */

     CFGREG_SPLIT1_L1E1_CFG_MANUAL_MODE_DIG_GAIN_Q_MASK           = 0x00000FC0,  /*!<  Splitter AFE1 L1E1 Manual Mode Digital Gain Configuration Register Mask Value */
     CFGREG_SPLIT1_L1E1_CFG_MANUAL_MODE_DIG_GAIN_I_MASK           = 0x0000003F,  /*!<  Splitter AFE1 L1E1 Manual Mode Digital Gain Configuration Register Mask Value */

     CFGREG_SPLIT25_CTRL_CLEAR_MASK                               = 0x00000001,  /*!<  Splitter AFE25 Control Register Mask Value */

     CFGREG_SPLIT25_CFG_GAIN_CHANGE_CNT_MASK                      = 0x000FFFFF,  /*!<  Splitter AFE25 Gain Change Configuration Register Mask Value */

     CFGREG_SPLIT25_CFG_LIMITER_THRESHOLD_MASK                    = 0x000FFFFF,  /*!<  Splitter AFE25 Limiter Threshold Configuration Register Mask Value */

     CFGREG_SPLIT25_CFG_MANUAL_MODE_PGA2_GAIN_MASK                = 0x00000004,  /*!<  Splitter AFE25 Manual Mode Configuration Register Mask Value */
     CFGREG_SPLIT25_CFG_MANUAL_MODE_PGA1_GAIN_MASK                = 0x00000002,  /*!<  Splitter AFE25 Manual Mode Configuration Register Mask Value */
     CFGREG_SPLIT25_CFG_MANUAL_MODE_EN_MASK                       = 0x00000001,  /*!<  Splitter AFE25 Manual Mode Configuration Register Mask Value */

     CFGREG_SPLIT25_L2_CFG_DEC_BY_THREE_EN_MASK                   = 0x00000400,  /*!<  Splitter AFE25 L2 Configuration Register Mask Value */
     CFGREG_SPLIT25_L2_CFG_DEC_SAMPLES_SHIFT_MASK                 = 0x00000380,  /*!<  Splitter AFE25 L2 Configuration Register Mask Value */
     CFGREG_SPLIT25_L2_CFG_CARR_MODE_MASK                         = 0x00000060,  /*!<  Splitter AFE25 L2 Configuration Register Mask Value */
     CFGREG_SPLIT25_L2_CFG_CARR_ENABLE_MASK                       = 0x00000010,  /*!<  Splitter AFE25 L2 Configuration Register Mask Value */
     CFGREG_SPLIT25_L2_CFG_LPF_K_PARAM_MASK                       = 0x0000000E,  /*!<  Splitter AFE25 L2 Configuration Register Mask Value */
     CFGREG_SPLIT25_L2_CFG_IQ_SAME_GAIN_MASK                      = 0x00000001,  /*!<  Splitter AFE25 L2 Configuration Register Mask Value */

     CFGREG_SPLIT25_L2_CFG_CARR_FREQ_STEP_MASK                    = 0xFFFFFFFF,  /*!<  Splitter AFE25 L2 Carrier Frequency Configuration Register Mask Value */

     CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_MAX_MASK                   = 0x0FFFC000,  /*!<  Splitter AFE25 L2 Threshold I Configuration Register Mask Value */
     CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_MIN_MASK                   = 0x00003FFF,  /*!<  Splitter AFE25 L2 Threshold I Configuration Register Mask Value */

     CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_MAX_MASK                   = 0x0FFFC000,  /*!<  Splitter AFE25 L2 Threshold Q Configuration Register Mask Value */
     CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_MIN_MASK                   = 0x00003FFF,  /*!<  Splitter AFE25 L2 Threshold Q Configuration Register Mask Value */

     CFGREG_SPLIT25_L2_CFG_MANUAL_MODE_DIG_GAIN_Q_MASK            = 0x00000FC0,  /*!<  Splitter AFE25 L2 Manual Mode Digital Gain Configuration Register Mask Value */
     CFGREG_SPLIT25_L2_CFG_MANUAL_MODE_DIG_GAIN_I_MASK            = 0x0000003F,  /*!<  Splitter AFE25 L2 Manual Mode Digital Gain Configuration Register Mask Value */

     CFGREG_SPLIT25_E6_CFG_DEC_BY_THREE_EN_MASK                   = 0x00000400,  /*!<  Splitter AFE25 E6 Configuration Register Mask Value */
     CFGREG_SPLIT25_E6_CFG_DEC_SAMPLES_SHIFT_MASK                 = 0x00000380,  /*!<  Splitter AFE25 E6 Configuration Register Mask Value */
     CFGREG_SPLIT25_E6_CFG_CARR_MODE_MASK                         = 0x00000060,  /*!<  Splitter AFE25 E6 Configuration Register Mask Value */
     CFGREG_SPLIT25_E6_CFG_CARR_ENABLE_MASK                       = 0x00000010,  /*!<  Splitter AFE25 E6 Configuration Register Mask Value */
     CFGREG_SPLIT25_E6_CFG_LPF_K_PARAM_MASK                       = 0x0000000E,  /*!<  Splitter AFE25 E6 Configuration Register Mask Value */
     CFGREG_SPLIT25_E6_CFG_IQ_SAME_GAIN_MASK                      = 0x00000001,  /*!<  Splitter AFE25 E6 Configuration Register Mask Value */

     CFGREG_SPLIT25_E6_CFG_CARR_FREQ_STEP_MASK                    = 0xFFFFFFFF,  /*!<  Splitter AFE25 E6 Carrier Frequency Configuration Register Mask Value */

     CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_MAX_MASK                   = 0x0FFFC000,  /*!<  Splitter AFE25 E6 Threshold I Configuration Register Mask Value */
     CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_MIN_MASK                   = 0x00003FFF,  /*!<  Splitter AFE25 E6 Threshold I Configuration Register Mask Value */

     CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_MAX_MASK                   = 0x0FFFC000,  /*!<  Splitter AFE25 E6 Threshold Q Configuration Register Mask Value */
     CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_MIN_MASK                   = 0x00003FFF,  /*!<  Splitter AFE25 E6 Threshold Q Configuration Register Mask Value */

     CFGREG_SPLIT25_E6_CFG_MANUAL_MODE_DIG_GAIN_Q_MASK            = 0x00000FC0,  /*!<  Splitter AFE25 E6 Manual Mode Digital Gain Configuration Register Mask Value */
     CFGREG_SPLIT25_E6_CFG_MANUAL_MODE_DIG_GAIN_I_MASK            = 0x0000003F,  /*!<  Splitter AFE25 E6 Manual Mode Digital Gain Configuration Register Mask Value */

     CFGREG_SPLIT25_L5E5A_CFG_DEC_BY_THREE_EN_MASK                = 0x00000400,  /*!<  Splitter AFE25 L5E5A Configuration Register Mask Value */
     CFGREG_SPLIT25_L5E5A_CFG_DEC_SAMPLES_SHIFT_MASK              = 0x00000380,  /*!<  Splitter AFE25 L5E5A Configuration Register Mask Value */
     CFGREG_SPLIT25_L5E5A_CFG_CARR_MODE_MASK                      = 0x00000060,  /*!<  Splitter AFE25 L5E5A Configuration Register Mask Value */
     CFGREG_SPLIT25_L5E5A_CFG_CARR_ENABLE_MASK                    = 0x00000010,  /*!<  Splitter AFE25 L5E5A Configuration Register Mask Value */
     CFGREG_SPLIT25_L5E5A_CFG_LPF_K_PARAM_MASK                    = 0x0000000E,  /*!<  Splitter AFE25 L5E5A Configuration Register Mask Value */
     CFGREG_SPLIT25_L5E5A_CFG_IQ_SAME_GAIN_MASK                   = 0x00000001,  /*!<  Splitter AFE25 L5E5A Configuration Register Mask Value */

     CFGREG_SPLIT25_L5E5A_CFG_CARR_FREQ_STEP_MASK                 = 0xFFFFFFFF,  /*!<  Splitter AFE25 L5E5A Carrier Frequency Configuration Register Mask Value */

     CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_MAX_MASK                = 0x0FFFC000,  /*!<  Splitter AFE25 L5E5A Threshold I Configuration Register Mask Value */
     CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_MIN_MASK                = 0x00003FFF,  /*!<  Splitter AFE25 L5E5A Threshold I Configuration Register Mask Value */

     CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_MAX_MASK                = 0x0FFFC000,  /*!<  Splitter AFE25 L5E5A Threshold Q Configuration Register Mask Value */
     CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_MIN_MASK                = 0x00003FFF,  /*!<  Splitter AFE25 L5E5A Threshold Q Configuration Register Mask Value */

     CFGREG_SPLIT25_L5E5A_CFG_MANUAL_MODE_DIG_GAIN_Q_MASK         = 0x00000FC0,  /*!<  Splitter AFE25 L5E5A Manual Mode Digital Gain Configuration Register Mask Value */
     CFGREG_SPLIT25_L5E5A_CFG_MANUAL_MODE_DIG_GAIN_I_MASK         = 0x0000003F,  /*!<  Splitter AFE25 L5E5A Manual Mode Digital Gain Configuration Register Mask Value */

     CFGREG_SPLIT25_E5B_CFG_DEC_BY_THREE_EN_MASK                  = 0x00000400,  /*!<  Splitter AFE25 E5B Configuration Register Mask Value */
     CFGREG_SPLIT25_E5B_CFG_DEC_SAMPLES_SHIFT_MASK                = 0x00000380,  /*!<  Splitter AFE25 E5B Configuration Register Mask Value */
     CFGREG_SPLIT25_E5B_CFG_CARR_MODE_MASK                        = 0x00000060,  /*!<  Splitter AFE25 E5B Configuration Register Mask Value */
     CFGREG_SPLIT25_E5B_CFG_CARR_ENABLE_MASK                      = 0x00000010,  /*!<  Splitter AFE25 E5B Configuration Register Mask Value */
     CFGREG_SPLIT25_E5B_CFG_LPF_K_PARAM_MASK                      = 0x0000000E,  /*!<  Splitter AFE25 E5B Configuration Register Mask Value */
     CFGREG_SPLIT25_E5B_CFG_IQ_SAME_GAIN_MASK                     = 0x00000001,  /*!<  Splitter AFE25 E5B Configuration Register Mask Value */

     CFGREG_SPLIT25_E5B_CFG_CARR_FREQ_STEP_MASK                   = 0xFFFFFFFF,  /*!<  Splitter AFE25 E5B Carrier Frequency Configuration Register Mask Value */

     CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_MAX_MASK                  = 0x0FFFC000,  /*!<  Splitter AFE25 E5B Threshold I Configuration Register Mask Value */
     CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_MIN_MASK                  = 0x00003FFF,  /*!<  Splitter AFE25 E5B Threshold I Configuration Register Mask Value */

     CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_MAX_MASK                  = 0x0FFFC000,  /*!<  Splitter AFE25 E5B Threshold Q Configuration Register Mask Value */
     CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_MIN_MASK                  = 0x00003FFF,  /*!<  Splitter AFE25 E5B Threshold Q Configuration Register Mask Value */

     CFGREG_SPLIT25_E5B_CFG_MANUAL_MODE_DIG_GAIN_Q_MASK           = 0x00000FC0,  /*!<  Splitter AFE25 E5B Manual Mode Digital Gain Configuration Register Mask Value */
     CFGREG_SPLIT25_E5B_CFG_MANUAL_MODE_DIG_GAIN_I_MASK           = 0x0000003F,  /*!<  Splitter AFE25 E5B Manual Mode Digital Gain Configuration Register Mask Value */

     CFGREG_LNA125_CONF_L25_TUNE_SRC_MASK                         = 0x00000008,  /*!<  LNA125 Configuration Register Mask Value */
     CFGREG_LNA125_CONF_L1_TUNE_SRC_MASK                          = 0x00000004,  /*!<  LNA125 Configuration Register Mask Value */
     CFGREG_LNA125_CONF_EN_L25_MASK                               = 0x00000002,  /*!<  LNA125 Configuration Register Mask Value */
     CFGREG_LNA125_CONF_EN_L1_MASK                                = 0x00000001,  /*!<  LNA125 Configuration Register Mask Value */

     CFGREG_LNA125_TUNE_CONF_L25_MASK                             = 0xFFFF0000,  /*!<  LNA125 Configuration Register Mask Value */
     CFGREG_LNA125_TUNE_CONF_L1_MASK                              = 0x0000FFFF,  /*!<  LNA125 Configuration Register Mask Value */

     CFGREG_BALUN_MIXER1_CONF_BALUN_TUNE_SRC_MASK                 = 0x00040000,  /*!<  BALUN and MIXER 1 Configuration Register Mask Value */
     CFGREG_BALUN_MIXER1_CONF_BALUN_TUNE_MASK                     = 0x0003FFFC,  /*!<  BALUN and MIXER 1 Configuration Register Mask Value */
     CFGREG_BALUN_MIXER1_CONF_BALUN_EN_MASK                       = 0x00000002,  /*!<  BALUN and MIXER 1 Configuration Register Mask Value */
     CFGREG_BALUN_MIXER1_CONF_MIXER_EN_MASK                       = 0x00000001,  /*!<  BALUN and MIXER 1 Configuration Register Mask Value */

     CFGREG_BALUN_MIXER25_CONF_BALUN_TUNE_SRC_MASK                = 0x00040000,  /*!<  BALUN and MIXER 25 Configuration Register Mask Value */
     CFGREG_BALUN_MIXER25_CONF_BALUN_TUNE_MASK                    = 0x0003FFFC,  /*!<  BALUN and MIXER 25 Configuration Register Mask Value */
     CFGREG_BALUN_MIXER25_CONF_BALUN_EN_MASK                      = 0x00000002,  /*!<  BALUN and MIXER 25 Configuration Register Mask Value */
     CFGREG_BALUN_MIXER25_CONF_MIXER_EN_MASK                      = 0x00000001,  /*!<  BALUN and MIXER 25 Configuration Register Mask Value */

     CFGREG_PMU_CONF_FLASH_POR_EN_MASK                            = 0x00040000,  /*!<  PMU Configuration register Mask Value */
     CFGREG_PMU_CONF_FLASH_BOD_EN_MASK                            = 0x00020000,  /*!<  PMU Configuration register Mask Value */
     CFGREG_PMU_CONF_DCDC_MODE_MASK                               = 0x00010000,  /*!<  PMU Configuration register Mask Value */
     CFGREG_PMU_CONF_DCDC_GENTRIM_SRC_MASK                        = 0x00008000,  /*!<  PMU Configuration register Mask Value */
     CFGREG_PMU_CONF_DCDC_GENTRIM_MASK                            = 0x00007000,  /*!<  PMU Configuration register Mask Value */
     CFGREG_PMU_CONF_DCDC_TRIM_RESET_MASK                         = 0x00000800,  /*!<  PMU Configuration register Mask Value */
     CFGREG_PMU_CONF_IREF_CAL_EN_MASK                             = 0x00000400,  /*!<  PMU Configuration register Mask Value */
     CFGREG_PMU_CONF_IREF_TRIM_VAL_SRC_MASK                       = 0x00000200,  /*!<  PMU Configuration register Mask Value */
     CFGREG_PMU_CONF_IREF_TRIM_VAL_MASK                           = 0x000001FE,  /*!<  PMU Configuration register Mask Value */
     CFGREG_PMU_CONF_IREF_TRIM_EN_MASK                            = 0x00000001,  /*!<  PMU Configuration register Mask Value */

     CFGREG_PMU_STAT_IREF_TRIM_VAL_MASK                           = 0x000001FE,  /*!<  Power Management Unit Status Mask Value */
     CFGREG_PMU_STAT_IREF_CAL_RDY_MASK                            = 0x00000001,  /*!<  Power Management Unit Status Mask Value */

     CFGREG_PM_STAT_IREF_COMP_MASK                                = 0x00000008,  /*!<  GNSS-AFE PM Status Register Mask Value */
     CFGREG_PM_STAT_VREF_COMP_MASK                                = 0x00000004,  /*!<  GNSS-AFE PM Status Register Mask Value */
     CFGREG_PM_STAT_PWR_UP_P_MASK                                 = 0x00000002,  /*!<  GNSS-AFE PM Status Register Mask Value */
     CFGREG_PM_STAT_PWR_UP_N_MASK                                 = 0x00000001,  /*!<  GNSS-AFE PM Status Register Mask Value */

     CFGREG_PM_CONF_LDO_ADC_EXT_DECAP_MASK                        = 0x04000000,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_LDO_IF_EXT_DECAP_MASK                         = 0x02000000,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_LDO_DPLL_EXT_DECAP_MASK                       = 0x01000000,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_LDO_APLL_EXT_DECAP_MASK                       = 0x00800000,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_LDO_RF_EXT_DECAP_MASK                         = 0x00400000,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_RC_FILTER_MASK                                = 0x00200000,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_CAL_EN_MASK                                   = 0x00100000,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_TEST_EN_MASK                                  = 0x00080000,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_LDO_ADC_EN_MASK                               = 0x00040000,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_LDO_IF_EN_MASK                                = 0x00020000,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_LDO_DPLL_EN_MASK                              = 0x00010000,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_LDO_APLL_EN_MASK                              = 0x00008000,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_LDO_RF_EN_MASK                                = 0x00004000,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_VREF_EN_MASK                                  = 0x00002000,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_IREF_TRIM_SRC_MASK                            = 0x00001000,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_IREF_TRIM_MASK                                = 0x00000F00,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_IREF_EN_MASK                                  = 0x00000080,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_BGVR_TRIM_SRC_MASK                            = 0x00000040,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_BGVR_TRIM_MASK                                = 0x0000003C,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_BGVR_EN_MASK                                  = 0x00000002,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_PM_CONF_NRST_MASK                                     = 0x00000001,  /*!<  GNSS-AFE PM Configuration Register Mask Value */

     CFGREG_GNSSAFE_TEST_CONF_PLLTEST_ATT_MASK                    = 0x00000F00,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_EN_MASK               = 0x00000080,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_DIR_MASK              = 0x00000040,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_PRESC_MASK            = 0x00000030,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_EN_MASK                = 0x00000008,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_DIR_MASK               = 0x00000004,  /*!<  GNSS-AFE PM Configuration Register Mask Value */
     CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_PRESC_MASK             = 0x00000003,  /*!<  GNSS-AFE PM Configuration Register Mask Value */

     CFGREG_SPARE_CONF_SIG_MASK                                   = 0xFFFFFFFF,  /*!<  SPARE Configuration Register Mask Value */

     CFGREG_SPARE_STAT_SIG_MASK                                   = 0xFFFFFFFF,  /*!<  SPARE Configuration Register Mask Value */

};

/** Registers Shifts */
enum
{

     CFGREG_IRQMAP_IRQMAP_SHIFT                                   =  1,  /*!<  Interrupt Mapping Register Shift Value */

     CFGREG_UNLOCK_UNLOCK_KEY_SHIFT                               =  0,  /*!<  Unlock Register Shift Value */

     CFGREG_IRQFLAGS_PLL_FAIL_SHIFT                               =  2,  /*!<  32kHz RTC Clock Interrupt Flags Shift Value */
     CFGREG_IRQFLAGS_XTAL_CORE_FAIL_SHIFT                         =  1,  /*!<  32kHz RTC Clock Interrupt Flags Shift Value */
     CFGREG_IRQFLAGS_XTAL_RTC_FAIL_SHIFT                          =  0,  /*!<  32kHz RTC Clock Interrupt Flags Shift Value */

     CFGREG_MEM_CONF_TEST1_SHIFT                                  =  5,  /*!<  Memory Configuration Register Shift Value */
     CFGREG_MEM_CONF_RME_SHIFT                                    =  4,  /*!<  Memory Configuration Register Shift Value */
     CFGREG_MEM_CONF_RM_SHIFT                                     =  0,  /*!<  Memory Configuration Register Shift Value */

     CFGREG_RTCSTAT_RC_RTC_VAL_SHIFT                              =  2,  /*!<  32kHz RTC Clock Status Shift Value */
     CFGREG_RTCSTAT_RC_RTC_RDY_SHIFT                              =  1,  /*!<  32kHz RTC Clock Status Shift Value */
     CFGREG_RTCSTAT_CLK_RTC_SEL_SHIFT                             =  0,  /*!<  32kHz RTC Clock Status Shift Value */

     CFGREG_RTCCONF_RC_RTC_VAL_SRC_SHIFT                          = 10,  /*!<  32kHz RTC Clock Configuration Shift Value */
     CFGREG_RTCCONF_RC_RTC_VAL_SHIFT                              =  3,  /*!<  32kHz RTC Clock Configuration Shift Value */
     CFGREG_RTCCONF_RC_RTC_CAL_SHIFT                              =  2,  /*!<  32kHz RTC Clock Configuration Shift Value */
     CFGREG_RTCCONF_XTAL_RTC_SEL_SHIFT                            =  1,  /*!<  32kHz RTC Clock Configuration Shift Value */
     CFGREG_RTCCONF_XTAL_RTC_TEST_SHIFT                           =  0,  /*!<  32kHz RTC Clock Configuration Shift Value */

     CFGREG_COREFREQ_CLK_TEST_OUT_EN_SHIFT                        = 11,  /*!<  External XTAL Configuration Register Shift Value */
     CFGREG_COREFREQ_CLK_RC_CORE_VAL_SRC_SHIFT                    = 10,  /*!<  External XTAL Configuration Register Shift Value */
     CFGREG_COREFREQ_CLK_RC_CORE_VAL_SHIFT                        =  4,  /*!<  External XTAL Configuration Register Shift Value */
     CFGREG_COREFREQ_CLK_RC_CORE_CAL_SHIFT                        =  3,  /*!<  External XTAL Configuration Register Shift Value */
     CFGREG_COREFREQ_CLK_CORE_SEL_SHIFT                           =  1,  /*!<  External XTAL Configuration Register Shift Value */
     CFGREG_COREFREQ_CLK_XTAL_CORE_TEST_SHIFT                     =  0,  /*!<  External XTAL Configuration Register Shift Value */

     CFGREG_COREFREQ_PLL_CTRL_FINE_LOAD_SHIFT                     = 17,  /*!<  PLL Configuration Register Shift Value */
     CFGREG_COREFREQ_PLL_CTRL_LOAD_SHIFT                          = 16,  /*!<  PLL Configuration Register Shift Value */
     CFGREG_COREFREQ_PLL_CTRL_PVT_LOAD_SHIFT                      = 10,  /*!<  PLL Configuration Register Shift Value */
     CFGREG_COREFREQ_PLL_PLL_VFB_EN_SHIFT                         =  9,  /*!<  PLL Configuration Register Shift Value */
     CFGREG_COREFREQ_PLL_TEST_SHIFT                               =  8,  /*!<  PLL Configuration Register Shift Value */
     CFGREG_COREFREQ_PLL_N_SHIFT                                  =  2,  /*!<  PLL Configuration Register Shift Value */
     CFGREG_COREFREQ_PLL_REF_SEL_SHIFT                            =  1,  /*!<  PLL Configuration Register Shift Value */
     CFGREG_COREFREQ_PLL_EN_SHIFT                                 =  0,  /*!<  PLL Configuration Register Shift Value */

     CFGREG_COREFREQ_STAT_CTRL_FINE_SHIFT                         = 17,  /*!<  Core Clock Status Register Shift Value */
     CFGREG_COREFREQ_STAT_CTRL_PVT_SHIFT                          = 11,  /*!<  Core Clock Status Register Shift Value */
     CFGREG_COREFREQ_STAT_RC_CORE_VAL_SHIFT                       =  5,  /*!<  Core Clock Status Register Shift Value */
     CFGREG_COREFREQ_STAT_RC_CORE_RDY_SHIFT                       =  4,  /*!<  Core Clock Status Register Shift Value */
     CFGREG_COREFREQ_STAT_PLL_LOCK_SHIFT                          =  3,  /*!<  Core Clock Status Register Shift Value */
     CFGREG_COREFREQ_STAT_XTAL_CORE_LOCK_SHIFT                    =  2,  /*!<  Core Clock Status Register Shift Value */
     CFGREG_COREFREQ_STAT_CLK_CORE_SEL_SHIFT                      =  0,  /*!<  Core Clock Status Register Shift Value */

     CFGREG_GNSSAFE_CONF_GNSS_OUT_BAND_SHIFT                      =  2,  /*!<  GNSS-AFE Configuration Register Shift Value */
     CFGREG_GNSSAFE_CONF_GNSS_AUX1_EN_SHIFT                       =  1,  /*!<  GNSS-AFE Configuration Register Shift Value */
     CFGREG_GNSSAFE_CONF_GNSS_AUX0_EN_SHIFT                       =  0,  /*!<  GNSS-AFE Configuration Register Shift Value */

     CFGREG_PLL1_CONF_EN_BOOST_SHIFT                              = 27,  /*!<  PLL1 Configuration Register Shift Value */
     CFGREG_PLL1_CONF_ADC_CLK_DIV_SHIFT                           = 26,  /*!<  PLL1 Configuration Register Shift Value */
     CFGREG_PLL1_CONF_LOPCB_EN_SHIFT                              = 25,  /*!<  PLL1 Configuration Register Shift Value */
     CFGREG_PLL1_CONF_TEST_EN_SHIFT                               = 24,  /*!<  PLL1 Configuration Register Shift Value */
     CFGREG_PLL1_CONF_FCW_SHIFT                                   =  1,  /*!<  PLL1 Configuration Register Shift Value */
     CFGREG_PLL1_CONF_EN_SHIFT                                    =  0,  /*!<  PLL1 Configuration Register Shift Value */

     CFGREG_PLL1DCO_CONF_AMP_SHIFT                                = 22,  /*!<  PLL1 DCO Configuration Register Shift Value */
     CFGREG_PLL1DCO_CONF_AMP_LOAD_SHIFT                           = 21,  /*!<  PLL1 DCO Configuration Register Shift Value */
     CFGREG_PLL1DCO_CONF_CTRL_PVT_SHIFT                           = 16,  /*!<  PLL1 DCO Configuration Register Shift Value */
     CFGREG_PLL1DCO_CONF_CTRL_FINE_SHIFT                          =  1,  /*!<  PLL1 DCO Configuration Register Shift Value */
     CFGREG_PLL1DCO_CONF_CTRL_LOAD_SHIFT                          =  0,  /*!<  PLL1 DCO Configuration Register Shift Value */

     CFGREG_PLL1_STAT_PH_VAR_SHIFT                                = 21,  /*!<  PLL1 Status Register Shift Value */
     CFGREG_PLL1_STAT_PHERR_SHIFT                                 =  1,  /*!<  PLL1 Status Register Shift Value */
     CFGREG_PLL1_STAT_LOCK_SHIFT                                  =  0,  /*!<  PLL1 Status Register Shift Value */

     CFGREG_PLL1_STAT2_TV_SHIFT                                   =  0,  /*!<  PLL1 Status Register Shift Value */

     CFGREG_PLL1DCO_STAT_AMP_SHIFT                                = 20,  /*!<  PLL1 TDC Status Register Shift Value */
     CFGREG_PLL1DCO_STAT_AMP_LOW_SHIFT                            = 19,  /*!<  PLL1 TDC Status Register Shift Value */
     CFGREG_PLL1DCO_STAT_CTRL_PVT_SHIFT                           = 14,  /*!<  PLL1 TDC Status Register Shift Value */
     CFGREG_PLL1DCO_STAT_CTRL_FINE_SHIFT                          =  0,  /*!<  PLL1 TDC Status Register Shift Value */

     CFGREG_PLL25_CONF_EN_BOOST_SHIFT                             = 27,  /*!<  PLL25 Configuration Register Shift Value */
     CFGREG_PLL25_CONF_ADC_CLK_DIV_SHIFT                          = 26,  /*!<  PLL25 Configuration Register Shift Value */
     CFGREG_PLL25_CONF_LOPCB_EN_SHIFT                             = 25,  /*!<  PLL25 Configuration Register Shift Value */
     CFGREG_PLL25_CONF_TEST_EN_SHIFT                              = 24,  /*!<  PLL25 Configuration Register Shift Value */
     CFGREG_PLL25_CONF_FCW_SHIFT                                  =  1,  /*!<  PLL25 Configuration Register Shift Value */
     CFGREG_PLL25_CONF_EN_SHIFT                                   =  0,  /*!<  PLL25 Configuration Register Shift Value */

     CFGREG_PLL25DCO_CONF_AMP_SHIFT                               = 23,  /*!<  PLL25 DCO Configuration Register Shift Value */
     CFGREG_PLL25DCO_CONF_AMP_LOAD_SHIFT                          = 22,  /*!<  PLL25 DCO Configuration Register Shift Value */
     CFGREG_PLL25DCO_CONF_CTRL_PVT_SHIFT                          = 16,  /*!<  PLL25 DCO Configuration Register Shift Value */
     CFGREG_PLL25DCO_CONF_CTRL_FINE_SHIFT                         =  1,  /*!<  PLL25 DCO Configuration Register Shift Value */
     CFGREG_PLL25DCO_CONF_CTRL_LOAD_SHIFT                         =  0,  /*!<  PLL25 DCO Configuration Register Shift Value */

     CFGREG_PLL25_STAT_PH_VAR_SHIFT                               = 21,  /*!<  PLL25 Status Register Shift Value */
     CFGREG_PLL25_STAT_PHERR_SHIFT                                =  1,  /*!<  PLL25 Status Register Shift Value */
     CFGREG_PLL25_STAT_LOCK_SHIFT                                 =  0,  /*!<  PLL25 Status Register Shift Value */

     CFGREG_PLL25_STAT2_TV_SHIFT                                  =  0,  /*!<  PLL25 Status Register Shift Value */

     CFGREG_PLL25DCO_STAT_AMP_SHIFT                               = 21,  /*!<  PLL25 TDC Status Register Shift Value */
     CFGREG_PLL25DCO_STAT_AMP_LOW_SHIFT                           = 20,  /*!<  PLL25 TDC Status Register Shift Value */
     CFGREG_PLL25DCO_STAT_CTRL_PVT_SHIFT                          = 14,  /*!<  PLL25 TDC Status Register Shift Value */
     CFGREG_PLL25DCO_STAT_CTRL_FINE_SHIFT                         =  0,  /*!<  PLL25 TDC Status Register Shift Value */

     CFGREG_IF1_CONF_IF_BANDCUT_SHIFT                             = 14,  /*!<  IF1 Configuration Register Shift Value */
     CFGREG_IF1_CONF_OFFSET_CAL_Q_SHIFT                           = 10,  /*!<  IF1 Configuration Register Shift Value */
     CFGREG_IF1_CONF_OFFSET_CAL_DIRECTION_Q_SHIFT                 =  9,  /*!<  IF1 Configuration Register Shift Value */
     CFGREG_IF1_CONF_OFFSET_CAL_I_SHIFT                           =  5,  /*!<  IF1 Configuration Register Shift Value */
     CFGREG_IF1_CONF_OFFSET_CAL_DIRECTION_I_SHIFT                 =  4,  /*!<  IF1 Configuration Register Shift Value */
     CFGREG_IF1_CONF_PGA2_EN_SHIFT                                =  3,  /*!<  IF1 Configuration Register Shift Value */
     CFGREG_IF1_CONF_PGA1_EN_SHIFT                                =  2,  /*!<  IF1 Configuration Register Shift Value */
     CFGREG_IF1_CONF_PREAMP_EN_SHIFT                              =  1,  /*!<  IF1 Configuration Register Shift Value */
     CFGREG_IF1_CONF_EN_SHIFT                                     =  0,  /*!<  IF1 Configuration Register Shift Value */

     CFGREG_IF25_CONF_IF_BANDCUT_SHIFT                            = 14,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_IF25_CONF_OFFSET_CAL_Q_SHIFT                          = 10,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_IF25_CONF_OFFSET_CAL_DIRECTION_Q_SHIFT                =  9,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_IF25_CONF_OFFSET_CAL_I_SHIFT                          =  5,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_IF25_CONF_OFFSET_CAL_DIRECTION_I_SHIFT                =  4,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_IF25_CONF_PGA2_EN_SHIFT                               =  3,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_IF25_CONF_PGA1_EN_SHIFT                               =  2,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_IF25_CONF_PREAMP_EN_SHIFT                             =  1,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_IF25_CONF_EN_SHIFT                                    =  0,  /*!<  IF25 Configuration Register Shift Value */

     CFGREG_PROBE_CONF_PLL15FB_SEL_SHIFT                          = 15,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_PROBE_CONF_PLL15FB_EN_SHIFT                           = 14,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_PROBE_CONF_IF25_PROBEB_SELECT_SHIFT                   = 11,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_PROBE_CONF_IF25_PROBEB_EN_SHIFT                       = 10,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_PROBE_CONF_IF25_PROBEA_SELECT_SHIFT                   =  7,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_PROBE_CONF_IF1_PROBEB_SELECT_SHIFT                    =  4,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_PROBE_CONF_IF1_PROBEB_EN_SHIFT                        =  3,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_PROBE_CONF_IF1_PROBEA_SELECT_SHIFT                    =  0,  /*!<  IF25 Configuration Register Shift Value */

     CFGREG_ADC1_CONF_CLK_SEL_SHIFT                               =  6,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_ADC1_CONF_CLK_CONF_SHIFT                              =  4,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_ADC1_CONF_SAH_IBIAS_CTRL_SHIFT                        =  3,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_ADC1_CONF_CAL_EN_SHIFT                                =  2,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_ADC1_CONF_SAH_EN_SHIFT                                =  1,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_ADC1_CONF_ADC_EN_SHIFT                                =  0,  /*!<  IF25 Configuration Register Shift Value */

     CFGREG_ADC25_CONF_CLK_SEL_SHIFT                              =  6,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_ADC25_CONF_CLK_CONF_SHIFT                             =  4,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_ADC25_CONF_SAH_IBIAS_CTRL_SHIFT                       =  3,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_ADC25_CONF_CAL_EN_SHIFT                               =  2,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_ADC25_CONF_SAH_EN_SHIFT                               =  1,  /*!<  IF25 Configuration Register Shift Value */
     CFGREG_ADC25_CONF_ADC_EN_SHIFT                               =  0,  /*!<  IF25 Configuration Register Shift Value */

     CFGREG_SPLIT1_CTRL_CLEAR_SHIFT                               =  0,  /*!<  Splitter AFE1 Control Register Shift Value */

     CFGREG_SPLIT1_CFG_GAIN_CHANGE_CNT_SHIFT                      =  0,  /*!<  Splitter AFE1 Gain Change Configuration Register Shift Value */

     CFGREG_SPLIT1_CFG_LIMITER_THRESHOLD_SHIFT                    =  0,  /*!<  Splitter AFE1 Limiter Threshold Configuration Register Shift Value */

     CFGREG_SPLIT1_CFG_MANUAL_MODE_PGA2_GAIN_SHIFT                =  2,  /*!<  Splitter AFE1 Manual Mode Configuration Register Shift Value */
     CFGREG_SPLIT1_CFG_MANUAL_MODE_PGA1_GAIN_SHIFT                =  1,  /*!<  Splitter AFE1 Manual Mode Configuration Register Shift Value */
     CFGREG_SPLIT1_CFG_MANUAL_MODE_EN_SHIFT                       =  0,  /*!<  Splitter AFE1 Manual Mode Configuration Register Shift Value */

     CFGREG_SPLIT1_L1E1_CFG_DEC_BY_THREE_EN_SHIFT                 = 10,  /*!<  Splitter AFE1 L1E1 Configuration Register Shift Value */
     CFGREG_SPLIT1_L1E1_CFG_DEC_SAMPLES_SHIFT_SHIFT               =  7,  /*!<  Splitter AFE1 L1E1 Configuration Register Shift Value */
     CFGREG_SPLIT1_L1E1_CFG_CARR_MODE_SHIFT                       =  5,  /*!<  Splitter AFE1 L1E1 Configuration Register Shift Value */
     CFGREG_SPLIT1_L1E1_CFG_CARR_ENABLE_SHIFT                     =  4,  /*!<  Splitter AFE1 L1E1 Configuration Register Shift Value */
     CFGREG_SPLIT1_L1E1_CFG_LPF_K_PARAM_SHIFT                     =  1,  /*!<  Splitter AFE1 L1E1 Configuration Register Shift Value */
     CFGREG_SPLIT1_L1E1_CFG_IQ_SAME_GAIN_SHIFT                    =  0,  /*!<  Splitter AFE1 L1E1 Configuration Register Shift Value */

     CFGREG_SPLIT1_L1E1_CFG_CARR_FREQ_STEP_SHIFT                  =  0,  /*!<  Splitter AFE1 L1E1 Carrier Frequency Configuration Register Shift Value */

     CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_MAX_SHIFT                 = 14,  /*!<  Splitter AFE1 L1E1 Threshold I Configuration Register Shift Value */
     CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_MIN_SHIFT                 =  0,  /*!<  Splitter AFE1 L1E1 Threshold I Configuration Register Shift Value */

     CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_MAX_SHIFT                 = 14,  /*!<  Splitter AFE1 L1E1 Threshold Q Configuration Register Shift Value */
     CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_MIN_SHIFT                 =  0,  /*!<  Splitter AFE1 L1E1 Threshold Q Configuration Register Shift Value */

     CFGREG_SPLIT1_L1E1_CFG_MANUAL_MODE_DIG_GAIN_Q_SHIFT          =  6,  /*!<  Splitter AFE1 L1E1 Manual Mode Digital Gain Configuration Register Shift Value */
     CFGREG_SPLIT1_L1E1_CFG_MANUAL_MODE_DIG_GAIN_I_SHIFT          =  0,  /*!<  Splitter AFE1 L1E1 Manual Mode Digital Gain Configuration Register Shift Value */

     CFGREG_SPLIT25_CTRL_CLEAR_SHIFT                              =  0,  /*!<  Splitter AFE25 Control Register Shift Value */

     CFGREG_SPLIT25_CFG_GAIN_CHANGE_CNT_SHIFT                     =  0,  /*!<  Splitter AFE25 Gain Change Configuration Register Shift Value */

     CFGREG_SPLIT25_CFG_LIMITER_THRESHOLD_SHIFT                   =  0,  /*!<  Splitter AFE25 Limiter Threshold Configuration Register Shift Value */

     CFGREG_SPLIT25_CFG_MANUAL_MODE_PGA2_GAIN_SHIFT               =  2,  /*!<  Splitter AFE25 Manual Mode Configuration Register Shift Value */
     CFGREG_SPLIT25_CFG_MANUAL_MODE_PGA1_GAIN_SHIFT               =  1,  /*!<  Splitter AFE25 Manual Mode Configuration Register Shift Value */
     CFGREG_SPLIT25_CFG_MANUAL_MODE_EN_SHIFT                      =  0,  /*!<  Splitter AFE25 Manual Mode Configuration Register Shift Value */

     CFGREG_SPLIT25_L2_CFG_DEC_BY_THREE_EN_SHIFT                  = 10,  /*!<  Splitter AFE25 L2 Configuration Register Shift Value */
     CFGREG_SPLIT25_L2_CFG_DEC_SAMPLES_SHIFT_SHIFT                =  7,  /*!<  Splitter AFE25 L2 Configuration Register Shift Value */
     CFGREG_SPLIT25_L2_CFG_CARR_MODE_SHIFT                        =  5,  /*!<  Splitter AFE25 L2 Configuration Register Shift Value */
     CFGREG_SPLIT25_L2_CFG_CARR_ENABLE_SHIFT                      =  4,  /*!<  Splitter AFE25 L2 Configuration Register Shift Value */
     CFGREG_SPLIT25_L2_CFG_LPF_K_PARAM_SHIFT                      =  1,  /*!<  Splitter AFE25 L2 Configuration Register Shift Value */
     CFGREG_SPLIT25_L2_CFG_IQ_SAME_GAIN_SHIFT                     =  0,  /*!<  Splitter AFE25 L2 Configuration Register Shift Value */

     CFGREG_SPLIT25_L2_CFG_CARR_FREQ_STEP_SHIFT                   =  0,  /*!<  Splitter AFE25 L2 Carrier Frequency Configuration Register Shift Value */

     CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_MAX_SHIFT                  = 14,  /*!<  Splitter AFE25 L2 Threshold I Configuration Register Shift Value */
     CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_MIN_SHIFT                  =  0,  /*!<  Splitter AFE25 L2 Threshold I Configuration Register Shift Value */

     CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_MAX_SHIFT                  = 14,  /*!<  Splitter AFE25 L2 Threshold Q Configuration Register Shift Value */
     CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_MIN_SHIFT                  =  0,  /*!<  Splitter AFE25 L2 Threshold Q Configuration Register Shift Value */

     CFGREG_SPLIT25_L2_CFG_MANUAL_MODE_DIG_GAIN_Q_SHIFT           =  6,  /*!<  Splitter AFE25 L2 Manual Mode Digital Gain Configuration Register Shift Value */
     CFGREG_SPLIT25_L2_CFG_MANUAL_MODE_DIG_GAIN_I_SHIFT           =  0,  /*!<  Splitter AFE25 L2 Manual Mode Digital Gain Configuration Register Shift Value */

     CFGREG_SPLIT25_E6_CFG_DEC_BY_THREE_EN_SHIFT                  = 10,  /*!<  Splitter AFE25 E6 Configuration Register Shift Value */
     CFGREG_SPLIT25_E6_CFG_DEC_SAMPLES_SHIFT_SHIFT                =  7,  /*!<  Splitter AFE25 E6 Configuration Register Shift Value */
     CFGREG_SPLIT25_E6_CFG_CARR_MODE_SHIFT                        =  5,  /*!<  Splitter AFE25 E6 Configuration Register Shift Value */
     CFGREG_SPLIT25_E6_CFG_CARR_ENABLE_SHIFT                      =  4,  /*!<  Splitter AFE25 E6 Configuration Register Shift Value */
     CFGREG_SPLIT25_E6_CFG_LPF_K_PARAM_SHIFT                      =  1,  /*!<  Splitter AFE25 E6 Configuration Register Shift Value */
     CFGREG_SPLIT25_E6_CFG_IQ_SAME_GAIN_SHIFT                     =  0,  /*!<  Splitter AFE25 E6 Configuration Register Shift Value */

     CFGREG_SPLIT25_E6_CFG_CARR_FREQ_STEP_SHIFT                   =  0,  /*!<  Splitter AFE25 E6 Carrier Frequency Configuration Register Shift Value */

     CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_MAX_SHIFT                  = 14,  /*!<  Splitter AFE25 E6 Threshold I Configuration Register Shift Value */
     CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_MIN_SHIFT                  =  0,  /*!<  Splitter AFE25 E6 Threshold I Configuration Register Shift Value */

     CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_MAX_SHIFT                  = 14,  /*!<  Splitter AFE25 E6 Threshold Q Configuration Register Shift Value */
     CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_MIN_SHIFT                  =  0,  /*!<  Splitter AFE25 E6 Threshold Q Configuration Register Shift Value */

     CFGREG_SPLIT25_E6_CFG_MANUAL_MODE_DIG_GAIN_Q_SHIFT           =  6,  /*!<  Splitter AFE25 E6 Manual Mode Digital Gain Configuration Register Shift Value */
     CFGREG_SPLIT25_E6_CFG_MANUAL_MODE_DIG_GAIN_I_SHIFT           =  0,  /*!<  Splitter AFE25 E6 Manual Mode Digital Gain Configuration Register Shift Value */

     CFGREG_SPLIT25_L5E5A_CFG_DEC_BY_THREE_EN_SHIFT               = 10,  /*!<  Splitter AFE25 L5E5A Configuration Register Shift Value */
     CFGREG_SPLIT25_L5E5A_CFG_DEC_SAMPLES_SHIFT_SHIFT             =  7,  /*!<  Splitter AFE25 L5E5A Configuration Register Shift Value */
     CFGREG_SPLIT25_L5E5A_CFG_CARR_MODE_SHIFT                     =  5,  /*!<  Splitter AFE25 L5E5A Configuration Register Shift Value */
     CFGREG_SPLIT25_L5E5A_CFG_CARR_ENABLE_SHIFT                   =  4,  /*!<  Splitter AFE25 L5E5A Configuration Register Shift Value */
     CFGREG_SPLIT25_L5E5A_CFG_LPF_K_PARAM_SHIFT                   =  1,  /*!<  Splitter AFE25 L5E5A Configuration Register Shift Value */
     CFGREG_SPLIT25_L5E5A_CFG_IQ_SAME_GAIN_SHIFT                  =  0,  /*!<  Splitter AFE25 L5E5A Configuration Register Shift Value */

     CFGREG_SPLIT25_L5E5A_CFG_CARR_FREQ_STEP_SHIFT                =  0,  /*!<  Splitter AFE25 L5E5A Carrier Frequency Configuration Register Shift Value */

     CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_MAX_SHIFT               = 14,  /*!<  Splitter AFE25 L5E5A Threshold I Configuration Register Shift Value */
     CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_MIN_SHIFT               =  0,  /*!<  Splitter AFE25 L5E5A Threshold I Configuration Register Shift Value */

     CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_MAX_SHIFT               = 14,  /*!<  Splitter AFE25 L5E5A Threshold Q Configuration Register Shift Value */
     CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_MIN_SHIFT               =  0,  /*!<  Splitter AFE25 L5E5A Threshold Q Configuration Register Shift Value */

     CFGREG_SPLIT25_L5E5A_CFG_MANUAL_MODE_DIG_GAIN_Q_SHIFT        =  6,  /*!<  Splitter AFE25 L5E5A Manual Mode Digital Gain Configuration Register Shift Value */
     CFGREG_SPLIT25_L5E5A_CFG_MANUAL_MODE_DIG_GAIN_I_SHIFT        =  0,  /*!<  Splitter AFE25 L5E5A Manual Mode Digital Gain Configuration Register Shift Value */

     CFGREG_SPLIT25_E5B_CFG_DEC_BY_THREE_EN_SHIFT                 = 10,  /*!<  Splitter AFE25 E5B Configuration Register Shift Value */
     CFGREG_SPLIT25_E5B_CFG_DEC_SAMPLES_SHIFT_SHIFT               =  7,  /*!<  Splitter AFE25 E5B Configuration Register Shift Value */
     CFGREG_SPLIT25_E5B_CFG_CARR_MODE_SHIFT                       =  5,  /*!<  Splitter AFE25 E5B Configuration Register Shift Value */
     CFGREG_SPLIT25_E5B_CFG_CARR_ENABLE_SHIFT                     =  4,  /*!<  Splitter AFE25 E5B Configuration Register Shift Value */
     CFGREG_SPLIT25_E5B_CFG_LPF_K_PARAM_SHIFT                     =  1,  /*!<  Splitter AFE25 E5B Configuration Register Shift Value */
     CFGREG_SPLIT25_E5B_CFG_IQ_SAME_GAIN_SHIFT                    =  0,  /*!<  Splitter AFE25 E5B Configuration Register Shift Value */

     CFGREG_SPLIT25_E5B_CFG_CARR_FREQ_STEP_SHIFT                  =  0,  /*!<  Splitter AFE25 E5B Carrier Frequency Configuration Register Shift Value */

     CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_MAX_SHIFT                 = 14,  /*!<  Splitter AFE25 E5B Threshold I Configuration Register Shift Value */
     CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_MIN_SHIFT                 =  0,  /*!<  Splitter AFE25 E5B Threshold I Configuration Register Shift Value */

     CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_MAX_SHIFT                 = 14,  /*!<  Splitter AFE25 E5B Threshold Q Configuration Register Shift Value */
     CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_MIN_SHIFT                 =  0,  /*!<  Splitter AFE25 E5B Threshold Q Configuration Register Shift Value */

     CFGREG_SPLIT25_E5B_CFG_MANUAL_MODE_DIG_GAIN_Q_SHIFT          =  6,  /*!<  Splitter AFE25 E5B Manual Mode Digital Gain Configuration Register Shift Value */
     CFGREG_SPLIT25_E5B_CFG_MANUAL_MODE_DIG_GAIN_I_SHIFT          =  0,  /*!<  Splitter AFE25 E5B Manual Mode Digital Gain Configuration Register Shift Value */

     CFGREG_LNA125_CONF_L25_TUNE_SRC_SHIFT                        =  3,  /*!<  LNA125 Configuration Register Shift Value */
     CFGREG_LNA125_CONF_L1_TUNE_SRC_SHIFT                         =  2,  /*!<  LNA125 Configuration Register Shift Value */
     CFGREG_LNA125_CONF_EN_L25_SHIFT                              =  1,  /*!<  LNA125 Configuration Register Shift Value */
     CFGREG_LNA125_CONF_EN_L1_SHIFT                               =  0,  /*!<  LNA125 Configuration Register Shift Value */

     CFGREG_LNA125_TUNE_CONF_L25_SHIFT                            = 16,  /*!<  LNA125 Configuration Register Shift Value */
     CFGREG_LNA125_TUNE_CONF_L1_SHIFT                             =  0,  /*!<  LNA125 Configuration Register Shift Value */

     CFGREG_BALUN_MIXER1_CONF_BALUN_TUNE_SRC_SHIFT                = 18,  /*!<  BALUN and MIXER 1 Configuration Register Shift Value */
     CFGREG_BALUN_MIXER1_CONF_BALUN_TUNE_SHIFT                    =  2,  /*!<  BALUN and MIXER 1 Configuration Register Shift Value */
     CFGREG_BALUN_MIXER1_CONF_BALUN_EN_SHIFT                      =  1,  /*!<  BALUN and MIXER 1 Configuration Register Shift Value */
     CFGREG_BALUN_MIXER1_CONF_MIXER_EN_SHIFT                      =  0,  /*!<  BALUN and MIXER 1 Configuration Register Shift Value */

     CFGREG_BALUN_MIXER25_CONF_BALUN_TUNE_SRC_SHIFT               = 18,  /*!<  BALUN and MIXER 25 Configuration Register Shift Value */
     CFGREG_BALUN_MIXER25_CONF_BALUN_TUNE_SHIFT                   =  2,  /*!<  BALUN and MIXER 25 Configuration Register Shift Value */
     CFGREG_BALUN_MIXER25_CONF_BALUN_EN_SHIFT                     =  1,  /*!<  BALUN and MIXER 25 Configuration Register Shift Value */
     CFGREG_BALUN_MIXER25_CONF_MIXER_EN_SHIFT                     =  0,  /*!<  BALUN and MIXER 25 Configuration Register Shift Value */

     CFGREG_PMU_CONF_FLASH_POR_EN_SHIFT                           = 18,  /*!<  PMU Configuration register Shift Value */
     CFGREG_PMU_CONF_FLASH_BOD_EN_SHIFT                           = 17,  /*!<  PMU Configuration register Shift Value */
     CFGREG_PMU_CONF_DCDC_MODE_SHIFT                              = 16,  /*!<  PMU Configuration register Shift Value */
     CFGREG_PMU_CONF_DCDC_GENTRIM_SRC_SHIFT                       = 15,  /*!<  PMU Configuration register Shift Value */
     CFGREG_PMU_CONF_DCDC_GENTRIM_SHIFT                           = 12,  /*!<  PMU Configuration register Shift Value */
     CFGREG_PMU_CONF_DCDC_TRIM_RESET_SHIFT                        = 11,  /*!<  PMU Configuration register Shift Value */
     CFGREG_PMU_CONF_IREF_CAL_EN_SHIFT                            = 10,  /*!<  PMU Configuration register Shift Value */
     CFGREG_PMU_CONF_IREF_TRIM_VAL_SRC_SHIFT                      =  9,  /*!<  PMU Configuration register Shift Value */
     CFGREG_PMU_CONF_IREF_TRIM_VAL_SHIFT                          =  1,  /*!<  PMU Configuration register Shift Value */
     CFGREG_PMU_CONF_IREF_TRIM_EN_SHIFT                           =  0,  /*!<  PMU Configuration register Shift Value */

     CFGREG_PMU_STAT_IREF_TRIM_VAL_SHIFT                          =  1,  /*!<  Power Management Unit Status Shift Value */
     CFGREG_PMU_STAT_IREF_CAL_RDY_SHIFT                           =  0,  /*!<  Power Management Unit Status Shift Value */

     CFGREG_PM_STAT_IREF_COMP_SHIFT                               =  3,  /*!<  GNSS-AFE PM Status Register Shift Value */
     CFGREG_PM_STAT_VREF_COMP_SHIFT                               =  2,  /*!<  GNSS-AFE PM Status Register Shift Value */
     CFGREG_PM_STAT_PWR_UP_P_SHIFT                                =  1,  /*!<  GNSS-AFE PM Status Register Shift Value */
     CFGREG_PM_STAT_PWR_UP_N_SHIFT                                =  0,  /*!<  GNSS-AFE PM Status Register Shift Value */

     CFGREG_PM_CONF_LDO_ADC_EXT_DECAP_SHIFT                       = 26,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_LDO_IF_EXT_DECAP_SHIFT                        = 25,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_LDO_DPLL_EXT_DECAP_SHIFT                      = 24,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_LDO_APLL_EXT_DECAP_SHIFT                      = 23,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_LDO_RF_EXT_DECAP_SHIFT                        = 22,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_RC_FILTER_SHIFT                               = 21,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_CAL_EN_SHIFT                                  = 20,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_TEST_EN_SHIFT                                 = 19,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_LDO_ADC_EN_SHIFT                              = 18,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_LDO_IF_EN_SHIFT                               = 17,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_LDO_DPLL_EN_SHIFT                             = 16,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_LDO_APLL_EN_SHIFT                             = 15,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_LDO_RF_EN_SHIFT                               = 14,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_VREF_EN_SHIFT                                 = 13,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_IREF_TRIM_SRC_SHIFT                           = 12,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_IREF_TRIM_SHIFT                               =  8,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_IREF_EN_SHIFT                                 =  7,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_BGVR_TRIM_SRC_SHIFT                           =  6,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_BGVR_TRIM_SHIFT                               =  2,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_BGVR_EN_SHIFT                                 =  1,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_PM_CONF_NRST_SHIFT                                    =  0,  /*!<  GNSS-AFE PM Configuration Register Shift Value */

     CFGREG_GNSSAFE_TEST_CONF_PLLTEST_ATT_SHIFT                   =  8,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_EN_SHIFT              =  7,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_DIR_SHIFT             =  6,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_PRESC_SHIFT           =  4,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_EN_SHIFT               =  3,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_DIR_SHIFT              =  2,  /*!<  GNSS-AFE PM Configuration Register Shift Value */
     CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_PRESC_SHIFT            =  0,  /*!<  GNSS-AFE PM Configuration Register Shift Value */

     CFGREG_SPARE_CONF_SIG_SHIFT                                  =  0,  /*!<  SPARE Configuration Register Shift Value */

     CFGREG_SPARE_STAT_SIG_SHIFT                                  =  0,  /*!<  SPARE Configuration Register Shift Value */

};

/** Bits Set Helper Macros */

#define CFGREG_IRQMAP_IRQMAP_BITS_SET(x)                                       ((x << CFGREG_IRQMAP_IRQMAP_SHIFT) & CFGREG_IRQMAP_IRQMAP_MASK)                                                          /*!<  Interrupt Mapping Register Bits Set */

#define CFGREG_UNLOCK_UNLOCK_KEY_BITS_SET(x)                                   ((x << CFGREG_UNLOCK_UNLOCK_KEY_SHIFT) & CFGREG_UNLOCK_UNLOCK_KEY_MASK)                                                  /*!<  Unlock Register Bits Set */

#define CFGREG_IRQFLAGS_PLL_FAIL_BITS_SET(x)                                   ((x << CFGREG_IRQFLAGS_PLL_FAIL_SHIFT) & CFGREG_IRQFLAGS_PLL_FAIL_MASK)                                                  /*!<  32kHz RTC Clock Interrupt Flags Bits Set */
#define CFGREG_IRQFLAGS_XTAL_CORE_FAIL_BITS_SET(x)                             ((x << CFGREG_IRQFLAGS_XTAL_CORE_FAIL_SHIFT) & CFGREG_IRQFLAGS_XTAL_CORE_FAIL_MASK)                                      /*!<  32kHz RTC Clock Interrupt Flags Bits Set */
#define CFGREG_IRQFLAGS_XTAL_RTC_FAIL_BITS_SET(x)                              ((x << CFGREG_IRQFLAGS_XTAL_RTC_FAIL_SHIFT) & CFGREG_IRQFLAGS_XTAL_RTC_FAIL_MASK)                                        /*!<  32kHz RTC Clock Interrupt Flags Bits Set */

#define CFGREG_MEM_CONF_TEST1_BITS_SET(x)                                      ((x << CFGREG_MEM_CONF_TEST1_SHIFT) & CFGREG_MEM_CONF_TEST1_MASK)                                                        /*!<  Memory Configuration Register Bits Set */
#define CFGREG_MEM_CONF_RME_BITS_SET(x)                                        ((x << CFGREG_MEM_CONF_RME_SHIFT) & CFGREG_MEM_CONF_RME_MASK)                                                            /*!<  Memory Configuration Register Bits Set */
#define CFGREG_MEM_CONF_RM_BITS_SET(x)                                         ((x << CFGREG_MEM_CONF_RM_SHIFT) & CFGREG_MEM_CONF_RM_MASK)                                                              /*!<  Memory Configuration Register Bits Set */

#define CFGREG_RTCSTAT_RC_RTC_VAL_BITS_SET(x)                                  ((x << CFGREG_RTCSTAT_RC_RTC_VAL_SHIFT) & CFGREG_RTCSTAT_RC_RTC_VAL_MASK)                                                /*!<  32kHz RTC Clock Status Bits Set */
#define CFGREG_RTCSTAT_RC_RTC_RDY_BITS_SET(x)                                  ((x << CFGREG_RTCSTAT_RC_RTC_RDY_SHIFT) & CFGREG_RTCSTAT_RC_RTC_RDY_MASK)                                                /*!<  32kHz RTC Clock Status Bits Set */
#define CFGREG_RTCSTAT_CLK_RTC_SEL_BITS_SET(x)                                 ((x << CFGREG_RTCSTAT_CLK_RTC_SEL_SHIFT) & CFGREG_RTCSTAT_CLK_RTC_SEL_MASK)                                              /*!<  32kHz RTC Clock Status Bits Set */

#define CFGREG_RTCCONF_RC_RTC_VAL_SRC_BITS_SET(x)                              ((x << CFGREG_RTCCONF_RC_RTC_VAL_SRC_SHIFT) & CFGREG_RTCCONF_RC_RTC_VAL_SRC_MASK)                                        /*!<  32kHz RTC Clock Configuration Bits Set */
#define CFGREG_RTCCONF_RC_RTC_VAL_BITS_SET(x)                                  ((x << CFGREG_RTCCONF_RC_RTC_VAL_SHIFT) & CFGREG_RTCCONF_RC_RTC_VAL_MASK)                                                /*!<  32kHz RTC Clock Configuration Bits Set */
#define CFGREG_RTCCONF_RC_RTC_CAL_BITS_SET(x)                                  ((x << CFGREG_RTCCONF_RC_RTC_CAL_SHIFT) & CFGREG_RTCCONF_RC_RTC_CAL_MASK)                                                /*!<  32kHz RTC Clock Configuration Bits Set */
#define CFGREG_RTCCONF_XTAL_RTC_SEL_BITS_SET(x)                                ((x << CFGREG_RTCCONF_XTAL_RTC_SEL_SHIFT) & CFGREG_RTCCONF_XTAL_RTC_SEL_MASK)                                            /*!<  32kHz RTC Clock Configuration Bits Set */
#define CFGREG_RTCCONF_XTAL_RTC_TEST_BITS_SET(x)                               ((x << CFGREG_RTCCONF_XTAL_RTC_TEST_SHIFT) & CFGREG_RTCCONF_XTAL_RTC_TEST_MASK)                                          /*!<  32kHz RTC Clock Configuration Bits Set */

#define CFGREG_COREFREQ_CLK_TEST_OUT_EN_BITS_SET(x)                            ((x << CFGREG_COREFREQ_CLK_TEST_OUT_EN_SHIFT) & CFGREG_COREFREQ_CLK_TEST_OUT_EN_MASK)                                    /*!<  External XTAL Configuration Register Bits Set */
#define CFGREG_COREFREQ_CLK_RC_CORE_VAL_SRC_BITS_SET(x)                        ((x << CFGREG_COREFREQ_CLK_RC_CORE_VAL_SRC_SHIFT) & CFGREG_COREFREQ_CLK_RC_CORE_VAL_SRC_MASK)                            /*!<  External XTAL Configuration Register Bits Set */
#define CFGREG_COREFREQ_CLK_RC_CORE_VAL_BITS_SET(x)                            ((x << CFGREG_COREFREQ_CLK_RC_CORE_VAL_SHIFT) & CFGREG_COREFREQ_CLK_RC_CORE_VAL_MASK)                                    /*!<  External XTAL Configuration Register Bits Set */
#define CFGREG_COREFREQ_CLK_RC_CORE_CAL_BITS_SET(x)                            ((x << CFGREG_COREFREQ_CLK_RC_CORE_CAL_SHIFT) & CFGREG_COREFREQ_CLK_RC_CORE_CAL_MASK)                                    /*!<  External XTAL Configuration Register Bits Set */
#define CFGREG_COREFREQ_CLK_CORE_SEL_BITS_SET(x)                               ((x << CFGREG_COREFREQ_CLK_CORE_SEL_SHIFT) & CFGREG_COREFREQ_CLK_CORE_SEL_MASK)                                          /*!<  External XTAL Configuration Register Bits Set */
#define CFGREG_COREFREQ_CLK_XTAL_CORE_TEST_BITS_SET(x)                         ((x << CFGREG_COREFREQ_CLK_XTAL_CORE_TEST_SHIFT) & CFGREG_COREFREQ_CLK_XTAL_CORE_TEST_MASK)                              /*!<  External XTAL Configuration Register Bits Set */

#define CFGREG_COREFREQ_PLL_CTRL_FINE_LOAD_BITS_SET(x)                         ((x << CFGREG_COREFREQ_PLL_CTRL_FINE_LOAD_SHIFT) & CFGREG_COREFREQ_PLL_CTRL_FINE_LOAD_MASK)                              /*!<  PLL Configuration Register Bits Set */
#define CFGREG_COREFREQ_PLL_CTRL_LOAD_BITS_SET(x)                              ((x << CFGREG_COREFREQ_PLL_CTRL_LOAD_SHIFT) & CFGREG_COREFREQ_PLL_CTRL_LOAD_MASK)                                        /*!<  PLL Configuration Register Bits Set */
#define CFGREG_COREFREQ_PLL_CTRL_PVT_LOAD_BITS_SET(x)                          ((x << CFGREG_COREFREQ_PLL_CTRL_PVT_LOAD_SHIFT) & CFGREG_COREFREQ_PLL_CTRL_PVT_LOAD_MASK)                                /*!<  PLL Configuration Register Bits Set */
#define CFGREG_COREFREQ_PLL_PLL_VFB_EN_BITS_SET(x)                             ((x << CFGREG_COREFREQ_PLL_PLL_VFB_EN_SHIFT) & CFGREG_COREFREQ_PLL_PLL_VFB_EN_MASK)                                      /*!<  PLL Configuration Register Bits Set */
#define CFGREG_COREFREQ_PLL_TEST_BITS_SET(x)                                   ((x << CFGREG_COREFREQ_PLL_TEST_SHIFT) & CFGREG_COREFREQ_PLL_TEST_MASK)                                                  /*!<  PLL Configuration Register Bits Set */
#define CFGREG_COREFREQ_PLL_N_BITS_SET(x)                                      ((x << CFGREG_COREFREQ_PLL_N_SHIFT) & CFGREG_COREFREQ_PLL_N_MASK)                                                        /*!<  PLL Configuration Register Bits Set */
#define CFGREG_COREFREQ_PLL_REF_SEL_BITS_SET(x)                                ((x << CFGREG_COREFREQ_PLL_REF_SEL_SHIFT) & CFGREG_COREFREQ_PLL_REF_SEL_MASK)                                            /*!<  PLL Configuration Register Bits Set */
#define CFGREG_COREFREQ_PLL_EN_BITS_SET(x)                                     ((x << CFGREG_COREFREQ_PLL_EN_SHIFT) & CFGREG_COREFREQ_PLL_EN_MASK)                                                      /*!<  PLL Configuration Register Bits Set */

#define CFGREG_COREFREQ_STAT_CTRL_FINE_BITS_SET(x)                             ((x << CFGREG_COREFREQ_STAT_CTRL_FINE_SHIFT) & CFGREG_COREFREQ_STAT_CTRL_FINE_MASK)                                      /*!<  Core Clock Status Register Bits Set */
#define CFGREG_COREFREQ_STAT_CTRL_PVT_BITS_SET(x)                              ((x << CFGREG_COREFREQ_STAT_CTRL_PVT_SHIFT) & CFGREG_COREFREQ_STAT_CTRL_PVT_MASK)                                        /*!<  Core Clock Status Register Bits Set */
#define CFGREG_COREFREQ_STAT_RC_CORE_VAL_BITS_SET(x)                           ((x << CFGREG_COREFREQ_STAT_RC_CORE_VAL_SHIFT) & CFGREG_COREFREQ_STAT_RC_CORE_VAL_MASK)                                  /*!<  Core Clock Status Register Bits Set */
#define CFGREG_COREFREQ_STAT_RC_CORE_RDY_BITS_SET(x)                           ((x << CFGREG_COREFREQ_STAT_RC_CORE_RDY_SHIFT) & CFGREG_COREFREQ_STAT_RC_CORE_RDY_MASK)                                  /*!<  Core Clock Status Register Bits Set */
#define CFGREG_COREFREQ_STAT_PLL_LOCK_BITS_SET(x)                              ((x << CFGREG_COREFREQ_STAT_PLL_LOCK_SHIFT) & CFGREG_COREFREQ_STAT_PLL_LOCK_MASK)                                        /*!<  Core Clock Status Register Bits Set */
#define CFGREG_COREFREQ_STAT_XTAL_CORE_LOCK_BITS_SET(x)                        ((x << CFGREG_COREFREQ_STAT_XTAL_CORE_LOCK_SHIFT) & CFGREG_COREFREQ_STAT_XTAL_CORE_LOCK_MASK)                            /*!<  Core Clock Status Register Bits Set */
#define CFGREG_COREFREQ_STAT_CLK_CORE_SEL_BITS_SET(x)                          ((x << CFGREG_COREFREQ_STAT_CLK_CORE_SEL_SHIFT) & CFGREG_COREFREQ_STAT_CLK_CORE_SEL_MASK)                                /*!<  Core Clock Status Register Bits Set */

#define CFGREG_GNSSAFE_CONF_GNSS_OUT_BAND_BITS_SET(x)                          ((x << CFGREG_GNSSAFE_CONF_GNSS_OUT_BAND_SHIFT) & CFGREG_GNSSAFE_CONF_GNSS_OUT_BAND_MASK)                                /*!<  GNSS-AFE Configuration Register Bits Set */
#define CFGREG_GNSSAFE_CONF_GNSS_AUX1_EN_BITS_SET(x)                           ((x << CFGREG_GNSSAFE_CONF_GNSS_AUX1_EN_SHIFT) & CFGREG_GNSSAFE_CONF_GNSS_AUX1_EN_MASK)                                  /*!<  GNSS-AFE Configuration Register Bits Set */
#define CFGREG_GNSSAFE_CONF_GNSS_AUX0_EN_BITS_SET(x)                           ((x << CFGREG_GNSSAFE_CONF_GNSS_AUX0_EN_SHIFT) & CFGREG_GNSSAFE_CONF_GNSS_AUX0_EN_MASK)                                  /*!<  GNSS-AFE Configuration Register Bits Set */

#define CFGREG_PLL1_CONF_EN_BOOST_BITS_SET(x)                                  ((x << CFGREG_PLL1_CONF_EN_BOOST_SHIFT) & CFGREG_PLL1_CONF_EN_BOOST_MASK)                                                /*!<  PLL1 Configuration Register Bits Set */
#define CFGREG_PLL1_CONF_ADC_CLK_DIV_BITS_SET(x)                               ((x << CFGREG_PLL1_CONF_ADC_CLK_DIV_SHIFT) & CFGREG_PLL1_CONF_ADC_CLK_DIV_MASK)                                          /*!<  PLL1 Configuration Register Bits Set */
#define CFGREG_PLL1_CONF_LOPCB_EN_BITS_SET(x)                                  ((x << CFGREG_PLL1_CONF_LOPCB_EN_SHIFT) & CFGREG_PLL1_CONF_LOPCB_EN_MASK)                                                /*!<  PLL1 Configuration Register Bits Set */
#define CFGREG_PLL1_CONF_TEST_EN_BITS_SET(x)                                   ((x << CFGREG_PLL1_CONF_TEST_EN_SHIFT) & CFGREG_PLL1_CONF_TEST_EN_MASK)                                                  /*!<  PLL1 Configuration Register Bits Set */
#define CFGREG_PLL1_CONF_FCW_BITS_SET(x)                                       ((x << CFGREG_PLL1_CONF_FCW_SHIFT) & CFGREG_PLL1_CONF_FCW_MASK)                                                          /*!<  PLL1 Configuration Register Bits Set */
#define CFGREG_PLL1_CONF_EN_BITS_SET(x)                                        ((x << CFGREG_PLL1_CONF_EN_SHIFT) & CFGREG_PLL1_CONF_EN_MASK)                                                            /*!<  PLL1 Configuration Register Bits Set */

#define CFGREG_PLL1DCO_CONF_AMP_BITS_SET(x)                                    ((x << CFGREG_PLL1DCO_CONF_AMP_SHIFT) & CFGREG_PLL1DCO_CONF_AMP_MASK)                                                    /*!<  PLL1 DCO Configuration Register Bits Set */
#define CFGREG_PLL1DCO_CONF_AMP_LOAD_BITS_SET(x)                               ((x << CFGREG_PLL1DCO_CONF_AMP_LOAD_SHIFT) & CFGREG_PLL1DCO_CONF_AMP_LOAD_MASK)                                          /*!<  PLL1 DCO Configuration Register Bits Set */
#define CFGREG_PLL1DCO_CONF_CTRL_PVT_BITS_SET(x)                               ((x << CFGREG_PLL1DCO_CONF_CTRL_PVT_SHIFT) & CFGREG_PLL1DCO_CONF_CTRL_PVT_MASK)                                          /*!<  PLL1 DCO Configuration Register Bits Set */
#define CFGREG_PLL1DCO_CONF_CTRL_FINE_BITS_SET(x)                              ((x << CFGREG_PLL1DCO_CONF_CTRL_FINE_SHIFT) & CFGREG_PLL1DCO_CONF_CTRL_FINE_MASK)                                        /*!<  PLL1 DCO Configuration Register Bits Set */
#define CFGREG_PLL1DCO_CONF_CTRL_LOAD_BITS_SET(x)                              ((x << CFGREG_PLL1DCO_CONF_CTRL_LOAD_SHIFT) & CFGREG_PLL1DCO_CONF_CTRL_LOAD_MASK)                                        /*!<  PLL1 DCO Configuration Register Bits Set */

#define CFGREG_PLL1_STAT_PH_VAR_BITS_SET(x)                                    ((x << CFGREG_PLL1_STAT_PH_VAR_SHIFT) & CFGREG_PLL1_STAT_PH_VAR_MASK)                                                    /*!<  PLL1 Status Register Bits Set */
#define CFGREG_PLL1_STAT_PHERR_BITS_SET(x)                                     ((x << CFGREG_PLL1_STAT_PHERR_SHIFT) & CFGREG_PLL1_STAT_PHERR_MASK)                                                      /*!<  PLL1 Status Register Bits Set */
#define CFGREG_PLL1_STAT_LOCK_BITS_SET(x)                                      ((x << CFGREG_PLL1_STAT_LOCK_SHIFT) & CFGREG_PLL1_STAT_LOCK_MASK)                                                        /*!<  PLL1 Status Register Bits Set */

#define CFGREG_PLL1_STAT2_TV_BITS_SET(x)                                       ((x << CFGREG_PLL1_STAT2_TV_SHIFT) & CFGREG_PLL1_STAT2_TV_MASK)                                                          /*!<  PLL1 Status Register Bits Set */

#define CFGREG_PLL1DCO_STAT_AMP_BITS_SET(x)                                    ((x << CFGREG_PLL1DCO_STAT_AMP_SHIFT) & CFGREG_PLL1DCO_STAT_AMP_MASK)                                                    /*!<  PLL1 TDC Status Register Bits Set */
#define CFGREG_PLL1DCO_STAT_AMP_LOW_BITS_SET(x)                                ((x << CFGREG_PLL1DCO_STAT_AMP_LOW_SHIFT) & CFGREG_PLL1DCO_STAT_AMP_LOW_MASK)                                            /*!<  PLL1 TDC Status Register Bits Set */
#define CFGREG_PLL1DCO_STAT_CTRL_PVT_BITS_SET(x)                               ((x << CFGREG_PLL1DCO_STAT_CTRL_PVT_SHIFT) & CFGREG_PLL1DCO_STAT_CTRL_PVT_MASK)                                          /*!<  PLL1 TDC Status Register Bits Set */
#define CFGREG_PLL1DCO_STAT_CTRL_FINE_BITS_SET(x)                              ((x << CFGREG_PLL1DCO_STAT_CTRL_FINE_SHIFT) & CFGREG_PLL1DCO_STAT_CTRL_FINE_MASK)                                        /*!<  PLL1 TDC Status Register Bits Set */

#define CFGREG_PLL25_CONF_EN_BOOST_BITS_SET(x)                                 ((x << CFGREG_PLL25_CONF_EN_BOOST_SHIFT) & CFGREG_PLL25_CONF_EN_BOOST_MASK)                                              /*!<  PLL25 Configuration Register Bits Set */
#define CFGREG_PLL25_CONF_ADC_CLK_DIV_BITS_SET(x)                              ((x << CFGREG_PLL25_CONF_ADC_CLK_DIV_SHIFT) & CFGREG_PLL25_CONF_ADC_CLK_DIV_MASK)                                        /*!<  PLL25 Configuration Register Bits Set */
#define CFGREG_PLL25_CONF_LOPCB_EN_BITS_SET(x)                                 ((x << CFGREG_PLL25_CONF_LOPCB_EN_SHIFT) & CFGREG_PLL25_CONF_LOPCB_EN_MASK)                                              /*!<  PLL25 Configuration Register Bits Set */
#define CFGREG_PLL25_CONF_TEST_EN_BITS_SET(x)                                  ((x << CFGREG_PLL25_CONF_TEST_EN_SHIFT) & CFGREG_PLL25_CONF_TEST_EN_MASK)                                                /*!<  PLL25 Configuration Register Bits Set */
#define CFGREG_PLL25_CONF_FCW_BITS_SET(x)                                      ((x << CFGREG_PLL25_CONF_FCW_SHIFT) & CFGREG_PLL25_CONF_FCW_MASK)                                                        /*!<  PLL25 Configuration Register Bits Set */
#define CFGREG_PLL25_CONF_EN_BITS_SET(x)                                       ((x << CFGREG_PLL25_CONF_EN_SHIFT) & CFGREG_PLL25_CONF_EN_MASK)                                                          /*!<  PLL25 Configuration Register Bits Set */

#define CFGREG_PLL25DCO_CONF_AMP_BITS_SET(x)                                   ((x << CFGREG_PLL25DCO_CONF_AMP_SHIFT) & CFGREG_PLL25DCO_CONF_AMP_MASK)                                                  /*!<  PLL25 DCO Configuration Register Bits Set */
#define CFGREG_PLL25DCO_CONF_AMP_LOAD_BITS_SET(x)                              ((x << CFGREG_PLL25DCO_CONF_AMP_LOAD_SHIFT) & CFGREG_PLL25DCO_CONF_AMP_LOAD_MASK)                                        /*!<  PLL25 DCO Configuration Register Bits Set */
#define CFGREG_PLL25DCO_CONF_CTRL_PVT_BITS_SET(x)                              ((x << CFGREG_PLL25DCO_CONF_CTRL_PVT_SHIFT) & CFGREG_PLL25DCO_CONF_CTRL_PVT_MASK)                                        /*!<  PLL25 DCO Configuration Register Bits Set */
#define CFGREG_PLL25DCO_CONF_CTRL_FINE_BITS_SET(x)                             ((x << CFGREG_PLL25DCO_CONF_CTRL_FINE_SHIFT) & CFGREG_PLL25DCO_CONF_CTRL_FINE_MASK)                                      /*!<  PLL25 DCO Configuration Register Bits Set */
#define CFGREG_PLL25DCO_CONF_CTRL_LOAD_BITS_SET(x)                             ((x << CFGREG_PLL25DCO_CONF_CTRL_LOAD_SHIFT) & CFGREG_PLL25DCO_CONF_CTRL_LOAD_MASK)                                      /*!<  PLL25 DCO Configuration Register Bits Set */

#define CFGREG_PLL25_STAT_PH_VAR_BITS_SET(x)                                   ((x << CFGREG_PLL25_STAT_PH_VAR_SHIFT) & CFGREG_PLL25_STAT_PH_VAR_MASK)                                                  /*!<  PLL25 Status Register Bits Set */
#define CFGREG_PLL25_STAT_PHERR_BITS_SET(x)                                    ((x << CFGREG_PLL25_STAT_PHERR_SHIFT) & CFGREG_PLL25_STAT_PHERR_MASK)                                                    /*!<  PLL25 Status Register Bits Set */
#define CFGREG_PLL25_STAT_LOCK_BITS_SET(x)                                     ((x << CFGREG_PLL25_STAT_LOCK_SHIFT) & CFGREG_PLL25_STAT_LOCK_MASK)                                                      /*!<  PLL25 Status Register Bits Set */

#define CFGREG_PLL25_STAT2_TV_BITS_SET(x)                                      ((x << CFGREG_PLL25_STAT2_TV_SHIFT) & CFGREG_PLL25_STAT2_TV_MASK)                                                        /*!<  PLL25 Status Register Bits Set */

#define CFGREG_PLL25DCO_STAT_AMP_BITS_SET(x)                                   ((x << CFGREG_PLL25DCO_STAT_AMP_SHIFT) & CFGREG_PLL25DCO_STAT_AMP_MASK)                                                  /*!<  PLL25 TDC Status Register Bits Set */
#define CFGREG_PLL25DCO_STAT_AMP_LOW_BITS_SET(x)                               ((x << CFGREG_PLL25DCO_STAT_AMP_LOW_SHIFT) & CFGREG_PLL25DCO_STAT_AMP_LOW_MASK)                                          /*!<  PLL25 TDC Status Register Bits Set */
#define CFGREG_PLL25DCO_STAT_CTRL_PVT_BITS_SET(x)                              ((x << CFGREG_PLL25DCO_STAT_CTRL_PVT_SHIFT) & CFGREG_PLL25DCO_STAT_CTRL_PVT_MASK)                                        /*!<  PLL25 TDC Status Register Bits Set */
#define CFGREG_PLL25DCO_STAT_CTRL_FINE_BITS_SET(x)                             ((x << CFGREG_PLL25DCO_STAT_CTRL_FINE_SHIFT) & CFGREG_PLL25DCO_STAT_CTRL_FINE_MASK)                                      /*!<  PLL25 TDC Status Register Bits Set */

#define CFGREG_IF1_CONF_IF_BANDCUT_BITS_SET(x)                                 ((x << CFGREG_IF1_CONF_IF_BANDCUT_SHIFT) & CFGREG_IF1_CONF_IF_BANDCUT_MASK)                                              /*!<  IF1 Configuration Register Bits Set */
#define CFGREG_IF1_CONF_OFFSET_CAL_Q_BITS_SET(x)                               ((x << CFGREG_IF1_CONF_OFFSET_CAL_Q_SHIFT) & CFGREG_IF1_CONF_OFFSET_CAL_Q_MASK)                                          /*!<  IF1 Configuration Register Bits Set */
#define CFGREG_IF1_CONF_OFFSET_CAL_DIRECTION_Q_BITS_SET(x)                     ((x << CFGREG_IF1_CONF_OFFSET_CAL_DIRECTION_Q_SHIFT) & CFGREG_IF1_CONF_OFFSET_CAL_DIRECTION_Q_MASK)                      /*!<  IF1 Configuration Register Bits Set */
#define CFGREG_IF1_CONF_OFFSET_CAL_I_BITS_SET(x)                               ((x << CFGREG_IF1_CONF_OFFSET_CAL_I_SHIFT) & CFGREG_IF1_CONF_OFFSET_CAL_I_MASK)                                          /*!<  IF1 Configuration Register Bits Set */
#define CFGREG_IF1_CONF_OFFSET_CAL_DIRECTION_I_BITS_SET(x)                     ((x << CFGREG_IF1_CONF_OFFSET_CAL_DIRECTION_I_SHIFT) & CFGREG_IF1_CONF_OFFSET_CAL_DIRECTION_I_MASK)                      /*!<  IF1 Configuration Register Bits Set */
#define CFGREG_IF1_CONF_PGA2_EN_BITS_SET(x)                                    ((x << CFGREG_IF1_CONF_PGA2_EN_SHIFT) & CFGREG_IF1_CONF_PGA2_EN_MASK)                                                    /*!<  IF1 Configuration Register Bits Set */
#define CFGREG_IF1_CONF_PGA1_EN_BITS_SET(x)                                    ((x << CFGREG_IF1_CONF_PGA1_EN_SHIFT) & CFGREG_IF1_CONF_PGA1_EN_MASK)                                                    /*!<  IF1 Configuration Register Bits Set */
#define CFGREG_IF1_CONF_PREAMP_EN_BITS_SET(x)                                  ((x << CFGREG_IF1_CONF_PREAMP_EN_SHIFT) & CFGREG_IF1_CONF_PREAMP_EN_MASK)                                                /*!<  IF1 Configuration Register Bits Set */
#define CFGREG_IF1_CONF_EN_BITS_SET(x)                                         ((x << CFGREG_IF1_CONF_EN_SHIFT) & CFGREG_IF1_CONF_EN_MASK)                                                              /*!<  IF1 Configuration Register Bits Set */

#define CFGREG_IF25_CONF_IF_BANDCUT_BITS_SET(x)                                ((x << CFGREG_IF25_CONF_IF_BANDCUT_SHIFT) & CFGREG_IF25_CONF_IF_BANDCUT_MASK)                                            /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_IF25_CONF_OFFSET_CAL_Q_BITS_SET(x)                              ((x << CFGREG_IF25_CONF_OFFSET_CAL_Q_SHIFT) & CFGREG_IF25_CONF_OFFSET_CAL_Q_MASK)                                        /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_IF25_CONF_OFFSET_CAL_DIRECTION_Q_BITS_SET(x)                    ((x << CFGREG_IF25_CONF_OFFSET_CAL_DIRECTION_Q_SHIFT) & CFGREG_IF25_CONF_OFFSET_CAL_DIRECTION_Q_MASK)                    /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_IF25_CONF_OFFSET_CAL_I_BITS_SET(x)                              ((x << CFGREG_IF25_CONF_OFFSET_CAL_I_SHIFT) & CFGREG_IF25_CONF_OFFSET_CAL_I_MASK)                                        /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_IF25_CONF_OFFSET_CAL_DIRECTION_I_BITS_SET(x)                    ((x << CFGREG_IF25_CONF_OFFSET_CAL_DIRECTION_I_SHIFT) & CFGREG_IF25_CONF_OFFSET_CAL_DIRECTION_I_MASK)                    /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_IF25_CONF_PGA2_EN_BITS_SET(x)                                   ((x << CFGREG_IF25_CONF_PGA2_EN_SHIFT) & CFGREG_IF25_CONF_PGA2_EN_MASK)                                                  /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_IF25_CONF_PGA1_EN_BITS_SET(x)                                   ((x << CFGREG_IF25_CONF_PGA1_EN_SHIFT) & CFGREG_IF25_CONF_PGA1_EN_MASK)                                                  /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_IF25_CONF_PREAMP_EN_BITS_SET(x)                                 ((x << CFGREG_IF25_CONF_PREAMP_EN_SHIFT) & CFGREG_IF25_CONF_PREAMP_EN_MASK)                                              /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_IF25_CONF_EN_BITS_SET(x)                                        ((x << CFGREG_IF25_CONF_EN_SHIFT) & CFGREG_IF25_CONF_EN_MASK)                                                            /*!<  IF25 Configuration Register Bits Set */

#define CFGREG_PROBE_CONF_PLL15FB_SEL_BITS_SET(x)                              ((x << CFGREG_PROBE_CONF_PLL15FB_SEL_SHIFT) & CFGREG_PROBE_CONF_PLL15FB_SEL_MASK)                                        /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_PROBE_CONF_PLL15FB_EN_BITS_SET(x)                               ((x << CFGREG_PROBE_CONF_PLL15FB_EN_SHIFT) & CFGREG_PROBE_CONF_PLL15FB_EN_MASK)                                          /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_PROBE_CONF_IF25_PROBEB_SELECT_BITS_SET(x)                       ((x << CFGREG_PROBE_CONF_IF25_PROBEB_SELECT_SHIFT) & CFGREG_PROBE_CONF_IF25_PROBEB_SELECT_MASK)                          /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_PROBE_CONF_IF25_PROBEB_EN_BITS_SET(x)                           ((x << CFGREG_PROBE_CONF_IF25_PROBEB_EN_SHIFT) & CFGREG_PROBE_CONF_IF25_PROBEB_EN_MASK)                                  /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_PROBE_CONF_IF25_PROBEA_SELECT_BITS_SET(x)                       ((x << CFGREG_PROBE_CONF_IF25_PROBEA_SELECT_SHIFT) & CFGREG_PROBE_CONF_IF25_PROBEA_SELECT_MASK)                          /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_PROBE_CONF_IF1_PROBEB_SELECT_BITS_SET(x)                        ((x << CFGREG_PROBE_CONF_IF1_PROBEB_SELECT_SHIFT) & CFGREG_PROBE_CONF_IF1_PROBEB_SELECT_MASK)                            /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_PROBE_CONF_IF1_PROBEB_EN_BITS_SET(x)                            ((x << CFGREG_PROBE_CONF_IF1_PROBEB_EN_SHIFT) & CFGREG_PROBE_CONF_IF1_PROBEB_EN_MASK)                                    /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_PROBE_CONF_IF1_PROBEA_SELECT_BITS_SET(x)                        ((x << CFGREG_PROBE_CONF_IF1_PROBEA_SELECT_SHIFT) & CFGREG_PROBE_CONF_IF1_PROBEA_SELECT_MASK)                            /*!<  IF25 Configuration Register Bits Set */

#define CFGREG_ADC1_CONF_CLK_SEL_BITS_SET(x)                                   ((x << CFGREG_ADC1_CONF_CLK_SEL_SHIFT) & CFGREG_ADC1_CONF_CLK_SEL_MASK)                                                  /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_ADC1_CONF_CLK_CONF_BITS_SET(x)                                  ((x << CFGREG_ADC1_CONF_CLK_CONF_SHIFT) & CFGREG_ADC1_CONF_CLK_CONF_MASK)                                                /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_ADC1_CONF_SAH_IBIAS_CTRL_BITS_SET(x)                            ((x << CFGREG_ADC1_CONF_SAH_IBIAS_CTRL_SHIFT) & CFGREG_ADC1_CONF_SAH_IBIAS_CTRL_MASK)                                    /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_ADC1_CONF_CAL_EN_BITS_SET(x)                                    ((x << CFGREG_ADC1_CONF_CAL_EN_SHIFT) & CFGREG_ADC1_CONF_CAL_EN_MASK)                                                    /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_ADC1_CONF_SAH_EN_BITS_SET(x)                                    ((x << CFGREG_ADC1_CONF_SAH_EN_SHIFT) & CFGREG_ADC1_CONF_SAH_EN_MASK)                                                    /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_ADC1_CONF_ADC_EN_BITS_SET(x)                                    ((x << CFGREG_ADC1_CONF_ADC_EN_SHIFT) & CFGREG_ADC1_CONF_ADC_EN_MASK)                                                    /*!<  IF25 Configuration Register Bits Set */

#define CFGREG_ADC25_CONF_CLK_SEL_BITS_SET(x)                                  ((x << CFGREG_ADC25_CONF_CLK_SEL_SHIFT) & CFGREG_ADC25_CONF_CLK_SEL_MASK)                                                /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_ADC25_CONF_CLK_CONF_BITS_SET(x)                                 ((x << CFGREG_ADC25_CONF_CLK_CONF_SHIFT) & CFGREG_ADC25_CONF_CLK_CONF_MASK)                                              /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_ADC25_CONF_SAH_IBIAS_CTRL_BITS_SET(x)                           ((x << CFGREG_ADC25_CONF_SAH_IBIAS_CTRL_SHIFT) & CFGREG_ADC25_CONF_SAH_IBIAS_CTRL_MASK)                                  /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_ADC25_CONF_CAL_EN_BITS_SET(x)                                   ((x << CFGREG_ADC25_CONF_CAL_EN_SHIFT) & CFGREG_ADC25_CONF_CAL_EN_MASK)                                                  /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_ADC25_CONF_SAH_EN_BITS_SET(x)                                   ((x << CFGREG_ADC25_CONF_SAH_EN_SHIFT) & CFGREG_ADC25_CONF_SAH_EN_MASK)                                                  /*!<  IF25 Configuration Register Bits Set */
#define CFGREG_ADC25_CONF_ADC_EN_BITS_SET(x)                                   ((x << CFGREG_ADC25_CONF_ADC_EN_SHIFT) & CFGREG_ADC25_CONF_ADC_EN_MASK)                                                  /*!<  IF25 Configuration Register Bits Set */

#define CFGREG_SPLIT1_CTRL_CLEAR_BITS_SET(x)                                   ((x << CFGREG_SPLIT1_CTRL_CLEAR_SHIFT) & CFGREG_SPLIT1_CTRL_CLEAR_MASK)                                                  /*!<  Splitter AFE1 Control Register Bits Set */

#define CFGREG_SPLIT1_CFG_GAIN_CHANGE_CNT_BITS_SET(x)                          ((x << CFGREG_SPLIT1_CFG_GAIN_CHANGE_CNT_SHIFT) & CFGREG_SPLIT1_CFG_GAIN_CHANGE_CNT_MASK)                                /*!<  Splitter AFE1 Gain Change Configuration Register Bits Set */

#define CFGREG_SPLIT1_CFG_LIMITER_THRESHOLD_BITS_SET(x)                        ((x << CFGREG_SPLIT1_CFG_LIMITER_THRESHOLD_SHIFT) & CFGREG_SPLIT1_CFG_LIMITER_THRESHOLD_MASK)                            /*!<  Splitter AFE1 Limiter Threshold Configuration Register Bits Set */

#define CFGREG_SPLIT1_CFG_MANUAL_MODE_PGA2_GAIN_BITS_SET(x)                    ((x << CFGREG_SPLIT1_CFG_MANUAL_MODE_PGA2_GAIN_SHIFT) & CFGREG_SPLIT1_CFG_MANUAL_MODE_PGA2_GAIN_MASK)                    /*!<  Splitter AFE1 Manual Mode Configuration Register Bits Set */
#define CFGREG_SPLIT1_CFG_MANUAL_MODE_PGA1_GAIN_BITS_SET(x)                    ((x << CFGREG_SPLIT1_CFG_MANUAL_MODE_PGA1_GAIN_SHIFT) & CFGREG_SPLIT1_CFG_MANUAL_MODE_PGA1_GAIN_MASK)                    /*!<  Splitter AFE1 Manual Mode Configuration Register Bits Set */
#define CFGREG_SPLIT1_CFG_MANUAL_MODE_EN_BITS_SET(x)                           ((x << CFGREG_SPLIT1_CFG_MANUAL_MODE_EN_SHIFT) & CFGREG_SPLIT1_CFG_MANUAL_MODE_EN_MASK)                                  /*!<  Splitter AFE1 Manual Mode Configuration Register Bits Set */

#define CFGREG_SPLIT1_L1E1_CFG_DEC_BY_THREE_EN_BITS_SET(x)                     ((x << CFGREG_SPLIT1_L1E1_CFG_DEC_BY_THREE_EN_SHIFT) & CFGREG_SPLIT1_L1E1_CFG_DEC_BY_THREE_EN_MASK)                      /*!<  Splitter AFE1 L1E1 Configuration Register Bits Set */
#define CFGREG_SPLIT1_L1E1_CFG_DEC_SAMPLES_SHIFT_BITS_SET(x)                   ((x << CFGREG_SPLIT1_L1E1_CFG_DEC_SAMPLES_SHIFT_SHIFT) & CFGREG_SPLIT1_L1E1_CFG_DEC_SAMPLES_SHIFT_MASK)                  /*!<  Splitter AFE1 L1E1 Configuration Register Bits Set */
#define CFGREG_SPLIT1_L1E1_CFG_CARR_MODE_BITS_SET(x)                           ((x << CFGREG_SPLIT1_L1E1_CFG_CARR_MODE_SHIFT) & CFGREG_SPLIT1_L1E1_CFG_CARR_MODE_MASK)                                  /*!<  Splitter AFE1 L1E1 Configuration Register Bits Set */
#define CFGREG_SPLIT1_L1E1_CFG_CARR_ENABLE_BITS_SET(x)                         ((x << CFGREG_SPLIT1_L1E1_CFG_CARR_ENABLE_SHIFT) & CFGREG_SPLIT1_L1E1_CFG_CARR_ENABLE_MASK)                              /*!<  Splitter AFE1 L1E1 Configuration Register Bits Set */
#define CFGREG_SPLIT1_L1E1_CFG_LPF_K_PARAM_BITS_SET(x)                         ((x << CFGREG_SPLIT1_L1E1_CFG_LPF_K_PARAM_SHIFT) & CFGREG_SPLIT1_L1E1_CFG_LPF_K_PARAM_MASK)                              /*!<  Splitter AFE1 L1E1 Configuration Register Bits Set */
#define CFGREG_SPLIT1_L1E1_CFG_IQ_SAME_GAIN_BITS_SET(x)                        ((x << CFGREG_SPLIT1_L1E1_CFG_IQ_SAME_GAIN_SHIFT) & CFGREG_SPLIT1_L1E1_CFG_IQ_SAME_GAIN_MASK)                            /*!<  Splitter AFE1 L1E1 Configuration Register Bits Set */

#define CFGREG_SPLIT1_L1E1_CFG_CARR_FREQ_STEP_BITS_SET(x)                      ((x << CFGREG_SPLIT1_L1E1_CFG_CARR_FREQ_STEP_SHIFT) & CFGREG_SPLIT1_L1E1_CFG_CARR_FREQ_STEP_MASK)                        /*!<  Splitter AFE1 L1E1 Carrier Frequency Configuration Register Bits Set */

#define CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_MAX_BITS_SET(x)                     ((x << CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_MAX_SHIFT) & CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_MAX_MASK)                      /*!<  Splitter AFE1 L1E1 Threshold I Configuration Register Bits Set */
#define CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_MIN_BITS_SET(x)                     ((x << CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_MIN_SHIFT) & CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_MIN_MASK)                      /*!<  Splitter AFE1 L1E1 Threshold I Configuration Register Bits Set */

#define CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_MAX_BITS_SET(x)                     ((x << CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_MAX_SHIFT) & CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_MAX_MASK)                      /*!<  Splitter AFE1 L1E1 Threshold Q Configuration Register Bits Set */
#define CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_MIN_BITS_SET(x)                     ((x << CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_MIN_SHIFT) & CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_MIN_MASK)                      /*!<  Splitter AFE1 L1E1 Threshold Q Configuration Register Bits Set */

#define CFGREG_SPLIT1_L1E1_CFG_MANUAL_MODE_DIG_GAIN_Q_BITS_SET(x)              ((x << CFGREG_SPLIT1_L1E1_CFG_MANUAL_MODE_DIG_GAIN_Q_SHIFT) & CFGREG_SPLIT1_L1E1_CFG_MANUAL_MODE_DIG_GAIN_Q_MASK)        /*!<  Splitter AFE1 L1E1 Manual Mode Digital Gain Configuration Register Bits Set */
#define CFGREG_SPLIT1_L1E1_CFG_MANUAL_MODE_DIG_GAIN_I_BITS_SET(x)              ((x << CFGREG_SPLIT1_L1E1_CFG_MANUAL_MODE_DIG_GAIN_I_SHIFT) & CFGREG_SPLIT1_L1E1_CFG_MANUAL_MODE_DIG_GAIN_I_MASK)        /*!<  Splitter AFE1 L1E1 Manual Mode Digital Gain Configuration Register Bits Set */

#define CFGREG_SPLIT25_CTRL_CLEAR_BITS_SET(x)                                  ((x << CFGREG_SPLIT25_CTRL_CLEAR_SHIFT) & CFGREG_SPLIT25_CTRL_CLEAR_MASK)                                                /*!<  Splitter AFE25 Control Register Bits Set */

#define CFGREG_SPLIT25_CFG_GAIN_CHANGE_CNT_BITS_SET(x)                         ((x << CFGREG_SPLIT25_CFG_GAIN_CHANGE_CNT_SHIFT) & CFGREG_SPLIT25_CFG_GAIN_CHANGE_CNT_MASK)                              /*!<  Splitter AFE25 Gain Change Configuration Register Bits Set */

#define CFGREG_SPLIT25_CFG_LIMITER_THRESHOLD_BITS_SET(x)                       ((x << CFGREG_SPLIT25_CFG_LIMITER_THRESHOLD_SHIFT) & CFGREG_SPLIT25_CFG_LIMITER_THRESHOLD_MASK)                          /*!<  Splitter AFE25 Limiter Threshold Configuration Register Bits Set */

#define CFGREG_SPLIT25_CFG_MANUAL_MODE_PGA2_GAIN_BITS_SET(x)                   ((x << CFGREG_SPLIT25_CFG_MANUAL_MODE_PGA2_GAIN_SHIFT) & CFGREG_SPLIT25_CFG_MANUAL_MODE_PGA2_GAIN_MASK)                  /*!<  Splitter AFE25 Manual Mode Configuration Register Bits Set */
#define CFGREG_SPLIT25_CFG_MANUAL_MODE_PGA1_GAIN_BITS_SET(x)                   ((x << CFGREG_SPLIT25_CFG_MANUAL_MODE_PGA1_GAIN_SHIFT) & CFGREG_SPLIT25_CFG_MANUAL_MODE_PGA1_GAIN_MASK)                  /*!<  Splitter AFE25 Manual Mode Configuration Register Bits Set */
#define CFGREG_SPLIT25_CFG_MANUAL_MODE_EN_BITS_SET(x)                          ((x << CFGREG_SPLIT25_CFG_MANUAL_MODE_EN_SHIFT) & CFGREG_SPLIT25_CFG_MANUAL_MODE_EN_MASK)                                /*!<  Splitter AFE25 Manual Mode Configuration Register Bits Set */

#define CFGREG_SPLIT25_L2_CFG_DEC_BY_THREE_EN_BITS_SET(x)                      ((x << CFGREG_SPLIT25_L2_CFG_DEC_BY_THREE_EN_SHIFT) & CFGREG_SPLIT25_L2_CFG_DEC_BY_THREE_EN_MASK)                        /*!<  Splitter AFE25 L2 Configuration Register Bits Set */
#define CFGREG_SPLIT25_L2_CFG_DEC_SAMPLES_SHIFT_BITS_SET(x)                    ((x << CFGREG_SPLIT25_L2_CFG_DEC_SAMPLES_SHIFT_SHIFT) & CFGREG_SPLIT25_L2_CFG_DEC_SAMPLES_SHIFT_MASK)                    /*!<  Splitter AFE25 L2 Configuration Register Bits Set */
#define CFGREG_SPLIT25_L2_CFG_CARR_MODE_BITS_SET(x)                            ((x << CFGREG_SPLIT25_L2_CFG_CARR_MODE_SHIFT) & CFGREG_SPLIT25_L2_CFG_CARR_MODE_MASK)                                    /*!<  Splitter AFE25 L2 Configuration Register Bits Set */
#define CFGREG_SPLIT25_L2_CFG_CARR_ENABLE_BITS_SET(x)                          ((x << CFGREG_SPLIT25_L2_CFG_CARR_ENABLE_SHIFT) & CFGREG_SPLIT25_L2_CFG_CARR_ENABLE_MASK)                                /*!<  Splitter AFE25 L2 Configuration Register Bits Set */
#define CFGREG_SPLIT25_L2_CFG_LPF_K_PARAM_BITS_SET(x)                          ((x << CFGREG_SPLIT25_L2_CFG_LPF_K_PARAM_SHIFT) & CFGREG_SPLIT25_L2_CFG_LPF_K_PARAM_MASK)                                /*!<  Splitter AFE25 L2 Configuration Register Bits Set */
#define CFGREG_SPLIT25_L2_CFG_IQ_SAME_GAIN_BITS_SET(x)                         ((x << CFGREG_SPLIT25_L2_CFG_IQ_SAME_GAIN_SHIFT) & CFGREG_SPLIT25_L2_CFG_IQ_SAME_GAIN_MASK)                              /*!<  Splitter AFE25 L2 Configuration Register Bits Set */

#define CFGREG_SPLIT25_L2_CFG_CARR_FREQ_STEP_BITS_SET(x)                       ((x << CFGREG_SPLIT25_L2_CFG_CARR_FREQ_STEP_SHIFT) & CFGREG_SPLIT25_L2_CFG_CARR_FREQ_STEP_MASK)                          /*!<  Splitter AFE25 L2 Carrier Frequency Configuration Register Bits Set */

#define CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_MAX_BITS_SET(x)                      ((x << CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_MAX_SHIFT) & CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_MAX_MASK)                        /*!<  Splitter AFE25 L2 Threshold I Configuration Register Bits Set */
#define CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_MIN_BITS_SET(x)                      ((x << CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_MIN_SHIFT) & CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_MIN_MASK)                        /*!<  Splitter AFE25 L2 Threshold I Configuration Register Bits Set */

#define CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_MAX_BITS_SET(x)                      ((x << CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_MAX_SHIFT) & CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_MAX_MASK)                        /*!<  Splitter AFE25 L2 Threshold Q Configuration Register Bits Set */
#define CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_MIN_BITS_SET(x)                      ((x << CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_MIN_SHIFT) & CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_MIN_MASK)                        /*!<  Splitter AFE25 L2 Threshold Q Configuration Register Bits Set */

#define CFGREG_SPLIT25_L2_CFG_MANUAL_MODE_DIG_GAIN_Q_BITS_SET(x)               ((x << CFGREG_SPLIT25_L2_CFG_MANUAL_MODE_DIG_GAIN_Q_SHIFT) & CFGREG_SPLIT25_L2_CFG_MANUAL_MODE_DIG_GAIN_Q_MASK)          /*!<  Splitter AFE25 L2 Manual Mode Digital Gain Configuration Register Bits Set */
#define CFGREG_SPLIT25_L2_CFG_MANUAL_MODE_DIG_GAIN_I_BITS_SET(x)               ((x << CFGREG_SPLIT25_L2_CFG_MANUAL_MODE_DIG_GAIN_I_SHIFT) & CFGREG_SPLIT25_L2_CFG_MANUAL_MODE_DIG_GAIN_I_MASK)          /*!<  Splitter AFE25 L2 Manual Mode Digital Gain Configuration Register Bits Set */

#define CFGREG_SPLIT25_E6_CFG_DEC_BY_THREE_EN_BITS_SET(x)                      ((x << CFGREG_SPLIT25_E6_CFG_DEC_BY_THREE_EN_SHIFT) & CFGREG_SPLIT25_E6_CFG_DEC_BY_THREE_EN_MASK)                        /*!<  Splitter AFE25 E6 Configuration Register Bits Set */
#define CFGREG_SPLIT25_E6_CFG_DEC_SAMPLES_SHIFT_BITS_SET(x)                    ((x << CFGREG_SPLIT25_E6_CFG_DEC_SAMPLES_SHIFT_SHIFT) & CFGREG_SPLIT25_E6_CFG_DEC_SAMPLES_SHIFT_MASK)                    /*!<  Splitter AFE25 E6 Configuration Register Bits Set */
#define CFGREG_SPLIT25_E6_CFG_CARR_MODE_BITS_SET(x)                            ((x << CFGREG_SPLIT25_E6_CFG_CARR_MODE_SHIFT) & CFGREG_SPLIT25_E6_CFG_CARR_MODE_MASK)                                    /*!<  Splitter AFE25 E6 Configuration Register Bits Set */
#define CFGREG_SPLIT25_E6_CFG_CARR_ENABLE_BITS_SET(x)                          ((x << CFGREG_SPLIT25_E6_CFG_CARR_ENABLE_SHIFT) & CFGREG_SPLIT25_E6_CFG_CARR_ENABLE_MASK)                                /*!<  Splitter AFE25 E6 Configuration Register Bits Set */
#define CFGREG_SPLIT25_E6_CFG_LPF_K_PARAM_BITS_SET(x)                          ((x << CFGREG_SPLIT25_E6_CFG_LPF_K_PARAM_SHIFT) & CFGREG_SPLIT25_E6_CFG_LPF_K_PARAM_MASK)                                /*!<  Splitter AFE25 E6 Configuration Register Bits Set */
#define CFGREG_SPLIT25_E6_CFG_IQ_SAME_GAIN_BITS_SET(x)                         ((x << CFGREG_SPLIT25_E6_CFG_IQ_SAME_GAIN_SHIFT) & CFGREG_SPLIT25_E6_CFG_IQ_SAME_GAIN_MASK)                              /*!<  Splitter AFE25 E6 Configuration Register Bits Set */

#define CFGREG_SPLIT25_E6_CFG_CARR_FREQ_STEP_BITS_SET(x)                       ((x << CFGREG_SPLIT25_E6_CFG_CARR_FREQ_STEP_SHIFT) & CFGREG_SPLIT25_E6_CFG_CARR_FREQ_STEP_MASK)                          /*!<  Splitter AFE25 E6 Carrier Frequency Configuration Register Bits Set */

#define CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_MAX_BITS_SET(x)                      ((x << CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_MAX_SHIFT) & CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_MAX_MASK)                        /*!<  Splitter AFE25 E6 Threshold I Configuration Register Bits Set */
#define CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_MIN_BITS_SET(x)                      ((x << CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_MIN_SHIFT) & CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_MIN_MASK)                        /*!<  Splitter AFE25 E6 Threshold I Configuration Register Bits Set */

#define CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_MAX_BITS_SET(x)                      ((x << CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_MAX_SHIFT) & CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_MAX_MASK)                        /*!<  Splitter AFE25 E6 Threshold Q Configuration Register Bits Set */
#define CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_MIN_BITS_SET(x)                      ((x << CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_MIN_SHIFT) & CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_MIN_MASK)                        /*!<  Splitter AFE25 E6 Threshold Q Configuration Register Bits Set */

#define CFGREG_SPLIT25_E6_CFG_MANUAL_MODE_DIG_GAIN_Q_BITS_SET(x)               ((x << CFGREG_SPLIT25_E6_CFG_MANUAL_MODE_DIG_GAIN_Q_SHIFT) & CFGREG_SPLIT25_E6_CFG_MANUAL_MODE_DIG_GAIN_Q_MASK)          /*!<  Splitter AFE25 E6 Manual Mode Digital Gain Configuration Register Bits Set */
#define CFGREG_SPLIT25_E6_CFG_MANUAL_MODE_DIG_GAIN_I_BITS_SET(x)               ((x << CFGREG_SPLIT25_E6_CFG_MANUAL_MODE_DIG_GAIN_I_SHIFT) & CFGREG_SPLIT25_E6_CFG_MANUAL_MODE_DIG_GAIN_I_MASK)          /*!<  Splitter AFE25 E6 Manual Mode Digital Gain Configuration Register Bits Set */

#define CFGREG_SPLIT25_L5E5A_CFG_DEC_BY_THREE_EN_BITS_SET(x)                   ((x << CFGREG_SPLIT25_L5E5A_CFG_DEC_BY_THREE_EN_SHIFT) & CFGREG_SPLIT25_L5E5A_CFG_DEC_BY_THREE_EN_MASK)                  /*!<  Splitter AFE25 L5E5A Configuration Register Bits Set */
#define CFGREG_SPLIT25_L5E5A_CFG_DEC_SAMPLES_SHIFT_BITS_SET(x)                 ((x << CFGREG_SPLIT25_L5E5A_CFG_DEC_SAMPLES_SHIFT_SHIFT) & CFGREG_SPLIT25_L5E5A_CFG_DEC_SAMPLES_SHIFT_MASK)              /*!<  Splitter AFE25 L5E5A Configuration Register Bits Set */
#define CFGREG_SPLIT25_L5E5A_CFG_CARR_MODE_BITS_SET(x)                         ((x << CFGREG_SPLIT25_L5E5A_CFG_CARR_MODE_SHIFT) & CFGREG_SPLIT25_L5E5A_CFG_CARR_MODE_MASK)                              /*!<  Splitter AFE25 L5E5A Configuration Register Bits Set */
#define CFGREG_SPLIT25_L5E5A_CFG_CARR_ENABLE_BITS_SET(x)                       ((x << CFGREG_SPLIT25_L5E5A_CFG_CARR_ENABLE_SHIFT) & CFGREG_SPLIT25_L5E5A_CFG_CARR_ENABLE_MASK)                          /*!<  Splitter AFE25 L5E5A Configuration Register Bits Set */
#define CFGREG_SPLIT25_L5E5A_CFG_LPF_K_PARAM_BITS_SET(x)                       ((x << CFGREG_SPLIT25_L5E5A_CFG_LPF_K_PARAM_SHIFT) & CFGREG_SPLIT25_L5E5A_CFG_LPF_K_PARAM_MASK)                          /*!<  Splitter AFE25 L5E5A Configuration Register Bits Set */
#define CFGREG_SPLIT25_L5E5A_CFG_IQ_SAME_GAIN_BITS_SET(x)                      ((x << CFGREG_SPLIT25_L5E5A_CFG_IQ_SAME_GAIN_SHIFT) & CFGREG_SPLIT25_L5E5A_CFG_IQ_SAME_GAIN_MASK)                        /*!<  Splitter AFE25 L5E5A Configuration Register Bits Set */

#define CFGREG_SPLIT25_L5E5A_CFG_CARR_FREQ_STEP_BITS_SET(x)                    ((x << CFGREG_SPLIT25_L5E5A_CFG_CARR_FREQ_STEP_SHIFT) & CFGREG_SPLIT25_L5E5A_CFG_CARR_FREQ_STEP_MASK)                    /*!<  Splitter AFE25 L5E5A Carrier Frequency Configuration Register Bits Set */

#define CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_MAX_BITS_SET(x)                   ((x << CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_MAX_SHIFT) & CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_MAX_MASK)                  /*!<  Splitter AFE25 L5E5A Threshold I Configuration Register Bits Set */
#define CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_MIN_BITS_SET(x)                   ((x << CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_MIN_SHIFT) & CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_MIN_MASK)                  /*!<  Splitter AFE25 L5E5A Threshold I Configuration Register Bits Set */

#define CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_MAX_BITS_SET(x)                   ((x << CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_MAX_SHIFT) & CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_MAX_MASK)                  /*!<  Splitter AFE25 L5E5A Threshold Q Configuration Register Bits Set */
#define CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_MIN_BITS_SET(x)                   ((x << CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_MIN_SHIFT) & CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_MIN_MASK)                  /*!<  Splitter AFE25 L5E5A Threshold Q Configuration Register Bits Set */

#define CFGREG_SPLIT25_L5E5A_CFG_MANUAL_MODE_DIG_GAIN_Q_BITS_SET(x)            ((x << CFGREG_SPLIT25_L5E5A_CFG_MANUAL_MODE_DIG_GAIN_Q_SHIFT) & CFGREG_SPLIT25_L5E5A_CFG_MANUAL_MODE_DIG_GAIN_Q_MASK)    /*!<  Splitter AFE25 L5E5A Manual Mode Digital Gain Configuration Register Bits Set */
#define CFGREG_SPLIT25_L5E5A_CFG_MANUAL_MODE_DIG_GAIN_I_BITS_SET(x)            ((x << CFGREG_SPLIT25_L5E5A_CFG_MANUAL_MODE_DIG_GAIN_I_SHIFT) & CFGREG_SPLIT25_L5E5A_CFG_MANUAL_MODE_DIG_GAIN_I_MASK)    /*!<  Splitter AFE25 L5E5A Manual Mode Digital Gain Configuration Register Bits Set */

#define CFGREG_SPLIT25_E5B_CFG_DEC_BY_THREE_EN_BITS_SET(x)                     ((x << CFGREG_SPLIT25_E5B_CFG_DEC_BY_THREE_EN_SHIFT) & CFGREG_SPLIT25_E5B_CFG_DEC_BY_THREE_EN_MASK)                      /*!<  Splitter AFE25 E5B Configuration Register Bits Set */
#define CFGREG_SPLIT25_E5B_CFG_DEC_SAMPLES_SHIFT_BITS_SET(x)                   ((x << CFGREG_SPLIT25_E5B_CFG_DEC_SAMPLES_SHIFT_SHIFT) & CFGREG_SPLIT25_E5B_CFG_DEC_SAMPLES_SHIFT_MASK)                  /*!<  Splitter AFE25 E5B Configuration Register Bits Set */
#define CFGREG_SPLIT25_E5B_CFG_CARR_MODE_BITS_SET(x)                           ((x << CFGREG_SPLIT25_E5B_CFG_CARR_MODE_SHIFT) & CFGREG_SPLIT25_E5B_CFG_CARR_MODE_MASK)                                  /*!<  Splitter AFE25 E5B Configuration Register Bits Set */
#define CFGREG_SPLIT25_E5B_CFG_CARR_ENABLE_BITS_SET(x)                         ((x << CFGREG_SPLIT25_E5B_CFG_CARR_ENABLE_SHIFT) & CFGREG_SPLIT25_E5B_CFG_CARR_ENABLE_MASK)                              /*!<  Splitter AFE25 E5B Configuration Register Bits Set */
#define CFGREG_SPLIT25_E5B_CFG_LPF_K_PARAM_BITS_SET(x)                         ((x << CFGREG_SPLIT25_E5B_CFG_LPF_K_PARAM_SHIFT) & CFGREG_SPLIT25_E5B_CFG_LPF_K_PARAM_MASK)                              /*!<  Splitter AFE25 E5B Configuration Register Bits Set */
#define CFGREG_SPLIT25_E5B_CFG_IQ_SAME_GAIN_BITS_SET(x)                        ((x << CFGREG_SPLIT25_E5B_CFG_IQ_SAME_GAIN_SHIFT) & CFGREG_SPLIT25_E5B_CFG_IQ_SAME_GAIN_MASK)                            /*!<  Splitter AFE25 E5B Configuration Register Bits Set */

#define CFGREG_SPLIT25_E5B_CFG_CARR_FREQ_STEP_BITS_SET(x)                      ((x << CFGREG_SPLIT25_E5B_CFG_CARR_FREQ_STEP_SHIFT) & CFGREG_SPLIT25_E5B_CFG_CARR_FREQ_STEP_MASK)                        /*!<  Splitter AFE25 E5B Carrier Frequency Configuration Register Bits Set */

#define CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_MAX_BITS_SET(x)                     ((x << CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_MAX_SHIFT) & CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_MAX_MASK)                      /*!<  Splitter AFE25 E5B Threshold I Configuration Register Bits Set */
#define CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_MIN_BITS_SET(x)                     ((x << CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_MIN_SHIFT) & CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_MIN_MASK)                      /*!<  Splitter AFE25 E5B Threshold I Configuration Register Bits Set */

#define CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_MAX_BITS_SET(x)                     ((x << CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_MAX_SHIFT) & CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_MAX_MASK)                      /*!<  Splitter AFE25 E5B Threshold Q Configuration Register Bits Set */
#define CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_MIN_BITS_SET(x)                     ((x << CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_MIN_SHIFT) & CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_MIN_MASK)                      /*!<  Splitter AFE25 E5B Threshold Q Configuration Register Bits Set */

#define CFGREG_SPLIT25_E5B_CFG_MANUAL_MODE_DIG_GAIN_Q_BITS_SET(x)              ((x << CFGREG_SPLIT25_E5B_CFG_MANUAL_MODE_DIG_GAIN_Q_SHIFT) & CFGREG_SPLIT25_E5B_CFG_MANUAL_MODE_DIG_GAIN_Q_MASK)        /*!<  Splitter AFE25 E5B Manual Mode Digital Gain Configuration Register Bits Set */
#define CFGREG_SPLIT25_E5B_CFG_MANUAL_MODE_DIG_GAIN_I_BITS_SET(x)              ((x << CFGREG_SPLIT25_E5B_CFG_MANUAL_MODE_DIG_GAIN_I_SHIFT) & CFGREG_SPLIT25_E5B_CFG_MANUAL_MODE_DIG_GAIN_I_MASK)        /*!<  Splitter AFE25 E5B Manual Mode Digital Gain Configuration Register Bits Set */

#define CFGREG_LNA125_CONF_L25_TUNE_SRC_BITS_SET(x)                            ((x << CFGREG_LNA125_CONF_L25_TUNE_SRC_SHIFT) & CFGREG_LNA125_CONF_L25_TUNE_SRC_MASK)                                    /*!<  LNA125 Configuration Register Bits Set */
#define CFGREG_LNA125_CONF_L1_TUNE_SRC_BITS_SET(x)                             ((x << CFGREG_LNA125_CONF_L1_TUNE_SRC_SHIFT) & CFGREG_LNA125_CONF_L1_TUNE_SRC_MASK)                                      /*!<  LNA125 Configuration Register Bits Set */
#define CFGREG_LNA125_CONF_EN_L25_BITS_SET(x)                                  ((x << CFGREG_LNA125_CONF_EN_L25_SHIFT) & CFGREG_LNA125_CONF_EN_L25_MASK)                                                /*!<  LNA125 Configuration Register Bits Set */
#define CFGREG_LNA125_CONF_EN_L1_BITS_SET(x)                                   ((x << CFGREG_LNA125_CONF_EN_L1_SHIFT) & CFGREG_LNA125_CONF_EN_L1_MASK)                                                  /*!<  LNA125 Configuration Register Bits Set */

#define CFGREG_LNA125_TUNE_CONF_L25_BITS_SET(x)                                ((x << CFGREG_LNA125_TUNE_CONF_L25_SHIFT) & CFGREG_LNA125_TUNE_CONF_L25_MASK)                                            /*!<  LNA125 Configuration Register Bits Set */
#define CFGREG_LNA125_TUNE_CONF_L1_BITS_SET(x)                                 ((x << CFGREG_LNA125_TUNE_CONF_L1_SHIFT) & CFGREG_LNA125_TUNE_CONF_L1_MASK)                                              /*!<  LNA125 Configuration Register Bits Set */

#define CFGREG_BALUN_MIXER1_CONF_BALUN_TUNE_SRC_BITS_SET(x)                    ((x << CFGREG_BALUN_MIXER1_CONF_BALUN_TUNE_SRC_SHIFT) & CFGREG_BALUN_MIXER1_CONF_BALUN_TUNE_SRC_MASK)                    /*!<  BALUN and MIXER 1 Configuration Register Bits Set */
#define CFGREG_BALUN_MIXER1_CONF_BALUN_TUNE_BITS_SET(x)                        ((x << CFGREG_BALUN_MIXER1_CONF_BALUN_TUNE_SHIFT) & CFGREG_BALUN_MIXER1_CONF_BALUN_TUNE_MASK)                            /*!<  BALUN and MIXER 1 Configuration Register Bits Set */
#define CFGREG_BALUN_MIXER1_CONF_BALUN_EN_BITS_SET(x)                          ((x << CFGREG_BALUN_MIXER1_CONF_BALUN_EN_SHIFT) & CFGREG_BALUN_MIXER1_CONF_BALUN_EN_MASK)                                /*!<  BALUN and MIXER 1 Configuration Register Bits Set */
#define CFGREG_BALUN_MIXER1_CONF_MIXER_EN_BITS_SET(x)                          ((x << CFGREG_BALUN_MIXER1_CONF_MIXER_EN_SHIFT) & CFGREG_BALUN_MIXER1_CONF_MIXER_EN_MASK)                                /*!<  BALUN and MIXER 1 Configuration Register Bits Set */

#define CFGREG_BALUN_MIXER25_CONF_BALUN_TUNE_SRC_BITS_SET(x)                   ((x << CFGREG_BALUN_MIXER25_CONF_BALUN_TUNE_SRC_SHIFT) & CFGREG_BALUN_MIXER25_CONF_BALUN_TUNE_SRC_MASK)                  /*!<  BALUN and MIXER 25 Configuration Register Bits Set */
#define CFGREG_BALUN_MIXER25_CONF_BALUN_TUNE_BITS_SET(x)                       ((x << CFGREG_BALUN_MIXER25_CONF_BALUN_TUNE_SHIFT) & CFGREG_BALUN_MIXER25_CONF_BALUN_TUNE_MASK)                          /*!<  BALUN and MIXER 25 Configuration Register Bits Set */
#define CFGREG_BALUN_MIXER25_CONF_BALUN_EN_BITS_SET(x)                         ((x << CFGREG_BALUN_MIXER25_CONF_BALUN_EN_SHIFT) & CFGREG_BALUN_MIXER25_CONF_BALUN_EN_MASK)                              /*!<  BALUN and MIXER 25 Configuration Register Bits Set */
#define CFGREG_BALUN_MIXER25_CONF_MIXER_EN_BITS_SET(x)                         ((x << CFGREG_BALUN_MIXER25_CONF_MIXER_EN_SHIFT) & CFGREG_BALUN_MIXER25_CONF_MIXER_EN_MASK)                              /*!<  BALUN and MIXER 25 Configuration Register Bits Set */

#define CFGREG_PMU_CONF_FLASH_POR_EN_BITS_SET(x)                               ((x << CFGREG_PMU_CONF_FLASH_POR_EN_SHIFT) & CFGREG_PMU_CONF_FLASH_POR_EN_MASK)                                          /*!<  PMU Configuration register Bits Set */
#define CFGREG_PMU_CONF_FLASH_BOD_EN_BITS_SET(x)                               ((x << CFGREG_PMU_CONF_FLASH_BOD_EN_SHIFT) & CFGREG_PMU_CONF_FLASH_BOD_EN_MASK)                                          /*!<  PMU Configuration register Bits Set */
#define CFGREG_PMU_CONF_DCDC_MODE_BITS_SET(x)                                  ((x << CFGREG_PMU_CONF_DCDC_MODE_SHIFT) & CFGREG_PMU_CONF_DCDC_MODE_MASK)                                                /*!<  PMU Configuration register Bits Set */
#define CFGREG_PMU_CONF_DCDC_GENTRIM_SRC_BITS_SET(x)                           ((x << CFGREG_PMU_CONF_DCDC_GENTRIM_SRC_SHIFT) & CFGREG_PMU_CONF_DCDC_GENTRIM_SRC_MASK)                                  /*!<  PMU Configuration register Bits Set */
#define CFGREG_PMU_CONF_DCDC_GENTRIM_BITS_SET(x)                               ((x << CFGREG_PMU_CONF_DCDC_GENTRIM_SHIFT) & CFGREG_PMU_CONF_DCDC_GENTRIM_MASK)                                          /*!<  PMU Configuration register Bits Set */
#define CFGREG_PMU_CONF_DCDC_TRIM_RESET_BITS_SET(x)                            ((x << CFGREG_PMU_CONF_DCDC_TRIM_RESET_SHIFT) & CFGREG_PMU_CONF_DCDC_TRIM_RESET_MASK)                                    /*!<  PMU Configuration register Bits Set */
#define CFGREG_PMU_CONF_IREF_CAL_EN_BITS_SET(x)                                ((x << CFGREG_PMU_CONF_IREF_CAL_EN_SHIFT) & CFGREG_PMU_CONF_IREF_CAL_EN_MASK)                                            /*!<  PMU Configuration register Bits Set */
#define CFGREG_PMU_CONF_IREF_TRIM_VAL_SRC_BITS_SET(x)                          ((x << CFGREG_PMU_CONF_IREF_TRIM_VAL_SRC_SHIFT) & CFGREG_PMU_CONF_IREF_TRIM_VAL_SRC_MASK)                                /*!<  PMU Configuration register Bits Set */
#define CFGREG_PMU_CONF_IREF_TRIM_VAL_BITS_SET(x)                              ((x << CFGREG_PMU_CONF_IREF_TRIM_VAL_SHIFT) & CFGREG_PMU_CONF_IREF_TRIM_VAL_MASK)                                        /*!<  PMU Configuration register Bits Set */
#define CFGREG_PMU_CONF_IREF_TRIM_EN_BITS_SET(x)                               ((x << CFGREG_PMU_CONF_IREF_TRIM_EN_SHIFT) & CFGREG_PMU_CONF_IREF_TRIM_EN_MASK)                                          /*!<  PMU Configuration register Bits Set */

#define CFGREG_PMU_STAT_IREF_TRIM_VAL_BITS_SET(x)                              ((x << CFGREG_PMU_STAT_IREF_TRIM_VAL_SHIFT) & CFGREG_PMU_STAT_IREF_TRIM_VAL_MASK)                                        /*!<  Power Management Unit Status Bits Set */
#define CFGREG_PMU_STAT_IREF_CAL_RDY_BITS_SET(x)                               ((x << CFGREG_PMU_STAT_IREF_CAL_RDY_SHIFT) & CFGREG_PMU_STAT_IREF_CAL_RDY_MASK)                                          /*!<  Power Management Unit Status Bits Set */

#define CFGREG_PM_STAT_IREF_COMP_BITS_SET(x)                                   ((x << CFGREG_PM_STAT_IREF_COMP_SHIFT) & CFGREG_PM_STAT_IREF_COMP_MASK)                                                  /*!<  GNSS-AFE PM Status Register Bits Set */
#define CFGREG_PM_STAT_VREF_COMP_BITS_SET(x)                                   ((x << CFGREG_PM_STAT_VREF_COMP_SHIFT) & CFGREG_PM_STAT_VREF_COMP_MASK)                                                  /*!<  GNSS-AFE PM Status Register Bits Set */
#define CFGREG_PM_STAT_PWR_UP_P_BITS_SET(x)                                    ((x << CFGREG_PM_STAT_PWR_UP_P_SHIFT) & CFGREG_PM_STAT_PWR_UP_P_MASK)                                                    /*!<  GNSS-AFE PM Status Register Bits Set */
#define CFGREG_PM_STAT_PWR_UP_N_BITS_SET(x)                                    ((x << CFGREG_PM_STAT_PWR_UP_N_SHIFT) & CFGREG_PM_STAT_PWR_UP_N_MASK)                                                    /*!<  GNSS-AFE PM Status Register Bits Set */

#define CFGREG_PM_CONF_LDO_ADC_EXT_DECAP_BITS_SET(x)                           ((x << CFGREG_PM_CONF_LDO_ADC_EXT_DECAP_SHIFT) & CFGREG_PM_CONF_LDO_ADC_EXT_DECAP_MASK)                                  /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_LDO_IF_EXT_DECAP_BITS_SET(x)                            ((x << CFGREG_PM_CONF_LDO_IF_EXT_DECAP_SHIFT) & CFGREG_PM_CONF_LDO_IF_EXT_DECAP_MASK)                                    /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_LDO_DPLL_EXT_DECAP_BITS_SET(x)                          ((x << CFGREG_PM_CONF_LDO_DPLL_EXT_DECAP_SHIFT) & CFGREG_PM_CONF_LDO_DPLL_EXT_DECAP_MASK)                                /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_LDO_APLL_EXT_DECAP_BITS_SET(x)                          ((x << CFGREG_PM_CONF_LDO_APLL_EXT_DECAP_SHIFT) & CFGREG_PM_CONF_LDO_APLL_EXT_DECAP_MASK)                                /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_LDO_RF_EXT_DECAP_BITS_SET(x)                            ((x << CFGREG_PM_CONF_LDO_RF_EXT_DECAP_SHIFT) & CFGREG_PM_CONF_LDO_RF_EXT_DECAP_MASK)                                    /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_RC_FILTER_BITS_SET(x)                                   ((x << CFGREG_PM_CONF_RC_FILTER_SHIFT) & CFGREG_PM_CONF_RC_FILTER_MASK)                                                  /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_CAL_EN_BITS_SET(x)                                      ((x << CFGREG_PM_CONF_CAL_EN_SHIFT) & CFGREG_PM_CONF_CAL_EN_MASK)                                                        /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_TEST_EN_BITS_SET(x)                                     ((x << CFGREG_PM_CONF_TEST_EN_SHIFT) & CFGREG_PM_CONF_TEST_EN_MASK)                                                      /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_LDO_ADC_EN_BITS_SET(x)                                  ((x << CFGREG_PM_CONF_LDO_ADC_EN_SHIFT) & CFGREG_PM_CONF_LDO_ADC_EN_MASK)                                                /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_LDO_IF_EN_BITS_SET(x)                                   ((x << CFGREG_PM_CONF_LDO_IF_EN_SHIFT) & CFGREG_PM_CONF_LDO_IF_EN_MASK)                                                  /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_LDO_DPLL_EN_BITS_SET(x)                                 ((x << CFGREG_PM_CONF_LDO_DPLL_EN_SHIFT) & CFGREG_PM_CONF_LDO_DPLL_EN_MASK)                                              /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_LDO_APLL_EN_BITS_SET(x)                                 ((x << CFGREG_PM_CONF_LDO_APLL_EN_SHIFT) & CFGREG_PM_CONF_LDO_APLL_EN_MASK)                                              /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_LDO_RF_EN_BITS_SET(x)                                   ((x << CFGREG_PM_CONF_LDO_RF_EN_SHIFT) & CFGREG_PM_CONF_LDO_RF_EN_MASK)                                                  /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_VREF_EN_BITS_SET(x)                                     ((x << CFGREG_PM_CONF_VREF_EN_SHIFT) & CFGREG_PM_CONF_VREF_EN_MASK)                                                      /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_IREF_TRIM_SRC_BITS_SET(x)                               ((x << CFGREG_PM_CONF_IREF_TRIM_SRC_SHIFT) & CFGREG_PM_CONF_IREF_TRIM_SRC_MASK)                                          /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_IREF_TRIM_BITS_SET(x)                                   ((x << CFGREG_PM_CONF_IREF_TRIM_SHIFT) & CFGREG_PM_CONF_IREF_TRIM_MASK)                                                  /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_IREF_EN_BITS_SET(x)                                     ((x << CFGREG_PM_CONF_IREF_EN_SHIFT) & CFGREG_PM_CONF_IREF_EN_MASK)                                                      /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_BGVR_TRIM_SRC_BITS_SET(x)                               ((x << CFGREG_PM_CONF_BGVR_TRIM_SRC_SHIFT) & CFGREG_PM_CONF_BGVR_TRIM_SRC_MASK)                                          /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_BGVR_TRIM_BITS_SET(x)                                   ((x << CFGREG_PM_CONF_BGVR_TRIM_SHIFT) & CFGREG_PM_CONF_BGVR_TRIM_MASK)                                                  /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_BGVR_EN_BITS_SET(x)                                     ((x << CFGREG_PM_CONF_BGVR_EN_SHIFT) & CFGREG_PM_CONF_BGVR_EN_MASK)                                                      /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_PM_CONF_NRST_BITS_SET(x)                                        ((x << CFGREG_PM_CONF_NRST_SHIFT) & CFGREG_PM_CONF_NRST_MASK)                                                            /*!<  GNSS-AFE PM Configuration Register Bits Set */

#define CFGREG_GNSSAFE_TEST_CONF_PLLTEST_ATT_BITS_SET(x)                       ((x << CFGREG_GNSSAFE_TEST_CONF_PLLTEST_ATT_SHIFT) & CFGREG_GNSSAFE_TEST_CONF_PLLTEST_ATT_MASK)                          /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_EN_BITS_SET(x)                  ((x << CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_EN_SHIFT) & CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_EN_MASK)                /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_DIR_BITS_SET(x)                 ((x << CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_DIR_SHIFT) & CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_DIR_MASK)              /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_PRESC_BITS_SET(x)               ((x << CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_PRESC_SHIFT) & CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_PRESC_MASK)          /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_EN_BITS_SET(x)                   ((x << CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_EN_SHIFT) & CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_EN_MASK)                  /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_DIR_BITS_SET(x)                  ((x << CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_DIR_SHIFT) & CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_DIR_MASK)                /*!<  GNSS-AFE PM Configuration Register Bits Set */
#define CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_PRESC_BITS_SET(x)                ((x << CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_PRESC_SHIFT) & CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_PRESC_MASK)            /*!<  GNSS-AFE PM Configuration Register Bits Set */

#define CFGREG_SPARE_CONF_SIG_BITS_SET(x)                                      ((x << CFGREG_SPARE_CONF_SIG_SHIFT) & CFGREG_SPARE_CONF_SIG_MASK)                                                        /*!<  SPARE Configuration Register Bits Set */

#define CFGREG_SPARE_STAT_SIG_BITS_SET(x)                                      ((x << CFGREG_SPARE_STAT_SIG_SHIFT) & CFGREG_SPARE_STAT_SIG_MASK)                                                        /*!<  SPARE Configuration Register Bits Set */


/** Bits Get Helper Macros */

#define CFGREG_IRQMAP_IRQMAP_BITS_GET(x)                                       ((x & CFGREG_IRQMAP_IRQMAP_MASK) >> CFGREG_IRQMAP_IRQMAP_SHIFT)                                                          /*!<  Interrupt Mapping Register Bits Get */

#define CFGREG_UNLOCK_UNLOCK_KEY_BITS_GET(x)                                   ((x & CFGREG_UNLOCK_UNLOCK_KEY_MASK) >> CFGREG_UNLOCK_UNLOCK_KEY_SHIFT)                                                  /*!<  Unlock Register Bits Get */

#define CFGREG_IRQFLAGS_PLL_FAIL_BITS_GET(x)                                   ((x & CFGREG_IRQFLAGS_PLL_FAIL_MASK) >> CFGREG_IRQFLAGS_PLL_FAIL_SHIFT)                                                  /*!<  32kHz RTC Clock Interrupt Flags Bits Get */
#define CFGREG_IRQFLAGS_XTAL_CORE_FAIL_BITS_GET(x)                             ((x & CFGREG_IRQFLAGS_XTAL_CORE_FAIL_MASK) >> CFGREG_IRQFLAGS_XTAL_CORE_FAIL_SHIFT)                                      /*!<  32kHz RTC Clock Interrupt Flags Bits Get */
#define CFGREG_IRQFLAGS_XTAL_RTC_FAIL_BITS_GET(x)                              ((x & CFGREG_IRQFLAGS_XTAL_RTC_FAIL_MASK) >> CFGREG_IRQFLAGS_XTAL_RTC_FAIL_SHIFT)                                        /*!<  32kHz RTC Clock Interrupt Flags Bits Get */

#define CFGREG_MEM_CONF_TEST1_BITS_GET(x)                                      ((x & CFGREG_MEM_CONF_TEST1_MASK) >> CFGREG_MEM_CONF_TEST1_SHIFT)                                                        /*!<  Memory Configuration Register Bits Get */
#define CFGREG_MEM_CONF_RME_BITS_GET(x)                                        ((x & CFGREG_MEM_CONF_RME_MASK) >> CFGREG_MEM_CONF_RME_SHIFT)                                                            /*!<  Memory Configuration Register Bits Get */
#define CFGREG_MEM_CONF_RM_BITS_GET(x)                                         ((x & CFGREG_MEM_CONF_RM_MASK) >> CFGREG_MEM_CONF_RM_SHIFT)                                                              /*!<  Memory Configuration Register Bits Get */

#define CFGREG_RTCSTAT_RC_RTC_VAL_BITS_GET(x)                                  ((x & CFGREG_RTCSTAT_RC_RTC_VAL_MASK) >> CFGREG_RTCSTAT_RC_RTC_VAL_SHIFT)                                                /*!<  32kHz RTC Clock Status Bits Get */
#define CFGREG_RTCSTAT_RC_RTC_RDY_BITS_GET(x)                                  ((x & CFGREG_RTCSTAT_RC_RTC_RDY_MASK) >> CFGREG_RTCSTAT_RC_RTC_RDY_SHIFT)                                                /*!<  32kHz RTC Clock Status Bits Get */
#define CFGREG_RTCSTAT_CLK_RTC_SEL_BITS_GET(x)                                 ((x & CFGREG_RTCSTAT_CLK_RTC_SEL_MASK) >> CFGREG_RTCSTAT_CLK_RTC_SEL_SHIFT)                                              /*!<  32kHz RTC Clock Status Bits Get */

#define CFGREG_RTCCONF_RC_RTC_VAL_SRC_BITS_GET(x)                              ((x & CFGREG_RTCCONF_RC_RTC_VAL_SRC_MASK) >> CFGREG_RTCCONF_RC_RTC_VAL_SRC_SHIFT)                                        /*!<  32kHz RTC Clock Configuration Bits Get */
#define CFGREG_RTCCONF_RC_RTC_VAL_BITS_GET(x)                                  ((x & CFGREG_RTCCONF_RC_RTC_VAL_MASK) >> CFGREG_RTCCONF_RC_RTC_VAL_SHIFT)                                                /*!<  32kHz RTC Clock Configuration Bits Get */
#define CFGREG_RTCCONF_RC_RTC_CAL_BITS_GET(x)                                  ((x & CFGREG_RTCCONF_RC_RTC_CAL_MASK) >> CFGREG_RTCCONF_RC_RTC_CAL_SHIFT)                                                /*!<  32kHz RTC Clock Configuration Bits Get */
#define CFGREG_RTCCONF_XTAL_RTC_SEL_BITS_GET(x)                                ((x & CFGREG_RTCCONF_XTAL_RTC_SEL_MASK) >> CFGREG_RTCCONF_XTAL_RTC_SEL_SHIFT)                                            /*!<  32kHz RTC Clock Configuration Bits Get */
#define CFGREG_RTCCONF_XTAL_RTC_TEST_BITS_GET(x)                               ((x & CFGREG_RTCCONF_XTAL_RTC_TEST_MASK) >> CFGREG_RTCCONF_XTAL_RTC_TEST_SHIFT)                                          /*!<  32kHz RTC Clock Configuration Bits Get */

#define CFGREG_COREFREQ_CLK_TEST_OUT_EN_BITS_GET(x)                            ((x & CFGREG_COREFREQ_CLK_TEST_OUT_EN_MASK) >> CFGREG_COREFREQ_CLK_TEST_OUT_EN_SHIFT)                                    /*!<  External XTAL Configuration Register Bits Get */
#define CFGREG_COREFREQ_CLK_RC_CORE_VAL_SRC_BITS_GET(x)                        ((x & CFGREG_COREFREQ_CLK_RC_CORE_VAL_SRC_MASK) >> CFGREG_COREFREQ_CLK_RC_CORE_VAL_SRC_SHIFT)                            /*!<  External XTAL Configuration Register Bits Get */
#define CFGREG_COREFREQ_CLK_RC_CORE_VAL_BITS_GET(x)                            ((x & CFGREG_COREFREQ_CLK_RC_CORE_VAL_MASK) >> CFGREG_COREFREQ_CLK_RC_CORE_VAL_SHIFT)                                    /*!<  External XTAL Configuration Register Bits Get */
#define CFGREG_COREFREQ_CLK_RC_CORE_CAL_BITS_GET(x)                            ((x & CFGREG_COREFREQ_CLK_RC_CORE_CAL_MASK) >> CFGREG_COREFREQ_CLK_RC_CORE_CAL_SHIFT)                                    /*!<  External XTAL Configuration Register Bits Get */
#define CFGREG_COREFREQ_CLK_CORE_SEL_BITS_GET(x)                               ((x & CFGREG_COREFREQ_CLK_CORE_SEL_MASK) >> CFGREG_COREFREQ_CLK_CORE_SEL_SHIFT)                                          /*!<  External XTAL Configuration Register Bits Get */
#define CFGREG_COREFREQ_CLK_XTAL_CORE_TEST_BITS_GET(x)                         ((x & CFGREG_COREFREQ_CLK_XTAL_CORE_TEST_MASK) >> CFGREG_COREFREQ_CLK_XTAL_CORE_TEST_SHIFT)                              /*!<  External XTAL Configuration Register Bits Get */

#define CFGREG_COREFREQ_PLL_CTRL_FINE_LOAD_BITS_GET(x)                         ((x & CFGREG_COREFREQ_PLL_CTRL_FINE_LOAD_MASK) >> CFGREG_COREFREQ_PLL_CTRL_FINE_LOAD_SHIFT)                              /*!<  PLL Configuration Register Bits Get */
#define CFGREG_COREFREQ_PLL_CTRL_LOAD_BITS_GET(x)                              ((x & CFGREG_COREFREQ_PLL_CTRL_LOAD_MASK) >> CFGREG_COREFREQ_PLL_CTRL_LOAD_SHIFT)                                        /*!<  PLL Configuration Register Bits Get */
#define CFGREG_COREFREQ_PLL_CTRL_PVT_LOAD_BITS_GET(x)                          ((x & CFGREG_COREFREQ_PLL_CTRL_PVT_LOAD_MASK) >> CFGREG_COREFREQ_PLL_CTRL_PVT_LOAD_SHIFT)                                /*!<  PLL Configuration Register Bits Get */
#define CFGREG_COREFREQ_PLL_PLL_VFB_EN_BITS_GET(x)                             ((x & CFGREG_COREFREQ_PLL_PLL_VFB_EN_MASK) >> CFGREG_COREFREQ_PLL_PLL_VFB_EN_SHIFT)                                      /*!<  PLL Configuration Register Bits Get */
#define CFGREG_COREFREQ_PLL_TEST_BITS_GET(x)                                   ((x & CFGREG_COREFREQ_PLL_TEST_MASK) >> CFGREG_COREFREQ_PLL_TEST_SHIFT)                                                  /*!<  PLL Configuration Register Bits Get */
#define CFGREG_COREFREQ_PLL_N_BITS_GET(x)                                      ((x & CFGREG_COREFREQ_PLL_N_MASK) >> CFGREG_COREFREQ_PLL_N_SHIFT)                                                        /*!<  PLL Configuration Register Bits Get */
#define CFGREG_COREFREQ_PLL_REF_SEL_BITS_GET(x)                                ((x & CFGREG_COREFREQ_PLL_REF_SEL_MASK) >> CFGREG_COREFREQ_PLL_REF_SEL_SHIFT)                                            /*!<  PLL Configuration Register Bits Get */
#define CFGREG_COREFREQ_PLL_EN_BITS_GET(x)                                     ((x & CFGREG_COREFREQ_PLL_EN_MASK) >> CFGREG_COREFREQ_PLL_EN_SHIFT)                                                      /*!<  PLL Configuration Register Bits Get */

#define CFGREG_COREFREQ_STAT_CTRL_FINE_BITS_GET(x)                             ((x & CFGREG_COREFREQ_STAT_CTRL_FINE_MASK) >> CFGREG_COREFREQ_STAT_CTRL_FINE_SHIFT)                                      /*!<  Core Clock Status Register Bits Get */
#define CFGREG_COREFREQ_STAT_CTRL_PVT_BITS_GET(x)                              ((x & CFGREG_COREFREQ_STAT_CTRL_PVT_MASK) >> CFGREG_COREFREQ_STAT_CTRL_PVT_SHIFT)                                        /*!<  Core Clock Status Register Bits Get */
#define CFGREG_COREFREQ_STAT_RC_CORE_VAL_BITS_GET(x)                           ((x & CFGREG_COREFREQ_STAT_RC_CORE_VAL_MASK) >> CFGREG_COREFREQ_STAT_RC_CORE_VAL_SHIFT)                                  /*!<  Core Clock Status Register Bits Get */
#define CFGREG_COREFREQ_STAT_RC_CORE_RDY_BITS_GET(x)                           ((x & CFGREG_COREFREQ_STAT_RC_CORE_RDY_MASK) >> CFGREG_COREFREQ_STAT_RC_CORE_RDY_SHIFT)                                  /*!<  Core Clock Status Register Bits Get */
#define CFGREG_COREFREQ_STAT_PLL_LOCK_BITS_GET(x)                              ((x & CFGREG_COREFREQ_STAT_PLL_LOCK_MASK) >> CFGREG_COREFREQ_STAT_PLL_LOCK_SHIFT)                                        /*!<  Core Clock Status Register Bits Get */
#define CFGREG_COREFREQ_STAT_XTAL_CORE_LOCK_BITS_GET(x)                        ((x & CFGREG_COREFREQ_STAT_XTAL_CORE_LOCK_MASK) >> CFGREG_COREFREQ_STAT_XTAL_CORE_LOCK_SHIFT)                            /*!<  Core Clock Status Register Bits Get */
#define CFGREG_COREFREQ_STAT_CLK_CORE_SEL_BITS_GET(x)                          ((x & CFGREG_COREFREQ_STAT_CLK_CORE_SEL_MASK) >> CFGREG_COREFREQ_STAT_CLK_CORE_SEL_SHIFT)                                /*!<  Core Clock Status Register Bits Get */

#define CFGREG_GNSSAFE_CONF_GNSS_OUT_BAND_BITS_GET(x)                          ((x & CFGREG_GNSSAFE_CONF_GNSS_OUT_BAND_MASK) >> CFGREG_GNSSAFE_CONF_GNSS_OUT_BAND_SHIFT)                                /*!<  GNSS-AFE Configuration Register Bits Get */
#define CFGREG_GNSSAFE_CONF_GNSS_AUX1_EN_BITS_GET(x)                           ((x & CFGREG_GNSSAFE_CONF_GNSS_AUX1_EN_MASK) >> CFGREG_GNSSAFE_CONF_GNSS_AUX1_EN_SHIFT)                                  /*!<  GNSS-AFE Configuration Register Bits Get */
#define CFGREG_GNSSAFE_CONF_GNSS_AUX0_EN_BITS_GET(x)                           ((x & CFGREG_GNSSAFE_CONF_GNSS_AUX0_EN_MASK) >> CFGREG_GNSSAFE_CONF_GNSS_AUX0_EN_SHIFT)                                  /*!<  GNSS-AFE Configuration Register Bits Get */

#define CFGREG_PLL1_CONF_EN_BOOST_BITS_GET(x)                                  ((x & CFGREG_PLL1_CONF_EN_BOOST_MASK) >> CFGREG_PLL1_CONF_EN_BOOST_SHIFT)                                                /*!<  PLL1 Configuration Register Bits Get */
#define CFGREG_PLL1_CONF_ADC_CLK_DIV_BITS_GET(x)                               ((x & CFGREG_PLL1_CONF_ADC_CLK_DIV_MASK) >> CFGREG_PLL1_CONF_ADC_CLK_DIV_SHIFT)                                          /*!<  PLL1 Configuration Register Bits Get */
#define CFGREG_PLL1_CONF_LOPCB_EN_BITS_GET(x)                                  ((x & CFGREG_PLL1_CONF_LOPCB_EN_MASK) >> CFGREG_PLL1_CONF_LOPCB_EN_SHIFT)                                                /*!<  PLL1 Configuration Register Bits Get */
#define CFGREG_PLL1_CONF_TEST_EN_BITS_GET(x)                                   ((x & CFGREG_PLL1_CONF_TEST_EN_MASK) >> CFGREG_PLL1_CONF_TEST_EN_SHIFT)                                                  /*!<  PLL1 Configuration Register Bits Get */
#define CFGREG_PLL1_CONF_FCW_BITS_GET(x)                                       ((x & CFGREG_PLL1_CONF_FCW_MASK) >> CFGREG_PLL1_CONF_FCW_SHIFT)                                                          /*!<  PLL1 Configuration Register Bits Get */
#define CFGREG_PLL1_CONF_EN_BITS_GET(x)                                        ((x & CFGREG_PLL1_CONF_EN_MASK) >> CFGREG_PLL1_CONF_EN_SHIFT)                                                            /*!<  PLL1 Configuration Register Bits Get */

#define CFGREG_PLL1DCO_CONF_AMP_BITS_GET(x)                                    ((x & CFGREG_PLL1DCO_CONF_AMP_MASK) >> CFGREG_PLL1DCO_CONF_AMP_SHIFT)                                                    /*!<  PLL1 DCO Configuration Register Bits Get */
#define CFGREG_PLL1DCO_CONF_AMP_LOAD_BITS_GET(x)                               ((x & CFGREG_PLL1DCO_CONF_AMP_LOAD_MASK) >> CFGREG_PLL1DCO_CONF_AMP_LOAD_SHIFT)                                          /*!<  PLL1 DCO Configuration Register Bits Get */
#define CFGREG_PLL1DCO_CONF_CTRL_PVT_BITS_GET(x)                               ((x & CFGREG_PLL1DCO_CONF_CTRL_PVT_MASK) >> CFGREG_PLL1DCO_CONF_CTRL_PVT_SHIFT)                                          /*!<  PLL1 DCO Configuration Register Bits Get */
#define CFGREG_PLL1DCO_CONF_CTRL_FINE_BITS_GET(x)                              ((x & CFGREG_PLL1DCO_CONF_CTRL_FINE_MASK) >> CFGREG_PLL1DCO_CONF_CTRL_FINE_SHIFT)                                        /*!<  PLL1 DCO Configuration Register Bits Get */
#define CFGREG_PLL1DCO_CONF_CTRL_LOAD_BITS_GET(x)                              ((x & CFGREG_PLL1DCO_CONF_CTRL_LOAD_MASK) >> CFGREG_PLL1DCO_CONF_CTRL_LOAD_SHIFT)                                        /*!<  PLL1 DCO Configuration Register Bits Get */

#define CFGREG_PLL1_STAT_PH_VAR_BITS_GET(x)                                    ((x & CFGREG_PLL1_STAT_PH_VAR_MASK) >> CFGREG_PLL1_STAT_PH_VAR_SHIFT)                                                    /*!<  PLL1 Status Register Bits Get */
#define CFGREG_PLL1_STAT_PHERR_BITS_GET(x)                                     ((x & CFGREG_PLL1_STAT_PHERR_MASK) >> CFGREG_PLL1_STAT_PHERR_SHIFT)                                                      /*!<  PLL1 Status Register Bits Get */
#define CFGREG_PLL1_STAT_LOCK_BITS_GET(x)                                      ((x & CFGREG_PLL1_STAT_LOCK_MASK) >> CFGREG_PLL1_STAT_LOCK_SHIFT)                                                        /*!<  PLL1 Status Register Bits Get */

#define CFGREG_PLL1_STAT2_TV_BITS_GET(x)                                       ((x & CFGREG_PLL1_STAT2_TV_MASK) >> CFGREG_PLL1_STAT2_TV_SHIFT)                                                          /*!<  PLL1 Status Register Bits Get */

#define CFGREG_PLL1DCO_STAT_AMP_BITS_GET(x)                                    ((x & CFGREG_PLL1DCO_STAT_AMP_MASK) >> CFGREG_PLL1DCO_STAT_AMP_SHIFT)                                                    /*!<  PLL1 TDC Status Register Bits Get */
#define CFGREG_PLL1DCO_STAT_AMP_LOW_BITS_GET(x)                                ((x & CFGREG_PLL1DCO_STAT_AMP_LOW_MASK) >> CFGREG_PLL1DCO_STAT_AMP_LOW_SHIFT)                                            /*!<  PLL1 TDC Status Register Bits Get */
#define CFGREG_PLL1DCO_STAT_CTRL_PVT_BITS_GET(x)                               ((x & CFGREG_PLL1DCO_STAT_CTRL_PVT_MASK) >> CFGREG_PLL1DCO_STAT_CTRL_PVT_SHIFT)                                          /*!<  PLL1 TDC Status Register Bits Get */
#define CFGREG_PLL1DCO_STAT_CTRL_FINE_BITS_GET(x)                              ((x & CFGREG_PLL1DCO_STAT_CTRL_FINE_MASK) >> CFGREG_PLL1DCO_STAT_CTRL_FINE_SHIFT)                                        /*!<  PLL1 TDC Status Register Bits Get */

#define CFGREG_PLL25_CONF_EN_BOOST_BITS_GET(x)                                 ((x & CFGREG_PLL25_CONF_EN_BOOST_MASK) >> CFGREG_PLL25_CONF_EN_BOOST_SHIFT)                                              /*!<  PLL25 Configuration Register Bits Get */
#define CFGREG_PLL25_CONF_ADC_CLK_DIV_BITS_GET(x)                              ((x & CFGREG_PLL25_CONF_ADC_CLK_DIV_MASK) >> CFGREG_PLL25_CONF_ADC_CLK_DIV_SHIFT)                                        /*!<  PLL25 Configuration Register Bits Get */
#define CFGREG_PLL25_CONF_LOPCB_EN_BITS_GET(x)                                 ((x & CFGREG_PLL25_CONF_LOPCB_EN_MASK) >> CFGREG_PLL25_CONF_LOPCB_EN_SHIFT)                                              /*!<  PLL25 Configuration Register Bits Get */
#define CFGREG_PLL25_CONF_TEST_EN_BITS_GET(x)                                  ((x & CFGREG_PLL25_CONF_TEST_EN_MASK) >> CFGREG_PLL25_CONF_TEST_EN_SHIFT)                                                /*!<  PLL25 Configuration Register Bits Get */
#define CFGREG_PLL25_CONF_FCW_BITS_GET(x)                                      ((x & CFGREG_PLL25_CONF_FCW_MASK) >> CFGREG_PLL25_CONF_FCW_SHIFT)                                                        /*!<  PLL25 Configuration Register Bits Get */
#define CFGREG_PLL25_CONF_EN_BITS_GET(x)                                       ((x & CFGREG_PLL25_CONF_EN_MASK) >> CFGREG_PLL25_CONF_EN_SHIFT)                                                          /*!<  PLL25 Configuration Register Bits Get */

#define CFGREG_PLL25DCO_CONF_AMP_BITS_GET(x)                                   ((x & CFGREG_PLL25DCO_CONF_AMP_MASK) >> CFGREG_PLL25DCO_CONF_AMP_SHIFT)                                                  /*!<  PLL25 DCO Configuration Register Bits Get */
#define CFGREG_PLL25DCO_CONF_AMP_LOAD_BITS_GET(x)                              ((x & CFGREG_PLL25DCO_CONF_AMP_LOAD_MASK) >> CFGREG_PLL25DCO_CONF_AMP_LOAD_SHIFT)                                        /*!<  PLL25 DCO Configuration Register Bits Get */
#define CFGREG_PLL25DCO_CONF_CTRL_PVT_BITS_GET(x)                              ((x & CFGREG_PLL25DCO_CONF_CTRL_PVT_MASK) >> CFGREG_PLL25DCO_CONF_CTRL_PVT_SHIFT)                                        /*!<  PLL25 DCO Configuration Register Bits Get */
#define CFGREG_PLL25DCO_CONF_CTRL_FINE_BITS_GET(x)                             ((x & CFGREG_PLL25DCO_CONF_CTRL_FINE_MASK) >> CFGREG_PLL25DCO_CONF_CTRL_FINE_SHIFT)                                      /*!<  PLL25 DCO Configuration Register Bits Get */
#define CFGREG_PLL25DCO_CONF_CTRL_LOAD_BITS_GET(x)                             ((x & CFGREG_PLL25DCO_CONF_CTRL_LOAD_MASK) >> CFGREG_PLL25DCO_CONF_CTRL_LOAD_SHIFT)                                      /*!<  PLL25 DCO Configuration Register Bits Get */

#define CFGREG_PLL25_STAT_PH_VAR_BITS_GET(x)                                   ((x & CFGREG_PLL25_STAT_PH_VAR_MASK) >> CFGREG_PLL25_STAT_PH_VAR_SHIFT)                                                  /*!<  PLL25 Status Register Bits Get */
#define CFGREG_PLL25_STAT_PHERR_BITS_GET(x)                                    ((x & CFGREG_PLL25_STAT_PHERR_MASK) >> CFGREG_PLL25_STAT_PHERR_SHIFT)                                                    /*!<  PLL25 Status Register Bits Get */
#define CFGREG_PLL25_STAT_LOCK_BITS_GET(x)                                     ((x & CFGREG_PLL25_STAT_LOCK_MASK) >> CFGREG_PLL25_STAT_LOCK_SHIFT)                                                      /*!<  PLL25 Status Register Bits Get */

#define CFGREG_PLL25_STAT2_TV_BITS_GET(x)                                      ((x & CFGREG_PLL25_STAT2_TV_MASK) >> CFGREG_PLL25_STAT2_TV_SHIFT)                                                        /*!<  PLL25 Status Register Bits Get */

#define CFGREG_PLL25DCO_STAT_AMP_BITS_GET(x)                                   ((x & CFGREG_PLL25DCO_STAT_AMP_MASK) >> CFGREG_PLL25DCO_STAT_AMP_SHIFT)                                                  /*!<  PLL25 TDC Status Register Bits Get */
#define CFGREG_PLL25DCO_STAT_AMP_LOW_BITS_GET(x)                               ((x & CFGREG_PLL25DCO_STAT_AMP_LOW_MASK) >> CFGREG_PLL25DCO_STAT_AMP_LOW_SHIFT)                                          /*!<  PLL25 TDC Status Register Bits Get */
#define CFGREG_PLL25DCO_STAT_CTRL_PVT_BITS_GET(x)                              ((x & CFGREG_PLL25DCO_STAT_CTRL_PVT_MASK) >> CFGREG_PLL25DCO_STAT_CTRL_PVT_SHIFT)                                        /*!<  PLL25 TDC Status Register Bits Get */
#define CFGREG_PLL25DCO_STAT_CTRL_FINE_BITS_GET(x)                             ((x & CFGREG_PLL25DCO_STAT_CTRL_FINE_MASK) >> CFGREG_PLL25DCO_STAT_CTRL_FINE_SHIFT)                                      /*!<  PLL25 TDC Status Register Bits Get */

#define CFGREG_IF1_CONF_IF_BANDCUT_BITS_GET(x)                                 ((x & CFGREG_IF1_CONF_IF_BANDCUT_MASK) >> CFGREG_IF1_CONF_IF_BANDCUT_SHIFT)                                              /*!<  IF1 Configuration Register Bits Get */
#define CFGREG_IF1_CONF_OFFSET_CAL_Q_BITS_GET(x)                               ((x & CFGREG_IF1_CONF_OFFSET_CAL_Q_MASK) >> CFGREG_IF1_CONF_OFFSET_CAL_Q_SHIFT)                                          /*!<  IF1 Configuration Register Bits Get */
#define CFGREG_IF1_CONF_OFFSET_CAL_DIRECTION_Q_BITS_GET(x)                     ((x & CFGREG_IF1_CONF_OFFSET_CAL_DIRECTION_Q_MASK) >> CFGREG_IF1_CONF_OFFSET_CAL_DIRECTION_Q_SHIFT)                      /*!<  IF1 Configuration Register Bits Get */
#define CFGREG_IF1_CONF_OFFSET_CAL_I_BITS_GET(x)                               ((x & CFGREG_IF1_CONF_OFFSET_CAL_I_MASK) >> CFGREG_IF1_CONF_OFFSET_CAL_I_SHIFT)                                          /*!<  IF1 Configuration Register Bits Get */
#define CFGREG_IF1_CONF_OFFSET_CAL_DIRECTION_I_BITS_GET(x)                     ((x & CFGREG_IF1_CONF_OFFSET_CAL_DIRECTION_I_MASK) >> CFGREG_IF1_CONF_OFFSET_CAL_DIRECTION_I_SHIFT)                      /*!<  IF1 Configuration Register Bits Get */
#define CFGREG_IF1_CONF_PGA2_EN_BITS_GET(x)                                    ((x & CFGREG_IF1_CONF_PGA2_EN_MASK) >> CFGREG_IF1_CONF_PGA2_EN_SHIFT)                                                    /*!<  IF1 Configuration Register Bits Get */
#define CFGREG_IF1_CONF_PGA1_EN_BITS_GET(x)                                    ((x & CFGREG_IF1_CONF_PGA1_EN_MASK) >> CFGREG_IF1_CONF_PGA1_EN_SHIFT)                                                    /*!<  IF1 Configuration Register Bits Get */
#define CFGREG_IF1_CONF_PREAMP_EN_BITS_GET(x)                                  ((x & CFGREG_IF1_CONF_PREAMP_EN_MASK) >> CFGREG_IF1_CONF_PREAMP_EN_SHIFT)                                                /*!<  IF1 Configuration Register Bits Get */
#define CFGREG_IF1_CONF_EN_BITS_GET(x)                                         ((x & CFGREG_IF1_CONF_EN_MASK) >> CFGREG_IF1_CONF_EN_SHIFT)                                                              /*!<  IF1 Configuration Register Bits Get */

#define CFGREG_IF25_CONF_IF_BANDCUT_BITS_GET(x)                                ((x & CFGREG_IF25_CONF_IF_BANDCUT_MASK) >> CFGREG_IF25_CONF_IF_BANDCUT_SHIFT)                                            /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_IF25_CONF_OFFSET_CAL_Q_BITS_GET(x)                              ((x & CFGREG_IF25_CONF_OFFSET_CAL_Q_MASK) >> CFGREG_IF25_CONF_OFFSET_CAL_Q_SHIFT)                                        /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_IF25_CONF_OFFSET_CAL_DIRECTION_Q_BITS_GET(x)                    ((x & CFGREG_IF25_CONF_OFFSET_CAL_DIRECTION_Q_MASK) >> CFGREG_IF25_CONF_OFFSET_CAL_DIRECTION_Q_SHIFT)                    /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_IF25_CONF_OFFSET_CAL_I_BITS_GET(x)                              ((x & CFGREG_IF25_CONF_OFFSET_CAL_I_MASK) >> CFGREG_IF25_CONF_OFFSET_CAL_I_SHIFT)                                        /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_IF25_CONF_OFFSET_CAL_DIRECTION_I_BITS_GET(x)                    ((x & CFGREG_IF25_CONF_OFFSET_CAL_DIRECTION_I_MASK) >> CFGREG_IF25_CONF_OFFSET_CAL_DIRECTION_I_SHIFT)                    /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_IF25_CONF_PGA2_EN_BITS_GET(x)                                   ((x & CFGREG_IF25_CONF_PGA2_EN_MASK) >> CFGREG_IF25_CONF_PGA2_EN_SHIFT)                                                  /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_IF25_CONF_PGA1_EN_BITS_GET(x)                                   ((x & CFGREG_IF25_CONF_PGA1_EN_MASK) >> CFGREG_IF25_CONF_PGA1_EN_SHIFT)                                                  /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_IF25_CONF_PREAMP_EN_BITS_GET(x)                                 ((x & CFGREG_IF25_CONF_PREAMP_EN_MASK) >> CFGREG_IF25_CONF_PREAMP_EN_SHIFT)                                              /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_IF25_CONF_EN_BITS_GET(x)                                        ((x & CFGREG_IF25_CONF_EN_MASK) >> CFGREG_IF25_CONF_EN_SHIFT)                                                            /*!<  IF25 Configuration Register Bits Get */

#define CFGREG_PROBE_CONF_PLL15FB_SEL_BITS_GET(x)                              ((x & CFGREG_PROBE_CONF_PLL15FB_SEL_MASK) >> CFGREG_PROBE_CONF_PLL15FB_SEL_SHIFT)                                        /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_PROBE_CONF_PLL15FB_EN_BITS_GET(x)                               ((x & CFGREG_PROBE_CONF_PLL15FB_EN_MASK) >> CFGREG_PROBE_CONF_PLL15FB_EN_SHIFT)                                          /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_PROBE_CONF_IF25_PROBEB_SELECT_BITS_GET(x)                       ((x & CFGREG_PROBE_CONF_IF25_PROBEB_SELECT_MASK) >> CFGREG_PROBE_CONF_IF25_PROBEB_SELECT_SHIFT)                          /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_PROBE_CONF_IF25_PROBEB_EN_BITS_GET(x)                           ((x & CFGREG_PROBE_CONF_IF25_PROBEB_EN_MASK) >> CFGREG_PROBE_CONF_IF25_PROBEB_EN_SHIFT)                                  /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_PROBE_CONF_IF25_PROBEA_SELECT_BITS_GET(x)                       ((x & CFGREG_PROBE_CONF_IF25_PROBEA_SELECT_MASK) >> CFGREG_PROBE_CONF_IF25_PROBEA_SELECT_SHIFT)                          /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_PROBE_CONF_IF1_PROBEB_SELECT_BITS_GET(x)                        ((x & CFGREG_PROBE_CONF_IF1_PROBEB_SELECT_MASK) >> CFGREG_PROBE_CONF_IF1_PROBEB_SELECT_SHIFT)                            /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_PROBE_CONF_IF1_PROBEB_EN_BITS_GET(x)                            ((x & CFGREG_PROBE_CONF_IF1_PROBEB_EN_MASK) >> CFGREG_PROBE_CONF_IF1_PROBEB_EN_SHIFT)                                    /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_PROBE_CONF_IF1_PROBEA_SELECT_BITS_GET(x)                        ((x & CFGREG_PROBE_CONF_IF1_PROBEA_SELECT_MASK) >> CFGREG_PROBE_CONF_IF1_PROBEA_SELECT_SHIFT)                            /*!<  IF25 Configuration Register Bits Get */

#define CFGREG_ADC1_CONF_CLK_SEL_BITS_GET(x)                                   ((x & CFGREG_ADC1_CONF_CLK_SEL_MASK) >> CFGREG_ADC1_CONF_CLK_SEL_SHIFT)                                                  /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_ADC1_CONF_CLK_CONF_BITS_GET(x)                                  ((x & CFGREG_ADC1_CONF_CLK_CONF_MASK) >> CFGREG_ADC1_CONF_CLK_CONF_SHIFT)                                                /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_ADC1_CONF_SAH_IBIAS_CTRL_BITS_GET(x)                            ((x & CFGREG_ADC1_CONF_SAH_IBIAS_CTRL_MASK) >> CFGREG_ADC1_CONF_SAH_IBIAS_CTRL_SHIFT)                                    /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_ADC1_CONF_CAL_EN_BITS_GET(x)                                    ((x & CFGREG_ADC1_CONF_CAL_EN_MASK) >> CFGREG_ADC1_CONF_CAL_EN_SHIFT)                                                    /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_ADC1_CONF_SAH_EN_BITS_GET(x)                                    ((x & CFGREG_ADC1_CONF_SAH_EN_MASK) >> CFGREG_ADC1_CONF_SAH_EN_SHIFT)                                                    /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_ADC1_CONF_ADC_EN_BITS_GET(x)                                    ((x & CFGREG_ADC1_CONF_ADC_EN_MASK) >> CFGREG_ADC1_CONF_ADC_EN_SHIFT)                                                    /*!<  IF25 Configuration Register Bits Get */

#define CFGREG_ADC25_CONF_CLK_SEL_BITS_GET(x)                                  ((x & CFGREG_ADC25_CONF_CLK_SEL_MASK) >> CFGREG_ADC25_CONF_CLK_SEL_SHIFT)                                                /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_ADC25_CONF_CLK_CONF_BITS_GET(x)                                 ((x & CFGREG_ADC25_CONF_CLK_CONF_MASK) >> CFGREG_ADC25_CONF_CLK_CONF_SHIFT)                                              /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_ADC25_CONF_SAH_IBIAS_CTRL_BITS_GET(x)                           ((x & CFGREG_ADC25_CONF_SAH_IBIAS_CTRL_MASK) >> CFGREG_ADC25_CONF_SAH_IBIAS_CTRL_SHIFT)                                  /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_ADC25_CONF_CAL_EN_BITS_GET(x)                                   ((x & CFGREG_ADC25_CONF_CAL_EN_MASK) >> CFGREG_ADC25_CONF_CAL_EN_SHIFT)                                                  /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_ADC25_CONF_SAH_EN_BITS_GET(x)                                   ((x & CFGREG_ADC25_CONF_SAH_EN_MASK) >> CFGREG_ADC25_CONF_SAH_EN_SHIFT)                                                  /*!<  IF25 Configuration Register Bits Get */
#define CFGREG_ADC25_CONF_ADC_EN_BITS_GET(x)                                   ((x & CFGREG_ADC25_CONF_ADC_EN_MASK) >> CFGREG_ADC25_CONF_ADC_EN_SHIFT)                                                  /*!<  IF25 Configuration Register Bits Get */

#define CFGREG_SPLIT1_CTRL_CLEAR_BITS_GET(x)                                   ((x & CFGREG_SPLIT1_CTRL_CLEAR_MASK) >> CFGREG_SPLIT1_CTRL_CLEAR_SHIFT)                                                  /*!<  Splitter AFE1 Control Register Bits Get */

#define CFGREG_SPLIT1_CFG_GAIN_CHANGE_CNT_BITS_GET(x)                          ((x & CFGREG_SPLIT1_CFG_GAIN_CHANGE_CNT_MASK) >> CFGREG_SPLIT1_CFG_GAIN_CHANGE_CNT_SHIFT)                                /*!<  Splitter AFE1 Gain Change Configuration Register Bits Get */

#define CFGREG_SPLIT1_CFG_LIMITER_THRESHOLD_BITS_GET(x)                        ((x & CFGREG_SPLIT1_CFG_LIMITER_THRESHOLD_MASK) >> CFGREG_SPLIT1_CFG_LIMITER_THRESHOLD_SHIFT)                            /*!<  Splitter AFE1 Limiter Threshold Configuration Register Bits Get */

#define CFGREG_SPLIT1_CFG_MANUAL_MODE_PGA2_GAIN_BITS_GET(x)                    ((x & CFGREG_SPLIT1_CFG_MANUAL_MODE_PGA2_GAIN_MASK) >> CFGREG_SPLIT1_CFG_MANUAL_MODE_PGA2_GAIN_SHIFT)                    /*!<  Splitter AFE1 Manual Mode Configuration Register Bits Get */
#define CFGREG_SPLIT1_CFG_MANUAL_MODE_PGA1_GAIN_BITS_GET(x)                    ((x & CFGREG_SPLIT1_CFG_MANUAL_MODE_PGA1_GAIN_MASK) >> CFGREG_SPLIT1_CFG_MANUAL_MODE_PGA1_GAIN_SHIFT)                    /*!<  Splitter AFE1 Manual Mode Configuration Register Bits Get */
#define CFGREG_SPLIT1_CFG_MANUAL_MODE_EN_BITS_GET(x)                           ((x & CFGREG_SPLIT1_CFG_MANUAL_MODE_EN_MASK) >> CFGREG_SPLIT1_CFG_MANUAL_MODE_EN_SHIFT)                                  /*!<  Splitter AFE1 Manual Mode Configuration Register Bits Get */

#define CFGREG_SPLIT1_L1E1_CFG_DEC_BY_THREE_EN_BITS_GET(x)                     ((x & CFGREG_SPLIT1_L1E1_CFG_DEC_BY_THREE_EN_MASK) >> CFGREG_SPLIT1_L1E1_CFG_DEC_BY_THREE_EN_SHIFT)                      /*!<  Splitter AFE1 L1E1 Configuration Register Bits Get */
#define CFGREG_SPLIT1_L1E1_CFG_DEC_SAMPLES_SHIFT_BITS_GET(x)                   ((x & CFGREG_SPLIT1_L1E1_CFG_DEC_SAMPLES_SHIFT_MASK) >> CFGREG_SPLIT1_L1E1_CFG_DEC_SAMPLES_SHIFT_SHIFT)                  /*!<  Splitter AFE1 L1E1 Configuration Register Bits Get */
#define CFGREG_SPLIT1_L1E1_CFG_CARR_MODE_BITS_GET(x)                           ((x & CFGREG_SPLIT1_L1E1_CFG_CARR_MODE_MASK) >> CFGREG_SPLIT1_L1E1_CFG_CARR_MODE_SHIFT)                                  /*!<  Splitter AFE1 L1E1 Configuration Register Bits Get */
#define CFGREG_SPLIT1_L1E1_CFG_CARR_ENABLE_BITS_GET(x)                         ((x & CFGREG_SPLIT1_L1E1_CFG_CARR_ENABLE_MASK) >> CFGREG_SPLIT1_L1E1_CFG_CARR_ENABLE_SHIFT)                              /*!<  Splitter AFE1 L1E1 Configuration Register Bits Get */
#define CFGREG_SPLIT1_L1E1_CFG_LPF_K_PARAM_BITS_GET(x)                         ((x & CFGREG_SPLIT1_L1E1_CFG_LPF_K_PARAM_MASK) >> CFGREG_SPLIT1_L1E1_CFG_LPF_K_PARAM_SHIFT)                              /*!<  Splitter AFE1 L1E1 Configuration Register Bits Get */
#define CFGREG_SPLIT1_L1E1_CFG_IQ_SAME_GAIN_BITS_GET(x)                        ((x & CFGREG_SPLIT1_L1E1_CFG_IQ_SAME_GAIN_MASK) >> CFGREG_SPLIT1_L1E1_CFG_IQ_SAME_GAIN_SHIFT)                            /*!<  Splitter AFE1 L1E1 Configuration Register Bits Get */

#define CFGREG_SPLIT1_L1E1_CFG_CARR_FREQ_STEP_BITS_GET(x)                      ((x & CFGREG_SPLIT1_L1E1_CFG_CARR_FREQ_STEP_MASK) >> CFGREG_SPLIT1_L1E1_CFG_CARR_FREQ_STEP_SHIFT)                        /*!<  Splitter AFE1 L1E1 Carrier Frequency Configuration Register Bits Get */

#define CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_MAX_BITS_GET(x)                     ((x & CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_MAX_MASK) >> CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_MAX_SHIFT)                      /*!<  Splitter AFE1 L1E1 Threshold I Configuration Register Bits Get */
#define CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_MIN_BITS_GET(x)                     ((x & CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_MIN_MASK) >> CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_MIN_SHIFT)                      /*!<  Splitter AFE1 L1E1 Threshold I Configuration Register Bits Get */

#define CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_MAX_BITS_GET(x)                     ((x & CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_MAX_MASK) >> CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_MAX_SHIFT)                      /*!<  Splitter AFE1 L1E1 Threshold Q Configuration Register Bits Get */
#define CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_MIN_BITS_GET(x)                     ((x & CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_MIN_MASK) >> CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_MIN_SHIFT)                      /*!<  Splitter AFE1 L1E1 Threshold Q Configuration Register Bits Get */

#define CFGREG_SPLIT1_L1E1_CFG_MANUAL_MODE_DIG_GAIN_Q_BITS_GET(x)              ((x & CFGREG_SPLIT1_L1E1_CFG_MANUAL_MODE_DIG_GAIN_Q_MASK) >> CFGREG_SPLIT1_L1E1_CFG_MANUAL_MODE_DIG_GAIN_Q_SHIFT)        /*!<  Splitter AFE1 L1E1 Manual Mode Digital Gain Configuration Register Bits Get */
#define CFGREG_SPLIT1_L1E1_CFG_MANUAL_MODE_DIG_GAIN_I_BITS_GET(x)              ((x & CFGREG_SPLIT1_L1E1_CFG_MANUAL_MODE_DIG_GAIN_I_MASK) >> CFGREG_SPLIT1_L1E1_CFG_MANUAL_MODE_DIG_GAIN_I_SHIFT)        /*!<  Splitter AFE1 L1E1 Manual Mode Digital Gain Configuration Register Bits Get */

#define CFGREG_SPLIT25_CTRL_CLEAR_BITS_GET(x)                                  ((x & CFGREG_SPLIT25_CTRL_CLEAR_MASK) >> CFGREG_SPLIT25_CTRL_CLEAR_SHIFT)                                                /*!<  Splitter AFE25 Control Register Bits Get */

#define CFGREG_SPLIT25_CFG_GAIN_CHANGE_CNT_BITS_GET(x)                         ((x & CFGREG_SPLIT25_CFG_GAIN_CHANGE_CNT_MASK) >> CFGREG_SPLIT25_CFG_GAIN_CHANGE_CNT_SHIFT)                              /*!<  Splitter AFE25 Gain Change Configuration Register Bits Get */

#define CFGREG_SPLIT25_CFG_LIMITER_THRESHOLD_BITS_GET(x)                       ((x & CFGREG_SPLIT25_CFG_LIMITER_THRESHOLD_MASK) >> CFGREG_SPLIT25_CFG_LIMITER_THRESHOLD_SHIFT)                          /*!<  Splitter AFE25 Limiter Threshold Configuration Register Bits Get */

#define CFGREG_SPLIT25_CFG_MANUAL_MODE_PGA2_GAIN_BITS_GET(x)                   ((x & CFGREG_SPLIT25_CFG_MANUAL_MODE_PGA2_GAIN_MASK) >> CFGREG_SPLIT25_CFG_MANUAL_MODE_PGA2_GAIN_SHIFT)                  /*!<  Splitter AFE25 Manual Mode Configuration Register Bits Get */
#define CFGREG_SPLIT25_CFG_MANUAL_MODE_PGA1_GAIN_BITS_GET(x)                   ((x & CFGREG_SPLIT25_CFG_MANUAL_MODE_PGA1_GAIN_MASK) >> CFGREG_SPLIT25_CFG_MANUAL_MODE_PGA1_GAIN_SHIFT)                  /*!<  Splitter AFE25 Manual Mode Configuration Register Bits Get */
#define CFGREG_SPLIT25_CFG_MANUAL_MODE_EN_BITS_GET(x)                          ((x & CFGREG_SPLIT25_CFG_MANUAL_MODE_EN_MASK) >> CFGREG_SPLIT25_CFG_MANUAL_MODE_EN_SHIFT)                                /*!<  Splitter AFE25 Manual Mode Configuration Register Bits Get */

#define CFGREG_SPLIT25_L2_CFG_DEC_BY_THREE_EN_BITS_GET(x)                      ((x & CFGREG_SPLIT25_L2_CFG_DEC_BY_THREE_EN_MASK) >> CFGREG_SPLIT25_L2_CFG_DEC_BY_THREE_EN_SHIFT)                        /*!<  Splitter AFE25 L2 Configuration Register Bits Get */
#define CFGREG_SPLIT25_L2_CFG_DEC_SAMPLES_SHIFT_BITS_GET(x)                    ((x & CFGREG_SPLIT25_L2_CFG_DEC_SAMPLES_SHIFT_MASK) >> CFGREG_SPLIT25_L2_CFG_DEC_SAMPLES_SHIFT_SHIFT)                    /*!<  Splitter AFE25 L2 Configuration Register Bits Get */
#define CFGREG_SPLIT25_L2_CFG_CARR_MODE_BITS_GET(x)                            ((x & CFGREG_SPLIT25_L2_CFG_CARR_MODE_MASK) >> CFGREG_SPLIT25_L2_CFG_CARR_MODE_SHIFT)                                    /*!<  Splitter AFE25 L2 Configuration Register Bits Get */
#define CFGREG_SPLIT25_L2_CFG_CARR_ENABLE_BITS_GET(x)                          ((x & CFGREG_SPLIT25_L2_CFG_CARR_ENABLE_MASK) >> CFGREG_SPLIT25_L2_CFG_CARR_ENABLE_SHIFT)                                /*!<  Splitter AFE25 L2 Configuration Register Bits Get */
#define CFGREG_SPLIT25_L2_CFG_LPF_K_PARAM_BITS_GET(x)                          ((x & CFGREG_SPLIT25_L2_CFG_LPF_K_PARAM_MASK) >> CFGREG_SPLIT25_L2_CFG_LPF_K_PARAM_SHIFT)                                /*!<  Splitter AFE25 L2 Configuration Register Bits Get */
#define CFGREG_SPLIT25_L2_CFG_IQ_SAME_GAIN_BITS_GET(x)                         ((x & CFGREG_SPLIT25_L2_CFG_IQ_SAME_GAIN_MASK) >> CFGREG_SPLIT25_L2_CFG_IQ_SAME_GAIN_SHIFT)                              /*!<  Splitter AFE25 L2 Configuration Register Bits Get */

#define CFGREG_SPLIT25_L2_CFG_CARR_FREQ_STEP_BITS_GET(x)                       ((x & CFGREG_SPLIT25_L2_CFG_CARR_FREQ_STEP_MASK) >> CFGREG_SPLIT25_L2_CFG_CARR_FREQ_STEP_SHIFT)                          /*!<  Splitter AFE25 L2 Carrier Frequency Configuration Register Bits Get */

#define CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_MAX_BITS_GET(x)                      ((x & CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_MAX_MASK) >> CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_MAX_SHIFT)                        /*!<  Splitter AFE25 L2 Threshold I Configuration Register Bits Get */
#define CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_MIN_BITS_GET(x)                      ((x & CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_MIN_MASK) >> CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_MIN_SHIFT)                        /*!<  Splitter AFE25 L2 Threshold I Configuration Register Bits Get */

#define CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_MAX_BITS_GET(x)                      ((x & CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_MAX_MASK) >> CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_MAX_SHIFT)                        /*!<  Splitter AFE25 L2 Threshold Q Configuration Register Bits Get */
#define CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_MIN_BITS_GET(x)                      ((x & CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_MIN_MASK) >> CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_MIN_SHIFT)                        /*!<  Splitter AFE25 L2 Threshold Q Configuration Register Bits Get */

#define CFGREG_SPLIT25_L2_CFG_MANUAL_MODE_DIG_GAIN_Q_BITS_GET(x)               ((x & CFGREG_SPLIT25_L2_CFG_MANUAL_MODE_DIG_GAIN_Q_MASK) >> CFGREG_SPLIT25_L2_CFG_MANUAL_MODE_DIG_GAIN_Q_SHIFT)          /*!<  Splitter AFE25 L2 Manual Mode Digital Gain Configuration Register Bits Get */
#define CFGREG_SPLIT25_L2_CFG_MANUAL_MODE_DIG_GAIN_I_BITS_GET(x)               ((x & CFGREG_SPLIT25_L2_CFG_MANUAL_MODE_DIG_GAIN_I_MASK) >> CFGREG_SPLIT25_L2_CFG_MANUAL_MODE_DIG_GAIN_I_SHIFT)          /*!<  Splitter AFE25 L2 Manual Mode Digital Gain Configuration Register Bits Get */

#define CFGREG_SPLIT25_E6_CFG_DEC_BY_THREE_EN_BITS_GET(x)                      ((x & CFGREG_SPLIT25_E6_CFG_DEC_BY_THREE_EN_MASK) >> CFGREG_SPLIT25_E6_CFG_DEC_BY_THREE_EN_SHIFT)                        /*!<  Splitter AFE25 E6 Configuration Register Bits Get */
#define CFGREG_SPLIT25_E6_CFG_DEC_SAMPLES_SHIFT_BITS_GET(x)                    ((x & CFGREG_SPLIT25_E6_CFG_DEC_SAMPLES_SHIFT_MASK) >> CFGREG_SPLIT25_E6_CFG_DEC_SAMPLES_SHIFT_SHIFT)                    /*!<  Splitter AFE25 E6 Configuration Register Bits Get */
#define CFGREG_SPLIT25_E6_CFG_CARR_MODE_BITS_GET(x)                            ((x & CFGREG_SPLIT25_E6_CFG_CARR_MODE_MASK) >> CFGREG_SPLIT25_E6_CFG_CARR_MODE_SHIFT)                                    /*!<  Splitter AFE25 E6 Configuration Register Bits Get */
#define CFGREG_SPLIT25_E6_CFG_CARR_ENABLE_BITS_GET(x)                          ((x & CFGREG_SPLIT25_E6_CFG_CARR_ENABLE_MASK) >> CFGREG_SPLIT25_E6_CFG_CARR_ENABLE_SHIFT)                                /*!<  Splitter AFE25 E6 Configuration Register Bits Get */
#define CFGREG_SPLIT25_E6_CFG_LPF_K_PARAM_BITS_GET(x)                          ((x & CFGREG_SPLIT25_E6_CFG_LPF_K_PARAM_MASK) >> CFGREG_SPLIT25_E6_CFG_LPF_K_PARAM_SHIFT)                                /*!<  Splitter AFE25 E6 Configuration Register Bits Get */
#define CFGREG_SPLIT25_E6_CFG_IQ_SAME_GAIN_BITS_GET(x)                         ((x & CFGREG_SPLIT25_E6_CFG_IQ_SAME_GAIN_MASK) >> CFGREG_SPLIT25_E6_CFG_IQ_SAME_GAIN_SHIFT)                              /*!<  Splitter AFE25 E6 Configuration Register Bits Get */

#define CFGREG_SPLIT25_E6_CFG_CARR_FREQ_STEP_BITS_GET(x)                       ((x & CFGREG_SPLIT25_E6_CFG_CARR_FREQ_STEP_MASK) >> CFGREG_SPLIT25_E6_CFG_CARR_FREQ_STEP_SHIFT)                          /*!<  Splitter AFE25 E6 Carrier Frequency Configuration Register Bits Get */

#define CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_MAX_BITS_GET(x)                      ((x & CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_MAX_MASK) >> CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_MAX_SHIFT)                        /*!<  Splitter AFE25 E6 Threshold I Configuration Register Bits Get */
#define CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_MIN_BITS_GET(x)                      ((x & CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_MIN_MASK) >> CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_MIN_SHIFT)                        /*!<  Splitter AFE25 E6 Threshold I Configuration Register Bits Get */

#define CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_MAX_BITS_GET(x)                      ((x & CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_MAX_MASK) >> CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_MAX_SHIFT)                        /*!<  Splitter AFE25 E6 Threshold Q Configuration Register Bits Get */
#define CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_MIN_BITS_GET(x)                      ((x & CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_MIN_MASK) >> CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_MIN_SHIFT)                        /*!<  Splitter AFE25 E6 Threshold Q Configuration Register Bits Get */

#define CFGREG_SPLIT25_E6_CFG_MANUAL_MODE_DIG_GAIN_Q_BITS_GET(x)               ((x & CFGREG_SPLIT25_E6_CFG_MANUAL_MODE_DIG_GAIN_Q_MASK) >> CFGREG_SPLIT25_E6_CFG_MANUAL_MODE_DIG_GAIN_Q_SHIFT)          /*!<  Splitter AFE25 E6 Manual Mode Digital Gain Configuration Register Bits Get */
#define CFGREG_SPLIT25_E6_CFG_MANUAL_MODE_DIG_GAIN_I_BITS_GET(x)               ((x & CFGREG_SPLIT25_E6_CFG_MANUAL_MODE_DIG_GAIN_I_MASK) >> CFGREG_SPLIT25_E6_CFG_MANUAL_MODE_DIG_GAIN_I_SHIFT)          /*!<  Splitter AFE25 E6 Manual Mode Digital Gain Configuration Register Bits Get */

#define CFGREG_SPLIT25_L5E5A_CFG_DEC_BY_THREE_EN_BITS_GET(x)                   ((x & CFGREG_SPLIT25_L5E5A_CFG_DEC_BY_THREE_EN_MASK) >> CFGREG_SPLIT25_L5E5A_CFG_DEC_BY_THREE_EN_SHIFT)                  /*!<  Splitter AFE25 L5E5A Configuration Register Bits Get */
#define CFGREG_SPLIT25_L5E5A_CFG_DEC_SAMPLES_SHIFT_BITS_GET(x)                 ((x & CFGREG_SPLIT25_L5E5A_CFG_DEC_SAMPLES_SHIFT_MASK) >> CFGREG_SPLIT25_L5E5A_CFG_DEC_SAMPLES_SHIFT_SHIFT)              /*!<  Splitter AFE25 L5E5A Configuration Register Bits Get */
#define CFGREG_SPLIT25_L5E5A_CFG_CARR_MODE_BITS_GET(x)                         ((x & CFGREG_SPLIT25_L5E5A_CFG_CARR_MODE_MASK) >> CFGREG_SPLIT25_L5E5A_CFG_CARR_MODE_SHIFT)                              /*!<  Splitter AFE25 L5E5A Configuration Register Bits Get */
#define CFGREG_SPLIT25_L5E5A_CFG_CARR_ENABLE_BITS_GET(x)                       ((x & CFGREG_SPLIT25_L5E5A_CFG_CARR_ENABLE_MASK) >> CFGREG_SPLIT25_L5E5A_CFG_CARR_ENABLE_SHIFT)                          /*!<  Splitter AFE25 L5E5A Configuration Register Bits Get */
#define CFGREG_SPLIT25_L5E5A_CFG_LPF_K_PARAM_BITS_GET(x)                       ((x & CFGREG_SPLIT25_L5E5A_CFG_LPF_K_PARAM_MASK) >> CFGREG_SPLIT25_L5E5A_CFG_LPF_K_PARAM_SHIFT)                          /*!<  Splitter AFE25 L5E5A Configuration Register Bits Get */
#define CFGREG_SPLIT25_L5E5A_CFG_IQ_SAME_GAIN_BITS_GET(x)                      ((x & CFGREG_SPLIT25_L5E5A_CFG_IQ_SAME_GAIN_MASK) >> CFGREG_SPLIT25_L5E5A_CFG_IQ_SAME_GAIN_SHIFT)                        /*!<  Splitter AFE25 L5E5A Configuration Register Bits Get */

#define CFGREG_SPLIT25_L5E5A_CFG_CARR_FREQ_STEP_BITS_GET(x)                    ((x & CFGREG_SPLIT25_L5E5A_CFG_CARR_FREQ_STEP_MASK) >> CFGREG_SPLIT25_L5E5A_CFG_CARR_FREQ_STEP_SHIFT)                    /*!<  Splitter AFE25 L5E5A Carrier Frequency Configuration Register Bits Get */

#define CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_MAX_BITS_GET(x)                   ((x & CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_MAX_MASK) >> CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_MAX_SHIFT)                  /*!<  Splitter AFE25 L5E5A Threshold I Configuration Register Bits Get */
#define CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_MIN_BITS_GET(x)                   ((x & CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_MIN_MASK) >> CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_MIN_SHIFT)                  /*!<  Splitter AFE25 L5E5A Threshold I Configuration Register Bits Get */

#define CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_MAX_BITS_GET(x)                   ((x & CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_MAX_MASK) >> CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_MAX_SHIFT)                  /*!<  Splitter AFE25 L5E5A Threshold Q Configuration Register Bits Get */
#define CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_MIN_BITS_GET(x)                   ((x & CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_MIN_MASK) >> CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_MIN_SHIFT)                  /*!<  Splitter AFE25 L5E5A Threshold Q Configuration Register Bits Get */

#define CFGREG_SPLIT25_L5E5A_CFG_MANUAL_MODE_DIG_GAIN_Q_BITS_GET(x)            ((x & CFGREG_SPLIT25_L5E5A_CFG_MANUAL_MODE_DIG_GAIN_Q_MASK) >> CFGREG_SPLIT25_L5E5A_CFG_MANUAL_MODE_DIG_GAIN_Q_SHIFT)    /*!<  Splitter AFE25 L5E5A Manual Mode Digital Gain Configuration Register Bits Get */
#define CFGREG_SPLIT25_L5E5A_CFG_MANUAL_MODE_DIG_GAIN_I_BITS_GET(x)            ((x & CFGREG_SPLIT25_L5E5A_CFG_MANUAL_MODE_DIG_GAIN_I_MASK) >> CFGREG_SPLIT25_L5E5A_CFG_MANUAL_MODE_DIG_GAIN_I_SHIFT)    /*!<  Splitter AFE25 L5E5A Manual Mode Digital Gain Configuration Register Bits Get */

#define CFGREG_SPLIT25_E5B_CFG_DEC_BY_THREE_EN_BITS_GET(x)                     ((x & CFGREG_SPLIT25_E5B_CFG_DEC_BY_THREE_EN_MASK) >> CFGREG_SPLIT25_E5B_CFG_DEC_BY_THREE_EN_SHIFT)                      /*!<  Splitter AFE25 E5B Configuration Register Bits Get */
#define CFGREG_SPLIT25_E5B_CFG_DEC_SAMPLES_SHIFT_BITS_GET(x)                   ((x & CFGREG_SPLIT25_E5B_CFG_DEC_SAMPLES_SHIFT_MASK) >> CFGREG_SPLIT25_E5B_CFG_DEC_SAMPLES_SHIFT_SHIFT)                  /*!<  Splitter AFE25 E5B Configuration Register Bits Get */
#define CFGREG_SPLIT25_E5B_CFG_CARR_MODE_BITS_GET(x)                           ((x & CFGREG_SPLIT25_E5B_CFG_CARR_MODE_MASK) >> CFGREG_SPLIT25_E5B_CFG_CARR_MODE_SHIFT)                                  /*!<  Splitter AFE25 E5B Configuration Register Bits Get */
#define CFGREG_SPLIT25_E5B_CFG_CARR_ENABLE_BITS_GET(x)                         ((x & CFGREG_SPLIT25_E5B_CFG_CARR_ENABLE_MASK) >> CFGREG_SPLIT25_E5B_CFG_CARR_ENABLE_SHIFT)                              /*!<  Splitter AFE25 E5B Configuration Register Bits Get */
#define CFGREG_SPLIT25_E5B_CFG_LPF_K_PARAM_BITS_GET(x)                         ((x & CFGREG_SPLIT25_E5B_CFG_LPF_K_PARAM_MASK) >> CFGREG_SPLIT25_E5B_CFG_LPF_K_PARAM_SHIFT)                              /*!<  Splitter AFE25 E5B Configuration Register Bits Get */
#define CFGREG_SPLIT25_E5B_CFG_IQ_SAME_GAIN_BITS_GET(x)                        ((x & CFGREG_SPLIT25_E5B_CFG_IQ_SAME_GAIN_MASK) >> CFGREG_SPLIT25_E5B_CFG_IQ_SAME_GAIN_SHIFT)                            /*!<  Splitter AFE25 E5B Configuration Register Bits Get */

#define CFGREG_SPLIT25_E5B_CFG_CARR_FREQ_STEP_BITS_GET(x)                      ((x & CFGREG_SPLIT25_E5B_CFG_CARR_FREQ_STEP_MASK) >> CFGREG_SPLIT25_E5B_CFG_CARR_FREQ_STEP_SHIFT)                        /*!<  Splitter AFE25 E5B Carrier Frequency Configuration Register Bits Get */

#define CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_MAX_BITS_GET(x)                     ((x & CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_MAX_MASK) >> CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_MAX_SHIFT)                      /*!<  Splitter AFE25 E5B Threshold I Configuration Register Bits Get */
#define CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_MIN_BITS_GET(x)                     ((x & CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_MIN_MASK) >> CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_MIN_SHIFT)                      /*!<  Splitter AFE25 E5B Threshold I Configuration Register Bits Get */

#define CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_MAX_BITS_GET(x)                     ((x & CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_MAX_MASK) >> CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_MAX_SHIFT)                      /*!<  Splitter AFE25 E5B Threshold Q Configuration Register Bits Get */
#define CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_MIN_BITS_GET(x)                     ((x & CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_MIN_MASK) >> CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_MIN_SHIFT)                      /*!<  Splitter AFE25 E5B Threshold Q Configuration Register Bits Get */

#define CFGREG_SPLIT25_E5B_CFG_MANUAL_MODE_DIG_GAIN_Q_BITS_GET(x)              ((x & CFGREG_SPLIT25_E5B_CFG_MANUAL_MODE_DIG_GAIN_Q_MASK) >> CFGREG_SPLIT25_E5B_CFG_MANUAL_MODE_DIG_GAIN_Q_SHIFT)        /*!<  Splitter AFE25 E5B Manual Mode Digital Gain Configuration Register Bits Get */
#define CFGREG_SPLIT25_E5B_CFG_MANUAL_MODE_DIG_GAIN_I_BITS_GET(x)              ((x & CFGREG_SPLIT25_E5B_CFG_MANUAL_MODE_DIG_GAIN_I_MASK) >> CFGREG_SPLIT25_E5B_CFG_MANUAL_MODE_DIG_GAIN_I_SHIFT)        /*!<  Splitter AFE25 E5B Manual Mode Digital Gain Configuration Register Bits Get */

#define CFGREG_LNA125_CONF_L25_TUNE_SRC_BITS_GET(x)                            ((x & CFGREG_LNA125_CONF_L25_TUNE_SRC_MASK) >> CFGREG_LNA125_CONF_L25_TUNE_SRC_SHIFT)                                    /*!<  LNA125 Configuration Register Bits Get */
#define CFGREG_LNA125_CONF_L1_TUNE_SRC_BITS_GET(x)                             ((x & CFGREG_LNA125_CONF_L1_TUNE_SRC_MASK) >> CFGREG_LNA125_CONF_L1_TUNE_SRC_SHIFT)                                      /*!<  LNA125 Configuration Register Bits Get */
#define CFGREG_LNA125_CONF_EN_L25_BITS_GET(x)                                  ((x & CFGREG_LNA125_CONF_EN_L25_MASK) >> CFGREG_LNA125_CONF_EN_L25_SHIFT)                                                /*!<  LNA125 Configuration Register Bits Get */
#define CFGREG_LNA125_CONF_EN_L1_BITS_GET(x)                                   ((x & CFGREG_LNA125_CONF_EN_L1_MASK) >> CFGREG_LNA125_CONF_EN_L1_SHIFT)                                                  /*!<  LNA125 Configuration Register Bits Get */

#define CFGREG_LNA125_TUNE_CONF_L25_BITS_GET(x)                                ((x & CFGREG_LNA125_TUNE_CONF_L25_MASK) >> CFGREG_LNA125_TUNE_CONF_L25_SHIFT)                                            /*!<  LNA125 Configuration Register Bits Get */
#define CFGREG_LNA125_TUNE_CONF_L1_BITS_GET(x)                                 ((x & CFGREG_LNA125_TUNE_CONF_L1_MASK) >> CFGREG_LNA125_TUNE_CONF_L1_SHIFT)                                              /*!<  LNA125 Configuration Register Bits Get */

#define CFGREG_BALUN_MIXER1_CONF_BALUN_TUNE_SRC_BITS_GET(x)                    ((x & CFGREG_BALUN_MIXER1_CONF_BALUN_TUNE_SRC_MASK) >> CFGREG_BALUN_MIXER1_CONF_BALUN_TUNE_SRC_SHIFT)                    /*!<  BALUN and MIXER 1 Configuration Register Bits Get */
#define CFGREG_BALUN_MIXER1_CONF_BALUN_TUNE_BITS_GET(x)                        ((x & CFGREG_BALUN_MIXER1_CONF_BALUN_TUNE_MASK) >> CFGREG_BALUN_MIXER1_CONF_BALUN_TUNE_SHIFT)                            /*!<  BALUN and MIXER 1 Configuration Register Bits Get */
#define CFGREG_BALUN_MIXER1_CONF_BALUN_EN_BITS_GET(x)                          ((x & CFGREG_BALUN_MIXER1_CONF_BALUN_EN_MASK) >> CFGREG_BALUN_MIXER1_CONF_BALUN_EN_SHIFT)                                /*!<  BALUN and MIXER 1 Configuration Register Bits Get */
#define CFGREG_BALUN_MIXER1_CONF_MIXER_EN_BITS_GET(x)                          ((x & CFGREG_BALUN_MIXER1_CONF_MIXER_EN_MASK) >> CFGREG_BALUN_MIXER1_CONF_MIXER_EN_SHIFT)                                /*!<  BALUN and MIXER 1 Configuration Register Bits Get */

#define CFGREG_BALUN_MIXER25_CONF_BALUN_TUNE_SRC_BITS_GET(x)                   ((x & CFGREG_BALUN_MIXER25_CONF_BALUN_TUNE_SRC_MASK) >> CFGREG_BALUN_MIXER25_CONF_BALUN_TUNE_SRC_SHIFT)                  /*!<  BALUN and MIXER 25 Configuration Register Bits Get */
#define CFGREG_BALUN_MIXER25_CONF_BALUN_TUNE_BITS_GET(x)                       ((x & CFGREG_BALUN_MIXER25_CONF_BALUN_TUNE_MASK) >> CFGREG_BALUN_MIXER25_CONF_BALUN_TUNE_SHIFT)                          /*!<  BALUN and MIXER 25 Configuration Register Bits Get */
#define CFGREG_BALUN_MIXER25_CONF_BALUN_EN_BITS_GET(x)                         ((x & CFGREG_BALUN_MIXER25_CONF_BALUN_EN_MASK) >> CFGREG_BALUN_MIXER25_CONF_BALUN_EN_SHIFT)                              /*!<  BALUN and MIXER 25 Configuration Register Bits Get */
#define CFGREG_BALUN_MIXER25_CONF_MIXER_EN_BITS_GET(x)                         ((x & CFGREG_BALUN_MIXER25_CONF_MIXER_EN_MASK) >> CFGREG_BALUN_MIXER25_CONF_MIXER_EN_SHIFT)                              /*!<  BALUN and MIXER 25 Configuration Register Bits Get */

#define CFGREG_PMU_CONF_FLASH_POR_EN_BITS_GET(x)                               ((x & CFGREG_PMU_CONF_FLASH_POR_EN_MASK) >> CFGREG_PMU_CONF_FLASH_POR_EN_SHIFT)                                          /*!<  PMU Configuration register Bits Get */
#define CFGREG_PMU_CONF_FLASH_BOD_EN_BITS_GET(x)                               ((x & CFGREG_PMU_CONF_FLASH_BOD_EN_MASK) >> CFGREG_PMU_CONF_FLASH_BOD_EN_SHIFT)                                          /*!<  PMU Configuration register Bits Get */
#define CFGREG_PMU_CONF_DCDC_MODE_BITS_GET(x)                                  ((x & CFGREG_PMU_CONF_DCDC_MODE_MASK) >> CFGREG_PMU_CONF_DCDC_MODE_SHIFT)                                                /*!<  PMU Configuration register Bits Get */
#define CFGREG_PMU_CONF_DCDC_GENTRIM_SRC_BITS_GET(x)                           ((x & CFGREG_PMU_CONF_DCDC_GENTRIM_SRC_MASK) >> CFGREG_PMU_CONF_DCDC_GENTRIM_SRC_SHIFT)                                  /*!<  PMU Configuration register Bits Get */
#define CFGREG_PMU_CONF_DCDC_GENTRIM_BITS_GET(x)                               ((x & CFGREG_PMU_CONF_DCDC_GENTRIM_MASK) >> CFGREG_PMU_CONF_DCDC_GENTRIM_SHIFT)                                          /*!<  PMU Configuration register Bits Get */
#define CFGREG_PMU_CONF_DCDC_TRIM_RESET_BITS_GET(x)                            ((x & CFGREG_PMU_CONF_DCDC_TRIM_RESET_MASK) >> CFGREG_PMU_CONF_DCDC_TRIM_RESET_SHIFT)                                    /*!<  PMU Configuration register Bits Get */
#define CFGREG_PMU_CONF_IREF_CAL_EN_BITS_GET(x)                                ((x & CFGREG_PMU_CONF_IREF_CAL_EN_MASK) >> CFGREG_PMU_CONF_IREF_CAL_EN_SHIFT)                                            /*!<  PMU Configuration register Bits Get */
#define CFGREG_PMU_CONF_IREF_TRIM_VAL_SRC_BITS_GET(x)                          ((x & CFGREG_PMU_CONF_IREF_TRIM_VAL_SRC_MASK) >> CFGREG_PMU_CONF_IREF_TRIM_VAL_SRC_SHIFT)                                /*!<  PMU Configuration register Bits Get */
#define CFGREG_PMU_CONF_IREF_TRIM_VAL_BITS_GET(x)                              ((x & CFGREG_PMU_CONF_IREF_TRIM_VAL_MASK) >> CFGREG_PMU_CONF_IREF_TRIM_VAL_SHIFT)                                        /*!<  PMU Configuration register Bits Get */
#define CFGREG_PMU_CONF_IREF_TRIM_EN_BITS_GET(x)                               ((x & CFGREG_PMU_CONF_IREF_TRIM_EN_MASK) >> CFGREG_PMU_CONF_IREF_TRIM_EN_SHIFT)                                          /*!<  PMU Configuration register Bits Get */

#define CFGREG_PMU_STAT_IREF_TRIM_VAL_BITS_GET(x)                              ((x & CFGREG_PMU_STAT_IREF_TRIM_VAL_MASK) >> CFGREG_PMU_STAT_IREF_TRIM_VAL_SHIFT)                                        /*!<  Power Management Unit Status Bits Get */
#define CFGREG_PMU_STAT_IREF_CAL_RDY_BITS_GET(x)                               ((x & CFGREG_PMU_STAT_IREF_CAL_RDY_MASK) >> CFGREG_PMU_STAT_IREF_CAL_RDY_SHIFT)                                          /*!<  Power Management Unit Status Bits Get */

#define CFGREG_PM_STAT_IREF_COMP_BITS_GET(x)                                   ((x & CFGREG_PM_STAT_IREF_COMP_MASK) >> CFGREG_PM_STAT_IREF_COMP_SHIFT)                                                  /*!<  GNSS-AFE PM Status Register Bits Get */
#define CFGREG_PM_STAT_VREF_COMP_BITS_GET(x)                                   ((x & CFGREG_PM_STAT_VREF_COMP_MASK) >> CFGREG_PM_STAT_VREF_COMP_SHIFT)                                                  /*!<  GNSS-AFE PM Status Register Bits Get */
#define CFGREG_PM_STAT_PWR_UP_P_BITS_GET(x)                                    ((x & CFGREG_PM_STAT_PWR_UP_P_MASK) >> CFGREG_PM_STAT_PWR_UP_P_SHIFT)                                                    /*!<  GNSS-AFE PM Status Register Bits Get */
#define CFGREG_PM_STAT_PWR_UP_N_BITS_GET(x)                                    ((x & CFGREG_PM_STAT_PWR_UP_N_MASK) >> CFGREG_PM_STAT_PWR_UP_N_SHIFT)                                                    /*!<  GNSS-AFE PM Status Register Bits Get */

#define CFGREG_PM_CONF_LDO_ADC_EXT_DECAP_BITS_GET(x)                           ((x & CFGREG_PM_CONF_LDO_ADC_EXT_DECAP_MASK) >> CFGREG_PM_CONF_LDO_ADC_EXT_DECAP_SHIFT)                                  /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_LDO_IF_EXT_DECAP_BITS_GET(x)                            ((x & CFGREG_PM_CONF_LDO_IF_EXT_DECAP_MASK) >> CFGREG_PM_CONF_LDO_IF_EXT_DECAP_SHIFT)                                    /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_LDO_DPLL_EXT_DECAP_BITS_GET(x)                          ((x & CFGREG_PM_CONF_LDO_DPLL_EXT_DECAP_MASK) >> CFGREG_PM_CONF_LDO_DPLL_EXT_DECAP_SHIFT)                                /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_LDO_APLL_EXT_DECAP_BITS_GET(x)                          ((x & CFGREG_PM_CONF_LDO_APLL_EXT_DECAP_MASK) >> CFGREG_PM_CONF_LDO_APLL_EXT_DECAP_SHIFT)                                /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_LDO_RF_EXT_DECAP_BITS_GET(x)                            ((x & CFGREG_PM_CONF_LDO_RF_EXT_DECAP_MASK) >> CFGREG_PM_CONF_LDO_RF_EXT_DECAP_SHIFT)                                    /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_RC_FILTER_BITS_GET(x)                                   ((x & CFGREG_PM_CONF_RC_FILTER_MASK) >> CFGREG_PM_CONF_RC_FILTER_SHIFT)                                                  /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_CAL_EN_BITS_GET(x)                                      ((x & CFGREG_PM_CONF_CAL_EN_MASK) >> CFGREG_PM_CONF_CAL_EN_SHIFT)                                                        /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_TEST_EN_BITS_GET(x)                                     ((x & CFGREG_PM_CONF_TEST_EN_MASK) >> CFGREG_PM_CONF_TEST_EN_SHIFT)                                                      /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_LDO_ADC_EN_BITS_GET(x)                                  ((x & CFGREG_PM_CONF_LDO_ADC_EN_MASK) >> CFGREG_PM_CONF_LDO_ADC_EN_SHIFT)                                                /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_LDO_IF_EN_BITS_GET(x)                                   ((x & CFGREG_PM_CONF_LDO_IF_EN_MASK) >> CFGREG_PM_CONF_LDO_IF_EN_SHIFT)                                                  /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_LDO_DPLL_EN_BITS_GET(x)                                 ((x & CFGREG_PM_CONF_LDO_DPLL_EN_MASK) >> CFGREG_PM_CONF_LDO_DPLL_EN_SHIFT)                                              /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_LDO_APLL_EN_BITS_GET(x)                                 ((x & CFGREG_PM_CONF_LDO_APLL_EN_MASK) >> CFGREG_PM_CONF_LDO_APLL_EN_SHIFT)                                              /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_LDO_RF_EN_BITS_GET(x)                                   ((x & CFGREG_PM_CONF_LDO_RF_EN_MASK) >> CFGREG_PM_CONF_LDO_RF_EN_SHIFT)                                                  /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_VREF_EN_BITS_GET(x)                                     ((x & CFGREG_PM_CONF_VREF_EN_MASK) >> CFGREG_PM_CONF_VREF_EN_SHIFT)                                                      /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_IREF_TRIM_SRC_BITS_GET(x)                               ((x & CFGREG_PM_CONF_IREF_TRIM_SRC_MASK) >> CFGREG_PM_CONF_IREF_TRIM_SRC_SHIFT)                                          /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_IREF_TRIM_BITS_GET(x)                                   ((x & CFGREG_PM_CONF_IREF_TRIM_MASK) >> CFGREG_PM_CONF_IREF_TRIM_SHIFT)                                                  /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_IREF_EN_BITS_GET(x)                                     ((x & CFGREG_PM_CONF_IREF_EN_MASK) >> CFGREG_PM_CONF_IREF_EN_SHIFT)                                                      /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_BGVR_TRIM_SRC_BITS_GET(x)                               ((x & CFGREG_PM_CONF_BGVR_TRIM_SRC_MASK) >> CFGREG_PM_CONF_BGVR_TRIM_SRC_SHIFT)                                          /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_BGVR_TRIM_BITS_GET(x)                                   ((x & CFGREG_PM_CONF_BGVR_TRIM_MASK) >> CFGREG_PM_CONF_BGVR_TRIM_SHIFT)                                                  /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_BGVR_EN_BITS_GET(x)                                     ((x & CFGREG_PM_CONF_BGVR_EN_MASK) >> CFGREG_PM_CONF_BGVR_EN_SHIFT)                                                      /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_PM_CONF_NRST_BITS_GET(x)                                        ((x & CFGREG_PM_CONF_NRST_MASK) >> CFGREG_PM_CONF_NRST_SHIFT)                                                            /*!<  GNSS-AFE PM Configuration Register Bits Get */

#define CFGREG_GNSSAFE_TEST_CONF_PLLTEST_ATT_BITS_GET(x)                       ((x & CFGREG_GNSSAFE_TEST_CONF_PLLTEST_ATT_MASK) >> CFGREG_GNSSAFE_TEST_CONF_PLLTEST_ATT_SHIFT)                          /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_EN_BITS_GET(x)                  ((x & CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_EN_MASK) >> CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_EN_SHIFT)                /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_DIR_BITS_GET(x)                 ((x & CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_DIR_MASK) >> CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_DIR_SHIFT)              /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_PRESC_BITS_GET(x)               ((x & CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_PRESC_MASK) >> CFGREG_GNSSAFE_TEST_CONF_PLL25TEST_MOD_PRESC_SHIFT)          /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_EN_BITS_GET(x)                   ((x & CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_EN_MASK) >> CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_EN_SHIFT)                  /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_DIR_BITS_GET(x)                  ((x & CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_DIR_MASK) >> CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_DIR_SHIFT)                /*!<  GNSS-AFE PM Configuration Register Bits Get */
#define CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_PRESC_BITS_GET(x)                ((x & CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_PRESC_MASK) >> CFGREG_GNSSAFE_TEST_CONF_PLL1TEST_MOD_PRESC_SHIFT)            /*!<  GNSS-AFE PM Configuration Register Bits Get */

#define CFGREG_SPARE_CONF_SIG_BITS_GET(x)                                      ((x & CFGREG_SPARE_CONF_SIG_MASK) >> CFGREG_SPARE_CONF_SIG_SHIFT)                                                        /*!<  SPARE Configuration Register Bits Get */

#define CFGREG_SPARE_STAT_SIG_BITS_GET(x)                                      ((x & CFGREG_SPARE_STAT_SIG_MASK) >> CFGREG_SPARE_STAT_SIG_SHIFT)                                                        /*!<  SPARE Configuration Register Bits Get */

/** @} */

#endif /* __CFG_REGS_H */
/** @} */
