/*H*****************************************************************************
*
* Copyright (c) 2020 ChipCraft Sp. z o.o. All rights reserved
*
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
*
* ******************************************************************************
* File Name : max2771.c
* Author    : Krzysztof Siwiec
* ******************************************************************************
* $Date: 2025-05-16 14:07:05 +0200 (Fri, 16 May 2025) $
* $Revision: 1152 $
*H*****************************************************************************/

#ifndef MAX2771_H_
#define MAX2771_H_

/*******************************************************************/
/** Configuration registers                                       **/
/*******************************************************************/

enum
{
    CONF1               = 0,           /*!< Configuration 1 register                */
    CONF2               = 1,           /*!< Configuration 2 register                */
    CONF3               = 2,           /*!< Configuration 3 register                */
    PLL_CONF            = 3,           /*!< PLL Configuration register              */
    PLL_INT             = 4,           /*!< PLL Integer Division Ratio register     */
    PLL_FRAC            = 5,           /*!< PLL fractional division ratio register  */
    CLK1_CONF           = 7,           /*!< Clock configuration 1 register          */
    CLK2_CONF           = 10,          /*!< Clock configuration 2 register          */
};

/*******************************************************************/
/** Configuration 1 register - configures RF and IF sections      **/
/*******************************************************************/

enum
{
    CONF1_FGAIN         = 1 << 0,       /*!< IF filter gain setting     */
    CONF1_FCENX         = 1 << 1,       /*!< Polyphase filter selection */
    CONF1_F2OR5         = 1 << 2,       /*!< Filter order selection     */
    CONF1_MIXPOLE       = 1 << 17,      /*!< Mixer pole selection       */
    CONF1_IDLE          = 1 << 30,      /*!< Idle enable                */
    CONF1_CHIPEN        = 1 << 31,      /*!< Chip enable                */
};
 
/** Configuration 1 register bit offsets */
enum
{
    CONF1_FBW_SHIFT         = 3,        /*!< IF filter bandwidth selection shift            */
    CONF1_FCEN_SHIFT        = 6,        /*!< IF filter center frequency setting shift       */
    CONF1_MIXERMODE_SHIFT   = 13,       /*!< Mixer mode selection shift                     */
    CONF1_LNAMODE_SHIFT     = 15,       /*!< LNA mode selection shift                       */
    CONF1_RSVD1_SHIFT       = 18,       /*!< Reserved: DO NOT CHANGE VALUE shift            */
    CONF1_RSVD2_SHIFT       = 20,       /*!< Reserved: DO NOT CHANGE VALUE shift            */
    CONF1_RSVD3_SHIFT       = 22,       /*!< Reserved: Write 1010 to this bitfield shift    */
    CONF1_RSVD4_SHIFT       = 26,       /*!< Reserved: Write 1111 to this bitfield shift    */
};
 
/** Configuration 1 register masks */
enum
{
    CONF1_FBW_MASK          = 0x7  << CONF1_FBW_SHIFT,          /*!< IF filter bandwidth selection mask         */
    CONF1_FCEN_MASK         = 0x7F << CONF1_FCEN_SHIFT,         /*!< IF filter center frequency setting mask    */
    CONF1_MIXERMODE_MASK    = 0x3  << CONF1_MIXERMODE_SHIFT,    /*!< Mixer mode selection mask                  */    
    CONF1_LNAMODE_MASK      = 0x3  << CONF1_LNAMODE_SHIFT,      /*!< LNA mode selection mask                    */
    CONF1_RSVD1_MASK        = 0x3  << CONF1_RSVD1_SHIFT,        /*!< Reserved: DO NOT CHANGE VALUE mask         */
    CONF1_RSVD2_MASK        = 0x3  << CONF1_RSVD2_SHIFT,        /*!< Reserved: DO NOT CHANGE VALUE mask         */
    CONF1_RSVD3_MASK        = 0xF  << CONF1_RSVD3_SHIFT,        /*!< Reserved: Write 1010 to this bitfield mask */
    CONF1_RSVD4_MASK        = 0xF  << CONF1_RSVD4_SHIFT,        /*!< Reserved: Write 1111 to this bitfield mask */
};

/** Configuration 1 register values */
enum
{
    CONF1_RSVD1_VAL = 1,
    CONF1_RSVD2_VAL = 2,
    CONF1_RSVD3_VAL = 8,
    CONF1_RSVD4_VAL = 8,
};
 
/** Configuration 1 register - CONF1_FBW_MASK */
enum
{
    CONF1_FBW_MASK_25       = 0x0,  /*!< 2.5 MHz                            */
    CONF1_FBW_MASK_87       = 0x1,  /*!< 8.7 MHz                            */
    CONF1_FBW_MASK_42       = 0x2,  /*!< 4.2 MHz                            */
    CONF1_FBW_MASK_234      = 0x3,  /*!< 23.4 MHz (lowpass mode only)       */
    CONF1_FBW_MASK_36       = 0x4,  /*!< 36.0 MHz (lowpass mode only)       */
    CONF1_FBW_MASK_164      = 0x7,  /*!< 16.4 MHz (lowpass mode only)       */
};

/** Configuration 1 register - CONF1_MIXERMODE_MASK */
enum
{
    CONF1_MIXERMODE_MASK_HB = 0x0,  /*!< High band mixer enabled            */
    CONF1_MIXERMODE_MASK_LB = 0x1,  /*!< Low band mixer enabled             */
    CONF1_MIXERMODE_MASK_D  = 0x2,  /*!< Both mixers disabled               */
};

/** Configuration 1 register - CONF1_LNAMODE_MASK */
enum
{
    CONF1_LNAMODE_MASK_HB   = 0x0,  /*!< High band LNA is active            */
    CONF1_LNAMODE_MASK_LB   = 0x1,  /*!< Low band LNA is active             */
    CONF1_LNAMODE_MASK_D    = 0x2,  /*!< Both LNAs disabled                 */
};

/*******************************************************************/
/** Configuration 2 register - configures AGC and output sections **/
/*******************************************************************/

enum
{
    CONF2_RSVD1       = 1 << 2,     /*!< Reserved: DO NOT CHANGE VALUE                                                  */
    CONF2_RSVD2       = 1 << 3,     /*!< Reserved: DO NOT CHANGE VALUE                                                  */
    CONF2_IQEN        = 1 << 27,    /*!< I and Q channels enable                                                        */
    CONF2_ANAIMON     = 1 << 28,    /*!< Enables continuous spectrum monitoring by routing analog I outputs to pins     */
    CONF2_RSVD3       = 1 << 31,    /*!< Reserved: DO NOT CHANGE VALUE                                                  */
};
 
/** Configuration 2 register bit offsets */
enum
{
    CONF2_DIEID_SHIFT           = 0,            /*!< Identifies version of IC shift                                                                 */
    CONF2_DRVCFG_SHIFT          = 4,            /*!< Output driver configuration shift                                                              */
    CONF2_BITS_SHIFT            = 6,            /*!< Number of bits in the ADC shift                                                                */
    CONF2_FORMAT_SHIFT          = 9,            /*!< Output data format shift                                                                       */
    CONF2_AGCMODE_SHIFT         = 11,           /*!< AGC mode control shift                                                                         */
    CONF2_SPI_SDIO_CONFIG_SHIFT = 13,           /*!< SPI SDIO pin configuration when tri-stated shift                                               */
    CONF2_GAINREF_SHIFT         = 15,           /*!< AGC gain reference value expressed by the number of MSB counts (magnitude bit density) shift   */
    CONF2_RSVD4_SHIFT           = 29,           /*!< Reserved: DO NOT CHANGE VALUE shift   */
};
 
/** Configuration 2 register masks */
enum
{
    CONF2_DIEID_MASK            = 0x3   << CONF2_DIEID_SHIFT,           /*!< Identifies version of IC mask                                                                  */
    CONF2_DRVCFG_MASK           = 0x3   << CONF2_DRVCFG_SHIFT,          /*!< Output driver configuration mask                                                               */
    CONF2_BITS_MASK             = 0x7   << CONF2_BITS_SHIFT,            /*!< Number of bits in the ADC mask                                                                 */
    CONF2_FORMAT_MASK           = 0x3   << CONF2_FORMAT_SHIFT,          /*!< Output data format mask                                                                        */
    CONF2_AGCMODE_MASK          = 0x3   << CONF2_AGCMODE_SHIFT,         /*!< AGC mode control mask                                                                          */
    CONF2_SPI_SDIO_CONFIG_MASK  = 0x3   << CONF2_SPI_SDIO_CONFIG_SHIFT, /*!< SPI SDIO pin configuration when tri-stated mask                                                */
    CONF2_GAINREF_MASK          = 0xFFF << CONF2_GAINREF_SHIFT,         /*!< AGC gain reference value expressed by the number of MSB counts (magnitude bit density) mask    */
    CONF2_RSVD4_MASK            = 0x3   << CONF2_RSVD4_SHIFT,           /*!< Reserved: DO NOT CHANGE VALUE mask                                                             */

};
 
/** Configuration 2 register - CONF2_DRVCFG_MASK */
enum
{
    CONF2_DRVCFG_MASK_0     = 0x0,  /*!< CMOS logic                             */
    CONF2_DRVCFG_MASK_2     = 0x2,  /*!< Analog outputs (ADC bypass mode)       */
    CONF2_DRVCFG_MASK_3     = 0x3,  /*!< Analog outputs (ADC bypass mode)       */
};

/** Configuration 2 register - CONF2_BITS_MASK */
enum
{
    CONF2_BITS_MASK_0       = 0x0,  /*!< 1 bit       */
    CONF2_BITS_MASK_2       = 0x2,  /*!< 2 bit       */
    CONF2_BITS_MASK_4       = 0x4,  /*!< 3 bit       */
};

/** Configuration 2 register - CONF2_FORMAT_MASK */
enum
{
    CONF2_FORMAT_MASK_0     = 0x0,  /*!< Unsigned binary                */
    CONF2_FORMAT_MASK_1     = 0x1,  /*!< Sign and magnitude             */
    CONF2_FORMAT_MASK_2     = 0x2,  /*!< Two's complement binary        */
    CONF2_FORMAT_MASK_3     = 0x3,  /*!< Two's complement binary        */
};

/** Configuration 2 register - CONF2_AGCMODE_MASK */
enum
{
    CONF2_AGCMODE_MASK_0    = 0x0,  /*!< Independent I and Q                        */
    CONF2_AGCMODE_MASK_2    = 0x2,  /*!< Gain set by programming of GAININ bits     */
};

/** Configuration 2 register - CONF2_SPI_SDIO_CONFIG_MASK */
enum
{
    CONF2_SPI_SDIO_CONFIG_MASK_0    = 0x0,  /*!< Nothing applied                */
    CONF2_SPI_SDIO_CONFIG_MASK_1    = 0x1,  /*!< Pull-down resistor applied     */
    CONF2_SPI_SDIO_CONFIG_MASK_2    = 0x2,  /*!< Pull-up resistor applied       */
    CONF2_SPI_SDIO_CONFIG_MASK_3    = 0x3,  /*!< Bus-hold applied               */
};

/********************************************************************************************/
/** Configuration 3 register - configures support and test functions for IF filter and AGC **/
/********************************************************************************************/

enum
{
    CONF3_STRMRST       = 1 << 0,       /*!< This command resets all the counters irrespective of the timing within the stream cycle.                   */
    CONF3_DATASYNCEN    = 1 << 1,       /*!< Enables the sync pulses at the DATASYNC output.                                                            */
    CONF3_TIMESYNCEN    = 1 << 2,       /*!< Enables the output of the time sync pulses at all times when streaming is enabled by the STRMEN command.   */
    CONF3_STAMPEN       = 1 << 3,       /*!< Enables the insertion of the frame number at the beginning of each frame.                                  */
    CONF3_STRMSTOP      = 1 << 9,       /*!< The rising edge of this bit disables data streaming to the output.                                         */
    CONF3_STRMSTART     = 1 << 10,      /*!< The rising edge of this bit enables data streaming to the output                                           */
    CONF3_STRMEN        = 1 << 11,      /*!< Enable DSP interface for serial streaming of data.                                                         */
    CONF3_PGAQEN        = 1 << 12,      /*!< Q-channel PGA enable                                                                                       */
    CONF3_PGAIEN        = 1 << 13,      /*!< I-channel PGA enable                                                                                       */
    CONF3_RSVD2         = 1 << 14,      /*!< Reserved: DO NOT CHANGE VALUE                                                                              */
    CONF3_FHIPEN        = 1 << 15,      /*!< Enable of highpass coupling between filter and PGA.                                                        */
    CONF3_RSVD3         = 1 << 16,      /*!< Reserved: DO NOT CHANGE VALUE                                                                              */
    CONF3_RSVD4         = 1 << 17,      /*!< Reserved: DO NOT CHANGE VALUE                                                                              */
    CONF3_RSVD5         = 1 << 18,      /*!< Reserved: DO NOT CHANGE VALUE                                                                              */
    CONF3_RSVD6         = 1 << 19,      /*!< Reserved: DO NOT CHANGE VALUE                                                                              */
    CONF3_HILOADEN      = 1 << 20,      /*!< Enable output driver to drive high loads                                                                   */
    CONF3_RSVD7         = 1 << 21,      /*!< Reserved: DO NOT CHANGE VALUE                                                                              */
};
 
/** Configuration 3 register bit offsets */
enum
{
    CONF3_STRMBITS_SHIFT    = 4,            /*!< Number of bits streamed shift                                              */
    CONF3_RSVD1_SHIFT       = 6,            /*!< Reserved: DO NOT CHANGE VALUE shift                                        */
    CONF3_GAININ_SHIFT      = 22,           /*!< PGA gain value programming in steps of approximately 1dB per LSB shift     */
    CONF3_RSVD8_SHIFT       = 28,           /*!< Reserved: DO NOT CHANGE VALUE shift                                        */
};
 
/** Configuration 3 register masks */
enum
{
    CONF3_STRMBITS_MASK     = 0x3   << CONF3_STRMBITS_SHIFT,    /*!< Number of bits streamed mask                                           */
    CONF3_RSVD1_MASK        = 0x7   << CONF3_RSVD1_SHIFT,       /*!< Reserved: DO NOT CHANGE VALUE mask                                     */
    CONF3_GAININ_MASK       = 0x3F  << CONF3_GAININ_SHIFT,      /*!< PGA gain value programming in steps of approximately 1dB per LSB mask  */
    CONF3_RSVD8_MASK        = 0xF   << CONF3_RSVD8_SHIFT,       /*!< Reserved: DO NOT CHANGE VALUE mask                                     */
};
 
/** Configuration 3 register - CONF3_STRMBITS_MASK */
enum
{
    CONF3_STRMBITS_MASK_1   = 0x1,      /*!<  I MSB, I LSB                  */
    CONF3_STRMBITS_MASK_3   = 0x3,      /*!<  I MSB, I LSB, Q MSB, Q LSB    */

};

/********************************************************************************************/
/**          PLL Configuration register - Configures PLL, VCO and CLK configuration        **/
/********************************************************************************************/

enum
{
    PLL_CONF_RSVD1      = 1 << 0,       /*!< Reserved: DO NOT CHANGE VALUE                                          */
    PLL_CONF_RSVD2      = 1 << 1,       /*!< Reserved: DO NOT CHANGE VALUE                                          */
    PLL_CONF_PWRSAV     = 1 << 2,       /*!< Enable PLL power-save mode                                             */
    PLL_CONF_INT_PLL    = 1 << 3,       /*!< PLL mode control. Selects either integer-N or fractional-N PLL mode.   */
    PLL_CONF_RSVD4      = 1 << 7,       /*!< Reserved: DO NOT CHANGE VALUE                                          */
    PLL_CONF_RSVD5      = 1 << 8,       /*!< Reserved: DO NOT CHANGE VALUE                                          */
    PLL_CONF_ICP        = 1 << 9,       /*!< Charge pump current selection                                          */
    PLL_CONF_RSVD7      = 1 << 13,      /*!< Reserved: DO NOT CHANGE VALUE                                          */
    PLL_CONF_RSVD10     = 1 << 23,      /*!< Reserved: DO NOT CHANGE VALUE                                          */
    PLL_CONF_REFOUTEN   = 1 << 24,      /*!< Output clock buffer enable                                             */
    PLL_CONF_RSVD11     = 1 << 25,      /*!< Reserved: DO NOT CHANGE VALUE                                          */
    PLL_CONF_RSVD12     = 1 << 26,      /*!< Reserved: DO NOT CHANGE VALUE                                          */
    PLL_CONF_RSVD13     = 1 << 27,      /*!< Reserved: DO NOT CHANGE VALUE                                          */
    PLL_CONF_LOBAND     = 1 << 28,      /*!< Local Oscillator band selection                                        */

};

/** PLL Configuration register bit offsets */
enum
{
    PLL_CONF_RSVD3_SHIFT    = 4,        /*!< Reserved: DO NOT CHANGE VALUE Shift                    */
    PLL_CONF_RSVD6_SHIFT    = 10,       /*!< Reserved: DO NOT CHANGE VALUE Shift                    */
    PLL_CONF_RSVD8_SHIFT    = 14,       /*!< Reserved: DO NOT CHANGE VALUE Shift                    */
    PLL_CONF_IXTAL_SHIFT    = 19,       /*!< Current programming for XTAL oscillator/buffer Shift   */
    PLL_CONF_RSVD9_SHIFT    = 21,       /*!< Reserved: DO NOT CHANGE VALUE Shift                    */
    PLL_CONF_REFDIV_SHIFT   = 29,       /*!< Clock output divider ratio Shift                       */

};

/** PLL Configuration register mask */
enum
{
    PLL_CONF_RSVD3_MASK     = 0x7   << PLL_CONF_RSVD3_SHIFT,        /*!< Reserved: DO NOT CHANGE VALUE Mask                     */
    PLL_CONF_RSVD6_MASK     = 0x7   << PLL_CONF_RSVD6_SHIFT,        /*!< Reserved: DO NOT CHANGE VALUE Mask                     */
    PLL_CONF_RSVD8_MASK     = 0x1F  << PLL_CONF_RSVD8_SHIFT,        /*!< Reserved: DO NOT CHANGE VALUE Mask                     */
    PLL_CONF_IXTAL_MASK     = 0x3   << PLL_CONF_IXTAL_SHIFT,        /*!< Current programming for XTAL oscillator/buffer Mask    */
    PLL_CONF_RSVD9_MASK     = 0x3   << PLL_CONF_RSVD9_SHIFT,        /*!< Reserved: DO NOT CHANGE VALUE Mask                     */
    PLL_CONF_REFDIV_MASK    = 0x7   << PLL_CONF_REFDIV_SHIFT,       /*!< Clock output divider ratio Mask                        */

};

/** PLL Configuration register - PLL_CONF_IXTAL_MASK */
enum
{
    PLL_CONF_IXTAL_MASK_1   = 0x1,      /*!<  Normal current    */
    PLL_CONF_IXTAL_MASK_3   = 0x3,      /*!<  High current      */
};

/** PLL Configuration register - PLL_CONF_REFDIV_MASK */
enum
{
    PLL_CONF_REFDIV_MASK_0      = 0x0,      /*!<  XTAL frequency x2     */
    PLL_CONF_REFDIV_MASK_1      = 0x1,      /*!<  XTAL frequency /4     */
    PLL_CONF_REFDIV_MASK_2      = 0x2,      /*!<  XTAL frequency /2     */
    PLL_CONF_REFDIV_MASK_3      = 0x3,      /*!<  XTAL frequency        */
    PLL_CONF_REFDIV_MASK_4      = 0x4,      /*!<  XTAL frequency x4     */
};

/*****************************************************************************************************************************/
/**          PLL Integer Division Ratio register - Configures PLL main and reference division ratios, other controls        **/
/*****************************************************************************************************************************/

/** PLL Integer Division Ratio register bit offsets */
enum
{
    PLL_INT_RSVD1_SHIFT     = 0,        /*!< Reserved: DO NOT CHANGE VALUE shift    */
    PLL_INT_RDIV_SHIFT      = 3,        /*!< PLL integer division ratio shift       */
    PLL_INT_NDIV_SHIFT      = 13,       /*!< PLL reference division ratio shift     */
    PLL_INT_RSVD2_SHIFT     = 28,       /*!< Reserved: DO NOT CHANGE VALUE shift    */
};

/** PLL Integer Division Ratio register mask */
enum
{
    PLL_INT_RSVD1_MASK      = 0x7      << PLL_INT_RSVD1_SHIFT,      /*!< Reserved: DO NOT CHANGE VALUE mask     */
    PLL_INT_RDIV_MASK       = 0x3FF    << PLL_INT_RDIV_SHIFT,       /*!< PLL integer division ratio mask        */
    PLL_INT_NDIV_MASK       = 0x7FFF   << PLL_INT_NDIV_SHIFT,       /*!< PLL reference division ratio mask      */
    PLL_INT_RSVD2_MASK      = 0xF      << PLL_INT_RSVD2_SHIFT,      /*!< Reserved: DO NOT CHANGE VALUE mask     */
};

/*****************************************************************************************************************************/
/**          PLL fractional division ratio register - configures PLL PLL fractional division ratio, other controls          **/
/*****************************************************************************************************************************/

enum
{
    PLL_FRAC_RSVD1      = 1 << 0,       /*!< Reserved: DO NOT CHANGE VALUE      */
    PLL_FRAC_RSVD2      = 1 << 1,       /*!< Reserved: DO NOT CHANGE VALUE      */
    PLL_FRAC_RSVD3      = 1 << 2,       /*!< Reserved: DO NOT CHANGE VALUE      */
    PLL_FRAC_RSVD4      = 1 << 3,       /*!< Reserved: DO NOT CHANGE VALUE      */
};

/** PLL Fractional Division Ratio register bit offsets */
enum
{
    PLL_FRAC_RSVD5_SHIFT    = 4,        /*!< Reserved: DO NOT CHANGE VALUE shift    */
    PLL_FRAC_FDIV_SHIFT     = 8,        /*!< PLL fractional division ratio shift    */
    PLL_FRAC_RSVD6_SHIFT    = 28,       /*!< Reserved: DO NOT CHANGE VALUE shift    */
};

/** PLL Fractional Division Ratio register bit mask */
enum
{
    PLL_FRAC_RSVD5_MASK     = 0xF      << PLL_FRAC_RSVD5_SHIFT,     /*!< Reserved: DO NOT CHANGE VALUE mask     */
    PLL_FRAC_RDIV_MASK      = 0xFFFFF  << PLL_FRAC_FDIV_SHIFT,      /*!< PLL fractional division ratio mask     */
    PLL_FRAC_RSVD6_MASK     = 0xF      << PLL_FRAC_RSVD6_SHIFT,     /*!< Reserved: DO NOT CHANGE VALUE mask     */
};

/*******************************************************************************/
/**          Clock configuration 1 register - configures clock signal         **/
/*******************************************************************************/

enum
{
    CLK1_CONF_MODE          = 1 << 0,       /*!< DSP interface mode selection                                                           */
    CLK1_CONF_RSVD1         = 1 << 1,       /*!< Reserved: DO NOT CHANGE VALUE                                                          */
    CLK1_CONF_ADCCLK        = 1 << 2,       /*!< ADC clock selection                                                                    */
    CLK1_CONF_FCLKIN        = 1 << 3,       /*!< Fractional clock divider selection                                                     */
    CLK1_CONF_EXTADCCLK     = 1 << 28,      /*!< Selects either internally generated or externally applied clock as ADC sampling clock  */
};

/** Clock configuration 1 register bit offsets */
enum
{
    CLK1_CONF_REFCLK_M_CNT_SHIFT    = 4,    /*!< Sets the value for the M counter shift     */
    CLK1_CONF_REFCLK_L_CNT_SHIFT    = 16,   /*!< Sets the value for the L counter shift     */
    CLK1_CONF_RSVD2_SHIFT           = 29,   /*!< Reserved: DO NOT CHANGE VALUE shift        */

};

/** Clock configuration 1 register bit mask */
enum
{
    CLK1_CONF_REFCLK_M_CNT_MASK     = 0xFFF      << CLK1_CONF_REFCLK_M_CNT_SHIFT,   /*!< Sets the value for the M counter mask  */
    CLK1_CONF_REFCLK_L_CNT_MASK     = 0xFFF      << CLK1_CONF_REFCLK_L_CNT_SHIFT,   /*!< Sets the value for the L counter mask  */
    CLK1_CONF_RSVD2_MASK            = 0x7        << CLK1_CONF_RSVD2_SHIFT,          /*!< Reserved: DO NOT CHANGE VALUE mask     */
};

/*******************************************************************************/
/**          Clock configuration 2 register - configures clock signal         **/
/*******************************************************************************/

enum
{
    CLK2_CONF_CLKOUT_SEL        = 1 << 2,       /*!< CLKOUT selection                       */
    CLK2_CONF_PREFRACDIV_SEL    = 1 << 3,       /*!< Fractional clock divider selection     */
    CLK2_CONF_RSVD2             = 1 << 28,      /*!< Reserved: DO NOT CHANGE VALUE          */
};

/** Clock configuration 2 register bit offsets */
enum
{
    CLK2_CONF_RSVD1_SHIFT           = 0,    /*!< Sets the value for the M counter shift     */
    CLK2_CONF_ADCCLK_M_CNT_SHIFT    = 4,    /*!< Sets the value for the L counter shift     */
    CLK2_CONF_ADCCLK_L_CNT_SHIFT    = 16,   /*!< Sets the value for the L counter shift     */
    CLK2_CONF_RSVD3_SHIFT           = 29,   /*!< Reserved: DO NOT CHANGE VALUE shift        */
};

/** Clock configuration 2 register bit mask */
enum
{
    CLK2_CONF_RSVD1_MASK            = 0x3      << CLK2_CONF_RSVD1_SHIFT,            /*!< Reserved: DO NOT CHANGE VALUE mask         */
    CLK2_CONF_ADCCLK_M_CNT_MASK     = 0xFFF    << CLK2_CONF_ADCCLK_M_CNT_SHIFT,     /*!< Sets the value for the M counter mask      */
    CLK2_CONF_ADCCLK_L_CNT_MASK     = 0xFFF    << CLK2_CONF_ADCCLK_L_CNT_SHIFT,     /*!< Sets the value for the L counter mask      */
    CLK2_CONF_RSVD3_MASK            = 0x7      << CLK2_CONF_RSVD3_SHIFT,            /*!< Reserved: DO NOT CHANGE VALUE mask         */
};

/*******************************************************************************/
/**          Bands                                                            **/
/*******************************************************************************/

enum max2771_band
{
    L1E1 = 0,   /*!< L1/E1 band     */
    L5E5 = 1,   /*!< L5/E5 band     */
    L2E6 = 2,   /*!< L2/E6 band     */
};

/*******************************************************************************/
/**          Functions                                                        **/
/*******************************************************************************/

int max2771_gpio_conf();
int max2771_gpio_unconf();
int max2771_spi_conf();
int max2771_spi_unconf();

uint32_t max2771_spi_write(enum max2771_band band, uint32_t address, uint32_t data);
uint32_t max2771_conf_band(enum max2771_band band, uint32_t adc_freq);

#endif /* MAX2771_H_ */
