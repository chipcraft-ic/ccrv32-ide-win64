/* ----------------------------------------------------------------------
*
* Copyright (c) 2021 ChipCraft Sp. z o.o. All rights reserved
*
* $Date: 2024-08-06 11:52:57 +0200 (wto, 06 sie 2024) $
* $Revision: 1090 $
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

#include <ccrv32.h>
#include <ccrv32-csr.h>
#include <ccrv32-amba.h>
#include <ccrv32-amba-cfgregs.h>
#include <ccrv32-amba-gpio.h>
#include <ccrv32-amba-edac.h>
#include <ccrv32-amba-memctrl.h>
#include <ccrv32-amba-flash.h>

#include <board.h>
#include <flash.h>
#include <ccnv2.h>

/**
 * @brief Initialize the CCNV2 GPIOs
 */
void configure_gpio(void)
{

    /* Enable GPIO controller */
    AMBA_GPIO_PTR->CTRL |= GPIO_CTRL_EN;

    /* Set UART0 to GPIO 0, 1, 2, 3 */
    AMBA_GPIO_PTR->ALTER_LO |= GPIO_CONFIG_MASK(0,GPIO_ALTER_0);
    AMBA_GPIO_PTR->ALTER_LO |= GPIO_CONFIG_MASK(1,GPIO_ALTER_0);
    AMBA_GPIO_PTR->ALTER_LO |= GPIO_CONFIG_MASK(2,GPIO_ALTER_0);
    AMBA_GPIO_PTR->ALTER_LO |= GPIO_CONFIG_MASK(3,GPIO_ALTER_0);

    /* Set I2C0 to GPIO 4, 5 */
    AMBA_GPIO_PTR->ALTER_LO |= GPIO_CONFIG_MASK(4,GPIO_ALTER_1);
    AMBA_GPIO_PTR->ALTER_LO |= GPIO_CONFIG_MASK(5,GPIO_ALTER_1);

    /* Set pull-up to GPIO 4, 5 */
    AMBA_GPIO_PTR->PULL_LO |= GPIO_CONFIG_MASK(4,GPIO_PULL_UP);
    AMBA_GPIO_PTR->PULL_LO |= GPIO_CONFIG_MASK(5,GPIO_PULL_UP);

    /* Set UART3 to GPIO 12, 13, 14, 15 */
    AMBA_GPIO_PTR->ALTER_LO |= GPIO_CONFIG_MASK(12,GPIO_ALTER_0);
    AMBA_GPIO_PTR->ALTER_LO |= GPIO_CONFIG_MASK(13,GPIO_ALTER_0);
    AMBA_GPIO_PTR->ALTER_LO |= GPIO_CONFIG_MASK(14,GPIO_ALTER_0);
    AMBA_GPIO_PTR->ALTER_LO |= GPIO_CONFIG_MASK(15,GPIO_ALTER_0);

    /* Enable PPS output */
    AMBA_GPIO_PTR->ALTER_HI |= GPIO_CONFIG_MASK(30,GPIO_ALTER_2);

    /* Disable GPIO controller */
    AMBA_GPIO_PTR->CTRL &= ~GPIO_CTRL_EN;

}

/**
 * @brief Initialize the CCNV2 EDAC controllers
 */
void configure_edac(void)
{
    AMBA_APB0_CFG_PTR->APB1_CFG = AMBA_APB1_EN;
    for (int i=0; i<RAM_PARTITIONS; i++)
    {
        AMBA_EDAC_PTR(i)->SCR_PERIOD = 10000;
        AMBA_EDAC_PTR(i)->CTRL = EDAC_CTRL_SLP_EN | EDAC_CTRL_ECC_EN | EDAC_CTRL_ERR_EN | EDAC_CTRL_SBR_EN | EDAC_CTRL_INJ_EN;
    }
    AMBA_APB0_CFG_PTR->APB1_CFG &= ~AMBA_APB1_EN;
}

/**
 * @brief Initialize the CCNV2 PLL
 */
void configure_pll(void)
{
	//Clear clock scaling
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_COREFREQ_CLK = CFG_REGS_PTR->CFGREG_COREFREQ_CLK & ~CFGREG_COREFREQ_CLK_CORE_XTAL_SCALE_MASK & ~CFGREG_COREFREQ_CLK_GNSSAFE_XTAL_SCALE_MASK;
	//Set XTAL prescaler for MCU
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_COREFREQ_CLK |= (1 << CFGREG_COREFREQ_CLK_CORE_XTAL_SCALE_SHIFT);

    //CCNV2_B1 TCXO fix
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;    
    CFG_REGS_PTR->CFGREG_PM_CONF = CFGREG_PM_CONF_DEF  | CFGREG_PM_CONF_BGVR_EN_MASK  | CFGREG_PM_CONF_IREF_EN_MASK | CFGREG_PM_CONF_VREF_EN_MASK |
    CFGREG_PM_CONF_LDO_ADC_EXT_DECAP_MASK | CFGREG_PM_CONF_LDO_IF_EXT_DECAP_MASK | CFGREG_PM_CONF_LDO_DPLL_EXT_DECAP_MASK |
                            CFGREG_PM_CONF_LDO_APLL_EXT_DECAP_MASK | CFGREG_PM_CONF_LDO_RF_EXT_DECAP_MASK |
                            CFGREG_PM_CONF_LDO_ADC_EN_MASK | CFGREG_PM_CONF_LDO_IF_EN_MASK | CFGREG_PM_CONF_LDO_DPLL_EN_MASK | CFGREG_PM_CONF_LDO_APLL_EN_MASK |
                            CFGREG_PM_CONF_LDO_RF_EN_MASK;
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF; 
    CFG_REGS_PTR->CFGREG_SPARE_CONF = 0x8;

    /* Configure PLL */
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_COREFREQ_PLL = CFGREG_COREFREQ_PLL_EN_MASK | (MCU_PLL_N << CFGREG_COREFREQ_PLL_N_SHIFT) | CFGREG_COREFREQ_PLL_REF_SEL_MASK;
    while ((CFG_REGS_PTR->CFGREG_COREFREQ_STAT & CFGREG_COREFREQ_STAT_PLL_LOCK_MASK) == 0);
    /* Switch to PLL */
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_COREFREQ_CLK = (CFGREG_COREFREQ_CLK_DEF & ~CFGREG_COREFREQ_CLK_CORE_SEL_MASK) | 2 << CFGREG_COREFREQ_CLK_CORE_SEL_SHIFT;
    /* switch debugger to new baud rate */
    csr_write(mdbgbaud,DBG_UART_PRES((CORE_FREQ / DBG_BAUDRATE) / 16, (CORE_FREQ / DBG_BAUDRATE) % 16));
}

/**
 * @brief Initialize the CCNV2 board
 */
void board_init(void)
{
    configure_gpio();
}

// helper function
static inline uint32_t int_log2(const uint32_t x){return (31 - __builtin_clzl(x));}

/**
 * @brief Initialize the CCNV2 hardware
 */
void hardware_init(void)
{

    /* Enable APB1 Bridge */
    AMBA_APB0_CFG_PTR->APB1_CFG = AMBA_APB1_EN;

    flash_configure_default( CORE_FREQ );
    flash_sequential_prefetch_enable();
    flash_ECC_enable();

    configure_pll();
    configure_edac();

}

/**
 * @brief Initialize GNSS splitter
 */
void gnss_splitter_init(void)
{
    /********************/
    /* Configure SPLIT1 */
    /********************/

    /* L1E1 */

    // Set threshold values for standard deviation of I channel
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I = (0x1000 << CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_MAX_SHIFT) |
                                                       (0x0C00 << CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_I_MIN_SHIFT) ;

    // Set threshold values for standard deviation of Q channel
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q = (0x1000 << CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_MAX_SHIFT) |
                                                       (0x0C00 << CFGREG_SPLIT1_L1E1_CFG_THRESHOLD_Q_MIN_SHIFT) ;

    // Set carrier frequency to remove 0 when sample freq is 64*1.023MHz -> (0MHz / 64*1.023MHz) * 2^32 = 0
    // Signal present at 4*1.023MHz - remaining IF equal to 4*1.023MHz
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT1_L1E1_CFG_CARR_FREQ = (1 <<28) << CFGREG_SPLIT1_L1E1_CFG_CARR_FREQ_STEP_SHIFT;

    // Set smooth filter alpha, disble carrier removal and set carrier mode, set decimation to 3
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT1_L1E1_CFG =  CFGREG_SPLIT1_L1E1_CFG_DEC_BY_THREE_EN_MASK          | // decimation by 3
                                           (0 << CFGREG_SPLIT1_L1E1_CFG_CARR_MODE_SHIFT)         | // carr mode 0
                                           (0 << CFGREG_SPLIT1_L1E1_CFG_CARR_ENABLE_SHIFT)       | // carr enable
                                           (7 << CFGREG_SPLIT1_L1E1_CFG_LPF_K_PARAM_SHIFT)       ; // alpha = (1 / 1024)

    /* General */

    // Set gain chagne counter to 199999 (default value), so for 200MHz clk gain will change every 1ms
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT1_CFG_GAIN_CHANGE = 199999 << CFGREG_SPLIT1_CFG_GAIN_CHANGE_CNT_SHIFT;

    // Set maximal number of extremum samples between gain updates to 50000 (default value),
    // so if more than 25% (limiter/gain_change) of signal is extremum PGA will be disabled
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT1_CFG_LIMITER = 50000 << CFGREG_SPLIT1_CFG_LIMITER_THRESHOLD_SHIFT;

    // Disable manual mode
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT1_CFG_MANUAL_MODE = 0;

    // Clear splitter - in simulation this is needed to remove Xs, in real life probably won't change anything
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT1_CTRL = 1;
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT1_CTRL = 0;

    /*********************/
    /* Configure SPLIT25 */
    /*********************/

    /* L2 */

    // Set threshold values for standard deviation of I channel
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_L2_CFG_THRESHOLD_I = (0x1000 << CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_MAX_SHIFT) |
                                                      (0x0C00 << CFGREG_SPLIT25_L2_CFG_THRESHOLD_I_MIN_SHIFT) ;

    // Set threshold values for standard deviation of Q channel
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q = (0x1000 << CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_MAX_SHIFT) |
                                                      (0x0C00 << CFGREG_SPLIT25_L2_CFG_THRESHOLD_Q_MIN_SHIFT) ;

    // Set carrier frequency to remove 6*1.023MHz when sample freq is 192*1.023MHz -> (6*1.023MHz / 192*1.023MHz) * 2^32 = 134217728
    // Signal present at 6*1.023MHz - remaining IF equal to 0MHz
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_L2_CFG_CARR_FREQ = 134217728 << CFGREG_SPLIT25_L2_CFG_CARR_FREQ_STEP_SHIFT;

    // Set smooth filter alpha, enable carrier removal and set carrier mode, set decimation to 3
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_L2_CFG = CFGREG_SPLIT25_L2_CFG_DEC_BY_THREE_EN_MASK           | // decimation by 3
                                          (0 << CFGREG_SPLIT25_L2_CFG_CARR_MODE_SHIFT)         | // carr mode 1
                                          CFGREG_SPLIT25_L2_CFG_CARR_ENABLE_MASK               | // enable carrier removal
                                          (7 << CFGREG_SPLIT25_L2_CFG_LPF_K_PARAM_SHIFT)       ; // alpha = (1 / 1024)

    /* E6 */

    // Set threshold values for standard deviation of I channel
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_E6_CFG_THRESHOLD_I = (0x1000 << CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_MAX_SHIFT) |
                                                      (0x0C00 << CFGREG_SPLIT25_E6_CFG_THRESHOLD_I_MIN_SHIFT) ;

    // Set threshold values for standard deviation of Q channel
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q = (0x1000 << CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_MAX_SHIFT) |
                                                      (0x0C00 << CFGREG_SPLIT25_E6_CFG_THRESHOLD_Q_MIN_SHIFT) ;

    // Set carrier frequency to remove 51*1.023MHz when sample freq is 192*1.023MHz -> (51*1.023MHz / 192*1.023MHz) * 2^32 = 1140850688
    // Signal present at 56*1.023MHz - remaining IF equal to 4*1.023MHz
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_E6_CFG_CARR_FREQ = 1140850688 << CFGREG_SPLIT25_E6_CFG_CARR_FREQ_STEP_SHIFT;

    // Set smooth filter alpha, enable carrier removal and set carrier mode, set decimation to 3
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_E6_CFG =  CFGREG_SPLIT25_E6_CFG_DEC_BY_THREE_EN_MASK          | // decimation by 3
                                          (0 << CFGREG_SPLIT25_E6_CFG_CARR_MODE_SHIFT)         | // carr mode 1
                                          (1 << CFGREG_SPLIT25_E6_CFG_CARR_ENABLE_SHIFT)       | // enable carrier removal
                                          (7 << CFGREG_SPLIT25_E6_CFG_LPF_K_PARAM_SHIFT)       ; // alpha = (1 / 1024)

    /* L5E5A */

    // Set threshold values for standard deviation of I channel
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I = (0x1000 << CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_MAX_SHIFT) |
                                                         (0x0C00 << CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_I_MIN_SHIFT) ;

    // Set threshold values for standard deviation of Q channel
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q = (0x1000 << CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_MAX_SHIFT) |
                                                         (0x0C00 << CFGREG_SPLIT25_L5E5A_CFG_THRESHOLD_Q_MIN_SHIFT) ;

    // Set carrier frequency to remove -45*1.023MHz when sample freq is 192*1.023MHz -> 2^32 - (45*1.023MHz / 192*1.023MHz) * 2^32 = 1006632960
    // Signal present at -44*1.023MHz - remaining IF equal to 1.023MHz
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_L5E5A_CFG_CARR_FREQ = 1006632960 << CFGREG_SPLIT25_L5E5A_CFG_CARR_FREQ_STEP_SHIFT;

    // Set smooth filter alpha, enable carrier removal and set carrier mode, set decimation to 3
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_L5E5A_CFG = CFGREG_SPLIT25_L5E5A_CFG_DEC_BY_THREE_EN_MASK         | // decimation by 3
                                           (1 << CFGREG_SPLIT25_L5E5A_CFG_CARR_MODE_SHIFT)         | // carr mode 0
                                           (1 << CFGREG_SPLIT25_L5E5A_CFG_CARR_ENABLE_SHIFT)       | // carr enable
                                           (7 << CFGREG_SPLIT25_L5E5A_CFG_LPF_K_PARAM_SHIFT)       ; // alpha = (1 / 1024)

    /* E5B */

    // Set threshold values for standard deviation of I channel
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I = (0x1000 << CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_MAX_SHIFT) |
                                                       (0x0C00 << CFGREG_SPLIT25_E5B_CFG_THRESHOLD_I_MIN_SHIFT) ;

    // Set threshold values for standard deviation of Q channel
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q = (0x1000 << CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_MAX_SHIFT) |
                                                       (0x0C00 << CFGREG_SPLIT25_E5B_CFG_THRESHOLD_Q_MIN_SHIFT) ;

    // Set carrier frequency to remove 0 when sample freq is 192*1.023MHz -> (0MHz / 192*1.023MHz) * 2^32 = 0
    // Signal present at -14*1.023MHz - remaining IF equal to -14*1.023MHz
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_E5B_CFG_CARR_FREQ = 0 << CFGREG_SPLIT25_E5B_CFG_CARR_FREQ_STEP_SHIFT;

    // Set smooth filter alpha, enable carrier removal and set carrier mode, set decimation to 3
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_E5B_CFG = CFGREG_SPLIT25_E5B_CFG_DEC_BY_THREE_EN_MASK           | // decimation by 3
                                           (1 << CFGREG_SPLIT25_E5B_CFG_CARR_MODE_SHIFT)         | // carr mode 1
                                           (0 << CFGREG_SPLIT25_E5B_CFG_CARR_ENABLE_MASK)        | // disable carrier removal
                                           (7 << CFGREG_SPLIT25_E5B_CFG_LPF_K_PARAM_SHIFT)       ; // alpha = (1 / 1024)

    /* General */

    // Set gain chagne counter to 199999 (default value), so for 200MHz clk gain will change every 1ms
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_CFG_GAIN_CHANGE = 199999 << CFGREG_SPLIT1_CFG_GAIN_CHANGE_CNT_SHIFT;

    // Set maximal number of extremum samples between gain updates to 50000 (default value),
    // so if more than 25% (limiter/gain_change) of signal is extremum PGA will be disabled
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_CFG_LIMITER = 50000 << CFGREG_SPLIT1_CFG_LIMITER_THRESHOLD_SHIFT;

    // Disable manual mode
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_CFG_MANUAL_MODE = 0;

    // Clear splitter - in simulation this is needed to remove Xs, in real life probably won't change anything
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_CTRL = 1;
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_CTRL = 0;

}

/**
 * @brief Initialize GNSS AFE registers
 */
void gnss_afe_regs(void)
{

    /* Configure AFE PM */
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;    
    CFG_REGS_PTR->CFGREG_PM_CONF = CFGREG_PM_CONF_DEF  | CFGREG_PM_CONF_BGVR_EN_MASK  | CFGREG_PM_CONF_IREF_EN_MASK | CFGREG_PM_CONF_VREF_EN_MASK |
    CFGREG_PM_CONF_LDO_ADC_EXT_DECAP_MASK | CFGREG_PM_CONF_LDO_IF_EXT_DECAP_MASK | CFGREG_PM_CONF_LDO_DPLL_EXT_DECAP_MASK |
                            CFGREG_PM_CONF_LDO_APLL_EXT_DECAP_MASK | CFGREG_PM_CONF_LDO_RF_EXT_DECAP_MASK |
                            CFGREG_PM_CONF_LDO_ADC_EN_MASK | CFGREG_PM_CONF_LDO_IF_EN_MASK | CFGREG_PM_CONF_LDO_DPLL_EN_MASK | CFGREG_PM_CONF_LDO_APLL_EN_MASK |
                            CFGREG_PM_CONF_LDO_RF_EN_MASK;
    //CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    //CFG_REGS_PTR->CFGREG_SPARE_CONF  = 1;
  
    /* Configure GNSSAFE1 */

    /* Configure PLL1 */
	// Set the prescaler to 2 and multipliaction factor to 5363466 (81.84 * 38.4M/2 ~= 1536*1.023e6, the error is 70 Hz, which is negligable) - hex 0x51d70a, gives ADC freq of 192*1.023e6 when divided by 8
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_PLL1_CONF  = CFGREG_PLL1_CONF_EN_MASK | (1 << CFGREG_PLL1_CONF_PRESC_SHIFT) | (0x51d70a << CFGREG_PLL1_CONF_FCW_SHIFT) | CFGREG_PLL1_CONF_ADC_CLK_DIV_MASK;

    /* Configure LNA125*/
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_LNA125_CONF = CFGREG_LNA125_CONF_DEF | CFGREG_LNA125_CONF_EN_L1_MASK | CFGREG_LNA125_CONF_EN_L25_MASK | CFGREG_LNA125_CONF_L1_TUNE_SRC_MASK |
                                        CFGREG_LNA125_CONF_L25_TUNE_SRC_MASK;

    /* Configure BALUN_MIXER1*/
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_BALUN_MIXER1_CONF = CFGREG_BALUN_MIXER1_CONF_DEF | CFGREG_BALUN_MIXER1_CONF_MIXER_EN_MASK | CFGREG_BALUN_MIXER1_CONF_BALUN_EN_MASK |
                                        CFGREG_BALUN_MIXER1_CONF_BALUN_TUNE_SRC_MASK | (0x00FF << CFGREG_BALUN_MIXER1_CONF_BALUN_TUNE_SHIFT);

    /* Configure IF1 */
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_IF1_CONF  = CFGREG_IF1_CONF_DEF | CFGREG_IF1_CONF_EN_MASK | CFGREG_IF1_CONF_PREAMP_EN_MASK |
                                     CFGREG_IF1_CONF_PGA1_EN_MASK | CFGREG_IF1_CONF_PGA2_EN_MASK | (0 << CFGREG_IF1_CONF_IF_BANDCUT_SHIFT);

    /* Configure ADC1 */
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_ADC1_CONF = CFGREG_ADC1_CONF_DEF | CFGREG_ADC1_CONF_ADC_EN_MASK | CFGREG_ADC1_CONF_SAH_EN_MASK | (0 << CFGREG_ADC1_CONF_CLK_SEL_SHIFT) | (0 << CFGREG_ADC1_CONF_CLK_CONF_SHIFT) | CFGREG_ADC1_CONF_CAL_EN_MASK;

    /* Configure GNSSAFE25 */
	
	/* Configure PLL25 */
	// Set the prescaler to 2 and multipliaction factor to 4169257 (63.61781311035156 * 38.4M/2 ~= 1194*1.023e6, the error is 12 Hz, which is negligable) - hex 0x3f9e29
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_PLL25_CONF  = CFGREG_PLL25_CONF_EN_MASK | (0x3f9e29 << CFGREG_PLL25_CONF_FCW_SHIFT) | CFGREG_PLL25_CONF_ADC_CLK_DIV_MASK;

    /* Configure BALUN_MIXER25*/
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_BALUN_MIXER25_CONF = CFGREG_BALUN_MIXER25_CONF_DEF | CFGREG_BALUN_MIXER25_CONF_MIXER_EN_MASK | CFGREG_BALUN_MIXER25_CONF_BALUN_EN_MASK |
                                        CFGREG_BALUN_MIXER25_CONF_BALUN_TUNE_SRC_MASK | (0x00FF << CFGREG_BALUN_MIXER25_CONF_BALUN_TUNE_SHIFT);

    /* Configure IF25 */
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_IF25_CONF  = CFGREG_IF25_CONF_DEF | CFGREG_IF25_CONF_EN_MASK | CFGREG_IF25_CONF_PREAMP_EN_MASK |
                                      CFGREG_IF25_CONF_PGA1_EN_MASK | CFGREG_IF25_CONF_PGA2_EN_MASK                     ;
    /* Configure ADC25 */
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_ADC25_CONF = CFGREG_ADC25_CONF_DEF | CFGREG_ADC25_CONF_ADC_EN_MASK | CFGREG_ADC25_CONF_SAH_EN_MASK | (0 << CFGREG_ADC25_CONF_CLK_SEL_SHIFT) | (0 << CFGREG_ADC25_CONF_CLK_CONF_SHIFT) | CFGREG_ADC25_CONF_CAL_EN_MASK;

    while( (CFG_REGS_PTR->CFGREG_PM_STAT & CFGREG_PM_STAT_PWR_UP_P_MASK) == 0) ;
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_PM_CONF |= CFGREG_PM_CONF_NRST_MASK;


    while( (CFG_REGS_PTR->CFGREG_PLL1_STAT & CFGREG_PLL1_STAT_LOCK_MASK) == 0){};
	while( (CFG_REGS_PTR->CFGREG_PLL25_STAT & CFGREG_PLL25_STAT_LOCK_MASK) == 0){};
}

/**
 * @brief Initialize GNSS AFE
 */
int gnss_afe_init(void)
{
    /* Initialize registers */
    gnss_afe_regs();

    /* Initialize splitter */
    gnss_splitter_init();

    return 0;
}

/**
 * @brief Initialize hyperbus
 */
void hyperbus_configure(void)
{

    uint32_t data;

    /* Enable GPIO controller */
    AMBA_GPIO_PTR->CTRL |= GPIO_CTRL_EN;

    /* Enable APB1 Bridge */
    AMBA_APB0_CFG_PTR->APB1_CFG = AMBA_APB1_EN;

    /* Set Hyperbus pins */
    for (int i=16; i<29; i++){
        AMBA_GPIO_PTR->ALTER_HI  |= GPIO_CONFIG_MASK(i,GPIO_ALTER_0);
        AMBA_GPIO_PTR->DRIVER_HI |= GPIO_CONFIG_MASK(i,GPIO_DRIVE_1);
        AMBA_GPIO_PTR->SLEW_RATE |= 1<<i;
    }

    AMBA_MEMCTRL_PTR->DEVICE_ADDR_LO[0] = ROM_EXT_BASE;
    AMBA_MEMCTRL_PTR->DEVICE_ADDR_HI[0] = ROM_EXT_BASE + 0x4000000 - 1;
    AMBA_MEMCTRL_PTR->DEVICE_ADDR_LO[1] = RAM_EXT_BASE;
    AMBA_MEMCTRL_PTR->DEVICE_ADDR_HI[1] = RAM_EXT_BASE + 0x0800000 - 1;

    AMBA_MEMCTRL_PTR->DEVICE_CONF[1] = MEMCTRL_DEVICE_CONF_EN | MEMCTRL_BUILD_LATENCY(6);

    AMBA_MEMCTRL_PTR->CONF = MEMCTRL_CONF_CKPAD_EN | MEMCTRL_CONF_CSPAD_EN | MEMCTRL_CONF_CKMST_EN | MEMCTRL_BUILD_PRESCALER(1);

                                      // read             register space   linear burst     conf0
    AMBA_MEMCTRL_PTR->COMMAND_DATA[1] = (1 << (47-16)) | (1 << (46-16)) | (1 << (45-16)) | (1 << (24-16));
    AMBA_MEMCTRL_PTR->COMMAND_DATA[0] = 0;

    // set CS
    AMBA_MEMCTRL_PTR->CONF |= 2 | (1<<8);
    // issue read command
    AMBA_MEMCTRL_PTR->COMMAND = 1;
    // wait for busy
    while (AMBA_MEMCTRL_PTR->STATUS & 1);
    // clear CS
    AMBA_MEMCTRL_PTR->CONF &= ~(2 | (1<<8));

    data = AMBA_MEMCTRL_PTR->STATUS >> 16;
    //printf("Config reg 0 = 0x%x\n",(unsigned int)data);

                                      // register space   linear burst     conf0
    AMBA_MEMCTRL_PTR->COMMAND_DATA[1] = (1 << (46-16)) | (1 << (45-16)) | (1 << (24-16));
    // defaults + 3 cycle latency
    data &= ~(0xF << 4);
    data |= 0xE << 4;
    AMBA_MEMCTRL_PTR->COMMAND_DATA[0] = data;

    // set CS
    AMBA_MEMCTRL_PTR->CONF |= 2 | (1<<8);
    // issue zero delay write
    AMBA_MEMCTRL_PTR->COMMAND = 4;
    // wait for busy
    while (AMBA_MEMCTRL_PTR->STATUS & 1);
    // clear CS
    AMBA_MEMCTRL_PTR->CONF &= ~(2 | (1<<8));

    // set new latency
    AMBA_MEMCTRL_PTR->DEVICE_CONF[1] = MEMCTRL_DEVICE_CONF_EN | MEMCTRL_BUILD_LATENCY(3);

}
