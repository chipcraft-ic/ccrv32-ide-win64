/* ----------------------------------------------------------------------
*
* Copyright (c) 2021 ChipCraft Sp. z o.o. All rights reserved
*
* $Date: 2022-01-19 09:38:48 +0100 (Wed, 19 Jan 2022) $
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
    /* Configure PLL */
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_COREFREQ_PLL = CFGREG_COREFREQ_PLL_EN_MASK | (16 << CFGREG_COREFREQ_PLL_N_SHIFT) | CFGREG_COREFREQ_PLL_REF_SEL_MASK;
    while ((CFG_REGS_PTR->CFGREG_COREFREQ_STAT & CFGREG_COREFREQ_STAT_PLL_LOCK_MASK) == 0);
    /* Switch to PLL */
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_COREFREQ_CLK = 2 << CFGREG_COREFREQ_CLK_CORE_SEL_SHIFT;
}

/**
 * @brief Initialize the CCNV2 board
 */
void board_init(void)
{
    configure_gpio();
}

/**
 * @brief Initialize the CCNV2 hardware
 */
void hardware_init(void)
{

    /* Enable APB1 Bridge */
    AMBA_APB0_CFG_PTR->APB1_CFG = AMBA_APB1_EN;

    flash_configure(CORE_FREQ,(uint8_t)FLASH_READ_WAIT_STATES_CALC(CORE_FREQ),1,0);
    flash_enable_ECC();

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

    // Set carrier frequency to remove 5MHz when sample freq is 16*1.023MHz -> (5MHz / 16*1.023MHz) * 2^32 = 1312001251
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT1_L1E1_CFG_CARR_FREQ = 1312001251 << CFGREG_SPLIT1_L1E1_CFG_CARR_FREQ_STEP_SHIFT;

    // Set smooth filter alpha, enable carrier removal and set carrier mode, set decimation to 16
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT1_L1E1_CFG = CFGREG_SPLIT1_L1E1_CFG_DEC_BY_THREE_EN_MASK           | // decimation by 3
//                                         (4 << CFGREG_SPLIT1_L1E1_CFG_DEC_SAMPLES_SHIFT_SHIFT) | // 2^4 = 16 dec samples
                                           (1 << CFGREG_SPLIT1_L1E1_CFG_CARR_MODE_SHIFT)         | // carr mode 1
                                           CFGREG_SPLIT1_L1E1_CFG_CARR_ENABLE_MASK               | // enable carrier removal
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

    // Set carrier frequency to remove 5MHz when sample freq is 16*1.023MHz -> (5MHz / 16*1.023MHz) * 2^32 = 1312001251
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_L2_CFG_CARR_FREQ = 1312001251 << CFGREG_SPLIT25_L2_CFG_CARR_FREQ_STEP_SHIFT;

    // Set smooth filter alpha, enable carrier removal and set carrier mode, set decimation to 16
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_L2_CFG = CFGREG_SPLIT25_L2_CFG_DEC_BY_THREE_EN_MASK           | // decimation by 3
//                                        (4 << CFGREG_SPLIT25_L2_CFG_DEC_SAMPLES_SHIFT_SHIFT) | // 2^4 = 16 dec samples
                                          (1 << CFGREG_SPLIT25_L2_CFG_CARR_MODE_SHIFT)         | // carr mode 1
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

    // Set carrier frequency to remove 5MHz when sample freq is 16*1.023MHz -> (5MHz / 16*1.023MHz) * 2^32 = 1312001251
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_E6_CFG_CARR_FREQ = 1312001251 << CFGREG_SPLIT25_E6_CFG_CARR_FREQ_STEP_SHIFT;

    // Set smooth filter alpha, enable carrier removal and set carrier mode, set decimation to 16
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_E6_CFG = CFGREG_SPLIT25_E6_CFG_DEC_BY_THREE_EN_MASK           | // decimation by 3
//                                        (4 << CFGREG_SPLIT25_E6_CFG_DEC_SAMPLES_SHIFT_SHIFT) | // 2^4 = 16 dec samples
                                          (1 << CFGREG_SPLIT25_E6_CFG_CARR_MODE_SHIFT)         | // carr mode 1
                                          CFGREG_SPLIT25_E6_CFG_CARR_ENABLE_MASK               | // enable carrier removal
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

    // Set carrier frequency to remove 5MHz when sample freq is 16*1.023MHz -> (5MHz / 16*1.023MHz) * 2^32 = 1312001251
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_L5E5A_CFG_CARR_FREQ = 1312001251 << CFGREG_SPLIT25_L5E5A_CFG_CARR_FREQ_STEP_SHIFT;

    // Set smooth filter alpha, enable carrier removal and set carrier mode, set decimation to 16
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_L5E5A_CFG = CFGREG_SPLIT25_L5E5A_CFG_DEC_BY_THREE_EN_MASK           | // decimation by 3
//                                           (4 << CFGREG_SPLIT25_L5E5A_CFG_DEC_SAMPLES_SHIFT_SHIFT) | // 2^4 = 16 dec samples
                                             (1 << CFGREG_SPLIT25_L5E5A_CFG_CARR_MODE_SHIFT)         | // carr mode 1
                                             CFGREG_SPLIT25_L5E5A_CFG_CARR_ENABLE_MASK               | // enable carrier removal
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

    // Set carrier frequency to remove 5MHz when sample freq is 16*1.023MHz -> (5MHz / 16*1.023MHz) * 2^32 = 1312001251
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_E5B_CFG_CARR_FREQ = 1312001251 << CFGREG_SPLIT25_E5B_CFG_CARR_FREQ_STEP_SHIFT;

    // Set smooth filter alpha, enable carrier removal and set carrier mode, set decimation to 16
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_SPLIT25_E5B_CFG = CFGREG_SPLIT25_E5B_CFG_DEC_BY_THREE_EN_MASK           | // decimation by 3
//                                         (4 << CFGREG_SPLIT25_E5B_CFG_DEC_SAMPLES_SHIFT_SHIFT) | // 2^4 = 16 dec samples
                                           (1 << CFGREG_SPLIT25_E5B_CFG_CARR_MODE_SHIFT)         | // carr mode 1
                                           CFGREG_SPLIT25_E5B_CFG_CARR_ENABLE_MASK               | // enable carrier removal
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
    /* Configure GNSSAFE1 */

    /* Configure PLL1 */
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_PLL1_CONF  = CFGREG_PLL1_CONF_DEF | CFGREG_PLL1_CONF_EN_MASK;
    /* Configure IF1 */
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_IF1_CONF  = CFGREG_IF1_CONF_DEF | CFGREG_IF1_CONF_EN_MASK | CFGREG_IF1_CONF_PREAMP_EN_MASK |
                                     CFGREG_IF1_CONF_PGA1_EN_MASK | CFGREG_IF1_CONF_PGA2_EN_MASK                    ;
    /* Configure ADC1 */
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_ADC1_CONF = CFGREG_ADC1_CONF_DEF | CFGREG_ADC1_CONF_ADC_EN_MASK;

    /* Configure GNSSAFE25 */

    /* Configure PLL25 */
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_PLL25_CONF  = CFGREG_PLL25_CONF_DEF | CFGREG_PLL25_CONF_EN_MASK;
    /* Configure IF25 */
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_IF25_CONF  = CFGREG_IF25_CONF_DEF | CFGREG_IF25_CONF_EN_MASK | CFGREG_IF25_CONF_PREAMP_EN_MASK |
                                      CFGREG_IF25_CONF_PGA1_EN_MASK | CFGREG_IF25_CONF_PGA2_EN_MASK                     ;
    /* Configure ADC25 */
    CFG_REGS_PTR->CFGREG_UNLOCK = CFGREG_UNLOCK_DEF;
    CFG_REGS_PTR->CFGREG_ADC25_CONF = CFGREG_ADC25_CONF_DEF | CFGREG_ADC25_CONF_ADC_EN_MASK;

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
