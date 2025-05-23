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

#include <stdio.h>
#include <stdbool.h>
#include <board.h>
#include <ccrv32.h>
#include <ccrv32-amba.h>
#include <ccrv32-amba-gpio.h>
#include <ccrv32-amba-spi.h>

#include "max2771.h"

//Configure GPIOs for MAX2771 programming
int max2771_gpio_conf()
{
    //set GPIOs as output
    AMBA_GPIO_PTR->OUTSET |= (1 << MAX2771_L1E1_CS) | (1 << MAX2771_L5E5_CS) | (1 << MAX2771_L2E6_CS);
    AMBA_GPIO_PTR->DIRSET |= (1 << MAX2771_L1E1_CS) | (1 << MAX2771_L5E5_CS) | (1 << MAX2771_L2E6_CS);
    return 1;
}

//Return to default GPIOs settings
int max2771_gpio_unconf()
{
    //reset GPIOs
    AMBA_GPIO_PTR->OUTCLR |= (1 << MAX2771_L1E1_CS) | (1 << MAX2771_L5E5_CS) | (1 << MAX2771_L2E6_CS);
    AMBA_GPIO_PTR->DIRCLR |= (1 << MAX2771_L1E1_CS) | (1 << MAX2771_L5E5_CS) | (1 << MAX2771_L2E6_CS);
    return 1;
}

//Configure SPI for MAX2771 programming
int max2771_spi_conf()
{
    AMBA_SPI_PTR(MAX2771_SPI_NUM)->CTRL &= ~SPI_CTRL_MODE_MASK;
    AMBA_SPI_PTR(MAX2771_SPI_NUM)->CTRL |= SPI_MODE0 << SPI_CTRL_MODE_SHIFT;
    AMBA_SPI_PTR(MAX2771_SPI_NUM)->CTRL &= ~SPI_CTRL_FLEN_MASK;
    AMBA_SPI_PTR(MAX2771_SPI_NUM)->CTRL |= SPI_FLEN16 << SPI_CTRL_FLEN_SHIFT;
    AMBA_SPI_PTR(MAX2771_SPI_NUM)->CTRL &= ~SPI_CTRL_PRESCALER_MASK;
    AMBA_SPI_PTR(MAX2771_SPI_NUM)->CTRL |= 16 << SPI_CTRL_PRESC_SHIFT;
    AMBA_SPI_PTR(MAX2771_SPI_NUM)->CTRL |= SPI_CTRL_MSB;
    AMBA_SPI_PTR(MAX2771_SPI_NUM)->CTRL |= SPI_CTRL_EN;
    return 1;
}

//Reset SPI
int max2771_spi_unconf()
{
    AMBA_SPI_PTR(MAX2771_SPI_NUM)->CTRL = 0;
    return 1;
}

//Perform write operation
uint32_t max2771_spi_write(enum max2771_band band, uint32_t address,uint32_t data)
{

    uint32_t watchdog_spi = (10000 / (1000000000 / PERIPH0_FREQ)) + 1;
    while((AMBA_SPI_PTR(MAX2771_SPI_NUM)->STATUS & SPI_STAT_TDRE) == 0 && watchdog_spi > 0 )
    {
        watchdog_spi -= 1;
    }
    
    //chip select gpio number for proper band
    uint8_t max_gpio_cs;
    switch(band)
    {
        case L1E1:
            max_gpio_cs = MAX2771_L1E1_CS;
            break;
        case L5E5:
            max_gpio_cs = MAX2771_L5E5_CS;
            break;
        case L2E6:
            max_gpio_cs = MAX2771_L2E6_CS;
            break;
        default:
            max_gpio_cs = 0;
            return 0;
    }

    //Disable SPI at MAX2771
    AMBA_GPIO_PTR->OUTSET = 1 << max_gpio_cs;

    //Wait more than 100 ns
    watchdog_spi = (100 / (1000000000 / PERIPH0_FREQ)) + 1;
    while(watchdog_spi>0)
    {
        watchdog_spi -= 1;
    }

    //Enable SPI at MAX2771
    AMBA_GPIO_PTR->OUTCLR = 1 << max_gpio_cs;

    //Send frame
    //send address
    AMBA_SPI_PTR(MAX2771_SPI_NUM)->TDR = ((((address & 0x0000000F) << 1) + 0) << 3);

    //wait for transmission end
    watchdog_spi = (100000 / (1000000000 / PERIPH0_FREQ)) + 1;
    while((AMBA_SPI_PTR(MAX2771_SPI_NUM)->STATUS & SPI_STAT_TDRE) == 0 && watchdog_spi > 0 )
    {
        watchdog_spi -= 1;
    }

    //send data MSB
    AMBA_SPI_PTR(MAX2771_SPI_NUM)->TDR = ((data >> 16) & 0x0000FFFF);

    //wait for transmission end
    watchdog_spi = (100000 / (1000000000 / PERIPH0_FREQ)) + 1;
    while((AMBA_SPI_PTR(MAX2771_SPI_NUM)->STATUS & SPI_STAT_TDRE) == 0 && watchdog_spi > 0 )
    {
        watchdog_spi -= 1;
    }

    //send data LSB
    AMBA_SPI_PTR(MAX2771_SPI_NUM)->TDR = data & 0x0000FFFF;

    //wait for transmission end
    watchdog_spi = (100000 / (1000000000 / PERIPH0_FREQ)) + 1;
    while(((AMBA_SPI_PTR(MAX2771_SPI_NUM)->STATUS & SPI_STAT_TDRE) == 0 || (AMBA_SPI_PTR(MAX2771_SPI_NUM)->STATUS & SPI_STAT_TXC) == 0) && watchdog_spi > 0 )
    {
        watchdog_spi -= 1;
    }

    //Disable SPI at MAX2771
    AMBA_GPIO_PTR->OUTSET = 1 << max_gpio_cs;

    return 1;
}

//Perform read operation
uint32_t max2771_spi_read(enum max2771_band band, uint32_t address)
{

    uint32_t data = 0;

    uint32_t watchdog_spi = (20000 / (1000000000 / PERIPH0_FREQ)) + 1;
    while((AMBA_SPI_PTR(MAX2771_SPI_NUM)->STATUS & SPI_STAT_TDRE) == 0 && watchdog_spi > 0 )
    {
        watchdog_spi -= 1;
    }
    
    //chip select gpio number for proper band
    uint8_t max_gpio_cs;
    switch(band)
    {
        case L1E1:
            max_gpio_cs = MAX2771_L1E1_CS;
            break;
        case L5E5:
            max_gpio_cs = MAX2771_L5E5_CS;
            break;
        case L2E6:
            max_gpio_cs = MAX2771_L2E6_CS;
            break;
        default:
            max_gpio_cs = 0;
            return 0;
    }

    //Disable SPI at MAX2771
    AMBA_GPIO_PTR->OUTSET = 1 << max_gpio_cs;

    //Wait more than 200 ns
    watchdog_spi = (200 / (1000000000 / PERIPH0_FREQ)) + 1;
    while(watchdog_spi>0)
    {
        watchdog_spi -= 1;
    }

    //Enable SPI at MAX2771
    AMBA_GPIO_PTR->OUTCLR = 1 << max_gpio_cs;

    //Send frame
    //send address
    AMBA_SPI_PTR(MAX2771_SPI_NUM)->TDR = ((((address & 0x0000000F) << 1) + 1) << 3);

    //wait for transmission end
    watchdog_spi = (200000 / (1000000000 / PERIPH0_FREQ)) + 1;
    while((AMBA_SPI_PTR(MAX2771_SPI_NUM)->STATUS & SPI_STAT_TXC) == 0 && watchdog_spi > 0 )
    {
        watchdog_spi -= 1;
    }

    data = AMBA_SPI_PTR(MAX2771_SPI_NUM)->RDR;
    data = 0;

    //send data MSB
    AMBA_SPI_PTR(MAX2771_SPI_NUM)->TDR = 0xFFFFFFFF;

    //wait for transmission end
    watchdog_spi = (200000 / (1000000000 / PERIPH0_FREQ)) + 1;
    while((AMBA_SPI_PTR(MAX2771_SPI_NUM)->STATUS & SPI_STAT_TXC) == 0 && watchdog_spi > 0 )
    {
        watchdog_spi -= 1;
    }

    data = (AMBA_SPI_PTR(MAX2771_SPI_NUM)->RDR) << 16;

    //send data LSB
    AMBA_SPI_PTR(MAX2771_SPI_NUM)->TDR = 0xFFFFFFFF;

    //wait for transmission end
    watchdog_spi = (200000 / (1000000000 / PERIPH0_FREQ)) + 1;
    while(((AMBA_SPI_PTR(MAX2771_SPI_NUM)->STATUS & SPI_STAT_TXC) == 0) && watchdog_spi > 0 )
    {
        watchdog_spi -= 1;
    }

    data |= AMBA_SPI_PTR(MAX2771_SPI_NUM)->RDR & 0x0000FFFF;

    //Disable SPI at MAX2771
    AMBA_GPIO_PTR->OUTSET = 1 << max_gpio_cs;

    return data;
}

//configure band
uint32_t max2771_conf_band(enum max2771_band band, uint32_t adc_freq)
{

    bool beidou = false;
    bool glonass = false;
    uint32_t data;

    if ((adc_freq != 16368000) && (adc_freq != 32736000) && (adc_freq != 65472000))
    {
        return -1;
    }

    max2771_gpio_conf();
    max2771_spi_conf();
    switch(band)
    {
        case L1E1:
            if (beidou)
            {

                //write register 0
                data = CONF1_FGAIN | CONF1_F2OR5 | (CONF1_FBW_MASK_36 << CONF1_FBW_SHIFT) | (24 << CONF1_FCEN_SHIFT) | CONF1_MIXPOLE |
                    (CONF1_RSVD1_VAL << CONF1_RSVD1_SHIFT) | (CONF1_RSVD2_VAL << CONF1_RSVD2_SHIFT) | (CONF1_RSVD3_VAL << CONF1_RSVD3_SHIFT) |
                    (CONF1_RSVD4_VAL << CONF1_RSVD4_SHIFT) | CONF1_CHIPEN;
                max2771_spi_write(band,CONF1,data);

                //write register 1
                data = CONF2_RSVD2 | (CONF2_BITS_MASK_2 << CONF2_BITS_SHIFT) | (CONF2_FORMAT_MASK_1 << CONF2_FORMAT_SHIFT) |
                    (170 << CONF2_GAINREF_SHIFT) | CONF2_IQEN | (1 << CONF2_RSVD4_SHIFT);
                max2771_spi_write(band,CONF2,data);

                //write register 2
                data = CONF3_TIMESYNCEN | CONF3_STAMPEN | (CONF3_STRMBITS_MASK_1 << CONF3_STRMBITS_SHIFT) | CONF3_RSVD1_MASK | CONF3_PGAQEN |
                    CONF3_PGAIEN | CONF3_FHIPEN | CONF3_RSVD3 | CONF3_RSVD4 | CONF3_RSVD5 | CONF3_RSVD6 | CONF3_HILOADEN | CONF3_RSVD7 | (0x3A << CONF3_GAININ_SHIFT);
                max2771_spi_write(band,CONF3,data);

                //write register 3
                data = (0x10 << PLL_CONF_RSVD8_SHIFT) | (PLL_CONF_IXTAL_MASK_1 << PLL_CONF_IXTAL_SHIFT) | PLL_CONF_RSVD10 | PLL_CONF_REFOUTEN | PLL_CONF_RSVD13;
				if (adc_freq == 16368000) data |= (PLL_CONF_REFDIV_MASK_3 << PLL_CONF_REFDIV_SHIFT);
                if (adc_freq == 32736000) data |= (PLL_CONF_REFDIV_MASK_0 << PLL_CONF_REFDIV_SHIFT);
                if (adc_freq == 65472000) data |= (PLL_CONF_REFDIV_MASK_4 << PLL_CONF_REFDIV_SHIFT);
                max2771_spi_write(band,PLL_CONF,data);

                //write register 4
                data = (1 << PLL_INT_RDIV_SHIFT) | (95 << PLL_INT_NDIV_SHIFT);
                max2771_spi_write(band,PLL_INT,data);

                //write register 5
                data = (7 << PLL_FRAC_RSVD5_SHIFT) | (0x0A6E9B << PLL_FRAC_FDIV_SHIFT);
                max2771_spi_write(band,PLL_FRAC,data);

                //write register 7
                data = (0x61B << CLK1_CONF_REFCLK_M_CNT_SHIFT) | (0x100 << CLK1_CONF_REFCLK_L_CNT_SHIFT) | CLK1_CONF_RSVD1;
                max2771_spi_write(band,CLK1_CONF,data);

                //write register 10
                data = (0x61B << CLK2_CONF_ADCCLK_M_CNT_SHIFT) | (0x100 << CLK2_CONF_ADCCLK_L_CNT_SHIFT);
                max2771_spi_write(band,CLK2_CONF,data);

            }
            else if (glonass)
            {

                //write register 0
                data = CONF1_FGAIN | CONF1_F2OR5 | (CONF1_FBW_MASK_36 << CONF1_FBW_SHIFT) | (24 << CONF1_FCEN_SHIFT) | CONF1_MIXPOLE |
                    (CONF1_RSVD1_VAL << CONF1_RSVD1_SHIFT) | (CONF1_RSVD2_VAL << CONF1_RSVD2_SHIFT) | (CONF1_RSVD3_VAL << CONF1_RSVD3_SHIFT) |
                    (CONF1_RSVD4_VAL << CONF1_RSVD4_SHIFT) | CONF1_CHIPEN;
                max2771_spi_write(band,CONF1,data);

                //write register 1
                data = CONF2_RSVD2 | (CONF2_BITS_MASK_2 << CONF2_BITS_SHIFT) | (CONF2_FORMAT_MASK_1 << CONF2_FORMAT_SHIFT) |
                    (170 << CONF2_GAINREF_SHIFT) | CONF2_IQEN | (1 << CONF2_RSVD4_SHIFT);
                max2771_spi_write(band,CONF2,data);

                //write register 2
                data = CONF3_TIMESYNCEN | CONF3_STAMPEN | (CONF3_STRMBITS_MASK_1 << CONF3_STRMBITS_SHIFT) | CONF3_RSVD1_MASK | CONF3_PGAQEN |
                    CONF3_PGAIEN | CONF3_FHIPEN | CONF3_RSVD3 | CONF3_RSVD4 | CONF3_RSVD5 | CONF3_RSVD6 | CONF3_HILOADEN | CONF3_RSVD7 | (0x3A << CONF3_GAININ_SHIFT);
                max2771_spi_write(band,CONF3,data);

                //write register 3
				data = (0x10 << PLL_CONF_RSVD8_SHIFT) | (PLL_CONF_IXTAL_MASK_1 << PLL_CONF_IXTAL_SHIFT) | PLL_CONF_RSVD10 | PLL_CONF_REFOUTEN | PLL_CONF_RSVD13;
				if (adc_freq == 16368000) data |= (PLL_CONF_REFDIV_MASK_3 << PLL_CONF_REFDIV_SHIFT);
                if (adc_freq == 32736000) data |= (PLL_CONF_REFDIV_MASK_0 << PLL_CONF_REFDIV_SHIFT);
                if (adc_freq == 65472000) data |= (PLL_CONF_REFDIV_MASK_4 << PLL_CONF_REFDIV_SHIFT);
                max2771_spi_write(band,PLL_CONF,data);

                //write register 4
                data = (1 << PLL_INT_RDIV_SHIFT) | (97 << PLL_INT_NDIV_SHIFT);
                max2771_spi_write(band,PLL_INT,data);

                //write register 5
                data = (7 << PLL_FRAC_RSVD5_SHIFT) | (0x0A6E9B << PLL_FRAC_FDIV_SHIFT);
                max2771_spi_write(band,PLL_FRAC,data);

                //write register 7
                data = (0x61B << CLK1_CONF_REFCLK_M_CNT_SHIFT) | (0x100 << CLK1_CONF_REFCLK_L_CNT_SHIFT) | CLK1_CONF_RSVD1;
                max2771_spi_write(band,CLK1_CONF,data);

                //write register 10
                data = (0x61B << CLK2_CONF_ADCCLK_M_CNT_SHIFT) | (0x100 << CLK2_CONF_ADCCLK_L_CNT_SHIFT);
                max2771_spi_write(band,CLK2_CONF,data);

            }
            else
            {

                //write register 0
                data = CONF1_FGAIN | CONF1_F2OR5 | (CONF1_FBW_MASK_36 << CONF1_FBW_SHIFT) | (24 << CONF1_FCEN_SHIFT) | CONF1_MIXPOLE |
                    (CONF1_RSVD1_VAL << CONF1_RSVD1_SHIFT) | (CONF1_RSVD2_VAL << CONF1_RSVD2_SHIFT) | (CONF1_RSVD3_VAL << CONF1_RSVD3_SHIFT) |
                    (CONF1_RSVD4_VAL << CONF1_RSVD4_SHIFT) | CONF1_CHIPEN;
                max2771_spi_write(band,CONF1,data);

                //write register 1
                data = CONF2_RSVD2 | (CONF2_BITS_MASK_2 << CONF2_BITS_SHIFT) | (CONF2_FORMAT_MASK_1 << CONF2_FORMAT_SHIFT) |
                    (170 << CONF2_GAINREF_SHIFT) | CONF2_IQEN | (1 << CONF2_RSVD4_SHIFT);
                max2771_spi_write(band,CONF2,data);

                //write register 2
                data = CONF3_TIMESYNCEN | CONF3_STAMPEN | (CONF3_STRMBITS_MASK_1 << CONF3_STRMBITS_SHIFT) | CONF3_RSVD1_MASK | CONF3_PGAQEN |
                    CONF3_PGAIEN | CONF3_FHIPEN | CONF3_RSVD3 | CONF3_RSVD4 | CONF3_RSVD5 | CONF3_RSVD6 | CONF3_HILOADEN | CONF3_RSVD7 | (0x3A << CONF3_GAININ_SHIFT);
                max2771_spi_write(band,CONF3,data);

                //write register 3
				data = (0x10 << PLL_CONF_RSVD8_SHIFT) | (PLL_CONF_IXTAL_MASK_1 << PLL_CONF_IXTAL_SHIFT) | PLL_CONF_RSVD10 | PLL_CONF_REFOUTEN | PLL_CONF_RSVD13;
				if (adc_freq == 16368000) data |= (PLL_CONF_REFDIV_MASK_3 << PLL_CONF_REFDIV_SHIFT);
                if (adc_freq == 32736000) data |= (PLL_CONF_REFDIV_MASK_0 << PLL_CONF_REFDIV_SHIFT);
                if (adc_freq == 65472000) data |= (PLL_CONF_REFDIV_MASK_4 << PLL_CONF_REFDIV_SHIFT);
                max2771_spi_write(band,PLL_CONF,data);

                //write register 4
                data = (1 << PLL_INT_RDIV_SHIFT) | (96 << PLL_INT_NDIV_SHIFT);
                max2771_spi_write(band,PLL_INT,data);

                //write register 5
                data = (7 << PLL_FRAC_RSVD5_SHIFT) | (0x0A6E9B << PLL_FRAC_FDIV_SHIFT);
                max2771_spi_write(band,PLL_FRAC,data);

                //write register 7
                data = (0x61B << CLK1_CONF_REFCLK_M_CNT_SHIFT) | (0x100 << CLK1_CONF_REFCLK_L_CNT_SHIFT) | CLK1_CONF_RSVD1;
                max2771_spi_write(band,CLK1_CONF,data);

                //write register 10
                data = (0x61B << CLK2_CONF_ADCCLK_M_CNT_SHIFT) | (0x100 << CLK2_CONF_ADCCLK_L_CNT_SHIFT);
                max2771_spi_write(band,CLK2_CONF,data);

            }
            break;
        case L5E5:

            //write register 0
            data = CONF1_FGAIN | CONF1_F2OR5 | (CONF1_FBW_MASK_164 << CONF1_FBW_SHIFT) | (88 << CONF1_FCEN_SHIFT) | (CONF1_MIXERMODE_MASK_LB << CONF1_MIXERMODE_SHIFT) |
                (CONF1_LNAMODE_MASK_LB << CONF1_LNAMODE_SHIFT) | (CONF1_RSVD1_VAL << CONF1_RSVD1_SHIFT) | (CONF1_RSVD2_VAL << CONF1_RSVD2_SHIFT) |
                (CONF1_RSVD3_VAL << CONF1_RSVD3_SHIFT) | (CONF1_RSVD4_VAL << CONF1_RSVD4_SHIFT) | CONF1_CHIPEN;
            max2771_spi_write(band,CONF1,data);

            //write register 1
            data = CONF2_RSVD2 | (CONF2_BITS_MASK_2 << CONF2_BITS_SHIFT) | (CONF2_FORMAT_MASK_1 << CONF2_FORMAT_SHIFT) |
                (170 << CONF2_GAINREF_SHIFT) | CONF2_IQEN | (1 << CONF2_RSVD4_SHIFT);
            max2771_spi_write(band,CONF2,data);

            //write register 2
            data = CONF3_TIMESYNCEN | CONF3_STAMPEN | (CONF3_STRMBITS_MASK_1 << CONF3_STRMBITS_SHIFT) | CONF3_RSVD1_MASK | CONF3_PGAQEN |
                CONF3_PGAIEN | CONF3_RSVD3 | CONF3_RSVD4 | CONF3_RSVD5 | CONF3_RSVD6 | CONF3_RSVD7 | (0x3A << CONF3_GAININ_SHIFT);
            max2771_spi_write(band,CONF3,data);

            //write register 3
            data = PLL_CONF_INT_PLL | (0x10 << PLL_CONF_RSVD8_SHIFT) | (PLL_CONF_IXTAL_MASK_1 << PLL_CONF_IXTAL_SHIFT) | PLL_CONF_RSVD10 |
                PLL_CONF_REFOUTEN | PLL_CONF_RSVD13 | PLL_CONF_LOBAND | (PLL_CONF_REFDIV_MASK_4 << PLL_CONF_REFDIV_SHIFT);
            max2771_spi_write(band,PLL_CONF,data);

            //write register 4
            data = (8 << PLL_INT_RDIV_SHIFT) | (575 << PLL_INT_NDIV_SHIFT);
            max2771_spi_write(band,PLL_INT,data);

            //write register 5
            data = (7 << PLL_FRAC_RSVD5_SHIFT) | (0x0F5655 << PLL_FRAC_FDIV_SHIFT);
            max2771_spi_write(band,PLL_FRAC,data);

            //write register 7
            data = CLK1_CONF_EXTADCCLK | (0x61B << CLK1_CONF_REFCLK_M_CNT_SHIFT) | (0x100 << CLK1_CONF_REFCLK_L_CNT_SHIFT) | CLK1_CONF_RSVD1;
            max2771_spi_write(band,CLK1_CONF,data);

            //write register 10
            data = (0x61B << CLK2_CONF_ADCCLK_M_CNT_SHIFT) | (0x100 << CLK2_CONF_ADCCLK_L_CNT_SHIFT);
            max2771_spi_write(band,CLK2_CONF,data);

            break;
        case L2E6:

            //write register 0
            data = CONF1_FGAIN | CONF1_F2OR5 | (CONF1_FBW_MASK_36 << CONF1_FBW_SHIFT) | (88 << CONF1_FCEN_SHIFT) | (CONF1_MIXERMODE_MASK_LB << CONF1_MIXERMODE_SHIFT) |
                (CONF1_LNAMODE_MASK_LB << CONF1_LNAMODE_SHIFT) | CONF1_MIXPOLE | (CONF1_RSVD1_VAL << CONF1_RSVD1_SHIFT) | (CONF1_RSVD2_VAL << CONF1_RSVD2_SHIFT) |
                (CONF1_RSVD3_VAL << CONF1_RSVD3_SHIFT) | (CONF1_RSVD4_VAL << CONF1_RSVD4_SHIFT) | CONF1_CHIPEN;
            max2771_spi_write(band,CONF1,data);

            //write register 1
            data = CONF2_RSVD2 | (CONF2_BITS_MASK_2 << CONF2_BITS_SHIFT) | (CONF2_FORMAT_MASK_1 << CONF2_FORMAT_SHIFT) |
                (170 << CONF2_GAINREF_SHIFT) | CONF2_IQEN | (1 << CONF2_RSVD4_SHIFT);
            max2771_spi_write(band,CONF2,data);

            //write register 2
            data = CONF3_TIMESYNCEN | CONF3_STAMPEN | (CONF3_STRMBITS_MASK_1 << CONF3_STRMBITS_SHIFT) | CONF3_RSVD1_MASK | CONF3_PGAQEN |
                CONF3_PGAIEN | CONF3_RSVD3 | CONF3_RSVD4 | CONF3_RSVD5 | CONF3_RSVD6 | CONF3_RSVD7 | (0x3A << CONF3_GAININ_SHIFT);
            max2771_spi_write(band,CONF3,data);

            //write register 3
            data = (0x10 << PLL_CONF_RSVD8_SHIFT) | (PLL_CONF_IXTAL_MASK_1 << PLL_CONF_IXTAL_SHIFT) | PLL_CONF_RSVD10 |
                PLL_CONF_REFOUTEN | PLL_CONF_RSVD13 | PLL_CONF_LOBAND | (PLL_CONF_REFDIV_MASK_4 << PLL_CONF_REFDIV_SHIFT);
            max2771_spi_write(band,PLL_CONF,data);

            //write register 4
            data = (1 << PLL_INT_RDIV_SHIFT) | (76 << PLL_INT_NDIV_SHIFT);
            max2771_spi_write(band,PLL_INT,data);

            //write register 5
            data = (7 << PLL_FRAC_RSVD5_SHIFT) | (0x090000 << PLL_FRAC_FDIV_SHIFT);
            max2771_spi_write(band,PLL_FRAC,data);

            //write register 7
            data = CLK1_CONF_EXTADCCLK | (0x61B << CLK1_CONF_REFCLK_M_CNT_SHIFT) | (0x100 << CLK1_CONF_REFCLK_L_CNT_SHIFT) | CLK1_CONF_RSVD1;
            max2771_spi_write(band,CLK1_CONF,data);

            //write register 10
            data = (0x61B << CLK2_CONF_ADCCLK_M_CNT_SHIFT) | (0x100 << CLK2_CONF_ADCCLK_L_CNT_SHIFT);
            max2771_spi_write(band,CLK2_CONF,data);

            break;
        default:
            max2771_gpio_unconf();
            max2771_spi_unconf();
            return 0;

    }
    max2771_gpio_unconf();
    max2771_spi_unconf();
    return 1;
}

