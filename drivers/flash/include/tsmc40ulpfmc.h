/*H*****************************************************************************
 *
 * Copyright (c) 2022 ChipCraft Sp. z o.o. All rights reserved
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
 * File Name : tsmc40ulpfmc.h
 * Author    : Maciej Plasota
 * ******************************************************************************
 * $Date: 2022-08-02 11:34:14 +0200 (wto, 02 sie 2022) $
 * $Revision: 876 $
 *H*****************************************************************************/

#ifndef _FLASH_H_
#define _FLASH_H_
#pragma once

#include <ccrv32-amba.h>
#include <ccrv32-amba-flash.h>
#include <ccrv32-csr.h>
#include <stdbool.h>

/*! \brief flash driver status returned by some of the functions.
 */
typedef enum
{
    READY = 0, BUSY, PROGRAMMING_ERROR, LOCK_ERROR, ARGUMENT_ERROR, ECC_CORR_ERROR, ECC_UNCORR_ERROR
} flash_access_status_t;

/*! \brief Helper function for calculating wait states count based on HCLK frequency.
 *
 * \param hclk_freq_hz The frequency of HCLK clock to be computed on.
 * \param delay_ns_x_10 The required time delay in 1/10ths of a ns (so 12.5 ns should be passed as 125).
 * \retval LOCK_ERROR operation should have been unlocked first.
 */
uint32_t flash_calc_clock_wait_states(uint32_t const hclk_freq_hz, uint32_t const delay_ns_x_10);

/*! \brief Configure READ wait states count.
 *
 * \param wait_states_count The number of READ wait states to be set.
 *
 * This function configures the number of wait states to be used when reading flash memory content.
 */
static inline void flash_set_read_wait_states(uint8_t const wait_states_count)
{
    AMBA_FLASH_PTR->CTRL = (AMBA_FLASH_PTR->CTRL & ~FLASH_CTRL_READ_WAIT_STATES) | ( (wait_states_count << FLASH_CTRL_READ_WAIT_STATES_OFFSET) & FLASH_CTRL_READ_WAIT_STATES);
}

/*! \brief Read READ wait states count.
 *
 * This function reads back the number of wait states to be used when reading flash memory content.
 *
 * \return Number of wait states during READ operation.
 */
static inline uint8_t flash_get_read_wait_states()
{
    return ( (AMBA_FLASH_PTR->CTRL & FLASH_CTRL_READ_WAIT_STATES) >> FLASH_CTRL_READ_WAIT_STATES_OFFSET);
}

/*! \brief Enable sequential prefetch.
 */
static inline void flash_enable_sequential_prefetch()
{
    AMBA_FLASH_PTR->CTRL |= FLASH_CTRL_SEQUENTIAL_PREFETCH;
}

/*! \brief Disable sequential prefetch.
 */
static inline void flash_disable_sequential_prefetch()
{
    AMBA_FLASH_PTR->CTRL &= ~FLASH_CTRL_SEQUENTIAL_PREFETCH;
}

/*! \brief Check if sequential prefetch is enabled.
 *
 * \return State of sequential prefetch configuration
 *  \retval true sequential prefetch is enabled
 *  \retval false sequential prefetch is disabled
 */
static inline bool flash_is_sequential_prefetch_enabled()
{
    return ( (AMBA_FLASH_PTR->CTRL & FLASH_CTRL_SEQUENTIAL_PREFETCH) != 0);
}

/*! \brief Enable treating all new reads as start of a sequential burst.
 */
static inline void flash_enable_start_all_reads_as_sequential()
{
    AMBA_FLASH_PTR->CTRL |= FLASH_CTRL_ASSUME_READS_SEQUENTIAL;
}

/*! \brief Disable treating all new reads as start of a sequential burst.
 */
static inline void flash_disable_start_all_reads_as_sequential()
{
    AMBA_FLASH_PTR->CTRL &= ~FLASH_CTRL_ASSUME_READS_SEQUENTIAL;
}

/*! \brief Check if treating all new reads as start of a sequential burst is enabled.
 *
 * \return State of treating all new reads as start of a sequential burst configuration
 *  \retval true treating all new reads as start of a sequential burst is enabled
 *  \retval false treating all new reads as start of a sequential burst is disabled
 */
static inline bool flash_is_start_all_reads_as_sequential_enabled()
{
    return ( (AMBA_FLASH_PTR->CTRL & FLASH_CTRL_ASSUME_READS_SEQUENTIAL) != 0);
}

/*! \brief Enable reprogram zeros on write functionality.
 */
static inline void flash_enable_reprogram_zeros_on_write()
{
    AMBA_FLASH_PTR->CTRL |= FLASH_CTRL_REPROGRAM_ZEROS_ON_WRITE;
}

/*! \brief Disable reprogram zeros on write functionality.
 */
static inline void flash_disable_reprogram_zeros_on_write()
{
    AMBA_FLASH_PTR->CTRL &= ~FLASH_CTRL_REPROGRAM_ZEROS_ON_WRITE;
}

/*! \brief Check if reprogram zeros on write functionality is enabled.
 *
 * \return State of reprogram zeros on write functionality configuration
 *  \retval true reprogram zeros on write functionality is enabled
 *  \retval false reprogram zeros on write functionality is disabled
 */
static inline bool flash_is_reprogram_zeros_on_write_enabled()
{
    return ( (AMBA_FLASH_PTR->CTRL & FLASH_CTRL_REPROGRAM_ZEROS_ON_WRITE) != 0);
}

/*! \brief Enable memory high endurance mode.
 */
static inline void flash_enable_high_endurance_mode()
{
    AMBA_FLASH_PTR->CTRL |= FLASH_CTRL_HIGH_ENDURANCE_MODE;
}

/*! \brief Disable memory high endurance mode.
 */
static inline void flash_disable_high_endurance_mode()
{
    AMBA_FLASH_PTR->CTRL &= ~FLASH_CTRL_HIGH_ENDURANCE_MODE;
}

/*! \brief Check if memory high endurance mode is enabled.
 *
 * \return State of memory high endurance mode configuration
 *  \retval true memory high endurance mode is enabled
 *  \retval false memory high endurance mode is disabled
 */
static inline bool flash_is_high_endurance_mode_enabled()
{
    return ( (AMBA_FLASH_PTR->CTRL & FLASH_CTRL_HIGH_ENDURANCE_MODE) != 0);
}

/*! \brief Enable memory low voltage mode.
 */
static inline void flash_enable_low_voltage_mode()
{
    AMBA_FLASH_PTR->CTRL |= FLASH_CTRL_LOW_VOLTAGE_MODE;
}

/*! \brief Disable memory low voltage mode.
 */
static inline void flash_disable_low_voltage_mode()
{
    AMBA_FLASH_PTR->CTRL &= ~FLASH_CTRL_LOW_VOLTAGE_MODE;
}

/*! \brief Check if memory low voltage mode is enabled.
 *
 * \return State of memory low voltage mode configuration
 *  \retval true memory low voltage mode is enabled
 *  \retval false memory low voltage mode is disabled
 */
static inline bool flash_is_low_voltage_mode_enabled()
{
    return ( (AMBA_FLASH_PTR->CTRL & FLASH_CTRL_LOW_VOLTAGE_MODE) != 0);
}

/*! \brief Enable ECC verification on read.
 */
static inline void flash_enable_ECC()
{
    AMBA_FLASH_PTR->CTRL |= FLASH_CTRL_ECC_ENABLE;
}

/*! \brief Disable ECC verification on read.
 */
static inline void flash_disable_ECC()
{
    AMBA_FLASH_PTR->CTRL &= ~FLASH_CTRL_ECC_ENABLE;
}

/*! \brief Check if ECC verification on read is enabled.
 *
 * \return State of ECC verification on read configuration
 *  \retval true ECC verification on read is enabled
 *  \retval false ECC verification on read is disabled
 */
static inline bool flash_is_ECC_enabled()
{
    return ( (AMBA_FLASH_PTR->CTRL & FLASH_CTRL_ECC_ENABLE) != 0);
}

/*! \brief Enable ECC reporting on AHB during data read.
 */
static inline void flash_enable_ECC_reporting_on_AHB()
{
    AMBA_FLASH_PTR->CTRL |= FLASH_CTRL_REPORT_ECC_ON_AHB_ENABLE;
}

/*! \brief Disable ECC reporting on AHB during data read.
 */
static inline void flash_disable_ECC_reporting_on_AHB()
{
    AMBA_FLASH_PTR->CTRL &= ~FLASH_CTRL_REPORT_ECC_ON_AHB_ENABLE;
}

/*! \brief Check if ECC reporting on AHB during data read is enabled.
 *
 * \return State of ECC reporting on AHB during data read configuration
 *  \retval true ECC reporting on AHB during data read is enabled
 *  \retval false ECC reporting on AHB during data read is disabled
 */
static inline bool flash_is_ECC_enabled_reporting_on_AHB()
{
    return ( (AMBA_FLASH_PTR->CTRL & FLASH_CTRL_REPORT_ECC_ON_AHB_ENABLE) != 0);
}

/*! \brief Configure SMWR prescaler.
 *
 * \param prescaler Enum for the prescaler value to be set.
 *
 * This function configures the prescaler for clock supplied to Smart Write Memory module.
 * Note that the clock frequency for this module can't exceed 254 MHz.
 */
static inline void flash_set_smwr_prescaler(amba_flash_SMWR_prescaler_t const prescaler)
{
    AMBA_FLASH_PTR->CTRL = (AMBA_FLASH_PTR->CTRL & ~FLASH_CTRL_SMWR_PRESCALER) | ( (prescaler << FLASH_CTRL_SMWR_PRESCALER_OFFSET) & FLASH_CTRL_SMWR_PRESCALER);
}

/*! \brief Read SMWR prescaler configuration.
 *
 * This function reads back the prescaler for clock supplied to Smart Write Memory module.
 *
 * \return Enum of the prescaler value set.
 */
static inline amba_flash_SMWR_prescaler_t flash_get_smwr_prescaler()
{
    return ( (AMBA_FLASH_PTR->CTRL & FLASH_CTRL_SMWR_PRESCALER) >> FLASH_CTRL_SMWR_PRESCALER_OFFSET);
}

/*! \brief Enable Sector Buffer self time bypass.
 */
static inline void flash_enable_self_time_bypass()
{
    AMBA_FLASH_PTR->CTRL |= FLASH_CTRL_SELF_TIME_BYPASS_ENABLE;
}

/*! \brief Disable Sector Buffer self time bypass.
 */
static inline void flash_disable_self_time_bypass()
{
    AMBA_FLASH_PTR->CTRL &= ~FLASH_CTRL_SELF_TIME_BYPASS_ENABLE;
}

/*! \brief Check if Sector Buffer self time bypass is enabled.
 *
 * \return State of Sector Buffer self time bypass configuration
 *  \retval true Sector Buffer self time bypass is enabled
 *  \retval false Sector Buffer self time bypass is disabled
 */
static inline bool flash_is_self_time_bypass_enabled()
{
    return ( (AMBA_FLASH_PTR->CTRL & FLASH_CTRL_SELF_TIME_BYPASS_ENABLE) != 0);
}

/*! \brief Configure Sector Buffer READ Margin.
 *
 * \param read_margin The value of read margin to be set.
 *
 */
static inline void flash_set_read_margin(uint8_t const read_margin)
{
    AMBA_FLASH_PTR->CTRL = (AMBA_FLASH_PTR->CTRL & ~FLASH_CTRL_READ_MARGIN) | ( (read_margin << FLASH_CTRL_READ_MARGIN_OFFSET) & FLASH_CTRL_READ_MARGIN);
    AMBA_FLASH_PTR->CTRL |= FLASH_CTRL_READ_MARGIN_ENABLE;
}

/*! \brief Reset Sector Buffer READ Margin.
 */
static inline void flash_reset_read_margin()
{
    AMBA_FLASH_PTR->CTRL &= ~FLASH_CTRL_READ_MARGIN_ENABLE;
}

/*! \brief Read Sector Buffer READ Margin.
 *
 * This function reads back the Sector Buffer READ Margin.
 *
 * \return Sector Buffer READ Margin.
 */
static inline uint8_t flash_get_read_margin()
{
    return ( (AMBA_FLASH_PTR->CTRL & FLASH_CTRL_READ_MARGIN) >> FLASH_CTRL_READ_MARGIN_OFFSET);
}

/*! \brief Configure MV in SMWR OPTION 0.
 *
 * \param value The value of MV to be set.
 *
 */
static inline void flash_set_MV(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_0 = (AMBA_FLASH_PTR->SMWR_OPTION_0 & ~FLASH_SMWR_OPTION_0_MV_MASK) | ( (value << FLASH_SMWR_OPTION_0_MV_OFFSET) & FLASH_SMWR_OPTION_0_MV_MASK);
}

/*! \brief Read MV configuration from SMWR OPTION 0.
 *
 * This function reads back the MV configuration from SMWR OPTION 0 register.
 *
 * \return MV value set.
 */
static inline uint8_t flash_get_MV()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_0 & FLASH_SMWR_OPTION_0_MV_MASK) >> FLASH_SMWR_OPTION_0_MV_OFFSET);
}

/*! \brief Configure MV FINAL in SMWR OPTION 0.
 *
 * \param value The value of MV FINAL to be set.
 *
 */
static inline void flash_set_MV_FINAL(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_0 = (AMBA_FLASH_PTR->SMWR_OPTION_0 & ~FLASH_SMWR_OPTION_0_MV_FINAL_MASK) | ( (value << FLASH_SMWR_OPTION_0_MV_FINAL_OFFSET) & FLASH_SMWR_OPTION_0_MV_FINAL_MASK);
}

/*! \brief Read MV FINAL configuration from SMWR OPTION 0.
 *
 * This function reads back the MV FINAL configuration from SMWR OPTION 0 register.
 *
 * \return MV FINAL value set.
 */
static inline uint8_t flash_get_MV_FINAL()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_0 & FLASH_SMWR_OPTION_0_MV_FINAL_MASK) >> FLASH_SMWR_OPTION_0_MV_FINAL_OFFSET);
}

/*! \brief Configure WIPGM in SMWR OPTION 0.
 *
 * \param value The value of WIPGM to be set.
 *
 */
static inline void flash_set_WIPGM(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_0 = (AMBA_FLASH_PTR->SMWR_OPTION_0 & ~FLASH_SMWR_OPTION_0_WIPGM_MASK) | ( (value << FLASH_SMWR_OPTION_0_WIPGM_OFFSET) & FLASH_SMWR_OPTION_0_WIPGM_MASK);
}

/*! \brief Read WIPGM configuration from SMWR OPTION 0.
 *
 * This function reads back the WIPGM configuration from SMWR OPTION 0 register.
 *
 * \return WIPGM value set.
 */
static inline uint8_t flash_get_WIPGM()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_0 & FLASH_SMWR_OPTION_0_WIPGM_MASK) >> FLASH_SMWR_OPTION_0_WIPGM_OFFSET);
}

/*! \brief Configure WIPGM FINAL in SMWR OPTION 0.
 *
 * \param value The value of WIPGM FINAL to be set.
 *
 */
static inline void flash_set_WIPGM_FINAL(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_0 = (AMBA_FLASH_PTR->SMWR_OPTION_0 & ~FLASH_SMWR_OPTION_0_WIPGM_FINAL_MASK) | ( (value << FLASH_SMWR_OPTION_0_WIPGM_FINAL_OFFSET) & FLASH_SMWR_OPTION_0_WIPGM_FINAL_MASK);
}

/*! \brief Read WIPGM FINAL configuration from SMWR OPTION 0.
 *
 * This function reads back the WIPGM FINAL configuration from SMWR OPTION 0 register.
 *
 * \return WIPGM FINAL value set.
 */
static inline uint8_t flash_get_WIPGM_FINAL()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_0 & FLASH_SMWR_OPTION_0_WIPGM_FINAL_MASK) >> FLASH_SMWR_OPTION_0_WIPGM_FINAL_OFFSET);
}

/*! \brief Configure TERS in SMWR OPTION 1.
 *
 * \param value The value of TERS to be set.
 *
 */
static inline void flash_set_TERS(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_1 = (AMBA_FLASH_PTR->SMWR_OPTION_1 & ~FLASH_SMWR_OPTION_1_TERS_MASK) | ( (value << FLASH_SMWR_OPTION_1_TERS_OFFSET) & FLASH_SMWR_OPTION_1_TERS_MASK);
}

/*! \brief Read TERS configuration from SMWR OPTION 1.
 *
 * This function reads back the TERS configuration from SMWR OPTION 1 register.
 *
 * \return TERS value set.
 */
static inline uint8_t flash_get_TERS()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_1 & FLASH_SMWR_OPTION_1_TERS_MASK) >> FLASH_SMWR_OPTION_1_TERS_OFFSET);
}

/*! \brief Configure TPGM in SMWR OPTION 1.
 *
 * \param value The value of TPGM to be set.
 *
 */
static inline void flash_set_TPGM(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_1 = (AMBA_FLASH_PTR->SMWR_OPTION_1 & ~FLASH_SMWR_OPTION_1_TPGM_MASK) | ( (value << FLASH_SMWR_OPTION_1_TPGM_OFFSET) & FLASH_SMWR_OPTION_1_TPGM_MASK);
}

/*! \brief Read TPGM configuration from SMWR OPTION 1.
 *
 * This function reads back the TPGM configuration from SMWR OPTION 1 register.
 *
 * \return TPGM value set.
 */
static inline uint8_t flash_get_TPGM()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_1 & FLASH_SMWR_OPTION_1_TPGM_MASK) >> FLASH_SMWR_OPTION_1_TPGM_OFFSET);
}

/*! \brief Configure TNVS in SMWR OPTION 1.
 *
 * \param value The value of TNVS to be set.
 *
 */
static inline void flash_set_TNVS(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_1 = (AMBA_FLASH_PTR->SMWR_OPTION_1 & ~FLASH_SMWR_OPTION_1_TNVS_MASK) | ( (value << FLASH_SMWR_OPTION_1_TNVS_OFFSET) & FLASH_SMWR_OPTION_1_TNVS_MASK);
}

/*! \brief Read TNVS configuration from SMWR OPTION 1.
 *
 * This function reads back the TNVS configuration from SMWR OPTION 1 register.
 *
 * \return TNVS value set.
 */
static inline uint8_t flash_get_TNVS()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_1 & FLASH_SMWR_OPTION_1_TNVS_MASK) >> FLASH_SMWR_OPTION_1_TNVS_OFFSET);
}

/*! \brief Configure TNVH in SMWR OPTION 1.
 *
 * \param value The value of TNVH to be set.
 *
 */
static inline void flash_set_TNVH(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_1 = (AMBA_FLASH_PTR->SMWR_OPTION_1 & ~FLASH_SMWR_OPTION_1_TNVH_MASK) | ( (value << FLASH_SMWR_OPTION_1_TNVH_OFFSET) & FLASH_SMWR_OPTION_1_TNVH_MASK);
}

/*! \brief Read TNVH configuration from SMWR OPTION 1.
 *
 * This function reads back the TNVH configuration from SMWR OPTION 1 register.
 *
 * \return TNVH value set.
 */
static inline uint8_t flash_get_TNVH()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_1 & FLASH_SMWR_OPTION_1_TNVH_MASK) >> FLASH_SMWR_OPTION_1_TNVH_OFFSET);
}

/*! \brief Configure TPGS in SMWR OPTION 1.
 *
 * \param value The value of TPGS to be set.
 *
 */
static inline void flash_set_TPGS(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_1 = (AMBA_FLASH_PTR->SMWR_OPTION_1 & ~FLASH_SMWR_OPTION_1_TPGS_MASK) | ( (value << FLASH_SMWR_OPTION_1_TPGS_OFFSET) & FLASH_SMWR_OPTION_1_TPGS_MASK);
}

/*! \brief Read TPGS configuration from SMWR OPTION 1.
 *
 * This function reads back the TPGS configuration from SMWR OPTION 1 register.
 *
 * \return TPGS value set.
 */
static inline uint8_t flash_get_TPGS()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_1 & FLASH_SMWR_OPTION_1_TPGS_MASK) >> FLASH_SMWR_OPTION_1_TPGS_OFFSET);
}

/*! \brief Configure MAX ERASE in SMWR OPTION 1.
 *
 * \param value The value of MAX ERASE to be set.
 *
 */
static inline void flash_set_MAX_ERASE(uint16_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_1 = (AMBA_FLASH_PTR->SMWR_OPTION_1 & ~FLASH_SMWR_OPTION_1_MAX_ERASE_MASK) | ( (value << FLASH_SMWR_OPTION_1_MAX_ERASE_OFFSET) & FLASH_SMWR_OPTION_1_MAX_ERASE_MASK);
}

/*! \brief Read MAX ERASE configuration from SMWR OPTION 1.
 *
 * This function reads back the MAX ERASE configuration from SMWR OPTION 1 register.
 *
 * \return MAX ERASE value set.
 */
static inline uint16_t flash_get_MAX_ERASE()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_1 & FLASH_SMWR_OPTION_1_MAX_ERASE_MASK) >> FLASH_SMWR_OPTION_1_MAX_ERASE_OFFSET);
}

/*! \brief Configure MAX PROG in SMWR OPTION 1.
 *
 * \param value The value of MAX PROG to be set.
 *
 */
static inline void flash_set_MAX_PROG(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_1 = (AMBA_FLASH_PTR->SMWR_OPTION_1 & ~FLASH_SMWR_OPTION_1_MAX_PROG_MASK) | ( (value << FLASH_SMWR_OPTION_1_MAX_PROG_OFFSET) & FLASH_SMWR_OPTION_1_MAX_PROG_MASK);
}

/*! \brief Read MAX PROG configuration from SMWR OPTION 1.
 *
 * This function reads back the MAX PROG configuration from SMWR OPTION 1 register.
 *
 * \return MAX PROG value set.
 */
static inline uint8_t flash_get_MAX_PROG()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_1 & FLASH_SMWR_OPTION_1_MAX_PROG_MASK) >> FLASH_SMWR_OPTION_1_MAX_PROG_OFFSET);
}

/*! \brief Configure THVS in SMWR OPTION 2.
 *
 * \param value The value of THVS to be set.
 *
 */
static inline void flash_set_THVS(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_2 = (AMBA_FLASH_PTR->SMWR_OPTION_2 & ~FLASH_SMWR_OPTION_2_THVS_MASK) | ( (value << FLASH_SMWR_OPTION_2_THVS_OFFSET) & FLASH_SMWR_OPTION_2_THVS_MASK);
}

/*! \brief Read THVS configuration from SMWR OPTION 2.
 *
 * This function reads back the THVS configuration from SMWR OPTION 2 register.
 *
 * \return THVS value set.
 */
static inline uint8_t flash_get_THVS()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_2 & FLASH_SMWR_OPTION_2_THVS_MASK) >> FLASH_SMWR_OPTION_2_THVS_OFFSET);
}

/*! \brief Configure TRCV in SMWR OPTION 2.
 *
 * \param value The value of TRCV to be set.
 *
 */
static inline void flash_set_TRCV(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_2 = (AMBA_FLASH_PTR->SMWR_OPTION_2 & ~FLASH_SMWR_OPTION_2_TRCV_MASK) | ( (value << FLASH_SMWR_OPTION_2_TRCV_OFFSET) & FLASH_SMWR_OPTION_2_TRCV_MASK);
}

/*! \brief Read TRCV configuration from SMWR OPTION 2.
 *
 * This function reads back the TRCV configuration from SMWR OPTION 2 register.
 *
 * \return TRCV value set.
 */
static inline uint8_t flash_get_TRCV()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_2 & FLASH_SMWR_OPTION_2_TRCV_MASK) >> FLASH_SMWR_OPTION_2_TRCV_OFFSET);
}

/*! \brief Configure EPP in SMWR OPTION 2.
 *
 * \param value The value of EPP to be set.
 *
 */
static inline void flash_set_EPP(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_2 = (AMBA_FLASH_PTR->SMWR_OPTION_2 & ~FLASH_SMWR_OPTION_2_EPP_MASK) | ( (value << FLASH_SMWR_OPTION_2_EPP_OFFSET) & FLASH_SMWR_OPTION_2_EPP_MASK);
}

/*! \brief Read EPP configuration from SMWR OPTION 2.
 *
 * This function reads back the EPP configuration from SMWR OPTION 2 register.
 *
 * \return EPP value set.
 */
static inline uint8_t flash_get_EPP()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_2 & FLASH_SMWR_OPTION_2_EPP_MASK) >> FLASH_SMWR_OPTION_2_EPP_OFFSET);
}

/*! \brief Configure EPE in SMWR OPTION 2.
 *
 * \param value The value of EPE to be set.
 *
 */
static inline void flash_set_EPE(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_2 = (AMBA_FLASH_PTR->SMWR_OPTION_2 & ~FLASH_SMWR_OPTION_2_EPE_MASK) | ( (value << FLASH_SMWR_OPTION_2_EPE_OFFSET) & FLASH_SMWR_OPTION_2_EPE_MASK);
}

/*! \brief Read EPE configuration from SMWR OPTION 2.
 *
 * This function reads back the EPE configuration from SMWR OPTION 2 register.
 *
 * \return EPE value set.
 */
static inline uint8_t flash_get_EPE()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_2 & FLASH_SMWR_OPTION_2_EPE_MASK) >> FLASH_SMWR_OPTION_2_EPE_OFFSET);
}

/*! \brief Configure WHV in SMWR OPTION 2.
 *
 * \param value The value of WHV to be set.
 *
 */
static inline void flash_set_WHV(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_2 = (AMBA_FLASH_PTR->SMWR_OPTION_2 & ~FLASH_SMWR_OPTION_2_WHV_MASK) | ( (value << FLASH_SMWR_OPTION_2_WHV_OFFSET) & FLASH_SMWR_OPTION_2_WHV_MASK);
}

/*! \brief Read WHV configuration from SMWR OPTION 2.
 *
 * This function reads back the WHV configuration from SMWR OPTION 2 register.
 *
 * \return WHV value set.
 */
static inline uint8_t flash_get_WHV()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_2 & FLASH_SMWR_OPTION_2_WHV_MASK) >> FLASH_SMWR_OPTION_2_WHV_OFFSET);
}

/*! \brief Configure POST TERS in SMWR OPTION 2.
 *
 * \param value The value of POST TERS to be set.
 *
 */
static inline void flash_set_POST_TERS(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_2 = (AMBA_FLASH_PTR->SMWR_OPTION_2 & ~FLASH_SMWR_OPTION_2_POST_TERS_MASK) | ( (value << FLASH_SMWR_OPTION_2_POST_TERS_OFFSET) & FLASH_SMWR_OPTION_2_POST_TERS_MASK);
}

/*! \brief Read POST TERS configuration from SMWR OPTION 2.
 *
 * This function reads back the POST TERS configuration from SMWR OPTION 2 register.
 *
 * \return POST TERS value set.
 */
static inline uint8_t flash_get_POST_TERS()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_2 & FLASH_SMWR_OPTION_2_POST_TERS_MASK) >> FLASH_SMWR_OPTION_2_POST_TERS_OFFSET);
}

/*! \brief Configure POST TPGM in SMWR OPTION 2.
 *
 * \param value The value of POST TPGM to be set.
 *
 */
static inline void flash_set_POST_TPGM(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_2 = (AMBA_FLASH_PTR->SMWR_OPTION_2 & ~FLASH_SMWR_OPTION_2_POST_TPGM_MASK) | ( (value << FLASH_SMWR_OPTION_2_POST_TPGM_OFFSET) & FLASH_SMWR_OPTION_2_POST_TPGM_MASK);
}

/*! \brief Read POST TPGM configuration from SMWR OPTION 2.
 *
 * This function reads back the POST TPGM configuration from SMWR OPTION 2 register.
 *
 * \return POST TPGM value set.
 */
static inline uint8_t flash_get_POST_TPGM()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_2 & FLASH_SMWR_OPTION_2_POST_TPGM_MASK) >> FLASH_SMWR_OPTION_2_POST_TPGM_OFFSET);
}

/*! \brief Configure VERIFY in SMWR OPTION 2.
 *
 * \param value The value of VERIFY to be set.
 *
 */
static inline void flash_set_VERIFY(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_2 = (AMBA_FLASH_PTR->SMWR_OPTION_2 & ~FLASH_SMWR_OPTION_2_VERIFY_MASK) | ( (value << FLASH_SMWR_OPTION_2_VERIFY_OFFSET) & FLASH_SMWR_OPTION_2_VERIFY_MASK);
}

/*! \brief Read VERIFY configuration from SMWR OPTION 2.
 *
 * This function reads back the VERIFY configuration from SMWR OPTION 2 register.
 *
 * \return VERIFY value set.
 */
static inline uint8_t flash_get_VERIFY()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_2 & FLASH_SMWR_OPTION_2_VERIFY_MASK) >> FLASH_SMWR_OPTION_2_VERIFY_OFFSET);
}

/*! \brief Configure TPGM OPTION in SMWR OPTION 2.
 *
 * \param value The value of TPGM OPTION to be set.
 *
 */
static inline void flash_set_TPGM_OPTION(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_2 = (AMBA_FLASH_PTR->SMWR_OPTION_2 & ~FLASH_SMWR_OPTION_2_TPGM_OPTION_MASK) | ( (value << FLASH_SMWR_OPTION_2_TPGM_OPTION_OFFSET) & FLASH_SMWR_OPTION_2_TPGM_OPTION_MASK);
}

/*! \brief Read TPGM OPTION configuration from SMWR OPTION 2.
 *
 * This function reads back the TPGM OPTION configuration from SMWR OPTION 2 register.
 *
 * \return TPGM OPTION value set.
 */
static inline uint8_t flash_get_TPGM_OPTION()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_2 & FLASH_SMWR_OPTION_2_TPGM_OPTION_MASK) >> FLASH_SMWR_OPTION_2_TPGM_OPTION_OFFSET);
}

/*! \brief Configure MASK0 in SMWR OPTION 2.
 *
 * \param value The value of MASK0 to be set.
 *
 */
static inline void flash_set_MASK0(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_2 = (AMBA_FLASH_PTR->SMWR_OPTION_2 & ~FLASH_SMWR_OPTION_2_MASK0_MASK) | ( (value << FLASH_SMWR_OPTION_2_MASK0_OFFSET) & FLASH_SMWR_OPTION_2_MASK0_MASK);
}

/*! \brief Read MASK0 configuration from SMWR OPTION 2.
 *
 * This function reads back the MASK0 configuration from SMWR OPTION 2 register.
 *
 * \return MASK0 value set.
 */
static inline uint8_t flash_get_MASK0()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_2 & FLASH_SMWR_OPTION_2_MASK0_MASK) >> FLASH_SMWR_OPTION_2_MASK0_OFFSET);
}

/*! \brief Configure DISABLE PRE READ in SMWR OPTION 2.
 *
 * \param value The value of DISABLE PRE READ to be set.
 *
 */
static inline void flash_set_DISABLE_PRE_READ(uint8_t value)
{
    AMBA_FLASH_PTR->SMWR_OPTION_2 = (AMBA_FLASH_PTR->SMWR_OPTION_2 & ~FLASH_SMWR_OPTION_2_DISABLE_PRE_READ_MASK) | ( (value << FLASH_SMWR_OPTION_2_DISABLE_PRE_READ_OFFSET) & FLASH_SMWR_OPTION_2_DISABLE_PRE_READ_MASK);
}

/*! \brief Read DISABLE PRE READ configuration from SMWR OPTION 2.
 *
 * This function reads back the DISABLE PRE READ configuration from SMWR OPTION 2 register.
 *
 * \return DISABLE PRE READ value set.
 */
static inline uint8_t flash_get_DISABLE_PRE_READ()
{
    return ( (AMBA_FLASH_PTR->SMWR_OPTION_2 & FLASH_SMWR_OPTION_2_DISABLE_PRE_READ_MASK) >> FLASH_SMWR_OPTION_2_DISABLE_PRE_READ_OFFSET);
}

/*! \brief Configure HEM WHV COUNTER in SMWR OPTION 3.
*
* \param value The value of HEM WHV COUNTER to be set.
*
*/
static inline void flash_set_HEM_WHV_COUNTER(uint8_t value)
{
   AMBA_FLASH_PTR->SMWR_OPTION_3 = (AMBA_FLASH_PTR->SMWR_OPTION_3 & ~FLASH_SMWR_OPTION_3_HEM_WHV_COUNTER_MASK) | ( (value << FLASH_SMWR_OPTION_3_HEM_WHV_COUNTER_OFFSET) & FLASH_SMWR_OPTION_3_HEM_WHV_COUNTER_MASK);
}

/*! \brief Read HEM WHV COUNTER configuration from SMWR OPTION 3.
*
* This function reads back the HEM WHV COUNTER configuration from SMWR OPTION 3 register.
*
* \return WHV COUNTER value set.
*/
static inline uint8_t flash_get_HEM_WHV_COUNTER()
{
   return ( (AMBA_FLASH_PTR->SMWR_OPTION_3 & FLASH_SMWR_OPTION_3_HEM_WHV_COUNTER_MASK) >> FLASH_SMWR_OPTION_3_HEM_WHV_COUNTER_OFFSET);
}

/*! \brief Configure HEM MAX ERASE in SMWR OPTION 3.
*
* \param value The value of HEM MAX ERASE to be set.
*
*/
static inline void flash_set_HEM_MAX_ERASE(uint16_t value)
{
   AMBA_FLASH_PTR->SMWR_OPTION_3 = (AMBA_FLASH_PTR->SMWR_OPTION_3 & ~FLASH_SMWR_OPTION_3_HEM_MAX_ERASE_MASK) | ( (value << FLASH_SMWR_OPTION_3_HEM_MAX_ERASE_OFFSET) & FLASH_SMWR_OPTION_3_HEM_MAX_ERASE_MASK);
}

/*! \brief Read HEM MAX ERASE configuration from SMWR OPTION 3.
*
* This function reads back the HEM MAX ERASE configuration from SMWR OPTION 3 register.
*
* \return HEM MAX ERASE value set.
*/
static inline uint16_t flash_get_HEM_MAX_ERASE()
{
   return ( (AMBA_FLASH_PTR->SMWR_OPTION_3 & FLASH_SMWR_OPTION_3_HEM_MAX_ERASE_MASK) >> FLASH_SMWR_OPTION_3_HEM_MAX_ERASE_OFFSET);
}

/*! \brief Configure Smart Program WHV.
 *
 * \param value The value of SMP WHV to be set.
 *
 */
static inline void flash_set_SMP_WHV_configuration(uint64_t value)
{
    AMBA_FLASH_PTR->SMWR_SMP_WHV_1 = (uint32_t)value;
    AMBA_FLASH_PTR->SMWR_SMP_WHV_2 = (uint32_t)(value >> 32);
}

/*! \brief Read Smart Program WHV value.
 *
 * This function reads back the value of SMP WHV.
 *
 * \return The value of SMP WHV.
 */
static inline uint64_t flash_get_SMP_WHV_configuration()
{
    return AMBA_FLASH_PTR->SMWR_SMP_WHV_1 + (((uint64_t)AMBA_FLASH_PTR->SMWR_SMP_WHV_2) << 32);
}

/*! \brief Configure Smart Erase WHV.
 *
 * \param value The value of SME WHV to be set.
 *
 */
static inline void flash_set_SME_WHV_configuration(uint64_t value)
{
    AMBA_FLASH_PTR->SMWR_SME_WHV_1 = (uint32_t)value;
    AMBA_FLASH_PTR->SMWR_SME_WHV_2 = (uint32_t)(value >> 32);
}

/*! \brief Read Smart Erase WHV value.
 *
 * This function reads back the value of SME WHV.
 *
 * \return The value of SME WHV.
 */
static inline uint64_t flash_get_SME_WHV_configuration()
{
    return AMBA_FLASH_PTR->SMWR_SME_WHV_1 + (((uint64_t)AMBA_FLASH_PTR->SMWR_SME_WHV_2) << 32);
}

/*! \brief Sets the configuration of the Flash controller.
 *
 * \param hclk_freq_hz The frequency of HCLK clock to be configured.
 * \param wait_states_count The number of wait states for read operation.
 * \param sequential_prefetch_enable Enables sequential prefetch feature.
 * \param start_all_reads_as_sequential_enable Enables treating all new reads as start of a sequential burst feature.
 */
void flash_configure(uint32_t hclk_freq_hz,
                     uint8_t wait_states_count,
                     bool sequential_prefetch_enable,
                     bool start_all_reads_as_sequential_enable);

/*! \brief Unlock access to COMMAND register.
 *
 * This function will unlock access to commands modifying content of program memory and their fuse bits.
 */
static inline void flash_unlock_command()
{
    AMBA_FLASH_PTR->LOCK = FLASH_LOCK_ACCESS_PASSWORD;
}

/*! \brief Unlock access to COMMAND register.
 *
 * This function will unlock access to commands modifying content of user page and their fuse bits.
 */
static inline void flash_unlock_user_row_command()
{
    AMBA_FLASH_PTR->LOCK = FLASH_LOCK_USER_ROW_ACCESS_PASSWORD;
}

/*! \brief Unlock access to COMMAND register.
 *
 * This function will unlock access to commands modifying content of factory row and their fuse bits.
 */
static inline void flash_unlock_factory_row_command()
{
    AMBA_FLASH_PTR->LOCK = FLASH_LOCK_FACTORY_ROW_ACCESS_PASSWORD;
}

/*! \brief Unlock access to COMMAND register.
 *
 * This function will unlock access to commands modifying content of manufacturer row and their fuse bits.
 */
static inline void flash_unlock_manufacturer_row_command()
{
    AMBA_FLASH_PTR->LOCK = FLASH_LOCK_MANUFACTURER_ROW_ACCESS_PASSWORD;
}

/*! \brief Unlock access to COMMAND register.
 *
 * This function will unlock access to commands modifying content of IFREN pages and their fuse bits.
 */
static inline void flash_unlock_ifren_memory_command()
{
    AMBA_FLASH_PTR->LOCK = FLASH_LOCK_IFREN_ACCESS_PASSWORD;
}

/*! \brief Unlock access to COMMAND register.
 *
 * This function will unlock access to commands modifying content of IFREN1 pages and their fuse bits.
 */
static inline void flash_unlock_ifren1_memory_command()
{
    AMBA_FLASH_PTR->LOCK = FLASH_LOCK_IFREN1_ACCESS_PASSWORD;
}

/*! \brief Unlock access to COMMAND register.
 *
 * This function will unlock access to commands modifying REDEN configuration and their fuse bits.
 */
static inline void flash_unlock_reden_command()
{
    AMBA_FLASH_PTR->LOCK = FLASH_LOCK_REDEN_CONFIG_ACCESS_PASSWORD;
}

/*! \brief Unlock access to COMMAND register.
 *
 * This function will unlock access to Erase Reference Cell command and its fuse bit.
 */
static inline void flash_unlock_erase_reference_cell_command()
{
    AMBA_FLASH_PTR->LOCK = FLASH_LOCK_ERASE_REFERENCE_CELL_ACCESS_PASSWORD;
}

/*! \brief Issue command to the controller (via COMMAND register).
 */
static inline void flash_issue_command(amba_flash_command_t command)
{
    AMBA_FLASH_PTR->COMMAND = command;
}

/*! \brief Sets the TIMING configuration register based on the frequency of HCLK clock.
 *
 * \param hclk_freq_hz The frequency of HCLK clock to be configured.
 *
 */
void flash_configure_timings(uint32_t hclk_freq_hz);

/*! \brief Clears (set to 0xFF) the content of Sector Buffer.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_clear_sector_buffer();

/*! \brief Writes quad-word of data into the sector buffer.
 *
 * \param sector_buffer_start_word_offset The first word offset of the data within the sector buffer (needs to be aligned to 4 x 32-bit words).
 * \param data_qword The data quad-word to be written into sector buffer.
 *
 * This function sets up all registers and starts the write operation but doesn't wait for it to complete.
 * User is expected to monitor status until the write process is completed.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_qword_to_sector_buffer(uint16_t sector_buffer_start_word_offset, const uint32_t data_qword[4]);

/*! \brief Writes single word of data into the sector buffer.
 *
 * \param sector_buffer_word_offset The word offset of the data within the sector buffer.
 * \param data_word The data word to be written into sector buffer.
 *
 * This function uses direct access to the Sector Buffer, meaning it won't return until the write actually completed.
 * Status returned shall either be READY or one of the error codes.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_word_to_sector_buffer_blocking(uint16_t sector_buffer_word_offset, uint32_t data_word);

/*! \brief Reads single word of data from the sector buffer.
 *
 * \param sector_buffer_word_offset The word offset of the data within the sector buffer.
 * \param data_word The data word to read from sector buffer.
 *
 * This function reads single data word from the Sector Buffer. Direct access scheme is used.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_read_sector_buffer_word(uint8_t sector_buffer_word_offset, uint32_t* data_word);

/*! \brief Unlocks ability to write data into sector buffer by directly addressing FLASH memory address range.
 */
static inline void flash_unlock_write_sector_data(void)
{
  //  CSR_CTRL_PTR->ROM_UNLOCK = CSR_ROM_UNLOCK_KEY | CSR_ROM_UNLOCK;
}

/*! \brief Writes single sector content to program memory.
 *
 * \param destination_address The start address of memory to be written (should be aligned to 4 x 32-bit words).
 * \param data_buffer The pointer to memory buffer containing data to be written.
 * \param num_words The number of words to be written (needs to be a multiple of 4)).
 *
 * This function copies provided data to sector buffer and initiates write operation.
 * This function will not wait for the write to finish.
 * Input buffer can contain partial sector data, but it may not cross the sector boundary (256 words).
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data could not be written to flash memory due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_sector(uint32_t* destination_address, uint32_t const* data_buffer, uint16_t num_words);

/*! \brief Writes single sector content to program memory.
 *
 * \param destination_address The start address of memory to be written (should be aligned to 4 x 32-bit words).
 * \param data_buffer The pointer to memory buffer containing data to be written.
 * \param num_words The number of words to be written (needs to be a multiple of 4)).
 *
 * This function copies provided data to sector buffer through APB interface and initiates write operation.
 * This function will not wait for the write to finish.
 * Input buffer can contain partial sector data, but it may not cross the sector boundary (256 words).
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data could not be written to flash memory due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_sector_slow(uint32_t* destination_address, uint32_t const* data_buffer, uint16_t num_words);

/*! \brief Writes data quad-word to program memory.
 *
 * \param destination_address The start address of memory to be written (needs to be aligned to 4 x 32-bit words).
 * \param data_qword Data quad-word to be written.
 *
 * This function copies provided data words to sector buffer and initiates write operation (previous sector buffer content is lost).
 * This function will not wait for the write to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data could not be written to flash memory due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_qword(uint32_t* destination_address, const uint32_t data_qword[4]);

/*! \brief Writes data quad-word to program memory.
 *
 * \param destination_address The start address of memory to be written (needs to be aligned to 4 x 32-bit words).
 * \param data_qword Data quad-word to be written.
 *
 * This function writes provided data words to program memory. It doesn't use sector buffer to do this, so it's content will be untouched.
 * This function will not wait for the write operation to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data could not be written to flash memory due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_qword_immediate(uint32_t* destination_address, const uint32_t data_qword[4]);

/*! \brief Erases single data page from program memory.
 *
 * \param destination_address Address within the memory page to be erased.
 *
 * This function initializes erasure of single data page from program memory.
 * It will not wait for the erase operation to complete.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_erase_page(uint32_t* destination_address);

/*! \brief Erases whole program memory.
 *
 * This function initializes erasure of whole program memory (data + region lock bits + debugger lock bits).
 * It will not wait for the erase operation to complete.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_erase_all();

/*! \brief Erases reference cells of memory macros.
 *
 * This function initializes erasure of memory macros reference cells.
 * It will not wait for the erase operation to complete.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_erase_reference_cell();

/*! \brief Locks region of program memory for write and erase.
 *
 * \param address_within_region_to_lock Address within the memory region to be locked.
 *
 * This function initializes procedure for locking single memory region. It will not wait for the operation to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_lock_region(const uint32_t* address_within_region_to_lock);

/*! \brief Locks ability to read memory content via debugger.
 *
 * This function initializes procedure for locking debugger read access to memory.
 * It will not wait for the operation to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_lock_debugger_read();

/*! \brief Locks ability to access processor through debugger.
 *
 * This function initializes procedure for locking debugger access.
 * It will not wait for the operation to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_lock_debugger_access();

/*! \brief Checks if region containing provided address is locked for write/erase access.
 *
 * \param address_within_region_to_check_lock Address within the region to check for.
 *
 * \return State of the region to check for.
 *   \retval true region is locked for write/erase
 *   \retval false region is open for write/erase
 */
bool flash_is_region_locked(const uint32_t* address_within_region_to_check_lock);

/*! \brief Check if read flash memory through debugger is enabled.
 *
 * \return State of debugger read access
 *  \retval true debugger read is disabled
 *  \retval false debugger read is enabled
 */
static inline bool flash_is_debugger_read_locked()
{
    return ( (AMBA_FLASH_PTR->MASTER_LOCKS & FLASH_MASTER_LOCKS_DEBUGGER_READ_DISABLED) != 0);
}

/*! \brief Check if access through debugger is enabled.
 *
 * \return State of debugger access to the processor
 *  \retval true debugger access is disabled
 *  \retval false debugger access is enabled
 */
static inline bool flash_is_debugger_access_locked()
{
    return ( (AMBA_FLASH_PTR->MASTER_LOCKS & FLASH_MASTER_LOCKS_DEBUGGER_ACCESS_DISABLED) != 0);
}

/*! \brief Writes quad-word of data into the factory row.
 *
 * \param word_offset_in_row The word offset of the data within the factory row (needs to be aligned to 4 x 32-bit words).
 * \param data_qword The data quad-word to be written into the factory row.
 *
 * This function sets up all registers and starts the write operation but doesn't wait for it to complete.
 * User is expected to monitor status to learn when the write process is completed.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_factory_row_qword(uint16_t word_offset_in_row, const uint32_t data_qword[4]);

/*! \brief Writes quad-word of data into the factory row.
 *
 * \param word_offset_in_row The word offset of the data within the factory row (needs to be aligned to 4 x 32-bit words).
 * \param data_qword The data quad-word to be written into the factory row.
 *
 * This function won't exit until the write is completed.
 * Status returned shall either be READY or one of the error codes.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_factory_row_qword_blocking(uint16_t const word_offset_in_row, const uint32_t data_qword[4]);

/*! \brief Erases content of factory row.
 *
 * This function initializes erasure of the data contained in factory row.
 * It will not wait for the erase operation to complete.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_erase_factory_row();

/*! \brief Reads quad-word of data from the factory row.
 *
 * \param word_offset_in_row The word offset of the data within the factory row (needs to be aligned to 4 x 32-bit words).
 * \param data_qword Pointer to a buffer where read data will be placed.
 *
 * This function reads data quad-word from the factory row.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_read_factory_row_qword(uint16_t word_offset_in_row, uint32_t data_qword[4]);

/*! \brief Reads single word of data from the factory row.
 *
 * \param word_offset_in_row The word offset of the data within the factory row.
 * \param data_word Pointer to a buffer where read data will be placed.
 *
 * This function reads data word from the factory row.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_read_factory_row_word(uint16_t word_offset_in_row, uint32_t* data_word);

/*! \brief Locks ability to write, read and erase data within factory row.
 *
 * This function initializes procedure for locking factory row write/read/erase access to memory.
 * It will not wait for the operation to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_lock_factory_row();

/*! \brief Check if read/write/erase access to factory row is allowed.
 *
 * \return State of factory row access
 *  \retval true factory row is locked
 *  \retval false factory row is accessible
 */
static inline bool flash_is_factory_row_locked()
{
    return ( (AMBA_FLASH_PTR->MASTER_LOCKS & FLASH_MASTER_LOCKS_FACTORY_ROW_LOCKED) != 0);
}

/*! \brief Writes quad-word of data into the user page.
 *
 * \param word_offset_in_row The word offset of the data within the user row (needs to be aligned to 4 x 32-bit words).
 * \param data_qword The data quad-word to be written into the user row.
 *
 * This function sets up all registers and starts the write operation but doesn't wait for it to complete.
 * User is expected to monitor status to learn when the write process is completed.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_user_row_qword(uint16_t word_offset_in_row, const uint32_t data_qword[4]);

/*! \brief Writes quad-word of data into the user page.
 *
 * \param word_offset_in_row The word offset of the data within the user row (needs to be aligned to 4 x 32-bit words).
 * \param data_qword The data quad-word to be written into the user row.
 *
 * This function won't exit until the write is completed.
 * Status returned shall either be READY or one of the error codes.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_user_row_qword_blocking(uint16_t word_offset_in_row, const uint32_t data_qword[4]);

/*! \brief Reads quad-word of data from the user page.
 *
 * \param word_offset_in_row The word offset of the data within the user page (needs to be aligned to 4 x 32-bit words).
 * \param data_qword Pointer to a buffer where read data will be placed.
 *
 * This function reads data quad-word from the user page.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_read_user_row_qword(uint16_t const word_offset_in_row, uint32_t data_qword[4]);

/*! \brief Reads single word of data from the user page.
 *
 * \param word_offset_in_row The word offset of the data within the user page.
 * \param data_word Pointer to a buffer where read data will be placed.
 *
 * This function reads data word from the user page.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_read_user_row_word(uint16_t word_offset_in_row, uint32_t* data_word);

/*! \brief Erases content of user page.
 *
 * This function initializes erasure of the data contained in user page.
 * It will not wait for the erase operation to complete.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_erase_user_row();

/*! \brief Locks ability to write and erase data within user page.
 *
 * This function initializes procedure for locking user row write/erase access to memory.
 * It will not wait for the operation to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_lock_user_row();

/*! \brief Check if write/erase access to user page is allowed.
 *
 * \return Status of user page accessibility
 *  \retval true user page is locked
 *  \retval false user page is accessible
 */
static inline bool flash_is_user_row_locked()
{
    return ( (AMBA_FLASH_PTR->MASTER_LOCKS & FLASH_MASTER_LOCKS_USER_ROW_LOCKED) != 0);
}

/*! \brief Writes quad-word of data into the manufacturer page.
 *
 * \param word_offset_in_row The word offset of the data within the manufacturer page (needs to be aligned to 4 x 32-bit words).
 * \param data_qword The data quad-word to be written into the manufacturer page.
 *
 * This function sets up all registers and starts the write operation but doesn't wait for it to complete.
 * User is expected to monitor status to learn when the write process is completed.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_manufacturer_row_qword(uint16_t const word_offset_in_row, const uint32_t data_qword[4]);

/*! \brief Writes quad-word of data into the manufacturer page.
 *
 * \param word_offset_in_row The word offset of the data within the manufacturer page (needs to be aligned to 4 x 32-bit words).
 * \param data_qword The data quad-word to be written into the manufacturer page.
 *
 * This function uses direct access to the manufacturer page, meaning it won't exit until the write actually completed.
 * Status returned shall either be READY or one of the error codes.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_manufacturer_row_qword_blocking(uint16_t word_offset_in_row, const uint32_t data_qword[4]);

/*! \brief Erases content of manufacturer page.
 *
 * This function initializes erasure of the data contained in manufacturer page.
 * It will not wait for the erase operation to complete.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_erase_manufacturer_row();

/*! \brief Reads quad-word of data from the manufacturer page.
 *
 * \param word_offset_in_row The word offset of the data within the manufacturer row (needs to be aligned to 4 x 32-bit words).
 * \param data_qword A pointer to quad-word to place data read from the manufacturer row into.
 *
 * This function reads data quad-word from the user page. Direct access scheme is used.
 * This function reads single data word from the manufacturer page. Direct access scheme is used.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_read_manufacturer_row_qword(uint16_t word_offset_in_row, uint32_t data_qword[4]);

/*! \brief Reads single word of data from the manufacturer page.
 *
 * \param word_offset_in_row The word offset of the data within the manufacturer page.
 * \param data_word Pointer to a buffer where read data will be placed.
 *
 * This function reads data word from the manufacturer page.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_read_manufacturer_row_word(uint16_t word_offset_in_row, uint32_t* data_word);

/*! \brief Locks ability to write and erase data within manufacturer page.
 *
 * This function initializes procedure for locking manufacturer page write/erase access to memory.
 * It will not wait for the operation to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_lock_manufacturer_row();

/*! \brief Check if write/erase access to manufacturer page is allowed.
 *
 * \return Status of manufacturer page accessibility
 *  \retval true manufacturer page is locked
 *  \retval false manufacturer page is accessible
 */
static inline bool flash_is_manufacturer_row_locked()
{
    return ( (AMBA_FLASH_PTR->MASTER_LOCKS & FLASH_MASTER_LOCKS_MANUFACTURER_ROW_LOCKED) != 0);
}

/*! \brief Erases whole flash memory (including all rows and lock bits).
 *
 * This function initializes erasure of whole flash memory (data + all data rows + all lock bits).
 * It will not wait for the erase operation to complete.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_chip_erase();

/*! \brief Locks ability to issue chip erase command.
 *
 * This function initializes procedure for locking access to chip erase command.
 * It will not wait for the operation to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_lock_chip_erase();

/*! \brief Check if chip erase command is available.
 *
 * \return Status of chip erase command
 *  \retval true access to chip erase command is locked
 *  \retval false chip erase command is allowed
 */
static inline bool flash_is_chip_erase_locked()
{
    return ( (AMBA_FLASH_PTR->MASTER_LOCKS & FLASH_MASTER_LOCKS_CHIP_ERASE_LOCKED) != 0);
}

/*! \brief Enables FLASH READY interrupt.
 */
static inline void flash_enable_ready_interrupt()
{
    AMBA_FLASH_PTR->IRQM |= FLASH_IRQM_READY_IE;
}

/*! \brief Disables FLASH READY interrupt.
 */
static inline void flash_disable_ready_interrupt()
{
    AMBA_FLASH_PTR->IRQM &= ~FLASH_IRQM_READY_IE;
}

/*! \brief Check if FLASH READY interrupt is enabled.
 *
 * \return Status of FLASH READY interrupt
 *  \retval true interrupt is enabled
 *  \retval false interrupt is disabled
 */
static inline bool flash_is_ready_interrupt_enabled()
{
    return ( (AMBA_FLASH_PTR->IRQM & FLASH_IRQM_READY_IE) != 0);
}

/*! \brief Enables PROGRAMMING ERROR interrupt.
 */
static inline void flash_enable_programming_error_interrupt()
{
    AMBA_FLASH_PTR->IRQM |= FLASH_IRQM_PROGRAMMING_ERROR_IE;
}

/*! \brief Disables PROGRAMMING ERROR interrupt.
 */
static inline void flash_disable_programming_error_interrupt()
{
    AMBA_FLASH_PTR->IRQM &= ~FLASH_IRQM_PROGRAMMING_ERROR_IE;
}

/*! \brief Check if PROGRAMMING ERROR interrupt is enabled.
 *
 * \return Status of PROGRAMMING ERROR interrupt
 *  \retval true interrupt is enabled
 *  \retval false interrupt is disabled
 */
static inline bool flash_is_programming_error_interrupt_enabled()
{
    return ( (AMBA_FLASH_PTR->IRQM & FLASH_IRQM_PROGRAMMING_ERROR_IE) != 0);
}

/*! \brief Enables LOCK ERROR interrupt.
 */
static inline void flash_enable_lock_error_interrupt()
{
    AMBA_FLASH_PTR->IRQM |= FLASH_IRQM_LOCK_ERROR_IE;
}

/*! \brief Disables LOCK ERROR interrupt.
 */
static inline void flash_disable_lock_error_interrupt()
{
    AMBA_FLASH_PTR->IRQM &= ~FLASH_IRQM_LOCK_ERROR_IE;
}

/*! \brief Check if LOCK ERROR interrupt is enabled.
 *
 * \return Status of LOCK ERROR interrupt
 *  \retval true interrupt is enabled
 *  \retval false interrupt is disabled
 */
static inline bool flash_is_lock_error_interrupt_enabled()
{
    return ( (AMBA_FLASH_PTR->IRQM & FLASH_IRQM_LOCK_ERROR_IE) != 0);
}

/*! \brief Enables ECC CORRECTABLE ERROR interrupt.
 */
static inline void flash_enable_ecc_correctable_error_interrupt()
{
    AMBA_FLASH_PTR->IRQM |= FLASH_IRQM_ECC_CORRECTABLE_ERROR_IE;
}

/*! \brief Disables ECC CORRECTABLE ERROR interrupt.
 */
static inline void flash_disable_ecc_correctable_error_interrupt()
{
    AMBA_FLASH_PTR->IRQM &= ~FLASH_IRQM_ECC_CORRECTABLE_ERROR_IE;
}

/*! \brief Check if ECC CORRECTABLE ERROR interrupt is enabled.
 *
 * \return Status of ECC CORRECTABLE ERROR interrupt
 *  \retval true interrupt is enabled
 *  \retval false interrupt is disabled
 */
static inline bool flash_is_ecc_correctable_error_interrupt_enabled()
{
    return ( (AMBA_FLASH_PTR->IRQM & FLASH_IRQM_ECC_CORRECTABLE_ERROR_IE) != 0);
}

/*! \brief Enables ECC UNCORRECTABLE ERROR interrupt.
 */
static inline void flash_enable_ecc_uncorrectable_error_interrupt()
{
    AMBA_FLASH_PTR->IRQM |= FLASH_IRQM_ECC_UNCORRECTABLE_ERROR_IE;
}

/*! \brief Disables ECC UNCORRECTABLE ERROR interrupt.
 */
static inline void flash_disable_ecc_uncorrectable_error_interrupt()
{
    AMBA_FLASH_PTR->IRQM &= ~FLASH_IRQM_ECC_UNCORRECTABLE_ERROR_IE;
}

/*! \brief Check if ECC UNCORRECTABLE ERROR interrupt is enabled.
 *
 * \return Status of ECC UNCORRECTABLE ERROR interrupt
 *  \retval true interrupt is enabled
 *  \retval false interrupt is disabled
 */
static inline bool flash_is_ecc_uncorrectable_error_interrupt_enabled()
{
    return ( (AMBA_FLASH_PTR->IRQM & FLASH_IRQM_ECC_UNCORRECTABLE_ERROR_IE) != 0);
}

/*! \brief Returns programmable sector size (in bytes)
 *
 * \return Size of data sector in bytes.
 */
uint32_t flash_get_sector_size_in_bytes();

/*! \brief Returns erase'able page size (in bytes)
 *
 * \return Size of data page in bytes.
 */
uint32_t flash_get_page_size_in_bytes();

/*! \brief Returns number of paged within single flash memory module.
 *
 * \return Number of data pages within memory module
 */
uint32_t flash_get_module_size_in_pages();

/*! \brief Return the number of pages per program memory region.
 *
 * \return The number of pages contained in a single region (the smallest entity that can be locked).
 */
static inline uint32_t flash_get_region_size_in_pages()
{
#ifndef BOARD_CCNV2_A1
    return (AMBA_FLASH_PTR->INFO & FLASH_INFO_REGION_SIZE_MASK) >> FLASH_INFO_REGION_SIZE_OFFSET;
#else
    // fix for an issue, where REGION size width is to small on CCNV2_A1 to fit the correct value
    return 16;
#endif
}

/*! \brief Return the number of flash memory modules.
 *
 * \return The number of flash memory modules.
 */
static inline uint32_t flash_get_number_of_modules()
{
    return (AMBA_FLASH_PTR->INFO & FLASH_INFO_MODULES_NUMBER_MASK) >> FLASH_INFO_MODULES_NUMBER_OFFSET;
}

/*! \brief Loop while module is busy.
 *
 * Function can be used to poll the status of flash controller. Polling will stop, when busy status gets deasserted.
 *
 * \return Status of flash controller.
 *   \retval READY all operations completed. Ready for next command
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR command should have been unlocked first (by writing a password to LOCK register).
 *   \retval ECC_CORR_ERROR ECC correctable error detected during flash data read.
 *   \retval ECC_UNCORR_ERROR ECC uncorrectable error detected during flash data read - data read is invalid.
 */
flash_access_status_t flash_loop_while_busy();

/*! \brief Check status of the flash controller.
 *
 * Function can be used to check status of operation performed by the flash controller.
 * The status only applies complex operations (like program/erase) initiated through COMMAND interface.
 *
 * \return Status of flash controller.
 *   \retval READY all operations completed. Ready for next command
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR command should have been unlocked first (by writing a password to LOCK register).
 *   \retval ECC_CORR_ERROR ECC correctable error detected during flash data read.
 *   \retval ECC_UNCORR_ERROR ECC uncorrectable error detected during flash data read - data read is invalid.
 */
flash_access_status_t flash_check_status();

/*! \brief Configure mapping of the FLASH peripheral interrupt.
 *
 * \param irq_mapping mapping mask of the FLASH peripheral interrupt.
 */
static inline void flash_set_mapping_of_interrupts(uint32_t irq_mapping)
{
    AMBA_FLASH_PTR->IRQMAP = irq_mapping;
}

/*! \brief Returns mapping of the FLASH peripheral interrupt.
 *
 * \return \c mapping mapping mask of the FLASH peripheral interrupt.
 */
static inline uint32_t flash_get_mapping_of_interrupts()
{
    return AMBA_FLASH_PTR->IRQMAP;
}

/*! \brief Reads data from main array.
 *
 * \param address Address to start reading from.
 * \param data Pointer to output data.
 * \param words Number of words to read.
 */
void flash_read(const uint32_t* address, uint32_t* data, uint32_t words);

#endif  // _FLASH_H_
