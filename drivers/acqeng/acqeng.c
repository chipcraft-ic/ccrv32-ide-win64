/*H*****************************************************************************
 *
 * Copyright (c) 2021 ChipCraft Sp. z o.o. All rights reserved
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
 * File Name : acqeng.c
 * Author    : Sebastian Cieslak
 * ******************************************************************************
 * $Date: 2022-04-04 11:08:39 +0200 (pon, 04 kwi 2022) $
 * $Revision: 853 $
 *H*****************************************************************************/

#include "acqeng.h"

//------------------------------------------------------------------------------
// Generic functions
//------------------------------------------------------------------------------

void acqeng_wait_for_prn_generation_done(volatile amba_acqeng_t *acqeng) {
    // Prepare variables
    uint32_t status;

    // Wait for PRNDONE flag
    while (1) {
        // Get status
        status = acqeng_get_status(acqeng);

        // Check if PRN generation was finished
        if (acqeng_is_prn_generation_done_from_status(status)) {
            break;
        }
    }
}

uint8_t acqeng_wait_for_not_busy(volatile amba_acqeng_t *acqeng) {
    // Prepare variables
    uint8_t result = 0; // [7:3] - not used,  [2] - autoerr,
                        //   [1] - sat_found, [0] - done
    uint32_t status;

    // Wait for BUSY flag low
    while (1) {
        // Get status
        status = acqeng_get_status(acqeng);

        // Check if auto error occured
        if (acqeng_is_auto_error_from_status(status)) {
            result |= 4;
        }

        // Check if sattelite was found
        if (acqeng_is_satellite_found_from_status(status)) {
            result |= 2;
        }

        // Check if acquisition was finished
        if (acqeng_is_acquisition_done_from_status(status)) {
            result |= 1;
        }

        // Check if engine is not busy
        if (acqeng_is_busy_from_status(status) == 0) {
            break;
        }
    }

    // Return result
    return result;
}

uint8_t acqeng_wait_for_done(volatile amba_acqeng_t *acqeng) {
    // Prepare variables
    uint8_t result = 0; // [7:2] - not used, [1] - autoerr, [0] - sat_found
    uint32_t status;

    // Wait for DONE flag
    while (1) {
        // Get status
        status = acqeng_get_status(acqeng);

        // Check if auto error occured
        if (acqeng_is_auto_error_from_status(status)) {
            result |= 2;
        }

        // Check if sattelite was found
        if (acqeng_is_satellite_found_from_status(status)) {
            result |= 1;
        }

        // Check if acquisition was finished
        if (acqeng_is_acquisition_done_from_status(status)) {
            break;
        }
    }

    // Return result
    return result;
}

void acqeng_wait_for_ready_low(volatile amba_acqeng_t *acqeng) {
    // Wait for READY flag low
    while (1) {
        if (acqeng_is_axi_stream_ready_enabled(acqeng) == 0) {
            break;
        }
    }
}

bool acqeng_set_carrier_mode(volatile amba_acqeng_t *acqeng, uint8_t carr_mode) {
    // Check if a carrier mode is in the correct range
    if (carr_mode > 3) {
        return 1;
    }
    else {
        _acqeng_set_carrier_mode(acqeng, carr_mode);
        return 0;
    }
}

bool acqeng_set_samples_per_chip(volatile amba_acqeng_t *acqeng, uint8_t spc_shift) {
    // Check if a number of samples per chip is in the correct range
    if (spc_shift > 15) {
        return 1;
    }
    else {
        _acqeng_set_samples_per_chip(acqeng, spc_shift);
        return 0;
    }
}

bool acqeng_set_accumulation_step(volatile amba_acqeng_t *acqeng, uint8_t step) {
    // Check if a step value is in the correct range
    if (acqeng_is_auto_mode_enabled(acqeng) && (step > 7)) {
        return 1;
    }
    else {
        _acqeng_set_accumulation_step(acqeng, step);
        return 0;
    }
}

acqeng_check_config_t acqeng_check_config(volatile amba_acqeng_t *acqeng) {
    // Get control register
    uint32_t control = acqeng_get_control(acqeng);

    // Check if clock request is enabled
    if (!acqeng_is_clock_request_enabled_from_control(control)) {
        return ACQENG_CLKREQ_OFF_ERR;
    }

    // Check if diagnostic MBIST mode is disabled
    if (acqeng_is_diag_mbist_mode_enabled_from_control(control)) {
        return ACQENG_DIAGMBIST_ON_ERR;
    }

    // Get configuration register
    uint32_t config = acqeng_get_config(acqeng);

    // Get info register
    uint32_t info = acqeng_get_info(acqeng);

    // Check if auto mode is disabled
    if (acqeng_is_auto_mode_enabled_from_config(config) && !acqeng_has_auto_mode_from_info(info)) {
        return ACQENG_AUTO_MODE_ON_ERR;
    }

    // Check if system choice is correct
    if (!(((acqeng_is_gps_search_enabled_from_config(config)     && acqeng_is_l1e1_freq_search_enabled_from_config(config)  && acqeng_has_gps_l1_prn_gen_from_info(info)     ) +
           (acqeng_is_gps_search_enabled_from_config(config)     && acqeng_is_l2_freq_search_enabled_from_config(config)    && acqeng_has_gps_l2_prn_gen_from_info(info)     ) +
           (acqeng_is_gps_search_enabled_from_config(config)     && acqeng_is_l5e5a_freq_search_enabled_from_config(config) && acqeng_has_gps_l5_prn_gen_from_info(info)     ) +
           (acqeng_is_galileo_search_enabled_from_config(config) && acqeng_is_l1e1_freq_search_enabled_from_config(config)  && acqeng_has_galileo_e1_prn_gen_from_info(info) ) +
           (acqeng_is_galileo_search_enabled_from_config(config) && acqeng_is_e6_freq_search_enabled_from_config(config)    && acqeng_has_galileo_e6_prn_gen_from_info(info) ) +
           (acqeng_is_galileo_search_enabled_from_config(config) && acqeng_is_l5e5a_freq_search_enabled_from_config(config) && acqeng_has_galileo_e5a_prn_gen_from_info(info)) +
           (acqeng_is_galileo_search_enabled_from_config(config) && acqeng_is_e5b_freq_search_enabled_from_config(config)   && acqeng_has_galileo_e5b_prn_gen_from_info(info)) +
           (acqeng_is_galileo_search_enabled_from_config(config) && acqeng_is_c_freq_search_enabled_from_config(config)     && acqeng_has_galileo_c_prn_gen_from_info(info)  ) +
           (acqeng_is_glonass_search_enabled_from_config(config) && acqeng_is_l1e1_freq_search_enabled_from_config(config)  && acqeng_has_glonass_l1_prn_gen_from_info(info) ) +
           (acqeng_is_beidou_search_enabled_from_config(config)  &&                                                            acqeng_has_beidou_prn_gen_from_info(info)     ) +
           (acqeng_is_qzss_search_enabled_from_config(config)    && acqeng_is_l1e1_freq_search_enabled_from_config(config)  && acqeng_has_qzss_l1_prn_gen_from_info(info)    ) +
           (acqeng_is_sbas_search_enabled_from_config(config)    && acqeng_is_l1e1_freq_search_enabled_from_config(config)  && acqeng_has_sbas_l1_prn_gen_from_info(info)    ) +
           (acqeng_is_navic_search_enabled_from_config(config)   && acqeng_is_l5e5a_freq_search_enabled_from_config(config) && acqeng_has_navic_l5_prn_gen_from_info(info)   )) == 1)) {
        return ACQENG_SYSTEM_CHOICE_ERR;
    }

    // Check if FFT is disabled
    if (acqeng_is_fft_enabled_from_config(config)) {
        return ACQENG_FFT_ON_ERR;
    }

    // Check if PRN 0 was set by user
    if (acqeng_get_prn_mask_0(acqeng) & 1) {
        return ACQENG_PRN_MASK_ERR;
    }

    // Check if maximal value of Doppler range is greater or equal than minimal value
    if ((int32_t)acqeng_get_max_doppler_range(acqeng) < (int32_t)acqeng_get_min_doppler_range(acqeng)) {
        return ACQENG_DOPP_RANGE_ERR;
    }

    // Check if carrier frequency satisfies Nyquist criterion
    if (acqeng_get_carrier_freq(acqeng) > 0x7FFFFFFF) {
        return ACQENG_FREQ_CARR_NYQUIST_WRNG;
    }

    // Check if Doppler frequency satisfies Nyquist criterion
    if (acqeng_get_doppler_freq(acqeng) > 0x7FFFFFFF) {
        return ACQENG_FREQ_DOPP_NYQUIST_WRNG;
    }

    // Get status register
    uint32_t status = acqeng_get_status(acqeng);

    // Check if status was clear
    if (status & ~AEC_STATUS_PRNSHFT_MASK) {
        return ACQENG_STATUS_ERR;
    }

    // Configuration is ok
    return ACQENG_OK;
}

uint32_t acqeng_hz_to_reg(int32_t freq_hz, uint32_t freq_sample_hz) {
    return (uint32_t)((freq_hz * 0x100000000) / freq_sample_hz);
}

int32_t acqeng_reg_to_hz(uint32_t freq_reg, uint32_t freq_sample_hz) {
    return (int32_t)(((int64_t)(int32_t)freq_reg * freq_sample_hz) / 0x100000000);
}

