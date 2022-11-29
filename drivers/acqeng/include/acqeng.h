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
 * File Name : acqeng.h
 * Author    : Sebastian Cieslak
 * ******************************************************************************
 * $Date: 2022-04-04 11:08:39 +0200 (pon, 04 kwi 2022) $
 * $Revision: 853 $
 *H*****************************************************************************/

#ifndef _ACQENG_H_
#define _ACQENG_H_
#pragma once

#include <stdbool.h>
#include <ccrv32-amba.h>
#include <ccrv32-amba-acqeng.h>
#include "streamer.h"

//------------------------------------------------------------------------------
// Enums
//------------------------------------------------------------------------------
/*! \brief Error type returned by the check_config function.
 */
typedef enum {
    ACQENG_OK                     = 0,
    ACQENG_CLKREQ_OFF_ERR         = 1,
    ACQENG_DIAGMBIST_ON_ERR       = 2,
    ACQENG_AUTO_MODE_ON_ERR       = 3,
    ACQENG_FFT_ON_ERR             = 4,
    ACQENG_STATUS_ERR             = 5,
    ACQENG_SYSTEM_CHOICE_ERR      = 6,
    ACQENG_PRN_MASK_ERR           = 7,
    ACQENG_DOPP_RANGE_ERR         = 8,
    ACQENG_FREQ_CARR_NYQUIST_WRNG = 9,
    ACQENG_FREQ_DOPP_NYQUIST_WRNG = 10
} acqeng_check_config_t;

//------------------------------------------------------------------------------
// STATUS
//------------------------------------------------------------------------------
//----
// All
//----
/*! \brief Returns Acquisition Engine status register.
 *
 * Be aware that the PRNDONE, SATFO and DONE bits of the Status register will be cleared after readout.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Acquisition Engine status register.
 */
static inline uint32_t acqeng_get_status(volatile amba_acqeng_t *acqeng) {
    return acqeng->STATUS;
}

//-----
// BUSY
//-----
/*! \brief Tests if the status indicates that the Acquisition Engine is busy.
 *
 * \param status Content of the status register.
 *
 * \return Value of the BUSY bit.
 *   \retval true  Acquisition Engine is busy.
 *   \retval false Acquisition Engine is not busy.
 */
static inline bool acqeng_is_busy_from_status(uint32_t status) {
    return status & AEC_STATUS_BUSY;
}

//--------
// PRNDONE
//--------
/*! \brief Tests if the status indicates that the Acquisition Engine finished PRN code generation.
 *
 *  \param status Content of the status register.
 *
 *  Be aware that the PRNDONE bit of the Status register will be cleared after Status readout.
 *
 *  \return Value of the PRNDONE bit.
 *    \retval true  PRN code generation was finished.
 *    \retval false PRN code generation was not finished.
 */
static inline bool acqeng_is_prn_generation_done_from_status(uint32_t status) {
    return status & AEC_STATUS_PRNDONE;
}

//------
// SATFO
//------
/*! \brief Tests if the status indicates that the Acquisition Engine found satellite.
 *
 *  \param status Content of the status register.
 *
 *  Be aware that the SATFO bit of the Status register will be cleared after Status readout.
 *
 *  \return Value of the SATFO bit.
 *    \retval true  Satellite was found.
 *    \retval false Satellite was not found.
 */
static inline bool acqeng_is_satellite_found_from_status(uint32_t status) {
    return status & AEC_STATUS_SATFO;
}

//-----
// DONE
//-----
/*! \brief Tests if the status indicates that the Acquisition Engine finished acquisition process.
 *
 *  \param status Content of the status register.
 *
 *  Be aware that the DONE bit of the Status register will be cleared after Status readout.
 *
 *  \return Value of the DONE bit.
 *    \retval true  Acquisition was finished.
 *    \retval false Acquisition was not finished.
 */
static inline bool acqeng_is_acquisition_done_from_status(uint32_t status) {
    return status & AEC_STATUS_DONE;
}

//--------
// PRNSHFT
//--------
/*! \brief Returns PRN shift where the highest correlation was found.
 *
 *  \param status Content of the status register.
 *
 *  This field is set in the two cases: when the satellite is found (first detection during acquisition)
 *  and at the end of acquisition process (shift where maximal correlation was found.
 *
 *  This field is related to the PRN step, so real_shift = prn_shift * prn_step.
 *
 *  \return \c PRN shift where maximal correlation was found.
 */
static inline uint16_t acqeng_get_prn_shift_from_status(uint32_t status) {
    return (status & AEC_STATUS_PRNSHFT_MASK) >> AEC_STATUS_PRNSHFT_SHIFT;
}

//--------
// AUTOERR
//--------
/*! \brief Tests if the status indicates that error occured in the auto mode acquisition.
 *
 *  \param status Content of the status register.
 *
 *  Be aware that the AUTOERR bit of the Status register will be cleared after Status readout.
 *
 *  \return Value of the AUTOERR bit.
 *    \retval true  Error occured
 *    \retval false Error did not occur.
 */
static inline bool acqeng_is_auto_error_from_status(uint32_t status) {
    return status & AEC_STATUS_AUTOERR;
}

//------
// Mixed
//------
/*! \brief Wait for the end of PRN code generation.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 */
void acqeng_wait_for_prn_generation_done(volatile amba_acqeng_t *acqeng);

/*! \brief Wait for processing all valid samples that are inside the engine (BUSY low)
 * and check if the satellite was found or if the acquisition process was finished.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Result: [7:3] - not used, [2] - autoerr, [1] - sat_found, [0] - done
 */
uint8_t acqeng_wait_for_not_busy(volatile amba_acqeng_t *acqeng);

/*! \brief Wait for the end of the acquisition process and check if the satellite was found.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * In manual mode (AUTO == 0) autoerr cannot be set, so result can be treated as bool (sat_found).
 *
 * \return Result: [7:2] - not used, [1] - autoerr, [0] - sat_found
 */
uint8_t acqeng_wait_for_done(volatile amba_acqeng_t *acqeng);

//------------------------------------------------------------------------------
// CTRL
//------------------------------------------------------------------------------
//----
// All
//----
/*! \brief Sets Acquisition Engine control register.
*
* \param acqeng Base address of the Acquisition Engine instance.
* \param control Content of the control register.
*/
static inline void acqeng_set_control(volatile amba_acqeng_t *acqeng, uint32_t control) {
    acqeng->CTRL = control;
}

/*! \brief Returns Acquisition Engine control register.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Acquisition Engine control register.
 */
static inline uint32_t acqeng_get_control(volatile amba_acqeng_t *acqeng) {
    return acqeng->CTRL;
}

//-------
// CLKREQ
//-------
/*! \brief Enables clock request in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * The clock request register is directly connected to the integrated clock gating cell,
 * so this field have to be set before the Acquisition Engine usage.
 */
static inline void acqeng_enable_clock_request(volatile amba_acqeng_t *acqeng) {
    acqeng->CTRL |= AEC_CTRL_CLKREQ;
}

/*! \brief Disables clock request in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 */
static inline void acqeng_disable_clock_request(volatile amba_acqeng_t *acqeng) {
    acqeng->CTRL &= ~AEC_CTRL_CLKREQ;
}

/*! \brief Tests if the clock request is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the CLKREQ bit.
 *   \retval true  Clock request is enabled.
 *   \retval false Clock request is disabled.
 */
static inline bool acqeng_is_clock_request_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CTRL & AEC_CTRL_CLKREQ;
}

/*! \brief Tests if control indicates that the clock request is enabled.
 *
 * \param control Content of the control register.
 *
 * \return Value of the CLKREQ bit.
 *   \retval true  Clock request is enabled.
 *   \retval false Clock request is disabled.
 */
static inline bool acqeng_is_clock_request_enabled_from_control(uint32_t control) {
    return control & AEC_CTRL_CLKREQ;
}

//----------
// DIAGMBIST
//----------
/*! \brief Enables diagnostic access MBIST mode in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a diagnostic access MBIST mode is set, readed data from shorter than 32 bits memories
 * will be extended to keep 4 bit pattern (e.g. {data[15:2], data[17:0]}.
 */
static inline void acqeng_enable_diag_mbist_mode(volatile amba_acqeng_t *acqeng) {
    acqeng->CTRL |= AEC_CTRL_DIAGMBIST;
}

/*! \brief Disables diagnostic access MBIST mode in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a diagnostic access MBIST mode is not set, readed data from shorter than 32 bits memories
 * will be extended with sign (e.g. {{14{data[17]}}, data[17:0]}.
 */
static inline void acqeng_disable_diag_mbist_mode(volatile amba_acqeng_t *acqeng) {
    acqeng->CTRL &= ~AEC_CTRL_DIAGMBIST;
}

/*! \brief Tests if the diagnostic access MBIST mode is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the DIAGMBIST bit.
 *   \retval true  Diagnostic access MBIST mode is enabled.
 *   \retval false Diagnostic access MBIST mode is disabled.
 */
static inline bool acqeng_is_diag_mbist_mode_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CTRL & AEC_CTRL_DIAGMBIST;
}

/*! \brief Tests if control indicates that the diagnostic access MBIST mode is enabled.
 *
 * \param control Content of the control register.
 *
 * \return Value of the DIAGMBIST bit.
 *   \retval true  Diagnostic access MBIST mode is enabled.
 *   \retval false Diagnostic access MBIST mode is disabled.
 */
static inline bool acqeng_is_diag_mbist_mode_enabled_from_control(uint32_t control) {
    return control & AEC_CTRL_DIAGMBIST;
}

//----
// ACT
//----
/*! \brief Activates auto mode acquisition.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * Be aware that this field is automatically cleared after one cycle.
 */
static inline void acqeng_start_auto_mode(volatile amba_acqeng_t *acqeng) {
    acqeng->CTRL |= AEC_CTRL_ACT;
}

//------
// READY
//------
/*! \brief Enables AXI Stream READY in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If an AXI Stream READY signal is set, the Acquisition Engine Core starts receiving samples
 * from the Data Streamers.
 *
 * Be aware that this field is cleared after the whole Data Streamer transfer (AXI Stream LAST).
 */
static inline void acqeng_enable_axi_stream_ready(volatile amba_acqeng_t *acqeng) {
    acqeng->CTRL |= AEC_CTRL_READY;
}

/*! \brief Disables AXI Stream READY in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If an AXI Stream READY signal is not set, the Acquisition Engine Core won't receive samples
 * from the Data Streamers.
 */
static inline void acqeng_disable_axi_stream_ready(volatile amba_acqeng_t *acqeng) {
    acqeng->CTRL &= ~AEC_CTRL_READY;
}

/*! \brief Tests if the AXI Stream READY is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the READY bit.
 *   \retval true  AXI Stream READY is enabled.
 *   \retval false AXI Stream READY is disabled.
 */
static inline bool acqeng_is_axi_stream_ready_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CTRL & AEC_CTRL_READY;
}

/*! \brief Tests if control indicates that the AXI Stream READY is enabled.
 *
 * \param control Content of the control register.
 *
 * \return Value of the READY bit.
 *   \retval true  AXI Stream READY is enabled.
 *   \retval false AXI Stream READY is disabled.
 */
static inline bool acqeng_is_axi_stream_ready_enabled_from_control(uint32_t control) {
    return control & AEC_CTRL_READY;
}

//------
// PRNCL
//------
/*! \brief Clear the PRN generator.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * Be aware that this field is automatically cleared after one cycle.
 */
static inline void acqeng_clear_prn_generator(volatile amba_acqeng_t *acqeng) {
    acqeng->CTRL |= AEC_CTRL_PRNCL;
}

//------
// PRNLO
//------
/*! \brief Load a new PRN code to the matched filter.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * Be aware that this field is automatically cleared after one cycle.
 */
static inline void acqeng_load_prn_code(volatile amba_acqeng_t *acqeng) {
    acqeng->CTRL |= AEC_CTRL_PRNLO;
}

//-------
// CARRCL
//-------
/*! \brief Clear the carrier mixer.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * Be aware that this field is automatically cleared after one cycle.
 */
static inline void acqeng_clear_carrier_mixer(volatile amba_acqeng_t *acqeng) {
    acqeng->CTRL |= AEC_CTRL_CARRCL;
}

//-------
// POSTCL
//-------
/*! \brief Clear the post processing module.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * Be aware that this field is automatically cleared after one cycle.
 */
static inline void acqeng_clear_post_processing(volatile amba_acqeng_t *acqeng) {
    acqeng->CTRL |= AEC_CTRL_POSTCL;
}

//--------
// DROPSMP
//--------
/*! \brief Enables next sample drop in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a DROPSMP is set, the next AXI Stream sample will be dropped. Should be used
 * to drop the first sample only (data streamers are byte aligned, dropping gives possibility to 4b alignment).
 *
 * Be aware that this field is cleared after the next valid sample (AXI Stream VALID and READY).
 */
static inline void acqeng_enable_drop_next_sample(volatile amba_acqeng_t *acqeng) {
    acqeng->CTRL |= AEC_CTRL_DROPSMP;
}

/*! \brief Disables next sample drop in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a DROPSMP is not set, all samples are passed from streamers to the core.
 */
static inline void acqeng_disable_drop_next_sample(volatile amba_acqeng_t *acqeng) {
    acqeng->CTRL &= ~AEC_CTRL_DROPSMP;
}

/*! \brief Tests if the next sample drop is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the DROPSMP bit.
 *   \retval true  Next sample drop is enabled.
 *   \retval false Next sample drop is disabled.
 */
static inline bool acqeng_is_drop_next_sample_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CTRL & AEC_CTRL_DROPSMP;
}

/*! \brief Tests if control indicates that the next sample drop is enabled.
 *
 * \param control Content of the control register.
 *
 * \return Value of the DROPSMP bit.
 *   \retval true  Next sample drop is enabled.
 *   \retval false Next sample drop is disabled.
 */
static inline bool acqeng_is_drop_next_sample_enabled_from_control(uint32_t control) {
    return control & AEC_CTRL_DROPSMP;
}

//---------
// REFILLMF
//---------
/*! \brief Force refilling of the matched filter data shift register.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * Set flag (filling_sr_r) in the post processing layer that indicates that
 * the data shift register is empty. Use it after full accumulation period only (wait for BUSY low),
 * otherwise the post-processing layer may stop accepting new data.
 *
 * Be aware that this field is automatically cleared after one cycle.
 */
static inline void acqeng_force_refilling_mf(volatile amba_acqeng_t *acqeng) {
    acqeng->CTRL |= AEC_CTRL_REFILLMF;
}

//-------
// AUTOCL
//-------
/*! \brief Clear the auto FSM.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * Be aware that this field is automatically cleared after one cycle.
 */
static inline void acqeng_clear_auto(volatile amba_acqeng_t *acqeng) {
    acqeng->CTRL |= AEC_CTRL_AUTOCL;
}

//------
// Mixed
//------
/*! \brief Wait for the end of the data streamer transfer (READY low).
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 */
void acqeng_wait_for_ready_low(volatile amba_acqeng_t *acqeng);

/*! \brief Enable clock request and clear rest of control fields.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 */
static inline void acqeng_enable(volatile amba_acqeng_t *acqeng) {
    acqeng->CTRL = AEC_CTRL_CLKREQ;
}

/*! \brief Clear all acquisition engine modules, but keep clock request set.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 */
static inline void acqeng_clear_all(volatile amba_acqeng_t *acqeng) {
    acqeng->CTRL = AEC_CTRL_CLKREQ | AEC_CTRL_PRNCL | AEC_CTRL_CARRCL | AEC_CTRL_POSTCL | AEC_CTRL_AUTOCL;
}

/*! \brief Prepare acquisition engine to the Doppler frequency change,
 * so clear carrier mixer and post-processing module, but keep clock request set.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 */
static inline void acqeng_clear_dopp_change(volatile amba_acqeng_t *acqeng) {
    acqeng->CTRL = AEC_CTRL_CLKREQ | AEC_CTRL_CARRCL | AEC_CTRL_POSTCL;
}

//------------------------------------------------------------------------------
// CONFIG
//------------------------------------------------------------------------------
//----
// All
//----
/*! \brief Sets Acquisition Engine configuration register.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 * \param config Content of the configuration register.
 */
static inline void acqeng_set_config(volatile amba_acqeng_t *acqeng, uint32_t config) {
    acqeng->CONFIG = config;
}

/*! \brief Returns Acquisition Engine configuration register.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Acquisition Engine configuration register.
 */
static inline uint32_t acqeng_get_config(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG;
}

//-----
// AUTO
//-----
/*! \brief Enables automatic mode in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If an automatic mode is set, acquisition engine will perform acquisition for
 * all choosed systems, choosed range of Doppler frequencies and all PRN codes.
 */
static inline void acqeng_enable_auto_mode(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_AUTO;
}

/*! \brief Disables automatic mode in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If an automatic mode is not set, acquisition engine will perform acquisition for
 * only one set of system, Doppler frequency and PRN code.
 */
static inline void acqeng_disable_auto_mode(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_AUTO;
}

/*! \brief Tests if the automatic mode is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the AUTO bit.
 *   \retval true  Automatic mode is enabled.
 *   \retval false Automatic mode is disabled.
 */
static inline bool acqeng_is_auto_mode_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_AUTO;
}

/*! \brief Tests if config indicates that the automatic mode is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the AUTO bit.
 *   \retval true  Automatic mode is enabled.
 *   \retval false Automatic mode is disabled.
 */
static inline bool acqeng_is_auto_mode_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_AUTO;
}

//----
// GPS
//----
/*! \brief Enables GPS search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a GPS search is set, acquisition engine will perform acquisition for
 * the GPS system.
 */
static inline void acqeng_enable_gps_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_GPS;
}

/*! \brief Disables GPS search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a GPS search is not set, acquisition engine will not perform acquisition for
 * the GPS system.
 */
static inline void acqeng_disable_gps_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_GPS;
}

/*! \brief Tests if the GPS search is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the GPS bit.
 *   \retval true  GPS search is enabled.
 *   \retval false GPS search is disabled.
 */
static inline bool acqeng_is_gps_search_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_GPS;
}

/*! \brief Tests if config indicates that the GPS search is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the GPS bit.
 *   \retval true  GPS search is enabled.
 *   \retval false GPS search is disabled.
 */
static inline bool acqeng_is_gps_search_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_GPS;
}

//------
// GALIL
//------
/*! \brief Enables Galileo search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a Galileo search is set, acquisition engine will perform acquisition for
 * the Galileo system.
 */
static inline void acqeng_enable_galileo_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_GALIL;
}

/*! \brief Disables Galileo search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a Galileo search is not set, acquisition engine will not perform acquisition for
 * the Galileo system.
 */
static inline void acqeng_disable_galileo_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_GALIL;
}

/*! \brief Tests if the Galileo search is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the GALIL bit.
 *   \retval true  Galileo search is enabled.
 *   \retval false Galileo search is disabled.
 */
static inline bool acqeng_is_galileo_search_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_GALIL;
}

/*! \brief Tests if config indicates that the Galileo search is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the GALIL bit.
 *   \retval true  Galileo search is enabled.
 *   \retval false Galileo search is disabled.
 */
static inline bool acqeng_is_galileo_search_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_GALIL;
}

//------
// GLONA
//------
/*! \brief Enables GLONASS search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a GLONASS search is set, acquisition engine will perform acquisition for
 * the GLONASS system.
 */
static inline void acqeng_enable_glonass_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_GLONA;
}

/*! \brief Disables GLONASS search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a GLONASS search is not set, acquisition engine will not perform acquisition for
 * the GLONASS system.
 */
static inline void acqeng_disable_glonass_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_GLONA;
}

/*! \brief Tests if the GLONASS search is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the GLONA bit.
 *   \retval true  GLONASS search is enabled.
 *   \retval false GLONASS search is disabled.
 */
static inline bool acqeng_is_glonass_search_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_GLONA;
}

/*! \brief Tests if config indicates that the GLONASS search is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the GLONA bit.
 *   \retval true  GLONASS search is enabled.
 *   \retval false GLONASS search is disabled.
 */
static inline bool acqeng_is_glonass_search_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_GLONA;
}

//-----
// BEID
//-----
/*! \brief Enables Beidou search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a Beidou search is set, acquisition engine will perform acquisition for
 * the Beidou system.
 */
static inline void acqeng_enable_beidou_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_BEID;
}

/*! \brief Disables Beidou search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a Beidou search is not set, acquisition engine will not perform acquisition for
 * the Beidou system.
 */
static inline void acqeng_disable_beidou_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_BEID;
}

/*! \brief Tests if the Beidou search is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the BEID bit.
 *   \retval true  Beidou search is enabled.
 *   \retval false Beidou search is disabled.
 */
static inline bool acqeng_is_beidou_search_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_BEID;
}

/*! \brief Tests if config indicates that the Beidou search is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the BEID bit.
 *   \retval true  Beidou search is enabled.
 *   \retval false Beidou search is disabled.
 */
static inline bool acqeng_is_beidou_search_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_BEID;
}

//-----
// QZSS
//-----
/*! \brief Enables QZSS search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a QZSS search is set, acquisition engine will perform acquisition for
 * the QZSS system.
 */
static inline void acqeng_enable_qzss_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_QZSS;
}

/*! \brief Disables QZSS search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a QZSS search is not set, acquisition engine will not perform acquisition for
 * the QZSS system.
 */
static inline void acqeng_disable_qzss_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_QZSS;
}

/*! \brief Tests if the QZSS search is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the QZSS bit.
 *   \retval true  QZSS search is enabled.
 *   \retval false QZSS search is disabled.
 */
static inline bool acqeng_is_qzss_search_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_QZSS;
}

/*! \brief Tests if config indicates that the QZSS search is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the QZSS bit.
 *   \retval true  QZSS search is enabled.
 *   \retval false QZSS search is disabled.
 */
static inline bool acqeng_is_qzss_search_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_QZSS;
}

//------
// NAVIC
//------
/*! \brief Enables NAVIC search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a NAVIC search is set, acquisition engine will perform acquisition for
 * the NAVIC system.
 */
static inline void acqeng_enable_navic_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_NAVIC;
}

/*! \brief Disables NAVIC search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a NAVIC search is not set, acquisition engine will not perform acquisition for
 * the NAVIC system.
 */
static inline void acqeng_disable_navic_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_NAVIC;
}

/*! \brief Tests if the NAVIC search is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the NAVIC bit.
 *   \retval true  NAVIC search is enabled.
 *   \retval false NAVIC search is disabled.
 */
static inline bool acqeng_is_navic_search_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_NAVIC;
}

/*! \brief Tests if config indicates that the NAVIC search is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the NAVIC bit.
 *   \retval true  NAVIC search is enabled.
 *   \retval false NAVIC search is disabled.
 */
static inline bool acqeng_is_navic_search_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_NAVIC;
}

//------
// L1E1F
//------
/*! \brief Enables L1E1 frequency search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a L1E1 frequency search is set, acquisition engine will perform acquisition for
 * the L1E1 frequency.
 */
static inline void acqeng_enable_l1e1_freq_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_L1E1F;
}

/*! \brief Disables L1E1 frequency search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a L1E1 frequency search is not set, acquisition engine will not perform acquisition for
 * the L1E1 frequency.
 */
static inline void acqeng_disable_l1e1_freq_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_L1E1F;
}

/*! \brief Tests if the L1E1 frequency search is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the L1E1F bit.
 *   \retval true  L1E1 frequency search is enabled.
 *   \retval false L1E1 frequency search is disabled.
 */
static inline bool acqeng_is_l1e1_freq_search_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_L1E1F;
}

/*! \brief Tests if config indicates that the L1E1 frequency search is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the L1E1F bit.
 *   \retval true  L1E1 frequency search is enabled.
 *   \retval false L1E1 frequency search is disabled.
 */
static inline bool acqeng_is_l1e1_freq_search_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_L1E1F;
}

//----
// L2F
//----
/*! \brief Enables L2 frequency search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a L2 frequency search is set, acquisition engine will perform acquisition for
 * the L2 frequency.
 */
static inline void acqeng_enable_l2_freq_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_L2F;
}

/*! \brief Disables L2 frequency search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a L2 frequency search is not set, acquisition engine will not perform acquisition for
 * the L2 frequency.
 */
static inline void acqeng_disable_l2_freq_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_L2F;
}

/*! \brief Tests if the L2 frequency search is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the L2F bit.
 *   \retval true  L2 frequency search is enabled.
 *   \retval false L2 frequency search is disabled.
 */
static inline bool acqeng_is_l2_freq_search_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_L2F;
}

/*! \brief Tests if config indicates that the L2 frequency search is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the L2F bit.
 *   \retval true  L2 frequency search is enabled.
 *   \retval false L2 frequency search is disabled.
 */
static inline bool acqeng_is_l2_freq_search_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_L2F;
}

//----
// E6F
//----
/*! \brief Enables E6 frequency search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a E6 frequency search is set, acquisition engine will perform acquisition for
 * the E6 frequency.
 */
static inline void acqeng_enable_e6_freq_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_E6F;
}

/*! \brief Disables E6 frequency search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a E6 frequency search is not set, acquisition engine will not perform acquisition for
 * the E6 frequency.
 */
static inline void acqeng_disable_e6_freq_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_E6F;
}

/*! \brief Tests if the E6 frequency search is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the E6F bit.
 *   \retval true  E6 frequency search is enabled.
 *   \retval false E6 frequency search is disabled.
 */
static inline bool acqeng_is_e6_freq_search_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_E6F;
}

/*! \brief Tests if config indicates that the E6 frequency search is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the E6F bit.
 *   \retval true  E6 frequency search is enabled.
 *   \retval false E6 frequency search is disabled.
 */
static inline bool acqeng_is_e6_freq_search_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_E6F;
}

//-------
// L5E5AF
//-------
/*! \brief Enables L5E5a frequency search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a L5E5a frequency search is set, acquisition engine will perform acquisition for
 * the L5E5a frequency.
 */
static inline void acqeng_enable_l5e5a_freq_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_L5E5AF;
}

/*! \brief Disables L5E5a frequency search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a L5E5a frequency search is not set, acquisition engine will not perform acquisition for
 * the L5E5a frequency.
 */
static inline void acqeng_disable_l5e5a_freq_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_L5E5AF;
}

/*! \brief Tests if the L5E5a frequency search is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the L5E5AF bit.
 *   \retval true  L5E5a frequency search is enabled.
 *   \retval false L5E5a frequency search is disabled.
 */
static inline bool acqeng_is_l5e5a_freq_search_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_L5E5AF;
}

/*! \brief Tests if config indicates that the L5E5a frequency search is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the L5E5AF bit.
 *   \retval true  L5E5a frequency search is enabled.
 *   \retval false L5E5a frequency search is disabled.
 */
static inline bool acqeng_is_l5e5a_freq_search_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_L5E5AF;
}

//-----
// E5BF
//-----
/*! \brief Enables E5b frequency search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a E5b frequency search is set, acquisition engine will perform acquisition for
 * the E5b frequency.
 */
static inline void acqeng_enable_e5b_freq_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_E5BF;
}

/*! \brief Disables E5b frequency search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a E5b frequency search is not set, acquisition engine will not perform acquisition for
 * the E5b frequency.
 */
static inline void acqeng_disable_e5b_freq_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_E5BF;
}

/*! \brief Tests if the E5b frequency search is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the E5BF bit.
 *   \retval true  E5b frequency search is enabled.
 *   \retval false E5b frequency search is disabled.
 */
static inline bool acqeng_is_e5b_freq_search_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_E5BF;
}

/*! \brief Tests if config indicates that the E5b frequency search is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the E5BF bit.
 *   \retval true  E5b frequency search is enabled.
 *   \retval false E5b frequency search is disabled.
 */
static inline bool acqeng_is_e5b_freq_search_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_E5BF;
}

//---
// SF
//---
/*! \brief Enables S frequency search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a S frequency search is set, acquisition engine will perform acquisition for
 * the S frequency.
 */
static inline void acqeng_enable_s_freq_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_SF;
}

/*! \brief Disables S frequency search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a S frequency search is not set, acquisition engine will not perform acquisition for
 * the S frequency.
 */
static inline void acqeng_disable_s_freq_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_SF;
}

/*! \brief Tests if the S frequency search is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the SF bit.
 *   \retval true  S frequency search is enabled.
 *   \retval false S frequency search is disabled.
 */
static inline bool acqeng_is_s_freq_search_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_SF;
}

/*! \brief Tests if config indicates that the S frequency search is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the SF bit.
 *   \retval true  S frequency search is enabled.
 *   \retval false S frequency search is disabled.
 */
static inline bool acqeng_is_s_freq_search_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_SF;
}

//---
// CF
//---
/*! \brief Enables C frequency search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a C frequency search is set, acquisition engine will perform acquisition for
 * the C frequency.
 */
static inline void acqeng_enable_c_freq_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_CF;
}

/*! \brief Disables C frequency search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a C frequency search is not set, acquisition engine will not perform acquisition for
 * the C frequency.
 */
static inline void acqeng_disable_c_freq_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_CF;
}

/*! \brief Tests if the C frequency search is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the CF bit.
 *   \retval true  C frequency search is enabled.
 *   \retval false C frequency search is disabled.
 */
static inline bool acqeng_is_c_freq_search_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_CF;
}

/*! \brief Tests if config indicates that the C frequency search is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the CF bit.
 *   \retval true  C frequency search is enabled.
 *   \retval false C frequency search is disabled.
 */
static inline bool acqeng_is_c_freq_search_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_CF;
}

//------
// MIXER
//------
/*! \brief Enables carrier mixer in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a carrier mixer is enabled, acquisition engine will remove carrier and Doppler
 * frequencies from the signal.
 */
static inline void acqeng_enable_carrier_mixer(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_MIXER;
}

/*! \brief Disables carrier mixer in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a carrier mixer is disabled, acquisition engine will not remove carrier and Doppler
 * frequencies from the signal.
 */
static inline void acqeng_disable_carrier_mixer(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_MIXER;
}

/*! \brief Tests if the carrier mixer is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the MIXER bit.
 *   \retval true  Carrier mixer is enabled.
 *   \retval false Carrier mixer is disabled.
 */
static inline bool acqeng_is_carrier_mixer_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_MIXER;
}

/*! \brief Tests if config indicates that the carrier mixer is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the MIXER bit.
 *   \retval true  Carrier mixer is enabled.
 *   \retval false Carrier mixer is disabled.
 */
static inline bool acqeng_is_carrier_mixer_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_MIXER;
}

//----
// FFT
//----
/*! \brief Enables FFT in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 */
static inline void acqeng_enable_fft(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_FFT;
}

/*! \brief Disables FFT in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 */
static inline void acqeng_disable_fft(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_FFT;
}

/*! \brief Tests if the FFT is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the FFT bit.
 *   \retval true  FFT is enabled.
 *   \retval false FFT is disabled.
 */
static inline bool acqeng_is_fft_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_FFT;
}

/*! \brief Tests if config indicates that the FFT is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the FFT bit.
 *   \retval true  FFT is enabled.
 *   \retval false FFT is disabled.
 */
static inline bool acqeng_is_fft_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_FFT;
}

//------
// CARRM
//------
/*! \brief Sets the carrier mode (0 - 3).
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param carr_mode Carrier mode.
 *
 * This function additionally checks if a carrier mode is in the correct range.
 *
 * \return Operation status.
 *   \retval true  ERROR: Illegal carrier mode.
 *   \retval false OK
 */
bool acqeng_set_carrier_mode(volatile amba_acqeng_t *acqeng, uint8_t carr_mode);

/*! \brief Sets the carrier mode (0 - 3).
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param carr_mode Carrier mode.
 *
 * Be aware, this function does not check if carrier mode is in the legal range.
 */
static inline void _acqeng_set_carrier_mode(volatile amba_acqeng_t *acqeng, uint8_t carr_mode) {
    acqeng->CONFIG = (acqeng->CONFIG & ~AEC_CONFIG_CARRM_MASK) | ((carr_mode << AEC_CONFIG_CARRM_SHIFT) & AEC_CONFIG_CARRM_MASK);
}

/*! \brief Returns a carrier mode.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Carrier mode.
 */
static inline uint8_t acqeng_get_carrier_mode(volatile amba_acqeng_t *acqeng) {
    return (acqeng->CONFIG & AEC_CONFIG_CARRM_MASK) >> AEC_CONFIG_CARRM_SHIFT;
}

/*! \brief Returns a carrier mode.
 *
 * \param config Content of the configuration register.
 *
 * \return \c Carrier mode.
 */
static inline uint8_t acqeng_get_carrier_mode_from_config(uint32_t config) {
    return (config & AEC_CONFIG_CARRM_MASK) >> AEC_CONFIG_CARRM_SHIFT;
}

//-------
// BOCMIX
//-------
/*! \brief Enables PRN BOC mixer in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a PRN BOC mixer is enabled, acquisition engine will mix a PRN code with the BOC(1,1).
 * This setting will extend length of a PRN code twice and decrease a PRN code generation
 * frequency twice.
 */
static inline void acqeng_enable_boc_mixer(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_BOCMIX;
}

/*! \brief Disables PRN BOC mixer in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a PRN BOC mixer is disabled, acquisition engine will not mix a PRN code with the BOC(1,1).
 */
static inline void acqeng_disable_boc_mixer(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_BOCMIX;
}

/*! \brief Tests if the PRN BOC mixer is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the BOCMIX bit.
 *   \retval true  PRN BOC mixer is enabled.
 *   \retval false PRN BOC mixer is disabled.
 */
static inline bool acqeng_is_boc_mixer_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_BOCMIX;
}

/*! \brief Tests if config indicates that the PRN BOC mixer is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the BOCMIX bit.
 *   \retval true  PRN BOC mixer is enabled.
 *   \retval false PRN BOC mixer is disabled.
 */
static inline bool acqeng_is_boc_mixer_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_BOCMIX;
}

//-----
// SPCS
//-----
/*! \brief Sets a number of samples per chip (number = (1 << SPCS)).
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param spc_shift Shift to calculate number of samples per chip.
 *
 * This function additionally checks if a number of samples per chip is in the correct range.
 *
 * \return Operation status.
 *   \retval true  ERROR: Illegal number of samples per chip.
 *   \retval false OK
 */
bool acqeng_set_samples_per_chip(volatile amba_acqeng_t *acqeng, uint8_t spc_shift);

/*! \brief Sets a number of samples per chip (number = (1 << SPCS)).
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param spc_shift Shift to calculate number of samples per chip.
 *
 * Be aware, this function does not check if a number of samples per chip is in the legal range.
 */
static inline void _acqeng_set_samples_per_chip(volatile amba_acqeng_t *acqeng, uint8_t spc_shift) {
    acqeng->CONFIG = (acqeng->CONFIG & ~AEC_CONFIG_SPCS_MASK) | ((spc_shift << AEC_CONFIG_SPCS_SHIFT) & AEC_CONFIG_SPCS_MASK);
}

/*! \brief Returns a number of samples per chip (number = (1 << SPCS)).
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Number of samples per chip (number = (1 << SPCS)).
 */
static inline uint8_t acqeng_get_samples_per_chip(volatile amba_acqeng_t *acqeng) {
    return (acqeng->CONFIG & AEC_CONFIG_SPCS_MASK) >> AEC_CONFIG_SPCS_SHIFT;
}

/*! \brief Returns a number of samples per chip (number = (1 << SPCS)).
 *
 * \param config Content of the configuration register.
 *
 * \return \c Number of samples per chip (number = (1 << SPCS)).
 */
static inline uint8_t acqeng_get_samples_per_chip_from_config(uint32_t config) {
    return (config & AEC_CONFIG_SPCS_MASK) >> AEC_CONFIG_SPCS_SHIFT;
}

//-------
// CYCLIC
//-------
/*! \brief Enables cyclic data in auto mode in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a cyclic data is enabled, acquisition engine will use the first part of data as the last one in auto mode.
 */
static inline void acqeng_enable_cyclic_data(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_CYCLIC;
}

/*! \brief Disables cyclic data in auto mode in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a a cyclic data is disabled, acquisition engine will not use the first part of data as the last one in auto mode.
 */
static inline void acqeng_disable_cyclic_data(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_CYCLIC;
}

/*! \brief Tests if the cyclic data is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the CYCLIC bit.
 *   \retval true  Cyclic data is enabled.
 *   \retval false Cyclic data is disabled.
 */
static inline bool acqeng_is_cyclic_data_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_CYCLIC;
}

/*! \brief Tests if config indicates that the cyclic data is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the CYCLIC bit.
 *   \retval true  Cyclic data is enabled.
 *   \retval false Cyclic data is disabled.
 */
static inline bool acqeng_is_cyclic_data_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_CYCLIC;
}

//-----
// SBAS
//-----
/*! \brief Enables SBAS search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a SBAS search is set, acquisition engine will perform acquisition for
 * the SBAS system.
 */
static inline void acqeng_enable_sbas_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_SBAS;
}

/*! \brief Disables SBAS search in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If a SBAS search is not set, acquisition engine will not perform acquisition for
 * the SBAS system.
 */
static inline void acqeng_disable_sbas_search(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_SBAS;
}

/*! \brief Tests if the SBAS search is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the SBAS bit.
 *   \retval true  SBAS search is enabled.
 *   \retval false SBAS search is disabled.
 */
static inline bool acqeng_is_sbas_search_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_SBAS;
}

/*! \brief Tests if config indicates that the SBAS search is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the SBAS bit.
 *   \retval true  SBAS search is enabled.
 *   \retval false SBAS search is disabled.
 */
static inline bool acqeng_is_sbas_search_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_SBAS;
}

//--------
// ALWFINE
//--------
/*! \brief Enables always fine search in auto mode in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If an always fine search is enabled, acquisition engine in auto mode will perform
 * fine search for all satellites.
 */
static inline void acqeng_enable_always_fine(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG |= AEC_CONFIG_ALWFINE;
}

/*! \brief Disables always fine search in auto mode in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * If an always fine search is disabled, acquisition engine in auto mode will perform
 * fine search only for satellites that were found in the coarse search.
 */
static inline void acqeng_disable_always_fine(volatile amba_acqeng_t *acqeng) {
    acqeng->CONFIG &= ~AEC_CONFIG_ALWFINE;
}

/*! \brief Tests if the always fine search is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the ALWFINE bit.
 *   \retval true  Always fine is enabled.
 *   \retval false Always fine is disabled.
 */
static inline bool acqeng_is_always_fine_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->CONFIG & AEC_CONFIG_ALWFINE;
}

/*! \brief Tests if config indicates that the always fine search is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the ALWFINE bit.
 *   \retval true  Always fine is enabled.
 *   \retval false Always fine is disabled.
 */
static inline bool acqeng_is_always_fine_enabled_from_config(uint32_t config) {
    return config & AEC_CONFIG_ALWFINE;
}

//------
// Mixed
//------
/* \brief Sets configuration for GPS L1 with carrier mixer enabled.
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param carr_mode Carrier mode.
 */
static inline void acqeng_enable_gps_l1_mixer(volatile amba_acqeng_t *acqeng, uint8_t carr_mode) {
    acqeng->CONFIG = AEC_CONFIG_GPS | AEC_CONFIG_L1E1F | AEC_CONFIG_MIXER | ((carr_mode << AEC_CONFIG_CARRM_SHIFT) & AEC_CONFIG_CARRM_MASK);
}

/* \brief Sets configuration for Galileo E1 with carrier mixer and BOC mixer enabled.
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param carr_mode Carrier mode.
 */
static inline void acqeng_enable_galileo_e1_mixer(volatile amba_acqeng_t *acqeng, uint8_t carr_mode) {
    acqeng->CONFIG = AEC_CONFIG_GALIL | AEC_CONFIG_L1E1F | AEC_CONFIG_MIXER | ((carr_mode << AEC_CONFIG_CARRM_SHIFT) & AEC_CONFIG_CARRM_MASK) | AEC_CONFIG_BOCMIX;
}

/* \brief Sets configuration for GLONASS L1 with carrier mixer enabled.
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param carr_mode Carrier mode.
 */
static inline void acqeng_enable_glonass_l1_mixer(volatile amba_acqeng_t *acqeng, uint8_t carr_mode) {
    acqeng->CONFIG = AEC_CONFIG_GLONA | AEC_CONFIG_L1E1F | AEC_CONFIG_MIXER | ((carr_mode << AEC_CONFIG_CARRM_SHIFT) & AEC_CONFIG_CARRM_MASK);
}

/* \brief Sets configuration for Beidou with carrier mixer enabled.
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param carr_mode Carrier mode.
 */
static inline void acqeng_enable_beidou_mixer(volatile amba_acqeng_t *acqeng, uint8_t carr_mode) {
    acqeng->CONFIG = AEC_CONFIG_BEID | AEC_CONFIG_MIXER | ((carr_mode << AEC_CONFIG_CARRM_SHIFT) & AEC_CONFIG_CARRM_MASK);
}

/* \brief Sets configuration for SBAS L1 with carrier mixer enabled.
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param carr_mode Carrier mode.
 */
static inline void acqeng_enable_sbas_l1_mixer(volatile amba_acqeng_t *acqeng, uint8_t carr_mode) {
    acqeng->CONFIG = AEC_CONFIG_SBAS | AEC_CONFIG_L1E1F | AEC_CONFIG_MIXER | ((carr_mode << AEC_CONFIG_CARRM_SHIFT) & AEC_CONFIG_CARRM_MASK);
}

/* \brief Sets configuration for NavIC L5 with carrier mixer enabled.
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param carr_mode Carrier mode.
 */
static inline void acqeng_enable_navic_l5_mixer(volatile amba_acqeng_t *acqeng, uint8_t carr_mode) {
    acqeng->CONFIG = AEC_CONFIG_NAVIC | AEC_CONFIG_L5E5AF | AEC_CONFIG_MIXER | ((carr_mode << AEC_CONFIG_CARRM_SHIFT) & AEC_CONFIG_CARRM_MASK);
}

/* \brief Sets configuration for QZSS L1 with carrier mixer enabled.
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param carr_mode Carrier mode.
 */
static inline void acqeng_enable_qzss_l1_mixer(volatile amba_acqeng_t *acqeng, uint8_t carr_mode) {
    acqeng->CONFIG = AEC_CONFIG_QZSS | AEC_CONFIG_L1E1F | AEC_CONFIG_MIXER | ((carr_mode << AEC_CONFIG_CARRM_SHIFT) & AEC_CONFIG_CARRM_MASK);
}

/* \brief Sets configuration for auto mode for GPS L1 with carrier mixer enabled.
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param carr_mode Carrier mode.
 * \param spc_shift Samples per Chip as Shift.
 * \param is_cyclic Cyclic Data enable.
 */
static inline void acqeng_enable_auto_gps_l1_mixer(volatile amba_acqeng_t *acqeng, uint8_t carr_mode, uint8_t spc_shift, bool is_cyclic, bool always_fine) {
    acqeng->CONFIG = AEC_CONFIG_AUTO | AEC_CONFIG_GPS | AEC_CONFIG_L1E1F | AEC_CONFIG_MIXER | ((carr_mode << AEC_CONFIG_CARRM_SHIFT) & AEC_CONFIG_CARRM_MASK) |
                     ((spc_shift << AEC_CONFIG_SPCS_SHIFT) & AEC_CONFIG_SPCS_MASK) | (is_cyclic ? AEC_CONFIG_CYCLIC : 0) | (always_fine ? AEC_CONFIG_ALWFINE : 0);
}

/* \brief Sets configuration for auto mode for Galileo E1 with carrier mixer and BOC mixer enabled.
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param carr_mode Carrier mode.
 * \param spc_shift Samples per Chip as Shift.
 * \param is_cyclic Cyclic Data enable.
 */
static inline void acqeng_enable_auto_galileo_e1_mixer(volatile amba_acqeng_t *acqeng, uint8_t carr_mode, uint8_t spc_shift, bool is_cyclic, bool always_fine) {
    acqeng->CONFIG = AEC_CONFIG_AUTO | AEC_CONFIG_GALIL | AEC_CONFIG_L1E1F | AEC_CONFIG_MIXER | ((carr_mode << AEC_CONFIG_CARRM_SHIFT) & AEC_CONFIG_CARRM_MASK) |
                     AEC_CONFIG_BOCMIX | ((spc_shift << AEC_CONFIG_SPCS_SHIFT) & AEC_CONFIG_SPCS_MASK) | (is_cyclic ? AEC_CONFIG_CYCLIC : 0) | (always_fine ? AEC_CONFIG_ALWFINE : 0);
}

/* \brief Sets configuration for auto mode for GLONASS L1 with carrier mixer enabled.
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param carr_mode Carrier mode.
 * \param spc_shift Samples per Chip as Shift.
 * \param is_cyclic Cyclic Data enable.
 */
static inline void acqeng_enable_auto_glonass_l1_mixer(volatile amba_acqeng_t *acqeng, uint8_t carr_mode, uint8_t spc_shift, bool is_cyclic, bool always_fine) {
    acqeng->CONFIG = AEC_CONFIG_AUTO | AEC_CONFIG_GLONA | AEC_CONFIG_L1E1F | AEC_CONFIG_MIXER | ((carr_mode << AEC_CONFIG_CARRM_SHIFT) & AEC_CONFIG_CARRM_MASK) |
                     ((spc_shift << AEC_CONFIG_SPCS_SHIFT) & AEC_CONFIG_SPCS_MASK) | (is_cyclic ? AEC_CONFIG_CYCLIC : 0) | (always_fine ? AEC_CONFIG_ALWFINE : 0);
}

/* \brief Sets configuration for auto mode for Beidou with carrier mixer enabled.
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param carr_mode Carrier mode.
 * \param spc_shift Samples per Chip as Shift.
 * \param is_cyclic Cyclic Data enable
 */
static inline void acqeng_enable_auto_beidou_mixer(volatile amba_acqeng_t *acqeng, uint8_t carr_mode, uint8_t spc_shift, bool is_cyclic, bool always_fine) {
    acqeng->CONFIG = AEC_CONFIG_AUTO | AEC_CONFIG_BEID | AEC_CONFIG_MIXER | ((carr_mode << AEC_CONFIG_CARRM_SHIFT) & AEC_CONFIG_CARRM_MASK) |
                     ((spc_shift << AEC_CONFIG_SPCS_SHIFT) & AEC_CONFIG_SPCS_MASK) | (is_cyclic ? AEC_CONFIG_CYCLIC : 0) | (always_fine ? AEC_CONFIG_ALWFINE : 0);
}

/* \brief Sets configuration for auto mode for SBAS L1 with carrier mixer enabled.
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param carr_mode Carrier mode.
 * \param spc_shift Samples per Chip as Shift.
 * \param is_cyclic Cyclic Data enable.
 */
static inline void acqeng_enable_auto_sbas_l1_mixer(volatile amba_acqeng_t *acqeng, uint8_t carr_mode, uint8_t spc_shift, bool is_cyclic, bool always_fine) {
    acqeng->CONFIG = AEC_CONFIG_AUTO | AEC_CONFIG_SBAS | AEC_CONFIG_L1E1F | AEC_CONFIG_MIXER | ((carr_mode << AEC_CONFIG_CARRM_SHIFT) & AEC_CONFIG_CARRM_MASK) |
                     ((spc_shift << AEC_CONFIG_SPCS_SHIFT) & AEC_CONFIG_SPCS_MASK) | (is_cyclic ? AEC_CONFIG_CYCLIC : 0) | (always_fine ? AEC_CONFIG_ALWFINE : 0);
}

/* \brief Sets configuration for auto mode for NavIC L5 with carrier mixer enabled.
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param carr_mode Carrier mode.
 * \param spc_shift Samples per Chip as Shift.
 * \param is_cyclic Cyclic Data enable.
 */
static inline void acqeng_enable_auto_navic_l5_mixer(volatile amba_acqeng_t *acqeng, uint8_t carr_mode, uint8_t spc_shift, bool is_cyclic, bool always_fine) {
    acqeng->CONFIG = AEC_CONFIG_AUTO | AEC_CONFIG_NAVIC | AEC_CONFIG_L5E5AF | AEC_CONFIG_MIXER | ((carr_mode << AEC_CONFIG_CARRM_SHIFT) & AEC_CONFIG_CARRM_MASK) |
                     ((spc_shift << AEC_CONFIG_SPCS_SHIFT) & AEC_CONFIG_SPCS_MASK) | (is_cyclic ? AEC_CONFIG_CYCLIC : 0) | (always_fine ? AEC_CONFIG_ALWFINE : 0);
}

/* \brief Sets configuration for auto mode for QZSS L1 with carrier mixer enabled.
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param carr_mode Carrier mode.
 * \param spc_shift Samples per Chip as Shift.
 * \param is_cyclic Cyclic Data enable.
 */
static inline void acqeng_enable_auto_qzss_l1_mixer(volatile amba_acqeng_t *acqeng, uint8_t carr_mode, uint8_t spc_shift, bool is_cyclic, bool always_fine) {
    acqeng->CONFIG = AEC_CONFIG_AUTO | AEC_CONFIG_QZSS | AEC_CONFIG_L1E1F | AEC_CONFIG_MIXER | ((carr_mode << AEC_CONFIG_CARRM_SHIFT) & AEC_CONFIG_CARRM_MASK) |
                     ((spc_shift << AEC_CONFIG_SPCS_SHIFT) & AEC_CONFIG_SPCS_MASK) | (is_cyclic ? AEC_CONFIG_CYCLIC : 0) | (always_fine ? AEC_CONFIG_ALWFINE : 0);
}

//------------------------------------------------------------------------------
// CFG_F_IF
//------------------------------------------------------------------------------
/*! \brief Sets a carrier frequency register ((f_carr / f_sample) * (2^32)).
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param carr_freq Carrier frequency.
 */
static inline void acqeng_set_carrier_freq(volatile amba_acqeng_t *acqeng, uint32_t carr_freq) {
    acqeng->CFG_F_IF = carr_freq;
}

/*! \brief Returns a carrier frequency register ((f_carr / f_sample) * (2^32)).
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Carrier frequency.
 */
static inline uint32_t acqeng_get_carrier_freq(volatile amba_acqeng_t *acqeng) {
    return acqeng->CFG_F_IF;
}

//------------------------------------------------------------------------------
// CFG_F_DOPP
//------------------------------------------------------------------------------
/*! \brief Sets a Doppler frequency register ((f_dopp / f_sample) * (2^32)).
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param dopp_freq Doppler frequency.
 */
static inline void acqeng_set_doppler_freq(volatile amba_acqeng_t *acqeng, uint32_t dopp_freq) {
    acqeng->CFG_F_DOPP = dopp_freq;
}

/*! \brief Returns a Doppler frequency register ((f_dopp / f_sample) * (2^32)).
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Doppler frequency.
 */
static inline uint32_t acqeng_get_doppler_freq(volatile amba_acqeng_t *acqeng) {
    return acqeng->CFG_F_DOPP;
}

//------------------------------------------------------------------------------
// SET_F_PHASE
//------------------------------------------------------------------------------
/*! \brief Sets a carrier mixer NCO phase register ((phase / 2PI) * (2^32)).
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 * \param phase  Phase.
 */
static inline void acqeng_set_phase(volatile amba_acqeng_t *acqeng, uint32_t phase) {
    acqeng->SET_F_PHASE = phase;
}

/*! \brief Returns a carrier mixer NCO phase register ((phase / 2PI) * (2^32)).
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Phase.
 */
static inline uint32_t acqeng_get_phase(volatile amba_acqeng_t *acqeng) {
    return acqeng->SET_F_PHASE;
}

//------------------------------------------------------------------------------
// CFG_FINE_F_DOPP
//------------------------------------------------------------------------------
/*! \brief Sets a fine Doppler search step value ((f_fine / f_sample) * (2^32)).
 *
 * \param acqeng    Base address of the Acquisition Engine instance.
 * \param fine_dopp Fine Doppler Search Step.
 */
static inline void acqeng_set_fine_doppler_step(volatile amba_acqeng_t *acqeng, uint32_t fine_dopp) {
    acqeng->CFG_FINE_F_DOPP = fine_dopp;
}

/*! \brief Returns a fine Doppler search step value ((f_fine / f_sample) * (2^32)).
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Fine Doppler Search Step.
 */
static inline uint32_t acqeng_get_fine_doppler_step(volatile amba_acqeng_t *acqeng) {
    return acqeng->CFG_FINE_F_DOPP;
}

//------------------------------------------------------------------------------
// CFG_PRN_CODE
//------------------------------------------------------------------------------
//----
// All
//----
/*! \brief Sets a PRN generation configuration register.
 *
 * \param acqeng     Base address of the Acquisition Engine instance.
 * \param prn_config PRN generation configuration.
 */
static inline void acqeng_set_prn_config(volatile amba_acqeng_t *acqeng, uint32_t prn_config) {
    acqeng->CFG_PRN_CODE = prn_config;
}

/*! \brief Returns a PRN generation configuration register.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c PRN generation configuration.
 */
static inline uint32_t acqeng_get_prn_config(volatile amba_acqeng_t *acqeng) {
    return acqeng->CFG_PRN_CODE;
}

//----
// PRN
//----
/*! \brief Sets a PRN number that will be generated.
 *
 * \param acqeng     Base address of the Acquisition Engine instance.
 * \param prn_number PRN number.
 */
static inline void acqeng_set_prn_number(volatile amba_acqeng_t *acqeng, uint8_t prn_number) {
    acqeng->CFG_PRN_CODE = (acqeng->CFG_PRN_CODE & ~AEC_CFG_PRN_CODE_PRN_MASK) | ((prn_number << AEC_CFG_PRN_CODE_PRN_SHIFT) & AEC_CFG_PRN_CODE_PRN_MASK);
}

/*! \brief Returns a PRN number.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c PRN number.
 */
static inline uint8_t acqeng_get_prn_number(volatile amba_acqeng_t *acqeng) {
    return (acqeng->CFG_PRN_CODE & AEC_CFG_PRN_CODE_PRN_MASK) >> AEC_CFG_PRN_CODE_PRN_SHIFT;
}

/*! \brief Returns a PRN number.
 *
 * \param prn_config Content of the PRN generation configuration register.
 *
 * \return \c PRN number.
 */
static inline uint8_t acqeng_get_prn_number_from_prn_config(uint32_t prn_config) {
    return (prn_config & AEC_CFG_PRN_CODE_PRN_MASK) >> AEC_CFG_PRN_CODE_PRN_SHIFT;
}

//------
// PRNBI
//------
/*! \brief Sets a number of PRN chips that will be generated.
 *
 * \param acqeng     Base address of the Acquisition Engine instance.
 * \param prn_length Number of PRN chips to generate.
 */
static inline void acqeng_set_prn_length(volatile amba_acqeng_t *acqeng, uint16_t prn_length) {
    acqeng->CFG_PRN_CODE = (acqeng->CFG_PRN_CODE & ~AEC_CFG_PRN_CODE_PRNBI_MASK) | ((prn_length << AEC_CFG_PRN_CODE_PRNBI_SHIFT) & AEC_CFG_PRN_CODE_PRNBI_MASK);
}

/*! \brief Returns a PRN length.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c PRN length.
 */
static inline uint16_t acqeng_get_prn_length(volatile amba_acqeng_t *acqeng) {
    return (acqeng->CFG_PRN_CODE & AEC_CFG_PRN_CODE_PRNBI_MASK) >> AEC_CFG_PRN_CODE_PRNBI_SHIFT;
}

/*! \brief Returns a PRN length.
 *
 * \param prn_config Content of the PRN generation configuration register.
 *
 * \return \c PRN length.
 */
static inline uint16_t acqeng_get_prn_length_from_prn_config(uint32_t prn_config) {
    return (prn_config & AEC_CFG_PRN_CODE_PRNBI_MASK) >> AEC_CFG_PRN_CODE_PRNBI_SHIFT;
}

//------------------------------------------------------------------------------
// CFG_PRN_FREQ
//------------------------------------------------------------------------------
/*! \brief Sets a PRN generation frequency register ((f_prn / f_sample) * (2^32)).
 *
 * \param acqeng   Base address of the Acquisition Engine instance.
 * \param prn_freq PRN generation frequency.
 */
static inline void acqeng_set_prn_freq(volatile amba_acqeng_t *acqeng, uint32_t prn_freq) {
    acqeng->CFG_PRN_FREQ = prn_freq;
}

/*! \brief Returns a PRN generation frequency register ((f_prn / f_sample) * (2^32)).
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c PRN generation frequency.
 */
static inline uint32_t acqeng_get_prn_freq(volatile amba_acqeng_t *acqeng) {
    return acqeng->CFG_PRN_FREQ;
}

//------------------------------------------------------------------------------
// CFG_POST
//------------------------------------------------------------------------------
//----
// All
//----
/*! \brief Sets a post-processing configuration register.
 *
 * \param acqeng          Base address of the Acquisition Engine instance.
 * \param post_processing Post-processing configuration.
 */
static inline void acqeng_set_post_processing(volatile amba_acqeng_t *acqeng, uint32_t post_processing) {
    acqeng->CFG_POST = post_processing;
}

/*! \brief Returns a post-processing configuration register.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Post-processing configuration.
 */
static inline uint32_t acqeng_get_post_processing(volatile amba_acqeng_t *acqeng) {
    return acqeng->CFG_POST;
}

//-------
// PERIOD
//-------
/*! \brief Sets a length of accumulation period in samples (PERIOD = real_period / prn_step).
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 * \param period Length of accumulation period in samples.
 */
static inline void acqeng_set_post_processing_period(volatile amba_acqeng_t *acqeng, uint16_t period) {
    acqeng->CFG_POST = (acqeng->CFG_POST & ~AEC_CFG_POST_PERIOD_MASK) | ((period << AEC_CFG_POST_PERIOD_SHIFT) & AEC_CFG_POST_PERIOD_MASK);
}

/*! \brief Returns a length of accumulation period in samples (PERIOD = real_period / prn_step).
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Length of accumulation period in samples.
 */
static inline uint16_t acqeng_get_post_processing_period(volatile amba_acqeng_t *acqeng) {
    return (acqeng->CFG_POST & AEC_CFG_POST_PERIOD_MASK) >> AEC_CFG_POST_PERIOD_SHIFT;
}

/*! \brief Returns a length of accumulation period in samples (PERIOD = real_period / prn_step).
 *
 * \param post_processing Content of the post-processing configuration register.
 *
 * \return \c Length of accumulation period in samples.
 */
static inline uint16_t acqeng_get_post_processing_period_from_post_processing(uint32_t post_processing) {
    return (post_processing & AEC_CFG_POST_PERIOD_MASK) >> AEC_CFG_POST_PERIOD_SHIFT;
}

//--------
// NPERCOH
//--------
/*! \brief Sets a number of periods to accumulate coherently (number = (1 << NPERCOH)).
 *
 * \param acqeng       Base address of the Acquisition Engine instance.
 * \param shift_number Shift to calculate number of accumulations.
 */
static inline void acqeng_set_number_of_coh_periods(volatile amba_acqeng_t *acqeng, uint8_t shift_number) {
    acqeng->CFG_POST = (acqeng->CFG_POST & ~AEC_CFG_POST_NPERCOH_MASK) | ((shift_number << AEC_CFG_POST_NPERCOH_SHIFT) & AEC_CFG_POST_NPERCOH_MASK);
}

/*! \brief Returns a number of periods to accumulate coherently (number = (1 << NPERCOH)).
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Number of periods to accumulate coherently.
 */
static inline uint8_t acqeng_get_number_of_coh_periods(volatile amba_acqeng_t *acqeng) {
    return (acqeng->CFG_POST & AEC_CFG_POST_NPERCOH_MASK) >> AEC_CFG_POST_NPERCOH_SHIFT;
}

/*! \brief Returns a number of periods to accumulate coherently (number = (1 << NPERCOH)).
 *
 * \param post_processing Content of the post-processing configuration register.
 *
 * \return \c Number of periods to accumulate coherently.
 */
static inline uint8_t acqeng_get_number_of_coh_periods_from_post_processing(uint32_t post_processing) {
    return (post_processing & AEC_CFG_POST_NPERCOH_MASK) >> AEC_CFG_POST_NPERCOH_SHIFT;
}

//---------
// NPERNCOH
//---------
/*! \brief Sets a number of periods to accumulate noncoherently (number = (1 << NPERNCOH)).
 *
 * \param acqeng       Base address of the Acquisition Engine instance.
 * \param shift_number Shift to calculate number of accumulations.
 */
static inline void acqeng_set_number_of_ncoh_periods(volatile amba_acqeng_t *acqeng, uint8_t shift_number) {
    acqeng->CFG_POST = (acqeng->CFG_POST & ~AEC_CFG_POST_NPERNCOH_MASK) | ((shift_number << AEC_CFG_POST_NPERNCOH_SHIFT) & AEC_CFG_POST_NPERNCOH_MASK);
}

/*! \brief Returns a number of periods to accumulate noncoherently (number = (1 << NPERNCOH)).
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Number of periods to accumulate noncoherently.
 */
static inline uint8_t acqeng_get_number_of_ncoh_periods(volatile amba_acqeng_t *acqeng) {
    return (acqeng->CFG_POST & AEC_CFG_POST_NPERNCOH_MASK) >> AEC_CFG_POST_NPERNCOH_SHIFT;
}

/*! \brief Returns a number of periods to accumulate noncoherently (number = (1 << NPERNCOH)).
 *
 * \param post_processing Content of the post-processing configuration register.
 *
 * \return \c Number of periods to accumulate noncoherently.
 */
static inline uint8_t acqeng_get_number_of_ncoh_periods_from_post_processing(uint32_t post_processing) {
    return (post_processing & AEC_CFG_POST_NPERNCOH_MASK) >> AEC_CFG_POST_NPERNCOH_SHIFT;
}

//-----
// STEP
//-----
/*! \brief Sets a PRN step that chooses which correlation results will be saved.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 * \param step   PRN step.
 *
 * This function additionally checks if the step is in the correct range.
 *
 * \return Operation status.
 *   \retval true  ERROR: Illegal accumulation step.
 *   \retval false OK
 */
bool acqeng_set_accumulation_step(volatile amba_acqeng_t *acqeng, uint8_t step);

/*! \brief Sets a PRN step that chooses which correlation results will be saved.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 * \param step   PRN step.
 *
 * Be aware, this function does not check if the step is in the legal range.
 */
static inline void _acqeng_set_accumulation_step(volatile amba_acqeng_t *acqeng, uint8_t step) {
    acqeng->CFG_POST = (acqeng->CFG_POST & ~AEC_CFG_POST_STEP_MASK) | ((step << AEC_CFG_POST_STEP_SHIFT) & AEC_CFG_POST_STEP_MASK);
}

/*! \brief Returns a PRN step that chooses which correlation results will be saved.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c PRN step.
 */
static inline uint8_t acqeng_get_accumulation_step(volatile amba_acqeng_t *acqeng) {
    return (acqeng->CFG_POST & AEC_CFG_POST_STEP_MASK) >> AEC_CFG_POST_STEP_SHIFT;
}

/*! \brief Returns a PRN step that chooses which correlation results will be saved.
 *
 * \param post_processing Content of the post-processing configuration register.
 *
 * \return \c PRN step.
 */
static inline uint8_t acqeng_get_accumulation_step_from_post_processing(uint32_t post_processing) {
    return (post_processing & AEC_CFG_POST_STEP_MASK) >> AEC_CFG_POST_STEP_SHIFT;
}

//------------------------------------------------------------------------------
// CFG_DET
//------------------------------------------------------------------------------
/*! \brief Sets a detection threshold.
 *
 * \param acqeng              Base address of the Acquisition Engine instance.
 * \param detection_threshold Detection threshold.
 */
static inline void acqeng_set_detection_threshold(volatile amba_acqeng_t *acqeng, uint32_t detection_threshold) {
    acqeng->CFG_DET = detection_threshold;
}

/*! \brief Returns a detection threshold.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Detection threshold.
 */
static inline uint32_t acqeng_get_detection_threshold(volatile amba_acqeng_t *acqeng) {
    return acqeng->CFG_DET;
}

//------------------------------------------------------------------------------
// CFG_MIN_F_DOPP
//------------------------------------------------------------------------------
/*! \brief Sets a minimal value of Doppler range ((f_min / f_sample) * (2^32)).
 *
 * \param acqeng   Base address of the Acquisition Engine instance.
 * \param min_dopp Minimal Doppler Range.
 */
static inline void acqeng_set_min_doppler_range(volatile amba_acqeng_t *acqeng, uint32_t min_dopp) {
    acqeng->CFG_MIN_F_DOPP = min_dopp;
}

/*! \brief Returns a minimal value of Doppler range ((f_min / f_sample) * (2^32)).
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Minimal Doppler Range.
 */
static inline uint32_t acqeng_get_min_doppler_range(volatile amba_acqeng_t *acqeng) {
    return acqeng->CFG_MIN_F_DOPP;
}

//------------------------------------------------------------------------------
// CFG_MAX_F_DOPP
//------------------------------------------------------------------------------
/*! \brief Sets a maximal value of Doppler range ((f_max / f_sample) * (2^32)).
 *
 * \param acqeng   Base address of the Acquisition Engine instance.
 * \param max_dopp Maximal Doppler Range.
 */
static inline void acqeng_set_max_doppler_range(volatile amba_acqeng_t *acqeng, uint32_t max_dopp) {
    acqeng->CFG_MAX_F_DOPP = max_dopp;
}

/*! \brief Returns a maximal value of Doppler range ((f_max / f_sample) * (2^32)).
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Maximal Doppler Range.
 */
static inline uint32_t acqeng_get_max_doppler_range(volatile amba_acqeng_t *acqeng) {
    return acqeng->CFG_MAX_F_DOPP;
}

//------------------------------------------------------------------------------
// CFG_PRN_MASK_0
//------------------------------------------------------------------------------
/*! \brief Sets a bit choice of PRN numbers (0-31) that will be searched in the auto mode (PRN == i).
 *
 * \param acqeng     Base address of the Acquisition Engine instance.
 * \param prn_mask_0 PRN Mask 0.
 */
static inline void acqeng_set_prn_mask_0(volatile amba_acqeng_t *acqeng, uint32_t prn_mask_0) {
    acqeng->CFG_PRN_MASK_0 = prn_mask_0;
}

/*! \brief Returns a bit choice of PRN numbers (0-31) that will be searched in the auto mode (PRN == i).
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c PRN Mask 0.
 */
static inline uint32_t acqeng_get_prn_mask_0(volatile amba_acqeng_t *acqeng) {
    return acqeng->CFG_PRN_MASK_0;
}

//------------------------------------------------------------------------------
// CFG_PRN_MASK_1
//------------------------------------------------------------------------------
/*! \brief Sets a bit choice of PRN numbers (32-63) that will be searched in the auto mode (PRN == i + 32).
 *
 * \param acqeng     Base address of the Acquisition Engine instance.
 * \param prn_mask_1 PRN Mask 1.
 */
static inline void acqeng_set_prn_mask_1(volatile amba_acqeng_t *acqeng, uint32_t prn_mask_1) {
    acqeng->CFG_PRN_MASK_1 = prn_mask_1;
}

/*! \brief Returns a bit choice of PRN numbers (32-63) that will be searched in the auto mode (PRN == i + 32).
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c PRN Mask 1.
 */
static inline uint32_t acqeng_get_prn_mask_1(volatile amba_acqeng_t *acqeng) {
    return acqeng->CFG_PRN_MASK_1;
}

//------------------------------------------------------------------------------
// INFO
//------------------------------------------------------------------------------
//----
// All
//----
/*! \brief Returns a info register.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Info.
 */
static inline uint32_t acqeng_get_info(volatile amba_acqeng_t *acqeng) {
    return acqeng->INFO;
}

//--------
// VERSION
//--------
/*! \brief Returns the Acquisition Engine version.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Acquisition Engine version.
 */
static inline uint8_t acqeng_get_version(volatile amba_acqeng_t *acqeng) {
    return (acqeng->INFO & AEC_INFO_VERSION_MASK) >> AEC_INFO_VERSION_SHIFT;
}

/*! \brief Returns the Acquisition Engine version.
 *
 * \param info Content of the info register.
 *
 * \return \c Acquisition Engine version.
 */
static inline uint8_t acqeng_get_version_from_info(uint32_t info) {
    return (info & AEC_INFO_VERSION_MASK) >> AEC_INFO_VERSION_SHIFT;
}

//---------
// HAS_AUTO
//---------
/*! \brief Tests if the Acquisition Engine has an automatic mode.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the HAS_AUTO bit.
 *   \retval true  Automatic mode is present.
 *   \retval false Automatic mode is not present.
 */
static inline bool acqeng_has_auto_mode(volatile amba_acqeng_t *acqeng) {
    return acqeng->INFO & AEC_INFO_HAS_AUTO;
}

/*! \brief Tests if info indicates that the Acquisition Engine has an automatic mode.
 *
 * \param info Content of the info register.
 *
 * \return Value of the HAS_AUTO bit.
 *   \retval true  Automatic mode is present.
 *   \retval false Automatic mode is not present.
 */
static inline bool acqeng_has_auto_mode_from_info(uint32_t info) {
    return info & AEC_INFO_HAS_AUTO;
}

//-------
// CORRWD
//-------
/*! \brief Returns a correlation width (accumulator RAM width).
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Correlation width.
 */
static inline uint8_t acqeng_get_correlation_width(volatile amba_acqeng_t *acqeng) {
    return (acqeng->INFO & AEC_INFO_CORRWD_MASK) >> AEC_INFO_CORRWD_SHIFT;
}

/*! \brief Returns a correlation width (accumulator RAM width).
 *
 * \param info Content of the info register.
 *
 * \return \c Correlation width.
 */
static inline uint8_t acqeng_get_correlation_width_from_info(uint32_t info) {
    return (info & AEC_INFO_CORRWD_MASK) >> AEC_INFO_CORRWD_SHIFT;
}

//-----
// RSVD
//-----
/*! \brief Returns a reserved field.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Reserved.
 */
static inline uint8_t acqeng_get_reserved_info(volatile amba_acqeng_t *acqeng) {
    return (acqeng->INFO & AEC_INFO_RSVD_MASK) >> AEC_INFO_RSVD_SHIFT;
}

/*! \brief Returns a reserved field.
 *
 * \param info Content of the info register.
 *
 * \return \c Reserved.
 */
static inline uint8_t acqeng_get_reserved_info_from_info(uint32_t info) {
    return (info & AEC_INFO_RSVD_MASK) >> AEC_INFO_RSVD_SHIFT;
}

//--------
// SBAS_L1
//--------
/*! \brief Tests if the Acquisition Engine has a SBAS L1 PRN generator.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the SBAS_L1 bit.
 *   \retval true  SBAS L1 PRN generator is present.
 *   \retval false SBAS L1 PRN generator is not present.
 */
static inline bool acqeng_has_sbas_l1_prn_gen(volatile amba_acqeng_t *acqeng) {
    return acqeng->INFO & AEC_INFO_SBAS_L1;
}

/*! \brief Tests if info indicates that the Acquisition Engine has a SBAS L1 PRN generator.
 *
 * \param info Content of the info register.
 *
 * \return Value of the SBAS_L1 bit.
 *   \retval true  SBAS L1 PRN generator is present.
 *   \retval false SBAS L1 PRN generator is not present.
 */
static inline bool acqeng_has_sbas_l1_prn_gen_from_info(uint32_t info) {
    return info & AEC_INFO_SBAS_L1;
}

//---------
// NAVIC_L5
//---------
/*! \brief Tests if the Acquisition Engine has a NAVIC L5 PRN generator.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the NAVIC L5 bit.
 *   \retval true  NAVIC L5 PRN generator is present.
 *   \retval false NAVIC L5 PRN generator is not present.
 */
static inline bool acqeng_has_navic_l5_prn_gen(volatile amba_acqeng_t *acqeng) {
    return acqeng->INFO & AEC_INFO_NAVIC_L5;
}

/*! \brief Tests if info indicates that the Acquisition Engine has a NAVIC L5 PRN generator.
 *
 * \param info Content of the info register.
 *
 * \return Value of the NAVIC L5 bit.
 *   \retval true  NAVIC L5 PRN generator is present.
 *   \retval false NAVIC L5 PRN generator is not present.
 */
static inline bool acqeng_has_navic_l5_prn_gen_from_info(uint32_t info) {
    return info & AEC_INFO_NAVIC_L5;
}

//--------
// QZSS L1
//--------
/*! \brief Tests if the Acquisition Engine has a QZSS L1 PRN generator.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the QZSS L1 bit.
 *   \retval true  QZSS L1 PRN generator is present.
 *   \retval false QZSS L1 PRN generator is not present.
 */
static inline bool acqeng_has_qzss_l1_prn_gen(volatile amba_acqeng_t *acqeng) {
    return acqeng->INFO & AEC_INFO_QZSS_L1;
}

/*! \brief Tests if info indicates that the Acquisition Engine has a QZSS L1 PRN generator.
 *
 * \param info Content of the info register.
 *
 * \return Value of the QZSS L1 bit.
 *   \retval true  QZSS L1 PRN generator is present.
 *   \retval false QZSS L1 PRN generator is not present.
 */
static inline bool acqeng_has_qzss_l1_prn_gen_from_info(uint32_t info) {
    return info & AEC_INFO_QZSS_L1;
}

//-----
// BEID
//-----
/*! \brief Tests if the Acquisition Engine has a Beidou PRN generator.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the BEID bit.
 *   \retval true  Beidou PRN generator is present.
 *   \retval false Beidou PRN generator is not present.
 */
static inline bool acqeng_has_beidou_prn_gen(volatile amba_acqeng_t *acqeng) {
    return acqeng->INFO & AEC_INFO_BEID;
}

/*! \brief Tests if info indicates that the Acquisition Engine has a Beidou PRN generator.
 *
 * \param info Content of the info register.
 *
 * \return Value of the BEID bit.
 *   \retval true  Beidou PRN generator is present.
 *   \retval false Beidou PRN generator is not present.
 */
static inline bool acqeng_has_beidou_prn_gen_from_info(uint32_t info) {
    return info & AEC_INFO_BEID;
}

//---------
// GLONA_L1
//---------
/*! \brief Tests if the Acquisition Engine has a GLONASS L1 PRN generator.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the GLONA_L1 bit.
 *   \retval true  GLONASS L1 PRN generator is present.
 *   \retval false GLONASS L1 PRN generator is not present.
 */
static inline bool acqeng_has_glonass_l1_prn_gen(volatile amba_acqeng_t *acqeng) {
    return acqeng->INFO & AEC_INFO_GLONA_L1;
}

/*! \brief Tests if info indicates that the Acquisition Engine has a GLONASS L1 PRN generator.
 *
 * \param info Content of the info register.
 *
 * \return Value of the GLONA_L1 bit.
 *   \retval true  GLONASS L1 PRN generator is present.
 *   \retval false GLONASS L1 PRN generator is not present.
 */
static inline bool acqeng_has_glonass_l1_prn_gen_from_info(uint32_t info) {
    return info & AEC_INFO_GLONA_L1;
}

//--------
// GALIL_C
//--------
/*! \brief Tests if the Acquisition Engine has a Galileo C PRN generator.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the GALIL_C bit.
 *   \retval true  Galileo C PRN generator is present.
 *   \retval false Galileo C PRN generator is not present.
 */
static inline bool acqeng_has_galileo_c_prn_gen(volatile amba_acqeng_t *acqeng) {
    return acqeng->INFO & AEC_INFO_GALIL_C;
}

/*! \brief Tests if info indicates that the Acquisition Engine has a Galileo C PRN generator.
 *
 * \param info Content of the info register.
 *
 * \return Value of the GALIL_C bit.
 *   \retval true  Galileo C PRN generator is present.
 *   \retval false Galileo C generator is not present.
 */
static inline bool acqeng_has_galileo_c_prn_gen_from_info(uint32_t info) {
    return info & AEC_INFO_GALIL_C;
}

//----------
// GALIL_E5B
//----------
/*! \brief Tests if the Acquisition Engine has a Galileo E5b PRN generator.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the GALIL_E5B bit.
 *   \retval true  Galileo E5b PRN generator is present.
 *   \retval false Galileo E5b PRN generator is not present.
 */
static inline bool acqeng_has_galileo_e5b_prn_gen(volatile amba_acqeng_t *acqeng) {
    return acqeng->INFO & AEC_INFO_GALIL_E5B;
}

/*! \brief Tests if info indicates that the Acquisition Engine has a Galileo E5b PRN generator.
 *
 * \param info Content of the info register.
 *
 * \return Value of the GALIL_E5B bit.
 *   \retval true  Galileo E5b PRN generator is present.
 *   \retval false Galileo E5b PRN generator is not present.
 */
static inline bool acqeng_has_galileo_e5b_prn_gen_from_info(uint32_t info) {
    return info & AEC_INFO_GALIL_E5B;
}

//----------
// GALIL_E5A
//----------
/*! \brief Tests if the Acquisition Engine has a Galileo E5a PRN generator.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the GALIL_E5A bit.
 *   \retval true  Galileo E5a PRN generator is present.
 *   \retval false Galileo E5a PRN generator is not present.
 */
static inline bool acqeng_has_galileo_e5a_prn_gen(volatile amba_acqeng_t *acqeng) {
    return acqeng->INFO & AEC_INFO_GALIL_E5A;
}

/*! \brief Tests if info indicates that the Acquisition Engine has a Galileo E5a PRN generator.
 *
 * \param info Content of the info register.
 *
 * \return Value of the GALIL_E5A bit.
 *   \retval true  Galileo E5a PRN generator is present.
 *   \retval false Galileo E5a PRN generator is not present.
 */
static inline bool acqeng_has_galileo_e5a_prn_gen_from_info(uint32_t info) {
    return info & AEC_INFO_GALIL_E5A;
}

//---------
// GALIL_E6
//---------
/*! \brief Tests if the Acquisition Engine has a Galileo E6 PRN generator.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the GALIL_E6 bit.
 *   \retval true  Galileo E6 PRN generator is present.
 *   \retval false Galileo E6 PRN generator is not present.
 */
static inline bool acqeng_has_galileo_e6_prn_gen(volatile amba_acqeng_t *acqeng) {
    return acqeng->INFO & AEC_INFO_GALIL_E6;
}

/*! \brief Tests if info indicates that the Acquisition Engine has a Galileo E6 PRN generator.
 *
 * \param info Content of the info register.
 *
 * \return Value of the GALIL_E6 bit.
 *   \retval true  Galileo E6 PRN generator is present.
 *   \retval false Galileo E6 PRN generator is not present.
 */
static inline bool acqeng_has_galileo_e6_prn_gen_from_info(uint32_t info) {
    return info & AEC_INFO_GALIL_E6;
}

//---------
// GALIL_E1
//---------
/*! \brief Tests if the Acquisition Engine has a Galileo E1 PRN generator.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the GALIL_E1 bit.
 *   \retval true  Galileo E1 PRN generator is present.
 *   \retval false Galileo E1 PRN generator is not present.
 */
static inline bool acqeng_has_galileo_e1_prn_gen(volatile amba_acqeng_t *acqeng) {
    return acqeng->INFO & AEC_INFO_GALIL_E1;
}

/*! \brief Tests if info indicates that the Acquisition Engine has a Galileo E1 PRN generator.
 *
 * \param info Content of the info register.
 *
 * \return Value of the GALIL_E1 bit.
 *   \retval true  Galileo E1 PRN generator is present.
 *   \retval false Galileo E1 PRN generator is not present.
 */
static inline bool acqeng_has_galileo_e1_prn_gen_from_info(uint32_t info) {
    return info & AEC_INFO_GALIL_E1;
}

//-------
// GPS_L5
//-------
/*! \brief Tests if the Acquisition Engine has a GPS L5 PRN generator.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the GPS_L5 bit.
 *   \retval true  GPS L5 PRN generator is present.
 *   \retval false GPS L5 PRN generator is not present.
 */
static inline bool acqeng_has_gps_l5_prn_gen(volatile amba_acqeng_t *acqeng) {
    return acqeng->INFO & AEC_INFO_GPS_L5;
}

/*! \brief Tests if info indicates that the Acquisition Engine has a GPS L5 PRN generator.
 *
 * \param info Content of the info register.
 *
 * \return Value of the GPS_L5 bit.
 *   \retval true  GPS L5 PRN generator is present.
 *   \retval false GPS L5 PRN generator is not present.
 */
static inline bool acqeng_has_gps_l5_prn_gen_from_info(uint32_t info) {
    return info & AEC_INFO_GPS_L5;
}

//-------
// GPS_L2
//-------
/*! \brief Tests if the Acquisition Engine has a GPS L2 PRN generator.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the GPS_L2 bit.
 *   \retval true  GPS L2 PRN generator is present.
 *   \retval false GPS L2 PRN generator is not present.
 */
static inline bool acqeng_has_gps_l2_prn_gen(volatile amba_acqeng_t *acqeng) {
    return acqeng->INFO & AEC_INFO_GPS_L2;
}

/*! \brief Tests if info indicates that the Acquisition Engine has a GPS L2 PRN generator.
 *
 * \param info Content of the info register.
 *
 * \return Value of the GPS_L2 bit.
 *   \retval true  GPS L2 PRN generator is present.
 *   \retval false GPS L2 PRN generator is not present.
 */
static inline bool acqeng_has_gps_l2_prn_gen_from_info(uint32_t info) {
    return info & AEC_INFO_GPS_L2;
}

//-------
// GPS_L1
//-------
/*! \brief Tests if the Acquisition Engine has a GPS L1 PRN generator.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the GPS_L1 bit.
 *   \retval true  GPS L1 PRN generator is present.
 *   \retval false GPS L1 PRN generator is not present.
 */
static inline bool acqeng_has_gps_l1_prn_gen(volatile amba_acqeng_t *acqeng) {
    return acqeng->INFO & AEC_INFO_GPS_L1;
}

/*! \brief Tests if info indicates that the Acquisition Engine has a GPS L1 PRN generator.
 *
 * \param info Content of the info register.
 *
 * \return Value of the GPS_L1 bit.
 *   \retval true  GPS L1 PRN generator is present.
 *   \retval false GPS L1 PRN generator is not present.
 */
static inline bool acqeng_has_gps_l1_prn_gen_from_info(uint32_t info) {
    return info & AEC_INFO_GPS_L1;
}

//------------------------------------------------------------------------------
// INFO_MF
//------------------------------------------------------------------------------
//----
// All
//----
/*! \brief Returns a matched filter info register.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Matched filter info.
 */
static inline uint32_t acqeng_get_matched_filter_info(volatile amba_acqeng_t *acqeng) {
    return acqeng->INFO_MF;
}

//-------
// MFDATA
//-------
/*! \brief Returns a matched filter data shift register length.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Matched filter data shift register length.
 */
static inline uint16_t acqeng_get_mf_data_sr_length(volatile amba_acqeng_t *acqeng) {
    return (acqeng->INFO_MF & AEC_INFO_MF_MFDATA_MASK) >> AEC_INFO_MF_MFDATA_SHIFT;
}

/*! \brief Returns a matched filter data shift register length.
 *
 * \param mf_info Content of the matched filter info register.
 *
 * \return \c Matched filter data shift register length.
 */
static inline uint16_t acqeng_get_mf_data_sr_length_from_mf_info(uint32_t mf_info) {
    return (mf_info & AEC_INFO_MF_MFDATA_MASK) >> AEC_INFO_MF_MFDATA_SHIFT;
}

//-------
// MFCODE
//-------
/*! \brief Returns a matched filter code shift register length.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Matched filter code shift register length.
 */
static inline uint16_t acqeng_get_mf_code_sr_length(volatile amba_acqeng_t *acqeng) {
    return (acqeng->INFO_MF & AEC_INFO_MF_MFCODE_MASK) >> AEC_INFO_MF_MFCODE_SHIFT;
}

/*! \brief Returns a matched filter code shift register length.
 *
 * \param mf_info Content of the matched filter info register.
 *
 * \return \c Matched filter code shift register length.
 */
static inline uint16_t acqeng_get_mf_code_sr_length_from_mf_info(uint32_t mf_info) {
    return (mf_info & AEC_INFO_MF_MFCODE_MASK) >> AEC_INFO_MF_MFCODE_SHIFT;
}

//------------------------------------------------------------------------------
// IRQMASK
//------------------------------------------------------------------------------
//----
// All
//----
/*! \brief Sets Acquisition Engine interrupt mask register.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 * \param irq_mask Content of the interrupt mask register.
 */
static inline void acqeng_set_interrupt_mask(volatile amba_acqeng_t *acqeng, uint32_t irq_mask) {
    acqeng->IRQMASK = irq_mask;
}

/*! \brief Returns Acquisition Engine interrupt mask register.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return \c Acquisition Engine interrupt mask register.
 */
static inline uint32_t acqeng_get_interrupt_mask(volatile amba_acqeng_t *acqeng) {
    return acqeng->IRQMASK;
}

//----------
// PRNDONEIE
//----------
/*! \brief Enables PRN generation done interrupt in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 */
static inline void acqeng_enable_PRNDONE_interrupt(volatile amba_acqeng_t *acqeng) {
    acqeng->IRQMASK |= AEC_IRQMASK_PRNDONEIE;
}

/*! \brief Disables PRN generation done interrupt in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 */
static inline void acqeng_disable_PRNDONE_interrupt(volatile amba_acqeng_t *acqeng) {
    acqeng->IRQMASK &= ~AEC_IRQMASK_PRNDONEIE;
}

/*! \brief Tests if the PRN generation done interrupt is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the PRNDONEIE bit.
 *   \retval true  PRN generation done interrupt is enabled.
 *   \retval false PRN generation done interrupt is disabled.
 */
static inline bool acqeng_is_PRNDONE_interrupt_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->IRQMASK & AEC_IRQMASK_PRNDONEIE;
}

/*! \brief Tests if interrupt mask indicates that the PRN generation done interrupt is enabled.
 *
 * \param mask Content of the interrupt mask register.
 *
 * \return Value of the PRNDONEIE bit.
 *   \retval true  PRN generation done interrupt is enabled.
 *   \retval false PRN generation done interrupt is disabled.
 */
static inline bool acqeng_is_PRNDONE_interrupt_enabled_from_mask(uint32_t mask) {
    return mask & AEC_IRQMASK_PRNDONEIE;
}

//--------
// SATFOIE
//--------
/*! \brief Enables satellite found interrupt in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 */
static inline void acqeng_enable_SATFO_interrupt(volatile amba_acqeng_t *acqeng) {
    acqeng->IRQMASK |= AEC_IRQMASK_SATFOIE;
}

/*! \brief Disables satellite found interrupt in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 */
static inline void acqeng_disable_SATFO_interrupt(volatile amba_acqeng_t *acqeng) {
    acqeng->IRQMASK &= ~AEC_IRQMASK_SATFOIE;
}

/*! \brief Tests if the satellite found interrupt is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the SATFOIE bit.
 *   \retval true  Satellite found interrupt is enabled.
 *   \retval false Satellite found interrupt is disabled.
 */
static inline bool acqeng_is_SATFO_interrupt_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->IRQMASK & AEC_IRQMASK_SATFOIE;
}

/*! \brief Tests if interrupt mask indicates that the satellite found interrupt is enabled.
 *
 * \param mask Content of the interrupt mask register.
 *
 * \return Value of the SATFOIE bit.
 *   \retval true  Satellite found interrupt is enabled.
 *   \retval false Satellite found interrupt is disabled.
 */
static inline bool acqeng_is_SATFO_interrupt_enabled_from_mask(uint32_t mask) {
    return mask & AEC_IRQMASK_SATFOIE;
}

//-------
// DONEIE
//-------
/*! \brief Enables acquisition done interrupt in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 */
static inline void acqeng_enable_DONE_interrupt(volatile amba_acqeng_t *acqeng) {
    acqeng->IRQMASK |= AEC_IRQMASK_DONEIE;
}

/*! \brief Disables acquisition done interrupt in the Acquisition Engine.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 */
static inline void acqeng_disable_DONE_interrupt(volatile amba_acqeng_t *acqeng) {
    acqeng->IRQMASK &= ~AEC_IRQMASK_DONEIE;
}

/*! \brief Tests if the acquisition done interrupt is enabled.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * \return Value of the DONEIE bit.
 *   \retval true  Acquisition done interrupt is enabled.
 *   \retval false Acquisition done interrupt is disabled.
 */
static inline bool acqeng_is_DONE_interrupt_enabled(volatile amba_acqeng_t *acqeng) {
    return acqeng->IRQMASK & AEC_IRQMASK_DONEIE;
}

/*! \brief Tests if interrupt mask indicates that the acquisition done interrupt is enabled.
 *
 * \param mask Content of the interrupt mask register.
 *
 * \return Value of the DONEIE bit.
 *   \retval true  Acquisition done interrupt is enabled.
 *   \retval false Acquisition done interrupt is disabled.
 */
static inline bool acqeng_is_DONE_interrupt_enabled_from_mask(uint32_t mask) {
    return mask & AEC_IRQMASK_DONEIE;
}

//------------------------------------------------------------------------------
// Generic functions
//------------------------------------------------------------------------------
/*! \brief Check correctness of acquisition engine configuration.
 *
 * \param acqeng Base address of the Acquisition Engine instance.
 *
 * This function checks clock request, diagnostic access MBIST mode, configuration
 * and frequencies.
 *
 * \return Operation status.
 *   \retval ACQENG_OK                     Configuration is correct.
 *   \retval ACQENG_CLKREQ_OFF_ERR         Clock request is disabled.
 *   \retval ACQENG_DIAGMBIST_ON_ERR       Diagnostic access MBIST mode is enabled.
 *   \retval ACQENG_AUTO_MODE_ON_ERR       Automatic mode enabled, but is not present in the chip.
 *   \retval ACQENG_FFT_ON_ERR             FFT enabled, but is not present in the chip.
 *   \retval ACQENG_STATUS_ERR             Status flags were not cleared.
 *   \retval ACQENG_SYSTEM_CHOICE_ERR      Illegal combination of system configuration.
 *   \retval ACQENG_PRN_MASK_ERR           PRN 0 set in mask.
 *   \retval ACQENG_DOPP_RANGE_ERR         Maximal value of Doppler range is smaller than minimal value.
 *   \retval ACQENG_FREQ_CARR_NYQUIST_WRNG Carrier frequency is greater than half of sampling frequency.
 *   \retval ACQENG_FREQ_DOPP_NYQUIST_WRNG Doppler frequency is greater than half of sampling frequency.
 */
acqeng_check_config_t acqeng_check_config(volatile amba_acqeng_t *acqeng);

/*! \brief Convert a frequency in Hz to the scaled normalized frequency (f/fs * 2^32).
 *
 * \param freq_hz        Frequency to convert in Hz.
 * \param freq_sample_hz Sampling frequency in Hz.
 *
 * \return \c Scaled normalized frequency.
 */
uint32_t acqeng_hz_to_reg(int32_t freq_hz, uint32_t freq_sample_hz);

/*! \brief Convert a scaled normalized frequency (f/fs * 2^32) to the frequency in Hz.
 *
 * \param freq_reg       Scaled normalized frequency to convert.
 * \param freq_sample_hz Sampling frequency in Hz.
 *
 * \return \c Frequency in Hz.
 */
int32_t acqeng_reg_to_hz(uint32_t freq_reg, uint32_t freq_sample_hz);

#endif // _ACQENG_H_

