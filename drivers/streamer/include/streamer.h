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
 * File Name : streamer.h
 * Author    : Sebastian Cieslak
 * ******************************************************************************
 * $Date: 2022-01-19 09:38:48 +0100 (śro, 19 sty 2022) $
 * $Revision: 814 $
 *H*****************************************************************************/

#ifndef _STREAMER_H_
#define _STREAMER_H_
#pragma once

#include <stdbool.h>
#include <ccrv32-amba.h>
#include <ccrv32-amba-streamer.h>

//------------------------------------------------------------------------------
// Enums
//------------------------------------------------------------------------------
/*! \brief Error type returned by the check_config function.
 */
typedef enum {
    STREAMER_OK                = 0,
    STREAMER_SSS_TOO_BIG_ERR   = 1,
    STREAMER_ADDR_ALIGN_ERR    = 2,
    STREAMER_CNT_ZERO_ERR      = 3,
    STREAMER_CNT_OVRFLW_ERR    = 4,
    STREAMER_ADDRREL_ALIGN_ERR = 5,
    STREAMER_CNTREL_OVRFLW_ERR = 6,
    STREAMER_CNTREL_ZERO_ERR   = 7
} streamer_check_config_t;

//------------------------------------------------------------------------------
// STATUS
//------------------------------------------------------------------------------
//----
// All
//----
/*! \brief Returns Streamer status register.
 *
 * Be aware that the MAR and MERR bits of the Status register will be cleared after readout.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return \c Streamer status register.
 */
static inline uint32_t streamer_get_status(volatile amba_streamer_t *streamer) {
    return streamer->STATUS;
}

//-----
// BUSY
//-----
/*! \brief Tests if the status indicates that the Streamer is busy.
 *
 * \param status Content of the status register.
 *
 * \return Value of the BUSY bit.
 *   \retval true  Streamer is busy.
 *   \retval false Streamer is not busy.
 */
static inline bool streamer_is_busy_from_status(uint32_t status) {
    return status & DS_STATUS_BUSY;
}

//----
// SCZ
//----
/*! \brief Tests if the status indicates that the Streamer counter is zero.
 *
 * \param status Content of the status register.
 *
 * \return Value of the SCZ bit.
 *   \retval true  Counter is equal zero.
 *   \retval false Counter is not equal zero.
 */
static inline bool streamer_is_counter_zero_from_status(uint32_t status) {
    return status & DS_STATUS_SCZ;
}

//-----
// SRCZ
//-----
/*! \brief Tests if the status indicates that the Streamer reload counter is zero.
 *
 * \param status Content of the status register.
 *
 * \return Value of the SRCZ bit.
 *   \retval true  Reload counter is equal zero.
 *   \retval false Reload counter is not equal zero.
 */
static inline bool streamer_is_counter_reload_zero_from_status(uint32_t status) {
    return status & DS_STATUS_SRCZ;
}

//----
// MAR
//----
/*! \brief Tests if the status indicates that the Streamer performed memory address reload.
 *
 * \param status Content of the status register.
 *
 * Be aware that the MAR bit of the Status register will be cleared after Status readout.
 *
 * \return Value of the MAR bit.
 *   \retval true  Reload occurred.
 *   \retval false Reload did not occur.
 */
static inline bool streamer_is_address_reload_from_status(uint32_t status) {
    return status & DS_STATUS_MAR;
}

//-----
// MERR
//-----
/*! \brief Tests if the status indicates that the Streamer encountered AHB error during memory readout.
 *
 * \param status Content of the status register.
 *
 * Be aware that the MERR bit of the Status register will be cleared after Status readout.
 *
 * \return Value of the MERR bit.
 *   \retval true  Error occurred.
 *   \retval false Error did not occur.
 */
static inline bool streamer_is_access_error_from_status(uint32_t status) {
    return status & DS_STATUS_MERR;
}

//------
// FIFOU
//------
/*! \brief Returns number of words stored in the FIFO.
 *
 * \param status Content of the status register.
 *
 * \return \c Number of words stored in the FIFO.
 */
static inline uint16_t streamer_get_fifo_usage_from_status(uint32_t status) {
    return (status & DS_STATUS_FIFOU_MASK) >> DS_STATUS_FIFOU_SHIFT;
}

//------------------------------------------------------------------------------
// CTRL
//------------------------------------------------------------------------------
//----
// All
//----
/*! \brief Sets Streamer control register.
 *
 * \param streamer Base address of the Streamer instance.
 * \param control Content of the control register.
 */
static inline void streamer_set_control(volatile amba_streamer_t *streamer, uint32_t control) {
    streamer->CTRL = control;
}

/*! \brief Returns Streamer control register.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return \c Streamer control register.
 */
static inline uint32_t streamer_get_control(volatile amba_streamer_t *streamer) {
    return streamer->CTRL;
}

//----
// ACT
//----
/*! \brief Starts loading data from the main memory.
 *
 * \param streamer Base address of the Streamer instance.
 */
static inline void streamer_start_loading(volatile amba_streamer_t *streamer) {
    streamer->CTRL |= DS_CTRL_ACT;
}

/*! \brief Stops loading data from the main memory.
 *
 * \param streamer Base address of the Streamer instance.
 */
static inline void streamer_stop_loading(volatile amba_streamer_t *streamer) {
    streamer->CTRL &= ~DS_CTRL_ACT;
}

/*! \brief Tests if loading is enabled.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return Value of the ACT bit.
 *   \retval true  Loading is enabled.
 *   \retval false Loading is disabled.
 */
static inline bool streamer_is_loading_started(volatile amba_streamer_t *streamer) {
    return streamer->CTRL & DS_CTRL_ACT;
}

/*! \brief Tests if control indicates that loading is enabled.
 *
 * \param control Content of the control register.
 *
 * \return Value of the ACT bit.
 *   \retval true  Loading is enabled.
 *   \retval false Loading is disabled.
 */
static inline bool streamer_is_loading_started_from_control(uint32_t control) {
    return control & DS_CTRL_ACT;
}

//----
// ENA
//----
/*! \brief Starts sending data on the AXI Stream interface.
 *
 * \param streamer Base address of the Streamer instance.
 */
static inline void streamer_start_streaming(volatile amba_streamer_t *streamer) {
    streamer->CTRL |= DS_CTRL_ENA;
}

/*! \brief Stops sending data on the AXI Stream interface.
 *
 * \param streamer Base address of the Streamer instance.
 */
static inline void streamer_stop_streaming(volatile amba_streamer_t *streamer) {
    streamer->CTRL &= ~DS_CTRL_ENA;
}

/*! \brief Tests if streaming is enabled.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return Value of the ENA bit.
 *   \retval true  Streaming is enabled.
 *   \retval false Streaming is disabled.
 */
static inline bool streamer_is_streaming_started(volatile amba_streamer_t *streamer) {
    return streamer->CTRL & DS_CTRL_ENA;
}

/*! \brief Tests if control indicates that streaming is enabled.
 *
 * \param control Content of the control register.
 *
 * \return Value of the ENA bit.
 *   \retval true  Streaming is enabled.
 *   \retval false Streaming is disabled.
 */
static inline bool streamer_is_streaming_started_from_control(uint32_t control) {
    return control & DS_CTRL_ENA;
}

//------
// FLUSH
//------
/*! \brief Clear all internal counters and a FIFO.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * Be aware that this field is automatically cleared after one cycle.
 */
static inline void streamer_flush(volatile amba_streamer_t *streamer) {
    streamer->CTRL |= DS_CTRL_FLUSH;
}

//------
// Mixed
//------
/*! \brief Starts loading and sending data.
 *
 * \param streamer Base address of the Streamer instance.
 */
static inline void streamer_start(volatile amba_streamer_t *streamer) {
    streamer->CTRL = DS_CTRL_ACT | DS_CTRL_ENA;
}

/*! \brief Stops loading and sending data.
 *
 * \param streamer Base address of the Streamer instance.
 */
static inline void streamer_stop(volatile amba_streamer_t *streamer) {
    streamer->CTRL = 0;
}

/*! \brief Clear all internal states of Streamer and stop loading and streaming.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * This function clears streamer, waits for BUSY == 0 (so for the finish of pending
 * AHB requests) and clears streamer again.
 * Be aware that the AXI Stream packet will be cut in the middle so the LAST will not be set.
 */
void streamer_flush_and_stop(volatile amba_streamer_t *streamer);

/*! \brief Clear all internal states of Streamer and stop loading and streaming without waiting.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * Be aware that this field is automatically cleared after one cycle.
 * This function does not terminate pending AHB request, so it is possible that
 * after flush some newly readed data will be put into the FIFO.
 */
static inline void _streamer_flush_and_stop(volatile amba_streamer_t *streamer) {
    streamer->CTRL = DS_CTRL_FLUSH;
}

//------------------------------------------------------------------------------
// CONFIG
//------------------------------------------------------------------------------
//----
// All
//----
/*! \brief Sets Streamer configuration register.
 *
 * \param streamer Base address of the Streamer instance.
 * \param config Content of the configuration register.
 *
 * This function additionally checks if a sample size value is in the correct range.
 *
 * \return Operation status.
 *   \retval true  ERROR: Sample size greater than the streamer interface
 *   \retval false OK
 */
bool streamer_set_config(volatile amba_streamer_t *streamer, uint32_t config);

/*! \brief Sets Streamer configuration register.
 *
 * \param streamer Base address of the Streamer instance.
 * \param config Content of the configuration register.
 */
static inline void _streamer_set_config(volatile amba_streamer_t *streamer, uint32_t config) {
    streamer->CONFIG = config;
}

/*! \brief Returns Streamer configuration register.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return \c Streamer configuration register.
 */
static inline uint32_t streamer_get_config(volatile amba_streamer_t *streamer) {
    return streamer->CONFIG;
}

//-----
// RING
//-----
/*! \brief Enables ring buffer in the Streamer.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * If a ring buffer is set, the reload counter will not be cleared after copying to the normal counter.
 */
static inline void streamer_enable_ring_buffer(volatile amba_streamer_t *streamer) {
    streamer->CONFIG |= DS_CONFIG_RING;
}

/*! \brief Disables ring buffer in the Streamer.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * If a ring buffer is not set, the reload counter will be cleared after copying to the normal counter.
 */
static inline void streamer_disable_ring_buffer(volatile amba_streamer_t *streamer) {
    streamer->CONFIG &= ~DS_CONFIG_RING;
}

/*! \brief Tests if the ring buffer is enabled.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return Value of the RING bit.
 *   \retval true  Ring buffer is enabled.
 *   \retval false Ring buffer is disabled.
 */
static inline bool streamer_is_ring_buffer_enabled(volatile amba_streamer_t *streamer) {
    return streamer->CONFIG & DS_CONFIG_RING;
}

/*! \brief Tests if config indicates that the ring buffer is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the RING bit.
 *   \retval true  Ring buffer is enabled.
 *   \retval false Ring buffer is disabled.
 */
static inline bool streamer_is_ring_buffer_enabled_from_config(uint32_t config) {
    return config & DS_CONFIG_RING;
}

//----
// SSS
//----
/*! \brief Sets the sample size (Encoded: SIZE = 1 << SSS).
 *
 * \param streamer    Base address of the Streamer instance.
 * \param sample_size Sample size (Encoded: SIZE = 1 << SSS).
 *
 * This function additionally checks if a sample size value is in the correct range.
 *
 * \return Operation status.
 *   \retval true  ERROR: Sample size greater than the streamer interface.
 *   \retval false OK
 */
bool streamer_set_sample_size(volatile amba_streamer_t *streamer, uint8_t sample_size);

/*! \brief Sets the sample size (Encoded: SIZE = 1 << SSS).
 *
 * \param streamer    Base address of the Streamer instance.
 * \param sample_size Sample size (Encoded: SIZE = 1 << SSS).
 *
 * Be aware, this function does not check if sample size is in the legal range.
 */
static inline void _streamer_set_sample_size(volatile amba_streamer_t *streamer, uint8_t sample_size) {
    streamer->CONFIG = (streamer->CONFIG & ~DS_CONFIG_SSS_MASK) | ((sample_size << DS_CONFIG_SSS_SHIFT) & DS_CONFIG_SSS_MASK);
}

/*! \brief Returns a sample size (Encoded: SIZE = 1 << SSS).
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return \c Sample size (Encoded: SIZE = 1 << SSS).
 */
static inline uint8_t streamer_get_sample_size(volatile amba_streamer_t *streamer) {
    return (streamer->CONFIG & DS_CONFIG_SSS_MASK) >> DS_CONFIG_SSS_SHIFT;
}

/*! \brief Returns a sample size (Encoded: SIZE = 1 << SSS).
 *
 * \param config Content of the configuration register.
 *
 * \return \c Sample size (Encoded: SIZE = 1 << SSS).
 */
static inline uint8_t streamer_get_sample_size_from_config(uint32_t config) {
    return (config & DS_CONFIG_SSS_MASK) >> DS_CONFIG_SSS_SHIFT;
}

//-----
// SEST
//-----
/*! \brief Enables sign extension in the Streamer.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * If a sign extension is set and a stream sample is smaller than the interface,
 * upper bits of AXI Stream Data will be used as a sign extension.
 */
static inline void streamer_enable_sign_extend(volatile amba_streamer_t *streamer) {
    streamer->CONFIG |= DS_CONFIG_SEST;
}

/*! \brief Disables sign extension in the Streamer.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * If a sign extension is not set and a stream sample is smaller than the interface,
 * upper bits of AXI Stream Data will be set to 0.
 */
static inline void streamer_disable_sign_extend(volatile amba_streamer_t *streamer) {
    streamer->CONFIG &= ~DS_CONFIG_SEST;
}

/*! \brief Tests if the sign extension is enabled.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return Value of the SEST bit.
 *   \retval true  Sign extension is enabled.
 *   \retval false Sign extension is disabled.
 */
static inline bool streamer_is_sign_extend_enabled(volatile amba_streamer_t *streamer) {
    return streamer->CONFIG & DS_CONFIG_SEST;
}

/*! \brief Tests if config indicates that the sign extension is enabled.
 *
 * \param config Content of the configuration register.
 *
 * \return Value of the SEST bit.
 *   \retval true  Sign extension is enabled.
 *   \retval false Sign extension is disabled.
 */
static inline bool streamer_is_sign_extend_enabled_from_config(uint32_t config) {
    return config & DS_CONFIG_SEST;
}

//------------------------------------------------------------------------------
// INFO
//------------------------------------------------------------------------------
//----
// All
//----
/*! \brief Returns Streamer information register.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return \c Streamer information register.
 */
static inline uint32_t streamer_get_info(volatile amba_streamer_t *streamer) {
    return streamer->INFO;
}

//-----
// SIDW
//-----
/*! \brief Returns streamer interface data width.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return \c Streamer interface data width.
 */
static inline uint16_t streamer_get_stream_interface_data_width(volatile amba_streamer_t *streamer) {
    return (streamer->INFO & DS_INFO_SIDW_MASK) >> DS_INFO_SIDW_SHIFT;
}

/*! \brief Returns streamer interface data width.
 *
 * \param info Content of the information register.
 *
 * \return \c Streamer interface data width.
 */
static inline uint16_t streamer_get_stream_interface_data_width_from_info(uint32_t info) {
    return (info & DS_INFO_SIDW_MASK) >> DS_INFO_SIDW_SHIFT;
}

//------
// FIFOD
//------
/*! \brief Returns a FIFO word width.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return \c FIFO word width.
 */
static inline uint16_t streamer_get_fifo_word_width(volatile amba_streamer_t *streamer) {
    return (streamer->INFO & DS_INFO_FIFOD_MASK) >> DS_INFO_FIFOD_SHIFT;
}

/*! \brief Returns a FIFO word width.
 *
 * \param info Content of the information register.
 *
 * \return \c FIFO word width.
 */
static inline uint16_t streamer_get_fifo_word_width_from_info(uint32_t info) {
    return (info & DS_INFO_FIFOD_MASK) >> DS_INFO_FIFOD_SHIFT;
}

//-------
// FIFOWD
//-------
/*! \brief Returns a FIFO depth.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return \c FIFO depth.
 */
static inline uint16_t streamer_get_fifo_depth(volatile amba_streamer_t *streamer) {
    return (streamer->INFO & DS_INFO_FIFOWD_MASK) >> DS_INFO_FIFOWD_SHIFT;
}

/*! \brief Returns a FIFO depth.
 *
 * \param info Content of the information register.
 *
 * \return \c FIFO depth.
 */
static inline uint16_t streamer_get_fifo_depth_from_info(uint32_t info) {
    return (info & DS_INFO_FIFOWD_MASK) >> DS_INFO_FIFOWD_SHIFT;
}

//------------------------------------------------------------------------------
// ADDRESS
//------------------------------------------------------------------------------
/*! \brief Sets Streamer address register.
 *
 * \param streamer Base address of the Streamer instance.
 * \param address  Memory start address.
 *
 * This function additionally checks if address is aligned to the stream sample size.
 *
 * \return Operation status.
 *   \retval true  ERROR: Address is not aligned to the sample size.
 *   \retval false OK
 */
bool streamer_set_address(volatile amba_streamer_t *streamer, uint32_t address);

/*! \brief Sets Streamer address register.
 *
 * \param streamer Base address of the Streamer instance.
 * \param address  Memory start address.
 *
 * Be aware, this function does not check if address is aligned to the stream sample size.
 */
static inline void _streamer_set_address(volatile amba_streamer_t *streamer, uint32_t address) {
    streamer->ADDRESS = address;
}

/*! \brief Returns a start address from which streamer will read data.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return \c Memory start address.
 */
static inline uint32_t streamer_get_address(volatile amba_streamer_t *streamer) {
    return streamer->ADDRESS;
}

//------------------------------------------------------------------------------
// ADDRESSREL
//------------------------------------------------------------------------------
/*! \brief Sets Streamer reload address register.
 *
 * \param streamer       Base address of the Streamer instance.
 * \param reload_address Memory start reload address.
 *
 * This function additionally checks if reload address is aligned to the stream sample size.
 *
 * \return Operation status.
 *   \retval true  ERROR: Reload address is not aligned to the sample size.
 *   \retval false OK
 */
bool streamer_set_reload_address(volatile amba_streamer_t *streamer, uint32_t reload_address);

/*! \brief Sets Streamer reload address register.
 *
 * \param streamer       Base address of the Streamer instance.
 * \param reload_address Memory start reload address.
 *
 * Be aware, this function does not check if reload address is aligned to the stream sample size.
 */
static inline void _streamer_set_reload_address(volatile amba_streamer_t *streamer, uint32_t reload_address) {
    streamer->ADDRESSREL = reload_address;
}

/*! \brief Returns a start reload address from which streamer will read data.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return \c Memory start reload address.
 */
static inline uint32_t streamer_get_reload_address(volatile amba_streamer_t *streamer) {
    return streamer->ADDRESSREL;
}

//------------------------------------------------------------------------------
// COUNTER
//------------------------------------------------------------------------------
/*! \brief Sets Streamer counter register.
 *
 * \param streamer Base address of the Streamer instance.
 * \param counter  Sample counter.
 *
 * This function additionally checks if counter is not too big.
 *
 * \return Operation status.
 *   \retval true  ERROR: Counter is too big.
 *   \retval false OK
 */
bool streamer_set_counter(volatile amba_streamer_t *streamer, uint32_t counter);

/*! \brief Sets Streamer counter register.
 *
 * \param streamer Base address of the Streamer instance.
 * \param counter  Sample counter.
 *
 * Be aware, this function does not check if sample counter is too big.
 */
static inline void _streamer_set_counter(volatile amba_streamer_t *streamer, uint32_t counter) {
    streamer->COUNTER = counter;
}

/*! \brief Returns a sample counter (number of samples to stream).
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return \c Sample counter.
 */
static inline uint32_t streamer_get_counter(volatile amba_streamer_t *streamer) {
    return streamer->COUNTER;
}

//------------------------------------------------------------------------------
// COUNTERREL
//------------------------------------------------------------------------------
/*! \brief Sets Streamer reload counter register.
 *
 * \param streamer       Base address of the Streamer instance.
 * \param reload_counter Sample reload counter.
 *
 * This function additionally checks if reload counter is not too big.
 *
 * \return Operation status.
 *   \retval true  ERROR: Reload counter is too big.
 *   \retval false OK
 */
bool streamer_set_reload_counter(volatile amba_streamer_t *streamer, uint32_t reload_counter);

/*! \brief Sets Streamer reload counter register.
 *
 * \param streamer       Base address of the Streamer instance.
 * \param reload_counter Sample reload counter.
 *
 * Be aware, this function does not check if sample reload counter is too big.
 */
static inline void _streamer_set_reload_counter(volatile amba_streamer_t *streamer, uint32_t reload_counter) {
    streamer->COUNTERREL = reload_counter;
}

/*! \brief Returns a sample reload counter (number of samples to stream after address reload).
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return \c Sample reload counter.
 */
static inline uint32_t streamer_get_reload_counter(volatile amba_streamer_t *streamer) {
    return streamer->COUNTERREL;
}

//------------------------------------------------------------------------------
// IRQMASK
//------------------------------------------------------------------------------
//----
// All
//----
/*! \brief Sets Streamer interrupt mask register.
 *
 * \param streamer Base address of the Streamer instance.
 * \param irq_mask Content of the interrupt mask register.
 */
static inline void streamer_set_interrupt_mask(volatile amba_streamer_t *streamer, uint32_t irq_mask) {
    streamer->IRQMASK = irq_mask;
}

/*! \brief Returns Streamer interrupt mask register.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return \c Streamer interrupt mask register.
 */
static inline uint32_t streamer_get_interrupt_mask(volatile amba_streamer_t *streamer) {
    return streamer->IRQMASK;
}

//------
// SCZIE
//------
/*! \brief Enables sample counter zero interrupt in the Streamer.
 *
 * \param streamer Base address of the Streamer instance.
 */
static inline void streamer_enable_SCZ_interrupt(volatile amba_streamer_t *streamer) {
    streamer->IRQMASK |= DS_IRQMASK_SCZIE;
}

/*! \brief Disables sample counter zero interrupt in the Streamer.
 *
 * \param streamer Base address of the Streamer instance.
 */
static inline void streamer_disable_SCZ_interrupt(volatile amba_streamer_t *streamer) {
    streamer->IRQMASK &= ~DS_IRQMASK_SCZIE;
}

/*! \brief Tests if the sample counter zero interrupt is enabled.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return Value of the SCZIE bit.
 *   \retval true  Sample counter zero interrupt is enabled.
 *   \retval false Sample counter zero interrupt is disabled.
 */
static inline bool streamer_is_SCZ_interrupt_enabled(volatile amba_streamer_t *streamer) {
    return streamer->IRQMASK & DS_IRQMASK_SCZIE;
}

/*! \brief Tests if interrupt mask indicates that the sample counter zero interrupt is enabled.
 *
 * \param mask Content of the interrupt mask register.
 *
 * \return Value of the SCZIE bit.
 *   \retval true  Sample counter zero interrupt is enabled.
 *   \retval false Sample counter zero interrupt is disabled.
 */
static inline bool streamer_is_SCZ_interrupt_enabled_from_mask(uint32_t mask) {
    return mask & DS_IRQMASK_SCZIE;
}

//-------
// SRCZIE
//-------
/*! \brief Enables sample reload counter zero interrupt in the Streamer.
 *
 * \param streamer Base address of the Streamer instance.
 */
static inline void streamer_enable_SRCZ_interrupt(volatile amba_streamer_t *streamer) {
    streamer->IRQMASK |= DS_IRQMASK_SRCZIE;
}

/*! \brief Disables sample reload counter zero interrupt in the Streamer.
 *
 * \param streamer Base address of the Streamer instance.
 */
static inline void streamer_disable_SRCZ_interrupt(volatile amba_streamer_t *streamer) {
    streamer->IRQMASK &= ~DS_IRQMASK_SRCZIE;
}

/*! \brief Tests if the sample reload counter zero interrupt is enabled.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return Value of the SRCZIE bit.
 *   \retval true  Sample reload counter zero interrupt is enabled.
 *   \retval false Sample reload counter zero interrupt is disabled.
 */
static inline bool streamer_is_SRCZ_interrupt_enabled(volatile amba_streamer_t *streamer) {
    return streamer->IRQMASK & DS_IRQMASK_SRCZIE;
}

/*! \brief Tests if interrupt mask indicates that the sample reload counter zero interrupt is enabled.
 *
 * \param mask Content of the interrupt mask register.
 *
 * \return Value of the SRCZIE bit.
 *   \retval true  Sample reload counter zero interrupt is enabled.
 *   \retval false Sample reload counter zero interrupt is disabled.
 */
static inline bool streamer_is_SRCZ_interrupt_enabled_from_mask(uint32_t mask) {
    return mask & DS_IRQMASK_SRCZIE;
}

//------
// MARIE
//------
/*! \brief Enables memory address reload interrupt in the Streamer.
 *
 * \param streamer Base address of the Streamer instance.
 */
static inline void streamer_enable_MAR_interrupt(volatile amba_streamer_t *streamer) {
    streamer->IRQMASK |= DS_IRQMASK_MARIE;
}

/*! \brief Disables memory address reload interrupt in the Streamer.
 *
 * \param streamer Base address of the Streamer instance.
 */
static inline void streamer_disable_MAR_interrupt(volatile amba_streamer_t *streamer) {
    streamer->IRQMASK &= ~DS_IRQMASK_MARIE;
}

/*! \brief Tests if the memory address reload interrupt is enabled.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return Value of the MARIE bit.
 *   \retval true  Memory address reload interrupt is enabled.
 *   \retval false Memory address reload interrupt is disabled.
 */
static inline bool streamer_is_MAR_interrupt_enabled(volatile amba_streamer_t *streamer) {
    return streamer->IRQMASK & DS_IRQMASK_MARIE;
}

/*! \brief Tests if interrupt mask indicates that the memory address reload interrupt is enabled.
 *
 * \param mask Content of the interrupt mask register.
 *
 * \return Value of the MARIE bit.
 *   \retval true  Memory address reload interrupt is enabled.
 *   \retval false Memory address reload interrupt is disabled.
 */
static inline bool streamer_is_MAR_interrupt_enabled_from_mask(uint32_t mask) {
    return mask & DS_IRQMASK_MARIE;
}

//-------
// MERRIE
//-------
/*! \brief Enables memory access error interrupt in the Streamer.
 *
 * \param streamer Base address of the Streamer instance.
 */
static inline void streamer_enable_MERR_interrupt(volatile amba_streamer_t *streamer) {
    streamer->IRQMASK |= DS_IRQMASK_MERRIE;
}

/*! \brief Disables memory access error interrupt in the Streamer.
 *
 * \param streamer Base address of the Streamer instance.
 */
static inline void streamer_disable_MERR_interrupt(volatile amba_streamer_t *streamer) {
    streamer->IRQMASK &= ~DS_IRQMASK_MERRIE;
}

/*! \brief Tests if the memory access error interrupt is enabled.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * \return Value of the MERRIE bit.
 *   \retval true  Memory access error interrupt is enabled.
 *   \retval false Memory access error interrupt is disabled.
 */
static inline bool streamer_is_MERR_interrupt_enabled(volatile amba_streamer_t *streamer) {
    return streamer->IRQMASK & DS_IRQMASK_MERRIE;
}

/*! \brief Tests if interrupt mask indicates that the memory access error interrupt is enabled.
 *
 * \param mask Content of the interrupt mask register.
 *
 * \return Value of the MERRIE bit.
 *   \retval true  Memory access error interrupt is enabled.
 *   \retval false Memory access error interrupt is disabled.
 */
static inline bool streamer_is_MERR_interrupt_enabled_from_mask(uint32_t mask) {
    return mask & DS_IRQMASK_MERRIE;
}

//------------------------------------------------------------------------------
// Mixed Registers
//------------------------------------------------------------------------------
/*! \brief Sets Streamer address and reload address registers to the same value.
 *
 * \param streamer Base address of the Streamer instance.
 * \param address  Memory start address.
 *
 * This function additionally checks if address is aligned to the stream sample size.
 *
 * \return Operation status.
 *   \retval true  ERROR: Address is not aligned to the sample size.
 *   \retval false OK
 */
bool streamer_set_addresses(volatile amba_streamer_t *streamer, uint32_t address);

/*! \brief Sets Streamer address and reload address registers to the same value.
 *
 * \param streamer Base address of the Streamer instance.
 * \param address  Memory start address.
 *
 * Be aware, this function does not check if address is aligned to the stream sample size.
 */
static inline void _streamer_set_addresses(volatile amba_streamer_t *streamer, uint32_t address) {
    streamer->ADDRESSREL = address;
    streamer->ADDRESS    = address;
}

/*! \brief Sets Streamer counter and reload counter registers to the same value.
 *
 * \param streamer Base address of the Streamer instance.
 * \param counter  Sample counter.
 *
 * This function additionally checks if counter is not too big.
 *
 * \return Operation status.
 *   \retval true  ERROR: Counter is too big.
 *   \retval false OK
 */
bool streamer_set_counters(volatile amba_streamer_t *streamer, uint32_t counter);

/*! \brief Sets Streamer counter and reload counter registers to the same value.
 *
 * \param streamer Base address of the Streamer instance.
 * \param counter  Sample counter.
 *
 * Be aware, this function does not check if sample counter is too big.
 */
static inline void _streamer_set_counters(volatile amba_streamer_t *streamer, uint32_t counter) {
    streamer->COUNTERREL = counter;
    streamer->COUNTER    = counter;
}

//------------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------------
/*! \brief Check correctness of streamer configuration.
 *
 * \param streamer Base address of the Streamer instance.
 *
 * This function checks address alignment, counter size and sample size.
 *
 * \return Operation status.
 *   \retval STREAMER_OK                Configuration is correct.
 *   \retval STREAMER_SSS_TOO_BIG_ERR   Sample size is greater than the interface size.
 *   \retval STREAMER_ADDR_ALIGN_ERR    Address is not aligned to the sample size.
 *   \retval STREAMER_CNT_ZERO_ERR      Counter is set to 0.
 *   \retval STREAMER_CNT_OVRFLW_ERR    Counter is too big.
 *   \retval STREAMER_ADDRREL_ALIGN_ERR Reload address is not aligned to the sample size.
 *   \retval STREAMER_CNTREL_OVRFLW_ERR Reload counter is too big.
 *   \retval STREAMER_CNTREL_ZERO_ERR   Reload counter is set to 0, but RING is set.
 */
streamer_check_config_t streamer_check_config(volatile amba_streamer_t *streamer);

#endif // _STREAMER_H_

