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
 * File Name : streamer.c
 * Author    : Sebastian Cieslak
 * ******************************************************************************
 * $Date: 2022-01-19 09:38:48 +0100 (śro, 19 sty 2022) $
 * $Revision: 814 $
 *H*****************************************************************************/

#include "streamer.h"

void streamer_flush_and_stop(volatile amba_streamer_t *streamer) {
    // Stop loading and streaming and flush the streamer
    _streamer_flush_and_stop(streamer);

    // Wait for the BUSY == 0
    while(streamer_is_busy_from_status(streamer_get_status(streamer)));

    // Flush again, because the first flush could occure in the middle of an AHB
    // transfer that will be finished.
    _streamer_flush_and_stop(streamer);
}

bool streamer_set_config(volatile amba_streamer_t *streamer, uint32_t config) {
    // Check if sample_size is greater than the stream interface
    if ((1 << ((config & DS_CONFIG_SSS_MASK) >> DS_CONFIG_SSS_SHIFT)) > streamer_get_stream_interface_data_width(streamer)) {
        return 1;
    }
    else {
        _streamer_set_config(streamer, config);
        return 0;
    }
}

bool streamer_set_sample_size(volatile amba_streamer_t *streamer, uint8_t sample_size) {
    // Check if sample_size is greater than the stream interface
    if ((1 << sample_size) > streamer_get_stream_interface_data_width(streamer)) {
        return 1;
    }
    else {
        _streamer_set_sample_size(streamer, sample_size);
        return 0;
    }
}

bool streamer_set_address(volatile amba_streamer_t *streamer, uint32_t address) {
    // Get a sample size value
    uint8_t sample_size = streamer_get_sample_size(streamer);

    // If a sample size is byte or smaller address will be always aligned or
    // if a sample size is greater than byte address must be aligned to it
    if ((sample_size <= 3) || ((address & ((1 << (sample_size - 3)) - 1)) == 0)) {
        _streamer_set_address(streamer, address);
        return 0;
    }
    else {
        return 1;
    }
}

bool streamer_set_reload_address(volatile amba_streamer_t *streamer, uint32_t reload_address) {
    // Get a sample size value
    uint8_t sample_size = streamer_get_sample_size(streamer);

    // If a sample size is byte or smaller address will be always aligned or
    // if a sample size is greater than byte address must be aligned to it
    if ((sample_size <= 3) || ((reload_address & ((1 << (sample_size - 3)) - 1)) == 0)) {
        _streamer_set_reload_address(streamer, reload_address);
        return 0;
    }
    else {
        return 1;
    }
}

bool streamer_set_addresses(volatile amba_streamer_t *streamer, uint32_t address) {
    // Get a sample size value
    uint8_t sample_size = streamer_get_sample_size(streamer);

    // If a sample size is byte or smaller address will be always aligned or
    // if a sample size is greater than byte address must be aligned to it
    if ((sample_size <= 3) || ((address & ((1 << (sample_size - 3)) - 1)) == 0)) {
        _streamer_set_addresses(streamer, address);
        return 0;
    }
    else {
        return 1;
    }
}

bool streamer_set_counter(volatile amba_streamer_t *streamer, uint32_t counter) {
    // Get a sample size value
    uint8_t sample_size = streamer_get_sample_size(streamer);

    // If a sample size is byte or smaller counter will not overflow or
    // if a sample size is greater than byte counter must be smaller than 2^32B
    if ((sample_size <= 3) || ((counter & (~(0xFFFFFFFF >> (sample_size - 3)))) == 0)) {
        _streamer_set_counter(streamer, counter);
        return 0;
    }
    else {
        return 1;
    }
}

bool streamer_set_reload_counter(volatile amba_streamer_t *streamer, uint32_t reload_counter) {
    // Get a sample size value
    uint8_t sample_size = streamer_get_sample_size(streamer);

    // If a sample size is byte or smaller counter will not overflow or
    // if a sample size is greater than byte counter must be smaller than 2^32B
    if ((sample_size <= 3) || ((reload_counter & (~(0xFFFFFFFF >> (sample_size - 3)))) == 0)) {
        _streamer_set_reload_counter(streamer, reload_counter);
        return 0;
    }
    else {
        return 1;
    }
}

bool streamer_set_counters(volatile amba_streamer_t *streamer, uint32_t counter) {
    // Get a sample size value
    uint8_t sample_size = streamer_get_sample_size(streamer);

    // If a sample size is byte or smaller counter will not overflow or
    // if a sample size is greater than byte counter must be smaller than 2^32B
    if ((sample_size <= 3) || ((counter & (~(0xFFFFFFFF >> (sample_size - 3)))) == 0)) {
        _streamer_set_counters(streamer, counter);
        return 0;
    }
    else {
        return 1;
    }
}

streamer_check_config_t streamer_check_config(volatile amba_streamer_t *streamer) {
    // Get a sample size value
    uint8_t sample_size = streamer_get_sample_size(streamer);

    // Check if sample_size is greater than the stream interface
    if ((1 << sample_size) > streamer_get_stream_interface_data_width(streamer)) {
        return STREAMER_SSS_TOO_BIG_ERR;
    }

    // Check if address is aligned to the sample_size
    if (!((sample_size <= 3) || ((streamer_get_address(streamer) & ((1 << (sample_size - 3)) - 1)) == 0))) {
        return STREAMER_ADDR_ALIGN_ERR;
    }

    // Get a counter value
    uint32_t counter = streamer_get_counter(streamer);

    // Check if counter is greater than 0
    if (counter == 0) {
        return STREAMER_CNT_ZERO_ERR;
    }

    // Check if counter is not too big: counter * (1<<sss) < 2^32B
    if (!((sample_size <= 3) || ((counter & (~(0xFFFFFFFF >> (sample_size - 3)))) == 0))) {
        return STREAMER_CNT_OVRFLW_ERR;
    }

    // Get a reaload counter value
    counter = streamer_get_reload_counter(streamer);

    // Check if a reload counter is not zero
    bool reload_counter_is_not_zero = (counter > 0);

    // Check if reload address is aligned to the sample_size
    if (reload_counter_is_not_zero && !((sample_size <= 3) || ((streamer_get_reload_address(streamer) & ((1 << (sample_size - 3)) - 1)) == 0))) {
        return STREAMER_ADDRREL_ALIGN_ERR;
    }

    // Check if reload counter is not too big: counter * (1<<sss) < 2^32B
    if (!((sample_size <= 3) || ((counter & (~(0xFFFFFFFF >> (sample_size - 3)))) == 0))) {
        return STREAMER_CNTREL_OVRFLW_ERR;
    }

    // Check if reload counter is not zero when RING is set
    if ((!reload_counter_is_not_zero) && streamer_is_ring_buffer_enabled(streamer)) {
        return STREAMER_CNTREL_ZERO_ERR;
    }

    // Configuration is ok
    return STREAMER_OK;
}

