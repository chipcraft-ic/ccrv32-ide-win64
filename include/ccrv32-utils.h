/* ----------------------------------------------------------------------
*
* Copyright (c) 2018 ChipCraft Sp. z o.o. All rights reserved
*
* $Date: 2019-10-03 19:22:07 +0200 (czw, 03 paź 2019) $
* $Revision: 474 $
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
 * @file    ccrv32-utils.h
 * @brief   CCRV32 Processor Utility functions and macros
 * @author  Krzysztof Marcinek
 */
#pragma once

/** allowed TIME_SOURCE values */
#define RTC        1   /*!< RTC clock as time source           */
#define TIMER32    2   /*!< 32-bit timer as time source        */
#define TIMER16    3   /*!< 16-bit timer as time source        */
#define SYSTICK    4   /*!< Systick Timer as time source       */
#define MTIMER     5   /*!< Machine Timer as time source       */
#define NONE       6   /*!< No time source                     */

/** allowed TIME_SOURCE values message sting */
#define ALLOWED_TIME_SOURCE "Allowed TIME_SOURCE values: RTC TIMER32 TIMER16 SYSTICK MTIMER NONE"

/** helper macro to expand define as string */
#define STR_HELPER(x) #x
/** helper macro to expand define as string */
#define STR(x) STR_HELPER(x)

/** helper macro for emitting memory barrier */
#define MEMORY_BARRIER() __asm__ __volatile__ ("fence" ::: "memory")

/** helper macro for emitting nop instruction */
#define NOP() __asm__ __volatile__ ("nop")

/** helper macro for clearing the pipeline */
#define CLEAR_PIPELINE() __asm__ __volatile__ ("nop;nop;nop;nop;nop;nop;")

/** helper macro for wait-for-interrupt instruction */
#define WFI() __asm__ __volatile__ ("fence; wfi" ::: "memory");
