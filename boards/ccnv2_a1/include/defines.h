/* ----------------------------------------------------------------------
*
* Copyright (c) 2019 ChipCraft Sp. z o.o. All rights reserved
*
* $Date: 2024-04-22 09:02:05 +0200 (pon, 22 kwi 2024) $
* $Revision: 1049 $
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

#ifndef _BOARD_DEFINES_H
#define _BOARD_DEFINES_H

/* DO NOT MODIFY */
#define IDCODE_PART_NUM         2
#define IDCODE_PART_VER         0
#define FLASH_SIZE              (2*1024*1024)
#define FLASH_PAGE_SIZE         8192
#define FLASH_DELAY             25UL // upper 3.472*7
#define FLASH_ENDURANCE         100000

#define RAM_SIZE                1572864

#define TCXO_FREQ               16368000UL
#define MCU_PLL_N               16

#define CORE_FREQ_DIV           1
#define APB0_FREQ_DIV           4
#define APB2_FREQ_DIV           4

#define MCU_FREQ                (TCXO_FREQ * MCU_PLL_N)
#define CORE_FREQ               (MCU_FREQ / CORE_FREQ_DIV)
#define PERIPH0_FREQ            (MCU_FREQ / APB0_FREQ_DIV)
#define PERIPH2_FREQ            (MCU_FREQ / APB2_FREQ_DIV)

#define DBG_BAUDRATE            921600

#define STDIO_UART              0
#define STDIO_BAUDRATE          460800
#define STDIO_RTSCTS            0

#define TIME_SOURCE             TIMER32
#define TIME_SOURCE_TIMER_ID    0
#define TIME_SOURCE_PRSC        ((PERIPH0_FREQ/1000000)-1)

#endif //* _BOARD_DEFINES_H */

