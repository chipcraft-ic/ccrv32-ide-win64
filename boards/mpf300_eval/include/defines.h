/* ----------------------------------------------------------------------
*
* Copyright (c) 2019 ChipCraft Sp. z o.o. All rights reserved
*
* $Date: 2025-01-11 22:33:12 +0100 (sob, 11 sty 2025) $
* $Revision: 1126 $
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
#define IDCODE_PART_NUM         0
#define IDCODE_PART_VER         0

/* EXTERNAL MT25QU01 MEMORY */
#define FLASH_SIZE              (-1)//(1024*1024*128)
#define FLASH_PAGE_SIZE         256

#define RAM_SIZE                655360 // must be specified if not power of 2

#define PERIPH0_FREQ            50000000UL
#define PERIPH2_FREQ            50000000UL
#define CORE_FREQ               50000000UL

#define STDIO_UART              0
#define STDIO_BAUDRATE          921600
#define STDIO_RTSCTS            0

#define TIME_SOURCE             TIMER32
#define TIME_SOURCE_TIMER_ID    0
#define TIME_SOURCE_PRSC        ((PERIPH0_FREQ/1000000)-1)

#define MAX2771_SPI_NUM         1
#define MAX2771_L1E1_CS         1
#define MAX2771_L5E5_CS         2
#define MAX2771_L2E6_CS         3

#endif //* _BOARD_DEFINES_H */

