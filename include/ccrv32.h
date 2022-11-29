/* ----------------------------------------------------------------------
*
* Copyright (c) 2018 ChipCraft Sp. z o.o. All rights reserved
*
* $Date: 2022-11-24 15:32:16 +0100 (czw, 24 lis 2022) $
* $Revision: 916 $
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
 * @file            ccrv32.h
 * @brief           CCRV32 Processor Core Peripheral Access Layer Header File.
 * @author          Rafal Harabien
 *
 * @defgroup        CCRV32 Processor Core Modules
 * CC Processor core definitions
 * @{
 */

#ifndef __CCRV32_H
#define __CCRV32_H

#include <stdint.h>

/**
 * @cond
 * enable/disable static assertions
 */
#if 0
 #define STATIC_ASSERT(expr, msg) _Static_assert(expr, msg)
#else
 #define STATIC_ASSERT(expr, msg)
#endif
/** @endcond */

/** Memory layout */
enum memory_layout_t
{
    ROM_BASE        = 0x00000000,  /*!< ROM Memory base address                    */
    ROM_EXT_BASE    = 0x10000000,  /*!< External ROM Memory base address           */
    RAM_BASE        = 0x40000000,  /*!< RAM Memory base address                    */
    RAM_EXT_BASE    = 0x50000000,  /*!< External ROM Memory base address           */
    DEBUG_BASE      = 0xC0000000,  /*!< DEBUGGER base address                      */
    SPRAM_BASE      = 0xD0000000,  /*!< Scratch-Pad RAM Memory base address        */
    AMBA_BASE       = 0xE0000000,  /*!< AMBA Bus base address                      */
    PERIPH_BASE     = 0xF0000000,  /*!< Core Peripherials base address             */
    MCORE_BASE      = 0xF0010000,  /*!< Multi-Core Controller base address         */
    PWD_BASE        = 0xF0020000,  /*!< Power Controller base address              */
    CSR_BASE        = 0xF0030000,  /*!< CSR base address                           */
    PLIC_BASE       = 0xF0038000,  /*!< PLIC base address                          */
    ICORE_BASE      = 0xF0038200,  /*!< ICORE base address                         */
    CLINT_BASE      = 0xF003C000,  /*!< CLINT base address                         */
    GNSS_BASE       = 0xF0040000,  /*!< GNSS Controller base address               */
    ICACHE_BASE     = 0xF0070000,  /*!< Instruction Cache Controller base address  */
    DCACHE_BASE     = 0xF0072000,  /*!< Data Cache Controller base address         */
    FFT_BASE        = 0xF0080000,  /*!< FFT Controller base address                */
    MBIST_BASE      = 0xF0090000,  /*!< Memory BIST Controller base address        */
    VITERBI_BASE    = 0xF00A0000,  /*!< Viterbi Decoder Controller base address    */
    DBGIFACE_BASE   = 0xF00B0000,  /*!< Debug Interface Controller base address    */
};

/** Default interrupt numbers */
enum
{
    EXCEPTION_IRQn     = 0,  /*!< Default Interrupt Number of Exceptions            */
    AMBA_UART0_IRQn    = 1,  /*!< Default Interrupt Number of UART 0 Controller     */
    AMBA_UART1_IRQn    = 2,  /*!< Default Interrupt Number of UART 1 Controller     */
    AMBA_UART2_IRQn    = 3,  /*!< Default Interrupt Number of UART 2 Controller     */
    AMBA_UART3_IRQn    = 4,  /*!< Default Interrupt Number of UART 3 Controller     */
    AMBA_UART4_IRQn    = 1,  /*!< Default Interrupt Number of UART 5 Controller     */
    AMBA_UART5_IRQn    = 2,  /*!< Default Interrupt Number of UART 6 Controller     */
    AMBA_UART6_IRQn    = 3,  /*!< Default Interrupt Number of UART 7 Controller     */
    AMBA_UART7_IRQn    = 4,  /*!< Default Interrupt Number of UART 8 Controller     */
    AMBA_SPI0_IRQn     = 5,  /*!< Default Interrupt Number of SPI 0 Controller      */
    AMBA_SPI1_IRQn     = 6,  /*!< Default Interrupt Number of SPI 1 Controller      */
    AMBA_SPI2_IRQn     = 5,  /*!< Default Interrupt Number of SPI 2 Controller      */
    AMBA_SPI3_IRQn     = 6,  /*!< Default Interrupt Number of SPI 3 Controller      */
    AMBA_GPIO0_IRQn    = 7,  /*!< Default Interrupt Number of GPIO Controller       */
    AMBA_OCCAN_IRQn    = 6,  /*!< Default Interrupt Number of OC CAN Controller     */
    AMBA_DMA_IRQn      = 8,  /*!< Default Interrupt Number of DMA Controller        */
    AMBA_TIMER32_IRQn  = 9,  /*!< Default Interrupt Number of 32-bit Timers         */
    AMBA_TIMER16_IRQn  = 10, /*!< Default Interrupt Number of 16-bit Timers         */
    AMBA_STT_IRQn      = 11, /*!< Default Interrupt Number of Systick Timer         */
    AMBA_RTC_IRQn      = 12, /*!< Default Interrupt Number of Real-Time Clock       */
    AMBA_BLE0_IRQn     = 11, /*!< Default Interrupt Number of BLE 0 Controller      */
    AMBA_BLE1_IRQn     = 12, /*!< Default Interrupt Number of BLE 1 Controller      */
    AMBA_ADC_IRQn      = 13, /*!< Default Interrupt Number of ADC Controller        */
    AMBA_I2C_MST_IRQn  = 14, /*!< Default Interrupt Number of I2C Master Controller */
    AMBA_I2C_SLV_IRQn  = 5,  /*!< Default Interrupt Number of I2C Slave Controller  */
    AMBA_PROM_IRQn     = 15, /*!< Default Interrupt Number of PROM Controller       */
    AMBA_1WIRE_IRQn    = 7,  /*!< Default Interrupt Number of 1 Wire Controller     */
    ICORE_IRQn         = 15, /*!< Default Interrupt Number of Inter-Core Interrupt  */
    FFT_IRQn           = 4,  /*!< Default Interrupt Number of FFT Controller        */
    GNSS_IRQn          = 4,  /*!< Default Interrupt Number of GNSS Controller       */
};

/**
 * @name Utils
 * @{
 */

/** Software Breakpoint */
#define BREAKPOINT() __asm__ __volatile__("ebreak")

/** End Simulation */
#define EXIT() __asm__ __volatile__("jal end_loop")

#ifndef _NO_SPRAM_MEMORY
/** Puts variable into Scratchpad RAM */
#define SPRAM_DATA __attribute__ ((section (".spram_data")))
#define SPRAM_BSS  __attribute__ ((section (".spram_bss")))
#endif /* _NO_SPRAM_MEMORY */
/** @} */

#endif /* __CCRV32_H */
/** @} */
