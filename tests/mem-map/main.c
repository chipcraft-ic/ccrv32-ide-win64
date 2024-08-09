/*H*****************************************************************************
*
* Copyright (c) 2019 ChipCraft Sp. z o.o. All rights reserved
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
* File Name : main.c
* Author    : Krzysztof Marcinek
* ******************************************************************************
* $Date: 2023-08-01 12:38:13 +0200 (wto, 01 sie 2023) $
* $Revision: 976 $
*H*****************************************************************************/

#include "board.h"
#include <ccrv32.h>
#include <ccrv32-csr.h>
#include <ccrv32-pwd.h>
#include <ccrv32-amba.h>
#include <ccrv32-amba-rtc.h>
#include <rtc_util.h>
#include <stdio.h>

#include "test.h"

extern volatile uint32_t _flag;

#ifdef MEM_SIM_TEST
    #define VERBOSE 0
#else
    #define VERBOSE 1
#endif

void test_nonword_store_access(uint32_t start, uint32_t end, uint32_t size, uint32_t halfw){
    volatile uint32_t *ptr;
    volatile uint32_t data;
    uint32_t i;

    ptr = (uint32_t *)start;

    while (((uint32_t)ptr < end) && (((uint32_t)ptr > 0) || ((uint32_t)ptr == start))){
        if (VERBOSE) printf("0x%08x ",(unsigned int)ptr);
        for (i=0;i<size;i++){
            _flag = 0;

            if (halfw == 1){
                __asm__ __volatile__ (
                    "sh %0, 0(%1)\n\t"
                    : "=r" (data)
                    : "r" (ptr));
            }
            else{
                __asm__ __volatile__ (
                    "sb %0, 0(%1)\n\t"
                    : "=r" (data)
                    : "r" (ptr));
            }

            if (VERBOSE) putchar('-'); // exception
            ptr++;
            assertEq(_flag,1);
        }
        if (VERBOSE) putchar('\n');
    }
    //if (VERBOSE) putchar('\n');

}

void test_nonword_load_access(uint32_t start, uint32_t end, uint32_t size, uint32_t halfw){
    volatile uint32_t *ptr;
    volatile uint32_t data;
    uint32_t i;

    ptr = (uint32_t *)start;

    while (((uint32_t)ptr < end) && (((uint32_t)ptr > 0) || ((uint32_t)ptr == start))){
        if (VERBOSE) printf("0x%08x ",(unsigned int)ptr);
        for (i=0;i<size;i++){
            _flag = 0;

            if (halfw == 1){
                __asm__ __volatile__ (
                    "lh %0, 0(%1)\n\t"
                    : "=r" (data)
                    : "r" (ptr));
            }
            else{
                __asm__ __volatile__ (
                    "lb %0, 0(%1)\n\t"
                    : "=r" (data)
                    : "r" (ptr));
            }

            if (VERBOSE) putchar('-'); // exception
            ptr++;
            assertEq(_flag,1);
        }
        if (VERBOSE) putchar('\n');
    }
    //if (VERBOSE) putchar('\n');

}

void test_mem_access(uint32_t start, uint32_t end, uint32_t size){
    volatile uint32_t *ptr;
    volatile uint32_t data;
    uint32_t i;

    ptr = (uint32_t *)start;

    while (((uint32_t)ptr < end) && (((uint32_t)ptr > 0) || ((uint32_t)ptr == start))){
        if (VERBOSE) printf("0x%08x ",(unsigned int)ptr);
        for (i=0;i<size;i++){
            _flag = 0;

            __asm__ __volatile__ (
                "lw %0, 0(%1)\n\t"
                : "=r" (data)
                : "r" (ptr));

            if (VERBOSE){
                if (_flag == 0){
                    if (data != 0){
                        data = data ^ (data>>8) ^ (data>>16) ^ (data>>24);
                        data &= 0x7F;
                        if (data < 33)
                            data += 33;
                        if (data == '-')
                            data--;
                        if (data == '?')
                            data--;
                        if (data == '.')
                            data++;
                        if (data > 126)
                            data = '?';
                        putchar(data);
                    }
                    else
                        putchar('.');
                }
                else{
                    putchar('-'); // exception
                }
            }
            if (((uint32_t)ptr > 0xE3000000) && ((uint32_t)ptr < 0xF0000000)){ // amba
                assertEq(_flag,1);
            }
            if (((uint32_t)ptr > 0xC0000000) && ((uint32_t)ptr < 0xD0000000)){ // debug
                assertEq(_flag,1);
            }
            ptr++;
        }
        if (VERBOSE) putchar('\n');
    }
    //if (VERBOSE) putchar('\n');

}

int check_reset_cause(void){
    if (PWD_PTR->RSTRSN == PWD_RSN_SOFT) {
        RTCenable();
        g_totalTests = RTCread((uint32_t*)&AMBA_RTC_PTR->BACKUP0);
        g_failedTests = RTCread((uint32_t*)&AMBA_RTC_PTR->BACKUP1);
        printTestSummary();
        return 1;
    }
    return 0;
}

void test_ram_space(void){

    uint32_t *addr;
    uint32_t totalTests = 0;
    uint32_t failedTests = 0;
    uint32_t ram_end = RAM_BASE + csr_read(mconfig0);

    if ((AMBA_APB0_CFG_PTR->INFO_0 & AMBA_RTC) == 0){
        printf("\nNo RTC found to test RAM space!\n\n");
        return;
    }
    if (csr_read(mconfig0) > 0x80000){
        printf("\nToo large RAM size to test!\n\n");
        return;
    }

    RTCenable();

    printf("\nTesting RAM space.\n\n");

    totalTests = g_totalTests;
    failedTests = g_failedTests;

    for (addr=(uint32_t*)RAM_BASE; (int)addr < ram_end; addr++){
        *addr=(int)addr;
    }
    for (addr=(uint32_t*)RAM_BASE; (int)addr < ram_end; addr++){
        totalTests++;
        if (*addr != (int)addr) {
            failedTests++;
        }
    }

    RTCwrite((uint32_t*)&AMBA_RTC_PTR->BACKUP0,totalTests);
    RTCwrite((uint32_t*)&AMBA_RTC_PTR->BACKUP1,failedTests);

    PWD_PTR->PWDRST = PWD_RST_KEY;
    for (;;);

}

int main(void)
{

    if (check_reset_cause()) return 0;

    printf("Creating memory access map.\n\n");

    #ifdef MEM_SIM_TEST
        printf("MEM_SIM_TEST: defined.\n\n");
    #else
        printf("MEM_SIM_TEST: undefined.\n\n");
    #endif

    #ifdef MEM_SIM_TEST
        PWD_PTR->CTRL |= (1<<PWD_CTRL_PER2INT_SHIFT) | (2<<PWD_CTRL_PER0INT_SHIFT) | PWD_CTRL_KEY;
    #endif

    test_mem_access(0x00000000,0x00000504,64);
    test_mem_access(0x0FFFFF00,0x10000000,64);
    test_mem_access(0x10000000,0x10000504,64);

    test_mem_access(0x1FFFFF00,0x20000504,64);
    test_mem_access(0x2FFFFF00,0x30000504,64);
    test_mem_access(0x3FFFFF00,0x40000504,64);
    test_mem_access(0x4007FF00,0x40080204,64);
    test_mem_access(0x4FFFFF00,0x50000504,64);
    test_mem_access(0x5FFFFF00,0x60000504,64);
    test_mem_access(0x6FFFFF00,0x70000504,64);
    test_mem_access(0x7FFFFF00,0x80000504,64);
    test_mem_access(0x8FFFFF00,0x90000504,64);
    test_mem_access(0x9FFFFF00,0xA0000504,64);
    test_mem_access(0xAFFFFF00,0xB0000504,64);
    test_mem_access(0xBFFFFF00,0xC0000504,64);
    test_mem_access(0xCFFFFF00,0xD0000504,64);
    test_mem_access(0xDFFFFF00,0xE0000504,64);

    if ((AMBA_APB0_CFG_PTR->INFO_0 & 0x1f80) != 0){ // uart and wdt can be emulated, check for gpio
        test_mem_access(0xEFFFFF00,0xF0000504,64);
    }

    test_mem_access(0xF000FE00,0xF0010200,64);
    test_mem_access(0xF001FE00,0xF0020200,64);
    test_mem_access(0xF002FE00,0xF0030200,64);
    test_mem_access(0xF0031E00,0xF0032200,64);
    test_mem_access(0xF003FE00,0xF0040200,64);
    test_mem_access(0xF004FE00,0xF0050200,64);
    test_mem_access(0xF005FE00,0xF0060200,64);
    test_mem_access(0xF006FE00,0xF0070200,64);
    //test_mem_access(0xF0071E00,0xF0072200,64); <- do not read 0xF0072000 address in lockstep
    test_mem_access(0xF007FE00,0xF0080200,64);
    test_mem_access(0xF0087E00,0xF0088A00,64);

    if ((AMBA_APB0_CFG_PTR->INFO_0 & 0x1f80) != 0){ // uart and wdt can be emulated, check for gpio

        test_mem_access(0xE0000000,0xE0010004,64);

        test_mem_access(0xE0FFFF00,0xE1000104,64);

        test_mem_access(0xE1FFFF00,0xE2000104,64);

        test_mem_access(0xE2000000,0xE2000204,64);
        test_mem_access(0xE2000F00,0xE2003000,64);
        test_mem_access(0xE2FFFF00,0xE3000104,64);

        test_mem_access(0xEFFFFF00,0xF0000104,64);

    }

    printf("\nTesting non-word load access.\n\n");

    test_nonword_load_access(0xF000FE00,0xF0010200,64,0);
    test_nonword_load_access(0xF000FE00,0xF0010200,64,1);
    test_nonword_load_access(0x80000000,0x80000204,64,0);
    test_nonword_load_access(0x80000000,0x80000204,64,1);
    test_nonword_load_access(0x82000000,0x80000204,64,0);
    test_nonword_load_access(0x82000000,0x80000204,64,1);

    printf("\nTesting non-word store access.\n\n");

    test_nonword_store_access(0xF000FE00,0xF0010200,64,0);
    test_nonword_store_access(0xF000FE00,0xF0010200,64,1);
    test_nonword_store_access(0xE0000000,0xE0000204,64,0);
    test_nonword_store_access(0xE0000000,0xE0000204,64,1);
    test_nonword_store_access(0xE2000000,0xE0000204,64,0);
    test_nonword_store_access(0xE2000000,0xE0000204,64,1);

    test_ram_space();

    printf("\nDone.\n");

    printTestSummary();

    return 0;

}
