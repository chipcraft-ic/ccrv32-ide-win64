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
* $Date: 2024-09-04 12:09:10 +0200 (śro, 04 wrz 2024) $
* $Revision: 1104 $
*H*****************************************************************************/

#include "board.h"
#include <ccrv32.h>
#include <ccrv32-csr.h>
#include <ccrv32-utils.h>
#include <ccrv32-pwd.h>
#include <ccrv32-mcore.h>
#include <ccrv32-dcache.h>
#include <ccrv32-mbist.h>
#include <ccrv32-amba.h>
#include <ccrv32-amba-rtc.h>
#include <core_util.h>
#include <rtc_util.h>
#include <stdio.h>
#include "test.h"

void singleTests(void)
{

    uint32_t cpu_info_0 = csr_read(mconfig0);
    uint32_t cpu_info_1 = csr_read(mconfig1);
    //uint32_t cpu_info_2 = csr_read(mconfig2);
    uint32_t fpu_num = (cpu_info_1 & CPU_FPU_MASK) >> CPU_FPU_SHIFT;

    int dctag_region = -128;
    if (cpu_info_1 & CPU_DCACHE)
    {
        // skip test in low-power data cache, tags could be too small for error injection
        if (((DCACHE_PTR->INFO & DCACHE_IMPL_MASK) >> DCACHE_IMPL_SHIFT) != DCACHE_IMPL_LP)
            dctag_region = 2;
    }
    if (fpu_num > 0)
        dctag_region += 1;
    if (cpu_info_0 & CPU_ICACHE)
        dctag_region += 2;

    if (PWD_PTR->RSTRSN == PWD_RSN_MBIST){

        g_failedTests = MBIST_PTR->SCRATCH0;
        g_totalTests = MBIST_PTR->SCRATCH1;

        if ((MBIST_PTR->CTRL & MBIST_CTRL_ALG_MASK) >> MBIST_CTRL_ALG_SHIFT != MBIST_ALG_ZERO_ONE || g_totalTests == 0){
            if ((MBIST_PTR->CTRL & MBIST_CTRL_ALG_MASK) >> MBIST_CTRL_ALG_SHIFT != MBIST_ALG_MARCH_D2PF) {
                return;
            }
        }

        uint8_t region = (MBIST_PTR->CTRL & MBIST_CTRL_REGN_MASK) >> MBIST_CTRL_REGN_SHIFT;
        uint8_t second_port = (MBIST_PTR->CTRL & MBIST_CTRL_SECOND_PORT) != 0;
        uint8_t switch_ports = (MBIST_PTR->CTRL & MBIST_CTRL_SWITCH_PORTS) != 0;
        uint8_t inject_idx_port0 = (MBIST_PTR->INJ & MBIST_INJ_IDX0_PORT0_MASK) >> MBIST_INJ_IDX0_PORT0_SHIFT;
        //uint8_t inject_idx_port1 = (MBIST_PTR->INJ & MBIST_INJ_IDX0_PORT1_MASK) >> MBIST_INJ_IDX0_PORT1_SHIFT;
        if (region == 0 && MBIST_PTR->INJ_ADDR_PORT0 == 0){

                            // enable injection   first error index                          second error index
            MBIST_PTR->INJ = MBIST_INJ_PORT0_EN | (region+1) << MBIST_INJ_IDX0_PORT0_SHIFT | (region+2) << MBIST_INJ_IDX1_PORT0_SHIFT;

            MBIST_PTR->INJ_ADDR_PORT0 = (region+1)*2;
            MBIST_PTR->CTRL = MBIST_CTRL_SINGLE;
            MEMORY_BARRIER();
            MBIST_PTR->RUN = MBIST_RUN_KEY;
            for(;;);
        }
        else if (region == 0 && second_port == 0){
            printf("\nSingle run tests - Register File Port0.\n");
            for (int i=0; i<26; i++)
            {
                //printf("REG0 %d: 0x%08x\n",i,(unsigned)MBIST_PTR->DET_LOGS[i]);
            }
            assertTrue(MBIST_PTR->ERRCNT_PORT0 != 0); // error count > 0
            assertTrue(MBIST_PTR->FIRST_ERR_PORT0[0] == MBIST_PTR->INJ_ADDR_PORT0); // error address matches
            assertTrue((MBIST_PTR->FIRST_IDX_PORT0[0] & 0xFF) == inject_idx_port0); // error index matches
            assertTrue(MBIST_PTR->FIRST_IDX_PORT0[0] & MBIST_ERR_IDX0_SGL); // single error indicator

            // check only if hardware support multiple errors injection
            if (MBIST_PTR->INJ & MBIST_INJ_IDX1_PORT0_MASK)
            {
                assertTrue(MBIST_PTR->FIRST_IDX_PORT0[0] & MBIST_ERR_IDX0_MPL); // multiple error error indicator
            }

            MBIST_PTR->SCRATCH0 = g_failedTests;
            MBIST_PTR->SCRATCH1 = g_totalTests;

            MBIST_PTR->INJ = MBIST_INJ_PORT0_EN | 4*(region+1) << MBIST_INJ_IDX0_PORT0_SHIFT;
            MBIST_PTR->INJ_ADDR_PORT0 = (region+1)*5;
            MBIST_PTR->CTRL |= MBIST_CTRL_SECOND_PORT;
            MEMORY_BARRIER();
            MBIST_PTR->RUN = MBIST_RUN_KEY;
            for(;;);
        }
        else if (region == 0 && second_port == 1){
            printf("\nSingle run tests - Register File Port1.\n");
            for (int i=0; i<26; i++)
            {
                //printf("REG0 %d: 0x%08x\n",i,(unsigned)MBIST_PTR->DET_LOGS[i]);
            }
            assertTrue(MBIST_PTR->ERRCNT_PORT0 != 0); // error count > 0
            assertTrue(MBIST_PTR->FIRST_ERR_PORT0[0] == MBIST_PTR->INJ_ADDR_PORT0); // error address matches
            assertTrue((MBIST_PTR->FIRST_IDX_PORT0[0] & 0xFF) == inject_idx_port0); // error index matches
            assertTrue(MBIST_PTR->FIRST_IDX_PORT0[0] & MBIST_ERR_IDX0_SGL); // single error indicator
            assertFalse(MBIST_PTR->FIRST_IDX_PORT0[0] & MBIST_ERR_IDX0_MPL); // multiple error error indicator

            MBIST_PTR->SCRATCH0 = g_failedTests;
            MBIST_PTR->SCRATCH1 = g_totalTests;

            if (dctag_region > 0){
                region = dctag_region;
                MBIST_PTR->INJ = MBIST_INJ_PORT0_EN | (region+1) << MBIST_INJ_IDX0_PORT0_SHIFT;
                MBIST_PTR->INJ_ADDR_PORT0 = (region+1)*2;
                MBIST_PTR->CTRL = MBIST_CTRL_SINGLE | (region << MBIST_CTRL_REGN_SHIFT);
                MEMORY_BARRIER();
                MBIST_PTR->RUN = MBIST_RUN_KEY;
                for(;;);
            }
        }
        else if (region > 0 && switch_ports == 0 && ((MBIST_PTR->CTRL&MBIST_CTRL_ALG_MASK)>>MBIST_CTRL_ALG_SHIFT != MBIST_ALG_MARCH_D2PF)){
            printf("\nSingle run tests - Data Cache Tags Port0.\n");
            for (int i=0; i<26; i++)
            {
                //printf("REG0 %d: 0x%08x\n",i,(unsigned)MBIST_PTR->DET_LOGS[i]);
            }
            assertTrue(MBIST_PTR->ERRCNT_PORT0 != 0); // error count > 0
            assertTrue(MBIST_PTR->FIRST_ERR_PORT0[0] == MBIST_PTR->INJ_ADDR_PORT0); // error address matches
            assertTrue((MBIST_PTR->FIRST_IDX_PORT0[0] & 0xFF) == inject_idx_port0); // error index matches
            assertTrue(MBIST_PTR->FIRST_IDX_PORT0[0] & MBIST_ERR_IDX0_SGL); // single error indicator
            assertFalse(MBIST_PTR->FIRST_IDX_PORT0[0] & MBIST_ERR_IDX0_MPL); // multiple error error indicator

            MBIST_PTR->SCRATCH0 = g_failedTests;
            MBIST_PTR->SCRATCH1 = g_totalTests;

            MBIST_PTR->INJ = MBIST_INJ_PORT0_EN | (region+8) << MBIST_INJ_IDX0_PORT0_SHIFT;
            MBIST_PTR->INJ_ADDR_PORT0 = (region+1)*3;
            MBIST_PTR->CTRL |= MBIST_CTRL_SINGLE | MBIST_CTRL_SWITCH_PORTS;
            MEMORY_BARRIER();
            MBIST_PTR->RUN = MBIST_RUN_KEY;
            for(;;);
        }
        else if (region > 0 && switch_ports == 1){
            printf("\nSingle run tests - Data Cache Tags Port1.\n");
            for (int i=0; i<26; i++)
            {
                //printf("REG0 %d: 0x%08x\n",i,(unsigned)MBIST_PTR->DET_LOGS[i]);
            }
            assertTrue(MBIST_PTR->ERRCNT_PORT0 != 0); // error count > 0
            assertTrue(MBIST_PTR->FIRST_ERR_PORT0[0] == MBIST_PTR->INJ_ADDR_PORT0); // error address matches
            assertTrue((MBIST_PTR->FIRST_IDX_PORT0[0] & 0xFF) == inject_idx_port0); // error index matches
            assertTrue(MBIST_PTR->FIRST_IDX_PORT0[0] & MBIST_ERR_IDX0_SGL); // single error indicator
            assertFalse(MBIST_PTR->FIRST_IDX_PORT0[0] & MBIST_ERR_IDX0_MPL); // multiple error error indicator

#ifndef BOARD_CCNV2_B1

            MBIST_PTR->SCRATCH0 = g_failedTests;
            MBIST_PTR->SCRATCH1 = g_totalTests;

            // port1 does not issue write, so we cant inject error on the test generator side
            //MBIST_PTR->INJ = MBIST_INJ_PORT1_EN | (region+10) << MBIST_INJ_IDX0_PORT1_SHIFT;
            //MBIST_PTR->INJ_ADDR_PORT1 = (region+4)*3;
            MBIST_PTR->INJ = MBIST_INJ_PORT0_EN | (region+10) << MBIST_INJ_IDX0_PORT0_SHIFT;
            MBIST_PTR->INJ_ADDR_PORT0 = (region+4)*3;

            MBIST_PTR->CTRL = MBIST_ALG_MARCH_D2PF << MBIST_CTRL_ALG_SHIFT | MBIST_CTRL_SINGLE | (region << MBIST_CTRL_REGN_SHIFT);
            MEMORY_BARRIER();
            MBIST_PTR->RUN = MBIST_RUN_KEY;
            for(;;);

#endif

        }
        else if ((MBIST_PTR->CTRL&MBIST_CTRL_ALG_MASK)>>MBIST_CTRL_ALG_SHIFT == MBIST_ALG_MARCH_D2PF){
            printf("\nSingle run tests - Data Cache Tags d2PF.\n");
            for (int i=0; i<26; i++)
            {
                //printf("REG0 %d: 0x%08x\n",i,(unsigned)MBIST_PTR->DET_LOGS[i]);
            }
            assertTrue(MBIST_PTR->ERRCNT_PORT1 != 0); // error count > 0
            //assertTrue(MBIST_PTR->FIRST_ERR_PORT1[0] == MBIST_PTR->INJ_ADDR_PORT1); // error address matches
            //assertTrue((MBIST_PTR->FIRST_IDX_PORT1[0] & 0xFF) == inject_idx_port1); // error index matches
            assertTrue(MBIST_PTR->FIRST_ERR_PORT1[0] == MBIST_PTR->INJ_ADDR_PORT0); // error address matches
            assertTrue((MBIST_PTR->FIRST_IDX_PORT1[0] & 0xFF) == inject_idx_port0); // error index matches
            assertTrue(MBIST_PTR->FIRST_IDX_PORT1[0] & MBIST_ERR_IDX0_SGL); // single error indicator
            assertFalse(MBIST_PTR->FIRST_IDX_PORT1[0] & MBIST_ERR_IDX0_MPL); // multiple error error indicator
        }

    }

}

void prepareRTC(void)
{
#ifndef BOARD_CCNV2_A1
    if ((AMBA_APB0_CFG_PTR->INFO_0 & AMBA_RTC) != 0){
        RTCenable();
        RTCwrite((uint32_t*)&AMBA_RTC_PTR->PRES, 0);
        RTCwrite((uint32_t*)&AMBA_RTC_PTR->PER, 0xFFFFFFFF);
        RTCwrite((uint32_t*)&AMBA_RTC_PTR->COMPARE, 0xFFFFFFFF);
        RTCwrite((uint32_t*)&AMBA_RTC_PTR->COUNT, 0);
    }
#endif
}

void fullTests(void)
{

    uint32_t status, mask, core_num, i;
    uint32_t count = 0;
    uint32_t *ptr, *reg;

    core_num = MCORE_PTR->CORE_NUM;

    if (PWD_PTR->RSTRSN == PWD_RSN_MBIST){

#ifndef BOARD_CCNV2_A1
        if ((AMBA_APB0_CFG_PTR->INFO_0 & AMBA_RTC) != 0){
            count = RTCread((uint32_t*)&AMBA_RTC_PTR->COUNT);
        }
#endif

        g_failedTests = MBIST_PTR->SCRATCH0;
        g_totalTests = MBIST_PTR->SCRATCH1;

        if ((MBIST_PTR->CTRL&MBIST_CTRL_ALG_MASK)>>MBIST_CTRL_ALG_SHIFT == MBIST_ALG_ZERO_ONE && g_totalTests > 0){
            return;
        }
        if ((MBIST_PTR->CTRL&MBIST_CTRL_ALG_MASK)>>MBIST_CTRL_ALG_SHIFT == MBIST_ALG_MARCH_D2PF){
            return;
        }

        if (MBIST_PTR->EXT_LOGS == 0){
            assertTrue(1);
            printf("No external MBIST detected.\n");
        }
        else if (MBIST_PTR->EXT_LOGS == 3){
            assertTrue(1);
            printf("External MBIST ok.\n");
        }
        else {
            assertTrue(0);
            printf("External MBIST failed.\n");
        }

        ptr = (uint32_t*)&(MBIST_PTR->INT_LOGS);
        for (i=0;i<core_num;i++){
            reg = ptr + i;
            status = *(reg);
            reg = ptr + i + core_num;
            mask = *(reg);
            assertTrue(mask!=0);
            if (mask == status){
                assertTrue(1);
                printf("Internal Core %d MBIST ok - mask %d, status %d.\n",(unsigned int)i,(unsigned int)mask,(unsigned int)status);
            }
            else{
                assertTrue(0);
                printf("Internal Core %d MBIST failed - mask %d, status %d.\n",(unsigned int)i,(unsigned int)mask,(unsigned int)status);
            }
        }

        ptr = (uint32_t*)&(MBIST_PTR->MSC_LOGS);
        for (i=0;i<core_num;i++){
            reg = ptr + i;
            status = *(reg);
            reg = ptr + i + core_num;
            mask = *(reg);
            if (mask != 0){
                if (mask == status){
                    assertTrue(1);
                    printf("Misc. Core %d MBIST ok - mask %d, status %d.\n",(unsigned int)i,(unsigned int)mask,(unsigned int)status);
                }
                else{
                    assertTrue(0);
                    printf("Misc. Core %d MBIST failed - mask %d, status %d.\n",(unsigned int)i,(unsigned int)mask,(unsigned int)status);
                }
            }
            else{
                printf("No misc. Core %d MBIST regions detected.\n",(unsigned int)i);
            }
        }

#ifndef BOARD_CCNV2_A1
        if ((AMBA_APB0_CFG_PTR->INFO_0 & AMBA_RTC) != 0){
            double time = (double)count/32.7680;
            printf("\nMBIST time: %.5f ms\n\n",time);
        }
#endif

        MBIST_PTR->SCRATCH0 = g_failedTests;
        MBIST_PTR->SCRATCH1 = g_totalTests;

        if ((MBIST_PTR->CTRL&MBIST_CTRL_ALG_MASK)>>MBIST_CTRL_ALG_SHIFT == MBIST_ALG_ZERO_ONE){
            printf("\nAlgorithm: MATS+.\n");
            MBIST_PTR->CTRL = MBIST_ALG_MATS_P << MBIST_CTRL_ALG_SHIFT;
            MEMORY_BARRIER();
            prepareRTC();
            MBIST_PTR->RUN = MBIST_RUN_KEY;
            for(;;);
        }
        else if ((MBIST_PTR->CTRL&MBIST_CTRL_ALG_MASK)>>MBIST_CTRL_ALG_SHIFT == MBIST_ALG_MATS_P){
            printf("\nAlgorithm: March C-.\n");
            MBIST_PTR->CTRL = MBIST_ALG_MARCH_CM << MBIST_CTRL_ALG_SHIFT;
            MEMORY_BARRIER();
            prepareRTC();
            MBIST_PTR->RUN = MBIST_RUN_KEY;
            for(;;);
        }
        else if ((MBIST_PTR->CTRL&MBIST_CTRL_ALG_MASK)>>MBIST_CTRL_ALG_SHIFT == MBIST_ALG_MARCH_CM){
            //printTestSummary();
            //return;
            MBIST_PTR->CTRL = MBIST_ALG_ZERO_ONE << MBIST_CTRL_ALG_SHIFT;
            return;
        }

    }

    printf("Starting MBIST test.\n");
    printf("\nAlgorithm: Zero-one.\n");
    MBIST_PTR->SCRATCH0 = 0;
    MBIST_PTR->SCRATCH1 = 0;
    MBIST_PTR->CTRL = MBIST_ALG_ZERO_ONE << MBIST_CTRL_ALG_SHIFT;
    MEMORY_BARRIER();
    prepareRTC();
    MBIST_PTR->RUN = MBIST_RUN_KEY;
    for(;;);

}

int main(void)
{

    if ((csr_read(mconfig1) & CPU_MBIST) == 0){
        printf("No MBIST found!\n");
        printTestSummary();
        return 0;
    }
    if ((csr_read(mconfig0) & CPU_PWD) == 0){
        printf("Power management controller required to test MBIST!\n");
        printTestSummary();
        return 0;
    }

    // Disable lockstep mode if present
    if (lockstepDisable() == 0){
        printf("\nDisabling lockstep mode!\n");
    }

    fullTests();
#ifndef BOARD_CCNV2_A1
    singleTests();
#endif

    printTestSummary();

}
