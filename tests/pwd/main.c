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
* $Date: 2022-01-19 09:38:48 +0100 (śro, 19 sty 2022) $
* $Revision: 814 $
*H*****************************************************************************/

#include "board.h"
#include <ccrv32.h>
#include <ccrv32-pwd.h>
#include <ccrv32-csr.h>
#include <ccrv32-mcore.h>
#include <ccrv32-dcache.h>
#include <ccrv32-clint.h>
#include <ccrv32-plic.h>
#include <ccrv32-amba.h>
#include <ccrv32-amba-timer.h>
#include <core_util.h>
#include <pwd_util.h>
#include <sys/lock.h>
#include <stdio.h>
#include "test.h"

__LOCK_INIT(static, int_lock);

static volatile int power_down = 0;
static volatile int core_power_down = 1;
static volatile int core_power_up = 1;
static volatile int icore_flag_count = 1;

void isr5(void)
{
    assertEq(AMBA_TIMER32_PTR(0)->IRQF,TIMER_OVFIF);
    AMBA_TIMER32_PTR(0)->IRQM = 0;
    AMBA_TIMER32_PTR(0)->IRQF = TIMER_OVFIF;
    power_down++;
}

void isr_soft(void){
    __lock_acquire(int_lock);
    assertEq(CLINT_PTR->MSIP[csr_read(mhartid)],1);
    icore_flag_count++;
    __lock_release(int_lock);
    CLINT_PTR->MSIP[csr_read(mhartid)] = 0;
}

static void testSingleCoreShutDown(void){
    power_down = 0;
    AMBA_TIMER32_PTR(0)->PER = 600;
    AMBA_TIMER32_PTR(0)->PRES = 60;
    AMBA_TIMER32_PTR(0)->COUNT = 0;
    AMBA_TIMER32_PTR(0)->CTRL = TIMER_CTRL_EN;
    AMBA_TIMER32_PTR(0)->IRQMAP = (1 << 5);
    AMBA_TIMER32_PTR(0)->IRQM = TIMER_OVFIE;

    PLIC_PTR->ENABLE |= (1 << 5);
    assertTrue(csr_read(mstatus) & MSTAT_MIE); // interrupts should be enabled in startup.S

    corePowerDown(1);

    if (PWD_PTR->INFO & PWD_INFO_COREPWD)
        assertEq(power_down,1);
    else{
        assertEq(1,1); // To mask unavailable function
        assertEq(1,1);
    }

}

static void testCoresProc(){

    csr_write(mie,csr_read(mie) | MIE_MSIE);
    assertTrue(csr_read(mstatus) & MSTAT_MIE); // interrupts should be enabled in startup.S

    while(power_down==0);
    while(core_power_down < csr_read(mhartid));
    core_power_down++;

    corePowerDown(1);

    while(icore_flag_count != MCORE_PTR->CORE_NUM);

    // core powered up again
    while(core_power_up < csr_read(mhartid));
    core_power_up++;
}

static void testMulticoreProc(){
    static volatile int i;

    while (power_down == 0);

    // wait a while and shutdown
    for(i=0;i<500;i++);
    while(core_power_down < csr_read(mhartid));
    core_power_down++;
    __lock_acquire(int_lock);

    //assertEq(PWD_PTR->CTRL&1,1);  // bit do not exist anymore
    if (PWD_PTR->INFO & PWD_INFO_MAINPWD)
        assertEq(PWD_PTR->STAT,1);
    else
        assertEq(1,1); // To mask unavailable function

    __lock_release(int_lock);

}

static void testShutDownProc(){
    // idle
    for(;;);
}

void testShutDown(void){
    static volatile int i;
    unsigned numOfCores, k, status;

    numOfCores = MCORE_PTR->CORE_NUM;

    // start cores
    power_down = 0;
    status = 1;
    for (k = 1; k < numOfCores; ++k){
        status <<= 1;
        status++;
        coreStart(k, &testShutDownProc, NULL);
    }
    for(i=0;i<500;++i);
    __lock_acquire(int_lock);
    assertEq(status,MCORE_PTR->STATUS);
    __lock_release(int_lock);

    // shutdown cores
    status--;
    MCORE_PTR->CORE_SHDN = status | MCORE_SHDN_KEY;
    for(i=0;i<500;++i);
    __lock_acquire(int_lock);
    assertEq(MCORE_PTR->STATUS,1);
    __lock_release(int_lock);

}

void testCores(void){
    static volatile int i;
    unsigned numOfCores, k;

    core_power_down = 1;
    core_power_up = 1;

    numOfCores = MCORE_PTR->CORE_NUM;

    // start cores
    power_down = 0;
    for (k = 1; k < numOfCores; ++k){
        coreStart(k, &testCoresProc, NULL);
    }
    for(i=0;i<500;++i);
    power_down = 1;
    for(i=0;i<5000;++i);
    __lock_acquire(int_lock);
    assertEq(core_power_down,MCORE_PTR->CORE_NUM);
    __lock_release(int_lock);

    if (PWD_PTR->INFO & PWD_INFO_MAINPWD)
        assertEq(PWD_PTR->STAT,(1<<MCORE_PTR->CORE_NUM)-2);
    else
        assertEq(1,1); // To mask unavailable function

    // trigger interrupt to wakeup core
    for (k = 1; k < numOfCores; ++k){
        CLINT_PTR->MSIP[k] = 1;
        for(i=0;i<500;++i);
    }

    for(i=0;i<5000;i++);
    __lock_acquire(int_lock);
    assertEq(core_power_up,MCORE_PTR->CORE_NUM);
    __lock_release(int_lock);

}

void testMulticore(void){
    unsigned numOfCores, i;

    numOfCores = MCORE_PTR->CORE_NUM;

    core_power_down = 1;

    // start cores
    for (i = 1; i < numOfCores; ++i){
        coreStart(i, &testMulticoreProc, NULL);
    }
    // test wake-on on other cores finish
    power_down = 1;

    mainPowerDown(1);

    while(MCORE_PTR->STATUS != 1);

}

int main(void){

    printf("\nStarting power management tests\n");

    if (MCORE_PTR->CORE_NUM > 1){

        // Disable lockstep mode if present
        if (lockstepDisable() == 0){
            printf("Disabling lockstep mode!\n");
        }

        testSingleCoreShutDown();
        testShutDown();
        testMulticore();
        testCores();

        assertTrue(g_totalTests >= 7+(2*(MCORE_PTR->CORE_NUM-1)));

    }
    else{
        if (AMBA_TIMER32_COUNT() == 0){
            printf("No Timer32 to perform single core tests!\n");
            printTestSummary();
            return 0;
        }
        else{
            testSingleCoreShutDown();
            assertTrue(g_totalTests >= 2);
        }
    }

    printTestSummary();

    return 0;
}
