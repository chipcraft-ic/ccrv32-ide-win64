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
* $Date: 2024-11-13 22:00:39 +0100 (śro, 13 lis 2024) $
* $Revision: 1116 $
*H*****************************************************************************/

#include "board.h"
#include <ccrv32.h>
#include <ccrv32-csr.h>
#include <ccrv32-mcore.h>
#include <ccrv32-dcache.h>
#include <ccrv32-clint.h>
#include <ccrv32-utils.h>
#include <core_util.h>
#include <stdio.h>
#include "test.h"

static volatile unsigned g_stopCore, g_coreProcState;

static volatile unsigned g_expectedSoftIrqDestCore, g_expectedSoftIrq, g_expectedTimerIrq;

void isr_soft(void)
{
    volatile unsigned i = 0;
    ok(g_expectedSoftIrq, "Unexpected software interrupt");
    CLINT_PTR->MSIP[csr_read(mhartid)] = 0;
    for (i = 0; i < 3; ++i);
    assertEq(CLINT_PTR->MSIP[csr_read(mhartid)], 0);
    assertEq(csr_read(mhartid), g_expectedSoftIrqDestCore);
    ++g_coreProcState;
}

void isr_timer(void)
{
    volatile unsigned i = 0;
    ok(g_expectedTimerIrq, "Unexpected timer interrupt");
    csr_write(mie,csr_read(mie) & ~MIE_MTIE);
    for (i = 0; i < 3; ++i);
    g_stopCore = 1;
}

static void coreProcSoft()
{
    /* Enable software interrupts */
    csr_write(mie,csr_read(mie) | MIE_MSIE);

    /* trigger interrupt on core 0 */
    g_coreProcState = 0;
    g_expectedSoftIrqDestCore = 0;
    g_expectedSoftIrq = 1;
    while(DCACHE_PTR->STCR & DCACHE_STCR_BUSY);
    CLINT_PTR->MSIP[0] = 1;

    /* Wait */
    while (!g_stopCore);
    g_expectedSoftIrq = 0;
}

static void coreProcTimer()
{
    volatile uint32_t clock_lo, clock_hi_old, clock_hi_new;
    volatile uint64_t clock_val;

    do {
        clock_hi_old = CLINT_PTR->MTIME_HI;
        clock_lo = CLINT_PTR->MTIME_LO;
        clock_hi_new = CLINT_PTR->MTIME_HI;
    } while (clock_hi_new != clock_hi_old);
    clock_val = (uint64_t)clock_lo;
    clock_val += ((uint64_t)clock_hi_old << 32);

    /* setup timer interrupt */
    g_expectedTimerIrq = 1;
    CLINT_PTR->MTIMECMP[csr_read(mhartid)] = clock_val + 100000;

    /* trigger timer interrupts */
    csr_write(mie,csr_read(mie) | MIE_MTIE);

    /* Wait */
    while (!g_stopCore);
    g_expectedTimerIrq = 0;
}

static void testSoftwareInterrupt(unsigned coreIndex)
{
    /* Start core */
    g_stopCore = 0;
    coreStart(coreIndex, &coreProcSoft, NULL);

    while (g_coreProcState != 1); // wait for software interrupt

    /* Make another software interrupt from core 0 */
    g_expectedSoftIrqDestCore = coreIndex;
    g_expectedSoftIrq = 1;
    while(DCACHE_PTR->STCR & DCACHE_STCR_BUSY);
    CLINT_PTR->MSIP[coreIndex] = 1; // trigger interrupt on another core

    /* wait for software interrupt */
    while (g_coreProcState != 2);

    /* Stop core */
    g_stopCore = 1;
    while (MCORE_PTR->STATUS != 1);

}

static void testTimerInterrupt(unsigned coreIndex)
{
    /* Start core */
    g_stopCore = 0;
    CLINT_PTR->MTIMECFG |= MTIMECFG_EN;
    coreStart(coreIndex, &coreProcTimer, NULL);
    while (MCORE_PTR->STATUS != 1);
}

int main(void)
{
    unsigned numOfCores, i;

    numOfCores = MCORE_PTR->CORE_NUM;
    printf("\nStarting CLINT test.\n");
    csr_write(mie,csr_read(mie) | MIE_MSIE);

    // Disable lockstep mode if present
    if (lockstepDisable() == 0){
        printf("Disabling lockstep mode!\n");
    }

    for (i = 1; i < numOfCores; ++i)
    {
        testSoftwareInterrupt(i);
    }
    for (i = 1; i < numOfCores; ++i)
    {
        testTimerInterrupt(i);
    }

    printTestSummary();

    return 0;
}
