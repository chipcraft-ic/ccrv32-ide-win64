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
* Author    : Rafal Harabien
* ******************************************************************************
* $Date: 2021-02-05 17:02:56 +0100 (pią, 05 lut 2021) $
* $Revision: 676 $
*H*****************************************************************************/

#include "board.h"
#include <ccrv32.h>
#include <ccrv32-csr.h>
#include <ccrv32-mcore.h>
#include <ccrv32-plic.h>
#include <ccrv32-icore.h>
#include <ccrv32-utils.h>
#include <core_util.h>
#include <stdio.h>
#include <errno.h>
#include <sys/lock.h>
#include "test.h"

void (*g_coreProcPtr)(void);

static unsigned g_currentCoreIndex;
static volatile unsigned g_stopCore, g_coreProcState;

void core_entrypoint(void);
void testMulticoreLibC(void);

void core_main(void)
{
    g_coreProcPtr();
}

static void coreProc1(void)
{
    assertEq(g_coreProcState, 0); // state 0
    assertEq(MCORE_PTR->STATUS, 1 | (1 << g_currentCoreIndex)); // 2 cores are running
    assertEq(csr_read(mhartid), g_currentCoreIndex);

    g_coreProcState = 1;
    while (!g_stopCore);

    g_coreProcState = 2;
}

static void testStartingAndStopingCore(unsigned index)
{
    /* Init global variables */
    g_currentCoreIndex = index;
    g_coreProcState = 0;
    g_stopCore = 0;
    g_coreProcPtr = coreProc1;

    /* Start core */
    MCORE_PTR->CORE_ADDR[index] = (uint32_t)core_entrypoint;
    NOP();
    assertEq(MCORE_PTR->CORE_ADDR[index], (uint32_t)core_entrypoint);
    MCORE_PTR->CORE_RUN[index] = MCORE_RUN_KEY;
    assertEq(MCORE_PTR->CORE_RUN[index], 0); // always 0

    /* Wait for state 1 */
    while (g_coreProcState != 1);
    assertEq(MCORE_PTR->STATUS, 1 | (1 << g_currentCoreIndex)); // 2 cores are running

    /* Try to start core again */
    assertEq(MCORE_PTR->CORE_ADDR[index], (uint32_t)core_entrypoint);
    MCORE_PTR->CORE_RUN[index] = MCORE_RUN_KEY;
    assertEq(MCORE_PTR->STATUS, 1 | (1 << g_currentCoreIndex)); // 2 cores are running

    /* Wait for core to finish running */
    g_stopCore = 1;
    while (MCORE_PTR->STATUS != 1);

    assertEq(g_coreProcState, 2); // core procedure ended
}

static volatile unsigned g_expectedIciDestCore, g_expectedIciSrcCore, g_expectedIci;

void isr15(void)
{
    volatile unsigned i = 0;

    ok(g_expectedIci, "Unexpected intercore interrupt");
    assertEq(ICORE_PTR->ICORE_IRQ_FLAG, 1 << g_expectedIciSrcCore);
    ICORE_PTR->ICORE_IRQ_FLAG = 1 << g_expectedIciSrcCore;
    for (i = 0; i < 3; ++i); // wait a moment - we cannot read IRQF immediately after write
    assertEq(ICORE_PTR->ICORE_IRQ_FLAG, 0);
    assertEq(csr_read(mhartid), g_expectedIciDestCore);
    ++g_coreProcState;
}

static void coreProc2(void)
{
    /* Enable interrupts */
    PLIC_PTR->ENABLE |= (1 << ICORE_IRQn); // IRQ 15
    uint32_t status = csr_read(mstatus);
    assertTrue(status & MSTAT_MIE); // interrupts should be enabled in startup.S

    /* trigger interrupt on core 0 */
    g_coreProcState = 0;
    g_expectedIciSrcCore = csr_read(mhartid);
    g_expectedIciDestCore = 0;
    g_expectedIci = 1;
    ICORE_PTR->ICORE_IRQ_TRIG = 1 << 0;

    /* Wait */
    while (!g_stopCore);
}

static void testIntercoreInterrupt(unsigned coreIndex)
{
    ICORE_PTR->ICORE_IRQ_MAP = 1 << ICORE_IRQn; // FIXME
    assertEq(ICORE_PTR->ICORE_IRQ_MAP, 1 << ICORE_IRQn); // 15
    assertEq(ICORE_PTR->ICORE_IRQ_FLAG, 0);

    /* Start core */
    g_stopCore = 0;
    g_coreProcPtr = coreProc2;
    MCORE_PTR->CORE_ADDR[coreIndex] = (uint32_t)core_entrypoint;
    MCORE_PTR->CORE_RUN[coreIndex] = MCORE_RUN_KEY;

    while (g_coreProcState != 1); // wait for ICI

    /* Make another ICI from core 0 */
    g_expectedIciSrcCore = 0;
    g_expectedIciDestCore = coreIndex;
    g_expectedIci = 1;
    ICORE_PTR->ICORE_IRQ_TRIG = 1 << coreIndex; // trigger interrupt on another core

    /* wait for ICI */
    while (g_coreProcState != 2);

    /* Stop core */
    g_stopCore = 1;
    while (MCORE_PTR->STATUS != 1);

    /* Try triggering with wrong values - nothing should happen */
    ICORE_PTR->ICORE_IRQ_TRIG = 0;
    ICORE_PTR->ICORE_IRQ_TRIG = ~((1 << MCORE_PTR->CORE_NUM) - 1);
}

#ifndef _NO_SPRAM_MEMORY

static int SPRAM_BSS g_spramBssVar;
static int SPRAM_DATA g_spramDataVar = 123;

static void spramTestCoreProc(void)
{
    int coreId = csr_read(mhartid);
    assertEq(g_spramBssVar, 0);
    assertEq(g_spramDataVar, 123);
    g_spramBssVar = 20 + coreId;
    g_spramDataVar = 30 + coreId;
}

static void testSpram(void)
{
    assertEq(g_spramBssVar, 0);
    assertEq(g_spramDataVar, 123);

    g_coreProcPtr = spramTestCoreProc;
    assertEq(MCORE_PTR->CORE_ADDR[1], (uint32_t)core_entrypoint);
    MCORE_PTR->CORE_RUN[1] = MCORE_RUN_KEY;
    while (MCORE_PTR->STATUS != 1);

    // check if nothing changed
    assertEq(g_spramBssVar, 0);
    assertEq(g_spramDataVar, 123);
}

#endif

static void errnoTestCoreProc(void)
{
    int coreId = csr_read(mhartid);
    errno = 32 + coreId;
    assertEq(errno, 32 + coreId);
}

static void testErrno(void)
{
    /* Test errno */
    errno = 123;

    g_coreProcPtr = errnoTestCoreProc;
    assertEq(MCORE_PTR->CORE_ADDR[1], (uint32_t)core_entrypoint);
    MCORE_PTR->CORE_RUN[1] = MCORE_RUN_KEY;
    while (MCORE_PTR->STATUS != 1);

    assertEq(errno, 123);
}

void __malloc_lock(struct _reent *REENT);
void __malloc_unlock(struct _reent *REENT);

static volatile int g_mlockCounter = 0;

static void mlockTestCoreProc(void)
{
    __malloc_lock(_REENT);
    ++g_mlockCounter;
    __malloc_unlock(_REENT);
}

static void testMallocLock(void)
{
    /* Test malloc lock */
    g_coreProcPtr = mlockTestCoreProc;
    __malloc_lock(_REENT);

    assertEq(MCORE_PTR->CORE_ADDR[1], (uint32_t)core_entrypoint);
    MCORE_PTR->CORE_RUN[1] = MCORE_RUN_KEY;
    for (volatile int i = 0; i < 10000; ++i); // delay
    int mlockCounterInLock = g_mlockCounter;
    __malloc_unlock(_REENT);
    assertEq(mlockCounterInLock, 0); // Note: this can print so we cannot call it during malloc lock

    while (MCORE_PTR->STATUS != 1);
    assertEq(g_mlockCounter, 1);
}

int main(void)
{
    unsigned numOfCores, i;

    numOfCores = MCORE_PTR->CORE_NUM;
    printf("\nStarting multicore test (%u cores)\n", numOfCores);
    PLIC_PTR->ENABLE |= (1 << ICORE_IRQn); // IRQ 15

    // Disable lockstep mode if present
    if (lockstepDisable() == 0){
        printf("Disabling lockstep mode!\n");
    }

    for (i = 1; i < numOfCores; ++i)
    {
        testStartingAndStopingCore(i);
        testIntercoreInterrupt(i);
    }

    if (numOfCores > 1)
    {
#ifndef _NO_SPRAM_MEMORY
        testSpram();
#endif
        testErrno();
        testMallocLock();
    }

    printTestSummary();

    return 0;
}
