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
* $Date: 2024-09-17 20:06:05 +0200 (wto, 17 wrz 2024) $
* $Revision: 1107 $
*H*****************************************************************************/

#include "board.h"
#include <ccrv32.h>
#include <ccrv32-csr.h>
#include <ccrv32-plic.h>
#include <ccrv32-amba.h>
#include <ccrv32-amba-timer.h>
#include <stdio.h>

#define TEST_COUNTERS_MODIFIERS extern
#include "test.h"

#define TEST_MARKER 0x12345678
#define GUARD_MARKER 0x87654321
#define BIT(n) (1 << (n))


unsigned g_failedTests, g_totalTests;
static volatile unsigned expectIrq1 = 0, expectIrq2 = 0, expectIrq3 = 0, expectIrq4 = 0;
static volatile unsigned g_irqTestState = 0;

void isr1(void)
{
    uint32_t status;

    assertTrue(expectIrq1);
    expectIrq1 = 0;
    AMBA_TIMER32_PTR(0)->IRQM = 0;
    AMBA_TIMER32_PTR(0)->IRQF = TIMER_OVFIF; // clear flag

    status = csr_read(mstatus);
    assertTrue(status & MSTAT_MIE); // set in startup.S
    assertTrue(status & MSTAT_MPIE);

    if (g_irqTestState == 1)
    {
        g_irqTestState = 2;
        expectIrq2 = 1;
        AMBA_TIMER32_PTR(1)->IRQM = TIMER_OVFIE;

        // we are now preempted by IRQ2
        while (g_irqTestState != 7);

        // we are back
        g_irqTestState = 8;
    }
}

void isr2(void)
{
    assertTrue(expectIrq2);
    expectIrq2 = 0;
    AMBA_TIMER32_PTR(1)->IRQM = 0;
    AMBA_TIMER32_PTR(1)->IRQF = TIMER_OVFIF; // clear flag

    if (g_irqTestState == 2)
    {
        g_irqTestState = 3;
        expectIrq3 = 1;
        AMBA_TIMER16_PTR(0)->IRQM = TIMER_OVFIE;

        // we are now preempted by IRQ3
        while (g_irqTestState != 6);

        // we are back
        g_irqTestState = 7;
    }
}

void isr3(void)
{
    uint32_t status;

    assertTrue(expectIrq3);
    expectIrq3 = 0;
    AMBA_TIMER16_PTR(0)->IRQM = 0;
    AMBA_TIMER16_PTR(0)->IRQF = TIMER_OVFIF; // clear flag

    status = csr_read(mstatus);
    assertTrue(status & MSTAT_MIE); // set in startup.S
    assertTrue(status & MSTAT_MPIE);

    if (g_irqTestState == 3)
    {
        g_irqTestState = 4;
        expectIrq4 = 1;
        AMBA_TIMER16_PTR(1)->IRQM = TIMER_OVFIE;

        // we are now preempted by IRQ4
        while (g_irqTestState != 5);

        // we are back
        g_irqTestState = 6;
    }
}

void isr4(void)
{
    assertTrue(expectIrq4);
    expectIrq4 = 0;
    AMBA_TIMER16_PTR(1)->IRQM = 0;
    AMBA_TIMER16_PTR(1)->IRQF = TIMER_OVFIF; // clear flag

    if (g_irqTestState == 4)
        g_irqTestState = 5;
}

static void configTimerForIrqTest(volatile amba_timer_t *timer, unsigned irqNum)
{
    timer->PER = 3000;
    timer->PRES = 100;
    timer->COUNT = 0;
    timer->CTRL = TIMER_CTRL_EN;
    timer->IRQMAP = BIT(irqNum);
}

static void testInterrupts(void)
{
    volatile unsigned i = 0;
    uint32_t status;

    g_irqTestState = 0;

    // Enable interrupts on core 0
    PLIC_PTR->ENABLE |= BIT(1) | BIT(2) | BIT(3) | BIT(4); // IRQ 1 - 4

    // Configure 4 timers
    configTimerForIrqTest(AMBA_TIMER32_PTR(0), 1);
    configTimerForIrqTest(AMBA_TIMER32_PTR(1), 2);
    configTimerForIrqTest(AMBA_TIMER16_PTR(0), 3);
    configTimerForIrqTest(AMBA_TIMER16_PTR(1), 4);

    // Test single interrupt
    expectIrq1 = 1;
    AMBA_TIMER32_PTR(0)->IRQM = TIMER_OVFIE;
    for (i = 0; expectIrq1 && i < 10000000; ++i);
    assertFalse(expectIrq1);

    status = csr_read(mstatus);
    assertTrue(status & MSTAT_MIE); // interrupts should still be enabled

    // Test preemption - first set priorities
    PLIC_PTR->PRIOR[1] = 1;
    PLIC_PTR->PRIOR[2] = 2;
    PLIC_PTR->PRIOR[3] = 3;
    PLIC_PTR->PRIOR[4] = 4;

    // Start testing IRQ preemption: IRQ1 -> IRQ2 -> IRQ3 -> IRQ4
    g_irqTestState = 1;
    expectIrq1 = 1;
    AMBA_TIMER32_PTR(0)->IRQM = TIMER_OVFIE;

    for (i = 0; g_irqTestState != 8 && i < 10000000; ++i);
    assertEq(g_irqTestState, 8);
    assertFalse(expectIrq1);
    assertFalse(expectIrq2);
    assertFalse(expectIrq3);
    assertFalse(expectIrq4);

    status = csr_read(mstatus);
    assertTrue(status & MSTAT_MIE); // interrupts should still be enabled

    // disable IRQ 1 - 4
    PLIC_PTR->ENABLE = BIT(0);
}

void testStackProtection(void); // TODO

int main(void)
{
    volatile uint32_t cpu_info;
    uint32_t pending, status;

    printf("\nStarting PLIC test.\n");
    pending = PLIC_PTR->PENDING;
    assertFalse(pending);
    status = csr_read(mstatus);
    assertTrue(status & MSTAT_MIE); // interrupts should be enabled in startup.S

    if ((AMBA_TIMER32_COUNT() < 2) || (AMBA_TIMER16_COUNT() < 2)){
        printf("Not enough timers to perform irq tests!\n");
        printTestSummary();
        return 0;
    }

    testInterrupts();

    cpu_info = csr_read(mconfig0);
    if (cpu_info & CPU_SPROT)
    {
        printf("Detected stack protection.\n");
        testStackProtection();
    }

    printTestSummary();

    return 0;
}
