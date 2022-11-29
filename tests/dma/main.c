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
* $Date: 2021-09-15 22:43:51 +0200 (śro, 15 wrz 2021) $
* $Revision: 755 $
*H*****************************************************************************/

#include "board.h"
#include <ccrv32.h>
#include <ccrv32-utils.h>
#include <ccrv32-csr.h>
#include <ccrv32-plic.h>
#include <ccrv32-icache.h>
#include <ccrv32-amba.h>
#include <ccrv32-amba-uart.h>
#include <ccrv32-amba-dma.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <test.h>

#if MCU == ccnv2
    #define UART_BAUDRATE (76800)
#else
    #define UART_BAUDRATE (9600)
#endif

#define TEST_UART 1
#define TEST_DMA_UP_CHNL 0
#define TEST_DMA_DWN_CHNL 0

/* Buffer for data */
static char g_buf[8];
static uint16_t g_halfWordBuf[4];
static const char *g_bufABCD = "ABCD";
static const char *g_bufEFGH = "EFGH";
static volatile amba_dma_channel_t *g_curChnl;
static volatile unsigned g_expectedIrq = 0;
static volatile unsigned g_expectedStsInIrq = 0;
static volatile uint32_t g_nextCounterReload = 0;
static volatile uint32_t g_nextAddrReload = 0;
static volatile uint32_t g_nextIrqMask = 0;

void isr1(void)
{
    assertTrue(g_expectedIrq);
    g_expectedIrq = 0;

    assertEq(g_curChnl->STATUS, g_expectedStsInIrq);

    g_curChnl->IRQM = g_nextIrqMask;
    g_nextIrqMask = 0;

    if (g_nextAddrReload)
    {
        g_curChnl->ADDRESSREL = g_nextAddrReload;
        g_nextAddrReload = 0;
    }

    if (g_nextCounterReload)
    {
        g_curChnl->COUNTERREL = g_nextCounterReload;
        g_nextCounterReload = 0;
    }
}

static uint32_t uartReadBlocking(volatile amba_uart_t *uart)
{
    volatile unsigned i = 0;
    /* Wait for new data */
    for (i = 0; i < 1000000 && !(uart->STATUS & UART_STAT_RXC); ++i);
    assertTrue(i < 1000000);

    /* Return data */
    return uart->RDR;
}

static void uartWriteBlockingSingle(volatile amba_uart_t *uart, uint32_t data)
{
    while (!(uart->STATUS & UART_STAT_TXDRE));
    uart->TDR = data;
    while (!(uart->STATUS & UART_STAT_TXC));
}

static void uartWriteBlocking(volatile amba_uart_t *uart, const void *data, size_t len)
{
    size_t i;

    for (i = 0; i < len; ++i)
    {
        while (!(uart->STATUS & UART_STAT_TXDRE));
        uart->TDR = ((const char*)data)[i];
    }
    while (!(uart->STATUS & UART_STAT_TXC));
}

/**
 * @brief Tests sending data through UART using DMA
 */
static void testDmaUartTx(void)
{
    volatile unsigned i = 0;
    volatile amba_uart_t *uart;
    volatile amba_dma_channel_t *chnl;

    uint32_t baudrate = UART_BAUDRATE;

    uart = AMBA_UART_PTR(TEST_UART);
    uart->PRES = AMBA_UART_PRES((PERIPH0_FREQ / baudrate) / 16, (PERIPH0_FREQ / baudrate) % 16);
    uart->MODE = UART_MODE_CHRL8 | UART_MODE_STOP_BITS_1 | UART_MODE_PARITY_NONE;
    uart->CTRL = UART_CTRL_TXEN | UART_CTRL_RXEN | UART_CTRL_LOOP;

    AMBA_DMA_PTR->CONF = DMA_CONF_DSEN;
    assertEq(AMBA_DMA_PTR->CONF, DMA_CONF_DSEN);

    chnl = AMBA_DMA_DWN_CH_PTR(TEST_DMA_DWN_CHNL);
    assertEq(chnl->STATUS, DMA_STAT_TCZ|DMA_STAT_RCZ);
    chnl->PSELECT = AMBA_DMA_UART_PSELECT(TEST_UART);
    assertEq(chnl->PSELECT, AMBA_DMA_UART_PSELECT(TEST_UART));
    chnl->ADDRESSREL = 0;
    chnl->COUNTERREL = 0;

    // No INC mode
    chnl->CTRL = 0;
    chnl->ADDRESS = (uint32_t)g_bufABCD;
    assertEq(chnl->ADDRESS, (uint32_t)g_bufABCD);
    chnl->COUNTER = 4; // 4 elements
    MEMORY_BARRIER();
    assertEq(chnl->COUNTER, 4);
    chnl->CTRL = DMA_CTRL_EN | DMA_CTRL_TRU8;
    assertEq(chnl->CTRL, DMA_CTRL_EN|DMA_CTRL_TRU8);
    //assertEq(chnl->STATUS, DMA_STAT_RCZ|DMA_STAT_ACT);

    assertEq(uartReadBlocking(uart), 'A');
    assertEq(uartReadBlocking(uart), 'A');
    assertEq(uartReadBlocking(uart), 'A');
    assertEq(uartReadBlocking(uart), 'A');
    for (i = 0; i < 10000; ++i);
    assertFalse(uart->STATUS & UART_STAT_RXC);

    assertEq(chnl->STATUS, DMA_STAT_TCZ|DMA_STAT_RCZ);
    assertEq(chnl->ADDRESS, 0);
    assertEq(chnl->COUNTER, 0);

    // INC mode
    chnl->ADDRESS = (uint32_t)g_bufABCD;
    chnl->CTRL = DMA_CTRL_EN | DMA_CTRL_TRU8 | DMA_CTRL_INC;
    chnl->COUNTER = 4; // 4 elements
    MEMORY_BARRIER();

    assertEq(uartReadBlocking(uart), 'A');
    assertEq(uartReadBlocking(uart), 'B');
    assertEq(uartReadBlocking(uart), 'C');
    assertEq(uartReadBlocking(uart), 'D');
    for (i = 0; i < 10000; ++i);
    assertFalse(uart->STATUS & UART_STAT_RXC);

    assertEq(chnl->STATUS, DMA_STAT_TCZ|DMA_STAT_RCZ);
    assertEq(chnl->ADDRESS, 0);
    assertEq(chnl->COUNTER, 0);

    // DEC mode
    chnl->ADDRESS = ((uint32_t)g_bufABCD) + 3;
    chnl->CTRL = DMA_CTRL_EN | DMA_CTRL_TRU8 | DMA_CTRL_DEC;
    chnl->COUNTER = 4; // 4 elements
    MEMORY_BARRIER();

    assertEq(uartReadBlocking(uart), 'D');
    assertEq(uartReadBlocking(uart), 'C'); // FAILS!
    assertEq(uartReadBlocking(uart), 'B');
    assertEq(uartReadBlocking(uart), 'A');
    for (i = 0; i < 10000; ++i);
    assertFalse(uart->STATUS & UART_STAT_RXC);

    assertEq(chnl->STATUS, DMA_STAT_TCZ|DMA_STAT_RCZ);
    assertEq(chnl->ADDRESS, 0);
    assertEq(chnl->COUNTER, 0);

    // Interrupts + reload
    g_curChnl = chnl;
    chnl->CTRL = 0;
    chnl->ADDRESS = (uint32_t)g_bufABCD;
    chnl->COUNTER = 2; // 2 elements
    MEMORY_BARRIER();
    chnl->ADDRESSREL = (uint32_t)g_bufEFGH;
    chnl->COUNTERREL = 1; // 1 element
    chnl->IRQM = DMA_RCZIE|DMA_TCZIE;
    g_expectedIrq = 1;
    g_expectedStsInIrq = DMA_STAT_ACT|DMA_STAT_RCZ|DMA_STAT_MAR;
    g_nextAddrReload = (uint32_t)g_bufABCD;
    g_nextCounterReload = 1;
    g_nextIrqMask = DMA_TCZIE;
    chnl->CTRL = DMA_CTRL_EN | DMA_CTRL_TRU8 | DMA_CTRL_INC;
    //assertEq(chnl->STATUS, DMA_STAT_ACT);
    assertEq(uartReadBlocking(uart), 'A');
    assertEq(uartReadBlocking(uart), 'B');

    assertFalse(g_expectedIrq);
    g_expectedIrq = 1;
    g_expectedStsInIrq = DMA_STAT_TCZ|DMA_STAT_RCZ|DMA_STAT_MAR;
    assertEq(uartReadBlocking(uart), 'E');

    assertFalse(g_expectedIrq);
    assertEq(uartReadBlocking(uart), 'A');

    for (i = 0; i < 10000; ++i);
    assertFalse(uart->STATUS & UART_STAT_RXC);
    assertEq(chnl->STATUS, DMA_STAT_TCZ|DMA_STAT_RCZ);
    assertEq(chnl->ADDRESSREL, (uint32_t)g_bufABCD);
    assertEq(chnl->ADDRESS, (uint32_t)g_bufABCD);
    assertEq(chnl->COUNTERREL, 0);
    assertEq(chnl->COUNTER, 0);

    // Test sending more bits
    g_halfWordBuf[0] = 0x13F;
    g_halfWordBuf[1] = 0x278;
    uart->MODE = UART_MODE_CHRL9;
    chnl->CTRL = 0;
    chnl->ADDRESS = (uint32_t)g_halfWordBuf;
    chnl->COUNTER = 2; // 2 elements
    MEMORY_BARRIER();
    chnl->CTRL = DMA_CTRL_EN | DMA_CTRL_TRU16 | DMA_CTRL_INC;

    assertEq(uartReadBlocking(uart), 0x13F);
    assertEq(uartReadBlocking(uart), 0x78);
    for (i = 0; i < 10000; ++i);
    assertFalse(uart->STATUS & UART_STAT_RXC);
    assertEq(chnl->STATUS, DMA_STAT_TCZ|DMA_STAT_RCZ);
    assertEq(chnl->COUNTER, 0);
}

/**
 * @brief Tests receiving data through UART using DMA
 */
static void testDmaUartRx(void)
{
    volatile unsigned i = 0;
    volatile amba_uart_t *uart;
    volatile amba_dma_channel_t *chnl;

    uint32_t baudrate = UART_BAUDRATE;

    uart = AMBA_UART_PTR(TEST_UART);
    uart->PRES = AMBA_UART_PRES((PERIPH0_FREQ / baudrate) / 16, (PERIPH0_FREQ / baudrate) % 16);
    uart->MODE = UART_MODE_CHRL8 | UART_MODE_STOP_BITS_1 | UART_MODE_PARITY_NONE;
    uart->CTRL = UART_CTRL_TXEN | UART_CTRL_RXEN | UART_CTRL_LOOP;

    AMBA_DMA_PTR->CONF = DMA_CONF_USEN;
    assertEq(AMBA_DMA_PTR->CONF, DMA_CONF_USEN);

    chnl = AMBA_DMA_UP_CH_PTR(TEST_DMA_UP_CHNL);
    assertEq(chnl->STATUS, DMA_STAT_TCZ|DMA_STAT_RCZ);
    chnl->PSELECT = AMBA_DMA_UART_PSELECT(TEST_UART);
    chnl->ADDRESSREL = 0;
    chnl->COUNTERREL = 0;
    assertEq(chnl->PSELECT, AMBA_DMA_UART_PSELECT(TEST_UART));

    // No INC mode
    chnl->CTRL = 0;
    chnl->ADDRESS = (uint32_t)g_buf;
    assertEq(chnl->ADDRESS, (uint32_t)g_buf);
    chnl->COUNTER = 4; // 4 elements
    MEMORY_BARRIER();
    assertEq(chnl->COUNTER, 4);
    chnl->CTRL = DMA_CTRL_EN | DMA_CTRL_TRU8;
    assertEq(chnl->CTRL, DMA_CTRL_EN|DMA_CTRL_TRU8);
    assertEq(chnl->STATUS, DMA_STAT_RCZ|DMA_STAT_ACT);

    uartWriteBlocking(uart, "ABCD", 4);
    for (i = 0; i < 10000; ++i);

    assertEq(chnl->STATUS, DMA_STAT_TCZ|DMA_STAT_RCZ);
    assertEq(chnl->ADDRESS, 0);
    assertEq(chnl->COUNTER, 0);
    assertEq(g_buf[0], 'D');

    // INC mode
    chnl->ADDRESS = (uint32_t)g_buf;
    chnl->CTRL = DMA_CTRL_EN | DMA_CTRL_TRU8 | DMA_CTRL_INC;
    chnl->COUNTER = 4; // 4 elements
    MEMORY_BARRIER();

    uartWriteBlocking(uart, "EFGH", 4);
    for (i = 0; i < 10000; ++i);

    assertEq(chnl->STATUS, DMA_STAT_TCZ|DMA_STAT_RCZ);
    assertEq(chnl->ADDRESS, 0);
    assertEq(chnl->COUNTER, 0);
    assertStrEq2(g_buf, "EFGH", 4);

    // DEC mode
    chnl->ADDRESS = ((uint32_t)g_buf) + 3;
    chnl->CTRL = DMA_CTRL_EN | DMA_CTRL_TRU8 | DMA_CTRL_DEC;
    chnl->COUNTER = 4; // 4 elements
    MEMORY_BARRIER();

    uartWriteBlocking(uart, "ABCD", 4);
    for (i = 0; i < 10000; ++i);

    assertEq(chnl->STATUS, DMA_STAT_TCZ|DMA_STAT_RCZ);
    assertEq(chnl->ADDRESS, 0);
    assertEq(chnl->COUNTER, 0);
    assertStrEq2(g_buf, "DCBA", 4);

    // Interrupts + reload
    g_curChnl = chnl;
    chnl->CTRL = 0;
    chnl->ADDRESS = (uint32_t)g_buf;
    chnl->COUNTER = 2; // 2 elements
    MEMORY_BARRIER();
    chnl->ADDRESSREL = ((uint32_t)g_buf) + 3;
    chnl->COUNTERREL = 1; // 1 element
    chnl->IRQM = DMA_RCZIE|DMA_TCZIE;
    g_expectedIrq = 1; // expect RCZ interrupt
    g_expectedStsInIrq = DMA_STAT_ACT|DMA_STAT_RCZ|DMA_STAT_MAR;
    g_nextAddrReload = ((uint32_t)g_buf) + 2;
    g_nextCounterReload = 1;
    g_nextIrqMask = DMA_RCZIE|DMA_TCZIE;
    chnl->CTRL = DMA_CTRL_EN | DMA_CTRL_TRU8 | DMA_CTRL_INC;
    assertEq(chnl->STATUS, DMA_STAT_ACT);
    uartWriteBlocking(uart, "AB", 2);
    for (i = 0; i < 1000; ++i);

    // Registers has reloaded, COUNTERREL!=0
    assertFalse(g_expectedIrq);
    assertEq(chnl->STATUS, DMA_STAT_ACT);
    g_expectedIrq = 1; // expected IRQ after reseting IRQ
    g_expectedStsInIrq = DMA_STAT_ACT|DMA_STAT_RCZ|DMA_STAT_MAR;
    g_nextIrqMask = DMA_TCZIE; // we won't change COUTER_RELOAD so disable RCZIE
    uartWriteBlocking(uart, "C", 1);
    for (i = 0; i < 1000; ++i);

    // Registers reloaded, COUNTERREL=0
    assertFalse(g_expectedIrq);
    assertEq(chnl->STATUS, DMA_STAT_ACT|DMA_STAT_RCZ);
    g_expectedIrq = 1; // expect TCZ interrupt
    g_expectedStsInIrq = DMA_STAT_TCZ|DMA_STAT_RCZ;
    g_nextIrqMask = 0;
    uartWriteBlocking(uart, "D", 1);
    for (i = 0; i < 1000; ++i);

    // Transfer ended
    assertFalse(g_expectedIrq);
    assertEq(chnl->STATUS, DMA_STAT_TCZ|DMA_STAT_RCZ);
    assertEq(chnl->ADDRESSREL, ((uint32_t)g_buf) + 2);
    assertEq(chnl->ADDRESS, ((uint32_t)g_buf) + 2);
    assertEq(chnl->COUNTERREL, 0);
    assertEq(chnl->COUNTER, 0);
    assertStrEq2(g_buf, "ABDC", 4);

    // Test receiving more bits
    uart->MODE = UART_MODE_CHRL9;
    chnl->CTRL = 0;
    chnl->ADDRESS = (uint32_t)g_halfWordBuf;
    chnl->COUNTER = 2; // 2 elements
    MEMORY_BARRIER();
    chnl->CTRL = DMA_CTRL_EN | DMA_CTRL_TRU16 | DMA_CTRL_INC;

    uartWriteBlockingSingle(uart, 0x13F);
    uartWriteBlockingSingle(uart, 0x78);
    for (i = 0; i < 100; ++i);
    assertEq(chnl->STATUS, DMA_STAT_TCZ|DMA_STAT_RCZ);
    assertEq(chnl->COUNTER, 0);
    assertEq(g_halfWordBuf[0], 0x13F);
    assertEq(g_halfWordBuf[1], 0x78);
}

/**
 * @brief Tests transmission and reception in parallel on single UART in loopback mode using two DMA channels
 */
static void testDmaUartTxRx(void)
{
    volatile unsigned i = 0;
    volatile amba_uart_t *uart;
    volatile amba_dma_channel_t *chnlTx, *chnlRx;
    
    uint32_t baudrate = UART_BAUDRATE;

    uart = AMBA_UART_PTR(TEST_UART);
    uart->PRES = AMBA_UART_PRES((PERIPH0_FREQ / baudrate) / 16, (PERIPH0_FREQ / baudrate) % 16);
    uart->MODE = UART_MODE_CHRL8 | UART_MODE_STOP_BITS_1 | UART_MODE_PARITY_NONE;
    uart->CTRL = UART_CTRL_TXEN | UART_CTRL_RXEN | UART_CTRL_LOOP;
    uart->RDR; // ignore data if there is any

    AMBA_DMA_PTR->CONF = DMA_CONF_DSEN|DMA_CONF_USEN;
    assertEq(AMBA_DMA_PTR->CONF, DMA_CONF_DSEN|DMA_CONF_USEN);

    chnlTx = AMBA_DMA_DWN_CH_PTR(TEST_DMA_DWN_CHNL);
    assertEq(chnlTx->STATUS, DMA_STAT_TCZ|DMA_STAT_RCZ);
    chnlTx->PSELECT = AMBA_DMA_UART_PSELECT(TEST_UART);
    assertEq(chnlTx->PSELECT, AMBA_DMA_UART_PSELECT(TEST_UART));

    chnlRx = AMBA_DMA_UP_CH_PTR(TEST_DMA_UP_CHNL);
    assertEq(chnlRx->STATUS, DMA_STAT_TCZ|DMA_STAT_RCZ);
    chnlRx->PSELECT = AMBA_DMA_UART_PSELECT(TEST_UART);
    assertEq(chnlRx->PSELECT, AMBA_DMA_UART_PSELECT(TEST_UART));

    // Setup receive channel
    memset(g_buf, 0, sizeof(g_buf));
    chnlRx->CTRL = 0;
    chnlRx->ADDRESS = (uint32_t)g_buf;
    assertEq(chnlRx->ADDRESS, (uint32_t)g_buf);
    chnlRx->COUNTER = 4; // 4 elements
    MEMORY_BARRIER();
    assertEq(chnlRx->COUNTER, 4);
    chnlRx->CTRL = DMA_CTRL_EN|DMA_CTRL_TRU8|DMA_CTRL_INC;

    // Setup transmission channel
    chnlTx->CTRL = 0;
    chnlTx->ADDRESS = (uint32_t)g_bufABCD;
    assertEq(chnlTx->ADDRESS, (uint32_t)g_bufABCD);
    chnlTx->COUNTER = 4; // 4 elements
    MEMORY_BARRIER();
    assertEq(chnlTx->COUNTER, 4);
    chnlTx->CTRL = DMA_CTRL_EN|DMA_CTRL_TRU8|DMA_CTRL_INC;

    // Start both channels
    assertEq(chnlRx->CTRL, DMA_CTRL_EN|DMA_CTRL_TRU8|DMA_CTRL_INC);
    assertEq(chnlTx->CTRL, DMA_CTRL_EN|DMA_CTRL_TRU8|DMA_CTRL_INC);
    assertEq(chnlRx->STATUS, DMA_STAT_RCZ|DMA_STAT_ACT);
    //assertEq(chnlTx->STATUS, DMA_STAT_RCZ|DMA_STAT_ACT);

    for (i = 0; i < 100000; ++i);
    assertEq(chnlRx->STATUS, DMA_STAT_TCZ|DMA_STAT_RCZ);
    assertEq(chnlRx->COUNTER, 0);
    assertStrEq2(g_buf, "ABCD", 4);
}

int main(void)
{
    unsigned dmaUpCh, dmaDownCh;
    uint32_t status;

    dmaDownCh = AMBA_DMA_DWN_COUNT();
    dmaUpCh = AMBA_DMA_UP_COUNT();
    printf("\nStarting DMA test (found %u downstream and %u upstream channels)\n", dmaDownCh, dmaUpCh);

    if (!dmaDownCh || !dmaUpCh)
    {
        printf("Cannot test DMA: not enough channels.\n");
        printTestSummary();
        return 0;
    }

    if (AMBA_UART_COUNT() <= TEST_UART)
    {
        printf("Cannot test DMA: not enough UARTs.\n");
        printTestSummary();
        return 0;
    }

    volatile uint32_t cpu_info = csr_read(mconfig0);
    if ((cpu_info & CPU_ICACHE) && (((ICACHE_PTR->INFO & ICACHE_IMPL_MASK) >> ICACHE_IMPL_SHIFT) == ICACHE_IMPL_FT))
    {
        printf("Fault tolerant instruction cache. Skipping test.\n");
        printTestSummary();
        return 0;
    }

    // Enable interrupts
    PLIC_PTR->ENABLE = (1 << 1);
    status = csr_read(mstatus);
    assertTrue(status & MSTAT_MIE); // interrupts should be enabled in startup.S

    assertEq(AMBA_DMA_PTR->IRQMAP, 1 << AMBA_DMA_IRQn);
    AMBA_DMA_PTR->IRQMAP = 1 << 1;

    // Test DMA transmission through UART
    testDmaUartTx();
    testDmaUartRx();
    testDmaUartTxRx();

    printTestSummary();

    return 0;
}
