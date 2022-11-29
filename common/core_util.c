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
* File Name : core_util.c
* Author    : Rafal Harabien
* ******************************************************************************
* $Date: 2022-01-19 09:38:48 +0100 (śro, 19 sty 2022) $
* $Revision: 814 $
*H*****************************************************************************/

#include <ccrv32.h>
#include <ccrv32-mcore.h>
#include <ccrv32-csr.h>
#include <ccrv32-plic.h>
#include <ccrv32-utils.h>
#include <sys/lock.h>
#include <stddef.h>

void core_entrypoint(void);

void (* volatile g_coreFnPtr)(void *param) = NULL;
void * volatile g_coreFnParam = NULL;
volatile int g_coreStarted = 0;
__LOCK_INIT(static, g_lock);

void __attribute__((weak)) core_main(void)
{
    void (*fn)(void *param) = g_coreFnPtr;
    void *param = g_coreFnParam;
    g_coreStarted = 1;
    fn(param);
}

int coreStart(int core, void (*fn)(void *param), void *param)
{
    __lock_acquire(g_lock);
    if (MCORE_PTR->STATUS & (1 << core))
    {
        __lock_release(g_lock);
        return -1;
    }

    g_coreFnPtr = fn;
    g_coreFnParam = param;
    MCORE_PTR->CORE_ADDR[core] = (uint32_t)core_entrypoint;
    MCORE_PTR->CORE_RUN[core] = MCORE_RUN_KEY;
    while (!(MCORE_PTR->STATUS & (1 << core)));
    while (g_coreStarted == 0);
    g_coreStarted = 0;
    __lock_release(g_lock);
    return 0;
}

int lockstepDisable()
{
    uint32_t mstatus;
    uint32_t mie;
    uint32_t enable;
    register uint32_t shdn_address asm("a4");
    register uint32_t shdn_key asm("a5");

    shdn_address = (uint32_t)&(MCORE_PTR->CORE_SHDN);
    shdn_key = MCORE_SHDN_KEY | 2;

    if ((csr_read(mconfig2) & CPU_DCLS) == 0)
        return -1;
    if (MCORE_PTR->STATUS != 3)
        return -1;
    enable = PLIC_PTR->ENABLE;
    mie = csr_read(mie);
    mstatus = csr_read(mstatus);
    CLEAR_PIPELINE();
    csr_write(mstatus,csr_read(mstatus) & ~MSTAT_MIE);
    csr_write(mie,0);
    PLIC_PTR->ENABLE = 0;
    while (csr_read(mie) != 0);
    MEMORY_BARRIER();
    CLEAR_PIPELINE();
    __asm__ __volatile__ ("sw %0, 0(%1)"::"r"(shdn_key),"r"(shdn_address)); // disable lockstep
    CLEAR_PIPELINE();
    csr_write(mstatus,mstatus);
    csr_write(mie,mie);
    PLIC_PTR->ENABLE = enable;
    return 0;
}
