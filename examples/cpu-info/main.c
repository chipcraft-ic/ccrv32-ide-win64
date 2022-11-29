/*H*****************************************************************************
*
* Copyright (c) 2021 ChipCraft Sp. z o.o. All rights reserved
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
#include <ccrv32-utils.h>
#include <ccrv32-mcore.h>
#include <ccrv32-csr.h>
#include <ccrv32-icache.h>
#include <ccrv32-dcache.h>
#include <ccrv32-jtag.h>
#include <ccrv32-amba.h>
#include <ccrv32-amba-rtc.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <test.h>

int main(void)
{

    uint32_t cpu_info_0 = csr_read(mconfig0);
    uint32_t cpu_info_1 = csr_read(mconfig1);
    uint32_t cpu_info_2 = csr_read(mconfig2);

    uint32_t fpu_num = (cpu_info_1 & CPU_FPU_MASK) >> CPU_FPU_SHIFT;
    uint32_t compr = cpu_info_1 & CPU_COMPR;
    uint32_t gnss_ise = CPU_INFO_GET_GNSS_ISE_NUM(cpu_info_1);

    uint32_t low_power = 0;

    char arch_str[15];

    printf("\nINFO: IDCODE:                   0x%08x\n",(unsigned int)IDCODE_GET);
    printf("INFO: Number of cores:          %d\n",(int)MCORE_PTR->CORE_NUM);
    printf("INFO: Core architecture:        ");

    if (fpu_num > 0) {
        if (compr > 0) {
            if (gnss_ise > 0) {
                sprintf(arch_str,"gcxgnss");
            }
            else
            {
                sprintf(arch_str,"gc");
            }
        }
        else {
            if (gnss_ise > 0) {
                sprintf(arch_str,"gxgnss");
            }
            else
            {
                sprintf(arch_str,"g");
            }
        }
    }
    else {
        if (compr > 0) {
            if (gnss_ise > 0) {
                sprintf(arch_str,"imacxgnss");
            }
            else
            {
                sprintf(arch_str,"imac");
            }
        }
        else {
            if (gnss_ise > 0) {
                sprintf(arch_str,"imaxgnss");
            }
            else
            {
                sprintf(arch_str,"ima");
            }
        }
    }

    if ((cpu_info_2 & CPU_ARCH_MASK) >> CPU_ARCH_SHIFT == 0) {
        printf("unknown\n");
    }
    else if ((cpu_info_2 & CPU_ARCH_MASK) >> CPU_ARCH_SHIFT == 1) {
        printf("CCRV32LP%s\n",arch_str);
        low_power = 1;
    }
    else if ((cpu_info_2 & CPU_ARCH_MASK) >> CPU_ARCH_SHIFT == 2) {
        if (cpu_info_2 & CPU_DCLS) {
            printf("CCRV32HR%s\n",arch_str);
        }
        else {
            printf("CCRV32ST%s\n",arch_str);
        }
    }

    printf("INFO: User mode:                ");
    if (cpu_info_0 & CPU_USER) {
        printf("enabled\n");
    }
    else {
        printf("disabled\n");
    }
    printf("INFO: Branch prediction:        ");
    switch ((cpu_info_1 & CPU_BPRED_MASK) >> CPU_BPRED_SHIFT) {
        case 0:
            if (low_power)
                printf("disabled\n");
            else
                printf("branch-always\n");
            break;
        case 1:
            printf("branch-never\n"); break;
        case 2:
            printf("opcode\n"); break;
        case 3:
            printf("PHT\n"); break;
        case 4:
            printf("gshare\n"); break;
    }
    printf("INFO: Area optimization:        ");
    if (cpu_info_1 & CPU_AROPT) {
        printf("enabled\n");
    }
    else {
        printf("disabled\n");
    }
    printf("INFO: Technology:               ");
    if (cpu_info_1 & CPU_TECH) {
        printf("FPGA\n");
    }
    else {
        printf("ASIC\n");
    }
    printf("INFO: Multiplier:               ");
    switch ((cpu_info_1 & CPU_MUL_MASK) >> CPU_MUL_SHIFT) {
        case 0:
            printf("iterative 16x16\n");
            printf("INFO: Fast 16-bit mult.:        ");
            if (cpu_info_1 & CPU_MULFAST) {
                printf("enabled\n");
            }
            else {
                printf("disabled\n");
            }
            break;
        case 1:
            printf("single-cycle 32x32\n"); break;
        case 2:
            printf("iterative 1-bit\n"); break;
    }
    printf("INFO: FPU units:                %d (double-precision)\n",(int)fpu_num);
    if (cpu_info_0 & CPU_ICACHE) {
        printf("INFO: Instruction cache:        %d ways * %f kbytes\n",(unsigned)(cpu_info_0 & CPU_ICWAY_MASK)>>CPU_ICWAY_SHIFT,(float)(((int)1 << ((cpu_info_0 & CPU_ICSIZE_MASK) >> CPU_ICSIZE_SHIFT))/1024.0));
        printf("INFO: Instruction cache arch.:  ");
        switch ((ICACHE_PTR->INFO & ICACHE_IMPL_MASK) >> ICACHE_IMPL_SHIFT) {
            case ICACHE_IMPL_HP:
                printf("high-performance\n"); break;
            case ICACHE_IMPL_FT:
                printf("fault-tolerant\n"); break;
        }
    }
    else {
        printf("INFO: Instruction cache:        absent");
    }

    if (cpu_info_1 & CPU_DCACHE) {
        printf("INFO: Data cache:               %d ways * %f kbytes\n",(unsigned)(cpu_info_1 & CPU_DCWAY_MASK)>>CPU_DCWAY_SHIFT,(float)(((int)1 << ((cpu_info_1 & CPU_DCSIZE_MASK) >> CPU_DCSIZE_SHIFT))/1024.0));
        printf("INFO: Data cache arch.:         ");
        switch ((DCACHE_PTR->INFO & DCACHE_IMPL_MASK) >> DCACHE_IMPL_SHIFT) {
            case DCACHE_IMPL_HP:
                printf("high-performance\n"); break;
            case DCACHE_IMPL_HS:
                printf("high-speed\n"); break;
            case DCACHE_IMPL_LP:
                printf("low-power\n"); break;
            case DCACHE_IMPL_FT:
                printf("fault-tolerant\n"); break;
        }
    }
    else {
        printf("INFO: Data cache:               absent");
    }

    printf("INFO: ROM size:                 %d kbytes\n",((int)1<<((cpu_info_0 & CPU_IMSIZE_MASK) >> CPU_IMSIZE_SHIFT))/1024);
    printf("INFO: RAM size:                 %d kbytes\n",((int)1<<((cpu_info_0 & CPU_DMSIZE_MASK) >> CPU_DMSIZE_SHIFT))/1024);
    if (cpu_info_0 & CPU_SPRAM) {
        printf("INFO: SP-RAM size:              %d kbytes\n",((int)1<<((cpu_info_0 & CPU_SPRSIZE_MASK) >> CPU_SPRSIZE_SHIFT))/1024);
    }

    return 0;
}
