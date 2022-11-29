/* ----------------------------------------------------------------------
*
* Copyright (c) 2021 ChipCraft Sp. z o.o. All rights reserved
*
* $Date: 2022-01-19 09:38:48 +0100 (śro, 19 sty 2022) $
* $Revision: 814 $
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
 * @file            ccnv2.h
 * @brief           CCNV2 Processor AMBA definitions
 * @author          Krzysztof Marcinek
 *
 * @defgroup        CCAMBA CCNV2 Peripherials
 * AMBA Bus Peripherials
 * @{
 */

#ifndef __CCRV32_CCNV2_H
#define __CCRV32_CCNV2_H

/** AMBA addresses */
enum
{
    AMBA_EDAC0_BASE      = AMBA_APB1_BASE + 0x00002000,     /*!< Main Memory EDAC0 base address     */
    AMBA_MBIST0_BASE     = AMBA_APB1_BASE + 0x00003000,     /*!< Main Memory MBIST0 base address    */
    AMBA_EDAC1_BASE      = AMBA_APB1_BASE + 0x00004000,     /*!< Main Memory EDAC1 base address     */
    AMBA_MBIST1_BASE     = AMBA_APB1_BASE + 0x00005000,     /*!< Main Memory MBIST1 base address    */
    AMBA_EDAC2_BASE      = AMBA_APB1_BASE + 0x00006000,     /*!< Main Memory EDAC2 base address     */
    AMBA_MBIST2_BASE     = AMBA_APB1_BASE + 0x00007000,     /*!< Main Memory MBIST2 base address    */
    AMBA_EDAC3_BASE      = AMBA_APB1_BASE + 0x00008000,     /*!< Main Memory EDAC3 base address     */
    AMBA_MBIST3_BASE     = AMBA_APB1_BASE + 0x00009000,     /*!< Main Memory MBIST3 base address    */
};

#define AMBA_EDAC_BASE(index) (AMBA_EDAC0_BASE+(index)*0x2000)                  /*!< Main Memory EDAC base address      */
#define AMBA_EDAC_PTR(index) ((volatile amba_edac_t*)AMBA_EDAC_BASE(index))     /*!< Main Memory EDAC pointer           */

/**
 * @name CCNV2 defines
 * @{
 */

#define RAM_PARTITIONS     0x03  /*!< Number of main memory RAM partitions     */

/** @} */

#endif /* __CCRV32_CCNV2_H */
/** @} */
