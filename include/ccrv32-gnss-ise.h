/* ----------------------------------------------------------------------
*
* Copyright (c) 2019 ChipCraft Sp. z o.o. All rights reserved
*
* $Date: 2024-07-31 10:10:22 +0200 (śro, 31 lip 2024) $
* $Revision: 1080 $
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
 * @file        ccrv32-gnss-ise.h
 * @brief       CCRV32 Processor GNSS definitions
 * @author      Krzysztof Marcinek
 *
 * @defgroup    ccrv32gnss GNSS Instructions
 * @{
 */
#ifndef __CCRV32_GNSS_ISE_H__
#define __CCRV32_GNSS_ISE_H__

#include <ccrv32-utils.h>
#include <stdint.h>

#ifndef __GNUC__
 #error Only GCC is supported by this header! Use your compiler inline assembly feature manually.
#endif

#ifndef INLINE
/// define INLINE
# define INLINE static __inline__
#endif

/** GNSS-ISE RF AFE Selection */
enum gnss_ise_rfafe_enum
{
    GNSS_ISE_RFAFE_INVALID    = -1, /*!< GNSS-ISE Invalid RF AFE         */

    GNSS_ISE_RFAFE_L1E1       = 1,  /*!< GNSS-ISE L1/E1 RF AFE           */
    GNSS_ISE_RFAFE_L5E5a      = 2,  /*!< GNSS-ISE L5/E5a RF AFE          */
    GNSS_ISE_RFAFE_E5b        = 3,  /*!< GNSS-ISE E5b RF AFE             */
    GNSS_ISE_RFAFE_L2         = 4,  /*!< GNSS-ISE L2 RF AFE              */
    GNSS_ISE_RFAFE_E6         = 5,  /*!< GNSS-ISE E6 RF AFE              */
    GNSS_ISE_RFAFE_AUX0       = 7,  /*!< GNSS-ISE AUX0 RF AFE            */
    GNSS_ISE_RFAFE_AUX1       = 8,  /*!< GNSS-ISE AUX1 RF AFE            */
    GNSS_ISE_RFAFE_AUX2       = 9,  /*!< GNSS-ISE AUX2 RF AFE            */
    GNSS_ISE_RFAFE_AUX3       = 10, /*!< GNSS-ISE AUX3 RF AFE            */
    GNSS_ISE_RFAFE_AUX4       = 11, /*!< GNSS-ISE AUX4 RF AFE            */
    GNSS_ISE_RFAFE_AUX5       = 12, /*!< GNSS-ISE AUX5 RF AFE            */
    GNSS_ISE_RFAFE_AUX6       = 13, /*!< GNSS-ISE AUX6 RF AFE            */
    GNSS_ISE_RFAFE_AUX7       = 14, /*!< GNSS-ISE AUX7 RF AFE            */
    GNSS_ISE_RFAFE_VIRT       = 15, /*!< GNSS-ISE Virtual RF AFE         */

    GNSS_ISE_RFAFE_MAX        = 16, /*!< GNSS-ISE Guard Value            */

};

// cosine BOC modulation define
#define GNSS_ISE_BOC_COS    (1<<4);
// TM modulation for L2C
#define GNSS_ISE_L2C_TM     (2<<5);

/// define EXTRACT
#define EXTRACT(a, size, offset) (((~(~0 << size) << offset) & a) >> offset)

/// define CUSTOMX_OPCODE
#define CUSTOMX_OPCODE(x) CUSTOM_ ## x
#define CUSTOM_0 0b0001011                /*!< define CUSTOM_0 */
#define CUSTOM_1 0b0101011                /*!< define CUSTOM_1 */
#define CUSTOM_2 0b1011011                /*!< define CUSTOM_2 */
#define CUSTOM_3 0b1111011                /*!< define CUSTOM_3 */

/// define CUSTOMX
#define CUSTOMX(X, rd, rs1, rs2, funct)         \
    CUSTOMX_OPCODE(X)                       |   \
    (rd                   << (7))           |   \
    (EXTRACT(funct, 3, 0) << (7+5))         |   \
    (rs1                  << (7+5+3))       |   \
    (rs2                  << (7+5+3+5))     |   \
    (EXTRACT(funct, 7, 3) << (7+5+3+5+5))

/// Standard macro that passes rd, rs1, and rs2 via registers
#define CUSTOM_INSTRUCTION_MACRO_2I1O(rd, rs1, rs2, funct)              \
  CUSTOM_INSTRUCTION_MACRO_RI_RI_RO(0, rd, rs1, rs2, funct, 10, 11, 12)

/// Standard macro that passes rd via register
#define CUSTOM_INSTRUCTION_MACRO_1O(rd, funct)                          \
    CUSTOM_INSTRUCTION_MACRO_RO(0, rd, funct, 10)

/// Standard macro that passes rs1 via register
#define CUSTOM_INSTRUCTION_MACRO_1I(rs1, funct)                         \
    CUSTOM_INSTRUCTION_MACRO_RI(0, rs1, funct, 10)

/// Standard macro that passes rd nad rs1 via registers
#define CUSTOM_INSTRUCTION_MACRO_1I1O(rd, rs1, funct)                   \
    CUSTOM_INSTRUCTION_MACRO_RI_RO(0, rd, rs1, funct, 10, 11)

/// Standard macro that passes rs1, and rs2 via registers
#define CUSTOM_INSTRUCTION_MACRO_2I(rs1, rs2, funct)                    \
    CUSTOM_INSTRUCTION_MACRO_RI_RI(0, rs1, rs2, funct, 10, 11)

/// Standard macro with no arguments
#define CUSTOM_INSTRUCTION_MACRO(funct)                                 \
    CUSTOM_INSTRUCTION_MACRO_0I0O(0, funct)

/// rd, rs1, and rs2 are data
/// rd_n, rs_1, and rs2_n are the register numbers to use
#define CUSTOM_INSTRUCTION_MACRO_RI_RI_RO(X, rd, rs1, rs2, funct, rd_n, rs1_n, rs2_n) { \
    register uint64_t rd_  asm ("x" # rd_n);                            \
    register uint64_t rs1_ asm ("x" # rs1_n) = (uint64_t) rs1;          \
    register uint64_t rs2_ asm ("x" # rs2_n) = (uint64_t) rs2;          \
    asm volatile (                                                      \
        ".word " STR(CUSTOMX(X, rd_n, rs1_n, rs2_n, funct)) "\n\t"      \
        : "=r" (rd_)                                                    \
        : [_rs1] "r" (rs1_), [_rs2] "r" (rs2_));                        \
    rd = rd_;                                                           \
}

/// rs1 is data
/// rs_1 is the register numbers to use
#define CUSTOM_INSTRUCTION_MACRO_RI(X, rs1, funct, rs1_n) { \
    register uint32_t rs1_ asm ("x" # rs1_n) = (uint32_t) rs1;          \
    asm volatile (                                                      \
        ".word " STR(CUSTOMX(X, 0, rs1_n, 0, funct)) "\n\t"             \
        :                                                               \
        : [_rs1] "r" (rs1_));                                           \
}

/// rd is data
/// rd_n is the register numbers to use
#define CUSTOM_INSTRUCTION_MACRO_RO(X, rd, funct, rd_n) { \
    register uint32_t rd_  asm ("x" # rd_n);                            \
    asm volatile (                                                      \
        ".word " STR(CUSTOMX(X, rd_n, 0, 0, funct)) "\n\t"              \
        : "=r" (rd_));                                                  \
    rd = rd_;                                                           \
}

/// no arguments
#define CUSTOM_INSTRUCTION_MACRO_0I0O(X, funct) { \
    asm volatile (                                                      \
        ".word " STR(CUSTOMX(X, 0, 0, 0, funct)) "\n\t"                 \
        :                                                               \
        : );                                                            \
}


/// rd nad rs1 are data
/// rd_n nad rs_1 are the register numbers to use
#define CUSTOM_INSTRUCTION_MACRO_RI_RO(X, rd, rs1, funct, rd_n, rs1_n) { \
    register uint32_t rd_  asm ("x" # rd_n);                            \
    register uint32_t rs1_ asm ("x" # rs1_n) = (uint32_t) rs1;          \
    asm volatile (                                                      \
        ".word " STR(CUSTOMX(X, rd_n, rs1_n, 0, funct)) "\n\t"          \
        : "=r" (rd_)                                                    \
        : [_rs1] "r" (rs1_));                                           \
    rd = rd_;                                                           \
}

/// rs1, and rs2 are data
/// rs_1, and rs2_n are the register numbers to use
#define CUSTOM_INSTRUCTION_MACRO_RI_RI(X, rs1, rs2, funct, rs1_n, rs2_n) { \
    register uint32_t rs1_ asm ("x" # rs1_n) = (uint32_t) rs1;          \
    register uint32_t rs2_ asm ("x" # rs2_n) = (uint32_t) rs2;          \
    asm volatile (                                                      \
        ".word " STR(CUSTOMX(X, 0, rs1_n, rs2_n, funct)) "\n\t"         \
        :                                                               \
        : [_rs1] "r" (rs1_), [_rs2] "r" (rs2_));                        \
}

/**
 * \defgroup common Common
 * @{
 */

/**
 * write status
 * @param status bits[0] = clock
 */
INLINE void GNSS_STATUS_WR(uint32_t status)
{
    CUSTOM_INSTRUCTION_MACRO_1I(status, 1);
}

/**
 * read status
 * @return bits[0] = clock
 */
INLINE uint32_t GNSS_STATUS_RD(void)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1O(res, 129);
    return res;
}

/**
 * store data in GNSS engine frontend register
 */
INLINE void GNSS_AFE_WR(uint64_t val)
{
    CUSTOM_INSTRUCTION_MACRO_2I(val & 0xFFFFFFFF, (val>>32) & 0xFFFFFFFF, 20);
}

/**
 * read selected analog frontend of GNSS channels.
 */
INLINE int32_t GNSS_AFE_RD(void)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1O(res, 141);
    return res;
}

/**
 * write free accu mode
 * @param val bits[15:0] = accu_loop[15:0]
 * @param trigger start trigger for current channel
 */
INLINE void GNSS_FREE_ACCU_WR(uint32_t val, uint32_t trigger)
{
    CUSTOM_INSTRUCTION_MACRO_2I(val, trigger, 18);
}

/** write free update
 * @param val bits[15:0] = pll_loop[15:0], bits[31:16] = dll_loop[15:0]
 */
INLINE void GNSS_FREE_UPDATE_WR(uint32_t val)
{
    CUSTOM_INSTRUCTION_MACRO_1I(val, 19);
}

/**
 * read free accu
 * @return val bits[0] = accu_loop[0], bits[1] = accu_loop[1], ...
 */
INLINE int32_t GNSS_FREE_ACCU_RD(void)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1O(res, 133);
    return res;
}

/**
 * read free update
 * @return bits[15:0] = pll_loop[15:0], bits[31:16] = dll_loop[15:0]
 */
INLINE int32_t GNSS_FREE_UPDATE_RD(void)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1O(res, 136);
    return res;
}

/** write joint configuration
 * @param val bits[15:0] = joint_use[15:0], bits[31:16] = costas_use[15:0]
 */
INLINE void GNSS_FREE_JOINT_WR(uint32_t val)
{
    CUSTOM_INSTRUCTION_MACRO_1I(val, 25);
}

/**
 * read joint configuration
 * @return bits[15:0] = joint_use[15:0], bits[31:16] = costas_use[15:0]
 */
INLINE int32_t GNSS_FREE_JOINT_RD(void)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1O(res, 145);
    return res;
}

/// perform single tracking step
INLINE void GNSS_TRACK_STEP(uint32_t val)
{
    CUSTOM_INSTRUCTION_MACRO_1I(val, 2);
}

/// set channel index
INLINE void GNSS_CHANN_SET(uint32_t chanNr)
{
    CUSTOM_INSTRUCTION_MACRO_1I(chanNr, 137);
}

/// get channel index
INLINE uint32_t GNSS_CHANN_GET(void)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1O(res, 134);
    return res;
}

/// set bank index
INLINE void GNSS_BANK_SET(uint32_t bankNr)
{
    CUSTOM_INSTRUCTION_MACRO_1I(bankNr, 143);
}

/// get bank index
INLINE uint32_t GNSS_BANK_GET(void)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1O(res, 144);
    return res;
}

/**
 * @}
 *
 * \defgroup carrier_nco Carrier NCO
 * @{
 */

/// set carrier frequency
INLINE void GNSS_CARR_FREQ(uint32_t freq)
{
    CUSTOM_INSTRUCTION_MACRO_1I(freq, 135);
}

/// execute carrier discriminator
INLINE int32_t GNSS_CARR_DISC(int32_t disc)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1I1O(res, disc, 130);
    return res;
}

/// get carrier sample
INLINE uint32_t GNSS_CARR_REM(uint32_t val)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_2I1O(res, val, 0, 131);
    return res;
}

/// set carrier register
INLINE void GNSS_CARR_SET(uint32_t val, uint32_t conf)
{
    CUSTOM_INSTRUCTION_MACRO_2I(val, conf, 3);
}

/**
 * @}
 *
 * \defgroup correlator Correlator
 * @{
 */

/// accumulator add
INLINE void GNSS_ACCU_ADD(uint32_t vIn, uint32_t codes)
{
    CUSTOM_INSTRUCTION_MACRO_2I(vIn, codes, 4);
}

/// accumulator get
INLINE int32_t GNSS_ACCU_GET(void)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1O(res, 132);
    return res;
}

/**
 * @}
 *
 * \defgroup discriminators Discriminators
 * @{
 */

/// compute PLL discriminator
INLINE int32_t GNSS_PLL_DISC(int32_t I, int32_t Q)
{
    volatile int32_t res;
    CUSTOM_INSTRUCTION_MACRO_2I1O(res,I,Q,256);
    return res;
}

/// define costas discriminator
#define GNSS_COST_DISC GNSS_PLL_COST
/// compute costas discriminator
INLINE int32_t GNSS_PLL_COST(int32_t I, int32_t Q)
{
    volatile int32_t res;
    CUSTOM_INSTRUCTION_MACRO_2I1O(res,I,Q,257);
    return res;
}

/// compute DLL discriminator
INLINE int32_t GNSS_DLL_DISC(int32_t I0, int32_t Q0, int32_t I2, int32_t Q2)
{
    volatile int32_t res;
    CUSTOM_INSTRUCTION_MACRO_2I(I0,Q0,5);
    CUSTOM_INSTRUCTION_MACRO_2I1O(res,I2,Q2,261);
    return res;
}

/**
 * write PLL joint parameters
 * @param alpha Alpha parameter for variance calculation, signed Q15.16
 * @param weight Discriminator weight in joint tracking, signed Q15.16
 */
INLINE void GNSS_PLL_JOINT_WR(uint32_t alpha, uint32_t weight)
{
    CUSTOM_INSTRUCTION_MACRO_2I(alpha, weight, 26);
}

/**
 * write DLL joint parameters
 * @param alpha Alpha parameter for variance calculation, signed Q15.16
 * @param weight Discriminator weight in joint tracking, signed Q15.16
 */
INLINE void GNSS_DLL_JOINT_WR(uint32_t alpha, uint32_t weight)
{
    CUSTOM_INSTRUCTION_MACRO_2I(alpha, weight, 27);
}

/// readout PLL discriminator variance
INLINE int32_t GNSS_PLL_VARI(void)
{
    volatile int32_t res;
    CUSTOM_INSTRUCTION_MACRO_1O(res, 146);
    return res;
}

/// readout DLL discriminator variance
INLINE int32_t GNSS_DLL_VARI(void)
{
    volatile int32_t res;
    CUSTOM_INSTRUCTION_MACRO_1O(res, 147);
    return res;
}

/**
 * @}
 *
 * \defgroup filters Filters
 * @{
 */

/// PLL filter reset
INLINE void GNSS_PLL_FLT_RST(void)
{
    CUSTOM_INSTRUCTION_MACRO(7);
}

/// DLL filter reset
INLINE void GNSS_DLL_FLT_RST(void)
{
    CUSTOM_INSTRUCTION_MACRO(9);
}

/// PLL filter set coefficients (second order)
INLINE void GNSS_PLL_FLT_COEF(uint32_t c1, uint32_t c2)
{
    CUSTOM_INSTRUCTION_MACRO_2I(c1, c2, 6);
}

/// PLL filter set coefficients (third order c1 and c2)
INLINE void GNSS_PLL_FLT_COEF_A(uint32_t c1, uint32_t c2)
{
    CUSTOM_INSTRUCTION_MACRO_2I(c1, c2, 21);
}

/// PLL filter set coefficients (third order c3)
INLINE void GNSS_PLL_FLT_COEF_B(uint32_t c3)
{
    CUSTOM_INSTRUCTION_MACRO_1I(c3, 22);
}

/// PLL filter set aiding
INLINE void GNSS_PLL_FLT_AID(uint32_t aid)
{
    CUSTOM_INSTRUCTION_MACRO_1I(aid, 23);
}

/// DLL filter set coefficients
INLINE void GNSS_DLL_FLT_COEF(uint32_t c1, uint32_t c2)
{
    CUSTOM_INSTRUCTION_MACRO_2I(c1, c2, 8);
}

/// DLL filter set aiding
INLINE void GNSS_DLL_FLT_AID(uint32_t aid)
{
    CUSTOM_INSTRUCTION_MACRO_1I(aid, 24);
}


/// PLL filter get
INLINE int32_t GNSS_PLL_FLT(int32_t disc)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1I1O(res, disc, 262);
    return res;
}

/// DLL filter get
INLINE int32_t GNSS_DLL_FLT(int32_t disc)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1I1O(res, disc, 264);
    return res;
}

/**
 * @}
 *
 * \defgroup code_generator Code Generator
 * @{
 */

/// get code sample
INLINE uint32_t GNSS_CODE_GET(void)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1O(res, 138);
    return res;
}

/// set code generator address
INLINE void GNSS_PCODE_ADDR_SET(uint32_t addr, uint32_t nco)
{
    CUSTOM_INSTRUCTION_MACRO_2I(addr, nco, 10);
}

/// write data
INLINE void GNSS_PCODE_WR(uint32_t val)
{
    CUSTOM_INSTRUCTION_MACRO_1I(val, 11);
}

/// diagnostic read data
INLINE uint32_t GNSS_PCODE_DIAG_RD(uint32_t addr)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1I1O(res, addr, 258);
    return res;
}

/// diagnostic write data
INLINE void GNSS_PCODE_DIAG_WR(uint32_t addr, uint32_t data)
{
    CUSTOM_INSTRUCTION_MACRO_2I(addr, data, 28);
}

/// set code length
INLINE void GNSS_PCODE_LEN(uint32_t len, uint32_t integr_mult, uint32_t coef, uint32_t scale)
{
    uint32_t val = (((integr_mult&255)<<16)|(scale&7)<<4)|(coef&15);
    CUSTOM_INSTRUCTION_MACRO_2I(len, val, 12);
}

/// set main ,boc and time multiplex
INLINE void GNSS_CODE_NCO_FREQ(uint32_t freq, uint32_t mod)
{
    CUSTOM_INSTRUCTION_MACRO_2I(freq, mod, 13);
}

/// set epl and code frequency
INLINE void GNSS_CODE_EPL_FREQ(uint32_t epl, uint32_t code)
{
    CUSTOM_INSTRUCTION_MACRO_2I(epl, code, 14);
}

/// set code generator address
INLINE void GNSS_SCODE_ADDR_SET(uint32_t addr)
{
    CUSTOM_INSTRUCTION_MACRO_1I(addr, 15);
}

/// write data
INLINE void GNSS_SCODE_WR(uint32_t val)
{
    CUSTOM_INSTRUCTION_MACRO_1I(val, 16);
}

/// set code length
INLINE void GNSS_SCODE_LEN(uint32_t len)
{
    CUSTOM_INSTRUCTION_MACRO_1I(len, 17);
}

/// execute code discriminator
INLINE int32_t GNSS_CODE_DISC(int32_t disc)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1I1O(res, disc, 139);
    return res;
}

/// get pseudorange parameters of currently selected channel
INLINE uint32_t GNSS_CODE_RNG(int32_t channel)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1I1O(res, channel, 140);
    return res;
}

// set bits recovery
INLINE void GNSS_BITS_CFG(uint32_t status)
{
    CUSTOM_INSTRUCTION_MACRO_1I(status, 29);
}

// read bits recovery
INLINE uint32_t GNSS_BITS_GET(void)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1O(res, 148);
    return res;
}

// read I2 value
INLINE uint32_t GNSS_I2_VAL_GET(void)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1O(res, 149);
    return res;
}

// read Q2 value
INLINE uint32_t GNSS_Q2_VAL_GET(void)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1O(res, 152);
    return res;
}

// read m4 value
INLINE uint32_t GNSS_M4_VAL_GET(void)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1O(res, 150);
    return res;
}

// read m2 scale value
INLINE uint32_t GNSS_M2_SCL_GET(void)
{
    volatile uint32_t res;
    CUSTOM_INSTRUCTION_MACRO_1O(res, 151);
    return res;
}

/** @} */

#endif /* __CCRV32_GNSS_ISE_H__ */

/** @} */
