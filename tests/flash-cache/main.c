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
* $Date: 2025-04-16 19:16:38 +0200 (śro, 16 kwi 2025) $
* $Revision: 1146 $
*H*****************************************************************************/

#ifndef TSMC40ULPFMC_RAM_BUFFERS
# define TSMC40ULPFMC_RAM_BUFFERS 3U
#endif /* TSMC40ULPFMC_RAM_BUFFERS */

#include <ccrv32-amba.h>
#include <ccrv32.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "board.h"
#if FLASH_SIZE > 0
# include "flash.h"
#endif
#include "test.h"

#ifdef MCU_CCNV2
# define _BOARD_SUPPORTED_ 1
#endif

#ifdef _BOARD_SUPPORTED_

#define PAGE_ALIGN( address ) \
    ( \
        ( address ) \
        & ( \
            ~ ( \
                ( uint32_t ) ( \
                    ( FLASH_PAGE_SIZE ) \
                    - 1U \
                ) \
            ) \
        ) \
    )
#define WORD_ALIGN( address ) \
    ( \
        ( \
            ( address ) \
            | ( sizeof( uint32_t ) - 1U ) \
        ) \
            ^ \
        ( sizeof( uint32_t ) - 1U ) \
    )
#define PATTERN 0x0ABCDEF0
#define PATTERN2 0x12345678

#if 16 > FLASH_PAGE_SIZE
# error "Flash page size too small for test!"
#endif

#define TEST_PAGES_USED ( TSMC40ULPFMC_RAM_BUFFERS + 1U )

#if FLASH_SIZE < ( TEST_PAGES_USED * FLASH_PAGE_SIZE )
# error "Flash size too small for test!"
#endif
/*
 * assumes which first page is free
 * since there's no pointer to end of programmed executable data
 */
#define TEST_FIRST_FREE_PAGE \
    ( FLASH_SIZE - ( TEST_PAGES_USED * FLASH_PAGE_SIZE ))

#define ARRAY_SIZE( ARRAY ) ( sizeof( ARRAY ) / sizeof( ARRAY[ 0U ] ))
#define STATUS_OK( STATUS ) ( PROGRAMMING_ERROR > STATUS )

#endif

static uint32_t const pattern[] = {
0xa81432dd, 0x06b7b51a, 0x6a0d6959, 0x5dc7923e,
0x445c2741, 0xa9edcd46, 0xb87f99f1, 0xbf550166,
0x4aa62dd7, 0x977dffea, 0x60f242c1, 0xce5f8efc,
0x4a079c11, 0x4623e487, 0x2f624fd6, 0x8ef709b9,
0x7afed9e2, 0xf7414de1, 0x2a2b9ce3, 0xaf9705b5,
0xa013d45d, 0x9bb6f669, 0x6621bd82, 0x24ff246c,
0x9850c501, 0x0c29cd5b, 0xc26c174d, 0xb25d4c2e,
0xf8652f11, 0x9d92843e, 0x92db9977, 0x9b1aacde,
0x18efbcbc, 0x8c526700, 0x25238aa2, 0x42cf8a20,
0xabbdc440, 0x688fa56b, 0x348d6cb2, 0x1f5d75f9,
0xb764867f, 0xce59b1b7, 0x0698ed3c, 0x5b227d51,
0xd6ec59f0, 0xdc666d53, 0xcb326455, 0x1dc0ff97,
0x4f1c04c5, 0xba24d7cc, 0xc43202aa, 0xafe8351b,
0xe095d9c6, 0xe38a6c54, 0x86590afd, 0x43c78ead,
0xc27092e5, 0x8dd0c61e, 0x9f5ba8ad, 0x13e97987,
0x280bf31e, 0xdd9f6969, 0x24e4a99c, 0x827a66a4,
0x8e4ae2b7, 0x2708a39a, 0x4915eb8c, 0x5b59537e,
0x29c74be6, 0x8be01452, 0xf0298d14, 0xd0b02911,
0xceceab39, 0xb4d697a8, 0xfcf3a120, 0xca637e92,
0x7bca5685, 0x60ff552a, 0x4b188c39, 0xd637eaf1,
0x89672f55, 0xfcea82a6, 0x2f9f4b68, 0x9baecca0,
0x61af68e0, 0xfbb1650d, 0x8233589d, 0x6192d1fd,
0xdb52b7aa, 0x57719a32, 0x3a1b5330, 0xc9e59223,
0x3a12b531, 0x6ff6f734, 0x390eab2b, 0x10aa9a09,
0x93c260e1, 0x713bd0a2, 0x8e43fb02, 0xb89c9a96,
0xc33f3d9c, 0xa8465df2, 0x56d12cbf, 0x65e5bc6c,
0x8008a28a, 0x7fd98b4c, 0x3870d4b7, 0xd900880f,
0xa5a2455e, 0xc7861fd2, 0xd65015e1, 0x3c296971,
0x57aae462, 0x7d777a85, 0x67bdbc84, 0x30a50288,
0x08f90f7f, 0x1643df2d, 0x00ca435e, 0xc9035f30,
0x920302f6, 0x2523be2b, 0xb8fa0875, 0x22337590,
0x95ff1355, 0x3a1a0fe8, 0xd39f36e8, 0x8db0d8ba,
0x1a093720, 0xf7d5858f, 0x6ae3c77e, 0x86c4dd62,
0xa6542107, 0x5dd928cb, 0xa2199c1a, 0xf44e74a4,
0x0494e336, 0x7ba79d8c, 0xb8ea3002, 0x633c5e4a,
0x1939045a, 0xe1f0ee81, 0xd023d2c6, 0xbfcab643,
0x21b24e6c, 0x79f24220, 0x4b7202a8, 0x8dd70441,
0xb2858aeb, 0x2875c0bb, 0x6dfc296f, 0x080f8ef1,
0xc865ad38, 0x92834b5b, 0xa6800c2d, 0x73d8e237,
0xb9099466, 0xa2c2b9d0, 0x1a9f6774, 0xa1b86c98,
0xd84f0d4b, 0x12fc1faa, 0xe153ef5a, 0x03905ca9,
0x7739179b, 0x760484a8, 0xa91bf539, 0x032bc15f,
0x1354f0ce, 0xee79d3b1, 0xe60f7290, 0xcc3fce0c,
0x715de628, 0x0a93551a, 0x6a02a34e, 0x44dbc7d7,
0x6c023cf8, 0xc57db339, 0x6b0c9297, 0xa1158d1e,
0xf88fa4ee, 0xca5720d3, 0xb5bd7183, 0x98ad521c,
0xb46c4e3a, 0xf11903f5, 0x3facd1b2, 0xb2689647,
0xa88c8fca, 0x7a9edf12, 0x4ad80bde, 0x5e093942,
0x4187dc90, 0x68442426, 0x5d0c2917, 0x33f961e5,
0xf72f206c, 0x6117910b, 0x8edd5758, 0xbf810c6e,
0xbb8b9e72, 0x441e2233, 0x1ce5f4b6, 0x28575db6,
0x8797b097, 0x12c994fa, 0xbe9c0b89, 0x3661ec4a,
0x72e0428e, 0x4679c837, 0xdf36db4f, 0x318f1c3a,
0xb6753919, 0x46ef7dd6, 0x0fa5c3b3, 0xee594e21,
0x516a87a1, 0x6fdc3872, 0xa66c27d0, 0xf14b9306,
0x937c398a, 0x0297e60f, 0x192b199f, 0x001a0bfc,
0xaf180e69, 0x2119e078, 0x449d30bb, 0x5f48bdf5,
0xaf996b13, 0x87a1234b, 0x4c798ede, 0x75c53c48,
0x000b6622, 0x9b28a16e, 0x87a4fed7, 0xbf2c41bf,
0xda2b889c, 0xa72c905a, 0x67c774c5, 0xe8605b75,
0x435fadde, 0xdd9ca914, 0x58bf72e1, 0x609129e6,
0x35d1825b, 0x240d4a84, 0xfe16d7df, 0x1df33aa4,
0x6ed0e9e9, 0x1bd959cc, 0x5a2e9298, 0x8d9f2690,
0x026dbc97, 0x6923df0e, 0xcc4db2ef, 0x4d7db75e,
0x16f793a3, 0x038cd715, 0x6cedcba8, 0xd6fab191,
0xb7be4afa, 0x1e1a9428, 0x6652c982, 0x0d6a3574,
0x9c6df0d3, 0x65676cd9, 0x4c38a492, 0x1d18f1f2,
0x4c521985, 0xb19604cb, 0x213d671a, 0x9e0dc2b2,
0x2dca86d9, 0x64e82054, 0x1a8c451f, 0xa53e7f81,
0x7e6ccba6, 0x12564cc4, 0xab160940, 0xcc413b42,
0x54b81b14, 0x40012964, 0xe94b5619, 0xfdaaa1cb,
0x1f953482, 0x59330166, 0xe13638c8, 0x2a6258bc,
0x05965246, 0x53b7e494, 0x72c4cc28, 0x6efab31d,
0xa63fc846, 0xddd8b677, 0x6ff4f8e5, 0x410a3ddd,
0xaa6d8355, 0x042107e1, 0xd4d5011f, 0x4859e6e5,
0x5b5587f0, 0x68805747, 0xc50313df, 0xaa7924f6,
0xc4e2b8d1, 0xb788b972, 0x6265155c, 0xe020aadd,
0x76c59261, 0x9b17f4ad, 0xe2e79cc3, 0xcadfdc6e,
0xd27f9f69, 0xffed03a3, 0x6a40ddcf, 0x6ba94098,
0x9a04c677, 0xdf4cf5a0, 0x838cb01a, 0xc6f33765,
0x3e86d0cd, 0xb61563b7, 0x76c1bebb, 0x1b5ab35b,
0xf1af07ea, 0x995f9ecb, 0xf0082f3e, 0x965dc05d,
0x138cafe0, 0xf433157b, 0x7d5a05f9, 0x99462928,
0x97c3968d, 0x2cf662fe, 0xe89ac9df, 0x7c8dcd8c,
0x73c37838, 0x103a2a51, 0x8400bf6e, 0x43723f15,
0x2ead05f0, 0x791af9eb, 0xeb49f71f, 0xc922cf66,
0x5aeaf432, 0xec5a73ac, 0xfddeaa78, 0xcf5e7cf4,
0xeb1b1695, 0x3acd01c8, 0xc4703515, 0x7c402144,
0x1f793787, 0x03261d9e, 0x47692348, 0xabdcc96c,
0x379767ba, 0xd4a9e8cd, 0x2c25e71a, 0x11aa6311,
0x5a30ac78, 0x41ce1f3d, 0x756a694e, 0x46746b3d,
0x469ab821, 0x8147e66d, 0x98627874, 0xc676e810,
0x917e334f, 0x1eaeafaa, 0x17321c35, 0xf6d8e8c9,
0x8a48e07e, 0xd1dd1172, 0x1dc321bd, 0x6c98f8c9,
0xa75c5fab, 0x8fecd68d, 0x9523c858, 0xba53a9cf,
0xd66b12ed, 0xa7e1b1ec, 0x87e4b7d7, 0xff10b3e3,
0xce69d149, 0x8add960f, 0xd10362c2, 0x4803c464,
0xf2778adc, 0x305b26e0, 0xce0586f4, 0x9c77528c,
0x334b61d8, 0x8c2e8daa, 0x58a29a41, 0x16294f06,
0xb3138461, 0x05c62b23, 0x2b934907, 0xf99e1536,
0x99d138ea, 0xbdf062eb, 0x132c9afb, 0xe24b4ce2,
0xfb5c28f0, 0xca9528d4, 0xb8972054, 0x5e8963d1,
0xa510bac2, 0xf0003d6d, 0x30ddd6a6, 0xa1fc94d5,
0x91061100, 0x120b5d5d, 0x0230ad42, 0x8a87c996,
0x61c64367, 0x19e69a9c, 0xc4826682, 0x6707da16,
0x30886f8e, 0xb6edead6, 0xb524efdd, 0x68007d26,
0x51d4f143, 0xf69266e5, 0x3b6acf02, 0xbbebc45b,
0x36d6ba42, 0x81dee90c, 0x321cbe8a, 0x42b39212,
0xcf202dbb, 0x60dd4cea, 0x14e07a77, 0x0e6a5bcc,
0x8f6246de, 0x626e1ed3, 0x010ed27b, 0x8c18c8c2,
0x322206a2, 0x409d9ac2, 0x4221f3ec, 0xba1de034,
0xc5ef0bdd, 0x46b7fbfa, 0xf4ed909e, 0x30d39b76,
0x25f89a16, 0x35000533, 0x44e903e1, 0x9d210e79,
0xc74dd079, 0xeb611ec4, 0xc78d27c3, 0x09f990a2,
0xcd795a9a, 0xe21591a5, 0x8337dae0, 0x1997e06a,
0x936557e6, 0xec75c023, 0xcd470c26, 0xff5b63fa,
0x0944fe50, 0x0b1ae711, 0x982514ae, 0x7f49fd89,
0xb552ec9b, 0x8f2b3a81, 0x90d0c0c0, 0x3be5ca45,
0x70cda613, 0xeee9d21e, 0x46f1913a, 0x69e25f54,
0x4e294ddb, 0x1e92e4ea, 0x9ca1e4ca, 0xfbdeb2a3,
0xc8b70535, 0x6ac8a042, 0x2226e639, 0xf50dfcd6,
0x4cdb0b28, 0xfeae04e0, 0x85cc091e, 0x9537edf3,
0xa01ce6b2, 0x497cbec3, 0xe72c8d03, 0xcf1f48cd,
0x97eb270b, 0x8befc1ea, 0x6322dca1, 0x050ad395,
0xa5ed8fa8, 0xe616a8cd, 0xe0162116, 0xd651b91d,
0x0cdbd013, 0x55353a15, 0x59ff2eba, 0xdbf29f7e,
0x33ed7d9d, 0x37e92b28, 0x469ef88d, 0x046d3b51,
0xe33efdf6, 0x487c5957, 0x5ba9a43b, 0xf34b3164,
0xf4a755c4, 0x4aa8e279, 0x32d7ade9, 0xf1bceb3a,
0x6d759fd2, 0xcc4a3299, 0x37b7daa1, 0x097e4f54,
0x89d74f58, 0x6c21f4d5, 0xdd55f2d6, 0xc83243df,
0xaf5702a8, 0xd23ecd6d, 0xd8ef854a, 0x91228dc7,
0xe4513012, 0xa7be6a49, 0x19fbcce0, 0xc1d6214f,
0x9fffa233, 0x0440a92f, 0x1c834435, 0xc6f91f51,
0xbbbc2457, 0xeacedfb8, 0xdd461e02, 0x347954ae,
0xb29fa65f, 0xa020d437, 0xe8562f6f, 0xbdae2572,
0xe3c5cbce, 0x8585a23c, 0x53864a5c, 0x2a020d96,
0xe6d31295, 0x8b379848, 0x9f202588, 0x7dda703f,
0xf91b3878, 0x2ec9857d, 0x657f37eb, 0x3006a781,
0x92055bd8, 0x6a0e95b1, 0x1701449a, 0xc083977c,
0xeb83e51b, 0x78907c38, 0xb97c9e10, 0x7700b156,
0xa4475824, 0x5c4418d9, 0x23940f61, 0xdbe2fb47,
0x0d867428, 0xcad8f750, 0x3dc6be1b, 0x3591bcb1,
0xc9c641c3, 0x70fce83e, 0xdff72ca0, 0x76284311,
0xbe81c2bf, 0x584754e6, 0xfe702918, 0x05974b5d,
0x299d976c, 0x0ba44f25, 0x4c31d997, 0x1181bde3,
0x8de9dd0a, 0xacb05408, 0x6727635b, 0x3a65bdf4,
0x07ee18ff, 0x3f82e032, 0x8440ed85, 0x1e9c0557,
0xa7234886, 0x644b895d, 0x8a17a865, 0x97897916,
0x140b1afe, 0x9d1f89e7, 0xb6153199, 0x45702955,
0x8d396c87, 0x70e6d1e8, 0x22777ddd, 0x4366d74c,
0x1b504594, 0x4c27ec22, 0x44ec1924, 0xd7190eb5,
0x6fc8a650, 0x921c3792, 0x5940b282, 0x903bea6d,
0x37b7a013, 0xa21a46be, 0x52687e25, 0x3b08992a,
0xa4d57a7e, 0x9b5569dd, 0x928dec91, 0x32c6bcc2,
0xe743f66e, 0x429982c6, 0x8b36bded, 0xab221206,
0x855a76a5, 0xd545620b, 0x8f5e12a2, 0x21c0e496,
0x09ec4fd4, 0x1f6758bb, 0xb0a3b945, 0x395a72df,
0xc351981d, 0xa0f38c1c, 0x3a62f696, 0x9313fa9f,
0x0100b5fe, 0xad6e2461, 0x727c2878, 0xaed6b602,
0xfee2a2f0, 0x0f6f3ae0, 0x53feb486, 0x3c6af8e5,
0x50713a72, 0x0f36959f, 0x5c6c6ba3, 0x06fba5dd,
0xc337b5d8, 0x9a644166, 0x3ef6f82a, 0x1d85ef05,
0xf6f694fa, 0x849bd274, 0x37b5d943, 0x8fbdb7cb,
0x00302645, 0xe9494c69, 0x8dcfe049, 0x24929ada,
0x1c962495, 0x3c9e0cb9, 0x2b95a531, 0x73834085,
0x4d63d97f, 0xd0b474bb, 0xee695587, 0x8561858b,
0x9ae156b1, 0x00baf5fb, 0x5c170abe, 0x2d95fecc,
0x57bb80c5, 0x23a1949e, 0x881a75c3, 0xc9ae568a,
0x8d4a6295, 0x9a1765d2, 0x9068b117, 0x4924f3da,
0x4eeb74ec, 0x1d223a4b, 0xe3698554, 0x14f66b49,
0xeb66e032, 0x2a15083f, 0x655a32ce, 0xc1b553c4,
0x34ba6bc2, 0x2e8f2284, 0x687814de, 0xbd514bf6,
0x8213192d, 0xfc75b079, 0xbc1b4039, 0xbf036459,
0x133c0427, 0x7363ee04, 0x65a3dae7, 0xc748bae2,
0xa5f458cb, 0x4f586f51, 0xd1dd5b91, 0x511c0263,
0x13f9e1f6, 0x09e55f03, 0xaa25ede9, 0x4bd475d5,
0x63b8f0ee, 0x5a859a37, 0x04c24349, 0xa94ba5de,
0xff07b5c7, 0xa426f124, 0x6380c3e9, 0xac73b0a3,
0xbda87f48, 0x2ae359ad, 0x9264bf1e, 0x27de92be,
0x706aecb6, 0xe1d01d76, 0x8518e5d4, 0x76cb928e,
0x927aa2dd, 0x252a42b1, 0x398a34e2, 0xa69acf77,
0xbb264ff8, 0x64fd194a, 0x463dbb93, 0x6266dbb1,
0xc4231723, 0x8c556b2e, 0x32ed1bb5, 0x6bf25730,
0xd4d0f723, 0x9e8c4521, 0xdb4dbdaf, 0xd5d838ea,
0x9f39bda2, 0x170677df, 0xecbb9733, 0xec89145d,
0x95c4fd4a, 0x68a3dbcf, 0x35273840, 0xf63c24a0,
0x47db5ba0, 0x31540c20, 0xa76d40fd, 0x516a1340,
0xb5f052dc, 0xb9544deb, 0xd9e234a7, 0x90aa9f07,
0x4dca3d96, 0xb1859d5d, 0xcb5a5d0a, 0x0b9bcb7b,
0xd2fa5d70, 0xb79d4746, 0x6617b14c, 0x963b495d,
0x06384f19, 0x66ac8514, 0x3a7f9172, 0x00771acf,
0x4a352b2a, 0xddc9ee9d, 0x9a978808, 0x3e5cd527,
0x9422a96e, 0x58d43276, 0xa89e1028, 0x9acc3fe4,
0xaaed37b3, 0x1145ca34, 0xaaba7b38, 0x2582fdcb,
0xc9e42696, 0x3bdd9304, 0x1ad13c0a, 0x1ef7e8b7,
0x70cbd81a, 0x142c3ea1, 0x7deee4dc, 0xe273d02e,
0x5870abe3, 0x3b1d2f64, 0x36cc43e5, 0x93c99585,
0xf2f77c38, 0x335bc03f, 0xe15cca68, 0x5ad3c10f,
0xf52d7508, 0x6e507581, 0x3d2338de, 0xe6bbd431,
0x4eb508c5, 0x1e4ad1b8, 0x44f3865e, 0x756e107d,
0x10aaa37b, 0x125aa596, 0x093ef66c, 0x6ed9bd46,
0xa9a4465d, 0x4813a17c, 0xb623a997, 0x634ecda3,
0xd8f03664, 0x2b9deefd, 0x0af128c2, 0x25085da2,
0xb4379a5a, 0xd4b52cfe, 0xc20bcc4d, 0xfa69985d,
0xf62e6660, 0x1e7f69c9, 0xafcaa356, 0x9c6750cb,
0xe86a6aa1, 0x80dc8db9, 0x7b8adcbf, 0xba4b18f6,
0x05a3955c, 0x8fab2b19, 0x9efc09a6, 0x23f246b5,
0x73cc635e, 0xb271a179, 0x83021ccd, 0x8f8b8509,
0x529f24af, 0x7830c8b6, 0x733f5294, 0x63f22056,
0x83655329, 0x6ca33edf, 0xf8dec058, 0xd3df21af,
0x717f5417, 0xb11acde9, 0x0721a5e4, 0xa0e5d6f7,
0xca2bae6e, 0x04d03d9e, 0x22533582, 0x9f3d995d,
0x2170062e, 0x2f2e1030, 0x92d48ff0, 0x0fd6a632,
0xc5f4935f, 0x5d402945, 0x41874baa, 0xd627238c,
0x61b07f22, 0xec2f5711, 0x1ab4bc98, 0x0067b09b,
0xcd643b60, 0xcc263ccc, 0xc6b05ea6, 0x9d1ba014,
0xf1ed7d7b, 0x15a85bb5, 0xa7f7a8e6, 0xbd48c905,
0xfd154f1a, 0x3f364f99, 0x921ae87b, 0x4b84fbab,
0x7089cf77, 0x40ccd36e, 0x74a8a21b, 0x50e57407,
0xd7a6f2fe, 0x53f198ed, 0xf1aafc8e, 0xc6546d9b,
0x80b73964, 0x2ae103f7, 0x0587dd95, 0x4a0892ed,
0x36f41cf2, 0xfd5c5d26, 0xa42d6dce, 0xf8c6d670,
0xa0b92840, 0xf84094cc, 0xf8d9c355, 0x9017980f,
0xde25e1aa, 0x876fdb84, 0xf2db241b, 0xf919a38d,
0x619939f1, 0x67b1bdb6, 0x32d4ca47, 0x7690d32e,
0xcc7fe904, 0x9b8968a8, 0x64c8c098, 0x5f72204e,
0xd1942079, 0xf77c5330, 0xb2e62db3, 0xcb8b3612,
0x2b09908b, 0x3b26fe5f, 0x5d193bc1, 0x4eb113d5,
0xa424f38e, 0xf1422a06, 0x1696c090, 0x8cbdb0ed,
0x94225e37, 0x10a999d9, 0x48c7ca40, 0x30f83db4,
0x4ee8dd73, 0xfaaa8290, 0x1a3915ff, 0xa326a7fb,
0x0fe9fee3, 0xb1235306, 0x924efcd8, 0x83a4f6e9,
0x59356aa1, 0xee0cd1b1, 0xa482d9e1, 0x5e30297c,
0x5d327b0a, 0x242419d5, 0x592f924d, 0xeaaa4129,
0xd694c067, 0xed709ab7, 0x514759b8, 0x255b2a96,
0x0ab07eb3, 0x85a19a92, 0xb91969b5, 0xe77b9fe6,
0x220e1718, 0xf132d59c, 0x63bd720e, 0x8b24d0da,
0xa09190fc, 0x07f013a9, 0x90a82da3, 0xed6e8c31,
0xa6d17b68, 0x31905735, 0xb8aa603f, 0x141ba214,
0xcfdea4ea, 0xd8657995, 0x333a9d68, 0x217f254d,
0xc2f23358, 0x04cb94ff, 0x03598512, 0xd5658c4f,
0x97700a7e, 0x2d79a4db, 0xec882a60, 0x330c7ad7,
0x6ea49498, 0xc308779a, 0xf7e1d76c, 0x3e78ab39,
0x7298d117, 0x769cd3ae, 0x3abb3949, 0xb88d0438,
0x7bd61674, 0xd99e557e, 0x954d6be1, 0x2db8d8ae,
0x7a2463c2, 0x0d7f507d, 0x7ee14b05, 0x2c1911b1,
0x820c815f, 0x4ab569f6, 0x87162f79, 0x998cf95c,
0x88c1185f, 0xfb422d63, 0xdb26bccf, 0xa30874d0,
0xa1d0816e, 0x24d21985, 0x38255bac, 0x2459a127,
0x29999b13, 0x379eb2c4, 0xd164659c, 0xe2dbb08e,
0x424d64c2, 0xe71089e9, 0x75a77aa7, 0x13577a37,
0xebb2f0c9, 0xc197af75, 0x0546f8b5, 0x16a1aef1,
0x73f572fc, 0xd8ee583c, 0x02b5475c, 0x3506145a,
0x7b63c31f, 0x365a78ce, 0xdee05e16, 0x678a9360,
0xe709af01, 0xd12355b8, 0x5652dd32, 0xa463ff50,
0x87694cc9, 0xa58694c3, 0xa81de3ec, 0x46f4a392,
0x44f8da6e, 0x54b0985a, 0x1a30c010, 0x3282123a,
0x360f206e, 0x60a20195, 0xaa047454, 0x911ecf1f,
0x2c45bdf4, 0xd1d9d092, 0x7c3da9f7, 0xc9210177,
0x8bf78627, 0x421d97f6, 0xeba8fd52, 0xc8d70971,
0xcbb90cac, 0x391d0a26, 0x1fea83b6, 0x37040505,
0xe0985d30, 0x6f5386d1, 0x7cf028dc, 0x8b9f39b8,
0x8b039f9b, 0xf961d60b, 0xfd3582a2, 0xcfe240cf,
0xb00d8e6b, 0x80d8f483, 0x4bfa232f, 0x528164be,
0x414b1460, 0x0673bf99, 0x057e8904, 0xb44d2482,
0x5f442250, 0x07da5c7b, 0x4d81203b, 0x673ce0ef,
0xb490dbbd, 0x87ed8b43, 0xd459772e, 0x1dc552ba,
0xd6c13bc4, 0x122b3eec, 0x5868a199, 0x863bef4a,
0x9c32da1b, 0x398e59f0, 0x29101d33, 0xe434c2fd,
0xfaddd2f5, 0x61393974, 0x37821b45, 0xb9cb393c,
0xb067a60f, 0xbb26ada1, 0x4bffa265, 0x11da7cb9,
0x5f619e3d, 0xbacedbe4, 0xda1c4290, 0xca2f9684,
0xa5e7343a, 0xa641efd1, 0xb2317030, 0xd3ae89d1,
0xa7a953ba, 0x3a25f1d5, 0x4e1ddd42, 0xc7c12235,
0x3551fe2c, 0x5c5ea83f, 0x7c891e50, 0x015d46a7,
0x64be1a68, 0xa4a6a1f4, 0x1d43c937, 0x8f8016cc,
0x168d5e8b, 0x0eb394aa, 0xee239ed9, 0x5c9d0d91,
0x55ad4be3, 0xfdc179d3, 0xb214bbab, 0xcb54e0bd,
0x864c2bf7, 0x2636bb16, 0xc9d208ac, 0x1a69076a,
0xfe5693c9, 0xba7898b2, 0x196dd688, 0xc7bc7cb9,
0x625b9f68, 0x8a3966c0, 0x347c0e2d, 0xe226469e,
0x6da0fdb4, 0x2d49a0f3, 0x51260f12, 0x324cf68e,
0x8d2852dd, 0xaaccdc43, 0x43cb3b5a, 0x72b4994f,
0x17ea1bc0, 0x6c49c2a5, 0x5426eb72, 0x5e724469,
0xbbabc1be, 0x20cb7b51, 0x45605e88, 0x0b408a89,
0xa3db411b, 0xf5c130ed, 0x4eba992d, 0x868dd999,
0x32aceb25, 0xcc0822da, 0x35108814, 0x44eeb585,
0xf3365071, 0x997f1500, 0xeeb3af72, 0xde0187e9,
0x423ddb4e, 0xbbf7ae4f, 0xbfcffbaf, 0xc32ccee4,
0xacac365c, 0x10185008, 0xf556b416, 0x92a0d9a3,
0x3b13c120, 0xa44018de, 0xe279f001, 0x6b74e7d0,
0x204c8b3b, 0x1dbbc938, 0xbe67d12a, 0x0aae03d5,
0x555f8fb3, 0xb4a9c1ce, 0xa475ee1f, 0xfb04842a,
0xd2aac5a0, 0x8fca9ae1, 0xa7e78ca7, 0xe6378f17,
0xcf7f3e61, 0xb6de9ad2, 0x435afad0, 0x74b6e8c6,
0x8b03d187, 0x08d0a1ed, 0xe9d76c88, 0x477ced27,
0xf28e3e6e, 0xb2aaee94, 0x5abc4559, 0x464ebb03,
0x6fbc463f, 0x471f2ea9, 0x58aed7ef, 0xb259d627,
0x514db7f2, 0x18c9901a, 0x5b96629f, 0x19809483,
0x07deecad, 0x1fd846d5, 0x278aa777, 0x460067d0,
0x78d68a11, 0xde646821, 0x0ca8c97f, 0xb39c43ce,
0x7f8f2d7c, 0xd12ef07f, 0x55c10aa2, 0xd07b999c,
0xa22213ec, 0x7be5b612, 0x0ff81fe7, 0x62c96f46,
0xf2a0a5ec, 0xb9ad14b8, 0x8203bff1, 0x8d4c1a27,
0x9defc9bd, 0x54045ec2, 0xdf298807, 0x08405e6e,
0xef44f3c3, 0x5155ec47, 0xc36c1019, 0x857269e7,
0x72f950c7, 0xe416f645, 0xd572d45b, 0x26bd532b,
0xcd9f0f21, 0x10c5c26e, 0xa491b852, 0xb9ae2ba9,
0x844d726d, 0x4b009e7b, 0xc17a9cda, 0x5c25e8ac,
0xe451d563, 0x228b957f, 0x1a2c280d, 0x5344f0f3,
0x9a2720a5, 0x0dec7a88, 0x736f3c30, 0xa23164fe,
0xc1f30faf, 0x4b032903, 0xee6efb65, 0x021ccd90,
0xe94d17ff, 0xa2ea55f8, 0xc6e7af16, 0xa71df502,
0x77855d55, 0xe7057f0e, 0x64621cee, 0xc64b5931,
0x9c1db4f1, 0xd20f6f32, 0xaab87576, 0xdc5eeede,
0x3a4573c3, 0xe304b489, 0xacf71844, 0x318cdaaf,
0x90a38874, 0x026053b0, 0x3eb86286, 0xe55060c5,
0xb46c2e6d, 0xc322cfe0, 0x79d69dfd, 0x8e449f1e,
0x6f6523f3, 0xe9179219, 0x0450e300, 0x1424b981,
0xcbc072db, 0x4d182db7, 0x54108ea2, 0x8ddd32f9,
0xa580ab38, 0xddfe7330, 0x0bc8ac7d, 0x61d61e5c,
0x998d453c, 0x3ce31235, 0x05844dbe, 0xfe7e2dd6,
0xd2782d82, 0xabab9584, 0x6641edfa, 0x80285c98,
0xacabfa89, 0x01da15dc, 0x20325a47, 0xf66ee705,
0xf0fa873d, 0x587e063a, 0x13572ee9, 0x293d842f,
0x87a6c1dd, 0x9158cc78, 0x6c92bcf5, 0xec437ea2,
0xb9909580, 0x6ef51016, 0x54e4d586, 0x825dd3bf,
0x3e52a055, 0x8578d607, 0x0407489b, 0xdea9f4c9,
0x4f823bdf, 0xc3f58b9b, 0x097c796b, 0x97a3f5cf,
0xc5a9eef2, 0x970b9436, 0x3bb9c9ad, 0x21c67191,
0xf28fd37f, 0x33b42763, 0x2686705c, 0x99f50ee3,
0x8df82637, 0x21e6f30a, 0x3c1bff83, 0x44d1a62f,
0x33d6c821, 0xf569e1fe, 0x7700492b, 0x6f47d57d,
0x11d9c429, 0x094d7ad1, 0x7f6b1e57, 0x6a782682,
0xc8bf62e9, 0x2541bf14, 0x229b7624, 0x56ff7da7,
0xdc66c81b, 0xc986f2d6, 0xadb05670, 0xca3729b9,
0xe55d5547, 0x3d4de5a0, 0x80500aa9, 0x34dbf58e,
0x254bd324, 0xe7fb937c, 0xc7c58950, 0x3da14fab,
0xd1f1b764, 0x87bd23ce, 0xc29d93f7, 0x96cccf60,
0x200aba51, 0x0cc6fee2, 0x25c0c518, 0x5c10bbb9,
0x3eb9ec31, 0x540cd365, 0x2604d2c6, 0xf2a2c9e9,
0x06e0a25b, 0x0a492e36, 0xee189b04, 0x81a944c8,
0xb18322e4, 0x379eb6ac, 0x694ca2df, 0x1dadcc70,
0x3fddc5ca, 0x424bbe42, 0x3a35d683, 0x2c922d5b,
0x6fff54ff, 0xd6e2a664, 0x52efc2d9, 0xbe0f6d3e,
0x01718ffe, 0x86279220, 0x4ac73cf4, 0x9e9a2dcf,
0xa291cccf, 0xc01db74b, 0x92097d12, 0x6cbee216,
0x881e6fb8, 0x94da1ed6, 0xd48856f3, 0x9daf7244,
0xf98a5479, 0xaa4e60df, 0xbf15e13c, 0xd6acb1f1,
0x81a0dc56, 0x246c2976, 0x347da2a3, 0x2e711e60,
0x96fd4707, 0xf5ea3ff6, 0xdee2fc14, 0xd738e8c6,
0x8c67f94f, 0xc52cc411, 0x3bc935c3, 0xd957b1d9,
0x00f10778, 0x9218ca4c, 0x5ba125fa, 0xb95ca972,
0x4fd33f84, 0x62e58987, 0xfa49edfd, 0xe6bb3c02,
0x735d7153, 0xcb8beb09, 0xeac5d208, 0xd239024b,
0xd3ca3a51, 0x11faf13f, 0xe3042e52, 0x7cf72e2a,
0x87bfd208, 0xd261c905, 0x2413402c, 0x2a38bce4,
0xe3371245, 0x46461d90, 0xd60f2f2e, 0xd9f082e7,
0x0c84096a, 0x547106aa, 0x02c4b20f, 0xd65d18a6,
0xa98e0024, 0x687280f7, 0xdb92341f, 0x14b6f1fd,
0xdde3b2f9, 0x2dd691a0, 0x9d0e6e62, 0x77578a6c,
0x4f020835, 0x8a7724aa, 0x314362de, 0xe742e3c5,
0xbfc535cd, 0xa753f4aa, 0x07c453bb, 0xb7b36bbe,
0xbf5000f0, 0xb3fe3c3b, 0xd1259643, 0x5562692a,
0x195d8cfc, 0xdfacc795, 0x5404ea93, 0x4c89b039,
0xe2b06ebb, 0x9d1d0150, 0x495fc5ff, 0x4a759333,
0xf135f5b2, 0x68024503, 0x068a6783, 0xa40a5281,
0x8f38454a, 0xdcb586fb, 0x1fcbdacb, 0xa0cafa61,
0x79992f2c, 0x941b2d3d, 0x34842723, 0x2299d3f7,
0x69d709e9, 0x7f6d29b9, 0x911ca980, 0x0303d390,
0x08aa4d1f, 0x822f649b, 0xdbcac7f1, 0xff4df118,
0xe9296f2b, 0xf1aeaba6, 0x6a621c0b, 0x0f94c8cc,
0x88ca8bbc, 0x11f9a719, 0xe5bf8904, 0x755b3955,
0x4dda4704, 0xa8da34a9, 0xd3640425, 0x2234f945,
0x700e3bca, 0x3c42da3e, 0xed23426c, 0xefff0221,
0x2c6ef51c, 0x801fd6e8, 0xc92ff111, 0x48a36231,
0x1f0e091d, 0xceb77ef0, 0x2afd6224, 0x58943948,
0xda31662c, 0x3092d84b, 0x44e9fc0f, 0x244c0b30,
0x951708ea, 0x594e3b3d, 0x825664c7, 0x28ab16dc,
0x225e1b30, 0x1da2abf0, 0xc0c160ac, 0x732f2e5a,
0x6599c7c3, 0xf68122ae, 0x3e83b67c, 0x8d0a57ce,
0x0d0030c6, 0x7fe7b906, 0x27551c6d, 0xa32e692f,
0xcfc72d36, 0x3eeecfcc, 0x69649567, 0xde70a78f,
0x54389f8c, 0x0e9cffd8, 0x6df1799a, 0xa2e1f49d,
0x960790fe, 0xcf1597f1, 0x5ee0ab53, 0x394d354d,
0xf1a26645, 0x99f0d043, 0x8dd8e532, 0xb0f9ba8b,
0x5b7ef53c, 0x63bc73bd, 0x554fedde, 0xeecffb05,
0xcf0bca0f, 0x6c8a29de, 0x3f96198c, 0x96d8415f,
0x13b68be0, 0x3cc89425, 0x4fc5e34b, 0x03dcbb90,
0xcefc25c5, 0x63c61215, 0x198d1580, 0xf9ae5483,
0x67f790f3, 0x41cebb90, 0x51baf91b, 0x9e07cbfe,
0x40b41821, 0xd370e885, 0x0682ca31, 0x08777aaa,
0x4222c217, 0x388a63ca, 0xbc8286d6, 0xc82b3ce2,
0x7b7b9d7e, 0xe0e27935, 0x37984251, 0x43e2bd49,
0xe476610a, 0x70cddc40, 0x3e8441d3, 0xad6cf494,
0xb85252f8, 0x63c3e6ff, 0xfdc2868e, 0xa8732a8f,
0xf4051fae, 0xa55d527a, 0x7da84834, 0x8de4e190,
0xcb784f3e, 0xdf66e10f, 0x2064a4b0, 0x49168621,
0xbbdf1e31, 0x733c4091, 0x7c76437e, 0x24e247d8,
0xb82c1111, 0x5a54af9f, 0xb3bbe2f2, 0x512126cf,
0x1384b467, 0xc9d99c0c, 0xca993d79, 0x91451877,
0xd3a86723, 0xd463abca, 0x7806d28c, 0x693939ea,
0x636214a7, 0xc1f26dfc, 0x701e1860, 0x90a19cb1,
0x4f0e8b10, 0xdaa4d24a, 0x5b34c161, 0x72758483,
0x119c9e28, 0x0dbef7b8, 0xc9d7d51d, 0x7b9cdcc9,
0xad62ee90, 0x1e215b5a, 0xde7c4672, 0x4f42dfd8,
0x7477e61c, 0xeadf6f66, 0xcc99b635, 0x0e00b5ac,
0xe3cba09e, 0x043e64ed, 0xefe172b6, 0x08354322,
0xa32ec1a3, 0x9d6720ae, 0xa7113c40, 0xc95b5ba0,
0x4cda934a, 0x1b16ae37, 0xf42159b1, 0xa9a722ab,
0x14d43635, 0xc365f0c2, 0xd04726fd, 0x12a7dcce,
0x95506760, 0x0bd513d6, 0x91f1b5fa, 0x66ffb94b,
0xaa6e1860, 0x0b923452, 0xf8bd1e81, 0x7d59146a,
0xe35d30b0, 0x94249b28, 0x9bba5e08, 0x54306ddc,
0xbc2ee888, 0x8b51845a, 0x5121baa4, 0x2f2abb45,
0x68c249b7, 0x7becb440, 0xb40a8edf, 0xec2e0a51,
0x75e1dc69, 0x52c7c00b, 0x25ea8855, 0xaf4b2530,
0xbe2aa632, 0xe1261666, 0xf0c40bfa, 0x0993f9f7,
0x883b3337, 0xfdd81f41, 0x3232bf09, 0xd1dd909a,
0x1af35293, 0xa3425099, 0x17319dab, 0x80430e38,
0xb780fb82, 0x27c978a3, 0xe8cf2034, 0x01098c10,
0x1a640ec4, 0x8c22f026, 0x3f2982d0, 0x0f437671,
0xf62f3d9a, 0x92103763, 0x660fb964, 0xe577fe3e,
0x5e03b786, 0xb08207c8, 0x0ab360ba, 0x3f702fe0,
0x6704420e, 0x022b01c7, 0x1d2f8eb5, 0xde35f510,
0x154c93c3, 0xa3467445, 0xb81a9444, 0x4d583293,
0xa0422496, 0x1e631681, 0x14c9b090, 0xbe11447d,
0xd2a2ab54, 0x86432143, 0xb9a0ea18, 0x4eb174f8,
0xd4942015, 0x627d1963, 0x67b4dc7a, 0x11862b5c,
0xc434330e, 0xe92d5e22, 0x505b6b4c, 0x30e979c2,
0x3c94eea6, 0xa417cb3f, 0x3ee97fb9, 0x4a84d839,
0x0be2de17, 0xa5e31905, 0x2c4f457f, 0x6b375c0d,
0xd74ec096, 0xb7b46e15, 0x59d78910, 0xbb68738d,
0x4020a794, 0xdecc2ac3, 0xe330153c, 0x32712c24,
0x157d3fc1, 0x19ac86b7, 0x88849874, 0xf85de331,
0x0e1f7f28, 0x84db6945, 0x1153c68b, 0xc50d2c2b,
0x1d682a62, 0x11d1252a, 0xd030ddc8, 0xc7fdea93,
0x17717297, 0x4d81d808, 0x9c42e27a, 0xf3d2b06f,
0x6aa349e8, 0x217fd092, 0xb0e0ec4a, 0x471e353e,
0xa8395eef, 0xe0f3d771, 0x70857e5d, 0xa0e3218c,
0x4042aa25, 0xbbeadd18, 0xb9e3774e, 0xcb7d93f0,
0x8f1579de, 0x76e91a51, 0xcd5a5a0a, 0x4d80d76c,
0x6e82f6d8, 0xd25763ce, 0x139fb781, 0xea9d9fc9,
0x9be1540f, 0x2baebb41, 0x2318fd29, 0x668e06a1,
0x0a807c66, 0x1b3a1cb2, 0xa632de83, 0xff94a8ee,
0xb3111659, 0x9ba349d2, 0x169bd298, 0x135cfc76,
0x5f39b26c, 0x1afd5ede, 0xec10fb89, 0x29a175a1,
0xeafcc8b0, 0x3d4223c8, 0xf84af2e5, 0xe3035092,
0x2d19f092, 0x81383669, 0x208328c5, 0x5c8207bb,
0x8c4fab91, 0x515a19f1, 0x77b329aa, 0xbd2ea962,
0xd24a6f75, 0x94ff2520, 0x1639485e, 0x1f7fcaca,
0xbdd78c7c, 0x5553e469, 0x52d9e111, 0x887153a1,
0x5664b146, 0x9bf93501, 0xdf276055, 0x245ba714,
0xea8e14d4, 0x0c800c9a, 0xdf30bfaa, 0x19e46194,
0x77007714, 0x100a072a, 0xf795da2a, 0xad527259,
0x8c699e27, 0x59886295, 0xc1bd0c46, 0x4f9900e8,
0xadb86188, 0x088c4498, 0xe172b148, 0xcadf70b0,
0x32c1e07a, 0x836dbdf4, 0xbc7b0d97, 0x1ecae783,
0xaca556e1, 0xb7019bde, 0x765045de, 0x5fd01bf4,
0x2ce3b755, 0x9a544d4e, 0x28139140, 0xd3159901,
0x1c309601, 0x3d25582f, 0xc0c1226b, 0x6ea71c15,
0xf0860f59, 0x58fb93a9, 0xc5d4b10c, 0x2dd957ca,
0xbddd4a71, 0x14d001dd, 0x33f48e3a, 0x09907354,
0x339a542a, 0x80edd8e5, 0x97548d52, 0x56ed3094,
0x3dc562c4, 0x08e3cea7, 0xf3850f12, 0x4f82aab9,
0x850a2e5c, 0x4b632539, 0x675c4d18, 0x260f46d1,
0xffffffff, 0x00000000, 0x67e5c65b, 0x4523cb05,
0x050cb29e, 0x9bc38a15, 0xd4f1b768, 0xaa8afacb,
0x7d78a4dd, 0x04a5c7da, 0x102af792, 0xdfc092e4,
0xda784245, 0x6f0aae49, 0xaa871399, 0x722ae01c,
0x67a60833, 0xbc49c6fc, 0x78e4726b, 0x3688a7b8,
0xd9b0b2a4, 0x49233a7d, 0x1c6cada3, 0xe67863e5,
0x444d9646, 0xd09f186a, 0x1068214f, 0x071e054e,
0xed7fd3f7, 0x14a348c2, 0xfdc7ff1d, 0x6c74a660,
0x4ff1c008, 0xf7858ef2, 0x5203b8e8, 0xd97bf7a9,
0x3b5bffde, 0x3360d92d, 0x4f487f9a, 0xd1a61579,
0xe304112b, 0xaddc4515, 0x25257849,
};
static uint32_t buf[ ARRAY_SIZE( pattern ) ] = { 0U, };

typedef enum {
    READ = 0U,
    WRITE = 1U,
    VALIDATION = 2U,
    OK
} status_type;

static char const * const msg[] = {
    [ READ ] = "read",
    [ WRITE ] = "write",
    [ VALIDATION ] = "validation"
};

static status_type test( uintptr_t const address )
{
    status_type result = OK;

    flash_access_status_t const write_status =
        flash_write(
            ( void * ) address,
            ( void const * ) pattern,
            sizeof( pattern )
        );
    if ( ! STATUS_OK( write_status )) {
        result = WRITE;
        goto done;
    }

    memset( buf, 0, sizeof( buf ));
    flash_access_status_t const read_status =
        flash_read(
            ( void * ) address,
            ( void * ) buf,
            sizeof( buf )
        );
    if ( ! STATUS_OK( read_status )) {
        result = READ;
        goto done;
    }

    for ( size_t i = 0U; i < ARRAY_SIZE( pattern ); ++i ) {
        if ( pattern[ i ] != buf[ i ] ) {
            printf( "validation failed @: %p\n", ( void * ) ( address + ( i * sizeof( uint32_t ))));
            result = VALIDATION;
            goto done;
        }
    }

    memset( buf, 0, sizeof( buf ));
    flash_sync();
    flash_access_status_t const read_status_after_sync =
        flash_read(
            ( void * ) address,
            ( void * ) buf,
            sizeof( buf )
        );
    if ( ! STATUS_OK( read_status_after_sync )) {
        result = READ;
        goto done;
    }

    for ( size_t i = 0U; i < ARRAY_SIZE( pattern ); ++i ) {
        if ( pattern[ i ] != buf[ i ] ) {
            printf( "validation failed @: %p\n", ( void * ) ( address + ( i * sizeof( uint32_t ))));
            result = VALIDATION;
            goto done;
        }
    }

    /* direct AHB read */
    for ( size_t i = 0U; i < ARRAY_SIZE( pattern ); ++i ) {
        uint32_t const * const rom = ( uint32_t const * )( address + ( i * sizeof( uint32_t )));
        uint32_t const word = *rom;
        if ( pattern[ i ] != word ) {
            printf( "validation failed @: %p\n", ( void * ) ( address + ( i * sizeof( uint32_t ))));
            result = VALIDATION;
            goto done;
        }
    }

done:
    return result;
}

typedef struct {
    unsigned reads;
    unsigned writes;
} rwstats;

static uintptr_t
get_random_page_address( uintptr_t const min, uintptr_t const max )
{
    long const obtained = random(); /* <0, RAND_MAX> */
    uintptr_t const result =
        min
        + (
            obtained
            / (
                RAND_MAX / ( max - min + 1U ) + 1U
            )
        );
    return result;
}

static size_t
get_performance_page_index( uintptr_t const base, uintptr_t const address )
{
    uintptr_t const page_base = PAGE_ALIGN( base );
    uintptr_t const page_address = PAGE_ALIGN( address );
    uintptr_t const difference = page_address - page_base;
    size_t const result = difference / FLASH_PAGE_SIZE;
    return result;
}

int main(void)
{
    printf("\nStarting FLASH cache tests, number of caches: %u\n", (unsigned) TSMC40ULPFMC_RAM_BUFFERS);

    uint8_t * const first_free_page =
        ( uint8_t * ) PAGE_ALIGN( TEST_FIRST_FREE_PAGE );
    static uint32_t pattern[
        TEST_PAGES_USED * FLASH_PAGE_SIZE / sizeof( uint32_t )
    ];

    flash_access_status_t status = ARGUMENT_ERROR;

    /* Enable APB1 Bridge */
    AMBA_APB0_CFG_PTR->APB1_CFG = AMBA_APB1_EN;

    {
        for (
            size_t i = 0U;
            i < ARRAY_SIZE( pattern );
            ++i
        ) {
            pattern[ i ] = PATTERN;
        }

        status =
            flash_write(
                first_free_page,
                ( uint8_t const * ) pattern,
                sizeof( pattern )
            );
        assertTrue( STATUS_OK( status ));

        /* reads of various lengths and starting addresses */
        /* check entire region word by word */
        {
            uint32_t buffer = 0U;
            for (
                size_t i = 0U;
                i < sizeof( pattern );
                i += sizeof( buffer )
            ) {
                status =
                    flash_read(
                        &( first_free_page[ i ] ),
                        ( void * ) &buffer,
                        sizeof( buffer )
                    );
                assertTrue( STATUS_OK( status ));
                assertTrue( PATTERN == buffer );
            }
        }

        /* Note for unaligned and byte reading: system is little endian. */
        /* less than word, within page */
        {
            uint8_t buffer = 0U;
            status =
                flash_read(
                    &( first_free_page[ 2U ] ),
                    ( void * ) &buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));
            assertTrue( 0xBC == buffer );
        }
        /* word, within page, unaligned */
        {
            uint32_t buffer = 0U;
            status =
                flash_read(
                    &( first_free_page[ 2U ] ),
                    ( void * ) &buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));
            /* BC 0A F0 DE -> 0xDEF00ABC */
            assertTrue( 0xDEF00ABC == buffer );
        }
        /* more than word, less than two words, within page */
        {
            uint8_t buffer[ 6U ] = { 0U, };
            status =
                flash_read(
                    &( first_free_page[ 9U ] ),
                    ( void * ) buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));
            assertTrue( 0xDE == buffer[ 0U ] );
            assertTrue( 0xBC == buffer[ 1U ] );
            assertTrue( 0x0A == buffer[ 2U ] );
            assertTrue( 0xF0 == buffer[ 3U ] );
            assertTrue( 0xDE == buffer[ 4U ] );
            assertTrue( 0xBC == buffer[ 5U ] );
        }
        /* multiple words, within page, unaligned */
        {
            uint32_t buffer[ 3U ] = { 0U, };
            status =
                flash_read(
                    &( first_free_page[ 9U ] ),
                    ( void * ) buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));
            assertTrue( 0xF00ABCDE == buffer[ 0U ] );
            assertTrue( 0xF00ABCDE == buffer[ 1U ] );
            assertTrue( 0xF00ABCDE == buffer[ 2U ] );
        }
        /* less than word, across pages */
        {
            uint8_t buffer[ 3U ] = { 0U, };
            status =
                flash_read(
                    &( first_free_page[ FLASH_PAGE_SIZE - 1U ] ),
                    ( void * ) buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));
            assertTrue( 0x0A == buffer[ 0U ] );
            assertTrue( 0xF0 == buffer[ 1U ] );
            assertTrue( 0xDE == buffer[ 2U ] );
        }
        /* word, across pages */
        {
            uint32_t buffer = 0U;
            status =
                flash_read(
                    &( first_free_page[ FLASH_PAGE_SIZE - 1U ] ),
                    ( void * ) &buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));
            assertTrue( 0xBCDEF00A == buffer );
        }
        /* more than word, less than two words, across pages */
        {
            uint8_t buffer[ 6U ] = { 0U, };
            status =
                flash_read(
                    &( first_free_page[ FLASH_PAGE_SIZE - 1U ] ),
                    ( void * ) buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));
            assertTrue( 0x0A == buffer[ 0U ] );
            assertTrue( 0xF0 == buffer[ 1U ] );
            assertTrue( 0xDE == buffer[ 2U ] );
            assertTrue( 0xBC == buffer[ 3U ] );
            assertTrue( 0x0A == buffer[ 4U ] );
            assertTrue( 0xF0 == buffer[ 5U ] );
        }
        /* multiple words, across pages, aligned */
        {
            uint32_t buffer[ 3U ] = { 0U, };
            status =
                flash_read(
                    &( first_free_page[ FLASH_PAGE_SIZE - sizeof( uint32_t ) ] ),
                    ( void * ) buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));
            assertTrue( PATTERN == buffer[ 0U ] );
            assertTrue( PATTERN == buffer[ 1U ] );
            assertTrue( PATTERN == buffer[ 2U ] );
        }
        /* multiple words, across pages, unaligned */
        {
            uint32_t buffer[ 3U ] = { 0U, };
            status =
                flash_read(
                    &(
                        first_free_page[
                            FLASH_PAGE_SIZE
                            - sizeof( uint32_t ) - 1U
                        ]
                    ),
                    ( void * ) buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));
            /* 0A F0 DE BC 0A F0 ... -> 0xBCDEF00A */
            assertTrue( 0xBCDEF00A == buffer[ 0U ] );
            assertTrue( 0xBCDEF00A == buffer[ 1U ] );
            assertTrue( 0xBCDEF00A == buffer[ 2U ] );
        }

        /* writes of various lengths and starting addresses */
        /* less than word, within page */
        {
            uint8_t const source = 0x12;
            status =
                flash_write(
                    &( first_free_page[ 2U ] ),
                    ( void * ) &source,
                    sizeof( source )
                );
            assertTrue( STATUS_OK( status ));

            uint32_t buffer = 0U;
            status =
                flash_read(
                    &( first_free_page[ 0U ] ),
                    ( void * ) &buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));

            assertTrue( 0x0A12DEF0 == buffer );
        }
        /* word, within page, unaligned */
        {
            uint32_t const source = PATTERN2; /* 78 56 34 12 */
            status =
                flash_write(
                    &( first_free_page[ 5U ] ),
                    ( void * ) &source,
                    sizeof( source )
                );
            assertTrue( STATUS_OK( status ));

            uint32_t buffer[ 2U ] = { 0U, };
            status =
                flash_read(
                    &( first_free_page[ 4U ] ),
                    ( void * ) buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));
            /* F0 78 56 34 12 DE BC 0A */
            assertTrue( 0x345678F0 == buffer[ 0U ] );
            assertTrue( 0x0ABCDE12 == buffer[ 1U ] );
        }
        /* word, within page, aligned */
        {
            uint32_t const source = PATTERN2;
            status =
                flash_write(
                    &( first_free_page[ 8U ] ),
                    ( void * ) &source,
                    sizeof( source )
                );
            assertTrue( STATUS_OK( status ));

            uint32_t buffer = 0U;
            status =
                flash_read(
                    &( first_free_page[ 8U ] ),
                    ( void * ) &buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));
            assertTrue( PATTERN2 == buffer );
        }
        /* more than word, less than two words, within page */
        {
            uint8_t source[ 7U ] = { 0x12, 0x34, 0x56, 0x78, 0xC0, 0xFF, 0xEE };
            status =
                flash_write(
                    &( first_free_page[ 3U ] ),
                    ( void * ) source,
                    sizeof( source )
                );
            assertTrue( STATUS_OK( status ));

            uint32_t buffer[ 3U ] = { 0U, };
            status =
                flash_read(
                    &( first_free_page[ 0U ] ),
                    ( void * ) buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));

            /* F0 DE 12 12 34 56 78 C0 FF EE 34 12 */
            assertTrue( 0x1212DEF0 == buffer[ 0U ] );
            assertTrue( 0xC0785634 == buffer[ 1U ] );
            assertTrue( 0x1234EEFF == buffer[ 2U ] );
        }
        /* multiple words, within page, unaligned */
        {
            uint32_t source[ 2U ] = { PATTERN2, PATTERN2 }; /* 78 56 34 12... */
            status =
                flash_write(
                    &( first_free_page[ 1U ] ),
                    ( void * ) source,
                    sizeof( source )
                );
            assertTrue( STATUS_OK( status ));

            uint32_t buffer[ 3U ] = { 0U, };
            status =
                flash_read(
                    &( first_free_page[ 0U ] ),
                    ( void * ) buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));
            /* F0 78 56 34 12 78 56 34 12 EE 34 12 */
            assertTrue( 0x345678F0 == buffer[ 0U ] );
            assertTrue( 0x34567812 == buffer[ 1U ] );
            assertTrue( 0x1234EE12 == buffer[ 2U ] );
        }
        /* multiple words, within page, aligned */
        {
            uint32_t source[ 3U ] = { PATTERN2, PATTERN2, PATTERN2 };
            status =
                flash_write(
                    &( first_free_page[ 0U ] ),
                    ( void * ) source,
                    sizeof( source )
                );
            assertTrue( STATUS_OK( status ));

            uint32_t buffer[ 3U ] = { 0U, };
            status =
                flash_read(
                    &( first_free_page[ 0U ] ),
                    ( void * ) buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));
            assertTrue( PATTERN2 == buffer[ 0U ] );
            assertTrue( PATTERN2 == buffer[ 1U ] );
            assertTrue( PATTERN2 == buffer[ 2U ] );
        }
        /* less than word, across pages */
        {
            uint8_t const source[ 2U ] = { 0x12, 0x34 };
            status =
                flash_write(
                    &(
                        first_free_page[
                            FLASH_PAGE_SIZE - 1U
                        ]
                    ),
                    ( void * ) source,
                    sizeof( source )
                );
            assertTrue( STATUS_OK( status ));

            uint32_t buffer[ 2U ] = { 0U, };
            status =
                flash_read(
                    &(
                        first_free_page[
                            FLASH_PAGE_SIZE
                            - sizeof( uint32_t )
                        ]
                    ),
                    ( void * ) buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));
            assertTrue( 0x12BCDEF0 == buffer[ 0U ] );
            assertTrue( 0x0ABCDE34 == buffer[ 1U ] );
        }
        /* word, across pages */
        {
            uint32_t const source = PATTERN2; /* 78 56 34 12 */
            status =
                flash_write(
                    &(
                        first_free_page[
                            FLASH_PAGE_SIZE - 2U
                        ]
                    ),
                    ( void * ) &source,
                    sizeof( source )
                );
            assertTrue( STATUS_OK( status ));

            uint32_t buffer[ 2U ] = { 0U, };
            status =
                flash_read(
                    &(
                        first_free_page[
                            FLASH_PAGE_SIZE
                            - sizeof( uint32_t )
                        ]
                    ),
                    ( void * ) buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));
            /* F0 DE 78 56 34 12 BC 0A */
            assertTrue( 0x5678DEF0 == buffer[ 0U ] );
            assertTrue( 0x0ABC1234 == buffer[ 1U ] );
        }
        /* more than word, less than two words, across pages */
        {
            uint8_t source[ 6U ] = { 0xC0, 0xFF, 0xEE, 0xC0, 0xFF, 0xEE };
            status =
                flash_write(
                    &(
                        first_free_page[
                            FLASH_PAGE_SIZE - 3U
                        ]
                    ),
                    ( void * ) source,
                    sizeof( source )
                );
            assertTrue( STATUS_OK( status ));

            uint32_t buffer[ 2U ] = { 0U, };
            status =
                flash_read(
                    &(
                        first_free_page[
                            FLASH_PAGE_SIZE
                            - sizeof( uint32_t )
                        ]
                    ),
                    ( void * ) buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));
            /* F0 C0 FF EE C0 FF EE 0A */
            assertTrue( 0xEEFFC0F0 == buffer[ 0U ] );
            assertTrue( 0x0AEEFFC0 == buffer[ 1U ] );
        }
        /* multiple words, across pages, aligned */
        {
            uint32_t source[ 2U ] = { PATTERN2, PATTERN2 };
            status =
                flash_write(
                    &(
                        first_free_page[
                            FLASH_PAGE_SIZE
                            - sizeof( uint32_t )
                        ]
                    ),
                    ( void * ) source,
                    sizeof( source )
                );
            assertTrue( STATUS_OK( status ));

            uint32_t buffer[ 2U ] = { 0U, };
            status =
                flash_read(
                    &(
                        first_free_page[
                            FLASH_PAGE_SIZE
                            - sizeof( uint32_t )
                        ]
                    ),
                    ( void * ) buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));
            assertTrue( PATTERN2 == buffer[ 0U ] );
            assertTrue( PATTERN2 == buffer[ 1U ] );
        }
        /* multiple words, across pages, unaligned */
        {
            uint32_t source[ 2U ] = { PATTERN2, PATTERN2 }; /* 78 56 34 12... */
            status =
                flash_write(
                    &(
                        first_free_page[
                            FLASH_PAGE_SIZE - 1U
                        ]
                    ),
                    ( void * ) source,
                    sizeof( source )
                );
            assertTrue( STATUS_OK( status ));

            uint32_t buffer[ 3U ] = { 0U, };
            status =
                flash_read(
                    &(
                        first_free_page[
                            FLASH_PAGE_SIZE
                            - sizeof( uint32_t )
                        ]
                    ),
                    ( void * ) buffer,
                    sizeof( buffer )
                );
            assertTrue( STATUS_OK( status ));
            /* 78 56 34 78 56 34 12 78 56 34 12 0A */
            assertTrue( 0x78345678 == buffer[ 0U ] );
            assertTrue( 0x78123456 == buffer[ 1U ] );
            assertTrue( 0x0A123456 == buffer[ 2U ] );
        }
    }

    flash_sync();

    /* performance testing */
#define PERFORMANCE_RW_CASES 1000U
    /*
     * Do PERFORMANCE_RW_CASES cycles of reads and writes.
     * In every cycle choose random read address and then random write address.
     * Do read from chosen address, single word. Do write to chosen address,
     * single word.
     */

    printf("\nStarting flash cache performance tests\n");

/* case 1: pages fit within cache */
#define PERFORMACE_CASE_1_PAGES ( TSMC40ULPFMC_RAM_BUFFERS - 1U )

static rwstats case_1_stats[ PERFORMACE_CASE_1_PAGES ];

#if FLASH_SIZE < ( PERFORMACE_CASE_1_PAGES * FLASH_PAGE_SIZE )
# error "Flash size too small for test!"
#endif

#define PERFORMACE_CASE_1_FIRST_FREE_PAGE \
    ( FLASH_SIZE - ( PERFORMACE_CASE_1_PAGES * FLASH_PAGE_SIZE ))

    printf("\nCase 1 (less pages than caches), %u pages, %u read-writes\n", (unsigned) PERFORMACE_CASE_1_PAGES, (unsigned) PERFORMANCE_RW_CASES );
    printf("\nStart\n");
    for ( size_t i = 0U; i < PERFORMANCE_RW_CASES; ++i ) {
        uintptr_t read_address =
            get_random_page_address(
                PERFORMACE_CASE_1_FIRST_FREE_PAGE,
                FLASH_SIZE - 1U
            );
        read_address = WORD_ALIGN( read_address );
        uint32_t read_buffer = 0U;
        flash_access_status_t const read_status =
            flash_read(
                ( void * ) read_address,
                ( void * ) &read_buffer,
                sizeof( read_buffer )
            );
        assertTrue( STATUS_OK( read_status ));
        ++( case_1_stats[ get_performance_page_index( PERFORMACE_CASE_1_FIRST_FREE_PAGE, read_address ) ].reads );

        uintptr_t write_address =
            get_random_page_address(
                PERFORMACE_CASE_1_FIRST_FREE_PAGE,
                FLASH_SIZE - 1U
            );
        write_address = WORD_ALIGN( write_address );
        uint32_t const write_buffer = 0x600DCAFE;
        flash_access_status_t const write_status =
            flash_write(
                ( void * ) write_address,
                ( void * ) &write_buffer,
                sizeof( write_buffer )
            );
        assertTrue( STATUS_OK( write_status ));
        ++( case_1_stats[ get_performance_page_index( PERFORMACE_CASE_1_FIRST_FREE_PAGE, write_address ) ].writes );
    }
    printf("\nFinish\n");
    for ( size_t i = 0U; i < PERFORMACE_CASE_1_PAGES; ++i ) {
        printf( "page %u, reads: %u, writes: %u\n", (unsigned) i, case_1_stats[ i ].reads, case_1_stats[ i ].writes );
    }

    flash_sync();

/* case 2: pages don't fit within cache */
#define PERFORMACE_CASE_2_PAGES ( TSMC40ULPFMC_RAM_BUFFERS + 3U )

static rwstats case_2_stats[ PERFORMACE_CASE_2_PAGES ];

#if FLASH_SIZE < ( PERFORMACE_CASE_2_PAGES * FLASH_PAGE_SIZE )
# error "Flash size too small for test!"
#endif

#define PERFORMACE_CASE_2_FIRST_FREE_PAGE \
    ( FLASH_SIZE - ( PERFORMACE_CASE_2_PAGES * FLASH_PAGE_SIZE ))

    printf("\nCase 2 (more pages than caches), %u pages, %u read-writes\n", (unsigned) PERFORMACE_CASE_2_PAGES, (unsigned) PERFORMANCE_RW_CASES );
    printf("\nStart\n");
    for ( size_t i = 0U; i < PERFORMANCE_RW_CASES; ++i ) {
        uintptr_t read_address =
            get_random_page_address(
                PERFORMACE_CASE_2_FIRST_FREE_PAGE,
                FLASH_SIZE - 1U
            );
        read_address = WORD_ALIGN( read_address );
        uint32_t read_buffer = 0U;
        flash_access_status_t const read_status =
            flash_read(
                ( void * ) read_address,
                ( void * ) &read_buffer,
                sizeof( read_buffer )
            );
        assertTrue( STATUS_OK( read_status ));
        ++( case_2_stats[ get_performance_page_index( PERFORMACE_CASE_2_FIRST_FREE_PAGE, read_address ) ].reads );

        uintptr_t write_address =
            get_random_page_address(
                PERFORMACE_CASE_2_FIRST_FREE_PAGE,
                FLASH_SIZE - 1U
            );
        write_address = WORD_ALIGN( write_address );
        uint32_t const write_buffer = 0x600DCAFE;
        flash_access_status_t const write_status =
            flash_write(
                ( void * ) write_address,
                ( void * ) &write_buffer,
                sizeof( write_buffer )
            );
        assertTrue( STATUS_OK( write_status ));
        ++( case_2_stats[ get_performance_page_index( PERFORMACE_CASE_2_FIRST_FREE_PAGE, write_address ) ].writes );
    }
    for ( size_t i = 0U; i < PERFORMACE_CASE_2_PAGES; ++i ) {
        printf( "page %u, reads: %u, writes: %u\n", (unsigned) i, case_2_stats[ i ].reads, case_2_stats[ i ].writes );
    }
    printf("\nFinish\n");

    flash_sync();

    printf("\nStart page validation tests\n");
#define VALIDATION_WR_CASES 100U
    flash_cache_threshold( 0U, 0U );
    for ( uintptr_t i = 0x100000; i < 0x200000; i += 0x2000 ) {
        for ( size_t try = 0U; try < VALIDATION_WR_CASES; ++try ) {
            status_type const s = test( PAGE_ALIGN( i ));
            assertTrue( OK == s );
            if( OK != s ) {
                printf( "page %p failed at %s\n", ( void * ) i, msg[ s ] );
            }
        }
    }
    printf("\nFinish\n");
    printTestSummary();

    return 0;
}
