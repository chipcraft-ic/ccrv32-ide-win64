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
* $Date: 2024-06-04 16:09:31 +0200 (wto, 04 cze 2024) $
* $Revision: 1059 $
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
    printf("\nFinish\n");
    for ( size_t i = 0U; i < PERFORMACE_CASE_2_PAGES; ++i ) {
        printf( "page %u, reads: %u, writes: %u\n", (unsigned) i, case_2_stats[ i ].reads, case_2_stats[ i ].writes );
    }

    flash_sync();

    printTestSummary();

    return 0;
}
