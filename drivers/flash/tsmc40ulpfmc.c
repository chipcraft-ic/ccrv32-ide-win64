/*H*****************************************************************************
 *
 * Copyright (c) 2022 ChipCraft Sp. z o.o. All rights reserved
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
 * File Name : tsmc40ulpfmc.c
 * Author    : Maciej Plasota
 * ******************************************************************************
 * $Date: 2024-06-04 16:08:03 +0200 (wto, 04 cze 2024) $
 * $Revision: 1058 $
 *H*****************************************************************************/

#include <specialreg.h>

/* Macros below are defined as enum values in ccrv32.h */
#undef ROM_BASE
#undef RAM_BASE
#undef SPRAM_BASE
#undef AMBA_BASE
#undef PLIC_BASE

#include <board.h>
#include <ccrv32.h>
#include <ccrv32-csr.h>
#include <ccrv32-dcache.h>
#include <limits.h> /* CHAR_BIT */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/lock.h>

#include "flash.h"

#define HCLK_10_MHz 10000000U
#define HCLK_254_MHz 254000000U

#define TSMC40ULPFMC_PAGE_SIZE FLASH_PAGE_SIZE
#define TSMC40ULPFMC_SECTOR_SIZE \
    ( \
        sizeof( AMBA_FLASH_PTR->SECTOR_BUFFER ) \
            / \
        sizeof( AMBA_FLASH_PTR->SECTOR_BUFFER[ 0U ] ) \
    )

/*
 * We need to get the word that contains given (possibly unaligned) address.
 * This means we should align down, by zeroing two least significant bits.
 */
#define WORD_ALIGN( address ) \
    ( \
        ( \
            ( address ) \
            | ( sizeof( uint32_t ) - 1U ) \
        ) \
            ^ \
        ( sizeof( uint32_t ) - 1U ) \
    )

#define PAGE_ALIGN( address ) \
    ( \
        ( \
            ( address ) \
            | ( TSMC40ULPFMC_PAGE_SIZE - 1U ) \
        ) \
            ^ \
        ( TSMC40ULPFMC_PAGE_SIZE - 1U ) \
    )

#define BYTE2WORD( byte, index ) \
    ( uint32_t ) ( \
        ( ( uint32_t ) ( byte ) ) \
        << ( ( sizeof( uint32_t ) - 1U - ( index ) ) * CHAR_BIT ) \
    )

#define WORD2BYTE( word, index ) \
	( uint8_t ) ( \
		( ( uint32_t ) ( word ) ) \
		>> ( ( sizeof( uint32_t ) - 1U - ( index ) ) * CHAR_BIT ) \
	)

#define CLEARBYTE( word, index ) \
    { \
        ( word ) |= BYTE2WORD( 0xFF, ( index ) ); \
        ( word ) ^= BYTE2WORD( 0xFF, ( index ) ); \
    }

#define IS_WORD_ALIGNED( ADDRESS ) \
    ( \
        0U == ( ADDRESS & 0x00000003 ) \
    )

#define IS_QWORD_ALIGNED( ADDRESS ) \
    ( \
        0U == ( ADDRESS & 0x0000000F ) \
    )

#define IS_SECTOR_ALIGNED( ADDRESS ) \
    ( \
        0U == ( ADDRESS & ( TSMC40ULPFMC_SECTOR_SIZE - 1U )) \
    )

#define IS_PAGE_ALIGNED( ADDRESS ) \
    ( \
        0U == ( ADDRESS & ( TSMC40ULPFMC_PAGE_SIZE - 1U )) \
    )

#define IS_WITHIN( LOWER, VALUE, UPPER ) \
    ( \
        true \
        && ( LOWER <= VALUE ) \
        && ( VALUE <= UPPER ) \
    )

#ifndef TSMC40ULPFMC_RAM_BUFFERS_DEBUG
# define TSMC40ULPFMC_RAM_BUFFERS_DEBUG 0
#endif /* TSMC40ULPFMC_RAM_BUFFERS_DEBUG */

#ifndef TSMC40ULPFMC_RAM_BUFFERS
# define TSMC40ULPFMC_RAM_BUFFERS 3U
#endif /* TSMC40ULPFMC_RAM_BUFFERS */

#ifndef TSMC40ULPFMC_WRITE_THRESHOLD_MIN
# define TSMC40ULPFMC_WRITE_THRESHOLD_MIN 8U
#endif /* TSMC40ULPFMC_WRITE_THRESHOLD_MIN */

#ifndef TSMC40ULPFMC_WRITE_THRESHOLD_MAX
# define TSMC40ULPFMC_WRITE_THRESHOLD_MAX 16U
#endif /* TSMC40ULPFMC_WRITE_THRESHOLD_MAX */

static inline uint32_t
endianess_swap( uint32_t const value )
{
    uint32_t const result =
        0U
        | (( value << 24U ) & 0xFF000000 )
        | (( value << 8U ) & 0x00FF0000 )
        | (( value >> 8U ) & 0x0000FF00 )
        | (( value >> 24U ) & 0x000000FF )
        ;
    return result;
}

/*
 * Hardware only provides a sector buffer.
 * Page is composed of multiple sectors.
 * We can only erase a page, i.e. multiple sectors.
 * If we want to do a partial write, we need external buffer
 * or we'll lose information.
 */

typedef struct __attribute__(( aligned ( 4 ))) {
    uint8_t data[ TSMC40ULPFMC_PAGE_SIZE ];
} tsmc40ulpfmc_cache_page_buffer_data_type;

typedef struct {
    bool used;
    uintptr_t address;
    uint8_t writes;
    tsmc40ulpfmc_cache_page_buffer_data_type buffer;
} tsmc40ulpfmc_cache_page_buffer_type;

typedef struct {
    _LOCK_RECURSIVE_T lock;
    tsmc40ulpfmc_cache_page_buffer_type * last;
    tsmc40ulpfmc_cache_page_buffer_type * most;
    tsmc40ulpfmc_cache_page_buffer_type * unused;
    tsmc40ulpfmc_cache_page_buffer_type * cache;
    struct {
        uint8_t min;
        uint8_t max;
    } threshold;
} tsmc40ulpfmc_cache_type;

static tsmc40ulpfmc_cache_page_buffer_type tsmc40ulpfmc_cache_page_buffer[ TSMC40ULPFMC_RAM_BUFFERS ];
static tsmc40ulpfmc_cache_type tsmc40ulpfmc_cache = {
    .last = &( tsmc40ulpfmc_cache_page_buffer[ 0U ] ),
    .most = &( tsmc40ulpfmc_cache_page_buffer[ 0U ] ),
    .unused = &( tsmc40ulpfmc_cache_page_buffer[ 0U ] ),
    .cache = tsmc40ulpfmc_cache_page_buffer,
    .threshold = {
        .min = TSMC40ULPFMC_WRITE_THRESHOLD_MIN,
        .max = TSMC40ULPFMC_WRITE_THRESHOLD_MAX
    }
};

#if TSMC40ULPFMC_RAM_BUFFERS_DEBUG
static void
tsmc40ulpfmc_cache_print_state( void )
{
        printf(
            "cache: last: %u, most: %u, unused: %u\n",
            ( (uintptr_t) tsmc40ulpfmc_cache.last - (uintptr_t) tsmc40ulpfmc_cache.cache ) / sizeof( tsmc40ulpfmc_cache_page_buffer_type ),
            ( (uintptr_t) tsmc40ulpfmc_cache.most - (uintptr_t) tsmc40ulpfmc_cache.cache ) / sizeof( tsmc40ulpfmc_cache_page_buffer_type ),
            ( (uintptr_t) tsmc40ulpfmc_cache.unused - (uintptr_t) tsmc40ulpfmc_cache.cache ) / sizeof( tsmc40ulpfmc_cache_page_buffer_type )
        );
    for ( size_t i = 0U; i < TSMC40ULPFMC_RAM_BUFFERS; ++i ) {
        tsmc40ulpfmc_cache_page_buffer_type * const buffer =
            &( tsmc40ulpfmc_cache.cache[ i ] );
        printf( "[%u]: %c, 0x%x, %u\n", (unsigned) i, buffer->used ? 'Y' : 'N', (unsigned) buffer->address, (unsigned) buffer->writes );
    }
}
#else
# define tsmc40ulpfmc_cache_print_state() ( void ) 0
#endif /* TSMC40ULPFMC_RAM_BUFFERS_DEBUG */

/*
 * Copy from RAM buffer to flash using word-based AHB.
 */
static size_t
tsmc40ulpfmc_memcpy_to_flash(
    uint8_t * const dst, /* has to be word-aligned */
    uint8_t const * const src, /* has to be word-aligned */
    size_t bytes /* has to be multiple fo word */
) {
    size_t index = 0U;

    uintptr_t const start = ( uintptr_t ) dst;
    uintptr_t const end = start + bytes;

    for (
        uintptr_t i = start;
        i < end;
        ( i += sizeof( uint32_t )), ( index += sizeof( uint32_t ))
    ) {
        /* we have to read byte by byte, because src may not be aligned */
        register uint32_t wordbuffer =
            0U
            | BYTE2WORD( src[ index + 0U ], 0U )
            | BYTE2WORD( src[ index + 1U ], 1U )
            | BYTE2WORD( src[ index + 2U ], 2U )
            | BYTE2WORD( src[ index + 3U ], 3U )
            ;
        wordbuffer = endianess_swap( wordbuffer );
        /*
         * we treat wordbuffer as byte stream, so big endian
         * however we write little endian word to flash
         * so the easiest option is to switch endianess
         */
        *(( uint32_t volatile * ) ( i )) = wordbuffer;
    }

    return index;
}

static size_t
tsmc40ulpfmc_memcpy_from_flash(
    uint8_t * const dst, /* has to be word-aligned */
    uint8_t const * const src, /* has to be word-aligned */
    size_t bytes /* has to be multiple fo word */
) {
    size_t index = 0U;

    uintptr_t const start = ( uintptr_t ) src;
    uintptr_t const end = start + bytes;

    for (
        uintptr_t i = start;
        i < end;
        ( i += sizeof( uint32_t )), ( index += sizeof( uint32_t ))
    ) {
        register uint32_t wordbuffer = *(( uint32_t const * ) i );
        wordbuffer = endianess_swap( wordbuffer );

        dst[ index + 0U ] = WORD2BYTE( wordbuffer, 0U );
        dst[ index + 1U ] = WORD2BYTE( wordbuffer, 1U );
        dst[ index + 2U ] = WORD2BYTE( wordbuffer, 2U );
        dst[ index + 3U ] = WORD2BYTE( wordbuffer, 3U );
    }

    return index;
}

static tsmc40ulpfmc_cache_page_buffer_type *
tsmc40ulpfmc_cache_page_buffer_from_address(
    void const * const address
)
{
    register tsmc40ulpfmc_cache_page_buffer_type * result = NULL;

    uintptr_t const address_ = ( uintptr_t ) address;
    uintptr_t const page = PAGE_ALIGN( address_ );

    __libc_lock_acquire_recursive( &( tsmc40ulpfmc_cache.lock ));
    for ( size_t i = 0U; i < TSMC40ULPFMC_RAM_BUFFERS; ++i ) {
        result = &( tsmc40ulpfmc_cache.cache[ i ] );
        if (( result->used ) && ( page == result->address )) {
            break;
        }
        result = NULL;
    }
    __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));

    return result;
}

static void
tsmc40ulpfmc_cache_page_buffer_free(
    tsmc40ulpfmc_cache_page_buffer_type * const buffer
)
{
    __libc_lock_acquire_recursive( &( tsmc40ulpfmc_cache.lock ));
    buffer->used = false;
    buffer->address = 0U;
    buffer->writes = 0U;
    ( void ) memset( buffer->buffer.data, 0, TSMC40ULPFMC_PAGE_SIZE );

    /* update most */
    tsmc40ulpfmc_cache.most = tsmc40ulpfmc_cache.last;
    for ( size_t i = 0U; i < TSMC40ULPFMC_RAM_BUFFERS; ++i ) {
        tsmc40ulpfmc_cache_page_buffer_type * const buffer_ =
            &( tsmc40ulpfmc_cache.cache[ i ] );
        if ( ! buffer->used ) {
            continue;
        }
        if ( tsmc40ulpfmc_cache.most->writes < buffer_->writes ) {
            tsmc40ulpfmc_cache.most = buffer_;
        }
    }

    __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));
}

static flash_access_status_t
tsmc40ulpfmc_commit_page_blocking(
    tsmc40ulpfmc_cache_page_buffer_type * const buffer
);

/*
 * Get current unused page buffer cache.
 * Commit one with most writes and set it as unused.
 */
static tsmc40ulpfmc_cache_page_buffer_type *
tsmc40ulpfmc_cache_page_buffer_allocate(
    void const * const address
)
{
    tsmc40ulpfmc_cache_page_buffer_type * result = NULL;

    uintptr_t const address_ = ( uintptr_t ) address;
    uintptr_t const page = PAGE_ALIGN( address_ );

    __libc_lock_acquire_recursive( &( tsmc40ulpfmc_cache.lock ));

    /* find next unused */
    for ( size_t i = 0U; i < TSMC40ULPFMC_RAM_BUFFERS; ++i ) {
        if ( tsmc40ulpfmc_cache.cache[ i ].used ) {
            continue;
        }
        /* skip current unused, obviously */
        if ( tsmc40ulpfmc_cache.unused == &( tsmc40ulpfmc_cache.cache[ i ] )) {
            continue;
        }
        result = tsmc40ulpfmc_cache.unused;
        tsmc40ulpfmc_cache.unused = &( tsmc40ulpfmc_cache.cache[ i ] );
        goto done;
    }

    /*
     * we need to free a page, use the first used one,
     * that is not the last used page (as it's likely to be used again)
     * use the most used page as safe default in case of corner cases
     */
    tsmc40ulpfmc_cache_page_buffer_type * candidate = tsmc40ulpfmc_cache.most;
    for ( size_t i = 0U; i < TSMC40ULPFMC_RAM_BUFFERS; ++i ) {
        /*
         * skip the last used page
         * this is done to avoid flip-flops between two pages
         * being written one adfter another, causing spurious flash accesses
         */
        if ( tsmc40ulpfmc_cache.last == &( tsmc40ulpfmc_cache.cache[ i ] )) {
            continue;
        }
        if ( tsmc40ulpfmc_cache.cache[ i ].used ) {
            candidate = &( tsmc40ulpfmc_cache.cache[ i ] );
            break;
        }
    }

    flash_access_status_t const status =
        /* commit will also free candidate and fix most if it succeeds */
        tsmc40ulpfmc_commit_page_blocking( candidate );
    if ( BUSY < status ) {
        /* backtrack and return NULL */
        tsmc40ulpfmc_cache.unused->used = false;
        goto failure_io;
    }
    tsmc40ulpfmc_cache_page_buffer_free( candidate );

    result = tsmc40ulpfmc_cache.unused;
    tsmc40ulpfmc_cache.unused = candidate;
done:
    result->used = true;
    result->address = page;
    result->writes = 0U;
    /* copy words from flash to page buffer without modification */
    ( void ) tsmc40ulpfmc_memcpy_from_flash(
        result->buffer.data,
        ( uint8_t const * ) page,
        TSMC40ULPFMC_PAGE_SIZE
    );
failure_io:
    __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));
    return result;
}

static flash_access_status_t
tsmc40ulpfmc_commit_page_nonblocking(
    tsmc40ulpfmc_cache_page_buffer_type * const buffer
)
{
    /* Assume the arguments are proper. */
    flash_access_status_t result = flash_check_status();
    if ( READY == result ) {
        result = tsmc40ulpfmc_commit_page_blocking( buffer );
    }

    return result;
}

static flash_access_status_t
tsmc40ulpfmc_commit_page_blocking(
    tsmc40ulpfmc_cache_page_buffer_type * const buffer
)
{
    /* Assume the arguments are proper. */
    flash_access_status_t result = ARGUMENT_ERROR;
    /*
     * Flash page consists of several sectors. We'll:
     * 1. erase entire page with given address
     * 2. refill it with given buffer data, sector by sector
     */
    __libc_lock_acquire_recursive( &( tsmc40ulpfmc_cache.lock ));

    uintptr_t address = PAGE_ALIGN( buffer->address );

    if (( ! buffer->used ) || ( FLASH_SIZE <= address )) {
        goto failure_arguments;
    }

    if ( 0U == buffer->writes ) {
        result = READY;
        goto done;
    }

#if TSMC40ULPFMC_RAM_BUFFERS_DEBUG
    printf( "write page 0x%x to flash\n",(unsigned)buffer->address );
#endif /* TSMC40ULPFMC_RAM_BUFFERS_DEBUG */

    AMBA_FLASH_PTR->ADDRESS = ( uint32_t ) address;
    flash_unlock_command();
    flash_issue_command( FLASH_COMMAND_ERASE_PAGE );
    result = flash_check_status();

    for (
        size_t sector = 0U;
        sector < ( TSMC40ULPFMC_PAGE_SIZE / TSMC40ULPFMC_SECTOR_SIZE );
        ++sector
    ) {
        flash_loop_while_busy();
        flash_sector_buffer_clear();
        flash_loop_while_busy();

        /* fill sector buffer with data */
        ( void ) tsmc40ulpfmc_memcpy_to_flash(
            ( uint8_t * ) ( address + ( sector * TSMC40ULPFMC_SECTOR_SIZE )),
            &( buffer->buffer.data[ sector * TSMC40ULPFMC_SECTOR_SIZE ] ),
            TSMC40ULPFMC_SECTOR_SIZE
        );

        /* commit sector buffer to flash */
        AMBA_FLASH_PTR->ADDRESS =
            ( uint32_t ) ( address + ( sector * TSMC40ULPFMC_SECTOR_SIZE ));
        flash_unlock_command();
        flash_issue_command( FLASH_COMMAND_WRITE_SECTOR );
        result = flash_check_status();
        if ( BUSY < result ) {
            goto failure_sector_buffer_write;
        }
    }

    /*
     * Need to flush data cache,
     * as it may not see updated values
     * in AHB address space
     */
    {
        DCACHE_PTR->FLUSH = 1U;
    }

    /* just set writes to zero, buffer may be rewritten shortly */
    buffer->writes = 0U;

failure_sector_buffer_write:
done:
failure_arguments:
    __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));
    return result;
}

static flash_access_status_t
tsmc40ulpfmc_cache_aware_read(
    void const * const address,
    uint8_t * const data,
    size_t const bytes
)
{
    flash_access_status_t result = ARGUMENT_ERROR;

    /*
     * by setting end byte to last actually written we work around situation
     * where one byte after is the first byte of a new word
     * depending on position (first of new word or not), we would need to
     * either not write a word or do write it, using branch
     * if instead we use last write, we don't have such problem, because
     * we always know we'll write the word including last byte
     */
    uintptr_t const uintptr_start = ( uintptr_t ) address;
    uintptr_t const uintptr_end = uintptr_start + bytes - 1U;

    /* cache operates on pages, calculate first and last page to read */
    uintptr_t const uintptr_start_page = PAGE_ALIGN( uintptr_start );
    /* last page containing last read byte */
    uintptr_t const uintptr_end_page = PAGE_ALIGN( uintptr_end );
    /*
     * AHB operates on words, calculate first and last words
     * to read in flash.
     */
    uintptr_t const uintptr_start_word = WORD_ALIGN( uintptr_start );
    uintptr_t const uintptr_end_word = WORD_ALIGN( uintptr_end );
    /*
     * where the first word is in first page,
     * between 0 and TSMC40ULPFMC_PAGE_SIZE exclusive
     */
    size_t const start_page_first_word_offset =
        ( uintptr_start_word - uintptr_start_page );
    /*
     * where the last word is in last page,
     * between 0 and TSMC40ULPFMC_PAGE_SIZE exclusive
     */
    size_t const end_page_last_word_offset =
        ( uintptr_end_word - uintptr_end_page );
    /* where the first byte is in first word, between 0 and 3 */
    size_t const start_page_first_word_first_byte_offset =
        ( uintptr_start - uintptr_start_word );
    /* where the last byte is in last word, between 0 and 3 */
    size_t const end_page_last_word_last_byte_offset =
        ( uintptr_end - uintptr_end_word );
    /*
     * difference_in_pages == 0U: we stay within the same page
     * else we need to get at least two different pages
     * ultimately we read (difference_in_pages + 1) pages
     */
    size_t const difference_in_pages =
        ( uintptr_end_page - uintptr_start_page ) / TSMC40ULPFMC_PAGE_SIZE;
    size_t const difference_in_words =
        ( uintptr_end_word - uintptr_start_word ) / ( sizeof( uint32_t ));

    /*
     * Reading is done word-aligned so it's faster for direct access.
     * Possible unaligned bytes in first and last words are handled separately.
     */
    register uint32_t wordbuffer = 0U;
    size_t buffer_index = 0U;
    tsmc40ulpfmc_cache_page_buffer_type * buffer = NULL;
    uint8_t const * src = NULL;

    /*
     * handle first page:
     * 1. test whether we read from cache or directly from flash
     * 2. read bytes within page from:
     *    start_page_first_word_offset + start_page_first_word_first_byte_offset
     *    to either:
     *    end_page_last_word_offset + end_page_last_word_last_byte_offset
     *    or end of page, depending on bytes count to read
     *
     * We have to read full words due to limitations of AHB. We do this to
     * maintain same code regardless of cache or direct access.
     */
     {
        src = ( uint8_t const * ) uintptr_start_page;
        __libc_lock_acquire_recursive( &( tsmc40ulpfmc_cache.lock ));
        buffer =
            tsmc40ulpfmc_cache_page_buffer_from_address(
                ( void const * ) uintptr_start_page
            );
        if ( NULL != buffer ) {
            src = buffer->buffer.data;
        }

        /*
         * handle first word within page:
         * copy bytes from uintptr_start_word_offset to either
         * end of word or uintptr_end_word (inclusive)
         */
        {
            /* in case of direct access, h/w translates into AHB access */
            wordbuffer =
                *(
                    ( uint32_t const * ) (
                        &( src[ start_page_first_word_offset ] )
                    )
                );
            wordbuffer = endianess_swap( wordbuffer );
            size_t const last_byte_offset = /* will be read often */
                (
                    ( 0U == difference_in_words )
                        ?
                            /* max 4, same word */
                            ( end_page_last_word_last_byte_offset + 1U )
                        :
                            sizeof( uint32_t )
                );
            for (
                size_t i = start_page_first_word_first_byte_offset;
                i < last_byte_offset;
                ( ++i ), ( ++buffer_index )
            ) {
                data[ buffer_index ] = WORD2BYTE( wordbuffer, i );
            }
        }

        if ( 0U == difference_in_words ) {
            goto done; /* we're done */
        }

        /*
         * handle intermediate full words, only if > 2 words are read
         * note: total words read == (difference_in_words + 1)
         * so if only two words are read, we won't enter this loop
         */
        {
            size_t const first_page_intermediate_words =
                (
                    ( 0U == difference_in_pages )
                        ?
                            difference_in_words
                        :
                            (
                                (
                                    TSMC40ULPFMC_PAGE_SIZE
                                    - start_page_first_word_offset
                                ) / sizeof( uint32_t )
                            )
                ) - 1U;
            size_t const copied =
                tsmc40ulpfmc_memcpy_from_flash(
                    &( data[ buffer_index ] ),
                    &( src[ start_page_first_word_offset + sizeof( uint32_t )]),
                    first_page_intermediate_words * sizeof( uint32_t )
                );
            buffer_index += copied;
        }

        if ( 0U == difference_in_pages ) {
            goto handle_end_word;
        }

        __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));
     }

    /*
     * handle intermediate full pages, only if > 2 pages are written:
     * note: pages written == (difference_in_pages + 1)
     * so if only two pages are written, we won't enter this loop
     *
     * We have to read full words due to limitations of AHB. We do this to
     * maintain same code regardless of cache or direct access.
     */
    for (
        size_t i = 1U; /* iterate through pages */
        i < difference_in_pages;
        ++i
    ) {
        uintptr_t const page_to_handle =
            uintptr_start_page + ( TSMC40ULPFMC_PAGE_SIZE * i );
        src = ( uint8_t const * ) page_to_handle;
        __libc_lock_acquire_recursive( &( tsmc40ulpfmc_cache.lock ));
        buffer =
            tsmc40ulpfmc_cache_page_buffer_from_address(
                ( void const * ) page_to_handle
            );
        if ( NULL != buffer ) {
            src = buffer->buffer.data;
        }

        /* read all words in the page */
        size_t const copied = tsmc40ulpfmc_memcpy_from_flash(
            &( data[ buffer_index ] ),
            src,
            TSMC40ULPFMC_PAGE_SIZE
        );
        buffer_index += copied;

        __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));
    }

    /*
     * handle last page:
     * 1. test whether we read from cache or directly from flash
     * 2. read bytes within page from:
     *    0
     *    to:
     *    end_page_last_word_offset + end_page_last_word_last_byte_offset
     *
     * We have to read full words due to limitations of AHB. We do this to
     * maintain same code regardless of cache or direct access.
     */
     {
        src = ( uint8_t const * ) uintptr_end_page;
        __libc_lock_acquire_recursive( &( tsmc40ulpfmc_cache.lock ));
        buffer =
            tsmc40ulpfmc_cache_page_buffer_from_address(
                ( void const * ) uintptr_end_page
            );
        if ( NULL != buffer ) {
            src = buffer->buffer.data;
        }

        /*
         * handle intermediate full words
         */
        size_t const copied =
            tsmc40ulpfmc_memcpy_from_flash(
                &( data[ buffer_index ] ),
                src,
                end_page_last_word_offset
            );
        buffer_index += copied;

handle_end_word:
        /*
         * handle last word within last page:
         */
        {
            /* in case of direct access, h/w translates into AHB access */
            wordbuffer =
                *(
                    ( uint32_t const * ) (
                        &( src[ end_page_last_word_offset ] )
                    )
                );
            wordbuffer = endianess_swap( wordbuffer );
            for (
                size_t i = 0U;
                i < ( end_page_last_word_last_byte_offset + 1U );
                ( ++i ), ( ++buffer_index )
            ) {
                data[ buffer_index ] = WORD2BYTE( wordbuffer, i );
            }
        }

done:
        __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));
     }

    result = flash_check_status();
    return result;
}

static void
tsmc40ulpfmc_cache_page_buffer_write_notify(
    tsmc40ulpfmc_cache_page_buffer_type * const buffer
)
{
    __libc_lock_acquire_recursive( &( tsmc40ulpfmc_cache.lock ));
    ++( buffer->writes );

    tsmc40ulpfmc_cache.last = buffer;
    if ( tsmc40ulpfmc_cache.most->writes < buffer->writes ) {
        tsmc40ulpfmc_cache.most = buffer;
    }
    __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));
}

static void
tsmc40ulpfmc_cache_maintainance( void )
{
    __libc_lock_acquire_recursive( &( tsmc40ulpfmc_cache.lock ));
    for ( size_t i = 0U; i < TSMC40ULPFMC_RAM_BUFFERS; ++i ) {
        tsmc40ulpfmc_cache_page_buffer_type * const buffer =
            &( tsmc40ulpfmc_cache.cache[ i ] );
        if ( ! buffer->used ) {
            continue;
        }
        if ( tsmc40ulpfmc_cache.threshold.max <= buffer->writes ) {
            /* commit will also fix most if it succeeds */
            tsmc40ulpfmc_commit_page_blocking( buffer );
        }
        /* successful blocking commit shall reset writes to zero */
        if ( tsmc40ulpfmc_cache.threshold.min <= buffer->writes ) {
            /* commit will also fix most if it succeeds */
            tsmc40ulpfmc_commit_page_nonblocking( buffer );
        }
    }
    __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));
}

static flash_access_status_t
tsmc40ulpfmc_cache_aware_write(
    void * const address,
    uint8_t const * const data,
    size_t const bytes
)
{
    flash_access_status_t result = ARGUMENT_ERROR;

    /*
     * by setting end byte to last actually written we work around situation
     * where one byte after is the first byte of a new word
     * depending on position (first of new word or not), we would need to
     * either not write a word or do write it, using branch
     * if instead we use last write, we don't have such problem, because
     * we always know we'll write the word including last byte
     */
    uintptr_t const uintptr_start = ( uintptr_t ) address;
    uintptr_t const uintptr_end = uintptr_start + bytes - 1U;

    /* cache operates on pages, calculate first and last page to read */
    uintptr_t const uintptr_start_page = PAGE_ALIGN( uintptr_start );
    /* last page containing last read byte */
    uintptr_t const uintptr_end_page = PAGE_ALIGN( uintptr_end );
    /*
     * AHB operates on words, calculate first and last words
     * to read in flash.
     */
    uintptr_t const uintptr_start_word = WORD_ALIGN( uintptr_start );
    uintptr_t const uintptr_end_word = WORD_ALIGN( uintptr_end );
    /*
     * where the first word is in first page,
     * between 0 and TSMC40ULPFMC_PAGE_SIZE exclusive
     */
    size_t const start_page_first_word_offset =
        ( uintptr_start_word - uintptr_start_page );
    /*
     * where the last word is in last page,
     * between 0 and TSMC40ULPFMC_PAGE_SIZE exclusive
     */
    size_t const end_page_last_word_offset =
        ( uintptr_end_word - uintptr_end_page );
    /* where the first byte is in first word, between 0 and 3 */
    size_t const start_page_first_word_first_byte_offset =
        ( uintptr_start - uintptr_start_word );
    /* where the last byte is in last word, between 0 and 3 */
    size_t const end_page_last_word_last_byte_offset =
        ( uintptr_end - uintptr_end_word );
    /*
     * difference_in_pages == 0U: we stay within the same page
     * else we need to get at least two different pages
     * ultimately we read (difference_in_pages + 1) pages
     */
    size_t const difference_in_pages =
        ( uintptr_end_page - uintptr_start_page ) / TSMC40ULPFMC_PAGE_SIZE;
    size_t const difference_in_words =
        ( uintptr_end_word - uintptr_start_word ) / ( sizeof( uint32_t ));

    size_t buffer_index = 0U;
    tsmc40ulpfmc_cache_page_buffer_type * buffer = NULL;
    uint8_t * dst = NULL;

    tsmc40ulpfmc_cache_print_state();
    /*
     * handle first page:
     * 1. write bytes within page from:
     *    start_page_first_word_offset + start_page_first_word_first_byte_offset
     *    to either:
     *    end_page_last_word_offset + end_page_last_word_last_byte_offset
     *    or end of page, depending on bytes count to read
     *
     * We have to read full words due to limitations of AHB. We do this to
     * maintain same code regardless of cache or direct access.
     */
     {
        __libc_lock_acquire_recursive( &( tsmc40ulpfmc_cache.lock ));
        buffer =
            tsmc40ulpfmc_cache_page_buffer_from_address(
                ( void const * ) uintptr_start_page
            );
        if ( NULL == buffer ) {
            buffer =
                tsmc40ulpfmc_cache_page_buffer_allocate(
                    ( void const * ) uintptr_start_page
                );
        }
        if ( NULL == buffer ) {
            result = PROGRAMMING_ERROR;
            __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));
            goto failure_buffer;
        }
        dst = buffer->buffer.data;

        /*
         * handle first word within page:
         * copy bytes from uintptr_start_word_offset to either
         * end of word or uintptr_end_word (inclusive)
         */
        {
            size_t const last_byte_offset =
                (
                    ( 0U == difference_in_words )
                        ?
                            /* max 4, same word */
                            ( end_page_last_word_last_byte_offset + 1U )
                        :
                            sizeof( uint32_t )
                );
            size_t const bytes_to_copy =
                last_byte_offset - start_page_first_word_first_byte_offset;
            ( void ) memcpy(
                &(
                    dst[
                        start_page_first_word_offset
                        + start_page_first_word_first_byte_offset
                    ]
                 ),
                &( data[ buffer_index ] ),
                bytes_to_copy
            );
            buffer_index += bytes_to_copy;
        }

        if ( 0U == difference_in_words ) {
            goto done; /* we're done */
        }

        /*
         * handle intermediate full words, only if > 2 words are read
         * note: total words read == (difference_in_words + 1)
         * so if only two words are read, we won't enter this loop
         */
        {
            size_t const first_page_intermediate_words =
                (
                    ( 0U == difference_in_pages )
                        ?
                            difference_in_words
                        :
                            (
                                (
                                    TSMC40ULPFMC_PAGE_SIZE
                                    - start_page_first_word_offset
                                ) / sizeof( uint32_t )
                            )
                ) - 1U;
            size_t const bytes_to_copy =
                first_page_intermediate_words * sizeof( uint32_t );
            ( void ) memcpy(
                &( dst[ start_page_first_word_offset + sizeof( uint32_t ) ] ),
                &( data[ buffer_index ] ),
                bytes_to_copy
            );
            buffer_index += bytes_to_copy;
        }

        if ( 0U == difference_in_pages ) {
            goto handle_end_word;
        }

        tsmc40ulpfmc_cache_page_buffer_write_notify( buffer );
        __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));
     }

    tsmc40ulpfmc_cache_print_state();
    /*
     * handle intermediate full pages, only if > 2 pages are written:
     * note: pages written == (difference_in_pages + 1)
     * so if only two pages are written, we won't enter this loop
     *
     * We have to read full words due to limitations of AHB. We do this to
     * maintain same code regardless of cache or direct access.
     */
    for (
        size_t i = 1U; /* iterate through pages */
        i < difference_in_pages;
        ++i
    ) {
        uintptr_t const page_to_handle =
            uintptr_start_page + ( TSMC40ULPFMC_PAGE_SIZE * i );
        __libc_lock_acquire_recursive( &( tsmc40ulpfmc_cache.lock ));
        buffer =
            tsmc40ulpfmc_cache_page_buffer_from_address(
                ( void const * ) page_to_handle
            );
        if ( NULL == buffer ) {
            buffer =
                tsmc40ulpfmc_cache_page_buffer_allocate(
                    ( void const * ) page_to_handle
                );
        }
        if ( NULL == buffer ) {
            result = PROGRAMMING_ERROR;
            __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));
            goto failure_buffer;
        }
        dst = buffer->buffer.data;

        ( void ) memcpy(
            dst,
            &( data[ buffer_index ] ),
            TSMC40ULPFMC_PAGE_SIZE
        );
        buffer_index += TSMC40ULPFMC_PAGE_SIZE;

        tsmc40ulpfmc_cache_page_buffer_write_notify( buffer );
        __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));
        tsmc40ulpfmc_cache_print_state();
    }

    /*
     * handle last page:
     * 1. write bytes within page from:
     *    0
     *    to:
     *    end_page_last_word_offset + end_page_last_word_last_byte_offset
     *
     * We have to read full words due to limitations of AHB. We do this to
     * maintain same code regardless of cache or direct access.
     */
     {
        __libc_lock_acquire_recursive( &( tsmc40ulpfmc_cache.lock ));
        buffer =
            tsmc40ulpfmc_cache_page_buffer_from_address(
                ( void const * ) uintptr_end_page
            );
        if ( NULL == buffer ) {
            buffer =
                tsmc40ulpfmc_cache_page_buffer_allocate(
                    ( void const * ) uintptr_end_page
                );
        }
        if ( NULL == buffer ) {
            result = PROGRAMMING_ERROR;
            __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));
            goto failure_buffer;
        }
        dst = buffer->buffer.data;

        /*
         * handle intermediate full words
         */
        ( void ) memcpy(
            dst,
            &( data[ buffer_index ] ),
            end_page_last_word_offset
        );
        buffer_index += end_page_last_word_offset;

handle_end_word:
        /*
         * handle last word within last page:
         */
        {
            ( void ) memcpy(
                &( dst[ end_page_last_word_offset ] ),
                &( data[ buffer_index ] ),
                end_page_last_word_last_byte_offset + 1U
            );
            buffer_index += ( end_page_last_word_last_byte_offset + 1U );
        }

done:
        tsmc40ulpfmc_cache_page_buffer_write_notify( buffer );
        __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));
     }

    tsmc40ulpfmc_cache_print_state();
    tsmc40ulpfmc_cache_maintainance();
    result = flash_check_status();
failure_buffer:
    return result;
}



/*! \brief Clears (set to 0xFF) the content of Sector Buffer.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t
flash_sector_buffer_clear( void )
{
    flash_unlock_command();
    flash_issue_command( FLASH_COMMAND_CLR_SECTOR_BUFFER );
    return flash_check_status();
}

/*! \brief Writes quad-word of data into the sector buffer.
 *
 * \param sector_buffer_start_word_offset The first word offset of the data within the sector buffer (needs to be aligned to 4 x 32-bit words).
 * \param data_qword The data quad-word to be written into sector buffer.
 *
 * This function sets up all registers and starts the write operation but doesn't wait for it to complete.
 * User is expected to monitor status until the write process is completed.
 * The write is done using APB indirect access.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t
flash_sector_buffer_write_qword(
    uint32_t const offset,
    flash_qword_t const data
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    uint32_t const address = offset % TSMC40ULPFMC_SECTOR_SIZE;
    if ( ! IS_QWORD_ALIGNED( address )) {
        goto failure_arguments;
    }

    AMBA_FLASH_PTR->DATA_WORD_0 = data.data[ 0U ];
    AMBA_FLASH_PTR->DATA_WORD_1 = data.data[ 1U ];
    AMBA_FLASH_PTR->DATA_WORD_2 = data.data[ 2U ];
    AMBA_FLASH_PTR->DATA_WORD_3 = data.data[ 3U ];
    AMBA_FLASH_PTR->ADDRESS = address;

    flash_unlock_command();
    flash_issue_command( FLASH_COMMAND_WRITE_SECTOR_BUFFER_QWORD );
    result = flash_check_status();

failure_arguments:
    return result;
}

/*! \brief Writes single word of data into the sector buffer.
 *
 * \param offset Sector buffer offset (needs to be aligned to 32-bit word).
 * \param data The data word to be written into sector buffer.
 * \warning Given offset shall be adjusted modulo sector buffer size.
 *
 * This function uses direct access to the Sector Buffer, meaning it won't return until the write actually completed.
 * Status returned shall either be READY or one of the error codes.
 * The write is done using APB direct access.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t
flash_sector_buffer_write_word_direct(
    uint32_t const offset,
    uint32_t const data
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    uint32_t const address = offset % TSMC40ULPFMC_SECTOR_SIZE;
    if ( ! IS_WORD_ALIGNED( address )) {
        goto failure_arguments;
    }

    AMBA_FLASH_PTR->SECTOR_BUFFER[ address ] = data;
    result = flash_check_status();

failure_arguments:
    return result;
}

/*! \brief Reads qword of data from the sector buffer.
 *
 * \param offset Sector buffer offset (needs to be aligned to 128-bit qword).
 * \param data Container for qword to read from sector buffer.
 * \warning Given offset shall be adjusted modulo sector buffer size.
 *
 * This function reads single data qword from the Sector Buffer.
 * The read is done using APB indirect access.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t
flash_sector_buffer_read_qword(
    uint32_t const offset,
    flash_qword_t * const data
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    uint32_t const address = offset % TSMC40ULPFMC_SECTOR_SIZE;
    if ( ! IS_QWORD_ALIGNED( address )) {
        goto failure_arguments;
    }
    if ( NULL == data ) {
        goto failure_arguments;
    }

    AMBA_FLASH_PTR->ADDRESS = address;

    flash_unlock_command();
    flash_issue_command( FLASH_COMMAND_READ_SECTOR_BUFFER_QWORD );

    result = flash_check_status();
    if ( BUSY < result ) {
        goto failure_command;
    }

    data->data[ 0U ] = AMBA_FLASH_PTR->DATA_WORD_0;
    data->data[ 1U ] = AMBA_FLASH_PTR->DATA_WORD_1;
    data->data[ 2U ] = AMBA_FLASH_PTR->DATA_WORD_2;
    data->data[ 3U ] = AMBA_FLASH_PTR->DATA_WORD_3;

failure_command:
failure_arguments:
    return result;
}

/*! \brief Reads single word of data from the sector buffer.
 *
 * \param offset Sector buffer offset (needs to be aligned to 32-bit word).
 * \param data Container for word to read from sector buffer.
 *
 * This function reads single data word from the Sector Buffer.
 * The read is done using APB direct access.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t
flash_sector_buffer_read_word(
    uint32_t const offset,
    uint32_t * const data
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    uint32_t const address = offset % TSMC40ULPFMC_SECTOR_SIZE;
    if ( ! IS_WORD_ALIGNED( address )) {
        goto failure_arguments;
    }
    if ( NULL == data ) {
        goto failure_arguments;
    }

    *data = AMBA_FLASH_PTR->SECTOR_BUFFER[ address ];

    result = flash_check_status();

failure_arguments:
    return result;
}

/*! \brief Unlocks ability to write data into flash.
 */
void
flash_unlock( void )
{
    csr_write( mromunlock, ROM_UNLOCK_KEY | ROM_UNLOCK_VALUE );
}

/*! \brief Writes contents of the sector buffer into program memory.
 * Since cache is used, the contents are written into it. Then a non-blocking
 * cache page commit is done.
 *
 * \param offset Flash offset (needs to be aligned to sector size).
 *
 * This function stores current contents of the sector buffer in program memory.
 * The write is done using APB indirect access.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 *
 * Important: for cache-awareness, this function shall write contents of the
 * sector buffer in appropriate cache buffer page, allocating it if needed.
 */
flash_access_status_t
flash_sector_buffer_write(
    void * const address
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    uint32_t const offset = ( uintptr_t ) address;
    if ( ! IS_SECTOR_ALIGNED( offset )) {
        goto failure_arguments;
    }

    uintptr_t const page = PAGE_ALIGN(( uintptr_t ) address );
    size_t const page_start_offset = (( uintptr_t ) address ) - page;

    __libc_lock_acquire_recursive( &( tsmc40ulpfmc_cache.lock ));

    tsmc40ulpfmc_cache_page_buffer_type * buffer =
        tsmc40ulpfmc_cache_page_buffer_from_address(
            ( void const * ) page
        );
    if ( NULL == buffer ) {
        buffer =
            tsmc40ulpfmc_cache_page_buffer_allocate(
                ( void const * ) page
            );
    }
    if ( NULL == buffer ) {
        result = PROGRAMMING_ERROR;
        goto failure_buffer;
    }

    ( void ) tsmc40ulpfmc_memcpy_from_flash(
        &( buffer->buffer.data[ page_start_offset ] ),
        /* read will be done via AHB, word-based */
        ( uint8_t const * ) AMBA_FLASH_PTR->SECTOR_BUFFER,
        TSMC40ULPFMC_SECTOR_SIZE
    );

    tsmc40ulpfmc_cache_page_buffer_write_notify( buffer );
    result = tsmc40ulpfmc_commit_page_blocking( buffer );

failure_buffer:
    __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));

failure_arguments:
    return result;
}

/*! \brief Writes data qword to program memory.
 *
 * \param address The start address of memory to be written (needs to be aligned to 128-bit qwords).
 * \param data Data quad-word to be written.
 *
 * This function copies provided data qword to sector buffer and initiates
 * qword write operation. The previous contents of sector buffer space assigned
 * to the given addess are replaced with given qword data. Note that only a
 * qword is written, not all data from sector buffer.
 * This function will not wait for the write to finish.
 * The write is done using APB indirect access.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data could not be written to flash memory due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t
flash_write_qword(
	void * const address,
    flash_qword_t const data
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    uint32_t const offset = ( uintptr_t ) address;
    if ( ! IS_QWORD_ALIGNED( offset )) {
        goto failure_arguments;
    }

    uint8_t buffer[ 4U * sizeof( uint32_t ) ] = { 0U, };

    for ( size_t i = 0U; i < 4U; ++i ) {
        register uint32_t wordbuffer = data.data[ i ];
        wordbuffer = endianess_swap( wordbuffer );

        buffer[ ( i * sizeof( uint32_t )) + 0U ] = WORD2BYTE( wordbuffer, 0U );
        buffer[ ( i * sizeof( uint32_t )) + 1U ] = WORD2BYTE( wordbuffer, 1U );
        buffer[ ( i * sizeof( uint32_t )) + 2U ] = WORD2BYTE( wordbuffer, 2U );
        buffer[ ( i * sizeof( uint32_t )) + 3U ] = WORD2BYTE( wordbuffer, 3U );
    }

    result =
        tsmc40ulpfmc_cache_aware_write(
            address,
            buffer,
            sizeof( buffer )
        );

failure_arguments:
    return result;
}

/*! \brief Writes data qword to program memory.
 *
 * \param address The start address of memory to be written (needs to be aligned to 128-bit qwords).
 * \param data Data quad-word to be written.
 *
 * This function writes provided data qword to program memory. Sector buffer is
 * not used. This function will not wait for the write to finish.
 * The write is done using APB indirect access.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data could not be written to flash memory due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t
flash_write_qword_immediate(
	void * const address,
    flash_qword_t const data
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    uint32_t const offset = ( uintptr_t ) address;
    if ( ! IS_QWORD_ALIGNED( offset )) {
        goto failure_arguments;
    }

    uint8_t buffer[ 4U * sizeof( uint32_t ) ] = { 0U, };

    for ( size_t i = 0U; i < 4U; ++i ) {
        register uint32_t wordbuffer = data.data[ i ];
        wordbuffer = endianess_swap( wordbuffer );

        buffer[ ( i * sizeof( uint32_t )) + 0U ] = WORD2BYTE( wordbuffer, 0U );
        buffer[ ( i * sizeof( uint32_t )) + 1U ] = WORD2BYTE( wordbuffer, 1U );
        buffer[ ( i * sizeof( uint32_t )) + 2U ] = WORD2BYTE( wordbuffer, 2U );
        buffer[ ( i * sizeof( uint32_t )) + 3U ] = WORD2BYTE( wordbuffer, 3U );
    }

    result =
        tsmc40ulpfmc_cache_aware_write(
            address,
            buffer,
            sizeof( buffer )
        );

    flash_sync();

failure_arguments:
    return result;
}

/*! \brief Erases single data page from program memory.
 *
 * \param address Address to be erased, must be page-aligned.
 * \see flash_page_size
 *
 * This function initializes erasure of single page from program memory.
 * It will not wait for the erase operation to complete.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t
flash_erase_page(
	void * const address
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    uint32_t const offset = ( uintptr_t ) address;
    if ( ! IS_PAGE_ALIGNED( offset )) {
        goto failure_arguments;
    }

    __libc_lock_acquire_recursive( &( tsmc40ulpfmc_cache.lock ));

    tsmc40ulpfmc_cache_page_buffer_type * const buffer =
        tsmc40ulpfmc_cache_page_buffer_from_address( address );
    if ( NULL != buffer ) {
        tsmc40ulpfmc_cache_page_buffer_free( buffer );
    }

    AMBA_FLASH_PTR->ADDRESS = offset;
    flash_unlock_command();
    flash_issue_command( FLASH_COMMAND_ERASE_PAGE );
    result = flash_check_status();

    /*
     * Need to flush data cache,
     * as it may not see updated values
     * in AHB address space
     */
    {
        DCACHE_PTR->FLUSH = 1U;
    }

    __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));

failure_arguments:
    return result;
}

/*! \brief Erases whole program memory.
 *
 * This function initializes erasure of whole program memory (data + region lock bits + debugger lock bits).
 * It will not wait for the erase operation to complete.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t
flash_erase_all( void )
{
    flash_access_status_t result = ARGUMENT_ERROR;

    __libc_lock_acquire_recursive( &( tsmc40ulpfmc_cache.lock ));

    for ( size_t i = 0U; i < TSMC40ULPFMC_RAM_BUFFERS; ++i ) {
        tsmc40ulpfmc_cache_page_buffer_free(
            &( tsmc40ulpfmc_cache.cache[ i ] )
        );
    }

    tsmc40ulpfmc_cache.last = &( tsmc40ulpfmc_cache.cache[ 0U ] );
    tsmc40ulpfmc_cache.most = &( tsmc40ulpfmc_cache.cache[ 0U ] );
    tsmc40ulpfmc_cache.unused = &( tsmc40ulpfmc_cache.cache[ 0U ] );

    flash_unlock_command();
    flash_issue_command( FLASH_COMMAND_ERASE_ALL) ;
    result = flash_check_status();

    /*
     * Need to flush data cache,
     * as it may not see updated values
     * in AHB address space
     */
    {
        DCACHE_PTR->FLUSH = 1U;
    }

    __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));

    return result;
}

/*! \brief Erases reference cells of memory macros.
 *
 * This function initializes erasure of memory macros reference cells.
 * It will not wait for the erase operation to complete.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t
flash_erase_reference_cell( void )
{
    flash_unlock_erase_reference_cell_command();
    flash_issue_command( FLASH_COMMAND_ERASE_REFERENCE_CELL );
    return flash_check_status();
}

/*! \brief Locks region of program memory for write and erase.
 *
 * \param address_within_region_to_lock Address within the memory region to be locked.
 *
 * This function initializes procedure for locking single memory region. It will not wait for the operation to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t
flash_lock_region(
    void const * const address
)
{
    AMBA_FLASH_PTR->ADDRESS = ( uint32_t )(( uintptr_t ) address );

    flash_unlock_command();
    flash_issue_command( FLASH_COMMAND_LOCK_REGION );
    return flash_check_status();
}

/*! \brief Locks ability to read memory content via debugger.
 *
 * This function initializes procedure for locking debugger read access to memory.
 * It will not wait for the operation to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t
flash_lock_debugger_read( void )
{
    flash_unlock_command();
    flash_issue_command( FLASH_COMMAND_LOCK_DEBUGGER_READ );
    return flash_check_status();
}

/*! \brief Locks ability to access processor through debugger.
 *
 * This function initializes procedure for locking debugger access.
 * It will not wait for the operation to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t
flash_lock_debugger_access( void )
{
    flash_unlock_command();
    flash_issue_command( FLASH_COMMAND_LOCK_DEBUGGER_ACCESS );
    return flash_check_status();
}

/*! \brief Checks if region containing provided address is locked for write/erase access.
 *
 * \param address_within_region_to_check_lock Address within the region to check for.
 *
 * \return State of the region to check for.
 *   \retval true region is locked for write/erase
 *   \retval false region is open for write/erase
 */
bool
flash_is_region_locked(
    void const * const address
)
{
    uint32_t const offset = ( uint32_t )(( uintptr_t ) address );
    uint32_t const region_lock_bit_number = offset / flash_region_size();

    uint8_t const region_locks_reg_offset = region_lock_bit_number / 32U;
    uint8_t const region_locks_bit_offset = region_lock_bit_number % 32U;

    bool const result =
        (
            0U != (
                AMBA_FLASH_PTR->REGION_LOCKS[ region_locks_reg_offset ]
                & ( 1U << region_locks_bit_offset )
            )
        );
    return result;
}

/*! \brief Writes quad-word of data into the factory row.
 *
 * \param offset Offset within the factory row (needs to be aligned to 128-bit qwords).
 * \param data The data quad-word to be written into the factory row.
 *
 * This function sets up all registers and starts the write operation but doesn't wait for it to complete.
 * User is expected to monitor status to learn when the write process is completed.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t
flash_factory_row_write_qword(
	uint32_t offset,
    flash_qword_t const data
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    if ( ! IS_QWORD_ALIGNED( offset )) {
        goto failure_arguments;
    }
    if ( flash_page_size() <= offset ) {
        goto failure_arguments;
    }

    AMBA_FLASH_PTR->DATA_WORD_0 = data.data[ 0U ];
    AMBA_FLASH_PTR->DATA_WORD_1 = data.data[ 1U ];
    AMBA_FLASH_PTR->DATA_WORD_2 = data.data[ 2U ];
    AMBA_FLASH_PTR->DATA_WORD_3 = data.data[ 3U ];
    AMBA_FLASH_PTR->ADDRESS = offset;

    flash_unlock_factory_row_command();
    flash_issue_command( FLASH_COMMAND_WRITE_FACTORY_ROW_QWORD );
    result = flash_check_status();

failure_arguments:
    return result;
}

/*! \brief Reads quad-word of data from the factory row.
 *
 * \param offset The offset in the factory row (needs to be aligned to 128-bit qwords).
 * \param data Pointer to a buffer where read data will be placed.
 *
 * This function reads data quad-word from the factory row.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t
flash_factory_row_read_qword(
	uint32_t offset,
    flash_qword_t * const data
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    if ( ! IS_QWORD_ALIGNED( offset )) {
        goto failure_arguments;
    }
    if ( flash_page_size() <= offset ) {
        goto failure_arguments;
    }
    if ( NULL == data ) {
        goto failure_arguments;
    }

    AMBA_FLASH_PTR->ADDRESS = offset;

    flash_unlock_factory_row_command();
    flash_issue_command( FLASH_COMMAND_READ_FACTORY_ROW_QWORD );
    result = flash_check_status();
    if ( BUSY < result ) {
        goto failure_command;
    }
    flash_loop_while_busy();

    data->data[ 0U ] = AMBA_FLASH_PTR->DATA_WORD_0;
    data->data[ 1U ] = AMBA_FLASH_PTR->DATA_WORD_1;
    data->data[ 2U ] = AMBA_FLASH_PTR->DATA_WORD_2;
    data->data[ 3U ] = AMBA_FLASH_PTR->DATA_WORD_3;

failure_command:
failure_arguments:
    return result;

}

/*! \brief Reads single word of data from the factory row.
 *
 * \param offset The offset in the factory row (needs to be aligned to 32-bit qwords).
 * \param data Pointer to a buffer where read data will be placed.
 *
 * This function reads data word from the factory row.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t
flash_factory_row_read_word(
	uint32_t offset,
    uint32_t * const data
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    if ( ! IS_WORD_ALIGNED( offset )) {
        goto failure_arguments;
    }
    if ( flash_page_size() <= offset ) {
        goto failure_arguments;
    }
    if ( NULL == data ) {
        goto failure_arguments;
    }

    uint32_t const qoffset = offset & 0xFFFFFFF0;
    flash_qword_t qdata = { .data = { 0U, }};

    result = flash_factory_row_read_qword( qoffset, &qdata );
    if ( READY < result ) {
        goto failure_command;
    }

    *data = qdata.data[ ( offset >> 2U ) % 4U ];

failure_command:
failure_arguments:
    return result;
}

/*! \brief Erases content of factory row.
 *
 * This function initializes erasure of the data contained in factory row.
 * It will not wait for the erase operation to complete.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t
flash_factory_row_erase( void )
{
    flash_unlock_factory_row_command();
    flash_issue_command( FLASH_COMMAND_ERASE_FACTORY_ROW );
    return flash_check_status();
}

/*! \brief Locks ability to write, read and erase data within factory row.
 *
 * This function initializes procedure for locking factory row write/read/erase access to memory.
 * It will not wait for the operation to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t
flash_factory_row_lock( void )
{
    flash_unlock_factory_row_command();
    flash_issue_command( FLASH_COMMAND_LOCK_FACTORY_ROW );
    return flash_check_status();
}

/*! \brief Writes quad-word of data into the user row.
 *
 * \param offset Offset within the user row (needs to be aligned to 128-bit qwords).
 * \param data The data quad-word to be written into the user row.
 *
 * This function sets up all registers and starts the write operation but doesn't wait for it to complete.
 * User is expected to monitor status to learn when the write process is completed.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t
flash_user_row_write_qword(
	uint32_t offset,
    flash_qword_t const data
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    if ( ! IS_QWORD_ALIGNED( offset )) {
        goto failure_arguments;
    }
    if ( flash_page_size() <= offset ) {
        goto failure_arguments;
    }

    AMBA_FLASH_PTR->DATA_WORD_0 = data.data[ 0U ];
    AMBA_FLASH_PTR->DATA_WORD_1 = data.data[ 1U ];
    AMBA_FLASH_PTR->DATA_WORD_2 = data.data[ 2U ];
    AMBA_FLASH_PTR->DATA_WORD_3 = data.data[ 3U ];
    AMBA_FLASH_PTR->ADDRESS = offset;

    flash_unlock_user_row_command();
    flash_issue_command( FLASH_COMMAND_WRITE_USER_ROW_QWORD );
    result = flash_check_status();

failure_arguments:
    return result;
}

/*! \brief Reads quad-word of data from the user row.
 *
 * \param offset The offset in the user row (needs to be aligned to 128-bit qwords).
 * \param data Pointer to a buffer where read data will be placed.
 *
 * This function reads data quad-word from the user row.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t
flash_user_row_read_qword(
	uint32_t offset,
    flash_qword_t * const data
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    if ( ! IS_QWORD_ALIGNED( offset )) {
        goto failure_arguments;
    }
    if ( flash_page_size() <= offset ) {
        goto failure_arguments;
    }
    if ( NULL == data ) {
        goto failure_arguments;
    }

    AMBA_FLASH_PTR->ADDRESS = offset;

    flash_unlock_user_row_command();
    flash_issue_command( FLASH_COMMAND_READ_USER_ROW_QWORD );
    result = flash_check_status();
    if ( BUSY < result ) {
        goto failure_command;
    }
    flash_loop_while_busy();

    data->data[ 0U ] = AMBA_FLASH_PTR->DATA_WORD_0;
    data->data[ 1U ] = AMBA_FLASH_PTR->DATA_WORD_1;
    data->data[ 2U ] = AMBA_FLASH_PTR->DATA_WORD_2;
    data->data[ 3U ] = AMBA_FLASH_PTR->DATA_WORD_3;

failure_command:
failure_arguments:
    return result;

}

/*! \brief Reads single word of data from the user row.
 *
 * \param offset The offset in the user row (needs to be aligned to 32-bit qwords).
 * \param data Pointer to a buffer where read data will be placed.
 *
 * This function reads data word from the user row.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t
flash_user_row_read_word(
	uint32_t offset,
    uint32_t * const data
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    if ( ! IS_WORD_ALIGNED( offset )) {
        goto failure_arguments;
    }
    if ( flash_page_size() <= offset ) {
        goto failure_arguments;
    }
    if ( NULL == data ) {
        goto failure_arguments;
    }

    uint32_t const qoffset = offset & 0xFFFFFFF0;
    flash_qword_t qdata = { .data = { 0U, }};

    result = flash_user_row_read_qword( qoffset, &qdata );
    if ( READY < result ) {
        goto failure_command;
    }

    *data = qdata.data[ ( offset >> 2U ) % 4U ];

failure_command:
failure_arguments:
    return result;
}

/*! \brief Erases content of user row.
 *
 * This function initializes erasure of the data contained in user row.
 * It will not wait for the erase operation to complete.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t
flash_user_row_erase( void )
{
    flash_unlock_user_row_command();
    flash_issue_command( FLASH_COMMAND_ERASE_USER_ROW );
    return flash_check_status();
}

/*! \brief Locks ability to write, read and erase data within user row.
 *
 * This function initializes procedure for locking user row write/read/erase access to memory.
 * It will not wait for the operation to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t
flash_user_row_lock( void )
{
    flash_unlock_user_row_command();
    flash_issue_command( FLASH_COMMAND_LOCK_USER_ROW );
    return flash_check_status();
}

/*! \brief Writes quad-word of data into the manufacturer row.
 *
 * \param offset Offset within the manufacturer row (needs to be aligned to 128-bit qwords).
 * \param data The data quad-word to be written into the manufacturer row.
 *
 * This function sets up all registers and starts the write operation but doesn't wait for it to complete.
 * User is expected to monitor status to learn when the write process is completed.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t
flash_manufacturer_row_write_qword(
	uint32_t offset,
    flash_qword_t const data
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    if ( ! IS_QWORD_ALIGNED( offset )) {
        goto failure_arguments;
    }
    if ( flash_page_size() <= offset ) {
        goto failure_arguments;
    }

    AMBA_FLASH_PTR->DATA_WORD_0 = data.data[ 0U ];
    AMBA_FLASH_PTR->DATA_WORD_1 = data.data[ 1U ];
    AMBA_FLASH_PTR->DATA_WORD_2 = data.data[ 2U ];
    AMBA_FLASH_PTR->DATA_WORD_3 = data.data[ 3U ];
    AMBA_FLASH_PTR->ADDRESS = offset;

    flash_unlock_manufacturer_row_command();
    flash_issue_command( FLASH_COMMAND_WRITE_MANUFACTURER_ROW_QWORD );
    result = flash_check_status();

failure_arguments:
    return result;
}

/*! \brief Reads quad-word of data from the manufacturer row.
 *
 * \param offset The offset in the manufacturer row (needs to be aligned to 128-bit qwords).
 * \param data Pointer to a buffer where read data will be placed.
 *
 * This function reads data quad-word from the manufacturer row.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t
flash_manufacturer_row_read_qword(
	uint32_t offset,
    flash_qword_t * const data
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    if ( ! IS_QWORD_ALIGNED( offset )) {
        goto failure_arguments;
    }
    if ( flash_page_size() <= offset ) {
        goto failure_arguments;
    }
    if ( NULL == data ) {
        goto failure_arguments;
    }

    AMBA_FLASH_PTR->ADDRESS = offset;

    flash_unlock_manufacturer_row_command();
    flash_issue_command( FLASH_COMMAND_READ_MANUFACTURER_ROW_QWORD );
    result = flash_check_status();
    if ( BUSY < result ) {
        goto failure_command;
    }
    flash_loop_while_busy();

    data->data[ 0U ] = AMBA_FLASH_PTR->DATA_WORD_0;
    data->data[ 1U ] = AMBA_FLASH_PTR->DATA_WORD_1;
    data->data[ 2U ] = AMBA_FLASH_PTR->DATA_WORD_2;
    data->data[ 3U ] = AMBA_FLASH_PTR->DATA_WORD_3;

failure_command:
failure_arguments:
    return result;

}

/*! \brief Reads single word of data from the manufacturer row.
 *
 * \param offset The offset in the manufacturer row (needs to be aligned to 32-bit qwords).
 * \param data Pointer to a buffer where read data will be placed.
 *
 * This function reads data word from the manufacturer row.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t
flash_manufacturer_row_read_word(
	uint32_t offset,
    uint32_t * const data
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    if ( ! IS_WORD_ALIGNED( offset )) {
        goto failure_arguments;
    }
    if ( flash_page_size() <= offset ) {
        goto failure_arguments;
    }
    if ( NULL == data ) {
        goto failure_arguments;
    }

    uint32_t const qoffset = offset & 0xFFFFFFF0;
    flash_qword_t qdata = { .data = { 0U, }};

    result = flash_manufacturer_row_read_qword( qoffset, &qdata );
    if ( READY < result ) {
        goto failure_command;
    }

    *data = qdata.data[ ( offset >> 2U ) % 4U ];

failure_command:
failure_arguments:
    return result;
}

/*! \brief Erases content of manufacturer row.
 *
 * This function initializes erasure of the data contained in manufacturer row.
 * It will not wait for the erase operation to complete.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t
flash_manufacturer_row_erase( void )
{
    flash_unlock_manufacturer_row_command();
    flash_issue_command( FLASH_COMMAND_ERASE_MANUFACTURER_ROW );
    return flash_check_status();
}

/*! \brief Locks ability to write, read and erase data within manufacturer row.
 *
 * This function initializes procedure for locking manufacturer row write/read/erase access to memory.
 * It will not wait for the operation to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t
flash_manufacturer_row_lock( void )
{
    flash_unlock_manufacturer_row_command();
    flash_issue_command( FLASH_COMMAND_LOCK_MANUFACTURER_ROW );
    return flash_check_status();
}

/*! \brief Erases whole flash memory (including all rows and lock bits).
 *
 * This function initializes erasure of whole flash memory (data + all data rows + all lock bits).
 * It will not wait for the erase operation to complete.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t
flash_chip_erase( void )
{
    flash_unlock_command();
    flash_issue_command( FLASH_COMMAND_CHIP_ERASE );
    return flash_check_status();
}

/*! \brief Locks ability to issue chip erase command.
 *
 * This function initializes procedure for locking access to chip erase command.
 * It will not wait for the operation to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t
flash_chip_erase_lock( void )
{
    flash_unlock_command();
    flash_issue_command( FLASH_COMMAND_LOCK_CHIP_ERASE );
    return flash_check_status();
}

/*! \brief Reads data from main array.
 *
 * \param address Address to start reading from.
 * \param data Pointer to output data buffer.
 * \param bytes Number of bytes to read, must fit in output data buffer.
 *
 * The read access is done using AHB interface.
 * The address is allowed to be unaligned.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t
flash_read(
    void const * const address,
    uint8_t * const data,
    size_t const bytes
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    if ( NULL == data ) {
        goto failure_arguments;
    }

    /*
     * by setting end byte to last actually read we work around situation
     * where one byte after is the first byte of a new word
     * depending on position (first of new word or not), we would need to
     * either not read a word or do read it, using branch
     * if instead we use last read, we don't have such problem, because
     * we always know we'll read the word including last byte
     */
    uintptr_t const uintptr_start = ( uintptr_t ) address;
    uintptr_t const uintptr_end = uintptr_start + bytes - 1U;

    if (( FLASH_SIZE <= uintptr_start ) && ( FLASH_SIZE <= uintptr_end )) {
        goto failure_arguments;
    }

    result = tsmc40ulpfmc_cache_aware_read( address, data, bytes );

failure_arguments:
    return result;
}

/*! \brief Write data to main array.
 *
 * \param address Address to start writing from.
 * \param data Pointer to output data buffer.
 * \param bytes Number of bytes to write, must fit in output data buffer.
 *
 * The write access is done using AHB interface.
 * The address is allowed to be unaligned.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t
flash_write(
    void * const address,
    uint8_t const * const data,
    size_t const bytes
)
{
    flash_access_status_t result = ARGUMENT_ERROR;
    if ( NULL == data ) {
        goto failure_arguments;
    }

    /*
     * by setting end byte to last actually written we work around situation
     * where one byte after is the first byte of a new word
     * depending on position (first of new word or not), we would need to
     * either not write a word or do write it, using branch
     * if instead we use last write, we don't have such problem, because
     * we always know we'll write the word including last byte
     */
    uintptr_t const uintptr_start = ( uintptr_t ) address;
    uintptr_t const uintptr_end = uintptr_start + bytes - 1U;

    if (( FLASH_SIZE <= uintptr_start ) && ( FLASH_SIZE <= uintptr_end )) {
        goto failure_arguments;
    }

    result = tsmc40ulpfmc_cache_aware_write( address, data, bytes );

    /*
     * Need to flush data cache,
     * as it may not see updated values
     * in AHB address space
     */
    {
        DCACHE_PTR->FLUSH = 1U;
    }

failure_arguments:
    return result;
}

/************** config **************/

/*! \brief Sets the configuration of the Flash controller.
 *
 * \param hclk_freq_hz The frequency of HCLK clock to be configured.
 * \note Read wait state calculated from hclk_freq_hz.
 * \note Sequential prefetch is disabled.
 */
void
flash_configure_default(
    uint32_t const hclk_freq_hz
)
{
    flash_configure_custom(
        hclk_freq_hz,
        flash_CTRL_read_wait_states( hclk_freq_hz ),
        false,
        false
    );
}

/*! \brief Sets the configuration of the Flash controller.
 *
 * \param hclk_freq_hz The frequency of HCLK clock to be configured.
 * \param read_wait_states The number of wait states for read operation.
 * \param sequential_prefetch_enable Enables sequential prefetch feature.
 * \param start_all_reads_as_sequential_enable Enables treating all new reads as start of a sequential burst feature.
 */
void
flash_configure_custom(
    uint32_t const hclk_freq_hz,
    flash_read_wait_states_t const read_wait_states,
    bool const sequential_prefetch,
    bool const all_reads_sequential
)
{
    flash_configure_timings( hclk_freq_hz );
    flash_read_wait_states_set( read_wait_states );
    if ( sequential_prefetch ) {
        flash_sequential_prefetch_enable();
    }
    if ( all_reads_sequential ) {
        flash_all_reads_sequential_enable();
    }

    amba_flash_SMWR_prescaler_t const prescaler =
        flash_CTRL_SMWR_prescaler( hclk_freq_hz );
    flash_SMWR_prescaler_set( prescaler );
}

/*! \brief Helper function for calculating SMWR prescaler based on HCLK frequency.
 *
 * \param hclk_freq_hz The frequency of HCLK clock to be computed on.
 * \retval SMW_PRESCALER for flash CTRL register.
 */
amba_flash_SMWR_prescaler_t
flash_CTRL_SMWR_prescaler( uint32_t const hclk_freq_hz )
{
    /*
     * Choose prescaler so that:
     * HCLK_10_MHz <= hclk_freq_hz / ( 2 ^ prescaler ) <= HCLK_254_MHz
     * where prescaler == 0, 1, 2 or 3
     *
     * We want ( hclk_freq_hz / ( 2 ^ prescaler )) to be as large as possible,
     * so smallest possible prescaler.
     * Note that x / ( 2 ^ y ) == x << y.
     */
    amba_flash_SMWR_prescaler_t result = FLASH_CTRL_SMWR_PRESCALER_1;
    for ( uint8_t i = 0U; i < 4U; ++i ) {
        if ( IS_WITHIN( HCLK_10_MHz, ( hclk_freq_hz >> i ), HCLK_254_MHz )) {
            result = ( amba_flash_SMWR_prescaler_t ) i;
            break;
        }
    }
    return result;
}

/*! \brief Helper function for calculating wait states count based on HCLK frequency.
 *
 * \param hclk_freq_hz The frequency of HCLK clock to be computed on.
 * \param delay_ns_x_10 The required time delay in 1/10ths of a ns (so 12.5 ns should be passed as 125).
 * \retval Wait state.
 */
static uint32_t
flash_calc_clock_wait_states(
    uint32_t const hclk_freq_hz,
    uint32_t const delay_ns_x_10
)
{
    uint64_t clockTickIn01NS = 10000000000ULL / (uint64_t)hclk_freq_hz;
    uint32_t waitStatesCount = (delay_ns_x_10) / clockTickIn01NS;
    if((clockTickIn01NS < delay_ns_x_10) && ((clockTickIn01NS * waitStatesCount) <= delay_ns_x_10))
    {
        ++waitStatesCount;
    }
    return waitStatesCount;
}

/*! Internal type for wait state calculation. */
typedef enum {
    TARGET_2_NS = 0U,
    TARGET_12_5_NS,
    TARGET_20_NS,
    TARGET_100_NS,
    TARGET_1000_NS,
    TARGET_FLASH_DELAY
} timing_calculation_target;

typedef struct {
    uint32_t ns_x_10;
    uint32_t mask;
    uint8_t offset;
} timing_calculation_metadata;

static timing_calculation_metadata const timings____[] = {
    [ TARGET_2_NS ] = {
        .ns_x_10 = 20U,
        .mask = FLASH_TIMING_COUNT_2_NS_MASK,
        .offset = FLASH_TIMING_COUNT_2_NS_OFFSET
    },
    [ TARGET_12_5_NS ] = {
        .ns_x_10 = 125U,
        .mask = FLASH_TIMING_COUNT_12_5_NS_MASK,
        .offset = FLASH_TIMING_COUNT_12_5_NS_OFFSET
    },
    [ TARGET_20_NS ] = {
        .ns_x_10 = 200U,
        .mask = FLASH_TIMING_COUNT_20_NS_MASK,
        .offset = FLASH_TIMING_COUNT_20_NS_OFFSET
    },
    [ TARGET_100_NS ] = {
        .ns_x_10 = 1000U,
        .mask = FLASH_TIMING_COUNT_100_NS_MASK,
        .offset = FLASH_TIMING_COUNT_100_NS_OFFSET
    },
    [ TARGET_1000_NS ] = {
        .ns_x_10 = 10000U,
        .mask = FLASH_TIMING_COUNT_1_US_MASK,
        .offset = FLASH_TIMING_COUNT_1_US_OFFSET
    },
    [ TARGET_FLASH_DELAY ] = {
        .ns_x_10 = 10U * FLASH_DELAY,
        .mask = 0x3F << FLASH_CTRL_READ_WAIT_STATES_OFFSET,
        .offset = FLASH_CTRL_READ_WAIT_STATES_OFFSET
    }
};

static timing_calculation_target const timings_external_to_internal____[] = {
    [ FLASH_TIMING_COUNT_2_NS ] = TARGET_2_NS,
    [ FLASH_TIMING_COUNT_12_5_NS ] = TARGET_12_5_NS,
    [ FLASH_TIMING_COUNT_20_NS ] = TARGET_20_NS,
    [ FLASH_TIMING_COUNT_100_NS ] = TARGET_100_NS,
    [ FLASH_TIMING_COUNT_1_US ] = TARGET_1000_NS
};

static uint32_t
timing_calculation_internal(
    uint32_t const hclk_freq_hz,
    timing_calculation_target const target
)
{
    timing_calculation_metadata const * const metadata =
        &( timings____[ target ] );
    /*
     * For now we shall reuse flash_calc_clock_wait_states().
     * TODO: more precise calculations.
     */
    uint32_t result =
        flash_calc_clock_wait_states( hclk_freq_hz, metadata->ns_x_10 );
    /* must be this order due to mask definitions */
    result <<= metadata->offset;
    result &= metadata->mask;
    return result;
}

/*! \brief Calculate proper read wait states for given clock in Hz.
 * \param hclk_freq_hz The frequency of HCLK clock to be computed on.
 * \retval Properly offset wait state value that can be ORed with CTRL register.
 */
flash_read_wait_states_t
flash_CTRL_read_wait_states(
    uint32_t const hclk_freq_hz
)
{
    uint32_t value =
        timing_calculation_internal( hclk_freq_hz, TARGET_FLASH_DELAY );
    /* Read wait states value isn't offset. */
    value >>= FLASH_CTRL_READ_WAIT_STATES_OFFSET;
    flash_read_wait_states_t const result = {
        .data =
            (
                value
                & (
                    FLASH_CTRL_READ_WAIT_STATES
                    >> FLASH_CTRL_READ_WAIT_STATES_OFFSET
                )
            )
    };
    return result;
}

/*! \brief Calculate TIMING register value for given clock and  target value.
 * \param hclk_freq_hz The frequency of HCLK clock to be computed on.
 * \param target Minimum time the flash should be in a wait state.
 * \retval Properly offset wait state that can be ORed with TIMING register.
~~~~
uint32_t const value =
    flash_tsmc40ulpfmc_TIMING_wait_states(
        hclk_freq_hz,
        FLASH_TIMING_COUNT_12_5_NS
    );
(( amba_flash_t * ) data )->TIMING |= value;
~~~~
 */
uint32_t
flash_TIMING_wait_states(
    uint32_t const hclk_freq_hz,
    flash_TIMING_target_t const target
)
{
    return
        timing_calculation_internal(
            hclk_freq_hz,
            timings_external_to_internal____[ target ]
        );
}

/*! \brief Sets the TIMING configuration register based on the frequency of HCLK clock.
 *
 * \param hclk_freq_hz The frequency of HCLK clock to be configured.
 *
 */
void flash_configure_timings(uint32_t const hclk_freq_hz)
{
    uint32_t const value =
        0
        | flash_TIMING_wait_states( hclk_freq_hz, FLASH_TIMING_COUNT_2_NS )
        | flash_TIMING_wait_states( hclk_freq_hz, FLASH_TIMING_COUNT_12_5_NS )
        | flash_TIMING_wait_states( hclk_freq_hz, FLASH_TIMING_COUNT_20_NS )
        | flash_TIMING_wait_states( hclk_freq_hz, FLASH_TIMING_COUNT_100_NS )
        | flash_TIMING_wait_states( hclk_freq_hz, FLASH_TIMING_COUNT_1_US )
        ;
    AMBA_FLASH_PTR->TIMING = value;
}

/************* utility **************/

/*! \brief Returns programmable sector size (in bytes)
 *
 * \return Size of data sector in bytes.
 */
size_t
flash_sector_size( void )
{
    size_t result = 0U;
    uint32_t info = 0U;
    FLASH_GENERIC_GET(
        INFO,
        info,
        FLASH_INFO_SECTOR_SIZE_MASK,
        FLASH_INFO_SECTOR_SIZE_OFFSET
    );
    switch( info ) {
        case FLASH_INFO_SECTOR_SIZE_1024B:
            result = 1024U;
            break;
        default:
            result = 1024U;
            break;
    }
    return result;
}

/*! \brief Returns erase'able page size (in bytes)
 *
 * \return Size of data page in bytes.
 */
size_t
flash_page_size( void )
{
    size_t result = 0U;
    uint32_t info = 0U;
    FLASH_GENERIC_GET(
        INFO,
        info,
        FLASH_INFO_PAGE_SIZE_MASK,
        FLASH_INFO_PAGE_SIZE_OFFSET
    );
    switch( info ) {
        case FLASH_INFO_PAGE_SIZE_8_SECTORS:
            result = 8U;
            break;
        default:
            result = 8U;
            break;
    }
    result *= flash_sector_size();
    return result;
}

/*! \brief Returns number of paged within single flash memory module.
 *
 * \return Number of data pages within memory module
 */
size_t
flash_module_size( void )
{
    size_t result = 0U;
    uint32_t info = 0U;
    FLASH_GENERIC_GET(
        INFO,
        info,
        FLASH_INFO_MODULE_SIZE_MASK,
        FLASH_INFO_MODULE_SIZE_OFFSET
    );
    switch( info ) {
        case FLASH_INFO_MODULE_SIZE_128_PAGES:
            result = 128U;
            break;
        default:
            result = 128U;
            break;
    }
    result *= flash_page_size();
    return result;
}

/*! \brief Return the number of bytes per program memory region.
 * \note ( flash_region_size() % flash_page_size() ) == 0U
 * \return The number of pages contained in a single region (the smallest entity that can be locked).
 */
size_t
flash_region_size( void )
{
    size_t result = 0U;
#ifdef BOARD_CCNV2_A1
    /* fix for an issue, where REGION size width is too small on CCNV2_A1 to fit the correct value */
    result = 16U;
#else
    result =
        ( size_t )(
            (
                AMBA_FLASH_PTR->INFO
                & FLASH_INFO_REGION_SIZE_MASK
            ) >> FLASH_INFO_REGION_SIZE_OFFSET
        );
#endif
    result *= flash_page_size();
    return result;
}

/*! \brief Return the number of flash memory modules.
 *
 * \return The number of flash memory modules.
 */
uint32_t
flash_modules( void )
{
    uint32_t const result =
        (
            (
                AMBA_FLASH_PTR->INFO
                & FLASH_INFO_MODULES_NUMBER_MASK
            ) >> FLASH_INFO_MODULES_NUMBER_OFFSET
        );
    return result;
}

/*! \brief Loop while module is busy.
 *
 * Function can be used to poll the status of flash controller. Polling will stop, when busy status gets deasserted.
 *
 * \return Status of flash controller.
 *   \retval READY all operations completed. Ready for next command
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR command should have been unlocked first (by writing a password to LOCK register).
 *   \retval ECC_CORR_ERROR ECC correctable error detected during flash data read.
 *   \retval ECC_UNCORR_ERROR ECC uncorrectable error detected during flash data read - data read is invalid.
 */
flash_access_status_t
flash_loop_while_busy( void )
{
    flash_access_status_t result = READY;
    volatile uint32_t status;
    do {
        status = AMBA_FLASH_PTR->STATUS;
    } while( 0U != ( status & FLASH_STATUS_BUSY ));

    if ( 0U != ( status & FLASH_STATUS_PROGRAMMING_ERROR )) {
        result = PROGRAMMING_ERROR;
    }
    else if ( 0U != ( status & FLASH_STATUS_LOCK_ERROR )) {
        result = LOCK_ERROR;
    }
    else if ( 0U != ( status & FLASH_STATUS_UNCORRECTABLE_ECC_ERR )) {
        result = ECC_UNCORR_ERROR;
    }
    else if ( 0U != ( status & FLASH_STATUS_CORRECTABLE_ECC_ERR )) {
        result = ECC_CORR_ERROR;
    }

    return result;
}

/*! \brief Check status of the flash controller.
 *
 * Function can be used to check status of operation performed by the flash controller.
 * The status only applies complex operations (like program/erase) initiated through COMMAND interface.
 *
 * \return Status of flash controller.
 *   \retval READY all operations completed. Ready for next command
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR command should have been unlocked first (by writing a password to LOCK register).
 *   \retval ECC_CORR_ERROR ECC correctable error detected during flash data read.
 *   \retval ECC_UNCORR_ERROR ECC uncorrectable error detected during flash data read - data read is invalid.
 */
flash_access_status_t
flash_check_status( void )
{
    flash_access_status_t result = READY;
    volatile uint32_t status = AMBA_FLASH_PTR->STATUS;

    if ( 0U != ( status & FLASH_STATUS_PROGRAMMING_ERROR )) {
        result = PROGRAMMING_ERROR;
    } else if ( 0U != ( status & FLASH_STATUS_LOCK_ERROR )) {
        result = LOCK_ERROR;
    } else if ( 0U != ( status & FLASH_STATUS_UNCORRECTABLE_ECC_ERR )) {
        result = ECC_UNCORR_ERROR;
    } else if ( 0U != ( status & FLASH_STATUS_CORRECTABLE_ECC_ERR )) {
        result = ECC_CORR_ERROR;
    } else if ( 0U != ( status & FLASH_STATUS_BUSY )) {
        result = BUSY;
    }

    return result;
}

flash_access_status_t
flash_sync( void )
{
    flash_access_status_t result = READY;

    __libc_lock_acquire_recursive( &( tsmc40ulpfmc_cache.lock ));

#if TSMC40ULPFMC_RAM_BUFFERS_DEBUG
    printf( "dump changed page buffers to flash\n" );
#endif /* TSMC40ULPFMC_RAM_BUFFERS_DEBUG */

    for ( size_t i = 0U; i < TSMC40ULPFMC_RAM_BUFFERS; ++i ) {
        tsmc40ulpfmc_cache_page_buffer_type * const buffer =
            &( tsmc40ulpfmc_cache.cache[ i ] );
        if ( ! buffer->used ) {
            continue;
        }
        result = tsmc40ulpfmc_commit_page_blocking( buffer );
        if ( BUSY < result ) {
            goto failure_io;
        }
    }

failure_io:
    __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));

    return result;
}

void
flash_cache_threshold( uint8_t const min, uint8_t const max )
{
    __libc_lock_acquire_recursive( &( tsmc40ulpfmc_cache.lock ));

    tsmc40ulpfmc_cache.threshold.min = min;
    tsmc40ulpfmc_cache.threshold.max = max;

    __libc_lock_release_recursive( &( tsmc40ulpfmc_cache.lock ));
}

