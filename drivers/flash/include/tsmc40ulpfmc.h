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
 * File Name : tsmc40ulpfmc.h
 * Author    : Maciej Plasota
 * ******************************************************************************
 * $Date: 2024-06-04 16:08:03 +0200 (wto, 04 cze 2024) $
 * $Revision: 1058 $
 *H*****************************************************************************/

#ifndef _FLASH_H_
#define _FLASH_H_
#pragma once

#include <ccrv32-amba.h>
#include <ccrv32-amba-flash.h>
#include <ccrv32-csr.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FLASH_GENERIC_ENABLE( REGISTER, MASK ) \
    AMBA_FLASH_PTR->REGISTER |= ( MASK )

#define FLASH_GENERIC_DISABLE( REGISTER, MASK ) \
    AMBA_FLASH_PTR->REGISTER &= ~( MASK )

#define FLASH_GENERIC_STATE( REGISTER, MASK ) \
    ( \
        0U != ( AMBA_FLASH_PTR->REGISTER & ( MASK )) \
    )

#define FLASH_GENERIC_SET( REGISTER, VALUE, MASK, OFFSET ) \
    do { \
        register uint32_t value = AMBA_FLASH_PTR->REGISTER; \
        value &= ~( MASK ); \
        value |= \
            ( \
                ((( uint32_t ) VALUE ) << OFFSET ) \
                & MASK \
            ); \
        AMBA_FLASH_PTR->REGISTER = value; \
    } while( false )

#define FLASH_GENERIC_GET( REGISTER, CONTAINER, MASK, OFFSET ) \
    do { \
        register uint32_t const value = \
            ((( AMBA_FLASH_PTR->REGISTER ) & MASK ) >> OFFSET ); \
        CONTAINER = value & ( MASK >> OFFSET ); \
    } while( false )

typedef struct {
    uint32_t data[ 4U ];
} flash_qword_t;

typedef struct {
    unsigned data : 4U;
} flash_read_margin_t;

typedef struct {
    unsigned data : 6U;
} flash_read_wait_states_t;

/** Flash Timing Register values usable with helper function. */
typedef enum
{
    FLASH_TIMING_COUNT_2_NS,
    FLASH_TIMING_COUNT_12_5_NS,
    FLASH_TIMING_COUNT_20_NS,
    FLASH_TIMING_COUNT_100_NS,
    FLASH_TIMING_COUNT_1_US
} flash_TIMING_target_t;

/*! \brief flash driver status returned by some of the functions.
 */
typedef enum
{
    READY = 0U,
    BUSY = 1U,
    PROGRAMMING_ERROR,
    LOCK_ERROR,
    ARGUMENT_ERROR,
    ECC_CORR_ERROR,
    ECC_UNCORR_ERROR
} flash_access_status_t;

#define CHIPCRAFT_SDK_FLASH_ACCESS_STATUS_T_DEFINED 2

/*************** I/O ****************/

/*! \brief Clears (set to 0xFF) the content of Sector Buffer.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t
flash_sector_buffer_clear( void );

/*! \brief Writes quad-word of data into the sector buffer.
 *
 * \param offset Sector buffer offset (needs to be aligned to 128-bit qword).
 * \param data The data quad-word to be written into sector buffer.
 * \warning Given offset shall be adjusted modulo sector buffer size.
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
);

/*! \brief Writes single word of data into the sector buffer.
 *
 * \param offset Sector buffer offset (needs to be aligned to 32-bit word).
 * \param data The data word to be written into sector buffer.
 * \warning Given offset shall be adjusted modulo sector buffer size.
 *
 * This function uses direct access to the Sector Buffer, meaning it will
 * block until the write actually completed.
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
);

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
);

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
);

/*! \brief Unlocks ability to write data into flash.
 */
void flash_unlock( void );

/*! \brief Writes contents of the sector buffer into program memory.
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
 */
flash_access_status_t
flash_sector_buffer_write(
    void * const address
);

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
);

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
);

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
);

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
flash_erase_all( void );

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
flash_erase_reference_cell( void );

/*! \brief Locks region of program memory for write and erase.
 *
 * \param address Address within the memory region to be locked.
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
);

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
flash_lock_debugger_read( void );

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
flash_lock_debugger_access( void );

/*! \brief Checks if region containing provided address is locked for write/erase access.
 *
 * \param address Address within the region to check for.
 *
 * \return State of the region to check for.
 *   \retval true region is locked for write/erase
 *   \retval false region is open for write/erase
 */
bool
flash_is_region_locked(
    void const * const address
);

/*! \brief Check if read flash memory through debugger is enabled.
 *
 * \return State of debugger read access
 *  \retval true debugger read is disabled
 *  \retval false debugger read is enabled
 */
static inline bool
flash_is_debugger_read_locked( void )
{
    return FLASH_GENERIC_STATE(
        MASTER_LOCKS,
        FLASH_MASTER_LOCKS_DEBUGGER_READ_DISABLED
    );
}

/*! \brief Check if access through debugger is enabled.
 *
 * \return State of debugger access to the processor
 *  \retval true debugger access is disabled
 *  \retval false debugger access is enabled
 */
static inline bool
flash_is_debugger_access_locked( void )
{
    return FLASH_GENERIC_STATE(
        MASTER_LOCKS,
        FLASH_MASTER_LOCKS_DEBUGGER_ACCESS_DISABLED
    );
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
);

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
);

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
);

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
flash_factory_row_erase( void );

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
flash_factory_row_lock( void );

/*! \brief Check if read/write/erase access to factory row is allowed.
 *
 * \return State of factory row access
 *  \retval true factory row is locked
 *  \retval false factory row is accessible
 */
static inline bool
flash_factory_row_is_locked( void )
{
    return FLASH_GENERIC_STATE(
        MASTER_LOCKS,
        FLASH_MASTER_LOCKS_FACTORY_ROW_LOCKED
    );
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
);

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
);

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
);

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
flash_user_row_erase( void );

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
flash_user_row_lock( void );

/*! \brief Check if read/write/erase access to user row is allowed.
 *
 * \return State of user row access
 *  \retval true user row is locked
 *  \retval false user row is accessible
 */
static inline bool
flash_user_row_is_locked( void )
{
    return FLASH_GENERIC_STATE(
        MASTER_LOCKS,
        FLASH_MASTER_LOCKS_USER_ROW_LOCKED
    );
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
);

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
);

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
);

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
flash_manufacturer_row_erase( void );

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
flash_manufacturer_row_lock( void );

/*! \brief Check if read/write/erase access to manufacturer row is allowed.
 *
 * \return State of manufacturer row access
 *  \retval true manufacturer row is locked
 *  \retval false manufacturer row is accessible
 */
static inline bool
flash_manufacturer_row_is_locked( void )
{
    return FLASH_GENERIC_STATE(
        MASTER_LOCKS,
        FLASH_MASTER_LOCKS_MANUFACTURER_ROW_LOCKED
    );
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
flash_chip_erase( void );

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
flash_chip_erase_lock( void );

/*! \brief Check if chip erase command is available.
 *
 * \return Status of chip erase command
 *  \retval true access to chip erase command is locked
 *  \retval false chip erase command is allowed
 */
static inline bool
flash_chip_erase_is_locked( void )
{
    return FLASH_GENERIC_STATE(
        MASTER_LOCKS,
        FLASH_MASTER_LOCKS_CHIP_ERASE_LOCKED
    );
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
);

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
);

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
);

/*! \brief Sets the configuration of the Flash controller.
 *
 * \param hclk_freq_hz The frequency of HCLK clock to be configured.
 * \param read_wait_states The number of wait states for read operation.
 * \param sequential_prefetch Sets sequential prefetch feature.
 * \param all_reads_sequential All new reads start a burst sequence.
 */
void
flash_configure_custom(
    uint32_t const hclk_freq_hz,
    flash_read_wait_states_t const read_wait_states,
    bool const sequential_prefetch,
    bool const all_reads_sequential
);

/*! \brief Helper function for calculating SMWR prescaler based on HCLK frequency.
 *
 * \param hclk_freq_hz The frequency of HCLK clock to be computed on.
 * \retval SMW_PRESCALER for flash CTRL register.
 */
amba_flash_SMWR_prescaler_t
flash_CTRL_SMWR_prescaler(
    uint32_t const hclk_freq_hz
);

/*! \brief Configure SMWR prescaler.
 *
 * \param prescaler Enum for the prescaler value to be set.
 *
 * This function configures the prescaler for clock supplied to Smart Write Memory module.
 * Note that the clock frequency for this module can't exceed 254 MHz.
 */
static inline void
flash_SMWR_prescaler_set(
    amba_flash_SMWR_prescaler_t const prescaler
)
{
    FLASH_GENERIC_SET(
        CTRL,
        prescaler,
        FLASH_CTRL_SMWR_PRESCALER,
        FLASH_CTRL_SMWR_PRESCALER_OFFSET
    );
}

/*! \brief Read SMWR prescaler configuration.
 *
 * This function reads back the prescaler for clock supplied to Smart Write Memory module.
 *
 * \return Enum of the prescaler value set.
 */
static inline amba_flash_SMWR_prescaler_t
flash_SMWR_prescaler_get( void )
{
    uint32_t result = 0U;
    FLASH_GENERIC_GET(
        CTRL,
        result,
        FLASH_CTRL_SMWR_PRESCALER,
        FLASH_CTRL_SMWR_PRESCALER_OFFSET
    );
    return ( amba_flash_SMWR_prescaler_t ) result;
}

/*! \brief Calculate proper read wait states for given clock in Hz.
 * \param hclk_freq_hz The frequency of HCLK clock to be computed on.
 * \retval Wait state value.
 */
flash_read_wait_states_t
flash_CTRL_read_wait_states(
    uint32_t const hclk_freq_hz
);

/*! \brief Configure READ wait states count.
 *
 * \param read_wait_states_count The number of READ wait states to be set.
 *
 * This function configures the number of wait states to be used when reading
 * flash memory content.
 */
static inline void
flash_read_wait_states_set(
    flash_read_wait_states_t const read_wait_states
)
{
    FLASH_GENERIC_SET(
        CTRL,
        read_wait_states.data,
        FLASH_CTRL_READ_WAIT_STATES,
        FLASH_CTRL_READ_WAIT_STATES_OFFSET
    );
}

/*! \brief Read READ wait states count.
 *
 * This function reads back the number of wait states to be used when reading flash memory content.
 *
 * \return Number of wait states during READ operation.
 */
static inline flash_read_wait_states_t
flash_read_wait_states_get( void )
{
    flash_read_wait_states_t result = { 0U };
    FLASH_GENERIC_GET(
        CTRL,
        result.data,
        FLASH_CTRL_READ_WAIT_STATES,
        FLASH_CTRL_READ_WAIT_STATES_OFFSET
    );
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
(( amba_flash_t * ) data )->TIMING &= ~( FLASH_TIMING_COUNT_12_5_NS_MASK );
(( amba_flash_t * ) data )->TIMING |= value;
~~~~
 */
uint32_t
flash_TIMING_wait_states(
    uint32_t const hclk_freq_hz,
    flash_TIMING_target_t const target
);

/*! \brief Sets the TIMING configuration register based on the frequency of HCLK clock.
 *
 * \param hclk_freq_hz The frequency of HCLK clock to be configured.
 */
void
flash_configure_timings(
    uint32_t const hclk_freq_hz
);

/*! \brief Enable sequential prefetch.
 */
static inline void
flash_sequential_prefetch_enable( void )
{
    FLASH_GENERIC_ENABLE( CTRL, FLASH_CTRL_SEQUENTIAL_PREFETCH );
}

/*! \brief Disable sequential prefetch.
 */
static inline void
flash_sequential_prefetch_disable( void )
{
    FLASH_GENERIC_DISABLE( CTRL, FLASH_CTRL_SEQUENTIAL_PREFETCH );
}

/*! \brief Check if sequential prefetch is enabled.
 *
 * \return State of sequential prefetch configuration
 *  \retval true sequential prefetch is enabled
 *  \retval false sequential prefetch is disabled
 */
static inline bool
flash_sequential_prefetch_state( void )
{
    return FLASH_GENERIC_STATE( CTRL, FLASH_CTRL_SEQUENTIAL_PREFETCH );
}

/*! \brief Enable treating all new reads as start of a sequential burst.
 */
static inline void
flash_all_reads_sequential_enable( void )
{
    FLASH_GENERIC_ENABLE( CTRL, FLASH_CTRL_ASSUME_READS_SEQUENTIAL );
}

/*! \brief Disable treating all new reads as start of a sequential burst.
 */
static inline void
flash_all_reads_sequential_disable( void )
{
    FLASH_GENERIC_DISABLE( CTRL, FLASH_CTRL_ASSUME_READS_SEQUENTIAL );
}

/*! \brief Check if treating all new reads as start of a sequential burst is enabled.
 *
 * \return State of treating all new reads as start of a sequential burst configuration
 *  \retval true treating all new reads as start of a sequential burst is enabled
 *  \retval false treating all new reads as start of a sequential burst is disabled
 */
static inline bool
flash_all_reads_sequential_state( void )
{
    return FLASH_GENERIC_STATE( CTRL, FLASH_CTRL_ASSUME_READS_SEQUENTIAL );
}

/*! \brief Enable reprogram zeros on write functionality.
 */
static inline void
flash_reprogram_zeros_on_write_enable( void )
{
    FLASH_GENERIC_ENABLE( CTRL, FLASH_CTRL_REPROGRAM_ZEROS_ON_WRITE );
}

/*! \brief Disable reprogram zeros on write functionality.
 */
static inline void
flash_reprogram_zeros_on_write_disable( void )
{
    FLASH_GENERIC_DISABLE( CTRL, FLASH_CTRL_REPROGRAM_ZEROS_ON_WRITE );
}

/*! \brief Check if reprogram zeros on write functionality is enabled.
 *
 * \return State of reprogram zeros on write functionality configuration
 *  \retval true reprogram zeros on write functionality is enabled
 *  \retval false reprogram zeros on write functionality is disabled
 */
static inline bool
flash_reprogram_zeros_on_write_state( void )
{
    return FLASH_GENERIC_STATE( CTRL, FLASH_CTRL_REPROGRAM_ZEROS_ON_WRITE );
}

/*! \brief Enable memory high endurance mode.
 */
static inline void
flash_high_endurance_mode_enable( void )
{
    FLASH_GENERIC_ENABLE( CTRL, FLASH_CTRL_HIGH_ENDURANCE_MODE );
}

/*! \brief Disable memory high endurance mode.
 */
static inline void
flash_high_endurance_mode_disable( void )
{
    FLASH_GENERIC_DISABLE( CTRL, FLASH_CTRL_HIGH_ENDURANCE_MODE );
}

/*! \brief Check if memory high endurance mode is enabled.
 *
 * \return State of memory high endurance mode configuration
 *  \retval true memory high endurance mode is enabled
 *  \retval false memory high endurance mode is disabled
 */
static inline bool
flash_high_endurance_mode_state( void )
{
    return FLASH_GENERIC_STATE( CTRL, FLASH_CTRL_HIGH_ENDURANCE_MODE );
}

/*! \brief Enable memory low voltage mode.
 */
static inline void
flash_low_voltage_mode_enable( void )
{
    FLASH_GENERIC_ENABLE( CTRL, FLASH_CTRL_LOW_VOLTAGE_MODE );
}

/*! \brief Disable memory low voltage mode.
 */
static inline void
flash_low_voltage_mode_disable( void )
{
    FLASH_GENERIC_DISABLE( CTRL, FLASH_CTRL_LOW_VOLTAGE_MODE );
}

/*! \brief Check if memory low voltage mode is enabled.
 *
 * \return State of memory low voltage mode configuration
 *  \retval true memory low voltage mode is enabled
 *  \retval false memory low voltage mode is disabled
 */
static inline bool
flash_low_voltage_mode_state( void )
{
    return FLASH_GENERIC_STATE( CTRL, FLASH_CTRL_LOW_VOLTAGE_MODE );
}

/*! \brief Enable ECC verification on read.
 */
static inline void
flash_ECC_enable( void )
{
    FLASH_GENERIC_ENABLE( CTRL, FLASH_CTRL_ECC_ENABLE );
}

/*! \brief Disable ECC verification on read.
 */
static inline void
flash_ECC_disable( void )
{
    FLASH_GENERIC_DISABLE( CTRL, FLASH_CTRL_ECC_ENABLE );
}

/*! \brief Check if ECC verification on read is enabled.
 *
 * \return State of ECC verification on read configuration
 *  \retval true ECC verification on read is enabled
 *  \retval false ECC verification on read is disabled
 */
static inline bool
flash_ECC_state( void )
{
    return FLASH_GENERIC_STATE( CTRL, FLASH_CTRL_ECC_ENABLE );
}

/*! \brief Enable ECC reporting on AHB during data read.
 */
static inline void
flash_AHB_ECC_enable( void )
{
    FLASH_GENERIC_ENABLE( CTRL, FLASH_CTRL_REPORT_ECC_ON_AHB_ENABLE );
}

/*! \brief Disable ECC reporting on AHB during data read.
 */
static inline void
flash_AHB_ECC_disable( void )
{
    FLASH_GENERIC_DISABLE( CTRL, FLASH_CTRL_REPORT_ECC_ON_AHB_ENABLE );
}

/*! \brief Check if ECC reporting on AHB during data read is enabled.
 *
 * \return State of ECC reporting on AHB during data read configuration
 *  \retval true ECC reporting on AHB during data read is enabled
 *  \retval false ECC reporting on AHB during data read is disabled
 */
static inline bool
flash_AHB_ECC_state( void )
{
    return FLASH_GENERIC_STATE( CTRL, FLASH_CTRL_REPORT_ECC_ON_AHB_ENABLE );
}

/*! \brief Enable Sector Buffer self time bypass.
 */
static inline void
flash_self_time_bypass_enable( void )
{
    FLASH_GENERIC_ENABLE( CTRL, FLASH_CTRL_SELF_TIME_BYPASS_ENABLE );
}

/*! \brief Disable Sector Buffer self time bypass.
 */
static inline void
flash_self_time_bypass_disable( void )
{
    FLASH_GENERIC_DISABLE( CTRL, FLASH_CTRL_SELF_TIME_BYPASS_ENABLE );
}

/*! \brief Check if Sector Buffer self time bypass is enabled.
 *
 * \return State of Sector Buffer self time bypass configuration
 *  \retval true Sector Buffer self time bypass is enabled
 *  \retval false Sector Buffer self time bypass is disabled
 */
static inline bool
flash_self_time_bypass_state( void )
{
    return FLASH_GENERIC_STATE( CTRL, FLASH_CTRL_SELF_TIME_BYPASS_ENABLE );
}

/*! \brief Reset Sector Buffer READ Margin.
 */
static inline void
flash_read_margin_clear( void )
{
    AMBA_FLASH_PTR->CTRL &= ~FLASH_CTRL_READ_MARGIN_ENABLE;
}

/*! \brief Configure Sector Buffer READ Margin.
 *
 * \param read_margin The value of read margin to be set.
 *
 */
static inline void
flash_read_margin_set(
    flash_read_margin_t const read_margin
)
{
    register uint32_t value = AMBA_FLASH_PTR->CTRL;
    value &= ~FLASH_CTRL_READ_MARGIN;
    value |=
        (
            (
                (( uint32_t ) read_margin.data )
                << FLASH_CTRL_READ_MARGIN_OFFSET
            )
            & FLASH_CTRL_READ_MARGIN
        );
    value |= FLASH_CTRL_READ_MARGIN_ENABLE;
    AMBA_FLASH_PTR->CTRL = value;
}

/*! \brief Read Sector Buffer READ Margin.
 *
 * This function reads back the Sector Buffer READ Margin.
 *
 * \return Sector Buffer READ Margin.
 */
static inline flash_read_margin_t
flash_read_margin_get( void )
{
    flash_read_margin_t result = { 0U };
    FLASH_GENERIC_GET(
        CTRL,
        result.data,
        FLASH_CTRL_READ_MARGIN,
        FLASH_CTRL_READ_MARGIN_OFFSET
    );
    return result;
}

/*! \brief Configure MV in SMWR OPTION 0.
 *
 * \param value The value of MV to be set.
 *
 */
static inline void
flash_MV_set(
    uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_0,
        data,
        FLASH_SMWR_OPTION_0_MV_MASK,
        FLASH_SMWR_OPTION_0_MV_OFFSET
    );
}

/*! \brief Read MV configuration from SMWR OPTION 0.
 *
 * This function reads back the MV configuration from SMWR OPTION 0 register.
 *
 * \return MV value set.
 */
static inline uint8_t
flash_MV_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_0,
        result,
        FLASH_SMWR_OPTION_0_MV_MASK,
        FLASH_SMWR_OPTION_0_MV_OFFSET
    );
    return result;
}

/*! \brief Configure MV FINAL in SMWR OPTION 0.
 *
 * \param value The value of MV FINAL to be set.
 *
 */
static inline void
flash_MV_FINAL_set(
    uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_0,
        data,
        FLASH_SMWR_OPTION_0_MV_FINAL_MASK,
        FLASH_SMWR_OPTION_0_MV_FINAL_OFFSET
    );
}

/*! \brief Read MV FINAL configuration from SMWR OPTION 0.
 *
 * This function reads back the MV FINAL configuration from SMWR OPTION 0 register.
 *
 * \return MV FINAL value set.
 */
static inline uint8_t
flash_MV_FINAL_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_0,
        result,
        FLASH_SMWR_OPTION_0_MV_FINAL_MASK,
        FLASH_SMWR_OPTION_0_MV_FINAL_OFFSET
    );
    return result;
}

/*! \brief Configure WIPGM in SMWR OPTION 0.
 *
 * \param value The value of WIPGM to be set.
 *
 */
static inline void
flash_WIPGM_set(
    uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_0,
        data,
        FLASH_SMWR_OPTION_0_WIPGM_MASK,
        FLASH_SMWR_OPTION_0_WIPGM_OFFSET
    );
}

/*! \brief Read WIPGM configuration from SMWR OPTION 0.
 *
 * This function reads back the WIPGM configuration from SMWR OPTION 0 register.
 *
 * \return WIPGM value set.
 */
static inline uint8_t
flash_WIPGM_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_0,
        result,
        FLASH_SMWR_OPTION_0_WIPGM_MASK,
        FLASH_SMWR_OPTION_0_WIPGM_OFFSET
    );
    return result;
}

/*! \brief Configure WIPGM FINAL in SMWR OPTION 0.
 *
 * \param value The value of WIPGM FINAL to be set.
 *
 */
static inline void
flash_WIPGM_FINAL_set(
    uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_0,
        data,
        FLASH_SMWR_OPTION_0_WIPGM_FINAL_MASK,
        FLASH_SMWR_OPTION_0_WIPGM_FINAL_OFFSET
    );
}

/*! \brief Read WIPGM FINAL configuration from SMWR OPTION 0.
 *
 * This function reads back the WIPGM FINAL configuration from SMWR OPTION 0 register.
 *
 * \return WIPGM FINAL value set.
 */
static inline uint8_t
flash_WIPGM_FINAL_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_0,
        result,
        FLASH_SMWR_OPTION_0_WIPGM_FINAL_MASK,
        FLASH_SMWR_OPTION_0_WIPGM_FINAL_OFFSET
    );
    return result;
}

/*! \brief Configure TERS in SMWR OPTION 1.
 *
 * \param value The value of TERS to be set.
 *
 */
static inline void
flash_TERS_set(
	uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_1,
        data,
        FLASH_SMWR_OPTION_1_TERS_MASK,
        FLASH_SMWR_OPTION_1_TERS_OFFSET
    );
}

/*! \brief Read TERS configuration from SMWR OPTION 1.
 *
 * This function reads back the TERS configuration from SMWR OPTION 1 register.
 *
 * \return TERS value set.
 */
static inline uint8_t
flash_TERS_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_1,
        result,
        FLASH_SMWR_OPTION_1_TERS_MASK,
        FLASH_SMWR_OPTION_1_TERS_OFFSET
    );
    return result;
}

/*! \brief Configure TPGM in SMWR OPTION 1.
 *
 * \param value The value of TPGM to be set.
 *
 */
static inline void
flash_TPGM_set(
	uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_1,
        data,
        FLASH_SMWR_OPTION_1_TPGM_MASK,
        FLASH_SMWR_OPTION_1_TPGM_OFFSET
    );
}

/*! \brief Read TPGM configuration from SMWR OPTION 1.
 *
 * This function reads back the TPGM configuration from SMWR OPTION 1 register.
 *
 * \return TPGM value set.
 */
static inline uint8_t
flash_TPGM_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_1,
        result,
        FLASH_SMWR_OPTION_1_TPGM_MASK,
        FLASH_SMWR_OPTION_1_TPGM_OFFSET
    );
    return result;
}

/*! \brief Configure TNVS in SMWR OPTION 1.
 *
 * \param value The value of TNVS to be set.
 *
 */
static inline void
flash_TNVS_set(
	uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_1,
        data,
        FLASH_SMWR_OPTION_1_TNVS_MASK,
        FLASH_SMWR_OPTION_1_TNVS_OFFSET
    );
}

/*! \brief Read TNVS configuration from SMWR OPTION 1.
 *
 * This function reads back the TNVS configuration from SMWR OPTION 1 register.
 *
 * \return TNVS value set.
 */
static inline uint8_t
flash_TNVS_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_1,
        result,
        FLASH_SMWR_OPTION_1_TNVS_MASK,
        FLASH_SMWR_OPTION_1_TNVS_OFFSET
    );
    return result;
}

/*! \brief Configure TNVH in SMWR OPTION 1.
 *
 * \param value The value of TNVH to be set.
 *
 */
static inline void
flash_TNVH_set(
	uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_1,
        data,
        FLASH_SMWR_OPTION_1_TNVH_MASK,
        FLASH_SMWR_OPTION_1_TNVH_OFFSET
    );
}

/*! \brief Read TNVH configuration from SMWR OPTION 1.
 *
 * This function reads back the TNVH configuration from SMWR OPTION 1 register.
 *
 * \return TNVH value set.
 */
static inline uint8_t
flash_TNVH_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_1,
        result,
        FLASH_SMWR_OPTION_1_TNVH_MASK,
        FLASH_SMWR_OPTION_1_TNVH_OFFSET
    );
    return result;
}

/*! \brief Configure TPGS in SMWR OPTION 1.
 *
 * \param value The value of TPGS to be set.
 *
 */
static inline void
flash_TPGS_set(
	uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_1,
        data,
        FLASH_SMWR_OPTION_1_TPGS_MASK,
        FLASH_SMWR_OPTION_1_TNVH_OFFSET
    );
}

/*! \brief Read TPGS configuration from SMWR OPTION 1.
 *
 * This function reads back the TPGS configuration from SMWR OPTION 1 register.
 *
 * \return TPGS value set.
 */
static inline uint8_t
flash_TPGS_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_1,
        result,
        FLASH_SMWR_OPTION_1_TPGS_MASK,
        FLASH_SMWR_OPTION_1_TPGS_OFFSET
    );
    return result;
}

/*! \brief Configure MAX ERASE in SMWR OPTION 1.
 *
 * \param value The value of MAX ERASE to be set.
 *
 */
static inline void
flash_MAX_ERASE_set(
	uint16_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_1,
        data,
        FLASH_SMWR_OPTION_1_MAX_ERASE_MASK,
        FLASH_SMWR_OPTION_1_MAX_ERASE_OFFSET
    );
}

/*! \brief Read MAX ERASE configuration from SMWR OPTION 1.
 *
 * This function reads back the MAX ERASE configuration from SMWR OPTION 1 register.
 *
 * \return MAX ERASE value set.
 */
static inline uint16_t
flash_MAX_ERASE_get( void )
{
    uint16_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_1,
        result,
        FLASH_SMWR_OPTION_1_MAX_ERASE_MASK,
        FLASH_SMWR_OPTION_1_MAX_ERASE_OFFSET
    );
    return result;
}

/*! \brief Configure MAX PROG in SMWR OPTION 1.
 *
 * \param value The value of MAX PROG to be set.
 *
 */
static inline void
flash_MAX_PROG_set(
	uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_1,
        data,
        FLASH_SMWR_OPTION_1_MAX_PROG_MASK,
        FLASH_SMWR_OPTION_1_MAX_PROG_OFFSET
    );
}

/*! \brief Read MAX PROG configuration from SMWR OPTION 1.
 *
 * This function reads back the MAX PROG configuration from SMWR OPTION 1 register.
 *
 * \return MAX PROG value set.
 */
static inline uint8_t
flash_MAX_PROG_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_1,
        result,
        FLASH_SMWR_OPTION_1_MAX_PROG_MASK,
        FLASH_SMWR_OPTION_1_MAX_PROG_OFFSET
    );
    return result;
}

/*! \brief Configure THVS in SMWR OPTION 2.
 *
 * \param value The value of THVS to be set.
 *
 */
static inline void
flash_THVS_set(
	uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_2,
        data,
        FLASH_SMWR_OPTION_2_THVS_MASK,
        FLASH_SMWR_OPTION_2_THVS_OFFSET
    );
}

/*! \brief Read THVS configuration from SMWR OPTION 2.
 *
 * This function reads back the THVS configuration from SMWR OPTION 2 register.
 *
 * \return THVS value set.
 */
static inline uint8_t
flash_THVS_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_2,
        result,
        FLASH_SMWR_OPTION_2_THVS_MASK,
        FLASH_SMWR_OPTION_2_THVS_OFFSET
    );
    return result;
}

/*! \brief Configure TRCV in SMWR OPTION 2.
 *
 * \param value The value of TRCV to be set.
 *
 */
static inline void
flash_TRCV_set(
	uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_2,
        data,
        FLASH_SMWR_OPTION_2_TRCV_MASK,
        FLASH_SMWR_OPTION_2_TRCV_OFFSET
    );
}

/*! \brief Read TRCV configuration from SMWR OPTION 2.
 *
 * This function reads back the TRCV configuration from SMWR OPTION 2 register.
 *
 * \return TRCV value set.
 */
static inline uint8_t
flash_TRCV_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_2,
        result,
        FLASH_SMWR_OPTION_2_TRCV_MASK,
        FLASH_SMWR_OPTION_2_TRCV_OFFSET
    );
    return result;
}

/*! \brief Configure EPP in SMWR OPTION 2.
 *
 * \param value The value of EPP to be set.
 *
 */
static inline void
flash_EPP_set(
	uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_2,
        data,
        FLASH_SMWR_OPTION_2_EPP_MASK,
        FLASH_SMWR_OPTION_2_EPP_OFFSET
    );
}

/*! \brief Read EPP configuration from SMWR OPTION 2.
 *
 * This function reads back the EPP configuration from SMWR OPTION 2 register.
 *
 * \return EPP value set.
 */
static inline uint8_t
flash_EPP_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_2,
        result,
        FLASH_SMWR_OPTION_2_EPP_MASK,
        FLASH_SMWR_OPTION_2_EPP_OFFSET
    );
    return result;
}

/*! \brief Configure EPE in SMWR OPTION 2.
 *
 * \param value The value of EPE to be set.
 *
 */
static inline void
flash_EPE_set(
	uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_2,
        data,
        FLASH_SMWR_OPTION_2_EPE_MASK,
        FLASH_SMWR_OPTION_2_EPE_OFFSET
    );
}

/*! \brief Read EPE configuration from SMWR OPTION 2.
 *
 * This function reads back the EPE configuration from SMWR OPTION 2 register.
 *
 * \return EPE value set.
 */
static inline uint8_t
flash_EPE_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_2,
        result,
        FLASH_SMWR_OPTION_2_EPE_MASK,
        FLASH_SMWR_OPTION_2_EPE_OFFSET
    );
    return result;
}

/*! \brief Configure WHV in SMWR OPTION 2.
 *
 * \param value The value of WHV to be set.
 *
 */
static inline void
flash_WHV_set(
	uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_2,
        data,
        FLASH_SMWR_OPTION_2_WHV_MASK,
        FLASH_SMWR_OPTION_2_WHV_OFFSET
    );
}

/*! \brief Read WHV configuration from SMWR OPTION 2.
 *
 * This function reads back the WHV configuration from SMWR OPTION 2 register.
 *
 * \return WHV value set.
 */
static inline uint8_t
flash_WHV_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_2,
        result,
        FLASH_SMWR_OPTION_2_WHV_MASK,
        FLASH_SMWR_OPTION_2_WHV_OFFSET
    );
    return result;
}

/*! \brief Configure POST TERS in SMWR OPTION 2.
 *
 * \param value The value of POST TERS to be set.
 *
 */
static inline void
flash_POST_TERS_set(
	uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_2,
        data,
        FLASH_SMWR_OPTION_2_POST_TERS_MASK,
        FLASH_SMWR_OPTION_2_POST_TERS_OFFSET
    );
}

/*! \brief Read POST TERS configuration from SMWR OPTION 2.
 *
 * This function reads back the POST TERS configuration from SMWR OPTION 2 register.
 *
 * \return POST TERS value set.
 */
static inline uint8_t
flash_POST_TERS_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_2,
        result,
        FLASH_SMWR_OPTION_2_POST_TERS_MASK,
        FLASH_SMWR_OPTION_2_POST_TERS_OFFSET
    );
    return result;
}

/*! \brief Configure POST TPGM in SMWR OPTION 2.
 *
 * \param value The value of POST TPGM to be set.
 *
 */
static inline void
flash_POST_TPGM_set(
	uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_2,
        data,
        FLASH_SMWR_OPTION_2_POST_TPGM_MASK,
        FLASH_SMWR_OPTION_2_POST_TPGM_OFFSET
    );
}

/*! \brief Read POST TPGM configuration from SMWR OPTION 2.
 *
 * This function reads back the POST TPGM configuration from SMWR OPTION 2 register.
 *
 * \return POST TPGM value set.
 */
static inline uint8_t
flash_POST_TPGM_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_2,
        result,
        FLASH_SMWR_OPTION_2_POST_TPGM_MASK,
        FLASH_SMWR_OPTION_2_POST_TPGM_OFFSET
    );
    return result;
}

/*! \brief Configure VERIFY in SMWR OPTION 2.
 *
 * \param value The value of VERIFY to be set.
 *
 */
static inline void
flash_VERIFY_set(
	uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_2,
        data,
        FLASH_SMWR_OPTION_2_VERIFY_MASK,
        FLASH_SMWR_OPTION_2_VERIFY_OFFSET
    );
}

/*! \brief Read VERIFY configuration from SMWR OPTION 2.
 *
 * This function reads back the VERIFY configuration from SMWR OPTION 2 register.
 *
 * \return VERIFY value set.
 */
static inline uint8_t
flash_VERIFY_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_2,
        result,
        FLASH_SMWR_OPTION_2_VERIFY_MASK,
        FLASH_SMWR_OPTION_2_VERIFY_OFFSET
    );
    return result;
}

/*! \brief Configure TPGM OPTION in SMWR OPTION 2.
 *
 * \param value The value of TPGM OPTION to be set.
 *
 */
static inline void
flash_TPGM_OPTION_set(
	uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_2,
        data,
        FLASH_SMWR_OPTION_2_TPGM_OPTION_MASK,
        FLASH_SMWR_OPTION_2_TPGM_OPTION_OFFSET
    );
}

/*! \brief Read TPGM OPTION configuration from SMWR OPTION 2.
 *
 * This function reads back the TPGM OPTION configuration from SMWR OPTION 2 register.
 *
 * \return TPGM OPTION value set.
 */
static inline uint8_t
flash_TPGM_OPTION_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_2,
        result,
        FLASH_SMWR_OPTION_2_TPGM_OPTION_MASK,
        FLASH_SMWR_OPTION_2_TPGM_OPTION_OFFSET
    );
    return result;
}

/*! \brief Configure MASK0 in SMWR OPTION 2.
 *
 * \param value The value of MASK0 to be set.
 *
 */
static inline void
flash_MASK0_set(
	uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_2,
        data,
        FLASH_SMWR_OPTION_2_MASK0_MASK,
        FLASH_SMWR_OPTION_2_MASK0_OFFSET
    );
}

/*! \brief Read MASK0 configuration from SMWR OPTION 2.
 *
 * This function reads back the MASK0 configuration from SMWR OPTION 2 register.
 *
 * \return MASK0 value set.
 */
static inline uint8_t
flash_MASK0_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_2,
        result,
        FLASH_SMWR_OPTION_2_MASK0_MASK,
        FLASH_SMWR_OPTION_2_MASK0_OFFSET
    );
    return result;
}

/*! \brief Configure DISABLE PRE READ in SMWR OPTION 2.
 *
 * \param value The value of DISABLE PRE READ to be set.
 *
 */
static inline void
flash_DISABLE_PRE_READ_set(
	uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_2,
        data,
        FLASH_SMWR_OPTION_2_DISABLE_PRE_READ_MASK,
        FLASH_SMWR_OPTION_2_DISABLE_PRE_READ_OFFSET
    );
}

/*! \brief Read DISABLE PRE READ configuration from SMWR OPTION 2.
 *
 * This function reads back the DISABLE PRE READ configuration from SMWR OPTION 2 register.
 *
 * \return DISABLE PRE READ value set.
 */
static inline uint8_t
flash_DISABLE_PRE_READ_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_2,
        result,
        FLASH_SMWR_OPTION_2_DISABLE_PRE_READ_MASK,
        FLASH_SMWR_OPTION_2_DISABLE_PRE_READ_OFFSET
    );
    return result;
}

/*! \brief Configure HEM WHV COUNTER in SMWR OPTION 3.
*
* \param value The value of HEM WHV COUNTER to be set.
*
*/
static inline void
flash_HEM_WHV_COUNTER_set(
	uint8_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_3,
        data,
        FLASH_SMWR_OPTION_3_HEM_WHV_COUNTER_MASK,
        FLASH_SMWR_OPTION_3_HEM_WHV_COUNTER_OFFSET
    );
}

/*! \brief Read HEM WHV COUNTER configuration from SMWR OPTION 3.
*
* This function reads back the HEM WHV COUNTER configuration from SMWR OPTION 3 register.
*
* \return WHV COUNTER value set.
*/
static inline uint8_t
flash_HEM_WHV_COUNTER_get( void )
{
    uint8_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_3,
        result,
        FLASH_SMWR_OPTION_3_HEM_WHV_COUNTER_MASK,
        FLASH_SMWR_OPTION_3_HEM_WHV_COUNTER_OFFSET
    );
    return result;
}

/*! \brief Configure HEM MAX ERASE in SMWR OPTION 3.
*
* \param value The value of HEM MAX ERASE to be set.
*
*/
static inline void
flash_HEM_MAX_ERASE_set(
	uint16_t const data
)
{
    FLASH_GENERIC_SET(
        SMWR_OPTION_3,
        data,
        FLASH_SMWR_OPTION_3_HEM_MAX_ERASE_MASK,
        FLASH_SMWR_OPTION_3_HEM_MAX_ERASE_OFFSET
    );
}

/*! \brief Read HEM MAX ERASE configuration from SMWR OPTION 3.
*
* This function reads back the HEM MAX ERASE configuration from SMWR OPTION 3 register.
*
* \return HEM MAX ERASE value set.
*/
static inline uint16_t
flash_HEM_MAX_ERASE_get( void )
{
    uint16_t result = 0U;
    FLASH_GENERIC_GET(
        SMWR_OPTION_3,
        result,
        FLASH_SMWR_OPTION_3_HEM_MAX_ERASE_MASK,
        FLASH_SMWR_OPTION_3_HEM_MAX_ERASE_OFFSET
    );
    return result;
}

/*! \brief Configure Smart Program WHV.
 *
 * \param value The value of SMP WHV to be set.
 *
 */
static inline void
flash_SMP_WHV_configuration_set(
	uint64_t const data
)
{
    AMBA_FLASH_PTR->SMWR_SMP_WHV_1 = ( uint32_t ) data;
    AMBA_FLASH_PTR->SMWR_SMP_WHV_2 = ( uint32_t )( data >> 32U );
}

/*! \brief Read Smart Program WHV value.
 *
 * This function reads back the value of SMP WHV.
 *
 * \return The value of SMP WHV.
 */
static inline uint64_t
flash_SMP_WHV_configuration_get( void )
{
    uint64_t result = 0U;
    result |= AMBA_FLASH_PTR->SMWR_SMP_WHV_1;
    result <<= 32U;
    result |= AMBA_FLASH_PTR->SMWR_SMP_WHV_2;
    return result;
}

/*! \brief Configure Smart Erase WHV.
 *
 * \param value The value of SME WHV to be set.
 *
 */
static inline void
flash_SME_WHV_configuration_set(
	uint64_t const data
)
{
    AMBA_FLASH_PTR->SMWR_SME_WHV_1 = ( uint32_t ) data;
    AMBA_FLASH_PTR->SMWR_SME_WHV_2 = ( uint32_t )( data >> 32U );
}

/*! \brief Read Smart Erase WHV value.
 *
 * This function reads back the value of SME WHV.
 *
 * \return The value of SME WHV.
 */
static inline uint64_t
flash_SME_WHV_configuration_get( void )
{
    uint64_t result = 0U;
    result |= AMBA_FLASH_PTR->SMWR_SME_WHV_1;
    result <<= 32U;
    result |= AMBA_FLASH_PTR->SMWR_SME_WHV_2;
    return result;
}

/*! \brief Unlock access to COMMAND register.
 *
 * This function will unlock access to commands modifying content of program memory and their fuse bits.
 */
static inline void
flash_unlock_command( void )
{
    AMBA_FLASH_PTR->LOCK = FLASH_LOCK_ACCESS_PASSWORD;
}

/*! \brief Unlock access to COMMAND register.
 *
 * This function will unlock access to commands modifying content of factory row and their fuse bits.
 */
static inline void
flash_unlock_factory_row_command( void )
{
    AMBA_FLASH_PTR->LOCK = FLASH_LOCK_FACTORY_ROW_ACCESS_PASSWORD;
}

/*! \brief Unlock access to COMMAND register.
 *
 * This function will unlock access to commands modifying content of user page and their fuse bits.
 */
static inline void
flash_unlock_user_row_command( void )
{
    AMBA_FLASH_PTR->LOCK = FLASH_LOCK_USER_ROW_ACCESS_PASSWORD;
}

/*! \brief Unlock access to COMMAND register.
 *
 * This function will unlock access to commands modifying content of manufacturer row and their fuse bits.
 */
static inline void
flash_unlock_manufacturer_row_command( void )
{
    AMBA_FLASH_PTR->LOCK = FLASH_LOCK_MANUFACTURER_ROW_ACCESS_PASSWORD;
}

/*! \brief Unlock access to COMMAND register.
 *
 * This function will unlock access to commands modifying content of IFREN pages and their fuse bits.
 */
static inline void
flash_unlock_ifren_memory_command( void )
{
    AMBA_FLASH_PTR->LOCK = FLASH_LOCK_IFREN_ACCESS_PASSWORD;
}

/*! \brief Unlock access to COMMAND register.
 *
 * This function will unlock access to commands modifying content of IFREN1 pages and their fuse bits.
 */
static inline void
flash_unlock_ifren1_memory_command( void )
{
    AMBA_FLASH_PTR->LOCK = FLASH_LOCK_IFREN1_ACCESS_PASSWORD;
}

/*! \brief Unlock access to COMMAND register.
 *
 * This function will unlock access to commands modifying REDEN configuration and their fuse bits.
 */
static inline void
flash_unlock_reden_command( void )
{
    AMBA_FLASH_PTR->LOCK = FLASH_LOCK_REDEN_CONFIG_ACCESS_PASSWORD;
}

/*! \brief Unlock access to COMMAND register.
 *
 * This function will unlock access to Erase Reference Cell command and its fuse bit.
 */
static inline void
flash_unlock_erase_reference_cell_command( void )
{
    AMBA_FLASH_PTR->LOCK = FLASH_LOCK_ERASE_REFERENCE_CELL_ACCESS_PASSWORD;
}

/*! \brief Issue command to the controller (via COMMAND register).
 */
static inline void
flash_issue_command( amba_flash_command_t const command )
{
    AMBA_FLASH_PTR->COMMAND = (( uint32_t ) command) & FLASH_COMMAND_MASK;
}

/*! \brief Enables FLASH READY interrupt.
 */
static inline void
flash_ready_interrupt_enable( void )
{
    FLASH_GENERIC_ENABLE( IRQM, FLASH_IRQM_READY_IE );
}

/*! \brief Disables FLASH READY interrupt.
 */
static inline void
flash_ready_interrupt_disable( void )
{
    FLASH_GENERIC_DISABLE( IRQM, FLASH_IRQM_READY_IE );
}

/*! \brief Check if FLASH READY interrupt is enabled.
 *
 * \return Status of FLASH READY interrupt
 *  \retval true interrupt is enabled
 *  \retval false interrupt is disabled
 */
static inline bool
flash_ready_interrupt_state( void )
{
    return FLASH_GENERIC_STATE( IRQM, FLASH_IRQM_READY_IE );
}

/*! \brief Enables PROGRAMMING ERROR interrupt.
 */
static inline void
flash_programming_error_interrupt_enable( void )
{
    FLASH_GENERIC_ENABLE( IRQM, FLASH_IRQM_PROGRAMMING_ERROR_IE );
}

/*! \brief Disables PROGRAMMING ERROR interrupt.
 */
static inline void
flash_programming_error_interrupt_disable( void )
{
    FLASH_GENERIC_DISABLE( IRQM, FLASH_IRQM_PROGRAMMING_ERROR_IE );
}

/*! \brief Check if PROGRAMMING ERROR interrupt is enabled.
 *
 * \return Status of PROGRAMMING ERROR interrupt
 *  \retval true interrupt is enabled
 *  \retval false interrupt is disabled
 */
static inline bool
flash_programming_error_interrupt_state( void )
{
    return FLASH_GENERIC_STATE( IRQM, FLASH_IRQM_PROGRAMMING_ERROR_IE );
}

/*! \brief Enables LOCK ERROR interrupt.
 */
static inline void
flash_lock_error_interrupt_enable( void )
{
    FLASH_GENERIC_ENABLE( IRQM, FLASH_IRQM_LOCK_ERROR_IE );
}

/*! \brief Disables LOCK ERROR interrupt.
 */
static inline void
flash_lock_error_interrupt_disable( void )
{
    FLASH_GENERIC_DISABLE( IRQM, FLASH_IRQM_LOCK_ERROR_IE );
}

/*! \brief Check if LOCK ERROR interrupt is enabled.
 *
 * \return Status of LOCK ERROR interrupt
 *  \retval true interrupt is enabled
 *  \retval false interrupt is disabled
 */
static inline bool
flash_lock_error_interrupt_state( void )
{
    return FLASH_GENERIC_STATE( IRQM, FLASH_IRQM_LOCK_ERROR_IE );
}

/*! \brief Enables ECC CORRECTABLE ERROR interrupt.
 */
static inline void
flash_ecc_correctable_error_interrupt_enable( void )
{
    FLASH_GENERIC_ENABLE( IRQM, FLASH_IRQM_ECC_CORRECTABLE_ERROR_IE );
}

/*! \brief Disables ECC CORRECTABLE ERROR interrupt.
 */
static inline void
flash_ecc_correctable_error_interrupt_disable( void )
{
    FLASH_GENERIC_DISABLE( IRQM, FLASH_IRQM_ECC_CORRECTABLE_ERROR_IE );
}

/*! \brief Check if ECC CORRECTABLE ERROR interrupt is enabled.
 *
 * \return Status of ECC CORRECTABLE ERROR interrupt
 *  \retval true interrupt is enabled
 *  \retval false interrupt is disabled
 */
static inline bool
flash_ecc_correctable_error_interrupt_state( void )
{
    return FLASH_GENERIC_STATE( IRQM, FLASH_IRQM_ECC_CORRECTABLE_ERROR_IE );
}

/*! \brief Enables ECC UNCORRECTABLE ERROR interrupt.
 */
static inline void
flash_ecc_uncorrectable_error_interrupt_enable( void )
{
    FLASH_GENERIC_ENABLE( IRQM, FLASH_IRQM_ECC_UNCORRECTABLE_ERROR_IE );
}

/*! \brief Disables ECC UNCORRECTABLE ERROR interrupt.
 */
static inline void
flash_ecc_uncorrectable_error_interrupt_disable( void )
{
    FLASH_GENERIC_DISABLE( IRQM, FLASH_IRQM_ECC_UNCORRECTABLE_ERROR_IE );
}

/*! \brief Check if ECC UNCORRECTABLE ERROR interrupt is enabled.
 *
 * \return Status of ECC UNCORRECTABLE ERROR interrupt
 *  \retval true interrupt is enabled
 *  \retval false interrupt is disabled
 */
static inline bool
flash_ecc_uncorrectable_error_interrupt_state( void )
{
    return FLASH_GENERIC_STATE( IRQM, FLASH_IRQM_ECC_UNCORRECTABLE_ERROR_IE );
}

/*! \brief Configure mapping of the FLASH peripheral interrupt.
 *
 * \param irq_mapping mapping mask of the FLASH peripheral interrupt.
 */
static inline void
flash_irq_map_set(
	uint32_t const irq_map
)
{
    AMBA_FLASH_PTR->IRQMAP = irq_map;
}

/*! \brief Returns mapping of the FLASH peripheral interrupt.
 *
 * \return \c mapping mapping mask of the FLASH peripheral interrupt.
 */
static inline uint32_t
flash_irq_map_get( void )
{
    return AMBA_FLASH_PTR->IRQMAP;
}

/************* utility **************/

/*! \brief Returns programmable sector size (in bytes)
 *
 * \return Size of data sector in bytes.
 */
size_t
flash_sector_size( void );

/*! \brief Returns erase'able page size (in bytes)
 *
 * \return Size of data page in bytes.
 */
size_t
flash_page_size( void );

/*! \brief Returns number of bytes within single flash memory module.
 * \note ( flash_module_size() % flash_page_size() ) == 0U
 * \return Number of data pages within memory module
 */
size_t
flash_module_size( void );

/*! \brief Return the number of bytes per program memory region.
 * \note ( flash_region_size() % flash_page_size() ) == 0U
 * \return The number of pages contained in a single region (the smallest entity that can be locked).
 */
size_t
flash_region_size( void );

/*! \brief Return the number of flash memory modules.
 *
 * \return The number of flash memory modules.
 */
uint32_t
flash_modules( void );

/*! \brief Loop while module is busy.
 *
 * Function can be used to poll the status of flash controller. Polling will stop, when busy status gets deasserted.
 *
 * \return Status of flash controller.
 *   \retval READY all operations completed. Ready for next command
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR command should have been unlocked first (by writing a password to LOCK register).
 *   \retval ECC_CORR_ERROR ECC correctable error detected during flash data read.
 *   \retval ECC_UNCORR_ERROR ECC uncorrectable error detected during flash data read - data read is invalid.
 */
flash_access_status_t
flash_loop_while_busy( void );

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
flash_check_status( void );

/*! \brief Ensure any buffered data chages are synchronized to flash.
 *
 * Function can be used to write any outstanding buffered writes to the flash.
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
flash_sync( void );

/*! \brief Set minimum and maximum writes threshold for cache synchronization.
 *
 * TSMC40ULPFMC driver uses page buffer cache to optimize flash accesses. The
 * cache is synchronized to theunderlying storage after some amount of writes
 * were done. The threshold acts as a limit of writes after which cache must be
 * synchronized. Minimum threshold value is the amount of writes after which the
 * driver will try flushing cache if possible. Maximum threshold value is the
 * amount of writes after which the driver shall always perform a flush, waiting
 * until cache has been written to the underlying storage device.
 */
void
flash_cache_threshold( uint8_t const min, uint8_t const max );

#endif  // _FLASH_H_

