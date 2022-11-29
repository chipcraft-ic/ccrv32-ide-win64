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
 * $Date: 2022-03-27 20:28:45 +0200 (nie, 27 mar 2022) $
 * $Revision: 845 $
 *H*****************************************************************************/

#include <stddef.h>

#include "flash.h"

/*! \brief Helper function for calculating wait states count based on HCLK frequency.
 *
 * \param hclk_freq_hz The frequency of HCLK clock to be computed on.
 * \param delay_ns_x_10 The required time delay in 1/10ths of a ns (so 12.5 ns should be passed as 125).
 * \retval LOCK_ERROR operation should have been unlocked first.
 */
uint32_t flash_calc_clock_wait_states(uint32_t const hclk_freq_hz, uint32_t const delay_ns_x_10)
{
    uint64_t clockTickIn01NS = 10000000000ULL / (uint64_t)hclk_freq_hz;
    uint32_t waitStatesCount = (delay_ns_x_10) / clockTickIn01NS;
    if((clockTickIn01NS < delay_ns_x_10) && ((clockTickIn01NS * waitStatesCount) <= delay_ns_x_10))
    {
        ++waitStatesCount;
    }
    return waitStatesCount;
}

/*! \brief Sets the configuration of the Flash controller.
 *
 * \param hclk_freq_hz The frequency of HCLK clock to be configured.
 * \param wait_states_count The number of wait states for read operation.
 * \param sequential_prefetch_enable Enables sequential prefetch feature.
 * \param start_all_reads_as_sequential_enable Enables treating all new reads as start of a sequential burst feature.
 */
void flash_configure(uint32_t const hclk_freq_hz, uint8_t const wait_states_count, bool const sequential_prefetch_enable, bool const start_all_reads_as_sequential_enable)
{
    flash_configure_timings(hclk_freq_hz);
    flash_set_read_wait_states(wait_states_count);
    sequential_prefetch_enable ? flash_enable_sequential_prefetch() : flash_disable_sequential_prefetch();
    start_all_reads_as_sequential_enable ? flash_enable_start_all_reads_as_sequential() : flash_disable_start_all_reads_as_sequential();
    if (hclk_freq_hz < 254000000UL)
    {
        flash_set_smwr_prescaler(FLASH_CTRL_SMWR_PRESCALER_1);
    }
    else if(hclk_freq_hz/2 < 254000000UL)
    {
        flash_set_smwr_prescaler(FLASH_CTRL_SMWR_PRESCALER_2);
    }
    else if(hclk_freq_hz/4 < 254000000UL)
    {
        flash_set_smwr_prescaler(FLASH_CTRL_SMWR_PRESCALER_4);
    }
    else
    {
        flash_set_smwr_prescaler(FLASH_CTRL_SMWR_PRESCALER_8);
    }
}

/*! \brief Sets the TIMING configuration register based on the frequency of HCLK clock.
 *
 * \param hclk_freq_hz The frequency of HCLK clock to be configured.
 *
 */
void flash_configure_timings(uint32_t const hclk_freq_hz)
{
    uint32_t regValue = 0;
    regValue |= ((flash_calc_clock_wait_states(hclk_freq_hz, 20) << FLASH_TIMING_COUNT_2_NS_OFFSET) & FLASH_TIMING_COUNT_2_NS_MASK);
    regValue |= ((flash_calc_clock_wait_states(hclk_freq_hz, 125) << FLASH_TIMING_COUNT_12_5_NS_OFFSET) & FLASH_TIMING_COUNT_12_5_NS_MASK);
    regValue |= ((flash_calc_clock_wait_states(hclk_freq_hz, 200) << FLASH_TIMING_COUNT_20_NS_OFFSET) & FLASH_TIMING_COUNT_20_NS_MASK);
    regValue |= ((flash_calc_clock_wait_states(hclk_freq_hz, 1000) << FLASH_TIMING_COUNT_100_NS_OFFSET) & FLASH_TIMING_COUNT_100_NS_MASK);
    regValue |= ((flash_calc_clock_wait_states(hclk_freq_hz, 10000) << FLASH_TIMING_COUNT_1_US_OFFSET) & FLASH_TIMING_COUNT_1_US_MASK);
    AMBA_FLASH_PTR->TIMING = regValue;
}

/*! \brief Clears (set to 0xFF) the content of Sector Buffer.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_clear_sector_buffer()
{
    flash_issue_command(FLASH_COMMAND_CLR_SECTOR_BUFFER);
    return flash_check_status();
}

/*! \brief Writes quad-word of data into the sector buffer.
 *
 * \param sector_buffer_start_word_offset The first word offset of the data within the sector buffer (needs to be aligned to 4 x 32-bit words).
 * \param data_qword The data quad-word to be written into sector buffer.
 *
 * This function sets up all registers and starts the write operation but doesn't wait for it to complete.
 * User is expected to monitor status until the write process is completed.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_qword_to_sector_buffer(uint16_t const sector_buffer_start_word_offset, const uint32_t data_qword[4])
{
    if((sector_buffer_start_word_offset > (sizeof(AMBA_FLASH_PTR->SECTOR_BUFFER) / sizeof(*(AMBA_FLASH_PTR->SECTOR_BUFFER)))) ||
       ((sector_buffer_start_word_offset % 4) != 0))
    {
        return ARGUMENT_ERROR;
    }
    AMBA_FLASH_PTR->DATA_WORD_0 = data_qword[0];
    AMBA_FLASH_PTR->DATA_WORD_1 = data_qword[1];
    AMBA_FLASH_PTR->DATA_WORD_2 = data_qword[2];
    AMBA_FLASH_PTR->DATA_WORD_3 = data_qword[3];
    AMBA_FLASH_PTR->ADDRESS = sector_buffer_start_word_offset * 4;
    flash_unlock_command();
    flash_issue_command(FLASH_COMMAND_WRITE_SECTOR_BUFFER_QWORD);
    return flash_check_status();
}

/*! \brief Writes single word of data into the sector buffer.
 *
 * \param sector_buffer_word_offset The word offset of the data within the sector buffer.
 * \param data_word The data word to be written into sector buffer.
 *
 * This function uses direct access to the Sector Buffer, meaning it won't return until the write actually completed.
 * Status returned shall either be READY or one of the error codes.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_word_to_sector_buffer_blocking(uint16_t const sector_buffer_word_offset, uint32_t const data_word)
{
    if(sector_buffer_word_offset > (sizeof(AMBA_FLASH_PTR->SECTOR_BUFFER) / sizeof(*(AMBA_FLASH_PTR->SECTOR_BUFFER))))
    {
        return ARGUMENT_ERROR;
    }
    AMBA_FLASH_PTR->SECTOR_BUFFER[sector_buffer_word_offset] = data_word;
    return flash_check_status();
}

/*! \brief Reads single word of data from the sector buffer.
 *
 * \param sector_buffer_word_offset The word offset of the data within the sector buffer.
 * \param data_word The data word to read from sector buffer.
 *
 * This function reads single data word from the Sector Buffer. Direct access scheme is used.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_read_sector_buffer_word(uint8_t const sector_buffer_word_offset, uint32_t* const data_word)
{
    if(sector_buffer_word_offset > (sizeof (AMBA_FLASH_PTR->SECTOR_BUFFER) / sizeof (* (AMBA_FLASH_PTR->SECTOR_BUFFER))))
    {
        return ARGUMENT_ERROR;
    }
    *data_word = AMBA_FLASH_PTR->SECTOR_BUFFER[sector_buffer_word_offset];
    return flash_check_status();
}

/*! \brief Writes single sector content to program memory.
 *
 * \param destination_address The start address of memory to be written (should be aligned to 4 x 32-bit words).
 * \param data_buffer The pointer to memory buffer containing data to be written.
 * \param num_words The number of words to be written (needs to be a multiple of 4)).
 *
 * This function copies provided data to sector buffer and initiates write operation.
 * This function will not wait for the write to finish.
 * Input buffer can contain partial sector data, but it may not cross the sector boundary (256 words).
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data could not be written to flash memory due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_sector(uint32_t* const destination_address, uint32_t const* const data_buffer, uint16_t const num_words)
{
    uint32_t index;
    volatile flash_access_status_t status;
    if(((((uint32_t)destination_address - ROM_BASE) % 4) != 0) || (((uint32_t)destination_address - ROM_BASE) + num_words * 4 > (((((uint32_t)destination_address - ROM_BASE) / flash_get_sector_size_in_bytes()) + 1) * flash_get_sector_size_in_bytes())))
    {
        return ARGUMENT_ERROR;
    }
    status = flash_clear_sector_buffer();
    if(status == BUSY)
    {
        status = flash_loop_while_busy();
    }
    if(status == READY)
    {
        flash_unlock_write_sector_data();
        for(index = 0; index < num_words; index++)
        {
            destination_address[index] = data_buffer[index];
        }
        AMBA_FLASH_PTR->ADDRESS = (uint32_t)destination_address;
        flash_unlock_command();
        flash_issue_command (FLASH_COMMAND_WRITE_SECTOR);
        status = flash_check_status();
    }
    return status;
}

/*! \brief Writes single sector content to program memory.
 *
 * \param destination_address The start address of memory to be written (should be aligned to 4 x 32-bit words).
 * \param data_buffer The pointer to memory buffer containing data to be written.
 * \param num_words The number of words to be written (needs to be a multiple of 4)).
 *
 * This function copies provided data to sector buffer through APB interface and initiates write operation.
 * This function will not wait for the write to finish.
 * Input buffer can contain partial sector data, but it may not cross the sector boundary (256 words).
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data could not be written to flash memory due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_sector_slow(uint32_t* const destination_address, uint32_t const* const data_buffer, uint16_t const num_words)
{
    uint32_t index;
    volatile flash_access_status_t status;
    if( ( (((uint32_t)destination_address - ROM_BASE) % 4) != 0) || (((uint32_t)destination_address - ROM_BASE) + num_words * 4 > ( ( (((uint32_t)destination_address - ROM_BASE) / flash_get_sector_size_in_bytes()) + 1) * flash_get_sector_size_in_bytes())))
    {
        return ARGUMENT_ERROR;
    }
    status = flash_clear_sector_buffer();
    if(status == BUSY)
    {
        status = flash_loop_while_busy();
    }
    if(status == READY)
    {
        for(index = 0; index < num_words; index++)
        {
            AMBA_FLASH_PTR->SECTOR_BUFFER[(((uint32_t)destination_address % flash_get_sector_size_in_bytes())/4) + index] = data_buffer[index];
        }
        AMBA_FLASH_PTR->ADDRESS = (uint32_t)destination_address;
        flash_unlock_command();
        flash_issue_command (FLASH_COMMAND_WRITE_SECTOR);
        status = flash_check_status();
    }
    return status;
}

/*! \brief Writes data quad-word to program memory.
 *
 * \param destination_address The start address of memory to be written (needs to be aligned to 4 x 32-bit words).
 * \param data_qword Data quad-word to be written.
 *
 * This function copies provided data words to sector buffer and initiates write operation (previous sector buffer content is lost).
 * This function will not wait for the write to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data could not be written to flash memory due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_qword(uint32_t* destination_address, const uint32_t data_qword[4])
{
    volatile flash_access_status_t status;
    if( (((uint32_t)destination_address - ROM_BASE) % (4*4)) != 0)
    {
        return ARGUMENT_ERROR;
    }
    status = flash_clear_sector_buffer();
    if(status == BUSY)
    {
        status = flash_loop_while_busy();
    }
    if(status == READY)
    {
        flash_unlock_write_sector_data();

        for (size_t index = 0; index < 4; index++)
        {
            *destination_address = data_qword[index];
            ++destination_address;
        }

        AMBA_FLASH_PTR->ADDRESS = (uint32_t)destination_address;
        flash_unlock_command();
        flash_issue_command (FLASH_COMMAND_WRITE_QWORD);
        status = flash_check_status();
    }
    return status;
}

/*! \brief Writes data quad-word to program memory.
 *
 * \param destination_address The start address of memory to be written (needs to be aligned to 4 x 32-bit words).
 * \param data_qword Data quad-word to be written.
 *
 * This function writes provided data words to program memory. It doesn't use sector buffer to do this, so it's content will be untouched.
 * This function will not wait for the write operation to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data could not be written to flash memory due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_qword_immediate(uint32_t* const destination_address, const uint32_t data_qword[4])
{
    if( (((uint32_t)destination_address - ROM_BASE) % (4*4)) != 0)
    {
        return ARGUMENT_ERROR;
    }
    AMBA_FLASH_PTR->DATA_WORD_0 = data_qword[0];
    AMBA_FLASH_PTR->DATA_WORD_1 = data_qword[1];
    AMBA_FLASH_PTR->DATA_WORD_2 = data_qword[2];
    AMBA_FLASH_PTR->DATA_WORD_3 = data_qword[3];
    AMBA_FLASH_PTR->ADDRESS = (uint32_t)destination_address;
    flash_unlock_command();
    flash_issue_command (FLASH_COMMAND_WRITE_IMMEDIATE);
    return flash_check_status();
}

/*! \brief Erases single data page from program memory.
 *
 * \param destination_address Address within the memory page to be erased.
 *
 * This function initializes erasure of single data page from program memory.
 * It will not wait for the erase operation to complete.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_erase_page(uint32_t* const destination_address)
{
    AMBA_FLASH_PTR->ADDRESS = (uint32_t)destination_address;
    flash_unlock_command();
    flash_issue_command (FLASH_COMMAND_ERASE_PAGE);
    return flash_check_status();
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
flash_access_status_t flash_erase_all()
{
    flash_unlock_command();
    flash_issue_command (FLASH_COMMAND_ERASE_ALL);
    return flash_check_status();
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
flash_access_status_t flash_erase_reference_cell()
{
    flash_unlock_erase_reference_cell_command();
    flash_issue_command (FLASH_COMMAND_ERASE_REFERENCE_CELL);
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
flash_access_status_t flash_lock_region(const uint32_t* const address_within_region_to_lock)
{
    AMBA_FLASH_PTR->ADDRESS = (uint32_t)address_within_region_to_lock;
    flash_unlock_command();
    flash_issue_command (FLASH_COMMAND_LOCK_REGION);
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
flash_access_status_t flash_lock_debugger_read()
{
    flash_unlock_command();
    flash_issue_command (FLASH_COMMAND_LOCK_DEBUGGER_READ);
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
flash_access_status_t flash_lock_debugger_access()
{
    flash_unlock_command();
    flash_issue_command (FLASH_COMMAND_LOCK_DEBUGGER_ACCESS);
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
bool flash_is_region_locked(const uint32_t* const address_within_region_to_check_lock)
{
    uint32_t memory_offset = (uint32_t)address_within_region_to_check_lock - ROM_BASE;
    uint8_t region_locks_reg_offset = 0;
    uint8_t region_locks_bit_offset = 0;
    uint32_t region_lock_bit_number = (memory_offset / flash_get_page_size_in_bytes()) / flash_get_region_size_in_pages();

    region_locks_reg_offset = region_lock_bit_number / 32;
    region_locks_bit_offset = region_lock_bit_number % 32;

    return ( (AMBA_FLASH_PTR->REGION_LOCKS[region_locks_reg_offset] & (1 << region_locks_bit_offset)) != 0);
}

/*! \brief Writes quad-word of data into the factory row.
 *
 * \param word_offset_in_row The word offset of the data within the factory row (needs to be aligned to 4 x 32-bit words).
 * \param data_qword The data quad-word to be written into the factory row.
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
flash_access_status_t flash_write_factory_row_qword(uint16_t word_offset_in_row, const uint32_t data_qword[4])
{
    if((word_offset_in_row > (flash_get_page_size_in_bytes()/4)) ||
       ((word_offset_in_row % 4) != 0))
    {
        return ARGUMENT_ERROR;
    }
    AMBA_FLASH_PTR->DATA_WORD_0 = data_qword[0];
    AMBA_FLASH_PTR->DATA_WORD_1 = data_qword[1];
    AMBA_FLASH_PTR->DATA_WORD_2 = data_qword[2];
    AMBA_FLASH_PTR->DATA_WORD_3 = data_qword[3];
    AMBA_FLASH_PTR->ADDRESS = word_offset_in_row * 4;
    flash_unlock_factory_row_command();
    flash_issue_command (FLASH_COMMAND_WRITE_FACTORY_ROW_QWORD);
    return flash_check_status();
}

/*! \brief Writes quad-word of data into the factory row.
 *
 * \param word_offset_in_row The word offset of the data within the factory row (needs to be aligned to 4 x 32-bit words).
 * \param data_qword The data quad-word to be written into the factory row.
 *
 * This function won't exit until the write is completed.
 * Status returned shall either be READY or one of the error codes.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_factory_row_qword_blocking(uint16_t const word_offset_in_row, const uint32_t data_qword[4])
{
    flash_access_status_t status = flash_write_factory_row_qword(word_offset_in_row, data_qword);
    if(status == BUSY)
    {
        status = flash_loop_while_busy();
    }
    return status;
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
flash_access_status_t flash_erase_factory_row()
{
    flash_unlock_factory_row_command();
    flash_issue_command (FLASH_COMMAND_ERASE_FACTORY_ROW);
    return flash_check_status();
}

/*! \brief Reads quad-word of data from the factory row.
 *
 * \param word_offset_in_row The word offset of the data within the factory row (needs to be aligned to 4 x 32-bit words).
 * \param data_qword Pointer to a buffer where read data will be placed.
 *
 * This function reads data quad-word from the factory row.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_read_factory_row_qword(uint16_t const word_offset_in_row, uint32_t data_qword[4])
{
    if((word_offset_in_row > (flash_get_page_size_in_bytes()/4)) ||
       ((word_offset_in_row % 4) != 0))
    {
        return ARGUMENT_ERROR;
    }
    AMBA_FLASH_PTR->ADDRESS = word_offset_in_row * 4;
    flash_unlock_factory_row_command();
    flash_issue_command(FLASH_COMMAND_READ_FACTORY_ROW_QWORD);
    flash_access_status_t status =  flash_check_status();
    if(status == BUSY)
    {
        status = flash_loop_while_busy();
    }
    data_qword[0] = AMBA_FLASH_PTR->DATA_WORD_0;
    data_qword[1] = AMBA_FLASH_PTR->DATA_WORD_1;
    data_qword[2] = AMBA_FLASH_PTR->DATA_WORD_2;
    data_qword[3] = AMBA_FLASH_PTR->DATA_WORD_3;

    return status;
}

/*! \brief Reads single word of data from the factory row.
 *
 * \param word_offset_in_row The word offset of the data within the factory row.
 * \param data_word Pointer to a buffer where read data will be placed.
 *
 * This function reads data word from the factory row.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_read_factory_row_word(uint16_t const word_offset_in_row, uint32_t* data_word)
{
    uint32_t dataQword[4];
    flash_access_status_t status = flash_read_factory_row_qword(((word_offset_in_row / 4) * 4), dataQword);
    *data_word = dataQword[word_offset_in_row%4];
    return status;
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
flash_access_status_t flash_lock_factory_row()
{
    flash_unlock_factory_row_command();
    flash_issue_command(FLASH_COMMAND_LOCK_FACTORY_ROW);
    return flash_check_status();
}

/*! \brief Writes quad-word of data into the user page.
 *
 * \param word_offset_in_row The word offset of the data within the user row (needs to be aligned to 4 x 32-bit words).
 * \param data_qword The data quad-word to be written into the user row.
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
flash_access_status_t flash_write_user_row_qword(uint16_t const word_offset_in_row, const uint32_t data_qword[4])
{
    if((word_offset_in_row > (flash_get_page_size_in_bytes()/4)) ||
       ((word_offset_in_row % 4) != 0))
    {
        return ARGUMENT_ERROR;
    }
    AMBA_FLASH_PTR->DATA_WORD_0 = data_qword[0];
    AMBA_FLASH_PTR->DATA_WORD_1 = data_qword[1];
    AMBA_FLASH_PTR->DATA_WORD_2 = data_qword[2];
    AMBA_FLASH_PTR->DATA_WORD_3 = data_qword[3];
    AMBA_FLASH_PTR->ADDRESS = word_offset_in_row * 4;
    flash_unlock_user_row_command();
    flash_issue_command (FLASH_COMMAND_WRITE_USER_ROW_QWORD);
    return flash_check_status();
}

/*! \brief Writes quad-word of data into the user page.
 *
 * \param word_offset_in_row The word offset of the data within the user row (needs to be aligned to 4 x 32-bit words).
 * \param data_qword The data quad-word to be written into the user row.
 *
 * This function won't exit until the write is completed.
 * Status returned shall either be READY or one of the error codes.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_user_row_qword_blocking(uint16_t const word_offset_in_row, const uint32_t data_qword[4])
{
    flash_access_status_t status = flash_write_user_row_qword(word_offset_in_row, data_qword);
    if(status == BUSY)
    {
        status = flash_loop_while_busy();
    }
    return status;
}

/*! \brief Reads quad-word of data from the user page.
 *
 * \param word_offset_in_row The word offset of the data within the user page (needs to be aligned to 4 x 32-bit words).
 * \param data_qword Pointer to a buffer where read data will be placed.
 *
 * This function reads data quad-word from the user page.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_read_user_row_qword(uint16_t const word_offset_in_row, uint32_t data_qword[4])
{
    if((word_offset_in_row > (flash_get_page_size_in_bytes()/4)) ||
       ((word_offset_in_row % 4) != 0))
    {
        return ARGUMENT_ERROR;
    }
    AMBA_FLASH_PTR->ADDRESS = word_offset_in_row * 4;
    flash_unlock_user_row_command();
    flash_issue_command(FLASH_COMMAND_READ_USER_ROW_QWORD);
    flash_access_status_t status = flash_check_status();
    if(status == BUSY)
    {
        status = flash_loop_while_busy();
    }
    data_qword[0] = AMBA_FLASH_PTR->DATA_WORD_0;
    data_qword[1] = AMBA_FLASH_PTR->DATA_WORD_1;
    data_qword[2] = AMBA_FLASH_PTR->DATA_WORD_2;
    data_qword[3] = AMBA_FLASH_PTR->DATA_WORD_3;

    return status;
}

/*! \brief Reads single word of data from the user page.
 *
 * \param word_offset_in_row The word offset of the data within the user page.
 * \param data_word Pointer to a buffer where read data will be placed.
 *
 * This function reads data word from the user page.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_read_user_row_word(uint16_t const word_offset_in_row, uint32_t* data_word)
{
    uint32_t dataQword[4];
    flash_access_status_t status = flash_read_user_row_qword(((word_offset_in_row / 4) * 4), dataQword);
    *data_word = dataQword[word_offset_in_row%4];
    return status;
}

/*! \brief Erases content of user page.
 *
 * This function initializes erasure of the data contained in user page.
 * It will not wait for the erase operation to complete.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_erase_user_row()
{
    flash_unlock_user_row_command();
    flash_issue_command (FLASH_COMMAND_ERASE_USER_ROW);
    return flash_check_status();
}

/*! \brief Locks ability to write and erase data within user page.
 *
 * This function initializes procedure for locking user row write/erase access to memory.
 * It will not wait for the operation to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_lock_user_row()
{
    flash_unlock_user_row_command();
    flash_issue_command (FLASH_COMMAND_LOCK_USER_ROW);
    return flash_check_status();
}

/*! \brief Writes quad-word of data into the manufacturer page.
 *
 * \param word_offset_in_row The word offset of the data within the manufacturer page (needs to be aligned to 4 x 32-bit words).
 * \param data_qword The data quad-word to be written into the manufacturer page.
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
flash_access_status_t flash_write_manufacturer_row_qword(uint16_t const word_offset_in_row, const uint32_t data_qword[4])
{
    if((word_offset_in_row > (flash_get_page_size_in_bytes()/4)) ||
       ((word_offset_in_row % 4) != 0))
    {
        return ARGUMENT_ERROR;
    }
    AMBA_FLASH_PTR->DATA_WORD_0 = data_qword[0];
    AMBA_FLASH_PTR->DATA_WORD_1 = data_qword[1];
    AMBA_FLASH_PTR->DATA_WORD_2 = data_qword[2];
    AMBA_FLASH_PTR->DATA_WORD_3 = data_qword[3];
    AMBA_FLASH_PTR->ADDRESS = word_offset_in_row * 4;
    flash_unlock_manufacturer_row_command();
    flash_issue_command (FLASH_COMMAND_WRITE_MANUFACTURER_ROW_QWORD);
    return flash_check_status();
}

/*! \brief Writes quad-word of data into the manufacturer page.
 *
 * \param word_offset_in_row The word offset of the data within the manufacturer page (needs to be aligned to 4 x 32-bit words).
 * \param data_qword The data quad-word to be written into the manufacturer page.
 *
 * This function uses direct access to the manufacturer page, meaning it won't exit until the write actually completed.
 * Status returned shall either be READY or one of the error codes.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR programming error occurred (data could not be written to the buffer due to invalid parameters or asset being inaccessible).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_write_manufacturer_row_qword_blocking(uint16_t const word_offset_in_row, const uint32_t data_qword[4])
{
    flash_access_status_t status = flash_write_manufacturer_row_qword(word_offset_in_row, data_qword);
    if(status == BUSY)
    {
        status = flash_loop_while_busy();
    }
    return status;
}

/*! \brief Erases content of manufacturer page.
 *
 * This function initializes erasure of the data contained in manufacturer page.
 * It will not wait for the erase operation to complete.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_erase_manufacturer_row()
{
    flash_unlock_manufacturer_row_command();
    flash_issue_command (FLASH_COMMAND_ERASE_MANUFACTURER_ROW);
    return flash_check_status();
}

/*! \brief Reads quad-word of data from the manufacturer page.
 *
 * \param word_offset_in_row The word offset of the data within the manufacturer row (needs to be aligned to 4 x 32-bit words).
 * \param data_qword A pointer to quad-word to place data read from the manufacturer row into.
 *
 * This function reads data quad-word from the user page. Direct access scheme is used.
 * This function reads single data word from the manufacturer page. Direct access scheme is used.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_read_manufacturer_row_qword(uint16_t const word_offset_in_row, uint32_t data_qword[4])
{
    if((word_offset_in_row > (flash_get_page_size_in_bytes()/4)) ||
       ((word_offset_in_row % 4) != 0))
    {
        return ARGUMENT_ERROR;
    }
    AMBA_FLASH_PTR->ADDRESS = word_offset_in_row * 4;
    flash_unlock_manufacturer_row_command();
    flash_issue_command(FLASH_COMMAND_READ_MANUFACTURER_ROW_QWORD);
    flash_access_status_t status = flash_check_status();
    if(status == BUSY)
    {
        status = flash_loop_while_busy();
    }
    data_qword[0] = AMBA_FLASH_PTR->DATA_WORD_0;
    data_qword[1] = AMBA_FLASH_PTR->DATA_WORD_1;
    data_qword[2] = AMBA_FLASH_PTR->DATA_WORD_2;
    data_qword[3] = AMBA_FLASH_PTR->DATA_WORD_3;

    return status;
}

/*! \brief Reads single word of data from the manufacturer page.
 *
 * \param word_offset_in_row The word offset of the data within the manufacturer page.
 * \param data_word Pointer to a buffer where read data will be placed.
 *
 * This function reads data word from the manufacturer page.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval PROGRAMMING_ERROR data could not be read from the buffer due to invalid parameters or asset being inaccessible.
 *   \retval ARGUMENT_ERROR invalid function input parameter.
 */
flash_access_status_t flash_read_manufacturer_row_word(uint16_t const word_offset_in_row, uint32_t* data_word)
{
    uint32_t dataQword[4];
    flash_access_status_t status = flash_read_manufacturer_row_qword(((word_offset_in_row / 4) * 4), dataQword);
    *data_word = dataQword[word_offset_in_row%4];
    return status;
}

/*! \brief Locks ability to write and erase data within manufacturer page.
 *
 * This function initializes procedure for locking manufacturer page write/erase access to memory.
 * It will not wait for the operation to finish.
 *
 * \return Status of operation.
 *   \retval READY operation completed.
 *   \retval BUSY operation in progress.
 *   \retval PROGRAMMING_ERROR programming error occurred - data page could not be erased due to invalid parameters or asset being inaccessible (region locked).
 *   \retval LOCK_ERROR operation should have been unlocked first.
 */
flash_access_status_t flash_lock_manufacturer_row()
{
    flash_unlock_manufacturer_row_command();
    flash_issue_command (FLASH_COMMAND_LOCK_MANUFACTURER_ROW);
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
flash_access_status_t flash_chip_erase()
{
    flash_unlock_command();
    flash_issue_command (FLASH_COMMAND_CHIP_ERASE);
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
flash_access_status_t flash_lock_chip_erase()
{
    flash_unlock_command();
    flash_issue_command (FLASH_COMMAND_LOCK_CHIP_ERASE);
    return flash_check_status();
}

/*! \brief Returns programmable sector size (in bytes)
 *
 * \return Size of data sector in bytes.
 */
uint32_t flash_get_sector_size_in_bytes()
{
    uint32_t sector_size = (AMBA_FLASH_PTR->INFO & FLASH_INFO_SECTOR_SIZE_MASK) >> FLASH_INFO_SECTOR_SIZE_OFFSET;
    if(sector_size == FLASH_INFO_SECTOR_SIZE_1024B)
    {
        return 1024;
    }
    else
    {
        return 1024;
    }
}

/*! \brief Returns erase'able page size (in bytes)
 *
 * \return Size of data page in bytes.
 */
uint32_t flash_get_page_size_in_bytes()
{
    uint32_t page_size_in_sectors = (AMBA_FLASH_PTR->INFO & FLASH_INFO_PAGE_SIZE_MASK) >> FLASH_INFO_PAGE_SIZE_OFFSET;
    if(page_size_in_sectors == FLASH_INFO_PAGE_SIZE_8_SECTORS)
    {
        return 8 * flash_get_sector_size_in_bytes();
    }
    else
    {
        return 8 * flash_get_sector_size_in_bytes();
    }
}

/*! \brief Returns number of paged within single flash memory module.
 *
 * \return Number of data pages within memory module
 */
uint32_t flash_get_module_size_in_pages()
{
    uint32_t module_size = (AMBA_FLASH_PTR->INFO & FLASH_INFO_MODULE_SIZE_MASK) >> FLASH_INFO_MODULE_SIZE_OFFSET;
    if(module_size == FLASH_INFO_MODULE_SIZE_128_PAGES)
    {
        return 128;
    }
    else
    {
        return 128;
    }
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
flash_access_status_t flash_loop_while_busy()
{
    volatile uint32_t status;
    do
    {
        status = AMBA_FLASH_PTR->STATUS;
    }while( (status & FLASH_STATUS_BUSY) != 0);

    if(status & FLASH_STATUS_PROGRAMMING_ERROR)
    {
        return PROGRAMMING_ERROR;
    }
    else if(status & FLASH_STATUS_LOCK_ERROR)
    {
        return LOCK_ERROR;
    }
    else if(status & FLASH_STATUS_UNCORRECTABLE_ECC_ERR)
    {
        return ECC_UNCORR_ERROR;
    }
    else if(status & FLASH_STATUS_CORRECTABLE_ECC_ERR)
    {
        return ECC_CORR_ERROR;
    }
    else
    {
        return READY;
    }
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
flash_access_status_t flash_check_status()
{
    volatile uint32_t status = AMBA_FLASH_PTR->STATUS;
    if(status & FLASH_STATUS_PROGRAMMING_ERROR)
    {
        return PROGRAMMING_ERROR;
    }
    else if(status & FLASH_STATUS_LOCK_ERROR)
    {
        return LOCK_ERROR;
    }
    else if(status & FLASH_STATUS_UNCORRECTABLE_ECC_ERR)
    {
        return ECC_UNCORR_ERROR;
    }
    else if(status & FLASH_STATUS_CORRECTABLE_ECC_ERR)
    {
        return ECC_CORR_ERROR;
    }
    else if(status & FLASH_STATUS_BUSY)
    {
        return BUSY;
    }
    else
    {
        return READY;
    }
}

/*! \brief Reads data from main array.
 *
 * \param address Address to start read from.
 * \param data Pointer to output data.
 * \param words Number of words to read.
 */
void flash_read(const uint32_t* address, uint32_t* data, uint32_t words)
{
    for (uint32_t i=0; i<words; i++)
    {
        data[i] = *(address+i);
    }
}
