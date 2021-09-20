/*!
 * @file      smtc_hal_flash.c
 *
 * @brief     Board specific package FLASH API implementation
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
 /*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "stm32wbxx_hal.h"
#include "smtc_hal_flash.h"
#include "utilities.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief Initializes the FlashUserStartAddr to FLASH_USER_END_ADDR to avoid erase a occupied memory .
 */
uint32_t flash_user_start_addr = FLASH_USER_END_ADDR;

/**
 * @brief  Gets the page of a given address
 * @param  Addr: Address of the FLASH Memory
 * @retval The page of a given address
 */
static uint32_t get_page( uint32_t address );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/*!
 * @brief Initializes the FLASH module and find the first empty page.
 *
 * @returns User flash start address
 */
uint32_t flash_init( void )
{
    uint8_t  status = SUCCESS;
    uint8_t  buffer[ADDR_FLASH_PAGE_SIZE / 10]; 
    uint8_t  index_page = FLASH_USER_START_PAGE;
    uint64_t nb_empty_bytes = 0;

    while( ( nb_empty_bytes != ADDR_FLASH_PAGE_SIZE ) && ( index_page < FLASH_USER_END_PAGE ) )
    {
        nb_empty_bytes = 0;

        for( uint16_t j = 0; j < 0x10; j++ )
        {
            /* Start from ADDR_FLASH_PAGE_7 because of the bootloader */
            flash_read_buffer( ADDR_FLASH_PAGE_0 + ( index_page * ADDR_FLASH_PAGE_SIZE ) + ( j * 0x100 ), buffer,
                               0x100 );

            for( uint16_t i = 0; i < 0x100; i++ )
            {
                if( buffer[i] == 0xFF )
                {
                    nb_empty_bytes++;
                }
            }
        }
        index_page++;  // Check next page
    }
    flash_user_start_addr = ADDR_FLASH_PAGE_0 + ( index_page * ADDR_FLASH_PAGE_SIZE );

    return status;
}

uint8_t flash_erase_page( uint32_t addr, uint8_t nb_page )
{
    uint8_t  status          = SUCCESS;
    uint8_t  hal_status      = SUCCESS;
    uint32_t first_user_page = 0, nb_of_pages_max = 0;
    uint32_t page_error            = 0;
    uint8_t  flash_operation_retry = 0;

    FLASH_EraseInitTypeDef EraseInitStruct;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock( );

    /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_OPTVERR );

    /* Erase the user Flash area
    (area defined by FlashUserStartAddr and FLASH_USER_END_ADDR) ***********/

    /* Get the 1st page to erase */
    first_user_page = get_page( addr );

    /* Get the number of pages to erase from 1st page */
    nb_of_pages_max = get_page( FLASH_USER_END_ADDR ) - get_page( flash_user_start_addr ) + 1;

    if( ( flash_user_start_addr > addr ) || ( nb_page > nb_of_pages_max ) )
    {
        status = FAIL;
        return status;
    }

    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page      = first_user_page;
    EraseInitStruct.NbPages   = nb_page;

    /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
     you have to make sure that these data are rewritten before they are accessed during code
     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
     DCRST and ICRST bits in the FLASH_CR register. */
    do
    {
        hal_status = HAL_FLASHEx_Erase( &EraseInitStruct, &page_error );
        flash_operation_retry++;
    } while( ( hal_status != HAL_OK ) && ( flash_operation_retry < FLASH_OPERATION_MAX_RETRY ) );

    if( flash_operation_retry >= FLASH_OPERATION_MAX_RETRY )
    {
        /*
          Error occurred while  erase.
          User can add here some code to deal with this error.
          PageError will contain the faulty  and then to know the code error on this ,
          user can call function 'HAL_FLASH_GetError()'
        */
        /* Infinite loop */
        while( 1 )
        {
        }
    }
    else
    {
        flash_operation_retry = 0;
    }

    /* Lock the Flash to disable the flash control register access (recommended
    to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock( );

    return status;
}

uint8_t flash_force_erase_page( uint32_t addr, uint8_t nb_page )
{
    uint8_t  status                = SUCCESS;
    uint8_t  hal_status            = SUCCESS;
    uint32_t first_user_page       = 0;
    uint32_t page_error            = 0;
    uint8_t  flash_operation_retry = 0;

    FLASH_EraseInitTypeDef EraseInitStruct;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock( );

    /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_OPTVERR );

    /* Erase the user Flash area
    (area defined by FlashUserStartAddr and FLASH_USER_END_ADDR) ***********/

    /* Get the 1st page to erase */
    first_user_page = get_page( addr );

    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page      = first_user_page;
    EraseInitStruct.NbPages   = nb_page;

    /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
     you have to make sure that these data are rewritten before they are accessed during code
     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
     DCRST and ICRST bits in the FLASH_CR register. */
    do
    {
        hal_status = HAL_FLASHEx_Erase( &EraseInitStruct, &page_error );
        flash_operation_retry++;
    } while( ( hal_status != HAL_OK ) && ( flash_operation_retry < FLASH_OPERATION_MAX_RETRY ) );

    if( flash_operation_retry >= FLASH_OPERATION_MAX_RETRY )
    {
        /*
          Error occurred while  erase.
          User can add here some code to deal with this error.
          PageError will contain the faulty  and then to know the code error on this ,
          user can call function 'HAL_FLASH_GetError()'
        */
        /* Infinite loop */
        while( 1 )
        {
        }
    }
    else
    {
        flash_operation_retry = 0;
    }

    /* Lock the Flash to disable the flash control register access (recommended
    to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock( );

    return status;
}

uint32_t flash_write_buffer( uint32_t addr, uint8_t* buffer, uint32_t size )
{
    uint8_t  status       = SUCCESS;
    uint8_t  hal_status   = SUCCESS;
    uint32_t buffer_index = 0, nb_of_pages_max = 0, real_size = 0, addr_end = 0;
    uint64_t data64                = 0;
    uint8_t  flash_operation_retry = 0;

    /* Complete size for FLASH_TYPEPROGRAM_DOUBLEWORD operation*/
    if( ( size % 8 ) != 0 )
    {
        real_size = size + ( 8 - ( size % 8 ) );
    }
    else
    {
        real_size = size;
    }

    addr_end = addr + real_size;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock( );

    /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_OPTVERR );

    /* Get the number of pages available */
    nb_of_pages_max = get_page( FLASH_USER_END_ADDR ) - get_page( flash_user_start_addr ) + 1;

    if( ( flash_user_start_addr > addr ) || ( ( real_size / ADDR_FLASH_PAGE_SIZE ) > nb_of_pages_max ) )
    {
        status = FAIL;
        return status;
    }

    /* Program the user Flash area word by word
    (area defined by FlashUserStartAddr and FLASH_USER_END_ADDR) ***********/

    while( addr < addr_end )
    {
        data64 = 0;
        for( uint8_t i = 0; i < 8; i++ )
        {
            data64 += ( ( ( uint64_t ) buffer[buffer_index + i] ) << ( i * 8 ) );
        }

        do
        {
            hal_status = HAL_FLASH_Program( FLASH_TYPEPROGRAM_DOUBLEWORD, addr, data64 );
            flash_operation_retry++;
        } while( ( hal_status != HAL_OK ) && ( flash_operation_retry < FLASH_OPERATION_MAX_RETRY ) );

        if( flash_operation_retry >= FLASH_OPERATION_MAX_RETRY )
        {
            /* Error occurred while writing data in Flash memory.
            User can add here some code to deal with this error */
            /* Infinite loop */
            while( 1 )
            {
            }
        }
        else
        {
            flash_operation_retry = 0;
            /* increment to next double word*/
            addr         = addr + 8;
            buffer_index = buffer_index + 8;
        }
    }

    /* Lock the Flash to disable the flash control register access (recommended
    to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock( );

    return real_size;
}

void flash_read_buffer( uint32_t addr, uint8_t* buffer, uint32_t size )
{
    uint32_t flash_index = 0;
    __IO uint8_t data8   = 0;

    while( flash_index < size )
    {
        data8 = *( __IO uint32_t* ) ( addr + flash_index );

        buffer[flash_index] = data8;

        flash_index++;
    }
}

uint32_t flash_get_user_start_addr( void ) { return flash_user_start_addr; }

void flash_set_user_start_addr( uint32_t addr ) { flash_user_start_addr = addr; }

static uint32_t get_page( uint32_t addr ) { return ( addr - FLASH_BASE ) / FLASH_PAGE_SIZE; }

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
