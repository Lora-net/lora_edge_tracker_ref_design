/*!
 * @file      smtc_hal_mcu.h
 *
 * @brief     Board specific package MCU API definition.
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
#ifndef SMTC_HAL_MCU_H
#define SMTC_HAL_MCU_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*!
 * @brief Begins critical section \note this call adds a mask variable in the context
 */
#define CRITICAL_SECTION_BEGIN( ) \
    uint32_t mask;                \
    hal_mcu_critical_section_begin( &mask )

/*!
 * @brief Ends critical section \note this shall be called in the same context as previous CRITICAL_SECTION_BEGIN( )
 */
#define CRITICAL_SECTION_END( ) hal_mcu_critical_section_end( &mask )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Disable interrupts, begins critical section
 *
 * @param [in] mask Pointer to a variable where to store the CPU IRQ mask
 */
void hal_mcu_critical_section_begin( uint32_t* mask );

/*!
 * @brief Ends critical section
 *
 * @param [in] mask Pointer to a variable where the CPU IRQ mask was stored
 */
void hal_mcu_critical_section_end( uint32_t* mask );

/*!
 * @brief Initializes HAL used MCU
 */
void hal_mcu_init( void );

/*!
 * @brief Initializes MCU after a stop mode
 */
void hal_mcu_reinit( void );

/*!
 * @brief Initializes HAL used Peripherals
 */
void hal_mcu_init_periph( void );

/*!
 * @brief Disable irq at core side
 */
void hal_mcu_disable_irq( void );

/*!
 * @brief Enable irq at core side
 */
void hal_mcu_enable_irq( void );

/*!
 * @brief Resets the MCU
 */
void hal_mcu_reset( void );

/*!
 * @brief To be called in case of panic on mcu side
 */
void hal_mcu_panic( void );

/*!
 * @brief Initializes HAL used MCU radio pins
 *
 * @param [in] context Pointer to a variable holding the communication interface
 *                     id as well as the radio pins assignment.
 */
void hal_mcu_init_radio( const void* context );

/*!
 * @brief Sets the MCU in sleep mode for the given number of seconds.
 *
 * @param [in] seconds Number of seconds to stay in sleep mode
 */
void hal_mcu_set_sleep_for_s( const int32_t seconds );

/*!
 * @brief Waits for delay microseconds
 *
 * @param [in] microseconds Delay to wait in microseconds
 */
void hal_mcu_wait_us( const int32_t microseconds );

/*!
 * @brief Get Vref intern from the MCU in mV
 *
 * @returns Vref In in mV.
 */
uint16_t hal_mcu_get_vref_level( void );

/*!
 * @brief Get Temperature intern from the MCU in Celsius degree
 *
 * @returns Temperature in Celsius degree.
 */
int16_t hal_mcu_get_temperature( void );

/*!
 * @brief Activate the forward of the LSE on RCO pin
 *
 * @param enable Enable or disable the LSCO
 */
void hal_mcu_system_clock_forward_LSE( bool enable );

/*!
 * @brief Activate the SMPS
 *
 * @param enable Enable or disable the SMPS
 */
void hal_mcu_smps_enable( bool enable );

/*!
 * @brief Prints debug trace
 *
 * @param variadics arguments
 */
void hal_mcu_trace_print( const char* fmt, ... );

/*!
 * @brief Suspend low power process and avoid looping on it
 */
void hal_mcu_disable_low_power_wait( void );

/*!
 * @brief Enable low power process
 */
void hal_mcu_enable_low_power_wait( void );

/*!
 * @brief Suspend once low power process and avoid looping on it once
 */
void hal_mcu_disable_once_low_power_wait( void );

/*!
 * @brief Enable/Disable partial sleep
 */
void hal_mcu_partial_sleep_enable( bool enable );

/*!
 * @brief Enter in low power state
 */
void hal_mcu_low_power_handler( void );

/*!
 * @brief Init the software watchdog
 *
 * @param value value in ms of the watchdog
 */
void hal_mcu_init_software_watchdog( uint32_t value );

/*!
 * @brief Set the reload value of the software watchdog
 *
 * @param value value in ms of the watchdog
 */
void hal_mcu_set_software_watchdog_value( uint32_t value );

/*!
 * @brief Start the software watchdog
 */
void hal_mcu_start_software_watchdog( void );

/*!
 * @brief Reset the software watchdog
 */
void hal_mcu_reset_software_watchdog( void );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_HAL_MCU_H

/* --- EOF ------------------------------------------------------------------ */
