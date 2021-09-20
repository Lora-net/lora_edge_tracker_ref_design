/*!
 * @file      smtc_hal_uart.h
 *
 * @brief     Board specific package UART API definition.
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
#ifndef SMTC_HAL_UART_H
#define SMTC_HAL_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "stm32wbxx_hal.h"
#include "smtc_hal_gpio_pin_names.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

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
 * @brief Initializes the MCU UART peripheral
 *
 * @param [in] id UART interface id [1:N]
 * @param [in] uart_tx UART TX pin name to be used
 * @param [in] uart_rx UART RX pin name to be used
 */
void hal_uart_init( const uint32_t id, const hal_gpio_pin_names_t uart_tx, const hal_gpio_pin_names_t uart_rx );

/*!
 * @brief Deinitializes the MCU UART peripheral
 *
 * @param [in] id UART interface id [1:N]
 */
void hal_uart_deinit( const uint32_t id );

/*!
 * @brief Send an amount on data on the UART bus
 *
 * @param [in] id UART interface id [1:N]
 * @param [in] buff buffer containing data to send
 * @param [in] len data length to send
 */
void hal_uart_tx( const uint32_t id, uint8_t* buff, uint16_t len );

/*!
 * @brief Receive an amount on data on the UART bus
 *
 * @param [in] id UART interface id [1:N]
 * @param [in] rx_buffer buffer receiving data
 * @param [in] len data length to receive
 */
void hal_uart_rx( const uint32_t id, uint8_t* rx_buffer, uint16_t len );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_HAL_UART_H

/* --- EOF ------------------------------------------------------------------ */
