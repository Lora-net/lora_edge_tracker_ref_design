/*!
 * @file      board-config.h
 *
 * @brief     board specific pinout
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

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*!
 * @brief Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME 5

/*!
 * @brief Board MCU pins definitions
 */

/* LR1110 */
#define RADIO_RESET PA_0
#define RADIO_MOSI PA_7
#define RADIO_MISO PA_6
#define RADIO_SCLK PA_5
#define RADIO_NSS PA_4
#define RADIO_BUSY PB_0
#define RADIO_EVENT PB_4

#define VCC_SWITCH_WIFI_BLE PA_3
#define SWITCH_WIFI_BLE PA_15
#define GPS_SWITCH PB_8
#define LNA_PON PA_8

/* Sensors */
#define I2C_SCL PB_6
#define I2C_SDA PB_7

#define VCC_SENSORS_MCU PA_1

#define EFFECT_HALL_OUT PB_2
#define ACC_INT1 PB_1
#define USER_BUTTON PB_9

/* LED */
#define LED_RX PE_4
#define LED_TX PB_5

/* USB */
#define USB_DP PA_12
#define USB_DM PA_11

/* Debug pins definition */
#define BOARD_DBG_PIN_TX PA_9
#define BOARD_DBG_PIN_RX PA_10

#define OSC_LSE_IN PC_14
#define OSC_LSE_OUT PC_15

#define OSC_HSE_IN PH_0
#define OSC_HSE_OUT PH_1

#define RCC_LSCO

#define BOARD_SWO PB_3
#define SWCLK PA_14
#define SWDAT PA_13

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // BOARD_CONFIG_H

/* --- EOF ------------------------------------------------------------------ */
