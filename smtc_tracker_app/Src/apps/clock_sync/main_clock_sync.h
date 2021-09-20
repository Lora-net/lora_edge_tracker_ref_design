/*!
 * @ingroup   apps_clock_sync
 * @file      main_clock_sync.h
 *
 * @brief     LR1110 Simple LoRaWAN clock synchronization example configuration
 *
 * @copyright
 * @parblock
 * Revised BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
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
 * @endparblock
 */

/*!
 * @addtogroup apps_clock_sync
 * LR1110 Simple LoRaWAN clock synchronization example
 * @{
 */

#ifndef MAIN_CLOCK_SYNC_H
#define MAIN_CLOCK_SYNC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- Application Configuration -----------------------------------------------
 */

/*!
 * @brief Defines the delay in milliseconds between two time displays..
 */
#define APP_GETTIME_PERIOD_MS 20000

/*
 * -----------------------------------------------------------------------------
 * --- LoRaWAN Configuration ---------------------------------------------------
 */

/*!
 * @brief LoRaWAN regulatory region.
 * One of:
 * LR1110_LORAWAN_REGION_EU868
 * LR1110_LORAWAN_REGION_US915
 * LR1110_LORAWAN_REGION_AU915
 * LR1110_LORAWAN_REGION_AS923_GRP1
 * LR1110_LORAWAN_REGION_CN470
 * LR1110_LORAWAN_REGION_AS923_GRP2
 * LR1110_LORAWAN_REGION_AS923_GRP3
 * LR1110_LORAWAN_REGION_IN865
 * LR1110_LORAWAN_REGION_KR920
 * LR1110_LORAWAN_REGION_RU864
 */
#define LORAWAN_REGION_USED LR1110_LORAWAN_REGION_EU868

/*!
 * @brief LoRaWAN regulatory region country. define LoRaWAN subregion countries to activate or not the LBT, 0 means
 * disable, 1 means enable
 */
#define LORAWAN_COUNTRY_JAPAN 0

/*!
 * @brief LoRaWAN class.
 * One of:
 *  LR1110_LORAWAN_CLASS_A
 *  LR1110_LORAWAN_CLASS_C
 */
#define LORAWAN_CLASS_USED LR1110_LORAWAN_CLASS_A

/*!
 * @brief LoRaWAN ETSI duty cycle control enable/disable
 * Supported values:
 *  LR1110_MODEM_DUTY_CYCLE_ENABLE
 *  LR1110_MODEM_DUTY_CYCLE_DISABLE
 *
 * @remark Please note that ETSI mandates duty cycled transmissions. Set to false only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON LR1110_MODEM_DUTY_CYCLE_ENABLE

/*!
 * @brief Default datarate
 * Supported values:
 *  LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED
 *  LR1110_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE
 *  LR1110_MODEM_ADR_PROFILE_MOBILE_LOW_POWER
 *  LR1110_MODEM_ADR_PROFILE_CUSTOM
 */
#define LORAWAN_DEFAULT_DATARATE LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED

/*!
 * @brief ADR custom list when LORAWAN_DEFAULT_DATARATE is set to LR1110_MODEM_ADR_PROFILE_CUSTOM
 */
uint8_t adr_custom_list[16] = { 0x05, 0x05, 0x05, 0x04, 0x04, 0x04, 0x03, 0x03,
                                0x03, 0x02, 0x02, 0x02, 0x01, 0x01, 0x00, 0x00 };

#ifdef __cplusplus
}
#endif

#endif /* MAIN_CLOCK_SYNC_H */

/*!
 * @}
 */

/* --- EOF ------------------------------------------------------------------ */
