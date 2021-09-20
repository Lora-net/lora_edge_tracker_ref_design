/*!
 * @ingroup   simple_wifi_example
 * @file      main_test_wifi.h
 *
 * @brief     Wi-Fi Example configuration
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
 * @addtogroup simple_wifi_example
 * LR1110 Simple Wi-Fi test application
 * @{
 */

#ifndef MAIN_TEST_WIFI_H
#define MAIN_TEST_WIFI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "wifi_scan.h"

/*
 * -----------------------------------------------------------------------------
 * --- Application Configuration -----------------------------------------------
 */

/*!
 * @brief Wi-Fi scanning interval, in milliseconds.
 */
#define WIFI_SCAN_PERIOD_MS 10000

/*
 * -----------------------------------------------------------------------------
 * --- Wi-Fi Configuration -----------------------------------------------------
 */

/*!
 * @brief Wi-Fi signal type for passive scanning.
 */
#define WIFI_TYPE_SCAN LR1110_MODEM_WIFI_TYPE_SCAN_B

/*!
 * @brief Wi-Fi capture mode
 */
#define WIFI_SCAN_MODE LR1110_MODEM_WIFI_SCAN_MODE_BEACON_AND_PKT

/*!
 * @brief The maximal duration of a single preamble search. Expressed in ms. Range of allowed values
 * is [1:65535].
 * @note Value 0 is forbidden.
 */
#define WIFI_TIMEOUT_IN_MS WIFI_TIMEOUT_IN_MS_DEFAULT

/*!
 * @brief The number of internal scan sequences per channel scanned. Range of accepted values
 * is [1:255].
 * @note Value 0 is forbidden.
 */
#define WIFI_NBR_RETRIALS WIFI_NBR_RETRIALS_DEFAULT

/*!
 * @brief The maximal number of results to gather. When this limit is reached, the passive scan
 * automatically stops. Range of allowed values is [1:32].
 * @note Value 0 is forbidden.
 */
#define WIFI_MAX_RESULTS WIFI_MAX_RESULTS_DEFAULT

#ifdef __cplusplus
}
#endif

#endif /* MAIN_TEST_WIFI_H */

/*!
 * @}
 */

/* --- EOF ------------------------------------------------------------------ */
