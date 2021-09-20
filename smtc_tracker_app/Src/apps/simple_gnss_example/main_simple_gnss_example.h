/*!
 * @ingroup   simple_gnss_example
 * @file      main_simple_gnss_example.h
 *
 * @brief     GNSS Example configuration
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

#ifndef MAIN_TEST_GNSS_H
#define MAIN_TEST_GNSS_H

#include "lr1110_modem_gnss.h"

/*!
 * @addtogroup simple_gnss_example
 * LR1110 Simple GNSS example
 * @{
 */

/*
 * -----------------------------------------------------------------------------
 * --- Application Configuration -----------------------------------------------
 */

/*!
 * @brief GNSS scanning interval, in milliseconds.
 */
#define GNSS_SCAN_PERIOD_MS 10000

/*!
 * @brief Define and set to a non-zero value to echo input characters on the console output.
 */
#define CONSOLE_INPUT_ECHO 1

/*
 * -----------------------------------------------------------------------------
 * --- GNSS Configuration ------------------------------------------------------
 */

/*!
 * @brief GNSS Scan type
 *
 * Supported values:
 *  ASSISTED_MODE
 *  AUTONOMOUS_MODE
 *
 * Assisted mode arrives faster at a solution but needs an estimate of the current
 *  time and position to start with.
 */
#define GNSS_SCAN_TYPE ASSISTED_MODE

/*!
 * @brief Approximated device position latitude when using assist mode, in degrees.
 */
#define GNSS_ASSIST_LATITUDE 45.208

/*!
 * @brief Approximated device position longitude when using assist mode, in degrees.
 */
#define GNSS_ASSIST_LONGITUDE 5.781

/*!
 * @brief GNSS constellation(s) to use.
 *
 * Supported values:
 *  LR1110_MODEM_GNSS_GPS_MASK
 *  LR1110_MODEM_GNSS_BEIDOU_MASK
 */
#define GNSS_CONSTELLATION ( LR1110_MODEM_GNSS_GPS_MASK | LR1110_MODEM_GNSS_BEIDOU_MASK )

/*!
 * @}
 */

#endif /* MAIN_TEST_GNSS_H */

/* --- EOF ------------------------------------------------------------------ */
