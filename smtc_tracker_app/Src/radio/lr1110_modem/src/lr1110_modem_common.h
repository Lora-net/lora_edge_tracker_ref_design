/*!
 * @file      lr1110_modem_common.h
 *
 * @brief     modem driver common definition for LR1110
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

#ifndef LR1110_MODEM_COMMON_H
#define LR1110_MODEM_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define LR1110_MODEM_EVENT_RESET_COUNT_DATA_LENGTH ( 2 )
#define LR1110_MODEM_EVENT_TX_DONE_STATUS_DATA_LENGTH ( 2 )
#define LR1110_MODEM_EVENT_DOWN_DATA_DATA_LENGTH ( 2 )
#define LR1110_MODEM_EVENT_UPLOAD_DONE_DATA_LENGTH ( 2 )
#define LR1110_MODEM_EVENT_SET_CONF_DATA_LENGTH ( 2 )
#define LR1110_MODEM_EVENT_LINK_STATUS_DATA_LENGTH ( 2 )

#define LR1110_MODEM_EVENT_MAX_LENGTH_BUFFER ( 1024 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef enum
{
    LR1110_MODEM_GROUP_ID_SYSTEM = 0x01,
    LR1110_MODEM_GROUP_ID_WIFI   = 0x03,
    LR1110_MODEM_GROUP_ID_GNSS   = 0x04,
    LR1110_MODEM_GROUP_ID_CRYPTO = 0x05,
    LR1110_MODEM_GROUP_ID_MODEM  = 0x06,
} lr1110_modem_api_group_id_t;

typedef enum
{
    LR1110_MODEM_RESPONSE_CODE_OK                  = 0x00,
    LR1110_MODEM_RESPONSE_CODE_UNKOWN              = 0x01,
    LR1110_MODEM_RESPONSE_CODE_NOT_IMPLEMENTED     = 0x02,
    LR1110_MODEM_RESPONSE_CODE_NOT_INITIALIZED     = 0x03,
    LR1110_MODEM_RESPONSE_CODE_INVALID             = 0x04,
    LR1110_MODEM_RESPONSE_CODE_BUSY                = 0x05,
    LR1110_MODEM_RESPONSE_CODE_FAIL                = 0x06,
    LR1110_MODEM_RESPONSE_CODE_BAD_FORMAT          = 0x07,
    LR1110_MODEM_RESPONSE_CODE_BAD_FILE_UPLOAD_CRC = 0x08,
    LR1110_MODEM_RESPONSE_CODE_BAD_SIGNATURE       = 0x09,
    LR1110_MODEM_RESPONSE_CODE_BAD_SIZE            = 0x0A,
    LR1110_MODEM_RESPONSE_CODE_BAD_FRAME           = 0x0F,
    LR1110_MODEM_RESPONSE_CODE_NO_TIME             = 0x10,
} lr1110_modem_response_code_t;

/*!
 * @brief Event type for modem operation
 */
typedef enum
{
    LR1110_MODEM_LORAWAN_EVENT_RESET                 = 0x00,
    LR1110_MODEM_LORAWAN_EVENT_ALARM                 = 0x01,
    LR1110_MODEM_LORAWAN_EVENT_JOINED                = 0x02,
    LR1110_MODEM_LORAWAN_EVENT_TX_DONE               = 0x03,
    LR1110_MODEM_LORAWAN_EVENT_DOWN_DATA             = 0x04,
    LR1110_MODEM_LORAWAN_EVENT_UPLOAD_DONE           = 0x05,
    LR1110_MODEM_LORAWAN_EVENT_SET_CONF              = 0x06,
    LR1110_MODEM_LORAWAN_EVENT_MUTE                  = 0x07,
    LR1110_MODEM_LORAWAN_EVENT_STREAM_DONE           = 0x08,
    LR1110_MODEM_LORAWAN_EVENT_JOIN_FAIL             = 0x0A,
    LR1110_MODEM_LORAWAN_EVENT_WIFI_SCAN_DONE        = 0x0B,
    LR1110_MODEM_LORAWAN_EVENT_GNSS_SCAN_DONE        = 0x0C,
    LR1110_MODEM_LORAWAN_EVENT_TIME_UPDATED_ALC_SYNC = 0x0D,
    LR1110_MODEM_LORAWAN_EVENT_ADR_MOBILE_TO_STATIC  = 0x0E,
    LR1110_MODEM_LORAWAN_EVENT_NEW_LINK_ADR          = 0x0F,
    LR1110_MODEM_LORAWAN_EVENT_NO_EVENT              = 0xFF,
} lr1110_modem_lorawan_event_type_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR1110_MODEM_COMMON_H

/* --- EOF ------------------------------------------------------------------ */
