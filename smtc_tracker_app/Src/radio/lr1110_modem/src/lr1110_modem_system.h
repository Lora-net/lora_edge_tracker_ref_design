/*!
 * \file      lr1110_modem_system.h
 *
 * \brief     System driver for LR1110 modem
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
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH S.A. BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __LR1110_MODEM_SYSTEM_H__
#define __LR1110_MODEM_SYSTEM_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>
#include "lr1110_modem_common.h"

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

/*!
 * \brief Regulator mode values
 */
typedef enum
{
    LR1110_MODEM_SYSTEM_REG_MODE_LDO  = 0x00,  //!< (Default)
    LR1110_MODEM_SYSTEM_REG_MODE_DCDC = 0x01,
} lr1110_modem_system_reg_mode_t;

/*!
 * \brief RF switch configuration pin
 */
enum lr1110_modem_system_rfswitch_cfg_pin_e
{
    LR1110_MODEM_SYSTEM_RFSW0_HIGH = ( 1 << 0 ),
    LR1110_MODEM_SYSTEM_RFSW1_HIGH = ( 1 << 1 ),
    LR1110_MODEM_SYSTEM_RFSW2_HIGH = ( 1 << 2 ),
    LR1110_MODEM_SYSTEM_RFSW3_HIGH = ( 1 << 3 ),
    LR1110_MODEM_SYSTEM_RFSW4_HIGH = ( 1 << 4 ),
};

/*!
 * \brief Low Frequency clock configurations values
 */
typedef enum
{
    LR1110_MODEM_SYSTEM_LFCLK_RC   = 0x00,  //!<  (Default)
    LR1110_MODEM_SYSTEM_LFCLK_XTAL = 0x01,
    LR1110_MODEM_SYSTEM_LFCLK_EXT  = 0x02
} lr1110_modem_system_lfclk_cfg_t;

/*!
 * \brief TCXO supply voltage values
 */
typedef enum
{
    LR1110_MODEM_SYSTEM_TCXO_CTRL_1_6V = 0x00,  //!< Supply voltage = 1.6v
    LR1110_MODEM_SYSTEM_TCXO_CTRL_1_7V = 0x01,  //!< Supply voltage = 1.7v
    LR1110_MODEM_SYSTEM_TCXO_CTRL_1_8V = 0x02,  //!< Supply voltage = 1.8v
    LR1110_MODEM_SYSTEM_TCXO_CTRL_2_2V = 0x03,  //!< Supply voltage = 2.2v
    LR1110_MODEM_SYSTEM_TCXO_CTRL_2_4V = 0x04,  //!< Supply voltage = 2.4v
    LR1110_MODEM_SYSTEM_TCXO_CTRL_2_7V = 0x05,  //!< Supply voltage = 2.7v
    LR1110_MODEM_SYSTEM_TCXO_CTRL_3_0V = 0x06,  //!< Supply voltage = 3.0v
    LR1110_MODEM_SYSTEM_TCXO_CTRL_3_3V = 0x07,  //!< Supply voltage = 3.3v
} lr1110_modem_system_tcxo_supply_voltage_t;

/*!
 * \brief RF switch configuration parameters
 */
typedef struct
{
    uint8_t enable;
    uint8_t standby;
    uint8_t rx;
    uint8_t tx;
    uint8_t tx_hp;
    uint8_t tx_hf;
    uint8_t gnss;
    uint8_t wifi;
} lr1110_modem_system_rf_switch_cfg_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Write words into auxiliary memory space of LR1110.
 *
 * The words are 32 bits long. The writing operations write contiguously in
 * auxiliary memory, starting at the address provided.
 *
 * \param [in] context Chip implementation context
 *
 * \param [in] address The auxiliary memory address to start writing operation
 *
 * \param [in] buffer The buffer of words to write into memory. Its size must be
 * enough to contain length words. A word is 32 bits long
 *
 * \param [in] length Number of words to write into memory
 *
 * \see lr1110_modem_system_read_auxreg32
 */
lr1110_modem_response_code_t lr1110_modem_system_write_auxreg32( const void* context, const uint32_t address,
                                                                 const uint32_t* buffer, const uint8_t length );

/*!
 * \brief Read words into auxiliary memory space of LR1110.
 *
 * The words are 32 bits long. The reading operations read contiguously from
 * auxiliary memory, starting at the address provided.
 *
 * \param [in] context Chip implementation context
 *
 * \param [in] address The auxiliary memory address to start reading operation
 *
 * \param [out] buffer Pointer to a words array to be filled with content from
 * memory. Its size must be enough to contain at least length words. A word is
 * 32 bits long
 * 
 * \param [in] length Number of words to read from memory
 *
 * \see lr1110_modem_system_write_auxreg32
 */
lr1110_modem_response_code_t lr1110_modem_system_read_auxreg32( const void* context, const uint32_t address,
                                                                uint32_t* buffer, const uint8_t length );

/*!
 * \brief Write words into register memory space of LR1110.
 *
 * The words are 32 bits long. The writing operations write contiguously in
 * register memory, starting at the address provided.
 *
 * \param [in] context Chip implementation context
 *
 * \param [in] address The register memory address to start writing operation
 *
 * \param [in] buffer The buffer of words to write into memory. Its size must be
 * enough to contain length words. A word is 32 bits long
 *
 * \param [in] length Number of words to write into memory
 *
 * \see lr1110_modem_system_read_regmem32
 */
lr1110_modem_response_code_t lr1110_modem_system_write_regmem32( const void* context, const uint32_t address,
                                                                 const uint32_t* buffer, const uint8_t length );

/*!
 * \brief Read words into register memory space of LR1110.
 *
 * The words are 32 bits long. The reading operations read contiguously from
 * register memory, starting at the address provided.
 *
 * \param [in] context Chip implementation context
 *
 * \param [in] address The register memory address to start reading operation
 *
 * \param [in] length Number of words to read from memory
 *
 * \param [out] buffer Pointer to a words array to be filled with content from
 * memory. Its size must be enough to contain at least length words. A word is
 * 32 bits long
 *
 * \see lr1110_modem_system_write_regmem32
 */
lr1110_modem_response_code_t lr1110_modem_system_read_regmem32( const void* context, const uint32_t address,
                                                                uint32_t* buffer, const uint8_t length );

/*!
 * \brief Configure the regulator mode to be used in specific modes
 *
 * This function shall only be called in standby RC mode.
 *
 * The reg_mode parameter defines if the DC-DC converter is switched on in the
 * following modes: STANDBY XOSC, FS, RX, TX and RX_CAPTURE.
 *
 * \param [in] context Chip implementation context
 *
 * \param [in] reg_mode Regulator mode configuration
 */
lr1110_modem_response_code_t lr1110_modem_system_set_reg_mode( const void*                          context,
                                                               const lr1110_modem_system_reg_mode_t reg_mode );

/*!
 * \brief Set the RF switch configurations for each RF setup
 *
 * This function shall only be called in standby RC mode.
 *
 * By default, no DIO is used to control a RF switch. All DIOs are set in
 * High-Z mode.
 *
 * \param [in] context Chip implementation context
 *
 * \param [in] rf_switch_cfg Pointer to a structure that holds the switches
 * configuration
 */
lr1110_modem_response_code_t lr1110_modem_system_set_dio_as_rf_switch(
    const void* context, const lr1110_modem_system_rf_switch_cfg_t* rf_switch_cfg );

/*!
 * \brief Defines which clock is used as Low Frequency (LF) clock
 *
 * \param [in] context Chip implementation context
 *
 * \param [in] lfclock_cfg Low frequency clock configuration
 *
 * \param [in] wait_for_32k_ready
 */
lr1110_modem_response_code_t lr1110_modem_system_cfg_lfclk( const void*                              context,
                                                               const lr1110_modem_system_lfclk_cfg_t lfclock_cfg,
                                                               const bool wait_for_32k_ready );

/*!
 * \brief Enable and configure TCXO supply voltage and detection timeout
 *
 * This function shall only be called in standby RC mode.
 *
 * The timeout parameter is the maximum time the firmware waits for the TCXO
 * to be ready. The timeout duration is given by:
 * \f$ timeout\_duration\_us = timeout \times 30.52 \f$
 *
 * The TCXO mode can be disabled by setting timeout parameter to 0.
 *
 * \param [in] context Chip implementation context
 *
 * \param [in] tune Supply voltage value
 *
 * \param [in] timeout
 */
lr1110_modem_response_code_t lr1110_modem_system_set_tcxo_mode( const void*                                     context,
                                                                const lr1110_modem_system_tcxo_supply_voltage_t tune,
                                                                const uint32_t timeout );

/*!
 * \brief Software reset of the chip.
 *
 * This function should be used to reboot the chip in a specified mode.
 * Rebooting in flash mode presumes that the content in flash memory is not
 * corrupted (i.e. the integrity check performed by the bootloader before
 * executing the first instruction in flash is OK).
 *
 * \param [in] context context abstraction
 *
 * \param [in] stay_in_bootloader Selector to stay in bootloader or execute
 * flash code after reboot. If true, the bootloader will not execute the flash
 * code but activate SPI interface to allow firmware upgrade
 */
void lr1110_modem_system_reboot( const void* context, const bool stay_in_bootloader );

#ifdef __cplusplus
}
#endif

#endif  // __LR1110_MODEM_SYSTEM_H__

/* --- EOF ------------------------------------------------------------------ */
