/*!
 * \file      utilities.h
 *
 * \brief     Helper functions implementation
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
#ifndef __UTILITIES_H__
#define __UTILITIES_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*!
 * \brief Returns 2 raised to the power of n
 *
 * \param [in] n power value
 * \retval result of raising 2 to the power n
 */
#define POW2( n ) ( 1 << n )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS 1
#endif

#ifndef FAIL
#define FAIL 0
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Initializes the pseudo random generator initial value
 *
 * \param [in] seed Pseudo random generator initial value
 */
void srand1( uint32_t seed );

/*!
 * \brief Computes a random number
 * \retval random random value
 */
int32_t rand1( void );

/*!
 * \brief Computes a random number between min and max
 *
 * \param [in] min range minimum value
 * \param [in] max range maximum value
 * \retval random random value in range min..max
 */
int32_t randr( int32_t min, int32_t max );

/*!
 * \brief Copies size elements of src array to dst array
 *
 * \remark STM32 Standard memcpy function only works on pointers that are aligned
 *
 * \param [out] dst  Destination array
 * \param [in]  src  Source array
 * \param [in]  size Number of bytes to be copied
 */
void memcpy1( uint8_t* dst, const uint8_t* src, uint16_t size );

/*!
 * \brief Copies size elements of src array to dst array reversing the byte order
 *
 * \param [out] dst  Destination array
 * \param [in]  src  Source array
 * \param [in]  size Number of bytes to be copied
 */
void memcpyr( uint8_t* dst, const uint8_t* src, uint16_t size );

/*!
 * \brief Set size elements of dst array with value
 *
 * \remark STM32 Standard memset function only works on pointers that are aligned
 *
 * \param [out] dst   Destination array
 * \param [in]  value Default value
 * \param [in]  size  Number of bytes to be copied
 */
void memset1( uint8_t* dst, uint8_t value, uint16_t size );

/*!
 * \brief Converts a nibble to an hexadecimal character
 *
 * \param [in] a   Nibble to be converted
 * \retval hexChar Converted hexadecimal character
 */
int8_t Nibble2HexChar( uint8_t a );

#ifdef __cplusplus
}
#endif

#endif  // __UTILITIES_H__

/* --- EOF ------------------------------------------------------------------ */
