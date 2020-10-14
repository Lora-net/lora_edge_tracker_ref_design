/*!
 * \file      smtc_hal_gpio.h
 *
 * \brief     Board specific package GPIO API definition.
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
#ifndef __SMTC_HAL_GPIO_H__
#define __SMTC_HAL_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif
/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

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

/*!
 * \brief GPIO setup data structure
 */
typedef struct hal_gpio_s
{
    hal_gpio_pin_names_t pin;
    uint32_t             mode;
    uint32_t             pull;
    uint32_t             speed;
    uint32_t             alternate;
} hal_gpio_t;

/*!
 * \brief GPIO IRQ data context
 */
typedef struct hal_gpio_irq_s
{
    hal_gpio_pin_names_t pin;
    void*                context;
    void ( *callback )( void* context );
} hal_gpio_irq_t;

/*!
 * \brief GPIO Pull modes
 */
typedef enum gpio_pull_mode_e
{
    HAL_GPIO_PULL_MODE_NONE = 0,
    HAL_GPIO_PULL_MODE_UP   = 1,
    HAL_GPIO_PULL_MODE_DOWN = 2,
} gpio_pull_mode_t;

/*!
 * \brief GPIO IRQ modes
 */
typedef enum gpio_irq_mode_e
{
    HAL_GPIO_IRQ_MODE_OFF            = 0,
    HAL_GPIO_IRQ_MODE_RISING         = 1,
    HAL_GPIO_IRQ_MODE_FALLING        = 2,
    HAL_GPIO_IRQ_MODE_RISING_FALLING = 3,
} gpio_irq_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Initializes given pin as output with given initial value
 *
 * \param [in] pin   MCU pin to be initialized
 * \param [in] value MCU initial pit state
 *
 */
void hal_gpio_init_out( const hal_gpio_pin_names_t pin, const uint32_t value );

/*!
 * \brief Deinitializes given pin
 *
 * \param [in] pin   MCU pin to be deinitialized
 *
 */
void hal_gpio_deinit( const hal_gpio_pin_names_t pin );

/*!
 * \brief Initializes given pin as input
 *
 * \param [in] pin MCU pin to be initialized
 * \param [in] pull_mode MCU pin pull mode [HAL_GPIO_PULL_MODE_NONE,
 *                                          HAL_GPIO_PULL_MODE_UP,
 *                                          HAL_GPIO_PULL_MODE_DOWN]
 * \param [in] irq_mode MCU IRQ mode [HAL_GPIO_IRQ_MODE_OFF,
 *                                     HAL_GPIO_IRQ_MODE_RISING,
 *                                     HAL_GPIO_IRQ_MODE_FALLING,
 *                                     HAL_GPIO_IRQ_MODE_RISING_FALLING]
 * \param [in] irq Pointer to IRQ data context.
 *                 NULL when HAL_GPIO_IRQ_MODE_OFF
 *                 pin parameter is initialized
 */
void hal_gpio_init_in( const hal_gpio_pin_names_t pin, const gpio_pull_mode_t pull_mode, const gpio_irq_mode_t irq_mode,
                       hal_gpio_irq_t* irq );

/*!
 * \brief Attaches given callback to the MCU IRQ handler
 *
 * \param [in] irq Pointer to IRQ data context
 */
void hal_gpio_irq_attach( const hal_gpio_irq_t* irq );

/*!
 * \brief Detattaches callback from the MCU IRQ handler
 *
 * \param [in] irq     Pointer to IRQ data context
 */
void hal_gpio_irq_deatach( const hal_gpio_irq_t* irq );

/*!
 * \brief Enables all GPIO MCU interrupts
 */
void hal_gpio_irq_enable( void );

/*!
 * \brief Disables all GPIO MCU interrupts
 */
void hal_gpio_irq_disable( void );

/*!
 * \brief Sets MCU pin to given value
 *
 * \param [in] pin   MCU pin to be set
 * \param [in] value MCU pin state to be set
 */
void hal_gpio_set_value( const hal_gpio_pin_names_t pin, const uint32_t value );

/*!
 * \brief Toggles MCU pin state value
 *
 * \param [in] pin   MCU pin to be toggled
 */
void hal_gpio_toggle( const hal_gpio_pin_names_t pin );

/*!
 * \brief Gets MCU pin state value
 *
 * \param [in] pin   MCU pin to be read
 *
 * \retval value Current MCU pin state
 */
uint32_t hal_gpio_get_value( const hal_gpio_pin_names_t pin );

/*!
 * \brief Indicates if there are gpio IRQs pending.
 *
 * \retval pendig [true: IRQ pending
 *                 false: No IRQ pending]
 */
bool hal_gpio_is_pending_irq( void );

/*!
 * \brief EXTI IRQ Handler.
 */
void EXTI0_IRQHandler( void );
void EXTI1_IRQHandler( void );
void EXTI2_IRQHandler( void );
void EXTI3_IRQHandler( void );
void EXTI4_IRQHandler( void );
void EXTI9_5_IRQHandler( void );
void EXTI15_10_IRQHandler( void );

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_HAL_GPIO_H__
