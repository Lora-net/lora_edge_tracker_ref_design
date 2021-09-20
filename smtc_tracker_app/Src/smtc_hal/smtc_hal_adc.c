/*!
 * @file      smtc_hal_adc.c
 *
 * @brief     Board specific package ADC API implementation
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "stm32wbxx_hal.h"
#include "smtc_hal_mcu.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct hal_adc_s
{
    ADC_TypeDef*           interface;
    ADC_HandleTypeDef      handle;
    DMA_HandleTypeDef      hdma_adc1;
    DMA_HandleTypeDef*     DMA_Handle;
    ADC_ChannelConfTypeDef channel;
} hal_adc_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static hal_adc_t hal_adc;

uint16_t adc_converted_data[2];   /* ADC group regular conversion data (array of data) */
uint8_t  dma_transfer_status = 0; /* Variable set into DMA interruption callback */
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief  Conversion complete callback in non blocking mode
 * @param  hadc: ADC handle
 * @note   This example shows a simple way to report end of conversion
 *         and get conversion result. You can add your own implementation.
 * @retval None
 */
void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef* hadc );

/**
 * @brief  Conversion DMA half-transfer callback in non blocking mode
 * @note   This example shows a simple way to report end of conversion
 *         and get conversion result. You can add your own implementation.
 * @retval None
 */
void HAL_ADC_ConvHalfCpltCallback( ADC_HandleTypeDef* hadc );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_adc_init( void )
{
    /* Enable DMA controller clock */

    /* DMA controller clock enable */
    __HAL_RCC_DMAMUX1_CLK_ENABLE( );
    __HAL_RCC_DMA1_CLK_ENABLE( );

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( DMA1_Channel1_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( DMA1_Channel1_IRQn );

    hal_adc.handle.Instance = ADC1;

    hal_adc.handle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
    hal_adc.handle.Init.Resolution            = ADC_RESOLUTION_12B;
    hal_adc.handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hal_adc.handle.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hal_adc.handle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hal_adc.handle.Init.LowPowerAutoWait      = DISABLE;
    hal_adc.handle.Init.ContinuousConvMode    = DISABLE;
    hal_adc.handle.Init.NbrOfConversion       = 2;
    hal_adc.handle.Init.DiscontinuousConvMode = ENABLE;
    hal_adc.handle.Init.NbrOfDiscConversion   = 1;
    hal_adc.handle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hal_adc.handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hal_adc.handle.Init.DMAContinuousRequests = DISABLE;
    hal_adc.handle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    hal_adc.handle.Init.OversamplingMode      = DISABLE;

    if( HAL_ADC_Init( &hal_adc.handle ) != HAL_OK )
    {
        hal_mcu_panic( );
    }

    hal_adc.channel.SingleDiff   = ADC_SINGLE_ENDED;
    hal_adc.channel.OffsetNumber = ADC_OFFSET_NONE;
    hal_adc.channel.Offset       = 0;
    hal_adc.channel.Channel      = ADC_CHANNEL_VREFINT;
    hal_adc.channel.Rank         = ADC_REGULAR_RANK_1;
    hal_adc.channel.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;

    if( HAL_ADC_ConfigChannel( &hal_adc.handle, &hal_adc.channel ) != HAL_OK )
    {
        hal_mcu_panic( );
    }

    hal_adc.channel.Channel      = ADC_CHANNEL_TEMPSENSOR;
    hal_adc.channel.Rank         = ADC_REGULAR_RANK_2;
    hal_adc.channel.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
    if( HAL_ADC_ConfigChannel( &hal_adc.handle, &hal_adc.channel ) != HAL_OK )
    {
        hal_mcu_panic( );
    }

    /* Run the ADC calibration in single-ended mode */
    if( HAL_ADCEx_Calibration_Start( &hal_adc.handle, ADC_SINGLE_ENDED ) != HAL_OK )
    {
        /* Calibration Error */
        hal_mcu_panic( );
    }

    /*## Start ADC conversions ###############################################*/
    /* Start ADC group regular conversion with DMA */
    if( HAL_ADC_Start_DMA( &hal_adc.handle, ( uint32_t* ) adc_converted_data, 2 ) != HAL_OK )
    {
        /* ADC conversion start error */
        hal_mcu_panic( );
    }
}

uint16_t hal_adc_get_vref_int( void )
{
    uint16_t vref_analog_mvolt = 0U;

    /* Check whether ADC has converted all ranks of the sequence */
    while( vref_analog_mvolt == 0 )
    {
        if( HAL_ADC_Start( &hal_adc.handle ) != HAL_OK )
        {
            hal_mcu_panic( );
        }

        /* Wait for ADC conversion and DMA transfer completion (update of variable ubDmaTransferStatus) */
        HAL_Delay( 1 );

        if( dma_transfer_status == 1 )
        {
            dma_transfer_status = 0;
            vref_analog_mvolt   = __LL_ADC_CALC_VREFANALOG_VOLTAGE( adc_converted_data[0], LL_ADC_RESOLUTION_12B );
        }
    }
    /* Read ADC Value */
    return vref_analog_mvolt;
}

int16_t hal_adc_get_temperature( uint16_t vdda_appli )
{
    int16_t temperature_degree_celsius = 0U;

    /* Check whether ADC has converted all ranks of the sequence */
    while( temperature_degree_celsius == 0 )
    {
        if( HAL_ADC_Start( &hal_adc.handle ) != HAL_OK )
        {
            hal_mcu_panic( );
        }

        /* Wait for ADC conversion and DMA transfer completion (update of variable ubDmaTransferStatus) */
        HAL_Delay( 1 );

        if( dma_transfer_status == 1 )
        {
            dma_transfer_status = 0;
            temperature_degree_celsius =
                __LL_ADC_CALC_TEMPERATURE( vdda_appli, adc_converted_data[1], LL_ADC_RESOLUTION_12B );
        }
    }
    /* Read ADC Value */
    return temperature_degree_celsius;
}

void hal_adc_deinit( void ) { HAL_ADC_DeInit( &hal_adc.handle ); }

void HAL_ADC_MspInit( ADC_HandleTypeDef* adc_handle )
{
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection    = RCC_ADCCLKSOURCE_SYSCLK;
    if( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
    {
        hal_mcu_panic( );
    }

    /* Peripheral clock enable */
    __HAL_RCC_ADC_CLK_ENABLE( );

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hal_adc.hdma_adc1.Instance                 = DMA1_Channel1;
    hal_adc.hdma_adc1.Init.Request             = DMA_REQUEST_ADC1;
    hal_adc.hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hal_adc.hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hal_adc.hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    hal_adc.hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hal_adc.hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hal_adc.hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    hal_adc.hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
    if( HAL_DMA_Init( &hal_adc.hdma_adc1 ) != HAL_OK )
    {
        hal_mcu_panic( );
    }

    __HAL_LINKDMA( &hal_adc.handle, DMA_Handle, hal_adc.hdma_adc1 );

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority( ADC1_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( ADC1_IRQn );
}

void HAL_ADC_MspDeInit( ADC_HandleTypeDef* adc_handle )
{
    /* Peripheral clock disable */
    __HAL_RCC_ADC_CLK_DISABLE( );

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit( hal_adc.DMA_Handle );

    /* ADC1 interrupt DeInit */
    HAL_NVIC_DisableIRQ( ADC1_IRQn );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/**
 * @brief This function handles DMA1 channel1 global interrupt.
 */
void DMA1_Channel1_IRQHandler( void ) { HAL_DMA_IRQHandler( &hal_adc.hdma_adc1 ); }

/**
 * @brief This function handles ADC1 global interrupt.
 */
void ADC1_IRQHandler( void ) { HAL_ADC_IRQHandler( &hal_adc.handle ); }

void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef* hadc )
{
    /* Update status variable of DMA transfer */
    dma_transfer_status = 1;
}

void HAL_ADC_ConvHalfCpltCallback( ADC_HandleTypeDef* hadc ) {}
