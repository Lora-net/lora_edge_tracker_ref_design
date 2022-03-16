/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : p2p_server_app.c
 * Description        : P2P Server Application
 ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"
#include "lr1110_tracker_board.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "p2p_server_app.h"
#include "stm32_seq.h"

#include "tracker_utility.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 
typedef struct
{
    uint8_t             Buffer[244];
    uint8_t             Len;
    uint8_t             ReadyToSend;
}P2P_ReadWriteValue_t;

typedef struct
{
    uint8_t               Notification_Status; /* used to chek if P2P Server is enabled to Notify */
    P2P_ReadWriteValue_t  ReadWrite;
    uint16_t              ConnectionHandle;
}P2P_Server_App_Context_t;
/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/**
 * START of Section BLE_APP_CONTEXT
 */

PLACE_IN_SECTION("BLE_APP_CONTEXT") static P2P_Server_App_Context_t P2P_Server_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void P2PS_Send_Notification(void);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void P2PS_STM_App_Notification(P2PS_STM_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_STM_App_Notification_1 */

/* USER CODE END P2PS_STM_App_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_STM_App_Notification_P2P_Evt_Opcode */

/* USER CODE END P2PS_STM_App_Notification_P2P_Evt_Opcode */

    case P2PS_STM__NOTIFY_ENABLED_EVT:
/* USER CODE BEGIN P2PS_STM__NOTIFY_ENABLED_EVT */
            P2P_Server_App_Context.Notification_Status = 1;
            HAL_DBG_TRACE_MSG("-- P2P APPLICATION SERVER : NOTIFICATION ENABLED\n\r"); 
/* USER CODE END P2PS_STM__NOTIFY_ENABLED_EVT */
      break;

    case P2PS_STM_NOTIFY_DISABLED_EVT:
/* USER CODE BEGIN P2PS_STM_NOTIFY_DISABLED_EVT */
            P2P_Server_App_Context.Notification_Status = 0;
            HAL_DBG_TRACE_MSG("-- P2P APPLICATION SERVER : NOTIFICATION DISABLED\n\r");
/* USER CODE END P2PS_STM_NOTIFY_DISABLED_EVT */
      break;
      
        case P2PS_STM_WRITE_EVT:
        {
/* USER CODE BEGIN P2PS_STM_WRITE_EVT */
            uint8_t out_buffer[244];
            uint8_t out_buffer_size = 0;

            out_buffer_size = tracker_parse_cmd(pNotification->DataTransfered.pPayload, out_buffer, true);

            if(out_buffer_size > 0)
            {
                P2P_Server_App_Context.ReadWrite.ReadyToSend = 1;
                P2P_Server_App_Context.ReadWrite.Len = out_buffer_size;
                memcpy(P2P_Server_App_Context.ReadWrite.Buffer,out_buffer,out_buffer_size);
                UTIL_SEQ_SetTask( 1 << CFG_TASK_ANSWER_CMD_ID, CFG_SCH_PRIO_0);
            }

/* USER CODE END P2PS_STM_WRITE_EVT */
            break;
        }

        case P2PS_STM_BOOT_REQUEST_EVT:
        {
            pNotification->DataTransfered.pPayload[2] = 188; // Force the nb_page to erase up to the sector 196 ( 7 sectors for bootloader & 188 sectors for the app)
            *(uint32_t*)SRAM1_BASE = *(uint32_t*)pNotification->DataTransfered.pPayload;
            hal_mcu_reset( );
            break;
        }

    default:
/* USER CODE BEGIN P2PS_STM_App_Notification_default */

/* USER CODE END P2PS_STM_App_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_STM_App_Notification_2 */

/* USER CODE END P2PS_STM_App_Notification_2 */
  return;
}

void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_APP_Notification_1 */

/* USER CODE END P2PS_APP_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_APP_Notification_P2P_Evt_Opcode */

/* USER CODE END P2PS_APP_Notification_P2P_Evt_Opcode */
  case PEER_CONN_HANDLE_EVT :
/* USER CODE BEGIN PEER_CONN_HANDLE_EVT */

/* USER CODE END PEER_CONN_HANDLE_EVT */
    break;

    case PEER_DISCON_HANDLE_EVT :
/* USER CODE BEGIN PEER_DISCON_HANDLE_EVT */
    leds_off(LED_RX_MASK);
/* USER CODE END PEER_DISCON_HANDLE_EVT */
    break;
    
    default:
/* USER CODE BEGIN P2PS_APP_Notification_default */

/* USER CODE END P2PS_APP_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_APP_Notification_2 */

/* USER CODE END P2PS_APP_Notification_2 */
  return;
}

void P2PS_APP_Init(void)
{
    /* USER CODE BEGIN P2PS_APP_Init */
    UTIL_SEQ_RegTask( 1<< CFG_TASK_ANSWER_CMD_ID, UTIL_SEQ_RFU, P2PS_Send_Notification );

    /**
    * Initialize notification Service
    */
    P2P_Server_App_Context.Notification_Status = 0; 
    /* USER CODE END P2PS_APP_Init */
    return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
void P2PS_Send_Notification( void )
{
    if( P2P_Server_App_Context.ReadWrite.ReadyToSend == 0x01 )
    {
        P2P_Server_App_Context.ReadWrite.ReadyToSend = 0;
        HAL_DBG_TRACE_MSG( "-- P2P APPLICATION SERVER  : INFORM CLIENT ASNWER CMD \n\r" );
        if( P2P_Server_App_Context.Notification_Status == 1 )
        {
            P2PS_STM_App_Update_Char( P2P_NOTIFY_CHAR_UUID, P2P_Server_App_Context.ReadWrite.Buffer,
                                      P2P_Server_App_Context.ReadWrite.Len );
        }
        else
        {
            P2PS_STM_App_Update_Char( P2P_WRITE_CHAR_UUID, P2P_Server_App_Context.ReadWrite.Buffer,
                                      P2P_Server_App_Context.ReadWrite.Len );
        }
    }

    return;
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
