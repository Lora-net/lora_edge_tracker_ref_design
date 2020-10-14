/**
 ******************************************************************************
 * @file    lls.c
 * @author  MCD Application Team
 * @brief   Link Loss Service
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include "common_blesvc.h"


/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  uint16_t SvcHdle;                  /**< Service handle */
  uint16_t AlertLevelCharHdle;       /**< Characteristic handle */
}LLS_Context_t;


/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/**
 * START of Section BLE_DRIVER_CONTEXT
 */

PLACE_IN_SECTION("BLE_DRIVER_CONTEXT") static LLS_Context_t LLS_Context;

/**
 * END of Section BLE_DRIVER_CONTEXT
 */


/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t LLS_Event_Handler(void *pckt);


/* Functions Definition ------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t LLS_Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blue_aci *blue_evt;
  aci_gatt_attribute_modified_event_rp0    * attribute_modified;
  LLS_App_Notification_evt_t Notification;

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

  switch(event_pckt->evt)
  {
    case EVT_VENDOR:
    {
      blue_evt = (evt_blue_aci*)event_pckt->data;
      switch(blue_evt->ecode)
      {
        case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        {
          attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blue_evt->data;
          if(attribute_modified->Attr_Handle == (LLS_Context.AlertLevelCharHdle + 1))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            if(attribute_modified->Attr_Data[0] == 0)
            {
              Notification.LLS_Evt_Opcode = LLS_NO_ALERT_EVT;
              LLS_App_Notification(&Notification);
            }
            else if(attribute_modified->Attr_Data[0] == 1)
            {
              Notification.LLS_Evt_Opcode = LLS_MID_ALERT_EVT;
              LLS_App_Notification(&Notification);
            }
            else if(attribute_modified->Attr_Data[0] == 2)
            {
              Notification.LLS_Evt_Opcode = LLS_HIGH_ALERT_EVT;
              LLS_App_Notification(&Notification);
            }
          }
        }
        break;

        default:
          break;
      }
    }
    break; /* HCI_EVT_VENDOR_SPECIFIC */

    default:
      break;
  }

  return(return_value);
}/* end LLS_Event_Handler */


/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void LLS_Init(void)
{
  uint16_t uuid;
  tBleStatus hciCmdResult;

  /**
   *	Register the event handler to the BLE controller
   */
  SVCCTL_RegisterSvcHandler(LLS_Event_Handler);

  /**
   *  Add Link Loss Service
   *
   * Max_Attribute_Records = 2*no_of_char + 1
   * service_max_attribute_record = 1 for Link Loss service +
   *                                2 for alert level characteristic +
   */
  uuid = LINK_LOSS_SERVICE_UUID;
  hciCmdResult = aci_gatt_add_service(UUID_TYPE_16,
                                   (Service_UUID_t *) &uuid,
                                   PRIMARY_SERVICE,
                                   3,
                                   &(LLS_Context.SvcHdle));

  if (hciCmdResult == BLE_STATUS_SUCCESS)
  {
    BLE_DBG_LLS_MSG ("Link Loss Service (LLS) is added Successfully %04X\n", 
                 LLS_Context.SvcHdle);
  }
  else
  {
    BLE_DBG_LLS_MSG ("FAILED to add Link Loss Service (LLS), Error: %02X !!\n", 
                 hciCmdResult);
  }

  /**
   *  Add Alert Level Characteristic
   */
  uuid = LINK_LOSS_ALERT_LEVEL_CHARACTERISTIC_UUID;
  hciCmdResult = aci_gatt_add_char(LLS_Context.SvcHdle,
                                   UUID_TYPE_16,
                                   (Char_UUID_t *) &uuid ,
                                   1,
                                   CHAR_PROP_READ | CHAR_PROP_WRITE,
                                   ATTR_PERMISSION_NONE,
                                   GATT_NOTIFY_ATTRIBUTE_WRITE,
                                   MIN_ENCRY_KEY_SIZE,
                                   CHAR_VALUE_LEN_CONSTANT,
                                   &(LLS_Context.AlertLevelCharHdle));

  if (hciCmdResult == BLE_STATUS_SUCCESS)
  {
    BLE_DBG_LLS_MSG ("Alert Level Characteristic Added Successfully  %04X \n", 
                 LLS_Context.AlertLevelCharHdle);
  }
  else
  {
    BLE_DBG_LLS_MSG ("FAILED to add Alert Level Characteristic, Error: %02X !!\n", 
                 hciCmdResult);
  }

  return;
}


/**
 * @brief  Characteristic update
 * @param  UUID: UUID of the characteristic
 * @retval AlertLevelValue: The new value to be written
 */
tBleStatus LLS_Update_Char(uint16_t UUID, uint8_t *pPayload)
{
  tBleStatus return_value = BLE_STATUS_FAILED;

  if(UUID == LINK_LOSS_ALERT_LEVEL_CHARACTERISTIC_UUID)
  {
    return_value = aci_gatt_update_char_value(LLS_Context.SvcHdle,
                                              LLS_Context.AlertLevelCharHdle,
                                              0, /* charValOffset */
                                              1, /* charValueLen */
                                              (uint8_t *) pPayload);
  }
  
  return return_value;
}/* end LLS_Update_Char() */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
