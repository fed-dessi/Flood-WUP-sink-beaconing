/***************************************************************************//**
 * @file main.c
 * @brief main.c
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

// -----------------------------------------------------------------------------
//                                   Includes
// -----------------------------------------------------------------------------
#include "sl_component_catalog.h"
#include "sl_system_init.h"
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
  #include "sl_power_manager.h"
#endif
#include "app_init.h"
#include "app_process.h"
#if defined(SL_CATALOG_KERNEL_PRESENT)
  #include "sl_system_kernel.h"
#else // SL_CATALOG_KERNEL_PRESENT
  #include "sl_system_process_action.h"
#endif // SL_CATALOG_KERNEL_PRESENT

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "sl_sleeptimer.h"
#include "sl_udelay.h"

#include "string.h"
#include "strings.h"
#include "stdio.h"

#include "sl_uartdrv_usart_vcom_config.h"
#include "sl_uartdrv_instances.h"
#include "sl_board_control_config.h"
#include "sl_power_manager_config.h"
#include "sl_led.h"
#include "sl_simple_led_instances.h"
// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
#define QUEUE_DEFAULT_LENGTH 16
#define RETRANSMISSION_BUFFER_DEFAULT_LENGTH 10
#define STACK_SIZE (configMINIMAL_STACK_SIZE * 2)
#define SLEEPTIMER_DELAY_MS 1000
#define PACKET_GENERATION_MS_DELAY 1000

enum wupSequence{
  Wb,
  Wd,
  Wr
};

#pragma pack(push,1)
typedef struct
{
  uint16_t wupSeq;
  uint16_t hopCount;
  uint16_t pktSeq; //Packet Sequence #
} pkt_header_t;

typedef struct
{
  pkt_header_t header;
  uint8_t payload[10];
} pkt_t;
#pragma pack(pop)
// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------
/// Transmitter Task
static StaticTask_t transmitterTaskTCB;
static StackType_t transmitterTaskStack[STACK_SIZE];
static void transmitterTaskFunction ();
static TaskHandle_t transmitterTaskHandle;

///Receiver Task
static StaticTask_t receiverTaskTCB;
static StackType_t receiverTaskStack[STACK_SIZE];
static void receiverTaskFunction ();
static TaskHandle_t receiverTaskHandle;

///Beacon Task
static StaticTask_t beaconTaskTCB;
static StackType_t beaconTaskStack[STACK_SIZE];
static void beaconTaskFunction ();
static TaskHandle_t beaconTaskHandle;

///Packet generator Task
static StaticTask_t pktGeneratorTaskTCB;
static StackType_t pktGeneratorTaskStack[STACK_SIZE];
static void pktGeneratorTaskFunction ();
static TaskHandle_t pktGeneratorTaskHandle;

///Delayer Task
static StaticTask_t delayerTaskTCB;
static StackType_t delayerTaskStack[configMINIMAL_STACK_SIZE];
static void delayerTaskFunction ();
static TaskHandle_t delayerTaskHandle;

///RFSense callback
static void rfSenseCb(void);

///Callback Function
static void timerCallback(sl_sleeptimer_timer_handle_t *handle, void *data);


/// Queue handle and space
static QueueHandle_t transmitterQueueHandle;
static StaticQueue_t transmitterQueueDataStruct;
static uint8_t transmitterQueue[sizeof(pkt_t) * QUEUE_DEFAULT_LENGTH];
// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------
/// A static handle of a RAIL instance
static RAIL_Handle_t rail_handle;

/// RAIL tx and rx queue
static uint8_t railTxFifo[sizeof(pkt_t) * QUEUE_DEFAULT_LENGTH];
static uint8_t railRxFifo[sizeof(pkt_t) * QUEUE_DEFAULT_LENGTH];
static uint16_t rxFifoSize = sizeof(pkt_t) * QUEUE_DEFAULT_LENGTH;

///Dummy packet structs
static pkt_t generatedPacket, txPacket, rxPacket;

///VCOM Serial print buffer
static uint8_t transmitterBuffer[100];

///Rx Packet handle, details and info
static RAIL_RxPacketHandle_t packet_handle;
static RAIL_RxPacketInfo_t packet_info;

/// Pointer used to force context switch from ISR
static BaseType_t xHigherPriorityTaskWoken;

///Retransmission buffer and index
static pkt_t retransmissionBuffer[QUEUE_DEFAULT_LENGTH];
static uint32_t retransmissionBufferIndex = 0;
static volatile bool found;

///Sleeptimer handles
static sl_sleeptimer_timer_handle_t delayerSleeptimerHandle;

static volatile bool wait, isTimerRunning;

static uint16_t hopCount = 0;
static uint32_t pktSequenceNumber = 1;
// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------
/******************************************************************************
 * Main function
 *****************************************************************************/
int main(void)
{
  // Initialize Silicon Labs device, system, service(s) and protocol stack(s).
  // Note that if the kernel is present, processing task(s) will be created by
  // this call.
  sl_system_init();

  // Initialize the application. For example, create periodic timer(s) or
  // task(s) if the kernel is present.
  rail_handle = app_init();


  //Transmitter Task
    transmitterTaskHandle = xTaskCreateStatic (transmitterTaskFunction, "transmitterTask", STACK_SIZE, NULL, 3, transmitterTaskStack, &transmitterTaskTCB);
    if (transmitterTaskHandle == NULL)
      {
        return 0;
      }

    //Receiver Task
    receiverTaskHandle = xTaskCreateStatic (receiverTaskFunction, "receiverTask", STACK_SIZE, NULL, 4, receiverTaskStack, &receiverTaskTCB);
    if (receiverTaskHandle == NULL)
     {
       return(0);
     }

    //Beacon Task
    beaconTaskHandle = xTaskCreateStatic (beaconTaskFunction, "beaconTask", STACK_SIZE, NULL, 2, beaconTaskStack, &beaconTaskTCB);
    if (beaconTaskHandle == NULL)
     {
       return 0;
     }

    //Packet Generator Task
    pktGeneratorTaskHandle = xTaskCreateStatic (pktGeneratorTaskFunction, "pktGeneratorTask", STACK_SIZE, NULL, 1, pktGeneratorTaskStack, &pktGeneratorTaskTCB);
    if (pktGeneratorTaskHandle == NULL)
     {
       return 0;
     }

    //Delayer Task
    //This task prevents going into sleep mode again after waking up. "Sleep mode" is just the Idle Task running,
    //when we wake up with RFSense we would immediately fall back into it after the callback.
    delayerTaskHandle = xTaskCreateStatic (delayerTaskFunction, "delayerTask", configMINIMAL_STACK_SIZE, NULL, 1, delayerTaskStack, &delayerTaskTCB);
    if (delayerTaskHandle == NULL)
     {
       return(0);
     }

    //setting tx fifo
    RAIL_SetTxFifo (rail_handle, railTxFifo, 0, sizeof(pkt_t) * QUEUE_DEFAULT_LENGTH);

    //enabling vcom
    GPIO_PinOutSet (SL_BOARD_ENABLE_VCOM_PORT, SL_BOARD_ENABLE_VCOM_PIN);

    //Init Queues
    transmitterQueueHandle = xQueueCreateStatic(QUEUE_DEFAULT_LENGTH, sizeof(pkt_t), transmitterQueue, &transmitterQueueDataStruct);


#if defined(SL_CATALOG_KERNEL_PRESENT)
  // Start the kernel. Task(s) created in app_init() will start running.
  sl_system_kernel_start();
#else // SL_CATALOG_KERNEL_PRESENT
  while (1) {
    // Do not remove this call: Silicon Labs components process action routine
    // must be called from the super loop.
    sl_system_process_action();

    // Application process.
    app_process_action(rail_handle);

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
    // Let the CPU go to sleep if the system allows it.
    sl_power_manager_sleep();
#endif
  }
#endif // SL_CATALOG_KERNEL_PRESENT
}

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------
void beaconTaskFunction(){
  int i = 0;
  while(i<3){
      generatedPacket.header.hopCount = 0;
      generatedPacket.header.pktSeq = 0;
      generatedPacket.header.wupSeq = Wb;

      xQueueSend(transmitterQueueHandle, (void *)&generatedPacket, 0);

      i++;
      vTaskDelay(pdMS_TO_TICKS(1000));
  }
  //Start the packet generator task
  xTaskNotifyGive(pktGeneratorTaskHandle);
  while(true){
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

    generatedPacket.header.hopCount = 0;
    generatedPacket.header.pktSeq = 0;
    generatedPacket.header.wupSeq = Wb;

    xQueueSend(transmitterQueueHandle, (void *)&generatedPacket, 0);
  }
}

///Packet Generator Task
void pktGeneratorTaskFunction (){
  //Wait for the initial beaconing phase to finish before generating the packets
  ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
  while (1)
    {
      vTaskDelay(pdMS_TO_TICKS(PACKET_GENERATION_MS_DELAY));
      //Start the radio on RX if we've just woken up from idle
      RAIL_StartRx (rail_handle, 0 , NULL);


      generatedPacket.header.pktSeq = pktSequenceNumber;
      generatedPacket.header.wupSeq = Wd;
      generatedPacket.header.hopCount = hopCount + 1;

      if(retransmissionBufferIndex < QUEUE_DEFAULT_LENGTH){
          memcpy(&retransmissionBuffer[retransmissionBufferIndex], &generatedPacket, sizeof(pkt_t));
          retransmissionBufferIndex++;
      }else{
          for(int i = 0; i<QUEUE_DEFAULT_LENGTH - 1; i++){
              memcpy(&retransmissionBuffer[i], &retransmissionBuffer[i+1], sizeof(pkt_t));
          }
          memcpy(&retransmissionBuffer[QUEUE_DEFAULT_LENGTH - 1], &generatedPacket, sizeof(pkt_t));
      }

      pktSequenceNumber++;

      xQueueSend(transmitterQueueHandle, (void *)&generatedPacket, 0);

      xTaskNotifyGive(delayerTaskHandle);
    }
}


void transmitterTaskFunction(){
  while(1){
      xQueueReceive(transmitterQueueHandle, &(txPacket), portMAX_DELAY);

      //Check that we don't overflow the tx buffer
      while(RAIL_GetTxFifoSpaceAvailable(rail_handle) < sizeof(pkt_t) * 2){
          sl_sleeptimer_delay_millisecond (100);
      }
      //Simulate sending a WUP packet to wake up nodes on the sub GHZ frequency.
      //In our case we send the actual packet
      RAIL_WriteTxFifo (rail_handle, (uint8_t*) &txPacket, sizeof(pkt_t), false);
      while (RAIL_StartTx (rail_handle, 1, 0, NULL) != RAIL_STATUS_NO_ERROR);
      //Wait for 100ms to be sure that the node have woken up
      //We are still in the rx wake up window (1sec)
      sl_sleeptimer_delay_millisecond (100);
      //Send the actual flood data packet
      RAIL_WriteTxFifo (rail_handle, (uint8_t*) &txPacket, sizeof(pkt_t), false);
      while (RAIL_STATUS_NO_ERROR != RAIL_StartTx (rail_handle, 0, 0, NULL));

      //SERIAL OUTPUT FOR DEBUGGING PURPOSES
      if(txPacket.header.wupSeq == Wb){
          snprintf ((char*)&transmitterBuffer, 100, "\r\nBeacon update sent!\r\n");
      }
      if(txPacket.header.wupSeq == Wd){
          snprintf ((char*)&transmitterBuffer, 100, "Packet sent:\r\nSequence number: %u\r\nWUP Sequence: %u\r\nHop Count: %u\r\n", txPacket.header.pktSeq, txPacket.header.wupSeq, txPacket.header.hopCount);
      }


      while (ECODE_OK != UARTDRV_TransmitB (sl_uartdrv_usart_vcom_handle, &transmitterBuffer[0], strlen ((char*)transmitterBuffer)));
  }
}

///Receiver Task
void receiverTaskFunction (){
  while (true)
    {
      ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
      packet_handle = RAIL_GetRxPacketInfo (rail_handle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);

      if (packet_handle != RAIL_RX_PACKET_HANDLE_INVALID){
          sl_sleeptimer_is_timer_running(&delayerSleeptimerHandle, &isTimerRunning);
          if(isTimerRunning){
              sl_sleeptimer_restart_timer_ms(&delayerSleeptimerHandle, SLEEPTIMER_DELAY_MS, timerCallback, (void*)&wait, 0, 0);
          }

          RAIL_CopyRxPacket (&rxPacket, &packet_info);
          RAIL_ReleaseRxPacket (rail_handle, packet_handle);

          if(rxPacket.header.wupSeq == Wr){
              if(rxPacket.header.hopCount == hopCount){
                  snprintf ((char*)&transmitterBuffer, 100, "\r\nRetransmit Packet received:\r\nPacket Sequence: %u\r\n", rxPacket.header.pktSeq);
                  while (ECODE_OK != UARTDRV_TransmitB (sl_uartdrv_usart_vcom_handle, &transmitterBuffer[0], strlen ((char*)transmitterBuffer)));

                  for(unsigned int i = 0; i < retransmissionBufferIndex; i++){
                      if(retransmissionBuffer[i].header.pktSeq == rxPacket.header.pktSeq){
                          xQueueSend(transmitterQueueHandle, (void *)&retransmissionBuffer[i], 0);
                          found = true;
                      }
                      if(found && retransmissionBuffer[i].header.pktSeq > rxPacket.header.pktSeq)
                        xQueueSend(transmitterQueueHandle, (void *)&retransmissionBuffer[i], 0);
                  }
                  found = false;
              }
          }
      }
    }
}


///Delayer task, avoids we immediately go to sleep after waking up with RFSense
void delayerTaskFunction ()
{
  while (1)
    {
      ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
      sl_sleeptimer_is_timer_running(&delayerSleeptimerHandle, &isTimerRunning);
      if(!isTimerRunning){
          wait = true;
          sl_sleeptimer_start_timer_ms(&delayerSleeptimerHandle, SLEEPTIMER_DELAY_MS, timerCallback, (void*)&wait, 0, 0);
          while(wait);
      }
      //TODO: remove after finishing the debugging phase
      if(true)
        __asm__("nop");
    }
}

void timerCallback(sl_sleeptimer_timer_handle_t *handle, void *data){
  volatile bool *wait_flag = (bool*)data;

  *wait_flag = false;
}


///Idle Task Hook, we turn off the radio and start the RFSense peripheral on the Sub GHZ freq before entering "sleep mode"
void vApplicationIdleHook ()
{
  // Starting RFSENSE before going to sleep
  RAIL_Idle (rail_handle, RAIL_IDLE, true);
  RAIL_StartRfSense (rail_handle, RAIL_RFSENSE_SUBGHZ_LOW_SENSITIVITY, 50, rfSenseCb);
}

///RFSense Callback function
void rfSenseCb ()
{
  //We've woken up with RFSense, now we schedule a receiving window of 1s
  //and notify the delayer task so we don't immediately go to sleep
  if (RAIL_StartRx (rail_handle, 0 , NULL) == RAIL_STATUS_NO_ERROR)
    {
      xHigherPriorityTaskWoken = pdFALSE;
      vTaskNotifyGiveFromISR(delayerTaskHandle, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


///RAIL event handler
void sl_rail_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
  if (events & RAIL_EVENT_CAL_NEEDED)
    {
      RAIL_Calibrate (rail_handle, NULL, RAIL_CAL_ALL_PENDING);
    }
  if (events & RAIL_EVENT_TX_PACKET_SENT)
    {
      sl_led_toggle (&sl_led_led0);
      sl_udelay_wait (10000);
      sl_led_toggle (&sl_led_led0);
    }
  if (events & RAIL_EVENT_RX_PACKET_RECEIVED)
    {
      sl_led_toggle (&sl_led_led1);
      sl_udelay_wait (10000);
      sl_led_toggle (&sl_led_led1);
      xHigherPriorityTaskWoken = pdFALSE;
      //new rx -> deferred handler architecture
      RAIL_HoldRxPacket (rail_handle);
      vTaskNotifyGiveFromISR(receiverTaskHandle, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}



RAIL_Status_t RAILCb_SetupRxFifo (RAIL_Handle_t railHandle)
{
  RAIL_Status_t status = RAIL_SetRxFifo (railHandle, &railRxFifo[0], &rxFifoSize);
  if (rxFifoSize != sizeof(pkt_t) * QUEUE_DEFAULT_LENGTH)
    {
      // We set up an incorrect FIFO size
      return RAIL_STATUS_INVALID_PARAMETER;
    }
  if (status == RAIL_STATUS_INVALID_STATE)
    {
      // Allow failures due to multiprotocol
      return RAIL_STATUS_NO_ERROR;
    }
  return status;
}
