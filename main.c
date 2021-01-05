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

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#include <queue.h>
#include <sl_sleeptimer.h>
#include <sl_udelay.h>

#include "string.h"
#include "strings.h"
#include "stdio.h"

#include <sl_uartdrv_usart_vcom_config.h>
#include <sl_uartdrv_instances.h>
#include <sl_board_control_config.h>
#include <sl_power_manager_config.h>
// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
#define QUEUE_DEFAULT_LENGTH 16
#define RETRANSMISSION_BUFFER_DEFAULT_LENGTH 10
#define STACK_SIZE (configMINIMAL_STACK_SIZE * 2)
#define SLEEPTIMER_DELAY_MS 1000

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
///Idle task and Timer task definitions
static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

/// Transmitter Task
static StaticTask_t transmitterTaskTCB;
static StackType_t transmitterTaskStack[STACK_SIZE];
static void transmitterTaskFunction (void*);
static TaskHandle_t transmitterTaskHandle;

///Receiver Task
static StaticTask_t receiverTaskTCB;
static StackType_t receiverTaskStack[STACK_SIZE];
static void receiverTaskFunction (void*);
static TaskHandle_t receiverTaskHandle;

///Receiver Task
static StaticTask_t beaconTaskTCB;
static StackType_t beaconTaskStack[STACK_SIZE];
static void beaconTaskFunction (void*);
static TaskHandle_t beaconTaskHandle;


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

///Dummy packet structs
static pkt_t generatedPacket, txPacket;

///VCOM Serial print buffer
static uint8_t transmitterBuffer[100];

static uint16_t hopCount = 0;
static uint32_t pktSequenceNumber = 0;
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

  //Transmitter Task initialization
  transmitterTaskHandle = xTaskCreateStatic (transmitterTaskFunction, "transmitterTask", STACK_SIZE, NULL, 2, transmitterTaskStack, &transmitterTaskTCB);
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

  //Packet Generator Task Initialization
  beaconTaskHandle = xTaskCreateStatic (beaconTaskFunction, "beaconTask", STACK_SIZE, NULL, 3, beaconTaskStack, &beaconTaskTCB);
  if (beaconTaskHandle == NULL)
   {
     return 0;
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
///Idle and Timer Task definition
void vApplicationGetIdleTaskMemory (StaticTask_t **ppxIdleTaskTCBBuffer,
                               StackType_t **ppxIdleTaskStackBuffer,
                               uint32_t *pulIdleTaskStackSize)
{
  //Declare static variables so they're not placed on the stack

  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory (StaticTask_t **ppxTimerTaskTCBBuffer,
                                StackType_t **ppxTimerTaskStackBuffer,
                                uint32_t *pulTimerTaskStackSize)
{

  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void beaconTaskFunction(void *bt){
  int i = 0;
  while(i<3){
      generatedPacket.header.hopCount = 0;
      generatedPacket.header.pktSeq = 0;
      generatedPacket.header.wupSeq = Wb;

      xQueueSend(transmitterQueueHandle, (void *)&generatedPacket, 0);

      i++;
      vTaskDelay(pdMS_TO_TICKS(1500));
  }

  while(true){
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

    generatedPacket.header.hopCount = 0;
    generatedPacket.header.pktSeq = 0;
    generatedPacket.header.wupSeq = Wb;

    xQueueSend(transmitterQueueHandle, (void *)&generatedPacket, 0);
  }
}

void transmitterTaskFunction(void *tt){
  while(1){
      xQueueReceive(transmitterQueueHandle, &(txPacket), portMAX_DELAY);

      //Check that we don't overflow the tx buffer
      while(RAIL_GetTxFifoSpaceAvailable(rail_handle) < sizeof(pkt_t) * 2){
          sl_sleeptimer_delay_millisecond (100);
      }
      //Simulate sending a WUP packet to wake up nodes on the sub GHZ frequency.
      //In our case we send the actual packet
      RAIL_WriteTxFifo (rail_handle, (uint8_t*) &txPacket, sizeof(pkt_t), false);
      while (RAIL_StartTx (rail_handle, 21, 0, NULL) != RAIL_STATUS_NO_ERROR);
      //Wait for 100ms to be sure that the node have woken up
      //We are still in the rx wake up window (1sec)
      sl_sleeptimer_delay_millisecond (100);
      //Send the actual flood data packet
      RAIL_WriteTxFifo (rail_handle, (uint8_t*) &txPacket, sizeof(pkt_t), false);
      while (RAIL_STATUS_NO_ERROR != RAIL_StartTx (rail_handle, 0, 0, NULL));

      //SERIAL OUTPUT FOR DEBUGGING PURPOSES
      if(txPacket.header.wupSeq == Wb){
          snprintf (&transmitterBuffer, 100, "\r\nBeacon update sent!\r\n");
      }else{
          snprintf (&transmitterBuffer, 100, "Packet sent:\r\nSequence number: %lu\r\nWUP Sequence: %u\r\n", txPacket.header.pktSeq, txPacket.header.wupSeq);
      }


      while (ECODE_OK != UARTDRV_TransmitB (sl_uartdrv_usart_vcom_handle, &transmitterBuffer[0], strlen (transmitterBuffer)));
  }
}

//TODO: Create Receiver task: when the sink receives a Wr packet and the hopcount is (own+1) it checks its retransmission buffer for the packet and retransmits

//TODO: Create Package generation task

//TODO: Create delayer task

//TODO: Create rfSense callback

//TODO: Implement Idle Hook

//TODO: Implement RAIL event Handler
