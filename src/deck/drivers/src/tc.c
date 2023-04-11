#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "system.h"
#include "semphr.h"

#include "autoconf.h"
#include "debug.h"
#include "log.h"
#include "assert.h"
#include "adhocdeck.h"
#include "tc.h"
#include "swarm_ranging.h"

static uint16_t MY_UWB_ADDRESS;

static QueueHandle_t rxQueue;
static Flooding_Topology_Table_set_t floodingTopologyTableSet; 
static UWB_Message_Listener_t listener;
static MPR_Selector_Set_t MPRSelectorSet;
static TaskHandle_t uwbTcTxTaskHandle = 0;
static TaskHandle_t uwbTcRxTaskHandle = 0;

static int tcSeqNumber = 1;

uint16_t tcCheckTable[TC_CHECK_TABLE_SIZE] = {0};

void tcRxCallback(void *parameters) {
  DEBUG_PRINT("tcRxCallback \n");
}

void tcTxCallback(void *parameters) {
  DEBUG_PRINT("tcTxCallback \n");
}

static void uwbTcTxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t txPacketCache;
  txPacketCache.header.type = TC;

  while (true) {
    printFloodingTopologyTableSet(&floodingTopologyTableSet);
    int msgLen = generateTcMessage((Tc_Message_t *) &txPacketCache.payload);
    txPacketCache.header.length = sizeof(Packet_Header_t) + msgLen;
    uwbSendPacketBlock(&txPacketCache);
    /* jitter */
    int jitter = (int) (rand() / (float) RAND_MAX * 9) - 4;
    vTaskDelay(TC_INTERVAL + M2T(jitter));
  }
}

static void uwbTcRxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t rxPacketCache;

  while (true) {
    if (xQueueReceive(rxQueue, &rxPacketCache, portMAX_DELAY)) {
      Tc_Message_t *tcMessage = (Tc_Message_t *) &rxPacketCache.payload;
      if (checkTcMessage(tcMessage)) {
        processTcMessage(tcMessage);
        // if(MprSelectorSetFind(&ms, tcMessage->header.srcAddress) != -1)
        // {uwbSendPacketBlock(&rxPacketCache);}
        if((MPRNeighborBitMap >> tcMessage->header.srcAddress) &= (uint64_t) 1)
        uwbSendPacketBlock(&rxPacketCache);
      }
    }
  }
}

void tcInit() {
  MY_UWB_ADDRESS = getUWBAddress();
  rxQueue = xQueueCreate(TC_RX_QUEUE_SIZE, TC_RX_QUEUE_ITEM_SIZE);
  floodingTopologyTableSetInit(&floodingTopologyTableSet);

  listener.type = TC;
  listener.rxQueue = rxQueue;
  listener.rxCb = tcRxCallback;
  listener.txCb = tcTxCallback;
  uwbRegisterListener(&listener);

  xTaskCreate(uwbTcTxTask, ADHOC_DECK_FLOODING_TX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbTcTxTaskHandle);//
  xTaskCreate(uwbTcRxTask, ADHOC_DECK_FLOODING_RX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbTcRxTaskHandle);//
}

int generateTcMessage(Tc_Message_t *tcMessage) {
  floodingTopologyTableSetClearExpire(&floodingTopologyTableSet);//删除原有拓扑信息表
  int8_t bodyUnitNumber = 0;
  int curSeqNumber = tcSeqNumber;
  /* 生成TC消息主干，将本地拓扑信息表放到TC报文中 */
  uint16_t addressIndex;
  for (addressIndex = 0; addressIndex < RANGING_TABLE_SIZE; addressIndex++) {
    if (bodyUnitNumber >= MAX_BODY_UNIT_NUMBER) {
      break;
    }
    /* 更新拓扑信息表 */
    int16_t distance = getDistance(addressIndex);
    if (distance >= 0) {
      tcMessage->bodyUnits[bodyUnitNumber].dstAddress = addressIndex;
      tcMessage->bodyUnits[bodyUnitNumber].distance = distance;
      floodingTopologyTableSetUpdate(&floodingTopologyTableSet, MY_UWB_ADDRESS,
                                     addressIndex, distance);
      bodyUnitNumber++;
    }
  }
  /* 生成TC消息头 */
  tcMessage->header.srcAddress = MY_UWB_ADDRESS;
  tcMessage->header.msgLength = sizeof(Tc_Message_Header_t) + sizeof(Tc_Body_Unit_t) * bodyUnitNumber;
  tcMessage->header.msgSequence = curSeqNumber;
  tcMessage->header.timeToLive = TC_TIME_TO_LIVE;
  tcSeqNumber++;
  return tcMessage->header.msgLength;
}

void processTcMessage(Tc_Message_t *tcMessage) {
  int8_t bodyUnitNumberMax = (tcMessage->header.msgLength -
      sizeof(Tc_Message_Header_t)) / sizeof(Tc_Body_Unit_t);
  for (int8_t bodyUnitNumber = 0; bodyUnitNumber < bodyUnitNumberMax; bodyUnitNumber++) {
    Tc_Body_Unit_t *bodyUnit = &tcMessage->bodyUnits[bodyUnitNumber];
    // 更新获取到的拓扑信息，用于维护本地拓扑表
    floodingTopologyTableSetUpdate(&floodingTopologyTableSet, tcMessage->header.srcAddress,
                                   bodyUnit->dstAddress, bodyUnit->distance);
  }
}

bool checkTcMessage(Tc_Message_t *tcMessage) {
  if (tcMessage == NULL ||
      tcMessage->header.srcAddress == MY_UWB_ADDRESS ||
      tcMessage->header.timeToLive == 0 ||
      tcMessage->header.msgSequence <= tcCheckTable[tcMessage->header.srcAddress]) {
    return false;
  }
  tcMessage->header.timeToLive--;
  tcCheckTable[tcMessage->header.srcAddress] = tcMessage->header.msgSequence;
  return true;
}

// void printTcMessage(Tc_Message_t *tcMessage)
// {
//   d;
// }