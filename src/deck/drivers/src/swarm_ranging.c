#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "system.h"

#include "autoconf.h"
#include "debug.h"
#include "log.h"
#include "assert.h"
#include "adhocdeck.h"
#include "ranging_struct.h"
#include "swarm_ranging.h"

static uint16_t MY_UWB_ADDRESS;

static QueueHandle_t rxQueue;
static Ranging_Table_Set_t rangingTableSet;
static Two_Hop_Neighbor_Table_Set_t twoHopNeighborTableSet;
static MPR_Selector_Set_t MPRSelectorSet;
static UWB_Message_Listener_t listener;
static TaskHandle_t uwbRangingTxTaskHandle = 0;
static TaskHandle_t uwbRangingRxTaskHandle = 0;

static Timestamp_Tuple_t TfBuffer[Tf_BUFFER_POOL_SIZE] = {0};
static int TfBufferIndex = 0;
static int rangingSeqNumber = 1;

static logVarId_t idVelocityX, idVelocityY, idVelocityZ;
static float velocity;

int16_t distanceTowards[RANGING_TABLE_SIZE + 1] = {[0 ... RANGING_TABLE_SIZE] = -1};

static bool checkAddress(uint16_t address) {
  if(MY_UWB_ADDRESS == 1 && (address == 2 || address == 3)) return true;
  if(MY_UWB_ADDRESS == 2 && (address == 1 || address == 5 || address == 6)) return true;
  if(MY_UWB_ADDRESS == 3 && (address == 1 || address == 4 || address == 5)) return true;
  if(MY_UWB_ADDRESS == 4 && address == 3) return true;
  if(MY_UWB_ADDRESS == 5 && (address == 2 || address == 3)) return true;
  if(MY_UWB_ADDRESS == 6 && address == 2) return true;
  return false;
}

void rangingRxCallback(void *parameters) {
  // DEBUG_PRINT("rangingRxCallback \n");

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  UWB_Packet_t *packet = (UWB_Packet_t *) parameters;

  dwTime_t rxTime;
  dwt_readrxtimestamp((uint8_t *) &rxTime.raw);
  Ranging_Message_With_Timestamp_t rxMessageWithTimestamp;
  rxMessageWithTimestamp.rxTime = rxTime;
  Ranging_Message_t *rangingMessage = (Ranging_Message_t *) packet->payload;
  rxMessageWithTimestamp.rangingMessage = *rangingMessage;

  if(checkAddress(rangingMessage->header.srcAddress)){
    xQueueSendFromISR(rxQueue, &rxMessageWithTimestamp, &xHigherPriorityTaskWoken);
  }
}

void rangingTxCallback(void *parameters) {
  dwTime_t txTime;
  dwt_readtxtimestamp((uint8_t *) &txTime.raw);
  TfBufferIndex++;
  TfBufferIndex %= Tf_BUFFER_POOL_SIZE;
  TfBuffer[TfBufferIndex].seqNumber = rangingSeqNumber;
  TfBuffer[TfBufferIndex].timestamp = txTime;
}

int16_t getDistance(uint16_t neighborAddress) {
  ASSERT(neighborAddress <= RANGING_TABLE_SIZE);
  return distanceTowards[neighborAddress];
}

void setDistance(uint16_t neighborAddress, int16_t distance) {
  ASSERT(neighborAddress <= RANGING_TABLE_SIZE);
  distanceTowards[neighborAddress] = distance;
}

static void uwbRangingTxTask(void *parameters) {
  systemWaitStart();

  /* velocity log variable id */
  idVelocityX = logGetVarId("stateEstimate", "vx");
  idVelocityY = logGetVarId("stateEstimate", "vy");
  idVelocityZ = logGetVarId("stateEstimate", "vz");

  UWB_Packet_t txPacketCache;
  txPacketCache.header.type = RANGING;
//  txPacketCache.header.mac = ? TODO init mac header
  while (true) {
    int msgLen = generateRangingMessage((Ranging_Message_t *) &txPacketCache.payload);
    txPacketCache.header.length = sizeof(Packet_Header_t) + msgLen;
    uwbSendPacketBlock(&txPacketCache);
    vTaskDelay(TX_PERIOD_IN_MS);
  }
}

static void uwbRangingRxTask(void *parameters) {
  systemWaitStart();

  Ranging_Message_With_Timestamp_t rxPacketCache;

  while (true) {
    if (xQueueReceive(rxQueue, &rxPacketCache, portMAX_DELAY)) {
      // DEBUG_PRINT("uwbRangingRxTask: received ranging message \n");
      processRangingMessage(&rxPacketCache);
      // ADD: calculate MPR
      populateTwoHopNeighborSet(&rxPacketCache);
      MPRNeighborBitMap = populateMPRSet(twoHopNeighborBitMap);
      populateMPRSelectorSet(&rxPacketCache);
      DEBUG_PRINT("MPR RECORD: %llu \n", MPRNeighborBitMap);
    }
  }
}

void rangingInit() {
  MY_UWB_ADDRESS = getUWBAddress();
  DEBUG_PRINT("MY_UWB_ADDRESS = %d \n", MY_UWB_ADDRESS);
  rxQueue = xQueueCreate(RANGING_RX_QUEUE_SIZE, RANGING_RX_QUEUE_ITEM_SIZE);
  rangingTableSetInit(&rangingTableSet);
  // ADD: Two hop neighbor
  twoHopNeighborTableSetInit(&twoHopNeighborTableSet);
  // ADD: MPR
  MPRNeighborBitMap = 0;
  // ADD: MPR selector
  MPRSelectorSetInit(&MPRSelectorSet);

  listener.type = RANGING;
  listener.rxQueue = NULL; // handle rxQueue in swarm_ranging.c instead of adhocdeck.c
  listener.rxCb = rangingRxCallback;
  listener.txCb = rangingTxCallback;
  uwbRegisterListener(&listener);

  idVelocityX = logGetVarId("stateEstimate", "vx");
  idVelocityY = logGetVarId("stateEstimate", "vy");
  idVelocityZ = logGetVarId("stateEstimate", "vz");

  xTaskCreate(uwbRangingTxTask, ADHOC_DECK_RANGING_TX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRangingTxTaskHandle); // TODO optimize STACK SIZE
  xTaskCreate(uwbRangingRxTask, ADHOC_DECK_RANGING_RX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRangingRxTaskHandle); // TODO optimize STACK SIZE
}

int16_t computeDistance(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp,
                        Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr,
                        Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf) {

  int64_t tRound1, tReply1, tRound2, tReply2, diff1, diff2, tprop_ctn;
  tRound1 = (Rr.timestamp.full - Tp.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tReply1 = (Tr.timestamp.full - Rp.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tRound2 = (Rf.timestamp.full - Tr.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tReply2 = (Tf.timestamp.full - Rr.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  diff1 = tRound1 - tReply1;
  diff2 = tRound2 - tReply2;
  tprop_ctn = (diff1 * tReply2 + diff2 * tReply1 + diff2 * diff1) / (tRound1 + tRound2 + tReply1 + tReply2);
  int16_t distance = (int16_t) tprop_ctn * 0.4691763978616;

  bool isErrorOccurred = false;
  if (distance > 1000 || distance < 0) {
    DEBUG_PRINT("isErrorOccurred\n");
    isErrorOccurred = true;
  }

  if (tRound2 < 0 || tReply2 < 0) {
    DEBUG_PRINT("tRound2 < 0 || tReply2 < 0\n");
    isErrorOccurred = true;
  }

  if (isErrorOccurred) {
    return 0;
  }

  return distance;
}

void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp) {
  Ranging_Message_t *rangingMessage = &rangingMessageWithTimestamp->rangingMessage;
  uint16_t neighborAddress = rangingMessage->header.srcAddress;
  set_index_t neighborIndex = findInRangingTableSet(&rangingTableSet, neighborAddress);

  /* handle new neighbor */
  if (neighborIndex == -1) {
    if (rangingTableSet.freeQueueEntry == -1) {
      /* ranging table set is full, ignore this ranging message */
      return;
    }
    Ranging_Table_t table;
    rangingTableInit(&table, neighborAddress);
    neighborIndex = rangingTableSetInsert(&rangingTableSet, &table);
  }

  Ranging_Table_t *neighborRangingTable = &rangingTableSet.setData[neighborIndex].data;
  Ranging_Table_Tr_Rr_Buffer_t *neighborTrRrBuffer = &neighborRangingTable->TrRrBuffer;

  /* update Re */
  neighborRangingTable->Re.timestamp = rangingMessageWithTimestamp->rxTime;
  neighborRangingTable->Re.seqNumber = rangingMessage->header.msgSequence;

  /* update Tr and Rr */
  Timestamp_Tuple_t neighborTr = rangingMessage->header.lastTxTimestamp;
  if (neighborTr.timestamp.full && neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr.timestamp.full
      && neighborTr.seqNumber == neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr.seqNumber) {
    rangingTableBufferUpdate(&neighborRangingTable->TrRrBuffer,
                             neighborTr,
                             neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr);
  }

  /* update Rf */
  Timestamp_Tuple_t neighborRf = {.timestamp.full = 0};
  if (rangingMessage->header.filter & (1 << (getUWBAddress() % 16))) {
    /* retrieve body unit */
    uint8_t bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
    for (int i = 0; i < bodyUnitCount; i++) {
      if (rangingMessage->bodyUnits[i].address == getUWBAddress()) {
        neighborRf = rangingMessage->bodyUnits[i].timestamp;
        break;
      }
    }
  }

  if (neighborRf.timestamp.full) {
    neighborRangingTable->Rf = neighborRf;
    // TODO it is possible that can not find corresponding Tf
    /* find corresponding Tf in TfBuffer */
    for (int i = 0; i < Tf_BUFFER_POOL_SIZE; i++) {
      if (TfBuffer[i].seqNumber == neighborRf.seqNumber) {
        neighborRangingTable->Tf = TfBuffer[i];
      }
    }

    Ranging_Table_Tr_Rr_Candidate_t Tr_Rr_Candidate = rangingTableBufferGetCandidate(&neighborRangingTable->TrRrBuffer,
                                                                                     neighborRangingTable->Tf);
    /* try to compute distance */
    if (Tr_Rr_Candidate.Tr.timestamp.full && Tr_Rr_Candidate.Rr.timestamp.full &&
        neighborRangingTable->Tp.timestamp.full && neighborRangingTable->Rp.timestamp.full &&
        neighborRangingTable->Tf.timestamp.full && neighborRangingTable->Rf.timestamp.full) {
      int16_t distance = computeDistance(neighborRangingTable->Tp, neighborRangingTable->Rp,
                                         Tr_Rr_Candidate.Tr, Tr_Rr_Candidate.Rr,
                                         neighborRangingTable->Tf, neighborRangingTable->Rf);
      if (distance > 0) {
        neighborRangingTable->distance = distance;
        setDistance(neighborRangingTable->neighborAddress, distance);
      } else {
        // DEBUG_PRINT("distance is not updated since some error occurs\n");
      }
    }
  }

  /* Tp <- Tf, Rp <- Rf */
  if (neighborRangingTable->Tf.timestamp.full && neighborRangingTable->Rf.timestamp.full) {
    rangingTableShift(neighborRangingTable);
  }

  /* update Rr */
  neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr = neighborRangingTable->Re;

  /* update expiration time */
  neighborRangingTable->expirationTime = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);

  neighborRangingTable->state = RECEIVED;
}

int generateRangingMessage(Ranging_Message_t *rangingMessage) {
#ifdef ENABLE_BUS_BOARDING_SCHEME
  sortRangingTableSet(&rangingTableSet);
#endif
  rangingTableSetClearExpire(&rangingTableSet);
  int8_t bodyUnitNumber = 0;
  rangingSeqNumber++;
  int curSeqNumber = rangingSeqNumber;
  rangingMessage->header.filter = 0;
  /* generate message body */
  for (set_index_t index = rangingTableSet.fullQueueEntry; index != -1;
       index = rangingTableSet.setData[index].next) {
    Ranging_Table_t *table = &rangingTableSet.setData[index].data;
    if (bodyUnitNumber >= MAX_BODY_UNIT_NUMBER) {
      break;
    }
    if (table->state == RECEIVED) {
      rangingMessage->bodyUnits[bodyUnitNumber].address = table->neighborAddress;
      /* It is possible that Re is not the newest timestamp, because the newest may be in rxQueue
       * waiting to be handled.
       */
      rangingMessage->bodyUnits[bodyUnitNumber].timestamp = table->Re;
      bodyUnitNumber++;
      table->state = TRANSMITTED;
      rangingMessage->header.filter |= 1 << (table->neighborAddress % 16);
    }
  }
  /* generate message header */
  rangingMessage->header.srcAddress = MY_UWB_ADDRESS;
  rangingMessage->header.msgLength = sizeof(Ranging_Message_Header_t) + sizeof(Body_Unit_t) * bodyUnitNumber;
  rangingMessage->header.msgSequence = curSeqNumber;
  rangingMessage->header.lastTxTimestamp = TfBuffer[TfBufferIndex];
  // ADD: MPR record
  rangingMessage->header.MPRNeighborBitMap = MPRNeighborBitMap;
  float velocityX = logGetFloat(idVelocityX);
  float velocityY = logGetFloat(idVelocityY);
  float velocityZ = logGetFloat(idVelocityZ);
  velocity = sqrt(pow(velocityX, 2) + pow(velocityY, 2) + pow(velocityZ, 2));
  /* velocity in cm/s */
  rangingMessage->header.velocity = (short) (velocity * 100);
  return rangingMessage->header.msgLength;
}

void populateTwoHopNeighborSet(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp) {
  // expire two hop neighbor set
  twoHopNeighborTableSetClearExpire(&twoHopNeighborTableSet);

  Ranging_Message_t *rangingMessage = &rangingMessageWithTimestamp->rangingMessage;
  uint16_t oneHopAddress = rangingMessage->header.srcAddress;
  int8_t bodyUnitNumberMax = (rangingMessage->header.msgLength -
                             sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
  for(int8_t bodyUnitNumber = 0; bodyUnitNumber < bodyUnitNumberMax; bodyUnitNumber++) {
    uint16_t twoHopAddress = rangingMessage->bodyUnits[bodyUnitNumber].address;
    /* Two hop Neighbor judgment */
    // two hop neighbor is myself
    if (twoHopAddress == MY_UWB_ADDRESS) continue;
    // one hop address is not in ranging table set
    if ((((uint64_t)1 << oneHopAddress) & rangingNeighborBitMap) == 0) continue;
    // two hop neighbor is one of my one hop neighbor
    if (((twoHopNeighborBitMap | ((uint64_t)1 << twoHopAddress)) &
        rangingNeighborBitMap) != 0) continue;
    /* Insert two hop neighbor */
    findAndInsertInTwoHopNeighborTableSet(&twoHopNeighborTableSet, oneHopAddress, twoHopAddress);
  }
}

// Input: twoHopNeighborBitMap
// Return: MPRNeighborBitMap
Neighbor_Bit_Map_t populateMPRSet(Neighbor_Bit_Map_t twoHopBitMap) {
  // 清空原有MPR集合
  Neighbor_Bit_Map_t _MPRNeighborBitMap = 0;

  Neighbor_Bit_Map_t coveredTwoHopNeighbor = 0;
  uint8_t twoHopNeighborReachCount[TWO_HOP_NEIGHBOR_TABLE_SIZE] = {0};
  /* 1 => 找到唯一可以到达二跳节点的一跳节点 */ 
  // 计算每个两跳邻居的可到达路由的数量
  for(set_index_t index = twoHopNeighborTableSet.fullQueueEntry; index != -1;
      index = twoHopNeighborTableSet.setData[index].next) {
    uint16_t twoHopAddress = twoHopNeighborTableSet.setData[index].data.twoHopNeighborAddress;
    twoHopNeighborReachCount[twoHopAddress]++;
  }
  // 添加到MPR集合中
  for(set_index_t index = twoHopNeighborTableSet.fullQueueEntry; index != -1;
      index = twoHopNeighborTableSet.setData[index].next) {
    uint16_t oneHopAddress = twoHopNeighborTableSet.setData[index].data.oneHopNeighborAddress;
    uint16_t twoHopAddress = twoHopNeighborTableSet.setData[index].data.twoHopNeighborAddress;
    if(twoHopNeighborReachCount[twoHopAddress] > 1) continue;
    // add MPR neighbor
    neighborBitMapSet(&_MPRNeighborBitMap, oneHopAddress);
  }
  // 计算剩余未被覆盖的二跳节点集合
  for(set_index_t index = twoHopNeighborTableSet.fullQueueEntry; index != -1;
      index = twoHopNeighborTableSet.setData[index].next) {
    uint16_t oneHopAddress = twoHopNeighborTableSet.setData[index].data.oneHopNeighborAddress;
    uint16_t twoHopAddress = twoHopNeighborTableSet.setData[index].data.twoHopNeighborAddress;
    if((((uint64_t)1 << oneHopAddress) & _MPRNeighborBitMap) == 0) continue;
    neighborBitMapSet(&coveredTwoHopNeighbor, twoHopAddress);
  }
  /* 2 => 计算能够最大限度地覆盖其余两个跳节点的一跳邻居 */
  Neighbor_Bit_Map_t restTwoHopNeighbor = (~coveredTwoHopNeighbor) & twoHopBitMap;
  while(restTwoHopNeighbor != 0){
    uint8_t oneHopNeighborReachCapacity[RANGING_TABLE_SIZE] = {0};
    int16_t bestOneHopNeighbor = -1;
    uint8_t bestOneHopNeighborCapacity = 0;
    // 计算一跳节点的二跳节点
    for(set_index_t index = twoHopNeighborTableSet.fullQueueEntry; index != -1;
        index = twoHopNeighborTableSet.setData[index].next) {
      uint16_t oneHopAddress = twoHopNeighborTableSet.setData[index].data.oneHopNeighborAddress;
      uint16_t twoHopAddress = twoHopNeighborTableSet.setData[index].data.twoHopNeighborAddress;
      // 删除二跳节点不在剩余节点
      if((((uint64_t)1 << twoHopAddress) & restTwoHopNeighbor) == 0) continue;
      oneHopNeighborReachCapacity[oneHopAddress]++;
    }
    // 找到含有最多的剩余二跳节点的一跳节点
    for(int address = 0; address < RANGING_TABLE_SIZE; address++) {
      if(oneHopNeighborReachCapacity[address] > bestOneHopNeighborCapacity) {
        bestOneHopNeighborCapacity = oneHopNeighborReachCapacity[address];
        bestOneHopNeighbor = address;
      }
    }
    // 将其添加到MPR集合中
    neighborBitMapSet(&_MPRNeighborBitMap, bestOneHopNeighbor);
    for(set_index_t index = twoHopNeighborTableSet.fullQueueEntry; index != -1;
        index = twoHopNeighborTableSet.setData[index].next) {
      uint16_t oneHopAddress = twoHopNeighborTableSet.setData[index].data.oneHopNeighborAddress;
      uint16_t twoHopAddress = twoHopNeighborTableSet.setData[index].data.twoHopNeighborAddress;
      if(oneHopAddress == bestOneHopNeighbor) {
        neighborBitMapSet(&coveredTwoHopNeighbor, twoHopAddress);
      }
    }
    // 计算剩余二跳邻居集合
    restTwoHopNeighbor = ~coveredTwoHopNeighbor & twoHopBitMap;
  }

  return _MPRNeighborBitMap;
}

void populateMPRSelectorSet(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp) {
  // expire MPR Selector Set
  MPRSelectorSetClearExpire(&MPRSelectorSet);

  Ranging_Message_t *rangingMessage = &rangingMessageWithTimestamp->rangingMessage;
  uint16_t srcAddress = rangingMessage->header.srcAddress;
  Neighbor_Bit_Map_t MPRNeighborBitMap = rangingMessage->header.MPRNeighborBitMap;
  if((((uint64_t)1 << MY_UWB_ADDRESS) & MPRNeighborBitMap) != 0) {
    MPRSelectorSetInsert(&MPRSelectorSet, srcAddress);
  }
}

bool isMPRSelector(uint16_t neighborAddress) {
  Neighbor_Bit_Map_t MPRSelectorBitMap = MPRSelectorSet.MPRSelectorBitMap;
  if((((uint64_t)1 << neighborAddress) & MPRSelectorBitMap) != 0) {
    return true;
  }
  return false;
}

LOG_GROUP_START(Ranging)
        LOG_ADD(LOG_INT16, distTo1, distanceTowards + 1)
        LOG_ADD(LOG_INT16, distTo2, distanceTowards + 2)
        LOG_ADD(LOG_INT16, distTo3, distanceTowards + 3)
        LOG_ADD(LOG_INT16, distTo4, distanceTowards + 4)
        LOG_ADD(LOG_INT16, distTo5, distanceTowards + 5)
        LOG_ADD(LOG_INT16, distTo6, distanceTowards + 6)
        LOG_ADD(LOG_INT16, distTo7, distanceTowards + 7)
        LOG_ADD(LOG_INT16, distTo8, distanceTowards + 8)
        LOG_ADD(LOG_INT16, distTo8, distanceTowards + 9)
LOG_GROUP_STOP(Ranging)
