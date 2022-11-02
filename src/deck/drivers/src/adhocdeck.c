#define DEBUG_MODULE "DWM3k"

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "autoconf.h"
#include "debug.h"
#include "deck.h"
#include "estimator.h"
#include "log.h"
#include "param.h"
#include "system.h"

#include "adhocdeck.h"
#include "dwTypes.h"
#include "libdw3000.h"
#include "dw3000.h"
#include "ranging_struct.h"

#define CS_PIN DECK_GPIO_IO1

// LOCO deck alternative IRQ and RESET pins(IO_2, IO_4) instead of default (RX1, TX1), leaving UART1 free for use
#ifdef CONFIG_DECK_ADHOCDECK_USE_ALT_PINS
#define GPIO_PIN_IRQ      DECK_GPIO_IO2

#ifndef ADHOCDECK_ALT_PIN_RESET
#define GPIO_PIN_RESET    DECK_GPIO_IO4
#else
#define GPIO_PIN_RESET 	ADHOCDECK_ALT_PIN_RESET
#endif

#define EXTI_PortSource EXTI_PortSourceGPIOB
#define EXTI_PinSource    EXTI_PinSource5
#define EXTI_LineN          EXTI_Line5
#elif defined(CONFIG_DECK_ADHOCDECK_USE_UART2_PINS)
#define GPIO_PIN_IRQ 	  DECK_GPIO_TX2
#define GPIO_PIN_RESET 	DECK_GPIO_RX2
#define EXTI_PortSource EXTI_PortSourceGPIOA
#define EXTI_PinSource 	EXTI_PinSource2
#define EXTI_LineN 		  EXTI_Line2
#else
#define GPIO_PIN_IRQ 	  DECK_GPIO_RX1
#define GPIO_PIN_RESET 	DECK_GPIO_TX1
#define EXTI_PortSource EXTI_PortSourceGPIOC
#define EXTI_PinSource 	EXTI_PinSource11
#define EXTI_LineN 		  EXTI_Line11
#endif

#define DEFAULT_RX_TIMEOUT 0xFFFFF

static address_t MY_UWB_ADDRESS;
static bool isInit = false;
#ifdef CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
static bool isUWBStart = false;
#endif
static TaskHandle_t uwbTaskHandle = 0;
static TaskHandle_t uwbTxTaskHandle = 0;
static TaskHandle_t uwbRxTaskHandle = 0;
static TaskHandle_t uwbRangingTaskHandle = 0;
static SemaphoreHandle_t algoSemaphore;
static QueueHandle_t txQueue;
static QueueHandle_t rxQueue;

static logVarId_t idVelocityX, idVelocityY, idVelocityZ;
static float velocity;

/* rx buffer used in rx_callback */
static uint8_t rxBuffer[RX_BUFFER_SIZE];
Timestamp_Tuple_t TfBuffer[Tf_BUFFER_POOL_SIZE] = {0};
static int TfBufferIndex = 0;
static int rangingSeqNumber = 1;

/* log block */
int16_t distanceTowards[RANGING_TABLE_SIZE + 1] = {0};
uint32_t LOG_RANGING_COUNT = 0;

static void txCallback() {
  dwTime_t txTime;
  dwt_readtxtimestamp((uint8_t *) &txTime.raw);
  TfBufferIndex++;
  TfBufferIndex %= Tf_BUFFER_POOL_SIZE;
  TfBuffer[TfBufferIndex].seqNumber = rangingSeqNumber;
  TfBuffer[TfBufferIndex].timestamp = txTime;
}

static void rxCallback() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint32_t dataLength = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
  if (dataLength != 0 && dataLength <= FRAME_LEN_MAX) {
    dwt_readrxdata(rxBuffer, dataLength - FCS_LEN, 0); /* No need to read the FCS/CRC. */
  }
  dwTime_t rxTime;
  dwt_readrxtimestamp((uint8_t *) &rxTime.raw);
  Ranging_Message_With_Timestamp_t rxMessageWithTimestamp;
  rxMessageWithTimestamp.rxTime = rxTime;
  Ranging_Message_t *rangingMessage = (Ranging_Message_t *) &rxBuffer;
  rxMessageWithTimestamp.rangingMessage = *rangingMessage;
  xQueueSendFromISR(rxQueue, &rxMessageWithTimestamp, &xHigherPriorityTaskWoken);
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void rxTimeoutCallback() {
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void rxErrorCallback() {
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static int uwbInit() {
  /* Need to make sure DW IC is in IDLE_RC before proceeding */
  while (!dwt_checkidlerc()) {

  }
#ifdef CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
  while (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    vTaskDelay(100);
  }
  while (dwt_configure(&config) == DWT_ERROR) {
    vTaskDelay(100);
  }
#else
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    return DWT_ERROR;
  }
  if (dwt_configure(&config) == DWT_ERROR) {
    return DWT_ERROR;
  }
#endif
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);

  /* Configure Antenna Delay */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Auto re-enable receiver after a frame reception failure (except a frame
   * wait timeout), the receiver will re-enable to re-attempt reception. */
  dwt_or32bitoffsetreg(SYS_CFG_ID, 0, SYS_CFG_RXAUTR_BIT_MASK);
  dwt_setrxtimeout(DEFAULT_RX_TIMEOUT);

  dwt_setcallbacks(&txCallback, &rxCallback, &rxTimeoutCallback, &rxErrorCallback, NULL, NULL);
  /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and
   * RX errors). */
  dwt_setinterrupt(SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK,
                   0, DWT_ENABLE_INT);

  /* Clearing the SPI ready interrupt */
  dwt_write32bitreg(SYS_STATUS_ID,
                    SYS_STATUS_RCINIT_BIT_MASK | SYS_STATUS_SPIRDY_BIT_MASK);

  algoSemaphore = xSemaphoreCreateMutex();

  return DWT_SUCCESS;
}

static void uwbTxTask(void *parameters) {
  systemWaitStart();
#ifdef CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
  while (!isUWBStart) {
    vTaskDelay(500);
  }
#endif
  while (txQueue == 0) {
    DEBUG_PRINT("txQueue is not init\n");
    vTaskDelay(M2T(1000));
  }

  Ranging_Message_t packetCache;

  while (true) {
    if (xQueueReceive(txQueue, &packetCache, portMAX_DELAY)) {
      dwt_forcetrxoff();
      dwt_writetxdata(packetCache.header.msgLength, (uint8_t *) &packetCache, 0);
      dwt_writetxfctrl(packetCache.header.msgLength + FCS_LEN, 0, 1);
      /* Start transmission. */
      if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) ==
          DWT_ERROR) {
        DEBUG_PRINT("uwbTxTask:  TX ERROR\n");
      }
    }
  }
}

int16_t computeDistance(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp,
                        Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr,
                        Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf) {
  LOG_RANGING_COUNT++;
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
//    DEBUG_PRINT("isErrorOccurred\n");
    isErrorOccurred = true;
  }

  if (tRound2 < 0 || tReply2 < 0) {
//    DEBUG_PRINT("tRound2 < 0 || tReply2 < 0\n");
    isErrorOccurred = true;
  }

  if (isErrorOccurred) {
    return 0;
  }

  return distance;
}

void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp) {
  Ranging_Message_t *rangingMessage = &rangingMessageWithTimestamp->rangingMessage;
  address_t neighborAddress = rangingMessage->header.srcAddress;
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
  if (rangingMessage->header.filter & (1 << (MY_UWB_ADDRESS % 16))) {
    /* retrieve body unit */
    uint8_t bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
    for (int i = 0; i < bodyUnitCount; i++) {
      if (rangingMessage->bodyUnits[i].address == MY_UWB_ADDRESS) {
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
        neighborRangingTable->Tf.timestamp.full && neighborRangingTable->Rf.timestamp.full &&
        neighborRangingTable->Tp.seqNumber == neighborRangingTable->Rp.seqNumber &&
        neighborRangingTable->Tf.seqNumber == neighborRangingTable->Rf.seqNumber) {
      int16_t distance = computeDistance(neighborRangingTable->Tp, neighborRangingTable->Rp,
                                         Tr_Rr_Candidate.Tr, Tr_Rr_Candidate.Rr,
                                         neighborRangingTable->Tf, neighborRangingTable->Rf);
      if (distance > 0) {
        neighborRangingTable->distance = distance;
        distanceTowards[neighborRangingTable->neighborAddress] = distance;
      } else {
//        DEBUG_PRINT("distance is not updated since some error occurs");
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

static void generateRangingMessage(Ranging_Message_t *rangingMessage) {
#ifdef ENABLE_BUS_BOARDING_SCHEME
  sortRangingTableSet(&rangingTableSet);
#endif
  int8_t bodyUnitNumber = 0;
  rangingSeqNumber++;
  int curSeqNumber = rangingSeqNumber;
  rangingMessage->header.filter = 0;
  /* generate message body */
  for (set_index_t index = rangingTableSet.fullQueueEntry; index != -1;
       index = rangingTableSet.setData[index].next) {
    Ranging_Table_t *table = &rangingTableSet.setData[index].data;
    if (bodyUnitNumber >= MAX_BODY_UNIT_NUMBER) {
      break; //TODO test 1023 byte
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
  float velocityX = logGetFloat(idVelocityX);
  float velocityY = logGetFloat(idVelocityY);
  float velocityZ = logGetFloat(idVelocityZ);
  velocity = sqrt(pow(velocityX, 2) + pow(velocityY, 2) + pow(velocityZ, 2));
  /* velocity in cm/s */
  rangingMessage->header.velocity = (short) (velocity * 100);
}

static void uwbRxTask(void *parameters) {
  systemWaitStart();
#ifdef CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
  while (!isUWBStart) {
    vTaskDelay(500);
  }
#endif
  while (rxQueue == 0) {
    DEBUG_PRINT("rxQueue is not init\n");
    vTaskDelay(M2T(1000));
  }
  Ranging_Message_With_Timestamp_t rxPacketCache;

  while (true) {
    if (xQueueReceive(rxQueue, &rxPacketCache, portMAX_DELAY)) {
      processRangingMessage(&rxPacketCache);
    }
  }
}

static void uwbRangingTask(void *parameters) {
  systemWaitStart();
#ifdef CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
  while (!isUWBStart) {
    vTaskDelay(500);
  }
#endif
  while (txQueue == 0) {
    DEBUG_PRINT("txQueue is not init\n");
    vTaskDelay(M2T(1000));
  }
  /* velocity log variable id */
  idVelocityX = logGetVarId("stateEstimate", "vx");
  idVelocityY = logGetVarId("stateEstimate", "vy");
  idVelocityZ = logGetVarId("stateEstimate", "vz");

  Ranging_Message_t txPacketCache;
  while (true) {
    generateRangingMessage(&txPacketCache);
    xQueueSend(txQueue, &txPacketCache, portMAX_DELAY);
    vTaskDelay(TX_PERIOD_IN_MS);
  }
}

static void uwbTask(void *parameters) {
  systemWaitStart();
#ifdef CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
  if (uwbInit() == DWT_SUCCESS) {
    isUWBStart = true;
  }
#endif

  while (1) {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
      do {
        xSemaphoreTake(algoSemaphore, portMAX_DELAY);
        dwt_isr();
        xSemaphoreGive(algoSemaphore);
#ifdef CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
        vTaskDelay(M2T(5)); // TODO check if necessary since increasing FREERTOS_HEAP_SIZE
#endif
      } while (digitalRead(GPIO_PIN_IRQ) != 0);
    }
  }
}
static uint8_t spiTxBuffer[196];
static uint8_t spiRxBuffer[196];
static uint16_t spiSpeed = SPI_BAUDRATE_2MHZ;

/************ Low level ops for libdw **********/
static void spiWrite(const void *header, size_t headerLength, const void *data,
                     size_t dataLength) {
  spiBeginTransaction(spiSpeed);
  digitalWrite(CS_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memcpy(spiTxBuffer + headerLength, data, dataLength);
  spiExchange(headerLength + dataLength, spiTxBuffer, spiRxBuffer);
  digitalWrite(CS_PIN, HIGH);
  spiEndTransaction();
}

static void spiRead(const void *header, size_t headerLength, void *data,
                    size_t dataLength) {
  spiBeginTransaction(spiSpeed);
  digitalWrite(CS_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memset(spiTxBuffer + headerLength, 0, dataLength);
  spiExchange(headerLength + dataLength, spiTxBuffer, spiRxBuffer);
  memcpy(data, spiRxBuffer + headerLength, dataLength);
  digitalWrite(CS_PIN, HIGH);
  spiEndTransaction();
}

#ifdef CONFIG_DECK_ADHOCDECK_USE_ALT_PINS
void __attribute__((used)) EXTI5_Callback(void)
#elif defined(CONFIG_DECK_ADHOCDECK_USE_UART2_PINS)
void __attribute__((used)) EXTI2_Callback(void)
#else
void __attribute__((used)) EXTI11_Callback(void)
#endif
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Unlock interrupt handling task
  vTaskNotifyGiveFromISR(uwbTaskHandle, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken) {
    portYIELD();
  }
}

static void spiSetSpeed(dwSpiSpeed_t speed) {
  if (speed == dwSpiSpeedLow) {
    spiSpeed = SPI_BAUDRATE_2MHZ;
  } else if (speed == dwSpiSpeedHigh) {
    spiSpeed = SPI_BAUDRATE_21MHZ;
  }
}

static void delayms(unsigned int delay) { vTaskDelay(M2T(delay)); }

static void reset(void) {
  digitalWrite(GPIO_PIN_RESET, 0);
  vTaskDelay(M2T(10));
  digitalWrite(GPIO_PIN_RESET, 1);
  vTaskDelay(M2T(10));
}
extern dwOps_t dwt_ops = {
    .spiRead = spiRead,
    .spiWrite = spiWrite,
    .spiSetSpeed = spiSetSpeed,
    .delayms = delayms,
    .reset = reset
};

static void pinInit() {
  EXTI_InitTypeDef EXTI_InitStructure;

  spiBegin();

  // Set up interrupt
  SYSCFG_EXTILineConfig(EXTI_PortSource, EXTI_PinSource);

  EXTI_InitStructure.EXTI_Line = EXTI_LineN;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // Init pins
  pinMode(CS_PIN, OUTPUT);
  pinMode(GPIO_PIN_RESET, OUTPUT);
  pinMode(GPIO_PIN_IRQ, INPUT);

  //Reset the DW3000 chip
  dwt_ops.reset();
}

static void queueInit() {
  txQueue = xQueueCreate(TX_QUEUE_SIZE, TX_QUEUE_ITEM_SIZE);
  rxQueue = xQueueCreate(RX_QUEUE_SIZE, RX_QUEUE_ITEM_SIZE);
}

static void uwbStart() {
  /* Create UWB Task */
  xTaskCreate(uwbTask, ADHOC_DECK_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbTaskHandle);
  xTaskCreate(uwbTxTask, ADHOC_DECK_TX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbTxTaskHandle);
  xTaskCreate(uwbRxTask, ADHOC_DECK_RX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRxTaskHandle);
  xTaskCreate(uwbRangingTask, ADHOC_DECK_RANGING_TX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRangingTaskHandle);
}
/*********** Deck driver initialization ***************/
static void dwm3000Init(DeckInfo *info) {
  pinInit();
  queueInit();
  rangingTableSetInit(&rangingTableSet);
#ifndef CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
  if (uwbInit() == DWT_SUCCESS) {
    uwbStart();
    isInit = true;
  } else {
    isInit = false;
  }
#else
  uwbStart();
  isInit = true;
#endif
}

static bool dwm3000Test() {
  if (!isInit) {
    DEBUG_PRINT("Error while initializing DWM3000\n");
  }

  return isInit;
}

static const DeckDriver dwm3000_deck = {
    .vid = 0xBC,
    .pid = 0x06,
    .name = "DWM3000",

#ifdef CONFIG_DECK_ADHOCDECK_USE_ALT_PINS
    .usedGpio = DECK_USING_IO_1 | DECK_USING_IO_2 | DECK_USING_IO_4,
#elif defined(CONFIG_DECK_ADHOCDECK_USE_UART2_PINS)
    .usedGpio = DECK_USING_IO_1 | DECK_USING_UART2,
#else
    .usedGpio = DECK_USING_IO_1 | DECK_USING_UART1,
#endif
    .usedPeriph = DECK_USING_SPI,
    .requiredEstimator = kalmanEstimator,
#ifdef ADHOCDECK_NO_LOW_INTERFERENCE
    .requiredLowInterferenceRadioMode = false,
#else
    .requiredLowInterferenceRadioMode = true,
#endif

    .init = dwm3000Init,
    .test = dwm3000Test,
};

DECK_DRIVER(dwm3000_deck);

PARAM_GROUP_START(deck)
        PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, DWM3000, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(Ranging)
        LOG_ADD(LOG_INT16, distTo1, distanceTowards + 1)
        LOG_ADD(LOG_INT16, distTo2, distanceTowards + 2)
        LOG_ADD(LOG_INT16, distTo3, distanceTowards + 3)
        LOG_ADD(LOG_INT16, distTo4, distanceTowards + 4)
        LOG_ADD(LOG_INT16, distTo5, distanceTowards + 5)
        LOG_ADD(LOG_INT16, distTo6, distanceTowards + 6)
        LOG_ADD(LOG_INT16, distTo7, distanceTowards + 7)
        LOG_ADD(LOG_INT16, distTo8, distanceTowards + 8)
        LOG_ADD(LOG_FLOAT, velocity, &velocity)
        LOG_ADD(LOG_UINT32, LOG_RANGING_COUNT, &LOG_RANGING_COUNT)
LOG_GROUP_STOP(Ranging)

PARAM_GROUP_START(ADHOC)
        PARAM_ADD_CORE(PARAM_UINT16 | PARAM_PERSISTENT, MY_UWB_ADDRESS, &MY_UWB_ADDRESS)
PARAM_GROUP_STOP(ADHOC)