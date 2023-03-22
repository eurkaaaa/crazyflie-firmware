#ifndef __TC_H__
#define __TC_H__

#include "stdint.h"
#include "adhocdeck.h"
#include "flooding_struct.h"
#include "FreeRTOS.h"
#include "dwTypes.h"
#include "ranging_struct.h"

#define MAX_TC_BODY_UNIT_NUMBER 30
#define TC_CHECK_TABLE_SIZE 10

typedef portTickType Time_t;
typedef short set_index_t;


/* Tc Package */
/* Tc Body Unit */
typedef struct {
  uint16_t dstAddress; // 2 byte
  int16_t distance;// 2 byte
} __attribute__((packed)) Tc_Body_Unit_t; // 6 byte

/* Tc Message Header */
typedef struct {
  uint16_t srcAddress; // 2 byte
  uint16_t msgSequence; // 2 byte
  uint16_t msgLength; // 2 byte
  uint8_t timeToLive; // 1 byte
} __attribute__((packed)) Tc_Message_Header_t; // 7 byte

/* Tc Message */
typedef struct {
  Tc_Message_Header_t header; // 7 byte
  Tc_Body_Unit_t bodyUnits[MAX_TC_BODY_UNIT_NUMBER]; // 6 * MAX_TC_BODY_UNIT_NUMBER byte
} __attribute__((packed)) Tc_Message_t; // 7 + 6 * MAX_Tc_BODY_UNIT_NUMBER byte

/* Queue Constants */
#define TC_RX_QUEUE_SIZE 10
#define TC_RX_QUEUE_ITEM_SIZE sizeof (UWB_Packet_t)

/* Tc Constants */
#define TC_INTERVAL 500
#define TC_TIME_TO_LIVE 6

/* Tc Operations */
void tcInit();

int generateTcMessage(Tc_Message_t *tcMessage);

void processTcMessage(Tc_Message_t *tcMessage);

bool checkTcMessage(Tc_Message_t *tcMessage);

/* Ms Operations */

#endif