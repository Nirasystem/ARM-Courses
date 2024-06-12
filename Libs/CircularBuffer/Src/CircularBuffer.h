#ifndef _CIRCULAR_BUFFER_H_
#define _CIRCULAR_BUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

//                         Circular Buffer (FIFO)
//            Overflow 0                           Overflow 1
//  _______________________________   |  ________________________________
// |_______|_____________|_________|  | |_____|_________________|________|
// 0       R             W         S  | 0     W                 R        S
// 
// R: RPos, W: WPos, S: Size, OVF: Overflow
//
// Formulas: Available (Read)
// OVF  R    W
//  0   R <  W   =>   W - R
//  1   R >  W   =>   S - R + W 
//  0   R == W   =>   0
//  1   R == W   =>   S
//  Available = S * OVF + W - R
//
// Formulas: Space (Write)
// OVF  R    W
//  0   R <  W   =>   S - W + R
//  1   R >  W   =>   R - W
//  0   R == W   =>   S
//  1   R == W   =>   0
//  Space = S * !OVF + R - W

typedef enum {
  UART_CircularBuffer_Result_Ok,
  UART_CircularBuffer_Result_NoSpace,
  UART_CircularBuffer_Result_NoAvailable,
} UART_CircularBuffer_Result;

typedef struct {
  UART_HandleTypeDef*     HUART;
  uint8_t*                Buffer;
  int16_t                 WPos;
  int16_t                 RPos;
  int16_t                 Size;
  int16_t                 PendingBytes;
  uint8_t                 Overflow;
  uint8_t                 InReceive;
  uint8_t                 InTransmit;
} UART_CircularBuffer;

void UART_CircularBuffer_init(UART_CircularBuffer* buf, UART_HandleTypeDef* huart, uint8_t* dataBuf, int16_t size);

int16_t UART_CircularBuffer_available(UART_CircularBuffer* buf);
int16_t UART_CircularBuffer_space(UART_CircularBuffer* buf);

int16_t UART_CircularBuffer_availableUncheck(UART_CircularBuffer* buf);
int16_t UART_CircularBuffer_spaceUncheck(UART_CircularBuffer* buf);

UART_CircularBuffer_Result UART_CircularBuffer_writeBytes(UART_CircularBuffer* buf, uint8_t* data, uint32_t len);
UART_CircularBuffer_Result UART_CircularBuffer_readBytes(UART_CircularBuffer* buf, uint8_t* data, uint32_t len);
UART_CircularBuffer_Result UART_CircularBuffer_getBytes(UART_CircularBuffer* buf, uint8_t* data, uint32_t len);

UART_CircularBuffer_Result UART_CircularBuffer_writeFmt(UART_CircularBuffer* buf, const char* fmt, ...);

void UART_CircularBuffer_moveWritePos(UART_CircularBuffer* buf, int16_t step);
void UART_CircularBuffer_moveReadPos(UART_CircularBuffer* buf, int16_t step);

int16_t UART_CircularBuffer_directAvailable(UART_CircularBuffer* buf);
int16_t UART_CircularBuffer_directSpace(UART_CircularBuffer* buf);
int16_t UART_CircularBuffer_directAvailableAt(UART_CircularBuffer* buf, int16_t index);
int16_t UART_CircularBuffer_directSpaceAt(UART_CircularBuffer* buf, int16_t index);

uint8_t* UART_CircularBuffer_getReadDataPtr(UART_CircularBuffer* buf);
uint8_t* UART_CircularBuffer_getWriteDataPtr(UART_CircularBuffer* buf);
uint8_t* UART_CircularBuffer_getReadDataPtrAt(UART_CircularBuffer* buf, int16_t index);
uint8_t* UART_CircularBuffer_getWriteDataPtrAt(UART_CircularBuffer* buf, int16_t index);

int16_t UART_CircularBuffer_findByte(UART_CircularBuffer* buf, uint8_t val);
int16_t UART_CircularBuffer_findByteAt(UART_CircularBuffer* buf, int16_t index, uint8_t val);
int16_t UART_CircularBuffer_findPat(UART_CircularBuffer* buf, uint8_t* pat, int16_t len);
int16_t UART_CircularBuffer_compareAt(UART_CircularBuffer* buf, int16_t index, uint8_t* pat, int16_t len);


// UART Rx APIs
void UART_CircularBuffer_receive(UART_CircularBuffer* buf);
void UART_CircularBuffer_handleRx(UART_CircularBuffer* buf);

void UART_CircularBuffer_receiveIdle(UART_CircularBuffer* buf);
void UART_CircularBuffer_handleRxEvent(UART_CircularBuffer* buf, uint16_t len);
// UART Tx APIS
void UART_CircularBuffer_transmit(UART_CircularBuffer* buf);
void UART_CircularBuffer_handleTx(UART_CircularBuffer* buf);

void UART_CircularBuffer_resetIO(UART_CircularBuffer* buf);


#ifdef __cplusplus
};
#endif

#endif
