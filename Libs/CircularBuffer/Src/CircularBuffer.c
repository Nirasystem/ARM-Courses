#include "CircularBuffer.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

void UART_CircularBuffer_init(UART_CircularBuffer* buf, UART_HandleTypeDef* huart, uint8_t* dataBuf, int16_t size) {
  buf->HUART = huart;
  buf->Buffer = dataBuf;
  buf->Size = size;
  buf->Overflow = 0;
  buf->RPos = 0;
  buf->WPos = 0;
  buf->InReceive = 0;
  buf->InTransmit = 0;
}

int16_t UART_CircularBuffer_availableUncheck(UART_CircularBuffer* buf) {
  return buf->Size * buf->Overflow + buf->WPos - buf->RPos;
}
int16_t UART_CircularBuffer_spaceUncheck(UART_CircularBuffer* buf) {
  return buf->Size * (!buf->Overflow) + buf->RPos - buf->WPos;
}

int16_t UART_CircularBuffer_available(UART_CircularBuffer* buf) {
  int16_t bytesLen = buf->HUART->hdmarx ? 
                      buf->PendingBytes - __HAL_DMA_GET_COUNTER(buf->HUART->hdmarx) :
                      buf->PendingBytes - buf->HUART->RxXferCount;
  
  if (bytesLen > 0) {
    UART_CircularBuffer_moveWritePos(buf, bytesLen);
    buf->PendingBytes -= bytesLen;
  }
  
  return UART_CircularBuffer_availableUncheck(buf);
}
int16_t UART_CircularBuffer_space(UART_CircularBuffer* buf) {
  int16_t bytesLen = buf->HUART->hdmatx ? 
                      buf->PendingBytes - __HAL_DMA_GET_COUNTER(buf->HUART->hdmatx) :
                      buf->PendingBytes - buf->HUART->TxXferCount;
  
  if (bytesLen > 0) {
    UART_CircularBuffer_moveReadPos(buf, bytesLen);
    buf->PendingBytes -= bytesLen;
  }
  
  return UART_CircularBuffer_spaceUncheck(buf);
}

UART_CircularBuffer_Result UART_CircularBuffer_writeBytes(UART_CircularBuffer* buf, uint8_t* data, uint32_t len) {
  if (UART_CircularBuffer_space(buf) < len) {
    return UART_CircularBuffer_Result_NoSpace;
  }
  
  if (buf->WPos + len >= buf->Size) {
    int16_t tmpLen = buf->Size - buf->WPos;
    
    memcpy(&buf->Buffer[buf->WPos], data, tmpLen);
    data += tmpLen;
    len -= tmpLen;
    // buf->WPos = (buf->WPos + tmpLen) % buf->Size
    buf->WPos = 0;
    buf->Overflow = 1;
  }
  
  if (len > 0) {
    memcpy(&buf->Buffer[buf->WPos], data, len);
    buf->WPos += len;
  }
    
  return UART_CircularBuffer_Result_Ok;
}
UART_CircularBuffer_Result UART_CircularBuffer_readBytes(UART_CircularBuffer* buf, uint8_t* data, uint32_t len) {
  if (UART_CircularBuffer_available(buf) < len) {
    return UART_CircularBuffer_Result_NoAvailable;
  }
  
  if (buf->RPos + len >= buf->Size) {
    int16_t tmpLen = buf->Size - buf->RPos;
    
    memcpy(data, &buf->Buffer[buf->RPos], tmpLen);
    data += tmpLen;
    len -= tmpLen;
    // buf->RPos = (buf->RPos + tmpLen) % buf->Size
    buf->RPos = 0;
    buf->Overflow = 0;
  }
  
  if (len > 0) {
    memcpy(data, &buf->Buffer[buf->RPos], len);
    buf->RPos += len;
  }
    
  return UART_CircularBuffer_Result_Ok;
}
UART_CircularBuffer_Result UART_CircularBuffer_getBytes(UART_CircularBuffer* buf, uint8_t* data, uint32_t len) {
  if (UART_CircularBuffer_available(buf) < len) {
    return UART_CircularBuffer_Result_NoAvailable;
  }
  
  int16_t rpos = buf->RPos;
  
  if (rpos + len >= buf->Size) {
    int16_t tmpLen = buf->Size - rpos;
    
    memcpy(data, &buf->Buffer[rpos], tmpLen);
    data += tmpLen;
    len -= tmpLen;
    // rpos = (rpos + tmpLen) % buf->Size
    rpos = 0;
    buf->Overflow = 0;
  }
  
  if (len > 0) {
    memcpy(data, &buf->Buffer[rpos], len);
  }
    
  return UART_CircularBuffer_Result_Ok;
}

UART_CircularBuffer_Result UART_CircularBuffer_writeFmt(UART_CircularBuffer* buf, const char* fmt, ...) {
  char temp[64];
  
  va_list args;
  va_start(args, fmt);
  int16_t len = vsnprintf(temp, sizeof(temp) - 1, fmt, args);
  va_end(args);
  
  return UART_CircularBuffer_writeBytes(buf, (uint8_t*) temp, len);
}


void UART_CircularBuffer_moveWritePos(UART_CircularBuffer* buf, int16_t step) {
  buf->WPos += step;
  if (buf->WPos >= buf->Size) {
    buf->WPos = 0;
    buf->Overflow = 1;
  }
}

void UART_CircularBuffer_moveReadPos(UART_CircularBuffer* buf, int16_t step) {
  buf->RPos += step;
  if (buf->RPos >= buf->Size) {
    buf->RPos = 0;
    buf->Overflow = 0;
  }
}
int16_t UART_CircularBuffer_directAvailable(UART_CircularBuffer* buf) {
  return buf->Overflow ? buf->Size - buf->RPos :
                         buf->WPos - buf->RPos;
}
int16_t UART_CircularBuffer_directSpace(UART_CircularBuffer* buf) {
  return buf->Overflow ? buf->RPos - buf->WPos :
                         buf->Size - buf->WPos;
}
int16_t UART_CircularBuffer_directAvailableAt(UART_CircularBuffer* buf, int16_t index) {
  int16_t len = UART_CircularBuffer_availableUncheck(buf);
  int16_t dirLen = UART_CircularBuffer_directAvailable(buf);
  if (len == dirLen) {
      return len - index;
  }
  else {
      return dirLen > index ? dirLen - index :
                              buf->WPos - (index - dirLen);
  }
}
int16_t UART_CircularBuffer_directSpaceAt(UART_CircularBuffer* buf, int16_t index) {
  int16_t len = UART_CircularBuffer_spaceUncheck(buf);
  int16_t dirLen = UART_CircularBuffer_directSpace(buf);
  if (len == dirLen) {
      return len - index;
  }
  else {
      return dirLen > index ? dirLen - index :
                              buf->RPos - (index - dirLen);
  }
}
uint8_t* UART_CircularBuffer_getReadDataPtr(UART_CircularBuffer* buf) {
  return &buf->Buffer[buf->RPos];
}
uint8_t* UART_CircularBuffer_getWriteDataPtr(UART_CircularBuffer* buf) {
  return &buf->Buffer[buf->WPos];
}
uint8_t* UART_CircularBuffer_getReadDataPtrAt(UART_CircularBuffer* buf, int16_t index) {
  index += buf->RPos;

  if (index >= buf->Size) {
      index %= buf->Size;
  }

  return &buf->Buffer[index];
}
uint8_t* UART_CircularBuffer_getWriteDataPtrAt(UART_CircularBuffer* buf, int16_t index) {
  index += buf->WPos;

  if (index >= buf->Size) {
      index %= buf->Size;
  }

  return &buf->Buffer[index];
}

int16_t UART_CircularBuffer_findByte(UART_CircularBuffer* buf, uint8_t val) {
  if (UART_CircularBuffer_available(buf) <= 0) {
    return -1;
  }
  
  int16_t offset = 0;
  int16_t len = UART_CircularBuffer_directAvailable(buf);
  uint8_t* base = UART_CircularBuffer_getReadDataPtr(buf);
  uint8_t* p = memchr(base, val, len);
  
  if (p == NULL && buf->Overflow) {
    offset = len;
    base = buf->Buffer;
    len = buf->WPos;
    p = memchr(base, val, len);
  }
  
  return p == NULL ? -1 : (int16_t)(p - base) + offset;
}

int16_t UART_CircularBuffer_findPat(UART_CircularBuffer* buf, uint8_t* pat, int16_t len) {
  if (UART_CircularBuffer_available(buf) < len) {
    return -1;
  }

  int16_t index = 0;

  while ((index = UART_CircularBuffer_findByteAt(buf, index, *pat)) >= 0) {
    if (UART_CircularBuffer_compareAt(buf, index, pat, len) == 0) {
      return index;
    }
    index++;
  }

  return -1;
}

int16_t UART_CircularBuffer_findByteAt(UART_CircularBuffer* buf, int16_t offset, uint8_t val) {
  int16_t tmpLen = 0;
  uint8_t* pStart = UART_CircularBuffer_getReadDataPtrAt(buf, offset);
  uint8_t* pEnd;

  if (UART_CircularBuffer_availableUncheck(buf) < offset) {
      return -1;
  }

  tmpLen = UART_CircularBuffer_directAvailableAt(buf, offset);
  pEnd = memchr(pStart, val, tmpLen);
  if (!pEnd && (tmpLen + offset) < UART_CircularBuffer_available(buf)) {
      pStart = buf->Buffer;
      pEnd = memchr(pStart, val, buf->WPos);
  }

  return pEnd != NULL ? (int16_t)(pEnd - pStart) + offset : -1;
}

int16_t UART_CircularBuffer_compareAt(UART_CircularBuffer* buf, int16_t index, uint8_t* val, int16_t len) {
  int16_t result;
  int16_t tmpLen;

  if (len == 0) {
    return 0;
  }

  if (UART_CircularBuffer_availableUncheck(buf) - index < len) {
    return -2;
  }

  tmpLen = UART_CircularBuffer_directAvailableAt(buf, index);
  if (tmpLen < len) {
      if ((result = (int8_t) memcmp(UART_CircularBuffer_getReadDataPtrAt(buf, index), val, tmpLen)) != 0) {
          return result;
      }

      index += tmpLen;
      val += tmpLen;
      len -= tmpLen;
  }

  return (int8_t) memcmp(UART_CircularBuffer_getReadDataPtrAt(buf, index), val, len);
}

// --------------------- UART Rx APIs --------------------------
void UART_CircularBuffer_receive(UART_CircularBuffer* buf) {
  if (!buf->InReceive) {
    buf->PendingBytes = UART_CircularBuffer_directSpace(buf);
    if (buf->PendingBytes > 0) {
      buf->InReceive = 1;
      // Check IT or DMA
      if (buf->HUART->hdmarx) {
        HAL_UART_Receive_DMA(
            buf->HUART, 
            UART_CircularBuffer_getWriteDataPtr(buf), 
            buf->PendingBytes
        );      
      }
      else {
        HAL_UART_Receive_IT(
            buf->HUART, 
            UART_CircularBuffer_getWriteDataPtr(buf), 
            buf->PendingBytes
        );
      }
    }
  }
}
void UART_CircularBuffer_handleRx(UART_CircularBuffer* buf) {
  if (buf->InReceive) {
    buf->InReceive = 0;
    UART_CircularBuffer_moveWritePos(buf, buf->PendingBytes);
    UART_CircularBuffer_receive(buf);
  }
}
void UART_CircularBuffer_receiveIdle(UART_CircularBuffer* buf) {
  if (!buf->InReceive) {
    buf->PendingBytes = UART_CircularBuffer_directSpace(buf);
    if (buf->PendingBytes > 0) {
      buf->InReceive = 1;
      // Check IT or DMA
      if (buf->HUART->hdmarx) {
        HAL_UARTEx_ReceiveToIdle_DMA(
          buf->HUART, 
          UART_CircularBuffer_getWriteDataPtr(buf), 
          buf->PendingBytes
        );        
      }
      else {
        HAL_UARTEx_ReceiveToIdle_IT(
          buf->HUART, 
          UART_CircularBuffer_getWriteDataPtr(buf), 
          buf->PendingBytes
        );
      }
    }
  }
}
void UART_CircularBuffer_handleRxEvent(UART_CircularBuffer* buf, uint16_t len) {
  if (buf->InReceive) {
    buf->InReceive = 0;
    buf->PendingBytes -= len;
    UART_CircularBuffer_moveWritePos(buf, len);
    UART_CircularBuffer_receiveIdle(buf);
  }
}

// --------------------- UART Tx APIs --------------------------
void UART_CircularBuffer_transmit(UART_CircularBuffer* buf) {
  if (!buf->InTransmit) {
    buf->PendingBytes = UART_CircularBuffer_directAvailable(buf);
    if (buf->PendingBytes > 0) {
      buf->InTransmit = 1;
      // Check IT or DMA
      if (buf->HUART->hdmatx) {
        HAL_UART_Transmit_DMA(
            buf->HUART, 
            UART_CircularBuffer_getReadDataPtr(buf), 
            buf->PendingBytes
        );
      }
      else {
        HAL_UART_Transmit_IT(
            buf->HUART, 
            UART_CircularBuffer_getReadDataPtr(buf), 
            buf->PendingBytes
        );
      }
    }
  }
}
void UART_CircularBuffer_handleTx(UART_CircularBuffer* buf) {
  if (buf->InTransmit) {
    buf->InTransmit = 0;
    UART_CircularBuffer_moveReadPos(buf, buf->PendingBytes);
    UART_CircularBuffer_transmit(buf);
  }
}

void UART_CircularBuffer_resetIO(UART_CircularBuffer* buf) {
  buf->InReceive = 0;
  buf->InTransmit = 0;
  buf->RPos = 0;
  buf->WPos = 0;
  buf->Overflow = 0;
  buf->PendingBytes = 0;
}
