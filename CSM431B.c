// This Source Code Form is subject to the terms of the MIT License.
// If a copy of the MIT was not distributed with this file, You can obtain one at https://opensource.org/licenses/MIT.
// Copyright (C) GX_WangYifan.
// All Rights Reserved.


#include "CSM431B.h"

#ifdef DEBUG
#include <stdint-gcc.h>
#include "DEBUG.h"
#endif

#define RST_SET     GPIO_writePin(GPIO_RST, 0)
#define RST_RESET   GPIO_writePin(GPIO_RST, 1)
#define CFG_LOW     GPIO_writePin(GPIO_CFG, 0)
#define CFG_HIGH    GPIO_writePin(GPIO_CFG, 1)
#define CS_LOW      GPIO_writePin(SPI_CS_GPIO, 0)
#define CS_HIGH     GPIO_writePin(SPI_CS_GPIO, 1)

uint16_t _readState = Init;
uint16_t _writeState = Init;
uint16_t _deviceState = Init;
volatile _flags CSM431Bflags;
volatile _cycleCounts CycleCounts;
volatile _counts ReadNumberFrameCounts;
volatile _counts ReadDataFrameCounts;
volatile _data ReceiveData;

// CSM431BInit - Initializes the CSM431B sensor
void CSM431BInit(uint16_t isAlreadyConfigured)
{
    switch (_deviceState)
    {
    case Init:
        
        break;
    
    case ReadyToConfigure:
        if (CSM431Bflags.isCSM431BSeted)
        {
            // If already set, skip initialization
            return;
        }
        
        break;
    }
    ReadNumberFrameCounts.frame2ByteDivideCount = 8;
    ReadNumberFrameCounts.frame1ByteDivideCount = 15;
    if (isAlreadyConfigured)
    {
        // If already configured, skip initialization
        return;
    }
}

void SendReadCommand(uint16_t isNull)
{
    uint16_t _readyToReadNumber = 0;
    uint16_t sendData;
    uint16_t FeedbackSendReadFrame[11];
    if (CSM431Bflags.isDelay50us && ! CSM431Bflags.isSendNumberCommandFrame)
    {
        CS_LOW;
        if (!isNull)
        {
            sendData = (readNumberCommandFrame[ReadNumberFrameCounts.frame2ByteDivideIndex] & 0xFF) << 8 + (readNumberCommandFrame[ReadNumberFrameCounts.frame2ByteDivideIndex + 1] & 0xFF);

            SPI_transmit16Bits(SPIA_BASE, sendData);
        }
        else
        {
            ReceiveData.recviveNumberFrameData[ReadNumberFrameCounts.frame2ByteDivideIndex] = SPI_receive16Bits(SPIA_BASE, CPOL, 0, CPHA);
        }
        
        CSM431Bflags.isSendNumberCommandFrame = _true;
        _readyToReadNumber = FeedbackSendReadFrame[7];

    }
    return _readyToReadNumber;
}

uint16_t OnRead(uint16_t isNull)
{
    uint16_t sendData;

}

uint16_t sendWriteCommand()
{

}

void OnWrite()
{
    
}

void ReadEvent()
{
    uint16_t readyToReadNumber = 0;
    switch (_readState)
    {
    case Init:
        
        break;
    case ReadNumberNotNull:
        if (ReadNumberFrameCounts.frame2ByteDivideIndex == 0 && CSM431Bflags.isDelay50us)
        {
            CSM431Bflags.isMosiNull = _false;
            SendReadCommand(CSM431Bflags.isMosiNull);
            ReadNumberFrameCounts.frame2ByteDivideIndex++;
            return;
        }
        if (ReadNumberFrameCounts.frame2ByteDivideIndex >= ReadNumberFrameCounts.frame2ByteNotNullCount)
        {
            _readState = ReadNumberWithNull;
            CSM431Bflags.isMosiNull = _true;
        }
        else
        {
            SendReadCommand(CSM431Bflags.isMosiNull);
            ReadNumberFrameCounts.frame2ByteDivideIndex++;
        }        
        break;

    case ReadNumberWithNull:
        if (ReadNumberFrameCounts.frame2ByteDivideIndex >= ReadNumberFrameCounts.frame2ByteDivideCount)
        {
            CSM431Bflags.isFrameEnd = _true;
            CS_HIGH;
            CSM431Bflags.isDelay50us = _false;
            ReadNumberFrameCounts.frame2ByteDivideIndex = 0;
            ReceiveData.receviveFrameNumber = (ReceiveData.recviveNumberFrameData[0] & 0xFF) << 24
                + (ReceiveData.recviveNumberFrameData[1] & 0xFF) << 16
                + (ReceiveData.recviveNumberFrameData[2] & 0xFF) << 8
                + (ReceiveData.recviveNumberFrameData[3] & 0xFF);

            if (readyToReadNumber != 0)
            {
                _readState = ReadDataNotNull;
            }
            else
            {
                _readState = ReadNumberNotNull;
            }

        }
        else
        {
            SendReadCommand(CSM431Bflags.isMosiNull);
        }

        if (readyToReadNumber != 0)
        {
            ReadDataFrameCounts.frame2ByteDivideCount = readyToReadNumber / 2 + 3;
        }
        break;

        /******************************************************************/
    case ReadDataNotNull:
        if (ReadDataFrameCounts.frame2ByteDivideIndex == 0 && CSM431Bflags.isDelay50us)
        {
            CSM431Bflags.isMosiNull = _false;
            OnRead(CSM431Bflags.isMosiNull);
            ReadDataFrameCounts.frame2ByteDivideIndex++;
            return;
        }
        if (ReadDataFrameCounts.frame2ByteDivideIndex >= ReadDataFrameCounts.frame2ByteNotNullCount)
        {
            _readState = ReadDataWithNull;
            CSM431Bflags.isMosiNull = _true;
        }
        else
        {
            OnRead(CSM431Bflags.isMosiNull);
            ReadDataFrameCounts.frame2ByteDivideIndex++;
        }
        break;

    case ReadDataWithNull:

        break;

    }    
    
}

void WriteEvent()
{
    switch (_writeState)
    {
    case Init:
        
        break;

    case WriteNumberNotNull:
        
        break;

    case WriteNumberWithNull:
        // Code to handle writing number with null
        break;

    case WriteDataNotNull:
        // Code to handle writing data not null
        break;

    case WriteDataWithNull:
        // Code to handle writing data with null
        break;

    default:
        // Handle unexpected state
        break;
    }
}

//50us at least interval
void CreateFrameInterval()
{
    if (CSM431Bflags.isDelay50us)
    {
        CycleCounts.cycle50usCount = 0;
        return;
    }
    else
    {        
        CycleCounts.cycle50usCount++;
        if (CycleCounts.cycle50usCount >= 50 / InterruptIntervalUS + 1)
        {
            CSM431Bflags.isDelay50us = _true;
            CycleCounts.cycle50usCount = 0;
        }

        return;
    }
}

//100us at least interval
void CreateRstLowInterval()
{
    if (CSM431Bflags.isDelay100us)
    {
        CycleCounts.cycle100usCount = 0;
        return;
    }
    else
    {
        CycleCounts.cycle100usCount++;
        if (CycleCounts.cycle100usCount >= 100 / InterruptIntervalUS + 1)
        {
            CSM431Bflags.isDelay100us = _true;
            CycleCounts.cycle100usCount = 0;
        }

        return;
    }
}

//200us at least interval
void CreateRstHighInterval()
{
    if (CSM431Bflags.isDelay200us)
    {
        CycleCounts.cycle200usCount = 0;
        return;
    }
    else
    {
        CycleCounts.cycle200usCount++;
        if (CycleCounts.cycle200usCount >= 100 / InterruptIntervalUS + 1)
        {
            CSM431Bflags.isDelay200us = _true;
            CycleCounts.cycle200usCount = 0;
        }

        return;
    }
}

uint16_t Crc16X25(const uint16_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    while (len-- > 0)
    {
        crc = (crc >> 8) ^ crc16X25Table[(crc ^ *data++) & 0xff];
    }

    return (~crc) & 0xFFFF;
}

void OnReceiveData(uint16_t* data, uint16_t len)
{
    CSM431Bflags.isReadyToReceive = _true;
}


