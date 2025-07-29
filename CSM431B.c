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
uint16_t crcInput[64];
volatile uint16_t writeFrameHead[9];
volatile _flags CSM431Bflags;
volatile _cycleCounts CycleCounts;
volatile _counts ReadNumberFrameCounts;
volatile _counts ReadDataFrameCounts;
volatile _counts WriteFrame1Counts;
volatile _counts WriteFrame2Counts;
volatile _data ReceiveData;

void VaribleInit()
{

}

// CSM431BInit - Initializes the CSM431B sensor
void CSM431BInit(uint16_t isAlreadyConfigured)
{
    switch (_deviceState)
    {
    case Init:
        CSM431Bflags.isDelay100us = _true;
        CSM431Bflags.isDelay200us = _true;
        break;
    
    case OnReset:
        RST_SET;
        if (CSM431Bflags.isDelay100us)
        {
            _deviceState = OnSetDelay;
            CSM431Bflags.isDelay200us = _false;
        }
        break;
    case OnSetDelay:
        RST_RESET;
        if (CSM431Bflags.isDelay200us)
        {            
            CSM431Bflags.isCSM431BSeted = _true;
            _deviceState = ReadyToConfig;
        }
        break;
    case Ready:
        
        break;
    }

    ReadNumberFrameCounts.frame2ByteDivideCount = 8;
    WriteFrame1Counts.frame2ByteDivideCount = SendFrameLengthCan1 / 2;
    WriteFrame2Counts.frame2ByteDivideCount = SendFrameLengthCan2 / 2;
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


void OnConfigure(uint16_t address, uint32_t value)
{
    uint16_t sendDataAddress = 0xFF00 + (address & 0xFF);
    uint16_t crcOutput = 0;
    crcInput[0] = 0x07;
    crcInput[1] = 0x02;
    crcInput[2] = 0xFF;
    crcInput[3] = address & 0xFF;
    crcInput[4] = (value & 0xFF000000) >> 24;
    crcInput[5] = (value & 0x00FF0000) >> 16;
    crcInput[6] = (value & 0x0000FF00) >> 8;
    crcInput[7] = (value & 0x000000FF);

    CS_LOW;
    SPI_transmit16Bits(SPIA_BASE, 0x0702);
    SPI_transmit16Bits(SPIA_BASE, sendDataAddress);
    SPI_transmit16Bits(SPIA_BASE, (value & 0xFFFF0000) >> 16);
    SPI_transmit16Bits(SPIA_BASE, (value & 0xFFFF));
    crcOutput = Crc16X25(crcInput, 8);
    CS_HIGH;
}

void ReceiveEvent()
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
        _readState = ReadFree;
        break;

    case ReadFree:
        break;
    }
    
}

void SendEvent(uint16_t *data)
{    
    switch (_writeState)
    {
    case Init:
        
        break;

    case WriteFrameHeadCan1:
        writeFrameHead[0] = FrameHead;
        writeFrameHead[1] = SendFrameLengthCan1 + 2;
        writeFrameHead[2] = WriteDataOnTransmit;
        writeFrameHead[3] = SpiToCan1;
        writeFrameHead[4] = NowFrameType;
        writeFrameHead[5] = SendFrameIDCan1 & 0xFF;        
        if (NowFrameType == CanfdStandardFrame)
        {
            writeFrameHead[6] = (SendFrameIDCan1 & 0x0700) >> 8;
            writeFrameHead[7] = writeFrameHead[8] = 0;
        }
        else
        {
            writeFrameHead[6] = (SendFrameIDCan1 & 0x0000FF00) >> 8;
            writeFrameHead[7] = (SendFrameIDCan1 & 0x00FF0000) >> 16;
            writeFrameHead[8] = (SendFrameIDCan1 & 0x1F000000) >> 24;
        }

        CS_LOW;
        SPI_transmit16Bits(SPIA_BASE, (writeFrameHead[0] & 0xFF) << 8 | (writeFrameHead[1] & 0xFF));
        SPI_transmit16Bits(SPIA_BASE, (writeFrameHead[2] & 0xFF) << 8 | (writeFrameHead[3] & 0xFF));
        SPI_transmit16Bits(SPIA_BASE, (writeFrameHead[4] & 0xFF) << 8 | (writeFrameHead[5] & 0xFF));
        SPI_transmit16Bits(SPIA_BASE, (writeFrameHead[6] & 0xFF) << 8 | (writeFrameHead[7] & 0xFF));
        SPI_transmit16Bits(SPIA_BASE, ((writeFrameHead[8] & 0xFF) << 8) | (data[0] & 0xFF));
        _writeState = WriteFrameDataCan1;
        break;

    case WriteFrameDataCan1:
        if (! CSM431Bflags.isWriteFrame1End)
        {
            SPI_transmit16Bits(SPIA_BASE, (data[WriteFrame1Counts.frame2ByteDivideIndex + 1] & 0xFF) << 8 | (data[WriteFrame1Counts.frame2ByteDivideIndex + 2] & 0xFF));
        }

        WriteFrame1Counts.frame2ByteDivideCount++;
        if (WriteFrame1Counts.frame2ByteDivideIndex >= WriteFrame1Counts.frame2ByteDivideCount)
        {
            CS_HIGH;
            CSM431Bflags.isDelay50us = _false;
            CSM431Bflags.isWriteFrame1End = _true;
        }
        
        if (CSM431Bflags.isWriteFrame1End && CSM431Bflags.isDelay50us)
        {
            _writeState = WriteFrameHeadCan2;
            CSM431Bflags.isWriteFrame1End = _false;
            WriteFrame1Counts.frame2ByteDivideIndex = 0;
        }
        break;

    case WriteFrameHeadCan2:
        writeFrameHead[0] = FrameHead;
        writeFrameHead[1] = SendFrameLengthCan2 + 2;
        writeFrameHead[2] = WriteDataOnTransmit;
        writeFrameHead[3] = SpiToCan2;
        writeFrameHead[4] = NowFrameType;
        writeFrameHead[5] = SendFrameIDCan2 & 0xFF;
        if (NowFrameType == CanfdStandardFrame)
        {
            writeFrameHead[6] = (SendFrameIDCan2 & 0x0700) >> 8;
            writeFrameHead[7] = writeFrameHead[8] = 0;
        }
        else
        {
            writeFrameHead[6] = (SendFrameIDCan2 & 0x0000FF00) >> 8;
            writeFrameHead[7] = (SendFrameIDCan2 & 0x00FF0000) >> 16;
            writeFrameHead[8] = (SendFrameIDCan2 & 0x1F000000) >> 24;
        }

        CS_LOW;
        SPI_transmit16Bits(SPIA_BASE, (writeFrameHead[0] & 0xFF) << 8 | (writeFrameHead[1] & 0xFF));
        SPI_transmit16Bits(SPIA_BASE, (writeFrameHead[2] & 0xFF) << 8 | (writeFrameHead[3] & 0xFF));
        SPI_transmit16Bits(SPIA_BASE, (writeFrameHead[4] & 0xFF) << 8 | (writeFrameHead[5] & 0xFF));
        SPI_transmit16Bits(SPIA_BASE, (writeFrameHead[6] & 0xFF) << 8 | (writeFrameHead[7] & 0xFF));
        SPI_transmit16Bits(SPIA_BASE, ((writeFrameHead[8] & 0xFF) << 8) | (data[0] & 0xFF));
        _writeState = WriteFrameDataCan2;
        break;

    case WriteFrameDataCan2:
        if (!CSM431Bflags.isWriteFrame2End)
        {
            SPI_transmit16Bits(SPIA_BASE, (data[WriteFrame2Counts.frame2ByteDivideIndex + 1] & 0xFF) << 8 | (data[WriteFrame2Counts.frame2ByteDivideIndex + 2] & 0xFF));
        }

        WriteFrame2Counts.frame2ByteDivideCount++;
        if (WriteFrame2Counts.frame2ByteDivideIndex >= WriteFrame2Counts.frame2ByteDivideCount)
        {
            CS_HIGH;
            CSM431Bflags.isDelay50us = _false;
            CSM431Bflags.isWriteFrame2End = _true;
        }

        if (CSM431Bflags.isWriteFrame2End && CSM431Bflags.isDelay50us)
        {
            _writeState = WriteFree;
            CSM431Bflags.isWriteFrame2End = _false;
            WriteFrame2Counts.frame2ByteDivideIndex = 0;
        }
        break;

    case WriteFree:
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
        if (CycleCounts.cycle200usCount >= 200 / InterruptIntervalUS + 1)
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

void ReadyToInitCSM431B()
{
    CSM431Bflags.isDelay100us = _false;
    _readState = OnReset;
}

uint16_t SendCommand()
{
    if (_writeState == WriteFree)
    {
        _writeState = WriteFrameHeadCan1;
        WriteFrame1Counts.frame2ByteDivideIndex = 0;
        WriteFrame2Counts.frame2ByteDivideIndex = 0;
        return NoError;
    }
    else if (_writeState == Init)
    {
        return DeviceNotInited;
    }
    else
    {
        return CommandConflict; 
    }
}

uint16_t ReceiveCommand()
{
    if (_readState == ReadFree)
    {
        _readState = ReadNumberNotNull;
        return NoError;
    }
    else if (_readState == Init)
    {
        return DeviceNotInited;
    }
    else
    {
        return CommandConflict;
    }
}
