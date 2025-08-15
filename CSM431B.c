// This Source Code Form is subject to the terms of the MIT License.
// If a copy of the MIT was not distributed with this file, You can obtain one at https://opensource.org/licenses/MIT.
// Copyright (C) GX_WangYifan.
// All Rights Reserved.

#include "CSM431B.h"

#ifdef _release
#include "device.h"
#include "f28004x_spi.h"
#include "f28004x_gpio.h"
//#include "board.h"
//#include "f28004x_examples.h"
//#include "hw_memmap.h"
#include "stdint.h"
#include "gpio.h"
#endif

#ifdef _debug
#include <stdint-gcc.h>
//#include "DEBUG.h"
#endif

#define RST_ENABLE      GPIO_writePin(GPIO_RST, 0)
#define RST_DISABLE     GPIO_writePin(GPIO_RST, 1)
#define CFG_ENABLE      GPIO_writePin(GPIO_CFG, 0)
#define CFG_DISABLE     GPIO_writePin(GPIO_CFG, 1)
#define CS_LOW          GPIO_writePin(SPI_CS_GPIO, 0)
#define CS_HIGH         GPIO_writePin(SPI_CS_GPIO, 1)

uint16_t cycleNumber;
uint16_t XintDivide;
uint16_t debugCount;
uint16_t errorCode = NoError;
uint16_t _readState = _Init;
uint16_t _writeState = _Init;
uint16_t _deviceState = _Init;
uint16_t _errorFeedbackState = _Init;
uint16_t canChannel = SpiToCan1;
uint16_t nackNumber;
uint16_t crcInput[64];
uint16_t FeedbackSendReadFrame[11];
volatile uint16_t writeFrameHead[9];
volatile uint16_t receiveDataCan1[64];
volatile uint16_t receiveDataCan2[64];
volatile _flags CSM431Bflags;
volatile _rxCanData RxCanData;
volatile _cycleCounts CycleCounts;
volatile _counts ConfigFrameCounts;
volatile _counts ReadNumberFrameCounts;
volatile _counts ReadDataFrameCounts;
volatile _counts ErrorFeedabckFrameCounts;
volatile _counts WriteFrame1Counts;
volatile _counts WriteFrame2Counts;
volatile _data ReceiveData;
volatile _errorData ErrorFeedbackData;
volatile _serialFault SPIFaultCount;
volatile _canFault CanFD1FaultCount;
volatile _canFault CanFD2FaultCount;

uint16_t Crc16X25(const uint16_t* data, uint16_t len);
uint16_t C1188A(uint16_t* data, uint16_t len);
void SendEvent();
void ReceiveEvent();

void SendConfigFrame()
{
    uint16_t crcOutput = 0;
    CS_LOW;
    uint16_t i = 0;
    for (i = 0; i < 4; i++)
    {
        crcInput[i] = configFrameHead[i];
    }
    crcInput[4] = configIndex[ConfigFrameCounts.frameCountIndex] & 0xFF;
    crcInput[5] = (valueOnConfig[2 * ConfigFrameCounts.frameCountIndex] & 0xFF00) >> 8;
    crcInput[6] = valueOnConfig[2 * ConfigFrameCounts.frameCountIndex] & 0x00FF;
    crcInput[7] = (valueOnConfig[2 * ConfigFrameCounts.frameCountIndex + 1] & 0xFF00) >> 8;
    crcInput[8] = valueOnConfig[2 * ConfigFrameCounts.frameCountIndex + 1] & 0x00FF;
    crcOutput = Crc16X25(crcInput, 9);
    
    SPI_transmit24Bits(SPIA_BASE,
        ((configFrameHead[0] & 0xFF) << 16)
        | ((configFrameHead[1] & 0xFF) << 8)
        | (configFrameHead[2] & 0xFF),
        0);
    SPI_transmit24Bits(SPIA_BASE,
        ((configFrameHead[3] & 0xFF) << 16)
        | ((configIndex[ConfigFrameCounts.frameCountIndex] & 0xFF) << 8)
        | ((valueOnConfig[2 * ConfigFrameCounts.frameCountIndex] & 0xFF00) >> 8)
        , 0);
    SPI_transmit24Bits(SPIA_BASE,
        ((valueOnConfig[2 * ConfigFrameCounts.frameCountIndex] & 0xFF) << 16)
        | ((valueOnConfig[2 * ConfigFrameCounts.frameCountIndex + 1] & 0xFF00))
        | ((valueOnConfig[2 * ConfigFrameCounts.frameCountIndex + 1] & 0xFF))
        , 0);
    SPI_transmit16Bits(SPIA_BASE, ((crcOutput & 0xFF) << 8) | ((crcOutput & 0xFF00) >> 8));
    
    // SPI_transmit24Bits(SPIA_BASE, 0xAC0702, 0);
    // SPI_transmit24Bits(SPIA_BASE, 0xFF0000, 0);
    // SPI_transmit24Bits(SPIA_BASE, 0x000000, 0);
    // SPI_transmit16Bits(SPIA_BASE, 0xAC07);

    // SPI_transmit24Bits(SPIA_BASE, 0XAC0301, 0);
    // SPI_transmit16Bits(SPIA_BASE, 0XFF01);    
    // crcInput[0] = 0xAC;
    // crcInput[1] = 0x03;
    // crcInput[2] = 0x01;
    // crcInput[3] = 0xFF;
    // crcInput[4] = 0x01;
    // crcOutput = Crc16X25(crcInput, 5);
    // SPI_transmit16Bits(SPIA_BASE, ((crcOutput & 0xFF) << 8) | ((crcOutput & 0xFF00) >> 8));

    CS_HIGH;
}

void  SendFeedbackFrame()
{
    uint16_t SendFeedbackFrame[3];
    uint32_t SendFeedbackFrame32[2];
    uint16_t status = 0;
    CS_LOW;
    SendFeedbackFrame32[0] = SPI_receive24Bits(SPIA_BASE, SPI_DATA_BIG_ENDIAN, 0x000000, 0);
    //SendFeedbackFrame32[1] = SPI_receive24Bits(SPIA_BASE, SPI_DATA_BIG_ENDIAN, 0x000000, 0);
    SendFeedbackFrame[0] = SPI_receive16Bits(SPIA_BASE, SPI_DATA_BIG_ENDIAN, 0x000000, 0);
    SendFeedbackFrame[1] = SPI_receive16Bits(SPIA_BASE, SPI_DATA_BIG_ENDIAN, 0x000000, 0);
    //SendFeedbackFrame[0] = SPI_receive16Bits(SPIA_BASE, SPI_DATA_BIG_ENDIAN, 0, TxDelay);
    //SendFeedbackFrame[1] = SPI_receive16Bits(SPIA_BASE, SPI_DATA_BIG_ENDIAN, 0, TxDelay);
    //SendFeedbackFrame[2] = SPI_receive16Bits(SPIA_BASE, SPI_DATA_BIG_ENDIAN, 0, TxDelay);
    CS_HIGH;
    status = (SendFeedbackFrame32[2] & 0xFF00) >> 8;
    // if (status == ConfigNoError)
    // {
    //     CycleCounts.cycleTimeoutCount = 0;
    //     _deviceState = WriteConfigFrame;
    // }
    // else
    // {
    //     CycleCounts.cycleTimeoutCount++;
    //     ConfigFrameCounts.frameCountIndex--;
    //     _deviceState = _Ready;
    // }
   
}

uint16_t SendReadCommand(uint16_t isNull)
{
    uint16_t ReadyToReadNumber = 0;
    uint16_t sendData;
    if (CSM431Bflags.isDelay50us && !CSM431Bflags.isSendNumberCommandFrame)
    {
        CS_LOW;
        if (!isNull)
        {
            sendData = (readNumberCommandFrame[ReadNumberFrameCounts.frame2ByteDivideIndex] & 0xFF) << 8 + (readNumberCommandFrame[ReadNumberFrameCounts.frame2ByteDivideIndex + 1] & 0xFF);

            SPI_transmit16Bits(SPIA_BASE, sendData);
        }
        else
        {
            ReceiveData.receiveNumberFrameData[ReadNumberFrameCounts.frame2ByteDivideIndex] = SPI_receive16Bits(SPIA_BASE, SPI_DATA_BIG_ENDIAN, 0, TxDelay);
        }

        CSM431Bflags.isSendNumberCommandFrame = _true;
        ReadyToReadNumber = FeedbackSendReadFrame[7];

    }
    return ReadyToReadNumber;
}

void OnRead(uint16_t isNull)
{

    if (! isNull)
    {
        SPI_transmit16Bits(SPIA_BASE, 0xAC00);
        SPI_transmit16Bits(SPIA_BASE, 0x03FF);        
    }
    else
    {
        ReceiveData.receiveDataFrameData[ReadDataFrameCounts.frame2ByteDivideIndex] = SPI_receive16Bits(SPIA_BASE, SPI_DATA_BIG_ENDIAN, 0, TxDelay);
    }

}

void ReceiveDecode()
{
    uint16_t length;
    uint16_t channel;
    uint32_t canID;
    uint16_t i = 0;
    if (cycleNumber >= ReceiveData.receiveFrameNumber - 1)
    {
        return;
    }

    if (ReceiveData.receiveDataFrameData[cycleNumber] == 0 && ReceiveData.receiveDataFrameData[cycleNumber + 1] == 0xAC)
    {
        for (i = 0; i < 128 - 1; i++)
        {
            ReceiveData.receiveDataFrameData[i] = ReceiveData.receiveDataFrameData[i + 1];
        }        
    }
    if (ReceiveData.receiveDataFrameData[cycleNumber] != 0xAC)
    {
        //ERROR
    }

    length = ReceiveData.receiveDataFrameData[cycleNumber + 1];
    channel = ReceiveData.receiveDataFrameData[cycleNumber + 3];
    canID = (ReceiveData.receiveDataFrameData[cycleNumber + 7])
        | (ReceiveData.receiveDataFrameData[cycleNumber + 8] << 8);
    canID = canID << 16;
    canID |= (ReceiveData.receiveDataFrameData[cycleNumber + 5])
        | (ReceiveData.receiveDataFrameData[cycleNumber + 6] << 8);
    
    switch (channel)
    {
    case SpiToCan1:
        RxCanData.receiveIDCan = canID;
        for (i = 0; i < (length - 5); i++)
        {
            RxCanData.receiveDataCan[i] = ReceiveData.receiveDataFrameData[cycleNumber + 9 + i];
        }
        break;
    
    case SpiToCan2:
        RxCanData.receiveIDCan = canID;
        for (i = 0; i < (length - 5); i++)
        {
            RxCanData.receiveDataCan[i] = ReceiveData.receiveDataFrameData[cycleNumber + 9 + i];
        }
        break;

//    default:
//        errorCode = ChannelError;
//        _deviceState = Error;
//        break;
    }
    cycleNumber += (length + 3);    
}

void OnErrorJudge(uint16_t* data)
{
    //#pragma region FaultCount
    if (data[0] != 0xAC0E && data[1] != 0x06FF) 
    {
        return;
    }
    
    SPIFaultCount.frameLengthFaultCount = (data[2] & 0xFF00) >> 8;
    SPIFaultCount.channelFaultCount = data[2] & 0xFF;
    SPIFaultCount.crcFaultCount = data[3];
    SPIFaultCount.frameTypeFaultCount = (data[4] & 0xFF00) >> 8;
    SPIFaultCount.frameIDFaultCount = data[4] & 0xFF;
    CanFD1FaultCount.receiveFaultCount = (data[5] & 0xFF00) >> 8;
    CanFD1FaultCount.sendFaultCount = data[5] & 0xFF;
    CanFD1FaultCount.negitiveFaultCount = (data[6] & 0xFF00) >> 8;
    CanFD1FaultCount.busoffFaultCount = data[6] & 0xFF;
    CanFD2FaultCount.receiveFaultCount = (data[7] & 0xFF00) >> 8;
    CanFD2FaultCount.sendFaultCount = data[7] & 0xFF;
    CanFD2FaultCount.negitiveFaultCount = (data[8] & 0xFF00) >> 8;
    CanFD2FaultCount.busoffFaultCount = data[8] & 0xFF;

#ifdef _noSlaveDebug
    if (CanFD1FaultCount.receiveFaultCount > 0 ||
        CanFD1FaultCount.sendFaultCount > 0 || CanFD1FaultCount.negitiveFaultCount > 0 ||
        CanFD1FaultCount.busoffFaultCount > 0 || CanFD2FaultCount.receiveFaultCount > 0 ||
        CanFD2FaultCount.sendFaultCount > 0 || CanFD2FaultCount.negitiveFaultCount > 0 ||
        CanFD2FaultCount.busoffFaultCount > 0)
    {
        if (!CSM431Bflags.isChannelSwitchedin100ms && (_deviceState == _Ready || _deviceState == _Busy || _deviceState == Error))
        {
            _deviceState = Error;
            return;
        }        
        canChannel = canChannel == SpiToCan1 ? SpiToCan2 : SpiToCan1;
        CSM431Bflags.isChannelSwitchedin100ms = _false;
    }
#endif
    
#ifndef _noSlaveDebug
    if (CanFD1FaultCount.receiveFaultCount > 0 ||
        CanFD1FaultCount.sendFaultCount > 0 || CanFD1FaultCount.negitiveFaultCount > 0 ||
        CanFD1FaultCount.busoffFaultCount > 0 || CanFD2FaultCount.receiveFaultCount > 0 ||
        CanFD2FaultCount.sendFaultCount > 0 || CanFD2FaultCount.negitiveFaultCount > 0 ||
        CanFD2FaultCount.busoffFaultCount > 0 || nackNumber >= 3)
    {
        if (!CSM431Bflags.isChannelSwitchedin100ms)
        {
            _deviceState = Error;
            return;
        }
        canChannel = canChannel == SpiToCan1 ? SpiToCan2 : SpiToCan1;
        CSM431Bflags.isChannelSwitchedin100ms = _false;
    }
#endif
    
//#pragma endregion
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
    crcOutput = Crc16X25(crcInput, 8);
    SPI_transmit16Bits(SPIA_BASE, 0x0702);
    SPI_transmit16Bits(SPIA_BASE, sendDataAddress);
    SPI_transmit16Bits(SPIA_BASE, (value & 0xFFFF0000) >> 16);
    SPI_transmit16Bits(SPIA_BASE, (value & 0xFFFF));
    SPI_transmit16Bits(SPIA_BASE, (crcOutput & 0xFFFF));
    CS_HIGH;
}

void CSM431BDeviceEvent()
{
    switch (_deviceState)
    {
    case _Init:
        CSM431Bflags.isDelay50us = _true;
        CSM431Bflags.isDelay100us = _true;
        CSM431Bflags.isDelay200ms = _true;
        CFG_DISABLE;
        CS_HIGH;
        break;

    case ReadyToCfgConfig:
        CFG_ENABLE;
        _deviceState = CfgConfig;
        CSM431Bflags.isDelay50us = _false;

    case CfgConfig:
        
        if (CSM431Bflags.isDelay50us)
        {
            RST_ENABLE;
            _deviceState = OnReset;
            CSM431Bflags.isDelay100us = _false;
        }
        break;

    case OnReset:
        if (CSM431Bflags.isDelay100us)
        {
            _deviceState = OnDelay200msPart1;
            CSM431Bflags.isDelay200ms = _false;
        }
        break;

    case OnDelay200msPart1:
        RST_DISABLE;
        if (CSM431Bflags.isDelay200ms)
        {
            _deviceState = WriteConfigFrame;
            CSM431Bflags.isDelay200us = _false;
        }
        break;    

    case WriteConfigFrame:
        if (CycleCounts.cycleTimeoutCount >= TimeoutCount)
        {
            CSM431Bflags.isInitTimeout = _true;
            CycleCounts.cycleTimeoutCount = 0;
            _deviceState = Error;
            return;
        }               

        SendConfigFrame();
        ConfigFrameCounts.frameCountIndex++;
        _deviceState = OnDelay3ms;
        CSM431Bflags.isDelay3ms = _false;
        break;

    case OnDelay3ms:
        if (CSM431Bflags.isDelay3ms)
        {
            _deviceState = WriteFeedbackFrame;
        }
        
        break;

    case WriteFeedbackFrame:
        //if (CSM431Bflags.isIntSet)
        {
            SendFeedbackFrame();

            if (ConfigFrameCounts.frameCountIndex >= RegisterConfigLength)
            {
                CSM431Bflags.isDelay3ms = _false;
                _deviceState = OnCfgReset;
            }
            else
            {
                _deviceState = PauseToLoop;
                CSM431Bflags.isDelay200us = _false;
            }

            CSM431Bflags.isIntSet = _false;
        }       
        
        break;

    case PauseToLoop:
        if (CSM431Bflags.isDelay200us)
        {
            _deviceState = WriteConfigFrame;
        }
        break;

    case OnCfgReset:
        if (CSM431Bflags.isDelay3ms)
        {
            CFG_DISABLE;
            _deviceState = OnDelay200msPart2;
            CSM431Bflags.isDelay200ms = _false;
        }
        
        break;

    case OnDelay200msPart2:
        if (CSM431Bflags.isDelay200ms)
        {
            //CFG_DISABLE;
            _writeState = WriteFree;
            _readState = ReadFree;
            _errorFeedbackState = ErrorReadFree;
            _deviceState = _Ready;
        }

        break;

    case _Busy:
        break;

    case _Ready:        
        if (CSM431Bflags.isReceiveCommandOn && _readState == ReadFree)
        {
            _deviceState = _Busy;
            CSM431Bflags.isReceiveCommandOn = _false;
            _readState = ReadNumber;
            ReadNumberFrameCounts.frame2ByteDivideIndex = 0;
            ReadDataFrameCounts.frame2ByteDivideIndex = 0;
            //ReceiveEvent();
            break;
        }
        // else if (CSM431Bflags.isCheckErrorCommandOn)
        // {
        //     _deviceState = _Busy;
        //     CSM431Bflags.isCheckErrorCommandOn = _false;
        //     _errorFeedbackState = ErrorReadCommand;
        //     break;
        //     //CheckErrorFeedbackEvent();
        // }
        else if (CSM431Bflags.isSendCommandOn && _writeState == WriteFree)
        {
            _deviceState = _Busy;
            CSM431Bflags.isSendCommandOn = _false;
            _writeState = WriteFrameHeadCan1;
            ReadDataFrameCounts.frame2ByteDivideIndex = 0;
            break;
            //SendEvent();
        }
                
        break;

    case Error:
        CSM431Bflags.isErrorOccurred = _true;
        break;
    }

    ReadNumberFrameCounts.frame2ByteDivideCount = 5;
    WriteFrame1Counts.frame2ByteDivideCount = SendFrameLengthCan / 2;
    WriteFrame2Counts.frame2ByteDivideCount = SendFrameLengthCan / 2;
    ErrorFeedabckFrameCounts.frame2ByteDivideCount = 9;
}

void ReceiveEvent()
{
    uint16_t tempRead;
    uint16_t ReadyToReadNumber = 0;

    switch (_readState)
    {
    case _Init:
        break;

    case ReadNumber:
        
        _deviceState = _Busy;
        
        if (ReadNumberFrameCounts.frame2ByteDivideIndex == 0)
        {
            CS_LOW;
            SPI_transmit16Bits(SPIA_BASE, 0xAC00);
            SPI_transmit16Bits(SPIA_BASE, 0x07FF);
        }
        
        if (ReadNumberFrameCounts.frame2ByteDivideIndex >= ReadNumberFrameCounts.frame2ByteDivideCount)
        {
            CS_HIGH;
            if (ReceiveData.receiveNumberFrameData[0] == 0xAC06)
            {
                ReceiveData.receiveFrameNumber = ((ReceiveData.receiveNumberFrameData[3] & 0xFF00) << 8)
                    | ((ReceiveData.receiveNumberFrameData[2] & 0xFF) << 8)
                    | ((ReceiveData.receiveNumberFrameData[2] & 0xFF00) >> 8);
            }
            else if (ReceiveData.receiveNumberFrameData[0] == 0x00AC)
            {
                ReceiveData.receiveFrameNumber = (ReceiveData.receiveNumberFrameData[3] & 0xFF) << 16
                    | (ReceiveData.receiveNumberFrameData[3] & 0xFF00)
                    | (ReceiveData.receiveNumberFrameData[2] & 0xFF);
            }
            
            
            ReadDataFrameCounts.frame2ByteDivideCount = ReceiveData.receiveFrameNumber / 2;
            ReadDataFrameCounts.frame2ByteDivideCount += (ReceiveData.receiveFrameNumber % 2 == 1 ? 1 : 0);
                        
            CSM431Bflags.isDelay50usForRead = _false;
            ReadNumberFrameCounts.frame2ByteDivideIndex = 0;
            _readState = ReadInterval;
        }
        else
        {            
            ReceiveData.receiveNumberFrameData[ReadNumberFrameCounts.frame2ByteDivideIndex] = SPI_receive16Bits(SPIA_BASE, SPI_DATA_BIG_ENDIAN, 0, TxDelay);
            ReadNumberFrameCounts.frame2ByteDivideIndex++;
        }        break;

    case ReadInterval:
        _deviceState = _Busy;
        if (CSM431Bflags.isDelay50usForRead)
        {            
            CS_LOW;
            SPI_transmit16Bits(SPIA_BASE, 0xAC00);
            SPI_transmit16Bits(SPIA_BASE, 0x03FF);
            _readState = ReadData;
        }
        
        break;
        // case ReadNumberWithNull:
    //     if (ReadNumberFrameCounts.frame2ByteDivideIndex >= ReadNumberFrameCounts.frame2ByteDivideCount)
    //     {
    //         CSM431Bflags.isFrameEnd = _true;
    //         CS_HIGH;
    //         CSM431Bflags.isDelay50us = _false;
    //         ReadNumberFrameCounts.frame2ByteDivideIndex = 0;
    //         

    //         if (ReceiveData.receviveFrameNumber != 0)
    //         {
    //             _readState = ReadDataNotNull;
    //             ReadDataFrameCounts.frame2ByteDivideCount = ReadyToReadNumber / 2 + (ReadyToReadNumber % 2 == 1 ? 1 : 0);
    //         }
    //         else
    //         {
    //             _readState = ReadNumberNotNull;
    //         }
    //     }
    //     else
    //     {
    //         SendReadCommand(CSM431Bflags.isMosiNull);
    //     }
    //     break;

    case ReadData:
        _deviceState = _Busy;
        // if (ReadDataFrameCounts.frame2ByteDivideIndex >= ReadDataFrameCounts.frame2ByteDivideCount)
        // {
        //     CS_HIGH;
        //     ReadDataFrameCounts.frame2ByteDivideIndex = 0;
        //     CSM431Bflags.isDelay50usForRead = _false;
        //     _readState = ReadOverInterval;
        //     cycleNumber = 0;
        //     ReceiveDecode();
        //     break;
        // }
        
        // tempRead = SPI_receive16Bits(SPIA_BASE, SPI_DATA_BIG_ENDIAN, 0, TxDelay);        
        
        // ReceiveData.receiveDataFrameData[2 * ReadDataFrameCounts.frame2ByteDivideIndex] = (tempRead & 0xFF00) >> 8;
        // ReceiveData.receiveDataFrameData[2 * ReadDataFrameCounts.frame2ByteDivideIndex + 1] = tempRead & 0xFF;
        // ReadDataFrameCounts.frame2ByteDivideIndex++;

        if (ReadDataFrameCounts.frame2ByteDivideIndex >= ReadDataFrameCounts.frame2ByteDivideCount)
                {
                    CS_HIGH;
                    ReadDataFrameCounts.frame2ByteDivideIndex = 0;
                    CSM431Bflags.isDelay50usForRead = _false;
                    _readState = ReadOverInterval;
                    cycleNumber = 0;
                    ReceiveDecode();
                    break;
                }

                tempRead = SPI_receive16Bits(SPIA_BASE, SPI_DATA_BIG_ENDIAN, 0, TxDelay);

                ReceiveData.receiveDataFrameData[2 * ReadDataFrameCounts.frame2ByteDivideIndex] = (tempRead & 0xFF00) >> 8;
                ReceiveData.receiveDataFrameData[2 * ReadDataFrameCounts.frame2ByteDivideIndex + 1] = tempRead & 0xFF;
                ReadDataFrameCounts.frame2ByteDivideIndex++;

        break;

    case ReadOverInterval:
        _deviceState = _Busy;
        if (CSM431Bflags.isDelay50usForRead)
        {
            _readState = ReadFree;
            _deviceState = _Ready;
        }       
        
        break;

    case ReadFree:
        ReadDataFrameCounts.frame2ByteDivideIndex = 0;
        ReadNumberFrameCounts.frame2ByteDivideIndex = 0;
        break;
    }
    
}

void SendEvent(uint16_t *data)
{
    switch (_writeState)
    {
    case _Init:
        break;

    case WriteFrameHeadCan1:
        
        _deviceState = _Busy;
        
        writeFrameHead[0] = FrameHead;
        writeFrameHead[1] = SendFrameLengthCan + 5;
        writeFrameHead[2] = WriteDataOnTransmit;
        writeFrameHead[3] = canChannel;
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

        if (WriteFrame1Counts.frame2ByteDivideIndex == 0)
        {
            CS_LOW;
            SPI_transmit16Bits(SPIA_BASE,
                (writeFrameHead[0] & 0xFF) << 8 | (writeFrameHead[1] & 0xFF));
            SPI_transmit16Bits(SPIA_BASE,
                (writeFrameHead[2] & 0xFF) << 8 | (writeFrameHead[3] & 0xFF));
            SPI_transmit16Bits(SPIA_BASE,
                (writeFrameHead[4] & 0xFF) << 8 | (writeFrameHead[5] & 0xFF));
            SPI_transmit16Bits(SPIA_BASE,
                (writeFrameHead[6] & 0xFF) << 8 | (writeFrameHead[7] & 0xFF));
            SPI_transmit16Bits(SPIA_BASE,
                ((writeFrameHead[8] & 0xFF) << 8) | (data[0] & 0xFF));
        }
        

        if (WriteFrame1Counts.frame2ByteDivideIndex == WriteFrame1Counts.frame2ByteDivideCount - 2)
        {
            SPI_transmit24Bits(SPIA_BASE,
                ((data[WriteFrame1Counts.frame2ByteDivideIndex + 1] & 0xFF) << 16)
                | (data[WriteFrame1Counts.frame2ByteDivideIndex + 2] & 0xFF00)
                | (data[WriteFrame1Counts.frame2ByteDivideIndex + 2] & 0xFF)
                , 0);
        }

        WriteFrame1Counts.frame2ByteDivideIndex++;
        if (WriteFrame1Counts.frame2ByteDivideCount < 3)
        {
            debugCount++;
        }
        
        if (WriteFrame1Counts.frame2ByteDivideIndex > WriteFrame1Counts.frame2ByteDivideCount - 2)
        {
            CS_HIGH;
            
            CSM431Bflags.isDelay50usForWrite = _false;
            CSM431Bflags.isWriteFrame1End = _true;
            _writeState = WriteOverInterval;
        }
                
        SPI_transmit16Bits(SPIA_BASE, (data[WriteFrame1Counts.frame2ByteDivideIndex + 1] & 0xFF) << 8 | ((data[WriteFrame1Counts.frame2ByteDivideIndex + 2] & 0xFF00) >> 8));        
        
        break;    

    case WriteOverInterval:
        if (CSM431Bflags.isDelay50usForWrite)
        {
            if (XintDivide >= 2)
            {
                CSM431Bflags.isWriteFrame1End = _false;
                WriteFrame2Counts.frame2ByteDivideIndex = 0;
                _writeState = ErrorReadCommand;
            }
            else
            {
                CSM431Bflags.isWriteFrame1End = _false;
                WriteFrame2Counts.frame2ByteDivideIndex = 0;
                _writeState = WriteFree;
                _deviceState = _Ready;
            }
        }
        
        break;

    case ErrorReadCommand:
        _deviceState = _Busy;        

        CS_LOW;
        SPI_transmit16Bits(SPIA_BASE, 0xAC00);
        SPI_transmit16Bits(SPIA_BASE, 0x06FF);
        _writeState = ErrorReadFeedback;
        break;

    case ErrorReadFeedback:
        if (ErrorFeedabckFrameCounts.frame2ByteDivideIndex > ErrorFeedabckFrameCounts.frame2ByteDivideCount)
        {
            CS_HIGH;            
            _writeState = WriteFree;
            ErrorFeedabckFrameCounts.frame2ByteDivideIndex = 0;
            _deviceState = _Ready;
            break;
        }

        ErrorFeedbackData.receiveDataFrameData[ErrorFeedabckFrameCounts.frame2ByteDivideIndex] = SPI_receive16Bits(SPIA_BASE, SPI_DATA_BIG_ENDIAN, 0, TxDelay);
        ErrorFeedabckFrameCounts.frame2ByteDivideIndex++;
        break;
        
    case WriteFree:
        break;
    }
}

void CheckErrorFeedbackEvent()
{
    // switch (_errorFeedbackState)
    // {
    // case _Init:
    //     break;
    
    // case ErrorReadCommand:
    //     if (_deviceState == _Ready)
    //     {
    //         _deviceState = _Busy;
    //     }

    //     CS_LOW;
    //     SPI_transmit16Bits(SPIA_BASE, 0xAC00);
    //     SPI_transmit16Bits(SPIA_BASE, 0x06FF);
    //     _errorFeedbackState = ErrorReadFeedback;
    //     break;

    // case ErrorReadFeedback:
    //     if (ErrorFeedabckFrameCounts.frame2ByteDivideIndex > ErrorFeedabckFrameCounts.frame2ByteDivideCount)
    //     {
    //         CS_HIGH;
    //         OnErrorJudge(ErrorFeedbackData.receiveDataFrameData);
    //         _errorFeedbackState = ErrorReadFree;
    //         ErrorFeedabckFrameCounts.frame2ByteDivideIndex = 0;
    //         _deviceState = _Ready;
    //         break;
    //     }
        
    //     ErrorFeedbackData.receiveDataFrameData[ErrorFeedabckFrameCounts.frame2ByteDivideIndex] = SPI_receive16Bits(SPIA_BASE, SPI_DATA_BIG_ENDIAN, 0, TxDelay);
    //     ErrorFeedabckFrameCounts.frame2ByteDivideIndex++;
    //     break;

    // case ErrorReadFree:
    //     break;
    // }
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

//50us forRead at least interval
void CreateReadInterval()
{
    if (CSM431Bflags.isDelay50usForRead)
    {
        CycleCounts.cycle50usForReadCount = 0;
        return;
    }
    else
    {
        CycleCounts.cycle50usForReadCount++;
        if (CycleCounts.cycle50usForReadCount >= 210 / InterruptIntervalUS + 1)
        {
            CSM431Bflags.isDelay50usForRead = _true;
            CycleCounts.cycle50usForReadCount = 0;
        }

        return;
    }
}

void CreateWriteInterval()
{
    if (CSM431Bflags.isDelay50usForWrite)
    {
        CycleCounts.cycle50usForWriteCount = 0;
        return;
    }
    else
    {
        CycleCounts.cycle50usForWriteCount++;
        if (CycleCounts.cycle50usForWriteCount >= 50 / InterruptIntervalUS + 1)
        {
            CSM431Bflags.isDelay50usForWrite = _true;
            CycleCounts.cycle50usForWriteCount = 0;
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
void CreateObligationInterval()
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

//1ms at least interval
void CreateCfgResetInterval()
{
    if (CSM431Bflags.isDelay1ms)
    {
        CycleCounts.cycle1msCount = 0;
        return;
    }
    else
    {
        CycleCounts.cycle1msCount++;
        if (CycleCounts.cycle1msCount >= 1000 / InterruptIntervalUS + 1)
        {
            CSM431Bflags.isDelay1ms = _true;
            CycleCounts.cycle1msCount = 0;
        }

        return;
    }
}

//100ms at least interval
void CreateChannelSwitchInterval()
{
    if (CSM431Bflags.isChannelSwitchedin100ms)
    {
        CycleCounts.cycle100msCount = 0;
        return;
    }
    else
    {
        CycleCounts.cycle100msCount++;
        if (CycleCounts.cycle100msCount >= 100000ul / InterruptIntervalUS + 1)
        {
            CSM431Bflags.isChannelSwitchedin100ms = _true;
            CycleCounts.cycle100msCount = 0;
        }

        return;
    }
}

//200ms at least interval
void CreateRstHighInterval()
{
    if (CSM431Bflags.isDelay200ms)
    {
        CycleCounts.cycle200msCount = 0;
        return;
    }
    else
    {
        CycleCounts.cycle200msCount++;
        if (CycleCounts.cycle200msCount >= 210000ul / InterruptIntervalUS + 1)
        {
            CSM431Bflags.isDelay200ms = _true;
            CycleCounts.cycle200msCount = 0;
        }

        return;
    }
}

//3ms at least interval
void CreateConfigCommandEndInterval()
{
    if (CSM431Bflags.isDelay3ms)
    {
        CycleCounts.cycle3msCount = 0;
        return;
    }
    else
    {
        CycleCounts.cycle3msCount++;
        if (CycleCounts.cycle3msCount >= 8000ul / InterruptIntervalUS + 1)
        {
            CSM431Bflags.isDelay3ms = _true;
            CycleCounts.cycle3msCount = 0;
        }

        return;
    }
}

void TimeInterval()
{
    CreateFrameInterval();
    CreateReadInterval();
    CreateWriteInterval();        
    CreateObligationInterval();  
    CreateChannelSwitchInterval();
    if (_deviceState != _Busy && _deviceState != _Ready && _deviceState != Error)
    {
        CreateCfgResetInterval();
        CreateRstHighInterval();
        CreateRstLowInterval();
        CreateConfigCommandEndInterval();
    }   
    
}

uint16_t C1188A(uint16_t* data, uint16_t len)
{
    uint16_t i, j;
    uint16_t sum;
    uint32_t crcSum;
    uint32_t mOr;
    uint32_t temp[64] = {0};
    for (i = 0; i < len - 1; i++)
    {
        temp[i] = data[i];
    }

    for (i = 0; i < len - 1; i++)
    {
        j = i > 15 ? i - 16 : i;
        temp[i] = (temp[i] << (16 - j) & 0xFFFF) | ((temp[i] >> j) & 0xFFFF);
        mOr = mOr ^ temp[i];
    }

    crcSum = mOr;
    j = len - 1;
    j = j > 15? j % 16 : j;
    crcSum = (crcSum >> (16 - j) & 0xFFFF) | ((crcSum << j) & 0xFFFF);
    sum = (uint16_t)crcSum;
    return sum;
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

uint16_t InitCommand()
{
    if (_deviceState == _Init)
    {
        _deviceState = ReadyToCfgConfig;
        return NoError;
    }
    else
    {
        return CommandConflict;
    }
}

uint16_t SendCommand(uint16_t count)
{
    if (_deviceState == _Busy)
    {
        if (_writeState == WriteFree)
        {
            WriteFrame1Counts.frame2ByteDivideIndex = 0;
        }
            CSM431Bflags.isSendCommandOn = _true;
        XintDivide++;
        if (XintDivide > 2)
        {
            XintDivide = 0;
        }

    }
    else if (_deviceState ==  _Ready)
    {
        if (_writeState == WriteFree)
        {
            _writeState =  WriteFrameHeadCan1;
            _deviceState = _Busy;
            WriteFrame1Counts.frame2ByteDivideIndex = 0;
            WriteFrame2Counts.frame2ByteDivideIndex = 0;
            XintDivide++;
            if (XintDivide > 2)
            {
                XintDivide = 0;
            }
            return NoError;
        }
        else if (_writeState == _Init)
        {
            return DeviceNotInited;
        }
        else
        {
            return CommandConflict;
        }
    }
    else
    {
        return DeviceNotInited;
    }
}

uint16_t ReceiveCommand()
{
    if (_deviceState == Error)
    {
        return ErrorOccurred;
    }
    
    if (_deviceState != _Ready && _deviceState != _Busy)
    {
        return DeviceNotInited;
    }
    
    if (_readState == ReadFree)
    {
        if (_deviceState == _Busy)
        {
            ReadDataFrameCounts.frame2ByteDivideIndex = 0;
            ReadNumberFrameCounts.frame2ByteDivideIndex = 0;
            CSM431Bflags.isReceiveCommandOn = _true;
            return CommandDelayed;
        }
        else if (_deviceState == _Ready)
        {
            CSM431Bflags.isMosiNull = _false;
            _readState = ReadNumber;
            _deviceState = _Busy;
            //ReceiveEvent();
            return NoError;
        }
    }
    else if (_readState == _Init)
    {
        return DeviceNotInited;
    }
    else
    {
        return CommandConflict;
    }
}

    uint16_t CheckErrorFeedbackCommand()
{
    if (_deviceState == Error)
    {
        return ErrorOccurred;
    }

    if (_deviceState != _Ready && _deviceState != _Busy)
    {
        return DeviceNotInited;
    }

    if (_deviceState == _Busy)
    {
        CSM431Bflags.isCheckErrorCommandOn = _true;
        return CommandDelayed;
    }

    if (_deviceState == _Ready && _errorFeedbackState == ErrorReadFree)
    {
        _errorFeedbackState = ErrorReadCommand;
        return NoError;
    }
    else if (_errorFeedbackState == _Init)
    {
        return DeviceNotInited;
    }
    else
    {
        return CommandConflict;
    }
}

uint16_t CheckIfError()
{
    return CSM431Bflags.isErrorOccurred;
}

uint16_t CheckIfCsm431BReady()
{
    if (CSM431Bflags.isInitTimeout)
    {
        return Timeout;
    }
    else
    {
        return _deviceState == _Ready ? _true : _false;
    }    
}

void IdelEvent(uint16_t sendData)
{
    OnErrorJudge(ErrorFeedbackData.receiveDataFrameData);
    CSM431BDeviceEvent();    
    //CheckErrorFeedbackEvent();
    SendEvent(sendData);
    ReceiveEvent();
    ReceiveDecode();
}

void ErrorResaet()
{
    _deviceState = _Init;
    _readState = _Init;
    _writeState = _Init;
    CSM431Bflags.isErrorOccurred = _false;
}

void AllLow()
{
    CFG_ENABLE;
    CS_LOW;
    RST_ENABLE;
}

void AllHigh()
{
    CFG_DISABLE;
    CS_HIGH;
    RST_DISABLE;
}

void TestInt()
{
    CS_HIGH;
    RST_DISABLE;
    CFG_ENABLE;
}
void TestIntL()
{
    CS_HIGH;
    RST_ENABLE;
    CFG_ENABLE;
}
