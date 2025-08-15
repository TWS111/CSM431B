// This Source Code Form is subject to the terms of the MIT License.
// If a copy of the MIT was not distributed with this file, You can obtain one at https://opensource.org/licenses/MIT.
// Copyright (C) GX_WangYifan.
// All Rights Reserved.

//#define _debug
#define _release
#define _noSlaveDebug

#ifdef _release
#include "device.h"
#define SPI_CS_GPIO 57U
#define GPIO_RST    58U
#define GPIO_CFG    40U
#endif

#ifdef _debug
#define SPIA_BASE   0
#define SPI_CS_GPIO 11
#define GPIO_RST    0
#define GPIO_CFG    0
#endif

#define _false  0
#define _true   1

#define SPI_DATA_LITTLE_ENDIAN  0U
#define SPI_DATA_BIG_ENDIAN     1U

#define TxDelay                 0

#define _Init                   0x00

//state- total state
#define ReadyToCfgConfig        0x11
#define CfgConfig               0x12
#define OnReset                 0x13
#define OnDelay200msPart1       0x14
#define WriteConfigFrame        0x15
#define OnDelay3ms              0x16
#define WriteFeedbackFrame      0x17
#define PauseToLoop             0x18
#define OnCfgReset              0x19
#define OnDelay200msPart2       0x1A
#define _Ready                  0x1B
#define _Busy                   0x1C
#define Error                   0x1D

//state- read state
#define ReadNumber              0x21
#define ReadInterval            0x22
#define ReadData                0x23
#define ReadOverInterval        0x24
#define ReadFree                0x2A

//state- write state
#define WriteFrameHeadCan1      0x31
#define WriteFrameDataCan1      0x32
#define WriteFrameHeadCan2      0x33
#define WriteFrameDataCan2      0x34
#define WriteOverInterval       0x35
#define WriteFree               0x3A

//state error feedback state
#define ErrorReadCommand        0x41
#define ErrorFrameInterval      0x42
#define ErrorReadFeedback       0x43
#define ErrorReadFree           0x4A

//frame field
#define FrameHead               0xAC

#define ReadRegisterOnConfig    0x01
#define WriteRegisterOnConfig   0x02
#define ReadDataOnTransmit      0x03
#define WriteDataOnTransmit     0x04
#define ReadAllRegisterOnConfig 0x05
#define CheckErrorOnTransmit    0x06
#define ReadLengthOnTransmit    0x07

#define SpiToCan1               0x04
#define SpiToCan2               0x05

//MCU config
#define InterruptIntervalUS     40
#define NowFrameType            CanfdExtendedFrame
#define SendFrameIDCan1         0x1502000
#define SendFrameIDCan2         0x121
#define ReceiveFrameIDCan1      0xA082000
#define SendFrameLengthCan      0x20
#define ReceiveFrameLengthCan   0x40

//Users Error
#define NoError                 0x60
#define CommandConflict         0x61
#define DeviceNotInited         0x62
#define Timeout                 0x63
#define FrameHeadError          0x64
#define ChannelError            0x65
#define ErrorOccurred           0x66
#define CommandDelayed          0x67

//CSM431B ErrorCode
#define ConfigNoError           0x70
#define RegisterAddressError    0x71
#define RegisterValueError      0x72
#define CrcCheckError           0x73
#define LengthError             0x74
#define CommandError            0x75

#define CanfdStandardFrame      0x80
#define CanfdExtendedFrame      0x88

//CSM431B config
#define RegisterConfigLength    12
#define TimeoutCount            10


typedef struct
{
    uint16_t isDelay50us;
    uint16_t isDelay50usForWrite;
    uint16_t isDelay50usForRead;
    uint16_t isDelay100us;
    uint16_t isDelay200us;
    uint16_t isDelay1ms;
    uint16_t isDelay3ms;
    uint16_t isDelay200ms;
    uint16_t isCSM431BSeted;
    uint16_t isMosiNull;
    uint16_t isBusBusy;
    uint16_t isFrameEnd;
    uint16_t isReceiveCommandOn;
    uint16_t isSendCommandOn;
    uint16_t isCheckErrorCommandOn;
    uint16_t isWriteFrame1End;
    uint16_t isWriteFrame2End;
    uint16_t isSendNumberCommandFrame;
    uint16_t isReadyToReceive;
    uint16_t isErrorOccurred;
    uint16_t isInitTimeout;
    uint16_t isConfigured;
    uint16_t isIntSet;
    uint16_t isChannelSwitchedin100ms;
    uint16_t isChannelSwitched;
}_flags;

typedef struct
{
    uint16_t frame2ByteDivideCount;
    uint16_t frame2ByteDivideIndex;
    uint16_t frame2ByteWriteCount;
    uint16_t frame2ByteWriteIndex;
    uint16_t frameCountIndex;
}_counts;

typedef struct
{
    uint16_t cycle50usCount;
    uint16_t cycle50usForReadCount;
    uint16_t cycle50usForWriteCount;
    uint16_t cycle100usCount;
    uint16_t cycle200usCount;
    uint16_t cycle1msCount;
    uint16_t cycle3msCount;
    uint16_t cycle100msCount;
    uint16_t cycle200msCount;
    uint16_t cycleTimeoutCount;
}_cycleCounts;

typedef struct
{
    uint16_t receiveNumberFrameData[35];
    uint16_t receiveFrameNumber;
    uint16_t receiveDataFrameData[130];
}_data;

typedef struct
{
    uint16_t receiveFrameNumber;
    uint16_t receiveDataFrameData[20];
}_errorData;

typedef struct
{
    uint16_t receiveIDCan;
    uint16_t receiveDataCan[64];
}_rxCanData;

typedef struct
{
    uint16_t frameLengthFaultCount;
    uint16_t channelFaultCount;
    uint16_t crcFaultCount;
    uint16_t frameTypeFaultCount;
    uint16_t frameIDFaultCount;
}_serialFault;

typedef struct
{
    uint16_t receiveFaultCount;
    uint16_t sendFaultCount;
    uint16_t negitiveFaultCount;
    uint16_t busoffFaultCount;
}_canFault;

static uint32_t configIndex[RegisterConfigLength] = {
    0x0000, 0x0001, 0x0004, 0x0005, 0x0006, 0x001B, 0x001C, 0x001D, 0x001E, 0x001F, 0x0020, 0x0021

};

static uint32_t configFrameHead[4] = {
    0x00AC, 0x0007, 0x0002, 0x00FF
};

static uint32_t valueOnConfig[RegisterConfigLength * 2] = {
    0x0000, 0x0000, //0x00
    0x0500, 0x0000, //0x01
    0x0000, 0x0000, //0x04
    0x0A05, 0x3500, //0x05 
    0x0A05, 0x3500, //0x06
    0x0000, 0x0000, //0x1B
    0x0100, 0x0100, //0x1C
    0x5500, 0x5500, //0x1D
    0x0F00, 0x0F00, //0x1E
    0x0100, 0x0100, //0x1F
    0x1400, 0x1400, //0x20
    0x0500, 0x0500, //0x21
};
static uint16_t readNumberCommandFrame[15] = {
    0x00AC, 0x0000, 0x0007, 0x00FF,
};

static uint16_t crc16X25Table[256] = {
    0X0000, 0X1189, 0X2312, 0X329B, 0X4624, 0X57AD, 0X6536, 0X74BF,
    0X8C48, 0X9DC1, 0XAF5A, 0XBED3, 0XCA6C, 0XDBE5, 0XE97E, 0XF8F7,
    0X1081, 0X0108, 0X3393, 0X221A, 0X56A5, 0X472C, 0X75B7, 0X643E,
    0X9CC9, 0X8D40, 0XBFDB, 0XAE52, 0XDAED, 0XCB64, 0XF9FF, 0XE876,
    0X2102, 0X308B, 0X0210, 0X1399, 0X6726, 0X76AF, 0X4434, 0X55BD,
    0XAD4A, 0XBCC3, 0X8E58, 0X9FD1, 0XEB6E, 0XFAE7, 0XC87C, 0XD9F5,
    0X3183, 0X200A, 0X1291, 0X0318, 0X77A7, 0X662E, 0X54B5, 0X453C,
    0XBDCB, 0XAC42, 0X9ED9, 0X8F50, 0XFBEF, 0XEA66, 0XD8FD, 0XC974,
    0X4204, 0X538D, 0X6116, 0X709F, 0X0420, 0X15A9, 0X2732, 0X36BB,
    0XCE4C, 0XDFC5, 0XED5E, 0XFCD7, 0X8868, 0X99E1, 0XAB7A, 0XBAF3,
    0X5285, 0X430C, 0X7197, 0X601E, 0X14A1, 0X0528, 0X37B3, 0X263A,
    0XDECD, 0XCF44, 0XFDDF, 0XEC56, 0X98E9, 0X8960, 0XBBFB, 0XAA72,
    0X6306, 0X728F, 0X4014, 0X519D, 0X2522, 0X34AB, 0X0630, 0X17B9,
    0XEF4E, 0XFEC7, 0XCC5C, 0XDDD5, 0XA96A, 0XB8E3, 0X8A78, 0X9BF1,
    0X7387, 0X620E, 0X5095, 0X411C, 0X35A3, 0X242A, 0X16B1, 0X0738,
    0XFFCF, 0XEE46, 0XDCDD, 0XCD54, 0XB9EB, 0XA862, 0X9AF9, 0X8B70,
    0X8408, 0X9581, 0XA71A, 0XB693, 0XC22C, 0XD3A5, 0XE13E, 0XF0B7,
    0X0840, 0X19C9, 0X2B52, 0X3ADB, 0X4E64, 0X5FED, 0X6D76, 0X7CFF,
    0X9489, 0X8500, 0XB79B, 0XA612, 0XD2AD, 0XC324, 0XF1BF, 0XE036,
    0X18C1, 0X0948, 0X3BD3, 0X2A5A, 0X5EE5, 0X4F6C, 0X7DF7, 0X6C7E,
    0XA50A, 0XB483, 0X8618, 0X9791, 0XE32E, 0XF2A7, 0XC03C, 0XD1B5,
    0X2942, 0X38CB, 0X0A50, 0X1BD9, 0X6F66, 0X7EEF, 0X4C74, 0X5DFD,
    0XB58B, 0XA402, 0X9699, 0X8710, 0XF3AF, 0XE226, 0XD0BD, 0XC134,
    0X39C3, 0X284A, 0X1AD1, 0X0B58, 0X7FE7, 0X6E6E, 0X5CF5, 0X4D7C,
    0XC60C, 0XD785, 0XE51E, 0XF497, 0X8028, 0X91A1, 0XA33A, 0XB2B3,
    0X4A44, 0X5BCD, 0X6956, 0X78DF, 0X0C60, 0X1DE9, 0X2F72, 0X3EFB,
    0XD68D, 0XC704, 0XF59F, 0XE416, 0X90A9, 0X8120, 0XB3BB, 0XA232,
    0X5AC5, 0X4B4C, 0X79D7, 0X685E, 0X1CE1, 0X0D68, 0X3FF3, 0X2E7A,
    0XE70E, 0XF687, 0XC41C, 0XD595, 0XA12A, 0XB0A3, 0X8238, 0X93B1,
    0X6B46, 0X7ACF, 0X4854, 0X59DD, 0X2D62, 0X3CEB, 0X0E70, 0X1FF9,
    0XF78F, 0XE606, 0XD49D, 0XC514, 0XB1AB, 0XA022, 0X92B9, 0X8330,
    0X7BC7, 0X6A4E, 0X58D5, 0X495C, 0X3DE3, 0X2C6A, 0X1EF1, 0X0F78
};

void CreateFrameInterval(void);
void TimeInterval(void);
void SendEvent(uint16_t *data);
void ReceiveEvent(void);
void CheckErrorFeedbackEvent(void);
void CSM431BDeviceEvent(void);
uint16_t SendCommand();
uint16_t ReceiveCommand(void);
uint16_t CheckErrorFeedbackCommand(void);
uint16_t InitCommand(void);
uint16_t* GetReceiveData();
void TimeInterval(void);
uint16_t CheckIfCsm431BReady(void);
uint16_t CheckIfError(void);
void ReceiveDecode(void);
uint16_t IntSet(void);
void IdelEvent(uint16_t sendData);
void AllLow();
void AllHigh();
void TestInt();
void TestIntL();
