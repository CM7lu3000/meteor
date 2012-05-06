// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
// Copyright © 2008 Synaptics Incorporated. All rights reserved. 
// 
// The information in this file is confidential under the terms 
// of a non-disclosure agreement with Synaptics and is provided 
// AS IS. 
// 
// The information in this file shall remain the exclusive property 
// of Synaptics and may be the subject of Synaptics’ patents, in 
// whole or part. Synaptics’ intellectual property rights in the 
// information in this file are not expressly or implicitly licensed 
// or otherwise transferred to you as a result of such information 
// being made available to you. 
//
// $RCSfile: RMI4Funcs.h,v $
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// #ifndef RMI4Funcs_H
#define RMI4Funcs_H

#define EError unsigned char

#define ESuccess                     0
#define I2C_SUCCESS 			0
#define EErrorTimeout               -1
#define EErrorFlashNotDisabled           -2
#define EErrorBootID                -3
#define EErrorFunctionNotSupported  -4
#define EErrorFlashImageCorrupted   -5
#define I2C_FAILURE -6

struct RMI4FunctionDescriptor
{
  unsigned char m_QueryBase;
  unsigned char m_CommandBase;
  unsigned char m_ControlBase;
  unsigned char m_DataBase;
  unsigned char m_IntSourceCount;
  unsigned char m_ID;
};

struct ConfigDataBlock
{
  unsigned short m_Address;
  unsigned char m_Data;
};

enum {
  i2c,
  spi
}protocolType;

typedef enum
{
  EAttnNone,
  EAttnLow,
  EAttnHigh,
  EAttnHighAndLow,
} EAttention;

typedef enum
{
  Ei2c,
  Espi,
} EProtocol;

struct RMI4FunctionDescriptor m_PdtF34Flash;
struct RMI4FunctionDescriptor m_PdtF01Common;
struct RMI4FunctionDescriptor m_BaseAddresses;

unsigned short m_uQuery_Base;

unsigned int m_lengthWritten, m_lengthRead;

struct ConfigDataBlock g_ConfigDataList[0x2d - 4];

unsigned int g_ConfigDataCount;

bool            m_bAttenAsserted;
EError          m_ret;
bool  m_bFlashProgOnStartup;
bool  m_bUnconfigured;


unsigned short m_BootloadID;
unsigned short m_BootID_Addr;

// Image file 
unsigned long m_checkSumImg;

// buffer for flash images ... tomv
unsigned char FirmwareImage[16000];  // make smaller and dynamic
unsigned char ConfigImage[16000];  // make smaller and dynamic
  
unsigned short m_bootloadImgID;
unsigned char m_firmwareImgVersion;
unsigned char *m_firmwareImgData;
unsigned char *m_configImgData;
unsigned short m_firmwareBlockSize;
unsigned short m_firmwareBlockCount;
unsigned short m_configBlockSize;
unsigned short m_configBlockCount;

// Registers for configuration flash
unsigned char m_uPageData[0x200];
unsigned char m_uStatus;
unsigned long m_firmwareImgSize;
unsigned long m_configImgSize;
unsigned long m_fileSize;

unsigned char m_uF01RMI_CommandBase;
unsigned char m_uF01RMI_DataBase;
unsigned char m_uF01RMI_QueryBase;
unsigned char m_uF01RMI_IntStatus;

unsigned char m_uF34Reflash_DataReg;
unsigned char m_uF34Reflash_BlockNum;
unsigned char m_uF34Reflash_BlockData;
unsigned char m_uF34Reflash_FlashControl;
unsigned char m_uF34ReflashQuery_BootID;
unsigned char m_uF34ReflashQuery_FirmwareBlockSize;
unsigned char m_uF34ReflashQuery_FirmwareBlockCount;
unsigned char m_uF34ReflashQuery_ConfigBlockSize;
unsigned char m_uF34ReflashQuery_ConfigBlockCount;
unsigned char m_uF34ReflashQuery_FlashPropertyQuery;

unsigned long m_FirmwareImgFile_checkSum;
  
//  Constants
static const unsigned char s_uF34ReflashCmd_FirmwareCrc   = 0x01;
static const unsigned char s_uF34ReflashCmd_FirmwareWrite = 0x02;
static const unsigned char s_uF34ReflashCmd_EraseAll      = 0x03;
static const unsigned char s_uF34ReflashCmd_ConfigRead    = 0x05;
static const unsigned char s_uF34ReflashCmd_ConfigWrite   = 0x06;
static const unsigned char s_uF34ReflashCmd_ConfigErase   = 0x07;
static const unsigned char s_uF34ReflashCmd_Enable        = 0x0f;
static const unsigned char s_uF34ReflashCmd_NormalResult  = 0x80; //Read the Flash control register. The result should be 
                                                                  //$80: Flash Command = Idle ($0), Flash Status = Success ($0), 
                                                                  //Program Enabled = Enabled (1). Any other value indicates an error.


// Device
void RMI4ReadPageDescriptionTable(struct i2c_client *client);
																  
void RMI4ResetDevice(struct i2c_client *client);
  
//**************************************************************
// This function polls the registers to ensure the device is in idled state
// Parameter errCount is the max times to read a register to make sure the state is ready
// Default number of checks is 3
//**************************************************************
//  bool UsePolling() { return (eAttention==EAttnNone) ? true : false; }
void RMI4WaitATTN(struct i2c_client *client); //, int errorCount=300);
 
EError RMI4ReadBootloadID(struct i2c_client *client);

EError RMI4WriteBootloadID(struct i2c_client *client);

bool RMI4ValidateBootloadID(struct i2c_client *client, unsigned short bootloadID);
  
EError RMI4IssueEnableFlashCommand(struct i2c_client *client);

EError RMI4IssueEraseCommand(struct i2c_client *client, unsigned char *command);

EError RMI4IssueFlashControlCommand(struct i2c_client *client, unsigned char *command);

// Configuration
 void RMI4ReadConfigInfo(struct i2c_client *client);
 
// Firmware (UI)
//**************************************************************
// Determine firmware organization - read firmware block size and firmware size
//**************************************************************
void RMI4ReadFirmwareInfo(struct i2c_client *client);

//**************************************************************
// Write firmware to device one block at a time
//**************************************************************
EError RMI4FlashFirmwareWrite(struct i2c_client *client);

//**************************************************************
// Calculates checksum for a given data block
//**************************************************************
void RMI4CalculateChecksum(unsigned short * data, unsigned short len, unsigned long *dataBlock);


void RMI4RMIInit(struct i2c_client *client, EProtocol pt, unsigned char i2cAddr, EAttention eAttn, unsigned int byteDelay,
                 unsigned int bitRate, unsigned long timeOut);

void RMI4RMIInitI2C(struct i2c_client *client, unsigned char i2cAddr, EAttention i2cAttn);

void RMI4RMIInitSPI(struct i2c_client *client, unsigned int byteDelay, unsigned int bitRate, unsigned long timeOut);
     
void RMI4WritePage(struct i2c_client *client);

// Flash entry and exit
void RMI4EnableFlashing(struct i2c_client *client);
 
//**************************************************************
// Resets device, wait for ATTN and ensure that $F01 flash prog is '0' - meaning
// the new firmware is valid and executing
//**************************************************************
EError RMI4DisableFlash(struct i2c_client *client);


// Configuration
unsigned short GetConfigSize(void);  
 
// Firmware (UI)
unsigned short GetFirmwareSize(void);

void RMI4ProgramFirmware(struct i2c_client *client);
 
void RMI4ProgramConfiguration(struct i2c_client *client);

// Image file
void RMI4Init(struct i2c_client *client);

// Device
bool RMI4isExpectedRegFormat(struct i2c_client *client);

void RMI4setFlashAddrForDifFormat(struct i2c_client *client);


unsigned char I2C_HAL_BitRead_ATTN(void);
void RMI4FuncsConstructor(void);


