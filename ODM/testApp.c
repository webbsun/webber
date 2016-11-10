//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "fmApi.h"
#include "smApi.h"
#include "smSrv.h"
#include "genApi.h"
#include "uart.h"
#include "pmApi.h"
#include "uart.h"
#include "msgDef.h"
#include "errDef.h"
#include "cmApi.h"
#include "mmApi.h"

#include <stdlib.h>
#include <string.h>
#include <ctype.h>

typedef struct {
    char *name;
	int (*func) (char *arg);
	char *desc;
} cmd_def;

/** Sensor address/Device ID define**/
#define AG6AXIS_EN	 0
#define AG6AXIS_ADDR 0xD6
#define AG6AXIS_WAMI 0X0F
#define AG6AXIS_RDID 0x68

#define ACCE_EN	   1
#define ACCE_ADDR 0x32
#define ACCE_WAMI 0X0F
#define ACCE_RDID 0x33

#define COMPASS_EN	 0
#define COMPASS_ADDR 0x3C
#define COMPASS_WAMI 0X0F
#define COMPASS_RDID 0x68

#define PRESSURE_EN 	0
#define PRESSURE_ADDR 0xBA
#define PRESSURE_WAMI 0X0F
#define PRESSURE_RDID 0x68

#define TEMEPERATURE_EN 	1
#define TEMEPERATURE_ADDR 0xBE
#define TEMEPERATURE_WAMI 0X0F
#define TEMEPERATURE_RDID 0xBC

enum smErmsg{
		TEMEPERATURE_ERR = -6, 
    PRESSURE_ERR	 	 = -5, 
    COMPASS_ERR 	 	 = -4, 
		GYRO_ERR 		 		 = -3, 
		AG6XIS_ERR 		 	 = -2, 
		I2CINIT_ERR 	 	 = -1,
		SMNO_ERR 			 	 = 0
};

enum fmErmsg
{
    VERIFY_ERR 	= -5,
    READ_ERR		= -4,
		PROGRAM_ERR = -3,
		ERASE_ERR 	= -2,
		READID_ERR 	= -1,
		FMNO_ERR 		=  0
};

osMessageQId testQueueHandle = NULL;
uint8_t logSuppress = 0;

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef hlpuart1;
//uint32_t extMem __attribute__ ((section(".ccm")));

int odmSpiFlashTest(char *arg);
int odmSensorTest(char *arg);
int odmPowerControlTest(char *arg);
int Wifi_test(char *arg);
int BLE_test(char *arg);
int odmModemTest(char *arg);
int odmGpsTest(char *arg);
int odmNotificationTest(char *arg);
int odmPowerKeysTest();
int odmPowerOffTest();
int odmPowerBatt1Test(char *arg);
int odmPowerBatt2Test(char *arg);
int odmPowerResetTest(char *arg);
int odmPowerModeTest(char *arg);
int odmFwVer(char *arg);
int odmAcceTest(char *arg);
int odmTempTest(char *arg);
int odmAcceFifoTest(char *arg);
int odmTempFifoTest(char *arg);
int odmGetSN(char *arg);
int odmSetSN(char *arg);
int odmFsTest(char *arg);
int odmMkfs(char *arg);
int odmLsfs(char *arg);
int odmRmfs(char *arg);
int odmLogMask(char *arg);
int odmsmTask(char *arg);

int help_cmd(char *arg);

const cmd_def cmdTable[] =
{
	{"ba1test", 	odmPowerBatt1Test, "Test battery\r\n\t" },
	{"ba2test", 	odmPowerBatt2Test, "Test battery per second\r\n\t" },
	{"bltest", 		odmNotificationTest, 		"Test buzzer and LED\r\n\t" },
	{"bletest", 	BLE_test, 		"Test BLE\r\n\t" },
	{"gpstest",		odmGpsTest, 	"Test GPS\r\n\t" },
	{"help", 		help_cmd, 		"Show command help\r\n\t" },
	{"wnctest", 	odmModemTest, 	"Test Modem\r\n\t" },
	//{"sentest",		odmSensorTest, 		"Test sensors\r\n\t" },
	{"sftest", 		odmSpiFlashTest, 	"Test SPI Flash\r\n\t" },
	{"pwrtest",		odmPowerControlTest, 	"Test Key/Guage/ADC\r\n\t" },
	{"wftest", 		Wifi_test, 		"Test Wi-Fi\r\n\t" },
	{"keytest", 	odmPowerKeysTest, "Test poweroff\r\n\t" },
	{"offtest", 	odmPowerOffTest, "Test poweroff\r\n\t" },
	{"rsttest", 	odmPowerResetTest, "software reset\r\n\t" },
	{"modtest", 	odmPowerModeTest, "STOP mode test\r\n\t" },
	{"gtest",			odmAcceTest, 	"Test acce. sensor\r\n\t" },
	{"ttest",			odmTempTest,	"Test temp. sensor\r\n\t" },
	{"girqtest",	odmAcceFifoTest,	"Test acce.	IRQ/FIFO\r\n\t"},
	{"tirqtest",	odmTempFifoTest, 	"Test temp. IRQ\r\n\t" },
	{"smtest",	odmsmTask, 	"smTask Test\r\n\t" },
	{"getsn",		odmGetSN, 		"Get serial number\r\n\t" },
	{"setsn",		odmSetSN, 		"Set serial number\r\n\t" },
	{"fstest",		odmFsTest, 		"Test FATFS\r\n\t" },
	{"mkfs",		odmMkfs, 		"Format FATFS\r\n\t" },
	{"ls",			odmLsfs, 		"List directory contents\r\n\t" },
	{"rm",			odmRmfs, 		"Remove files or directories\r\n\t" },
	{"logmask", 	odmLogMask, 	"Get/Set log mask\r\n\t" },
	{"ver", 		odmFwVer, 			"MCU BSP version\r\n\t" },
};

const char eCmd[] = "\r\nNo valid command";
const char sCmd[] = "\r\nCommand successful";
const char fCmd[] = "\r\nCommand failed";

const char bsKey[] = "\b \b";
const char prompt[] = "\r\ntrk>";
const char exitCmd[] = "Exit Command Mode\r\n";
const char hintCmd[] = "\r\nPress ESC to ";

const char title[] = "\r\n\n\tName\t\tDescription\r\n\t";
const char line[] = "---------------------------\r\n\t";
const char tab[] = "\t\t";

const char overMaxLen[] = " - Cmd over max. len\r\n";
const char illeChar[] = "Illegal character\r\n";

int help_cmd(char *arg)
{
	uint8_t idx;

	uartPrintf((char *)title);
	uartPrintf((char *)line);
	for(idx = 0; idx < sizeof(cmdTable)/sizeof(cmd_def); idx++) {
		osDelay(3);
		uartPrintf((char *)cmdTable[idx].name);
		if(strlen(cmdTable[idx].name) < 8)
			uartPrintf((char *)tab);
		else
			uartPrintf("\t");
		uartPrintf((char *)cmdTable[idx].desc);
	}
	
	return E_OK;
}

void cmdParse(char *arg)
{
	uint8_t idx;
	char *pcmd, *delim = " ";

	pcmd = strtok(arg, delim);
	for(idx = 0; idx < sizeof(cmdTable)/sizeof(cmd_def); idx++) {
		if(0 == strcmp(pcmd, cmdTable[idx].name)) {
			uartPrintf("\r\n");
			pcmd = strtok(NULL, delim);
			if(E_OK == cmdTable[idx].func(pcmd))
				uartPrintf((char *)sCmd);
			else
				uartPrintf((char *)fCmd);
			
			return;
		}
	}
	uartPrintf((char *)eCmd);
}

void charParse(char ch)
{
	static char cmdBuf[32];
	static uint8_t cmdIdx = 0;
	
	switch(ch)
	{
		case 0x08: //backspace
			if(cmdIdx) {
				uartPrintf((char *)bsKey);
				cmdIdx--;
			}
			break;
		case 0x0A: // carriage return
			break;
		case 0x0D: // line feed
			if(!logSuppress) {
				uartPrintf((char *)hintCmd);
				uartPrintf((char *)exitCmd);
			}
			logSuppress = 1;
			if(cmdIdx) {
				uart_rx_disable();
				cmdBuf[cmdIdx] = '\0';
				cmdParse(cmdBuf);
				cmdIdx = 0;
				uart_rx_enable();
			}
			uartPrintf((char *)prompt);
			break;
		case 0x1B: //escape
			cmdIdx = 0;
			uartPrintf("\r\n");
			if(logSuppress)
				uartPrintf((char *)exitCmd);
			logSuppress = 0;
			break;
		default:
			if(!logSuppress) {
				uartPrintf((char *)hintCmd);
				uartPrintf((char *)exitCmd);
			}
			logSuppress = 1;
			if(isprint(ch)) {
				send_char_to_uart(ch);
				
				if(cmdIdx < (sizeof(cmdBuf)-1))
					cmdBuf[cmdIdx++] = ch;
				else {
					uartPrintf((char *)overMaxLen);
					uartPrintf((char *)prompt);
					cmdIdx = 0;
				}
			} else {
				uartPrintf((char *)illeChar);
			}
		}
}

int odmNotificationTest(char *arg)
{
	uartPrintf("Verify LED and Buzzer\r\n");

	if(arg == NULL || arg[1] == 'n') {	// arg = 'on'
		uartPrintf("Turn on LED and Buzzer\r\n");
		ledCtrl(1, 1);
		bzCtrl(1);
	}
	if(arg == NULL) {
		osDelay(500);
	}
	if(arg == NULL || arg[1] == 'f') {	// arg = 'off'
		ledCtrl(0, 0);
		bzCtrl(0);
		uartPrintf("Turn off LED and Buzzer\r\n");
	}

	return 0;
}

void StartTestApp(void const * argument)
{
	osEvent ret;
	trkMsg *pMsg;
	char inBuf[MAX_UART_RX_LEN];
	uint8_t inNum, idx;

	/* init code for FATFS */
	MX_FATFS_Init();

	/* Create the queue(s) */
	/* definition and creation of testQueue for testing Application */
	osMessageQDef(testQueue, 8, uint32_t);
	testQueueHandle = osMessageCreate(osMessageQ(testQueue), NULL);
	/* assert() test for OS
	testQueueHandle = osMessageCreate (NULL, NULL);
	*/
  
	/* USER CODE BEGIN 5 */
	gLog(_INFO_LOG_|_TA_LOG_, "StartTestApp running\r\n");
	#if 0
	Wifi_test(NULL);
	BLE_test(NULL);
	odmSpiFlashTest(NULL);
	odmSensorTest(NULL);
	odmNotificationTest(NULL);
	odmPowerControlTest(NULL);
	odmModemTest(NULL);
	odmGpsTest(NULL);
	#endif
	/* Infinite loop */
	uart_rx_enable();
	odmGetStr(inBuf, MAX_UART_RX_LEN); // Clean ring buffer
	for(;;)
	{
 		ret = osMessageGet(testQueueHandle, osWaitForever);
    	pMsg = (trkMsg *)ret.value.p;

		switch(pMsg->hdr.id)
		{
			case TEST_MSG_TIMEOUT:
				break;
			case TEST_MSG_FROM_ISR:
				inNum = odmGetStr(inBuf, MAX_UART_RX_LEN);
				for(idx = 0; idx < inNum; idx++) {
					charParse(inBuf[idx]);
				}
				break;
			default:
				gLog(_WARN_LOG_|_TA_LOG_, "TA: Illegal msg id:0x%04x\r\n", pMsg->hdr.id);
		}
		
		gMemFree(pMsg);

	}
	/* USER CODE END 5 */ 
}

int Wifi_test(char *arg) {
	(void) arg;
	
	HAL_GPIO_WritePin(GPIOC, WIFI_3V3_EN_Pin, GPIO_PIN_SET); // power pin
	HAL_GPIO_WritePin(WIFI_PWD_L_GPIO_Port, WIFI_PWD_L_Pin, GPIO_PIN_SET); // enable pin
	//HAL_GPIO_WritePin(GPIOG, WIFI_SPI3_CS_3V3_Pin, GPIO_PIN_RESET); // spi set 1, usb set 0
	return E_OK;
}

int BLE_test(char *arg) {
	uint8_t TxData[]={0x0a};
	uint8_t RxData[20]={0};

	(void) arg;
	if (HAL_UART_Transmit(&huart1, TxData, 1, 1000)!= HAL_OK)
	{
		//error expection
		uartPrintf("BLE: Error HAL_UART_Transmit \r\n");
		/* User may add here some code to deal with this error */
		return E_HAL;
	}
	if (HAL_UART_Receive(&huart1, RxData, 1, 1000) != HAL_OK)
	{
		//error expection
		uartPrintf("BLE: Error HAL_UART_Receive \r\n");
		/* User may add here some code to deal with this error */
		return E_HAL;
	}
	//print RxData
	uartPrintf("0x%x \r\n", RxData[0]);
	if (RxData[0] == 0x0a)
	{
		uartPrintf("BLE UART Success \r\n");
		// BLE_WAKE_Pin - pull high
		uartPrintf("BLE WAKE Pin pull high.... \r\n");
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET); // 0 - pull high
	}
	
	return E_OK;
}

int odmSpiFlashTest(char *arg)
{
	int ret,i,cnt;	
	unsigned int addr=0;
	uint32_t RDID = 0;
	unsigned char  *txbuff = pvPortMalloc(256 * sizeof(unsigned char ));
	unsigned char  *rxbuff = pvPortMalloc(256 * sizeof(unsigned char ));
 
	(void) arg;
//	uartPrintf("Verify SPI Flash\r\n");
//	fmRDID(&RDID);
//	uartPrintf("Read SPI Flash Id:0x%08X...", RDID);

//	if(RDID != SPIFLASH4M_ID)
//	{
//		uartPrintf("Fail\r\n");
//		vPortFree(txbuff);
//		vPortFree(rxbuff);
//		return -1;
//	}
//	uartPrintf("Pass\r\n");

	
//  ret=fmSectErase(addr);
//	uartPrintf("Erase SPI Flash....");
//	if(ret<0)
//	{
//		uartPrintf("Fail\r\n");
//		vPortFree(txbuff);
//		vPortFree(rxbuff);
//		return -2;
//	}
//	uartPrintf("Pass\r\n");
	
//	for(i =0; i<256 ;i++)
//    txbuff[i]=i;

//	ret=fmProg(addr,256,txbuff);
//	uartPrintf("Prog SPI Flash....");
//	if(ret<0)
//	{
//		uartPrintf("Fail\r\n");
//		vPortFree(txbuff);
//		vPortFree(rxbuff);
//		return -3;
//	}
//	uartPrintf("Pass\r\n");
	addr=0;
osDelay(3000);
for(cnt=0;cnt<12;cnt++)
{
	for(i=0;i<16;i++)
	{
		ret=fmRead(addr,256,rxbuff);
		HAL_UART_Transmit(&hlpuart1,rxbuff, 256, osWaitForever);
	addr+=256;
	}
}		
//	uartPrintf("Read SPI Flash....");
//	if(ret<0)
//	{
//		uartPrintf("Fail\r\n");
//		vPortFree(txbuff);
//		vPortFree(rxbuff); 
//		return -4;
//	}
//	uartPrintf("Pass\r\n");

//	uartPrintf("Check SPI Flash data....");
//	for(i=0;i<256;i++)
//	{
//	  if(txbuff[i]!=rxbuff[i]) {
//	  	uartPrintf("Fail\r\n");
//	  	vPortFree(txbuff);
//		vPortFree(rxbuff);
//			return -5;
//	  }
//	}
//	uartPrintf("Pass\r\n");
//	
//	vPortFree(txbuff);
	vPortFree(rxbuff);
	return 0;
}

int odmSensorTest(char *arg)
{
	unsigned char chipid;

	(void) arg;
	uartPrintf("Verify Sensors\r\n");
	#if AG6AXIS_EN == 1
	uartPrintf("Read Gyro Id");
	if (HAL_I2C_Mem_Read( &hi2c1, AG6AXIS_ADDR, AG6AXIS_WAMI, I2C_MEMADD_SIZE_8BIT, &chipid, 1, 0x4000 )==HAL_OK)
	{
		uartPrintf(":0x%04X....", chipid);
		if(AG6axis_RDID != chipid) {
			uartPrintf("Fail\r\n");
		}
		uartPrintf("Pass\r\n");
	}
	else
	{
		uartPrintf("....Fail\r\n");
	}
	#endif 	
	
	#if ACCE_EN == 1
	uartPrintf("Read Acce Id");
	if (HAL_I2C_Mem_Read( &hi2c1, ACCE_ADDR, ACCE_WAMI, I2C_MEMADD_SIZE_8BIT, &chipid, 1, 0x4000 )==HAL_OK)
	{
		uartPrintf(":0x%04X....", chipid);
		if(ACCE_RDID != chipid) {
			uartPrintf("Fail\r\n");
		}
		uartPrintf("Pass\r\n");
	}
	else
	{
		uartPrintf("....Fail\r\n");
	}
	#endif
	
	#if COMPASS_EN == 1
	uartPrintf("Read Magn Id");
	if (HAL_I2C_Mem_Read( &hi2c1, COMPASS_ADDR, COMPASS_WAMI, I2C_MEMADD_SIZE_8BIT, &chipid, 1, 0x4000 )==HAL_OK)
	{
		uartPrintf(":0x%04X....", chipid);
		if(COMPASS_RDID != chipid) {
	 		uartPrintf("Fail\r\n");
		}
		uartPrintf("Pass\r\n");
	}
	else
	{
		uartPrintf("....Fail\r\n");
	}
	#endif
	
	#if PRESSURE_EN == 1
	uartPrintf("Read Pres Id");
	if (HAL_I2C_Mem_Read( &hi2c1, PRESSURE_ADDR, PRESSURE_WAMI, I2C_MEMADD_SIZE_8BIT, &chipid, 1, 0x4000 )==HAL_OK)
	{
		uartPrintf(":0x%04X....", chipid);
		if(PRESSURE_RDID != chipid) {
			uartPrintf("Fail\r\n");
		}
		uartPrintf("Pass\r\n");
	}
	else
	{
		uartPrintf("....Fail\r\n");
	}
	#endif
	  
	#if TEMEPERATURE_EN == 1
	uartPrintf("Read Temp Id");
	if (HAL_I2C_Mem_Read( &hi2c1, TEMEPERATURE_ADDR, TEMEPERATURE_WAMI, I2C_MEMADD_SIZE_8BIT, &chipid, 1, 0x4000 )==HAL_OK)
	{
		uartPrintf(":0x%04X....", chipid);
		if(TEMEPERATURE_RDID != chipid) {
			uartPrintf("Fail\r\n");
		}
		uartPrintf("Pass\r\n");
	}
	else
	{
		uartPrintf("....Fail\r\n");
	}
	#endif	
	return E_OK;
}

int odmPowerBatt1Test(char *arg)
{
	uint16_t adcVolt;
	int16_t adcTemp;
    uint16_t gStatus;
	uint16_t  gCurrent;
	uint16_t gVolt;
	uint16_t gRsoc;
	uint16_t gTempRaw;
	int16_t  gTemp;
	_Bool bResult;
	int ret;

    ret = pmGetBattGauge(GAUGE_CMD_STATUS, &gStatus);
    if(ret !=0)
        uartPrintf("batt error cmd=0x%x.\r\n", GAUGE_CMD_STATUS);
    ret = pmGetBattGauge(GAUGE_CMD_CURRENT, &gCurrent);
    if(ret !=0)
        uartPrintf("batt error cmd=0x%x.\r\n", GAUGE_CMD_CURRENT);
    ret = pmGetBattGauge(GAUGE_CMD_VOLTAGE, &gVolt);
    if(ret !=0)
        uartPrintf("batt error cmd=0x%x.\r\n", GAUGE_CMD_VOLTAGE);
    ret = pmGetBattGauge(GAUGE_CMD_RSOC, &gRsoc);
    if(ret !=0)
        uartPrintf("batt error cmd=0x%x.\r\n", GAUGE_CMD_RSOC);
    ret = pmGetBattGauge(GAUGE_CMD_TEMP, &gTempRaw);
    if(ret !=0) {
		gTemp = 0;
        uartPrintf("batt error cmd=0x%x.\r\n", GAUGE_CMD_TEMP);
    } else
        gTemp = (gTempRaw - 2730) / 10;
    ret = pmGetBattVbatAdc(&adcVolt);
    if(ret !=0)
        uartPrintf("adc volt error.\r\n");
    ret = pmGetBattTempAdc(&adcTemp);
    if(ret !=0)
        uartPrintf("adc temp error.\r\n");
    
    pmGetChgPin(&bResult);
    uartPrintf("%s v=%d T=%d ", arg?arg:"0", adcVolt, adcTemp);
    osDelay(100);
    uartPrintf("0x%04x %d %d %d %d %d\r\n", gStatus, (int16_t)gCurrent, gVolt, gRsoc, gTemp, bResult);
    return E_OK;
}

int odmPowerBatt2Test(char *arg)
{
    _Bool bResult;
	char secBuf[32];
    int i;
    
    i= 1;
    osDelay(1000);
    uartPrintf("sec,V,T,sta,i,v,%%,t,chg#\r\n");
    osDelay(1000);
    while(i)
    {
    	sprintf(secBuf, "%04d", i++);
        odmPowerBatt1Test(secBuf);
        osDelay(900 - 9);
        pmGetPwrKey(&bResult);
        if(bResult == 0)
            break;
    }
    return E_OK;
}

int odmPowerResetTest(char *arg)
{
	pmSetMcuReset(0);
    return E_OK;
}

int odmPowerModeTest(char *arg)
{
	pmSetPwrState(POWER_MODE_STOP2);
    return E_OK;
}

int odmPowerKeysTest()
{
    _Bool bResult;

    osDelay(100);
    uartPrintf("get pwr key status\r\n");
    pmGetPwrKey(&bResult);
    uartPrintf("value=%02d\r\n", bResult);

    osDelay(100);
    uartPrintf("get chg#.\r\n");
    pmGetChgPin(&bResult);
    uartPrintf("value=%02d\r\n", bResult);

    return E_OK;
}

int odmPowerOffTest()
{
	int i;

    uartPrintf("set mcu_off 1\r\n");
    uartPrintf("power off . ");
    pmSetMcuOff(1);
    for(i=0; i<100; i++)
    {
    	osDelay(10);
    	uartPrintf(". ");
    }
    uartPrintf("\r\nfailed in power off.\r\n");
    uartPrintf("%d. set mcu_off 0\r\n");
    pmSetMcuOff(0);
    return E_OK;
}

int odmPowerControlTest(char *arg)
{
	int ret;
    uint8_t cmd;
    uint16_t sResult;
    int16_t  tResult;
    uint32_t iResult;
    _Bool bResult;

	(void) arg;
	
    cmd = 1;
    osDelay(100);
    uartPrintf("%d. get pwr key status\r\n", cmd);
    pmGetPwrKey(&bResult);
    uartPrintf("   value=%02d\r\n", bResult);
    
    cmd++;
    osDelay(100);
    uartPrintf("%d. get chg#.\r\n", cmd);
    pmGetChgPin(&bResult);
    uartPrintf("   value=%02d\r\n", bResult);
    
    cmd++;
    osDelay(100);
    uartPrintf("%d. get RCC-CSR.\r\n", cmd);
    pmGetMcuRccCsr(&iResult);
    uartPrintf("   value=0x%08x\r\n", iResult);
        
    cmd++;
    osDelay(100);
    uartPrintf("%d. get temp adc.\r\n", cmd);
    ret = pmGetBattTempAdc(&tResult);
    uartPrintf("   result=%d, temp=%dC\r\n", ret, tResult);
    
    cmd++;
    osDelay(100);
    uartPrintf("%d. get Vbat adc.\r\n", cmd);
    ret = pmGetBattVbatAdc(&sResult);
    uartPrintf("   result=%d, Vbat=%dmV\r\n", ret, sResult);
    
    cmd++;
    osDelay(100);
    uartPrintf("%d. get Gauge information.\r\n", cmd);
    ret = pmGetBattGauge(0x0a, &sResult);
    uartPrintf("   result=%d, status=0x%x\r\n", ret, sResult);
    osDelay(100);
    ret = pmGetBattGauge(0x08, &sResult);
    uartPrintf("   result=%d, V=%d\r\n", ret, sResult);
    osDelay(100);
    ret = pmGetBattGauge(0x0c, &sResult);
    uartPrintf("   result=%d, I=%d\r\n", ret, (int16_t)sResult);
    osDelay(100);
    ret = pmGetBattGauge(0x06, &sResult);
    uartPrintf("   result=%d, temp=%d\r\n", ret, sResult);
    osDelay(100);
    ret = pmGetBattGauge(0x2c, &sResult);
    uartPrintf("   result=%d, rsoc=%d\r\n", ret, sResult);
    osDelay(100);

	return E_OK;
}

int odmModemTest(char *arg)
{
	int ret=0;
	char ATcomBuf[32]={0};
	char RecBuf[32]={0};

	HAL_StatusTypeDef uart_result = HAL_OK;

	(void) arg;
	memset(ATcomBuf, 0, sizeof(ATcomBuf));
	strcpy(ATcomBuf, "\r\nAT\r\n");

    ret = mmUartSendData((uint8_t*) ATcomBuf, (uint16_t)strlen(ATcomBuf), HAL_MAX_DELAY);
	gLog(_DEBUG_LOG_|_MM_LOG_, "mmUartSend ret = %d\r\n", ret);

	osDelay(100);

	memset(RecBuf, 0, sizeof(RecBuf));
	uart_result = HAL_UART_Receive(&huart2, (uint8_t *)RecBuf, sizeof(RecBuf), 5000);
	if(uart_result != HAL_OK)
	{
		//gLog(_CRIT_LOG_|_MM_LOG_, "Modem HAL_UART_Receive FAILED, uart_result %d\r\n", uart_result);
		uartPrintf("Modem HAL_UART_Receive FAILED\r\n");
		return -1;
	}

	uartPrintf("Received \"%s\" from huart2\r\n", RecBuf);

	if(strcmp(RecBuf, "\r\nOK\r\n") == 0)
	{
		ret = 0;
		uartPrintf("odmModemTest PASS\r\n");
	}
	else
	{
		ret =-1;
		//gLog(_CRIT_LOG_|_MM_LOG_, "odmModemTest FAILED! RecBuf=%s\r\n", RecBuf);
		uartPrintf("Modem FAILED! RecBuf=%s\r\n", RecBuf);
	}

	return ret;

}

int odmGpsTest(char *arg)
{
#ifndef FLASH_NO_FIRMWARE
    int i = 0;
    uint8_t aTxBuf[7] = "@ver\r\n";
    uint8_t aRxBuf[19] = {0};
    int ret = 0;
    uartPrintf("Verify GPS...\r\n");
    HAL_GPIO_WritePin(GPS_PMIC_CE_GPIO_Port, GPS_PMIC_CE_Pin, GPIO_PIN_SET);
    uartPrintf("GPS PMIC CE enabled.\r\n");
    for (i = 0; i < 3; i++) {
        ret = HAL_UART_Transmit(&huart3, (uint8_t*)aTxBuf, 6, 5000);
        if (ret != HAL_OK) {
            uartPrintf("{%d}Send @VER failed.\r\n", ret);
        }
        else
            uartPrintf("Send @VER success.\r\n");
        ret = HAL_UART_Receive(&huart3, (uint8_t*)aRxBuf, 19, 5000);
        if (ret == HAL_OK) {
            uartPrintf("Read version success.\r\n");
            uartPrintf("%s\r\n", (uint8_t*)aRxBuf);
            uartPrintf("GPS test ended.\r\n");
            return 0;
        }
        else {
           uartPrintf("Read version failed.\r\n");
           uartPrintf("%s\r\n", (uint8_t*)aRxBuf);
        }
    }
    uartPrintf("GPS test failed.\r\n");
    return -1;
#else
    int i = 0;
    uint8_t aTxBuf[] = {0x0D};
    uint8_t aRxBuf[2] = {0};
    uartPrintf("Verify GPS...\r\n");
    HAL_GPIO_WritePin(GPS_PMIC_CE_GPIO_Port, GPS_PMIC_CE_Pin, GPIO_PIN_SET);
    uartPrintf("GPS PMIC CE enabled.\r\n");
    /* Send re-send <CR> 10 times to get H> */
    for(i = 0; i < 10; i++) {
        osDelay(100);
        if (HAL_UART_Transmit(&huart3, (uint8_t*)aTxBuf, 1, HAL_MAX_DELAY) != HAL_OK) {
            uartPrintf("Send <CR> failed.\r\n");
        }
        else
            uartPrintf("Send <CR> success.\r\n");

        if (HAL_UART_Receive(&huart3, (uint8_t*)aRxBuf, 2, 100) == HAL_OK) {
            if ((char)aRxBuf[0] == 'H' && (char)aRxBuf[1] == '>') {
                uartPrintf("Read %c%c success.\r\n", aRxBuf[0], aRxBuf[1]);
                uartPrintf("GPS test ended.\r\n");
                return 0;
            }
        }
        else {
           uartPrintf("Read H> failed.\r\n");
        }
    }
    uartPrintf("GPS test failed.\r\n");
    return -1;
#endif
}

int odmAcceTest(char *arg)
{
		int16_t data[3], i;
		uint8_t RDID;
		smDev_st gsens;
		
		LIS2DH12_GetWHO_AM_I(&RDID);
		uartPrintf("gSensorID:  0x%02x\r\n",RDID);
		
		AccConfig(&gsens);		
		AccInit(&gsens);
		
		if(smSetAccPwr(SM_ENABLE) != E_OK)
			uartPrintf("PWR FAIL\n");

		for (i=0; i<10; i++)
		{
			osDelay(1000);
			if(GetAccVal(data) != E_OK)
			uartPrintf("GetData FAIL\n");
			uartPrintf("X=%d Y=%d Z=%d \r\n", data[0],data[1], data[2]);					
		}
	return E_OK;
}
void pf_init(void* ctx,_Bool sat)
{
	uartPrintf( "Snsor init %s\n",(sat)?"OK":"Fail");
}
int odmTempTest(char *arg)
{
	float temperatur;
	int32_t d3, d4, i;
	uint8_t RDID;

//	HTS221_Get_DeviceID(&hi2c1, &RDID);
//	uartPrintf("tSensorID:  0x%02x\r\n",RDID);

//    smSensorInt(SM_TIMPERIOD,pf_init,NULL);
//	osDelay(20);
//	
//	if(smSetTempPwr(SM_ENABLE) != E_OK)
//		uartPrintf("PWR FAIL\n");

//	for (;;)
//	{
//		osDelay(1000);
//		if(GetTempVal(&temperatur)!=E_OK)
//			uartPrintf("GetData FAIL\n");
//		floatToInt(temperatur, &d3, &d4, 2);
//		uartPrintf("Temper:  %d.%02d\r\n",(int)d3, (int)d4);
//	}
		for (;;)
		{
		
			osDelay(1000);
			if(AS6200_GetTempVal(&temperatur)!= E_OK)
				uartPrintf("AS6200_GetTempVal FAIL\n");
			floatToInt(temperatur, &d3, &d4, 2);
			uartPrintf("Temper:  %d.%02d\r\n",(int)d3, (int)d4);		
		}
		


	return E_OK;
}


void pf_accel(void* ctx,int16_t *value)
{
	uartPrintf( "ACC: X=%d Y=%d Z=%d\n", value[0],value[1], value[2]);	
	
}
int odmAcceFifoTest(char *arg)
{
		smAccCallback pfung;
		int cnt=0;
	
	 smSensorInt(SM_FIFOIRQ,pf_init,NULL);
	 osDelay(20);

	 if(smSetAccPwr(SM_ENABLE) != E_OK)
		 uartPrintf( "tPWR FAIL\n");		
		
	  pfung = pf_accel;

 	if(smGetAccData(pfung,NULL)!= E_OK)
	   uartPrintf("tGetData FAIL\n");

		for(;;)
		{	
			osDelay(100);
			if(cnt++ >100)
				break;
		}
		
	 if(smSetAccPwr(SM_DISABLE) != E_OK)
	  uartPrintf( "tPWR FAIL\n");	
	
		smGetAccData_remove();	 
	return E_OK;
}
void pf_temper(void* ctx,float value)
{
	int32_t d3, d4;
	floatToInt(value, &d3, &d4, 2);
	uartPrintf( "Temper:  %d.%02d\n",(int)d3, (int)d4);	
}


int odmTempFifoTest(char *arg)
{
		smTempCallback pfunT;
		int cnt=0;
	
	 smSensorInt(SM_FIFOIRQ,pf_init,NULL);
	 osDelay(20);
	
	 if(smSetTempPwr(SM_ENABLE) != E_OK)
		 uartPrintf( "tPWR FAIL\n");		
		
	  pfunT = pf_temper;

 	if(smGetTempData(pfunT,NULL)!= E_OK)
	   uartPrintf("tGetData FAIL\n");

		for(;;)
		{	
			osDelay(1);
			//if(cnt++ >100)
			//break;
		}
		
	 if(smSetTempPwr(SM_DISABLE) != E_OK)
	  uartPrintf( "tPWR FAIL\n");	
	
		smGetTempData_remove();	 
	return E_OK;
}

int odmGetSN(char *arg)
{
	char serBuf[20];
	uint32_t len;

	(void) arg;
	
	cmRead(CM_SERIAL, serBuf, &len);
	serBuf[16] = '\0';
	uartPrintf("SN: %s\r\n", serBuf);

	return E_OK;
}

int odmSetSN(char *arg)
{
	char serBuf[20];
	uint32_t len;

	if(arg == NULL)
		return E_ILLEGAL;

	len = strlen(arg);
	if(16 < len)
		return E_LEN;

	memset(serBuf, 0x00, sizeof(serBuf));
	strncpy(serBuf, arg, len);
	cmWrite(CM_SERIAL, serBuf, 16);
	cm_Program();

	return E_OK;
}


int odmFwVer(char *arg)
{
	uint32_t maj, min;

	(void) arg;
	gSwVer(&maj, &min);
	uartPrintf("MCU BSP ver:%02d.%02d\r\n", maj, min);

	return E_OK;
}

int odmFsTest(char *arg)
{
	fmSampCode(arg);
	
	return E_OK;
}

int odmMkfs(char *arg)
{
	if(f_mkfs(USER_Path, 1, 4096) != FR_OK)
	{
		/* FatFs Format Error */
		uartPrintf("Format failed\r\n");
		return E_HW;
	}
	return E_OK;
}

int odmLsfs(char *arg)
{
	FILINFO fileinfo;
	FRESULT res;
	DIR dir;
	char path[32] = "0:/";

	if(arg != NULL)
		strcat(path, arg);
	
	if(f_opendir(&dir, path) != FR_OK)
	{		
		uartPrintf("oepn directory Error\r\n");
		uartPrintf("Try format fs again\r\n");
		return E_HAL;
	}

	uartPrintf("List %s\r\n\r\n", path);
	while(1)
	{	
		res = f_readdir(&dir, &fileinfo); 
		
		if (res != FR_OK || fileinfo.fname[0] == 0)
		{
			break;	
		}
		
		if(fileinfo.fattrib & AM_DIR)
			uartPrintf("%s\t\t<DIR>\n", fileinfo.fname);
		else
			uartPrintf("%s\t\t%d\n", fileinfo.fname, fileinfo.fsize);
	}		
	f_closedir(&dir);
	
	return E_OK;
}

int odmRmfs(char *arg)
{
	char path[32] = "0:/";

	if(arg == NULL)
		return E_ILLEGAL;
	
	strcat(path, arg);

	if(FR_OK != f_unlink(path))
		return E_HAL;

	return E_OK;
}

int odmLogMask(char *arg)
{
	uint32_t conMask, logMask;
	char *pstr, *delim = " ";;

	if(E_OK != gLogGetMask(&conMask, &logMask)) {
		uartPrintf("Read log mask failed\r\n");
		return E_HAL;
	}
	uartPrintf("Get con mask: 0x%08X\r\n", conMask);
	uartPrintf("Get log mask: 0x%08X\r\n", logMask);

	if(arg) {
		conMask = strtoul(arg, NULL, 0);
		pstr = strtok(NULL, delim);
		if(NULL == pstr)
			return E_ILLEGAL;
		logMask = strtoul(pstr, NULL, 0);
		uartPrintf("Set con mask: 0x%08X\r\n", conMask);
		uartPrintf("Set log mask: 0x%08X\r\n", logMask);
		if(E_OK != gLogSetMask(conMask, logMask))
			return E_HAL;
	}

	return E_OK;
}



/************************************************
* 				Sensor manager sample task		 				*
************************************************/
uint8_t Int_complete = 0;
void printf_IsInitial(void* ctx,_Bool sat)
{	
	uartPrintf( "Snsor init %s\n",(sat)?"OK!":"Fail!");	
	Int_complete = 1;
	
}

void printf_temper(void *ctx, float value)
{
	int32_t d3, d4;
	floatToInt(value, &d3, &d4, 2);
	uartPrintf( "Temper:  %d.%02d\n",(int)d3, (int)d4);	
}

void printf_Accel(void *ctx, int16_t *value)
{
	uartPrintf( "ACC: X=%d Y=%d Z=%d\n", value[0],value[1], value[2]);	
	
}

void printf_AccelMv(void *ctx, int16_t *value)
{
	uartPrintf("\nTHA: X=%d Y=%d Z=%d\n\n", value[0],value[1], value[2]);	
	
}

int odmsmTask(char *arg)
{
		int ret;

		smTempCallback pfunT;
		smAccCallback pfunG;
		smAccMovCallback pMvfun;
	
#if 0
		if( smSensorInt(SM_TIMPERIOD,printf_IsInitial,NULL) != E_OK)
		uartPrintf( "Sensor initial FAIL\n");
		
		if(smSetTempQueryTime(1000) != E_OK) 	//1s
			uartPrintf( "set tpolling FAIL\n");
		if(smSetAccQueryTime(500) != E_OK)		//0.5s
		  uartPrintf( "set gpolling  FAIL\n");
# else	
		if( smSensorInt(SM_FIFOIRQ,printf_IsInitial,NULL) != E_OK)
		uartPrintf( "Sensor initial FAIL\n");
#endif	
	
	while(!Int_complete){osDelay(10);};
	Int_complete = 0;
		
	ret = smSetTempPwr(SM_ENABLE);
	if(ret != E_OK)
		uartPrintf( "TSensor initial ErrMsg:%d \r\n", ret);
	
	ret = smSetAccPwr(SM_ENABLE);
	if(ret != E_OK)
		uartPrintf( "GSensor initial ErrMsg:%d \r\n", ret);			

	
	pfunT = printf_temper;
	pfunG = printf_Accel;
	pMvfun = printf_AccelMv;
	

	if(smGetTempData(pfunT,NULL)!= E_OK)
	 uartPrintf("tGetData FAIL\n");
	
		
		if(smGetAccData(pfunG,NULL)!= E_OK)
	 uartPrintf( "gGetData FAIL\n");
		
	  if(smAccMvDt(30,(SMXUPE | SMYUPE), pMvfun, NULL)!= E_OK)
			uartPrintf( "gMvDt FAIL\n");

	for(;;)
	{
		osDelay(5000);
	if(smSetTempPwr(SM_DISABLE) != E_OK)
		uartPrintf( "tPWR1 FAIL\n");
	
	 osDelay(5000);
	if(smSetTempPwr(SM_ENABLE) != E_OK)
		uartPrintf( "tPWR2 FAIL\n");
	
	 osDelay(5000);
	if(smSetAccPwr(SM_DISABLE) != E_OK)
		uartPrintf( "gPWR1 FAIL\n");

		osDelay(5000);
	if(smSetAccPwr(SM_ENABLE) != E_OK)
		uartPrintf( "gPWR2 FAIL\n");
	
	}
	return E_OK;
}
	
