//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//
#include "fmApi.h"
#include "genApi.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "errDef.h"
#include "uart.h"

#include <string.h>

#define BKPADDR 	0x35F000
#define FATADDR 	0x1000
#define FAT1ADDR 	0x1000
#define FAT2ADDR 	0x2000
#define ROOTADDR 	0x3000

FATFS SPIFatFs; 
FIL file;
DIR dir;

osMutexId Mutex_QSPI = NULL;	
extern QSPI_HandleTypeDef hqspi;


__IO uint8_t CmdCplt, RxCplt, TxCplt, StatusMatch, TimeOut;


void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi)
{
  CmdCplt++;
}


void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
  RxCplt++;
}


 void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
  TxCplt++;
}


void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *hqspi)
{
  StatusMatch++;
}


void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi)
{
	
	gLog(_INFO_LOG_|_FM_LOG_, "Error HAL_QSPI_ErrorCallback");
}



int  QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef     sCommand;
  QSPI_AutoPollingTypeDef sConfig;

  /* Enable write operations ------------------------------------------ */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = WRITE_ENABLE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
		gLog(_INFO_LOG_|_FM_LOG_, "WriteEnable Err\r\n");
			return E_HAL;
  }

  /* Configure automatic polling mode to wait for write enabling ---- */
  sConfig.Match           = 0x02;
  sConfig.Mask            = 0x02;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x10;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  sCommand.Instruction    = READ_STATUS_REG_CMD;
  sCommand.DataMode       = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(hqspi, &sCommand, &sConfig, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
			gLog(_INFO_LOG_|_FM_LOG_, "WriteEnable Err\r\n");
			return E_HAL;
  }

	return E_OK;
}


int QSPI_Polling_IT(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef     sCommand;
  QSPI_AutoPollingTypeDef sConfig;

  /* Configure automatic polling mode to wait for memory ready ------ */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_STATUS_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  sConfig.Match           = 0x00;
  sConfig.Mask            = 0x01;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x10;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling_IT(hqspi, &sCommand, &sConfig) != HAL_OK)
  {
			gLog(_INFO_LOG_|_FM_LOG_, "Polling_IT Err\r\n");
			return E_HAL;
  }
	return E_OK;
}

int QSPI_Polling(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef     sCommand;
  QSPI_AutoPollingTypeDef sConfig;

  /* Configure automatic polling mode to wait for memory ready ------ */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_STATUS_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  sConfig.Match           = 0x00;
  sConfig.Mask            = 0x01;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x10;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling(hqspi, &sCommand, &sConfig,0xFFFFFFFF) != HAL_OK)
  {
			gLog(_INFO_LOG_|_FM_LOG_, "Polling Err\r\n");
			return E_HAL;
  }
	return E_OK;
}



int  OSPI_QuadEnable(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef sCommand;
  uint8_t reg;

  /* Read Volatile Configuration register --------------------------- */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_STATUS_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.NbData            = 1;

  if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
   {
			gLog(_INFO_LOG_|_FM_LOG_, "QuadEnable Err\r\n");
			return E_HAL;
  }
  if (HAL_QSPI_Receive(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
   gLog(_INFO_LOG_|_FM_LOG_, "QuadEnable Err\r\n");
			return E_HAL;
  }

  /* Enable write operations ---------------------------------------- */
  QSPI_WriteEnable(hqspi);

  /* Write Volatile Configuration register (with new dummy cycles) -- */
  sCommand.Instruction = 0x01;
  reg |= 0x40;


  if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
			gLog(_INFO_LOG_|_FM_LOG_, "QuadEnable Err\r\n");
			return E_HAL;
	}

  if (HAL_QSPI_Transmit(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
		{
		gLog(_INFO_LOG_|_FM_LOG_, "QuadEnable Err\r\n");
			return E_HAL;
	}

	return E_OK;
}

int QSPI_RDID(QSPI_HandleTypeDef *hqspi, uint32_t* data)
{
	/*@retval number of Byte for read */
		QSPI_CommandTypeDef sCommand;		

	/* Reading Sequence ------------------------------------------------ */
		sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
		sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
		sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
		sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
		sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

		sCommand.Instruction = RDID_CMD;
		sCommand.AddressMode = QSPI_ADDRESS_NONE;
		sCommand.DataMode    = QSPI_DATA_1_LINE;
		sCommand.Address     = 0;
		sCommand.NbData      = 3;
		sCommand.DummyCycles = 0;

		if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
		{
			gLog(_INFO_LOG_|_FM_LOG_, "RDID Err\r\n");
			return E_HAL;
		}
		RxCplt=0;
		if (HAL_QSPI_Receive(hqspi, (uint8_t*)data,0xFFFFFFFF) != HAL_OK)
		{
			gLog(_INFO_LOG_|_FM_LOG_, "RDID Err\r\n");
			return E_HAL;
		}
		
		return  E_OK;
}

int QSPI_RDSR(QSPI_HandleTypeDef *hqspi, uint8_t* data)
{
	/*@retval number of Byte for read */
		QSPI_CommandTypeDef sCommand;

	/* Reading Sequence ------------------------------------------------ */
		sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
		sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
		sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
		sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
		sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

		sCommand.Instruction = READ_STATUS_REG_CMD;
		sCommand.AddressMode = QSPI_ADDRESS_NONE;
		sCommand.DataMode    = QSPI_DATA_1_LINE;
		sCommand.Address     = 0;
		sCommand.NbData      = 1;
		sCommand.DummyCycles = 0;

		if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
		{
			gLog(_INFO_LOG_|_FM_LOG_, "RDSR Err\r\n");
			return E_HAL;
		}
		RxCplt=0;
		if (HAL_QSPI_Receive(hqspi, (uint8_t*)data,0xFFFFFFFF) != HAL_OK)
		{
					gLog(_INFO_LOG_|_FM_LOG_, "RDSRReceive Err\r\n");
					return E_HAL;
		}
		return E_OK;
}


int QSPI_WRSR(QSPI_HandleTypeDef *hqspi, uint8_t data)
{
  QSPI_CommandTypeDef sCommand;


 /* Enable write operations ---------------------------------------- */
  QSPI_WriteEnable(hqspi);

  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = WRITE_STATUS_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.NbData            = 1;


  if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		gLog(_INFO_LOG_|_FM_LOG_, "WRSR Err\r\n");
		return E_HAL;
	}

  if (HAL_QSPI_Transmit(hqspi, &data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		gLog(_INFO_LOG_|_FM_LOG_, "WRSRTransmit Err\r\n");
		return E_HAL;
	}

	return E_OK;
}

int QSPI_Read_IT(QSPI_HandleTypeDef *hqspi, uint32_t addr,uint32_t Bcount, uint8_t* data)
{
	/*@retval number of Byte for read */

	
		QSPI_CommandTypeDef sCommand;
		unsigned int timeout=0;

	/* Reading Sequence ------------------------------------------------ */
		sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
		sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
		sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
		sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
		sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

		sCommand.Instruction = QUAD_4READ_CMD;
		sCommand.AddressMode = QSPI_ADDRESS_4_LINES;
		sCommand.DataMode    = QSPI_DATA_4_LINES;
		sCommand.Address     = addr;
		sCommand.NbData      = Bcount;
		sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_4READ;

		if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
		{
			gLog(_INFO_LOG_|_FM_LOG_, "Read_IT Err\r\n");
			return E_HAL;
		}
		RxCplt=0;
		if (HAL_QSPI_Receive_DMA(hqspi, data) != HAL_OK)
		{
			gLog(_INFO_LOG_|_FM_LOG_, "ReceiveDMA Err\r\n");
			return E_HAL;
		}
			while(RxCplt ==0)
			{
				if(++timeout > 0x2000000)
				{
					gLog(_INFO_LOG_|_FM_LOG_, "ReceiveDMA timeout\r\n");
					return E_HAL;
				}
			};
			RxCplt = 0;


		return  E_OK;
}


int QSPI_Read(QSPI_HandleTypeDef *hqspi, uint32_t addr,uint32_t Bcount, uint8_t* data)
{
	/*@retval number of Byte for read */

		QSPI_CommandTypeDef sCommand;

	/* Reading Sequence ------------------------------------------------ */
		sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
		sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
		sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
		sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
		sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

		sCommand.Instruction = QUAD_4READ_CMD;
		sCommand.AddressMode = QSPI_ADDRESS_4_LINES;
		sCommand.DataMode    = QSPI_DATA_4_LINES;
		sCommand.Address     = addr;
		sCommand.NbData      = Bcount;
		sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_4READ;

		if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
		{
			gLog(_INFO_LOG_|_FM_LOG_, "Read Err\r\n");
			return E_HAL;
		}
		RxCplt=0;
		if (HAL_QSPI_Receive(hqspi, data,0xFFFFFFFF) != HAL_OK)
		{
			gLog(_INFO_LOG_|_FM_LOG_, "Receive Err\r\n");
			return E_HAL;
		}

		return E_OK;
}



int QSPI_Write_IT(QSPI_HandleTypeDef *hqspi, uint32_t addr,uint32_t Bcount, uint8_t* data)
{

		QSPI_CommandTypeDef sCommand;
		unsigned int timeout=0;
	
	  if(Bcount>256)
	  {
			gLog(_INFO_LOG_|_FM_LOG_, "Over page size 256Byte\r\n");
			return E_LEN;
		}

		/* Enable write operations ----------------------------------------- */
		QSPI_WriteEnable(hqspi);

		sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
		sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
		sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
		sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
		sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

		/* Writing Sequence ------------------------------------------------ */
		sCommand.Instruction = QUAD_PAGE_PROG_CMD;
		sCommand.AddressMode = QSPI_ADDRESS_4_LINES;
		sCommand.DataMode    = QSPI_DATA_4_LINES;
		sCommand.NbData      = Bcount;
		sCommand.Address     = addr;
		sCommand.DummyCycles = 0;
		if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
		{
			gLog(_INFO_LOG_|_FM_LOG_, "Program_IT Err\r\n");
			return E_HAL;
		}
		
		
		TxCplt = 0;
		if (HAL_QSPI_Transmit_DMA(hqspi, data) != HAL_OK)
		{
			gLog(_INFO_LOG_|_FM_LOG_, "TransmitDMA Err\r\n");
			return E_HAL;
		}
			while(TxCplt ==0)
			{
				if(++timeout > 0x2000000)
				{
					gLog(_INFO_LOG_|_FM_LOG_, "TransmitDMA timeout\r\n");
					return E_HAL;
				}
			};		
		TxCplt = 0;
		StatusMatch = 0;
		QSPI_Polling_IT(hqspi);
		while(StatusMatch ==0){};
	  return E_OK;
}


int QSPI_Write(QSPI_HandleTypeDef *hqspi, uint32_t addr,uint32_t Bcount, uint8_t* data)
{

		QSPI_CommandTypeDef sCommand;

	  if(Bcount>256)
	  {
		  gLog(_INFO_LOG_|_FM_LOG_, "Over page size 256Byte\r\n");
			return E_LEN;
		}

		/* Enable write operations ----------------------------------------- */
		QSPI_WriteEnable(hqspi);

		sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
		sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
		sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
		sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
		sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;


		/* Writing Sequence ------------------------------------------------ */
		sCommand.Instruction = QUAD_PAGE_PROG_CMD;
		sCommand.AddressMode = QSPI_ADDRESS_4_LINES;
		sCommand.DataMode    = QSPI_DATA_4_LINES;
		sCommand.NbData      = Bcount;
		sCommand.Address     = addr;
		sCommand.DummyCycles = 0;
		if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
		{
			gLog(_INFO_LOG_|_FM_LOG_, "Program Err\r\n");
			return E_HAL;
		}

		if (HAL_QSPI_Transmit(hqspi, data,0xFFFFFFFF) != HAL_OK)
		{
			gLog(_INFO_LOG_|_FM_LOG_, "Transmit Err\r\n");
			return E_HAL;
		}

		QSPI_Polling(hqspi);

	  return E_OK;
}



int QSPI_BlockErase_IT(QSPI_HandleTypeDef *hqspi, uint32_t addr)
{
		QSPI_CommandTypeDef sCommand;

		/* Enable write operations ----------------------------------------- */
		QSPI_WriteEnable(hqspi);


		sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
		sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
		sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
		sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
		sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

		/* Erasing Sequence -------------------------------------------------- */
		sCommand.Instruction = BULK_ERASE_CMD;
		sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
		sCommand.Address     = addr;
		sCommand.DataMode    = QSPI_DATA_NONE;
		sCommand.DummyCycles = 0;

		if (HAL_QSPI_Command_IT(hqspi, &sCommand) != HAL_OK)
		{
			gLog(_INFO_LOG_|_FM_LOG_, "BlockErase_IT Err\r\n");
			return E_HAL;
		}

			while(CmdCplt ==0){};
			CmdCplt = 0;

			StatusMatch = 0;
			QSPI_Polling_IT(hqspi);
			while(StatusMatch ==0){};

		return E_OK;
}
int QSPI_BlockErase(QSPI_HandleTypeDef *hqspi, uint32_t addr)
{
		QSPI_CommandTypeDef sCommand;

		/* Enable write operations ----------------------------------------- */
		QSPI_WriteEnable(hqspi);


		sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
		sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
		sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
		sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
		sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

		/* Erasing Sequence -------------------------------------------------- */
		sCommand.Instruction = BULK_ERASE_CMD;
		sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
		sCommand.Address     = addr;
		sCommand.DataMode    = QSPI_DATA_NONE;
		sCommand.DummyCycles = 0;

		if (HAL_QSPI_Command(hqspi, &sCommand,0xffffffff) != HAL_OK)
		{
			gLog(_INFO_LOG_|_FM_LOG_, "BlockErase_IT Err\r\n");
			return E_HAL;
		}

		QSPI_Polling(hqspi);
		return E_OK;
}
int QSPI_SectorErase_IT(QSPI_HandleTypeDef *hqspi, uint32_t addr)
{
		QSPI_CommandTypeDef sCommand;

		/* Enable write operations ----------------------------------------- */
		QSPI_WriteEnable(hqspi);

		sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
		sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
		sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
		sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
		sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

		/* Erasing Sequence -------------------------------------------------- */
		sCommand.Instruction = SECTOR_ERASE_CMD;
		sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
		sCommand.Address     = addr;
		sCommand.DataMode    = QSPI_DATA_NONE;
		sCommand.DummyCycles = 0;

		if (HAL_QSPI_Command_IT(hqspi, &sCommand) != HAL_OK)
		{
			gLog(_INFO_LOG_|_FM_LOG_, "SectorErase_IT Err\r\n");
			return E_HAL;
		}

			while(CmdCplt ==0){};
			CmdCplt = 0;

			StatusMatch = 0;
			QSPI_Polling_IT(hqspi);
			while(StatusMatch ==0){};

		return E_OK;
}

int QSPI_SectorErase(QSPI_HandleTypeDef *hqspi, uint32_t addr)
{
		QSPI_CommandTypeDef sCommand;

		/* Enable write operations ----------------------------------------- */
		QSPI_WriteEnable(hqspi);

	  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
		sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
		sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
		sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
		sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

		/* Erasing Sequence -------------------------------------------------- */
		sCommand.Instruction = SECTOR_ERASE_CMD;
		sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
		sCommand.Address     = addr;
		sCommand.DataMode    = QSPI_DATA_NONE;
		sCommand.DummyCycles = 0;

		if (HAL_QSPI_Command(hqspi, &sCommand,0xffffffff) != HAL_OK)
		{
			gLog(_INFO_LOG_|_FM_LOG_, "SectorErase Err\r\n");
			return E_HAL;
		}
		QSPI_Polling(hqspi);
		return E_OK;
}

int QSPI_DP(QSPI_HandleTypeDef *hqspi)
{
		QSPI_CommandTypeDef sCommand;

	  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
		sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
		sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
		sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
		sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

		/* DP Sequence -------------------------------------------------- */
		sCommand.Instruction = DEEP_POWER_CMD;
		sCommand.AddressMode = QSPI_ADDRESS_NONE;
		sCommand.Address     = 0;
		sCommand.DataMode    = QSPI_DATA_NONE;
		sCommand.DummyCycles = 0;

		if (HAL_QSPI_Command(hqspi, &sCommand,0xffffffff) != HAL_OK)
		{
			gLog(_INFO_LOG_|_FM_LOG_, "Deep PwrD CMD Err\r\n");
			return E_HAL;
		}
		
		osDelay(1); //delay minimum 50 us for into  deep power down mode
		return E_OK;
}

int QSPI_DPResume(QSPI_HandleTypeDef *hqspi)
{

		QSPI_CommandTypeDef sCommand;

	  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
		sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
		sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
		sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
		sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

		/* DP Sequence -------------------------------------------------- */
		sCommand.Instruction = 0x00;
		sCommand.AddressMode = QSPI_ADDRESS_NONE;
		sCommand.Address     = 0;
		sCommand.DataMode    = QSPI_DATA_NONE;
		sCommand.DummyCycles = 0;

		if (HAL_QSPI_Command(hqspi, &sCommand,0xffffffff) != HAL_OK)
		{
			gLog(_INFO_LOG_|_FM_LOG_, "Deep PwrD CMD Err\r\n");
			return E_HAL;
		}		
		osDelay(1); //delay minimum 50 us for into  deep power down mode
		return E_OK;
}

/*** FAT table backup app ***/
uint32_t datacopy(unsigned int saddr, unsigned int taddr)
{
			int i;
			unsigned char Dbuf[256],cnt;
			uint32_t checksum=0;
			for(cnt=0;cnt<15;i++)
			{
					fmRead(saddr,256,Dbuf);
					fmProg(taddr,256,Dbuf);			
					saddr+=256;
					taddr+=256;		
				
					for(i=0;i<256;i++)
					checksum+=Dbuf[i];				
			}
			return	checksum;	
}

void fatcopy(unsigned int saddr, unsigned int taddr)
{
			int i;
			unsigned char Dbuf[256],cnt;
			for(cnt=0;cnt<15;i++)
			{
					fmRead(saddr,256,Dbuf);
					fmProg(taddr,256,Dbuf);			
					saddr+=256;
					taddr+=256;			
			}	
}


uint32_t getBkchecksum(unsigned int taddr)
{
			int i;
			unsigned char Dbuf[256],cnt;
			uint32_t checksum=0;
			for(cnt=0;cnt<15;i++)
			{
					fmRead(taddr,256,Dbuf);
					taddr+=256;				
					for(i=0;i<256;i++)
					checksum+=Dbuf[i];				
			}
			return	checksum;	
}

_Bool datacmp(unsigned int addr1, unsigned int addr2)
{
			int i;
			unsigned char Dbuf[256],bkbuf[256],cnt;
			for(cnt=0;cnt<15;cnt++)
			{				
				fmRead(addr1,256,Dbuf);
				fmRead(addr2,256,bkbuf);
				addr1+=256;
				addr2+=256;
				
				for(i=0;i<256;i++)
				{
				 if(Dbuf[i] != bkbuf[i])
					return pdFAIL;
				}
			}
			return pdTRUE;
}
_Bool fatbkup(void)
{
		unsigned int checksum=0;
		unsigned char Dbuf[256],cnt=0;
		uint8_t* pChk;
	
		if(osMutexWait(Mutex_QSPI, 0x1FFFFFF) == osOK)
		{
			fmSectErase(BKPADDR);
			checksum=datacopy(FATADDR,BKPADDR);
			pChk=(uint8_t*)(&checksum);
			fmProg(BKPADDR+256*15,4,pChk);
			osMutexRelease(Mutex_QSPI);
		}
		return pdFALSE;
	
	  return pdTRUE;
}


_Bool  fatcheck(void)
{
	  unsigned int checksum=0,bksum=0,i=0;
		unsigned char Dbuf[256],bkbuf[256],cnt;
		uint8_t* pChk;
		pChk=(uint8_t*)(&checksum);
		
		checksum=getBkchecksum(BKPADDR);
		fmRead(BKPADDR+256*15,4,(uint8_t*)(&bksum));
				
		if(checksum == bksum)
		{					
			if(datacmp(FATADDR,BKPADDR)== pdFAIL )
			{
				fmSectErase(FATADDR);
				checksum=datacopy(BKPADDR,FATADDR);	
			}				
		}	
		return pdTRUE;
}

/*scan disk*/
int f_sckfat12(void)
{
		   int i=0;
			 uint8_t Fbuf[32];
			 uint8_t Attr=0;
			 uint16_t clst=0; 
			 uint32_t Filesize=0; 
			 uint32_t nclust =0;
	
			if(datacmp(FAT1ADDR,FAT2ADDR)== pdTRUE )
			{
				return 0;
			}
			else
			{
					fmRead(FAT1ADDR,1,&Fbuf[0]);
					fmRead(FAT2ADDR,1,&Fbuf[1]);
				
				if( (Fbuf[0]!=0xF0) && (Fbuf[1]==0xF0) )
				{
					// copy FAT2 to FAT1
					fatcopy(FAT2ADDR,FAT1ADDR);
					
				}
				else if((Fbuf[0]==0xF0)||(Fbuf[1]!=0xF0))
				{
					// copy FAT1 to FAT2
					 fatcopy(FAT1ADDR,FAT2ADDR);
				}
				else if((Fbuf[0]!=0xF0)||(Fbuf[1]!=0xF0))
				{
					// Dead
						return -1;
				}
																			
			}	
}	


/**
 * @brief  odm SW public API for QSPI Flash
 *
 */
int fmRead(uint32_t addr,uint32_t Blen, uint8_t* data)
{
	int ret;
	ret = QSPI_Read_IT(&hqspi,addr,Blen,data);
	return ret;
}
int fmProg(uint32_t addr,uint32_t Blen, uint8_t* data)
{
	int ret;
	ret = QSPI_Write_IT(&hqspi,addr,Blen,data);
	return ret;
}

int fmBkErase(uint32_t addr)
{
	int ret;
	ret = QSPI_BlockErase_IT(&hqspi,addr);
	return ret;
}
int fmSectErase(uint32_t addr)
{
	int ret;
	ret = QSPI_SectorErase_IT(&hqspi,addr);
	return ret;
}
int fmQuadEnable(void)
{
	int ret;
	ret = OSPI_QuadEnable(&hqspi);
	return ret;
}
int fmRDID(uint32_t* data)
{
	int ret;
	ret = QSPI_RDID(&hqspi,data);
	return ret;
}



/**
 * @brief   QSPI Flash test function
 *
 */

void FlashTestApp(char *arg)
{
			int ret;
			int ok=0;
			uint32_t addr=0;
			//unsigned char  *txBuff = pvPortMalloc(256 * sizeof(unsigned char ));
			//unsigned char  *rxBuff = pvPortMalloc(256 * sizeof(unsigned char ));
			unsigned char txBuff[256];
			unsigned char rxBuff[256];
			for (;;)
			{
					//osDelay(10);			
					if((addr&0x00000fff) == 0)
					{	
						ret=fmSectErase(addr);
						if(ret<0)
						{
							gLog(_INFO_LOG_|_FM_LOG_, "Erase Fail\r\n");
							while(1);
						}
					}
				
					for(int i=0 ; i<256 ;i++)
					{
						rxBuff[i]=0;
						txBuff[i]=i+addr;
					}
					ret=fmProg(addr,256,txBuff);
					if(ret!=0)
					{
						gLog(_INFO_LOG_|_FM_LOG_, "Program Fail\r\n");
						while(1);
					}
				
					ret=fmRead(addr,256,rxBuff);
					if(ret<0)
					{
						gLog(_INFO_LOG_|_FM_LOG_, "Read Fail\r\n");
						while(1);
					}
			
					ok=0;
					for(int i=0 ; i<256 ;i++)
					{
						 if (txBuff[i]!= rxBuff[i])
						 { 							 
							 ok=1;
							 break;
						 }
					}		
					if(ok)
					{
						gLog(_INFO_LOG_|_FM_LOG_, "compare different %d\n",addr);		
						while(1);
					}
					else 
						gLog(_INFO_LOG_|_FM_LOG_, "compare the same %d\n",addr);
							
					 addr+=256;		
					 if(addr >= QSPI_END_ADDR)
					 {						 
						addr=0;
						 gLog(_INFO_LOG_|_FM_LOG_, "Final all flash verify \n\n\n");
					 } 
			}
}	

/**
 * @brief   FatFs test function
 *
 */
int recursive(int a)
{
	if (a<2)
		return 0;
	else
	return(recursive(--a)+1);
}


#define COPYIMAGE 0
void fmSampCode(char *arg)
{

	UINT byteswritten, bytesread;                     /* File write/read counts */
	uint8_t wtext[] = "This is PEGA working with FatFs\0"; /* File write buffer */
	char sOK[] = "ok\n";
	FIL *pf = &file; 
	FILINFO fileinfo;
	FRESULT res;    
	char *fn;
	char rtext[50];
#if COPYIMAGE
	char *pImage;
	uint32_t iLen;
	unsigned char *imgbuf;

	pImage = (char *)0x08000000;
	iLen =1024*1024; //*(uint32_t *)0x08007014;
	imgbuf = pvPortMalloc(256);
#endif
	/* 
		Probe driver to USER_Path "0:/"
		this function move to odmEntry()
	*/  
	//FATFS_LinkDriver(&USER_Driver, USER_Path);
#if 1	
	// create new file system and mount on USER_Path "0:/"
	
	
	int a=recursive(10);
	uartPrintf("recursive:%d\n",a);
	uartPrintf("mount FATFS...");
	if(f_mount(&SPIFatFs, USER_Path, 0) != FR_OK)
	{
		uartPrintf("mount Error\n");
		goto err1;
	}
	uartPrintf(sOK);

	uartPrintf("mkfs...");
	if(f_mkfs(USER_Path, 1, 4096) != FR_OK)
	{
		/* FatFs Format Error */
		uartPrintf("Format Error\n");
		goto err1;
	}
	uartPrintf(sOK);
#endif
	uartPrintf("mkdir...");
	if(f_mkdir("0:/PEGA")!=FR_OK)
	{		
		uartPrintf("Create folder Error\n");
		goto err1;
	}    
	if(f_mkdir("0:/Netgear")!=FR_OK)
	{		
		uartPrintf("Create folder Error\n");
		goto err1;
	}
	uartPrintf(sOK);
	
	/*limit test*/
	
//	char str[10];
//	int num = 0;
//	
//	for(num=0;num<128;num++)
//	{
//		sprintf(str, "0:/webb%d",num);
//		
//		if(f_mkdir(str)!=FR_OK)
//		{		
//			uartPrintf("Create folder Error: %d\n",num);
//			goto err1;
//		}
//		uartPrintf(sOK);
//	}
//	

	uartPrintf("open file...");
	if(f_open(pf, "0:/PEGA/text1.txt", FA_OPEN_ALWAYS | FA_WRITE ) != FR_OK)
	{
		uartPrintf("Open file Error\n");
		goto err1;
	}	
	uartPrintf(sOK);

	uartPrintf("write file...");
	if(f_write(pf, wtext, sizeof(wtext), (UINT *)&byteswritten)!= FR_OK)
	{
		uartPrintf("write data Error\n");
		goto err1;	
	}
	else
	{
		uartPrintf("Write data %d Byte success\n",byteswritten);
	}	
			
	uartPrintf("close file...");
	if(f_close(pf)!= FR_OK)
	{	
		uartPrintf("close file Error\n");
		goto err1;	
	}
	uartPrintf(sOK);
	
	uartPrintf("open file...");
	if(f_open(pf, "0:/Netgear/text1.txt", FA_OPEN_ALWAYS | FA_WRITE ) != FR_OK)
	{
		uartPrintf("Open file Error\n");
		goto err1;
	}	
	uartPrintf(sOK);

	uartPrintf("write file...");
	if(f_write(pf, wtext, sizeof(wtext), (UINT *)&byteswritten)!= FR_OK)
	{
		uartPrintf("write data Error\n");
		goto err1;	
	}
	else
	{
		uartPrintf("Write data %d Byte success\n",byteswritten);
	}	
			
	uartPrintf("close file...");
	if(f_close(pf)!= FR_OK)
	{	
		uartPrintf("close file Error\n");
		goto err1;	
	}
	uartPrintf(sOK);
	
	
#if COPYIMAGE
	uartPrintf("Create MCU Image...");
	if(f_open(pf, "0:/MCU.bin", FA_OPEN_ALWAYS | FA_WRITE ) != FR_OK)
	{
		uartPrintf("Open file Error\n");
		goto err1;
	}	
	uartPrintf(sOK);

	uartPrintf("Copy Image to file...");
	if(f_write(pf, pImage, iLen, (UINT *)&byteswritten)!= FR_OK)
	{
		uartPrintf("write %d data Error\n", byteswritten);
		goto err1;	
	}
	else
	{
		uartPrintf("Write data %d Byte success\n",byteswritten);
	}

			
	uartPrintf("close file...");
	if(f_close(pf)!= FR_OK)
	{	
		uartPrintf("close file Error\n");
		goto err1;	
	}
	uartPrintf(sOK);
#endif
	/*************** folder list *****************/
	uartPrintf("open dir...");
	if(f_opendir(&dir,"0:/")!=FR_OK)
	{		
		uartPrintf("oepn directory Error\n");
		goto err1;	
	}
	uartPrintf(sOK);

	uartPrintf("List directory name:\n");
	while(1)
	{	
		res = f_readdir(&dir, &fileinfo); 
		
		if (res != FR_OK || fileinfo.fname[0] == 0)
		{
			break;	
		}
		else
		{
			fn = fileinfo.fname; 				
		}
		if(fileinfo.fattrib & AM_DIR)
			uartPrintf("[%s]\n",fn);
		else
			uartPrintf("%s\n",fn);
		osDelay(10);
	}		
	f_closedir(&dir);
				
	/*************** printf file data *****************/
	if(f_open(pf, "0:/PEGA/text1.txt", FA_READ ) != FR_OK)
	{
		uartPrintf("Open readonly file Error\n");
		goto err1;
	}	

	uartPrintf("check file: ");
	res = f_read(pf, rtext, sizeof(rtext),(UINT *)&bytesread);			        
	if((bytesread == 0) || (res != FR_OK))
	{		
		uartPrintf("read file Error\n");
		goto err1;
	}
	fn= rtext;
	uartPrintf("%s",fn);
	f_close(pf);
#if COPYIMAGE
	if(f_open(pf, "0:/MCU.bin", FA_READ ) != FR_OK)
	{
		uartPrintf("Open readonly file Error\n");
		goto err1;
	}	

	uartPrintf("\ncheck Image: ");
	while(iLen) {
		res = f_read(pf, imgbuf, 256,(UINT *)&bytesread);
		if((bytesread == 0) || (res != FR_OK))
		{
			uartPrintf("read file Error\n");
			goto err1;
		}
		if(0 != memcmp(imgbuf, pImage, bytesread)) {
			uartPrintf("check addres 0x%08X\n", pImage);
			goto err1;
		}
		pImage += bytesread;
		iLen -= bytesread;
		
	}
	uartPrintf(sOK);
	f_close(pf);
#endif
	uartPrintf("\nDelete file/directory...");
	res = f_unlink("0:/PEGA/text1.txt");
//	res += f_unlink("0:/PEGA");
//	res = f_unlink("0:/Netgear");
#if COPYIMAGE
	res += f_unlink("0:/MCU.bin");
#endif
	if(res != FR_OK)
	{
		uartPrintf("delete file Error\n");
		goto err1;
	}
	uartPrintf(sOK);
	
	
	
		/*limit test*/
	
	char str[10];
	int num = 0;
	
	for(num=0;num<128;num++)
	{
		sprintf(str, "0:/PEGA/webb%d",num);
		
		if(f_mkdir(str)!=FR_OK)
		{		
			uartPrintf("Create folder Error: %d\n",num);
			goto err1;
		}
		uartPrintf(sOK);
	}
	
	
//	uartPrintf("open file...");
//	if(f_open(pf, "0:/PEGA/text2.txt", FA_OPEN_ALWAYS | FA_WRITE ) != FR_OK)
//	{
//		uartPrintf("Open file Error\n");
//		goto err1;
//	}	
//	uartPrintf(sOK);

//	uartPrintf("write file...");
//	if(f_write(pf, wtext, sizeof(wtext), (UINT *)&byteswritten)!= FR_OK)
//	{
//		uartPrintf("write data Error\n");
//		goto err1;	
//	}
//	else
//	{
//		uartPrintf("Write data %d Byte success\n",byteswritten);
//	}	
//			
//	uartPrintf("close file...");
//	if(f_close(pf)!= FR_OK)
//	{	
//		uartPrintf("close file Error\n");
//		goto err1;	
//	}
//	uartPrintf(sOK);
	
	
	
	
#if 0
	f_mount(0, NULL, 0);
#endif	
err1:
#if COPYIMAGE
	vPortFree(imgbuf);
#endif
	return;
}




