/*
** =============================================================================
** Copyright (c) 2016  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or modify it under
** the terms of the GNU General Public License as published by the Free Software 
** Foundation; version 2.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
** FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License along with
** this program; if not, write to the Free Software Foundation, Inc., 51 Franklin
** Street, Fifth Floor, Boston, MA 02110-1301, USA.
**
** File:
**     tas2557-core.c
**
** Description:
**     TAS2557 common functions for Android Linux
**
** =============================================================================
*/

#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>

#include "tas2557.h"
#include "tas2557-core.h"

#define TAS2557_CAL_NAME    "/data/tas2557_cal.bin"

//set default PLL CLKIN to GPI2 (MCLK) = 0x00
#define TAS2557_DEFAULT_PLL_CLKIN 0x00

static void tas2557_load_calibration(struct tas2557_priv *pTAS2557,
	char *pFileName);
static void tas2557_load_data(struct tas2557_priv *pTAS2557, TData * pData,
	unsigned int nType);
static void tas2557_load_block(struct tas2557_priv *pTAS2557, TBlock * pBlock);
static void tas2557_load_configuration(struct tas2557_priv *pTAS2557,
	unsigned int nConfiguration, bool bLoadSame);
	
#define TAS2557_UDELAY 0xFFFFFFFE
#define TAS2557_MDELAY 0xFFFFFFFD

#define FW_ERR_HEADER -1
#define FW_ERR_SIZE -2

#define TAS2557_BLOCK_PLL				0x00
#define TAS2557_BLOCK_PGM_ALL			0x0d
#define TAS2557_BLOCK_PGM_DEV_A			0x01
#define TAS2557_BLOCK_PGM_DEV_B			0x08
#define TAS2557_BLOCK_CFG_COEFF_DEV_A	0x03
#define TAS2557_BLOCK_CFG_COEFF_DEV_B	0x0a
#define TAS2557_BLOCK_CFG_PRE_DEV_A		0x04
#define TAS2557_BLOCK_CFG_PRE_DEV_B		0x0b
#define TAS2557_BLOCK_CFG_POST			0x05
#define TAS2557_BLOCK_CFG_POST_POWER	0x06
#define TAS2557_BLOCK_CFG_CAL_A			0x10
#define TAS2557_BLOCK_CFG_CAL_B			0x20

static unsigned int p_tas2557_dboost_data[] = {
	TAS2557_DBOOST_CTL_REG, 0x08,	//enable, 0x0c=disable
	TAS2557_DBOOST_CFG_REG, 0x03,	//threshold -18dB +hysteresis 4dB
	0xFFFFFFFF, 0xFFFFFFFF
};

//This is only needed for PG1.0
static unsigned int p_tas2557_vpred_comp_data[] =
{
//Reverse attenuation compensation for Vpred 0.5dB
	TAS2557_VPRED_COMP_REG, 0x04, 0x43, 0xca, 0xd0, 0x22,
	0xFFFFFFFF, 0xFFFFFFFF
};

//This is only needed for PG1.0.
static unsigned int p_tas2557_thermal_foldback_data[] = {
	TAS2557_THERMAL_FOLDBACK_REG, 0x04, 0x48, 0x00, 0x00, 0x00,	//disable
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2557_boost_8Ohm_data[] =
{
	TAS2557_BOOST_HEADROOM, 0x04, 0x04, 0xcc, 0xcc, 0xcc,	// boost headroom 600mv
    TAS2557_BOOSTOFF_EFFICIENCY, 0x04, 0x67, 0xae,0x14,0x7a, //boost off efficiency 0.81
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2557_boost_6Ohm_data[] =
{
	TAS2557_BOOST_HEADROOM, 0x04, 0x06, 0x66, 0x66, 0x66, // boost headroom 800mv
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2557_boost_4Ohm_data[] =
{
	TAS2557_BOOST_HEADROOM, 0x04, 0x06, 0x66, 0x66, 0x66, // boost headroom 800mv
	TAS2557_BOOSTON_EFFICIENCY, 0x04, 0x73, 0x33, 0x33, 0x33,
	TAS2557_BOOSTOFF_EFFICIENCY, 0x04, 0x60, 0x00,0x00,0x00, //boost off efficiency 0.75
	0xFFFFFFFF, 0xFFFFFFFF
};

/* This is only required for PG2.0*/
static unsigned int p_tas2557_isense_threshold_data[] =
{
	TAS2557_ISENSE_THRESHOLD, 0x04, 0, 0, 0, 0,	// Make Isense threshold zero
	0xFFFFFFFF, 0xFFFFFFFF
};

#define TAS2557_STARTUP_DATA_PLL_CLKIN_INDEX 3
static unsigned int p_tas2557_startup_data[] = {
	TAS2557_CLK_ERR_CTRL, 0x03,	//enable clock error detection
	TAS2557_PLL_CLKIN_REG, TAS2557_DEFAULT_PLL_CLKIN,
	TAS2557_POWER_CTRL2_REG, 0xA0,	//Class-D, Boost power up
	TAS2557_POWER_CTRL2_REG, 0xA3,	//Class-D, Boost, IV sense power up
	TAS2557_POWER_CTRL1_REG, 0xF8,	//PLL, DSP, clock dividers power up
	TAS2557_UDELAY, 2000,		//delay
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2557_unmute_data[] = {
	TAS2557_MUTE_REG, 0x00,		//unmute
	TAS2557_SOFT_MUTE_REG, 0x00,	//soft unmute
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2557_shutdown_data[] = {
	TAS2557_SOFT_MUTE_REG, 0x01,	//soft mute
	TAS2557_UDELAY, 10000,		//delay 10ms
	TAS2557_MUTE_REG, 0x03,		//mute
	TAS2557_POWER_CTRL1_REG, 0x60,	//DSP power down
	TAS2557_UDELAY, 2000,		//delay 2ms
	TAS2557_POWER_CTRL2_REG, 0x00,	//Class-D, Boost power down
	TAS2557_POWER_CTRL1_REG, 0x00,	//all power down
	0xFFFFFFFF, 0xFFFFFFFF
};

#if 0
static unsigned int p_tas2557_shutdown_clk_err[] = {
	TAS2557_CLK_ERR_CTRL, 0x09,	//enable clock error detection on PLL
	0xFFFFFFFF, 0xFFFFFFFF
};
#endif

static unsigned int p_tas2557_mute_DSP_down_data[] = {
	TAS2557_MUTE_REG, 0x03,		//mute
	TAS2557_POWER_CTRL1_REG, 0x60,	//DSP power down
	TAS2557_UDELAY, 0xFF,		//delay
	0xFFFFFFFF, 0xFFFFFFFF
};

static int tas2557_dev_load_data(struct tas2557_priv *pTAS2557,
	unsigned int *pData)
{
	int ret = 0;
	unsigned int n = 0;
	unsigned int nRegister;
	unsigned int nData;

	do {
		nRegister = pData[n * 2];
		nData = pData[n * 2 + 1];
		if (nRegister == TAS2557_UDELAY)
			udelay(nData);
		else if (nRegister != 0xFFFFFFFF){
			ret = pTAS2557->write(pTAS2557, nRegister, nData);
			if(ret < 0) {
				dev_err(pTAS2557->dev, "Reg Write err %d\n", ret);
				break;
			}
		}
		n++;
	} while (nRegister != 0xFFFFFFFF);
	
	return ret;
}

static int tas2557_dev_load_blk_data(struct tas2557_priv *pTAS2557,
				  unsigned int *pData) {
	unsigned int nRegister;
	unsigned int *nData;
	unsigned char Buf[128];
	unsigned int nLength = 0;
	unsigned int nLoop = 0;
	unsigned int i =0;
	unsigned int nSize = 0;
	int ret = 0;

	do{
		nRegister = pData[nLength];
		nSize = pData[nLength + 1];
		nData = &pData[nLength + 2];
		if (nRegister == TAS2557_MDELAY){
			mdelay(nData[0]);
		}
		else{
			if (nRegister != 0xFFFFFFFF){
				if(nSize > 128){ 
					dev_err(pTAS2557->dev, 
						"%s, Line=%d, invalid size, maximum is 128 bytes!\n", 
						__FUNCTION__, __LINE__);
					break;
				}
				
				if(nSize > 1){
					for(i = 0; i < nSize; i++) Buf[i] = (unsigned char)nData[i];
					ret = pTAS2557->bulk_write(pTAS2557, nRegister, Buf, nSize);
				}else if(nSize == 1){
					ret = pTAS2557->write(pTAS2557,nRegister, nData[0]);
				}else{
					dev_err(pTAS2557->dev, 
						"%s, Line=%d,invalid size, minimum is 1 bytes!\n", 
						__FUNCTION__, __LINE__);
				}
				
				if(ret < 0) break;
			}
		}
		nLength = nLength + 2 + pData[nLength+1] ;
	} while (nRegister != 0xFFFFFFFF);
	
	return ret;
}

int tas2557_setLoad(struct tas2557_priv *pTAS2557, int load)
{
	int ret = 0;

	ret = pTAS2557->write(pTAS2557, 
			TAS2557_SNS_CTRL_REG, (load&0x03)<<1);
	if(ret<0)
		goto err;
	
	switch(load)
	{
		case 0:	//8Ohm
		ret = tas2557_dev_load_blk_data(pTAS2557, 
			p_tas2557_boost_8Ohm_data);
		break;
		
		case 1:	//6Ohm
		ret = tas2557_dev_load_blk_data(pTAS2557, 
			p_tas2557_boost_6Ohm_data);
		break;
		
		case 2:	//4Ohm
		ret = tas2557_dev_load_blk_data(pTAS2557, 
			p_tas2557_boost_4Ohm_data);
		break;
	}
	
err:	
	
	return ret;
}

int tas2557_load_platdata(struct tas2557_priv *pTAS2557)
{
	int ret = 0;
	
	ret = tas2557_setLoad(pTAS2557, pTAS2557->mnLoad);
		
	return ret;
}

int tas2557_load_default(struct tas2557_priv *pTAS2557)
{
	int ret = 0;
	
	ret = tas2557_dev_load_blk_data(pTAS2557, p_tas2557_dboost_data);
	if(ret < 0) goto err;
	
	/* This is not required for PG1.0 and 2.1, only PG2.0*/
	if(pTAS2557->mnPGID 
		== TAS2557_PG_VERSION_2P0)
		ret = tas2557_dev_load_blk_data(pTAS2557, 
				p_tas2557_isense_threshold_data);
				
	if(pTAS2557->mnPGID 
		== TAS2557_PG_VERSION_1P0){
		tas2557_dev_load_blk_data(pTAS2557, p_tas2557_vpred_comp_data);
		tas2557_dev_load_blk_data(pTAS2557, p_tas2557_thermal_foldback_data);
	}
	
	tas2557_load_platdata(pTAS2557);
	
err:
	
	return ret;
}

void tas2557_enable(struct tas2557_priv *pTAS2557, bool bEnable)
{
	dev_dbg(pTAS2557->dev, "Enable: %d\n", bEnable);
	if (bEnable) {
		if (!pTAS2557->mbPowerUp) {
			dev_dbg(pTAS2557->dev, "Enable: load startup sequence\n");
			tas2557_dev_load_data(pTAS2557, p_tas2557_startup_data);
			dev_dbg(pTAS2557->dev, "Enable: load unmute sequence\n");
			tas2557_dev_load_data(pTAS2557, p_tas2557_unmute_data);
			pTAS2557->mbPowerUp = true;
		}
	} else {
		if (pTAS2557->mbPowerUp) {
			dev_dbg(pTAS2557->dev, "Enable: load shutdown sequence\n");
			tas2557_dev_load_data(pTAS2557, p_tas2557_shutdown_data);
			//tas2557_dev_load_data(pTAS2557, p_tas2557_shutdown_clk_err);
			pTAS2557->mbPowerUp = false;
		}
	}
}

int tas2557_set_sampling_rate(struct tas2557_priv *pTAS2557, unsigned int nSamplingRate)
{
	TConfiguration *pConfiguration;
	unsigned int nConfiguration;

	dev_dbg(pTAS2557->dev, "tas2557_setup_clocks: nSamplingRate = %d [Hz]\n",
		nSamplingRate);

	if ((!pTAS2557->mpFirmware->mpPrograms) ||
		(!pTAS2557->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2557->dev, "Firmware not loaded\n");
		return -EINVAL;
	}

	pConfiguration = &(pTAS2557->mpFirmware->mpConfigurations[pTAS2557->mnCurrentConfiguration]);
	if (pConfiguration->mnSamplingRate == nSamplingRate) {
		dev_info(pTAS2557->dev, "Sampling rate for current configuration matches: %d\n",
			nSamplingRate);
		return 0;
	}

	for (nConfiguration = 0;
		nConfiguration < pTAS2557->mpFirmware->mnConfigurations;
		nConfiguration++) {
		pConfiguration =
			&(pTAS2557->mpFirmware->mpConfigurations[nConfiguration]);
		if ((pConfiguration->mnSamplingRate == nSamplingRate)
			&&(pConfiguration->mnProgram == pTAS2557->mnCurrentProgram)){
			dev_info(pTAS2557->dev,
				"Found configuration: %s, with compatible sampling rate %d\n",
				pConfiguration->mpName, nSamplingRate);
			tas2557_load_configuration(pTAS2557, nConfiguration, false);
			return 0;
		}
	}

	dev_err(pTAS2557->dev, "Cannot find a configuration that supports sampling rate: %d\n",
		nSamplingRate);

	return -EINVAL;
}

static void fw_print_header(struct tas2557_priv *pTAS2557, TFirmware * pFirmware)
{
	dev_info(pTAS2557->dev, "FW Size        = %d", pFirmware->mnFWSize);
	dev_info(pTAS2557->dev, "Checksum       = 0x%04X", pFirmware->mnChecksum);
	dev_info(pTAS2557->dev, "PPC Version    = 0x%04X", pFirmware->mnPPCVersion);
	dev_info(pTAS2557->dev, "FW  Version    = 0x%04X", pFirmware->mnFWVersion);
	dev_info(pTAS2557->dev, "Driver Version = 0x%04X", pFirmware->mnDriverVersion);
	dev_info(pTAS2557->dev, "Timestamp      = %d", pFirmware->mnTimeStamp);
	dev_info(pTAS2557->dev, "DDC Name       = %s", pFirmware->mpDDCName);
	dev_info(pTAS2557->dev, "Description    = %s", pFirmware->mpDescription);
}

inline unsigned int fw_convert_number(unsigned char *pData)
{
	return pData[3] + (pData[2] << 8) + (pData[1] << 16) + (pData[0] << 24);
}

static int fw_parse_header(struct tas2557_priv *pTAS2557, 
	TFirmware * pFirmware, unsigned char *pData,
	unsigned int nSize)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned char pMagicNumber[] = { 0x35, 0x35, 0x35, 0x32 };
	if (nSize < 102) {
		dev_err(pTAS2557->dev, "Firmware: Header too short");
		return -1;
	}

	if (memcmp(pData, pMagicNumber, 4)) {
		dev_err(pTAS2557->dev, "Firmware: Magic number doesn't match");
		return -1;
	}

	pData += 4;

	pFirmware->mnFWSize = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnChecksum = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnPPCVersion = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnFWVersion = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnDriverVersion = fw_convert_number(pData);
	pData += 4;		
			
	pFirmware->mnTimeStamp = fw_convert_number(pData);
	pData += 4;

	memcpy(pFirmware->mpDDCName, pData, 64);
	pData += 64;

	n = strlen(pData);
	pFirmware->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
	pData += n + 1;

	if ((pData - pDataStart) >= nSize) {
		dev_err(pTAS2557->dev, "Firmware: Header too short after DDC description");
		return -1;
	}

	pFirmware->mnDeviceFamily = fw_convert_number(pData);
	pData += 4;

	if(pFirmware->mnDeviceFamily != 0){
		dev_err(pTAS2557->dev, 
			"deviceFamily %d, not TAS device", pFirmware->mnDeviceFamily);
		return -1;
	}
	
	pFirmware->mnDevice = fw_convert_number(pData);
	pData += 4;

	if(pFirmware->mnDevice != 2){
		dev_err(pTAS2557->dev, 
			"device %d, not TAS2557", pFirmware->mnDevice);
		return -1;
	}
	
	fw_print_header(pTAS2557, pFirmware);

	return pData - pDataStart;
}

static int fw_parse_block_data(TBlock * pBlock, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;

	pBlock->mnType = fw_convert_number(pData);
	pData += 4;

	pBlock->mnCommands = fw_convert_number(pData);
	pData += 4;

	n = pBlock->mnCommands * 4;
	pBlock->mpData = kmemdup(pData, n, GFP_KERNEL);
	pData += n;

	return pData - pDataStart;
}

static int fw_parse_data(TData * pImageData, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int nBlock;
	unsigned int n;

	memcpy(pImageData->mpName, pData, 64);
	pData += 64;

	n = strlen(pData);
	pImageData->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
	pData += n + 1;

	pImageData->mnBlocks = (pData[0] << 8) + pData[1];
	pData += 2;

	pImageData->mpBlocks =
		kmalloc(sizeof(TBlock) * pImageData->mnBlocks, GFP_KERNEL);

	for (nBlock = 0; nBlock < pImageData->mnBlocks; nBlock++) {
		n = fw_parse_block_data(&(pImageData->mpBlocks[nBlock]), pData);
		pData += n;
	}

	return pData - pDataStart;
}

static int fw_parse_pll_data(TFirmware * pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nPLL;
	TPLL *pPLL;

	pFirmware->mnPLLs = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpPLLs = kmalloc(sizeof(TPLL) * pFirmware->mnPLLs, GFP_KERNEL);
	for (nPLL = 0; nPLL < pFirmware->mnPLLs; nPLL++) {
		pPLL = &(pFirmware->mpPLLs[nPLL]);

		memcpy(pPLL->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pPLL->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		n = fw_parse_block_data(&(pPLL->mBlock), pData);
		pData += n;
	}

	return pData - pDataStart;
}

static int fw_parse_program_data(TFirmware * pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nProgram;
	TProgram *pProgram;

	pFirmware->mnPrograms = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpPrograms =
		kmalloc(sizeof(TProgram) * pFirmware->mnPrograms, GFP_KERNEL);
	for (nProgram = 0; nProgram < pFirmware->mnPrograms; nProgram++) {
		pProgram = &(pFirmware->mpPrograms[nProgram]);
		memcpy(pProgram->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pProgram->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		pProgram->mnAppMode = pData[0];
		pData++;
		
		pProgram->mnBoost = (pData[0] << 8) + pData[1];
		pData += 2;
		
		n = fw_parse_data(&(pProgram->mData), pData);
		pData += n;
	}

	return pData - pDataStart;
}

static int fw_parse_configuration_data(TFirmware * pFirmware,
	unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nConfiguration;
	TConfiguration *pConfiguration;

	pFirmware->mnConfigurations = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpConfigurations =
		kmalloc(sizeof(TConfiguration) * pFirmware->mnConfigurations,
		GFP_KERNEL);
	for (nConfiguration = 0; nConfiguration < pFirmware->mnConfigurations;
		nConfiguration++) {
		pConfiguration = &(pFirmware->mpConfigurations[nConfiguration]);
		memcpy(pConfiguration->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pConfiguration->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		pConfiguration->mnProgram = pData[0];
		pData++;

		pConfiguration->mnPLL = pData[0];
		pData++;

		pConfiguration->mnSamplingRate = fw_convert_number(pData);
		pData += 4;

		n = fw_parse_data(&(pConfiguration->mData), pData);
		pData += n;
	}

	return pData - pDataStart;
}

int fw_parse_calibration_data(TFirmware * pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nCalibration;
	TCalibration *pCalibration;

	pFirmware->mnCalibrations = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpCalibrations =
		kmalloc(sizeof(TCalibration) * pFirmware->mnCalibrations, GFP_KERNEL);
	for (nCalibration = 0;
		nCalibration < pFirmware->mnCalibrations;
		nCalibration++) {
		pCalibration = &(pFirmware->mpCalibrations[nCalibration]);
		memcpy(pCalibration->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pCalibration->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		pCalibration->mnProgram = pData[0];
		pData++;

		pCalibration->mnConfiguration = pData[0];
		pData++;

		n = fw_parse_block_data(&(pCalibration->mBlock), pData);
		pData += n;
	}

	return pData - pDataStart;
}

static int fw_parse(struct tas2557_priv *pTAS2557,
	TFirmware * pFirmware,
	unsigned char *pData,
	unsigned int nSize)
{
	int nPosition = 0;

	nPosition = fw_parse_header(pTAS2557, pFirmware, pData, nSize);
	if (nPosition < 0) {
		dev_err(pTAS2557->dev, "Firmware: Wrong Header");
		return FW_ERR_HEADER;
	}

	if (nPosition >= nSize) {
		dev_err(pTAS2557->dev, "Firmware: Too short");
		return FW_ERR_SIZE;
	}

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_pll_data(pFirmware, pData);

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_program_data(pFirmware, pData);

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_configuration_data(pFirmware, pData);

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	if (nSize > 64)
		nPosition = fw_parse_calibration_data(pFirmware, pData);

	return 0;
}

static void tas2557_load_block(struct tas2557_priv *pTAS2557, TBlock * pBlock)
{
	unsigned int nCommand = 0;
	unsigned char nBook;
	unsigned char nPage;
	unsigned char nOffset;
	unsigned char nData;
	unsigned int nLength;
	unsigned char *pData = pBlock->mpData;

	dev_dbg(pTAS2557->dev, "TAS2557 load block: Type = %d, commands = %d\n",
		pBlock->mnType, pBlock->mnCommands);
	while (nCommand < pBlock->mnCommands) {
		pData = pBlock->mpData + nCommand * 4;

		nBook = pData[0];
		nPage = pData[1];
		nOffset = pData[2];
		nData = pData[3];

		nCommand++;

		if (nOffset <= 0x7F){
			pTAS2557->write(pTAS2557, TAS2557_REG(nBook, nPage, nOffset),
				nData);
		}else if (nOffset == 0x81) {
			unsigned int nSleep = (nBook << 8) + nPage;
			msleep(nSleep);
		}else if (nOffset == 0x85) {
			pData += 4;
			nLength = (nBook << 8) + nPage;
			nBook = pData[0];
			nPage = pData[1];
			nOffset = pData[2];
			if (nLength > 1)
				pTAS2557->bulk_write(pTAS2557, TAS2557_REG(nBook, nPage,
						nOffset), pData + 3, nLength);
			else
				pTAS2557->write(pTAS2557, TAS2557_REG(nBook, nPage, nOffset),
					pData[3]);

			nCommand++;
			if (nLength >= 2)
				nCommand += ((nLength - 2) / 4) + 1;
		}
	}
}

static void tas2557_load_data(struct tas2557_priv *pTAS2557, TData * pData,
	unsigned int nType)
{
	unsigned int nBlock;
	TBlock *pBlock;

	dev_dbg(pTAS2557->dev,
		"TAS2557 load data: %s, Blocks = %d, Block Type = %d\n", pData->mpName,
		pData->mnBlocks, nType);

	for (nBlock = 0; nBlock < pData->mnBlocks; nBlock++) {
		pBlock = &(pData->mpBlocks[nBlock]);
		if (pBlock->mnType == nType)
			tas2557_load_block(pTAS2557, pBlock);
	}
}

static void tas2557_load_configuration(struct tas2557_priv *pTAS2557,
	unsigned int nConfiguration, bool bLoadSame)
{
	TConfiguration *pCurrentConfiguration;
	TConfiguration *pNewConfiguration;
	TPLL *pNewPLL;

	dev_dbg(pTAS2557->dev, "tas2557_load_configuration: %d\n", nConfiguration);

	if ((!pTAS2557->mpFirmware->mpPrograms) ||
		(!pTAS2557->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2557->dev, "Firmware not loaded\n");
		return;
	}

	if (nConfiguration >= pTAS2557->mpFirmware->mnConfigurations) {
		dev_err(pTAS2557->dev, "Configuration %d doesn't exist\n",
			nConfiguration);
		return;
	}

	if ((nConfiguration == pTAS2557->mnCurrentConfiguration) && (!bLoadSame)) {
		dev_info(pTAS2557->dev, "Configuration %d is already loaded\n",
			nConfiguration);
		return;
	}

	pCurrentConfiguration =
		&(pTAS2557->mpFirmware->mpConfigurations[pTAS2557->mnCurrentConfiguration]);
	pNewConfiguration =
		&(pTAS2557->mpFirmware->mpConfigurations[nConfiguration]);

	if (pNewConfiguration->mnProgram != pCurrentConfiguration->mnProgram) {
		dev_err(pTAS2557->dev,
			"Configuration %d, %s doesn't share the same program as current %d\n",
			nConfiguration, pNewConfiguration->mpName, pCurrentConfiguration->mnProgram);
		return;
	}

	if (pNewConfiguration->mnPLL >= pTAS2557->mpFirmware->mnPLLs) {
		dev_err(pTAS2557->dev,
			"Configuration %d, %s doesn't have a valid PLL index %d\n",
			nConfiguration, pNewConfiguration->mpName, pNewConfiguration->mnPLL);
		return;
	}
	
	pNewPLL = &(pTAS2557->mpFirmware->mpPLLs[pNewConfiguration->mnPLL]);

	if (pTAS2557->mbPowerUp) {
		if (pNewConfiguration->mnPLL != pCurrentConfiguration->mnPLL) {
			dev_dbg(pTAS2557->dev,
				"TAS2557 is powered up -> mute and power down DSP before loading new configuration\n");
			//tas2557_dev_load_data(pTAS2557, p_tas2557_mute_DSP_down_data);
			tas2557_dev_load_data(pTAS2557, p_tas2557_shutdown_data);
			dev_dbg(pTAS2557->dev, "TAS2557: load new PLL: %s, block data\n",
				pNewPLL->mpName);
			tas2557_load_block(pTAS2557, &(pNewPLL->mBlock));
			pTAS2557->mnCurrentSampleRate = pNewConfiguration->mnSamplingRate;
			dev_dbg(pTAS2557->dev,
				"load new configuration: %s, pre block data\n",
				pNewConfiguration->mpName);
			tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_PRE_DEV_A);
			dev_dbg(pTAS2557->dev,
				"TAS2557: load new configuration: %s, coeff block data\n",
				pNewConfiguration->mpName);
			tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_COEFF_DEV_A);
			dev_dbg(pTAS2557->dev, "TAS2557: power up TAS2557\n");
			tas2557_dev_load_data(pTAS2557, p_tas2557_startup_data);
			dev_dbg(pTAS2557->dev, "TAS2557: unmute TAS2557\n");
			tas2557_dev_load_data(pTAS2557, p_tas2557_unmute_data);
		} else {
			dev_dbg(pTAS2557->dev,
				"TAS2557 is powered up, no change in PLL: load new configuration: %s, coeff block data\n",
				pNewConfiguration->mpName);
			tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_COEFF_DEV_A);	
		}
				
		pTAS2557->mbLoadConfigurationPostPowerUp = false;
	} else {
		dev_dbg(pTAS2557->dev,
			"TAS2557 was powered down\n");
		if (pNewConfiguration->mnPLL != pCurrentConfiguration->mnPLL) {
			dev_dbg(pTAS2557->dev, "TAS2557: load new PLL: %s, block data\n",
				pNewPLL->mpName);
			tas2557_load_block(pTAS2557, &(pNewPLL->mBlock));
			pTAS2557->mnCurrentSampleRate = pNewConfiguration->mnSamplingRate;
			dev_dbg(pTAS2557->dev,
				"load new configuration: %s, pre block data\n",
				pNewConfiguration->mpName);
			tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_PRE_DEV_A);
		}
		
		dev_dbg(pTAS2557->dev,
				"TAS2557: load new configuration: %s, coeff block data\n",
				pNewConfiguration->mpName);
		tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_COEFF_DEV_A);	
		
		pTAS2557->mbLoadConfigurationPostPowerUp = true;
	}

	pTAS2557->mnCurrentConfiguration = nConfiguration;
}

int tas2557_set_config(struct tas2557_priv *pTAS2557, int config)
{
	TConfiguration *pConfiguration;
	TProgram *pProgram;
	unsigned int nProgram = pTAS2557->mnCurrentProgram;
	unsigned int nConfiguration = config;

	if ((!pTAS2557->mpFirmware->mpPrograms) ||
		(!pTAS2557->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2557->dev, "Firmware not loaded\n");
		return -1;
	}

	if (nConfiguration >= pTAS2557->mpFirmware->mnConfigurations) {
		dev_err(pTAS2557->dev, "Configuration %d doesn't exist\n",
			nConfiguration);
		return -1;
	}

	pConfiguration = &(pTAS2557->mpFirmware->mpConfigurations[nConfiguration]);
	pProgram = &(pTAS2557->mpFirmware->mpPrograms[nProgram]);

	if (nProgram != pConfiguration->mnProgram) {
		dev_err(pTAS2557->dev,
			"Configuration %d, %s with Program %d isn't compatible with existing Program %d, %s\n",
			nConfiguration, pConfiguration->mpName, pConfiguration->mnProgram,
			nProgram, pProgram->mpName);
		return -1;
	}

	tas2557_load_configuration(pTAS2557, nConfiguration, false);

	return 0;
}

void tas2557_clear_firmware(TFirmware *pFirmware)
{
	unsigned int n, nn;
	if (!pFirmware) return;
	if (pFirmware->mpDescription) kfree(pFirmware->mpDescription);	

	for (n = 0; n < pFirmware->mnPLLs; n++)
	{
		kfree(pFirmware->mpPLLs[n].mpDescription);
		kfree(pFirmware->mpPLLs[n].mBlock.mpData);
	}
	kfree(pFirmware->mpPLLs);

	for (n = 0; n < pFirmware->mnPrograms; n++)
	{
		kfree(pFirmware->mpPrograms[n].mpDescription);
		kfree(pFirmware->mpPrograms[n].mData.mpDescription);
		for (nn = 0; nn < pFirmware->mpPrograms[n].mData.mnBlocks; nn++)
			kfree(pFirmware->mpPrograms[n].mData.mpBlocks[nn].mpData);
		kfree(pFirmware->mpPrograms[n].mData.mpBlocks);
	}
	kfree(pFirmware->mpPrograms);

	for (n = 0; n < pFirmware->mnConfigurations; n++)
	{
		kfree(pFirmware->mpConfigurations[n].mpDescription);
		kfree(pFirmware->mpConfigurations[n].mData.mpDescription);
		for (nn = 0; nn < pFirmware->mpConfigurations[n].mData.mnBlocks; nn++)
			kfree(pFirmware->mpConfigurations[n].mData.mpBlocks[nn].mpData);
		kfree(pFirmware->mpConfigurations[n].mData.mpBlocks);
	}
	kfree(pFirmware->mpConfigurations);

	for (n = 0; n < pFirmware->mnCalibrations; n++)
	{
		kfree(pFirmware->mpCalibrations[n].mpDescription);
		kfree(pFirmware->mpCalibrations[n].mBlock.mpData);
	}
	kfree(pFirmware->mpCalibrations);

	memset(pFirmware, 0x00, sizeof(TFirmware));
}

static void tas2557_load_calibration(struct tas2557_priv *pTAS2557,
	char *pFileName)
{
	int nResult;
	int nFile;
	mm_segment_t fs;
	unsigned char pBuffer[512];
	int nSize = 0;

	dev_dbg(pTAS2557->dev, "%s:\n", __func__);

	fs = get_fs();
	set_fs(KERNEL_DS);
	nFile = sys_open(pFileName, O_RDONLY, 0);

	dev_info(pTAS2557->dev, "TAS2557 calibration file = %s, handle = %d\n",
		pFileName, nFile);

	if (nFile >= 0) {
		nSize = sys_read(nFile, pBuffer, 512);
		sys_close(nFile);
	} else {
		dev_err(pTAS2557->dev, "TAS2557 cannot open calibration file: %s\n",
			pFileName);
	}

	set_fs(fs);

	if (!nSize)
		return;

	tas2557_clear_firmware(pTAS2557->mpCalFirmware);
		
	dev_info(pTAS2557->dev, "TAS2557 calibration file size = %d\n", nSize);
	nResult = fw_parse(pTAS2557, pTAS2557->mpCalFirmware, pBuffer, nSize);

	if (nResult) {
		dev_err(pTAS2557->dev, "TAS2557 calibration file is corrupt\n");
		return;
	}

	dev_info(pTAS2557->dev, "TAS2557 calibration: %d calibrations\n",
		pTAS2557->mpCalFirmware->mnCalibrations);
}

void tas2557_fw_ready(const struct firmware *pFW, void *pContext)
{
	struct tas2557_priv *pTAS2557 = (struct tas2557_priv *) pContext;
	int nResult;
	unsigned int nProgram = 0;
	unsigned int nSampleRate = 0;

	dev_info(pTAS2557->dev, "%s:\n", __func__);

	if (unlikely(!pFW) || unlikely(!pFW->data)) {
		dev_err(pTAS2557->dev, "%s firmware is not loaded.\n",
			TAS2557_FW_NAME);
		
		nResult = tas2557_load_default(pTAS2557);
		return;
	}

	if (pTAS2557->mpFirmware->mpConfigurations){
		nProgram = pTAS2557->mnCurrentProgram;
		nSampleRate = pTAS2557->mnCurrentSampleRate;
		dev_dbg(pTAS2557->dev, "clear current firmware\n");
		tas2557_clear_firmware(pTAS2557->mpFirmware);
	}	
		
	nResult = fw_parse(pTAS2557, pTAS2557->mpFirmware, 
		(unsigned char *) (pFW->data),	pFW->size);

	release_firmware(pFW);
	
	if (nResult) {
		dev_err(pTAS2557->dev, "firmware is corrupt\n");
		return;
	}

	if (!pTAS2557->mpFirmware->mnPrograms) {
		dev_err(pTAS2557->dev, "firmware contains no programs\n");
		return;
	}
	
	if (!pTAS2557->mpFirmware->mnConfigurations) {
		dev_err(pTAS2557->dev, 
			"firmware contains no configurations\n");
		return;
	}
	
	if(nProgram >= pTAS2557->mpFirmware->mnPrograms){
		dev_info(pTAS2557->dev, 
			"no previous program, set to default\n");
		nProgram = 0;
	}
		
	pTAS2557->mnCurrentSampleRate = nSampleRate;

	tas2557_set_program(pTAS2557, nProgram);
}	

int tas2557_set_program(struct tas2557_priv *pTAS2557,
	unsigned int nProgram)
{
	TPLL *pPLL;
	TConfiguration *pConfiguration;
	unsigned int nConfiguration = 0;
	unsigned int nSampleRate = 0;
	unsigned int Value = 0;
	bool bFound = false;
	int nResult = -1;

	if ((!pTAS2557->mpFirmware->mpPrograms) ||
		(!pTAS2557->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2557->dev, "Firmware not loaded\n");
		return -1;
	}
	
	if (nProgram >= pTAS2557->mpFirmware->mnPrograms) {
		dev_err(pTAS2557->dev, "TAS2557: Program %d doesn't exist\n",
			nConfiguration);
		return -1;
	}
	
	nConfiguration = 0;
	nSampleRate = pTAS2557->mnCurrentSampleRate;
	
	while (!bFound 
		&& (nConfiguration < pTAS2557->mpFirmware->mnConfigurations)) {
		if (pTAS2557->mpFirmware->mpConfigurations[nConfiguration].mnProgram 
			== nProgram){
			if(nSampleRate == 0){
				bFound = true;
				dev_info(pTAS2557->dev, "find default configuration %d\n", nConfiguration);
			}else if(nSampleRate 
				== pTAS2557->mpFirmware->mpConfigurations[nConfiguration].mnSamplingRate){
				bFound = true;
				dev_info(pTAS2557->dev, "find matching configuration %d\n", nConfiguration);
			}else{
				nConfiguration++;
			}
		}else{
			nConfiguration++;
		}
	}
	
	if (!bFound) {
		dev_err(pTAS2557->dev, 
			"Program %d, no valid configuration found for sample rate %d, ignore\n",
			nProgram, nSampleRate);
		return -1;
	}
	
	pTAS2557->mnCurrentProgram = nProgram;
	
	nResult = tas2557_dev_load_data(pTAS2557, p_tas2557_mute_DSP_down_data);
	if(nResult < 0){
		dev_err(pTAS2557->dev, 
			"device register access error\n");
		return -1;
	} 
		
	pTAS2557->write(pTAS2557, TAS2557_SW_RESET_REG, 0x01);

	udelay(1000);
	pTAS2557->mnCurrentBook = 0;
	pTAS2557->mnCurrentPage = 0;
	
	nResult = tas2557_load_default(pTAS2557);	
	
	dev_info(pTAS2557->dev, "load program %d\n", nProgram);
	tas2557_load_data(pTAS2557,
		&(pTAS2557->mpFirmware->mpPrograms[nProgram].mData),
		TAS2557_BLOCK_PGM_DEV_A);

	nResult = pTAS2557->read(pTAS2557, TAS2557_CRC_CHECKSUM_REG, &Value);
	dev_info(pTAS2557->dev, "uCDSP Checksum: 0x%02x\n", Value);
	
	pTAS2557->mnCurrentConfiguration = nConfiguration;
	pConfiguration =
		&(pTAS2557->mpFirmware->mpConfigurations[nConfiguration]);
	pPLL = &(pTAS2557->mpFirmware->mpPLLs[pConfiguration->mnPLL]);
	dev_dbg(pTAS2557->dev,
		"TAS2557 load PLL: %s block for Configuration %s\n",
		pPLL->mpName, pConfiguration->mpName);
	
	tas2557_load_block(pTAS2557, &(pPLL->mBlock));
	pTAS2557->mnCurrentSampleRate = pConfiguration->mnSamplingRate;
	dev_dbg(pTAS2557->dev,
		"load configuration %s conefficient pre block\n",
		pConfiguration->mpName);		
	tas2557_load_data(pTAS2557, &(pConfiguration->mData), TAS2557_BLOCK_CFG_PRE_DEV_A);

	nResult = pTAS2557->read(pTAS2557, TAS2557_PLL_CLKIN_REG, &Value);
	dev_info(pTAS2557->dev, "TAS2557 PLL_CLKIN = 0x%02X\n", Value);
	p_tas2557_startup_data[TAS2557_STARTUP_DATA_PLL_CLKIN_INDEX] = Value;

	tas2557_load_configuration(pTAS2557, nConfiguration, true);
	if (pTAS2557->mbPowerUp){
		dev_dbg(pTAS2557->dev, "device powered up, load startup\n");
		tas2557_dev_load_data(pTAS2557, p_tas2557_startup_data);
		dev_dbg(pTAS2557->dev,
			"device powered up, load unmute\n");
		tas2557_dev_load_data(pTAS2557, p_tas2557_unmute_data);
	}

	return 0;
}

int tas2557_set_calibration(struct tas2557_priv *pTAS2557,
	int nCalibration)
{
	if ((!pTAS2557->mpFirmware->mpPrograms) || (!pTAS2557->mpFirmware->mpConfigurations)) 
	{
		dev_err(pTAS2557->dev, "Firmware not loaded\n\r");
		return -1;
	}

	if (nCalibration == 0x00FF)
	{
		dev_info(pTAS2557->dev, "load new calibration file %s\n", TAS2557_CAL_NAME); 	
		tas2557_load_calibration(pTAS2557, TAS2557_CAL_NAME);
		nCalibration = 0;
	}

	if (nCalibration >= pTAS2557->mpFirmware->mnCalibrations) {
		dev_err(pTAS2557->dev,
			"Calibration %d doesn't exist\n", nCalibration);
		return -1;
	}

	pTAS2557->mnCurrentCalibration = nCalibration;
	if(pTAS2557->mbPowerUp){
		tas2557_load_block(pTAS2557, 
			&(pTAS2557->mpCalFirmware->mpCalibrations[pTAS2557->mnCurrentCalibration].mBlock));
		pTAS2557->mbLoadCalibrationPostPowerUp = false; 
	}else{
		pTAS2557->mbLoadCalibrationPostPowerUp = true; 
	}

	return 0;
}

int tas2557_parse_dt(struct device *dev,
			struct tas2557_priv *pTAS2557)
{
	struct device_node *np = dev->of_node;
	int rc= 0, ret = 0;

	rc = of_property_read_u32(np, "ti,load", &pTAS2557->mnLoad);
	if (rc) {
		dev_err(pTAS2557->dev, "Looking up %s property in node %s failed %d\n",
			"ti,load", np->full_name, rc);
		ret = -1;
	}else{
		dev_dbg(pTAS2557->dev, "ti,load=%d", pTAS2557->mnLoad);
	}

	pTAS2557->mnResetGPIO = of_get_named_gpio(np,
				"ti,cdc-reset-gpio", 0);
	if (pTAS2557->mnResetGPIO < 0) {
		dev_err(pTAS2557->dev, "Looking up %s property in node %s failed %d\n",
			"ti,cdc-reset-gpio", np->full_name,
			pTAS2557->mnResetGPIO);
		ret = -1;
	}else{
		dev_dbg(pTAS2557->dev, "ti,cdc-reset-gpio=%d", pTAS2557->mnResetGPIO);
	}

	return ret;
}

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2557 common functions for Android Linux");
MODULE_LICENSE("GPLv2");