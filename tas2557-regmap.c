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
**     tas2557-regmap.c
**
** Description:
**     I2C driver with regmap for Texas Instruments TAS2557 High Performance 4W Smart Amplifier
**
** =============================================================================
*/

#ifdef CONFIG_TAS2557_REGMAP

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

#ifdef CONFIG_TAS2557_CODEC
#include "tas2557-codec.h"
#endif

#ifdef CONFIG_TAS2557_MISC
#include "tas2557-misc.h"
#endif

#define ENABLE_TILOAD			//only enable this for in-system tuning or debug, not for production systems
#ifdef ENABLE_TILOAD
#include "tiload.h"
#endif

static void tas2557_change_book_page(struct tas2557_priv *pTAS2557, int nBook,
	int nPage)
{
	if ((pTAS2557->mnCurrentBook == nBook) 
		&& pTAS2557->mnCurrentPage == nPage){
		return;
	}

	if (pTAS2557->mnCurrentBook != nBook) {
		regmap_write(pTAS2557->mpRegmap, TAS2557_BOOKCTL_PAGE, 0);
		pTAS2557->mnCurrentPage = 0;
		regmap_write(pTAS2557->mpRegmap, TAS2557_BOOKCTL_REG, nBook);
		pTAS2557->mnCurrentBook = nBook;
		if (nPage != 0) {
			regmap_write(pTAS2557->mpRegmap, TAS2557_BOOKCTL_PAGE, nPage);
			pTAS2557->mnCurrentPage = nPage;
		}
	} else if (pTAS2557->mnCurrentPage != nPage) {
		regmap_write(pTAS2557->mpRegmap, TAS2557_BOOKCTL_PAGE, nPage);
		pTAS2557->mnCurrentPage = nPage;
	}
}

static int tas2557_dev_read(struct tas2557_priv *pTAS2557,
	unsigned int nRegister, unsigned int *pValue)
{
	int ret = 0;

	mutex_lock(&pTAS2557->dev_lock);	
	
	if (pTAS2557->mbTILoadActive) {
		if (!(nRegister & 0x80000000)){
			mutex_unlock(&pTAS2557->dev_lock);
			return 0;			// let only reads from TILoad pass.
		}
		nRegister &= ~0x80000000;
	}

/*	
	dev_dbg(pTAS2557->dev, "%s: BOOK:PAGE:REG %u:%u:%u\n", __func__,
		TAS2557_BOOK_ID(nRegister), TAS2557_PAGE_ID(nRegister),
		TAS2557_PAGE_REG(nRegister));
*/
	tas2557_change_book_page(pTAS2557, TAS2557_BOOK_ID(nRegister),
		TAS2557_PAGE_ID(nRegister));
	ret = regmap_read(pTAS2557->mpRegmap, TAS2557_PAGE_REG(nRegister), pValue);

	mutex_unlock(&pTAS2557->dev_lock);
	return ret;
}

static int tas2557_dev_write(struct tas2557_priv *pTAS2557,
	unsigned int nRegister, unsigned int nValue)
{
	int ret = 0;
	
	mutex_lock(&pTAS2557->dev_lock);
	if ((nRegister == 0xAFFEAFFE) && (nValue == 0xBABEBABE)) {
		pTAS2557->mbTILoadActive = true;
		mutex_unlock(&pTAS2557->dev_lock);
		return 0;
	}

	if ((nRegister == 0xBABEBABE) && (nValue == 0xAFFEAFFE)) {
		pTAS2557->mbTILoadActive = false;
		mutex_unlock(&pTAS2557->dev_lock);
		return 0;
	}

	if (pTAS2557->mbTILoadActive) {
		if (!(nRegister & 0x80000000)){
			mutex_unlock(&pTAS2557->dev_lock);
			return 0;			// let only writes from TILoad pass.
		}
		nRegister &= ~0x80000000;
	}

	tas2557_change_book_page(pTAS2557, TAS2557_BOOK_ID(nRegister),
		TAS2557_PAGE_ID(nRegister));
//  dev_err(codec->dev, "%s: BOOK:PAGE:REG %u:%u:%u, VAL: 0x%02x\n",
//      __func__, TAS2557_BOOK_ID(nRegister), TAS2557_PAGE_ID(nRegister),
//      TAS2557_PAGE_REG(nRegister), value);
	ret = regmap_write(pTAS2557->mpRegmap, TAS2557_PAGE_REG(nRegister),
		nValue);
	mutex_unlock(&pTAS2557->dev_lock);		
	
	return ret;
}

static int tas2557_dev_bulk_read(struct tas2557_priv *pTAS2557,
	unsigned int nRegister, u8 * pData, unsigned int nLength)
{
	int ret = 0;
	
	mutex_lock(&pTAS2557->dev_lock);
	if (pTAS2557->mbTILoadActive) {
		if (!(nRegister & 0x80000000)){
			mutex_unlock(&pTAS2557->dev_lock);
			return 0;			// let only writes from TILoad pass.
		}
		nRegister &= ~0x80000000;
	}

	tas2557_change_book_page(pTAS2557, TAS2557_BOOK_ID(nRegister),
		TAS2557_PAGE_ID(nRegister));
	ret = regmap_bulk_read(pTAS2557->mpRegmap, TAS2557_PAGE_REG(nRegister),
		pData, nLength);
	mutex_unlock(&pTAS2557->dev_lock);	

	return ret;
}

static int tas2557_dev_bulk_write(struct tas2557_priv *pTAS2557,
	unsigned int nRegister, u8 * pData, unsigned int nLength)
{
	int ret = 0;
	mutex_lock(&pTAS2557->dev_lock);
	if (pTAS2557->mbTILoadActive) {
		if (!(nRegister & 0x80000000)){
			mutex_unlock(&pTAS2557->dev_lock);
			return 0;			// let only writes from TILoad pass.
		}
		nRegister &= ~0x80000000;
	}

	tas2557_change_book_page(pTAS2557, TAS2557_BOOK_ID(nRegister),
		TAS2557_PAGE_ID(nRegister));
	ret = regmap_bulk_write(pTAS2557->mpRegmap, TAS2557_PAGE_REG(nRegister),
		pData, nLength);
	mutex_unlock(&pTAS2557->dev_lock);		
	
	return ret;
}

static int tas2557_dev_update_bits(struct tas2557_priv *pTAS2557,
	unsigned int nRegister, unsigned int nMask, unsigned int nValue)
{
	int ret = 0;
	
	mutex_lock(&pTAS2557->dev_lock);
	
	if (pTAS2557->mbTILoadActive) {
		if (!(nRegister & 0x80000000)){
			mutex_unlock(&pTAS2557->dev_lock);
			return 0;			// let only writes from TILoad pass.
		}
		nRegister &= ~0x80000000;
	}
	
	tas2557_change_book_page(pTAS2557, TAS2557_BOOK_ID(nRegister),
		TAS2557_PAGE_ID(nRegister));
	
	ret = regmap_update_bits(pTAS2557->mpRegmap, TAS2557_PAGE_REG(nRegister), nMask, nValue);
		
	mutex_unlock(&pTAS2557->dev_lock);		
	return ret;
}

static bool tas2557_volatile(struct device *pDev, unsigned int nRegister)
{
	return true;
}

static bool tas2557_writeable(struct device *pDev, unsigned int nRegister)
{
	return true;
}

static const struct regmap_config tas2557_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = tas2557_writeable,
	.volatile_reg = tas2557_volatile,
	.cache_type = REGCACHE_NONE,
	.max_register = 128,
};

static int tas2557_i2c_probe(struct i2c_client *pClient,
	const struct i2c_device_id *pID)
{
	struct tas2557_priv *pTAS2557;
	int nResult;

	dev_info(&pClient->dev, "%s enter\n", __FUNCTION__);
	
	pTAS2557 = devm_kzalloc(&pClient->dev, sizeof(struct tas2557_priv), GFP_KERNEL);
	if (!pTAS2557)
		return -ENOMEM;

	pTAS2557->dev = &pClient->dev;
	i2c_set_clientdata(pClient, pTAS2557);
	dev_set_drvdata(&pClient->dev, pTAS2557);

	if (pClient->dev.of_node){
		tas2557_parse_dt(&pClient->dev, pTAS2557);
	}

	if (gpio_is_valid(pTAS2557->mnResetGPIO)) {
#ifdef HW_RESET	//mandatory		
		devm_gpio_request_one(&pClient->dev, pTAS2557->mnResetGPIO,
			GPIOF_OUT_INIT_LOW, "TAS2557_RST");
		msleep(5);
		gpio_set_value_cansleep(pTAS2557->mnResetGPIO, 1);
		mdelay(1);
#endif		
	}

	pTAS2557->mpRegmap = devm_regmap_init_i2c(pClient, &tas2557_i2c_regmap);
	if (IS_ERR(pTAS2557->mpRegmap)) {
		nResult = PTR_ERR(pTAS2557->mpRegmap);
		dev_err(&pClient->dev, "Failed to allocate register map: %d\n",
			nResult);
		return nResult;
	}

	pTAS2557->read = tas2557_dev_read;
	pTAS2557->write = tas2557_dev_write;
	pTAS2557->bulk_read = tas2557_dev_bulk_read;
	pTAS2557->bulk_write = tas2557_dev_bulk_write;
	pTAS2557->update_bits = tas2557_dev_update_bits;
	pTAS2557->set_config = tas2557_set_config;
	pTAS2557->set_calibration = tas2557_set_calibration;
		
	mutex_init(&pTAS2557->dev_lock);
	
	/* Reset the chip */
	nResult = tas2557_dev_write(pTAS2557, TAS2557_SW_RESET_REG, 0x01);
	if(nResult < 0){
		dev_err(&pClient->dev, "I2C communication ERROR: %d\n",
			nResult);
		return nResult;
	}
	
	udelay(1000);

	pTAS2557->mpFirmware =
		devm_kzalloc(&pClient->dev, sizeof(TFirmware),
		GFP_KERNEL);
	if (!pTAS2557->mpFirmware)
		return -ENOMEM;

	pTAS2557->mpCalFirmware =
		devm_kzalloc(&pClient->dev, sizeof(TFirmware),
		GFP_KERNEL);
	if (!pTAS2557->mpCalFirmware)
		return -ENOMEM;

	pTAS2557->mnCurrentPage = 0;
	pTAS2557->mnCurrentBook = 0;

	nResult = tas2557_dev_read(pTAS2557, TAS2557_REV_PGID_REG, &pTAS2557->mnPGID);
	dev_info(&pClient->dev, "TAS2557 PGID: 0x%02x\n", pTAS2557->mnPGID);

	pTAS2557->mbTILoadActive = false;

#ifdef CONFIG_TAS2557_CODEC	
	mutex_init(&pTAS2557->codec_lock);
	tas2557_register_codec(pTAS2557);
#endif

#ifdef CONFIG_TAS2557_MISC	
	mutex_init(&pTAS2557->file_lock);
	tas2557_register_misc(pTAS2557);
#endif

#ifdef ENABLE_TILOAD
	tiload_driver_init(pTAS2557);
#endif

	nResult = request_firmware_nowait(THIS_MODULE, 1, TAS2557_FW_NAME,
		pTAS2557->dev, GFP_KERNEL, pTAS2557, tas2557_fw_ready);
		
	return nResult;
}

static int tas2557_i2c_remove(struct i2c_client *pClient)
{
	struct tas2557_priv *pTAS2557 = i2c_get_clientdata(pClient);
	
	dev_info(pTAS2557->dev, "%s\n", __FUNCTION__);
	
#ifdef CONFIG_TAS2557_CODEC		
	tas2557_deregister_codec(pTAS2557);
	mutex_destroy(&pTAS2557->codec_lock);
#endif

#ifdef CONFIG_TAS2557_MISC		
	tas2557_deregister_misc(pTAS2557);
	mutex_destroy(&pTAS2557->file_lock);
#endif
	
	return 0;
}

static const struct i2c_device_id tas2557_i2c_id[] = {
	{"tas2557", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tas2557_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id tas2557_of_match[] = {
	{.compatible = "ti,tas2557"},
	{},
};

MODULE_DEVICE_TABLE(of, tas2557_of_match);
#endif

static struct i2c_driver tas2557_i2c_driver = {
	.driver = {
			.name = "tas2557",
			.owner = THIS_MODULE,
#if defined(CONFIG_OF)
			.of_match_table = of_match_ptr(tas2557_of_match),
#endif
		},
	.probe = tas2557_i2c_probe,
	.remove = tas2557_i2c_remove,
	.id_table = tas2557_i2c_id,
};

module_i2c_driver(tas2557_i2c_driver);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2557 I2C Smart Amplifier driver");
MODULE_LICENSE("GPLv2");
#endif