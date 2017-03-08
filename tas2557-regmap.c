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
#include <linux/uaccess.h>
#include "tas2557.h"
#include "tas2557-core.h"

#ifdef CONFIG_TAS2557_CODEC
#include "tas2557-codec.h"
#endif

#ifdef CONFIG_TAS2557_MISC
#include "tas2557-misc.h"
#endif

#define ENABLE_TILOAD
#ifdef ENABLE_TILOAD
#include "tiload.h"
#endif

#define LOW_TEMPERATURE_GAIN 6

static int tas2557_change_book_page(
	struct tas2557_priv *pTAS2557,
	unsigned char nBook,
	unsigned char nPage)
{
	int nResult = 0;

	if ((pTAS2557->mnCurrentBook == nBook) 
		&& pTAS2557->mnCurrentPage == nPage)
		goto end;

	if (pTAS2557->mnCurrentBook != nBook) {
		nResult = regmap_write(pTAS2557->mpRegmap, TAS2557_BOOKCTL_PAGE, 0);
		if (nResult < 0) {
			dev_err(pTAS2557->dev, "%s, %d, I2C error %d\n",
				__func__, __LINE__, nResult);
			goto end;
		}
		pTAS2557->mnCurrentPage = 0;
		nResult = regmap_write(pTAS2557->mpRegmap, TAS2557_BOOKCTL_REG, nBook);
		if (nResult < 0) {
			dev_err(pTAS2557->dev, "%s, %d, I2C error %d\n",
				__func__, __LINE__, nResult);
			goto end;
		}
		pTAS2557->mnCurrentBook = nBook;
		if (nPage != 0) {
			nResult = regmap_write(pTAS2557->mpRegmap, TAS2557_BOOKCTL_PAGE, nPage);
			if (nResult < 0) {
				dev_err(pTAS2557->dev, "%s, %d, I2C error %d\n",
					__func__, __LINE__, nResult);
				goto end;
			}
			pTAS2557->mnCurrentPage = nPage;
		}
	} else if (pTAS2557->mnCurrentPage != nPage) {
		nResult = regmap_write(pTAS2557->mpRegmap, TAS2557_BOOKCTL_PAGE, nPage);
		if (nResult < 0) {
			dev_err(pTAS2557->dev, "%s, %d, I2C error %d\n",
				__func__, __LINE__, nResult);
			goto end;
		}
		pTAS2557->mnCurrentPage = nPage;
	}

end:

	return nResult;
}

static int tas2557_dev_read(
	struct tas2557_priv *pTAS2557,
	unsigned int nRegister,
	unsigned int *pValue)
{
	int nResult = 0;
	unsigned int Value = 0;

	mutex_lock(&pTAS2557->dev_lock);

	if (pTAS2557->mbTILoadActive) {
		if (!(nRegister & 0x80000000))
			goto end; /* let only reads from TILoad pass. */
		nRegister &= ~0x80000000;

		dev_dbg(pTAS2557->dev, "TiLoad R REG B[%d]P[%d]R[%d]\n",
				TAS2557_BOOK_ID(nRegister),
				TAS2557_PAGE_ID(nRegister),
				TAS2557_PAGE_REG(nRegister));
	}

	nResult = tas2557_change_book_page(pTAS2557, 
				TAS2557_BOOK_ID(nRegister),
				TAS2557_PAGE_ID(nRegister));
	if (nResult >= 0) {
		nResult = regmap_read(pTAS2557->mpRegmap, TAS2557_PAGE_REG(nRegister), &Value);
		if (nResult < 0) {
			dev_err(pTAS2557->dev, "%s, %d, I2C error %d\n",
				__func__, __LINE__, nResult);
			goto end;
		}
		*pValue = Value;
	}

end:

	mutex_unlock(&pTAS2557->dev_lock);
	return nResult;
}

static int tas2557_dev_write(
	struct tas2557_priv *pTAS2557,
	unsigned int nRegister,
	unsigned int nValue)
{
	int nResult = 0;

	mutex_lock(&pTAS2557->dev_lock);
	if ((nRegister == 0xAFFEAFFE) && (nValue == 0xBABEBABE)) {
		pTAS2557->mbTILoadActive = true;
		goto end;
	}

	if ((nRegister == 0xBABEBABE) && (nValue == 0xAFFEAFFE)) {
		pTAS2557->mbTILoadActive = false;
		goto end;
	}

	if (pTAS2557->mbTILoadActive) {
		if (!(nRegister & 0x80000000))
			goto end;/* let only writes from TILoad pass. */
		nRegister &= ~0x80000000;

		dev_dbg(pTAS2557->dev, "TiLoad W REG B[%d]P[%d]R[%d] =0x%x\n",
						TAS2557_BOOK_ID(nRegister),
						TAS2557_PAGE_ID(nRegister),
						TAS2557_PAGE_REG(nRegister),
						nValue);
	}

	nResult = tas2557_change_book_page(pTAS2557,
				TAS2557_BOOK_ID(nRegister),
				TAS2557_PAGE_ID(nRegister));
	if (nResult >= 0) {
		nResult = regmap_write(pTAS2557->mpRegmap, TAS2557_PAGE_REG(nRegister), nValue);
		if (nResult < 0)
			dev_err(pTAS2557->dev, "%s, %d, I2C error %d\n",
				__func__, __LINE__, nResult);
	}

end:

	mutex_unlock(&pTAS2557->dev_lock);

	return nResult;
}

static int tas2557_dev_bulk_read(
	struct tas2557_priv *pTAS2557,
	unsigned int nRegister,
	u8 *pData,
	unsigned int nLength)
{
	int nResult = 0;

	mutex_lock(&pTAS2557->dev_lock);
	if (pTAS2557->mbTILoadActive) {
		if (!(nRegister & 0x80000000))
			goto end; /* let only writes from TILoad pass. */

		nRegister &= ~0x80000000;
		dev_dbg(pTAS2557->dev, "TiLoad BR REG B[%d]P[%d]R[%d], count=%d\n",
				TAS2557_BOOK_ID(nRegister),
				TAS2557_PAGE_ID(nRegister),
				TAS2557_PAGE_REG(nRegister),
				nLength);
	}

	nResult = tas2557_change_book_page(pTAS2557,
				TAS2557_BOOK_ID(nRegister),
				TAS2557_PAGE_ID(nRegister));
	if (nResult >= 0) {
		nResult = regmap_bulk_read(pTAS2557->mpRegmap, TAS2557_PAGE_REG(nRegister), pData, nLength);
		if (nResult < 0)
			dev_err(pTAS2557->dev, "%s, %d, I2C error %d\n",
				__func__, __LINE__, nResult);
	}

end:

	mutex_unlock(&pTAS2557->dev_lock);
	return nResult;
}

static int tas2557_dev_bulk_write(
	struct tas2557_priv *pTAS2557,
	unsigned int nRegister,
	u8 *pData,
	unsigned int nLength)
{
	int nResult = 0;

	mutex_lock(&pTAS2557->dev_lock);
	if (pTAS2557->mbTILoadActive) {
		if (!(nRegister & 0x80000000))
			goto end; /* let only writes from TILoad pass. */

		nRegister &= ~0x80000000;

		dev_dbg(pTAS2557->dev, "TiLoad BW REG B[%d]P[%d]R[%d], count=%d\n",
				TAS2557_BOOK_ID(nRegister),
				TAS2557_PAGE_ID(nRegister),
				TAS2557_PAGE_REG(nRegister),
				nLength);
	}

	nResult = tas2557_change_book_page( pTAS2557,
				TAS2557_BOOK_ID(nRegister),
				TAS2557_PAGE_ID(nRegister));
	if (nResult >= 0) {
		nResult = regmap_bulk_write(pTAS2557->mpRegmap, TAS2557_PAGE_REG(nRegister), pData, nLength);
		if (nResult < 0)
			dev_err(pTAS2557->dev, "%s, %d, I2C error %d\n",
				__func__, __LINE__, nResult);
	}

end:

	mutex_unlock(&pTAS2557->dev_lock);
	return nResult;
}

static int tas2557_dev_update_bits(
	struct tas2557_priv *pTAS2557,
	unsigned int nRegister,
	unsigned int nMask,
	unsigned int nValue)
{
	int nResult = 0;

	mutex_lock(&pTAS2557->dev_lock);

	if (pTAS2557->mbTILoadActive) {
		if (!(nRegister & 0x80000000))
			goto end; /* let only writes from TILoad pass. */

		nRegister &= ~0x80000000;
		dev_dbg(pTAS2557->dev, "TiLoad SB REG B[%d]P[%d]R[%d], mask=0x%x, value=0x%x\n",
				TAS2557_BOOK_ID(nRegister),
				TAS2557_PAGE_ID(nRegister),
				TAS2557_PAGE_REG(nRegister),
				nMask, nValue);
	}

	nResult = tas2557_change_book_page( pTAS2557,
				TAS2557_BOOK_ID(nRegister),
				TAS2557_PAGE_ID(nRegister));
	if (nResult >= 0) {
		nResult = regmap_update_bits(pTAS2557->mpRegmap, TAS2557_PAGE_REG(nRegister), nMask, nValue);
		if (nResult < 0)
			dev_err(pTAS2557->dev, "%s, %d, I2C error %d\n",
				__func__, __LINE__, nResult);
	}

end:
	mutex_unlock(&pTAS2557->dev_lock);
	return nResult;
}

int tas2557_enableIRQ(struct tas2557_priv *pTAS2557, bool enable, bool clear)
{
	unsigned int nValue;
	int nResult = 0;

	if (enable) {
		if (clear) {
			nResult = pTAS2557->read(pTAS2557, TAS2557_FLAGS_1, &nValue);
			if (nResult >= 0)
				nResult = pTAS2557->read(pTAS2557, TAS2557_FLAGS_2, &nValue);
		}

		if (!pTAS2557->mbIRQEnable) {
			if (pTAS2557->mnIRQ != 0)
				enable_irq(pTAS2557->mnIRQ);
			pTAS2557->mbIRQEnable = true;
		}
	} else {
		if (pTAS2557->mbIRQEnable) {
			if (pTAS2557->mnIRQ != 0)
				disable_irq_nosync(pTAS2557->mnIRQ);
			pTAS2557->mbIRQEnable = false;
		}

		if (clear) {
			nResult = pTAS2557->read(pTAS2557, TAS2557_FLAGS_1, &nValue);
			if (nResult >= 0)
				nResult = pTAS2557->read(pTAS2557, TAS2557_FLAGS_2, &nValue);
		}
	}

	return nResult;
}

static void tas2557_hw_reset(struct tas2557_priv *pTAS2557)
{
#ifdef ENABLE_GPIO_RESET
	if (gpio_is_valid(pTAS2557->mnResetGPIO)) {
		devm_gpio_request_one(pTAS2557->dev, pTAS2557->mnResetGPIO,
			GPIOF_OUT_INIT_LOW, "TAS2557_RST");
		msleep(10);
		gpio_set_value_cansleep(pTAS2557->mnResetGPIO, 1);
		udelay(1000);
	}
#endif
	pTAS2557->mnCurrentBook = -1;
	pTAS2557->mnCurrentPage = -1;
}

static void irq_work_routine(struct work_struct *work)
{
	int nResult = 0;
	unsigned int nDevInt1Status = 0, nDevInt2Status = 0;
	unsigned int nDevPowerUpFlag = 0, nDevPowerStatus = 0;
	struct tas2557_priv *pTAS2557 =
		container_of(work, struct tas2557_priv, irq_work.work);

	if (!pTAS2557->mbPowerUp)
		return;

	nResult = tas2557_dev_read(pTAS2557, TAS2557_FLAGS_1, &nDevInt1Status);
	if (nResult >= 0)
		nResult = tas2557_dev_read(pTAS2557, TAS2557_FLAGS_2, &nDevInt2Status);

	if (nResult < 0)
		goto program;

	if (((nDevInt1Status & 0xdc) != 0) || ((nDevInt2Status & 0x0c) != 0)) {
		/* in case of INT_OC, INT_UV, INT_OT, INT_BO, INT_CL, INT_CLK1, INT_CLK2 */
		dev_err(pTAS2557->dev, "critical error: 0x%x, 0x%x\n", nDevInt1Status, nDevInt2Status);
		goto program;
	} else {
		nResult = tas2557_dev_read(pTAS2557, TAS2557_POWER_UP_FLAG_REG, &nDevPowerUpFlag);
		if (nResult < 0)
			goto program;
		if ((nDevPowerUpFlag & 0x40) == 0) {
			/* Class-D doesn't power on */
			nResult = tas2557_dev_read(pTAS2557, TAS2557_POWER_CTRL2_REG, &nDevPowerStatus);
			if (nResult < 0)
				goto program;
			if (nDevPowerStatus & 0x80)
				goto program; /* failed to power on the Class-D */
		}

		dev_dbg(pTAS2557->dev, "%s: INT1=0x%x, INT2=0x%x; PowerUpFlag=0x%x, PwrStatus=0x%x\n",
			__func__, nDevInt1Status, nDevInt2Status, nDevPowerUpFlag, nDevPowerStatus);
	}
	return;

program:
	/* hardware reset and reload */
	tas2557_hw_reset(pTAS2557);
	tas2557_set_program(pTAS2557, pTAS2557->mnCurrentProgram, pTAS2557->mnCurrentConfiguration);
}

static irqreturn_t tas2557_irq_handler(int irq, void *dev_id)
{
	struct tas2557_priv *pTAS2557 = (struct tas2557_priv *)dev_id;

	tas2557_enableIRQ(pTAS2557, false, false);
	/* get IRQ status after 100 ms */
	schedule_delayed_work(&pTAS2557->irq_work, msecs_to_jiffies(100));
	return IRQ_HANDLED;
}

static enum hrtimer_restart temperature_timer_func(struct hrtimer *timer)
{
	struct tas2557_priv *pTAS2557 = container_of(timer, struct tas2557_priv, mtimer);

	if (pTAS2557->mbPowerUp)
		schedule_work(&pTAS2557->mtimerwork);
	return HRTIMER_NORESTART;
}

static void timer_work_routine(struct work_struct *work)
{
	struct tas2557_priv *pTAS2557 = container_of(work, struct tas2557_priv, mtimerwork);
	int nResult, nTemp;

	if (!pTAS2557->mbPowerUp)
		goto end;

	nResult = tas2557_get_die_temperature(pTAS2557, &nTemp);
	if (nResult >= 0) {
		dev_dbg(pTAS2557->dev, "Die=0x%x, degree=%d\n", nTemp, (nTemp>>23));
		if ((nTemp & 0x80000000) != 0) {
			/* if Die temperature is below ZERO */
			if (pTAS2557->mnDevCurrentGain != LOW_TEMPERATURE_GAIN) {
				nResult = tas2557_set_DAC_gain(pTAS2557, LOW_TEMPERATURE_GAIN);
				if (nResult < 0)
					goto end;
				pTAS2557->mnDevCurrentGain = LOW_TEMPERATURE_GAIN;
				dev_dbg(pTAS2557->dev, "LOW Temp: set gain to %d\n", LOW_TEMPERATURE_GAIN);
			}
		} else {
			/* if Die temperature is above ZERO */
			if (pTAS2557->mnDevCurrentGain != pTAS2557->mnDevGain) {
				nResult = tas2557_set_DAC_gain(pTAS2557, pTAS2557->mnDevGain);
				if (nResult < 0)
					goto end;
				pTAS2557->mnDevCurrentGain = pTAS2557->mnDevGain;
				dev_dbg(pTAS2557->dev, "LOW Temp: set gain to original\n");
			}
		}

		if (pTAS2557->mbPowerUp)
			hrtimer_start(&pTAS2557->mtimer,
				ns_to_ktime((u64)LOW_TEMPERATURE_CHECK_PERIOD * NSEC_PER_MSEC), HRTIMER_MODE_REL);
	}

end:
	return;
}

#ifdef CONFIG_PM_SLEEP
static int tas2557_suspend(struct device *dev)
{
	struct tas2557_priv *pTAS2557 = dev_get_drvdata(dev);

	dev_dbg(pTAS2557->dev, "%s\n", __func__);
	if (hrtimer_active(&pTAS2557->mtimer)) {
		dev_dbg(pTAS2557->dev, "cancel die temp timer\n");
		hrtimer_cancel(&pTAS2557->mtimer);
	}

	return 0;
}

static int tas2557_resume(struct device *dev)
{
	struct tas2557_priv *pTAS2557 = dev_get_drvdata(dev);
	struct TProgram *pProgram;

	dev_dbg(pTAS2557->dev, "%s\n", __func__);
	if (!pTAS2557->mpFirmware->mpPrograms) {
		dev_dbg(pTAS2557->dev, "%s, firmware not loaded\n", __func__);
		goto end;
	}

	if (pTAS2557->mnCurrentProgram >= pTAS2557->mpCalFirmware->mnPrograms) {
		dev_err(pTAS2557->dev, "%s, firmware corrupted\n", __func__);
		goto end;
	}

	pProgram = &(pTAS2557->mpFirmware->mpPrograms[pTAS2557->mnCurrentProgram]);
	if (pTAS2557->mbPowerUp && (pProgram->mnAppMode == TAS2557_APP_TUNINGMODE)) {
		dev_dbg(pTAS2557->dev, "%s, start Die Temp check timer\n", __func__);
		hrtimer_start(&pTAS2557->mtimer,
			ns_to_ktime((u64)LOW_TEMPERATURE_CHECK_PERIOD * NSEC_PER_MSEC), HRTIMER_MODE_REL);
	}

end:

	return 0; 
}
#endif

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

/* tas2557_i2c_probe :
* platform dependent
* should implement hardware reset functionality
*/
static int tas2557_i2c_probe(struct i2c_client *pClient,
	const struct i2c_device_id *pID)
{
	struct tas2557_priv *pTAS2557;
	int nResult = 0;
	unsigned int nValue = 0;
	const char *pFWName;

	dev_info(&pClient->dev, "%s enter\n", __func__);

	pTAS2557 = devm_kzalloc(&pClient->dev, sizeof(struct tas2557_priv), GFP_KERNEL);
	if (!pTAS2557) {
		nResult = -ENOMEM;
		goto err;
	}

	pTAS2557->dev = &pClient->dev;
	i2c_set_clientdata(pClient, pTAS2557);
	dev_set_drvdata(&pClient->dev, pTAS2557);

	pTAS2557->mpRegmap = devm_regmap_init_i2c(pClient, &tas2557_i2c_regmap);
	if (IS_ERR(pTAS2557->mpRegmap)) {
		nResult = PTR_ERR(pTAS2557->mpRegmap);
		dev_err(&pClient->dev, "Failed to allocate register map: %d\n",
			nResult);
		goto err;
	}

	if (pClient->dev.of_node)
		tas2557_parse_dt(&pClient->dev, pTAS2557);

	tas2557_hw_reset(pTAS2557);

	pTAS2557->read = tas2557_dev_read;
	pTAS2557->write = tas2557_dev_write;
	pTAS2557->bulk_read = tas2557_dev_bulk_read;
	pTAS2557->bulk_write = tas2557_dev_bulk_write;
	pTAS2557->update_bits = tas2557_dev_update_bits;
	pTAS2557->enableIRQ = tas2557_enableIRQ;
	pTAS2557->set_config = tas2557_set_config;
	pTAS2557->set_calibration = tas2557_set_calibration;
	pTAS2557->hw_reset = tas2557_hw_reset;

	mutex_init(&pTAS2557->dev_lock);

	/* Reset the chip */
	nResult = tas2557_dev_write(pTAS2557, TAS2557_SW_RESET_REG, 0x01);
	if (nResult < 0) {
		dev_err(&pClient->dev, "I2c fail, %d\n", nResult);
		goto err;
	}

	msleep(1);
	tas2557_dev_read(pTAS2557, TAS2557_REV_PGID_REG, &nValue);
	pTAS2557->mnPGID = nValue;
	if (pTAS2557->mnPGID == TAS2557_PG_VERSION_2P1) {
		dev_info(pTAS2557->dev, "PG2.1 Silicon found\n");
		pFWName = TAS2557_FW_NAME;
	} else if (pTAS2557->mnPGID == TAS2557_PG_VERSION_1P0) {
		dev_info(pTAS2557->dev, "PG1.0 Silicon found\n");
		pFWName = TAS2557_PG1P0_FW_NAME;
	} else {
		nResult = -ENOTSUPP;
		dev_info(pTAS2557->dev, "unsupport Silicon 0x%x\n", pTAS2557->mnPGID);
		goto err;
	}

	if (gpio_is_valid(pTAS2557->mnGpioINT)) {
		nResult = gpio_request(pTAS2557->mnGpioINT, "TAS2557-IRQ");
		if (nResult < 0) {
			dev_err(pTAS2557->dev,
				"%s: GPIO %d request INT error\n",
				__func__, pTAS2557->mnGpioINT);
			goto err;
		}

		gpio_direction_input(pTAS2557->mnGpioINT);
		pTAS2557->mnIRQ = gpio_to_irq(pTAS2557->mnGpioINT);
		dev_dbg(pTAS2557->dev, "irq = %d\n", pTAS2557->mnIRQ);
		INIT_DELAYED_WORK(&pTAS2557->irq_work, irq_work_routine);
		nResult = request_threaded_irq(pTAS2557->mnIRQ, tas2557_irq_handler,
				NULL, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				pClient->name, pTAS2557);
		if (nResult < 0) {
			dev_err(pTAS2557->dev,
				"request_irq failed, %d\n", nResult);
			goto err;
		}
		disable_irq_nosync(pTAS2557->mnIRQ);
	}

	pTAS2557->mpFirmware = devm_kzalloc(&pClient->dev, sizeof(struct TFirmware), GFP_KERNEL);
	if (!pTAS2557->mpFirmware) {
		nResult = -ENOMEM;
		goto err;
	}

	pTAS2557->mpCalFirmware = devm_kzalloc(&pClient->dev, sizeof(struct TFirmware), GFP_KERNEL);
	if (!pTAS2557->mpCalFirmware) {
		nResult = -ENOMEM;
		goto err;
	}

#ifdef CONFIG_TAS2557_CODEC
	tas2557_register_codec(pTAS2557);
#endif

#ifdef CONFIG_TAS2557_MISC
	mutex_init(&pTAS2557->file_lock);
	tas2557_register_misc(pTAS2557);
#endif

#ifdef ENABLE_TILOAD
	tiload_driver_init(pTAS2557);
#endif

	hrtimer_init(&pTAS2557->mtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pTAS2557->mtimer.function = temperature_timer_func;
	INIT_WORK(&pTAS2557->mtimerwork, timer_work_routine);

	nResult = request_firmware_nowait(THIS_MODULE, 1, pFWName,
				pTAS2557->dev, GFP_KERNEL, pTAS2557, tas2557_fw_ready);

err:

	return nResult;
}

static int tas2557_i2c_remove(struct i2c_client *pClient)
{
	struct tas2557_priv *pTAS2557 = i2c_get_clientdata(pClient);

	dev_info(pTAS2557->dev, "%s\n", __func__);

#ifdef CONFIG_TAS2557_CODEC
	tas2557_deregister_codec(pTAS2557);
#endif

#ifdef CONFIG_TAS2557_MISC
	tas2557_deregister_misc(pTAS2557);
	mutex_destroy(&pTAS2557->file_lock);
#endif

	mutex_destroy(&pTAS2557->dev_lock);
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

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops tas2557_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tas2557_suspend, tas2557_resume)
};
#endif

static struct i2c_driver tas2557_i2c_driver = {
	.driver = {
			.name = "tas2557",
			.owner = THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
			.pm = &tas2557_pm_ops,
#endif
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
MODULE_LICENSE("GPL v2");
#endif