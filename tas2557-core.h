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
**     tas2557-core.h
**
** Description:
**     header file for tas2557-core.c
**
** =============================================================================
*/

#ifndef _TAS2557_CORE_H
#define _TAS2557_CORE_H

#include "tas2557.h"

int tas2557_parse_dt(struct device *dev,
			struct tas2557_priv *pTAS2557);
void tas2557_enable(struct tas2557_priv *pTAS2557, bool bEnable);
int tas2557_set_sampling_rate(struct tas2557_priv *pTAS2557, 
	unsigned int nSamplingRate);
int tas2557_set_config(struct tas2557_priv *pTAS2557, int config);
void tas2557_load_fs_firmware(struct tas2557_priv *pTAS2557,
	char *pFileName);
void tas2557_fw_ready(const struct firmware *pFW, void *pContext);
int tas2557_set_program(struct tas2557_priv *pTAS2557,
	unsigned int nProgram);
int tas2557_set_calibration(struct tas2557_priv *pTAS2557,
	int nCalibration);
int tas2557_load_default(struct tas2557_priv *pTAS2557);
int tas2557_setChannel(struct tas2557_priv *pTAS2557, int channel);
	
#endif /* _TAS2557_CORE_H */
