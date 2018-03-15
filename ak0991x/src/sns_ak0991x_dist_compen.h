/******************************************************************************
 *
 * Copyright (c) 2004 Asahi Kasei Microdevices Corporation, Japan
 * All Rights Reserved.
 *
 * This software program is the proprietary program of Asahi Kasei Microdevices
 * Corporation("AKM") licensed to authorized Licensee under the respective
 * agreement between the Licensee and AKM only for use with AKM's electronic
 * compass IC.
 *
 * THIS SOFTWARE IS PROVIDED TO YOU "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABLITY, FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT OF
 * THIRD PARTY RIGHTS, AND WE SHALL NOT BE LIABLE FOR ANY LOSSES AND DAMAGES
 * WHICH MAY OCCUR THROUGH USE OF THIS SOFTWARE.
 *
 ******************************************************************************/
#ifndef _SNSD_MAG_AKM_DIST_COMPEN_H
#define _SNSD_MAG_AKM_DIST_COMPEN_H

#include <stdint.h>

#include "sns_rc.h"
#include "sns_sensor.h"

#define AKSC_PDC_SIZE 27

sns_rc AKSC_DistCompen(
	const uint8_t pdc[AKSC_PDC_SIZE],
	float         data[3]
);

#endif //_SNSD_MAG_AKM_DIST_COMPEN_H
