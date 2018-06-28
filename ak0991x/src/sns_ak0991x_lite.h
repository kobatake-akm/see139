#pragma once
/**
 * @file sns_ak0991x_lite.h
 *
 * compile switches for SEE-Lite.
 *
 * Copyright (c) 2016-2018 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 **/

/*--------------------------------------------------------------------------
 *
 * COMPILE CONDITION
 *
 *-------------------------------------------------------------------------*/
// Target board HDK845
#define AK0991X_BOARD_HDK845

// Standard SEE Mode. Enabled all features.
#define AK0991X_ENABLE_DEBUG_MSG          // Enable debug messages
//#define AK0991X_ENABLE_DAE                // Enable DAE
//#define AK0991X_ENABLE_I3C_SUPPORT        // Enable support for I3C bus
//#define AK0991X_ENABLE_I3C_DEBUG
//#define AK0991X_ENABLE_DUAL_SENSOR        // Enable to set dual sensor support mode
//#define AK0991X_ENABLE_DEVICE_MODE_SENSOR // Enable devise_mode_sensor
//#define AK0991X_FORCE_MAX_ODR_50HZ        // Force MAX ODR to 50Hz

#ifdef AK0991X_ENABLE_DEVICE_MODE_SENSOR
#define MAX_DEVICE_MODE_SUPPORTED 2       // change number 2/4/8 in order to match the faccal num in the registry
#else
#define MAX_DEVICE_MODE_SUPPORTED 1
#endif
