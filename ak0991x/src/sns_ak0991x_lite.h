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
#define AK0991X_ENABLE_REGISTRY_ACCESS    // Enable registry access  -- note: normally defined in scons
#define AK0991X_ENABLE_ALL_ATTRIBUTES     // Enable all attribute service
#define AK0991X_ENABLE_DEBUG_MSG          // Enable debug messages
//#define AK0991X_ENABLE_DAE                // Enable DAE
#define AK0991X_ENABLE_DIAG_LOGGING       // Enable diagnostic logging
#define AK0991X_ENABLE_POWER_RAIL         // Enable power rail reference
#define AK0991X_ENABLE_DEINIT             // Enable deinit call
//#define AK0991X_ENABLE_S4S                // Enable S4S parts
#define AK0991X_ENABLE_FUSE               // Enable fuse rom
#define AK0991X_ENABLE_DRI                // Enable DRI
#define AK0991X_ENABLE_FIFO               // Enable FIFO
//#define AK0991X_ENABLE_I3C_SUPPORT        // Enable support for I3C bus
#define AK0991X_ENABLE_REG_WRITE_ACCESS   // Enable registry write access
#define AK0991X_ENABLE_REG_FAC_CAL        // Enable factory cal access
//#define AK0991X_FORCE_MAX_ODR_50HZ        // Force MAX ODR to 50Hz
//#define AK0991X_ENABLE_I3C_DEBUG
//#define AK0991X_ENABLE_DUAL_SENSOR        // Enable to set dual sensor support mode
//#define AK0991X_ENABLE_DEVICE_MODE_SENSOR // Enable devise_mode_sensor

#ifdef AK0991X_ENABLE_DEVICE_MODE_SENSOR
#define MAX_DEVICE_MODE_SUPPORTED 2       // change number 2/4/8 in order to match the faccal num in the registry
#else
#define MAX_DEVICE_MODE_SUPPORTED 1
#endif

#endif	// AK0991X_ENABLE_SEE_LITE
