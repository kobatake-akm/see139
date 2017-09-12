#pragma once
/**
 * @file sns_ak0991x_lite.h
 *
 * compile switches for SEE-Lite.
 *
 * Copyright (c) 2016-2017 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 **/

// Enable below macro to set SEE Lite mode
//#define AK0991X_ENABLE_SEE_LITE           // Enable SEE-Lite mode

#ifndef AK0991X_ENABLE_SEE_LITE

#define AK0991X_ENABLE_REGISTRY_SENSOR    // (TBD)Enable registry sensor
#define AK0991X_ENABLE_ALL_ATTRIBUTES     // Enable all attribute service
#define AK0991X_ENABLE_DEBUG_MSG          // Enable debug messages
#define AK0991X_ENABLE_DAE                // Enable DAE
#define AK0991X_ENABLE_DIAG_LOGGING       // Enable diagnostic logging
#define AK0991X_ENABLE_POWER_RAIL         // Enable power rail reference
#define AK0991X_ENABLE_DEINIT             // Enable deinit call
#define AK0991X_ENABLE_S4S                // Enable S4S parts

#endif	// AK0991X_ENABLE_SEE_LITE
