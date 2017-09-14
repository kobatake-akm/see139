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

#ifdef AK0991X_ENABLE_SEE_LITE

//#define AK0991X_ENABLE_REGISTRY_ACCESS    // Enable registry access
#define AK0991X_ENABLE_ALL_ATTRIBUTES     // Enable all attribute service
//#define AK0991X_ENABLE_DEBUG_MSG          // Enable debug messages
//#define AK0991X_ENABLE_DIAG_LOGGING       // Enable diagnostic logging
#define AK0991X_ENABLE_POWER_RAIL         // Enable power rail reference
//#define AK0991X_ENABLE_DEINIT             // Enable deinit call

//select target device
#define AK0991X_TARGET_AK09916C           //

//It depends on the target device,
//AK09913, AK09916C and AK09918 don't support these settings.
#ifdef  AK0991X_TARGET_AK09911
#define AK0991X_ENABLE_FUSE

#elif   AK0991X_TARGET_AK09912
#define AK0991X_ENABLE_FUSE
#define AK0991X_ENABLE_DRI

#elif   AK0991X_TARGET_AK09915C
#define AK0991X_ENABLE_DRI
#define AK0991X_ENABLE_FIFO

#elif   AK0991X_TARGET_AK09915D
#define AK0991X_ENABLE_DRI
#define AK0991X_ENABLE_FIFO
#define AK0991X_ENABLE_S4S

#elif   AK0991X_TARGET_AK09916D
#define AK0991X_ENABLE_DRI

#elif   AK0991X_TARGET_AK09917
#define AK0991X_ENABLE_DRI
#define AK0991X_ENABLE_FIFO
#define AK0991X_ENABLE_S4S

#endif

#else

#define AK0991X_ENABLE_REGISTRY_ACCESS    // Enable registry access
#define AK0991X_ENABLE_ALL_ATTRIBUTES     // Enable all attribute service
#define AK0991X_ENABLE_DEBUG_MSG          // Enable debug messages
#define AK0991X_ENABLE_DAE                // Enable DAE
#define AK0991X_ENABLE_DIAG_LOGGING       // Enable diagnostic logging
#define AK0991X_ENABLE_POWER_RAIL         // Enable power rail reference
#define AK0991X_ENABLE_DEINIT             // Enable deinit call
#define AK0991X_ENABLE_S4S                // Enable S4S parts


#define AK0991X_ENABLE_FUSE               // Enable fuse rom
#define AK0991X_ENABLE_DRI                // Enable DRI
#define AK0991X_ENABLE_FIFO               // Enable FIFO

#endif	// AK0991X_ENABLE_SEE_LITE
