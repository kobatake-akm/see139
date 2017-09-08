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
//#define AK0991X_ENABLE_SEE_LITE_MODE			// Enable SEE-Lite mode

#ifdef AK0991X_ENABLE_SEE_LITE_MODE

#define AK0991X_DAE_FORCE_NOT_AVAILABLE

#else

#define AK0991X_ENABLE_DEBUG_MSG				  // Enable debug messages
#define AK0991X_VERBOSE_DEBUG           	// Define to enable extra debugging
#define AK0991X_ENABLE_S4S								// Enable S4S parts

#endif	// AK0991X_ENABLE_SEE_LITE_MODE


