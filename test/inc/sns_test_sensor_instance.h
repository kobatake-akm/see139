#pragma once
/**
 * @file sns_test_sensor_instance.h
 *
 * Test Sensor Instance.
 *
 * Copyright (c) 2016 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * $Id: //components/rel/ssc.slpi/3.0/sensors/test/inc/sns_test_sensor_instance.h#1 $
 * $DateTime: 2016/11/30 12:38:10 $
 * $Change: 11880841 $
 *
 **/

#include <stdint.h>
#include "sns_sensor_instance.h"
#include "sns_event_service.h"

/** Forward Declaration of Sensor API */
sns_sensor_instance_api sns_test_sensor_instance_api;

typedef struct sns_test_instance_state
{
  /** Event Service handle. This is acquired during test
   *  registration. */
  sns_event_service     *event_service;

} sns_test_instance_state;

