#pragma once
/**
 * @file sns_self_test_sensor_instance.h
 *
 * Self Test Sensor Instance.
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * $Id:  $
 * $DateTime:  $
 * $Change:  $
 *
 **/

#include <stdint.h>
#include "sns_event_service.h"
#include "sns_sensor_instance.h"

/** Forward Declaration of Sensor API */
sns_sensor_instance_api sns_self_test_sensor_instance_api;

typedef struct sns_self_test_instance_state
{
  /** Event Service handle. This is acquired during test
   *  registration. */
  sns_event_service     *event_service;

} sns_self_test_instance_state;

