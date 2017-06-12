#pragma once
/**
 * @file sns_test_sensor_instance.h
 *
 * Test Sensor Instance.
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * $Id: //components/dev/ssc.slpi/3.0/maansyw.ssc.slpi.3.0.napali_06_11/sensors/test/inc/sns_test_sensor_instance.h#1 $
 * $DateTime: 2017/06/11 12:38:13 $
 * $Change: 13546828 $
 *
 **/

#include <stdint.h>
#include "sns_event_service.h"
#include "sns_sensor_instance.h"

/** Forward Declaration of Sensor API */
sns_sensor_instance_api sns_test_sensor_instance_api;

typedef struct sns_test_instance_state
{
  /** Event Service handle. This is acquired during test
   *  registration. */
  sns_event_service     *event_service;

} sns_test_instance_state;

sns_rc sns_test_inst_init(sns_sensor_instance *const this,
                          sns_sensor_state const *state);
sns_rc sns_test_inst_deinit(sns_sensor_instance *const this);