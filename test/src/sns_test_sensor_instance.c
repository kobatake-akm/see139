/**
 * @file sns_test_sensor_instance.c
 *
 * The Interrupt virtual Sensor Instance implementation
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * $Id: //components/rel/ssc.slpi/3.0/sensors/test/src/sns_test_sensor_instance.c#8 $
 * $DateTime: 2017/05/16 15:59:42 $
 * $Change: 13327771 $
 *
 **/

#include "sns_rc.h"
#include "sns_sensor_instance.h"
#include "sns_test_sensor.h"
#include "sns_test_sensor_instance.h"
#include "sns_types.h"

/** See sns_sensor_instance_api::init */
sns_rc sns_test_inst_init(sns_sensor_instance *const this,
                                 sns_sensor_state const *state)
{
  UNUSED_VAR(this);
  UNUSED_VAR(state);
  return SNS_RC_SUCCESS;
}

sns_rc sns_test_inst_deinit(sns_sensor_instance *const this)
{
  UNUSED_VAR(this);
  return SNS_RC_SUCCESS;
}

