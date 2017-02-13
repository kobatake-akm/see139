/**
 * @file sns_test.c
 *
 * Copyright (c) 2016 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * $Id: //components/rel/ssc.slpi/3.0/sensors/test/src/sns_test.c#1 $
 * $DateTime: 2016/11/30 12:38:10 $
 * $Change: 11880841 $
 *
 **/
#include "sns_rc.h"
#include "sns_register.h"
#include "sns_test_sensor.h"
#include "sns_test_sensor_instance.h"

/** Public Function Definitions. */

sns_rc sns_test_register(sns_register_cb const *register_api)
{
  register_api->init_sensor(sizeof(sns_test_state), &sns_test_sensor_api,
      &sns_test_sensor_instance_api);

  return SNS_RC_SUCCESS;
}
