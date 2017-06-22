/**
 * @file sns_self_test.c
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
#include "sns_rc.h"
#include "sns_register.h"
#include "sns_self_test_sensor.h"
#include "sns_self_test_sensor_instance.h"

/** Public Function Definitions. */

sns_rc sns_self_test_register(sns_register_cb const *register_api)
{
  register_api->init_sensor(sizeof(sns_self_test_state), &sns_self_test_sensor_api,
      &sns_self_test_sensor_instance_api);

  return SNS_RC_SUCCESS;
}
