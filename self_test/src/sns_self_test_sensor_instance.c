/**
 * @file sns_self_test_sensor_instance.c
 *
 * The self test Sensor Instance implementation
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc. All Rights
 * Reserved. Confidential and Proprietary - Qualcomm
 * Technologies, Inc.
 *
 * $Id:  $
 * $DateTime:  $
 * $Change:  $
 *
 **/

#include "sns_mem_util.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_sensor_event.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_self_test_sensor.h"
#include "sns_self_test_sensor_instance.h"
#include "sns_time.h"
#include "sns_types.h"

/** See sns_sensor_instance_api::notify_event */
static sns_rc sns_self_test_inst_notify_event(sns_sensor_instance *const this)
{
  UNUSED_VAR(this);
  return SNS_RC_SUCCESS;
}

/** See sns_sensor_instance_api::init */
static sns_rc sns_self_test_inst_init(sns_sensor_instance *const this,
                                 sns_sensor_state const *state)
{
  UNUSED_VAR(this);
  UNUSED_VAR(state);
  return SNS_RC_SUCCESS;
}

/** See sns_sensor_instance_api::set_client_config */
static sns_rc sns_self_test_inst_set_client_config(sns_sensor_instance *const this,
                                             sns_request const *client_request)
{
  UNUSED_VAR(this);
  UNUSED_VAR(client_request);
  return SNS_RC_SUCCESS;
}

static sns_rc sns_self_test_inst_deinit(sns_sensor_instance *const this)
{
  UNUSED_VAR(this);
  return SNS_RC_SUCCESS;
}

/** Public Data Definitions. */

sns_sensor_instance_api sns_self_test_sensor_instance_api =
{
  .struct_len        = sizeof(sns_sensor_instance_api),
  .init              = &sns_self_test_inst_init,
  .deinit            = &sns_self_test_inst_deinit,
  .set_client_config = &sns_self_test_inst_set_client_config,
  .notify_event      = &sns_self_test_inst_notify_event
};

