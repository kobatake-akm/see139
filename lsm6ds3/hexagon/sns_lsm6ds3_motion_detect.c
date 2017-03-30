/**
 * @file sns_lsm6ds3_motion_detect.c
 *
 * LSM6DS3 Motion Detect Sensor implementation.
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc. All Rights
 * Reserved. Confidential and Proprietary - Qualcomm
 * Technologies, Inc.
 **/

#include <string.h>
#include "sns_mem_util.h"
#include "sns_math_util.h"
#include "sns_types.h"
#include "sns_lsm6ds3_sensor.h"
#include "pb_encode.h"
#include "sns_service_manager.h"
#include "sns_attribute_util.h"
#include "sns_pb_util.h"

/**
 * Publish all Sensor attributes
 *
 * @param[i] this    reference to this Sensor
 *
 * @return none
 */
static void
lsm6ds3_publish_attributes(sns_sensor *const this)
{
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    values[0].has_sint = true;
    values[0].sint = SNS_STD_SENSOR_STREAM_TYPE_SINGLE_OUTPUT;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_STREAM_TYPE,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR,
      SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};
    values[0].has_flt = true;
    values[0].flt = 0;
    values[1].has_flt = true;
    values[1].flt = 0;
    values[2].has_flt = true;
    values[2].flt = 0;
    values[3].has_flt = true;
    values[3].flt = 0;
    values[4].has_flt = true;
    values[4].flt = 0;
    values[5].has_flt = true;
    values[5].flt = 0;
    values[6].has_flt = true;
    values[6].flt = 0;
    values[7].has_flt = true;
    values[7].flt = 0;
    values[8].has_flt = true;
    values[8].flt = 0;
    values[9].has_flt = true;
    values[9].flt = 0;
    values[10].has_flt = true;
    values[10].flt = 0;
    values[11].has_flt = true;
    values[11].flt = 0;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_PLACEMENT,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    values[0].has_sint = true;
    values[0].sint = 24;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_ACTIVE_CURRENT,
        values, ARR_SIZE(values), false);
  }

  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    char const op_mode1[] = LSM6DS3_HIGH_PERF;

    values[0].str.funcs.encode = pb_encode_string_cb;
    values[0].str.arg = &((pb_buffer_arg)
        { .buf = op_mode1, .buf_len = sizeof(op_mode1) });
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_OP_MODES,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    char const proto1[] = "sns_physical_sensor_test.proto";
    char const proto2[] = "sns_motion_detect.proto";

    values[0].str.funcs.encode = pb_encode_string_cb;
    values[0].str.arg = &((pb_buffer_arg)
        { .buf = proto1, .buf_len = sizeof(proto1) });
    values[1].str.funcs.encode = pb_encode_string_cb;
    values[1].str.arg = &((pb_buffer_arg)
        { .buf = proto2, .buf_len = sizeof(proto2) });
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_API,
        values, ARR_SIZE(values), false);
  }
  {
    char const name[] = "stm_lsm6ds3";
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = name, .buf_len = sizeof(name) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_NAME, &value, 1, false);
  }
  {
    char const type[] = "motion_detect";
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = type, .buf_len = sizeof(type) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
  }
  {
    char const vendor[] = "template";
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = vendor, .buf_len = sizeof(vendor) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_VENDOR, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = false;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = false;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_DYNAMIC, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = true;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_DRI, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = false;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_STREAM_SYNC, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = 0x0100;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_VERSION, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = SNS_STD_SENSOR_RIGID_BODY_TYPE_DISPLAY;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_RIGID_BODY, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = 0;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_HW_ID, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = 0;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_FIFO_SIZE, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = 6; //uA
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_SLEEP_CURRENT, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    sns_motion_detect_event stream_event =
      sns_motion_detect_event_init_default;
    pb_get_encoded_size(&state->encoded_event_len,
                        sns_motion_detect_event_fields,
                        &stream_event);
    value.has_sint = true;
    value.sint = state->encoded_event_len;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_EVENT_SIZE, &value, 1, true);
  }
}

/* See sns_sensor::init */
sns_rc lsm6ds3_motion_detect_init(sns_sensor *const this)
{
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;

  state->sensor = LSM6DS3_MOTION_DETECT;
  state->sensor_client_present = false;
  sns_service_manager *smgr= this->cb->get_service_manager(this);
  state->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);
  state->scp_service = (sns_sync_com_port_service*)
		smgr->get_service(smgr, SNS_SYNC_COM_PORT_SERVICE);

  sns_memscpy(&state->my_suid,
              sizeof(state->my_suid),
              &((sns_sensor_uid)MOTION_DETECT_SUID),
              sizeof(sns_sensor_uid));

  lsm6ds3_publish_attributes(this);

  lsm6ds3_send_suid_req(this, "interrupt", 10);
  lsm6ds3_send_suid_req(this, "async_com_port", 15);
  lsm6ds3_send_suid_req(this, "timer", 6);
#if LSM6DS3_ENABLE_DEPENDENCY
  lsm6ds3_send_suid_req(this, "registry", 9);
#endif //

  return SNS_RC_SUCCESS;
}

sns_rc lsm6ds3_motion_detect_deinit(sns_sensor *const this)
{
  UNUSED_VAR(this);
  return SNS_RC_SUCCESS;
}

