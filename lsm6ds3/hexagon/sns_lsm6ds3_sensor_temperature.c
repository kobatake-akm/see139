/**
 * @file sns_lsm6ds3_sensor_temperature.c
 *
 * LSM6DS3 Sensor Temperature virtual Sensor implementation.
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc. All Rights
 * Reserved. Confidential and Proprietary - Qualcomm
 * Technologies, Inc.
 **/

#include <string.h>
#include "pb_encode.h"
#include "sns_attribute_util.h"
#include "sns_lsm6ds3_sensor.h"
#include "sns_mem_util.h"
#include "sns_pb_util.h"
#include "sns_service_manager.h"
#include "sns_types.h"

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

    sns_std_attr_value_data range1[] = {SNS_ATTR, SNS_ATTR};
    range1[0].has_flt = true;
    range1[0].flt = LSM6DS3_SENSOR_TEMPERATURE_RANGE_MIN;
    range1[1].has_flt = true;
    range1[1].flt = LSM6DS3_SENSOR_TEMPERATURE_RANGE_MAX;
    values[0].has_subtype = true;
    values[0].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
    values[0].subtype.values.arg =
      &((pb_buffer_arg){ .buf = range1, .buf_len = ARR_SIZE(range1) });

    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RANGES,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    values[0].has_sint = true;
    values[0].sint = SNS_STD_SENSOR_STREAM_TYPE_STREAMING;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_STREAM_TYPE,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR};
    values[0].has_sint = true;
    values[0].sint = 24;
    values[1].has_sint = true;
    values[1].sint = 70;
    values[2].has_sint = true;
    values[2].sint = 240; //uA
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_ACTIVE_CURRENT,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR};
    values[0].has_flt = true;
    values[0].flt = LSM6DS3_SENSOR_TEMP_ODR_1;
    values[1].has_flt = true;
    values[1].flt = LSM6DS3_SENSOR_TEMP_ODR_5;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RATES,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    values[0].has_flt = true;
    values[0].flt = LSM6DS3_SENSOR_TEMPERATURE_RESOLUTION;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RESOLUTIONS,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR};
    char const op_mode1[] = LSM6DS3_LPM;
    char const op_mode2[] = LSM6DS3_NORMAL;
    char const op_mode3[] = LSM6DS3_HIGH_PERF;

    values[0].str.funcs.encode = pb_encode_string_cb;
    values[0].str.arg = &((pb_buffer_arg)
        { .buf = op_mode1, .buf_len = sizeof(op_mode1) });
    values[1].str.funcs.encode = pb_encode_string_cb;
    values[1].str.arg = &((pb_buffer_arg)
        { .buf = op_mode2, .buf_len = sizeof(op_mode2) });
    values[2].str.funcs.encode = pb_encode_string_cb;
    values[2].str.arg = &((pb_buffer_arg)
        { .buf = op_mode3, .buf_len = sizeof(op_mode3) });
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_OP_MODES,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    char const proto1[] = "sns_sensor_temperature.proto";
    values[0].str.funcs.encode = pb_encode_string_cb;
    values[0].str.arg = &((pb_buffer_arg)
        { .buf = proto1, .buf_len = sizeof(proto1) });
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
    char const type[] = "sensor_temperature";
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
    value.boolean = false;
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
    float data[3] = {0};
    state->encoded_event_len =
        pb_get_encoded_size_sensor_stream_event(data, 3);
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = state->encoded_event_len;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_EVENT_SIZE, &value, 1, true);
  }
}

/* See sns_sensor::init */
sns_rc lsm6ds3_sensor_temp_init(sns_sensor *const this)
{
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;

  struct sns_service_manager *smgr= this->cb->get_service_manager(this);
  state->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);
  state->scp_service = (sns_sync_com_port_service*)
		smgr->get_service(smgr, SNS_SYNC_COM_PORT_SERVICE);

  state->sensor = LSM6DS3_SENSOR_TEMP;
  state->sensor_client_present = false;

  sns_memscpy(&state->my_suid,
              sizeof(state->my_suid),
              &((sns_sensor_uid)SENSOR_TEMPERATURE_SUID),
              sizeof(sns_sensor_uid));

  // initialize fac cal correction matrix to identity
  // sensor temperature only uses
  // state->fac_cal_corr_mat.e00 for scaling and 
  // state->fac_cal_bias[0] for bias correction
  // of it's single axis output
  state->fac_cal_corr_mat.e00 = 1.0;
  state->fac_cal_corr_mat.e11 = 1.0;
  state->fac_cal_corr_mat.e22 = 1.0;

  lsm6ds3_publish_attributes(this);

  lsm6ds3_send_suid_req(this, "data_acquisition_engine", sizeof("data_acquisition_engine"));
  lsm6ds3_send_suid_req(this, "interrupt", 10);
  lsm6ds3_send_suid_req(this, "async_com_port", 15);
  lsm6ds3_send_suid_req(this, "timer", 6);
  lsm6ds3_send_suid_req(this, "registry", 9);

  return SNS_RC_SUCCESS;
}

sns_rc lsm6ds3_sensor_temp_deinit(sns_sensor *const this)
{
  UNUSED_VAR(this);
  return SNS_RC_SUCCESS;
}

