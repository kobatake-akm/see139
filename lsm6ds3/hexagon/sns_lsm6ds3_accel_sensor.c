/**
 * @file sns_lsm6ds3_accel_sensor.c
 *
 * LSM6DS3 Accel virtual Sensor implementation.
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
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
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};

    sns_std_attr_value_data range1[] = {SNS_ATTR, SNS_ATTR};
    range1[0].has_flt = true;
    range1[0].flt = LSM6DS3_ACCEL_RANGE_2G_MIN;
    range1[1].has_flt = true;
    range1[1].flt = LSM6DS3_ACCEL_RANGE_2G_MAX;
    values[0].has_subtype = true;
    values[0].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
    values[0].subtype.values.arg =
      &((pb_buffer_arg){ .buf = range1, .buf_len = ARR_SIZE(range1) });

    sns_std_attr_value_data range2[] = {SNS_ATTR, SNS_ATTR};
    range2[0].has_flt = true;
    range2[0].flt = LSM6DS3_ACCEL_RANGE_4G_MIN;
    range2[1].has_flt = true;
    range2[1].flt = LSM6DS3_ACCEL_RANGE_4G_MAX;
    values[1].has_subtype = true;
    values[1].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
    values[1].subtype.values.arg =
      &((pb_buffer_arg){ .buf = range2, .buf_len = ARR_SIZE(range2) });

    sns_std_attr_value_data range3[] = {SNS_ATTR, SNS_ATTR};
    range3[0].has_flt = true;
    range3[0].flt = LSM6DS3_ACCEL_RANGE_8G_MIN;
    range3[1].has_flt = true;
    range3[1].flt = LSM6DS3_ACCEL_RANGE_8G_MAX;
    values[2].has_subtype = true;
    values[2].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
    values[2].subtype.values.arg =
      &((pb_buffer_arg){ .buf = range3, .buf_len = ARR_SIZE(range3) });

    sns_std_attr_value_data range4[] = {SNS_ATTR, SNS_ATTR};
    range4[0].has_flt = true;
    range4[0].flt = LSM6DS3_ACCEL_RANGE_16G_MIN;
    range4[1].has_flt = true;
    range4[1].flt = LSM6DS3_ACCEL_RANGE_16G_MAX;
    values[3].has_subtype = true;
    values[3].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
    values[3].subtype.values.arg =
      &((pb_buffer_arg){ .buf = range4, .buf_len = ARR_SIZE(range4) });
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
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR,
        SNS_ATTR, SNS_ATTR};
    values[0].has_flt = true;
    values[0].flt = LSM6DS3_ODR_13;
    values[1].has_flt = true;
    values[1].flt = LSM6DS3_ODR_26;
    values[2].has_flt = true;
    values[2].flt = LSM6DS3_ODR_52;
    values[3].has_flt = true;
    values[3].flt = LSM6DS3_ODR_104;
    values[4].has_flt = true;
    values[4].flt = LSM6DS3_ODR_208;
    values[5].has_flt = true;
    values[5].flt = LSM6DS3_ODR_416;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RATES,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};
    values[0].has_flt = true;
    values[0].flt = LSM6DS3_ACCEL_RESOLUTION_2G;
    values[1].has_flt = true;
    values[1].flt = LSM6DS3_ACCEL_RESOLUTION_4G;
    values[2].has_flt = true;
    values[2].flt = LSM6DS3_ACCEL_RESOLUTION_8G;
    values[3].has_flt = true;
    values[3].flt = LSM6DS3_ACCEL_RESOLUTION_16G; //mg/LSB
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
    char const proto1[] = "sns_accel.proto";
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
    char const type[] = "accel";
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
    value.sint = 0;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_HW_ID, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = LSM6DS3_MAX_FIFO; // samples
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
sns_rc lsm6ds3_accel_init(sns_sensor *const this)
{
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;
  uint8_t i = 0;

  struct sns_service_manager *smgr= this->cb->get_service_manager(this);
  state->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);
  state->scp_service = (sns_sync_com_port_service *)smgr->get_service(smgr, SNS_SYNC_COM_PORT_SERVICE);

  state->sensor = LSM6DS3_ACCEL;
  state->sensor_client_present = false;

  sns_memscpy(&state->my_suid,
              sizeof(state->my_suid),
              &((sns_sensor_uid)ACCEL_SUID),
              sizeof(sns_sensor_uid));

  // initialize axis conversion settings
  for(i = 0; i < TRIAXIS_NUM; i++)
  {
    state->axis_map[i].opaxis = i;
    state->axis_map[i].ipaxis = i;
    state->axis_map[i].invert = false;
  }

  // initialize fac cal correction matrix to identity
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

sns_rc lsm6ds3_accel_deinit(sns_sensor *const this)
{
  UNUSED_VAR(this);
  return SNS_RC_SUCCESS;
}

