/**
 * @file sns_ak0991x_magnetic_sensor.c
 *
 * AK0991X Magnetic virtual Sensor implementation.
 *
 * Copyright (c) 2016-2017 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 **/

#include <string.h>
#include "sns_mem_util.h"
#include "sns_types.h"
#include "sns_service_manager.h"
#include "sns_ak0991x_sensor.h"
#include "sns_pb_util.h"
#include "sns_attribute_util.h"

/**
 * Initialize attributes to their default state.  They may/will be updated
 * within notify_event.
 */
void ak0991x_publish_default_attributes(sns_sensor *const this)
{
  ak0991x_state *state = (ak0991x_state *)this->state->state;
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    values[0].has_sint = true;
    values[0].sint = SNS_STD_SENSOR_STREAM_TYPE_STREAMING;
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
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR};
    char const proto1[] = "sns_physical_sensor_test.proto";
    char const proto2[] = "sns_std_sensor.proto";

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
    char const name[] = "ak0991x";
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = name, .buf_len = sizeof(name) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_NAME, &value, 1, false);
  }
  {
    char const type[] = "mag";
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = type, .buf_len = sizeof(type) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
  }
  {
    char const vendor[] = "akm";
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
        this, SNS_STD_SENSOR_ATTRID_DYNAMIC, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = 0x00000100;
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
    float data[3] = {0};
    state->encoded_event_len = pb_get_encoded_size_sensor_stream_event(data, 3);
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = state->encoded_event_len;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_EVENT_SIZE, &value, 1, true);
  }
}

/* See sns_sensor::get_sensor_uid */
static sns_sensor_uid const *ak0991x_mag_get_sensor_uid(sns_sensor const *const this)
{
  UNUSED_VAR(this);
  static const sns_sensor_uid sensor_uid = MAG_SUID;

  return &sensor_uid;
}

/* See sns_sensor::init */
static sns_rc ak0991x_mag_init(sns_sensor *const this)
{
  ak0991x_state *state = (ak0991x_state *)this->state->state;

  struct sns_service_manager *smgr = this->cb->get_service_manager(this);
  state->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);
  state->scp_service =
     (sns_sync_com_port_service *)smgr->get_service(smgr, SNS_SYNC_COM_PORT_SERVICE);

  state->sensor_client_present = false;

  sns_memscpy(&state->my_suid,
              sizeof(state->my_suid),
              ak0991x_mag_get_sensor_uid(this),
              sizeof(sns_sensor_uid));
  ak0991x_publish_default_attributes(this);
  ak0991x_send_suid_req(this, "interrupt", 10);
  ak0991x_send_suid_req(this, "async_com_port", 15);
  ak0991x_send_suid_req(this, "timer", 6);
#if AK0991X_ENABLE_DEPENDENCY
  ak0991x_send_suid_req(this, "registry", 9);
#endif //

  return SNS_RC_SUCCESS;
}

/** See sns_sensor.h */
static sns_rc ak0991x_mag_deinit(sns_sensor *const this)
{
  UNUSED_VAR(this);
  return SNS_RC_SUCCESS;
}

/*===========================================================================
  Public Data Definitions
  ===========================================================================*/

sns_sensor_api ak0991x_mag_sensor_api = {
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &ak0991x_mag_init,
  .deinit             = &ak0991x_mag_deinit,
  .get_sensor_uid     = &ak0991x_mag_get_sensor_uid,
  .set_client_request = &ak0991x_set_client_request,
  .notify_event       = &ak0991x_sensor_notify_event,
};
