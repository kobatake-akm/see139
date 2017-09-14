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
#include "sns_ak0991x_ver.h"
#include "sns_pb_util.h"
#include "sns_attribute_util.h"
#include "sns_printf.h"

/**
 * Initialize attributes to their default state.  They may/will be updated
 * within notify_event.
 */
static void ak0991x_publish_default_attributes(sns_sensor *const this)
{
#ifdef  AK0991X_ENABLE_ALL_ATTRIBUTES
  ak0991x_state *state = (ak0991x_state *)this->state->state;
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    values[0].has_sint = true;
    values[0].sint = SNS_STD_SENSOR_STREAM_TYPE_STREAMING;
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_STREAM_TYPE,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR};
    char const proto1[] = "sns_mag.proto";
    values[0].str.funcs.encode = pb_encode_string_cb;
    values[0].str.arg = &((pb_buffer_arg)
        { .buf = proto1, .buf_len = sizeof(proto1) });
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
#endif
  {
    char const type[] = "mag";
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = type, .buf_len = sizeof(type) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
  }
#ifdef  AK0991X_ENABLE_ALL_ATTRIBUTES
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
    value.sint = AK0991X_DRIVER_VERSION;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_VERSION, &value, 1, false);
  }
  {
    float data[3] = {0};
    state->encoded_event_len = pb_get_encoded_size_sensor_stream_event(data, 3);
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = state->encoded_event_len;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_EVENT_SIZE, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = true;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_PHYSICAL_SENSOR, &value, 1, true);
  }
#endif
}


/* See sns_sensor::init */
sns_rc ak0991x_mag_init(sns_sensor *const this)
{
  ak0991x_state *state = (ak0991x_state *)this->state->state;
  uint8_t i = 0;

  struct sns_service_manager *smgr = this->cb->get_service_manager(this);
  state->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);
  state->scp_service =
     (sns_sync_com_port_service *)smgr->get_service(smgr, SNS_SYNC_COM_PORT_SERVICE);

  ak0991x_publish_default_attributes(this);

  AK0991X_PRINT(LOW, this, "ak0991x: init");

  state->hw_is_present = false;
  state->sensor_client_present = false;

  sns_memscpy(&state->my_suid,
              sizeof(state->my_suid),
              &((sns_sensor_uid)MAG_SUID),
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



#ifndef AK0991X_ENABLE_REGISTRY_ACCESS
  AK0991X_PRINT(ERROR, this, "Read Hardcode for AK09916C");
  state->com_port_info.com_config.bus_type = 0;
  state->com_port_info.com_config.bus_instance = 3;
  state->com_port_info.com_config.slave_control = 12;
  state->com_port_info.com_config.min_bus_speed_KHz = 400;
  state->com_port_info.com_config.max_bus_speed_KHz = 400;
  state->com_port_info.com_config.reg_addr_type = 0;
  state->irq_config.interrupt_num = 119;
  state->irq_config.interrupt_pull_type = 3;
  state->irq_config.is_chip_pin = 1;
  state->irq_config.interrupt_drive_strength = 0;
  state->irq_config.interrupt_trigger_type = 1;
  state->rail_config.num_of_rails = 1;
  state->registry_rail_on_state = 1;
  sns_strlcpy(state->rail_config.rails[0].name,
      "/pmic/client/sensor_vddio",
              sizeof(state->rail_config.rails[0].name));
  sns_strlcpy(state->rail_config.rails[1].name,
      "/pmic/client/sensor_vddio",
              sizeof(state->rail_config.rails[1].name));

  state->axis_map[0].ipaxis = TRIAXIS_X;
  state->axis_map[0].opaxis = TRIAXIS_X;
  state->axis_map[0].invert = false;
  state->axis_map[1].ipaxis = TRIAXIS_Y;
  state->axis_map[1].opaxis = TRIAXIS_Y;
  state->axis_map[1].invert = false;
  state->axis_map[2].ipaxis = TRIAXIS_Z;
  state->axis_map[2].opaxis = TRIAXIS_Z;
  state->axis_map[2].invert = false;

  state->fac_cal_bias[0] = 0.0;
  state->fac_cal_bias[1] = 0.0;
  state->fac_cal_bias[2] = 0.0;
  state->fac_cal_scale[0] = 0.0;
  state->fac_cal_scale[1] = 0.0;
  state->fac_cal_scale[2] = 0.0;
  state->fac_cal_corr_mat.e00 = 1.0;
  state->fac_cal_corr_mat.e01 = 0.0;
  state->fac_cal_corr_mat.e02 = 0.0;
  state->fac_cal_corr_mat.e10 = 0.0;
  state->fac_cal_corr_mat.e11 = 1.0;
  state->fac_cal_corr_mat.e12 = 0.0;
  state->fac_cal_corr_mat.e20 = 0.0;
  state->fac_cal_corr_mat.e21 = 0.0;
  state->fac_cal_corr_mat.e22 = 1.0;

  state->placement[0] = 0.0;
  state->placement[1] = 0.0;
  state->placement[2] = 0.0;
  state->placement[3] = 0.0;
  state->placement[4] = 0.0;
  state->placement[5] = 0.0;
  state->placement[6] = 0.0;
  state->placement[7] = 0.0;
  state->placement[8] = 0.0;
  state->placement[9] = 0.0;
  state->placement[10] = 0.0;
  state->placement[11] = 0.0;

  state->use_fifo = false;
  state->nsf = 0;
  state->sdr = 0;

  state->is_dri = false;
  state->resolution_idx = 0;
  state->hardware_id = 0;
  state->supports_sync_stream = false;
#endif

  ak0991x_send_suid_req(this, "data_acquisition_engine", sizeof("data_acquisition_engine"));
  ak0991x_send_suid_req(this, "interrupt", sizeof("interrupt"));
  ak0991x_send_suid_req(this, "async_com_port", sizeof("async_com_port"));
  ak0991x_send_suid_req(this, "timer", sizeof("timer"));
#ifdef AK0991X_ENABLE_REGISTRY_ACCESS
  ak0991x_send_suid_req(this, "registry", sizeof("registry"));
#endif

  return SNS_RC_SUCCESS;
}

#ifdef AK0991X_ENABLE_DEINIT
/** See sns_sensor.h */
sns_rc ak0991x_mag_deinit(sns_sensor *const this)
{
  UNUSED_VAR(this);
  return SNS_RC_SUCCESS;
}
#endif
