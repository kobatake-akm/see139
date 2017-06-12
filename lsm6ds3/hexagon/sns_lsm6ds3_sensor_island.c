/**
 * @file sns_lsm6ds3_sensor_island.c
 *
 * Common implementation for LSM6DS3 Sensors for island mode.
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include <string.h>
#include "sns_attribute_util.h"
#include "sns_diag_service.h"
#include "sns_event_service.h"
#include "sns_math_util.h"
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_sensor_util.h"
#include "sns_service.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_sync_com_port_service.h"
#include "sns_types.h"

#include "sns_lsm6ds3_hal.h"
#include "sns_lsm6ds3_sensor.h"

#include "pb_decode.h"
#include "pb_encode.h"
#include "sns_motion_detect.pb.h"
#include "sns_pb_util.h"
#include "sns_std.pb.h"
#include "sns_std_event_gated_sensor.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"
#include "sns_registry.pb.h"
#include "sns_printf.h"

static sns_sensor_uid const* lsm6ds3_accel_get_sensor_uid(sns_sensor const *const this)
{
  UNUSED_VAR(this);
  static const sns_sensor_uid sensor_uid = ACCEL_SUID;

  return &sensor_uid;
}

static sns_sensor_uid const* lsm6ds3_gyro_get_sensor_uid(sns_sensor const *const this)
{
  UNUSED_VAR(this);
  static const sns_sensor_uid sensor_uid = GYRO_SUID;

  return &sensor_uid;
}

static sns_sensor_uid const* lsm6ds3_motion_detect_get_sensor_uid(sns_sensor const *const this)
{
  UNUSED_VAR(this);
  static const sns_sensor_uid sensor_uid = MOTION_DETECT_SUID;

  return &sensor_uid;
}

static sns_sensor_uid const* lsm6ds3_sensor_temp_get_sensor_uid(sns_sensor const *const this)
{
  UNUSED_VAR(this);
  static const sns_sensor_uid sensor_uid = SENSOR_TEMPERATURE_SUID;

  return &sensor_uid;
}

static void
lsm6ds3_publish_available(sns_sensor *const this)
{
  sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
  value.has_boolean = true;
  value.boolean = true;
  sns_publish_attribute(
      this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, true);
}

/**
 * Publish attributes read from registry
 *
 * @param[i] this    reference to this Sensor
 *
 * @return none
 */
static void
lsm6ds3_publish_registry_attributes(sns_sensor *const this)
{
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = state->is_dri;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_DRI, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = state->supports_sync_stream;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_STREAM_SYNC, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = state->hardware_id;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_HW_ID, &value, 1, false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR,
      SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};
    for(uint8_t i = 0; i < 12; i++)
    {
      values[i].has_flt = true;
      values[i].flt = state->placement[i];
    }
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_PLACEMENT,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = state->registry_pf_cfg.rigid_body_type;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_RIGID_BODY, &value, 1, false);
  }
}

static void lsm6ds3_start_power_rail_timer(sns_sensor *const this,
                                           sns_time timeout_ticks,
                                           lsm6ds3_power_rail_pending_state pwr_rail_pend_state)
{
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;

  sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
  size_t req_len;
  uint8_t buffer[20];
  sns_memset(buffer, 0, sizeof(buffer));
  req_payload.is_periodic = false;
  req_payload.start_time = sns_get_system_time();
  req_payload.timeout_period = timeout_ticks;

  req_len = pb_encode_request(buffer, sizeof(buffer), &req_payload,
                              sns_timer_sensor_config_fields, NULL);
  if(req_len > 0)
  {
    sns_request timer_req =
      {  .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
         .request = buffer, .request_len = req_len  };
    state->timer_stream->api->send_request(state->timer_stream, &timer_req);
    state->power_rail_pend_state = pwr_rail_pend_state;
  }
  else
  {
    SNS_PRINTF(ERROR, this, "LSM timer req encode error");
  }
}

static void lsm6ds3_sensor_process_registry_event(sns_sensor *const this,
                                                  sns_sensor_event *event)
{
  bool rv = true;
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;

  pb_istream_t stream = pb_istream_from_buffer((void*)event->event,
      event->event_len);

  if(SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_EVENT == event->message_id)
  {
    sns_registry_read_event read_event = sns_registry_read_event_init_default;
    pb_buffer_arg group_name = {0,0};
    read_event.name.arg = &group_name;
    read_event.name.funcs.decode = pb_decode_string_cb;

    if(!pb_decode(&stream, sns_registry_read_event_fields, &read_event))
    {
      SNS_PRINTF(ERROR, this, "Error decoding registry event");
    }
    else
    {
      stream = pb_istream_from_buffer((void*)event->event, event->event_len);

      if(0 == strncmp((char*)group_name.buf, "lsm6ds3_0.accel.config",
                      group_name.buf_len) ||
         0 == strncmp((char*)group_name.buf, "lsm6ds3_0.gyro.config",
                      group_name.buf_len) ||
         0 == strncmp((char*)group_name.buf, "lsm6ds3_0.temp.config",
                      group_name.buf_len) ||
         0 == strncmp((char*)group_name.buf, "lsm6ds3_0.md.config",
                      group_name.buf_len))
      {
        {
          sns_registry_decode_arg arg = {
            .item_group_name = &group_name,
            .parse_info_len = 1,
            .parse_info[0] = {
              .group_name = "config",
              .parse_func = sns_registry_parse_phy_sensor_cfg,
              .parsed_buffer = &state->registry_cfg
            }
          };

          read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
          read_event.data.items.arg = &arg;

          rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
        }

        if(rv)
        {
          state->registry_cfg_received = true;
          state->is_dri = state->registry_cfg.is_dri;
          state->hardware_id = state->registry_cfg.hw_id;
          state->resolution_idx = state->registry_cfg.res_idx;
          state->supports_sync_stream = state->registry_cfg.sync_stream;

          SNS_PRINTF(LOW, this, "Registry read event for group %s received "
                                   "is_dri:%d, hardware_id:%lld ",
                                   (char*)group_name.buf,
                                   state->is_dri,
                                   state->hardware_id);
          SNS_PRINTF(LOW, this, "resolution_idx:%d, supports_sync_stream:%d ",
                                   state->resolution_idx,
                                   state->supports_sync_stream);
        }
      }
      else if(0 == strncmp((char*)group_name.buf, "lsm6ds3_0_platform.config",
                           group_name.buf_len))
      {
        {
          sns_registry_decode_arg arg = {
            .item_group_name = &group_name,
            .parse_info_len = 1,
            .parse_info[0] = {
              .group_name = "config",
              .parse_func = sns_registry_parse_phy_sensor_pf_cfg,
              .parsed_buffer = &state->registry_pf_cfg
            }
          };

          read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
          read_event.data.items.arg = &arg;

          rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
        }

        if(rv)
        {
          state->registry_pf_cfg_received = true;

          state->com_port_info.com_config.bus_type = state->registry_pf_cfg.bus_type;
          state->com_port_info.com_config.bus_instance = state->registry_pf_cfg.bus_instance;
          state->com_port_info.com_config.slave_control = state->registry_pf_cfg.slave_config;
          state->com_port_info.com_config.min_bus_speed_KHz = state->registry_pf_cfg.min_bus_speed_khz;
          state->com_port_info.com_config.max_bus_speed_KHz = state->registry_pf_cfg.max_bus_speed_khz;
          state->com_port_info.com_config.reg_addr_type = state->registry_pf_cfg.reg_addr_type;
          state->irq_config.interrupt_num = state->registry_pf_cfg.dri_irq_num;
          state->irq_config.interrupt_pull_type = state->registry_pf_cfg.irq_pull_type;
          state->irq_config.is_chip_pin = state->registry_pf_cfg.irq_is_chip_pin;
          state->irq_config.interrupt_drive_strength = state->registry_pf_cfg.irq_drive_strength;
          state->irq_config.interrupt_trigger_type = state->registry_pf_cfg.irq_trigger_type;
          state->rail_config.num_of_rails = state->registry_pf_cfg.num_rail;
          state->registry_rail_on_state = state->registry_pf_cfg.rail_on_state;
          sns_strlcpy(state->rail_config.rails[0].name,
                      state->registry_pf_cfg.vddio_rail,
                      sizeof(state->rail_config.rails[0].name));
          sns_strlcpy(state->rail_config.rails[1].name,
                      state->registry_pf_cfg.vdd_rail,
                      sizeof(state->rail_config.rails[1].name));

          SNS_PRINTF(LOW, this, "Registry read event for group %s received "
             "bus_type:%d bus_instance:%d slave_control:%d",
                                   (char*)group_name.buf,
                               state->com_port_info.com_config.bus_type,
                               state->com_port_info.com_config.bus_instance,
                               state->com_port_info.com_config.slave_control);

          SNS_PRINTF(LOW, this, "min_bus_speed_KHz :%d max_bus_speed_KHz:%d reg_addr_type:%d",
                               state->com_port_info.com_config.min_bus_speed_KHz,
                               state->com_port_info.com_config.max_bus_speed_KHz,
                               state->com_port_info.com_config.reg_addr_type);

          SNS_PRINTF(LOW, this, "interrupt_num:%d interrupt_pull_type:%d is_chip_pin:%d",
                               state->irq_config.interrupt_num,
                               state->irq_config.interrupt_pull_type,
                               state->irq_config.is_chip_pin);

          SNS_PRINTF(LOW, this, "interrupt_drive_strength:%d interrupt_trigger_type:%d"
             " rigid body type:%d",
             state->irq_config.interrupt_drive_strength,
             state->irq_config.interrupt_trigger_type,
             state->registry_pf_cfg.rigid_body_type);

          //SENSOR_PRINTF_LOW_FULL(this, "num_rail:%d, rail_on_state:%d, vddio_rail:%s, vdd_rail:%s", state->rail_config.num_of_rails,
          //   state->registry_rail_on_state,
          //   state->rail_config.rails[0].name,
          //   state->rail_config.rails[1].name);
        }
      }
      else if(0 == strncmp((char*)group_name.buf, "lsm6ds3_0_platform.placement",
                           group_name.buf_len))
      {
        {
          uint8_t arr_index = 0;
          pb_float_arr_arg arr_arg = {
            .arr = state->placement,
            .arr_index = &arr_index,
            .arr_len = 12
          };

          sns_registry_decode_arg arg = {
            .item_group_name = &group_name,
            .parse_info_len = 1,
            .parse_info[0] = {
              .group_name = "placement",
              .parse_func = sns_registry_parse_float_arr,
              .parsed_buffer = &arr_arg
            }
          };

          read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
          read_event.data.items.arg = &arg;

          rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
        }

        if(rv)
        {
          state->registry_placement_received = true;
          SNS_PRINTF(LOW, this, "Registry read event for group %s received "
                 "p[0]:%f p[6]:%f p[11]:%f",
                 (char*)group_name.buf, state->placement[0], state->placement[6],
                 state->placement[11]);
        }
      }
      else if(0 == strncmp((char*)group_name.buf, "lsm6ds3_0_platform.orient",
                           group_name.buf_len))
      {
        {
          sns_registry_decode_arg arg = {
            .item_group_name = &group_name,
            .parse_info_len = 1,
            .parse_info[0] = {
              .group_name = "orient",
              .parse_func = sns_registry_parse_axis_orientation,
              .parsed_buffer = state->axis_map
            }
          };

          read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
          read_event.data.items.arg = &arg;

          rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
        }

        if(rv)
        {
          state->registry_orient_received = true;
          //SENSOR_PRINTF_LOW_FULL(this, "Registry read event for group %s received ", (char*)group_name.buf);

          SNS_PRINTF(LOW, this, "Input Axis:%d maps to Output Axis:%d with inversion %d",
                 state->axis_map[0].ipaxis,
                 state->axis_map[0].opaxis, state->axis_map[0].invert);

          SNS_PRINTF(LOW, this, "Input Axis:%d maps to Output Axis:%d with inversion %d",
                 state->axis_map[1].ipaxis, state->axis_map[1].opaxis,
                 state->axis_map[1].invert);

          SNS_PRINTF(LOW, this, "Input Axis:%d maps to Output Axis:%d with inversion %d",
                 state->axis_map[2].ipaxis, state->axis_map[2].opaxis,
                 state->axis_map[2].invert);
        }
      }
        else if(0 == strncmp((char*)group_name.buf,
                           "lsm6ds3_0_platform.accel.fac_cal",
                           group_name.buf_len) ||
              0 == strncmp((char*)group_name.buf,
                         "lsm6ds3_0_platform.gyro.fac_cal",
                           group_name.buf_len) ||
              0 == strncmp((char*)group_name.buf,
                         "lsm6ds3_0_platform.temp.fac_cal",
                         group_name.buf_len))
        {
        {
          uint8_t bias_arr_index = 0, scale_arr_index = 0;
          pb_float_arr_arg bias_arr_arg = {
            .arr = state->fac_cal_bias,
            .arr_index = &bias_arr_index,
            .arr_len = TRIAXIS_NUM
          };

          pb_float_arr_arg scale_arr_arg = {
            .arr = state->fac_cal_scale,
            .arr_index = &scale_arr_index,
            .arr_len = TRIAXIS_NUM
          };

          sns_registry_decode_arg arg = {
            .item_group_name = &group_name,
            .parse_info_len = 3,
            .parse_info[0] = {
              .group_name = "bias",
              .parse_func = sns_registry_parse_float_arr,
              .parsed_buffer = &bias_arr_arg
            },
            .parse_info[1] = {
              .group_name = "scale",
              .parse_func = sns_registry_parse_float_arr,
              .parsed_buffer = &scale_arr_arg
            },
            .parse_info[2] = {
              .group_name = "corr_mat",
              .parse_func = sns_registry_parse_corr_matrix_3,
              .parsed_buffer = &state->fac_cal_corr_mat
            }
          };

          read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
          read_event.data.items.arg = &arg;

          rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
        }

        if(rv)
        {
          state->registry_fac_cal_received = true;
          if(state->fac_cal_scale[0] != 0.0)
          {
            state->fac_cal_corr_mat.e00 = state->fac_cal_scale[0];
            state->fac_cal_corr_mat.e11 = state->fac_cal_scale[1];
            state->fac_cal_corr_mat.e22 = state->fac_cal_scale[2];
          }

          //SENSOR_PRINTF_LOW_FULL(this, "Registry read event for group %s received ", group_name.buf);
          //SENSOR_PRINTF_LOW_FULL(this, "Fac Cal Corr Matrix e00:%f e01:%f e02:%f", state->fac_cal_corr_mat.e00,state->fac_cal_corr_mat.e01,
          //       state->fac_cal_corr_mat.e02);
          //SENSOR_PRINTF_LOW_FULL(this, "Fac Cal Corr Matrix e10:%f e11:%f e12:%f", state->fac_cal_corr_mat.e10,state->fac_cal_corr_mat.e11,
          //       state->fac_cal_corr_mat.e12);
          //SENSOR_PRINTF_LOW_FULL(this, "Fac Cal Corr Matrix e20:%f e21:%f e22:%f", state->fac_cal_corr_mat.e20,state->fac_cal_corr_mat.e21,
          //       state->fac_cal_corr_mat.e22);
          //SENSOR_PRINTF_LOW_FULL(this, "Fac Cal Bias x:%f y:%f z:%f", state->fac_cal_bias[0], state->fac_cal_bias[1],
          //       state->fac_cal_bias[2]);
        }
      }
      else if(0 == strncmp((char*)group_name.buf,
                           "lsm6ds3_0_platform.md.config",
                           group_name.buf_len))
      {
        {
          sns_registry_decode_arg arg = {
            .item_group_name = &group_name,
            .parse_info_len = 1,
            .parse_info[0] = {
              .group_name = "lsm6ds3_0_platform.md.config",
              .parse_func = sns_registry_parse_md_cfg,
              .parsed_buffer = &state->md_config
            }
          };

          read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
          read_event.data.items.arg = &arg;

          rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
        }

        if(rv)
        {
          state->registry_md_config_received = true;
          //SENSOR_PRINTF_LOW_FULL(this, "Registry read event for group %s received ", (char*)group_name.buf);

          //SENSOR_PRINTF_LOW_FULL(this, "MD Threshold:%f m/s2 MD Window:%f sec MD Disable :%d", state->md_config.thresh, state->md_config.win, state->md_config.disable);
        }
      }
      else
      {
        rv = false;
      }

      if(!rv)
      {
        //SENSOR_PRINTF_ERROR_FULL(this, "Error decoding registry group %s due to %s", (char*)group_name.buf,
        //                         PB_GET_ERROR(&stream));
      }
    }
  }
  else
  {
    SNS_PRINTF(ERROR, this, "Received unsupported registry event msg id %u",
                             event->message_id);
  }
}

static void lsm6ds3_sensor_send_registry_request(sns_sensor *const this,
                                                 char *reg_group_name)
{
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;
  uint8_t buffer[100];
  int32_t encoded_len;
  sns_memset(buffer, 0, sizeof(buffer));
  sns_rc rc = SNS_RC_SUCCESS;

  sns_registry_read_req read_request;
  pb_buffer_arg data = (pb_buffer_arg){ .buf = reg_group_name,
    .buf_len = (strlen(reg_group_name) + 1) };

  read_request.name.arg = &data;
  read_request.name.funcs.encode = pb_encode_string_cb;

  encoded_len = pb_encode_request(buffer, sizeof(buffer),
      &read_request, sns_registry_read_req_fields, NULL);
  if(0 < encoded_len)
  {
    sns_request request = (sns_request){
      .request_len = encoded_len, .request = buffer,
      .message_id = SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_REQ };
    rc = state->reg_data_stream->api->send_request(state->reg_data_stream, &request);
  }

  //SENSOR_PRINTF_LOW_FULL(this, "Sending registry request for group name:%s", reg_group_name);
}

/**
 * Sets instance config to run a self test.
 *
 * @param[i] this      Sensor reference
 * @param[i] instance  Sensor Instance reference
 *
 * @return none
 */
void lsm6ds3_set_self_test_inst_config(sns_sensor *this,
                              sns_sensor_instance *instance)
{

  sns_request config;

  config.message_id = SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG;
  config.request_len = 0;
  config.request = NULL;

  this->instance_api->set_client_config(instance, &config);
}

/** See sns_lsm6ds3_sensor.h*/
sns_rc lsm6ds3_sensor_notify_event(sns_sensor *const this)
{
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_svc = (sns_stream_service*)
              service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
  sns_time on_timestamp;
  uint8_t buffer[1];
  sns_rc rv = SNS_RC_SUCCESS;
  sns_sensor_event *event;

  if(state->fw_stream)
  {
    if((0 == sns_memcmp(&state->irq_suid, &((sns_sensor_uid){{0}}), sizeof(state->irq_suid)))
     || (0 == sns_memcmp(&state->acp_suid, &((sns_sensor_uid){{0}}), sizeof(state->acp_suid)))
     || (0 == sns_memcmp(&state->timer_suid, &((sns_sensor_uid){{0}}), sizeof(state->timer_suid)))
     || (0 == sns_memcmp(&state->dae_suid, &((sns_sensor_uid){{0}}), sizeof(state->dae_suid)))
     || (0 == sns_memcmp(&state->reg_suid, &((sns_sensor_uid){{0}}), sizeof(state->reg_suid))))
    {
      lsm6ds3_process_suid_events(this);

      // place a request to registry sensor
      if((0 != sns_memcmp(&state->reg_suid, &((sns_sensor_uid){{0}}), sizeof(state->reg_suid))))
      {
        if(state->reg_data_stream == NULL)
        {
          stream_svc->api->create_sensor_stream(stream_svc,
              this, state->reg_suid , &state->reg_data_stream);

          lsm6ds3_sensor_send_registry_request(this, "lsm6ds3_0_platform.config");
          lsm6ds3_sensor_send_registry_request(this, "lsm6ds3_0_platform.placement");
          lsm6ds3_sensor_send_registry_request(this, "lsm6ds3_0_platform.orient");
          lsm6ds3_sensor_send_registry_request(this, "lsm6ds3_0_platform.md.config");

          if(LSM6DS3_ACCEL == state->sensor)
          {
            lsm6ds3_sensor_send_registry_request(this, "lsm6ds3_0.accel.config");
            lsm6ds3_sensor_send_registry_request(
              this, "lsm6ds3_0_platform.accel.fac_cal");
          }
          else if(LSM6DS3_GYRO == state->sensor)
          {
            lsm6ds3_sensor_send_registry_request(this, "lsm6ds3_0.gyro.config");
            lsm6ds3_sensor_send_registry_request(
              this, "lsm6ds3_0_platform.gyro.fac_cal");
          }
          else if(LSM6DS3_SENSOR_TEMP == state->sensor)
          {
            lsm6ds3_sensor_send_registry_request(
              this, "lsm6ds3_0.temp.config");
          }
          else if(LSM6DS3_MOTION_DETECT == state->sensor)
          {
            lsm6ds3_sensor_send_registry_request(
              this, "lsm6ds3_0.md.config");
          }
        }
      }
    }
  }

  /**----------------------Handle a Timer Sensor event.-------------------*/
  if(NULL != state->timer_stream)
  {
    event = state->timer_stream->api->peek_input(state->timer_stream);
    while(NULL != event)
    {
      pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
                                                   event->event_len);
      sns_timer_sensor_event timer_event;

      if(SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id)
      {
        if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
        {
          if(state->power_rail_pend_state == LSM6DS3_POWER_RAIL_PENDING_INIT)
          {
            /**-------------------Read and Confirm WHO-AM-I------------------------*/
            buffer[0] = 0x0;
            rv = lsm6ds3_get_who_am_i(state->scp_service,state->com_port_info.port_handle, &buffer[0]);
            if(rv == SNS_RC_SUCCESS
               &&
               buffer[0] == LSM6DS3_WHOAMI_VALUE)
            {
              // Reset Sensor
              rv = lsm6ds3_reset_device(state->scp_service,state->com_port_info.port_handle,
                                        LSM6DS3_ACCEL | LSM6DS3_GYRO | LSM6DS3_MOTION_DETECT | LSM6DS3_SENSOR_TEMP);

              if(rv == SNS_RC_SUCCESS)
              {
                 state->hw_is_present = true;
              }
            }
            state->who_am_i = buffer[0];

            /**------------------Power Down and Close COM Port--------------------*/
            state->scp_service->api->sns_scp_update_bus_power(
                                                        state->com_port_info.port_handle,
                                                        false);

            state->scp_service->api->sns_scp_close(state->com_port_info.port_handle);
            state->scp_service->api->sns_scp_deregister_com_port(state->com_port_info.port_handle);

            /**----------------------Turn Power Rail OFF--------------------------*/
            state->rail_config.rail_vote = SNS_RAIL_OFF;
            state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
                                                                     this,
                                                                     &state->rail_config,
                                                                     NULL);
            if(state->hw_is_present)
            {
              lsm6ds3_publish_available(this);
            }
            else
            {
              rv = SNS_RC_INVALID_STATE;
              SNS_PRINTF(LOW, this, "LSM6DS3 HW absent");
            }
            state->power_rail_pend_state = LSM6DS3_POWER_RAIL_PENDING_NONE;
          }
          else if(state->power_rail_pend_state == LSM6DS3_POWER_RAIL_PENDING_SET_CLIENT_REQ)
          {
            sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
            if(NULL != instance)
            {
              lsm6ds3_instance_state *inst_state =
                 (lsm6ds3_instance_state*) instance->state->state;
              inst_state->instance_is_ready_to_configure = true;
              lsm6ds3_reval_instance_config(this, instance, state->sensor);
              if(inst_state->new_self_test_request)
              {
                lsm6ds3_set_self_test_inst_config(this, instance);
              }
            }
            state->power_rail_pend_state = LSM6DS3_POWER_RAIL_PENDING_NONE;
          }
        }
        else
        {
          SNS_PRINTF(ERROR, this, "pb_decode error");
        }
      }
      event = state->timer_stream->api->get_next_input(state->timer_stream);
    }
  }

  if(NULL != state->reg_data_stream)
  {
    event = state->reg_data_stream->api->peek_input(state->reg_data_stream);
    while(NULL != event)
    {
      lsm6ds3_sensor_process_registry_event(this, event);

      if(state->registry_cfg_received && state->registry_placement_received)
      {
        lsm6ds3_publish_registry_attributes(this);
      }

      event = state->reg_data_stream->api->get_next_input(state->reg_data_stream);
    }
  }

  if(0 != sns_memcmp(&state->timer_suid, &((sns_sensor_uid){{0}}), sizeof(state->timer_suid)) &&
     state->registry_pf_cfg_received && state->registry_cfg_received &&
     state->registry_orient_received &&
     state->registry_placement_received && state->registry_md_config_received)
  {
      state->registry_pf_cfg_received = false;

      /**-----------------Register and Open COM Port-------------------------*/
      if(NULL == state->com_port_info.port_handle)
      {
        rv = state->scp_service->api->sns_scp_register_com_port(
           &state->com_port_info.com_config,
                                                &state->com_port_info.port_handle);

        if(rv == SNS_RC_SUCCESS)
        {
          rv = state->scp_service->api->sns_scp_open(state->com_port_info.port_handle);
        }
      }

      /**---------------------Register Power Rails --------------------------*/
      if(0 != sns_memcmp(&state->timer_suid, &((sns_sensor_uid){{0}}), sizeof(state->timer_suid))
         && NULL == state->pwr_rail_service
         && rv == SNS_RC_SUCCESS)
      {
        state->rail_config.rail_vote = SNS_RAIL_OFF;

        state->pwr_rail_service =
         (sns_pwr_rail_service*)service_mgr->get_service(service_mgr,
                                                         SNS_POWER_RAIL_SERVICE);

        state->pwr_rail_service->api->sns_register_power_rails(state->pwr_rail_service,
                                                               &state->rail_config);

        /**---------------------Turn Power Rails ON----------------------------*/
        if(state->sensor == LSM6DS3_GYRO)
        {
          state->rail_config.rail_vote = SNS_RAIL_ON_NPM;
        }
        else
        {
          state->rail_config.rail_vote = state->registry_rail_on_state;
        }

        state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
                                                                 this,
                                                                 &state->rail_config,
                                                                 &on_timestamp);

        /**-------------Create a Timer stream for Power Rail ON timeout.---------*/
        if(NULL == state->timer_stream)
        {
          stream_svc->api->create_sensor_stream(stream_svc, this, state->timer_suid,
                                                &state->timer_stream);
          if(NULL != state->timer_stream)
          {
            lsm6ds3_start_power_rail_timer(this,
                                           sns_convert_ns_to_ticks(LSM6DS3_OFF_TO_IDLE_MS * 1000 * 1000),
                                           LSM6DS3_POWER_RAIL_PENDING_INIT);
          }
        }
      }
    }
  return rv;
}

static void lsm6ds3_send_flush_config(sns_sensor *const this,
                                       sns_sensor_instance *instance)
{
  sns_request config;

  config.message_id = SNS_STD_MSGID_SNS_STD_FLUSH_REQ;
  config.request_len = 0;
  config.request = NULL;

  this->instance_api->set_client_config(instance, &config);
}

/**
 * Returns decoded request message for type
 * sns_std_sensor_config.
 *
 * @param[in] in_request   Request as sotred in client_requests
 *                         list.
 * @param decoded_request  Standard decoded message.
 * @param decoded_payload  Decoded stream request payload.
 *
 * @return bool true if decode is successful else false
 */
static bool lsm6ds3_get_decoded_imu_request(sns_sensor const *this, sns_request const *in_request,
                                            sns_std_request *decoded_request,
                                            sns_std_sensor_config *decoded_payload)
{
  pb_istream_t stream;
  pb_simple_cb_arg arg =
      { .decoded_struct = decoded_payload,
        .fields = sns_std_sensor_config_fields };
  decoded_request->payload = (struct pb_callback_s)
      { .funcs.decode = &pb_decode_simple_cb, .arg = &arg };
  stream = pb_istream_from_buffer(in_request->request,
                                  in_request->request_len);
  if(!pb_decode(&stream, sns_std_request_fields, decoded_request))
  {
    SNS_PRINTF(ERROR, this, "LSM decode error");
    return false;
  }
  return true;
}

/**
 * Decodes self test requests.
 *
 * @param[i] this              Sensor reference
 * @param[i] request           Encoded input request
 * @param[o] decoded_request   Decoded standard request
 * @param[o] test_config       decoded self test request
 *
 * @return bool True if decoding is successfull else false.
 */
static bool lsm6ds3_get_decoded_self_test_request(sns_sensor const *this, sns_request const *request,
                                                  sns_std_request *decoded_request,
                                                  sns_physical_sensor_test_config *test_config)
{
  pb_istream_t stream;
  pb_simple_cb_arg arg =
      { .decoded_struct = test_config,
        .fields = sns_physical_sensor_test_config_fields };
  decoded_request->payload = (struct pb_callback_s)
      { .funcs.decode = &pb_decode_simple_cb, .arg = &arg };
  stream = pb_istream_from_buffer(request->request,
                                  request->request_len);
  if(!pb_decode(&stream, sns_std_request_fields, decoded_request))
  {
    SNS_PRINTF(ERROR, this, "LSM decode error");
    return false;
  }
  return true;
}

/**
 * Parses through all starndard streaming requests and deduces
 * best HW config for the inertial sensor type.
 *
 * @param[i] this             Sensor reference
 * @param[i] instance         Instance reference
 * @param[i] sensor_type      sensor type
 * @param[o] chosen_sample_rate   chosen sample rate in Hz
 * @param[o] chosen_report_rate   chosen report rate in Hz
 * @param[o] non_gated_sensor_client_present  True if non-gated
 *       client is present.
 * @param[o] gated_sensor_client_present  Tur if gated client is
 *       present.
 */
static void lsm6ds3_get_imu_config(sns_sensor *this,
                                   sns_sensor_instance *instance,
                                   lsm6ds3_sensor_type sensor_type,
                                   float *chosen_sample_rate,
                                   float *chosen_report_rate,
                                   bool *non_gated_sensor_client_present,
                                   bool *gated_sensor_client_present)
{
  UNUSED_VAR(this);
  sns_sensor_uid suid;
  sns_request const *request;

  if(sensor_type == LSM6DS3_ACCEL)
  {
    sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)ACCEL_SUID), sizeof(sns_sensor_uid));
  }
  else
  {
    sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)GYRO_SUID), sizeof(sns_sensor_uid));
  }

  *chosen_report_rate = 0;
  *chosen_sample_rate = 0;
  *non_gated_sensor_client_present = false;
  if(gated_sensor_client_present)
  {
    *gated_sensor_client_present = false;
  }

  /** Parse through existing requests and get fastest sample
   *  rate and report rate requests. */
  for(request = instance->cb->get_client_request(instance, &suid, true);
      NULL != request;
      request = instance->cb->get_client_request(instance, &suid, false))
  {
    sns_std_request decoded_request;
    sns_std_sensor_config decoded_payload = {0};

    if(request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG
       ||
       request->message_id == SNS_STD_EVENT_GATED_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
    {
      if(lsm6ds3_get_decoded_imu_request(this, request, &decoded_request, &decoded_payload))
      {
        float report_rate;
        *chosen_sample_rate = SNS_MAX(*chosen_sample_rate,
                                       decoded_payload.sample_rate);
        if(decoded_request.has_batching
           &&
           decoded_request.batching.batch_period > 0)
        {
          report_rate = (1000000.0 / (float)decoded_request.batching.batch_period);
        }
        else
        {
          report_rate = *chosen_sample_rate;
        }
        *chosen_report_rate = SNS_MAX(*chosen_report_rate,
                                      report_rate);

        if(request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
        {
          *non_gated_sensor_client_present = true;
        }
        else
        {
          if(gated_sensor_client_present)
          {
            *gated_sensor_client_present = true;
          }
        }
      }
    }
  }
}

/**
 * Determines motion detect config.
 *
 * @param[i] this   Sensor reference
 * @param[i] instance  Sensor Instance reference
 * @param[o] chosen_md_enable  True if MD should be enabled
 * @param[o] md_client_present True if there is an MD client
 *       present.
 *
 * @return none
 */
static void lsm6ds3_get_motion_detect_config(sns_sensor *this,
                                             sns_sensor_instance *instance,
                                             bool *chosen_md_enable,
                                             bool *md_client_present)
{
  UNUSED_VAR(this);
  sns_sensor_uid suid = MOTION_DETECT_SUID;
  lsm6ds3_instance_state *inst_state =
     (lsm6ds3_instance_state*)instance->state->state;
  sns_request const *request;

  *chosen_md_enable = false;
  *md_client_present = false;

  for(request = instance->cb->get_client_request(instance, &suid, true);
      NULL != request;
      request = instance->cb->get_client_request(instance, &suid, false))
  {
    if(request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG)
    {
      // Enable MD interrupt when:
      // 1. There is a new request and MD is in MDF state OR
      // 2. There is an existing request and MD is in MDE/MDD state

      *chosen_md_enable = ((inst_state->md_info.md_new_req &&
                           inst_state->md_info.md_state.motion_detect_event_type == SNS_MOTION_DETECT_EVENT_TYPE_FIRED)
                           ||
                           (inst_state->md_info.md_client_present &&
                            (inst_state->md_info.md_state.motion_detect_event_type == SNS_MOTION_DETECT_EVENT_TYPE_ENABLED
                             || inst_state->md_info.md_state.motion_detect_event_type == SNS_MOTION_DETECT_EVENT_TYPE_DISABLED)));

      *md_client_present = true;
      // Consumed new request
      inst_state->md_info.md_new_req = false;
      return;
    }
  }
}

/**
 * Parses through standard streaming requests for the sensor
 * temperature dataype and deduces best HW config.
 *
 * @param[i] this      Sensor reference
 * @param[i] instance  Sensor Intance reference
 * @param[o] chosen_sample_rate  chosen sample rate in Hz
 * @param[o] chosen_report_rate  chosen report rate in Hz
 * @param[o] sensor_temp_client_present  True if there is a
 *       sensor temp client present
 *
 * @return none
 */
static void lsm6ds3_get_sensor_temp_config(sns_sensor *this,
                                           sns_sensor_instance *instance,
                                           float *chosen_sample_rate,
                                           float *chosen_report_rate,
                                           bool *sensor_temp_client_present)
{
  UNUSED_VAR(this);
  lsm6ds3_instance_state *inst_state =
     (lsm6ds3_instance_state*)instance->state->state;
  sns_sensor_uid suid = SENSOR_TEMPERATURE_SUID;
  sns_request const *request;

  *chosen_report_rate = 0;
  *chosen_sample_rate = 0;
  *sensor_temp_client_present = false;

    /** Parse through existing requests and get fastest sample
   *  rate and report rate requests. */
  for(request = instance->cb->get_client_request(instance, &suid, true);
      NULL != request;
      request = instance->cb->get_client_request(instance, &suid, false))
  {
    sns_std_request decoded_request;
    sns_std_sensor_config decoded_payload = {0};

    if(request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
    {
      if(lsm6ds3_get_decoded_imu_request(this, request, &decoded_request, &decoded_payload))
      {
        float report_rate;
        *chosen_sample_rate = SNS_MAX(*chosen_sample_rate,
                                       decoded_payload.sample_rate);
        bool rc = sns_sensor_util_decide_max_batch(instance,&suid);
        //There is request with max batch not set .
        // do normal calculation
        if(!rc)
        {
        if(decoded_request.has_batching
           &&
           decoded_request.batching.batch_period > 0)
        {
          report_rate = (1000000.0 / (float)decoded_request.batching.batch_period);
        }
        else
        {
          report_rate = *chosen_sample_rate;
        }
        }
        else
        {
          report_rate = (1000000.0 / (float)UINT32_MAX);
        }
        *chosen_report_rate = SNS_MAX(*chosen_report_rate, report_rate);
        *sensor_temp_client_present = true;
      }
    }
  }
  inst_state->sensor_temp_info.report_rate_hz  = *chosen_report_rate;
  inst_state->sensor_temp_info.sampling_rate_hz = *chosen_sample_rate;
}

/**
 * Set standard streaming config for the instance.
 *
 * @param[i] this        Sensor reference
 * @param[i] instance    Sensor Instance reference
 * @param[i] chosen_report_rate   chosen report rate in Hz
 * @param[i] chosen_sample_rate   chosen sample rate in Hz
 * @param[i] registry_cfg Sensor specific registry configuration
 * @param[i] message_id           input message ID 
 *
 * @return none
 */
static void lsm6ds3_set_inst_config(sns_sensor *this,
                                    sns_sensor_instance *instance,
                                    float chosen_report_rate,
                                    float chosen_sample_rate,
                                    sns_lsm6ds3_registry_cfg registry_cfg,
                                    uint32_t message_id)
{
  sns_lsm6ds3_req new_client_config;
  sns_request config;

  new_client_config.desired_report_rate = chosen_report_rate;
  new_client_config.desired_sample_rate = chosen_sample_rate;
  new_client_config.registry_cfg = registry_cfg;

  config.message_id = message_id;
  config.request_len = sizeof(sns_lsm6ds3_req);
  config.request = &new_client_config;

  this->instance_api->set_client_config(instance, &config);
}

/**
 * Turns power rails off
 *
 * @paramp[i] this   Sensor reference
 *
 * @return none
 */
static void lsm6ds3_turn_rails_off(sns_sensor *this)
{
  sns_sensor *sensor;

  for(sensor = this->cb->get_library_sensor(this, true);
      NULL != sensor;
      sensor = this->cb->get_library_sensor(this, false))
  {
    lsm6ds3_state *sensor_state = (lsm6ds3_state*)sensor->state->state;
    if(sensor_state->rail_config.rail_vote != SNS_RAIL_OFF)
    {
      sensor_state->rail_config.rail_vote = SNS_RAIL_OFF;
      sensor_state->pwr_rail_service->api->sns_vote_power_rail_update(sensor_state->pwr_rail_service,
                                                                      sensor,
                                                                      &sensor_state->rail_config,
                                                                      NULL);
    }
  }
}

/**
 * Decodes a physical sensor self test request and updates
 * instance state with request info.
 *
 * @param[i] this      Sensor reference
 * @param[i] instance  Sensor Instance reference
 * @param[i] new_request Encoded request
 *
 * @return True if request is valid else false
 */
static bool lsm6ds3_extract_self_test_info(sns_sensor *this,
                              sns_sensor_instance *instance,
                              struct sns_request const *new_request)
{
  sns_std_request decoded_request;
  sns_physical_sensor_test_config test_config = sns_physical_sensor_test_config_init_default;
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;
  lsm6ds3_instance_state *inst_state = (lsm6ds3_instance_state*)instance->state->state;
  lsm6ds3_self_test_info *self_test_info;

  if(state->sensor == LSM6DS3_ACCEL)
  {
    self_test_info = &inst_state->accel_info.test_info;
  }
  else if(state->sensor == LSM6DS3_GYRO)
  {
    self_test_info = &inst_state->gyro_info.test_info;
  }
  else if(state->sensor == LSM6DS3_MOTION_DETECT)
  {
    self_test_info = &inst_state->md_info.test_info;
  }
  else if(state->sensor == LSM6DS3_SENSOR_TEMP)
  {
    self_test_info = &inst_state->sensor_temp_info.test_info;
  }
  else
  {
    return false;
  }

  if(lsm6ds3_get_decoded_self_test_request(this, new_request, &decoded_request, &test_config))
  {
    self_test_info->test_type = test_config.test_type;
    self_test_info->test_client_present = true;
    return true;
  }
  else
  {
    return false;
  }
}

/** See sns_lsm6ds3_sensor.h */
void lsm6ds3_reval_instance_config(sns_sensor *this,
                                   sns_sensor_instance *instance,
                                   lsm6ds3_sensor_type sensor_type)
{
  /**
   * 1. Get best non-gated and gated Accel Config.
   * 2. Get best Gyro Config.
   * 3. Get Motion Detect Config.
   * 4. Get best Sensor Temperature Config.
   * 5. Decide best Instance Config based on above outputs.
   */
  float chosen_sample_rate = 0;
  float chosen_report_rate = 0;
  float sample_rate = 0;
  float report_rate = 0;
  bool a_sensor_client_present_non_gated;
  bool a_sensor_client_present_gated;
  bool g_sensor_client_present;
  bool md_sensor_client_present;
  bool sensor_temp_client_present;
  lsm6ds3_instance_state *inst_state =
     (lsm6ds3_instance_state*)instance->state->state;
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;
  sns_lsm6ds3_registry_cfg registry_cfg = {.sensor_type = sensor_type};

  lsm6ds3_get_imu_config(this,
                         instance,
                         LSM6DS3_ACCEL,
                         &chosen_sample_rate,
                         &chosen_report_rate,
                         &a_sensor_client_present_non_gated,
                         &a_sensor_client_present_gated);

  inst_state->accel_info.gated_client_present = a_sensor_client_present_gated;

  lsm6ds3_get_imu_config(this,
                         instance,
                         LSM6DS3_GYRO,
                         &sample_rate,
                         &report_rate,
                         &g_sensor_client_present,
                         NULL);

  chosen_sample_rate = SNS_MAX(chosen_sample_rate, sample_rate);
  chosen_report_rate = SNS_MAX(chosen_report_rate, report_rate);

  lsm6ds3_get_motion_detect_config(this,
                                   instance,
                                   &inst_state->md_info.enable_md_int,
                                   &md_sensor_client_present);

  inst_state->md_info.md_client_present = md_sensor_client_present;

  if(a_sensor_client_present_non_gated)
  {
    inst_state->md_info.enable_md_int = false;
  }

  if(chosen_report_rate > 0.0 && chosen_sample_rate > 0.0)
  {
    uint32_t wm = (uint32_t)(chosen_sample_rate / chosen_report_rate);
    inst_state->fifo_info.max_requested_wmk = SNS_MAX(1, wm);
  }

  lsm6ds3_get_sensor_temp_config(this,
                                 instance,
                                 &sample_rate,
                                 &report_rate,
                                 &sensor_temp_client_present);

  chosen_sample_rate = SNS_MAX(chosen_sample_rate, sample_rate);
  chosen_report_rate = SNS_MAX(chosen_report_rate, report_rate);

  /** Start with a clean slate */
  inst_state->fifo_info.fifo_enabled = 0;
  inst_state->fifo_info.publish_sensors = 0;

  if(a_sensor_client_present_non_gated)
  {
    inst_state->fifo_info.fifo_enabled |= LSM6DS3_ACCEL;
    inst_state->fifo_info.publish_sensors |= LSM6DS3_ACCEL;
  }

  if(a_sensor_client_present_gated)
  {
    inst_state->fifo_info.fifo_enabled |= LSM6DS3_ACCEL;
    if(!inst_state->md_info.enable_md_int)
    {
      inst_state->fifo_info.publish_sensors |= LSM6DS3_ACCEL;
    }
  }

  if(g_sensor_client_present)
  {
    inst_state->fifo_info.fifo_enabled |= (LSM6DS3_ACCEL | LSM6DS3_GYRO);
    inst_state->fifo_info.publish_sensors |= LSM6DS3_GYRO;
  }

  if(md_sensor_client_present)
  {
    inst_state->fifo_info.fifo_enabled |= LSM6DS3_ACCEL;
  }

  if(sensor_temp_client_present)
  {
    inst_state->fifo_info.fifo_enabled |= LSM6DS3_ACCEL;
    inst_state->fifo_info.publish_sensors |= LSM6DS3_SENSOR_TEMP;
  }

  if(LSM6DS3_ACCEL == sensor_type || LSM6DS3_GYRO == sensor_type ||
     LSM6DS3_SENSOR_TEMP == sensor_type)
  {
    sns_memscpy(registry_cfg.fac_cal_bias, sizeof(registry_cfg.fac_cal_bias),
                state->fac_cal_bias, sizeof(state->fac_cal_bias));

    sns_memscpy(&registry_cfg.fac_cal_corr_mat, sizeof(registry_cfg.fac_cal_corr_mat),
                &state->fac_cal_corr_mat, sizeof(state->fac_cal_corr_mat));
  }

  lsm6ds3_set_inst_config(this,
                          instance,
                          chosen_report_rate,
                          chosen_sample_rate,
                          registry_cfg,
                          SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG);
}

/** See sns_lsm6ds3_sensor.h */
sns_sensor_instance* lsm6ds3_set_client_request(sns_sensor *const this,
                                                struct sns_request const *exist_request,
                                                struct sns_request const *new_request,
                                                bool remove)
{
  sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;
  sns_time on_timestamp;
  sns_time delta;
  bool reval_config = false;

  if(remove)
  {
    if(NULL != instance)
    {
      instance->cb->remove_client_request(instance, exist_request);
      /* Assumption: The FW will call deinit() on the instance before destroying it.
                     Putting all HW resources (sensor HW, COM port, power rail)in
                     low power state happens in Instance deinit().*/

      lsm6ds3_reval_instance_config(this, instance, state->sensor);
    }
  }
  else
  {
     // 1. If new request then:
     //     a. Power ON rails.
     //     b. Power ON COM port - Instance must handle COM port power.
     //     c. Create new instance.
     //     d. Re-evaluate existing requests and choose appropriate instance config.
     //     e. set_client_config for this instance.
     //     f. Add new_request to list of requests handled by the Instance.
     //     g. Power OFF COM port if not needed- Instance must handle COM port power.
     //     h. Return the Instance.
     // 2. If there is an Instance already present:
     //     a. Add new_request to list of requests handled by the Instance.
     //     b. Remove exist_request from list of requests handled by the Instance.
     //     c. Re-evaluate existing requests and choose appropriate Instance config.
     //     d. set_client_config for the Instance if not the same as current config.
     //     e. publish the updated config.
     //     f. Return the Instance.
     // 3.  If "flush" request:
     //     a. Perform flush on the instance.
     //     b. Return NULL.

     if(NULL == instance)
     {
       if(state->sensor == LSM6DS3_GYRO)
       {
         state->rail_config.rail_vote = SNS_RAIL_ON_NPM;
       }
       else
       {
         state->rail_config.rail_vote = state->registry_rail_on_state;
       }

       state->pwr_rail_service->api->sns_vote_power_rail_update(
                                            state->pwr_rail_service,
                                            this,
                                            &state->rail_config,
                                            &on_timestamp);

       delta = sns_get_system_time() - on_timestamp;

       // Use on_timestamp to determine correct Timer value.
       if(delta < sns_convert_ns_to_ticks(LSM6DS3_OFF_TO_IDLE_MS*1000*1000))
       {
         lsm6ds3_start_power_rail_timer(this,
                                        sns_convert_ns_to_ticks(LSM6DS3_OFF_TO_IDLE_MS*1000*1000) - delta,
                                        LSM6DS3_POWER_RAIL_PENDING_SET_CLIENT_REQ);
       }
       else
       {
         // rail is already ON
         reval_config = true;
       }

       /** create_instance() calls init() for the Sensor Instance */
       instance = this->cb->create_instance(this,
                                            sizeof(lsm6ds3_instance_state));
       /* If rail is already ON then flag instance OK to configure */
       if(reval_config)
       {
         lsm6ds3_instance_state *inst_state =
          (lsm6ds3_instance_state*)instance->state->state;

         inst_state->instance_is_ready_to_configure = true;
       }
     }
     else
     {
       if(NULL != exist_request
          &&
          NULL != new_request
          &&
          new_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ)
       {
         lsm6ds3_instance_state *inst_state =
          (lsm6ds3_instance_state*)instance->state->state;

         if(inst_state->fifo_info.fifo_enabled)
         {
           lsm6ds3_send_flush_config(this, instance);
           /** Do not update instance client request list at this point
           because FIFO flush is a transitory request for an on-going
           stream request. */
           return instance;
         }
         else
         {
           /** There aren't any FIFO sensors enabled to support flush */
           return NULL;
         }
       }
       else
       {
         reval_config = true;

         /** An existing client is changing request*/
         if((NULL != exist_request) && (NULL != new_request))
         {
           instance->cb->remove_client_request(instance, exist_request);
         }
         /** A new client sent new_request*/
         else if(NULL != new_request)
         {
           // No-op. new_request will be added to requests list below.
         }
       }
     }

     /** Add the new request to list of client_requests.*/
     if(NULL != instance)
     {
        lsm6ds3_instance_state *inst_state =
           (lsm6ds3_instance_state*)instance->state->state;
        if(NULL != new_request)
        {
          instance->cb->add_client_request(instance, new_request);

          if(new_request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG
             &&
             state->sensor == LSM6DS3_MOTION_DETECT)
          {
            inst_state->md_info.md_new_req = true;
          }
          if(new_request->message_id ==
             SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
          {
            if(lsm6ds3_extract_self_test_info(this, instance, new_request))
            {
              inst_state->new_self_test_request = true;
            }
          }
        }
        if(reval_config && inst_state->instance_is_ready_to_configure)
        {
          lsm6ds3_reval_instance_config(this, instance, state->sensor);

          if(inst_state->new_self_test_request)
          {
            lsm6ds3_set_self_test_inst_config(this, instance);
          }
        }
     }
  }

  if(NULL != instance &&
     NULL == instance->cb->get_client_request(instance,
       &(sns_sensor_uid)MOTION_DETECT_SUID, true) &&
     NULL == instance->cb->get_client_request(instance,
       &(sns_sensor_uid)ACCEL_SUID, true) &&
     NULL == instance->cb->get_client_request(instance,
       &(sns_sensor_uid)GYRO_SUID, true) &&
     NULL == instance->cb->get_client_request(instance,
       &(sns_sensor_uid)SENSOR_TEMPERATURE_SUID, true))
  {
    this->cb->remove_instance(instance);
    lsm6ds3_turn_rails_off(this);
  }

  return instance;
}

/** See sns_lsm6ds3_sensor.h */
void lsm6ds3_process_suid_events(sns_sensor *const this)
{
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;

  for(;
      0 != state->fw_stream->api->get_input_cnt(state->fw_stream);
      state->fw_stream->api->get_next_input(state->fw_stream))
  {
    sns_sensor_event *event =
      state->fw_stream->api->peek_input(state->fw_stream);

    if(SNS_SUID_MSGID_SNS_SUID_EVENT == event->message_id)
    {
      pb_istream_t stream = pb_istream_from_buffer((void*)event->event, event->event_len);
      sns_suid_event suid_event = sns_suid_event_init_default;
      pb_buffer_arg data_type_arg = { .buf = NULL, .buf_len = 0 };
      sns_sensor_uid uid_list;
      sns_suid_search suid_search;
      suid_search.suid = &uid_list;
      suid_search.num_of_suids = 0;

      suid_event.data_type.funcs.decode = &pb_decode_string_cb;
      suid_event.data_type.arg = &data_type_arg;
      suid_event.suid.funcs.decode = &pb_decode_suid_event;
      suid_event.suid.arg = &suid_search;

      if(!pb_decode(&stream, sns_suid_event_fields, &suid_event)) {
         SNS_PRINTF(ERROR, this, "SUID Decode failed");
         continue;
       }

      /* if no suids found, ignore the event */
      if(suid_search.num_of_suids == 0)
      {
        continue;
      }

      /* save suid based on incoming data type name */
      if(0 == strncmp(data_type_arg.buf, "data_acquisition_engine", data_type_arg.buf_len))
      {
        state->dae_suid = uid_list;
      }
      else if(0 == strncmp(data_type_arg.buf, "interrupt", data_type_arg.buf_len))
      {
        state->irq_suid = uid_list;
      }
      else if(0 == strncmp(data_type_arg.buf, "timer", data_type_arg.buf_len))
      {
        state->timer_suid = uid_list;
      }
      else if (0 == strncmp(data_type_arg.buf, "async_com_port",
                            data_type_arg.buf_len))
      {
        state->acp_suid = uid_list;
      }
      else if (0 == strncmp(data_type_arg.buf, "registry", data_type_arg.buf_len))
      {
        state->reg_suid = uid_list;
      }
      else
      {
        //SENSOR_PRINTF_ERROR_FULL(this, "invalid datatype_name %s", data_type_arg.buf);
      }
    }
  }
}

sns_sensor_api lsm6ds3_accel_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lsm6ds3_accel_init,
  .deinit             = &lsm6ds3_accel_deinit,
  .get_sensor_uid     = &lsm6ds3_accel_get_sensor_uid,
  .set_client_request = &lsm6ds3_set_client_request,
  .notify_event       = &lsm6ds3_sensor_notify_event,
};

sns_sensor_api lsm6ds3_gyro_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lsm6ds3_gyro_init,
  .deinit             = &lsm6ds3_gyro_deinit,
  .get_sensor_uid     = &lsm6ds3_gyro_get_sensor_uid,
  .set_client_request = &lsm6ds3_set_client_request,
  .notify_event       = &lsm6ds3_sensor_notify_event,
};

sns_sensor_api lsm6ds3_motion_detect_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lsm6ds3_motion_detect_init,
  .deinit             = &lsm6ds3_motion_detect_deinit,
  .get_sensor_uid     = &lsm6ds3_motion_detect_get_sensor_uid,
  .set_client_request = &lsm6ds3_set_client_request,
  .notify_event       = &lsm6ds3_sensor_notify_event,
};

sns_sensor_api lsm6ds3_sensor_temp_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lsm6ds3_sensor_temp_init,
  .deinit             = &lsm6ds3_sensor_temp_deinit,
  .get_sensor_uid     = &lsm6ds3_sensor_temp_get_sensor_uid,
  .set_client_request = &lsm6ds3_set_client_request,
  .notify_event       = &lsm6ds3_sensor_notify_event,
};

