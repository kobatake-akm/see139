/**
 * @file sns_lsm6ds3_sensor.c
 *
 * Common implementation for LSM6DS3 Sensors.
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include <string.h>
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_event_service.h"
#include "sns_stream_service.h"
#include "sns_service.h"
#include "sns_sensor_util.h"
#include "sns_mem_util.h"
#include "sns_math_util.h"
#include "sns_types.h"

#include "sns_lsm6ds3_sensor.h"
#include "sns_lsm6ds3_hal.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_std.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_motion_accel.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"
#include "sns_diag_service.h"

/** See sns_lem6ds3_sensor.h */
void lsm6ds3_publish_attributes(sns_sensor *const this)
{
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;
  sns_service_manager *manager = this->cb->get_service_manager(this);
  sns_attribute_service *attribute_service =
    (sns_attribute_service*)manager->get_service(manager, SNS_ATTRIBUTE_SERVICE);

  attribute_service->api->publish_attributes(attribute_service, this,
      state->attributes, ARR_SIZE(state->attributes));
}

static void lsm6ds3_start_power_rail_timer(sns_sensor *const this,
                                           sns_time timeout_ticks,
                                           lsm6ds3_power_rail_pending_state pwr_rail_pend_state)
{
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;

  sns_diag_service* diag = state->diag_service;

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
    diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
                             "LSM timer req encode error");
  }
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
  sns_diag_service* diag = state->diag_service;

  diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
 
  if(state->fw_stream)
  {
    if((0 == sns_memcmp(&state->irq_suid, &((sns_sensor_uid){{0}}), sizeof(state->irq_suid)))
     || (0 == sns_memcmp(&state->acp_suid, &((sns_sensor_uid){{0}}), sizeof(state->acp_suid)))
     || (0 == sns_memcmp(&state->timer_suid, &((sns_sensor_uid){{0}}), sizeof(state->timer_suid)))
#if LSM6DS3_ENABLE_DEPENDENCY
     || (0 == sns_memcmp(&state->reg_suid, &((sns_sensor_uid){{0}}), sizeof(state->reg_suid)))
#endif
    )
    {
      lsm6ds3_process_suid_events(this);
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
      if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
      {
        if(state->power_rail_pend_state == LSM6DS3_POWER_RAIL_PENDING_INIT)
        {
          /**-------------------Read and Confirm WHO-AM-I------------------------*/
          buffer[0] = 0x0;
          rv = lsm6ds3_get_who_am_i(state->com_port_info.port_handle, &buffer[0]);
          if(rv == SNS_RC_SUCCESS
             &&
             buffer[0] == LSM6DS3_WHOAMI_VALUE)
          {
            // Reset Sensor
            rv = lsm6ds3_reset_device(state->com_port_info.port_handle,
                                      LSM6DS3_ACCEL | LSM6DS3_GYRO | LSM6DS3_MOTION_ACCEL | LSM6DS3_SENSOR_TEMP);

            if(rv == SNS_RC_SUCCESS)
            {
               state->hw_is_present = true;
            }
          }
          state->who_am_i = buffer[0];

          /**------------------Power Down and Close COM Port--------------------*/
          state->com_port_info.port_handle->com_port_api->sns_scp_update_bus_power(
                                                      state->com_port_info.port_handle,
                                                      false);

          state->com_port_info.port_handle->com_port_api->sns_scp_close(state->com_port_info.port_handle);
          sns_scp_deregister_com_port(state->com_port_info.port_handle);

          /**----------------------Turn Power Rail OFF--------------------------*/
          state->rail_config.rail_vote = SNS_RAIL_OFF;
          state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
                                                                   this,
                                                                   &state->rail_config,
                                                                   NULL);
          if(state->hw_is_present)
          {
             if(state->sensor == LSM6DS3_ACCEL)
             {
               lsm6ds3_accel_init_attributes(this);
             }
             else if(state->sensor == LSM6DS3_GYRO)
             {
               lsm6ds3_gyro_init_attributes(this);
             }
             else if(state->sensor == LSM6DS3_MOTION_ACCEL)
             {
               lsm6ds3_motion_accel_init_attributes(this);
             }
             else if(state->sensor == LSM6DS3_SENSOR_TEMP)
             {
               lsm6ds3_sensor_temp_init_attributes(this);
             }
             else
             {
               diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
                                        "Unsupported Sensor");
             }

            lsm6ds3_publish_attributes(this);
          }
          else
          {
            rv = SNS_RC_INVALID_STATE;
            diag->api->sensor_printf(diag, this, SNS_LOW, __FILENAME__, __LINE__,
                                     "LSM6DS3 HW absent");

            state->rail_config.rail_vote = SNS_RAIL_OFF;
            state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
                                                                     this,
                                                                     &state->rail_config,
                                                                     NULL);
          }
          state->power_rail_pend_state = LSM6DS3_POWER_RAIL_PENDING_NONE;
        }
        else if(state->power_rail_pend_state == LSM6DS3_POWER_RAIL_PENDING_SET_CLIENT_REQ)
        {
          sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
          lsm6ds3_instance_state *inst_state =
             (lsm6ds3_instance_state*) instance->state->state;
          if(NULL != instance)
          {
            inst_state->instance_is_ready_to_configure = true;
            lsm6ds3_reval_instance_config(this, instance, state->sensor);
          }
          state->power_rail_pend_state = LSM6DS3_POWER_RAIL_PENDING_NONE;
        }
      }
      else
      {
        diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
                                 "pb_decode error");
      }

      event = state->timer_stream->api->get_next_input(state->timer_stream);
    }
  }

#if LSM6DS3_ENABLE_DEPENDENCY
  if(NULL != state->reg_data_stream)
  {
    event = state->reg_data_stream->api->peek_input(state->reg_data_stream);
    while(NULL != event)
    {
#endif  //LSM6DS3_ENABLE_DEPENDENCY
#if LSM6DS3_USE_DEFAULTS
      state->com_port_info.com_config.bus_instance = SPI_BUS_INSTANCE;
      state->com_port_info.com_config.bus_type = SNS_BUS_SPI;
      state->com_port_info.com_config.max_bus_speed_KHz = SPI_BUS_MAX_FREQ_KHZ;
      state->com_port_info.com_config.min_bus_speed_KHz = SPI_BUS_MIN_FREQ_KHZ;
      state->com_port_info.com_config.reg_addr_type = SNS_REG_ADDR_8_BIT;
      state->com_port_info.com_config.slave_control = SPI_SLAVE_CONTROL;
      state->irq_info.irq_drive_strength = SNS_INTERRUPT_DRIVE_STRENGTH_2_MILLI_AMP;
      state->irq_info.irq_num = IRQ_NUM;
      state->irq_info.irq_pull = SNS_INTERRUPT_PULL_TYPE_KEEPER;
      state->irq_info.irq_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_RISING;
      state->irq_info.is_chip_pin = true;
#else   //LSM6DS3_USE_DEFAULTS
      //TODO update to use Registry Sensor data
#endif  //LSM6DS3_USE_DEFAULTS

      /**-----------------Register and Open COM Port-------------------------*/
      if(NULL == state->com_port_info.port_handle)
      {
        sns_scp_register_com_port(&state->com_port_info.com_config,
                                  &state->com_port_info.port_handle);

        state->com_port_info.port_handle->com_port_api->sns_scp_open(
                                  state->com_port_info.port_handle);
      }
      /**---------------------Register Power Rails --------------------------*/
      if(0 != sns_memcmp(&state->timer_suid, &((sns_sensor_uid){{0}}), sizeof(state->timer_suid))
         && NULL == state->pwr_rail_service)
      {
        state->rail_config.rail_vote = SNS_RAIL_OFF;
        state->rail_config.num_of_rails = NUM_OF_RAILS;
        strlcpy(state->rail_config.rails[0].name,
                RAIL_1,
                sizeof(state->rail_config.rails[0].name));
        strlcpy(state->rail_config.rails[1].name,
                RAIL_2,
                sizeof(state->rail_config.rails[1].name));

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
          state->rail_config.rail_vote = SNS_RAIL_ON_LPM;
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

#if LSM6DS3_ENABLE_DEPENDENCY
      event = state->reg_data_stream->api->get_next_input(state->reg_data_stream);
    }
  }
#endif  //LSM6DS3_ENABLE_DEPENDENCY
  return rv;
}

/* See sns_sensor::get_attributes */
sns_sensor_attribute *lsm6ds3_get_attributes(sns_sensor const *const this,
                                                          uint32_t *attributes_len)
{
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;

  struct sns_service_manager *smgr= this->cb->get_service_manager(this);
  state->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);

  sns_diag_service* diag = state->diag_service;
  diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);


  *attributes_len = ARR_SIZE(state->attributes);
  return state->attributes;
}

/**
 * Returns decoded request message for type
 * sns_motion_accel_config.
 *
 * @param[in] in_request   Request as sotred in client_requests
 *                         list.
 * @param decoded_request  Standard decoded message.
 * @param decoded_payload  Decoded stream request payload.
 *
 * @return bool true if decode is successful else false
 */
static bool lsm6ds3_get_decoded_motion_accel_request(sns_sensor *const this, sns_request const *in_request,
                                                     sns_std_request *decoded_request,
                                                     sns_motion_accel_config *decoded_payload)
{
  lsm6ds3_state *state = (lsm6ds3_state *) this->state->state;
  sns_diag_service* diag = state->diag_service;
  pb_istream_t stream;
  pb_simple_cb_arg arg =
      { .decoded_struct = decoded_payload,
        .fields = sns_motion_accel_config_fields };
  decoded_request->payload = (struct pb_callback_s)
      { .funcs.decode = &pb_decode_simple_cb, .arg = &arg };
  stream = pb_istream_from_buffer(in_request->request,
                                  in_request->request_len);
  if(!pb_decode(&stream, sns_std_request_fields, decoded_request))
  {
    diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
                             "LSM decode error");
    return false;
  }
  return true;
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

  lsm6ds3_state *state = (lsm6ds3_state *) this->state->state;
  sns_diag_service* diag = state->diag_service;
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
    diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
                             "LSM decode error");
    return false;
  }
  return true;
}

static void lsm6ds3_get_imu_config(sns_sensor *this,
                                   sns_sensor_instance *instance,
                                   lsm6ds3_sensor_type sensor_type,
                                   float *chosen_sample_rate,
                                   float *chosen_report_rate,
                                   bool *sensor_client_present)
{
  UNUSED_VAR(this);
  lsm6ds3_instance_state *inst_state =
     (lsm6ds3_instance_state*)instance->state->state;
  sns_sensor_uid suid;
  sns_request const *request;

  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;

  sns_service_manager *smgr = this->cb->get_service_manager(this);
  state->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);

  sns_diag_service* diag = state->diag_service;
  diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
 
  if(sensor_type == LSM6DS3_ACCEL)
  {
    sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)ACCEL_SUID), sizeof(sns_sensor_uid));
    sns_memscpy(&inst_state->accel_info.suid,
                sizeof(inst_state->accel_info.suid),
                &suid,
                sizeof(suid));
    diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
  }
  else
  {
    sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)GYRO_SUID), sizeof(sns_sensor_uid));
    sns_memscpy(&inst_state->gyro_info.suid,
                sizeof(inst_state->gyro_info.suid),
                &suid,
                sizeof(suid));
  }

  *chosen_report_rate = 0;
  *chosen_sample_rate = 0;
  *sensor_client_present = false;

  /** Parse through existing requests and get fastest sample
   *  rate and report rate requests. */
  for(request = instance->cb->get_client_request(instance, &suid, true);
      NULL != request;
      request = instance->cb->get_client_request(instance, &suid, false))
  {
    sns_std_request decoded_request;
    sns_std_sensor_config decoded_payload;
    diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
 
    if(lsm6ds3_get_decoded_imu_request(this, request, &decoded_request, &decoded_payload))
    {
      diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
      if(request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
      {
        diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
        if(lsm6ds3_get_decoded_imu_request(this, request, &decoded_request, &decoded_payload))
        {
           diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
           if(request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
           {
             float report_rate;
             *chosen_sample_rate = SNS_MAX(*chosen_sample_rate,
                                            decoded_payload.sample_rate);
             if(decoded_request.has_batch_period
                &&
                decoded_request.batch_period > 0)
             {
               report_rate = 1000000 / decoded_request.batch_period;
             }
             else
             {
               report_rate = *chosen_sample_rate;
             }
             *chosen_report_rate = SNS_MAX(*chosen_report_rate,
                                            report_rate);
             *sensor_client_present = true;
             diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,"report_rate=%f",report_rate);
           }
           else  // TODO handle self-test request
           {
           }
        }
      }
    }
  }
}

static void lsm6ds3_get_motion_accel_config(sns_sensor *this,
                                            sns_sensor_instance *instance,
                                            float *chosen_sample_rate,
                                            float *chosen_report_rate,
                                            bool *chosen_md_enable,
                                            bool *ma_sensor_client_present)
{
  UNUSED_VAR(this);
  sns_sensor_uid suid = MOTION_ACCEL_SUID;
  lsm6ds3_instance_state *inst_state =
     (lsm6ds3_instance_state*)instance->state->state;
  sns_request const *request;

  sns_memscpy(&inst_state->motion_accel_info.suid,
              sizeof(inst_state->motion_accel_info.suid),
              &suid,
              sizeof(suid));

  *chosen_sample_rate = 0;
  *chosen_report_rate = 0;
  *chosen_md_enable = false;
  *ma_sensor_client_present = false;

  if(NULL != instance->cb->get_client_request(instance, &suid, true))
  {
    *chosen_md_enable = true;
  }

  /** Parse through existing requests and get fastest sample
   *  rate and report rate requests. */
  for(request = instance->cb->get_client_request(instance, &suid, true);
      NULL != request;
      request = instance->cb->get_client_request(instance, &suid, false))
  {
    sns_std_request decoded_request;
    sns_motion_accel_config decoded_payload;

    if(lsm6ds3_get_decoded_motion_accel_request(this, request,
                                                &decoded_request,
                                                &decoded_payload))
    {
      if(request->message_id == SNS_MOTION_ACCEL_MSGID_SNS_MOTION_ACCEL_CONFIG)
      {
        float report_rate;
        *chosen_sample_rate = SNS_MAX(*chosen_sample_rate,
                                      decoded_payload.sample_rate);
        if(decoded_request.has_batch_period
           &&
           decoded_request.batch_period > 0)
        {
          report_rate = 1000000 / decoded_request.batch_period;
        }
        else
        {
          report_rate = *chosen_sample_rate;
        }
        *chosen_report_rate = SNS_MAX(*chosen_report_rate,
                                      report_rate);
        *chosen_md_enable &= decoded_payload.enable_motion_detect;
        *ma_sensor_client_present = true;
      }
      else
      {
        // TODO self-test request
      }
    }
  }
}

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

  sns_memscpy(&inst_state->sensor_temp_info.suid,
              sizeof(inst_state->sensor_temp_info.suid),
              &suid,
              sizeof(suid));

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
    sns_std_sensor_config decoded_payload;

    if(lsm6ds3_get_decoded_imu_request(this, request, &decoded_request, &decoded_payload))
    {
      if(request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
      {
        float report_rate;
        *chosen_sample_rate = SNS_MAX(*chosen_sample_rate,
                                       decoded_payload.sample_rate);
        if(decoded_request.has_batch_period
           &&
           decoded_request.batch_period > 0)
        {
          report_rate = 1000000 / decoded_request.batch_period;
        }
        else
        {
          report_rate = *chosen_sample_rate;
        }
        *chosen_report_rate = SNS_MAX(*chosen_report_rate,
                                       report_rate);
        inst_state->sensor_temp_info.report_timer_hz = *chosen_report_rate;
        *sensor_temp_client_present = true;
      }
      else  // TODO handle self-test request
      {
      }
    }
  }
}

static void lsm6ds3_set_imu_inst_config(sns_sensor *this,
                                        sns_sensor_instance *instance,
                                        float chosen_report_rate,
                                        float chosen_sample_rate)
{
  sns_lsm6ds3_imu_req new_client_config;
  sns_request config;

  new_client_config.report_rate = chosen_report_rate;
  new_client_config.sample_rate = chosen_sample_rate;

  config.message_id = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG;
  config.request_len = sizeof(sns_lsm6ds3_imu_req);
  config.request = &new_client_config;

  this->instance_api->set_client_config(instance, &config);
}

static void lsm6ds3_set_ma_inst_config(sns_sensor *this,
                                       sns_sensor_instance *instance,
                                       float chosen_report_rate,
                                       float chosen_sample_rate,
                                       bool chosen_md_enable)
{
  sns_request config;
  sns_lsm6ds3_ma_req new_client_config;

  new_client_config.report_rate = chosen_report_rate;
  new_client_config.sample_rate = chosen_sample_rate;
  new_client_config.enable_motion_detect = chosen_md_enable;

  config.message_id = SNS_MOTION_ACCEL_MSGID_SNS_MOTION_ACCEL_CONFIG;
  config.request_len = sizeof(sns_lsm6ds3_ma_req);
  config.request = &new_client_config;

  this->instance_api->set_client_config(instance, &config);
}

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

void lsm6ds3_reval_instance_config(sns_sensor *this,
                              sns_sensor_instance *instance,
                              lsm6ds3_sensor_type sensor_type)
{
  /**
   * 1. Get best Accel Config.
   * 2. Get best Gyro Config.
   * 3. Get best Motion Accel Config.
   * 4. Get best Sensor Temperature Config.
   * 5. Decide best Instance Config based on above outputs.
   */

  float chosen_sample_rate = 0;
  float chosen_report_rate = 0;
  float sample_rate = 0;
  float report_rate = 0;
  bool chosen_md_enable;
  bool a_sensor_client_present;
  bool g_sensor_client_present;
  bool ma_sensor_client_present;
  bool sensor_temp_client_present;
  lsm6ds3_instance_state *inst_state =
     (lsm6ds3_instance_state*)instance->state->state;

  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;

  sns_service_manager *smgr = this->cb->get_service_manager(this);
  state->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);

  sns_diag_service* diag = state->diag_service;
  diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
 
  lsm6ds3_get_imu_config(this,
                         instance,
                         LSM6DS3_ACCEL,
                         &chosen_sample_rate,
                         &chosen_report_rate,
                         &a_sensor_client_present);
  lsm6ds3_get_imu_config(this,
                         instance,
                         LSM6DS3_GYRO,
                         &sample_rate,
                         &report_rate,
                         &g_sensor_client_present);

  chosen_sample_rate = SNS_MAX(chosen_sample_rate, sample_rate);
  chosen_report_rate = SNS_MAX(chosen_report_rate, report_rate);

  lsm6ds3_get_motion_accel_config(this,
                                  instance,
                                  &sample_rate,
                                  &report_rate,
                                  &chosen_md_enable,
                                  &ma_sensor_client_present);

  chosen_sample_rate = SNS_MAX(chosen_sample_rate, sample_rate);
  chosen_report_rate = SNS_MAX(chosen_report_rate, report_rate);

  lsm6ds3_get_sensor_temp_config(this,
                                 instance,
                                 &sample_rate,
                                 &report_rate,
                                 &sensor_temp_client_present);

  chosen_sample_rate = SNS_MAX(chosen_sample_rate, sample_rate);
  chosen_report_rate = SNS_MAX(chosen_report_rate, report_rate);

  // TODO: all these checks can be optimized.
  if(a_sensor_client_present)
  {
    diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
    inst_state->fifo_info.fifo_enabled |= LSM6DS3_ACCEL;
    inst_state->fifo_info.publish_sensors |= LSM6DS3_ACCEL;
  }
  else if(g_sensor_client_present || ma_sensor_client_present)
  {
    diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
    inst_state->fifo_info.publish_sensors &= ~LSM6DS3_ACCEL;
  }
  else
  {
    diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
    inst_state->fifo_info.fifo_enabled &= ~LSM6DS3_ACCEL;
    inst_state->fifo_info.publish_sensors &= ~LSM6DS3_ACCEL;
  }

  if(g_sensor_client_present)
  {
    inst_state->fifo_info.fifo_enabled |= (LSM6DS3_ACCEL | LSM6DS3_GYRO);
    inst_state->fifo_info.publish_sensors |= LSM6DS3_GYRO;
  }
  else if(a_sensor_client_present || ma_sensor_client_present)
  {
    inst_state->fifo_info.fifo_enabled &= ~LSM6DS3_GYRO;
    inst_state->fifo_info.publish_sensors &= ~LSM6DS3_GYRO;
  }
  else
  {
    inst_state->fifo_info.fifo_enabled &= ~(LSM6DS3_ACCEL | LSM6DS3_GYRO);
    inst_state->fifo_info.publish_sensors &= ~LSM6DS3_GYRO;
  }

  if(ma_sensor_client_present)
  {
    inst_state->fifo_info.fifo_enabled |= (LSM6DS3_MOTION_ACCEL | LSM6DS3_ACCEL);
    inst_state->fifo_info.publish_sensors |= LSM6DS3_MOTION_ACCEL;
  }
  else if(a_sensor_client_present || g_sensor_client_present)
  {
    inst_state->fifo_info.fifo_enabled &= ~LSM6DS3_MOTION_ACCEL;
    inst_state->fifo_info.publish_sensors &= ~LSM6DS3_MOTION_ACCEL;
  }
  else
  {
    inst_state->fifo_info.fifo_enabled &= ~(LSM6DS3_MOTION_ACCEL | LSM6DS3_ACCEL);
    inst_state->fifo_info.publish_sensors &= ~LSM6DS3_MOTION_ACCEL;
  }

  if(sensor_temp_client_present)
  {
    inst_state->fifo_info.fifo_enabled |= LSM6DS3_ACCEL;
    inst_state->fifo_info.publish_sensors |= LSM6DS3_SENSOR_TEMP;
  }
  else if(a_sensor_client_present || g_sensor_client_present || ma_sensor_client_present)
  {
    inst_state->fifo_info.publish_sensors &= ~LSM6DS3_SENSOR_TEMP;
  }
  else
  {
    inst_state->fifo_info.fifo_enabled &= ~LSM6DS3_ACCEL;
    inst_state->fifo_info.publish_sensors &= ~LSM6DS3_SENSOR_TEMP;
  }

  inst_state->motion_accel_info.md_client_present = chosen_md_enable;

  if(a_sensor_client_present || g_sensor_client_present)
  {
    chosen_md_enable = false;
  }

  inst_state->motion_accel_info.enable_md_int = chosen_md_enable;

  if(sensor_type == LSM6DS3_ACCEL
     ||
     sensor_type == LSM6DS3_GYRO
     ||
     sensor_type == LSM6DS3_SENSOR_TEMP)
  {
    lsm6ds3_set_imu_inst_config(this,
                                instance,
                                chosen_report_rate,
                                chosen_sample_rate);
  }
  else
  {
    lsm6ds3_set_ma_inst_config(this,
                               instance,
                               chosen_report_rate,
                               chosen_sample_rate,
                               chosen_md_enable);
  }
  diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
 
  if(!inst_state->fifo_info.fifo_enabled)
  {
     diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
     lsm6ds3_turn_rails_off(this);
     inst_state->instance_is_ready_to_configure = false;
  }
}

/** See sns_lsm6ds3_sensor.h */
sns_sensor_instance* lsm6ds3_set_client_request(sns_sensor *const this,
                                                struct sns_request *exist_request,
                                                struct sns_request *new_request,
                                                bool remove)
{
  sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;
  sns_time on_timestamp;
  sns_time delta;
  bool reval_config = false;

  sns_service_manager *smgr = this->cb->get_service_manager(this);
  state->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);

  sns_diag_service* diag = state->diag_service;
  diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
 
  if(remove)
  {
    diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
    if(NULL != instance)
    {
      diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
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
          state->rail_config.rail_vote = SNS_RAIL_ON_LPM;
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
        if(0)  // flush_req
        {
           // TODO Flush FIFO samples.

           instance = NULL;
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
        }
        if(reval_config && inst_state->instance_is_ready_to_configure)
        {
          lsm6ds3_reval_instance_config(this, instance, state->sensor);
        }
     }
  }

  return instance;
}

/** See sns_lsm6ds3_sensor.h */
void lsm6ds3_send_suid_req(sns_sensor *this, char *const data_type,
    uint32_t data_type_len)
{
  uint8_t buffer[50];
  sns_memset(buffer, 0, sizeof(buffer));
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;
  sns_service_manager *manager = this->cb->get_service_manager(this);
  sns_stream_service *stream_service =
    (sns_stream_service*)manager->get_service(manager, SNS_STREAM_SERVICE);
  size_t encoded_len;
  pb_buffer_arg data = (pb_buffer_arg){ .buf = data_type, .buf_len = data_type_len };

  sns_suid_req suid_req = sns_suid_req_init_default;
  suid_req.has_register_updates = true;
  suid_req.register_updates = true;
  suid_req.data_type.funcs.encode = &pb_encode_string_cb;
  suid_req.data_type.arg = &data;

  if(state->fw_stream == NULL)
  {
     stream_service->api->create_sensor_stream(stream_service,
         this, sns_get_suid_lookup(), &state->fw_stream);
  }

  encoded_len = pb_encode_request(buffer, sizeof(buffer),
      &suid_req, sns_suid_req_fields, NULL);
  if(0 < encoded_len)
  {
    sns_request request = (sns_request){
      .request_len = encoded_len, .request = buffer, .message_id = SNS_SUID_MSGID_SNS_PB_SUID_REQ };
    state->fw_stream->api->send_request(state->fw_stream, &request);
  }
}

/** See sns_lsm6ds3_sensor.h */
void lsm6ds3_process_suid_events(sns_sensor *const this)
{
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;

  sns_diag_service* diag = state->diag_service;

  for(;
      0 != state->fw_stream->api->get_input_cnt(state->fw_stream);
      state->fw_stream->api->get_next_input(state->fw_stream))
  {
    sns_sensor_event *event =
      state->fw_stream->api->peek_input(state->fw_stream);

    if(SNS_SUID_MSGID_SNS_PB_SUID_EVENT == event->message_id)
    {
      sns_sensor_uid suid;
      int num_suids_found = 0;
      char const *datatype_name;
      int datatype_name_len;

      if (!pb_decode_suid_event(event, &suid, 1, &num_suids_found,
                                &datatype_name, &datatype_name_len))
      {
        diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
                                 "pb_decode_suid_event() failed");
        continue;
      }

      /* if no suids found, ignore the event */
      if (num_suids_found == 0)
      {
        continue;
      }

      /* save suid based on incoming data type name */
      if (0 == strncmp(datatype_name, "interrupt", datatype_name_len))
      {
        state->irq_suid = suid;
      }
      else if(0 == strncmp(datatype_name, "timer", datatype_name_len))
      {
        state->timer_suid = suid;
      }
      else if (0 == strncmp(datatype_name, "async_com_port",
                            datatype_name_len))
      {
        state->acp_suid = suid;
      }
      else if (0 == strncmp(datatype_name, "registry", datatype_name_len))
      {
        state->reg_suid = suid;
      }
      else
      {
        diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
                                 "invalid datatype_name %s", datatype_name);
      }
    }
  }
}

