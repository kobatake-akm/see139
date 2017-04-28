/**
 * @file sns_lsm6ds3_sensor_island.c
 *
 * Common implementation for LSM6DS3 Sensors for island mode.
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc. All Rights 
 * Reserved. Confidential and Proprietary - Qualcomm 
 * Technologies, Inc. 
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
#include "sns_diag_service.h"
#include "sns_attribute_util.h"
#include "sns_sync_com_port_service.h"

#include "sns_lsm6ds3_sensor.h"
#include "sns_lsm6ds3_hal.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_std.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_motion_detect.pb.h"
#include "sns_std_event_gated_sensor.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"

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

  if(state->fw_stream)
  {
    if((0 == sns_memcmp(&state->irq_suid, &((sns_sensor_uid){{0}}), sizeof(state->irq_suid)))
     || (0 == sns_memcmp(&state->acp_suid, &((sns_sensor_uid){{0}}), sizeof(state->acp_suid)))
     || (0 == sns_memcmp(&state->timer_suid, &((sns_sensor_uid){{0}}), sizeof(state->timer_suid)))
     || (0 == sns_memcmp(&state->dae_suid, &((sns_sensor_uid){{0}}), sizeof(state->dae_suid)))
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
          if(NULL != instance)
          {
            lsm6ds3_instance_state *inst_state =
               (lsm6ds3_instance_state*) instance->state->state;
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
      state->com_port_info.com_config.bus_instance = BUS_INSTANCE;
      state->com_port_info.com_config.bus_type = BUS_TYPE;
      state->com_port_info.com_config.max_bus_speed_KHz = BUS_FREQ;
      state->com_port_info.com_config.min_bus_speed_KHz = SPI_BUS_MIN_FREQ_KHZ;
      state->com_port_info.com_config.reg_addr_type = SNS_REG_ADDR_8_BIT;
      state->com_port_info.com_config.slave_control = SLAVE_CONTROL;
      state->irq_config.interrupt_drive_strength = SNS_INTERRUPT_DRIVE_STRENGTH_2_MILLI_AMP;
      state->irq_config.interrupt_num = IRQ_NUM;
      state->irq_config.interrupt_pull_type = SNS_INTERRUPT_PULL_TYPE_KEEPER;
      state->irq_config.interrupt_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_RISING;
      state->irq_config.is_chip_pin = true;
#else   //LSM6DS3_USE_DEFAULTS
      //TODO update to use Registry Sensor data
#endif  //LSM6DS3_USE_DEFAULTS

      /**-----------------Register and Open COM Port-------------------------*/
      if(NULL == state->com_port_info.port_handle)
      {
        state->scp_service->api->sns_scp_register_com_port(&state->com_port_info.com_config,
                                                &state->com_port_info.port_handle);

        state->scp_service->api->sns_scp_open(state->com_port_info.port_handle);
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

static void  lsm6ds3_send_flush_config(sns_sensor *const this,
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

static void lsm6ds3_get_motion_detect_config(sns_sensor *this,
                                             sns_sensor_instance *instance,
                                             bool *chosen_md_enable,
                                             bool *md_client_present)
{
  UNUSED_VAR(this);
  sns_sensor_uid suid = MOTION_DETECT_SUID;
  lsm6ds3_instance_state *inst_state =
     (lsm6ds3_instance_state*)instance->state->state;

  *chosen_md_enable = false;

  if(NULL != instance->cb->get_client_request(instance, &suid, true))
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
        *chosen_report_rate = SNS_MAX(*chosen_report_rate, report_rate);
        *sensor_temp_client_present = true;
      }
    }
  }
  inst_state->sensor_temp_info.report_rate_hz  = *chosen_report_rate;
  inst_state->sensor_temp_info.sampling_rate_hz = *chosen_sample_rate;
}

static void lsm6ds3_set_inst_config(sns_sensor *this,
                                    sns_sensor_instance *instance,
                                    float chosen_report_rate,
                                    float chosen_sample_rate,
                                    uint32_t message_id)
{
  sns_lsm6ds3_req new_client_config;
  sns_request config;

  new_client_config.desired_report_rate = chosen_report_rate;
  new_client_config.desired_sample_rate = chosen_sample_rate;

  config.message_id = message_id;
  config.request_len = sizeof(sns_lsm6ds3_req);
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
   * 1. Get best non-gated and gated Accel Config.
   * 2. Get best Gyro Config.
   * 3. Get Motion Detect Config.
   * 4. Get best Sensor Temperature Config.
   * 5. Decide best Instance Config based on above outputs.
   */
  UNUSED_VAR(sensor_type);
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

  //md_sensor_client_present = inst_state->md_info.enable_md_int;

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

  lsm6ds3_set_inst_config(this,
                          instance,
                          chosen_report_rate,
                          chosen_sample_rate,
                          SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG);

  if(!inst_state->fifo_info.fifo_enabled)
  {
    lsm6ds3_turn_rails_off(this);
    inst_state->instance_is_ready_to_configure = false;
  }
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
        }
        if(reval_config && inst_state->instance_is_ready_to_configure)
        {
          lsm6ds3_reval_instance_config(this, instance, state->sensor);
        }
     }
  }

  // QC: Sensors are required to call remove_instance when clientless
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
  }

  return instance;
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
         diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
                                 "SUID Decode failed");
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
        diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
                                 "invalid datatype_name %s", data_type_arg.buf);
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

