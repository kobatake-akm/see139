/**
 * @file sns_ak0991x_sensor.c
 *
 * Common implementation for AK0991X Sensors.
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
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_event_service.h"
#include "sns_stream_service.h"
#include "sns_service.h"
#include "sns_sensor_util.h"
#include "sns_math_util.h"
#include "sns_types.h"

#include "sns_ak0991x_sensor.h"
#include "sns_ak0991x_hal.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_std.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"
#include "sns_diag_service.h"
#include "sns_sync_com_port_service.h"


static void ak0991x_start_power_rail_timer(sns_sensor *const this,
                                           sns_time timeout_ticks,
                                           ak0991x_power_rail_pending_state pwr_rail_pend_state)
{
  ak0991x_state *state = (ak0991x_state *)this->state->state;

  sns_diag_service *diag = state->diag_service;

  sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
  size_t                  req_len;
  uint8_t                 buffer[20];
  sns_memset(buffer, 0, sizeof(buffer));
  req_payload.is_periodic = false;
  req_payload.start_time = sns_get_system_time();
  req_payload.timeout_period = timeout_ticks;

  req_len = pb_encode_request(buffer, sizeof(buffer), &req_payload,
                              sns_timer_sensor_config_fields, NULL);

  if (req_len > 0)
  {
    sns_request timer_req =
    {  .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
       .request = buffer, .request_len = req_len};
    state->timer_stream->api->send_request(state->timer_stream, &timer_req);
    state->power_rail_pend_state = pwr_rail_pend_state;
  }
  else
  {
    diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
                             "AK0991x timer req encode error");
  }
}

/** See sns_ak0991x_sensor.h*/
sns_rc ak0991x_sensor_notify_event(sns_sensor *const this)
{
  ak0991x_state       *state = (ak0991x_state *)this->state->state;
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service  *stream_svc = (sns_stream_service *)
    service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
  sns_sync_com_port_service * scp_service = (sns_sync_com_port_service *)
    service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);

  sns_time         on_timestamp;
  uint8_t          buffer[AK0991X_NUM_READ_DEV_ID];
  sns_rc           rv = SNS_RC_SUCCESS;
  sns_sensor_event *event;

  sns_diag_service *diag = state->diag_service;

  if (state->fw_stream)
  {
    if ((0 == sns_memcmp(&state->irq_suid, &((sns_sensor_uid){{0}}), sizeof(state->irq_suid)))
        || (0 == sns_memcmp(&state->acp_suid, &((sns_sensor_uid){{0}}), sizeof(state->acp_suid)))
        || (0 ==
            sns_memcmp(&state->timer_suid, &((sns_sensor_uid){{0}}), sizeof(state->timer_suid)))
#if AK0991X_ENABLE_DEPENDENCY
        || (0 == sns_memcmp(&state->reg_suid, &((sns_sensor_uid){{0}}), sizeof(state->reg_suid)))
#endif
    )
    {
      ak0991x_process_suid_events(this);
    }
  }

  /**----------------------Handle a Timer Sensor event.-------------------*/
  if (NULL != state->timer_stream)
  {
    event = state->timer_stream->api->peek_input(state->timer_stream);

    while (NULL != event)
    {
      pb_istream_t stream = pb_istream_from_buffer((pb_byte_t *)event->event,
                                                   event->event_len);
      sns_timer_sensor_event timer_event;

      if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
      {
        if (state->power_rail_pend_state == AK0991X_POWER_RAIL_PENDING_INIT)
        {
          /**-------------------Read and Confirm WHO-AM-I------------------------*/
          rv = ak0991x_get_who_am_i(state->scp_service,
                                    state->com_port_info.port_handle, &buffer[0]);

          if (rv != SNS_RC_SUCCESS)
          {
            return rv;
          }

          state->who_am_i = buffer[1] << 8 | buffer[0];

          //Check AKM device ID
          if (buffer[0] == AK0991X_WHOAMI_COMPANY_ID)
          {
            if (buffer[1] == AK09911_WHOAMI_DEV_ID)
            {
              state->device_select = AK09911;
            }
            else if (buffer[1] == AK09912_WHOAMI_DEV_ID)
            {
              state->device_select = AK09912;
            }
            else if (buffer[1] == AK09913_WHOAMI_DEV_ID)
            {
              state->device_select = AK09913;
            }
            else if ((buffer[1] == AK09915_WHOAMI_DEV_ID) && (buffer[3] == AK09915C_SUB_ID))
            {
              state->device_select = AK09915C;
            }
            else if ((buffer[1] == AK09915_WHOAMI_DEV_ID) && (buffer[3] == AK09915D_SUB_ID))
            {
              state->device_select = AK09915D;
            }
            else if (buffer[1] == AK09916C_WHOAMI_DEV_ID)
            {
              state->device_select = AK09916C;
            }
            else if (buffer[1] == AK09916D_WHOAMI_DEV_ID)
            {
              state->device_select = AK09916D;
            }
            else if (buffer[1] == AK09918_WHOAMI_DEV_ID)
            {
              state->device_select = AK09918;
            }
            else
            {
              diag->api->sensor_printf(diag, this, SNS_LOW, __FILENAME__, __LINE__,
                                       "Unsupported Sensor");
              return SNS_RC_FAILED;
            }
          }
          else
          {
            diag->api->sensor_printf(diag, this, SNS_LOW, __FILENAME__, __LINE__,
                                     "Unsupported Sensor");
            return SNS_RC_FAILED;
          }

#if (!AK0991X_ENABLE_DEPENDENCY)
          // IRQ settings, it depends on the device.
          state->irq_info.irq_drive_strength = SNS_INTERRUPT_DRIVE_STRENGTH_2_MILLI_AMP;
          state->irq_info.irq_num = IRQ_NUM;

          switch (state->device_select)
          {
          case AK09911:
            state->irq_info.irq_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_RISING;
            state->irq_info.is_chip_pin = true;
            state->irq_info.irq_pull = SNS_INTERRUPT_PULL_TYPE_KEEPER;
            break;

          case AK09912:
            state->irq_info.irq_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_RISING;
            state->irq_info.is_chip_pin = true;
            state->irq_info.irq_pull = SNS_INTERRUPT_PULL_TYPE_KEEPER;
            break;

          case AK09913:
            state->irq_info.irq_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_RISING;
            state->irq_info.is_chip_pin = true;
            state->irq_info.irq_pull = SNS_INTERRUPT_PULL_TYPE_KEEPER;
            break;

          case AK09915C:
            state->irq_info.irq_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_RISING;
            state->irq_info.is_chip_pin = true;
            state->irq_info.irq_pull = SNS_INTERRUPT_PULL_TYPE_KEEPER;
            break;

          case AK09915D:
            state->irq_info.irq_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_FALLING;
            state->irq_info.is_chip_pin = true;
            state->irq_info.irq_pull = SNS_INTERRUPT_PULL_TYPE_PULL_UP;
            break;

          case AK09916C:
            state->irq_info.irq_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_RISING;
            state->irq_info.is_chip_pin = true;
            state->irq_info.irq_pull = SNS_INTERRUPT_PULL_TYPE_KEEPER;
            break;

          case AK09916D:
            state->irq_info.irq_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_FALLING;
            state->irq_info.is_chip_pin = true;
            state->irq_info.irq_pull = SNS_INTERRUPT_PULL_TYPE_PULL_UP;
            break;

          case AK09918:
            state->irq_info.irq_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_RISING;
            state->irq_info.is_chip_pin = true;
            state->irq_info.irq_pull = SNS_INTERRUPT_PULL_TYPE_KEEPER;
            break;

          default:
            diag->api->sensor_printf(diag, this, SNS_LOW, __FILENAME__, __LINE__,
                                     "Unsupported Sensor");
            return SNS_RC_FAILED;
          }
#endif //AK0991X_ENABLE_DEPENDENCY

          // Set sensitivity adjustment data
          rv = ak0991x_set_sstvt_adj(state->scp_service,
                                     state->com_port_info.port_handle, state->device_select,
                                     &state->sstvt_adj[0]);

          if (rv != SNS_RC_SUCCESS)
          {
            return rv;
          }

          // Reset Sensor
          rv = ak0991x_device_sw_reset(state->scp_service,
                                       state->com_port_info.port_handle);

          if (rv == SNS_RC_SUCCESS)
          {
            state->hw_is_present = true;
          }

          /**------------------Power Down and Close COM Port--------------------*/
          state->scp_service->api->sns_scp_update_bus_power(
            state->com_port_info.port_handle,
            false);

          state->scp_service->api->sns_scp_close(
            state->com_port_info.port_handle);
          state->scp_service->api->sns_scp_deregister_com_port(state->com_port_info.port_handle);

          /**----------------------Turn Power Rail OFF--------------------------*/
          state->rail_config.rail_vote = SNS_RAIL_OFF;
          state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
                                                                   this,
                                                                   &state->rail_config,
                                                                   NULL);

          if (state->hw_is_present)
          {
            ak0991x_publish_attributes(this, state->device_select);
          }
          else
          {
            rv = SNS_RC_INVALID_STATE;
            diag->api->sensor_printf(diag, this, SNS_LOW, __FILENAME__, __LINE__,
                                     "AK0991X HW absent");
            state->rail_config.rail_vote = SNS_RAIL_OFF;
            state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
                                                                     this,
                                                                     &state->rail_config,
                                                                     NULL);
          }

          state->power_rail_pend_state = AK0991X_POWER_RAIL_PENDING_NONE;
        }
        else if (state->power_rail_pend_state == AK0991X_POWER_RAIL_PENDING_SET_CLIENT_REQ)
        {
          sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);

          if (NULL != instance)
          {
            ak0991x_reval_instance_config(this, instance);
          }

          state->power_rail_pend_state = AK0991X_POWER_RAIL_PENDING_NONE;
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

#if AK0991X_ENABLE_DEPENDENCY

  if (NULL != state->reg_data_stream)
  {
    event = state->reg_data_stream->api->peek_input(state->reg_data_stream);

    while (NULL != event)
    {
#endif //AK0991X_ENABLE_DEPENDENCY
#if AK0991X_USE_DEFAULTS

      if (AK0991X_BUS_TYPE == AK0991X_SPI)
      {
        state->com_port_info.com_config.bus_instance = SPI_BUS_INSTANCE;
        state->com_port_info.com_config.bus_type = SNS_BUS_SPI;
        state->com_port_info.com_config.max_bus_speed_KHz = SPI_BUS_MAX_FREQ_KHZ;
        state->com_port_info.com_config.min_bus_speed_KHz = SPI_BUS_MIN_FREQ_KHZ;
        state->com_port_info.com_config.reg_addr_type = SNS_REG_ADDR_8_BIT;
        state->com_port_info.com_config.slave_control = SPI_SLAVE_CONTROL;
      }
      else
      {
        state->com_port_info.com_config.bus_instance = I2C_BUS_INSTANCE;
        state->com_port_info.com_config.bus_type = SNS_BUS_I2C;
        state->com_port_info.com_config.max_bus_speed_KHz = I2C_BUS_FREQ;
        state->com_port_info.com_config.min_bus_speed_KHz = I2C_BUS_FREQ;
        state->com_port_info.com_config.reg_addr_type = SNS_REG_ADDR_8_BIT;
        state->com_port_info.com_config.slave_control = I2C_SLAVE_ADDRESS;
      }

#if AK0991X_ENABLE_DEPENDENCY
      // IRQ settings, it depends on the device.
      state->irq_info.irq_drive_strength = SNS_INTERRUPT_DRIVE_STRENGTH_2_MILLI_AMP;
      state->irq_info.irq_num = IRQ_NUM;
      state->irq_info.irq_pull = AK0991X_INTERRUPT_PULL_TYPE;
      state->irq_info.irq_trigger_type = AK0991X_INTERRUPT_TRIGGER_TYPE;
#endif //AK0991X_ENABLE_DEPENDENCY
#else   //AK0991X_USE_DEFAULTS
      //TODO update to use Registry Sensor data
#endif //AK0991X_USE_DEFAULTS

      /**-----------------Register and Open COM Port-------------------------*/
      if (NULL == state->com_port_info.port_handle)
      {
          scp_service->api->sns_scp_register_com_port(&state->com_port_info.com_config,
                                                      &state->com_port_info.port_handle);

          state->scp_service->api->sns_scp_open(
          state->com_port_info.port_handle);
      }

      /**---------------------Register Power Rails --------------------------*/
      if (0 != sns_memcmp(&state->timer_suid, &((sns_sensor_uid){{0}}), sizeof(state->timer_suid))
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
          (sns_pwr_rail_service *)service_mgr->get_service(service_mgr,
                                                           SNS_POWER_RAIL_SERVICE);

        state->pwr_rail_service->api->sns_register_power_rails(state->pwr_rail_service,
                                                               &state->rail_config);

        /**---------------------Turn Power Rails ON----------------------------*/
        state->rail_config.rail_vote = SNS_RAIL_ON_NPM;
        state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
                                                                 this,
                                                                 &state->rail_config,
                                                                 &on_timestamp);

        /**-------------Create a Timer stream for Power Rail ON timeout.---------*/
        if (NULL == state->timer_stream)
        {
          stream_svc->api->create_sensor_stream(stream_svc, this, state->timer_suid,
                                                &state->timer_stream);

          if (NULL != state->timer_stream)
          {
            ak0991x_start_power_rail_timer(this,
                                           sns_convert_ns_to_ticks(
                                             AK0991X_OFF_TO_IDLE_MS * 1000 * 1000),
                                           AK0991X_POWER_RAIL_PENDING_INIT);
          }
        }

        diag->api->sensor_printf(diag, this, SNS_LOW, __FILENAME__, __LINE__,
                                 "power rail settings finished");
      }

#if AK0991X_ENABLE_DEPENDENCY
    event = state->reg_data_stream->api->get_next_input(state->reg_data_stream);
    }
  }
#endif //AK0991X_ENABLE_DEPENDENCY

  return rv;
}

/**
 * Returns decoded request message for type
 * sns_sensor_stream_config.
 *
 * @param[in] in_request   Request as sotred in client_requests
 *                         list.
 * @param decoded_request  Standard decoded message.
 * @param decoded_payload  Decoded stream request payload.
 *
 * @return bool true if decode is successful else false
 */
static bool ak0991x_get_decoded_mag_request(sns_sensor const *this,
                                            sns_request const *in_request,
                                            sns_std_request *decoded_request,
                                            sns_std_sensor_config *decoded_payload)
{
  ak0991x_state *state = (ak0991x_state *) this->state->state;
  sns_diag_service *diag = state->diag_service;
  pb_istream_t stream;
  pb_simple_cb_arg arg =
  { .decoded_struct = decoded_payload,
    .fields = sns_std_sensor_config_fields };
  decoded_request->payload = (struct pb_callback_s)
  { .funcs.decode = &pb_decode_simple_cb, .arg = &arg };
  stream = pb_istream_from_buffer(in_request->request,
                                  in_request->request_len);

  if (!pb_decode(&stream, sns_std_request_fields, decoded_request))
  {
    diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
                             "AK0991X decode error");
    return false;
  }

  return true;
}

static void ak0991x_get_mag_config(sns_sensor *this,
                                   sns_sensor_instance *instance,
                                   float *chosen_sample_rate,
                                   float *chosen_report_rate,
                                   bool *sensor_client_present)
{
  ak0991x_state *state = (ak0991x_state *)this->state->state;
  sns_diag_service *diag = state->diag_service;

  ak0991x_instance_state *inst_state =
    (ak0991x_instance_state *)instance->state->state;

  sns_sensor_uid suid;
  sns_request const *request;

  // QC: this memscpy can be removed. See LSM6DS3 example.
  sns_memscpy(&suid, sizeof(suid), &((sns_sensor_uid)MAG_SUID), sizeof(sns_sensor_uid));

  // QC: this memscpy can be removed. See LSM6DS3 example.
  sns_memscpy(&inst_state->mag_info.suid,
              sizeof(inst_state->mag_info.suid),
              &suid,
              sizeof(suid));

  *chosen_report_rate = 0;
  *chosen_sample_rate = 0;
  *sensor_client_present = false;

  /** Parse through existing requests and get fastest sample
   *  rate and report rate requests. */
  for (request = instance->cb->get_client_request(instance, &suid, true);
       NULL != request;
       request = instance->cb->get_client_request(instance, &suid, false))
  {
    sns_std_request decoded_request;
    sns_std_sensor_config decoded_payload;

    if (ak0991x_get_decoded_mag_request(this, request, &decoded_request, &decoded_payload))
    {
      if (request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
      {
        if (ak0991x_get_decoded_mag_request(this, request, &decoded_request, &decoded_payload))
        {
          if (request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
          {
            float report_rate;
            *chosen_sample_rate = SNS_MAX(*chosen_sample_rate,
                                          decoded_payload.sample_rate);

            if (decoded_request.has_batch_period
                &&
                decoded_request.batch_period > 0)
            {
              report_rate = 1000000.f / (float)decoded_request.batch_period;
            }
            else
            {
              report_rate = *chosen_sample_rate;
            }

            *chosen_report_rate = SNS_MAX(*chosen_report_rate,
                                          report_rate);
            *sensor_client_present = true;
          }
          else // TODO handle self-test request
          {
            uint32_t err = 0;
            sns_rc rv;
            rv = ak0991x_self_test(state->scp_service,
                                   state->com_port_info.port_handle,
                                   state->device_select,
                                   state->sstvt_adj,
                                   &err);

            if (rv != SNS_RC_SUCCESS)
            {
              diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
                                       "Test failed, err code = %ld", err);
            }
            else
            {
              diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
                                       "Test passed");
            }
          }
        }
      }
    }
  }
}

static void ak0991x_set_mag_inst_config(sns_sensor *this,
                                        sns_sensor_instance *instance,
                                        float chosen_report_rate,
                                        float chosen_sample_rate)
{
  sns_ak0991x_mag_req new_client_config;
  sns_request config;

  new_client_config.report_rate = chosen_report_rate;
  new_client_config.sample_rate = chosen_sample_rate;

  config.message_id = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG;
  config.request_len = sizeof(sns_ak0991x_mag_req);
  config.request = &new_client_config;

  this->instance_api->set_client_config(instance, &config);
}

void ak0991x_reval_instance_config(sns_sensor *this,
                                   sns_sensor_instance *instance)
{
  /**
   * 1. Get best Mag Config.
   * 2. Decide best Instance Config based on above outputs.
   */
  float chosen_sample_rate = 0;
  float chosen_report_rate = 0;
  float sample_rate = 0;
  float report_rate = 0;
  bool m_sensor_client_present;
  UNUSED_VAR(instance);

  //QC: below computations can be optimized to remove sample_rate and report_rate

  ak0991x_get_mag_config(this,
                         instance,
                         &chosen_sample_rate,
                         &chosen_report_rate,
                         &m_sensor_client_present);

  chosen_sample_rate = SNS_MAX(chosen_sample_rate, sample_rate);
  chosen_report_rate = SNS_MAX(chosen_report_rate, report_rate);

  ak0991x_set_mag_inst_config(this,
                              instance,
                              chosen_report_rate,
                              chosen_sample_rate);
}

/** See sns_ak0991x_sensor.h */
sns_sensor_instance *ak0991x_set_client_request(sns_sensor *const this,
                                                struct sns_request *exist_request,
                                                struct sns_request *new_request,
                                                bool remove)
{
  sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
  ak0991x_state *state = (ak0991x_state *)this->state->state;
  sns_time on_timestamp;
  sns_time delta;
  bool reval_config = false;

  if (remove)
  {
    if (NULL != instance)
    {
      instance->cb->remove_client_request(instance, exist_request);
      /* Assumption: The FW will call deinit() on the instance before destroying it.
                   Putting all HW resources (sensor HW, COM port, power rail)in
                   low power state happens in Instance deinit().*/

      ak0991x_reval_instance_config(this, instance);

      sns_sensor *sensor;

      for (sensor = this->cb->get_library_sensor(this, true);
           NULL != sensor;
           sensor = this->cb->get_library_sensor(this, false))
      {
        ak0991x_state *sensor_state = (ak0991x_state *)sensor->state->state;

        if (sensor_state->rail_config.rail_vote != SNS_RAIL_OFF)
        {
          sensor_state->rail_config.rail_vote = SNS_RAIL_OFF;
          sensor_state->pwr_rail_service->api->sns_vote_power_rail_update(
            sensor_state->pwr_rail_service,
            sensor,
            &sensor_state->rail_config,
            NULL);
        }
      }
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
    //     h. Power OFF COM port if not needed- Instance must handle COM port power.
    //     g. Return the Instance.
    // 2. If there is an Instance already present:
    //     a. Add new_request to list of requests handled by the Instance.
    //     b. Remove exist_request from list of requests handled by the Instance.
    //     c. Re-evaluate existing requests and choose appropriate instance config.
    //     d. set_client_config for the instance if not the same as current config.
    //     e. publish the updated config
    //     f. Return the Instance.
    // 3.  If "flush" request:
    //     a. Perform flush on the instance.
    //     b. Return NULL.

    if (NULL == instance)
    {
      state->rail_config.rail_vote = SNS_RAIL_ON_NPM;
      state->pwr_rail_service->api->sns_vote_power_rail_update(
        state->pwr_rail_service,
        this,
        &state->rail_config,
        &on_timestamp);

      delta = sns_get_system_time() - on_timestamp;

      // Use on_timestamp to determine correct Timer value.
      if (delta < sns_convert_ns_to_ticks(AK0991X_OFF_TO_IDLE_MS * 1000 * 1000))
      {
        ak0991x_start_power_rail_timer(this,
                                       sns_convert_ns_to_ticks(
                                         AK0991X_OFF_TO_IDLE_MS * 1000 * 1000) - delta,
                                       AK0991X_POWER_RAIL_PENDING_SET_CLIENT_REQ);
      }
      else
      {
        // rail is already ON
        reval_config = true;
      }

      /** create_instance() calls init() for the Sensor Instance */
      instance = this->cb->create_instance(this,
                                           sizeof(ak0991x_instance_state));
    }
    else
    {
      if (0) // flush_req
      {
        // TODO Flush FIFO samples.
        ak0991x_instance_state *inst_state =
          (ak0991x_instance_state *)instance->state->state;

        if (inst_state->mag_info.curr_odr != AK0991X_MAG_ODR_OFF)
        {
          ak0991x_flush_fifo(instance);
        }

        instance = NULL;
      }
      else
      {
        reval_config = true;

        /** An existing client is changing request*/
        if ((NULL != exist_request) && (NULL != new_request))
        {
          instance->cb->remove_client_request(instance, exist_request);
        }
        /** A new client sent new_request*/
        else if (NULL != new_request)
        {
          // No-op. new_request will be added to requests list below.
        }
      }
    }

    /** Add the new request to list of client_requests.*/
    if (NULL != instance)
    {
      if (NULL != new_request)
      {
        instance->cb->add_client_request(instance, new_request);
      }

      if (reval_config)
      {
        ak0991x_reval_instance_config(this, instance);
      }
    }
  }

  return instance;
}

/** See sns_ak0991x_sensor.h */
void ak0991x_send_suid_req(sns_sensor *this,
                           char *const data_type,
                           uint32_t data_type_len)
{
  uint8_t buffer[50];
  sns_memset(buffer, 0, sizeof(buffer));
  ak0991x_state *state = (ak0991x_state *)this->state->state;
  sns_service_manager *manager = this->cb->get_service_manager(this);
  sns_stream_service *stream_service =
    (sns_stream_service *)manager->get_service(manager, SNS_STREAM_SERVICE);
  size_t encoded_len;
  pb_buffer_arg data = (pb_buffer_arg){ .buf = data_type, .buf_len = data_type_len };

  sns_suid_req suid_req = sns_suid_req_init_default;
  suid_req.has_register_updates = true;
  suid_req.register_updates = true;
  suid_req.data_type.funcs.encode = &pb_encode_string_cb;
  suid_req.data_type.arg = &data;

  if (state->fw_stream == NULL)
  {
    stream_service->api->create_sensor_stream(stream_service,
                                              this, sns_get_suid_lookup(), &state->fw_stream);
  }

  encoded_len = pb_encode_request(buffer, sizeof(buffer),
                                  &suid_req, sns_suid_req_fields, NULL);

  if (0 < encoded_len)
  {
    sns_request request = (sns_request){
      .request_len = encoded_len, .request = buffer, .message_id = SNS_SUID_MSGID_SNS_SUID_REQ
    };
    state->fw_stream->api->send_request(state->fw_stream, &request);
  }
}

/** See sns_ak0991x_sensor.h */
void ak0991x_process_suid_events(sns_sensor *const this)
{
  ak0991x_state *state = (ak0991x_state *)this->state->state;

  sns_diag_service *diag = state->diag_service;

  for (;
       0 != state->fw_stream->api->get_input_cnt(state->fw_stream);
       state->fw_stream->api->get_next_input(state->fw_stream))
  {
    sns_sensor_event *event =
      state->fw_stream->api->peek_input(state->fw_stream);

    if (SNS_SUID_MSGID_SNS_SUID_EVENT == event->message_id)
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
      else if (0 == strncmp(datatype_name, "timer", datatype_name_len))
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
