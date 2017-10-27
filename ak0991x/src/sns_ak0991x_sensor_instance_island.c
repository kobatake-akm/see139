/**
 * @file sns_ak0991x_sensor_instance_island.c
 *
 * AK0991X Mag virtual Sensor Instance implementation.
 *
 * Copyright (c) 2016-2017 Asahi Kasei Microdevices
 * All Rights Reserved.
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 **/

#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_types.h"

#include "sns_ak0991x_hal.h"
#include "sns_ak0991x_sensor.h"
#include "sns_ak0991x_sensor_instance.h"
#include "sns_ak0991x_s4s.h"

#include "sns_interrupt.pb.h"
#include "sns_async_com_port.pb.h"
#include "sns_timer.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"
#include "sns_diag_service.h"
#include "sns_sync_com_port_service.h"
#include "sns_diag.pb.h"

extern log_sensor_state_raw_info log_mag_state_raw_info;

const odr_reg_map reg_map_ak0991x[AK0991X_REG_MAP_TABLE_SIZE] = {
  {
    .odr = AK0991X_ODR_0,
    .mag_odr_reg_value = AK0991X_MAG_ODR_OFF,
  },
  {
    .odr = AK0991X_ODR_1,
    .mag_odr_reg_value = AK0991X_MAG_ODR1,
  },
  {
    .odr = AK0991X_ODR_10,
    .mag_odr_reg_value = AK0991X_MAG_ODR10,
  },
  {
    .odr = AK0991X_ODR_20,
    .mag_odr_reg_value = AK0991X_MAG_ODR20,
  },
  {
    .odr = AK0991X_ODR_50,
    .mag_odr_reg_value = AK0991X_MAG_ODR50,
  },
  {
    .odr = AK0991X_ODR_100,
    .mag_odr_reg_value = AK0991X_MAG_ODR100,
  },
  {
    .odr = AK0991X_ODR_200,
    .mag_odr_reg_value = AK0991X_MAG_ODR200,
  }
};

static void ak0991x_process_com_port_vector(sns_port_vector *vector,
                                     void *user_arg)
{
  sns_sensor_instance *instance = (sns_sensor_instance *)user_arg;

  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;

  if (AKM_AK0991X_REG_HXL == vector->reg_addr)
  {
    if(state->num_samples != 0){
      sns_time first_timestamp = state->interrupt_timestamp - (state->averaged_interval * (state->num_samples - 1));
      ak0991x_process_mag_data_buffer(instance,
                                      first_timestamp,
                                      state->averaged_interval,
                                      vector->buffer,
                                      vector->bytes);
    }
    else{
      AK0991X_INST_PRINT(LOW, instance, "skip ak0991x_process_mag_data_buffer because num_samples=%d detected.", state->num_samples);
    }
  }
}

static sns_rc ak0991x_heart_beat_timer_event(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;
  sns_rc rv = SNS_RC_SUCCESS;

  if (state->mag_info.use_dri)
  {
    AK0991X_INST_PRINT(HIGH, this, "Detect streaming has stopped");
    // Streaming is unable to resume after 4 attempts
    if (state->heart_beat_attempt_count >= 4)
    {
     AK0991X_INST_PRINT(ERROR, this, "Streming is unable to resume after 3 attempts");
     rv = SNS_RC_INVALID_STATE;
    }
    // Perform a reset operation in an attempt to revive the sensor
    else
    {
      state->heart_beat_attempt_count++;
      ak0991x_flush_fifo(this);

      if(state->heart_beat_attempt_count >= 3)
      {
        rv = ak0991x_device_sw_reset(this,
                                     state->scp_service,
                                     state->com_port_info.port_handle);
        if (rv == SNS_RC_SUCCESS) {
          AK0991X_INST_PRINT(LOW, this, "soft reset called");
        } else {
          AK0991X_INST_PRINT(ERROR, this, "soft reset failed");
        }
        // Indicate streaming error
        rv = SNS_RC_NOT_AVAILABLE;
        ak0991x_reconfig_hw(this);
      }
      ak0991x_register_heart_beat_timer(this);
    }
  }
  else
  {
    uint8_t heart_beat_thresthold =
      ( state->mag_info.use_fifo )? 1 : 4;
    if (state->heart_beat_sample_count < heart_beat_thresthold)
    {
      state->heart_beat_sample_count++;
    }
    else
    {
      AK0991X_INST_PRINT(LOW, this, "heart_beat_gap=%u, heart_beat_timeout=%u",
        (uint32_t)((state->interrupt_timestamp-state->heart_beat_timestamp)/19200),
        (uint32_t)(state->heart_beat_timeout_period/19200));
      // Detect streaming has stopped
      if (state->interrupt_timestamp - state->heart_beat_timestamp > state->heart_beat_timeout_period)
      {
        AK0991X_INST_PRINT(HIGH, this, "Detect streaming has stopped");
        // Streaming is unable to resume after 3 attempts
        if (state->heart_beat_attempt_count >= 3)
        {
         AK0991X_INST_PRINT(ERROR, this, "Streming is unable to resume after 3 attempts");
         rv = SNS_RC_INVALID_STATE;
        }
        // Perform a reset operation in an attempt to revive the sensor
        else
        {
          rv = ak0991x_device_sw_reset(this,
                                       state->scp_service,
                                       state->com_port_info.port_handle);
          if (rv == SNS_RC_SUCCESS) {
            AK0991X_INST_PRINT(LOW, this, "soft reset called");
          } else {
            AK0991X_INST_PRINT(ERROR, this, "soft reset failed");
          }
          // Indicate streaming error
          rv = SNS_RC_NOT_AVAILABLE;
          ak0991x_reconfig_hw(this);
          state->heart_beat_attempt_count++;
        }
      }
      else
      {
        state->heart_beat_timestamp = state->interrupt_timestamp;
        state->heart_beat_sample_count = 0;
        state->heart_beat_attempt_count = 0;
      }
    }
  }

  return rv;
}

/** See sns_sensor_instance_api::notify_event */
static sns_rc ak0991x_inst_notify_event(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;
  sns_sensor_event    *event;
  sns_rc rv = SNS_RC_SUCCESS;

  // Turn COM port ON
  state->scp_service->api->sns_scp_update_bus_power(
    state->com_port_info.port_handle,
    true);

  state->latest_event_time = sns_get_system_time();

  ak0991x_dae_if_process_events(this);

#ifdef AK0991X_ENABLE_DRI
  // Handle interrupts
  sns_interrupt_event irq_event = sns_interrupt_event_init_zero;
  if (NULL != state->interrupt_data_stream)
  {
    event = state->interrupt_data_stream->api->peek_input(state->interrupt_data_stream);

    while (NULL != event)
    {
      if (SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT == event->message_id)
      {
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t *)event->event,
                                                     event->event_len);

        if(pb_decode(&stream, sns_interrupt_event_fields, &irq_event))
        {
//          AK0991X_INST_PRINT(LOW, this, "irq_event %u, now=%u",
//                             (uint32_t)irq_event.timestamp,
//                             (uint32_t)state->latest_event_time);

          state->irq_event_time = irq_event.timestamp;
          state->irq_info.detect_irq_event = true; // detect interrupt

          // Register for timer to enable heart beat function
          ak0991x_register_heart_beat_timer(this);
          state->heart_beat_attempt_count = 0;

          if(state->ascp_xfer_in_progress == 0)
          {
            ak0991x_flush_fifo(this);
            state->irq_info.detect_irq_event = false; // clear interrupt
          }
          else
          {
            AK0991X_INST_PRINT(LOW, this, "ascp_xfer_in_progress=%d.",state->ascp_xfer_in_progress);
            state->re_read_data_after_ascp = true;
          }
        }
      }
      else if (SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REG_EVENT == event->message_id)
      {
        state->irq_info.is_ready = true;
        ak0991x_start_mag_streaming(this);
      }
      else
      {
        AK0991X_INST_PRINT(ERROR, this, "Received invalid event id=%d",
                                      event->message_id);
      }
      event = state->interrupt_data_stream->api->get_next_input(state->interrupt_data_stream);
    }
  }

  // Handle Async Com Port events
  if (NULL != state->async_com_port_data_stream)
  {
    event = state->async_com_port_data_stream->api->peek_input(state->async_com_port_data_stream);

    while (NULL != event)
    {
      if (SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_VECTOR_RW == event->message_id)
      {
        pb_istream_t stream = pb_istream_from_buffer((uint8_t *)event->event, event->event_len);

        sns_ascp_for_each_vector_do(&stream, ak0991x_process_com_port_vector, (void *)this);

        state->ascp_xfer_in_progress--;
        AK0991X_INST_PRINT(LOW, this, "ascp_xfer_in_progress = %d", state->ascp_xfer_in_progress);

        if(state->re_read_data_after_ascp && (state->ascp_xfer_in_progress == 0))
        {
          AK0991X_INST_PRINT(LOW, this, "flush after ASCP read.");
          ak0991x_flush_fifo(this);
          state->re_read_data_after_ascp = false;
        }

        if(state->config_mag_after_ascp_xfer)
        {
          ak0991x_start_mag_streaming(this);
          state->config_mag_after_ascp_xfer = false;
        }
      }
      else if (SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_ERROR == event->message_id)
      {
        AK0991X_INST_PRINT(LOW, this, "Received ASYNC_COM_PORT_ERROR");
      }

      event = state->async_com_port_data_stream->api->get_next_input(
          state->async_com_port_data_stream);
    }
  }
#endif // AK0991X_ENABLE_DRI

  // Handle timer event
  if (NULL != state->timer_data_stream)
  {
    event = state->timer_data_stream->api->peek_input(state->timer_data_stream);

    while (NULL != event)
    {
      if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id)
      {
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t *)event->event,
                                                     event->event_len);
        sns_timer_sensor_event timer_event;

        if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
        {
          AK0991X_INST_PRINT(LOW, this, "Execute handle timer event. now=%u",
                             (uint32_t)state->latest_event_time);
 
          // for regular polling mode
          if (!state->mag_info.use_dri)
          {
            state->force_fifo_read_till_wm = true;
            ak0991x_flush_fifo(this);
          }
          rv = ak0991x_heart_beat_timer_event(this);
        }
        else
        {
          AK0991X_INST_PRINT(ERROR, this, "Failed decoding event");
        }
      }
      else if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_REG_EVENT == event->message_id)
      {
        //TODO:add support for handling SNS_TIMER_SENSOR_REG_EVENT timer event to successfully support S4S
        // When a range of start times is provided to the timer sensor, the timer sensor will pick a specific time.
        // That specific time will be returned in the SNS_TIMER_SENSOR_REG_EVENT event --
        // and will be needed by the mag sensor to populate the fields sent to the DAE sensor(so that timers remain synchronized in the DAE environment),
        // and the field in the Physical Sensor Config event (which needs absolute timing for the future events).
      }
      else
      {
        AK0991X_INST_PRINT(ERROR, this, "Received invalid event id=%d", event->message_id);
      }
      event = state->timer_data_stream->api->get_next_input(state->timer_data_stream);
    }
  }

  // Handle timer data stream for S4S
  ak0991x_s4s_handle_timer_data_stream(this);

  // Turn COM port OFF
  state->scp_service->api->sns_scp_update_bus_power(
    state->com_port_info.port_handle,
    false);
  return rv;
}

/** Public Data Definitions. */

sns_sensor_instance_api ak0991x_sensor_instance_api = {
  .struct_len        = sizeof(sns_sensor_instance_api),
  .init              = &ak0991x_inst_init,
  .deinit            = &ak0991x_inst_deinit,
  .set_client_config = &ak0991x_inst_set_client_config,
  .notify_event      = &ak0991x_inst_notify_event
};
