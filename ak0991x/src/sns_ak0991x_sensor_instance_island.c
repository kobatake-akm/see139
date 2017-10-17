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

#ifdef AK0991X_ENABLE_DRI
static void ak0991x_process_com_port_vector(sns_port_vector *vector,
                                     void *user_arg)
{
  sns_sensor_instance *instance = (sns_sensor_instance *)user_arg;

  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;

  if (AKM_AK0991X_REG_HXL == vector->reg_addr)
  {
    sns_time first_timestamp = state->interrupt_timestamp 
                               - (state->averaged_interval * (state->num_samples - 1));

    ak0991x_process_mag_data_buffer(instance,
                                    first_timestamp,
                                    state->averaged_interval,
                                    vector->buffer,
                                    vector->bytes);


      state->this_is_first_data = false;
      state->pre_timestamp = state->interrupt_timestamp;
  }
}
#endif

static sns_rc ak0991x_heart_beat_timer_event(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;
  sns_rc rv = SNS_RC_SUCCESS;
  uint8_t heart_beat_thresthold =
    (state->mag_info.use_fifo || state->mag_info.use_dri)? 1 : 4;

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
        state->called_handle_timer_reg_event = true;
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

  ak0991x_dae_if_process_events(this);

#ifdef AK0991X_ENABLE_DRI
  // Handle interrupts
  sns_interrupt_event irq_event = sns_interrupt_event_init_zero;
  if (NULL != state->interrupt_data_stream)
  {
    event = state->interrupt_data_stream->api->peek_input(state->interrupt_data_stream);

    while (NULL != event)
    {
      if (SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REG_EVENT == event->message_id)
      {
        state->irq_info.is_ready = true;
        ak0991x_start_mag_streaming(this);
      }
      else if (SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT == event->message_id)
      {
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t *)event->event,
                                                     event->event_len);

        if(pb_decode(&stream, sns_interrupt_event_fields, &irq_event))
        {

          // Check if the ak0991x_inst_notify_event is an actual DRDY event or not.
          if( ak0991x_is_drdy(this) && (state->ascp_xfer_in_progress == 0))
          {
            state->irq_event_time = irq_event.timestamp;
            state->irq_info.detect_irq_event = true; // detect interrupt
            if ((state->mag_info.device_select != AK09917) && (state->mag_info.use_fifo) && (state->mag_info.cur_wmk < 1))
            {
              ak0991x_flush_fifo(this);
            }
            else if(state->mag_info.use_fifo && state->data_over_run)
            {
              ak0991x_flush_fifo(this);
            }
            else
            {
              ak0991x_handle_interrupt_event(this);
            }
            AK0991X_INST_PRINT(LOW, this, "irq_event %u", (uint32_t)irq_event.timestamp);
            state->irq_info.detect_irq_event = false; // clear interrupt
          }
          else if (state->ascp_xfer_in_progress != 0)
          {
            state->re_read_data_after_ascp = true;
          }
          else
          {
            AK0991X_INST_PRINT(ERROR, this, "DRDY is not ready. Wrong interrupt.");
          }
          // Already got an interrupt event. Ignore additional events.
          event = state->interrupt_data_stream->api->get_next_input(state->interrupt_data_stream);

        }
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
      if (SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_ERROR == event->message_id)
      {
        AK0991X_INST_PRINT(LOW, this, "Received ASCP error event id=%d",
                                      event->message_id);
      }
      else if (SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_VECTOR_RW == event->message_id)
      {
        pb_istream_t stream = pb_istream_from_buffer((uint8_t *)event->event, event->event_len);

#ifdef AK0991X_ENABLE_DIAG_LOGGING
        sns_diag_service    *diag = state->diag_service;
        sns_memzero(&log_mag_state_raw_info, sizeof(log_mag_state_raw_info));
        log_mag_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;
        log_mag_state_raw_info.diag = diag;
        log_mag_state_raw_info.instance = this;
        log_mag_state_raw_info.sensor_uid = &state->mag_info.suid;
        ak0991x_log_sensor_state_raw_alloc(&log_mag_state_raw_info, 0);
#endif // AK0991X_ENABLE_DIAG_LOGGING

        sns_ascp_for_each_vector_do(&stream, ak0991x_process_com_port_vector, (void *)this);

#ifdef AK0991X_ENABLE_DIAG_LOGGING
        ak0991x_log_sensor_state_raw_submit(&log_mag_state_raw_info, true);
#endif // AK0991X_ENABLE_DIAG_LOGGING

        state->ascp_xfer_in_progress--;

        if(state->re_read_data_after_ascp && (state->ascp_xfer_in_progress == 0))
        {
          state->interrupt_timestamp = sns_get_system_time(); // For flush
          ak0991x_flush_fifo(this);
          state->re_read_data_after_ascp = false;
        }

        if(state->config_mag_after_ascp_xfer)
        {
          ak0991x_start_mag_streaming(this);
          state->config_mag_after_ascp_xfer = false;
        }
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
      pb_istream_t stream = pb_istream_from_buffer((pb_byte_t *)event->event,
                                                   event->event_len);
      sns_timer_sensor_event timer_event;

      if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
      {
        //TODO: Timer sensor never sends SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG event
        // It is a request message ID. Mag senosr sends to timer sensor
        if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG == event->message_id)
        {
          AK0991X_INST_PRINT(LOW, this, "Received config id=%d",
                                        event->message_id);
        }
        else if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id)
        {
//          AK0991X_INST_PRINT(LOW, this, "Execute handle timer event. pre-now=%u",(uint32_t)(sns_get_system_time() - state->pre_timestamp));
          if(state->called_handle_timer_reg_event){
            if (!state->mag_info.use_dri)
            {
              state->force_fifo_read_till_wm = true;
              state->interrupt_timestamp = sns_get_system_time(); // For Polling
              ak0991x_flush_fifo(this);
            }

            rv = ak0991x_heart_beat_timer_event(this);
          }else{
            AK0991X_INST_PRINT(LOW, this, "Wrong timer event...");
          }
        }
        else if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_REG_EVENT == event->message_id)
        {
            //TODO:add support for handling SNS_TIMER_SENSOR_REG_EVENT timer event to successfully support S4S
            // When a range of start times is provided to the timer sensor, the timer sensor will pick a specific time.
            // That specific time will be returned in the SNS_TIMER_SENSOR_REG_EVENT event -- 
            // and will be needed by the mag sensor to populate the fields sent to the DAE sensor(so that timers remain synchronized in the DAE environment),
            // and the field in the Physical Sensor Config event (which needs absolute timing for the future events).
            AK0991X_INST_PRINT(LOW, this, "Execute handle tiemr reg event");
            state->called_handle_timer_reg_event = true;
        }
      }
      else
      {
        AK0991X_INST_PRINT(ERROR, this, "Received invalid event id=%d",
                                      event->message_id);
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
