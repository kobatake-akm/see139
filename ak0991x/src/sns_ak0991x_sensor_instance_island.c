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

#ifdef AK0991X_ENABLE_DIAG_LOGGING
extern log_sensor_state_raw_info log_mag_state_raw_info;
#endif

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

/** See sns_sensor_instance_api::notify_event */
static sns_rc ak0991x_inst_notify_event(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;
  sns_interrupt_event irq_event = sns_interrupt_event_init_zero;
  sns_sensor_event    *event;
#ifdef AK0991X_ENABLE_DIAG_LOGGING
  sns_diag_service    *diag = state->diag_service;
#endif
  AK0991X_INST_PRINT(ERROR, this, "ak0991x_inst_notify_event called.");

  // Turn COM port ON
  state->scp_service->api->sns_scp_update_bus_power(
    state->com_port_info.port_handle,
    true);

#ifdef AK0991X_ENABLE_DAE
  ak0991x_dae_if_process_events(this);
#endif

  // Handle interrupts
  if (NULL != state->interrupt_data_stream)
  {
    event = state->interrupt_data_stream->api->peek_input(state->interrupt_data_stream);
//    AK0991X_INST_PRINT(ERROR, this, "interrupt_data_stream");

    while (NULL != event)
    {
      if (SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REG_EVENT == event->message_id)
      {
        AK0991X_INST_PRINT(ERROR, this, "SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REG_EVENT");
      	state->irq_info.is_ready = true;
				ak0991x_start_mag_streaming(this);
      }
      else if (SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT == event->message_id)
      {
        AK0991X_INST_PRINT(ERROR, this, "SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT");
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t *)event->event,
                                                     event->event_len);

        if (pb_decode(&stream, sns_interrupt_event_fields, &irq_event))
        {
        	state->irq_info.detect_irq_event = true; // detect interrupt

          AK0991X_INST_PRINT(ERROR, this, "irq_event_time %d", (uint32_t)irq_event.timestamp);
          AK0991X_INST_PRINT(ERROR, this, "use_dri=%d, cur_wmk=%d", state->mag_info.use_dri, state->mag_info.cur_wmk);
          AK0991X_INST_PRINT(ERROR, this, "use_fifo=%d, nsf=%d, sdr=%d",state->mag_info.use_fifo, state->mag_info.nsf, state->mag_info.sdr);

          // check DRDY bit to ignore wrong interrupt call by FW
//          uint8_t st1_status = ak0991x_get_status1(this);
//          if( st1_status & AK0991X_DOR_BIT )
//          {
//            AK0991X_INST_PRINT(ERROR, this, "Data Over Run");
//          }
//          if( st1_status & AK0991X_DRDY_BIT )
//          {
          	// adjust timestamp for jitter issues
            ak0991x_validate_timestamp(this, irq_event.timestamp);

  					if ((state->mag_info.use_fifo) && (state->mag_info.cur_wmk < 1))
  					{
  						AK0991X_INST_PRINT(ERROR, this, "handle interrupt event for fifo wmk<1");
  						ak0991x_flush_fifo(this);
  					}
  					else
  					{
  						AK0991X_INST_PRINT(ERROR, this, "handle interrupt event");
  						ak0991x_handle_interrupt_event(this);
  					}
  					AK0991X_INST_PRINT(ERROR, this, "interrupt event done.");
//          }
//          else{
//            AK0991X_INST_PRINT(ERROR, this, "Data is NOT ready. Wrong intterupt. Ignored. %d", (uint32_t)irq_event.timestamp);
//          }
					state->irq_info.detect_irq_event = false; // clear interrupt
        }
        else
        {
          AK0991X_INST_PRINT(ERROR, this, "pb_decode failed.");
        }
      }
      else
      {
        AK0991X_INST_PRINT(ERROR, this, "Received invalid event id=%d",
                                      event->message_id);
      }
      event = state->interrupt_data_stream->api->get_next_input(state->interrupt_data_stream);
      while (NULL != event){
      	event = state->interrupt_data_stream->api->get_next_input(state->interrupt_data_stream);
        AK0991X_INST_PRINT(ERROR, this, "more event on %d", (uint32_t)irq_event.timestamp);
      }
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
        sns_memzero(&log_mag_state_raw_info, sizeof(log_mag_state_raw_info));
        log_mag_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;
        log_mag_state_raw_info.diag = diag;
        log_mag_state_raw_info.instance = this;
        log_mag_state_raw_info.sensor_uid = &state->mag_info.suid;
        ak0991x_log_sensor_state_raw_alloc(&log_mag_state_raw_info, 0);
#endif
        sns_ascp_for_each_vector_do(&stream, ak0991x_process_mag_data_buffer, (void *)this);

#ifdef AK0991X_ENABLE_DIAG_LOGGING
        ak0991x_log_sensor_state_raw_submit(&log_mag_state_raw_info, true);
#endif
        AK0991X_INST_PRINT(LOW, this, "handle ASCP event done.");
      }

      event = state->async_com_port_data_stream->api->get_next_input(
          state->async_com_port_data_stream);
    }
  }

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
        if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG == event->message_id)
        {
          AK0991X_INST_PRINT(LOW, this, "Received config id=%d",
                                        event->message_id);
        }
        else if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id)
        {
          AK0991X_INST_PRINT(ERROR, this, "Execute handle timer event");
          AK0991X_INST_PRINT(ERROR, this, "use_dri=%d, cur_wmk=%d", state->mag_info.use_dri, state->mag_info.cur_wmk);
          AK0991X_INST_PRINT(ERROR, this, "use_fifo=%d, nsf=%d, sdr=%d",state->mag_info.use_fifo, state->mag_info.nsf, state->mag_info.sdr);
          ak0991x_handle_timer_event(this);
        }
        else if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_REG_EVENT == event->message_id)
        {
            AK0991X_INST_PRINT(ERROR, this, "Execute handle tiemr reg event");
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

#ifdef AK0991X_ENABLE_S4S
  // Handle timer event for s4s
  if (NULL != state->s4s_timer_data_stream)
  {
    event = state->s4s_timer_data_stream->api->peek_input(state->s4s_timer_data_stream);

    while (NULL != event)
    {
      pb_istream_t stream = pb_istream_from_buffer((pb_byte_t *)event->event,
                                                   event->event_len);
      sns_timer_sensor_event timer_event;

      if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
      {
        if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG == event->message_id)
        {
          AK0991X_INST_PRINT(LOW, this, "Received config id=%d",
                                        event->message_id);
        }
        else if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id)
        {
          ak0991x_handle_s4s_timer_event(this);
          AK0991X_INST_PRINT(ERROR, this, "Execute handle s4s timer event");
        }
        else if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_REG_EVENT == event->message_id)
        {
          AK0991X_INST_PRINT(ERROR, this, "Execute handle tiemr s4s reg event");
        }
        else
        {
          AK0991X_INST_PRINT(ERROR, this, "handle timer ERROR s4s");
        }
      }
      else
      {
        AK0991X_INST_PRINT(ERROR, this, "Received invalid event id=%d",
                                      event->message_id);
      }

      event = state->s4s_timer_data_stream->api->get_next_input(state->s4s_timer_data_stream);
    }
  }
#endif // AK0991X_ENABLE_S4S

  // Turn COM port OFF
  state->scp_service->api->sns_scp_update_bus_power(
    state->com_port_info.port_handle,
    false);
  return SNS_RC_SUCCESS;
}

/** Public Data Definitions. */

sns_sensor_instance_api ak0991x_sensor_instance_api = {
  .struct_len        = sizeof(sns_sensor_instance_api),
  .init              = &ak0991x_inst_init,
  .deinit            = &ak0991x_inst_deinit,
  .set_client_config = &ak0991x_inst_set_client_config,
  .notify_event      = &ak0991x_inst_notify_event
};
