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

/**
 * EDIT HISTORY FOR FILE
 *
 * This section contains comments describing changes made to the module.
 * Notice that changes are listed in reverse chronological order.
 *
 *
 * when         who     what, where, why
 * --------     ---     ------------------------------------------------
 * 04/04/17     AKM     Optimize code of MAG_SUID configuration.
 * 04/04/17     AKM     Fix bus_type of Async Com Port configuration.
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
#include "sns_printf.h"

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

static sns_rc ak0991x_mag_match_odr(float desired_sample_rate,
                                    float *chosen_sample_rate,
                                    ak0991x_mag_odr *chosen_reg_value,
                                    akm_device_type device_select)
{
  if (NULL == chosen_sample_rate
      ||
      NULL == chosen_reg_value)
  {
    return SNS_RC_NOT_SUPPORTED;
  }

  if (desired_sample_rate <= AK0991X_ODR_0)
  {
    *chosen_sample_rate = AK0991X_ODR_0;
    *chosen_reg_value = AK0991X_MAG_ODR_OFF;
  }
  else if (desired_sample_rate <= AK0991X_ODR_10)
  {
    if ((desired_sample_rate <= AK0991X_ODR_1) &&
        ((device_select == AK09915C) || (device_select == AK09915D) || (device_select == AK09917)))
    {
      *chosen_sample_rate = AK0991X_ODR_1;
      *chosen_reg_value = AK0991X_MAG_ODR1;
    }
    else
    {
      *chosen_sample_rate = AK0991X_ODR_10;
      *chosen_reg_value = AK0991X_MAG_ODR10;
    }
  }
  else if (desired_sample_rate <= AK0991X_ODR_20)
  {
    *chosen_sample_rate = AK0991X_ODR_20;
    *chosen_reg_value = AK0991X_MAG_ODR20;
  }
  else if (desired_sample_rate <= AK0991X_ODR_50)
  {
    *chosen_sample_rate = AK0991X_ODR_50;
    *chosen_reg_value = AK0991X_MAG_ODR50;
  }
  else if (desired_sample_rate <= AK0991X_ODR_100)
  {
    *chosen_sample_rate = AK0991X_ODR_100;
    *chosen_reg_value = AK0991X_MAG_ODR100;
  }
  else if ((desired_sample_rate <= AK0991X_ODR_200) &&
           ((device_select == AK09915C) || (device_select == AK09915D) || (device_select == AK09917)))
  {
    *chosen_sample_rate = AK0991X_ODR_200;
    *chosen_reg_value = AK0991X_MAG_ODR200;
  }
  else
  {
    return SNS_RC_FAILED;
  }

  return SNS_RC_SUCCESS;
}

/** See sns_sensor_instance_api::notify_event */
static sns_rc ak0991x_inst_notify_event(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;
  sns_interrupt_event irq_event = sns_interrupt_event_init_zero;
  sns_sensor_event    *event;
  sns_diag_service    *diag = state->diag_service;

  // Turn COM port ON
  state->scp_service->api->sns_scp_update_bus_power(
    state->com_port_info.port_handle,
    true);

  ak0991x_dae_if_process_events(this);

  // Handle interrupts
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

        if (pb_decode(&stream, sns_interrupt_event_fields, &irq_event))
        {
          state->interrupt_timestamp = irq_event.timestamp;
          // Add setting for timestamp in case of flush event caused by irq trigger
          state->irq_info.detect_irq_event = true;

          if ((state->mag_info.use_fifo) && (state->mag_info.cur_wmk < 2))
          {
            SNS_INST_PRINTF(LOW, this, "handle interrupt event for fifo wmk<2");
            ak0991x_flush_fifo(this);
          }
          else
          {
            SNS_INST_PRINTF(LOW, this, "handle interrupt event");
            ak0991x_handle_interrupt_event(this);
          }

          state->irq_info.detect_irq_event = false;
        }
      }
      else
      {
        SNS_INST_PRINTF(ERROR, this, "Received invalid event id=%d",
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
        SNS_INST_PRINTF(LOW, this, "Received ASCP error event id=%d",
                                      event->message_id);
      }
      else if (SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_VECTOR_RW == event->message_id)
      {
        pb_istream_t stream = pb_istream_from_buffer((uint8_t *)event->event, event->event_len);
        sns_memzero(&log_mag_state_raw_info, sizeof(log_mag_state_raw_info));
        log_mag_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;
        log_mag_state_raw_info.diag = diag;
        log_mag_state_raw_info.instance = this;
        log_mag_state_raw_info.sensor_uid = &state->mag_info.suid;
        ak0991x_log_sensor_state_raw_alloc(&log_mag_state_raw_info, 0);

        SNS_INST_PRINTF(LOW, this, "handle ASCP event");
 
        sns_ascp_for_each_vector_do(&stream, ak0991x_process_mag_data_buffer, (void *)this);

        ak0991x_log_sensor_state_raw_submit(&log_mag_state_raw_info, true);
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
          SNS_INST_PRINTF(LOW, this, "Received config id=%d",
                                        event->message_id);
        }
        else if (SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id)
        {
         //SNS_INST_PRINTF(ERROR, this, "Execute handle timer event");
          ak0991x_handle_timer_event(this);
        }
      }
      else
      {
        SNS_INST_PRINTF(ERROR, this, "Received invalid event id=%d",
                                      event->message_id);
      }

      event = state->timer_data_stream->api->get_next_input(state->timer_data_stream);
    }
  }

  // Turn COM port OFF
  state->scp_service->api->sns_scp_update_bus_power(
    state->com_port_info.port_handle,
    false);
  return SNS_RC_SUCCESS;
}

/** See sns_sensor_instance_api::set_client_config */
static sns_rc ak0991x_inst_set_client_config(sns_sensor_instance *const this,
                                             sns_request const *client_request)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;
  state->client_req_id = client_request->message_id;
  float           desired_sample_rate = 0;
  float           desired_report_rate = 0;
  float           mag_chosen_sample_rate = 0;
  ak0991x_mag_odr mag_chosen_sample_rate_reg_value;
  uint16_t        desired_wmk = 0;
  sns_rc          rv = SNS_RC_SUCCESS;
  float *fac_cal_bias = NULL;
  matrix3 *fac_cal_corr_mat = NULL;

  SNS_INST_PRINTF(MED, this, "inst_set_client_config msg_id %d", client_request->message_id);
  // Turn COM port ON
  state->scp_service->api->sns_scp_update_bus_power(
    state->com_port_info.port_handle,
    true);
  // Register for interrupt
  if(state->mag_info.use_dri && !ak0991x_dae_if_available(this))
  {
    ak0991x_register_interrupt(this);
  }

  SNS_INST_PRINTF(LOW, this, "dae_if_available %d",(int)ak0991x_dae_if_available(this));
 
  if (client_request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
  {
    // In case of DRI,
    //   1. Extract sample, report rates from client_request.
    //   2. sendRequest() for Intrerupt register/de-register in case of DRI using interrupt_data_stream.
    //   3. Configure sensor HW.
    //   4. Save the current config information like type, sample_rate, report_rate, etc.
    // In case of polling,
    //   1. Extract sample, report rates from client_request.
    //   2. Configure sensor HW.
    //   3. Save the current config information like type, sample_rate, report_rate, etc.
    //   4. sendRequest() for Timer to start/stop in case of polling using timer_data_stream.
    sns_ak0991x_mag_req *payload =
      (sns_ak0991x_mag_req *)client_request->request;
    desired_sample_rate = payload->sample_rate;
    desired_report_rate = payload->report_rate;

    if (desired_report_rate > desired_sample_rate)
    {
      // bad request. Return error or default report_rate to sample_rate
      desired_report_rate = desired_sample_rate;
    }

 
    SNS_INST_PRINTF(LOW, this, "desired_sample_rate=%d desired_report_rate=%d",
        (int)desired_sample_rate, (int)desired_report_rate);
 
    state->mag_info.flush_period = payload->flush_period;
    rv = ak0991x_mag_match_odr(desired_sample_rate,
                               &mag_chosen_sample_rate,
                               &mag_chosen_sample_rate_reg_value,
                               state->mag_info.device_select);
    state->mag_info.req_wmk = (uint16_t)(mag_chosen_sample_rate / desired_report_rate);
    if ((state->mag_info.use_fifo) && (state->mag_info.use_dri))
    {
      if (desired_report_rate != 0)
      {
        /* Water mark level : 0x00 -> 1step, 0x01F ->32step*/
        desired_wmk = state->mag_info.req_wmk - 1;
      }

      switch (state->mag_info.device_select)
      {
      case AK09915C:
      case AK09915D:

        if (AK09915_FIFO_SIZE <= desired_wmk)
        {
          desired_wmk = AK09915_FIFO_SIZE - 1;
        }

        break;

      case AK09917:

        if (AK09917_FIFO_SIZE <= desired_wmk)
        {
          desired_wmk = AK09917_FIFO_SIZE - 1;
        }

        break;

      default:
        desired_wmk = 0;
        break;
      }
    }

    SNS_INST_PRINTF(LOW, this, "desired_wmk=%d",desired_wmk);
 
    state->mag_info.cur_wmk = desired_wmk;

    if (rv != SNS_RC_SUCCESS)
    {
      // TODO Unsupported rate. Report error using sns_std_error_event.
      // return rv;
    }

    state->mag_req.sample_rate = mag_chosen_sample_rate;
    state->mag_info.desired_odr = mag_chosen_sample_rate_reg_value;

    SNS_INST_PRINTF(LOW, this, "sample_rate=%d, reg_value=%d, config_step=%d",
                    (int)mag_chosen_sample_rate,(int)mag_chosen_sample_rate_reg_value,
                    (int)state->config_step);

    if (AK0991X_CONFIG_IDLE == state->config_step &&
        ak0991x_dae_if_stop_streaming(this))
    {
      SNS_INST_PRINTF(LOW, this, "done dae_if_stop_streaming");
      state->config_step = AK0991X_CONFIG_STOPPING_STREAM;
    }

    if (state->config_step == AK0991X_CONFIG_IDLE)
    {
      // care the FIFO buffer if enabled FIFO
      if ((!state->this_is_first_data) && (state->mag_info.use_fifo))
      {
        ak0991x_flush_fifo(this);
        if (state->mag_info.desired_odr == AK0991X_MAG_ODR_OFF)
        {
          state->this_is_first_data = true;
        }
      }

      // hardware setting for measurement mode
      if (state->mag_info.use_dri && !ak0991x_dae_if_start_streaming(this))
      {
        ak0991x_reconfig_hw(this);
        SNS_INST_PRINTF(LOW, this, "done ak0991x_reconfig_hw");
      }
      // Register for timer
      if (!state->mag_info.use_dri && !ak0991x_dae_if_available(this))
      {
        ak0991x_reconfig_hw(this);
        ak0991x_register_timer(this);
        SNS_INST_PRINTF(ERROR, this, "done register_timer");
      }

      //ak0991x_dae_if_start_streaming(this);
      ak0991x_send_config_event(this);

    }

    // update registry configuration
    fac_cal_bias = state->mag_registry_cfg.fac_cal_bias;
    fac_cal_corr_mat = &state->mag_registry_cfg.fac_cal_corr_mat;

    if(NULL!= fac_cal_bias && NULL != fac_cal_corr_mat)
    {
      sns_memscpy(fac_cal_bias, sizeof(payload->registry_cfg.fac_cal_bias),
                  payload->registry_cfg.fac_cal_bias, 
                  sizeof(payload->registry_cfg.fac_cal_bias));

      sns_memscpy(fac_cal_corr_mat, sizeof(payload->registry_cfg.fac_cal_corr_mat),
                  &payload->registry_cfg.fac_cal_corr_mat, 
                  sizeof(payload->registry_cfg.fac_cal_corr_mat));

    }

   }
  else if(client_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ)
  {
    state->fifo_flush_in_progress = true;
    if(!ak0991x_dae_if_flush_samples(this))
    {
      ak0991x_flush_fifo(this);
      state->this_is_first_data = true;
      ak0991x_send_fifo_flush_done(this);
    }
  }
  else if (state->client_req_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
  {
    // 1. Extract test type from client_request.
    // 2. Configure sensor HW for test type.
    // 3. send_request() for Timer Sensor in case test needs polling/waits.
    // 4. Factory test is TBD.
  }

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
