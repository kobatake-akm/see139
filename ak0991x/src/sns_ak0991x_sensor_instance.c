/**
 * @file sns_ak0991x_sensor_instance.c
 *
 * AK0991X Mag virtual Sensor Instance implementation.
 *
 * Copyright (c) 2016 Qualcomm Technologies, Inc.
 * Copyright (c) 2016 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
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

const odr_reg_map reg_map_ak0991x[AK0991X_REG_MAP_TABLE_SIZE] =
{
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
  },
};

static sns_rc ak0991x_mag_match_odr(float desired_sample_rate,
                                      float *chosen_sample_rate,
                                      ak0991x_mag_odr *chosen_reg_value,
                                      akm_device_type device_select)
{
  uint8_t idx;
  uint16_t max_odr_setting;

  switch(device_select) {
    case AK09911:
    case AK09912:
    case AK09913:
    case AK09916C:
    case AK09916D:
    case AK09918:
      max_odr_setting = AK0991X_ODR_100;
      break;
    case AK09915C:
    case AK09915D:
      max_odr_setting = AK0991X_ODR_200;
      break;
    default:
      max_odr_setting = AK0991X_ODR_0;
      break;
  }

  if((max_odr_setting < desired_sample_rate)
     ||
     NULL == chosen_sample_rate
     ||
     NULL == chosen_reg_value)
  {
    return SNS_RC_NOT_SUPPORTED;
  }

  for(idx = 0; idx < ARR_SIZE(reg_map_ak0991x); idx++)
  {
    // Only AK09915C/D support ODR 1Hz
    if((idx == 1) && ((device_select != AK09915C) && (device_select != AK09915D)))
    {
      continue;
    }

    if(desired_sample_rate <= reg_map_ak0991x[idx].odr)
    {
      break;
    }
  }

  if (idx >= ARR_SIZE(reg_map_ak0991x))
  {
    return SNS_RC_NOT_SUPPORTED;
  }

  *chosen_sample_rate = reg_map_ak0991x[idx].odr;
  *chosen_reg_value = reg_map_ak0991x[idx].mag_odr_reg_value;

  return SNS_RC_SUCCESS;
}

/** See sns_sensor_instance_api::notify_event */
static sns_rc ak0991x_inst_notify_event(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state =
         (ak0991x_instance_state*)this->state->state;
  sns_interrupt_event irq_event = sns_interrupt_event_init_zero;
  sns_sensor_event *event;

  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  state->diag_service =  (sns_diag_service*)
      service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);

  sns_diag_service* diag = state->diag_service;
  diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);

  // Turn COM port ON
  state->com_port_info.port_handle->com_port_api->sns_scp_update_bus_power(state->com_port_info.port_handle,
                                                                           true);
  diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
 
  // Handle interrupts
  if(NULL != state->interrupt_data_stream)
  {
    event = state->interrupt_data_stream->api->peek_input(state->interrupt_data_stream);
    while(NULL != event)
    {
      diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
      if(event->message_id != SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT) // TODO add config update message ID support
      {
      diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
 
      diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid,
                                    SNS_ERROR, __FILENAME__, __LINE__,
                                    "Received invalid event id=%d",
                                    event->message_id);
      }
      else
      {
        diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
 
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
                                                   event->event_len);
        if(pb_decode(&stream, sns_interrupt_event_fields, &irq_event))
        {
          diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
          state->interrupt_timestamp = irq_event.timestamp;
          // Add setting for timestamp in case of flush event caused by irq trigger
          state->irq_info.detect_irq_event = true;
          if((state->mag_info.use_fifo) && (state->mag_info.cur_wmk < 2))
          {
            ak0991x_flush_fifo(this);
          }
          else
          {
            ak0991x_handle_interrupt_event(this);
          }
          state->irq_info.detect_irq_event = false;
          diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
        }
      }
    diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
    event = state->interrupt_data_stream->api->get_next_input(state->interrupt_data_stream);
    }
  }

  diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
 
  // Handle Async Com Port events
  if(NULL != state->async_com_port_data_stream)
  {
    event = state->async_com_port_data_stream->api->peek_input(state->async_com_port_data_stream);
    while(NULL != event)
    {
      diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
      if(SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_ERROR == event->message_id)
      {
      //TODO: Warning;
      diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid,
                                    SNS_ERROR, __FILENAME__, __LINE__,
                                    "Received ASCP error event id=%d",
                                    event->message_id);
      }
      else if(SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_VECTOR_RW == event->message_id)
      {
        pb_istream_t stream = pb_istream_from_buffer((uint8_t *)event->event, event->event_len);
        diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
        sns_ascp_for_each_vector_do(&stream, ak0991x_process_mag_data_buffer, (void *)this);
        diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
      }
      event = state->async_com_port_data_stream->api->get_next_input(state->async_com_port_data_stream);
      diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
    }
  }

  diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);

  // Handle timer event
  if(NULL != state->timer_data_stream)
  {
    event = state->timer_data_stream->api->peek_input(state->timer_data_stream);
    while(NULL != event)
    {
      pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
                                                   event->event_len);
      sns_timer_sensor_event timer_event;
      diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);

      if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
      {
        if(SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG == event->message_id)
        {
          diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid,
                                    SNS_ERROR, __FILENAME__, __LINE__,
                                    "Received config id=%d",
                                    event->message_id);
        }
        else if(SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT == event->message_id)
        {
          diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);

           ak0991x_handle_timer_event(this);
           diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
        }
      }
      else
      {
        diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid,
                                    SNS_ERROR, __FILENAME__, __LINE__,
                                    "Received invalid event id=%d",
                                    event->message_id);
      }
      event = state->timer_data_stream->api->get_next_input(state->timer_data_stream);
    }
  }

  // Turn COM port OFF
  state->com_port_info.port_handle->com_port_api->sns_scp_update_bus_power(state->com_port_info.port_handle,
                                                                           false);

  diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
  return SNS_RC_SUCCESS;
}

/** See sns_sensor_instance_api::init */
static sns_rc ak0991x_inst_init(sns_sensor_instance *const this,
    sns_sensor_state const *sstate)
{
  ak0991x_instance_state *state =
              (ak0991x_instance_state*)this->state->state;
  ak0991x_state *sensor_state =
              (ak0991x_state*)sstate->state;
  float data[3];
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_mgr = (sns_stream_service*)
              service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

  state->diag_service =  (sns_diag_service*)
      service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);

  sns_diag_service* diag = state->diag_service;
  diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_MED, __FILENAME__, __LINE__,__FUNCTION__);


    stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                 this,
                                                 sensor_state->irq_suid,
                                                 &state->interrupt_data_stream);

    stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                 this,
                                                 sensor_state->acp_suid,
                                                 &state->async_com_port_data_stream);

  /**-------------------------Init Mag State-------------------------*/
  state->mag_info.desired_odr = AK0991X_MAG_ODR_OFF;
  state->mag_info.curr_odr = AK0991X_MAG_ODR_OFF;
  sns_memscpy(&state->mag_info.sstvt_adj,
              sizeof(state->mag_info.sstvt_adj),
              &sensor_state->sstvt_adj,
              sizeof(sensor_state->sstvt_adj));
  sns_memscpy(&state->mag_info.device_select,
              sizeof(state->mag_info.device_select),
              &sensor_state->device_select,
              sizeof(sensor_state->device_select));
  state->mag_info.cur_wmk = 0;
  switch(state->mag_info.device_select) {
    case AK09911:
      state->mag_info.resolution = AK09911_RESOLUTION;
      state->mag_info.use_fifo = false;
      state->mag_info.max_fifo_size = AK09911_FIFO_SIZE;
      state->mag_info.use_dri = false;
      break;
    case AK09912:
      state->mag_info.resolution = AK09912_RESOLUTION;
      state->mag_info.use_fifo = false;
      state->mag_info.max_fifo_size = AK09912_FIFO_SIZE;
      state->mag_info.use_dri = AK0991X_USE_DRI;
      break;
    case AK09913:
      state->mag_info.resolution = AK09913_RESOLUTION;
      state->mag_info.use_fifo = false;
      state->mag_info.max_fifo_size = AK09913_FIFO_SIZE;
      state->mag_info.use_dri = false;
      break;
    case AK09915C:
    case AK09915D:
      state->mag_info.resolution = AK09915_RESOLUTION;
      state->mag_info.use_fifo = AK0991X_ENABLE_FIFO;
      state->mag_info.max_fifo_size = AK09915_FIFO_SIZE;
      state->mag_info.use_dri = AK0991X_USE_DRI;
      break;
    case AK09916C:
    case AK09916D:
      state->mag_info.resolution = AK09916_RESOLUTION;
      state->mag_info.use_fifo = false;
      state->mag_info.max_fifo_size = AK09916_FIFO_SIZE;
      state->mag_info.use_dri = false;
      break;
    case AK09918:
      state->mag_info.resolution = AK09918_RESOLUTION;
      state->mag_info.use_fifo = false;
      state->mag_info.max_fifo_size = AK09918_FIFO_SIZE;
      state->mag_info.use_dri = false;
      break;
    default:
      state->mag_info.resolution = 0;
      state->mag_info.use_fifo = false;
      state->mag_info.max_fifo_size = 0;
      state->mag_info.use_dri = false;
      break;
  }

  state->pre_timestamp = 0;
  state->this_is_first_data = true;

  state->encoded_mag_event_len = pb_get_encoded_size_sensor_stream_event(data, 3);


  /** Initialize Timer info to be used by the Instance */
  sns_memscpy(&state->timer_suid,
              sizeof(state->timer_suid),
              &sensor_state->timer_suid,
              sizeof(sensor_state->timer_suid));
  state->timer_stream_is_created = false;

  /** Initialize COM port to be used by the Instance */
  sns_memscpy(&state->com_port_info.com_config,
              sizeof(state->com_port_info.com_config),
              &sensor_state->com_port_info.com_config,
              sizeof(sensor_state->com_port_info.com_config));


  sns_scp_register_com_port(&state->com_port_info.com_config,
                            &state->com_port_info.port_handle);

  state->com_port_info.port_handle->com_port_api->sns_scp_open(state->com_port_info.port_handle);

  state->com_port_info.port_handle->com_port_api->sns_scp_update_bus_power(state->com_port_info.port_handle,
                                                                           false);

  /** Initialize IRQ info to be used by the Instance */
  sns_memscpy(&state->irq_info,
              sizeof(state->irq_info),
              &sensor_state->irq_info,
              sizeof(sensor_state->irq_info));

  state->irq_info.is_registered = false;
  state->irq_info.detect_irq_event = false;

  /** Configure the Async Com Port */
  uint8_t pb_encode_buffer[100];
  uint32_t enc_len;
  sns_async_com_port_config async_com_port_config;
  async_com_port_config.bus_instance = sensor_state->com_port_info.com_config.bus_instance;
  async_com_port_config.bus_type = SNS_ASYNC_COM_PORT_BUS_TYPE_SPI;
  async_com_port_config.max_bus_speed_kHz = sensor_state->com_port_info.com_config.max_bus_speed_KHz;
  async_com_port_config.min_bus_speed_kHz = sensor_state->com_port_info.com_config.min_bus_speed_KHz;
  async_com_port_config.reg_addr_type = SNS_ASYNC_COM_PORT_REG_ADDR_TYPE_8_BIT;
  async_com_port_config.slave_control = sensor_state->com_port_info.com_config.slave_control;
  enc_len = pb_encode_request(pb_encode_buffer, 100, &async_com_port_config,
    sns_async_com_port_config_fields, NULL);

  sns_request async_com_port_request =
  (sns_request)
  {
    .message_id = SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_CONFIG,
    .request_len = enc_len,
    .request = &pb_encode_buffer
  };
  state->async_com_port_data_stream->api->send_request(
    state->async_com_port_data_stream, &async_com_port_request);


  return SNS_RC_SUCCESS;
}

/** See sns_sensor_instance_api::set_client_config */
static sns_rc ak0991x_inst_set_client_config(sns_sensor_instance *const this,
                                             sns_request const *client_request)
{
  ak0991x_instance_state *state =
                  (ak0991x_instance_state*)this->state->state;
  state->client_req_id = client_request->message_id;
  float desired_sample_rate = 0;
  float desired_report_rate = 0;
  float mag_chosen_sample_rate = 0;
  ak0991x_mag_odr mag_chosen_sample_rate_reg_value;
  uint16_t desired_wmk = 0;
  sns_rc rv = SNS_RC_SUCCESS;

  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_mgr = (sns_stream_service*)
              service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
  state->diag_service =  (sns_diag_service*)
      service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);

  sns_diag_service* diag = state->diag_service;
  diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);

  // Turn COM port ON
  state->com_port_info.port_handle->com_port_api->sns_scp_update_bus_power(state->com_port_info.port_handle,
                                                                           true);
  diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
 

  // Reset the device if not streaming.
  if(state->mag_info.curr_odr == AK0991X_MAG_ODR_OFF)
  {
    diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
    ak0991x_device_sw_reset(state->com_port_info.port_handle);
  }

  if(client_request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
  {
     // 1. Extract sample, report rates from client_request.
     // 2. Configure sensor HW.
     // 3. sendRequest() for Timer to start/stop in case of polling using timer_data_stream.
     // 4. sendRequest() for Intrerupt register/de-register in case of DRI using interrupt_data_stream.
     // 5. Save the current config information like type, sample_rate, report_rate, etc.
     diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
     sns_ak0991x_mag_req *payload =
        (sns_ak0991x_mag_req*)client_request->request;
     desired_sample_rate = payload->sample_rate;
     desired_report_rate = payload->report_rate;

     if(desired_report_rate > desired_sample_rate)
     {
       diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
       // bad request. Return error or default report_rate to sample_rate
       desired_report_rate = desired_sample_rate;
     }

     diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,"sample_rate=%f report_rate=%f wmk=%d",desired_sample_rate,desired_report_rate,desired_wmk);
     rv = ak0991x_mag_match_odr(desired_sample_rate,
                                 &mag_chosen_sample_rate,
                                 &mag_chosen_sample_rate_reg_value,
                                 state->mag_info.device_select);
 
     if((state->mag_info.use_fifo) && (state->mag_info.use_dri))
     {
       if(desired_report_rate != 0)
       {
         /* Water mark level : 0x00 -> 1step, 0x01F ->32step*/
         desired_wmk = (int16_t)(mag_chosen_sample_rate / desired_report_rate) - 1;
         diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,"desired_wmk=%d",desired_wmk);
       }

       switch(state->mag_info.device_select)
       {
         case AK09915C:
         case AK09915D:
           if (AK09915_FIFO_SIZE <= desired_wmk) {
             desired_wmk = AK09915_FIFO_SIZE - 1;
           }
           break;
         default:
           desired_wmk = 0;
           break;
       }
     }
     state->mag_info.cur_wmk = desired_wmk;

    if(rv != SNS_RC_SUCCESS)
     {
       // TODO Unsupported rate. Report error using sns_std_error_event.
       // return rv;
     }
     diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,"%s rate=%f rate_reg=%d desired_wmk=%d",__FUNCTION__,mag_chosen_sample_rate, mag_chosen_sample_rate_reg_value,desired_wmk);
 
     state->mag_req.sample_rate = mag_chosen_sample_rate;
     state->mag_info.desired_odr = mag_chosen_sample_rate_reg_value;

     ak0991x_send_config_event(this);
     diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);

    if(state->mag_info.desired_odr != AK0991X_MAG_ODR_OFF) {
      if((!state->this_is_first_data) && (state->mag_info.use_fifo)) {
        ak0991x_flush_fifo(this);
      }
      ak0991x_start_mag_streaming(state);
    } else {
      if((!state->this_is_first_data) && (state->mag_info.use_fifo)) {
        ak0991x_flush_fifo(this);
        state->this_is_first_data = true;
      }
      ak0991x_stop_mag_streaming(state);
    }


     diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);

    if(state->mag_info.use_dri)
    {
      if(!state->irq_info.is_registered)
      {
        diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
        sns_request irq_req;
        sns_interrupt_req irq_req_payload;
        size_t irq_req_len;
        uint8_t buffer[20];
        sns_memset(buffer, 0, sizeof(buffer));
        diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
 
        irq_req_payload.interrupt_num = state->irq_info.irq_num;
        irq_req_payload.interrupt_trigger_type = state->irq_info.irq_trigger_type;
        irq_req_payload.interrupt_drive_strength = state->irq_info.irq_drive_strength;
        irq_req_payload.interrupt_pull_type = state->irq_info.irq_pull;
        irq_req_payload.is_chip_pin = state->irq_info.is_chip_pin;
 
        irq_req_len = pb_encode_request(buffer,
                                        sizeof(buffer),
                                        &irq_req_payload,
                                        sns_interrupt_req_fields,
                                        NULL);

        if(irq_req_len > 0)
        {
          irq_req.message_id = SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REQ;
          irq_req.request_len = irq_req_len;
          irq_req.request = buffer;
          diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
 
          if(NULL != state->interrupt_data_stream)
          {
            diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
            /** Send encoded request to Interrupt Sensor */
            state->interrupt_data_stream->api->send_request(state->interrupt_data_stream,
                                                            &irq_req);
            diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
 
            state->irq_info.is_registered = true;
          }
        }
      }
    }
    else
    {
      if(state->mag_req.sample_rate != AK0991X_ODR_0)
      {
        if(!state->timer_stream_is_created)
        {
          stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                         this,
                                                         state->timer_suid,
                                                         &state->timer_data_stream);
          state->timer_stream_is_created = true;
        }
        sns_request timer_req;
        sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
        size_t req_len;
        uint8_t buffer[20];
        sns_memset(buffer, 0, sizeof(buffer));
        req_payload.is_periodic = true;
        req_payload.start_time = sns_get_system_time();
        req_payload.timeout_period = sns_convert_ns_to_ticks(1 / state->mag_req.sample_rate * 1000 * 1000 * 1000);
  
        req_len = pb_encode_request(buffer,
                                    sizeof(buffer),
                                    &req_payload,
                                    sns_timer_sensor_config_fields,
                                    NULL);
      
        diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__, "AK0991x timer req");
        if(req_len > 0)
        {
           diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__, "AK0991x timer req");
          timer_req.message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG;
          timer_req.request_len = req_len;
          timer_req.request = buffer;
          if(NULL != state->timer_data_stream)
          {
            diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__, "AK0991x timer req");
            /** Send encoded request to Timer Sensor */
            state->timer_data_stream->api->send_request(state->timer_data_stream, &timer_req);
          }
        }
      }
      else
      {
        if(state->timer_stream_is_created)
        {
          stream_mgr->api->remove_stream(stream_mgr, state->timer_data_stream);
          state->timer_stream_is_created = false;
        }
      }
    }
  }
  else if(state->client_req_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
  {
     diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);
     // 1. Extract test type from client_request.
     // 2. Configure sensor HW for test type.
     // 3. send_request() for Timer Sensor in case test needs polling/waits.
     // 4. Factory test is TBD.
  }
  diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);


  // Turn COM port OFF
  state->com_port_info.port_handle->com_port_api->sns_scp_update_bus_power(state->com_port_info.port_handle,
                                                                           false);

  return SNS_RC_SUCCESS;
}

static sns_rc ak0991x_inst_deinit(sns_sensor_instance *const this,
    sns_sensor_state *sensor_state)
{
  UNUSED_VAR(sensor_state);
  ak0991x_instance_state *state =
                  (ak0991x_instance_state*)this->state->state;
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_mgr =
      (sns_stream_service*)service_mgr->get_service(service_mgr,
                                                    SNS_STREAM_SERVICE);
   state->diag_service =  (sns_diag_service*)
      service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);

 sns_diag_service* diag = state->diag_service;
  diag->api->sensor_inst_printf(diag, this, &state->mag_info.suid, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);

  if(state->timer_stream_is_created)
  {
    stream_mgr->api->remove_stream(stream_mgr, state->timer_data_stream);
    state->timer_stream_is_created = false;
  }

  stream_mgr->api->remove_stream(stream_mgr, state->interrupt_data_stream);

  stream_mgr->api->remove_stream(stream_mgr, state->async_com_port_data_stream);

  state->com_port_info.port_handle->com_port_api->sns_scp_close(state->com_port_info.port_handle);
  sns_scp_deregister_com_port(state->com_port_info.port_handle);

  return SNS_RC_SUCCESS;
}

/** Public Data Definitions. */

sns_sensor_instance_api ak0991x_sensor_instance_api =
{
  .struct_len        = sizeof(sns_sensor_instance_api),
  .init              = &ak0991x_inst_init,
  .deinit            = &ak0991x_inst_deinit,
  .set_client_config = &ak0991x_inst_set_client_config,
  .notify_event      = &ak0991x_inst_notify_event
};

