/**
 * @file sns_lsm6ds3_sensor_instance.c
 *
 * LSM6DS3 Accel virtual Sensor Instance implementation.
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
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

#include "sns_lsm6ds3_hal.h"
#include "sns_lsm6ds3_sensor.h"
#include "sns_lsm6ds3_sensor_instance.h"

#include "sns_interrupt.pb.h"
#include "sns_async_com_port.pb.h"
#include "sns_timer.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"
#include "sns_diag_service.h"

const odr_reg_map reg_map[LSM6DS3_REG_MAP_TABLE_SIZE] =
{
  {
    .odr = LSM6DS3_ODR_0,
    .accel_odr_reg_value = LSM6DS3_ACCEL_ODR_OFF,
    .gyro_odr_reg_value = LSM6DS3_GYRO_ODR_OFF,
    .discard_samples = 0
  },
  {
    .odr = LSM6DS3_ODR_13,
    .accel_odr_reg_value = LSM6DS3_ACCEL_ODR13,
    .gyro_odr_reg_value = LSM6DS3_GYRO_ODR13,
    .discard_samples = 2
  },
  {
    .odr = LSM6DS3_ODR_26,
    .accel_odr_reg_value = LSM6DS3_ACCEL_ODR26,
    .gyro_odr_reg_value = LSM6DS3_GYRO_ODR26,
    .discard_samples = 2
  },
  {
    .odr = LSM6DS3_ODR_52,
    .accel_odr_reg_value = LSM6DS3_ACCEL_ODR52,
    .gyro_odr_reg_value = LSM6DS3_GYRO_ODR52,
    .discard_samples = 2
  },
  {
    .odr = LSM6DS3_ODR_104,
    .accel_odr_reg_value = LSM6DS3_ACCEL_ODR104,
    .gyro_odr_reg_value = LSM6DS3_GYRO_ODR104,
    .discard_samples = 3
  },
  {
    .odr = LSM6DS3_ODR_208,
    .accel_odr_reg_value = LSM6DS3_ACCEL_ODR208,
    .gyro_odr_reg_value = LSM6DS3_GYRO_ODR208,
    .discard_samples = 4
  },
  {
    .odr = LSM6DS3_ODR_416,
    .accel_odr_reg_value = LSM6DS3_ACCEL_ODR416,
    .gyro_odr_reg_value = LSM6DS3_GYRO_ODR416,
    .discard_samples = 2
  },
  {
    .odr = LSM6DS3_ODR_833,
    .accel_odr_reg_value = LSM6DS3_ACCEL_ODR833,
    .gyro_odr_reg_value = LSM6DS3_GYRO_ODR833,
    .discard_samples = 2
  },
  {
    .odr = LSM6DS3_ODR_1660,
    .accel_odr_reg_value = LSM6DS3_ACCEL_ODR1660,
    .gyro_odr_reg_value = LSM6DS3_GYRO_ODR1660,
    .discard_samples = 2
  },
  {
    .odr = LSM6DS3_ODR_3330,
    .accel_odr_reg_value = LSM6DS3_ACCEL_ODR3330,
    .gyro_odr_reg_value = LSM6DS3_GYRO_ODR1660,
    .discard_samples = 2
  },
  {
    .odr = LSM6DS3_ODR_6660,
    .accel_odr_reg_value = LSM6DS3_ACCEL_ODR6660,
    .gyro_odr_reg_value = LSM6DS3_GYRO_ODR1660,
    .discard_samples = 2
  },
};

static void lsm6ds3_start_sensor_temp_report_timer(sns_sensor_instance *this, uint32_t timeout_ms)
{
  lsm6ds3_instance_state *state =
       (lsm6ds3_instance_state*)this->state->state;
  sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
  size_t req_len;
  uint8_t buffer[50];
  sns_memset(buffer, 0, sizeof(buffer));
  req_payload.is_periodic = false;
  req_payload.start_time = sns_get_system_time();
  req_payload.timeout_period = sns_convert_ns_to_ticks(timeout_ms * 1000 * 1000);

  req_len = pb_encode_request(buffer, sizeof(buffer), &req_payload,
                              sns_timer_sensor_config_fields, NULL);
  if(req_len > 0)
  {
    sns_request timer_req =
      {  .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
         .request = buffer, .request_len = req_len  };
    state->timer_data_stream->api->send_request(state->timer_data_stream, &timer_req);
  }
  else
  {
    //diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
    //                         "LSM timer req encode error");
  }
}

static sns_rc lsm6ds3_accel_match_odr(float desired_sample_rate,
                                      float *chosen_sample_rate,
                                      lsm6ds3_accel_odr *chosen_reg_value,
                                      uint8_t *num_samples_to_discard)
{
  uint8_t idx;

  if((LSM6DS3_ODR_416 < desired_sample_rate)
     ||
     NULL == chosen_sample_rate
     ||
     NULL == chosen_reg_value)
  {
    return SNS_RC_NOT_SUPPORTED;
  }

  for(idx = 0; idx < ARR_SIZE(reg_map); idx++)
  {
    if(desired_sample_rate <= reg_map[idx].odr)
    {
      break;
    }
  }

  if (idx >= ARR_SIZE(reg_map))
  {
    return SNS_RC_NOT_SUPPORTED;
  }

  *chosen_sample_rate = reg_map[idx].odr;
  *chosen_reg_value = reg_map[idx].accel_odr_reg_value;
  *num_samples_to_discard = reg_map[idx].discard_samples;

  return SNS_RC_SUCCESS;
}

static sns_rc lsm6ds3_gyro_match_odr(float desired_sample_rate,
                                     float *chosen_sample_rate,
                                     lsm6ds3_gyro_odr *chosen_reg_value)
{
  uint8_t idx;

  if((LSM6DS3_ODR_416 < desired_sample_rate)
     ||
     NULL == chosen_sample_rate
     ||
     NULL == chosen_reg_value)
  {
    return SNS_RC_NOT_SUPPORTED;
  }

  for(idx = 0; idx < ARR_SIZE(reg_map); idx++)
  {
    if(desired_sample_rate <= reg_map[idx].odr)
    {
      break;
    }
  }

  if (idx >= ARR_SIZE(reg_map))
  {
    return SNS_RC_NOT_SUPPORTED;
  }

  *chosen_sample_rate = reg_map[idx].odr;
  *chosen_reg_value = reg_map[idx].gyro_odr_reg_value;

  return SNS_RC_SUCCESS;
}

sns_rc lsm6ds3_validate_sensor_temp_odr(lsm6ds3_instance_state *state)
{
  if(state->sensor_temp_info.report_timer_hz <= LSM6DS3_SENSOR_TEMP_ODR_1)
  {
    state->sensor_temp_info.report_timer_hz = LSM6DS3_SENSOR_TEMP_ODR_1;
  }
  else if(state->sensor_temp_info.report_timer_hz > LSM6DS3_SENSOR_TEMP_ODR_1
          &&
          state->sensor_temp_info.report_timer_hz <= LSM6DS3_SENSOR_TEMP_ODR_5)
  {
    state->sensor_temp_info.report_timer_hz = LSM6DS3_SENSOR_TEMP_ODR_5;
  }
  else
  {
    return SNS_RC_NOT_SUPPORTED;
  }

  return SNS_RC_SUCCESS;
}

/** See sns_sensor_instance_api::notify_event */
static sns_rc lsm6ds3_inst_notify_event(sns_sensor_instance *const this)
{
  lsm6ds3_instance_state *state =
         (lsm6ds3_instance_state*)this->state->state;
  sns_interrupt_event irq_event = sns_interrupt_event_init_zero;
  sns_sensor_event *event;
  sns_diag_service* diag = state->diag_service;

  // Turn COM port ON
  state->com_port_info.port_handle->com_port_api->sns_scp_update_bus_power(state->com_port_info.port_handle,
                                                                           true);

  // Handle interrupts
  if(NULL != state->interrupt_data_stream)
  {
    event = state->interrupt_data_stream->api->peek_input(state->interrupt_data_stream);
    while(NULL != event)
    {
      if(event->message_id != SNS_INTERRUPT_MSGID_SNS_INTERRUPT_EVENT) // TODO add config update message ID support
      {
        diag->api->sensor_inst_printf(diag, this, &state->accel_info.suid,
                                      SNS_ERROR, __FILENAME__, __LINE__,
                                      "Received invalid event id=%d",
                                      event->message_id);
      }
      else
      {
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
                                                     event->event_len);
        if(pb_decode(&stream, sns_interrupt_event_fields, &irq_event))
        {
           if(state->motion_accel_info.enable_md_int)
           {
             /**
              * 1. Handle MD interrupt: Send MD fired event to client.
              * 2. Disable MD.
              * 3. Start Motion Accel FIFO stream with desired config.
              */
             lsm6ds3_handle_md_interrupt(this, irq_event.timestamp);
             if(state->motion_accel_info.md_intr_fired)
             {
               state->motion_accel_info.enable_md_int = false;
               lsm6ds3_update_md_intr(this, false, false);
               lsm6ds3_set_md_config(state, false);
               lsm6ds3_set_fifo_config(state,
                                       state->motion_accel_info.desired_wmk,
                                       state->motion_accel_info.desired_odr,
                                       LSM6DS3_GYRO_ODR_OFF,
                                       state->fifo_info.fifo_enabled);
               lsm6ds3_stop_fifo_streaming(state);
               lsm6ds3_set_fifo_wmk(state);
               lsm6ds3_start_fifo_streaming(state);
               lsm6ds3_enable_fifo_intr(state, state->fifo_info.fifo_enabled);
               lsm6ds3_dump_reg(state, state->fifo_info.fifo_enabled);
             }
           }
           else
           {
             state->interrupt_timestamp = irq_event.timestamp;
             lsm6ds3_handle_interrupt_event(this);
           }
        }
      }
      event = state->interrupt_data_stream->api->get_next_input(state->interrupt_data_stream);
    }
  }

  // Handle Async Com Port events
  if(NULL != state->async_com_port_data_stream)
  {
    event = state->async_com_port_data_stream->api->peek_input(state->async_com_port_data_stream);
    while(NULL != event)
    {
      if(SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_ERROR == event->message_id)
      {
        //TODO: Warning;
        diag->api->sensor_inst_printf(diag, this, &state->accel_info.suid,
                                      SNS_ERROR, __FILENAME__, __LINE__,
                                      "Received ASCP error event id=%d",
                                      event->message_id);
      }
      else if(SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_VECTOR_RW == event->message_id)
      {
        pb_istream_t stream = pb_istream_from_buffer((uint8_t *)event->event, event->event_len);
        sns_ascp_for_each_vector_do(&stream, lsm6ds3_process_fifo_data_buffer, (void *)this);
      }
      event = state->async_com_port_data_stream->api->get_next_input(state->async_com_port_data_stream);
    }
  }

  // Handle Timer events
  if(NULL != state->timer_data_stream)
  {
    event = state->timer_data_stream->api->peek_input(state->timer_data_stream);
    while(NULL != event)
    {
      pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event->event,
                                                     event->event_len);
      sns_timer_sensor_event timer_event;
      if(pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
      {
        if(event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT)
        {
          if(state->fifo_info.publish_sensors & LSM6DS3_SENSOR_TEMP
             &&
             state->sensor_temp_info.timer_is_active
             &&
             state->sensor_temp_info.report_timer_hz > 0)
          {
            state->sensor_temp_info.timer_is_active = false;
            lsm6ds3_handle_sensor_temp_sample(this);
            // Re-arm timer
            lsm6ds3_start_sensor_temp_report_timer(this,
                                                   1000 / state->sensor_temp_info.report_timer_hz);
            state->sensor_temp_info.timer_is_active = true;
          }
         }
      }
      else
      {
      }
      event = state->timer_data_stream->api->get_next_input(state->timer_data_stream);
    }
  }

  // Turn COM port OFF
  state->com_port_info.port_handle->com_port_api->sns_scp_update_bus_power(state->com_port_info.port_handle,
                                                                           false);

  return SNS_RC_SUCCESS;
}

/** See sns_sensor_instance_api::init */
static sns_rc lsm6ds3_inst_init(sns_sensor_instance *const this,
    sns_sensor_state const *sstate)
{
  lsm6ds3_instance_state *state =
              (lsm6ds3_instance_state*)this->state->state;
  lsm6ds3_state *sensor_state =
              (lsm6ds3_state*)sstate->state;
  float data[3];
  float temp_data[1];
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_mgr = (sns_stream_service*)
              service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

  state->diag_service =  (sns_diag_service*)
      service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);

  // Setup stream connections with dependent Sensors
  stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                 this,
                                                 sensor_state->irq_suid,
                                                 &state->interrupt_data_stream);

  stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                 this,
                                                 sensor_state->acp_suid,
                                                 &state->async_com_port_data_stream);

  stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                 this,
                                                 sensor_state->timer_suid,
                                                 &state->timer_data_stream);

  /**-------------------------Init FIFO State-------------------------*/
  state->fifo_info.fifo_enabled = 0;
  state->fifo_info.fifo_int_enabled = 0;
  state->fifo_info.fifo_rate = LSM6DS3_ACCEL_ODR_OFF;
  state->fifo_info.desired_fifo_rate = LSM6DS3_ACCEL_ODR_OFF;
  state->fifo_info.cur_wmk = 0;

  /**-------------------------Init Accel State-------------------------*/
  state->accel_info.desired_odr = LSM6DS3_ACCEL_ODR_OFF;
  state->accel_info.curr_odr = LSM6DS3_ACCEL_ODR_OFF;
  state->accel_info.sstvt = LSM6DS3_ACCEL_SSTVT_8G;
  state->accel_info.range = LSM6DS3_ACCEL_RANGE_8G;
  state->accel_info.bw = LSM6DS3_ACCEL_BW50;
  state->accel_info.lp_mode = false;

  /**-------------------------Init Gyro State-------------------------*/
  state->gyro_info.desired_odr = LSM6DS3_GYRO_ODR_OFF;
  state->gyro_info.curr_odr = LSM6DS3_GYRO_ODR_OFF;
  state->gyro_info.sstvt = LSM6DS3_GYRO_SSTVT_2000DPS;
  state->gyro_info.range = STM_LSM6DS3_GYRO_RANGE_2000DPS;

  state->encoded_imu_event_len = pb_get_encoded_size_sensor_stream_event(data, 3);
  state->encoded_sensor_temp_event_len = pb_get_encoded_size_sensor_stream_event(temp_data, 1);

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
static sns_rc lsm6ds3_inst_set_client_config(sns_sensor_instance *const this,
                                             sns_request const *client_request)
{
  lsm6ds3_instance_state *state =
                  (lsm6ds3_instance_state*)this->state->state;
  state->client_req_id = client_request->message_id;
  float desired_sample_rate = 0;
  float desired_report_rate = 0;
  float accel_chosen_sample_rate = 0;
  float gyro_chosen_sample_rate = 0;
  lsm6ds3_accel_odr accel_chosen_sample_rate_reg_value;
  lsm6ds3_gyro_odr gyro_chosen_sample_rate_reg_value;
  uint16_t desired_wmk = 0;
  sns_rc rv = SNS_RC_SUCCESS;
  uint8_t num_samples_to_discard;

  // Turn COM port ON
  state->com_port_info.port_handle->com_port_api->sns_scp_update_bus_power(state->com_port_info.port_handle,
                                                                           true);

  // Reset the device if not streaming.
  if(state->fifo_info.fifo_rate == LSM6DS3_ACCEL_ODR_OFF
     &&
     state->motion_accel_info.curr_odr == LSM6DS3_ACCEL_ODR_OFF)
  {
    lsm6ds3_reset_device(state->com_port_info.port_handle,
                         LSM6DS3_ACCEL | LSM6DS3_GYRO | LSM6DS3_MOTION_ACCEL | LSM6DS3_SENSOR_TEMP);
  }

  if(client_request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG
     ||
     client_request->message_id == SNS_MOTION_ACCEL_MSGID_SNS_MOTION_ACCEL_CONFIG)
  {
     // 1. Extract sample, report rates from client_request.
     // 2. Configure sensor HW.
     // 3. sendRequest() for Timer to start/stop in case of polling using timer_data_stream.
     // 4. sendRequest() for Intrerupt register/de-register in case of DRI using interrupt_data_stream.
     // 5. Save the current config information like type, sample_rate, report_rate, etc.
     if(client_request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
     {
       sns_lsm6ds3_imu_req *payload =
          (sns_lsm6ds3_imu_req*)client_request->request;
       desired_sample_rate = payload->sample_rate;
       desired_report_rate = payload->report_rate;
     }
     else if(client_request->message_id == SNS_MOTION_ACCEL_MSGID_SNS_MOTION_ACCEL_CONFIG)
     {
       sns_lsm6ds3_ma_req *payload =
          (sns_lsm6ds3_ma_req*)client_request->request;
       desired_sample_rate = payload->sample_rate;
       desired_report_rate = payload->report_rate;
       state->ma_req.enable_motion_detect = payload->enable_motion_detect;
     }
     else // TODO self-test
     {
     }

     if(desired_report_rate > desired_sample_rate)
     {
       // bad request. Return error or default report_rate to sample_rate
       desired_report_rate = desired_sample_rate;
     }

     rv = lsm6ds3_accel_match_odr(desired_sample_rate,
                                  &accel_chosen_sample_rate,
                                  &accel_chosen_sample_rate_reg_value,
                                  &num_samples_to_discard);
     if(rv != SNS_RC_SUCCESS)
     {
       // TODO Unsupported rate. Report error using sns_std_error_event.
       //return rv;
     }
     rv = lsm6ds3_gyro_match_odr(desired_sample_rate,
                                 &gyro_chosen_sample_rate,
                                 &gyro_chosen_sample_rate_reg_value);
     if(rv != SNS_RC_SUCCESS)
     {
       // TODO Unsupported rate. Report error using sns_std_error_event.
       //return rv;
     }

     if(state->fifo_info.publish_sensors & LSM6DS3_SENSOR_TEMP)
     {
       rv = lsm6ds3_validate_sensor_temp_odr(state);
       if(rv != SNS_RC_SUCCESS)
       {
         // TODO Unsupported rate. Report error using sns_std_error_event.
         //return rv;
       }
     }

     if(state->accel_info.curr_odr != accel_chosen_sample_rate_reg_value)
     {
       state->accel_info.num_samples_to_discard = num_samples_to_discard;
       state->gyro_info.num_samples_to_discard = num_samples_to_discard;
       state->motion_accel_info.num_samples_to_discard = num_samples_to_discard;
     }
     else
     {
       state->accel_info.num_samples_to_discard = 0;
       state->gyro_info.num_samples_to_discard = 0;
       state->motion_accel_info.num_samples_to_discard = 0;
     }

     if(desired_report_rate != 0)
     {
       desired_wmk = (uint16_t)(accel_chosen_sample_rate / desired_report_rate);
     }

     if(LSM6DS3_MAX_FIFO <= desired_wmk)
     {
       desired_wmk = LSM6DS3_MAX_FIFO;
     }

     if(!state->irq_info.is_registered)
     {
        sns_request irq_req;
        sns_interrupt_req irq_req_payload;
        size_t irq_req_len;
        uint8_t buffer[20];
        sns_memset(buffer, 0, sizeof(buffer));

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

          if(NULL != state->interrupt_data_stream)
          {
            /** Send encoded request to Interrupt Sensor */
            state->interrupt_data_stream->api->send_request(state->interrupt_data_stream,
                                                            &irq_req);

            state->irq_info.is_registered = true;
          }
        }
     }

     state->imu_req.sample_rate = accel_chosen_sample_rate;
     state->ma_req.sample_rate = accel_chosen_sample_rate;

     if(state->motion_accel_info.enable_md_int)
     {
       /** When the Instance is OK to turn MD interrupt ON, close
        *  any active FIFO stream. */
        if(state->fifo_info.fifo_rate > LSM6DS3_ACCEL_ODR_OFF)
        {
          lsm6ds3_set_fifo_config(state,
                                  0,
                                  LSM6DS3_ACCEL_ODR_OFF,
                                  LSM6DS3_GYRO_ODR_OFF,
                                  state->fifo_info.fifo_enabled);
          lsm6ds3_stop_fifo_streaming(state);
          state->fifo_info.desired_fifo_rate = LSM6DS3_ACCEL_ODR_OFF;
          state->accel_info.desired_odr = LSM6DS3_ACCEL_ODR_OFF;
          state->gyro_info.desired_odr = LSM6DS3_GYRO_ODR_OFF;
          lsm6ds3_set_fifo_wmk(state);
          lsm6ds3_disable_fifo_intr(state);
        }
       lsm6ds3_set_ma_config(state,
                             desired_wmk,
                             accel_chosen_sample_rate_reg_value,
                             state->fifo_info.fifo_enabled);
       lsm6ds3_set_md_config(state, true);
       lsm6ds3_update_md_intr(this, true, false);
     }
     else
     {
        if(state->motion_accel_info.md_client_present
           &&
           !state->motion_accel_info.enable_md_int)
        {
          lsm6ds3_update_md_intr(this, false, true);
          lsm6ds3_set_md_config(state, false);
        }
       lsm6ds3_set_fifo_config(state,
                               desired_wmk,
                               accel_chosen_sample_rate_reg_value,
                               gyro_chosen_sample_rate_reg_value,
                               state->fifo_info.fifo_enabled);
       lsm6ds3_send_config_event(this);
       //lsm6ds3_flush_fifo(accel_sensor);
       lsm6ds3_stop_fifo_streaming(state);
       lsm6ds3_set_fifo_wmk(state);
       lsm6ds3_start_fifo_streaming(state);

       // Enable interrupt only for accel, gyro and motion accel clients
       if(state->fifo_info.publish_sensors & (LSM6DS3_ACCEL | LSM6DS3_GYRO | LSM6DS3_MOTION_ACCEL))
       {
         lsm6ds3_enable_fifo_intr(state, state->fifo_info.fifo_enabled);
       }

       // Start 1Hz Timer for Sensor Temperature
       if(state->fifo_info.publish_sensors & LSM6DS3_SENSOR_TEMP
          &&
          !state->sensor_temp_info.timer_is_active
          &&
          state->sensor_temp_info.report_timer_hz > 0)
       {
         lsm6ds3_start_sensor_temp_report_timer(this,
                                                (1000 / state->sensor_temp_info.report_timer_hz));
         state->sensor_temp_info.timer_is_active = true;
       }
       else if(!(state->fifo_info.publish_sensors & LSM6DS3_SENSOR_TEMP)
               &&
               state->sensor_temp_info.timer_is_active)
       {
         state->sensor_temp_info.timer_is_active = false;
       }
     }
     lsm6ds3_dump_reg(state, state->fifo_info.fifo_enabled);
  }
  else if(state->client_req_id == SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
  {
     // 1. Extract test type from client_request.
     // 2. Configure sensor HW for test type.
     // 3. send_request() for Timer Sensor in case test needs polling/waits.
     // 4. Factory test is TBD.
  }

  // Turn COM port OFF
  state->com_port_info.port_handle->com_port_api->sns_scp_update_bus_power(state->com_port_info.port_handle,
                                                                           false);

  return SNS_RC_SUCCESS;
}

static sns_rc lsm6ds3_inst_deinit(sns_sensor_instance *const this,
    sns_sensor_state *sensor_state)
{
  UNUSED_VAR(sensor_state);
  lsm6ds3_instance_state *state =
                  (lsm6ds3_instance_state*)this->state->state;
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_mgr =
      (sns_stream_service*)service_mgr->get_service(service_mgr,
                                                    SNS_STREAM_SERVICE);

  stream_mgr->api->remove_stream(stream_mgr, state->interrupt_data_stream);
  stream_mgr->api->remove_stream(stream_mgr, state->async_com_port_data_stream);
  stream_mgr->api->remove_stream(stream_mgr, state->timer_data_stream);
  state->com_port_info.port_handle->com_port_api->sns_scp_close(state->com_port_info.port_handle);
  sns_scp_deregister_com_port(state->com_port_info.port_handle);

  return SNS_RC_SUCCESS;
}

/** Public Data Definitions. */

sns_sensor_instance_api lsm6ds3_sensor_instance_api =
{
  .struct_len        = sizeof(sns_sensor_instance_api),
  .init              = &lsm6ds3_inst_init,
  .deinit            = &lsm6ds3_inst_deinit,
  .set_client_config = &lsm6ds3_inst_set_client_config,
  .notify_event      = &lsm6ds3_inst_notify_event
};

