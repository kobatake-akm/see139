/**
 * @file sns_ak0991x_sensor_instance.c
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
 * Authors(, name)  : Masahiko Fukasawa, Tomoya Nakajima
 * Version          : v2017.06.13
 * Date(MM/DD/YYYY) : 06/13/2017
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
#include "sns_types.h"

#include "sns_ak0991x_hal.h"
#include "sns_ak0991x_sensor.h"
#include "sns_ak0991x_sensor_instance.h"

#include "sns_interrupt.pb.h"
#include "sns_async_com_port.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_diag_service.h"
#include "sns_diag.pb.h"
#include "sns_sync_com_port_service.h"
#include "sns_printf.h"

/** See sns_sensor_instance_api::init */
sns_rc ak0991x_inst_init(sns_sensor_instance *const this,
                                sns_sensor_state const *sstate)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;
  ak0991x_state *sensor_state =
    (ak0991x_state *)sstate->state;
  float               data[3];
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service  *stream_mgr = (sns_stream_service *)
    service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
  uint64_t buffer[10];
  pb_ostream_t stream = pb_ostream_from_buffer((pb_byte_t *)buffer, sizeof(buffer));
  sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
  sns_rc rv;
  uint8_t arr_index = 0;
  float diag_temp[AK0991X_NUM_AXES];
  pb_float_arr_arg arg = {.arr = (float*)diag_temp, .arr_len = AK0991X_NUM_AXES,
    .arr_index = &arr_index};
  batch_sample.sample.funcs.encode = &pb_encode_float_arr_cb;
  batch_sample.sample.arg = &arg;

  state->diag_service = (sns_diag_service *)
    service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);
  state->scp_service = (sns_sync_com_port_service *)
    service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);

  /**----------------Copy Sensor UID in instance state---------------*/
  sns_memscpy(&state->mag_info.suid,
              sizeof(state->mag_info.suid),
              &((sns_sensor_uid)MAG_SUID),
              sizeof(state->mag_info.suid));

  SNS_INST_PRINTF(ERROR, this, "ak0991x inst init" );

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

  switch (state->mag_info.device_select)
  {
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
    state->mag_info.resolution = AK09916_RESOLUTION;
    state->mag_info.use_fifo = false;
    state->mag_info.max_fifo_size = AK09916_FIFO_SIZE;
    state->mag_info.use_dri = false;
    break;

  case AK09916D:
    state->mag_info.resolution = AK09916_RESOLUTION;
    state->mag_info.use_fifo = false;
    state->mag_info.max_fifo_size = AK09916_FIFO_SIZE;
    state->mag_info.use_dri = AK0991X_USE_DRI;
    break;

  case AK09917:
    state->mag_info.resolution = AK09917_RESOLUTION;
    state->mag_info.use_fifo = AK0991X_ENABLE_FIFO;
    state->mag_info.max_fifo_size = AK09917_FIFO_SIZE;
    state->mag_info.use_dri = AK0991X_USE_DRI;
    break;

  case AK09918:
    state->mag_info.resolution = AK09918_RESOLUTION;
    state->mag_info.use_fifo = false;
    state->mag_info.max_fifo_size = AK09918_FIFO_SIZE;
    state->mag_info.use_dri = false;
    break;

  default:
    return SNS_RC_FAILED;
  }

  state->pre_timestamp = 0;
  state->this_is_first_data = true;

  state->encoded_mag_event_len = pb_get_encoded_size_sensor_stream_event(data, AK0991X_NUM_AXES);


  rv = stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                      this,
                                                      sensor_state->irq_suid,
                                                      &state->interrupt_data_stream);

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  rv = stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                      this,
                                                      sensor_state->acp_suid,
                                                      &state->async_com_port_data_stream);

  if (rv != SNS_RC_SUCCESS)
  {
    stream_mgr->api->remove_stream(stream_mgr, state->interrupt_data_stream);
    return rv;
  }


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

  state->scp_service->api->sns_scp_register_com_port(&state->com_port_info.com_config,
                                                     &state->com_port_info.port_handle );

  state->scp_service->api->sns_scp_open(state->com_port_info.port_handle);

  state->scp_service->api->sns_scp_update_bus_power(
    state->com_port_info.port_handle,
    false);

  /** Initialize IRQ info to be used by the Instance */
  sns_memscpy(&state->irq_info,
              sizeof(state->irq_info),
              &sensor_state->irq_config,
              sizeof(sensor_state->irq_config));

  state->irq_info.is_registered = false;
  state->irq_info.detect_irq_event = false;
  state->irq_info.is_ready = false;

  /** Configure the Async Com Port */
  uint8_t                   pb_encode_buffer[100];
  uint32_t                  enc_len;

  state->ascp_config.bus_instance = sensor_state->com_port_info.com_config.bus_instance;
  if (sensor_state->com_port_info.com_config.bus_type == SNS_BUS_I2C)
  {
    state->ascp_config.bus_type = SNS_ASYNC_COM_PORT_BUS_TYPE_I2C;
  }
  else
  {
    state->ascp_config.bus_type = SNS_ASYNC_COM_PORT_BUS_TYPE_SPI;
  }
  state->ascp_config.max_bus_speed_kHz =
    sensor_state->com_port_info.com_config.max_bus_speed_KHz;
  state->ascp_config.min_bus_speed_kHz =
    sensor_state->com_port_info.com_config.min_bus_speed_KHz;
  state->ascp_config.reg_addr_type = SNS_ASYNC_COM_PORT_REG_ADDR_TYPE_8_BIT;
  state->ascp_config.slave_control = sensor_state->com_port_info.com_config.slave_control;
  enc_len = pb_encode_request(pb_encode_buffer, 100, &state->ascp_config,
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

  /** Determine size of sns_diag_sensor_state_raw as defined in
   *  sns_diag.proto
   *  sns_diag_sensor_state_raw is a repeated array of samples of
   *  type sns_diag_batch sample. The following determines the
   *  size of sns_diag_sensor_state_raw with a single batch
   *  sample */
  if(pb_encode_tag(&stream, PB_WT_STRING,
                    sns_diag_sensor_state_raw_sample_tag))
  {
    if(pb_encode_delimited(&stream, sns_diag_batch_sample_fields,
                               &batch_sample))
    {
      state->log_raw_encoded_size = stream.bytes_written;
    }
  }


  SNS_INST_PRINTF(ERROR, this, "before dae_if init" );

  ak0991x_dae_if_init(this, stream_mgr, &sensor_state->dae_suid, &((sns_sensor_uid)MAG_SUID));

  return SNS_RC_SUCCESS;
}

// QC: Removed sensor_state parameter
sns_rc ak0991x_inst_deinit(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service  *stream_mgr =
    (sns_stream_service *)service_mgr->get_service(service_mgr,
                                                   SNS_STREAM_SERVICE);

  ak0991x_dae_if_deinit(state, stream_mgr);

  if (state->timer_stream_is_created)
  {
    stream_mgr->api->remove_stream(stream_mgr, state->timer_data_stream);
    state->timer_stream_is_created = false;
  }

  stream_mgr->api->remove_stream(stream_mgr, state->interrupt_data_stream);

  stream_mgr->api->remove_stream(stream_mgr, state->async_com_port_data_stream);

  state->scp_service->api->sns_scp_close(state->com_port_info.port_handle);
  state->scp_service->api->sns_scp_deregister_com_port(state->com_port_info.port_handle);

  return SNS_RC_SUCCESS;
}

