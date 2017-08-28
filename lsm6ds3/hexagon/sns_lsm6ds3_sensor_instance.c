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
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_sensor_instance.h"
#include "sns_sensor_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_types.h"

#include "sns_lsm6ds3_hal.h"
#include "sns_lsm6ds3_sensor.h"
#include "sns_lsm6ds3_sensor_instance.h"

#include "sns_async_com_port.pb.h"
#include "sns_interrupt.pb.h"

#include "pb_decode.h"
#include "pb_encode.h"
#include "sns_diag.pb.h"
#include "sns_diag_service.h"
#include "sns_pb_util.h"
#include "sns_sync_com_port_service.h"

static void inst_cleanup(sns_sensor_instance *const this, sns_stream_service *stream_mgr)
{
  lsm6ds3_instance_state *state = (lsm6ds3_instance_state*)this->state->state;

  lsm6ds3_dae_if_deinit(state, stream_mgr);

  sns_sensor_util_remove_sensor_instance_stream(this, &state->interrupt_data_stream);
  sns_sensor_util_remove_sensor_instance_stream(this, &state->async_com_port_data_stream);
  sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_data_stream);
  sns_sensor_util_remove_sensor_instance_stream(this, &state->timer_data_stream);

  if(NULL != state->scp_service)
  {
    state->scp_service->api->sns_scp_close(state->com_port_info.port_handle);
    state->scp_service->api->sns_scp_deregister_com_port(&state->com_port_info.port_handle);
    state->scp_service = NULL;
  }
}

/** See sns_sensor_instance_api::init */
sns_rc lsm6ds3_inst_init(sns_sensor_instance *const this,
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
  uint64_t buffer[10];
  pb_ostream_t stream = pb_ostream_from_buffer((pb_byte_t *)buffer, sizeof(buffer));
  sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
  uint8_t arr_index = 0;
  float diag_temp[LSM6DS3_NUM_AXES];
  pb_float_arr_arg arg = {.arr = (float*)diag_temp, .arr_len = LSM6DS3_NUM_AXES,
    .arr_index = &arr_index};
  batch_sample.sample.funcs.encode = &pb_encode_float_arr_cb;
  batch_sample.sample.arg = &arg;

  state->scp_service = (sns_sync_com_port_service*)
              service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);
  state->island_service = (sns_island_service*)
              service_mgr->get_service(service_mgr, SNS_ISLAND_SERVICE);

  /**---------Setup stream connections with dependent Sensors---------*/

  stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                 this,
                                                 sensor_state->common.irq_suid,
                                                 &state->interrupt_data_stream);

  stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                 this,
                                                 sensor_state->common.acp_suid,
                                                 &state->async_com_port_data_stream);

  /** Initialize COM port to be used by the Instance */
  sns_memscpy(&state->com_port_info.com_config,
              sizeof(state->com_port_info.com_config),
              &sensor_state->common.com_port_info.com_config,
              sizeof(sensor_state->common.com_port_info.com_config));

  state->scp_service->api->sns_scp_register_com_port(&state->com_port_info.com_config,
                                              &state->com_port_info.port_handle);

  if(NULL == state->interrupt_data_stream ||
     NULL == state->async_com_port_data_stream ||
     NULL == state->com_port_info.port_handle)
  {
    inst_cleanup(this, stream_mgr);
    return SNS_RC_FAILED;
  }

  /**----------- Copy all Sensor UIDs in instance state -------------*/
  sns_memscpy(&state->accel_info.suid,
              sizeof(state->accel_info.suid),
              &((sns_sensor_uid)ACCEL_SUID),
              sizeof(state->accel_info.suid));
  sns_memscpy(&state->gyro_info.suid,
              sizeof(state->gyro_info.suid),
              &((sns_sensor_uid)GYRO_SUID),
              sizeof(state->gyro_info.suid));
  sns_memscpy(&state->md_info.suid,
              sizeof(state->md_info.suid),
              &((sns_sensor_uid)MOTION_DETECT_SUID),
              sizeof(state->md_info.suid));
  sns_memscpy(&state->sensor_temp_info.suid,
              sizeof(state->sensor_temp_info.suid),
              &((sns_sensor_uid)SENSOR_TEMPERATURE_SUID),
              sizeof(state->sensor_temp_info.suid));
  sns_memscpy(&state->timer_suid,
              sizeof(state->timer_suid),
              &sensor_state->common.timer_suid,
              sizeof(sensor_state->common.timer_suid));

  /**-------------------------Init FIFO State-------------------------*/
  state->fifo_info.fifo_enabled = 0;
  state->fifo_info.fifo_rate = LSM6DS3_ACCEL_ODR_OFF;
  state->fifo_info.cur_wmk = 0;
  state->fifo_info.max_requested_wmk = 0;

  /**-------------------------Init Accel State-------------------------*/
  state->accel_info.curr_odr = LSM6DS3_ACCEL_ODR_OFF;
  state->accel_info.sstvt = LSM6DS3_ACCEL_SSTVT_8G;
  state->accel_info.range = LSM6DS3_ACCEL_RANGE_8G;
  state->accel_info.bw = LSM6DS3_ACCEL_BW50;
  state->accel_info.lp_mode = false;

  /**-------------------------Init Gyro State-------------------------*/
  state->gyro_info.curr_odr = LSM6DS3_GYRO_ODR_OFF;
  state->gyro_info.sstvt = LSM6DS3_GYRO_SSTVT_2000DPS;
  state->gyro_info.range = STM_LSM6DS3_GYRO_RANGE_2000DPS;

  state->encoded_imu_event_len = pb_get_encoded_size_sensor_stream_event(data, 3);
  state->encoded_sensor_temp_event_len = pb_get_encoded_size_sensor_stream_event(temp_data, 1);

  state->diag_service =  (sns_diag_service*)
      service_mgr->get_service(service_mgr, SNS_DIAG_SERVICE);

  state->scp_service =  (sns_sync_com_port_service*)
      service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);

  state->scp_service->api->sns_scp_open(state->com_port_info.port_handle);

  lsm6ds3_reset_device(  state->scp_service,state->com_port_info.port_handle,
                         LSM6DS3_ACCEL | LSM6DS3_GYRO | LSM6DS3_MOTION_DETECT | LSM6DS3_SENSOR_TEMP);

  state->scp_service->api->sns_scp_update_bus_power(state->com_port_info.port_handle,
                                                                           false);

  /** Initialize IRQ info to be used by the Instance */
  state->irq_info.irq_config = sensor_state->common.irq_config;
  state->irq_info.irq_ready = false;

  /** Configure the Async Com Port */
  {
    sns_data_stream* data_stream = state->async_com_port_data_stream;
    sns_com_port_config* com_config = &sensor_state->common.com_port_info.com_config;
    uint8_t pb_encode_buffer[100];
    sns_request async_com_port_request =
    {
      .message_id  = SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_CONFIG,
      .request     = &pb_encode_buffer
    };

    state->ascp_config.bus_type          = (com_config->bus_type == SNS_BUS_I2C) ? 
      SNS_ASYNC_COM_PORT_BUS_TYPE_I2C : SNS_ASYNC_COM_PORT_BUS_TYPE_SPI;
    state->ascp_config.slave_control     = com_config->slave_control;
    state->ascp_config.reg_addr_type     = SNS_ASYNC_COM_PORT_REG_ADDR_TYPE_8_BIT;
    state->ascp_config.min_bus_speed_kHz = com_config->min_bus_speed_KHz;
    state->ascp_config.max_bus_speed_kHz = com_config->max_bus_speed_KHz;
    state->ascp_config.bus_instance      = com_config->bus_instance;

    async_com_port_request.request_len =
      pb_encode_request(pb_encode_buffer,
                        sizeof(pb_encode_buffer),
                        &state->ascp_config,
                        sns_async_com_port_config_fields,
                        NULL);
    data_stream->api->send_request(data_stream, &async_com_port_request);
  }

  /** Copy down axis conversion settings */
  sns_memscpy(state->axis_map,  sizeof(sensor_state->common.axis_map),
              sensor_state->common.axis_map, sizeof(sensor_state->common.axis_map));

  /** Initialize factory calibration */
  state->accel_registry_cfg.fac_cal_corr_mat.e00 = 1.0;
  state->accel_registry_cfg.fac_cal_corr_mat.e11 = 1.0;
  state->accel_registry_cfg.fac_cal_corr_mat.e22 = 1.0;
  state->gyro_registry_cfg.fac_cal_corr_mat.e00 = 1.0;
  state->gyro_registry_cfg.fac_cal_corr_mat.e11 = 1.0;
  state->gyro_registry_cfg.fac_cal_corr_mat.e22 = 1.0;
  state->sensor_temp_registry_cfg.fac_cal_corr_mat.e00 = 1.0;
  state->sensor_temp_registry_cfg.fac_cal_corr_mat.e11 = 1.0;
  state->sensor_temp_registry_cfg.fac_cal_corr_mat.e22 = 1.0;

  /** Copy down MD configuration */
  sns_memscpy(&state->md_info.md_config, sizeof(state->md_info.md_config),
              &sensor_state->md_config, sizeof(sensor_state->md_config));

  /** Determine sizes of encoded logs */
  sns_diag_sensor_state_interrupt sensor_state_interrupt =
        sns_diag_sensor_state_interrupt_init_default;
  pb_get_encoded_size(&state->log_interrupt_encoded_size,
                      sns_diag_sensor_state_interrupt_fields,
                      &sensor_state_interrupt);

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

  lsm6ds3_dae_if_init(this, stream_mgr, &sensor_state->common.dae_suid, &sensor_state->my_suid);

  return SNS_RC_SUCCESS;
}

sns_rc lsm6ds3_inst_deinit(sns_sensor_instance *const this)
{
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_mgr =
      (sns_stream_service*)service_mgr->get_service(service_mgr,
                                                    SNS_STREAM_SERVICE);
  inst_cleanup(this, stream_mgr);

  return SNS_RC_SUCCESS;
}

/**
 * Sends a self-test completion event.
 *
 * @param[i] instance  Instance reference
 * @param[i] uid       Sensor UID
 *
 * @return none
 */
static void lsm6ds3_send_self_test_event(sns_sensor_instance *instance,
                                        sns_sensor_uid *uid, bool test_result,
                                        sns_physical_sensor_test_type type)
{
  uint8_t data[1] = {0};
  pb_buffer_arg buff_arg = (pb_buffer_arg)
      { .buf = &data, .buf_len = sizeof(data) };
  sns_physical_sensor_test_event test_event =
     sns_physical_sensor_test_event_init_default;

  test_event.test_passed = test_result;
  test_event.test_type = type;
  test_event.test_data.funcs.encode = &pb_encode_string_cb;
  test_event.test_data.arg = &buff_arg;

  pb_send_event(instance,
                sns_physical_sensor_test_event_fields,
                &test_event,
                sns_get_system_time(),
                SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_EVENT,
                uid);
}

/** See sns_lsm6ds3_hal.h */
void lsm6ds3_run_self_test(sns_sensor_instance *instance)
{
  lsm6ds3_instance_state *state = (lsm6ds3_instance_state*)instance->state->state;
  sns_rc rv = SNS_RC_SUCCESS;
  uint8_t buffer = 0;
  bool who_am_i_success = false;

  rv = lsm6ds3_get_who_am_i(state->scp_service,
                            state->com_port_info.port_handle,
                            &buffer);
  if(rv == SNS_RC_SUCCESS
     &&
     buffer == LSM6DS3_WHOAMI_VALUE)
  {
    who_am_i_success = true;
  }

  if(state->accel_info.test_info.test_client_present)
  {
    if(state->accel_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_COM)
    {
      lsm6ds3_send_self_test_event(instance, &state->accel_info.suid,
                                   who_am_i_success, SNS_PHYSICAL_SENSOR_TEST_TYPE_COM);
    }
    else if(state->accel_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY)
    {
      // Handle factory test. The driver may choose to reject any new
      // streaming/self-test requests when factory test is in progress.

      /** update_fac_cal_in_registry is used to demonstrate a registry write operation.*/
      state->update_fac_cal_in_registry = true;
      /** Using dummy data for registry write demonstration. */
      state->accel_registry_cfg.fac_cal_bias[0] = 0.01;
      state->accel_registry_cfg.fac_cal_bias[1] = 0.01;
      state->accel_registry_cfg.fac_cal_bias[2] = 0.01;
      state->accel_registry_cfg.version ++;
      lsm6ds3_send_self_test_event(instance, &state->accel_info.suid,
                                   true, SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY);
    }
    state->accel_info.test_info.test_client_present = false;
  }
  if(state->gyro_info.test_info.test_client_present)
  {
    if(state->gyro_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_COM)
    {
      lsm6ds3_send_self_test_event(instance, &state->gyro_info.suid,
                                   who_am_i_success, SNS_PHYSICAL_SENSOR_TEST_TYPE_COM);
    }
    else if(state->gyro_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY)
    {
      // Handle factory test. The driver may choose to reject any new
      // streaming/self-test requests when factory test is in progress.
    }
    state->gyro_info.test_info.test_client_present = false;
  }
  if(state->md_info.test_info.test_client_present)
  {
    if(state->md_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_COM)
    {
      lsm6ds3_send_self_test_event(instance, &state->md_info.suid,
                                   who_am_i_success, SNS_PHYSICAL_SENSOR_TEST_TYPE_COM);
    }
    else if(state->md_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY)
    {
      // Handle factory test. The driver may choose to reject any new
      // streaming/self-test requests when factory test is in progress.
    }
    state->md_info.test_info.test_client_present = false;
  }
  if(state->sensor_temp_info.test_info.test_client_present)
  {
    if(state->sensor_temp_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_COM)
    {
      lsm6ds3_send_self_test_event(instance, &state->sensor_temp_info.suid,
                                   who_am_i_success, SNS_PHYSICAL_SENSOR_TEST_TYPE_COM);
    }
    else if(state->sensor_temp_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY)
    {
      // Handle factory test. The driver may choose to reject any new
      // streaming/self-test requests when factory test is in progress.
    }
    state->sensor_temp_info.test_info.test_client_present = false;
  }
}

