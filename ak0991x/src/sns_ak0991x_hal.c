/**
 * @file sns_ak0991x_hal.c
 *
 * Copyright (c) 2016 Qualcomm Technologies, Inc.
 * Copyright (c) 2016 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 **/

#include "sns_rc.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_event_service.h"
#include "sns_mem_util.h"
#include "sns_math_util.h"
#include "sns_service_manager.h"
#include "sns_com_port_types.h"
#include "sns_sync_com_port.h"
#include "sns_types.h"

#include "sns_ak0991x_hal.h"
#include "sns_ak0991x_sensor.h"
#include "sns_ak0991x_sensor_instance.h"

#include "sns_async_com_port.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"

#include "sns_std_sensor.pb.h"

#include "sns_diag_service.h"
#include "sns_diag.pb.h"

/** Need to use ODR table. */
extern const odr_reg_map reg_map_ak0991x[AK0991X_REG_MAP_TABLE_SIZE];

typedef struct log_sensor_state_info{
  pb_ostream_t log_stream;
  void *log;
} log_sensor_state_info;

typedef struct log_sensor_state_state_raw_info{
  log_sensor_state_info log_info;
  uint32_t sample_cnt;
  sns_time last_timestamp;
  float last_data[3];
  sns_std_sensor_sample_status last_sample_status;
  sns_diag_batch_sample_type last_sample_type;
} log_sensor_state_raw_info;

/**
 * Read wrapper for Synch Com Port Service.
 *
 * @param[i] port_handle      port handle
 * @param[i] reg_addr         register address
 * @param[i] buffer           read buffer
 * @param[i] bytes            bytes to read
 * @param[o] xfer_bytes       bytes read
 *
 * @return sns_rc
 */
static sns_rc ak0991x_com_read_wrapper(
   sns_sync_com_port_handle *port_handle,
   uint32_t reg_addr,
   uint8_t *buffer,
   uint32_t bytes,
   uint32_t *xfer_bytes)
{
  sns_port_vector port_vec;
  port_vec.buffer = buffer;
  port_vec.bytes = bytes;
  port_vec.is_write = false;
  port_vec.reg_addr = reg_addr;

  return port_handle->com_port_api->sns_scp_register_rw(port_handle,
                                                        &port_vec,
                                                        1,
                                                        false,
                                                        xfer_bytes);
}

/**
 * Write wrapper for Synch Com Port Service.
 *
 * @param[i] port_handle      port handle
 * @param[i] reg_addr         register address
 * @param[i] buffer           write buffer
 * @param[i] bytes            bytes to write
 * @param[o] xfer_bytes       bytes written
 * @param[i] save_write_time  true to save write transfer time.
 *
 * @return sns_rc
 */
static sns_rc ak0991x_com_write_wrapper(
   sns_sync_com_port_handle *port_handle,
   uint32_t reg_addr,
   uint8_t *buffer,
   uint32_t bytes,
   uint32_t *xfer_bytes,
   bool save_write_time)
{
  sns_port_vector port_vec;
  port_vec.buffer = buffer;
  port_vec.bytes = bytes;
  port_vec.is_write = true;
  port_vec.reg_addr = reg_addr;

  return port_handle->com_port_api->sns_scp_register_rw(port_handle,
                                                        &port_vec,
                                                        1,
                                                        save_write_time,
                                                        xfer_bytes);
}

/**
 * If mask = 0x0 or 0xFF, or if size > 1, write reg_value
 * directly to reg_addr. Else, read value at reg_addr and only
 * modify bits defined by mask.
 *
 * @param[i] port_handle      handle to synch COM port
 * @param[i] reg_addr         reg addr to modify
 * @param[i] reg_value        value to write to register
 * @param[i] size             number of bytes to write
 * @param[o]  xfer_bytes      number of bytes transfered
 * @param[i] save_write_time  save write time input
 * @param[i] mask             bit mask to update
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc ak0991x_read_modify_write(sns_sync_com_port_handle *port_handle,
                            uint32_t reg_addr,
                            uint8_t *reg_value,
                            uint32_t size,
                            uint32_t *xfer_bytes,
                            bool save_write_time,
                            uint8_t mask)
{
  uint8_t rw_buffer = 0;
  uint32_t rw_bytes = 0;

  if((size > 1) || (mask == 0xFF) || (mask == 0x00))
  {
    ak0991x_com_write_wrapper(port_handle,
                          reg_addr,
                          &reg_value[0],
                          size,
                          xfer_bytes,
                          save_write_time);

  }
  else
  {
    // read current value from this register
    ak0991x_com_read_wrapper(port_handle,
                         reg_addr,
                         &rw_buffer,
                         1,
                         &rw_bytes);

    // generate new value
    rw_buffer = (rw_buffer & (~mask)) | (*reg_value & mask);

    // write new value to this register
    ak0991x_com_write_wrapper(port_handle,
                          reg_addr,
                          &rw_buffer,
                          1,
                          xfer_bytes,
                          save_write_time);

  }

  return SNS_RC_SUCCESS;;
}

/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_device_sw_reset(sns_sync_com_port_handle *port_handle)
{
  uint8_t buffer[1];
  sns_rc rv = SNS_RC_SUCCESS;
  sns_time cur_time;
  uint32_t xfer_bytes;

  buffer[0] = 0x01;
  rv = ak0991x_com_write_wrapper(port_handle,
                             AKM_AK0991X_REG_CNTL3,
                             &buffer[0],
                             1,
                             &xfer_bytes,
                             false);
  if(rv != SNS_RC_SUCCESS
     ||
     xfer_bytes != 1)
  {
    return rv;
  }

  cur_time = sns_get_system_time();
  do
  {
    if(sns_get_system_time() > (cur_time + sns_convert_ns_to_ticks(10*1000*1000)))
    {
      // Sensor HW has not recovered from SW reset.
      return SNS_RC_FAILED;
    }
    else
    {
      //1ms wait
      sns_busy_wait(sns_convert_ns_to_ticks(1*1000*1000));

      rv = ak0991x_com_read_wrapper(port_handle,
                                AKM_AK0991X_REG_CNTL3,
                                &buffer[0],
                                1,
                                &xfer_bytes);

      if(rv != SNS_RC_SUCCESS)
      {
        // TODO add debug log.
        // HW not ready. Keep going.
      }
      if(xfer_bytes != 1)
      {
        // TODO add error log.
        return SNS_RC_FAILED;
      }
    }

  } while((buffer[0] & 0x01));

  return SNS_RC_SUCCESS;
}

/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_reset_device(sns_sync_com_port_handle *port_handle,
                            akm_device_type device_select)
{
  UNUSED_VAR(device_select);
  sns_rc rv = SNS_RC_SUCCESS;

  rv = ak0991x_device_sw_reset(port_handle);

  return rv;
}

/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_set_mag_config(sns_sync_com_port_handle *port_handle,
                              ak0991x_mag_odr         curr_odr,
                              akm_device_type         device_select,
                              uint16_t                cur_wmk)
{
   sns_rc rv = SNS_RC_SUCCESS;
   uint32_t xfer_bytes;
   uint8_t buffer;

   // Configure control register 1
  if ((device_select == AK09912) || (device_select == AK09915C) || (device_select == AK09915D)) {
    if (device_select == AK09912) {
      buffer = 0x0
       | (AK0991X_NSF<<5); // NSF bit
    } else {
      buffer = 0x0
       | (AK0991X_NSF<<5)  // NSF bit
       | cur_wmk;          // WM[4:0] bits
    }
    rv = ak0991x_com_write_wrapper(port_handle,
                          AKM_AK0991X_REG_CNTL1,
                          &buffer,
                          1,
                          &xfer_bytes,
                          false);

    if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
    {
       return SNS_RC_FAILED;
    }
  }

  // Configure control register 2
  if((device_select == AK09915C) || (device_select == AK09915D))
  {
    buffer = 0x0
         | (AK0991X_ENABLE_FIFO<<7) // FIFO bit
         | (AK0991X_SDR<<6)         // SDR bit
         | (uint8_t)curr_odr;       // MODE[4:0] bits
  } else {
    buffer = 0x0
         | (uint8_t)curr_odr;       // MODE[4:0] bits
  }

  return ak0991x_com_write_wrapper(port_handle,
                            AKM_AK0991X_REG_CNTL2,
                            &buffer,
                            1,
                            &xfer_bytes,
                            false);
}

/**
 * see sns_ak0991x_hal.h
 */
void ak0991x_start_mag_streaming(ak0991x_instance_state *state)
{
  // Enable Mag Streaming

  state->mag_info.curr_odr =
                  state->mag_info.desired_odr;
  //Transit to Power-down mode first and then transit to other modes.
  ak0991x_set_mag_config(state->com_port_info.port_handle,
                         AK0991X_MAG_ODR_OFF,
                         state->mag_info.device_select,
                         state->mag_info.cur_wmk);
  ak0991x_set_mag_config(state->com_port_info.port_handle,
                         state->mag_info.curr_odr,
                         state->mag_info.device_select,
                         state->mag_info.cur_wmk);

}

/**
 * see sns_ak0991x_hal.h
 */
void ak0991x_stop_mag_streaming(ak0991x_instance_state *state)
{
  // Disable Mag Streaming

  state->mag_info.curr_odr =
                  state->mag_info.desired_odr;
  ak0991x_set_mag_config(state->com_port_info.port_handle,
                         AK0991X_MAG_ODR_OFF,
                         state->mag_info.device_select,
                         state->mag_info.cur_wmk);
}


/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_get_who_am_i(sns_sync_com_port_handle *port_handle,
                            uint8_t *buffer)
{
  sns_rc rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  rv = ak0991x_com_read_wrapper(port_handle,
                            AKM_AK0991X_REG_WIA1,
                            buffer,
                            AK0991X_NUM_READ_DEV_ID,
                            &xfer_bytes);

  if(rv != SNS_RC_SUCCESS
     ||
     xfer_bytes != AK0991X_NUM_READ_DEV_ID)
  {
    rv = SNS_RC_FAILED;
  }

  return rv;
}

//read_fifo_data
sns_rc ak0991x_get_fifo_data(sns_sync_com_port_handle *port_handle,
                            uint8_t *buffer)
{
  sns_rc rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  rv = ak0991x_com_read_wrapper(port_handle,
                            AKM_AK0991X_REG_HXL,
                            buffer,
                            AK0991X_NUM_DATA_HXL_TO_ST2,
                            &xfer_bytes);

  if(rv != SNS_RC_SUCCESS
     ||
     xfer_bytes != AK0991X_NUM_DATA_HXL_TO_ST2)
  {
    rv = SNS_RC_FAILED;
  }

  return rv;
}


/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_set_sstvt_adj(sns_sync_com_port_handle *port_handle,
                            akm_device_type device_select,
                            float *sstvt_adj)
{
  sns_rc rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;
  uint8_t  buffer[3];
  uint8_t  i;

  // If the device does not have FUSE ROM, we don't need to access it. 
  if((device_select == AK09918) ||
     (device_select == AK09916C) ||
     (device_select == AK09916D) ||
     (device_select == AK09915C) ||
     (device_select == AK09915D) ||
     (device_select == AK09913))
  {
    for(i = 0; i < AK0991X_NUM_SENSITIVITY; i++)
    {
      sstvt_adj[i] = 0.15f;
    }
  }
  else if((device_select == AK09912) ||
          (device_select == AK09911))
  {
    // Read Fuse ROM
    rv = ak0991x_com_read_wrapper(port_handle,
                            AKM_AK0991X_FUSE_ASAX,
                            buffer,
                            AK0991X_NUM_SENSITIVITY,
                            &xfer_bytes);

    if(rv != SNS_RC_SUCCESS
       ||
       xfer_bytes != AK0991X_NUM_SENSITIVITY)
    {
      return SNS_RC_FAILED;
    }

    if(device_select == AK09912_WHOAMI_DEV_ID)
    {
      for(i = 0; i < AK0991X_NUM_SENSITIVITY; i++)
      {
        sstvt_adj[i] = ((buffer[i] / 128.0f) + 1.0f) * 0.15f;
      }
    }
    else if(device_select == AK09911_WHOAMI_DEV_ID)
    {
      for(i = 0; i < AK0991X_NUM_SENSITIVITY; i++)
      {
        sstvt_adj[i] = ((buffer[i] / 256.0f) + 0.5f) * 0.6f;
      }
    }
  }
  else
  {
    // No device
    rv = SNS_RC_FAILED;
  }

  return rv;
}

/**
 * Provides sample interval based on current ODR.
 *
 * @param[i] curr_odr              Current FIFO ODR.
 *
 * @return sampling interval time in ticks
 */
sns_time ak0991x_get_sample_interval(ak0991x_mag_odr curr_odr)
{
  int8_t idx;
  sns_time sample_interval = 0;

  for(idx = 0; idx < ARR_SIZE(reg_map_ak0991x); idx++)
  {
    if(curr_odr == reg_map_ak0991x[idx].mag_odr_reg_value
       &&
       curr_odr != AK0991X_MAG_ODR_OFF)
    {
      sample_interval = sns_convert_ns_to_ticks(1000000000 / reg_map_ak0991x[idx].odr);
	  break;
    }
  }

  return sample_interval;
}

/**
 * Allocate Sensor State Log Packet 
 * Used to allocate either Sensor State Raw or Sensor State 
 * Hardware Interrupt Logs 
 *
 * @param[i] diag       Pointer to diag service 
 * @param[i] instance   Pointer to sensor instance
 * @param[i] sensor_uid SUID of the sensor
 * @param[i] log_info   Pointer to logging information 
 *                      pertaining to the sensor
 */
static void ak0991x_log_sensor_state_alloc(
  sns_diag_service *diag,
  sns_sensor_instance *const instance,
  struct sns_sensor_uid const *sensor_uid,
  log_sensor_state_info *log_info)
{
  // allocate memory for sensor state - raw sensor log packet
  log_info->log = diag->api->alloc_log(diag,
                                       instance,
                                       sensor_uid,
                                       diag->api->get_max_log_size(diag));

  if(NULL != log_info->log)
{
    log_info->log_stream = pb_ostream_from_buffer((pb_byte_t*)log_info->log,
                                                 diag->api->get_max_log_size(diag));
  }
}

/**
 * Submit the Sensor State Log Packet 
 * Used to submit either Sensor State Raw or Sensor State 
 * Hardware Interrupt Logs  
 *
 * @param[i] diag               Pointer to diag service
 * @param[i] instance           Pointer to sensor instance
 * @param[i] sensor_uid         SUID of the sensor
 * @param[i] log_info           Pointer to logging information 
 *                              pertaining to the sensor
 * @param[i] sensor_state_type  Type of sensor state information 
 *                              being logged 
 */
static void ak0991x_log_sensor_state_submit(
  sns_diag_service *diag,
  sns_sensor_instance *const instance,
  struct sns_sensor_uid const *sensor_uid,
  log_sensor_state_info *log_info,
  sns_diag_sensor_state_log sensor_state_type)
{
  if(NULL != log_info->log)
  {
    diag->api->submit_log(
      diag,
      instance,
      sensor_uid,
      log_info->log_stream.bytes_written,
      log_info->log,
      sensor_state_type);
  }
}

/**
 * Add raw uncalibrated sensor data to Sensor State Raw log 
 * packet 
 *  
 * @param[i] log_raw_info Pointer to logging information 
 *                        pertaining to the sensor
 * @param[i] raw_data     Uncalibrated sensor data to be logged
 * @param[i] timestamp    Timestamp of the sensor data
 * @param[i] status       Status of the sensor data
 */
static sns_rc ak0991x_log_sensor_state_raw_add(
  log_sensor_state_raw_info *log_raw_info,
  float *raw_data,
  sns_time timestamp,
  sns_std_sensor_sample_status status)
{
  sns_rc rc = SNS_RC_SUCCESS;

  if(NULL == log_raw_info->log_info.log)
  {
    return SNS_RC_NOT_SUPPORTED;
  }

  if(0 < log_raw_info->sample_cnt)
  {
    // FIRST sample
    // We skip logging for the first  sample but store the raw sensor data in
    // log_info. Logged samples will  trail the actual events sent by 1.
    // At the completion of the for loop, there will be exactly one
    // sample pending logging. This allows to determine the batch sample type
    // of the data sent
    sns_diag_batch_sample sample = sns_diag_batch_sample_init_default;
  uint8_t arr_index = 0;
    pb_float_arr_arg arg =
    {.arr = log_raw_info->last_data, .arr_len = 3, .arr_index = &arr_index};

    sample.sample_type = log_raw_info->last_sample_type;
    sample.timestamp = log_raw_info->last_timestamp;

    sample.sample.funcs.encode = &pb_encode_float_arr_cb;
    sample.sample.arg = &arg;

    sample.status = log_raw_info->last_sample_status;

    if(!pb_encode_tag(&log_raw_info->log_info.log_stream,
                      PB_WT_STRING,
                      sns_diag_sensor_state_raw_sample_tag))
    {
      rc = SNS_RC_FAILED;
    }
    else if(!pb_encode_delimited(&log_raw_info->log_info.log_stream,
                                 sns_diag_batch_sample_fields,
                                 &sample))
    {
      rc = SNS_RC_FAILED;
    }
  }

  // Backup last raw sensor data
  log_raw_info->last_timestamp = timestamp;
  log_raw_info->last_sample_status = status;
  sns_memscpy(log_raw_info->last_data,
              sizeof(log_raw_info->last_data),
              raw_data,
              sizeof(log_raw_info->last_data));
  log_raw_info->sample_cnt++;

  if(1 == log_raw_info->sample_cnt)
  {
    log_raw_info->last_sample_type = SNS_DIAG_BATCH_SAMPLE_TYPE_FIRST;
  }
  else if(1 < log_raw_info->sample_cnt)
  {
    log_raw_info->last_sample_type = SNS_DIAG_BATCH_SAMPLE_TYPE_INTERMEDIATE;
  }

  return rc;
}

/**
 * Submit the Sensor State Raw Log Packet
 *
 * @param[i] diag       Pointer to diag service
 * @param[i] instance   Pointer to sensor instance
 * @param[i] sensor_uid SUID of the sensor
 * @param[i] log_info   Pointer to logging information 
 *                      pertaining to the sensor
 */
static void ak0991x_log_sensor_state_raw_submit(
  sns_diag_service *diag,
  sns_sensor_instance *const instance,
  struct sns_sensor_uid const *sensor_uid,
  log_sensor_state_raw_info *log_raw_info)
{
  if(NULL != log_raw_info->log_info.log)
  {
    if(1 == log_raw_info->sample_cnt)
    {
      log_raw_info->last_sample_type = SNS_DIAG_BATCH_SAMPLE_TYPE_ONLY;
    }
    else if(1 < log_raw_info->sample_cnt)
    {
      log_raw_info->last_sample_type = SNS_DIAG_BATCH_SAMPLE_TYPE_LAST;
    }

    ak0991x_log_sensor_state_raw_add(
      log_raw_info,
      log_raw_info->last_data,
      log_raw_info->last_timestamp,
      log_raw_info->last_sample_status);

    ak0991x_log_sensor_state_submit(
      diag,
      instance,
      sensor_uid,
      &log_raw_info->log_info,
      SNS_DIAG_SENSOR_STATE_LOG_RAW);
  }
}

/**
 * Extract a mag sample from a segment of the mag buffer and generate an
 * event.
 *
 * @param[i] fifo_sample       The segment of fifo buffer that has the mag sample
 * @param[i] timestamp         Timestamp to be used for this sample
 * @param[i] instance          The current ak0991x sensor instance
 * @param[i] event_service     Event service for sending out the mag sample
 * @param[i] state             The state of the ak0991x sensor instance
 */
static void ak0991x_handle_mag_sample(uint8_t mag_sample[8],
                               sns_time timestamp,
                               sns_sensor_instance *const instance,
                               sns_event_service *event_service,
                               ak0991x_instance_state *state,
                               log_sensor_state_raw_info *log_mag_state_raw_info)
                               //pb_ostream_t *stream,
                               //bool logging_enabled,
                               //log_data *ldata)
{
  UNUSED_VAR(event_service);
  //UNUSED_VAR(stream);
  //UNUSED_VAR(logging_enabled);
  //UNUSED_VAR(ldata);

  float data[3];
  sns_std_sensor_sample_status status;
 
  data[0] =
     (int16_t)(((mag_sample[1] << 8) & 0xFF00) | mag_sample[0]) * state->mag_info.sstvt_adj[0];
  data[1] =
     (int16_t)(((mag_sample[3] << 8) & 0xFF00) | mag_sample[2]) * state->mag_info.sstvt_adj[1];
  data[2] =
     (int16_t)(((mag_sample[5] << 8) & 0xFF00) | mag_sample[4]) * state->mag_info.sstvt_adj[2];

  // Check magnetic sensor overflow 
  if(mag_sample[7] == AK0991X_HOFL_BIT) {
    status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_LOW;
  } else {
    status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;
  }

  pb_send_sensor_stream_event(instance,
                              &state->mag_info.suid,
                              timestamp,
                              SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                              status,
                              data,
                              ARR_SIZE(data),
                              state->encoded_mag_event_len);

  //debug
  state->m_stream_event[0] = data[0];
  state->m_stream_event[1] = data[1];
  state->m_stream_event[2] = data[2];

  // Log raw uncalibrated sensor data
  ak0991x_log_sensor_state_raw_add(
    log_mag_state_raw_info,
    data,
    timestamp,
    status);
}

void ak0991x_process_mag_data_buffer(sns_port_vector *vector,
                                     void            *user_arg)
{
  sns_sensor_instance *instance = (sns_sensor_instance *)user_arg;
 
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
     sns_service_manager *service_manager =
       instance->cb->get_service_manager(instance);
    sns_event_service *event_service =
       (sns_event_service*)service_manager->get_service(service_manager, SNS_EVENT_SERVICE);
 
  sns_diag_service* diag = state->diag_service;

  if(AKM_AK0991X_REG_ST1 == vector->reg_addr)
  { 
    uint32_t i;

    log_sensor_state_raw_info log_mag_state_raw_info;
    sns_time timestamp;
    uint16_t num_samples = state->mag_info.cur_wmk;

    sns_memzero(&log_mag_state_raw_info, sizeof(log_mag_state_raw_info));
    ak0991x_log_sensor_state_alloc(
      diag,
      instance,
      &state->mag_info.suid,
      &log_mag_state_raw_info.log_info);

    sns_time sample_interval_ticks = ak0991x_get_sample_interval(state->mag_info.curr_odr);

    sns_time interrupt_interval_ticks = (state->interrupt_timestamp - state->pre_timestamp) / (state->mag_info.cur_wmk + 1);

    for(i = 1; i < vector->bytes; i += AK0991X_NUM_DATA_HXL_TO_ST2)
    {
       if(state->this_is_first_data)
      {
        timestamp = state->interrupt_timestamp - (sample_interval_ticks * num_samples);
      }
      else
      {
        timestamp = state->interrupt_timestamp - (interrupt_interval_ticks * num_samples);
      }

      ak0991x_handle_mag_sample( &vector->buffer[i],
                                 timestamp,
                                 instance,
                                 event_service,
                                 state,
                                 &log_mag_state_raw_info);
      num_samples--;
    }

    state->this_is_first_data = false;
    state->pre_timestamp = state->interrupt_timestamp; 

    ak0991x_log_sensor_state_raw_submit(diag,
                                        instance,
                                        &state->mag_info.suid,
                                        &log_mag_state_raw_info);
 
  }
}

void ak0991x_flush_fifo(sns_sensor_instance *const instance)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
     sns_service_manager *service_manager =
       instance->cb->get_service_manager(instance);
    sns_event_service *event_service =
       (sns_event_service*)service_manager->get_service(service_manager, SNS_EVENT_SERVICE);
 
  sns_diag_service* diag = state->diag_service;
  log_sensor_state_raw_info log_mag_state_raw_info;

  sns_memzero(&log_mag_state_raw_info, sizeof(log_mag_state_raw_info));
  ak0991x_log_sensor_state_alloc(
      diag,
      instance,
      &state->mag_info.suid,
      &log_mag_state_raw_info.log_info);

    uint32_t i;

    diag->api->sensor_inst_printf(diag, instance, &state->mag_info.suid, SNS_ERROR, __FILENAME__,__LINE__,__FUNCTION__);

    sns_time timestamp;
    uint16_t num_samples = 0;

    uint8_t buffer[200];

    //Continue reading until fifo buffer is clear
    for(i = 0; i < state->mag_info.max_fifo_size; i++) {
      ak0991x_get_fifo_data(state->com_port_info.port_handle,
                               &buffer[i * AK0991X_NUM_DATA_HXL_TO_ST2]);
 
      if(buffer[i * AK0991X_NUM_DATA_HXL_TO_ST2 + 7] == AK0991X_INV_FIFO_DATA) {
        //fifo buffer is clear
        break;
      } else {
        num_samples++;
      }
 
   }
    
     sns_time sample_interval_ticks = ak0991x_get_sample_interval(state->mag_info.curr_odr);
     sns_time interrupt_interval_ticks;
   
    if(num_samples != 0)
    { 
      interrupt_interval_ticks = (state->interrupt_timestamp - state->pre_timestamp) / (num_samples);
    }
    else
    {
      interrupt_interval_ticks = 0;
    }



    for(i = 0; i < num_samples; i++)
    {
      // flush event trigger is IRQ
      if (state->irq_info.detect_irq_event)
      {
        if (state->this_is_first_data)
        {
          timestamp = state->interrupt_timestamp - (sample_interval_ticks * (state->mag_info.cur_wmk - i));
        }
        else
        {
          timestamp = state->interrupt_timestamp - (interrupt_interval_ticks * (state->mag_info.cur_wmk - i));
        }
      }
      else
      {
        timestamp = state->pre_timestamp + (sample_interval_ticks * (i + 1));
      }

      ak0991x_handle_mag_sample( &buffer[AK0991X_NUM_DATA_HXL_TO_ST2 * i],
                                 timestamp,
                                 instance,
                                 event_service,
                                 state,
                                 &log_mag_state_raw_info);
    }

    if (num_samples != 0)
    {
      if (state->irq_info.detect_irq_event)
      {
        state->pre_timestamp = state->interrupt_timestamp;
      }
      else
      {
        state->pre_timestamp = timestamp;
      }
    }

    state->this_is_first_data = false;
 
    ak0991x_log_sensor_state_raw_submit(diag,
                                        instance,
                                        &state->mag_info.suid,
                                        &log_mag_state_raw_info);
 
}

void ak0991x_handle_interrupt_event(sns_sensor_instance *const instance)
{

  uint8_t buffer[200];
  uint32_t enc_len;
  uint16_t num_of_bytes;

  ak0991x_instance_state *state =
     (ak0991x_instance_state*)instance->state->state;
  sns_port_vector async_read_msg;

  if((AK0991X_ENABLE_FIFO == 1) && ((state->mag_info.device_select == AK09915C) || (state->mag_info.device_select == AK09915D)))
  {
    // Water mark level : 0x0 -> 1step, 0x1F ->32step
    num_of_bytes = AK0991X_NUM_DATA_HXL_TO_ST2 * (state->mag_info.cur_wmk + 1) + 1;
  }
  else
  {
    num_of_bytes = AK0991X_NUM_DATA_ST1_TO_ST2;
  }

  // Compose the async com port message
  async_read_msg.bytes = num_of_bytes;
  async_read_msg.reg_addr = AKM_AK0991X_REG_ST1;
  async_read_msg.is_write = false;
  async_read_msg.buffer = NULL;

  sns_ascp_create_encoded_vectors_buffer(&async_read_msg, 1, true, buffer, sizeof(buffer), &enc_len);

  // Send message to Async Com Port
  sns_request async_com_port_request =
  (sns_request)
  {
    .message_id = SNS_ASYNC_COM_PORT_MSGID_SNS_ASYNC_COM_PORT_VECTOR_RW,
    .request_len = enc_len,
    .request = buffer
  };
  state->async_com_port_data_stream->api->send_request(
    state->async_com_port_data_stream, &async_com_port_request);
}

sns_rc ak0991x_handle_timer_event(sns_sensor_instance *const instance)
{
  ak0991x_instance_state *state =
     (ak0991x_instance_state*)instance->state->state;
  sns_service_manager *service_manager =
       instance->cb->get_service_manager(instance);
  sns_event_service *event_service =
       (sns_event_service*)service_manager->get_service(service_manager, SNS_EVENT_SERVICE);
 
  sns_diag_service* diag = state->diag_service;
  log_sensor_state_raw_info log_mag_state_raw_info;

  sns_memzero(&log_mag_state_raw_info, sizeof(log_mag_state_raw_info));
  ak0991x_log_sensor_state_alloc(
      diag,
      instance,
      &state->mag_info.suid,
      &log_mag_state_raw_info.log_info);

  sns_rc rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;
  uint8_t  buffer[AK0991X_NUM_DATA_ST1_TO_ST2];

  // Read ST1->ST2 at the first time
  rv = ak0991x_com_read_wrapper(state->com_port_info.port_handle,
                            AKM_AK0991X_REG_ST1,
                            &buffer[0],
                            AK0991X_NUM_DATA_ST1_TO_ST2,
                            &xfer_bytes);

  if(rv != SNS_RC_SUCCESS
     ||
     xfer_bytes != AK0991X_NUM_DATA_ST1_TO_ST2)
  {
    return SNS_RC_FAILED;
  }

  diag->api->sensor_inst_printf(diag, instance, &state->mag_info.suid, SNS_ERROR, __FILENAME__,__LINE__,__FUNCTION__);

  sns_time timestamp;

  timestamp = sns_get_system_time();

  ak0991x_handle_mag_sample( &buffer[1],
                                 timestamp,
                                 instance,
                                 event_service,
                                 state,
                                 &log_mag_state_raw_info);

  return SNS_RC_SUCCESS;
}

/** See sns_ak0991x_hal.h */
void ak0991x_send_config_event(sns_sensor_instance *const instance)
{
  ak0991x_instance_state *state =
     (ak0991x_instance_state*)instance->state->state;

  sns_std_sensor_config_event config_event =
    sns_std_sensor_config_event_init_default;

  sns_std_sensor_physical_config phy_sensor_config =
    sns_std_sensor_physical_config_init_default;

  // TODO: Use appropriate op_mode selected by driver.
  char operating_mode[] = "NORMAL";

  pb_buffer_arg op_mode_args;
  pb_buffer_arg payload_args;

  op_mode_args.buf = &operating_mode[0];
  op_mode_args.buf_len = sizeof(operating_mode);

  config_event.has_payload = true;
  config_event.payload.data.funcs.encode = &pb_encode_string_cb;
  config_event.payload.data.arg = &payload_args;

  //TODO_AKM
  phy_sensor_config.has_water_mark = false;//true;
  phy_sensor_config.water_mark = state->mag_info.cur_wmk;
  phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
  phy_sensor_config.operation_mode.arg = &op_mode_args;
  phy_sensor_config.has_active_current = true;
  phy_sensor_config.active_current = 240;
  phy_sensor_config.has_resolution = true;
  phy_sensor_config.resolution = AK09915_RESOLUTION;
  phy_sensor_config.range_count = 2;
  phy_sensor_config.range[0] = AK09915_MIN_RANGE;
  phy_sensor_config.range[1] = AK09915_MAX_RANGE;
  phy_sensor_config.has_stream_is_synchronous = false;
  phy_sensor_config.stream_is_synchronous = false;

  payload_args.buf = &phy_sensor_config;
  payload_args.buf_len = sizeof(phy_sensor_config);

  pb_send_event(instance,
                  sns_std_sensor_config_event_fields,
                  &config_event,
                  sns_get_system_time(),
                  SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG_EVENT,
                  &state->mag_info.suid);
}

