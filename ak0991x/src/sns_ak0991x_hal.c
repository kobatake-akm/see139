/**
 * @file sns_ak0991x_hal.c
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

#include "sns_rc.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_event_service.h"
#include "sns_mem_util.h"
#include "sns_math_util.h"
#include "sns_service_manager.h"
#include "sns_com_port_types.h"
#include "sns_sync_com_port_service.h"
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

typedef struct log_sensor_state_raw_info
{
  /* Size of a single encoded sample */
  size_t encoded_sample_size;
  /* Pointer to log*/
  void *log;
  /* Size of allocated space for log*/
  uint32_t log_size;
  /* Number of actual bytes written*/
  uint32_t bytes_written;
  /* Number of batch samples written*/
  uint32_t sample_cnt;
} log_sensor_state_raw_info;

/**
 * Encode Sensor State Log.Interrupt
 *  
 * @param[i] log Pointer to log packet information
 * @param[i] log_size Size of log packet information
 * @param[i] encoded_log_size Maximum permitted encoded size of 
 *                            the log
 * @param[o] encoded_log Pointer to location where encoded 
 *                       log should be generated
 *  
 * @return sns_rc,
 * SNS_RC_SUCCESS if encoding was succesful 
 * SNS_RC_FAILED otherwise 
 */
sns_rc ak0991x_encode_sensor_state_log_interrupt(
  void *log, size_t log_size, size_t encoded_log_size, void *encoded_log)
{
  UNUSED_VAR(log_size);
  sns_rc rc = SNS_RC_SUCCESS;

  if(NULL == encoded_log || NULL == log)
  {
    return SNS_RC_FAILED;
  }

  sns_diag_sensor_state_interrupt *sensor_state_interrupt =
    (sns_diag_sensor_state_interrupt *)log;
  pb_ostream_t stream = pb_ostream_from_buffer(encoded_log, encoded_log_size);

  if(!pb_encode(&stream, sns_diag_sensor_state_interrupt_fields,
                sensor_state_interrupt))
  {
    rc = SNS_RC_FAILED;
  }

  return rc;
}

/**
 * Encode log sensor state raw packet
 *  
 * @param[i] log Pointer to log packet information
 * @param[i] log_size Size of log packet information
 * @param[i] encoded_log_size Maximum permitted encoded size of 
 *                            the log
 * @param[o] encoded_log Pointer to location where encoded 
 *                       log should be generated
 * @param[o] bytes_written Pointer to actual bytes written 
 *                       during encode 
 *  
 * @return sns_rc 
 * SNS_RC_SUCCESS if encoding was succesful 
 * SNS_RC_FAILED otherwise
 */
sns_rc ak0991x_encode_log_sensor_state_raw(
  void *log, size_t log_size, size_t encoded_log_size, void *encoded_log,
  size_t *bytes_written)
{
  sns_rc rc = SNS_RC_SUCCESS;
  uint32_t i = 0;
  size_t encoded_sample_size = 0;
  size_t parsed_log_size = 0;
  sns_diag_batch_sample sample = sns_diag_batch_sample_init_default;
  sample.sample_count = 3;

  if(NULL == encoded_log || NULL == log || NULL == bytes_written)
  {
    return SNS_RC_FAILED;
  }

  if(!pb_get_encoded_size(&encoded_sample_size, sns_diag_batch_sample_fields,
                          &sample))
  {
    return SNS_RC_FAILED;
  }

  pb_ostream_t stream = pb_ostream_from_buffer(encoded_log, encoded_log_size);
  sns_diag_batch_sample *batch_sample = (sns_diag_batch_sample *)log;

  while(parsed_log_size < log_size &&
        (stream.bytes_written + encoded_sample_size)<= encoded_log_size &&
        i < (uint32_t)(-1))
  {
    if(!pb_encode_tag(&stream, PB_WT_STRING,
                      sns_diag_sensor_state_raw_sample_tag))
    {
      rc = SNS_RC_FAILED;
      break;
    }
    else if(!pb_encode_delimited(&stream, sns_diag_batch_sample_fields,
                                 &batch_sample[i]))
    {
      rc = SNS_RC_FAILED;
      break;
    }

    parsed_log_size += sizeof(sns_diag_batch_sample);
    i++;
  }

  if (SNS_RC_SUCCESS == rc)
  {
    *bytes_written = stream.bytes_written;
  }

  return rc;
}

/**
 * Allocate Sensor State Raw Log Packet 
 *
 * @param[i] diag       Pointer to diag service 
 * @param[i] instance   Pointer to sensor instance
 * @param[i] sensor_uid SUID of the sensor
 * @param[i] log_raw_info   Pointer to raw sensor state logging
 *       information pertaining to the sensor
 */
void ak0991x_log_sensor_state_raw_alloc(
  sns_diag_service *diag,
  sns_sensor_instance *const instance,
  struct sns_sensor_uid const *sensor_uid,
  log_sensor_state_raw_info *log_raw_info)
{
  // allocate memory for sensor state - raw sensor log packet

  log_raw_info->log_size = diag->api->get_max_log_size(diag);

  log_raw_info->log = diag->api->alloc_log(diag,
                                           instance,
                                           sensor_uid,
                                           log_raw_info->log_size,
                                           SNS_DIAG_SENSOR_STATE_LOG_RAW);
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
 *  
 * * @return sns_rc,
 * SNS_RC_SUCCESS if encoding was succesful 
 * SNS_RC_FAILED otherwise  
 */
sns_rc ak0991x_log_sensor_state_raw_add(
  log_sensor_state_raw_info *log_raw_info,
  float *raw_data,
  sns_time timestamp,
  sns_std_sensor_sample_status status)
{
  sns_rc rc = SNS_RC_SUCCESS;

  if(NULL == log_raw_info->log ||
     ((log_raw_info->bytes_written + sizeof(sns_diag_batch_sample)) >
     log_raw_info->log_size))
  {
    rc = SNS_RC_NOT_SUPPORTED;
  }
  else
  {
    sns_diag_batch_sample *sample =
        (sns_diag_batch_sample *)log_raw_info->log;

    if(0 == log_raw_info->sample_cnt)
    {
      sample[log_raw_info->sample_cnt].sample_type =
        SNS_DIAG_BATCH_SAMPLE_TYPE_FIRST;
    }
    else
    {
      sample[log_raw_info->sample_cnt].sample_type =
        SNS_DIAG_BATCH_SAMPLE_TYPE_INTERMEDIATE;
    }

    sample[log_raw_info->sample_cnt].timestamp = timestamp;
    sample[log_raw_info->sample_cnt].sample_count = 3;

    sns_memscpy(sample[log_raw_info->sample_cnt].sample,
                sizeof(sample[log_raw_info->sample_cnt].sample),
                raw_data,
                sizeof(sample[log_raw_info->sample_cnt].sample));

    sample[log_raw_info->sample_cnt].status = status;

    log_raw_info->bytes_written += sizeof(sns_diag_batch_sample);

    log_raw_info->sample_cnt++;
  }

  return rc;
}

/**
 * Submit the Sensor State Raw Log Packet
 *
 * @param[i] diag       Pointer to diag service
 * @param[i] instance   Pointer to sensor instance
 * @param[i] sensor_uid SUID of the sensor
 * @param[i] log_raw_info   Pointer to logging information 
 *                      pertaining to the sensor
 */
void ak0991x_log_sensor_state_raw_submit(
  sns_diag_service *diag,
  sns_sensor_instance *const instance,
  struct sns_sensor_uid const *sensor_uid,
  log_sensor_state_raw_info *log_raw_info)
{
  sns_diag_batch_sample *sample =
      (sns_diag_batch_sample *)log_raw_info->log;

  // overwriting previously sample_type for last sample
  if(1 == log_raw_info->sample_cnt)
  {
    sample[0].sample_type =
      SNS_DIAG_BATCH_SAMPLE_TYPE_ONLY;
  }
  else if(1 < log_raw_info->sample_cnt)
  {
    sample[log_raw_info->sample_cnt - 1].sample_type =
      SNS_DIAG_BATCH_SAMPLE_TYPE_LAST;
  }

  diag->api->submit_log(
        diag,
        instance,
        sensor_uid,
        log_raw_info->bytes_written,
        log_raw_info->log,
        SNS_DIAG_SENSOR_STATE_LOG_RAW,
        log_raw_info->sample_cnt * log_raw_info->encoded_sample_size,
        ak0991x_encode_log_sensor_state_raw);
}

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
static sns_rc ak0991x_com_read_wrapper(sns_sync_com_port_service * scp_service ,
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

  return scp_service->api->sns_scp_register_rw(port_handle,
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
static sns_rc ak0991x_com_write_wrapper(sns_sync_com_port_service * scp_service,
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

  return scp_service->api->sns_scp_register_rw(port_handle,
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
sns_rc ak0991x_read_modify_write(sns_sync_com_port_service * scp_service,
                                 sns_sync_com_port_handle *port_handle,
                                 uint32_t reg_addr,
                                 uint8_t *reg_value,
                                 uint32_t size,
                                 uint32_t *xfer_bytes,
                                 bool save_write_time,
                                 uint8_t mask)
{
  uint8_t  rw_buffer = 0;
  uint32_t rw_bytes = 0;

  if ((size > 1) || (mask == 0xFF) || (mask == 0x00))
  {
    ak0991x_com_write_wrapper(scp_service,
                              port_handle,
                              reg_addr,
                              &reg_value[0],
                              size,
                              xfer_bytes,
                              save_write_time);
  }
  else
  {
    // read current value from this register
    ak0991x_com_read_wrapper(scp_service,
                             port_handle,
                             reg_addr,
                             &rw_buffer,
                             1,
                             &rw_bytes);

    // generate new value
    rw_buffer = (rw_buffer & (~mask)) | (*reg_value & mask);

    // write new value to this register
    ak0991x_com_write_wrapper(scp_service,
                              port_handle,
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
sns_rc ak0991x_device_sw_reset(sns_sync_com_port_service * scp_service,
                               sns_sync_com_port_handle *port_handle)
{
  uint8_t  buffer[1];
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  buffer[0] = AK0991X_SOFT_RESET;
  rv = ak0991x_com_write_wrapper(scp_service,
                                 port_handle,
                                 AKM_AK0991X_REG_CNTL3,
                                 &buffer[0],
                                 1,
                                 &xfer_bytes,
                                 false);

  if (xfer_bytes != 1)
  {
    rv = SNS_RC_FAILED;
  }

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  //1ms wait
  sns_busy_wait(sns_convert_ns_to_ticks(1 * 1000 * 1000));

  return SNS_RC_SUCCESS;
}

/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_set_mag_config(sns_sync_com_port_service *scp_service,
                              sns_sync_com_port_handle *port_handle,
                              ak0991x_mag_odr desired_odr,
                              akm_device_type device_select,
                              uint16_t cur_wmk)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;
  uint8_t  buffer;

  // Configure control register 1
  if ((device_select == AK09912) || (device_select == AK09915C) || (device_select == AK09915D))
  {
    if (device_select == AK09912)
    {
      buffer = 0x0
        | (AK0991X_NSF << 5); // NSF bit
    }
    else
    {
      buffer = 0x0
        | (AK0991X_NSF << 5) // NSF bit
        | cur_wmk;           // WM[4:0] bits
    }

    rv = ak0991x_com_write_wrapper(scp_service,
                                   port_handle,
                                   AKM_AK0991X_REG_CNTL1,
                                   &buffer,
                                   1,
                                   &xfer_bytes,
                                   false);

    if (xfer_bytes != 1)
    {
      rv = SNS_RC_FAILED;
    }

    if (rv != SNS_RC_SUCCESS)
    {
      return rv;
    }
  }

  // Configure control register 2
  if ((device_select == AK09915C) || (device_select == AK09915D))
  {
    buffer = 0x0
      | (AK0991X_ENABLE_FIFO << 7) // FIFO bit
      | (AK0991X_SDR << 6)         // SDR bit
      | (uint8_t)desired_odr;      // MODE[4:0] bits
  }
  else
  {
    buffer = 0x0
      | (uint8_t)desired_odr; // MODE[4:0] bits
  }

  return ak0991x_com_write_wrapper(scp_service,
                                   port_handle,
                                   AKM_AK0991X_REG_CNTL2,
                                   &buffer,
                                   1,
                                   &xfer_bytes,
                                   false);
}

/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_start_mag_streaming(ak0991x_instance_state *state)
{
  sns_rc rv;

  // Enable Mag Streaming

  //Transit to Power-down mode first and then transit to other modes.
  rv = ak0991x_set_mag_config(state->scp_service,
                              state->com_port_info.port_handle,
                              AK0991X_MAG_ODR_OFF,
                              state->mag_info.device_select,
                              state->mag_info.cur_wmk);

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  rv = ak0991x_set_mag_config(state->scp_service,
                              state->com_port_info.port_handle,
                              state->mag_info.desired_odr,
                              state->mag_info.device_select,
                              state->mag_info.cur_wmk);

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  state->mag_info.curr_odr = state->mag_info.desired_odr;

  return SNS_RC_SUCCESS;
}

/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_stop_mag_streaming(ak0991x_instance_state *state)
{
  sns_rc rv;

  // Disable Mag Streaming

  rv = ak0991x_set_mag_config(state->scp_service,
                              state->com_port_info.port_handle,
                              AK0991X_MAG_ODR_OFF,
                              state->mag_info.device_select,
                              state->mag_info.cur_wmk);

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  state->mag_info.curr_odr = state->mag_info.desired_odr;

  return SNS_RC_SUCCESS;
}

/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_get_who_am_i(sns_sync_com_port_service *scp_service,
                            sns_sync_com_port_handle *port_handle,
                            uint8_t *buffer)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  rv = ak0991x_com_read_wrapper(scp_service,
                                port_handle,
                                AKM_AK0991X_REG_WIA1,
                                buffer,
                                AK0991X_NUM_READ_DEV_ID,
                                &xfer_bytes);

  if (xfer_bytes != AK0991X_NUM_READ_DEV_ID)
  {
    rv = SNS_RC_FAILED;
  }

  return rv;
}

/**
 * Read asa value.
 *
 * @param[i] port_handle              handle to synch COM port
 * @param[o] buffer                   fifo data
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
static sns_rc ak0991x_read_asa(sns_sync_com_port_service * scp_service,
                               sns_sync_com_port_handle *port_handle,
                               uint8_t *asa)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint8_t  buffer[1];
  uint32_t xfer_bytes;

  buffer[0] = AK0991X_MAG_FUSEROM;
  // Set Fuse ROM access mode
  rv = ak0991x_com_write_wrapper(scp_service,
                                 port_handle,
                                 AKM_AK0991X_REG_CNTL2,
                                 buffer,
                                 1,
                                 &xfer_bytes,
                                 false);

  if (xfer_bytes != 1)
  {
    rv = SNS_RC_FAILED;
  }

  if (rv != SNS_RC_SUCCESS)
  {
    // transit to power-down mode from Fuse ROM access mode if possible
    buffer[0] = AK0991X_MAG_ODR_OFF;
    ak0991x_com_write_wrapper(scp_service,
                              port_handle,
                              AKM_AK0991X_REG_CNTL2,
                              buffer,
                              1,
                              &xfer_bytes,
                              false);
    return rv;
  }


  // Read Fuse ROM
  rv = ak0991x_com_read_wrapper(scp_service,
                                port_handle,
                                AKM_AK0991X_FUSE_ASAX,
                                &asa[0],
                                AK0991X_NUM_SENSITIVITY,
                                &xfer_bytes);

  if (xfer_bytes != AK0991X_NUM_SENSITIVITY)
  {
    rv = SNS_RC_FAILED;
  }

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  buffer[0] = AK0991X_MAG_ODR_OFF;
  // Set power-down mode
  rv = ak0991x_com_write_wrapper(scp_service,
                                 port_handle,
                                 AKM_AK0991X_REG_CNTL2,
                                 buffer,
                                 1,
                                 &xfer_bytes,
                                 false);

  if (xfer_bytes != 1)
  {
    rv = SNS_RC_FAILED;
  }

  return rv;
}

/**
 * check threshold.
 *
 * @param[i] testno                   test number
 * @param[i] testdata                 test data
 * @param[i] lolimit                  low limit
 * @param[i] hilimit                  high limit
 * @param[o] err                      error code
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc ak0991x_test_threshold(uint16_t testno,
                              int16_t testdata,
                              int16_t lolimit,
                              int16_t hilimit,
                              uint32_t *err)
{
  if ((lolimit <= testdata) && (testdata <= hilimit))
  {
    return SNS_RC_SUCCESS;
  }
  else
  {
    *err = (uint32_t)((((uint32_t)testno) << 16) | ((uint16_t)testdata));
    return SNS_RC_FAILED;
  }
}

#define AKM_FST(no, data, lo, hi, err) \
  if (ak0991x_test_threshold((no), (data), (lo), (hi), (err)) \
      != SNS_RC_SUCCESS) { goto TEST_SEQUENCE_FAILED; }
/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_self_test(sns_sync_com_port_service * scp_service,
                         sns_sync_com_port_handle *port_handle,
                         akm_device_type device_select,
                         float *sstvt_adj,
                         uint32_t *err)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;
  sns_time usec_time_for_measure;
  uint8_t  asa[AK0991X_NUM_SENSITIVITY];
  uint8_t  buffer[AK0991X_NUM_DATA_ST1_TO_ST2];
  int16_t  data[3];

  // Initialize error code
  *err = 0;

  // Reset device
  rv = ak0991x_device_sw_reset(scp_service,port_handle);

  if (rv != SNS_RC_SUCCESS)
  {
    *err = ((TLIMIT_NO_RESET) << 16);
    goto TEST_SEQUENCE_FAILED;
  }

  /** Step 1
   *   If the device has FUSE ROM, test the sensitivity value
   **/
  if ((device_select == AK09911) || (device_select == AK09912))
  {
    rv = ak0991x_read_asa(scp_service,port_handle, asa);

    if (rv != SNS_RC_SUCCESS)
    {
      *err = ((TLIMIT_NO_READ_ASA) << 16);
      goto TEST_SEQUENCE_FAILED;
    }

    AKM_FST(TLIMIT_NO_ASAX, asa[0], TLIMIT_LO_ASAX, TLIMIT_HI_ASAX, err);
    AKM_FST(TLIMIT_NO_ASAY, asa[1], TLIMIT_LO_ASAY, TLIMIT_HI_ASAY, err);
    AKM_FST(TLIMIT_NO_ASAZ, asa[2], TLIMIT_LO_ASAZ, TLIMIT_HI_ASAZ, err);
  }

  /** Step 2
   *   Start self test
   **/
  buffer[0] = AK0991X_MAG_SELFTEST;
  rv = ak0991x_com_write_wrapper(scp_service,
                                 port_handle,
                                 AKM_AK0991X_REG_CNTL2,
                                 buffer,
                                 1,
                                 &xfer_bytes,
                                 false);

  if (rv != SNS_RC_SUCCESS
      ||
      xfer_bytes != 1)
  {
    *err = ((TLIMIT_NO_SET_SELFTEST) << 16);
    goto TEST_SEQUENCE_FAILED;
  }


  if (device_select == AK09918)
  {
    usec_time_for_measure = AK09918_TIME_FOR_MEASURE_US;
  }
  else if ((device_select == AK09916C) || (device_select == AK09916D))
  {
    usec_time_for_measure = AK09916_TIME_FOR_MEASURE_US;
  }
  else if ((device_select == AK09915C) || (device_select == AK09915D))
  {
    if (AK0991X_SDR == 1)
    {
      usec_time_for_measure = AK09915_TIME_FOR_LOW_NOISE_MODE_MEASURE_US;
    }
    else
    {
      usec_time_for_measure = AK09915_TIME_FOR_LOW_POWER_MODE_MEASURE_US;
    }
  }
  else if (device_select == AK09913)
  {
    usec_time_for_measure = AK09913_TIME_FOR_MEASURE_US;
  }
  else if (device_select == AK09912)
  {
    usec_time_for_measure = AK09912_TIME_FOR_MEASURE_US;
  }
  else if (device_select == AK09911)
  {
    usec_time_for_measure = AK09911_TIME_FOR_MEASURE_US;
  }
  else
  {
    *err = (TLIMIT_NO_INVALID_ID << 16);
    goto TEST_SEQUENCE_FAILED;
  }

  // To ensure that measurement is finished, wait for double as typical
  sns_busy_wait(sns_convert_ns_to_ticks(usec_time_for_measure * 1000 * 2));

  /** Step 3
   *   Read and check data
   **/
  rv = ak0991x_com_read_wrapper(scp_service,
                                port_handle,
                                AKM_AK0991X_REG_ST1,
                                buffer,
                                AK0991X_NUM_DATA_ST1_TO_ST2,
                                &xfer_bytes);

  if (rv != SNS_RC_SUCCESS
      ||
      xfer_bytes != AK0991X_NUM_DATA_ST1_TO_ST2)
  {
    *err = ((TLIMIT_NO_READ_DATA) << 16);
    goto TEST_SEQUENCE_FAILED;
  }

  // raw data in 16 bits
  data[0] = (int16_t)(((buffer[2] << 8) & 0xFF00) | buffer[1]);
  data[1] = (int16_t)(((buffer[4] << 8) & 0xFF00) | buffer[3]);
  data[2] = (int16_t)(((buffer[6] << 8) & 0xFF00) | buffer[5]);
  // adjust sensitivity
  data[0] = (int16_t)(data[0] * sstvt_adj[0]);
  data[1] = (int16_t)(data[1] * sstvt_adj[1]);
  data[2] = (int16_t)(data[2] * sstvt_adj[2]);

  // check read value
  if (device_select == AK09918)
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09918, TLIMIT_HI_SLF_RVHX_AK09918,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09918, TLIMIT_HI_SLF_RVHY_AK09918,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09918, TLIMIT_HI_SLF_RVHZ_AK09918,
            err);
  }
  else if ((device_select == AK09916C) || (device_select == AK09916D))
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09916, TLIMIT_HI_SLF_RVHX_AK09916,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09916, TLIMIT_HI_SLF_RVHY_AK09916,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09916, TLIMIT_HI_SLF_RVHZ_AK09916,
            err);
  }
  else if ((device_select == AK09915C) || (device_select == AK09915D))
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09915, TLIMIT_HI_SLF_RVHX_AK09915,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09915, TLIMIT_HI_SLF_RVHY_AK09915,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09915, TLIMIT_HI_SLF_RVHZ_AK09915,
            err);
  }
  else if (device_select == AK09913)
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09913, TLIMIT_HI_SLF_RVHX_AK09913,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09913, TLIMIT_HI_SLF_RVHY_AK09913,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09913, TLIMIT_HI_SLF_RVHZ_AK09913,
            err);
  }
  else if (device_select == AK09912)
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09912, TLIMIT_HI_SLF_RVHX_AK09912,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09912, TLIMIT_HI_SLF_RVHY_AK09912,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09912, TLIMIT_HI_SLF_RVHZ_AK09912,
            err);
  }
  else if (device_select == AK09911)
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09911, TLIMIT_HI_SLF_RVHX_AK09911,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09911, TLIMIT_HI_SLF_RVHY_AK09911,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09911, TLIMIT_HI_SLF_RVHZ_AK09911,
            err);
  }
  else
  {
    *err = (TLIMIT_NO_INVALID_ID << 16);
    goto TEST_SEQUENCE_FAILED;
  }

  AKM_FST(TLIMIT_NO_SLF_ST2, (buffer[8] & TLIMIT_ST2_MASK),
          TLIMIT_LO_SLF_ST2, TLIMIT_HI_SLF_ST2, err);

TEST_SEQUENCE_FAILED:

  if (*err == 0)
  {
    return SNS_RC_SUCCESS;
  }
  else
  {
    return SNS_RC_FAILED;
  }
}

/**
 * Get fifo data.
 *
 * @param[i] port_handle              handle to synch COM port
 * @param[o] buffer                   fifo data
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
static sns_rc ak0991x_get_fifo_data(ak0991x_instance_state *state,
                                    sns_sync_com_port_handle *port_handle,
                                    uint8_t *buffer)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  rv = ak0991x_com_read_wrapper(state->scp_service,
                                port_handle,
                                AKM_AK0991X_REG_HXL,
                                buffer,
                                AK0991X_NUM_DATA_HXL_TO_ST2,
                                &xfer_bytes);

  if (xfer_bytes != AK0991X_NUM_DATA_HXL_TO_ST2)
  {
    rv = SNS_RC_FAILED;
  }

  return rv;
}

/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_set_sstvt_adj(sns_sync_com_port_service* scp_service,
                             sns_sync_com_port_handle *port_handle,
                             akm_device_type device_select,
                             float *sstvt_adj)
{
  sns_rc  rv = SNS_RC_SUCCESS;
  uint8_t buffer[AK0991X_NUM_SENSITIVITY];
  uint8_t i;

  // If the device does not have FUSE ROM, we don't need to access it.
  if ((device_select != AK09911) && (device_select != AK09912))
  {
    for (i = 0; i < AK0991X_NUM_SENSITIVITY; i++)
    {
      sstvt_adj[i] = 1.0f;
    }

    return rv;
  }

  rv = ak0991x_read_asa(scp_service,port_handle, buffer);

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  if (device_select == AK09911)
  {
    for (i = 0; i < AK0991X_NUM_SENSITIVITY; i++)
    {
      sstvt_adj[i] = ((buffer[i] / 128.0f) + 1.0f);
    }
  }
  else if (device_select == AK09912)
  {
    for (i = 0; i < AK0991X_NUM_SENSITIVITY; i++)
    {
      sstvt_adj[i] = ((buffer[i] / 256.0f) + 0.5f);
    }
  }
  else
  {
    // no device
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
  int8_t   idx;
  sns_time sample_interval = 0;

  for (idx = 0; idx < ARR_SIZE(reg_map_ak0991x); idx++)
  {
    if (curr_odr == reg_map_ak0991x[idx].mag_odr_reg_value
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
{
  UNUSED_VAR(event_service);

  float                        data[3];
  sns_std_sensor_sample_status status;

  data[0] =
    (int16_t)(((mag_sample[1] << 8) & 0xFF00) | mag_sample[0]) * state->mag_info.sstvt_adj[0] *
    state->mag_info.resolution;
  data[1] =
    (int16_t)(((mag_sample[3] << 8) & 0xFF00) | mag_sample[2]) * state->mag_info.sstvt_adj[1] *
    state->mag_info.resolution;
  data[2] =
    (int16_t)(((mag_sample[5] << 8) & 0xFF00) | mag_sample[4]) * state->mag_info.sstvt_adj[2] *
    state->mag_info.resolution;

  // Check magnetic sensor overflow
  if ((mag_sample[7] & AK0991X_HOFL_BIT) != 0)
  {
    status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
  }
  else
  {
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
                                     void *user_arg)
{
  sns_sensor_instance *instance = (sns_sensor_instance *)user_arg;

  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
  sns_service_manager    *service_manager =
    instance->cb->get_service_manager(instance);
  sns_event_service *event_service =
    (sns_event_service *)service_manager->get_service(service_manager, SNS_EVENT_SERVICE);

  sns_diag_service *diag = state->diag_service;

  if (AKM_AK0991X_REG_ST1 == vector->reg_addr)
  {
    uint32_t i;

    log_sensor_state_raw_info log_mag_state_raw_info;
    sns_time                  timestamp;
    uint16_t                  cnt_for_ts = state->mag_info.cur_wmk;

    sns_memzero(&log_mag_state_raw_info, sizeof(log_mag_state_raw_info));
    log_mag_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;

    ak0991x_log_sensor_state_raw_alloc(
      diag,
      instance,
      &state->mag_info.suid,
      &log_mag_state_raw_info);

    sns_time sample_interval_ticks = ak0991x_get_sample_interval(state->mag_info.curr_odr);

    sns_time interrupt_interval_ticks = (state->interrupt_timestamp - state->pre_timestamp) /
      (state->mag_info.cur_wmk + 1);

    for (i = 1; i < vector->bytes; i += AK0991X_NUM_DATA_HXL_TO_ST2)
    {
      if (state->this_is_first_data)
      {
        timestamp = state->interrupt_timestamp - (sample_interval_ticks * cnt_for_ts);
      }
      else
      {
        timestamp = state->interrupt_timestamp - (interrupt_interval_ticks * cnt_for_ts);
      }

      ak0991x_handle_mag_sample(&vector->buffer[i],
                                timestamp,
                                instance,
                                event_service,
                                state,
                                &log_mag_state_raw_info);
      cnt_for_ts--;
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
  sns_service_manager    *service_manager =
    instance->cb->get_service_manager(instance);
  sns_event_service *event_service =
    (sns_event_service *)service_manager->get_service(service_manager, SNS_EVENT_SERVICE);

  sns_diag_service          *diag = state->diag_service;
  log_sensor_state_raw_info log_mag_state_raw_info;

  sns_memzero(&log_mag_state_raw_info, sizeof(log_mag_state_raw_info));
  log_mag_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;

  ak0991x_log_sensor_state_raw_alloc(
    diag,
    instance,
    &state->mag_info.suid,
    &log_mag_state_raw_info);

  uint32_t i;

  sns_time timestamp;
  uint16_t num_samples = 0;

  uint8_t buffer[AK0991X_MAX_FIFO_SIZE];

  //Continue reading until fifo buffer is clear
  for (i = 0; i < state->mag_info.max_fifo_size; i++)
  {
    ak0991x_get_fifo_data(state,state->com_port_info.port_handle,
                          &buffer[i * AK0991X_NUM_DATA_HXL_TO_ST2]);

    if ((buffer[i * AK0991X_NUM_DATA_HXL_TO_ST2 + 7] & AK0991X_INV_FIFO_DATA) != 0)
    {
      //fifo buffer is clear
      break;
    }
    else
    {
      num_samples++;
    }
  }

  sns_time sample_interval_ticks = ak0991x_get_sample_interval(state->mag_info.curr_odr);
  sns_time interrupt_interval_ticks;

  if (num_samples != 0)
  {
    interrupt_interval_ticks = (state->interrupt_timestamp - state->pre_timestamp) / (num_samples);
  }
  else
  {
    interrupt_interval_ticks = 0;
  }



  for (i = 0; i < num_samples; i++)
  {
    // flush event trigger is IRQ
    if (state->irq_info.detect_irq_event)
    {
      if (state->this_is_first_data)
      {
        timestamp = state->interrupt_timestamp -
          (sample_interval_ticks * (state->mag_info.cur_wmk - i));
      }
      else
      {
        timestamp = state->interrupt_timestamp -
          (interrupt_interval_ticks * (state->mag_info.cur_wmk - i));
      }
    }
    else
    {
      timestamp = state->pre_timestamp + (sample_interval_ticks * (i + 1));
    }

    ak0991x_handle_mag_sample(&buffer[AK0991X_NUM_DATA_HXL_TO_ST2 * i],
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

    state->this_is_first_data = false;

    ak0991x_log_sensor_state_raw_submit(diag,
                                        instance,
                                        &state->mag_info.suid,
                                        &log_mag_state_raw_info);
  }
}

void ak0991x_handle_interrupt_event(sns_sensor_instance *const instance)
{
  uint8_t  buffer[AK0991X_MAX_FIFO_SIZE];
  uint32_t enc_len;
  uint16_t num_of_bytes;

  ak0991x_instance_state *state =
    (ak0991x_instance_state *)instance->state->state;

  // QC: what's this for?
  instance->cb->get_service_manager(instance);

  sns_port_vector async_read_msg;

  if (state->mag_info.use_fifo)
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

  sns_ascp_create_encoded_vectors_buffer(&async_read_msg, 1, true, buffer, sizeof(buffer),
                                         &enc_len);

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
    (ak0991x_instance_state *)instance->state->state;
  sns_service_manager *service_manager =
    instance->cb->get_service_manager(instance);
  sns_event_service *event_service =
    (sns_event_service *)service_manager->get_service(service_manager, SNS_EVENT_SERVICE);

  sns_diag_service          *diag = state->diag_service;
  log_sensor_state_raw_info log_mag_state_raw_info;

  sns_memzero(&log_mag_state_raw_info, sizeof(log_mag_state_raw_info));
  log_mag_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;

  ak0991x_log_sensor_state_raw_alloc(
    diag,
    instance,
    &state->mag_info.suid,
    &log_mag_state_raw_info);

  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;
  uint8_t  buffer[AK0991X_NUM_DATA_ST1_TO_ST2];

  sns_time timestamp;
  timestamp = sns_get_system_time();

  // Read register ST1->ST2
  rv = ak0991x_com_read_wrapper(state->scp_service,
                                state->com_port_info.port_handle,
                                AKM_AK0991X_REG_ST1,
                                &buffer[0],
                                AK0991X_NUM_DATA_ST1_TO_ST2,
                                &xfer_bytes);

  if (xfer_bytes != AK0991X_NUM_DATA_ST1_TO_ST2)
  {
    rv = SNS_RC_FAILED;
  }

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  ak0991x_handle_mag_sample(&buffer[1],
                            timestamp,
                            instance,
                            event_service,
                            state,
                            &log_mag_state_raw_info);

  return SNS_RC_SUCCESS;
}

/** See sns_ak0991x_hal.h */
sns_rc ak0991x_send_config_event(sns_sensor_instance *const instance)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)instance->state->state;

  sns_std_sensor_physical_config_event phy_sensor_config =
    sns_std_sensor_physical_config_event_init_default;

  // TODO: Use appropriate op_mode selected by driver.
  char *operating_mode;

  switch (state->mag_info.device_select)
  {
  case AK09911:
    operating_mode = AK0991X_NORMAL;
    phy_sensor_config.has_water_mark = false;
    phy_sensor_config.water_mark = state->mag_info.cur_wmk + 1;
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = AK09911_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = AK09911_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = AK09911_MIN_RANGE;
    phy_sensor_config.range[1] = AK09911_MAX_RANGE;
    phy_sensor_config.has_stream_is_synchronous = false;
    phy_sensor_config.stream_is_synchronous = false;
    break;

  case AK09912:
    operating_mode = AK0991X_NORMAL;
    phy_sensor_config.has_water_mark = false;
    phy_sensor_config.water_mark = state->mag_info.cur_wmk + 1;
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = AK09912_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = AK09912_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = AK09912_MIN_RANGE;
    phy_sensor_config.range[1] = AK09912_MAX_RANGE;
    phy_sensor_config.has_stream_is_synchronous = false;
    phy_sensor_config.stream_is_synchronous = false;
    break;

  case AK09913:
    operating_mode = AK0991X_NORMAL;
    phy_sensor_config.has_water_mark = false;
    phy_sensor_config.water_mark = state->mag_info.cur_wmk + 1;
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = AK09913_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = AK09913_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = AK09913_MIN_RANGE;
    phy_sensor_config.range[1] = AK09913_MAX_RANGE;
    phy_sensor_config.has_stream_is_synchronous = false;
    phy_sensor_config.stream_is_synchronous = false;
    break;

  case AK09915C:

    if (AK0991X_SDR == 1)
    {
      operating_mode = AK0991X_LOW_NOISE;
    }
    else
    {
      operating_mode = AK0991X_LOW_POWER;
    }

    phy_sensor_config.has_water_mark = true;
    phy_sensor_config.water_mark = state->mag_info.cur_wmk + 1;
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = AK09915_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = AK09915_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = AK09915_MIN_RANGE;
    phy_sensor_config.range[1] = AK09915_MAX_RANGE;
    phy_sensor_config.has_stream_is_synchronous = false;
    phy_sensor_config.stream_is_synchronous = false;
    break;

  case AK09915D:

    if (AK0991X_SDR == 1)
    {
      operating_mode = AK0991X_LOW_NOISE;
    }
    else
    {
      operating_mode = AK0991X_LOW_POWER;
    }

    phy_sensor_config.has_water_mark = true;
    phy_sensor_config.water_mark = state->mag_info.cur_wmk + 1;
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = AK09915_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = AK09915_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = AK09915_MIN_RANGE;
    phy_sensor_config.range[1] = AK09915_MAX_RANGE;
    phy_sensor_config.has_stream_is_synchronous = false;
    phy_sensor_config.stream_is_synchronous = false;
    break;

  case AK09916C:
    operating_mode = AK0991X_NORMAL;
    phy_sensor_config.has_water_mark = false;
    phy_sensor_config.water_mark = state->mag_info.cur_wmk + 1;
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = AK09916_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = AK09916_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = AK09916_MIN_RANGE;
    phy_sensor_config.range[1] = AK09916_MAX_RANGE;
    phy_sensor_config.has_stream_is_synchronous = false;
    phy_sensor_config.stream_is_synchronous = false;
    break;

  case AK09916D:
    operating_mode = AK0991X_NORMAL;
    phy_sensor_config.has_water_mark = false;
    phy_sensor_config.water_mark = state->mag_info.cur_wmk + 1;
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = AK09916_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = AK09916_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = AK09916_MIN_RANGE;
    phy_sensor_config.range[1] = AK09916_MAX_RANGE;
    phy_sensor_config.has_stream_is_synchronous = false;
    phy_sensor_config.stream_is_synchronous = false;
    break;

  case AK09918:
    operating_mode = AK0991X_NORMAL;
    phy_sensor_config.has_water_mark = false;
    phy_sensor_config.water_mark = state->mag_info.cur_wmk + 1;
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = AK09916_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = AK09916_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = AK09916_MIN_RANGE;
    phy_sensor_config.range[1] = AK09916_MAX_RANGE;
    phy_sensor_config.has_stream_is_synchronous = false;
    phy_sensor_config.stream_is_synchronous = false;
    break;

  default:
    return SNS_RC_FAILED;
  }

  pb_buffer_arg op_mode_args;

  op_mode_args.buf = operating_mode;
  op_mode_args.buf_len = sizeof(operating_mode);

  phy_sensor_config.sample_rate = state->mag_req.sample_rate;

  pb_send_event(instance,
                sns_std_sensor_physical_config_event_fields,
                &phy_sensor_config,
                sns_get_system_time(),
                SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                &state->mag_info.suid);

  return SNS_RC_SUCCESS;
}
