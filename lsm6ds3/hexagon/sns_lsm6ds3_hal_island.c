/**
 * @file sns_lsm6ds3_hal.c
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
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

#include "sns_lsm6ds3_hal.h"
#include "sns_lsm6ds3_sensor.h"
#include "sns_lsm6ds3_sensor_instance.h"

#include "sns_async_com_port.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"

#include "sns_std_sensor.pb.h"

#include "sns_diag_service.h"
#include "sns_diag.pb.h"
#include "sns_timer.pb.h"
#include "sns_std_event_gated_sensor.pb.h"

/** Need to use ODR table. */
extern const odr_reg_map reg_map[LSM6DS3_REG_MAP_TABLE_SIZE];

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
 * @param[o] bytes_written Pointer to actual bytes written
 *       during encode
 *
 * @return sns_rc,
 * SNS_RC_SUCCESS if encoding was succesful
 * SNS_RC_FAILED otherwise
 */
sns_rc lsm6ds3_encode_sensor_state_log_interrupt(
  void *log, size_t log_size, size_t encoded_log_size, void *encoded_log,
  size_t *bytes_written)
{
  UNUSED_VAR(log_size);
  sns_rc rc = SNS_RC_SUCCESS;

  if(NULL == encoded_log || NULL == log || NULL ==bytes_written)
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

  if (SNS_RC_SUCCESS == rc)
  {
    *bytes_written = stream.bytes_written;
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
 *       during encode
 *
 * @return sns_rc
 * SNS_RC_SUCCESS if encoding was succesful
 * SNS_RC_FAILED otherwise
 */
sns_rc lsm6ds3_encode_log_sensor_state_raw(
  void *log, size_t log_size, size_t encoded_log_size, void *encoded_log,
  size_t *bytes_written)
{
  sns_rc rc = SNS_RC_SUCCESS;
  uint32_t i = 0;
  size_t encoded_sample_size = 0;
  size_t parsed_log_size = 0;
  sns_diag_batch_sample sample = sns_diag_batch_sample_init_default;
  sample.sample_count = 3;

  if(NULL == encoded_log || NULL == log || NULL ==bytes_written)
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
void lsm6ds3_log_sensor_state_raw_alloc(
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
sns_rc lsm6ds3_log_sensor_state_raw_add(
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
void lsm6ds3_log_sensor_state_raw_submit(
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
        lsm6ds3_encode_log_sensor_state_raw);
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
static sns_rc lsm6ds3_com_read_wrapper(
   sns_sync_com_port_service *scp_service,
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
static sns_rc lsm6ds3_com_write_wrapper(
   sns_sync_com_port_service *scp_service,
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
sns_rc lsm6ds3_read_modify_write(
                            sns_sync_com_port_service *scp_service,
                            sns_sync_com_port_handle *port_handle,
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
    lsm6ds3_com_write_wrapper(scp_service,
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
    lsm6ds3_com_read_wrapper(scp_service,
                         port_handle,
                         reg_addr,
                         &rw_buffer,
                         1,
                         &rw_bytes);

    // generate new value
    rw_buffer = (rw_buffer & (~mask)) | (*reg_value & mask);

    // write new value to this register
    lsm6ds3_com_write_wrapper(scp_service,
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
 * see sns_lsm6ds3_hal.h
 */
sns_rc lsm6ds3_device_sw_reset(sns_sync_com_port_service *scp_service,
                               sns_sync_com_port_handle *port_handle,
                               lsm6ds3_sensor_type sensor)
{
  UNUSED_VAR(sensor);
  uint8_t buffer[1];
  sns_rc rv = SNS_RC_SUCCESS;
  sns_time cur_time;
  uint32_t xfer_bytes;

  buffer[0] = 0x01;
  rv = lsm6ds3_com_write_wrapper(
                             scp_service,
                             port_handle,
                             STM_LSM6DS3_REG_CTRL3,
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

      rv = lsm6ds3_com_read_wrapper(scp_service,port_handle,
                                STM_LSM6DS3_REG_CTRL3,
                                &buffer[0],
                                1,
                                &xfer_bytes);

      if(rv != SNS_RC_SUCCESS)
      {
        // HW not ready. Keep going.
      }
      if(xfer_bytes != 1)
      {
        return SNS_RC_FAILED;
      }
    }

  } while((buffer[0] & 0x01));

  return SNS_RC_SUCCESS;
}

/**
 * see sns_lsm6ds3_hal.h
 */
sns_rc lsm6ds3_device_set_default_state(sns_sync_com_port_service * scp_service,
                                        sns_sync_com_port_handle *port_handle,
                                        lsm6ds3_sensor_type sensor)
{
  uint8_t buffer[1];
  sns_rc rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  if(sensor == LSM6DS3_ACCEL)
  {
    // reset Accel state only
  }
  else if(sensor == LSM6DS3_GYRO)
  {
    // reset Gyro state only
  }
  else if(sensor == (LSM6DS3_ACCEL | LSM6DS3_GYRO | LSM6DS3_MOTION_DETECT | LSM6DS3_SENSOR_TEMP))
  {
    //async read TODO
    //if(state->com_port_info.com_config.bus_type == SNS_BUS_SPI)
    if(1)
    {
      buffer[0] = 0x30;
      rv = lsm6ds3_read_modify_write(scp_service,
                             port_handle,
                             STM_LSM6DS3_REG_FIFO_CTRL2,
                             &buffer[0],
                             1,
                             &xfer_bytes,
                             false,
                             0x30);

      if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
      {
         return SNS_RC_FAILED;
      }
    }

    // Configure control register 3
    buffer[0] = 0x0
      | (0<<7)           // BOOT bit
      | (0<<6)           // BDU bit
      | (0<<5)           // H_LACTIVE bit
      | (0<<4)           // PP_OD bit
      | (0<<3)           // SIM bit
      | (1<<2)           // IF_INC
      | (0<<1)           // BLE
      | 0;               // SW_RESET

    rv = lsm6ds3_read_modify_write(scp_service,
                                   port_handle,
                                   STM_LSM6DS3_REG_CTRL3,
                                   &buffer[0],
                                   1,
                                   &xfer_bytes,
                                   false,
                                   0xFF);

    if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
    {
       return SNS_RC_FAILED;
    }

    rv = lsm6ds3_set_accel_config(scp_service,
                                  port_handle,
                                  LSM6DS3_ACCEL_ODR_OFF,
                                  LSM6DS3_ACCEL_SSTVT_8G,
                                  LSM6DS3_ACCEL_RANGE_8G,
                                  LSM6DS3_ACCEL_BW50);

    if(rv != SNS_RC_SUCCESS)
    {
       return SNS_RC_FAILED;
    }

    if(0)//(state->accel_info.lp_mode) TODO
    {
      buffer[0] = 0x10;
      rv = lsm6ds3_read_modify_write(scp_service,
                                     port_handle,
                                     STM_LSM6DS3_REG_CTRL6_G,
                                     &buffer[0],
                                     1,
                                     &xfer_bytes,
                                     false,
                                     0x10);

      if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
      {
         return SNS_RC_FAILED;
      }
    }
     //workaround enable HPF for XL here
     //initialize with high threshold
     buffer[0] = 0x3F;
     rv = lsm6ds3_read_modify_write(scp_service,
                                    port_handle,
                                    STM_LSM6DS3_REG_WAKE_THS,
                                    &buffer[0],
                                    1,
                                    &xfer_bytes,
                                    false,
                                    0x3F);

     if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
     {
        return SNS_RC_FAILED;
     }

     buffer[0] = 0
       | (0<<7)            // LPF2_XL_EN: refer to figure 5.
       | (0<<5)            // HPCF_XL[1:0]: 0-SlopeFilter=ODR/50; 1-HP Filter=ODR/100; 2-HP Filter=ODR/9; 3-HP Filter=ODR/400.
       | (0<<2)            // HP_SLOPE_XL_EN:
       | 0;                // LOW_PASS_ON_6D:
     rv = lsm6ds3_read_modify_write(scp_service,
                                    port_handle,
                                    STM_LSM6DS3_REG_CTRL8_XL,
                                    &buffer[0],
                                    1,
                                    &xfer_bytes,
                                    false,
                                    0xFF);

     if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
     {
        return SNS_RC_FAILED;
     }

     lsm6ds3_set_accel_config(scp_service,
                              port_handle,
                              LSM6DS3_ACCEL_ODR6660,
                              LSM6DS3_ACCEL_SSTVT_8G,
                              LSM6DS3_ACCEL_RANGE_8G,
                              LSM6DS3_ACCEL_BW50);

     buffer[0] = 0
       | (0<<7)            // TIMER_EN:
       | (0<<6)            // PEDO_EN:
       | (0<<5)            // TILT_EN:
       | (1<<4)            // SLOPE_FDS: refer to figure 5.
       | (0<<3)            // TAP_X_EN:
       | (0<<2)            // TAP_Y_EN:
       | (0<<1)            // TAP_Z_EN:
       | 0;                   // LIR: 0-interrupt not latched; 1-latched.

     rv = lsm6ds3_read_modify_write(scp_service,
                         port_handle,
                         STM_LSM6DS3_REG_TAP_CFG,
                         &buffer[0],
                         1,
                         &xfer_bytes,
                         false,
                         0x10);

     if(rv != SNS_RC_SUCCESS || xfer_bytes != 1)
     {
       return SNS_RC_FAILED;
     }

     //settling time for slope/Hp filter, 2 sample time
     sns_busy_wait(sns_convert_ns_to_ticks(1*1000*1000));  //1ms

     lsm6ds3_set_accel_config(scp_service,
                              port_handle,
                              LSM6DS3_ACCEL_ODR_OFF,
                              LSM6DS3_ACCEL_SSTVT_8G,
                              LSM6DS3_ACCEL_RANGE_8G,
                              LSM6DS3_ACCEL_BW50);

     lsm6ds3_set_gyro_config(scp_service,
                             port_handle,
                             LSM6DS3_GYRO_ODR_OFF,
                             LSM6DS3_GYRO_SSTVT_2000DPS,
                             STM_LSM6DS3_GYRO_RANGE_2000DPS);

     //ZRL shift
     buffer[0] = 0x40;
     rv = lsm6ds3_read_modify_write(scp_service,
                                    port_handle,
                                    0x00,
                                    &buffer[0],
                                    1,
                                    &xfer_bytes,
                                    false,
                                    0xFF);

     buffer[0] = 0x10;
     rv = lsm6ds3_read_modify_write(scp_service,
                                    port_handle,
                                    0x63,
                                    &buffer[0],
                                    1,
                                    &xfer_bytes,
                                    false,
                                    0x10);

     buffer[0] = 0x80;
     rv = lsm6ds3_read_modify_write(scp_service,
                                    port_handle,
                                    0x00,
                                    &buffer[0],
                                    1,
                                    &xfer_bytes,
                                    false,
                                    0xFF);
     buffer[0] = 0x02;
     rv = lsm6ds3_read_modify_write(scp_service,
                                    port_handle,
                                    0x02,
                                    &buffer[0],
                                    1,
                                    &xfer_bytes,
                                    false,
                                    0xFF);

     buffer[0] = 0x00;
     rv = lsm6ds3_read_modify_write(scp_service,
                                    port_handle,
                                    0x00,
                                    &buffer[0],
                                    1,
                                    &xfer_bytes,
                                    false,
                                    0xFF);
  }

  return SNS_RC_SUCCESS;
}

/**
 * see sns_lsm6ds3_hal.h
 */
sns_rc lsm6ds3_reset_device(sns_sync_com_port_service *scp_service,
                            sns_sync_com_port_handle *port_handle,
                            lsm6ds3_sensor_type sensor)
{
  sns_rc rv = SNS_RC_SUCCESS;

  /** HW reset only when both Accel and Gyro are requested for
   *  reset. */
  if( sensor == (LSM6DS3_ACCEL | LSM6DS3_GYRO | LSM6DS3_MOTION_DETECT | LSM6DS3_SENSOR_TEMP))
  {
     rv = lsm6ds3_device_sw_reset(scp_service, port_handle, sensor);
  }

  if(rv == SNS_RC_SUCCESS)
  {
    rv = lsm6ds3_device_set_default_state(scp_service, port_handle, sensor);
  }

  return rv;
}

/**
 * see sns_lsm6ds3_hal.h
 */
sns_rc lsm6ds3_set_gyro_config(sns_sync_com_port_service * scp_service,
                               sns_sync_com_port_handle *port_handle,
                               lsm6ds3_gyro_odr         curr_odr,
                               lsm6ds3_gyro_sstvt       sstvt,
                               lsm6ds3_gyro_range       range)
{
  UNUSED_VAR(sstvt);
  uint8_t buffer = (uint8_t)curr_odr | (uint8_t )range;
  uint32_t xfer_bytes;

  return lsm6ds3_com_write_wrapper(scp_service,
                                   port_handle,
                                   STM_LSM6DS3_REG_CTRL2_G,
                                   &buffer,
                                   1,
                                   &xfer_bytes,
                                   false);
}

/**
 * see sns_lsm6ds3_hal.h
 */
sns_rc lsm6ds3_set_accel_config(sns_sync_com_port_service * scp_service,
                                sns_sync_com_port_handle  *port_handle,
                                lsm6ds3_accel_odr       curr_odr,
                                lsm6ds3_accel_sstvt     sstvt,
                                lsm6ds3_accel_range     range,
                                lsm6ds3_accel_bw        bw)
{
  UNUSED_VAR(sstvt);
  uint8_t buffer = (uint8_t)curr_odr | (uint8_t )range | (uint8_t)bw;
  uint32_t xfer_bytes;

  return lsm6ds3_com_write_wrapper(scp_service,
                                   port_handle,
                                   STM_LSM6DS3_REG_CTRL1_A,
                                   &buffer,
                                   1,
                                   &xfer_bytes,
                                   false);
}

/**
 * Updates GYRO_SLEEP bit.
 *
 * @param[i] port_handle   synch COM port handle
 * @param[i] set_sleep     true to enable Gyro sleep else false
 *
 * @return sns_rc
 * SNS_RC_SUCCESS
 */
sns_rc lsm6ds3_set_gyro_sleep(sns_sync_com_port_service * scp_service,
                              sns_sync_com_port_handle *port_handle,
                              bool set_sleep)
{
  uint8_t buffer;
  uint32_t xfer_bytes;

  if(set_sleep)
  {
    buffer = 0x40;
  }
  else
  {
    buffer = 0x0;
  }
  // Update Gyro Sleep state
  lsm6ds3_read_modify_write(scp_service,
                            port_handle,
                            STM_LSM6DS3_REG_CTRL4,
                            &buffer,
                            1,
                            &xfer_bytes,
                            false,
                            0x40);
  return SNS_RC_SUCCESS;
}

/**
 * see sns_lsm6ds3_hal.h
 */
void lsm6ds3_stop_fifo_streaming(lsm6ds3_instance_state *state)
{
  uint8_t rw_buffer = 0x00;
  uint32_t xfer_bytes;

  if(state->fifo_info.fifo_enabled)
  {
    lsm6ds3_com_write_wrapper(state->scp_service,
                              state->com_port_info.port_handle,
                              STM_LSM6DS3_REG_FIFO_CTRL5,
                              &rw_buffer,
                              1,
                              &xfer_bytes,
                              false);
  }

    lsm6ds3_set_accel_config(
                           state->scp_service,
                           state->com_port_info.port_handle,
                           LSM6DS3_ACCEL_ODR_OFF,
                           state->accel_info.sstvt,
                           state->accel_info.range,
                           state->accel_info.bw);

    lsm6ds3_set_gyro_sleep(state->scp_service, state->com_port_info.port_handle, true);
    state->gyro_info.is_in_sleep = true;
  }

/**
 * see sns_lsm6ds3_hal.h
 */
void lsm6ds3_set_fifo_config(lsm6ds3_instance_state *state,
                             uint16_t desired_wmk,
                             lsm6ds3_accel_odr a_chosen_sample_rate,
                             lsm6ds3_gyro_odr g_chosen_sample_rate,
                             lsm6ds3_sensor_type sensor)
{
  UNUSED_VAR(sensor);
  state->fifo_info.fifo_rate = a_chosen_sample_rate;
  state->fifo_info.cur_wmk = desired_wmk;

  state->accel_info.curr_odr = state->fifo_info.fifo_rate;
  state->accel_info.sstvt = LSM6DS3_ACCEL_SSTVT_8G;
  state->accel_info.range = LSM6DS3_ACCEL_RANGE_8G;
  state->accel_info.bw = LSM6DS3_ACCEL_BW50;

  state->gyro_info.curr_odr = (sensor & LSM6DS3_GYRO) ? g_chosen_sample_rate : LSM6DS3_GYRO_ODR_OFF;
  state->gyro_info.range = STM_LSM6DS3_GYRO_RANGE_2000DPS;
  state->gyro_info.sstvt = LSM6DS3_GYRO_SSTVT_2000DPS;
}

/**
 * see sns_lsm6ds3_hal.h
 */
void lsm6ds3_set_fifo_wmk(lsm6ds3_instance_state *state)
{
  uint16_t wmk_words = 0;
  uint8_t wmkL = 0;
  uint8_t wmkH = 0;
  uint32_t xfer_bytes;
  uint8_t buffer;
  uint8_t decimation = 0;

  //convert samples to words
  if((state->accel_info.curr_odr > LSM6DS3_ACCEL_ODR_OFF)
     &&
     (state->fifo_info.fifo_enabled & LSM6DS3_ACCEL))
  {
    // TODO If Accel is ON because of Gyro then no need to add in FIFO
    wmk_words = state->fifo_info.cur_wmk * 3;
    decimation |= 0x1;
  }

  if((state->gyro_info.curr_odr > LSM6DS3_GYRO_ODR_OFF)
     &&
     (state->fifo_info.fifo_enabled & LSM6DS3_GYRO))
  {
    wmk_words += state->fifo_info.cur_wmk * 3;
    decimation |= 0x8;
  }

  // Set Accel decimation to no decimation
  buffer = decimation;
  lsm6ds3_read_modify_write(state->scp_service,
                            state->com_port_info.port_handle,
                            STM_LSM6DS3_REG_FIFO_CTRL3,
                            &buffer,
                            1,
                            &xfer_bytes,
                            false,
                            0xFF);
  // Configure FIFO WM
  wmkL = wmk_words & 0xFF;
  lsm6ds3_read_modify_write(state->scp_service,
                            state->com_port_info.port_handle,
                            STM_LSM6DS3_REG_FIFO_CTRL1,
                            &wmkL,
                            1,
                            &xfer_bytes,
                            false,
                            0x0);

  wmkH = (wmk_words >> 8) & 0x0F;
  lsm6ds3_read_modify_write(state->scp_service,
                            state->com_port_info.port_handle,
                            STM_LSM6DS3_REG_FIFO_CTRL2,
                            &wmkH,
                            1,
                            &xfer_bytes,
                            false,
                            0xF);

}

/**
 * see sns_lsm6ds3_hal.h
 */
void lsm6ds3_start_fifo_streaming(lsm6ds3_instance_state *state)
{
  // Enable FIFO Streaming
  // Enable Accel Streaming
  // Enable GYRO Streaming

  uint8_t rw_buffer = 0x00;
  uint32_t xfer_bytes;

  //start streaming,stream mode
  rw_buffer = 0x06 | (uint8_t)(state->fifo_info.fifo_rate >> 1);
  lsm6ds3_read_modify_write(state->scp_service,
                            state->com_port_info.port_handle,
                            STM_LSM6DS3_REG_FIFO_CTRL5,
                            &rw_buffer,
                            1,
                            &xfer_bytes,
                            false,
                            0xFF);

  lsm6ds3_set_accel_config(state->scp_service,
                           state->com_port_info.port_handle,
                           state->accel_info.curr_odr,
                           state->accel_info.sstvt,
                           state->accel_info.range,
                           state->accel_info.bw);
  lsm6ds3_set_gyro_config(state->scp_service,
                          state->com_port_info.port_handle,
                          state->gyro_info.curr_odr,
                          state->gyro_info.sstvt,
                          state->gyro_info.range);

  if(state->gyro_info.curr_odr > LSM6DS3_GYRO_ODR_OFF)
  {
    lsm6ds3_set_gyro_sleep(state->scp_service,state->com_port_info.port_handle, false);
    state->gyro_info.is_in_sleep = false;
  }
}

/**
 * see sns_lsm6ds3_hal.h
 */
void lsm6ds3_enable_fifo_intr(lsm6ds3_instance_state *state,
                              lsm6ds3_sensor_type sensor)
{
  UNUSED_VAR(sensor);
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;

  if(state->fifo_info.fifo_enabled)
  {
    if(state->fifo_info.cur_wmk > 0)
    {
      if(state->fifo_info.fifo_enabled & LSM6DS3_ACCEL
         ||
         state->fifo_info.fifo_enabled & LSM6DS3_GYRO)
      {
        // Configure lsm6ds3 FIFO control registers
        rw_buffer = 0x0
          | (0<<7)       // INT1 pedometer
          | (0<<6)       // INT1 sign_motion
          | (0<<5)       // INT1 FIFO_FULL flag
          | (1<<4)       // INT1 FIFO_OVR flag
          | (1<<3)       // INT1 FIFO_FTH flag
          | (0<<2)       // INT1 BOOT flag
          | (0<<1)       // INT1 DRDY_G flag
          | 0;           // INT1 DRDY_XL flag
        lsm6ds3_read_modify_write(state->scp_service,
                                  state->com_port_info.port_handle,
                                  STM_LSM6DS3_REG_INT1_CTRL,
                                  &rw_buffer,
                                  1,
                                  &xfer_bytes,
                                  false,
                                  0x38);
      }
    }
    else
    {
      // TODO add error code - trying to enable FIFO interrupt when wmk=0
    }
  }
}

/**
 * see sns_lsm6ds3_hal.h
 */
void lsm6ds3_disable_fifo_intr(lsm6ds3_instance_state *state)
{
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;

  lsm6ds3_read_modify_write(state->scp_service,
                            state->com_port_info.port_handle,
                            STM_LSM6DS3_REG_INT1_CTRL,
                            &rw_buffer,
                            1,
                            &xfer_bytes,
                            false,
                            0x38);
}

/**
 * see sns_lsm6ds3_hal.h
 */
sns_rc lsm6ds3_get_who_am_i(sns_sync_com_port_service *scp_service,
                            sns_sync_com_port_handle *port_handle,
                            uint8_t *buffer)
{
  sns_rc rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  rv = lsm6ds3_com_read_wrapper(scp_service,
                                port_handle,
                                STM_LSM6DS3_REG_WHO_AM_I,
                                buffer,
                                1,
                                &xfer_bytes);

  if(rv != SNS_RC_SUCCESS
     ||
     xfer_bytes != 1)
  {
    rv = SNS_RC_FAILED;
  }

  return rv;
}

/**
 * Reads four status registers starting from
 * STM_LSM6DS3_REG_FIFO_STATUS1.
 *
 * @param[i] state              Instance state
 * @param[o] buffer             status registers
 *
 * @return none
 */
static void lsm6ds3_read_fifo_status(lsm6ds3_instance_state *state,
                                     uint8_t *buffer)
{
  uint32_t xfer_bytes;

  lsm6ds3_com_read_wrapper(state->scp_service,
                           state->com_port_info.port_handle,
                           STM_LSM6DS3_REG_FIFO_STATUS1,
                           buffer,
                           4,
                           &xfer_bytes);

}

/**
 * Updates temp sensor polling configuration
 *
 * @param[i] instance   Sensor instance
 *
 * @return sampling interval time in ticks
 */
static void lsm6ds3_set_polling_config(sns_sensor_instance *const this)
{
  lsm6ds3_instance_state *state = (lsm6ds3_instance_state*)this->state->state;

  if(state->sensor_temp_info.sampling_intvl > 0)
  {
    if(!state->sensor_temp_info.timer_is_active)
    {
      lsm6ds3_start_sensor_temp_polling_timer(this);
    }
  }
  else if(state->sensor_temp_info.timer_is_active)
  {
    state->sensor_temp_info.timer_is_active = false;
  }
}

/**
 * Gets current Accel ODR.
 *
 * @param[i] curr_odr              Current FIFO ODR.
 *
 */
float lsm6ds3_get_accel_odr(lsm6ds3_accel_odr curr_odr)
{
  float odr = 0.0;
  int8_t idx;

  for(idx = 0; idx < ARR_SIZE(reg_map); idx++)
  {
    if(curr_odr == reg_map[idx].accel_odr_reg_value
       &&
       curr_odr != LSM6DS3_ACCEL_ODR_OFF)
    {
      odr = reg_map[idx].odr;
      break;
    }
  }

  return odr;
}

/**
 * Provides sample interval based on current ODR.
 *
 * @param[i] curr_odr              Current FIFO ODR.
 *
 * @return sampling interval time in ticks
 */
sns_time lsm6ds3_get_sample_interval(lsm6ds3_accel_odr curr_odr)
{
  sns_time  sample_interval = 0;
  float odr = lsm6ds3_get_accel_odr(curr_odr);

  if(odr > 0.0)
  {
    sample_interval = sns_convert_ns_to_ticks(1000000000 / odr);
  }

  return sample_interval;
}

/**
 * Extract a gyro sample from a segment of the fifo buffer and generate an
 * event.
 *
 * @param[i] fifo_sample        The segment of fifo buffer that has the gyro sample
 * @param[i] timestamp          Timestamp to be used for this sample
 * @param[i] instance           The current lsm6ds3 sensor instance
 * @param[i] event_service      Event service for sending out the gyro sample
 * @param[i] state              The state of the lsm6ds3 sensor instance
 */
static void lsm6ds3_handle_gyro_sample(const uint8_t fifo_sample[6],
                                sns_time timestamp,
                                sns_sensor_instance *const instance,
                                sns_event_service *event_service,
                                lsm6ds3_instance_state *state,
                                log_sensor_state_raw_info *log_gyro_state_raw_info)
{
  UNUSED_VAR(event_service);

  float data[3];

  data[0] =
     (int16_t)(((fifo_sample[1] << 8) & 0xFF00) | fifo_sample[0]) * state->gyro_info.sstvt / 1000 ;
  data[1] =
     (int16_t)(((fifo_sample[3] << 8) & 0xFF00) | fifo_sample[2]) * state->gyro_info.sstvt / 1000 ;
  data[2] =
     (int16_t)(((fifo_sample[5] << 8) & 0xFF00) | fifo_sample[4]) * state->gyro_info.sstvt / 1000 ;

  if(state->fifo_info.publish_sensors & LSM6DS3_GYRO)
  {
    sns_std_sensor_sample_status status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;

    if(state->gyro_info.num_samples_to_discard != 0)
    {
      status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
      state->gyro_info.num_samples_to_discard--;
    }
    pb_send_sensor_stream_event(instance,
                                &state->gyro_info.suid,
                                timestamp,
                                SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                                status,
                                data,
                                ARR_SIZE(data),
                                state->encoded_imu_event_len);

    // Log raw uncalibrated sensor data
    lsm6ds3_log_sensor_state_raw_add(
      log_gyro_state_raw_info,
      data,
      timestamp,
      status);
  }
}

/**
 * Extract a accel sample from a segment of the fifo buffer and generate an
 * event.
 *
 * @param[i] fifo_sample        The segment of fifo buffer that has the accel sample
 * @param[i] timestamp          Timestamp to be used for this sample
 * @param[i] instance           The current lsm6ds3 sensor instance
 * @param[i] event_service      Event service for sending out the gyro sample
 * @param[i] state              The state of the lsm6ds3 sensor instance
 */
static void lsm6ds3_handle_accel_sample(const uint8_t fifo_sample[6],
                                sns_time timestamp,
                                sns_sensor_instance *const instance,
                                sns_event_service *event_service,
                                lsm6ds3_instance_state *state,
                                log_sensor_state_raw_info *log_accel_state_raw_info)
{
  UNUSED_VAR(event_service);
  float data[3];
  data[0] =
     (int16_t)(((fifo_sample[1] << 8) & 0xFF00) | fifo_sample[0]) *state->accel_info.sstvt * G/1000000;
  data[1] =
     (int16_t)(((fifo_sample[3] << 8) & 0xFF00) | fifo_sample[2]) *state->accel_info.sstvt * G/1000000;
  data[2] =
     (int16_t)(((fifo_sample[5] << 8) & 0xFF00) | fifo_sample[4]) *state->accel_info.sstvt * G/1000000;

  if(state->fifo_info.publish_sensors & LSM6DS3_ACCEL
     || (state->accel_info.gated_client_present && !state->md_info.enable_md_int))
  {
    sns_std_sensor_sample_status status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;

    if(state->accel_info.num_samples_to_discard != 0)
    {
      status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
      state->accel_info.num_samples_to_discard--;
    }
    pb_send_sensor_stream_event(instance,
                                &state->accel_info.suid,
                                timestamp,
                                SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                                status,
                                data,
                                ARR_SIZE(data),
                                state->encoded_imu_event_len);

    // Log raw uncalibrated sensor data
    lsm6ds3_log_sensor_state_raw_add(
      log_accel_state_raw_info,
      data,
      timestamp,
      status);
  }
    }

void lsm6ds3_process_fifo_data_buffer(
  sns_sensor_instance *instance,
  bool                gyro_enabled,
  sns_time            first_timestamp,
  sns_time            sample_interval_ticks,
  const uint8_t       *fifo_start,
  size_t              num_bytes
)
{
  uint16_t num_sample_sets = 0;
  uint32_t i;
  lsm6ds3_instance_state *state = (lsm6ds3_instance_state *)instance->state->state;
  sns_service_manager *service_manager = instance->cb->get_service_manager(instance);
  sns_event_service *event_service =
     (sns_event_service*)service_manager->get_service(service_manager, SNS_EVENT_SERVICE);
  sns_diag_service* diag = state->diag_service;
  log_sensor_state_raw_info log_accel_state_raw_info, log_gyro_state_raw_info;

  // Allocate Sensor State Raw log packets for accel and gyro
  sns_memzero(&log_accel_state_raw_info, sizeof(log_accel_state_raw_info));
  log_accel_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;
  lsm6ds3_log_sensor_state_raw_alloc(diag, instance, &state->accel_info.suid,
    &log_accel_state_raw_info);

  if(gyro_enabled)
  {
    sns_memzero(&log_gyro_state_raw_info, sizeof(log_gyro_state_raw_info));
    log_gyro_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;
    lsm6ds3_log_sensor_state_raw_alloc(diag, instance, &state->gyro_info.suid,
    &log_gyro_state_raw_info);
  }

  for(i = 0; i < num_bytes; i += 6)
  {
    sns_time timestamp = first_timestamp + (num_sample_sets++ * sample_interval_ticks);
    if(gyro_enabled)
    {
       // First sample belongs to Gyro
      lsm6ds3_handle_gyro_sample(&fifo_start[i],
                                 timestamp,
                                 instance,
                                 event_service,
                                 state,
                                 &log_gyro_state_raw_info);
      i += 6;
    }
    lsm6ds3_handle_accel_sample(&fifo_start[i],
                                timestamp,
                                instance,
                                event_service,
                                state,
                                &log_accel_state_raw_info);
  }

  lsm6ds3_log_sensor_state_raw_submit(diag,
                                      instance,
                                      &state->accel_info.suid,
                                      &log_accel_state_raw_info);

  if(gyro_enabled)
  {
    lsm6ds3_log_sensor_state_raw_submit(diag,
                                        instance,
                                        &state->gyro_info.suid,
                                        &log_gyro_state_raw_info);
  }
}

/** See lsm6ds3_hal.h */
void lsm6ds3_send_fifo_flush_done(sns_sensor_instance *const instance)
{
  sns_service_manager *mgr = instance->cb->get_service_manager(instance);
  sns_event_service *e_service = (sns_event_service*)mgr->get_service(mgr, SNS_EVENT_SERVICE);
  sns_sensor_event *event = e_service->api->alloc_event(e_service, instance, 0);
  lsm6ds3_instance_state *state = (lsm6ds3_instance_state *)instance->state->state;

  if(NULL != event)
  {
    event->message_id = SNS_STD_MSGID_SNS_STD_FLUSH_EVENT;
    event->event_len = 0;
    event->timestamp = sns_get_system_time();

    if(state->fifo_info.publish_sensors & LSM6DS3_ACCEL)
    {
      e_service->api->publish_event(e_service, instance, event, &state->accel_info.suid);
    }

    if(state->fifo_info.publish_sensors & LSM6DS3_GYRO)
    {
      e_service->api->publish_event(e_service, instance, event, &state->gyro_info.suid);
    }
  }
}

void lsm6ds3_handle_interrupt_event(sns_sensor_instance *const instance)
{

  uint8_t fifo_status[4] = {0, 0, 0, 0};
  uint8_t buffer[100];
  uint32_t enc_len;

  lsm6ds3_instance_state *state =
     (lsm6ds3_instance_state*)instance->state->state;
  sns_port_vector async_read_msg;

  // Read the FIFO Status register
  lsm6ds3_read_fifo_status(state, &fifo_status[0]);

  // Calculate the number of bytes to be read
  uint16_t countH = fifo_status[1] & 0x0F;
  uint16_t countL = fifo_status[0] & 0xFF;
  uint16_t num_of_bytes =  (((countH << 8) & 0xFF00) | countL) * 2;

  if(num_of_bytes > 0)
  {
    // Compose the async com port message
    async_read_msg.bytes = num_of_bytes;
    async_read_msg.reg_addr = STM_LSM6DS3_REG_FIFO_DATA_OUT_L;
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
  else
  {
    // reset FIFO state if there are no samples to flush
    if(state->fifo_flush_in_progress)
    {
      state->fifo_flush_in_progress = false;
      lsm6ds3_send_fifo_flush_done(instance);
    }
  }
}

/**
 * see sns_lsm6ds3_hal.h
 */
void lsm6ds3_dump_reg(sns_sensor_instance *this,
                      lsm6ds3_sensor_type sensor)
{
  UNUSED_VAR(sensor);
  lsm6ds3_instance_state *state =
     (lsm6ds3_instance_state*)this->state->state;
  //sns_diag_service *diag = state->diag_service;
  uint32_t xfer_bytes;
  uint8_t reg_map[] = {
    STM_LSM6DS3_REG_FIFO_CTRL1,
    STM_LSM6DS3_REG_FIFO_CTRL2,
    STM_LSM6DS3_REG_FIFO_CTRL3,
    STM_LSM6DS3_REG_FIFO_CTRL4,
    STM_LSM6DS3_REG_FIFO_CTRL5,
    STM_LSM6DS3_REG_INT1_CTRL,
    STM_LSM6DS3_REG_INT2_CTRL,
    STM_LSM6DS3_REG_CTRL1_A,
    STM_LSM6DS3_REG_CTRL2_G,
    STM_LSM6DS3_REG_CTRL3,
    STM_LSM6DS3_REG_CTRL4,
    STM_LSM6DS3_REG_CTRL5,
    STM_LSM6DS3_REG_CTRL6_G,
    STM_LSM6DS3_REG_CTRL7_G,
    STM_LSM6DS3_REG_CTRL8_XL,
    STM_LSM6DS3_REG_CTRL9_A,
    STM_LSM6DS3_REG_CTRL10,
    STM_LSM6DS3_REG_TAP_CFG,
    STM_LSM6DS3_REG_WAKE_THS,
    STM_LSM6DS3_REG_WAKE_DUR,
    STM_LSM6DS3_REG_MD1_CFG,
    STM_LSM6DS3_REG_WAKE_SRC
  };

  uint8_t i = 0;
  uint16_t n = sizeof(reg_map)/sizeof(reg_map[0]);

  for(i=0; i<n;i++)
  {
    lsm6ds3_com_read_wrapper(state->scp_service,
                         state->com_port_info.port_handle,
                         reg_map[i],
                         &state->reg_status[i],
                         1,
                         &xfer_bytes);
    //diag->api->sensor_inst_printf(diag, this,
    //                              &state->accel_info.suid,
    //                              SNS_LOW, __FILENAME__, __LINE__,
    //                              "reg[0x%X] = 0x%X",
    //                              reg_map[i], state->reg_status[i]);
  }
}

void lsm6ds3_set_gated_accel_config(lsm6ds3_instance_state *state,
                      uint16_t desired_wmk,
                      lsm6ds3_accel_odr a_chosen_sample_rate,
                      lsm6ds3_sensor_type sensor)
{
  if(!state->accel_info.gated_client_present)
  {
    desired_wmk = 0;
    a_chosen_sample_rate = LSM6DS3_ACCEL_ODR_OFF;
  }

  if(sensor & LSM6DS3_ACCEL)
  {
    state->md_info.desired_wmk = desired_wmk;
    state->md_info.desired_odr = a_chosen_sample_rate;
    state->md_info.sstvt = LSM6DS3_ACCEL_SSTVT_8G;
    state->md_info.range = LSM6DS3_ACCEL_RANGE_8G;
    state->md_info.bw = LSM6DS3_ACCEL_BW50;
  }
}

void lsm6ds3_set_md_config(lsm6ds3_instance_state *state, bool enable)
{
  uint8_t rw_buffer = 0;
  uint8_t dur_set = LSM6DS3_MD_DUR;
  uint8_t thresh_set = enable ? LSM6DS3_MD_THRESH : 0x3F;
  uint32_t xfer_bytes;
  lsm6ds3_accel_odr accel_odr = enable ? LSM6DS3_MD_ODR : LSM6DS3_ACCEL_ODR_OFF;

  rw_buffer = thresh_set;
  lsm6ds3_read_modify_write(state->scp_service,
                            state->com_port_info.port_handle,
                            STM_LSM6DS3_REG_WAKE_THS,
                            &rw_buffer,
                            1,
                            &xfer_bytes,
                            false,
                            0x3F);

  rw_buffer = dur_set;
  lsm6ds3_read_modify_write(state->scp_service,
                            state->com_port_info.port_handle,
                            STM_LSM6DS3_REG_WAKE_DUR,
                            &rw_buffer,
                            1,
                            &xfer_bytes,
                            false,
                            0x60);

  lsm6ds3_set_accel_config(state->scp_service,
                           state->com_port_info.port_handle,
                           accel_odr,
                           LSM6DS3_ACCEL_SSTVT_8G,
                           LSM6DS3_ACCEL_RANGE_8G,
                           LSM6DS3_ACCEL_BW50);
}

void lsm6ds3_update_md_intr(sns_sensor_instance *const instance,
                            bool enable,
                            bool md_not_armed_event)
{
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;
  lsm6ds3_instance_state *state = (lsm6ds3_instance_state*)instance->state->state;
  sns_diag_service* diag = state->diag_service;

  if(enable)
  {
    rw_buffer = 0
      | (0<<7)            // INT1_INACT_STATE
      | (0<<6)            // INT1_SINGLE_TAP
      | (1<<5)            // INT1_WU
      | (0<<4)            // INT1_FF
      | (0<<3)            // INT1_DOUBLE_TAP
      | (0<<2)            // INT1_6D
      | (0<<1)            // INT1_TILT
      | 0;                // INT1_TIMER
  }

  lsm6ds3_read_modify_write(state->scp_service,
                            state->com_port_info.port_handle,
                            STM_LSM6DS3_REG_MD1_CFG,
                            &rw_buffer,
                            1,
                            &xfer_bytes,
                            false,
                            0x20);

  if(enable)
  {
    state->md_info.md_state.motion_detect_event_type = SNS_MOTION_DETECT_EVENT_TYPE_ENABLED;
  }
  else
  {
    state->md_info.md_state.motion_detect_event_type = SNS_MOTION_DETECT_EVENT_TYPE_DISABLED;
  }

  if(enable || md_not_armed_event)
  {
    diag->api->sensor_inst_printf(diag, instance,
                                  &state->md_info.suid,
                                  SNS_LOW, __FILENAME__, __LINE__,
                                  "lsm6ds3_update_md_intr md_is_armed=%d",
                                  state->md_info.md_state.motion_detect_event_type);
    pb_send_event(instance,
                  sns_motion_detect_event_fields,
                  &state->md_info.md_state,
                  sns_get_system_time(),
                  SNS_MOTION_DETECT_MSGID_SNS_MOTION_DETECT_EVENT,
                  &state->md_info.suid);
  }
}

/**
 * Changes all gated accel requests to non-gated accel requests.
 *
 * @param instance   Reference to the instance
 *
 * @return None
 */
static void lsm6ds3_convert_accel_gated_req_to_non_gated(
   sns_sensor_instance *const instance)
{
  sns_request *request;

  /** Parse through existing requests and change gated accel
   *  requests to non-gated accel requests. */
  for(request = (sns_request *)instance->cb->get_client_request(instance, &((sns_sensor_uid)ACCEL_SUID), true);
      NULL != request;
      request = (sns_request *)instance->cb->get_client_request(instance, &((sns_sensor_uid)ACCEL_SUID), false))
  {
    if(request->message_id == SNS_STD_EVENT_GATED_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
    {
      request->message_id = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG;
    }
  }
}

void lsm6ds3_handle_md_interrupt(sns_sensor_instance *const instance,
                                 sns_time irq_timestamp)
{
  lsm6ds3_instance_state *state =
     (lsm6ds3_instance_state*)instance->state->state;
  uint8_t rw_buffer = 0;
  uint32_t xfer_bytes;
  sns_diag_service* diag = state->diag_service;

  lsm6ds3_com_read_wrapper(state->scp_service,
                           state->com_port_info.port_handle,
                           STM_LSM6DS3_REG_WAKE_SRC,
                           &rw_buffer,
                           1,
                           &xfer_bytes);

  if(rw_buffer & 0x08)
  {
    state->md_info.md_state.motion_detect_event_type =
       SNS_MOTION_DETECT_EVENT_TYPE_FIRED;
    pb_send_event(instance,
                  sns_motion_detect_event_fields,
                  &state->md_info.md_state,
                  irq_timestamp,
                  SNS_MOTION_DETECT_MSGID_SNS_MOTION_DETECT_EVENT,
                  &state->md_info.suid);

    lsm6ds3_convert_accel_gated_req_to_non_gated(instance);

    diag->api->sensor_inst_printf(diag, instance,
                                  &state->md_info.suid,
                                  SNS_LOW, __FILENAME__, __LINE__,
                                  "MD fired");

    // Sensor State HW Interrupt Log
    sns_diag_sensor_state_interrupt *log =
      (sns_diag_sensor_state_interrupt *)diag->api->alloc_log(
        diag,
        instance,
        &state->md_info.suid,
        sizeof(sns_diag_sensor_state_interrupt),
        SNS_DIAG_SENSOR_STATE_LOG_INTERRUPT);

    if(NULL != log)
    {
      log->interrupt = SNS_DIAG_INTERRUPT_MOTION;
      log->timestamp = irq_timestamp;

      diag->api->submit_log(diag,
                            instance,
                            &state->md_info.suid,
                            sizeof(sns_diag_sensor_state_interrupt),
                            log,
                            SNS_DIAG_SENSOR_STATE_LOG_INTERRUPT,
                            state->log_interrupt_encoded_size,
                            lsm6ds3_encode_sensor_state_log_interrupt);
    }
  }
}

/** See sns_lsm6ds3_hal.h */
void lsm6ds3_send_config_event(sns_sensor_instance *const instance)
{
  lsm6ds3_instance_state *state =
     (lsm6ds3_instance_state*)instance->state->state;

  sns_std_sensor_physical_config_event phy_sensor_config =
     sns_std_sensor_physical_config_event_init_default;

  // TODO: Use appropriate op_mode selected by driver.
  char operating_mode[] = "NORMAL";

  pb_buffer_arg op_mode_args;

  op_mode_args.buf = &operating_mode[0];
  op_mode_args.buf_len = sizeof(operating_mode);

  phy_sensor_config.has_sample_rate = true;
  phy_sensor_config.sample_rate =
     (state->fifo_info.fifo_rate > 0) ?
     (float)(13 << ((state->fifo_info.fifo_rate >> 4) - 1)) : 0.0;

  phy_sensor_config.has_water_mark = true;
  phy_sensor_config.water_mark = state->fifo_info.cur_wmk;
  phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
  phy_sensor_config.operation_mode.arg = &op_mode_args;
  phy_sensor_config.has_active_current = true;
  phy_sensor_config.active_current = 240;
  phy_sensor_config.has_resolution = true;
  phy_sensor_config.resolution = LSM6DS3_ACCEL_RESOLUTION_8G;
  phy_sensor_config.range_count = 2;
  phy_sensor_config.range[0] = LSM6DS3_ACCEL_RANGE_8G_MIN;
  phy_sensor_config.range[1] = LSM6DS3_ACCEL_RANGE_8G_MAX;
  phy_sensor_config.has_stream_is_synchronous = true;
  phy_sensor_config.stream_is_synchronous = false;
  phy_sensor_config.has_dri_enabled = true;
  phy_sensor_config.dri_enabled = true;
  /* For sensors that route data through the SDC/DAE sensor, the DAE watermark
     should be set to the number of samples stored in SDC before waking up Q6. */
  phy_sensor_config.has_DAE_watermark = true;
  phy_sensor_config.DAE_watermark = 0;

  if(state->fifo_info.publish_sensors & LSM6DS3_ACCEL
     ||
     state->accel_info.gated_client_present)
  {
    pb_send_event(instance,
                  sns_std_sensor_physical_config_event_fields,
                  &phy_sensor_config,
                  sns_get_system_time(),
                  SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                  &state->accel_info.suid);
  }

  if(state->fifo_info.publish_sensors & LSM6DS3_GYRO)
  {
    // Override above values with gyro info
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = 1250;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = LSM6DS3_GYRO_SSTVT_2000DPS;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = LSM6DS3_GYRO_RANGE_2000_MIN;
    phy_sensor_config.range[1] = LSM6DS3_GYRO_RANGE_2000_MAX;
    phy_sensor_config.has_dri_enabled = true;
    phy_sensor_config.dri_enabled = true;
  /* For sensors that route data through the SDC/DAE sensor, the DAE watermark
     should be set to the number of samples stored in SDC before waking up Q6. */
    phy_sensor_config.has_DAE_watermark = true;
    phy_sensor_config.DAE_watermark = 0;

    pb_send_event(instance,
                  sns_std_sensor_physical_config_event_fields,
                  &phy_sensor_config,
                  sns_get_system_time(),
                  SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                  &state->gyro_info.suid);
  }


  if(state->fifo_info.publish_sensors & LSM6DS3_SENSOR_TEMP)
  {
    // Override above values with sensor temperature info
    phy_sensor_config.sample_rate = 1;
    phy_sensor_config.has_water_mark = false;
    phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
    phy_sensor_config.operation_mode.arg = &op_mode_args;
    phy_sensor_config.has_active_current = true;
    phy_sensor_config.active_current = 240;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = LSM6DS3_SENSOR_TEMPERATURE_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = LSM6DS3_SENSOR_TEMPERATURE_RANGE_MIN;
    phy_sensor_config.range[1] = LSM6DS3_SENSOR_TEMPERATURE_RANGE_MAX;
    phy_sensor_config.has_dri_enabled = true;
    phy_sensor_config.dri_enabled = false;
    phy_sensor_config.has_DAE_watermark = false;

    pb_send_event(instance,
                  sns_std_sensor_physical_config_event_fields,
                  &phy_sensor_config,
                  sns_get_system_time(),
                  SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                  &state->sensor_temp_info.suid);
  }
}

void lsm6ds3_convert_and_send_temp_sample(
  sns_sensor_instance *const instance,
  sns_time            timestamp,
  const uint8_t       temp_data[2])
{
  lsm6ds3_instance_state *state = (lsm6ds3_instance_state*)instance->state->state;
  int16_t temp_val = (int16_t)(temp_data[1] << 8 | temp_data[0]);
  float float_temp_val = (temp_val / 16.0) + 25.0;

  pb_send_sensor_stream_event(instance,
                              &state->sensor_temp_info.suid,
                              timestamp,
                              SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                              SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH,
                              &float_temp_val,
                              1,
                              state->encoded_sensor_temp_event_len);
}

/** See sns_lsm6ds3_hal.h */
void lsm6ds3_handle_sensor_temp_sample(sns_sensor_instance *const instance)
{
  lsm6ds3_instance_state *state =
     (lsm6ds3_instance_state*)instance->state->state;
  uint8_t temp_data[2] = {0};
  uint8_t buffer;
  uint32_t xfer_bytes = 0;

  lsm6ds3_com_read_wrapper(state->scp_service,
                   state->com_port_info.port_handle,
                   STM_LSM6DS3_REG_STATUS,
                   &buffer,
                   1,
                   &xfer_bytes);

  lsm6ds3_com_read_wrapper(state->scp_service,
                   state->com_port_info.port_handle,
                   STM_LSM6DS3_REG_OUT_TEMP_L,
                   &temp_data[0],
                   2,
                   &xfer_bytes);

  lsm6ds3_convert_and_send_temp_sample(instance,
                   sns_get_system_time(),
                   temp_data);
}

void lsm6ds3_start_sensor_temp_polling_timer(sns_sensor_instance *this)
{
  lsm6ds3_instance_state *state = (lsm6ds3_instance_state*)this->state->state;
  sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
  uint8_t buffer[50];
  sns_request timer_req = {
    .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
    .request    = buffer
  };

  sns_memset(buffer, 0, sizeof(buffer));
  req_payload.is_periodic = false;
  req_payload.start_time = sns_get_system_time();
  req_payload.timeout_period = state->sensor_temp_info.sampling_intvl;

  timer_req.request_len = pb_encode_request(buffer, sizeof(buffer), &req_payload,
                                            sns_timer_sensor_config_fields, NULL);
  if(timer_req.request_len > 0)
  {
    state->timer_data_stream->api->send_request(state->timer_data_stream, &timer_req);
    state->sensor_temp_info.timer_is_active = true;
  }
  else
  {
    //diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
    //                         "LSM timer req encode error");
  }
}

void lsm6ds3_reconfig_hw(sns_sensor_instance *this)
{
  lsm6ds3_instance_state *state = (lsm6ds3_instance_state*)this->state->state;

  bool enable_fifo_stream = true;

  if(state->md_info.enable_md_int)
  {
     if(state->fifo_info.fifo_rate > LSM6DS3_ACCEL_ODR_OFF
        && !(state->fifo_info.publish_sensors & (LSM6DS3_GYRO | LSM6DS3_SENSOR_TEMP | LSM6DS3_ACCEL)))
     {
       lsm6ds3_set_fifo_config(state,
                               0,
                               LSM6DS3_ACCEL_ODR_OFF,
                               LSM6DS3_GYRO_ODR_OFF,
                               state->fifo_info.fifo_enabled);
       lsm6ds3_stop_fifo_streaming(state);
       lsm6ds3_set_fifo_wmk(state);
       lsm6ds3_disable_fifo_intr(state);
       enable_fifo_stream = false;
     }

     lsm6ds3_set_md_config(state, true);
     if (state->irq_info.irq_ready)
     {
       lsm6ds3_update_md_intr(this, true, false);
     }
  }
  else
  {
    if(state->md_info.md_client_present
       &&
       !state->md_info.enable_md_int)
    {
      lsm6ds3_update_md_intr(this, false, true);
      lsm6ds3_set_md_config(state, false);
    }
  }

  if(enable_fifo_stream)
  {
    //lsm6ds3_flush_fifo(this);
    lsm6ds3_stop_fifo_streaming(state);
    lsm6ds3_set_fifo_wmk(state);
    lsm6ds3_start_fifo_streaming(state);

    // Enable interrupt only for accel, gyro and motion accel clients
    if(state->fifo_info.publish_sensors & (LSM6DS3_ACCEL | LSM6DS3_GYRO) &&
       state->irq_info.irq_ready )
    {
      lsm6ds3_enable_fifo_intr(state, state->fifo_info.fifo_enabled);
    }
    // Enable timer in case of sensor temp clients
    if(state->fifo_info.publish_sensors & LSM6DS3_SENSOR_TEMP &&
       !lsm6ds3_dae_if_available(this))
    {
      lsm6ds3_set_polling_config(this);
    }
  }
  
  if(state->fifo_info.publish_sensors != 0)
  {
    lsm6ds3_dae_if_start_streaming(this);
  }
  state->config_step = LSM6DS3_CONFIG_IDLE; /* done with reconfig */

  lsm6ds3_dump_reg(this, state->fifo_info.fifo_enabled);
}

