/**
 * @file sns_ak0991x_hal_island.c
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
#include "sns_ak0991x_s4s.h"

#include "sns_async_com_port.pb.h"
#include "sns_timer.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"

#include "sns_std_sensor.pb.h"

#include "sns_diag_service.h"
#include "sns_diag.pb.h"

#include "sns_cal_util.h"

//#define AK0991X_VERBOSE_DEBUG             // Define to enable extra debugging

/** Need to use ODR table. */
extern const odr_reg_map reg_map_ak0991x[AK0991X_REG_MAP_TABLE_SIZE];

log_sensor_state_raw_info log_mag_state_raw_info;

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
sns_rc ak0991x_encode_log_sensor_state_raw(
  void *log, size_t log_size, size_t encoded_log_size, void *encoded_log,
  size_t *bytes_written)
{
  sns_rc rc = SNS_RC_SUCCESS;
#ifdef AK0991X_ENABLE_DIAG_LOGGING
  uint32_t i = 0;
  size_t encoded_sample_size = 0;
  size_t parsed_log_size = 0;
  sns_diag_batch_sample batch_sample = sns_diag_batch_sample_init_default;
  uint8_t arr_index = 0;
  float temp[AK0991X_NUM_AXES];
  pb_float_arr_arg arg = {.arr = (float *)temp, .arr_len = AK0991X_NUM_AXES,
    .arr_index = &arr_index};

  if(NULL == encoded_log || NULL == log || NULL == bytes_written)
  {
    return SNS_RC_FAILED;
  }

  batch_sample.sample.funcs.encode = &pb_encode_float_arr_cb;
  batch_sample.sample.arg = &arg;

  if(!pb_get_encoded_size(&encoded_sample_size, sns_diag_batch_sample_fields,
                          &batch_sample))
  {
    return SNS_RC_FAILED;
  }

  pb_ostream_t stream = pb_ostream_from_buffer(encoded_log, encoded_log_size);
  ak0991x_batch_sample *raw_sample = (ak0991x_batch_sample *)log;

  while(parsed_log_size < log_size &&
        (stream.bytes_written + encoded_sample_size)<= encoded_log_size &&
        i < (uint32_t)(-1))
  {
    arr_index = 0;
    arg.arr = (float *)raw_sample[i].sample;

    batch_sample.sample_type = raw_sample[i].sample_type;
    batch_sample.status = raw_sample[i].status;
    batch_sample.timestamp = raw_sample[i].timestamp;

    if(!pb_encode_tag(&stream, PB_WT_STRING,
                      sns_diag_sensor_state_raw_sample_tag))
    {
      rc = SNS_RC_FAILED;
      break;
    }
    else if(!pb_encode_delimited(&stream, sns_diag_batch_sample_fields,
                                 &batch_sample))
    {
      rc = SNS_RC_FAILED;
      break;
    }

    parsed_log_size += sizeof(ak0991x_batch_sample);
    i++;
  }

  if (SNS_RC_SUCCESS == rc)
  {
    *bytes_written = stream.bytes_written;
  }
#else
  UNUSED_VAR(log);
  UNUSED_VAR(log_size);
  UNUSED_VAR(encoded_log_size);
  UNUSED_VAR(encoded_log);
  UNUSED_VAR(bytes_written);
#endif
  return rc;
}

/**
 * Allocate Sensor State Raw Log Packet
 *
 * @param[i] diag       Pointer to diag service
 * @param[i] log_size   Optional size of log packet to
 *    be allocated. If not provided by setting to 0, will
 *    default to using maximum supported log packet size
 */
void ak0991x_log_sensor_state_raw_alloc(
  log_sensor_state_raw_info *log_raw_info,
  uint32_t log_size)
{
#ifdef AK0991X_ENABLE_DIAG_LOGGING
  uint32_t max_log_size = 0;

  if(NULL == log_raw_info || NULL == log_raw_info->diag ||
     NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid)
  {
    return;
  }

  // allocate memory for sensor state - raw sensor log packet
  max_log_size = log_raw_info->diag->api->get_max_log_size(
       log_raw_info->diag);

  if(0 == log_size)
  {
    // log size not specified.
    // Use max supported log packet size
    log_raw_info->log_size = max_log_size;
  }
  else if(log_size <= max_log_size)
  {
    log_raw_info->log_size = log_size;
  }
  else
  {
    return;
  }

  log_raw_info->log = log_raw_info->diag->api->alloc_log(
    log_raw_info->diag,
    log_raw_info->instance,
    log_raw_info->sensor_uid,
    log_raw_info->log_size,
    SNS_DIAG_SENSOR_STATE_LOG_RAW);

  log_raw_info->log_sample_cnt = 0;
  log_raw_info->bytes_written = 0;
#else
  UNUSED_VAR(log_raw_info);
  UNUSED_VAR(log_size);
#endif
}

/**
 * Submit the Sensor State Raw Log Packet
 *
 * @param[i] log_raw_info   Pointer to logging information
 *       pertaining to the sensor
 * @param[i] batch_complete true if submit request is for end
 *       of batch
 *  */
void ak0991x_log_sensor_state_raw_submit(
  log_sensor_state_raw_info *log_raw_info,
  bool batch_complete)
{
#ifdef AK0991X_ENABLE_DIAG_LOGGING
  ak0991x_batch_sample *sample = NULL;

  if(NULL == log_raw_info || NULL == log_raw_info->diag ||
     NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid ||
     NULL == log_raw_info->log)
  {
    return;
  }

  sample = (ak0991x_batch_sample *)log_raw_info->log;

  if(batch_complete)
  {
    // overwriting previously sample_type for last sample
    if(1 == log_raw_info->batch_sample_cnt)
    {
      sample[0].sample_type =
        SNS_DIAG_BATCH_SAMPLE_TYPE_ONLY;
    }
    else if(1 < log_raw_info->batch_sample_cnt)
    {
      sample[log_raw_info->log_sample_cnt - 1].sample_type =
        SNS_DIAG_BATCH_SAMPLE_TYPE_LAST;
    }
  }

  log_raw_info->diag->api->submit_log(
        log_raw_info->diag,
        log_raw_info->instance,
        log_raw_info->sensor_uid,
        log_raw_info->bytes_written,
        log_raw_info->log,
        SNS_DIAG_SENSOR_STATE_LOG_RAW,
        log_raw_info->log_sample_cnt * log_raw_info->encoded_sample_size,
        ak0991x_encode_log_sensor_state_raw);

  log_raw_info->log = NULL;
#else
  UNUSED_VAR(log_raw_info);
  UNUSED_VAR(batch_complete);
#endif
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
#ifdef AK0991X_ENABLE_DIAG_LOGGING

  if(NULL == log_raw_info || NULL == log_raw_info->diag ||
     NULL == log_raw_info->instance || NULL == log_raw_info->sensor_uid ||
     NULL == raw_data || NULL == log_raw_info->log)
  {
    return SNS_RC_FAILED;
  }

  if( (log_raw_info->bytes_written + sizeof(ak0991x_batch_sample)) >
     log_raw_info->log_size)
  {
    // no more space in log packet
    // submit and allocate a new one
    ak0991x_log_sensor_state_raw_submit(log_raw_info, false);
    ak0991x_log_sensor_state_raw_alloc(log_raw_info, 0);
  }

  if(NULL == log_raw_info->log)
  {
    rc = SNS_RC_FAILED;
  }
  else
  {
    ak0991x_batch_sample *sample =
        (ak0991x_batch_sample *)log_raw_info->log;

    if(0 == log_raw_info->batch_sample_cnt)
    {
      sample[log_raw_info->log_sample_cnt].sample_type =
        SNS_DIAG_BATCH_SAMPLE_TYPE_FIRST;
    }
    else
    {
      sample[log_raw_info->log_sample_cnt].sample_type =
        SNS_DIAG_BATCH_SAMPLE_TYPE_INTERMEDIATE;
    }

    sample[log_raw_info->log_sample_cnt].timestamp = timestamp;

    sns_memscpy(sample[log_raw_info->log_sample_cnt].sample,
                sizeof(sample[log_raw_info->log_sample_cnt].sample),
                raw_data,
                sizeof(sample[log_raw_info->log_sample_cnt].sample));

    sample[log_raw_info->log_sample_cnt].status = status;

    log_raw_info->bytes_written += sizeof(ak0991x_batch_sample);

    log_raw_info->log_sample_cnt++;
    log_raw_info->batch_sample_cnt++;
  }
#else
  UNUSED_VAR(log_raw_info);
  UNUSED_VAR(raw_data);
  UNUSED_VAR(timestamp);
  UNUSED_VAR(status);
#endif
  return rc;
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
static sns_rc ak0991x_com_read_wrapper(sns_sync_com_port_service * scp_service,
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
sns_rc ak0991x_com_write_wrapper(sns_sensor_instance *const this,
                                 sns_sync_com_port_service * scp_service,
                                 sns_sync_com_port_handle *port_handle,
                                 uint32_t reg_addr,
                                 uint8_t *buffer,
                                 uint32_t bytes,
                                 uint32_t *xfer_bytes,
                                 bool save_write_time )
{

  sns_port_vector port_vec;
  port_vec.buffer = buffer;
  port_vec.bytes = bytes;
  port_vec.is_write = true;
  port_vec.reg_addr = reg_addr;

#ifdef AK0991X_VERBOSE_DEBUG
  if( this )
  {
    ak0991x_instance_state *state = (ak0991x_instance_state *)(this->state->state);
    sns_diag_service *diag = state->diag_service;
    if( diag )
    {
      AK0991X_INST_PRINT(LOW, this,
                         "ak0991x write reg:0x%x val[0]:0x%x, bytes %d",
                      reg_addr, (uint32_t) buffer[0], bytes);
    }
  }
#else
  UNUSED_VAR( this );
#endif
  return scp_service->api->sns_scp_register_rw(port_handle,
                                               &port_vec,
                                               1,
                                               save_write_time,
                                               xfer_bytes);
}

/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_device_sw_reset(sns_sensor_instance *const this,
                               sns_sync_com_port_service * scp_service,
                               sns_sync_com_port_handle *port_handle,
                               sns_diag_service *diag )
{
  uint8_t  buffer[1];
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  UNUSED_VAR(diag);

  buffer[0] = AK0991X_SOFT_RESET;
  rv = ak0991x_com_write_wrapper(this,
                                 scp_service,
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
  sns_busy_wait(sns_convert_ns_to_ticks(1000000LL));

  return SNS_RC_SUCCESS;
}

/**
 * Sets Mag ODR, range and sensitivity.
 *
 * @param[i] this        Current instance
 * @param[i] force_off   Turn device off rather than use instance state
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
static
sns_rc ak0991x_set_mag_config(sns_sensor_instance *const this,
                              bool force_off )
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)(this->state->state);
  sns_sync_com_port_service * scp_service = state->scp_service;
  sns_sync_com_port_handle *port_handle = state->com_port_info.port_handle;
  uint32_t xfer_bytes;
  uint8_t  buffer[2];
  ak0991x_mag_odr desired_odr = state->mag_info.desired_odr;
  akm_device_type device_select = state->mag_info.device_select;
  uint16_t cur_wmk = state->mag_info.cur_wmk;

  if( force_off )
  {
    desired_odr = AK0991X_MAG_ODR_OFF;
  }
#ifdef AK0991X_VERBOSE_DEBUG
  int i;
  static const struct {
    ak0991x_mag_odr odr;
    char* name;
  } odr_debug_string_map[] =
    {
      {AK0991X_MAG_ODR_OFF, "off" },
      {AK0991X_MAG_ODR_SNG_MEAS, "single measurment" },
      {AK0991X_MAG_ODR10, "10Hz" },
      {AK0991X_MAG_ODR20, "20Hz" },
      {AK0991X_MAG_ODR50, "50Hz" },
      {AK0991X_MAG_ODR100, "100Hz" },
      {AK0991X_MAG_ODR200, "200Hz" },
      {AK0991X_MAG_ODR1, "1Hz" },
      {AK0991X_MAG_SELFTEST, "selftest" },
      {AK0991X_MAG_FUSEROM, "fuserom" },
    };
  for( i = 0; i < ARR_SIZE( odr_debug_string_map ); i++ )
  {
    if( odr_debug_string_map[i].odr == desired_odr )
    {
      break;
    }
  }
  if( i < ARR_SIZE( odr_debug_string_map ) )
  {
    AK0991X_INST_PRINT(LOW, this,
                       "set_mag_config: Mode:%d, desire_ODR:%d dev:%d wmk:%d force_off:%d",
                                  odr_debug_string_map[i].odr, desired_odr, device_select,
                       cur_wmk, force_off);
  }
  else
  {
    AK0991X_INST_PRINT(LOW, this, "set_mag_config: INVALID ODR!: 0x%x dev:%d wmk:%d",
                                  desired_odr, device_select,
                                  cur_wmk );

  }
#else
  AK0991X_INST_PRINT(LOW, this, "set_mag_config: ODR: %d dev:%d wmk:%d",
                                desired_odr, device_select,
                                cur_wmk );
#endif

  sns_rc rv = SNS_RC_SUCCESS;
  // Set mag config for S4S
  rv = ak0991x_s4s_set_mag_config(this);
  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  // Configure control register 1
  if ((device_select == AK09912) || (device_select == AK09915C) || (device_select == AK09915D) || (device_select == AK09917))
  {
    if (device_select == AK09912)
    {
      buffer[0] = 0x0
        | (state->mag_info.nsf << 5); // NSF bit
    }
    else
    {
      buffer[0] = 0x0
        | (state->mag_info.nsf << 5) // NSF bit
        | cur_wmk;                   // WM[4:0] bits
    }
  }

  // Configure control register 2
  if ((device_select == AK09915C) || (device_select == AK09915D) || (device_select == AK09917))
  {
    buffer[1] = 0x0
      | ((uint8_t)state->mag_info.use_fifo << 7) // FIFO bit
      | (state->mag_info.sdr << 6)               // SDR bit
      | (uint8_t)desired_odr;                    // MODE[4:0] bits
  }
  else
  {
    buffer[1] = 0x0
      | (uint8_t)desired_odr; // MODE[4:0] bits
  }

  return ak0991x_com_write_wrapper(this,
                                   scp_service,
                                   port_handle,
                                   AKM_AK0991X_REG_CNTL1,
                                   buffer,
                                   2,
                                   &xfer_bytes,
                                   false);
}

/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_start_mag_streaming(sns_sensor_instance *const this )
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)(this->state->state);
  sns_rc rv;

  if(state->ascp_xfer_in_progress)
  {
    state->config_mag_after_ascp_xfer = true;
    return SNS_RC_SUCCESS;
  }

  // Enable Mag Streaming

  //Transit to Power-down mode first and then transit to other modes.
  rv = ak0991x_set_mag_config(this, true);

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  // Wait at least 100usec after power down-mode before setting another mode.
  sns_busy_wait(sns_convert_ns_to_ticks(AK0991X_TWAIT_USEC * 1000 * 2));

  rv = ak0991x_set_mag_config(this, false);

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  // check last timestamp
  sns_time now = sns_get_system_time();
  if( state->pre_timestamp > now ){
    AK0991X_INST_PRINT(LOW, this, "negative timestamp detected!!! Keep using pre_timestamp.");
  }else{
    state->pre_timestamp = now;
  }

  sns_time meas_usec;
  ak0991x_get_meas_time(state->mag_info.device_select, state->mag_info.sdr, &meas_usec);
  state->measurement_time = sns_convert_ns_to_ticks(meas_usec * 1000);
  state->this_is_first_data = true;
  state->mag_info.curr_odr = state->mag_info.desired_odr;
  state->force_fifo_read_till_wm = false;
  state->called_handle_timer_reg_event = false;

  return SNS_RC_SUCCESS;
}

/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_stop_mag_streaming(sns_sensor_instance *const this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)(this->state->state);
  sns_rc rv;

  // Disable Mag Streaming

  rv = ak0991x_set_mag_config(this, true );

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

#ifdef AK0991X_ENABLE_FUSE
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
static sns_rc ak0991x_read_asa(sns_sensor_instance *const this,
                               sns_sync_com_port_service * scp_service,
                               sns_sync_com_port_handle *port_handle,
                               sns_diag_service *diag,
                               uint8_t *asa)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint8_t  buffer[1];
  uint32_t xfer_bytes;

  UNUSED_VAR(diag);


  buffer[0] = AK0991X_MAG_FUSEROM;
  // Set Fuse ROM access mode
  rv = ak0991x_com_write_wrapper(this,
                                 scp_service,
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
    ak0991x_com_write_wrapper(this,
                              scp_service,
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
  rv = ak0991x_com_write_wrapper(this,
                                 scp_service,
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
#endif

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
 * Comvert buffered data to mag data.
 *
 * @param[i] this              reference to the instance
 * @param[i] buffer            mag register data
 * @param[o] out               output 16bit mag data
 *
 */
static void ak0991x_get_adjusted_mag_data(sns_sensor_instance *const this, uint8_t *const buffer, int16_t *out)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;

  for(int i=0; i<AK0991X_NUM_AXES; i++)
  {
    if (state->mag_info.device_select == AK09917)
    {
      out[i] = (int16_t)((((int16_t)buffer[i*2] << 8) & 0xFF00) | (int16_t)buffer[i*2 + 1]);
    }
    else
    {
      out[i] = (int16_t)((((int16_t)buffer[i*2 + 1] << 8) & 0xFF00) | (int16_t)buffer[i*2]);
    }
    // sensitivity adjustment
    out[0] = (int16_t)(out[i] * state->mag_info.sstvt_adj[i]);
  }
}

/**
 * Run a hardware self-test
 * @param[i]            reference to the instance
 * @param[o]            error code
 *
 * @return sns_rc
 * SNS_RC_FAILED
 * SNS_RC_SUCCESS
 */
sns_rc ak0991x_hw_self_test(sns_sensor_instance *const this,
                            uint32_t *err)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;
  sns_time usec_time_for_measure;
#ifdef AK0991X_ENABLE_FUSE
  uint8_t  asa[AK0991X_NUM_SENSITIVITY];
#endif
  uint8_t  buffer[AK0991X_NUM_DATA_ST1_TO_ST2];
  int16_t  data[3];
  uint8_t  sdr = 0;

  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;

  sns_diag_service *diag = state->diag_service;

  // Initialize error code
  *err = 0;

  // Reset device
  rv = ak0991x_device_sw_reset(this,
                               state->scp_service,
                               state->com_port_info.port_handle,
                               diag);

  if (rv != SNS_RC_SUCCESS)
  {
    *err = ((TLIMIT_NO_RESET) << 16);
    goto TEST_SEQUENCE_FAILED;
  }


  /** Step 1
   *   If the device has FUSE ROM, test the sensitivity value
   **/
#ifdef AK0991X_ENABLE_FUSE
  if ((state->mag_info.device_select == AK09911) || (state->mag_info.device_select == AK09912))
  {
    rv = ak0991x_read_asa(this,
                          state->scp_service,
                          state->com_port_info.port_handle,
                          diag,
                          asa);


    if (rv != SNS_RC_SUCCESS)
    {
      *err = ((TLIMIT_NO_READ_ASA) << 16);
      goto TEST_SEQUENCE_FAILED;
    }

    AKM_FST(TLIMIT_NO_ASAX, asa[0], TLIMIT_LO_ASAX, TLIMIT_HI_ASAX, err);
    AKM_FST(TLIMIT_NO_ASAY, asa[1], TLIMIT_LO_ASAY, TLIMIT_HI_ASAY, err);
    AKM_FST(TLIMIT_NO_ASAZ, asa[2], TLIMIT_LO_ASAZ, TLIMIT_HI_ASAZ, err);
  }
#endif // AK0991X_ENABLE_FUSE

  /** Step 2
   *   Start self test
   **/
  buffer[0] = AK0991X_MAG_SELFTEST;
  rv = ak0991x_com_write_wrapper(this,
                                 state->scp_service,
                                 state->com_port_info.port_handle,
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

  rv = ak0991x_get_meas_time(state->mag_info.device_select, sdr, &usec_time_for_measure);

  if(rv != SNS_RC_SUCCESS)
  {
    *err = (TLIMIT_NO_INVALID_ID << 16);
    goto TEST_SEQUENCE_FAILED;
  }

  // To ensure that measurement is finished, wait for double as typical
  sns_busy_wait(sns_convert_ns_to_ticks(usec_time_for_measure * 1000 * 2));

  /** Step 3
   *   Read and check data
   **/
  rv = ak0991x_com_read_wrapper(state->scp_service,
                                state->com_port_info.port_handle,
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

  ak0991x_get_adjusted_mag_data(this, buffer, &data[1]);  // data[0] is the ST1 register.

#ifdef AK0991X_ENABLE_ALL_DEVICES
  // check read value
  if (state->mag_info.device_select == AK09918)
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09918, TLIMIT_HI_SLF_RVHX_AK09918,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09918, TLIMIT_HI_SLF_RVHY_AK09918,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09918, TLIMIT_HI_SLF_RVHZ_AK09918,
            err);
  }
  else if (state->mag_info.device_select == AK09917)
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09917, TLIMIT_HI_SLF_RVHX_AK09917,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09917, TLIMIT_HI_SLF_RVHY_AK09917,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09917, TLIMIT_HI_SLF_RVHZ_AK09917,
            err);
  }
  else if ((state->mag_info.device_select == AK09916C) || (state->mag_info.device_select == AK09916D))
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09916, TLIMIT_HI_SLF_RVHX_AK09916,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09916, TLIMIT_HI_SLF_RVHY_AK09916,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09916, TLIMIT_HI_SLF_RVHZ_AK09916,
            err);
  }
  else if ((state->mag_info.device_select == AK09915C) || (state->mag_info.device_select == AK09915D))
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09915, TLIMIT_HI_SLF_RVHX_AK09915,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09915, TLIMIT_HI_SLF_RVHY_AK09915,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09915, TLIMIT_HI_SLF_RVHZ_AK09915,
            err);
  }
  else if (state->mag_info.device_select == AK09913)
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09913, TLIMIT_HI_SLF_RVHX_AK09913,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09913, TLIMIT_HI_SLF_RVHY_AK09913,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09913, TLIMIT_HI_SLF_RVHZ_AK09913,
            err);
  }
  else if (state->mag_info.device_select == AK09912)
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09912, TLIMIT_HI_SLF_RVHX_AK09912,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09912, TLIMIT_HI_SLF_RVHY_AK09912,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09912, TLIMIT_HI_SLF_RVHZ_AK09912,
            err);
  }
  else if (state->mag_info.device_select == AK09911)
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
#else
  AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX, TLIMIT_HI_SLF_RVHX, err);
  AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY, TLIMIT_HI_SLF_RVHY, err);
  AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ, TLIMIT_HI_SLF_RVHZ, err);
#endif // AK0991X_ENABLE_ALL_DEVICES

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

#if defined(AK0991X_ENABLE_DRI) || defined(AK0991X_ENABLE_FIFO)
/**
 * Read ST1(10h) register data.
 *
 * @param[i] state                    Instance state
 * @param[o] buffer                   st1 register data
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
static sns_rc ak0991x_read_st1(ak0991x_instance_state *state,
                               uint8_t *buffer)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  rv = ak0991x_com_read_wrapper(state->scp_service,
                                state->com_port_info.port_handle,
                                AKM_AK0991X_REG_ST1,
                                buffer,
                                1,
                                &xfer_bytes);

  if (xfer_bytes != 1)
  {
    rv = SNS_RC_FAILED;
  }

  return rv;
}
#endif // AK0991X_ENABLE_DRI || AK0991X_ENABLE_FIFO

/**
 * Read HXL(11h) to ST2(18h) register data.
 *
 * @param[i] state                    Instance state
 * @param[i] num_samples              number of samples
 * @param[o] buffer                   st1 register data
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
static sns_rc ak0991x_read_hxl_st2(ak0991x_instance_state *state,
                                   uint16_t num_samples,
                                   uint8_t *buffer)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  rv = ak0991x_com_read_wrapper(state->scp_service,
                                state->com_port_info.port_handle,
                                AKM_AK0991X_REG_HXL,
                                buffer,
                                (uint32_t)(num_samples * AK0991X_NUM_DATA_HXL_TO_ST2),
                                &xfer_bytes);

  if (xfer_bytes != (uint32_t)(num_samples * AK0991X_NUM_DATA_HXL_TO_ST2))
  {
    rv = SNS_RC_FAILED;
  }

  return rv;
}

/**
 * Read all data.
 *
 * @param[i] state                    Instance state
 * @param[o] buffer                   read data buffer
 *
 */
static void ak0991x_read_all_data(sns_sensor_instance *const instance,
                                       uint8_t *buffer)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;

  // For Polling + non FIFO mode
  if(!state->mag_info.use_fifo || (state->force_fifo_read_till_wm && state->mag_info.cur_wmk == 0) )
  {
    state->num_samples = 1;
    ak0991x_read_hxl_st2(state,
                         1,
                         &buffer[0]);
    return;
  }

#ifdef AK0991X_ENABLE_FIFO
  // From here for FIFO
  if (state->mag_info.device_select == AK09917)
  {
    //In case of AK09917, Read ST1 register to check FIFO samples
    uint8_t st1_buf = 0;
    if (SNS_RC_SUCCESS == ak0991x_read_st1(state, &st1_buf))
    {
      state->num_samples = st1_buf >> 2;
      AK0991X_INST_PRINT(LOW, instance, "num=%d st1=%x", state->num_samples, st1_buf);
#ifdef AK0991X_ENABLE_S4S
      //To avoid reporting incorrect data in S4S by FIFO+Pollng mode
      if (state->mag_info.use_sync_stream &&
          (state->num_samples < state->mag_info.cur_wmk + 1))
      {
        state->num_samples = state->mag_info.cur_wmk + 1;
      }
#endif
      if (state->num_samples > 0)
      {
        /*Number of bytes reading from sync-com-port should be less than AK0991X_MAX_FIFO_SIZE*/
        if ((state->num_samples * AK0991X_NUM_DATA_HXL_TO_ST2) > AK0991X_MAX_FIFO_SIZE)
        {
          SNS_INST_PRINTF(ERROR, instance,
              "FIFO size should not be greater than AK0991X_MAX_FIFO_SIZE."
              "So, num_samples to read limiting to max value");
          state->num_samples = (AK0991X_MAX_FIFO_SIZE / AK0991X_NUM_DATA_HXL_TO_ST2);
        }

        // Read fifo buffer(HXL to ST2 register)
        if (SNS_RC_SUCCESS != ak0991x_read_hxl_st2(state, state->num_samples, &buffer[0]))
        {
          SNS_INST_PRINTF(ERROR, instance, "Error in reading the FIFO buffer");
        }
      }
    }
    else
    {
      SNS_INST_PRINTF(ERROR, instance, "Error in reading length of the FIFO");
    }
  }
  else
  {
    //Continue reading until fifo buffer is clear
    //because there is no way to check FIFO samples for AK09915C/D.
    uint8_t i;
    for (i = 0; i < state->mag_info.max_fifo_size; i++)
    {
      //Read fifo buffer(HXL to ST2 register)
      if (SNS_RC_SUCCESS != ak0991x_read_hxl_st2(state,
                                                 1,
                                                 &buffer[i * AK0991X_NUM_DATA_HXL_TO_ST2]))
      {
        SNS_INST_PRINTF(ERROR, instance, "Error in reading the FIFO buffer");
      }

      if ((buffer[i * AK0991X_NUM_DATA_HXL_TO_ST2 + 7] & AK0991X_INV_FIFO_DATA) != 0)
      {
        //fifo buffer is clear
        break;
      }
      else
      {
        state->num_samples++;
      }
    }
  }

  // update num_samples when polling+FIFO mode
  if( state->mag_info.use_fifo && state->force_fifo_read_till_wm ){
    state->num_samples = state->mag_info.cur_wmk+1;
  }

#endif // AK0991X_ENABLE_FIFO
}

/**
 * see sns_ak0991x_hal.h
 */
sns_rc ak0991x_set_sstvt_adj(
#ifdef AK0991X_ENABLE_FUSE
                             sns_sync_com_port_service* scp_service,
                             sns_sync_com_port_handle *port_handle,
                             sns_diag_service *diag,
#endif
                             akm_device_type device_select,
                             float *sstvt_adj)
{
  sns_rc  rv = SNS_RC_SUCCESS;
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

#ifdef AK0991X_ENABLE_FUSE
  uint8_t buffer[AK0991X_NUM_SENSITIVITY];
  rv = ak0991x_read_asa(NULL, scp_service,port_handle, diag, buffer);

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
#endif

  return rv;
}

/**
 * Gets current ODR.
 *
 * @param[i] curr_odr              Current ODR.
 *
 * @return current ODR
 */
float ak0991x_get_mag_odr(ak0991x_mag_odr curr_odr)
{
  float odr = 0.0;
  int8_t idx;

  for (idx = 0; idx < ARR_SIZE(reg_map_ak0991x); idx++)
  {
    if (curr_odr == reg_map_ak0991x[idx].mag_odr_reg_value
        &&
        curr_odr != AK0991X_MAG_ODR_OFF)
    {
      odr = reg_map_ak0991x[idx].odr;
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
                                      log_sensor_state_raw_info *log_mag_state_raw_info
                                      )
{
  UNUSED_VAR(event_service);

  float ipdata[TRIAXIS_NUM] = {0}, opdata_raw[TRIAXIS_NUM] = {0};
  int16_t lsbdata[TRIAXIS_NUM] = {0};

  uint8_t i = 0;
  sns_std_sensor_sample_status status;

  /*
  AK0991X_INST_PRINT(LOW, instance, "fac_cal_corr_mat 00=%d 11=%d 22=%d, fac_cal_bias0=%d 1=%d 2=%d",
        (uint32_t)state->mag_registry_cfg.fac_cal_corr_mat.e00,
        (uint32_t)state->mag_registry_cfg.fac_cal_corr_mat.e11,
        (uint32_t)state->mag_registry_cfg.fac_cal_corr_mat.e22,
        (uint32_t)state->mag_registry_cfg.fac_cal_bias[0],
        (uint32_t)state->mag_registry_cfg.fac_cal_bias[1],
        (uint32_t)state->mag_registry_cfg.fac_cal_bias[2]);
   */

  ak0991x_get_adjusted_mag_data(instance, mag_sample, lsbdata);

  // Check magnetic sensor overflow
  if ((mag_sample[7] & AK0991X_HOFL_BIT) != 0)
  {
    AK0991X_INST_PRINT(LOW, instance, "sensor overflow: HOFL_BIT=1, use previous data.");
    status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
    lsbdata[TRIAXIS_X] =state->pre_lsbdata[TRIAXIS_X];
    lsbdata[TRIAXIS_Y] =state->pre_lsbdata[TRIAXIS_Y];
    lsbdata[TRIAXIS_Z] =state->pre_lsbdata[TRIAXIS_Z];
  }
  else
  {
    status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;
    state->pre_lsbdata[TRIAXIS_X] = lsbdata[TRIAXIS_X];
    state->pre_lsbdata[TRIAXIS_Y] = lsbdata[TRIAXIS_Y];
    state->pre_lsbdata[TRIAXIS_Z] = lsbdata[TRIAXIS_Z];
  }
  ipdata[TRIAXIS_X] = lsbdata[TRIAXIS_X] * state->mag_info.resolution;
  ipdata[TRIAXIS_Y] = lsbdata[TRIAXIS_Y] * state->mag_info.resolution;
  ipdata[TRIAXIS_Z] = lsbdata[TRIAXIS_Z] * state->mag_info.resolution;

  AK0991X_INST_PRINT(LOW, instance, "timestamp %u Mag[LSB] %d,%d,%d",
      (uint32_t)timestamp,
      (int16_t)(lsbdata[TRIAXIS_X]),
      (int16_t)(lsbdata[TRIAXIS_Y]),
      (int16_t)(lsbdata[TRIAXIS_Z]));

  // axis conversion
  for (i = 0; i < TRIAXIS_NUM; i++)
  {
    opdata_raw[state->axis_map[i].opaxis] = (state->axis_map[i].invert ? -1.0 : 1.0) *
      ipdata[state->axis_map[i].ipaxis];
    /*AK0991X_INST_PRINT(LOW, instance, "ip=%d op=%d invert=%d",
        (uint32_t)state->axis_map[i].ipaxis,
        (uint32_t)state->axis_map[i].opaxis,
        (uint32_t)state->axis_map[i].invert);*/
  }

  // factory calibration
  vector3 opdata_cal = sns_apply_calibration_correction_3(
      make_vector3_from_array(opdata_raw),
      make_vector3_from_array(state->mag_registry_cfg.fac_cal_bias),
      state->mag_registry_cfg.fac_cal_corr_mat);

  pb_send_sensor_stream_event(instance,
                              &state->mag_info.suid,
                              timestamp,
                              SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT,
                              status,
                              opdata_cal.data,
                              ARR_SIZE(opdata_cal.data),
                              state->encoded_mag_event_len);

  // Log raw uncalibrated sensor data
  ak0991x_log_sensor_state_raw_add(
    log_mag_state_raw_info,
    opdata_raw,
    timestamp,
    status);


}

void ak0991x_process_mag_data_buffer(sns_sensor_instance *instance,
                                     sns_time            first_timestamp,
                                     sns_time            sample_interval_ticks,
                                     uint8_t             *fifo_start,
                                     size_t              num_bytes
)
{
  uint16_t num_samples_sets = 0;
  uint32_t i;
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
  sns_service_manager *service_manager = instance->cb->get_service_manager(instance);
  sns_event_service *event_service =
    (sns_event_service*)service_manager->get_service(service_manager, SNS_EVENT_SERVICE);
//#ifdef AK0991X_ENABLE_DIAG_LOGGING
//  log_sensor_state_raw_info log_mag_state_raw_info;
//
//  // Allocate Sensor State Raw log packets for mag
//  sns_memzero(&log_mag_state_raw_info, sizeof(log_mag_state_raw_info));
//  log_mag_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;
//  ak0991x_log_sensor_state_raw_alloc(&log_mag_state_raw_info, 0);
//#endif
  for(i = 0; i < num_bytes; i += 8)
  {
    sns_time timestamp = first_timestamp + (num_samples_sets++ * sample_interval_ticks);
    ak0991x_handle_mag_sample(&fifo_start[i],
                              timestamp,
                              instance,
                              event_service,
                              state,
                              &log_mag_state_raw_info);
  }
}

/** See ak0991x_hal.h */
void ak0991x_send_fifo_flush_done(sns_sensor_instance *const instance)
{
#ifdef AK0991X_ENABLE_FIFO
  sns_service_manager *mgr = instance->cb->get_service_manager(instance);
  sns_event_service *e_service = (sns_event_service*)mgr->get_service(mgr,SNS_EVENT_SERVICE);
  sns_sensor_event *event = e_service->api->alloc_event(e_service, instance, 0);
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;

  if(NULL != event)
  {
    event->message_id = SNS_STD_MSGID_SNS_STD_FLUSH_EVENT;
    event->event_len = 0;
    event->timestamp = sns_get_system_time();

    e_service->api->publish_event(e_service, instance, event, &state->mag_info.suid);
  }
#else
  UNUSED_VAR(instance);
#endif // AK0991X_ENABLE_FIFO
}

static void ak0991x_validate_timestamp(sns_sensor_instance *const instance)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
  sns_time now = sns_get_system_time();
  uint8_t num_samples = state->num_samples;

#ifdef AK0991X_ENABLE_S4S
  // for S4S, no need to validate timestamp????
  if(state->mag_info.use_sync_stream){
    state->interrupt_timestamp = now;
    state->averaged_interval = (state->interrupt_timestamp - state->pre_timestamp) / num_samples;
    return;
  }
#endif // AK0991X_ENABLE_S4S

  bool enable_averaging = false;
  bool update_interrupt_timestamp = false;
  float averaging_weight;

  if(state->irq_info.detect_irq_event){
    state->interrupt_timestamp = state->irq_event_time; // for DRI interrupt
  }else{
    state->interrupt_timestamp = now; // for Polling or Flush
  }

  if(state->this_is_first_data){
    AK0991X_INST_PRINT(LOW, instance, "this_is_first_data");

    state->mag_info.data_count = 0;
    if(state->irq_info.detect_irq_event){
      if(state->mag_info.use_fifo && (num_samples > 1) ){
        // DRI + FIFO and more than 1 data
        state->averaged_interval = (state->interrupt_timestamp - state->pre_timestamp - state->measurement_time) / (num_samples - 1);
      }else{
        // DRI
        state->averaged_interval = ak0991x_get_sample_interval(state->mag_info.curr_odr);
      }
    }else{
      // Polling / Polling + FIFO
      state->averaged_interval = (state->interrupt_timestamp - state->pre_timestamp) / num_samples;
    }
  }else if(state->data_over_run){
    AK0991X_INST_PRINT(LOW, instance, "data over run detected");
    // no calculate averaged_interval
  }else if((state->mag_info.use_fifo) &&
      (state->interrupt_timestamp - state->pre_timestamp > state->averaged_interval * state->mag_info.max_fifo_size * 1.2 )){
    // no calculate averaged_interval
    AK0991X_INST_PRINT(LOW, instance, "possible data over run detected");
  }else{
    enable_averaging = true;
    if(state->irq_info.detect_irq_event)
    {
      // irq_event_time is the time when the FIFO buffer reaches the WM.
      update_interrupt_timestamp = true;
      num_samples = (uint8_t)state->mag_info.cur_wmk+1;
    }
    else
    {
      // timer event or fifo_flush
      if(!state->fifo_flush_in_progress){
        update_interrupt_timestamp = true;
      }
    }
  }
  if(num_samples>0){
    if(enable_averaging){
      if(state->mag_info.use_dri){
        averaging_weight = ( (state->mag_info.data_count > 5) && (!state->fifo_flush_in_progress) ) ? 0.95 : 0.05;
      }else{
        averaging_weight = 0.95;
      }
      state->averaged_interval = state->averaged_interval * averaging_weight +
         ((state->interrupt_timestamp - state->pre_timestamp) / num_samples) * (1.0 - averaging_weight);
    }
    if(update_interrupt_timestamp){
      state->interrupt_timestamp = state->pre_timestamp + (state->averaged_interval * num_samples);
    }
    AK0991X_INST_PRINT(LOW, instance, "num=%d averaged_interval=%u data_count=%d" ,num_samples, (uint32_t)state->averaged_interval,(uint16_t)state->mag_info.data_count);
    state->mag_info.data_count++;
  }else{
    AK0991X_INST_PRINT(LOW, instance, "ERROR: num_samples=0 !!!");
  }
}

bool ak0991x_is_drdy(sns_sensor_instance *const instance)
{
#ifdef AK0991X_ENABLE_DRI
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
  uint8_t st1_buf;

  ak0991x_read_st1(state, &st1_buf);

  state->num_samples = 1;

  // set num_samples
  if( state->mag_info.use_fifo ){
    if(state->mag_info.device_select == AK09917)
    {
      state->num_samples = st1_buf>>2;
    }else{
      state->num_samples = state->mag_info.cur_wmk + 1;
    }
  }

  // check data over run
  state->data_over_run = (st1_buf & AK0991X_DOR_BIT) ? true : false;

  // return drdy status
  return (st1_buf & AK0991X_DRDY_BIT) ? true : false;
#else
  UNUSED_VAR(instance);
  return false;
#endif
}

void ak0991x_flush_fifo(sns_sensor_instance *const instance)
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)instance->state->state;
  sns_service_manager    *service_manager =
    instance->cb->get_service_manager(instance);
  sns_event_service *event_service =
    (sns_event_service *)service_manager->get_service(service_manager, SNS_EVENT_SERVICE);

  uint32_t i;
  sns_time timestamp;
  uint8_t buffer[AK0991X_MAX_FIFO_SIZE];

#ifdef AK0991X_ENABLE_DIAG_LOGGING
  sns_diag_service          *diag = state->diag_service;
  log_sensor_state_raw_info log_mag_state_raw_info;

  sns_memzero(&log_mag_state_raw_info, sizeof(log_mag_state_raw_info));
  log_mag_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;
  log_mag_state_raw_info.diag = diag;
  log_mag_state_raw_info.instance = instance;
  log_mag_state_raw_info.sensor_uid = &state->mag_info.suid;
#endif

  ak0991x_read_all_data(instance, &buffer[0]);

  ak0991x_validate_timestamp(instance);
  // Allocate log packet memory only if there are samples to flush
  ak0991x_log_sensor_state_raw_alloc(&log_mag_state_raw_info, 0);

  for (i = 0; i < state->num_samples; i++)
  {
    // flush event trigger is IRQ
    if (state->irq_info.detect_irq_event)
    {
      if(i <= state->mag_info.cur_wmk){
        // data till the WM
        timestamp = state->interrupt_timestamp -
          (state->averaged_interval * (state->mag_info.cur_wmk - i));
      }else{
        // in case there are some data buffered in FIFO.
        timestamp = state->interrupt_timestamp +
          (state->averaged_interval * (i - state->mag_info.cur_wmk));
      }
    }
    else
    {
      timestamp = state->pre_timestamp + (state->averaged_interval * (i + 1));
    }

    ak0991x_handle_mag_sample(&buffer[AK0991X_NUM_DATA_HXL_TO_ST2 * i],
                              timestamp,
                              instance,
                              event_service,
                              state,
                              &log_mag_state_raw_info
                              );
  }

  if (state->num_samples != 0)
  {
    state->pre_timestamp = timestamp;
    state->this_is_first_data = false;

#ifdef AK0991X_ENABLE_DIAG_LOGGING
    ak0991x_log_sensor_state_raw_submit(&log_mag_state_raw_info, true);
#else
    UNUSED_VAR(log_mag_state_raw_info);
#endif
  }
  state->force_fifo_read_till_wm = false;
}

void ak0991x_handle_interrupt_event(sns_sensor_instance *const instance)
{
#ifdef AK0991X_ENABLE_DRI
  uint8_t  buffer[AK0991X_MAX_FIFO_SIZE];
  uint32_t enc_len;
  uint16_t num_of_bytes;

  ak0991x_instance_state *state =
    (ak0991x_instance_state *)instance->state->state;

  sns_port_vector async_read_msg;

  if (state->mag_info.use_fifo)
  {
    num_of_bytes = AK0991X_NUM_DATA_HXL_TO_ST2 * state->num_samples;
  }
  else
  {
    state->num_samples = 1;
    num_of_bytes = AK0991X_NUM_DATA_HXL_TO_ST2;
  }

  ak0991x_validate_timestamp(instance);

  // Compose the async com port message
  async_read_msg.bytes = num_of_bytes;
  async_read_msg.reg_addr = AKM_AK0991X_REG_HXL;
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

  state->ascp_xfer_in_progress++;
#else
  UNUSED_VAR(instance);
#endif
}

/** See sns_ak0991x_hal.h */
sns_rc ak0991x_send_config_event(sns_sensor_instance *const instance)
{
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)instance->state->state;

  sns_std_sensor_physical_config_event phy_sensor_config =
    sns_std_sensor_physical_config_event_init_default;

  char *operating_mode;

  switch (state->mag_info.device_select)
  {
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09911)
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
    break;
#endif
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09912)
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
    break;
#endif
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09913)
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
    break;
#endif
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09915C)
  case AK09915C:

    if (state->mag_info.sdr == 1)
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
    break;
#endif
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09915D)
  case AK09915D:

    if (state->mag_info.sdr == 1)
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
    break;
#endif
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09916C)
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
    break;
#endif
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09916D)
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
    break;
#endif
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09917)
  case AK09917:

    if (state->mag_info.sdr == 0)
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
    phy_sensor_config.active_current = AK09917_HI_PWR;
    phy_sensor_config.has_resolution = true;
    phy_sensor_config.resolution = AK09917_RESOLUTION;
    phy_sensor_config.range_count = 2;
    phy_sensor_config.range[0] = AK09917_MIN_RANGE;
    phy_sensor_config.range[1] = AK09917_MAX_RANGE;
    break;
#endif
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09918)
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
    break;
#endif
  default:
    return SNS_RC_FAILED;
  }

  //Set config event for s4s
  ak0991x_s4s_send_config_event(instance, &phy_sensor_config);

  pb_buffer_arg op_mode_args;
  op_mode_args.buf = operating_mode;
  op_mode_args.buf_len = sizeof(operating_mode);
  phy_sensor_config.operation_mode.funcs.encode = &pb_encode_string_cb;
  phy_sensor_config.operation_mode.arg = &op_mode_args;
  phy_sensor_config.has_sample_rate = true;
  phy_sensor_config.sample_rate = state->mag_req.sample_rate;

  AK0991X_INST_PRINT(LOW, instance,
                     "tx phys config evt: op_mode %u rate %u/100 wm %u sync %u",
                     operating_mode, (uint32_t)(state->mag_req.sample_rate * 100),
                     phy_sensor_config.has_water_mark ? phy_sensor_config.water_mark : 0,
                     phy_sensor_config.stream_is_synchronous);

  pb_send_event(instance,
                sns_std_sensor_physical_config_event_fields,
                &phy_sensor_config,
                sns_get_system_time(),
                SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT,
                &state->mag_info.suid);

  return SNS_RC_SUCCESS;
}

void ak0991x_register_interrupt(sns_sensor_instance *this)
{
#ifdef AK0991X_ENABLE_DRI
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  if(!state->irq_info.is_registered)
  {
    sns_data_stream* data_stream = state->interrupt_data_stream;
    uint8_t buffer[20] = {0};
    sns_request irq_req =
      {
        .message_id = SNS_INTERRUPT_MSGID_SNS_INTERRUPT_REQ,
        .request    = buffer
      };

    irq_req.request_len = pb_encode_request(buffer,
                                            sizeof(buffer),
                                            &state->irq_info.irq_config,
                                            sns_interrupt_req_fields,
                                            NULL);
    if(irq_req.request_len > 0)
    {
      data_stream->api->send_request(data_stream, &irq_req);
      state->irq_info.is_registered = true;
    }
  }
#else
  UNUSED_VAR(this);
#endif //A0991X_ENABLE_DRI
}

void ak0991x_register_timer(sns_sensor_instance *this,
                            bool register_s4s_timer)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;

  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_mgr = (sns_stream_service *)
      service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

  sns_request             timer_req;
  sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
  size_t                  req_len;
  uint8_t                 buffer[20] = {0};
  req_payload.is_periodic = true;
  req_payload.has_priority = true;

#ifdef AK0991X_ENABLE_S4S
  if (state->mag_info.use_sync_stream)
  {
    if (register_s4s_timer)
    {
      sns_time                t_ph_period = sns_convert_ns_to_ticks(
          AK0991X_S4S_INTERVAL_MS * 1000 * 1000);
      req_payload.start_time = sns_get_system_time() - t_ph_period;
      req_payload.start_config.early_start_delta = 0;
      req_payload.start_config.late_start_delta = t_ph_period;
      req_payload.priority = SNS_TIMER_PRIORITY_S4S;
      req_payload.timeout_period = t_ph_period;
    }
    else
    {
      req_payload.start_time = sns_get_system_time() - sns_convert_ns_to_ticks(
          AK0991X_S4S_INTERVAL_MS / (float)state->mag_info.s4s_t_ph * 1000 * 1000);
      req_payload.start_config.early_start_delta = 0;
      req_payload.start_config.late_start_delta = sns_convert_ns_to_ticks(
          2 * AK0991X_S4S_INTERVAL_MS / (float)state->mag_info.s4s_t_ph * 1000 * 1000);
      req_payload.priority = SNS_TIMER_PRIORITY_POLLING;

      if (state->mag_info.use_fifo)
      {
        req_payload.timeout_period = sns_convert_ns_to_ticks(
          AK0991X_S4S_INTERVAL_MS / (float)state->mag_info.s4s_t_ph * (state->mag_info.cur_wmk + 1) * 1000 * 1000);
      }
      else
      {
        req_payload.timeout_period = sns_convert_ns_to_ticks(
          AK0991X_S4S_INTERVAL_MS / (float)state->mag_info.s4s_t_ph * 1000 * 1000);
      }
    }
  }
  else
#endif // AK0991X_ENABLE_S4S
  {
    UNUSED_VAR(register_s4s_timer);
    req_payload.priority = SNS_TIMER_PRIORITY_POLLING;

    if (state->mag_info.use_fifo)
    {
      req_payload.timeout_period = sns_convert_ns_to_ticks(
        1 / state->mag_req.sample_rate * (state->mag_info.cur_wmk + 1) * 1000 * 1000 * 1000);
    }
    else
    {
      req_payload.timeout_period = sns_convert_ns_to_ticks(
        1 / state->mag_req.sample_rate * 1000 * 1000 * 1000);
    }
    //TODO: start time calculation should be similar to above use_sync_stream case
    //If this sensor is doing polling, it would be good to synchronize the Mag polling timer,
    //with other polling timers on the system.
    //For example, that the pressure sensor is polling at 20Hz.
    //It would be good to make sure the mag polling timer and the pressure polling timer are synchronized if possible.
    req_payload.start_time = sns_get_system_time();
  }

  if (state->mag_req.sample_rate != AK0991X_ODR_0)
  {
    AK0991X_INST_PRINT(LOW, this, "timeout_period=%u", (uint32_t)req_payload.timeout_period);
    AK0991X_INST_PRINT(LOW, this, "start_time=%u", (uint32_t)req_payload.start_time);
    AK0991X_INST_PRINT(LOW, this, "late_start_delta=%u", (uint32_t)req_payload.start_config.late_start_delta);

#ifdef AK0991X_ENABLE_S4S
    if (register_s4s_timer)
    {
      if (NULL == state->s4s_timer_data_stream)
      {
        stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                       this,
                                                       state->timer_suid,
                                                       &state->s4s_timer_data_stream
                                                       );
      }
    }
    else
#endif // AK0991X_ENABLE_S4S
    {
      if (NULL == state->timer_data_stream)
      {
        stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                       this,
                                                       state->timer_suid,
                                                       &state->timer_data_stream
                                                       );
      }
    }
 
    req_len = pb_encode_request(buffer,
                                sizeof(buffer),
                                &req_payload,
                                sns_timer_sensor_config_fields,
                                NULL);

    if (req_len > 0)
    {
      timer_req.message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG;
      timer_req.request_len = req_len;
      timer_req.request = buffer;

      /** Send encoded request to Timer Sensor */
#ifdef AK0991X_ENABLE_S4S
      if (register_s4s_timer)
      {
        state->s4s_timer_data_stream->api->send_request(state->s4s_timer_data_stream, &timer_req);
      }
      else
#endif // AK0991X_ENABLE_S4S
      {
        state->timer_data_stream->api->send_request(state->timer_data_stream, &timer_req);
      }
    }
    else
    {
      AK0991X_INST_PRINT(ERROR, this, "Fail to send request to Timer Sensor");
    }
  }
  else
  {
#ifdef AK0991X_ENABLE_S4S
    if (register_s4s_timer)
    {
      state->mag_info.s4s_sync_state = AK0991X_S4S_NOT_SYNCED;
      if (state->s4s_timer_data_stream != NULL)
      {
        stream_mgr->api->remove_stream(stream_mgr, state->s4s_timer_data_stream);
        state->s4s_timer_data_stream = NULL;
      }
    }
    else
#endif
    {
      if (state->timer_data_stream != NULL)
      {
        stream_mgr->api->remove_stream(stream_mgr, state->timer_data_stream);
        state->timer_data_stream = NULL;
        AK0991X_INST_PRINT(LOW, this, "remove timer.");
      }
    }
  }
}

sns_rc ak0991x_get_meas_time( akm_device_type device_select,
                              uint8_t sdr,
                              sns_time* meas_ts )
{
  sns_time usec_time_for_measure;
  if (device_select == AK09918)
  {
    usec_time_for_measure = AK09918_TIME_FOR_MEASURE_US;
  }
  else if (device_select == AK09917)
  {
    if (sdr == 0)
    {
      usec_time_for_measure = AK09917_TIME_FOR_LOW_NOISE_MODE_MEASURE_US;
    }
    else
    {
      usec_time_for_measure = AK09917_TIME_FOR_LOW_POWER_MODE_MEASURE_US;
    }
  }
  else if ((device_select == AK09916C) || (device_select == AK09916D))
  {
    usec_time_for_measure = AK09916_TIME_FOR_MEASURE_US;
  }
  else if ((device_select == AK09915C) || (device_select == AK09915D))
  {
    if (sdr == 1)
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
    return SNS_RC_FAILED;
  }
  *meas_ts = usec_time_for_measure;
  return SNS_RC_SUCCESS;
}

sns_rc ak0991x_reconfig_hw(sns_sensor_instance *this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  sns_rc rv = SNS_RC_SUCCESS;

  if (state->mag_info.desired_odr != AK0991X_MAG_ODR_OFF)
  {
    if ((state->mag_info.use_dri && state->irq_info.is_ready) ||
            (state->mag_info.use_dri && state->dae_if.mag.state == STREAMING) ||
            (!state->mag_info.use_dri))
    {
      AK0991X_INST_PRINT(LOW, this, "start_mag_streaming");
      ak0991x_start_mag_streaming(this);
    }
  }
  else
  {
    AK0991X_INST_PRINT(LOW, this, "ak0991x_stop_mag_streaming");
    rv = ak0991x_stop_mag_streaming(this);

    if (rv != SNS_RC_SUCCESS)
    {
      //state->mag_info.cur_wmk = pre_wmk;
      return rv;
    }
  }
  return rv;
}

/**
 * Runs a communication test - verifies WHO_AM_I, publishes self
 * test event.
 *
 * @param[i] instance    Instance reference
 * @param[i] uid         Sensor UID
 *
 * @return none
 */
static void ak0991x_send_com_test_event(sns_sensor_instance *instance,
                                        sns_sensor_uid *uid, bool test_result,
                                        sns_physical_sensor_test_type test_type)
{
  uint8_t data[1] = {0};
  pb_buffer_arg buff_arg = (pb_buffer_arg)
      { .buf = &data, .buf_len = sizeof(data) };
  sns_physical_sensor_test_event test_event =
    sns_physical_sensor_test_event_init_default;

  test_event.test_passed = test_result;
  test_event.test_type = test_type;
  test_event.test_data.funcs.encode = &pb_encode_string_cb;
  test_event.test_data.arg = &buff_arg;

  pb_send_event(instance,
                sns_physical_sensor_test_event_fields,
                &test_event,
                sns_get_system_time(),
                SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_EVENT,
                uid);
}

/** See sns_ak0991x_hal.h */
void ak0991x_run_self_test(sns_sensor_instance *instance)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)instance->state->state;
  sns_rc rv = SNS_RC_SUCCESS;

  if(state->mag_info.test_info.test_client_present)
  {
    if(state->mag_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_COM)
    {
      uint8_t buffer[AK0991X_NUM_READ_DEV_ID] = {0};
      bool who_am_i_success = false;

      rv = ak0991x_get_who_am_i(state->scp_service,
                                state->com_port_info.port_handle,
                                &buffer[0]);

      if(rv == SNS_RC_SUCCESS
         &&
         buffer[0] == AK0991X_WHOAMI_COMPANY_ID)
      {
        who_am_i_success = true;
        AK0991X_INST_PRINT(LOW, instance, "COM self-test success!!");
      }
      ak0991x_send_com_test_event(instance, &state->mag_info.suid, who_am_i_success,
                                  SNS_PHYSICAL_SENSOR_TEST_TYPE_COM);
    }
    else if(state->mag_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY)
    {
      // Handle factory test. The driver may choose to reject any new
      // streaming/self-test requests when factory test in progress.
    }
    else if(state->mag_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_HW)
    {
      bool hw_success = false;
      uint32_t err;

      rv = ak0991x_hw_self_test(instance, &err);

      if(rv == SNS_RC_SUCCESS
         &&
         err == 0)
      {
        hw_success = true;
        AK0991X_INST_PRINT(LOW, instance, "hw self-test success!!");
      }
      ak0991x_send_com_test_event(instance, &state->mag_info.suid, hw_success,
                                  SNS_PHYSICAL_SENSOR_TEST_TYPE_HW);
    }
    state->mag_info.test_info.test_client_present = false;
  }
}

