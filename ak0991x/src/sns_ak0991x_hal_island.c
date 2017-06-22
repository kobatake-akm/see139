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

/**
 * EDIT HISTORY FOR FILE
 *
 * This section contains comments describing changes made to the module.
 * Notice that changes are listed in reverse chronological order.
 *
 *
 * when         who     what, where, why
 * --------     ---     ------------------------------------------------
 * 04/04/17     AKM     Fix sensitivity configuration.
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
#include "sns_timer.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_async_com_port_pb_utils.h"

#include "sns_std_sensor.pb.h"

#include "sns_diag_service.h"
#include "sns_diag.pb.h"
#include "sns_printf.h"

#include "sns_cal_util.h"

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
static sns_rc ak0991x_com_write_wrapper(sns_sensor_instance *const this,
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
      SNS_INST_PRINTF(LOW, this,
                      "ak0991x write reg:0x%x val:0x%x, bytes %d",
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
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;
  uint8_t  buffer;
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
    SNS_INST_PRINTF(LOW, this, "set_mag_config: ODR: %s dev:%d wmk:%d",
                                  odr_debug_string_map[i].name, device_select,
                                  cur_wmk );
  }
  else
  {
    SNS_INST_PRINTF(LOW, this, "set_mag_config: INVALID ODR!: 0x%x dev:%d wmk:%d",
                                  desired_odr, device_select,
                                  cur_wmk );

  }
#else
  SNS_INST_PRINTF(LOW, this, "set_mag_config: ODR: %d dev:%d wmk:%d",
                                desired_odr, device_select,
                                cur_wmk );
#endif
  // Configure control register 1
  if ((device_select == AK09912) || (device_select == AK09915C) || (device_select == AK09915D) || (device_select == AK09917))
  {
    if (device_select == AK09912)
    {
      buffer = 0x0
        | (state->mag_info.nsf << 5); // NSF bit
    }
    else
    {
      buffer = 0x0
        | (state->mag_info.nsf << 5) // NSF bit
        | cur_wmk;                   // WM[4:0] bits
    }

    rv = ak0991x_com_write_wrapper(this,
                                   scp_service,
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
  if ((device_select == AK09915C) || (device_select == AK09915D) || (device_select == AK09917))
  {
    buffer = 0x0
      | ((uint8_t)state->mag_info.use_fifo << 7) // FIFO bit
      | (state->mag_info.sdr << 6)               // SDR bit
      | (uint8_t)desired_odr;                    // MODE[4:0] bits
  }
  else
  {
    buffer = 0x0
      | (uint8_t)desired_odr; // MODE[4:0] bits
  }

  return ak0991x_com_write_wrapper(this,
                                   scp_service,
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
sns_rc ak0991x_start_mag_streaming(sns_sensor_instance *const this )
{
  ak0991x_instance_state *state = (ak0991x_instance_state *)(this->state->state);
  sns_rc rv;

  // Enable Mag Streaming

  //Transit to Power-down mode first and then transit to other modes.
  rv = ak0991x_set_mag_config(this, true );

  if (rv != SNS_RC_SUCCESS)
  {
    return rv;
  }

  rv = ak0991x_set_mag_config(this, false );

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
  uint8_t  asa[AK0991X_NUM_SENSITIVITY];
  uint8_t  buffer[AK0991X_NUM_DATA_ST1_TO_ST2];
  int16_t  data[3];
  akm_device_type device_select;
  uint8_t  buffer_who_am_i[AK0991X_NUM_READ_DEV_ID];
  float    sstvt_adj[3];
  uint8_t  i;
  uint8_t  sdr = 0;

  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;

  sns_diag_service *diag = state->diag_service;

  // Initialize error code
  *err = 0;

  rv = ak0991x_get_who_am_i(state->scp_service,
                            state->com_port_info.port_handle,
                            &buffer_who_am_i[0]);

  if (rv != SNS_RC_SUCCESS)
  {
    *err = ((TLIMIT_NO_READ_ID) << 16);
    goto TEST_SEQUENCE_FAILED;
  }

  //Check AKM device ID
  if (buffer_who_am_i[0] == AK0991X_WHOAMI_COMPANY_ID)
  {
    if (buffer_who_am_i[1] == AK09911_WHOAMI_DEV_ID)
    {
      device_select = AK09911;
    }
    else if (buffer_who_am_i[1] == AK09912_WHOAMI_DEV_ID)
    {
      device_select = AK09912;
    }
    else if (buffer_who_am_i[1] == AK09913_WHOAMI_DEV_ID)
    {
      device_select = AK09913;
    }
    else if ((buffer_who_am_i[1] == AK09915_WHOAMI_DEV_ID) && (buffer_who_am_i[3] == AK09915C_SUB_ID))
    {
      device_select = AK09915C;
    }
    else if ((buffer_who_am_i[1] == AK09915_WHOAMI_DEV_ID) && (buffer_who_am_i[3] == AK09915D_SUB_ID))
    {
      device_select = AK09915D;
    }
    else if (buffer_who_am_i[1] == AK09916C_WHOAMI_DEV_ID)
    {
      device_select = AK09916C;
    }
    else if (buffer_who_am_i[1] == AK09916D_WHOAMI_DEV_ID)
    {
      device_select = AK09916D;
    }
    else if (buffer_who_am_i[1] == AK09917_WHOAMI_DEV_ID)
    {
      device_select = AK09917;
    }
    else if (buffer_who_am_i[1] == AK09918_WHOAMI_DEV_ID)
    {
      device_select = AK09918;
    }
    else
    {
     *err = ((TLIMIT_NO_READ_ID) << 16);
      goto TEST_SEQUENCE_FAILED;
    }
  }
  else
  {
    *err = ((TLIMIT_NO_READ_ID) << 16);
    goto TEST_SEQUENCE_FAILED;
  }

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
  if ((device_select == AK09911) || (device_select == AK09912))
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

    if (device_select == AK09911)
    {
      for (i = 0; i < AK0991X_NUM_SENSITIVITY; i++)
      {
        sstvt_adj[i] = ((asa[i] / 128.0f) + 1.0f);
      }
    }
    else
    {
      for (i = 0; i < AK0991X_NUM_SENSITIVITY; i++)
      {
        sstvt_adj[i] = ((asa[i] / 256.0f) + 0.5f);
      }
    }
  }
  else
  {
    for (i = 0; i < AK0991X_NUM_SENSITIVITY; i++)
    {
      sstvt_adj[i] = 1.0f;
    }
  }
 
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

  rv = ak0991x_get_meas_time(device_select, sdr, &usec_time_for_measure);
 
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
 
  if (device_select == AK09917)
  {
    // raw data in 16 bits
    data[0] = (int16_t)(((buffer[1] << 8) & 0xFF00) | buffer[0]);
    data[1] = (int16_t)(((buffer[3] << 8) & 0xFF00) | buffer[2]);
    data[2] = (int16_t)(((buffer[5] << 8) & 0xFF00) | buffer[4]);
  }
  else
  {
    // raw data in 16 bits
    data[0] = (int16_t)(((buffer[2] << 8) & 0xFF00) | buffer[1]);
    data[1] = (int16_t)(((buffer[4] << 8) & 0xFF00) | buffer[3]);
    data[2] = (int16_t)(((buffer[6] << 8) & 0xFF00) | buffer[5]);
  }
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
  else if (device_select == AK09917)
  {
    AKM_FST(TLIMIT_NO_SLF_RVHX, data[0], TLIMIT_LO_SLF_RVHX_AK09917, TLIMIT_HI_SLF_RVHX_AK09917,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHY, data[1], TLIMIT_LO_SLF_RVHY_AK09917, TLIMIT_HI_SLF_RVHY_AK09917,
            err);
    AKM_FST(TLIMIT_NO_SLF_RVHZ, data[2], TLIMIT_LO_SLF_RVHZ_AK09917, TLIMIT_HI_SLF_RVHZ_AK09917,
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
 * Read ST1 register data.
 *
 * @param[i] state                    Instance state
 * @param[i] port_handle              handle to synch COM port
 * @param[o] buffer                   st1 register data
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
static sns_rc ak0991x_read_st1(ak0991x_instance_state *state,
                               sns_sync_com_port_handle *port_handle,
                               uint8_t *buffer)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  rv = ak0991x_com_read_wrapper(state->scp_service,
                                port_handle,
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

/**
 * Get all fifo data.
 *
 * @param[i] state                    Instance state
 * @param[i] port_handle              handle to synch COM port
 * @param[i] num_samples              number of samples
 * @param[o] buffer                   fifo data
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
static sns_rc ak0991x_get_all_fifo_data(ak0991x_instance_state *state,
                                        sns_sync_com_port_handle *port_handle,
                                        uint16_t num_samples,
                                        uint8_t *buffer)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;

  rv = ak0991x_com_read_wrapper(state->scp_service,
                                port_handle,
                                AKM_AK0991X_REG_HXL,
                                buffer,
                                (uint32_t)(num_samples * AK0991X_NUM_DATA_HXL_TO_ST2),
                                &xfer_bytes);

  if (xfer_bytes != num_samples)
  {
    rv = SNS_RC_FAILED;
  }

  return rv;
}

/**
 * Get fifo data for 1 sample.
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
                             sns_diag_service *diag,
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
                                      log_sensor_state_raw_info *log_mag_state_raw_info)
{
  UNUSED_VAR(event_service);

  float ipdata[TRIAXIS_NUM] = {0}, opdata_raw[TRIAXIS_NUM] = {0};
  //float data[3];
  uint8_t i = 0;
  sns_std_sensor_sample_status status;

  SNS_INST_PRINTF(LOW, instance, "fac_cal_corr_mat 00=%d 11=%d 22=%d, fac_cal_bias0=%d 1=%d 2=%d",
        (uint32_t)state->mag_registry_cfg.fac_cal_corr_mat.e00,
        (uint32_t)state->mag_registry_cfg.fac_cal_corr_mat.e11,
        (uint32_t)state->mag_registry_cfg.fac_cal_corr_mat.e22,
        (uint32_t)state->mag_registry_cfg.fac_cal_bias[0],
        (uint32_t)state->mag_registry_cfg.fac_cal_bias[1],
        (uint32_t)state->mag_registry_cfg.fac_cal_bias[2]);



  if (state->mag_info.device_select == AK09917)
  {
    ipdata[TRIAXIS_X] =
      (int16_t)(((mag_sample[0] << 8) & 0xFF00) | mag_sample[1]) * state->mag_info.sstvt_adj[0] *
      state->mag_info.resolution;
    ipdata[TRIAXIS_Y] =
      (int16_t)(((mag_sample[2] << 8) & 0xFF00) | mag_sample[3]) * state->mag_info.sstvt_adj[1] *
      state->mag_info.resolution;
    ipdata[TRIAXIS_Z] =
      (int16_t)(((mag_sample[4] << 8) & 0xFF00) | mag_sample[5]) * state->mag_info.sstvt_adj[2] *
      state->mag_info.resolution;
  }
  else
  {
    ipdata[TRIAXIS_X] =
      (int16_t)(((mag_sample[1] << 8) & 0xFF00) | mag_sample[0]) * state->mag_info.sstvt_adj[0] *
      state->mag_info.resolution;
    ipdata[TRIAXIS_Y] =
      (int16_t)(((mag_sample[3] << 8) & 0xFF00) | mag_sample[2]) * state->mag_info.sstvt_adj[1] *
      state->mag_info.resolution;
    ipdata[TRIAXIS_Z] =
      (int16_t)(((mag_sample[5] << 8) & 0xFF00) | mag_sample[4]) * state->mag_info.sstvt_adj[2] *
      state->mag_info.resolution;
  }

  // Check magnetic sensor overflow
  if ((mag_sample[7] & AK0991X_HOFL_BIT) != 0)
  {
    status = SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE;
  }
  else
  {
    status = SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH;
  }

  // axis conversion
  for (i = 0; i < TRIAXIS_NUM; i++)
  {
    opdata_raw[state->axis_map[i].opaxis] = (state->axis_map[i].invert ? -1.0 : 1.0) *
      ipdata[state->axis_map[i].ipaxis];
    /*SNS_INST_PRINTF(LOW, instance, "ip=%d op=%d invert=%d",
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

  state->m_stream_event[0] = opdata_raw[TRIAXIS_X];
  state->m_stream_event[1] = opdata_raw[TRIAXIS_Y];
  state->m_stream_event[2] = opdata_raw[TRIAXIS_Z];


  // Log raw uncalibrated sensor data
  ak0991x_log_sensor_state_raw_add(
    log_mag_state_raw_info,
//    data,
    opdata_raw,
    timestamp,
    status);
}

void ak0991x_process_fifo_data_buffer(sns_sensor_instance *instance,
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
  log_sensor_state_raw_info log_mag_state_raw_info;

  // Allocate Sensor State Raw log packets for mag
  sns_memzero(&log_mag_state_raw_info, sizeof(log_mag_state_raw_info));
  log_mag_state_raw_info.encoded_sample_size = state->log_raw_encoded_size;
  ak0991x_log_sensor_state_raw_alloc(&log_mag_state_raw_info, 0);

  for(i = 0; i < num_bytes; i += 8)
  {
    // check if there's valid data in FIFO
    if ((fifo_start[i + 7] & AK0991X_INV_FIFO_DATA) && (state->mag_info.use_fifo))
    {
      break;
    }

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

  if (AKM_AK0991X_REG_ST1 == vector->reg_addr)
  {
    uint32_t i;

    sns_time                  timestamp;
    uint16_t                  cnt_for_ts = state->mag_info.cur_wmk;


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
  log_mag_state_raw_info.diag = diag;
  log_mag_state_raw_info.instance = instance;
  log_mag_state_raw_info.sensor_uid = &state->mag_info.suid;
  ak0991x_log_sensor_state_raw_alloc(&log_mag_state_raw_info, 0);

  uint32_t i;

  sns_time timestamp;
  uint16_t num_samples = 0;

  uint8_t buffer[AK0991X_MAX_FIFO_SIZE];

  if(state->mag_info.device_select == AK09917)
  {
    uint8_t st1_buf;
    //In case of AK09917,
    //Read ST1 register to check FIFO samples
    ak0991x_read_st1(state,state->com_port_info.port_handle,
                     &st1_buf);

    num_samples = st1_buf >> 2;
    SNS_INST_PRINTF(ERROR, instance, "num=%d st1=%x",num_samples,st1_buf );


    if(num_samples > 0)
    {
      //Read continuously all data in FIFO at one time.
      ak0991x_get_all_fifo_data(state,state->com_port_info.port_handle,
                                num_samples,&buffer[0]);
    }
  }
  else
  {
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

    ak0991x_log_sensor_state_raw_submit(&log_mag_state_raw_info, true);
  }
}

void ak0991x_handle_interrupt_event(sns_sensor_instance *const instance)
{
  uint8_t  buffer[AK0991X_MAX_FIFO_SIZE];
  uint32_t enc_len;
  uint16_t num_of_bytes;

  ak0991x_instance_state *state =
    (ak0991x_instance_state *)instance->state->state;

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
  log_mag_state_raw_info.diag = diag;
  log_mag_state_raw_info.instance = instance;
  log_mag_state_raw_info.sensor_uid = &state->mag_info.suid;
  ak0991x_log_sensor_state_raw_alloc(&log_mag_state_raw_info, 0);

  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;
  uint8_t  buffer[AK0991X_NUM_DATA_ST1_TO_ST2];

  sns_time timestamp;
  timestamp = sns_get_system_time();

  // TODO: update function to handle FIFO reads - will be added when S4S is added

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

  SNS_INST_PRINTF(LOW, instance, "handle timer event");

  ak0991x_handle_mag_sample(&buffer[1],
                            timestamp,
                            instance,
                            event_service,
                            state,
                            &log_mag_state_raw_info);
  ak0991x_log_sensor_state_raw_submit(&log_mag_state_raw_info, true);


  return SNS_RC_SUCCESS;
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
    phy_sensor_config.has_stream_is_synchronous = false;
    phy_sensor_config.stream_is_synchronous = false;
    break;

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

  phy_sensor_config.has_sample_rate = true;
  phy_sensor_config.sample_rate = state->mag_req.sample_rate;

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
}

void ak0991x_register_timer(sns_sensor_instance *this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;

  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_mgr = (sns_stream_service *)
    service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

  if (state->mag_req.sample_rate != AK0991X_ODR_0)
  {
    if (NULL == state->timer_data_stream)
    {
      stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                     this,
                                                     state->timer_suid,
                                                     &state->timer_data_stream);
    }

    sns_request             timer_req;
    sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
    size_t                  req_len;
    uint8_t                 buffer[20] = {0};
    req_payload.is_periodic = true;
    req_payload.start_time = sns_get_system_time();
    req_payload.timeout_period = sns_convert_ns_to_ticks(
        1 / state->mag_req.sample_rate * 1000 * 1000 * 1000);

    SNS_INST_PRINTF(LOW, this, "timeout_period=%d", (uint32_t)req_payload.timeout_period);

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
      state->timer_data_stream->api->send_request(state->timer_data_stream, &timer_req);
    }
    else
    {
      SNS_INST_PRINTF(ERROR, this, "Fail to send request to Timer Sensor");
    }
  }
  else
  {
    if (state->timer_data_stream != NULL)
    {
      stream_mgr->api->remove_stream(stream_mgr, state->timer_data_stream);
      state->timer_data_stream = NULL;
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

  SNS_INST_PRINTF(LOW, this, "ak0991x_reconfig_hw");

  if (state->mag_info.desired_odr != AK0991X_MAG_ODR_OFF)
  {
    if ((state->mag_info.use_dri && state->irq_info.is_ready) ||
         (state->mag_info.use_dri && state->dae_if.mag.state == STREAMING) ||
        (!state->mag_info.use_dri))
    {
      SNS_INST_PRINTF(LOW, this, "start_mag_streaming");
      ak0991x_start_mag_streaming(this);
    }
  }
  else
  {
    SNS_INST_PRINTF(LOW, this, "ak0991x_stop_mag_streaming");
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
        SNS_INST_PRINTF(ERROR, instance, "COM self-test success!!");
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
        SNS_INST_PRINTF(ERROR, instance, "hw self-test success!!");
      }
      ak0991x_send_com_test_event(instance, &state->mag_info.suid, hw_success,
                                  SNS_PHYSICAL_SENSOR_TEST_TYPE_HW);
    }
    state->mag_info.test_info.test_client_present = false;
  }
}

