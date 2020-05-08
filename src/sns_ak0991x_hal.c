/**
 * @file sns_ak0991x_hal.c
 *
 * Copyright (c) 2016-2019 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2016-2020 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 **/

#include "sns_rc.h"
#include "sns_time.h"
#include "sns_mem_util.h"
#include "sns_com_port_types.h"
#include "sns_sync_com_port_service.h"
#include "sns_types.h"

#include "sns_ak0991x_hal.h"
#include "sns_ak0991x_sensor.h"
#include "sns_ak0991x_sensor_instance.h"

#include "pb_encode.h"
#include "sns_pb_util.h"

#define AKM_FST(no, data, lo, hi, err) \
  if (ak0991x_test_threshold((no), (data), (lo), (hi), (err)) \
      != SNS_RC_SUCCESS) { goto TEST_SEQUENCE_FAILED; }

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

/**
 * Wait DRDY bit is ready
 * @param[i] this              reference to the instance
 * @param[i] timeout_ms        Waiting time for drdy
 *
 * @return sns_rc
 * SNS_RC_FAILED
 * SNS_RC_SUCCESS
 */
static sns_rc ak0991x_wait_drdy_poll(sns_sensor_instance *const this,
                                     int32_t timeout_ms)
{
  sns_rc  rv;
  int16_t i;
  uint8_t st1_buf;
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;

  for (i = 0; i < timeout_ms; i++)
  {
    /* check DRDY */
    rv = ak0991x_read_st1(state, &st1_buf);  // read ST1
    if (rv != SNS_RC_SUCCESS)
    {
      return rv;
    }

    state->data_is_ready = (st1_buf & AK0991X_DRDY_BIT) ? true : false; // check DRDY bit
    if (state->data_is_ready)
    {
      /* OK, DRDY bit is high */
      return SNS_RC_SUCCESS;
    }

    /* DRDY bit is still LOW, wait for a while and read again */
    sns_busy_wait(sns_convert_ns_to_ticks(1 * 1000 * 1000)); // Wait 1ms
  }

  return SNS_RC_FAILED;
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
static 
sns_rc ak0991x_hw_self_test(sns_sensor_instance *const this,
                            uint32_t *err)
{
  sns_rc   rv = SNS_RC_SUCCESS;
  uint32_t xfer_bytes;
  uint8_t  asa[AK0991X_NUM_SENSITIVITY];
  uint8_t  buffer[AK0991X_NUM_DATA_ST1_TO_ST2];
  int16_t  data[3];
  int      i;
  
  ak0991x_instance_state *state =
    (ak0991x_instance_state *)this->state->state;

  // Initialize error code
  *err = 0;

  // Reset device
  rv = ak0991x_device_sw_reset(this,
                               state->scp_service,
                               &state->com_port_info);

  if (rv != SNS_RC_SUCCESS)
  {
    *err = ((TLIMIT_NO_RESET) << 16);
    goto TEST_SEQUENCE_FAILED;
  }

  /** Step 1
   *   If the device has FUSE ROM, test the sensitivity value
   **/
  if ((state->mag_info.device_select == AK09911) || (state->mag_info.device_select == AK09912))
  {
    rv = ak0991x_read_asa(this,
                          state->scp_service,
                          state->com_port_info.port_handle,
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

  /** Step 2
   *   Continuous mode check
   **/

  /* Set to CNT measurement mode3 (50Hz) */
  buffer[0] = 0x00;
  buffer[1] = AK0991X_MAG_ODR50;
  /* Enable FIFO for AK09917D RevA/B bug */
  if (state->mag_info.device_select == AK09917)
  {
    buffer[1] |= AK0991X_FIFO_BIT;
  }
  rv = ak0991x_com_write_wrapper(this,
                   state->scp_service,
                   state->com_port_info.port_handle,
                   AKM_AK0991X_REG_CNTL1,
                   buffer,
                   2,
                   &xfer_bytes,
                   false);

  if (rv != SNS_RC_SUCCESS
    ||
    xfer_bytes != 2)
  {
    *err = ((TLIMIT_NO_CNT_CNTL2) << 16);
    goto TEST_SEQUENCE_FAILED;
  }

  for (i = 0; i < TLIMIT_NO_CNT_ITR; i++)
  {
    /* current test program sets to MODE3, i.e. 50Hz=20msec interval
     * so, wait for double length of measurement time.
     */
    rv = ak0991x_wait_drdy_poll(this, 40);
    if (rv != SNS_RC_SUCCESS)
    {
      *err = ((TLIMIT_NO_CNT_WAIT) << 16);
      goto TEST_SEQUENCE_FAILED;
    }

    /* Get measurement from device (1st). */
    rv = ak0991x_read_st1_st2(state, buffer);

    if (rv != SNS_RC_SUCCESS)
    {
      *err = ((TLIMIT_NO_CNT_READ) << 16);
      goto TEST_SEQUENCE_FAILED;
    }

    /* Check DRDY bit (1st) */
    state->data_is_ready = (buffer[0] & AK0991X_DRDY_BIT) ? true : false;
    AKM_FST(TLIMIT_NO_CNT_1ST, state->data_is_ready, TLIMIT_LO_CNT_1ST, TLIMIT_HI_CNT_1ST, err);

    /* Get measurement from device (2nd). */
    rv = ak0991x_read_st1_st2(state, buffer);

    if (rv != SNS_RC_SUCCESS)
    {
      *err = ((TLIMIT_NO_CNT_READ) << 16);
      goto TEST_SEQUENCE_FAILED;
    }

    /* Check DRDY bit (2nd) */
    state->data_is_ready = (buffer[0] & AK0991X_DRDY_BIT) ? true : false;
    AKM_FST(TLIMIT_NO_CNT_2ND, state->data_is_ready, TLIMIT_LO_CNT_2ND, TLIMIT_HI_CNT_2ND, err);
  }

  // Reset device
  rv = ak0991x_device_sw_reset(this,
                               state->scp_service,
                               &state->com_port_info);

  if (rv != SNS_RC_SUCCESS)
  {
    *err = ((TLIMIT_NO_RESET) << 16);
    goto TEST_SEQUENCE_FAILED;
  }

  /* Wait over 100us */
  sns_busy_wait(sns_convert_ns_to_ticks(100 * 1000));

  /** Step 3
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

  // To ensure that measurement is finished, wait for double as typical
  sns_busy_wait(sns_convert_ns_to_ticks(((uint64_t)state->mag_info.meas_time) * 1000 * 2));

  /** Step 4
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

  ak0991x_get_adjusted_mag_data(this, &buffer[1], &data[0]);

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

  AKM_FST(TLIMIT_NO_SLF_ST2, (buffer[8] & TLIMIT_ST2_MASK),
          TLIMIT_LO_SLF_ST2, TLIMIT_HI_SLF_ST2, err);

TEST_SEQUENCE_FAILED:

  if (*err == 0)
  {
    return SNS_RC_SUCCESS;
  }
  else
  {
    AK0991X_INST_PRINT(HIGH, this, "hw self-test failed!! err code = %x",*err);
    // Reset device
    ak0991x_device_sw_reset(this,
                            state->scp_service,
                            &state->com_port_info);

    return SNS_RC_FAILED;
  }
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

  AK0991X_INST_PRINT(HIGH, instance, "run_self_test:: present=%u type=%u",
                     state->mag_info.test_info.test_client_present,
                     state->mag_info.test_info.test_type);

  if(state->mag_info.test_info.test_client_present)
  {
    if(state->mag_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_COM)
    {
      uint8_t buffer[AK0991X_NUM_READ_DEV_ID] = {0};
      bool who_am_i_success = false;

      ak0991x_enter_i3c_mode(instance, &state->com_port_info, state->scp_service);

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
      else
      {
        AK0991X_INST_PRINT(ERROR, instance, "COM self-test failed!!");
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

      ak0991x_enter_i3c_mode(instance, &state->com_port_info, state->scp_service);

      AK0991X_INST_PRINT(LOW, instance, "hw self-test start!");
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
    else if(state->mag_info.test_info.test_type == SNS_PHYSICAL_SENSOR_TEST_TYPE_SW)
    {
      AK0991X_INST_PRINT(LOW, instance, "Not supported. type=%d",(uint8_t)state->mag_info.test_info.test_type);
      rv = SNS_RC_NOT_SUPPORTED;
    }
    state->mag_info.test_info.test_client_present = false;
  }
}

