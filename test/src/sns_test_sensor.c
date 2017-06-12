/**
 * @file sns_test_sensor.c
 *
 * The test virtual Sensor implementation
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * $Id: //components/dev/ssc.slpi/3.0/maansyw.ssc.slpi.3.0.napali_06_11/sensors/test/src/sns_test_sensor.c#1 $
 * $DateTime: 2017/06/11 12:38:13 $
 * $Change: 13546828 $
 *
 **/

#include "pb_decode.h"
#include "pb_encode.h"
#include "sns_attribute_service.h"
#include "sns_attribute_util.h"
#include "sns_data_stream.h"
#include "sns_diag_service.h"
#include "sns_event_service.h"
#include "sns_mem_util.h"
#include "sns_pb_util.h"
#include "sns_request.h"
#include "sns_sensor.h"
#include "sns_sensor_event.h"
#include "sns_sensor_instance.h"
#include "sns_sensor_uid.h"
#include "sns_sensor_util.h"
#include "sns_service.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_suid.pb.h"
#include "sns_test_sensor.h"
#include "sns_test_sensor_instance.h"
#include "sns_types.h"
#include "sns_printf.h"

typedef struct sns_test_implementation
{
  char* datatype;
  uint32_t datatype_len;
  sns_test_create_request_func create_request_func;
  sns_test_process_event_func process_event_func;
} sns_test_implementation;

/*
 * select a test implementation based on compile-time flag
 *
 * To add an implementation for a test sensor populate,
 *    1. data type
 *    2. create_request function
 *    3. process_event function
 *
 *    #2 and #3 are optional and can be NULL. In that case a
 *    default implementation for them will be selected.
 */

#if defined(SNS_TEST_ACCEL)
#include "sns_test_std_sensor.h"
 const sns_test_implementation test_sensor_impl = {
  "accel",
  sizeof("accel"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_AMD)
#include "sns_test_amd.h"
 const sns_test_implementation test_sensor_impl = {
  "amd",
  sizeof("amd"),
  sns_test_create_amd_request,
  sns_test_amd_process_event
};
#elif defined(SNS_TEST_RMD)
#include "sns_test_amd.h"
 const sns_test_implementation test_sensor_impl = {
  "rmd",
  sizeof("rmd"),
  sns_test_create_amd_request,
  sns_test_amd_process_event
};
#elif defined(SNS_TEST_WALK)
#include "sns_test_walk.h"
 const sns_test_implementation test_sensor_impl = {
  "ccd_walk",
  sizeof("ccd_walk"),
  sns_test_create_walk_request,
  sns_test_walk_process_event
};
#elif defined(SNS_TEST_TILT)
#include "sns_test_tilt.h"
 const sns_test_implementation test_sensor_impl = {
  "tilt",
  sizeof("tilt"),
  sns_test_create_tilt_request,
  sns_test_tilt_process_event
};
#elif defined(SNS_TEST_RESAMPLER)
#include "sns_test_resampler.h"
 const sns_test_implementation test_sensor_impl = {
  "resampler",
  sizeof("resampler"),
  sns_test_create_resampler_request,
  sns_test_resampler_process_event
};
#elif defined(SNS_TEST_GYRO_CAL)
#include "sns_test_gyro_cal.h"
 const sns_test_implementation test_sensor_impl = {
  "gyro_calibration",
  sizeof("gyro_calibration"),
  sns_test_on_change_sensor_create_request,
  sns_test_gyro_cal_process_event
};
#elif defined(SNS_TEST_GYRO_ROT_MATRIX)
#include "sns_test_gyro_rot_matrix.h"
 const sns_test_implementation test_sensor_impl = {
  "gyro_rot_matrix",
  sizeof("gyro_rot_matrix"),
  sns_test_create_gyro_rot_matrix_request,
  sns_test_gyro_rot_matrix_process_event
};
#elif defined(SNS_TEST_GRAVITY)
#include "sns_test_gravity.h"
 const sns_test_implementation test_sensor_impl = {
  "gravity",
  sizeof("gravity"),
  sns_test_create_gravity_request,
  sns_test_gravity_process_event
};
#elif defined(SNS_TEST_SENSOR_TEMP)
#include "sns_test_std_sensor.h"
 const sns_test_implementation test_sensor_impl = {
  "sensor_temperature",
  sizeof("sensor_temperature"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_GYRO)
#include "sns_test_std_sensor.h"
 const sns_test_implementation test_sensor_impl = {
  "gyro",
  sizeof("gyro"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_PRESSURE)
#include "sns_test_std_sensor.h"
 const sns_test_implementation test_sensor_impl = {
  "pressure",
  sizeof("pressure"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_MAG)
#include "sns_test_std_sensor.h"
 const sns_test_implementation test_sensor_impl = {
  "mag",
  sizeof("mag"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_AMBIENT_LIGHT)
#include "sns_test_std_sensor.h"
 const sns_test_implementation test_sensor_impl = {
  "ambient_light",
  sizeof("ambient_light"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_PROXIMITY)
#include "sns_test_std_sensor.h"
 const sns_test_implementation test_sensor_impl = {
  "proximity",
  sizeof("proximity"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_HUMIDITY)
#include "sns_test_std_sensor.h"
 const sns_test_implementation test_sensor_impl = {
  "humidity",
  sizeof("humidity"),
  sns_test_on_change_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_RGB)
#include "sns_test_std_sensor.h"
 const sns_test_implementation test_sensor_impl = {
  "rgb",
  sizeof("rgb"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_UV)
#include "sns_test_std_sensor.h"
 const sns_test_implementation test_sensor_impl = {
  "ultra_violet",
  sizeof("ultra_violet"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_AMBIENT_TEMPERATURE)
#include "sns_test_std_sensor.h"
 const sns_test_implementation test_sensor_impl = {
  "ambient_temperature",
  sizeof("ambient_temperature"),
  sns_test_on_change_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_GAME_RV)
#include "sns_test_std_sensor.h"
 const sns_test_implementation test_sensor_impl = {
  "game_rv",
  sizeof("game_rv"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_HALL)
#include "sns_test_hall.h"
const sns_test_implementation test_sensor_impl = {
  "hall",
  sizeof("hall"),
  sns_test_create_hall_request,
  sns_test_hall_process_event
};
#elif defined(SNS_TEST_PSMD)
#include "sns_test_psmd.h"
 const sns_test_implementation test_sensor_impl = {
  "psmd",
  sizeof("psmd"),
  sns_test_create_psmd_request,
  sns_test_psmd_process_event
};
#elif defined(SNS_TEST_REMOTE_PROC_STATE)
#include "sns_test_remote_proc_state.h"
 const sns_test_implementation test_sensor_impl = {
  "remote_proc_state",
  sizeof("remote_proc_state"),
  sns_test_create_remote_proc_state_request,
  sns_test_remote_proc_state_process_event
};
#elif defined(SNS_TEST_OEM1)
#include "sns_test_oem1.h"
 const sns_test_implementation test_sensor_impl = {
  "oem1",
  sizeof("oem1"),
  sns_test_create_oem1_request,
  sns_test_oem1_process_event
};
#else
 const sns_test_implementation test_sensor_impl = {
  NULL, 0, NULL, NULL
};
#endif

/**
 * Publish all saved attributes for test_sensor.
 */
 void
publish_attributes(sns_sensor* const this)
{
  {
    char const name[] = "test";
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = name, .buf_len = sizeof(name) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_NAME, &value, 1, false);
  }
  {
    char const type[] = "test";
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = type, .buf_len = sizeof(type) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
  }
  {
    char const vendor[] = "qualcomm";
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = vendor, .buf_len = sizeof(vendor) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_VENDOR, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = true;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_AVAILABLE, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = 1;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_VERSION, &value, 1, true);
  }
}

/* See sns_sensor::init */
 sns_rc
sns_test_init(sns_sensor* const this)
{
  sns_rc rc = SNS_RC_FAILED;
  sns_test_state* state = (sns_test_state*)this->state->state;
  publish_attributes(this);

  sns_service_manager *smgr= this->cb->get_service_manager(this);
  state->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);

  SNS_PRINTF(MED, this, "sns_test_init");

  state->test_sensor_create_request =
      (test_sensor_impl.create_request_func) ?
      test_sensor_impl.create_request_func : sns_test_std_sensor_create_request;
  state->test_sensor_process_event =
      (test_sensor_impl.process_event_func) ?
      test_sensor_impl.process_event_func : sns_test_std_sensor_process_event;

  /* if test implementation is enabled, initiate the test */
  if (test_sensor_impl.datatype != NULL)
  {
    state->remaining_events = NUM_EVENTS_TO_PROCESS;
    state->remaining_iterations = NUM_TEST_ITERATIONS;
    state->num_events_received = 0;
    sns_memset(state->test_data, 0x00, sizeof(state->test_data));

    //SENSOR_PRINTF_MED_FULL(this, "test_sensor type = %s", test_sensor_impl.datatype);
    state->suid_search[state->search_count].data_type_str = test_sensor_impl.datatype;
    state->suid_search[state->search_count].suid = state->uid1_list;
    state->search_count++;

    if (strcmp("resampler", test_sensor_impl.datatype) == 0)
    {
      state->suid_search[state->search_count].data_type_str = "gyro";
      state->suid_search[state->search_count].suid = state->uid2_list;
      state->search_count++;
    }
    rc = sns_search_suids(this, state->suid_search, state->search_count, &state->suid_stream);
    if (rc != SNS_RC_SUCCESS)
    {
      SNS_PRINTF(ERROR, this, "sns_search_suids() failed");
    }
  }

  SNS_PRINTF(MED, this, "sns_test_init done");

  return rc;
}

sns_rc
sns_test_deinit(sns_sensor* const this)
{
  UNUSED_VAR(this);
  return SNS_RC_SUCCESS;
}
