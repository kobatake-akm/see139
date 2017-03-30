/**
 * @file sns_test_std_sensor.c
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 **/
#include "sns_test_std_sensor.h"
#include "sns_pb_util.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_diag_service.h"
#include "sns_types.h"
#include "sns_test_sensor.h"

static const float TEST_SAMPLE_RATE = 1.0f;
static const float TEST_BATCH_PERIOD = 1000000.0f;

/** See sns_test_std_sensor.h */
void sns_test_std_sensor_create_request(const sns_sensor *sensor,
                                   void* payload,
                                   const pb_field_t** payload_fields,
                                   uint32_t* message_id,
                                   sns_std_request *std_req)
{
  UNUSED_VAR(sensor);
  sns_std_sensor_config* config = (sns_std_sensor_config*)payload;
  config->sample_rate = TEST_SAMPLE_RATE;
  *message_id = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG;
  *payload_fields = sns_std_sensor_config_fields;
  std_req->has_batch_period = true;
  std_req->batch_period = TEST_BATCH_PERIOD;
}

/** See sns_test_std_sensor.h */
void sns_test_std_sensor_process_event(const sns_sensor *sensor,
                                  void* event,
                                  uint32_t event_len,
                                  void* test_data,
                                  uint32_t message_id,
                                  sns_time timestamp)
{
  sns_test_state* state = (sns_test_state*)sensor->state->state;
  sns_diag_service* diag = state->diag_service;

  switch (message_id)
  {
    case SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG_EVENT:
    /* TODO Add handling for sensors that will use std sensor config event */
    
    break;
    
    case SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT:
    {
      diag->api->sensor_printf(diag, sensor, SNS_MED, __FILENAME__, __LINE__,
                               "test_std_sensor: sns_sensor_stream_msgid_sns_std_sensor_config_event");

      sns_std_sensor_physical_config_event phy_sensor_config =
          sns_std_sensor_physical_config_event_init_default;

      pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event,
                                                   event_len);

      if (!pb_decode(&stream, sns_std_sensor_physical_config_event_fields, &phy_sensor_config))
      {
        diag->api->sensor_printf(diag, sensor, SNS_ERROR, __FILENAME__, __LINE__,
                                 "pb_decode() failed for stream_event");
      }
      else
      {
        diag->api->sensor_printf(diag, sensor, SNS_MED, __FILENAME__, __LINE__,
                                 "sample_rate = %.2f", phy_sensor_config.sample_rate);
        diag->api->sensor_printf(diag, sensor, SNS_MED, __FILENAME__, __LINE__,
                                 "wm = %d  resolution = %0.4f  is_synch = %d",
                                 phy_sensor_config.water_mark, phy_sensor_config.resolution,
                                 phy_sensor_config.stream_is_synchronous);
        diag->api->sensor_printf(diag, sensor, SNS_MED, __FILENAME__, __LINE__,
                                 "min_range = %0.4f max_range = %0.4f active_current = %d",
                                 phy_sensor_config.range[0], phy_sensor_config.range[1],
                                 phy_sensor_config.active_current);
      }
    }
    break;
    case SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT:
    {
      float data[20] = { 0 };
      uint8_t arr_index = 0;
      uint8_t i;
      sns_std_sensor_event imu_event = sns_std_sensor_event_init_default;
      pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event,
                                                   event_len);
      pb_float_arr_arg arg = {
         .arr = data,
         .arr_len = ARR_SIZE(data),
         .arr_index = &arr_index
      };

      imu_event.data = (struct pb_callback_s) {
        .funcs.decode = &pb_decode_float_arr_cb, .arg = &arg
      };

      if (!pb_decode(&stream, sns_std_sensor_event_fields, &imu_event))
      {
        diag->api->sensor_printf(diag, sensor, SNS_ERROR, __FILENAME__, __LINE__,
                                 "pb_decode() failed for stream_event");
      }

      sns_test_std_sensor_data* tdata = (sns_test_std_sensor_data*)test_data;

      if (tdata->num_events > 0)
      {
        float delta = timestamp - tdata->last_ts;
        /* calculate running average of delta timestamps */
        tdata->avg_delta = (tdata->avg_delta * tdata->num_events + delta)
            / (tdata->num_events + 1);
      }
      tdata->last_ts = timestamp;
      tdata->num_events++;

      diag->api->sensor_printf(diag, sensor, SNS_MED, __FILENAME__, __LINE__,
                               "sensor sample #%04u: ts=%u ticks,"
                               " avg_delta=%u ticks",
                               tdata->num_events, (uint32_t)timestamp,
                               (uint32_t)tdata->avg_delta);
      for(i = 0; i < arr_index; i++)
      {
        diag->api->sensor_printf(diag, sensor, SNS_HIGH, __FILENAME__, __LINE__,
                         "value[%d] = [%.4f]", i, data[i]);
      }

    }
      break;
    default:
      diag->api->sensor_printf(diag, sensor, SNS_ERROR, __FILENAME__, __LINE__,
                               "incorrect message_id %u", message_id);
  }
}
