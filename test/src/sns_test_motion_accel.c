/**
 * @file sns_test_motion_accel.c
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc. All Rights
 * Reserved. Confidential and Proprietary - Qualcomm
 * Technologies, Inc.
 *
 **/
#include "sns_test_motion_accel.h"
#include "sns_pb_util.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_diag_service.h"
#include "sns_types.h"
#include "sns_test_sensor.h"
#include "sns_motion_accel.pb.h"
#include "sns_mem_util.h"
#include "sns_stream_service.h"
#include "sns_service_manager.h"
#include "sns_request.h"

/**
 * This Motion Accel Test Sensor (MATS) is an example 
 * implementation with a state machine to try and exercice 
 * streaming, batching and motion detection features of the 
 * Motion Accel Sensor. Also, this flow is for standalone MATS 
 * request. If there is a concurrent "accel" Sensor client then 
 * the flow may look different. In any case, the Sensor must 
 * comply with call flows for standalone and concurrent 
 * streaming as described in document 80-P9361-1. 
 *  
 * 1. The MATS queries the SUID Sensor for SUID of the
 *    "md_motion_accel" Sensor.
 * 2. Once it receives the SUID, it starts a stream with config:
 *    sample_rate = 10Hz
 *    report_rate = 10Hz
 *    motion_detect_enable = false
 * 3. After receiving 1000 motion accel samples, the MATS
 *    changes the request to enable motion detect with config:
 *    sample_rate = 10Hz
 *    report_rate = 10Hz
 *    motion_detect_enable = true
 * 4. If there is a motion detect interrupt event from the
 *    Sensor under test then the MATS is ready to receive motion
 *    accel samples at 10Hz sample and 10Hz report rate.
 * 5. After receiving 500 motion accel samples, the MATS changes
 *    the request to:
 *    sample_rate = 10Hz
 *    report_rate = 1Hz
 *    motion_detect_enable = false
 * 6. After the MATS receives 100 motion accel samples in this
 *    phase, it renables motion detect interrupt with config:
 *    sample_rate = 10Hz
 *    report_rate = 10Hz
 *    motion_detect_enable = true
 * 7. The process then repeats from step 4.
 */

static const float TEST_SAMPLE_RATE = 10.0f;
static const float TEST_REPORT_RATE_1_HZ = 1.0f;
static const float TEST_REPORT_RATE_10_HZ = 10.0f;

/** See sns_test_motion_accel.h */
void sns_test_motion_accel_create_request(const sns_sensor *sensor,
                                   void* payload,
                                   const pb_field_t** payload_fields,
                                   uint32_t* message_id,
                                   sns_std_request *std_req)
{
  sns_test_state* state = (sns_test_state*)sensor->state->state;
  sns_diag_service* diag = state->diag_service;
  sns_test_motion_accel_data *ma_data =
     (sns_test_motion_accel_data*)state->test_data;
  // Test state machine starts with 10Hz/10Hz request with MD disabled
  ma_data->sample_rate = TEST_SAMPLE_RATE;
  ma_data->report_rate = TEST_REPORT_RATE_10_HZ;
  ma_data->phase = MOTION_ACCEL_10_10;
  ma_data->ma_10_10_count = 0;

  sns_motion_accel_config* config = (sns_motion_accel_config*)payload;
  config->sample_rate = ma_data->sample_rate;
  config->enable_motion_detect = false;
  *message_id = SNS_MOTION_ACCEL_MSGID_SNS_MOTION_ACCEL_CONFIG;
  *payload_fields = sns_motion_accel_config_fields;
  std_req->has_batch_period = true;
  std_req->batch_period = (1000000/ma_data->report_rate);

  diag->api->sensor_printf(diag, sensor, SNS_HIGH, __FILENAME__, __LINE__,
                               "Phase switch to MOTION_ACCEL_10_10");
}

/** See sns_test_motion_accel.h */
void sns_test_motion_accel_process_event(const sns_sensor *sensor,
                                  void* event,
                                  uint32_t event_len,
                                  void* test_data,
                                  uint32_t message_id,
                                  sns_time timestamp)
{
  UNUSED_VAR(timestamp);
  sns_test_state* state = (sns_test_state*)sensor->state->state;
  sns_diag_service* diag = state->diag_service;
  bool update_req = false;
  sns_std_request std_req = sns_std_request_init_default;
  sns_motion_accel_config ma_req = sns_motion_accel_config_init_default;
  sns_test_motion_accel_data* tdata = (sns_test_motion_accel_data*)test_data;
  pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event,
                                               event_len);

  switch (message_id)
  {
    case SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG_EVENT:
    {
      diag->api->sensor_printf(diag, sensor, SNS_MED, __FILENAME__, __LINE__,
                               "test_std_sensor: sns_sensor_stream_msgid_sns_std_sensor_config_event");

      sns_std_sensor_config_event config_event =
          sns_std_sensor_config_event_init_default;
      pb_buffer_arg payload_args;

      config_event.payload.data.funcs.decode = &pb_decode_string_cb;
      config_event.payload.data.arg = &payload_args;

      if (!pb_decode(&stream, sns_std_sensor_config_event_fields, &config_event))
      {
        diag->api->sensor_printf(diag, sensor, SNS_ERROR, __FILENAME__, __LINE__,
                                 "pb_decode() failed for stream_event");
      }
      else
      {
        sns_std_sensor_physical_config phy_sensor_config =
            sns_std_sensor_physical_config_init_default;
        sns_memscpy(&phy_sensor_config, payload_args.buf_len,
                    payload_args.buf, payload_args.buf_len);

        diag->api->sensor_printf(diag, sensor, SNS_MED, __FILENAME__, __LINE__,
                                 "sample_rate = %.2f", config_event.sample_rate);
        if(config_event.has_payload)
        {
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
    }
    break;
    case SNS_MOTION_ACCEL_MSGID_SNS_MOTION_ACCEL_STREAM_EVENT:
    {
      sns_motion_accel_stream_event stream_event =
         sns_motion_accel_stream_event_init_default;

      if (!pb_decode(&stream, sns_motion_accel_stream_event_fields, &stream_event))
      {
        diag->api->sensor_printf(diag, sensor, SNS_ERROR, __FILENAME__, __LINE__,
                                 "pb_decode() failed for stream_event");
      }

      diag->api->sensor_printf(diag, sensor, SNS_HIGH, __FILENAME__, __LINE__,
                       "value = [%.4f %.4f %.4f]", stream_event.data[0],
                               stream_event.data[1], stream_event.data[2]);

      if(tdata->phase == MOTION_ACCEL_10_10)
      {
        tdata->ma_10_10_count++;
        if(tdata->ma_10_10_count > 1000)
        {
          // Change test sensor state to MD enabled request
          tdata->report_rate = TEST_REPORT_RATE_10_HZ;
          tdata->sample_rate = TEST_SAMPLE_RATE;
          tdata->phase = MOTION_ACCEL_MD;

          std_req.has_batch_period = true;
          std_req.batch_period = (1000000/tdata->report_rate);

          ma_req.enable_motion_detect = true;
          ma_req.sample_rate = tdata->sample_rate;

          update_req = true;
          diag->api->sensor_printf(diag, sensor, SNS_HIGH, __FILENAME__, __LINE__,
                       "Changing state to MOTION_ACCEL_MD from MOTION_ACCEL_10_10");
        }
      }
      else if(tdata->phase == MOTION_ACCEL_MD_FIRED_10_10)
      {
        tdata->ma_fired_10_10_count++;
        if(tdata->ma_fired_10_10_count > 500)
        {
          // Change test sensor state to 10hz/1hz
          tdata->report_rate = TEST_REPORT_RATE_1_HZ;
          tdata->sample_rate = TEST_SAMPLE_RATE;
          tdata->phase = MOTION_ACCEL_10_1;
          tdata->ma_10_1_count = 0;

          std_req.has_batch_period = true;
          std_req.batch_period = (1000000/tdata->report_rate);

          ma_req.enable_motion_detect = false;
          ma_req.sample_rate = tdata->sample_rate;

          update_req = true;
          diag->api->sensor_printf(diag, sensor, SNS_HIGH, __FILENAME__, __LINE__,
                       "Changing state to MOTION_ACCEL_10_1 from MOTION_ACCEL_MD_FIRED_10_10");
        }
      }
      else if(tdata->phase == MOTION_ACCEL_10_1)
      {
        tdata->ma_10_1_count++;
        if(tdata->ma_10_1_count > 100)
        {
          // Change test sensor state to enable MD
          tdata->report_rate = TEST_REPORT_RATE_10_HZ;
          tdata->sample_rate = TEST_SAMPLE_RATE;
          tdata->phase = MOTION_ACCEL_MD;

          std_req.has_batch_period = true;
          std_req.batch_period = (1000000/tdata->report_rate);

          ma_req.enable_motion_detect = true;
          ma_req.sample_rate = tdata->sample_rate;

          update_req = true;
          diag->api->sensor_printf(diag, sensor, SNS_HIGH, __FILENAME__, __LINE__,
                       "Changing state to MOTION_ACCEL_MD from MOTION_ACCEL_10_1");
        }
      }
      else
      {
        // ERROR
      }
    }
    break;

    case SNS_MOTION_ACCEL_MSGID_SNS_MOTION_ACCEL_MD_ARMED_EVENT:
    {
      sns_motion_accel_md_armed_event ma_event =
         sns_motion_accel_md_armed_event_init_default;

      if(pb_decode(&stream, sns_motion_accel_md_armed_event_fields, &ma_event))
      {
        if(ma_event.md_is_armed)
        {
          tdata->md_armed_count++;
        }
        else
        {
          tdata->md_not_armed_count++;
        }
      }
      diag->api->sensor_printf(diag, sensor, SNS_HIGH, __FILENAME__, __LINE__,
                 "SNS_MOTION_ACCEL_MSGID_SNS_MOTION_ACCEL_MD_ARMED_EVENT md_is_armed = %d",
                 ma_event.md_is_armed);
    }
    break;

    case SNS_MOTION_ACCEL_MSGID_SNS_MOTION_ACCEL_MD_FIRED_EVENT:
    {
      sns_motion_accel_md_fired_event ma_event =
         sns_motion_accel_md_fired_event_init_default;

      if(pb_decode(&stream, sns_motion_accel_md_fired_event_fields, &ma_event))
      {
        if(ma_event.md_has_fired)
        {
          tdata->md_fired_count++;
          // Motion accel sensor should transition to 10hz/10hz stream
          // after MD fires. Change test sensor state and reset event count
          // for motion accel data.
          tdata->phase = MOTION_ACCEL_MD_FIRED_10_10;
          tdata->ma_fired_10_10_count = 0;
        }
      }
      diag->api->sensor_printf(diag, sensor, SNS_HIGH, __FILENAME__, __LINE__,
                 "SNS_MOTION_ACCEL_MSGID_SNS_MOTION_ACCEL_MD_FIRED_EVENT md_fired = %d",
                  ma_event.md_has_fired);
    }
    break;

    default:
      diag->api->sensor_printf(diag, sensor, SNS_ERROR, __FILENAME__, __LINE__,
                               "incorrect message_id %u", message_id);
  }

  if(update_req)
  {
    if(NULL != state->sensor_stream)
    {
      size_t encoded_len;
      uint8_t buffer[100];
      sns_memset(buffer, 0, sizeof(buffer));

      encoded_len = pb_encode_request(buffer, sizeof(buffer), &ma_req,
                                      sns_motion_accel_config_fields, &std_req);
      if(0 < encoded_len)
      {
        sns_request request = (sns_request){ .message_id = SNS_MOTION_ACCEL_MSGID_SNS_MOTION_ACCEL_CONFIG,
            .request_len = encoded_len, .request = buffer };
        state->sensor_stream->api->send_request(state->sensor_stream, &request);
      }
      else
      {
        diag->api->sensor_printf(diag, sensor, SNS_ERROR, __FILENAME__, __LINE__,
                            "Failed to send sensor request, stream=%p",
                            state->sensor_stream);
      }
    }
  }
}

