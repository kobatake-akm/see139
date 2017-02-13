#pragma once
/**
 * @file sns_test_motion_accel.h
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc. All Rights 
 * Reserved. Confidential and Proprietary - Qualcomm 
 * Technologies, Inc. 
 *
 **/

#include "sns_sensor.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_std.pb.h"

/** Test Sensor State. */
typedef enum
{
  MOTION_ACCEL_DONT_TEST,
  MOTION_ACCEL_OFF,
  MOTION_ACCEL_10_10,
  MOTION_ACCEL_MD,
  MOTION_ACCEL_MD_FIRED_10_10,
  MOTION_ACCEL_10_1,
} ma_test_phase;

/**
 * Sensor specific test data structure
 *
 * This is used for collecting any test diagnostic data while
 * test runs.
 *
 * Note: size of this struct shall not exceed SNS_TEST_DATA_SIZE
 */
typedef struct _sns_test_motion_accel_data
{
  float sample_rate;
  float report_rate;
  uint16_t  md_fired_count;
  uint16_t  md_armed_count;
  uint16_t  md_not_armed_count;
  ma_test_phase  phase;
  uint32_t  ma_10_10_count;
  uint32_t  ma_fired_10_10_count;
  uint32_t  ma_10_1_count;
} sns_test_motion_accel_data;

/**
 * Create a request for Sensor
 *
 * @param[o] payload  request payload. size of the payload shall 
 *  not exceed SNS_TEST_REQ_PAYLOAD_SIZE
 * @param[o] payload_fields  pb fields for request message.
 * @param[o] message_id  request message ID
 * @param[o] std_req  sns_std_request for Sensor under test.
 *
 * return none
 */
void sns_test_motion_accel_create_request(const sns_sensor *sensor,
                                   void *payload,
                                   const pb_field_t **payload_fields,
                                   uint32_t *message_id,
                                   sns_std_request *std_req);

/**
 * Processes events from the ensor
 *
 * @param[i] event       input event
 * @param[i] event_len   input event length
 * @param[i] test_data   data specific to test
 * @param[i] message_id  event message ID
 * @param[i] timestamp   event timestamp
 *
 * return none
 */
void sns_test_motion_accel_process_event(const sns_sensor *sensor,
                                  void *event,
                                  uint32_t event_len,
                                  void *test_data,
                                  uint32_t message_id,
                                  sns_time timestamp);

