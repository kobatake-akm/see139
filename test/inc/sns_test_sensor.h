#pragma once
/**
 * @file sns_test_sensor.h
 *
 * The test virtual Sensor.
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * $Id: //components/rel/ssc.slpi/3.0/sensors/test/inc/sns_test_sensor.h#10 $
 * $DateTime: 2017/03/17 17:13:42 $
 * $Change: 12750453 $
 *
 **/

#include "sns_sensor.h"
#include "sns_data_stream.h"
#include "sns_std_sensor.pb.h"
#include "sns_diag_service.h"
#include "sns_sensor_util.h"
#include "sns_std.pb.h"
#include "sns_test_std_sensor.h"

/** Forward Declaration of Sensor API */
sns_sensor_api sns_test_sensor_api;

#define SNS_TEST_REQ_PAYLOAD_SIZE 128
#define SNS_TEST_DATA_SIZE 256

typedef void (* sns_test_create_request_func)(const sns_sensor *sensor,
                                              void*, const pb_field_t**,
                                              uint32_t*, sns_std_request*);
typedef void (* sns_test_process_event_func)(const sns_sensor *sensor,
                                             void*, uint32_t, void*,
                                             uint32_t, sns_time);

typedef struct _sns_test_state
{
  sns_data_stream* sensor_stream;
  sns_data_stream* suid_stream;
  sns_diag_service *diag_service;
  int32_t remaining_events;
  int32_t remaining_iterations;
  uint32_t num_events_received;
  sns_suid_search suid_search[2]; /* resampler uses 2, other use just 1 */
  uint8_t         search_count;
  uint64_t sns_pb_req_payload[SNS_TEST_REQ_PAYLOAD_SIZE/8];
  uint64_t test_data[SNS_TEST_DATA_SIZE/8];
  sns_test_create_request_func test_sensor_create_request;
  sns_test_process_event_func test_sensor_process_event;
} sns_test_state;

