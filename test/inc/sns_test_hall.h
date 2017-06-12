#pragma once
/**
 * @file sns_test_hall.h
 *
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * $Id:
 * //components/dev/ssc.slpi/3.0/kaushiks.ssc.slpi.3.0.napali_8996_19/sensors/test/inc/sns_test_hall.h#1
 * $ $DateTime: 2017/06/11 12:38:13 $ $Change: 13546828 $
 *
 **/

#include "sns_hall.pb.h"
#include "sns_mem_util.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_sensor_event.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_std.pb.h"
#include "sns_stream_service.h"
#include "sns_time.h"

/**
 * Create sensor request payload to be sent to sensor being
 * tested
 *
 * @param[io] payload - Pointer to payload buffer in which the
 *   request payload is passed back to the test framework.
 *   This payload is passed to the payload field in the
 *   sns_std_request as defined in sns_std.proto
 *   The length of the payload may not exceed
 *   SNS_TEST_REQ_PAYLOAD_LEN
 * @param[io] payload_fields - Protocol Buffer field descriptor
 *   that describes the payload
 * @param[io] message_id - Message ID to be sent with the
 *   request as defined in the .proto api of the sensor being
 *   tested
 *  @param[io] std_req - sns_std_request for Sensor under test.
 *
 * @return None
 */
void sns_test_create_hall_request(const sns_sensor *sensor,
                                      void *sensor_req,
                                      const pb_field_t **payload_fields,
                                      uint32_t *message_id,
                                      sns_std_request *std_req);


/**
 * Processes events from the Sensor
 *
 * @param[i] event       input event
 * @param[i] event_len   input event length
 * @param[i] test_data   data specific to test
 * @param[i] message_id  event message ID
 * @param[i] timestamp   event timestamp
 *
 * return none
 */
void sns_test_hall_process_event(const sns_sensor *sensor,
                                     void *event,
                                     uint32_t event_len,
                                     void *test_data,
                                     uint32_t message_id,
                                     sns_time timestamp);

