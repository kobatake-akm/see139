/**
 * @file sns_lsm6ds3_sensor.c
 *
 * Common implementation for LSM6DS3 Sensors.
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include <string.h>
#include "sns_attribute_util.h"
#include "sns_mem_util.h"
#include "sns_sensor_util.h"
#include "sns_service.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_types.h"

#include "sns_lsm6ds3_sensor.h"

#include "pb_decode.h"
#include "pb_encode.h"
#include "sns_pb_util.h"
#include "sns_suid.pb.h"

void lsm6ds3_send_suid_req(sns_sensor *this, char *const data_type,
    uint32_t data_type_len)
{
  uint8_t buffer[50];
  sns_memset(buffer, 0, sizeof(buffer));
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;
  sns_service_manager *manager = this->cb->get_service_manager(this);
  sns_stream_service *stream_service =
    (sns_stream_service*)manager->get_service(manager, SNS_STREAM_SERVICE);
  size_t encoded_len;
  pb_buffer_arg data = (pb_buffer_arg){ .buf = data_type, .buf_len = data_type_len };

  sns_suid_req suid_req = sns_suid_req_init_default;
  suid_req.has_register_updates = true;
  suid_req.register_updates = true;
  suid_req.data_type.funcs.encode = &pb_encode_string_cb;
  suid_req.data_type.arg = &data;

  if(state->fw_stream == NULL)
  {
     stream_service->api->create_sensor_stream(stream_service,
         this, sns_get_suid_lookup(), &state->fw_stream);
  }

  encoded_len = pb_encode_request(buffer, sizeof(buffer),
      &suid_req, sns_suid_req_fields, NULL);
  if(0 < encoded_len)
  {
    sns_request request = (sns_request){
      .request_len = encoded_len, .request = buffer, .message_id = SNS_SUID_MSGID_SNS_SUID_REQ };
    state->fw_stream->api->send_request(state->fw_stream, &request);
  }
}

