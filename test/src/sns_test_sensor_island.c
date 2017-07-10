/**
 * @file sns_test_sensor_island.c
 *
 * The test virtual Sensor implementation
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * $Id: //components/dev/ssc.slpi/3.0/maansyw.ssc.slpi.3.0.napali_06_11/sensors/test/src/sns_test_sensor_island.c#1 $
 * $DateTime: 2017/06/11 12:38:13 $
 * $Change: 13546828 $
 *
 **/

#include "pb_decode.h"
#include "pb_encode.h"
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
#include "sns_physical_sensor_test.pb.h"

#define SNS_SELF_TEST_AKM 1
#define SNS_SELF_TEST_TYPE 1

void
sns_test_send_sensor_request(sns_sensor* const this,
                             sns_sensor_uid suid,
                             void* payload,
                             pb_field_t const* payload_fields,
                             uint32_t message_id,
                             sns_std_request std_req)
{
  sns_test_state* state = (sns_test_state*)this->state->state;
  sns_service_manager* service_mgr = this->cb->get_service_manager(this);
  sns_stream_service* stream_mgr =
      (sns_stream_service*)service_mgr->get_service(service_mgr,
                                                    SNS_STREAM_SERVICE);

  size_t encoded_len;
  uint8_t buffer[100];
  sns_memset(buffer, 0, sizeof(buffer));

  stream_mgr->api->create_sensor_stream(stream_mgr,
                                        this,
                                        suid,
                                        &state->sensor_stream);

  if(message_id == SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG)
  {
    SNS_PRINTF(LOW, this, "Sending on-change reuest");
    sns_request request = (sns_request){ .message_id = message_id,
      .request_len = 0, .request = NULL };
    state->sensor_stream->api->send_request(state->sensor_stream, &request);
  }
  else
  {
    encoded_len = pb_encode_request(buffer, sizeof(buffer),
                                    payload, payload_fields, &std_req);

    if(0 < encoded_len && NULL != state->sensor_stream)
    {
      sns_request request = (sns_request){ .message_id = message_id,
        .request_len = encoded_len, .request = buffer };
      state->sensor_stream->api->send_request(state->sensor_stream, &request);
    }
    else
    {
      SNS_PRINTF(ERROR, this, "Failed to send sensor request, stream=%p", state->sensor_stream);
    }
  }
}

void sns_self_test_start_sensor_stream(sns_sensor* const this)
{
  sns_test_state* state = (sns_test_state*)this->state->state;
  //sns_service_manager* service_mgr = this->cb->get_service_manager(this);
  //sns_stream_service* stream_mgr =
  //  (sns_stream_service*)service_mgr->get_service(service_mgr,SNS_STREAM_SERVICE);

  size_t encoded_len;
  uint8_t buffer[100];
  sns_memset(buffer, 0, sizeof(buffer));
  sns_physical_sensor_test_config self_test_req = sns_physical_sensor_test_config_init_default;
  if (SNS_SELF_TEST_TYPE == 1)
  {
    self_test_req.test_type = SNS_PHYSICAL_SENSOR_TEST_TYPE_HW;
  }
  else
  {
    self_test_req.test_type = SNS_PHYSICAL_SENSOR_TEST_TYPE_COM;
  }

  SNS_PRINTF(ERROR, this, "sns_self_test_start_sensor_stream");
 
  encoded_len = pb_encode_request(buffer, sizeof(buffer),
                    &self_test_req, sns_physical_sensor_test_config_fields, NULL);

  if (0 < encoded_len && NULL != state->sensor_stream)
  {
    sns_request request =
      (sns_request){ .message_id = SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG,
        .request_len = encoded_len, .request = buffer };
    state->sensor_stream->api->send_request(state->sensor_stream, &request);
  }
  else
  {
    SNS_PRINTF(ERROR, this, "Failed to send sensor request");
  }
}

/**
 * @brief starts a stream for the tested sensor
 * @param this
 */
 void
sns_test_start_sensor_stream(sns_sensor* const this)
{
  sns_test_state* state = (sns_test_state*)this->state->state;
  void* payload = state->sns_pb_req_payload;
  uint32_t message_id;
  const pb_field_t* payload_fields = NULL;
  sns_std_request std_req = sns_std_request_init_default;

  state->test_sensor_create_request(this, payload, &payload_fields,
                                    &message_id, &std_req);

  /** TODO create stream for all SUIDs found for sensor type
   *  under test */
  sns_test_send_sensor_request(this, state->suid_search[0].suid[0],
                               payload, payload_fields, message_id, std_req);
  state->test_in_progress = true;
}

/**
 * @brief stops the stream for the tested sensor
 * @param this
 */
 void
sns_test_stop_sensor_stream(sns_sensor* const this)
{
  sns_test_state* state = (sns_test_state*)this->state->state;
  if (NULL != state->sensor_stream)
  {
    //SENSOR_PRINTF_HIGH_FULL(this, "removing stream for %s", state->suid_search[0].data_type_str);
    sns_service_manager* service_mgr = this->cb->get_service_manager(this);
    sns_stream_service* stream_mgr =
        (sns_stream_service*)service_mgr->get_service(service_mgr,
                                                      SNS_STREAM_SERVICE);
    stream_mgr->api->remove_stream(stream_mgr, state->sensor_stream);
    state->sensor_stream = NULL;
  }
}

 sns_rc
sns_test_handle_sensor_event(sns_sensor* const this)
{
  sns_test_state* s = (sns_test_state*)this->state->state;

  for (; s->sensor_stream->api->get_input_cnt(s->sensor_stream) != 0 &&
       s->remaining_events > 0;
       s->sensor_stream->api->get_next_input(s->sensor_stream))
  {
    SNS_PRINTF(ERROR, this, "sns_test_handle_sensor_event:before peek_input");
    sns_sensor_event* e = s->sensor_stream->api->peek_input(s->sensor_stream);
    if (e == NULL)
    {
      SNS_PRINTF(ERROR, this, "event is NULL");
      continue;
    }
    s->test_sensor_process_event(this, e->event, e->event_len, s->test_data,
                                 e->message_id, e->timestamp);
    s->remaining_events--;
    s->num_events_received++;
  }

  /* if all events are processed, stop the stream */
  if (s->remaining_events <= 0)
  {
    s->remaining_iterations--;
    if (SNS_SELF_TEST_AKM == 1)
    {
      static int i;
      if (i == 0)
      {
        SNS_PRINTF(ERROR, this, "sns_self_test_start_sensor_stream");
        sns_self_test_start_sensor_stream(this);

        s->remaining_events = NUM_EVENTS_TO_PROCESS;
        i++;
      }
      else
      {
         sns_test_stop_sensor_stream(this);
         SNS_PRINTF(ERROR, this, "sns_test_stop_sensor_stream");
      }
    }
    else
    {
      sns_test_stop_sensor_stream(this);
    }
    SNS_PRINTF(HIGH, this, "test iteration finished, remaining=%d",
                             s->remaining_iterations);
    if (s->remaining_iterations > 0)
    {
      s->remaining_events = NUM_EVENTS_TO_PROCESS;
      sns_test_start_sensor_stream(this);
    } else {
      SNS_PRINTF(HIGH, this, "test finished!");
      if (s->num_events_received == NUM_EVENTS_TO_PROCESS * NUM_TEST_ITERATIONS)
      {
        SNS_PRINTF(HIGH, this, "result = PASS");
      }
      else
      {
        SNS_PRINTF(HIGH, this, "result = FAIL");
      }
    }
  }

  return SNS_RC_SUCCESS;
}

/**
 * @brief handles events from the SUID discovery sensor
 *
 * Starts stream for the tested sensor when its SUID is available.
 *
 * @param[i] this
 *
 * @return sns_rc
 */
 sns_rc
sns_test_handle_suid_event(sns_sensor* const this)
{
  sns_test_state* state = (sns_test_state*)this->state->state;

  if(NULL != state->suid_stream)
  {
    SNS_PRINTF(ERROR, this, "sns_test_handle_suid_event1");
    for(; state->suid_stream->api->get_input_cnt(state->suid_stream) != 0 &&
        state->remaining_events > 0;
        state->suid_stream->api->get_next_input(state->suid_stream))
    {
      SNS_PRINTF(ERROR, this, "sns_test_handle_suid_event2");
 
      sns_sensor_event* e = state->suid_stream->api->peek_input(state->suid_stream);
      pb_istream_t stream = pb_istream_from_buffer((void*)e->event, e->event_len);
      sns_suid_event suid_event = sns_suid_event_init_default;
      pb_buffer_arg data_type_arg = { .buf = NULL, .buf_len = 0 };
      sns_sensor_uid uid_list[5];
      sns_suid_search suid_search;
      suid_search.suid = &uid_list[0];
      suid_search.num_of_suids = 0;
      int i;
      bool ready = false;

      suid_event.data_type.funcs.decode = &pb_decode_string_cb;
      suid_event.data_type.arg = &data_type_arg;
      suid_event.suid.funcs.decode = &pb_decode_suid_event;
      suid_event.suid.arg = &suid_search;

      if(!pb_decode(&stream, sns_suid_event_fields, &suid_event)) {
         SNS_PRINTF(ERROR, this, "Error decoding SUID Event");
         continue;
       }

      for(i = 0; i < state->search_count; i++)
      {
        if(0 == strncmp(data_type_arg.buf,
                         state->suid_search[i].data_type_str,
                         data_type_arg.buf_len))
        {
          int j;
          for(j = 0; j < suid_search.num_of_suids; j++)
          {
            sns_memscpy(&state->suid_search[i].suid[j],
                        sizeof(state->suid_search[i].suid[j]),
                        &suid_search.suid[j],
                        sizeof(suid_search.suid[j]));
            ready = true;
          }

          if(ready)
          {
            //SENSOR_PRINTF_MED_FULL(this, "SUID for %s received", state->suid_search[i].data_type_str);
          }
          break;
        }
      }

      if(ready && !state->test_in_progress)
      {
        SNS_PRINTF(ERROR, this, "Sensor streaming starts");
        sns_test_start_sensor_stream(this);
        break;
      }
    }
  }

  return SNS_RC_SUCCESS;
}

/* See sns_sensor::notify_event */
sns_rc
sns_test_notify_event(sns_sensor* const this)
{

  SNS_PRINTF(ERROR, this, "sns_test_notify_event");
 
  sns_test_state* s = (sns_test_state*)this->state->state;

  if(s->suid_stream)
  {
    SNS_PRINTF(ERROR, this, "sns_test_notify_event:suid_event");
 
    /* process events from SUID sensor */
    sns_test_handle_suid_event(this);
  }

  if(s->sensor_stream)
  {
    SNS_PRINTF(ERROR, this, "sns_test_notify_event:sensor_event");
 
    /* event from tested sensor */
    sns_test_handle_sensor_event(this);
  }

  return SNS_RC_SUCCESS;
}

/* See sns_sensor::get_sensor_uid */
 sns_sensor_uid const*
sns_test_get_sensor_uid(sns_sensor const* const this)
{
  UNUSED_VAR(this);
   static const sns_sensor_uid sensor_uid =
  {
    .sensor_uid = { TEST_SUID }
  };

  return &sensor_uid;
}

/* See sns_sensor::set_client_request */
 sns_sensor_instance*
sns_test_set_client_request(sns_sensor* const this,
                            struct sns_request const *exist_request,
                            struct sns_request const *new_request,
                            bool remove)
{
  UNUSED_VAR(this);
  UNUSED_VAR(exist_request);
  UNUSED_VAR(new_request);
  UNUSED_VAR(remove);
  return NULL;
}

sns_sensor_api sns_test_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &sns_test_init,
  .deinit             = &sns_test_deinit,
  .get_sensor_uid     = &sns_test_get_sensor_uid,
  .set_client_request = &sns_test_set_client_request,
  .notify_event       = &sns_test_notify_event,
};

