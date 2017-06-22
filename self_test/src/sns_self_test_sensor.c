/**
 * @file sns_self_test_sensor.c
 *
 * The self test Sensor implementation
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * $Id:  $
 * $DateTime:  $
 * $Change:  $
 *
 **/

#include <string.h>
#include "pb_decode.h"
#include "pb_encode.h"
#include "sns_attribute_service.h"
#include "sns_attribute_util.h"
#include "sns_data_stream.h"
#include "sns_diag_service.h"
#include "sns_event_service.h"
#include "sns_mem_util.h"
#include "sns_pb_util.h"
#include "sns_physical_sensor_test.pb.h"
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
#include "sns_self_test_sensor.h"
#include "sns_types.h"
#include "sns_printf.h"

#define TEST_SELF_SUID 0xcc, 0x43, 0x1f, 0x46, 0x2f, 0x7e, 0x53, 0x92, \
                     0x9f, 0x7a, 0x6c, 0x7f, 0x75, 0x48, 0xda, 0xbf

#if defined(SNS_SELF_TEST_ACCEL)
char *sensor_type = "accel";
#elif defined(SNS_SELF_TEST_GYRO)
char *sensor_type = "gyro";
#elif defined(SNS_SELF_TEST_MAG)
char *sensor_type = "mag";
#elif defined(SNS_SELF_TEST_PRESSURE)
char *sensor_type = "pressure";
#elif defined(SNS_SELF_TEST_AMBIENT_LIGHT)
char *sensor_type = "ambient_light";
#elif defined(SNS_SELF_TEST_PROXIMITY)
char *sensor_type = "proximity";
#elif defined(SNS_SELF_TEST_AMBIENT_TEMPERATURE)
char *sensor_type = "ambient_temperature";
#elif defined(SNS_SELF_TEST_HUMIDITY)
char *sensor_type = "humidity";
#elif defined(SNS_SELF_TEST_RGB)
char *sensor_type = "rgb";
#elif defined(SNS_SELF_TEST_UV)
char *sensor_type = "ultra_violet";
#elif defined(SNS_SELF_TEST_HALL)
char *sensor_type = "hall";
#else
char *sensor_type = "accel";
#endif

#if defined(SNS_TEST_TYPE_HW)
sns_physical_sensor_test_type sensor_test_type = SNS_PHYSICAL_SENSOR_TEST_TYPE_HW;
#elif defined(SNS_TEST_TYPE_SW)
sns_physical_sensor_test_type sensor_test_type = SNS_PHYSICAL_SENSOR_TEST_TYPE_SW;
#elif defined(SNS_TEST_TYPE_COM)
sns_physical_sensor_test_type sensor_test_type = SNS_PHYSICAL_SENSOR_TEST_TYPE_COM;
#elif defined(SNS_TEST_TYPE_FACTORY)
sns_physical_sensor_test_type sensor_test_type = SNS_PHYSICAL_SENSOR_TEST_TYPE_FACTORY;
#else
sns_physical_sensor_test_type sensor_test_type = SNS_PHYSICAL_SENSOR_TEST_TYPE_COM;
#endif


/**
 * Publish all saved attributes for self_test.
 */
static void
publish_attributes(sns_sensor* const this)
{
  {
    char const name[] = "self_test";
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = name, .buf_len = sizeof(name) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_NAME, &value, 1, false);
  }
  {
    char const type[] = "self_test";
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.str.funcs.encode = pb_encode_string_cb;
    value.str.arg = &((pb_buffer_arg)
        { .buf = type, .buf_len = sizeof(type) });
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_TYPE, &value, 1, false);
  }
  {
    char const vendor[] = "template";
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

static void sns_self_test_start_sensor_stream(sns_sensor* const this)
{
  sns_self_test_state* state = (sns_self_test_state*)this->state->state;
  //sns_diag_service *diag = state->diag_service;
  sns_service_manager* service_mgr = this->cb->get_service_manager(this);
  sns_stream_service* stream_mgr =
      (sns_stream_service*)service_mgr->get_service(service_mgr,
                                                    SNS_STREAM_SERVICE);
  size_t encoded_len;
  uint8_t buffer[100];
  sns_memset(buffer, 0, sizeof(buffer));
  sns_physical_sensor_test_config self_test_req = sns_physical_sensor_test_config_init_default;
  self_test_req.test_type = sensor_test_type;

  SNS_PRINTF(ERROR, this, "sns_self_test_start_sensor_stream");

  state->test_in_progress = true;

  if(NULL == state->sensor_stream)
  {
    stream_mgr->api->create_sensor_stream(stream_mgr, this,
                                          state->suid_search[0].suid[0], &state->sensor_stream);
  }

  encoded_len = pb_encode_request(buffer, sizeof(buffer),
                                  &self_test_req, sns_physical_sensor_test_config_fields, NULL);

  if(0 < encoded_len && NULL != state->sensor_stream)
  {
    sns_request request =
       (sns_request){ .message_id = SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG,
      .request_len = encoded_len, .request = buffer };
    state->sensor_stream->api->send_request(state->sensor_stream, &request);
  }
  else
  {
    SNS_PRINTF(ERROR,this, 
                             "Failed to send sensor request, stream=%p",
                             state->sensor_stream);
  }
}

static bool sns_self_test_is_ready(sns_sensor* const this)
{
  int i;
  bool ready = true;
  sns_self_test_state* state = (sns_self_test_state*)this->state->state;

  for(i = 0; i < state->suid_search_count; i++)
  {
    if(0 == sns_memcmp(&state->suid_search[i].suid[0],
                       &((sns_sensor_uid){{0}}),
                       sizeof(sns_sensor_uid)))
    {
      ready = false;
      break;
    }
  }

  return ready;
}

/**
 * Handles events from the SUID discovery sensor.
 *
 * Starts stream for the tested sensor when its SUID is available.
 *
 * @param[i] this      Reference to this Sensor
 *
 * @return sns_rc
 */
static sns_rc sns_self_test_handle_suid_event(sns_sensor* const this)
{
  sns_self_test_state* state = (sns_self_test_state*)this->state->state;
  //sns_diag_service* diag = state->diag_service;

  if(NULL != state->suid_stream)
  {
    for(; state->suid_stream->api->get_input_cnt(state->suid_stream) != 0;
        state->suid_stream->api->get_next_input(state->suid_stream))
    {
      sns_sensor_event* e = state->suid_stream->api->peek_input(state->suid_stream);
      pb_istream_t stream = pb_istream_from_buffer((void*)e->event, e->event_len);
      sns_suid_event suid_event = sns_suid_event_init_default;
      pb_buffer_arg data_type_arg = { .buf = NULL, .buf_len = 0 };
      sns_sensor_uid uid_list[5];
      sns_suid_search suid_search;
      suid_search.suid = &uid_list[0];
      suid_search.num_of_suids = 0;
      int i;

      suid_event.data_type.funcs.decode = &pb_decode_string_cb;
      suid_event.data_type.arg = &data_type_arg;
      suid_event.suid.funcs.decode = &pb_decode_suid_event;
      suid_event.suid.arg = &suid_search;

      if(!pb_decode(&stream, sns_suid_event_fields, &suid_event)) {
         //diag->api->sensor_printf(diag, this, ERROR, __FILENAME__, __LINE__,
          //                        "Error decoding SUID Event: %s",
           //                       PB_GET_ERROR(&stream));
         continue;
       }

      for(i = 0; i < state->suid_search_count; i++)
      {
        if(0 == strncmp(data_type_arg.buf,
                         state->suid_search[i].data_type_str,
                         data_type_arg.buf_len))
        {
          int j;
          for(j = 0;
              (j < suid_search.num_of_suids) && (j < state->suid_search[i].num_of_suids);
              j++)
          {
            sns_memscpy(&state->suid_search[i].suid[j],
                        sizeof(state->suid_search[i].suid[j]),
                        &suid_search.suid[j],
                        sizeof(suid_search.suid[j]));
          }
          break;
        }
      }

      if(sns_self_test_is_ready(this) && !state->test_in_progress)
      {
        SNS_PRINTF(ERROR,this,
                                 "Sensor streaming starts");
        sns_self_test_start_sensor_stream(this);
        break;
      }
    }
  }

  return SNS_RC_SUCCESS;
}

static void sns_self_test_handle_sensor_event(sns_sensor* const this)
{
  sns_self_test_state* s = (sns_self_test_state*)this->state->state;
  //sns_diag_service* diag = s->diag_service;

  for(; s->sensor_stream->api->get_input_cnt(s->sensor_stream) != 0;
       s->sensor_stream->api->get_next_input(s->sensor_stream))
  {
    sns_sensor_event* e = s->sensor_stream->api->peek_input(s->sensor_stream);
    if(e != NULL)
    {
      switch(e->message_id)
      {
        case SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG:
        {
           SNS_PRINTF( ERROR,this,
                                   "Received SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG");
        }
        break;
        case SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_EVENT:
        {
          SNS_PRINTF(ERROR,this,
                                   "Received SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_EVENT");
          pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)e->event,
                                                      e->event_len);
          pb_buffer_arg test_data;
          sns_physical_sensor_test_event test_event = sns_physical_sensor_test_event_init_default;
          test_event.test_data.funcs.decode = &pb_decode_string_cb;
          test_event.test_data.arg = &test_data;

          if (!pb_decode(&stream, sns_physical_sensor_test_event_fields, &test_event))
          {
            SNS_PRINTF(ERROR,this,
                                    "pb_decode() failed for stream_event");
          }
          SNS_PRINTF( ERROR,this,
                                "test_passed %d", test_event.test_passed);
         }
         break;

        default:
          //diag->api->sensor_printf(diag, this, ERROR, __FILENAME__, __LINE__,
                                   //"Unknown event message_id %d", e->message_id);
        break;
      }
    }
  }
}

/* See sns_sensor::notify_event */
static sns_rc sns_self_test_notify_event(sns_sensor* const this)
{
  sns_self_test_state* s = (sns_self_test_state*)this->state->state;

  if(s->suid_stream)
  {
    /* process events from SUID sensor */
    sns_self_test_handle_suid_event(this);
  }

  if(s->sensor_stream)
  {
    /* event from tested sensor */
    sns_self_test_handle_sensor_event(this);
  }

  return SNS_RC_SUCCESS;
}

/* See sns_sensor::init */
static sns_rc sns_self_test_init(sns_sensor* const this)
{
  sns_rc rc = SNS_RC_FAILED;
  sns_self_test_state* state =
     (sns_self_test_state*)this->state->state;
  publish_attributes(this);

  sns_service_manager *smgr = this->cb->get_service_manager(this);
  state->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);

  //sns_diag_service* diag = state->diag_service;

  SNS_PRINTF(ERROR,this,
                           "sns_self_test_init");

  state->suid_search[state->suid_search_count].data_type_str = sensor_type;
  state->suid_search[state->suid_search_count].suid = &state->sensor_suid;
  state->suid_search[state->suid_search_count++].num_of_suids = 1;

  rc = sns_search_suids(this, state->suid_search,
                        state->suid_search_count, &state->suid_stream);
  if (rc != SNS_RC_SUCCESS)
  {
    //diag->api->sensor_printf(diag, this, ERROR, __FILENAME__, __LINE__,
    //                         "sns_search_suids() failed");
  }

  //diag->api->sensor_printf(diag, this, ERROR, __FILENAME__, __LINE__,
  //                         "sns_self_test_init done");

  return rc;
}

/* See sns_sensor::get_sensor_uid */
static sns_sensor_uid const* sns_self_test_get_sensor_uid(
   sns_sensor const* const this)
{
  UNUSED_VAR(this);
  static const sns_sensor_uid sensor_uid =
  {
    .sensor_uid = { TEST_SELF_SUID }
  };

  return &sensor_uid;
}

/* See sns_sensor::set_client_request */
static sns_sensor_instance*
sns_self_test_set_client_request(sns_sensor* const this,
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

static  sns_rc
sns_self_test_deinit(sns_sensor* const this)
{
  UNUSED_VAR(this);
  return SNS_RC_SUCCESS;
}

sns_sensor_api sns_self_test_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &sns_self_test_init,
  .deinit             = &sns_self_test_deinit,
  .get_sensor_uid     = &sns_self_test_get_sensor_uid,
  .set_client_request = &sns_self_test_set_client_request,
  .notify_event       = &sns_self_test_notify_event,
};

