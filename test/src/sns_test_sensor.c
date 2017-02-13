/**
 * @file sns_test_sensor.c
 *
 * The test virtual Sensor implementation
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * $Id: //components/rel/ssc.slpi/3.0/sensors/test/src/sns_test_sensor.c#18 $
 * $DateTime: 2017/02/06 10:47:39 $
 * $Change: 12378096 $
 *
 **/

#include <string.h>
#include "sns_mem_util.h"
#include "sns_sensor.h"
#include "sns_sensor_instance.h"
#include "sns_test_sensor.h"
#include "sns_test_sensor_instance.h"
#include "sns_stream_service.h"
#include "sns_service_manager.h"
#include "sns_event_service.h"
#include "sns_attribute_service.h"
#include "sns_service.h"
#include "sns_sensor_util.h"
#include "sns_sensor_uid.h"
#include "sns_data_stream.h"
#include "sns_types.h"
#include "sns_request.h"
#include "sns_sensor_event.h"
#include "sns_sensor.h"
#include "sns_pb_util.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_suid.pb.h"
#include "sns_diag_service.h"

#define TEST_SUID 0x11,0xe8,0x65,0xd0,0xdd,0x70,0x4a,0x7e,\
                    0xaf,0x18,0x49,0x4e,0x3f,0x13,0x57,0x06

/* for sanity test on simulation */
#if defined(SNS_TEST_BUILD_GATING)
#define SNS_TEST_GRAVITY
#define NUM_EVENTS_TO_PROCESS 20
#define NUM_TEST_ITERATIONS 20
#else
/* for on-target test */
#define NUM_EVENTS_TO_PROCESS 1000
#define NUM_TEST_ITERATIONS 1
#endif


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
static const sns_test_implementation test_sensor_impl = {
  "accel",
  sizeof("accel"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_RESAMPLER)
#include "sns_test_resampler.h"
static const sns_test_implementation test_sensor_impl = {
  "resampler",
  sizeof("resampler"),
  sns_test_create_resampler_request,
  sns_test_resampler_process_event
};
#elif defined(SNS_TEST_GYRO_CAL)
#include "sns_test_gyro_cal.h"
static const sns_test_implementation test_sensor_impl = {
  "gyro_calibration",
  sizeof("gyro_calibration"),
  sns_test_create_gyro_cal_request,
  sns_test_gyro_cal_process_event
};
#elif defined(SNS_TEST_GYRO_ROT_MATRIX)
#include "sns_test_gyro_rot_matrix.h"
static const sns_test_implementation test_sensor_impl = {
  "gyro_rot_matrix",
  sizeof("gyro_rot_matrix"),
  sns_test_create_gyro_rot_matrix_request,
  sns_test_gyro_rot_matrix_process_event
};
#elif defined(SNS_TEST_GRAVITY)
#include "sns_test_gravity.h"
static const sns_test_implementation test_sensor_impl = {
  "gravity",
  sizeof("gravity"),
  sns_test_create_gravity_request,
  sns_test_gravity_process_event
};
#elif defined(SNS_TEST_SENSOR_TEMP)
#include "sns_test_std_sensor.h"
static const sns_test_implementation test_sensor_impl = {
  "sensor_temperature",
  sizeof("sensor_temperature"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_GYRO)
#include "sns_test_std_sensor.h"
static const sns_test_implementation test_sensor_impl = {
  "gyro",
  sizeof("gyro"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_PRESSURE)
#include "sns_test_std_sensor.h"
static const sns_test_implementation test_sensor_impl = {
  "pressure",
  sizeof("pressure"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_MAG)
#include "sns_test_std_sensor.h"
static const sns_test_implementation test_sensor_impl = {
  "mag",
  sizeof("mag"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_AMBIENT_LIGHT)
#include "sns_test_std_sensor.h"
static const sns_test_implementation test_sensor_impl = {
  "ambient_light",
  sizeof("ambient_light"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_PROXIMITY)
#include "sns_test_std_sensor.h"
static const sns_test_implementation test_sensor_impl = {
  "proximity",
  sizeof("proximity"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_HUMIDITY)
#include "sns_test_std_sensor.h"
static const sns_test_implementation test_sensor_impl = {
  "humidity",
  sizeof("humidity"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_RGB)
#include "sns_test_std_sensor.h"
static const sns_test_implementation test_sensor_impl = {
  "rgb",
  sizeof("rgb"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_UV)
#include "sns_test_std_sensor.h"
static const sns_test_implementation test_sensor_impl = {
  "uv",
  sizeof("uv"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_HALL)
#include "sns_test_std_sensor.h"
static const sns_test_implementation test_sensor_impl = {
  "hall",
  sizeof("hall"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_AMBIENT_TEMPERATRE)
#include "sns_test_std_sensor.h"
static const sns_test_implementation test_sensor_impl = {
  "ambient_temperature",
  sizeof("ambient_temperature"),
  sns_test_std_sensor_create_request,
  sns_test_std_sensor_process_event
};
#elif defined(SNS_TEST_MOTION_ACCEL)
#include "sns_test_motion_accel.h"
static const sns_test_implementation test_sensor_impl = {
  "md_motion_accel",
  sizeof("md_motion_accel"),
  sns_test_motion_accel_create_request,
  sns_test_motion_accel_process_event
};
#else
static const sns_test_implementation test_sensor_impl = {
  NULL, 0, NULL, NULL
};
#endif

/**
 * Publish all saved attributes for test_sensor.
 */
static void
publish_attributes(sns_sensor* const this)
{
  sns_test_state* state = (sns_test_state*)this->state->state;
  sns_service_manager* manager = this->cb->get_service_manager(this);
  sns_attribute_service* attribute_service =
      (sns_attribute_service*)manager->get_service(manager,
                                                   SNS_ATTRIBUTE_SERVICE);

  attribute_service->api->publish_attributes(attribute_service, this,
                                             state->attributes,
                                             ARR_SIZE(state->attributes));
}

/**
 * Initialize attributes to their default state.  They may/will be updated
 * within notify_event.
 */
static void
init_attributes(sns_sensor* const this)
{
  sns_test_state* state = (sns_test_state*)this->state->state;
  int8_t i = 0;

  static const char name[] = "test";
  static const char type[] = "test";
  static const char vendor[] = "template";
  static const bool available = true;
  static const uint32_t version = 1;

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_name,
    .value = (uintptr_t)&name,
    .value_len = strlen(name)
  };
  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_data_type,
    .value = (uintptr_t)&type,
    .value_len = strlen(type)
  };
  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_vendor,
    .value = (uintptr_t)&vendor,
    .value_len = strlen(vendor)
  };
  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_available,
    .value = (uintptr_t)&available,
    .value_len = 1
  };
  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_version,
    .value = (uintptr_t)&version,
    .value_len = 1
  };

}

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
  sns_diag_service* diag = state->diag_service;

  size_t encoded_len;
  uint8_t buffer[100];
  sns_memset(buffer, 0, sizeof(buffer));

  stream_mgr->api->create_sensor_stream(stream_mgr,
                                        this,
                                        suid,
                                        &state->sensor_stream);

  encoded_len = pb_encode_request(buffer, sizeof(buffer),
                                  payload, payload_fields, &std_req);

  if(0 < encoded_len && NULL != state->sensor_stream)
  {
    sns_request request = (sns_request){ .message_id = message_id,
      .request_len = encoded_len, .request = buffer };
    state->sensor_stream->api->send_request(state->sensor_stream, &request);
  } else
  {
    diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
                             "Failed to send sensor request, stream=%p",
                             state->sensor_stream);
  }
}

/**
 * @brief starts a stream for the tested sensor
 * @param this
 */
static void
sns_test_start_sensor_stream(sns_sensor* const this)
{
  sns_test_state* state = (sns_test_state*)this->state->state;
  void* payload = state->sns_pb_req_payload;
  uint32_t message_id;
  const pb_field_t* payload_fields = NULL;
  sns_std_request std_req = sns_std_request_init_default;

  state->test_sensor_create_request(this, payload, &payload_fields,
                                    &message_id, &std_req);

  sns_test_send_sensor_request(this, state->suid_search[0].suid, payload, payload_fields,
                               message_id, std_req);
}

/**
 * @brief stops the stream for the tested sensor
 * @param this
 */
static void
sns_test_stop_sensor_stream(sns_sensor* const this)
{
  sns_test_state* state = (sns_test_state*)this->state->state;
  sns_diag_service* diag = state->diag_service;
  if (NULL != state->sensor_stream)
  {
    diag->api->sensor_printf(diag, this, SNS_HIGH, __FILENAME__, __LINE__,
                             "removing stream for %s",
                             state->suid_search[0].data_type_str);
    sns_service_manager* service_mgr = this->cb->get_service_manager(this);
    sns_stream_service* stream_mgr =
        (sns_stream_service*)service_mgr->get_service(service_mgr,
                                                      SNS_STREAM_SERVICE);
    stream_mgr->api->remove_stream(stream_mgr, state->sensor_stream);
    state->sensor_stream = NULL;
  }
}

static sns_rc
sns_test_handle_sensor_event(sns_sensor* const this)
{
  sns_test_state* s = (sns_test_state*)this->state->state;

  sns_diag_service* diag = s->diag_service;

  for (; s->sensor_stream->api->get_input_cnt(s->sensor_stream) != 0 &&
       s->remaining_events > 0;
       s->sensor_stream->api->get_next_input(s->sensor_stream))
  {
    sns_sensor_event* e = s->sensor_stream->api->peek_input(s->sensor_stream);
    if (e == NULL)
    {
      diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
                               "event is NULL");
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
    sns_test_stop_sensor_stream(this);
    diag->api->sensor_printf(diag, this, SNS_HIGH, __FILENAME__, __LINE__,
                             "test iteration finished, remaining=%d",
                             s->remaining_iterations);
    if (s->remaining_iterations > 0)
    {
      s->remaining_events = NUM_EVENTS_TO_PROCESS;
      sns_test_start_sensor_stream(this);
    } else {
      diag->api->sensor_printf(diag, this, SNS_HIGH, __FILENAME__, __LINE__,
                               "test finished!");
      if (s->num_events_received == NUM_EVENTS_TO_PROCESS * NUM_TEST_ITERATIONS)
      {
        diag->api->sensor_printf(diag, this, SNS_HIGH, __FILENAME__, __LINE__,
                                 "result = PASS");
      }
      else
      {
        diag->api->sensor_printf(diag, this, SNS_HIGH, __FILENAME__, __LINE__,
                                 "result = FAIL");
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
static sns_rc
sns_test_handle_suid_event(sns_sensor* const this)
{
  sns_test_state* state = (sns_test_state*)this->state->state;
  sns_diag_service* diag = state->diag_service;

  diag->api->sensor_printf(diag, this, SNS_MED, __FILENAME__, __LINE__,__FUNCTION__);
 
  if(NULL != state->suid_stream &&
     0 < sns_process_suid_events(state->suid_stream, state->suid_search,
                                 state->search_count))
  {
    diag->api->sensor_printf(diag, this, SNS_MED, __FILENAME__, __LINE__,__FUNCTION__);
 
    bool ready = true;
    uint8_t i;
    for(i = 0; i<state->search_count; i++)
    {
      if(0 == sns_memcmp(&state->suid_search[i].suid,
                         &((sns_sensor_uid){{0}}),
                         sizeof(sns_sensor_uid)))
      {
         diag->api->sensor_printf(diag, this, SNS_MED, __FILENAME__, __LINE__,
                                 "%s failed",
                                 state->suid_search[i].data_type_str);
        ready = false;
      }
      else
      {
        diag->api->sensor_printf(diag, this, SNS_MED, __FILENAME__, __LINE__,
                                 "SUID for %s received",
                                 state->suid_search[i].data_type_str);
      }
    }
    if(ready)
    {
      diag->api->sensor_printf(diag, this, SNS_MED, __FILENAME__, __LINE__,
                               "Sensor streaming starts");
      sns_test_start_sensor_stream(this);
    }
  }

  return SNS_RC_SUCCESS;
}

/* See sns_sensor::notify_event */
sns_rc
sns_test_notify_event(sns_sensor* const this)
{
  sns_test_state* s = (sns_test_state*)this->state->state;

  sns_service_manager *smgr= this->cb->get_service_manager(this);
  s->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);

  sns_diag_service* diag = s->diag_service;

  diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);


  if (s->suid_stream)
  {
    /* process events from SUID sensor */
    sns_test_handle_suid_event(this);
  }

  if (s->sensor_stream)
  {
    /* event from tested sensor */
    sns_test_handle_sensor_event(this);
  }

  return SNS_RC_SUCCESS;
}

/* See sns_sensor::get_attributes */
static sns_sensor_attribute*
sns_test_get_attributes(sns_sensor const* const this,
                        uint32_t* attributes_len)
{
  sns_test_state* state = (sns_test_state*)this->state->state;

  sns_service_manager *smgr= this->cb->get_service_manager(this);
  state->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);

  sns_diag_service* diag = state->diag_service;

  diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);

  *attributes_len = ARR_SIZE(state->attributes);
  return state->attributes;
}

/* See sns_sensor::init */
static sns_rc
sns_test_init(sns_sensor* const this)
{
  sns_rc rc = SNS_RC_FAILED;
  sns_test_state* state = (sns_test_state*)this->state->state;
  init_attributes(this);
  publish_attributes(this);

  sns_service_manager *smgr= this->cb->get_service_manager(this);
  state->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);

  sns_diag_service* diag = state->diag_service;

  diag->api->sensor_printf(diag, this, SNS_MED, __FILENAME__, __LINE__,
                           "sns_test_init");
  diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
                           "sns_test_init");


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

    diag->api->sensor_printf(diag, this, SNS_MED, __FILENAME__, __LINE__,
                             "test_sensor type = %s",
                             test_sensor_impl.datatype);
    state->suid_search[state->search_count++].data_type_str = test_sensor_impl.datatype;
    if (strcmp("resampler", test_sensor_impl.datatype) == 0)
    {
      state->suid_search[state->search_count++].data_type_str = "gyro";
    }
    rc = sns_search_suids(this,state->suid_search,state->search_count, &state->suid_stream);
    if (rc != SNS_RC_SUCCESS)
    {
      diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,
                               "sns_search_suids() failed");
    }
  }

  diag->api->sensor_printf(diag, this, SNS_MED, __FILENAME__, __LINE__,
                           "sns_test_init done");

  return rc;
}


/* See sns_sensor::get_sensor_uid */
static sns_sensor_uid const*
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
static sns_sensor_instance*
sns_test_set_client_request(sns_sensor* const this,
                            struct sns_request* exist_request,
                            struct sns_request* new_request,
                            bool remove)
{
  UNUSED_VAR(this);
  UNUSED_VAR(exist_request);
  UNUSED_VAR(new_request);
  UNUSED_VAR(remove);
  return NULL;
}

static  sns_rc
sns_test_deinit(sns_sensor* const this)
{
  UNUSED_VAR(this);
  return SNS_RC_SUCCESS;
}

sns_sensor_api sns_test_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &sns_test_init,
  .deinit             = &sns_test_deinit,
  .get_sensor_uid     = &sns_test_get_sensor_uid,
  .get_attributes     = &sns_test_get_attributes,
  .set_client_request = &sns_test_set_client_request,
  .notify_event       = &sns_test_notify_event,
};

