/**
 * @file sns_ak0991x_dae_if_island.c
 *
 * AK0991X - DAE sensor interface
 *
 * Copyright (c) 2017 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include "sns_mem_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_types.h"
//#include "sns_event_service.h"

#include "sns_ak0991x_hal.h"
#include "sns_ak0991x_sensor.h"
#include "sns_ak0991x_sensor_instance.h"

#include "sns_dae.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_diag_service.h"


/*======================================================================================
  Helper Functions
  ======================================================================================*/

/* ------------------------------------------------------------------------------------ */
static bool send_mag_config(ak0991x_dae_stream *dae_stream, ak0991x_mag_info* mag_info)
{
  bool cmd_sent = false;
  sns_dae_set_streaming_config config_req = sns_dae_set_streaming_config_init_default;
  uint8_t encoded_msg[sns_dae_set_streaming_config_size];
  sns_request req = {
    .message_id   = SNS_DAE_MSGID_SNS_DAE_SET_STREAMING_CONFIG,
    .request      = encoded_msg
  };

  config_req.dae_watermark       = mag_info->cur_wmk;
  config_req.has_polling_config  = false;

  //TODO, To keep the variable name for compile
  config_req.has_accel_info      = true;
  config_req.accel_info.odr      = ak0991x_get_mag_odr(mag_info->curr_odr);

  if((req.request_len = 
      pb_encode_request(encoded_msg, 
                        sizeof(encoded_msg), 
                        &config_req,
                        sns_dae_set_streaming_config_fields, 
                        NULL)) > 0)
  {
    if(SNS_RC_SUCCESS == dae_stream->stream->api->send_request(dae_stream->stream, &req))
    {
      cmd_sent = true;
    }
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
static bool flush_hw(ak0991x_dae_stream *dae_stream)
{
  bool cmd_sent = false;
  sns_request req = {
    .message_id   = SNS_DAE_MSGID_SNS_DAE_FLUSH_HW,
    .request      = NULL,
    .request_len  = 0
  };

  if(SNS_RC_SUCCESS == dae_stream->stream->api->send_request(dae_stream->stream, &req))
  {
    cmd_sent = true;
    dae_stream->flushing_hw = true;
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
static bool flush_samples(ak0991x_dae_stream *dae_stream)
{
  bool cmd_sent = false;
  sns_request req = {
    .message_id   = SNS_DAE_MSGID_SNS_DAE_FLUSH_DATA_EVENTS,
    .request      = NULL,
    .request_len  = 0
  };

  if(SNS_RC_SUCCESS == dae_stream->stream->api->send_request(dae_stream->stream, &req))
  {
    cmd_sent = true;
    dae_stream->flushing_data = true;
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
static bool stop_streaming(ak0991x_dae_stream *dae_stream)
{
  bool cmd_sent = false;
  sns_request req = {
    .message_id   = SNS_DAE_MSGID_SNS_DAE_PAUSE_SAMPLING,
    .request      = NULL,
    .request_len  = 0
  };

  if(SNS_RC_SUCCESS == dae_stream->stream->api->send_request(dae_stream->stream, &req))
  {
    cmd_sent = true;
    dae_stream->state = STREAM_STOPPING;
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
static void process_fifo_samples(
  sns_sensor_instance *this, 
  sns_time            timestamp,
  uint8_t             *buf, 
  size_t              buf_len)
{
  uint8_t *fifo_start = buf + 1; /* 1st byte = CNTRL2 */
  uint16_t num_sample_sets = (buf_len / 8);
  ak0991x_mag_odr mag_odr = (ak0991x_mag_odr)(*buf & 0x1F);
  sns_time sampling_intvl = ak0991x_get_sample_interval(mag_odr);

  if(num_sample_sets >= 1 && sampling_intvl > 0)
  {
    sns_time first_timestamp = timestamp - sampling_intvl * (num_sample_sets - 1);
    ak0991x_process_fifo_data_buffer(this, 
                                     first_timestamp, 
                                     sampling_intvl,
                                     fifo_start, 
                                     buf_len - 1);

  }
}

/* ------------------------------------------------------------------------------------ */
static void process_data_event(
  sns_sensor_instance *this, 
  ak0991x_dae_stream  *dae_stream,
  pb_istream_t        *pbstream)
{
  pb_buffer_arg decode_arg;
  sns_dae_data_event data_event = sns_dae_data_event_init_default;
  data_event.sensor_data.funcs.decode = &pb_decode_string_cb;
  data_event.sensor_data.arg = &decode_arg;
  if(pb_decode(pbstream, sns_dae_data_event_fields, &data_event))
  {
    ak0991x_dae_if_info *dae_if = &((ak0991x_instance_state*)this->state->state)->dae_if;
    if (dae_stream == &dae_if->mag)
    {
      process_fifo_samples(
        this, data_event.timestamp, (uint8_t*)decode_arg.buf, decode_arg.buf_len);
    }
  }
}

/* ------------------------------------------------------------------------------------ */
static bool process_response(
  sns_sensor_instance *this, 
  ak0991x_dae_stream  *dae_stream,
  pb_istream_t        *pbstream)
{
  bool stream_usable = true;
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;

  sns_dae_resp resp = sns_dae_resp_init_default;
  if(pb_decode(pbstream, sns_dae_resp_fields, &resp))
  {
    switch(resp.msg_id)
    {
    case SNS_DAE_MSGID_SNS_DAE_SET_STATIC_CONFIG:
      if(SNS_STD_ERROR_NO_ERROR != resp.err)
      {
        /* DAE sensor does not have support for this driver */
        stream_usable = false;
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_S4S_DYNAMIC_CONFIG:
      break;
    case SNS_DAE_MSGID_SNS_DAE_SET_STREAMING_CONFIG:
      if(dae_stream->stream != NULL && dae_stream->state == STREAM_STARTING)
      {
        dae_stream->state = (SNS_STD_ERROR_NO_ERROR == resp.err) ? STREAMING : IDLE;
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_FLUSH_HW:
      if(state->config_step != AK0991X_CONFIG_IDLE)
      {
        ak0991x_start_mag_streaming(state);
        state->config_step = AK0991X_CONFIG_IDLE;
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_FLUSH_DATA_EVENTS:
      if(state->fifo_flush_in_progress)
      {
        state->fifo_flush_in_progress = false;
        ak0991x_send_fifo_flush_done(this);
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_PAUSE_SAMPLING:
      if(dae_stream->state == STREAM_STOPPING)
      {
        dae_stream->state = (SNS_STD_ERROR_NO_ERROR != resp.err) ? STREAMING : IDLE;
      }
      if(NULL != state->dae_if.mag.stream && 
         state->dae_if.mag.state != STREAM_STOPPING)
      {
        /* done waiting */
        if(state->config_step == AK0991X_CONFIG_STOPPING_STREAM)
        {
          ak0991x_dae_if_flush_hw(this);
        }
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_PAUSE_S4S_SCHED:
      break;

    case SNS_DAE_MSGID_SNS_DAE_RESP:
    case SNS_DAE_MSGID_SNS_DAE_DATA_EVENT:
      break; /* unexpected */
    }
  }
  return stream_usable;
}

/* ------------------------------------------------------------------------------------ */
static void process_events(sns_sensor_instance *this, ak0991x_dae_stream *dae_stream)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  sns_sensor_event *event;
  bool stream_usable = true;

  while(NULL != dae_stream->stream && 
        NULL != (event = dae_stream->stream->api->peek_input(dae_stream->stream)))
  {
    if (stream_usable)
    {
      pb_istream_t pbstream = 
        pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);

      if (SNS_DAE_MSGID_SNS_DAE_DATA_EVENT == event->message_id)
      {
        process_data_event(this, dae_stream, &pbstream);
      }
      else if(SNS_DAE_MSGID_SNS_DAE_INTERRUPT_EVENT == event->message_id)
      {
      }
      else if(SNS_DAE_MSGID_SNS_DAE_RESP == event->message_id)
      {
        stream_usable = process_response(this, dae_stream, &pbstream);
      }
      else
      {
        state->diag_service->api->sensor_inst_printf(
          state->diag_service, this, &state->mag_info.suid, SNS_ERROR, 
          __FILENAME__, __LINE__, "Unexpected message id %u", event->message_id);
        /* TODO - report unexpected message */
      }
    }
    event = dae_stream->stream->api->get_next_input(dae_stream->stream);
  }

  if(!stream_usable)
  {
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_mgr =
      (sns_stream_service*)service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
    stream_mgr->api->remove_stream(stream_mgr, dae_stream->stream);
    dae_stream->stream = NULL;
  }
}

/*======================================================================================
  Public Functions
  ======================================================================================*/
//bool ak0991x_dae_if_available(sns_sensor_instance *this)
//{
//  /* both streams must be availble */
//  ak0991x_dae_if_info *dae_if = &((ak0991x_instance_state*)this->state->state)->dae_if;
//  return (dae_if->ag.stream != NULL && dae_if->temp.stream != NULL);
//}
//
/* ------------------------------------------------------------------------------------ */
sns_rc ak0991x_dae_if_init(
  sns_sensor_instance *const this,
  sns_stream_service  *stream_mgr,
  sns_sensor_uid      *dae_suid)
{
  sns_rc rc = SNS_RC_NOT_AVAILABLE;
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  ak0991x_dae_if_info* dae_if = &state->dae_if;
  sns_dae_set_static_config config_req = sns_dae_set_static_config_init_default;

  stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                 this,
                                                 *dae_suid,
                                                 &dae_if->mag.stream);

  if(NULL != dae_if->mag.stream)
  {
    uint8_t encoded_msg[sns_dae_set_static_config_size];
    sns_request req = {
      .message_id  = SNS_DAE_MSGID_SNS_DAE_SET_STATIC_CONFIG,
      .request     = encoded_msg,
      .request_len = 0
    };

    dae_if->mag.nano_hal_vtable_name = "ak0991x_hal_table";

    sns_strlcpy(config_req.func_table_name,
                dae_if->mag.nano_hal_vtable_name, 
                sizeof(config_req.func_table_name));
    config_req.interrupt              = false;
    //config_req.axis_map_count       = 3;
    //config_req.axis_map             = {...};
    config_req.has_irq_config         = true;
    config_req.irq_config             = state->irq_info.irq_config;
    config_req.has_s4s_config         = false;
    config_req.ascp_config            = state->ascp_config;
    //TODO, Keep the variable name for compile
    config_req.has_accel_info         = true;

    req.request_len = pb_encode_request(encoded_msg, 
                                        sizeof(encoded_msg), 
                                        &config_req,
                                        sns_dae_set_static_config_fields, 
                                        NULL);
    if(0 < req.request_len)
    {
      rc = dae_if->mag.stream->api->send_request(dae_if->mag.stream, &req);
    }
  }

  if(SNS_RC_SUCCESS != rc)
  {
    ak0991x_dae_if_deinit(state, stream_mgr);
  }
  return rc;
}

/* ------------------------------------------------------------------------------------ */
void ak0991x_dae_if_deinit(ak0991x_instance_state *state, sns_stream_service *stream_mgr)
{
  if(NULL != state->dae_if.mag.stream)
  {
    stream_mgr->api->remove_stream(stream_mgr, state->dae_if.mag.stream);
    state->dae_if.mag.stream = NULL;
  }
}

/* ------------------------------------------------------------------------------------ */
bool ak0991x_dae_if_stop_streaming(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  ak0991x_dae_if_info    *dae_if = &state->dae_if;

  if(NULL != dae_if->mag.stream && 
     (dae_if->mag.state == STREAMING || dae_if->mag.state == STREAM_STARTING))
  {
    cmd_sent |= stop_streaming(&dae_if->mag);
  }

  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
bool ak0991x_dae_if_start_streaming(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  ak0991x_dae_if_info    *dae_if = &state->dae_if;

  if(NULL != dae_if->mag.stream &&
     (0 < state->mag_info.curr_odr))
  {
    cmd_sent |= send_mag_config(&dae_if->mag, &state->mag_info);
  }

  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
bool ak0991x_dae_if_flush_hw(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  ak0991x_dae_if_info *dae_if = &((ak0991x_instance_state*)this->state->state)->dae_if;

  if(NULL != dae_if->mag.stream && !dae_if->mag.flushing_hw)
  {
    cmd_sent |= flush_hw(&dae_if->mag);
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
bool ak0991x_dae_if_flush_samples(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  ak0991x_dae_if_info *dae_if = &((ak0991x_instance_state*)this->state->state)->dae_if;

  if(NULL != dae_if->mag.stream && !dae_if->mag.flushing_data)
  {
    if(!dae_if->mag.flushing_data)
    {
      flush_samples(&dae_if->mag);
    }
    cmd_sent |= dae_if->mag.flushing_data;
  }

  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
void ak0991x_dae_if_process_events(sns_sensor_instance *this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  process_events(this, &state->dae_if.mag);

  if(NULL == state->dae_if.mag.stream)
  {
    /* both streams are needed; if one was removed, remove the other one too */
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_mgr =
      (sns_stream_service*)service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
    ak0991x_dae_if_deinit(state, stream_mgr);
  }
}

