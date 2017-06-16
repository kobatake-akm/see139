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
#include "sns_printf.h"

#ifndef SNS_MAX
#define SNS_MAX(a,b) ({ __auto_type _a = (a);    \
                        __auto_type _b = (b);    \
                        _a > _b ? _a : _b; })
#endif /* SNS_MAX */

//#define AK0991X_DAE_FORCE_NOT_AVAILABLE

/*======================================================================================
  Helper Functions
  ======================================================================================*/
static bool stream_usable(ak0991x_dae_stream *dae_stream)
{
  return (NULL != dae_stream->stream && dae_stream->stream_usable);
}

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

  config_req.dae_watermark       = SNS_MAX(mag_info->req_wmk, 1);

  config_req.has_data_age_limit_ticks = true;
  config_req.data_age_limit_ticks =
    sns_convert_ns_to_ticks((uint64_t)mag_info->flush_period*1000ULL);

  config_req.has_polling_config  = !mag_info->use_dri;
  if( config_req.has_polling_config )
  {
    sns_time meas_usec;
    ak0991x_get_meas_time(mag_info->device_select, &meas_usec);
    config_req.polling_config.polling_interval_ticks =
      sns_convert_ns_to_ticks( 1000000000ULL * (uint64_t)mag_info->cur_wmk
                               / (uint64_t) mag_info->curr_odr );
    config_req.polling_config.polling_offset =
      sns_get_system_time() + sns_convert_ns_to_ticks( meas_usec * 1000ULL );
  }
  config_req.has_accel_info      = false;

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
      dae_stream->state = STREAM_STARTING;
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
static void process_response(
  sns_sensor_instance *this, 
  ak0991x_dae_stream  *dae_stream,
  pb_istream_t        *pbstream)
{
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
        dae_stream->stream_usable = false;
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_S4S_DYNAMIC_CONFIG:
      break;
    case SNS_DAE_MSGID_SNS_DAE_SET_STREAMING_CONFIG:
         SNS_INST_PRINTF(LOW, this,"DAE_SET_STREAMING_CONFIG");
      if(dae_stream->stream != NULL && dae_stream->state == STREAM_STARTING)
      {
        if(SNS_STD_ERROR_NO_ERROR == resp.err)
        {
          dae_stream->state = STREAMING;
          //ak0991x_start_mag_streaming(this);
        }
        else
        {
          dae_stream->state = IDLE;
        }
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_FLUSH_HW:
      SNS_INST_PRINTF(LOW, this,"DAE_FLUSH_HW");
      if(state->config_step != AK0991X_CONFIG_IDLE)
      {
        ak0991x_reconfig_hw(this);
        //ak0991x_start_mag_streaming(this);
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
}

/* ------------------------------------------------------------------------------------ */
static void process_events(sns_sensor_instance *this, ak0991x_dae_stream *dae_stream)
{
  sns_sensor_event *event;

  //SNS_INST_PRINTF(LOW, this,"line=%d process_events",__LINE__);
 
  while(NULL != dae_stream->stream && 
        NULL != (event = dae_stream->stream->api->peek_input(dae_stream->stream)))
  {
    if (dae_stream->stream_usable)
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
        SNS_INST_PRINTF(LOW, this,"line=%d SNS_DAE_RESP",__LINE__);
        process_response(this, dae_stream, &pbstream);
      }
      else if(SNS_STD_MSGID_SNS_STD_ERROR_EVENT == event->message_id)
      {
        dae_stream->stream_usable = false;
      }
      else
      {
        SNS_INST_PRINTF(LOW, this,"Unexpected message id %u", event->message_id);
      }
    }
    event = dae_stream->stream->api->get_next_input(dae_stream->stream);
  }

  if(!dae_stream->stream_usable)
  {
#if 0
  /* TODO - restore this once framework can properly deal with events published between
     the start and end of stream remove process */

    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_mgr =
      (sns_stream_service*)service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
    stream_mgr->api->remove_stream(stream_mgr, dae_stream->stream);
    dae_stream->stream = NULL;
#endif
  }
}

/*======================================================================================
  Public Functions
  ======================================================================================*/
/* ------------------------------------------------------------------------------------ */
bool ak0991x_dae_if_available(sns_sensor_instance *this)
{
  ak0991x_dae_if_info *dae_if = &((ak0991x_instance_state*)this->state->state)->dae_if;
  //SNS_INST_PRINTF(ERROR, this,"mag.stream=%d mag.stream_usable=%d",
  //   (int)(dae_if->mag.stream != NULL),  (int)dae_if->mag.stream_usable);

  return (dae_if->mag.stream != NULL && dae_if->mag.stream_usable);
}

/* ------------------------------------------------------------------------------------ */
sns_rc ak0991x_dae_if_init(
  sns_sensor_instance  *const this,
  sns_stream_service   *stream_mgr,
  sns_sensor_uid       *dae_suid,
  sns_sensor_uid const *parent_suid)
{
  SNS_INST_PRINTF(LOW, this,"line=%d dae_if_init",__LINE__);
  sns_rc rc = SNS_RC_NOT_AVAILABLE;
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  ak0991x_dae_if_info* dae_if = &state->dae_if;
  sns_dae_set_static_config config_req = sns_dae_set_static_config_init_default;

#ifdef AK0991X_DAE_FORCE_NOT_AVAILABLE
  if( dae_if->mag.stream != NULL )
  {
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_mgr =
      (sns_stream_service*)service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
    stream_mgr->api->remove_stream(stream_mgr, dae_if->mag.stream);
    dae_if->mag.stream = NULL;
  }
  dae_if->mag.stream_usable = false;
  return rc;
#endif

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
    config_req.interrupt              = true;
    config_req.axis_map_count         = 3;
    config_req.has_irq_config         = true;
#ifdef AK0991X_DAE_FORCE_POLLING
    config_req.interrupt              = false;
    config_req.has_irq_config         = false;
#endif /* AK0991X_DAE_FORCE_POLLING */
    config_req.irq_config             = state->irq_info.irq_config;
    config_req.has_s4s_config         = false;
    config_req.ascp_config            = state->ascp_config;
    config_req.has_accel_info         = false;

    uint8_t i = 0;

    for(i = 0; i < TRIAXIS_NUM; i++)
    {
      config_req.axis_map[i] = (state->axis_map[i].invert ? -1.0 : 1.0) *
        (state->axis_map[i].ipaxis + 1);
    }

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
  else
  {
    dae_if->mag.suid            = parent_suid;
    dae_if->mag.stream_usable   = true;

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

  if(stream_usable(&state->dae_if.mag) &&
     (dae_if->mag.state == STREAMING || dae_if->mag.state == STREAM_STARTING))
  {
    SNS_INST_PRINTF(LOW, this,"%s- Mag stream=%x", __FUNCTION__,&dae_if->mag.stream);
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
     (0 < state->mag_info.desired_odr))
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
  SNS_INST_PRINTF(LOW, this,"line=%d func=%s",__LINE__,__FUNCTION__);
 
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;

 
  process_events(this, &state->dae_if.mag);

  if(NULL == state->dae_if.mag.stream)
  {
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_mgr =
      (sns_stream_service*)service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
    ak0991x_dae_if_deinit(state, stream_mgr);
  }
}

