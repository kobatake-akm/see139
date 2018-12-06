/**
 * @file sns_ak0991x_dae_if_island.c
 *
 * AK0991X - DAE sensor interface
 *
 * Copyright (c) 2017-2018 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2017-2018 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include "sns_ak0991x_dae_if.h"

#include "sns_mem_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_sensor_util.h"
#include "sns_types.h"

#include "sns_ak0991x_hal.h"
#include "sns_ak0991x_sensor.h"
#include "sns_ak0991x_sensor_instance.h"
#include "sns_ak0991x_s4s.h"

#include "sns_dae.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"

//#define AK0991X_DAE_FORCE_NOT_AVAILABLE

#ifndef SNS_MAX
#define SNS_MAX(a,b) ({ __auto_type _a = (a);    \
                        __auto_type _b = (b);    \
                        _a > _b ? _a : _b; })
#endif /* SNS_MAX */

/*======================================================================================
  Helper Functions
  ======================================================================================*/
#if defined(AK0991X_ENABLE_DAE)
static void build_static_config_request(
  ak0991x_state             *sensor_state,
  sns_dae_set_static_config *config_req,
  int64_t hardware_id)
{
  if(hardware_id == 0)
  {
    sns_strlcpy(config_req->func_table_name, "ak0991x_hal_table",
               sizeof(config_req->func_table_name));
  }
  else
  {
    sns_strlcpy(config_req->func_table_name, "ak0991x_hal_table2",
               sizeof(config_req->func_table_name));
  }


#ifdef AK0991X_DAE_FORCE_POLLING
  config_req->interrupt              = 0;
  config_req->has_irq_config         = false;
  config_req->has_ibi_config         = false;
#else
  config_req->interrupt              = sensor_state->is_dri;
  config_req->has_irq_config         = sensor_state->is_dri == 1;
  config_req->has_ibi_config         = sensor_state->is_dri == 2;
#endif /* AK0991X_DAE_FORCE_POLLING */
  if(sensor_state->is_dri == 1)
  {
    config_req->irq_config           = sensor_state->irq_config;
  }
  else if(sensor_state->is_dri == 2)
  {
    config_req->ibi_config           =
    (sns_ibi_req){ .dynamic_slave_addr = sensor_state->com_port_info.com_config.slave_control,
                   .bus_instance = sensor_state->com_port_info.com_config.bus_instance,
                   .ibi_data_bytes = 0, };
  }
  switch (sensor_state->device_select)
  {
  case AK09911:
  case AK09912:
  case AK09913:
  case AK09916C:
  case AK09916D:
  case AK09918:
    config_req->has_s4s_config       = false;
    break;
  case AK09915C:
  case AK09915D:
  case AK09917:
#ifdef AK0991X_ENABLE_REGISTRY_ACCESS
    config_req->has_s4s_config       = sensor_state->registry_cfg.sync_stream;
#else
    config_req->has_s4s_config       = false;
#endif // AK0991X_ENABLE_REGISTRY_ACCESS
    break;
  default:
    config_req->has_s4s_config       = false;
    break;
  }
  config_req->s4s_config.st_reg_addr = AKM_AK0991X_REG_SYT;
  config_req->s4s_config.st_reg_data = 0;
  config_req->s4s_config.dt_reg_addr = AKM_AK0991X_REG_DT;

  sns_com_port_config const *com_config  = &sensor_state->com_port_info.com_config;
  sns_async_com_port_config *ascp_config = &config_req->ascp_config;

  ascp_config->bus_type             = (sns_async_com_port_bus_type)com_config->bus_type;
  ascp_config->slave_control        = com_config->slave_control;
  ascp_config->reg_addr_type        = SNS_ASYNC_COM_PORT_REG_ADDR_TYPE_8_BIT;
  ascp_config->min_bus_speed_kHz    = com_config->min_bus_speed_KHz;
  ascp_config->max_bus_speed_kHz    = com_config->max_bus_speed_KHz;
  ascp_config->bus_instance         = com_config->bus_instance;

  config_req->has_accel_info         = false;
}

static sns_rc send_static_config_request(
  sns_data_stream           *stream,
  sns_dae_set_static_config *config_req)
{
  sns_rc rc = SNS_RC_FAILED;
  uint8_t encoded_msg[sns_dae_set_static_config_size];
  sns_request req = {
    .message_id  = SNS_DAE_MSGID_SNS_DAE_SET_STATIC_CONFIG,
    .request     = encoded_msg,
    .request_len = 0
  };

  req.request_len = pb_encode_request(encoded_msg, sizeof(encoded_msg), config_req,
                                      sns_dae_set_static_config_fields, NULL);
  if(0 < req.request_len)
  {
    rc = stream->api->send_request(stream, &req);
  }
  return rc;
}

static bool stream_usable(ak0991x_dae_stream *dae_stream)
{
  return (NULL != dae_stream->stream && dae_stream->stream_usable);
}

/* ------------------------------------------------------------------------------------ */
static bool send_mag_config(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  ak0991x_instance_state *state      = (ak0991x_instance_state*)this->state->state;
  ak0991x_dae_stream     *dae_stream = &state->dae_if.mag;
  ak0991x_mag_info       *mag_info   = &state->mag_info;
  sns_dae_set_streaming_config config_req = sns_dae_set_streaming_config_init_default;
  uint8_t encoded_msg[sns_dae_set_streaming_config_size];
  sns_request req = {
    .message_id   = SNS_DAE_MSGID_SNS_DAE_SET_STREAMING_CONFIG,
    .request      = encoded_msg
  };
  uint16_t wm;
  sns_time meas_usec;
  ak0991x_get_meas_time(mag_info->device_select, mag_info->sdr, &meas_usec);

  AK0991X_INST_PRINT(HIGH, this, "send_mag_config:: stream=0x%x, #clk_err_meas_count=%u/%u, in_clk_err_proc=%u, use_dri=%u",
                     dae_stream->stream, 
                     state->mag_info.clock_error_meas_count,
                     AK0991X_IRQ_NUM_FOR_OSC_ERROR_CALC,
                     state->in_clock_error_procedure,
                     state->mag_info.use_dri);

  if(!state->mag_info.use_dri || 
      state->mag_info.clock_error_meas_count >= AK0991X_IRQ_NUM_FOR_OSC_ERROR_CALC)
  {
    config_req.dae_watermark = SNS_MAX(mag_info->req_wmk, 1);
    wm = !mag_info->use_fifo ? 1 : ((mag_info->device_select == AK09917) ? 
                                    mag_info->cur_wmk : mag_info->max_fifo_size);
  }
  else
  {
    config_req.dae_watermark = wm = 1;  // go into clock error procedure
  }

  config_req.has_data_age_limit_ticks = true;

  if (mag_info->max_batch )
  {
     config_req.data_age_limit_ticks = UINT64_MAX;
  }
  else
  {
    config_req.data_age_limit_ticks =
      sns_convert_ns_to_ticks((uint64_t)mag_info->flush_period*1000ULL);
    config_req.data_age_limit_ticks += config_req.data_age_limit_ticks / 10;
  }

  config_req.has_polling_config  = !mag_info->use_dri;
  if( config_req.has_polling_config )
  {
    if (mag_info->use_sync_stream)
    {
      config_req.polling_config.polling_interval_ticks =
        sns_convert_ns_to_ticks( 1000000000ULL * (uint64_t)(mag_info->cur_wmk + 1)
                                / (uint64_t) mag_info->curr_odr );
    }
    else
    {
      config_req.polling_config.polling_interval_ticks = 
        ak0991x_get_sample_interval(state->mag_info.desired_odr);
    }
    //TODO: it looks like the polling offset will not be adjusted for S4S. 
    //So it won't be synced with any other sensors
    config_req.polling_config.polling_offset =
      (state->system_time + state->averaged_interval) / state->averaged_interval *
      state->averaged_interval;
  }
  config_req.has_accel_info      = false;

  config_req.has_expected_get_data_bytes = true;
  config_req.expected_get_data_bytes = 
      (wm+1) * AK0991X_NUM_DATA_HXL_TO_ST2 + dae_stream->status_bytes_per_fifo;

    AK0991X_INST_PRINT(HIGH, this, "dae_watermark=%u, data_age_limit_ticks=0x%x%x, wm=%u,expected_get_data_bytes=%d ",
                       config_req.dae_watermark, (uint32_t)(config_req.data_age_limit_ticks>>32),(uint32_t)(config_req.data_age_limit_ticks &0xFFFFFFFF),
                        wm, config_req.expected_get_data_bytes);

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

  if(mag_info->use_sync_stream)
  {
    sns_dae_s4s_dynamic_config s4s_config_req = sns_dae_s4s_dynamic_config_init_default;
    uint8_t s4s_encoded_msg[sns_dae_s4s_dynamic_config_size];
    sns_request s4s_req = {
      .message_id  = SNS_DAE_MSGID_SNS_DAE_S4S_DYNAMIC_CONFIG,
      .request     = s4s_encoded_msg,
      .request_len = 0
    };

    //This is T_Ph start moment at the first time
    s4s_config_req.ideal_sync_offset = sns_get_system_time();
    s4s_config_req.sync_interval = sns_convert_ns_to_ticks(AK0991X_S4S_INTERVAL_MS * 1000 * 1000);
    s4s_config_req.resolution_ratio = AK0991X_S4S_RR;
    //st_delay is defined in the sns_dae.proto file
    //This is a hardware and sampling-rate dependent value which needs to be filled in by the vendor

    s4s_config_req.st_delay = sns_convert_ns_to_ticks( meas_usec / 2 * 1000ULL );

    if((s4s_req.request_len =
        pb_encode_request(s4s_encoded_msg,
                          sizeof(s4s_encoded_msg),
                          &s4s_config_req,
                          sns_dae_s4s_dynamic_config_fields,
                          NULL)) > 0)
    {
      // The mag driver on Q6 never receives this message. It only sends this message. It can be sent at any time
      if(SNS_RC_SUCCESS == dae_stream->stream->api->send_request(dae_stream->stream, &s4s_req))
      {
      }
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
  uint8_t             *buf,
  size_t              buf_len)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  uint16_t fifo_len = buf_len - state->dae_if.mag.status_bytes_per_fifo;
  uint32_t sampling_intvl;
  uint8_t wm = 1;
  ak0991x_mag_odr odr = (ak0991x_mag_odr)(buf[1] & 0x1F);
  bool is_orphan = false;

  //////////////////////////////
  // data buffer formed in sns_ak0991x_dae.c for non-fifo mode
  // buf[0] : CNTL1
  // buf[1] : CNTL2
  // buf[2] : ST1
  // buf[3] : HXL (HXH AK09917)
  // buf[4] : HXH (HXL AK09917)
  // buf[5] : HYL (HYH AK09917)
  // buf[6] : HYH (HYL AK09917)
  // buf[7] : HZL (HZH AK09917)
  // buf[8] : HZH (HZL AK09917)
  // buf[9] : TMPS
  // buf[10]: ST2
  //////////////////////////////

  if(state->in_clock_error_procedure)
  {
    state->num_samples = (buf[2] & AK0991X_DRDY_BIT) ? 1 : 0;
  }
  else
  {
    is_orphan = (state->dae_evnet_time < state->last_sw_reset_time);

    if(state->mag_info.use_fifo)
    {
      // num_samples when FIFO enabled.
      if(state->mag_info.device_select == AK09917)
      {
        state->num_samples = buf[2] >> 2;
        wm = (buf[0] & 0x1F) + 1;
      }
      else if(state->mag_info.device_select == AK09915C || state->mag_info.device_select == AK09915D)
      {
        state->num_samples = state->mag_info.cur_wmk + 1;
        wm = state->mag_info.cur_wmk;
      }
    }
    else
    {
      // num_samples when FIFO disabled.
      if(state->mag_info.use_dri)  // dri mode
      {
        state->num_samples = (buf[2] & AK0991X_DRDY_BIT) ? 1 : 0;
      }
      else  // polling mode: *** Doesn't care FIFO+Polling ***
      {
        if( is_orphan )  // orphan
        {
          state->num_samples = (buf[2] & AK0991X_DRDY_BIT) ? 1 : 0;
          AK0991X_INST_PRINT(MED, this, "orphan num_samples=%d", state->num_samples);
        }
        else
        {
          if(state->fifo_flush_in_progress) // flush request
          {
            // set check DRDY status when flush request in polling mode
            state->num_samples = (buf[2] & AK0991X_DRDY_BIT) ? 1 : 0;
            AK0991X_INST_PRINT(MED, this, "num_samples=%d in flush and polling", state->num_samples);
          }
          else // timer event
          {
            // set num samples=1 when regular polling mode.
            state->num_samples = 1;
          }
        }
      }
    }
  }

  if((state->num_samples*AK0991X_NUM_DATA_HXL_TO_ST2) > fifo_len)
  {
//    SNS_INST_PRINTF(
//      ERROR, this, "fifo_samples:: #samples %u disagrees with fifo len %u",
//      state->num_samples, fifo_len);
//    state->num_samples = fifo_len/AK0991X_NUM_DATA_HXL_TO_ST2;
  }

  if(state->mag_info.use_dri)
  {
    state->mag_info.data_count += state->num_samples;
  }

  if(state->num_samples >= 1)
  {
    if(!state->in_clock_error_procedure)
    {
      state->data_over_run = (buf[2] & AK0991X_DOR_BIT) ? true : false;
      state->data_is_ready = (buf[2] & AK0991X_DRDY_BIT) ? true : false;

      if( !is_orphan ) // regular sequence
      {
        if(state->last_sent_cfg.odr != odr || state->last_sent_cfg.fifo_wmk != wm)
        {
          sns_time meas_usec;
          state->new_cfg.odr      = odr;
          state->new_cfg.fifo_wmk = wm;

          AK0991X_INST_PRINT(MED, this, "ak0991x_send_config_event in DAE dae_evnet_time=%u", (uint32_t)state->dae_evnet_time);
          ak0991x_send_config_event(this);

          ak0991x_get_meas_time(state->mag_info.device_select, state->mag_info.sdr, &meas_usec);
          state->this_is_first_data = true;
          state->half_measurement_time =
            ((sns_convert_ns_to_ticks(meas_usec * 1000) *
              state->internal_clock_error) >> AK0991X_CALC_BIT_RESOLUTION)>>1;
          state->nominal_intvl = ak0991x_get_sample_interval(state->mag_info.curr_odr);
          state->averaged_interval =
            (state->nominal_intvl * state->internal_clock_error) >> AK0991X_CALC_BIT_RESOLUTION;
          state->pre_timestamp = state->odr_change_timestamp +
            (state->half_measurement_time<<1) - state->averaged_interval;
          state->previous_irq_time = state->pre_timestamp;
          state->mag_info.data_count = 0;
        }

        if(state->mag_info.use_dri)
        {
          ak0991x_validate_timestamp_for_dri(this);
        }
        else
        {
          ak0991x_validate_timestamp_for_polling(this);
        }
        sampling_intvl = state->averaged_interval;
      }
      else  // orphan
      {
        sampling_intvl = (ak0991x_get_sample_interval(odr) *
                          state->internal_clock_error) >> AK0991X_CALC_BIT_RESOLUTION;

        if(state->irq_info.detect_irq_event)
        {
          state->interrupt_timestamp = state->irq_event_time;
          state->first_data_ts_of_batch = state->interrupt_timestamp - sampling_intvl * (state->num_samples - 1);
        }
        else
        {
          //          state->first_data_ts_of_batch =  event_timestamp - sampling_intvl * (state->num_samples - 1);
          state->first_data_ts_of_batch =  state->pre_timestamp + sampling_intvl;
        }

        AK0991X_INST_PRINT(MED, this, "fifo_samples:: orphan batch odr=(%d->%d) wm=(%d->%d) num_samples=%d last_sw_reset=%u event_time=%u",
            odr,
            state->mag_info.curr_odr,
            wm,
            state->mag_info.cur_wmk,
            state->num_samples,
            (uint32_t)state->last_sw_reset_time,
            (uint32_t)state->dae_evnet_time);
      }

      ak0991x_process_mag_data_buffer(this,
                                      state->first_data_ts_of_batch,
                                      sampling_intvl,
                                      buf + state->dae_if.mag.status_bytes_per_fifo,
                                      fifo_len);
#ifdef AK0991X_ENABLE_TS_DEBUG
//      AK0991X_INST_PRINT(
//        MED, this, "fifo_samples:: odr=0x%X intvl=%u #samples=%u ts=%X-%X",
//        odr, (uint32_t)sampling_intvl, state->num_samples,
//        (uint32_t)state->first_data_ts_of_batch, (uint32_t)state->irq_event_time);
#endif

    }
    else  // in clock error procedure
    {
      AK0991X_INST_PRINT(LOW, this, "ak0991x_clock_error_calc_procedure call in DAE");
      ak0991x_clock_error_calc_procedure(this, &buf[2]);
      if (!state->in_clock_error_procedure && ak0991x_dae_if_stop_streaming(this))
      {
        AK0991X_INST_PRINT(LOW, this, "DONE clock error procedure");
        state->config_step = AK0991X_CONFIG_UPDATING_HW;
      }
      else
      {
        AK0991X_INST_PRINT(HIGH, this, "Discarding %u stale samples.", state->num_samples);
      }
    }

    if(state->mag_info.use_dri) // for DRI mode
    {
      state->heart_beat_attempt_count = 0;
      if (NULL != state->timer_data_stream)
      {
        sns_sensor_event *event =
          state->timer_data_stream->api->peek_input(state->timer_data_stream);

        while (NULL != event)
        {
          event = state->timer_data_stream->api->get_next_input(state->timer_data_stream);
        }
      }

      // keep re-register HB timer when DAE enabled.
      if(state->in_clock_error_procedure || state->mag_info.req_wmk != UINT32_MAX || is_orphan)
      {
        ak0991x_register_heart_beat_timer(this);
      }
    }
    else  // for Polling mode
    {
      // No heart_beat_timer_event when orphan.
      // Just update heart_beat_timestamp in order to ignore performing unnecessary heart beat detection
      if( is_orphan )
      {
        state->heart_beat_timestamp = state->system_time;
        state->heart_beat_sample_count = 0;
        state->heart_beat_attempt_count = 0;
//        AK0991X_INST_PRINT(HIGH, this, "Reset HB timestamp %u", (uint32_t)state->heart_beat_timestamp);
      }
      else
      {
        // check heart beat fire time
//        ak0991x_heart_beat_timer_event(this);
      }
    }
  }
}

static void estimate_event_type(
    sns_sensor_instance *this,
    uint8_t* buf)
{
  //////////////////////////////
  // data buffer formed in sns_ak0991x_dae.c for non-fifo mode
  // buf[0] : CNTL1
  // buf[1] : CNTL2
  // buf[2] : ST1
  // buf[3] : HXL (HXH AK09917)
  // buf[4] : HXH (HXL AK09917)
  // buf[5] : HYL (HYH AK09917)
  // buf[6] : HYH (HYL AK09917)
  // buf[7] : HZL (HZH AK09917)
  // buf[8] : HZH (HZL AK09917)
  // buf[9] : TMPS
  // buf[10]: ST2
  //////////////////////////////

  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  uint8_t wm = (buf[1] & AK0991X_FIFO_BIT) ? (buf[0] & 0x1F) + 1 : 1;
  sns_time polling_timestamp;

  if( buf[2] & AK0991X_DRDY_BIT )
  {
    state->irq_info.detect_irq_event = true;  // regular DRI/Polling detected
  }
  else if( state->mag_info.use_dri ) // DRI
  {
    if(state->dae_if.mag.flushing_data)
    {
      state->fifo_flush_in_progress = true;  // Flush request
    }
  }
  else  // Polling
  {
    polling_timestamp = state->pre_timestamp + state->averaged_interval * wm;

    // there is a chance to get wrong result
    if( state->dae_evnet_time > polling_timestamp - state->averaged_interval/200 &&
        state->dae_evnet_time < polling_timestamp + state->averaged_interval/200 )
    {
      state->irq_info.detect_irq_event = true;  // polling timer event
    }
    else
    {
      if(state->dae_if.mag.flushing_data)
      {
        state->fifo_flush_in_progress = true;  // Flush request
      }
    }
  }
  AK0991X_INST_PRINT(LOW, this, "estimated result: detect_irq=%d flush=%d flushing=%d",
      (uint8_t)state->irq_info.detect_irq_event,
      (uint8_t)state->fifo_flush_in_progress,
      (uint8_t)state->dae_if.mag.flushing_data);
}

/* ------------------------------------------------------------------------------------ */
static void process_data_event(
  sns_sensor_instance *this,
  pb_istream_t        *pbstream)
{
  pb_buffer_arg decode_arg;
  sns_dae_data_event data_event = sns_dae_data_event_init_default;
  data_event.sensor_data.funcs.decode = &pb_decode_string_cb;
  data_event.sensor_data.arg = &decode_arg;

  if(pb_decode(pbstream, sns_dae_data_event_fields, &data_event))
  {
    ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
    state->system_time = sns_get_system_time();
    state->dae_evnet_time = data_event.timestamp;
    state->irq_info.detect_irq_event = false;
    state->fifo_flush_in_progress = false;

#ifdef AK0991X_ENABLE_TIMESTAMP_TYPE
    if(data_event.has_timestamp_type)
    {
      if(state->mag_info.use_dri) // DRI
      {
        if( data_event.timestamp_type == sns_dae_timestamp_type_SNS_DAE_TIMESTAMP_TYPE_HW_IRQ )
        {
          state->irq_info.detect_irq_event = true;  // DRI interrupt
        }
      }
      else  // Polling
      {
        if( data_event.timestamp_type == sns_dae_timestamp_type_SNS_DAE_TIMESTAMP_TYPE_TIMER )
        {
          state->irq_info.detect_irq_event = true;  // timer event in polling mode
        }
      }

      if( data_event.timestamp_type == sns_dae_timestamp_type_SNS_DAE_TIMESTAMP_TYPE_SYSTEM_TIME &&
          state->dae_if.mag.flushing_data)
      {
        state->fifo_flush_in_progress = true;  // Flush request
      }
    }
    else
    {
      estimate_event_type(this, (uint8_t*)decode_arg.buf);
    }
#else
    estimate_event_type(this, (uint8_t*)decode_arg.buf);
#endif

#ifdef AK0991X_ENABLE_TIMESTAMP_TYPE
    AK0991X_INST_PRINT(HIGH, this, "process_data_event:%u. flush=%d data_count=%d ts_type=%d flushing=%d",
        (uint32_t)data_event.timestamp,
        (uint8_t)state->fifo_flush_in_progress,
        state->mag_info.data_count,
        data_event.timestamp_type,
        state->dae_if.mag.flushing_data);
#else
    AK0991X_INST_PRINT(HIGH, this, "process_data_event:%u. int=%d flush=%d data_count=%d",
        (uint32_t)data_event.timestamp,
        (uint8_t)state->irq_info.detect_irq_event,
        (uint8_t)state->fifo_flush_in_progress,
        state->mag_info.data_count);
#endif

    if(state->irq_info.detect_irq_event)
    {
      state->irq_event_time = state->dae_evnet_time;
    }

    process_fifo_samples(
      this, (uint8_t*)decode_arg.buf, decode_arg.buf_len);

    if(state->irq_info.detect_irq_event)
    {
      if(state->mag_info.use_dri)
      {
        // When DAE enable, validate timestamp can return false.
        if(state->mag_info.data_count == 0)
        {
          state->previous_irq_time = state->interrupt_timestamp;
        }
      }
      else
      {
        state->previous_irq_time = state->interrupt_timestamp;
      }
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
      AK0991X_INST_PRINT(LOW, this, "STATIC_CONFIG - err=%u state=%u", 
                         resp.err, dae_stream->state);
      if(SNS_STD_ERROR_NO_ERROR == resp.err)
      {
        dae_stream->state = IDLE;
        if(ak0991x_dae_if_start_streaming(this))
        {
          state->config_step = AK0991X_CONFIG_UPDATING_HW;
        }
      }
      else
      {
        /* DAE sensor does not have support for this driver */
        dae_stream->stream_usable = false;
        dae_stream->state = PRE_INIT;
        if(state->mag_info.desired_odr > AK0991X_MAG_ODR_OFF)
        {
          ak0991x_continue_client_config(this);
        }
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_S4S_DYNAMIC_CONFIG:
      //The DAE environment will send the ST/DT messages to the HW automatically
      //without involvement of either the normal mag sensor driver,
      //or the mag sensor driver in DAE environment.
      //The ST/DT messages will continue to be sent automatically until the PAUSE_S4S
      //message is sent by the mag driver to the DAE.
      //
      //Send ST/DT command
      //ak0991x_s4s_handle_timer_event(this);
      break;
    case SNS_DAE_MSGID_SNS_DAE_SET_STREAMING_CONFIG:
      AK0991X_INST_PRINT(LOW, this,"DAE_SET_STREAMING_CONFIG");
      if(dae_stream->stream != NULL && dae_stream->state == STREAM_STARTING)
      {
        if(SNS_STD_ERROR_NO_ERROR == resp.err)
        {
          dae_stream->state = STREAMING;
          ak0991x_reconfig_hw(this, true);
          state->config_step = AK0991X_CONFIG_IDLE;
        }
        else
        {
          dae_stream->state = IDLE;
        }
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_FLUSH_HW:
      AK0991X_INST_PRINT(LOW, this, "DAE_FLUSH_HW");
      dae_stream->flushing_hw = false;
      if(state->config_step != AK0991X_CONFIG_IDLE)
      {
        ak0991x_dae_if_start_streaming(this);
        ak0991x_dae_if_flush_samples(this);
        state->config_step = AK0991X_CONFIG_UPDATING_HW;
      }
      else if(state->heart_beat_attempt_count >= 3)
      {
        // Perform a reset operation in an attempt to revive the sensor
        ak0991x_reconfig_hw(this, true);
      }
      else
      {
        ak0991x_dae_if_flush_samples(this); // Flush client request
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_FLUSH_DATA_EVENTS:
      AK0991X_INST_PRINT(LOW, this, "DAE_FLUSH_DATA");
      ak0991x_send_fifo_flush_done(this);
      dae_stream->flushing_data = false;
      break;
    case SNS_DAE_MSGID_SNS_DAE_PAUSE_SAMPLING:
      AK0991X_INST_PRINT(LOW, this,
                         "DAE_PAUSE_SAMPLING stream_state=%u if_state=%u config_step=%u",
                         dae_stream->state, state->dae_if.mag.state, state->config_step);
      if(dae_stream->state == STREAM_STOPPING)
      {
        dae_stream->state = (SNS_STD_ERROR_NO_ERROR != resp.err) ? STREAMING : IDLE;
      }

      if(dae_stream->state != STREAM_STOPPING)
      {
        if(state->config_step == AK0991X_CONFIG_STOPPING_STREAM && 
           ak0991x_dae_if_flush_hw(this))
        {
          state->config_step = AK0991X_CONFIG_FLUSHING_HW;
        }
        else if(state->config_step == AK0991X_CONFIG_UPDATING_HW)
        {
          ak0991x_dae_if_start_streaming(this);
        }
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_PAUSE_S4S_SCHED:
      if(state->mag_info.use_sync_stream)
      {
      state->mag_info.s4s_dt_abort = true;
      ak0991x_s4s_handle_timer_event(this);
      state->mag_info.s4s_dt_abort = false;
      }
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

  while(NULL != dae_stream->stream &&
        NULL != (event = dae_stream->stream->api->peek_input(dae_stream->stream)))
  {
    if (dae_stream->stream_usable)
    {
      pb_istream_t pbstream =
        pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);

      if (SNS_DAE_MSGID_SNS_DAE_DATA_EVENT == event->message_id)
      {
        AK0991X_INST_PRINT(LOW, this,"DAE_DATA_EVENT");
        process_data_event(this, &pbstream);
      }
      else if(SNS_DAE_MSGID_SNS_DAE_INTERRUPT_EVENT == event->message_id)
      {
        AK0991X_INST_PRINT(LOW, this,"DAE_INTERRUPT_EVENT");
      }
      else if(SNS_DAE_MSGID_SNS_DAE_RESP == event->message_id)
      {
        AK0991X_INST_PRINT(LOW, this,"SNS_DAE_RESP");
        process_response(this, dae_stream, &pbstream);
      }
      else if(SNS_STD_MSGID_SNS_STD_ERROR_EVENT == event->message_id)
      {
        ak0991x_instance_state *state = (ak0991x_instance_state *)this->state->state;
        dae_stream->stream_usable = false;
        AK0991X_INST_PRINT(LOW, this,"SNS_STD_ERROR_EVENT");
        if(dae_stream->state == INIT_PENDING && 
           state->mag_info.desired_odr > AK0991X_MAG_ODR_OFF)
        {
          ak0991x_continue_client_config(this);
        }
      }
      else
      {
        AK0991X_INST_PRINT(ERROR, this,"Unexpected message id %u", event->message_id);
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

void ak0991x_dae_if_check_support(sns_sensor *this)
{
  ak0991x_state *state = (ak0991x_state*)this->state->state;

  if(NULL == state->dae_if.mag.stream)
  {
    sns_service_manager *svc_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_svc = (sns_stream_service*)
      svc_mgr->get_service(svc_mgr, SNS_STREAM_SERVICE);

    sns_sensor_uid dae_suid;
    if(sns_suid_lookup_get(&state->suid_lookup_data, "data_acquisition_engine", &dae_suid))
    {
      stream_svc->api->create_sensor_stream(stream_svc, this, dae_suid, &state->dae_if.mag.stream);
    }
  }

  if(NULL != state->dae_if.mag.stream)
  {
    sns_dae_set_static_config config_req = sns_dae_set_static_config_init_default;
    if(state->dae_if.mag.state == PRE_INIT)
    {
      state->dae_if.mag.state = INIT_PENDING;
      build_static_config_request(state, &config_req, state->hardware_id);
    }

    if(SNS_RC_SUCCESS != send_static_config_request(state->dae_if.mag.stream, &config_req))
    {
      AK0991X_PRINT(LOW, this, "check_support:: static config fail");
      state->dae_if.mag.state = UNAVAILABLE;
      sns_sensor_util_remove_sensor_stream(this, &state->dae_if.mag.stream);
    }
  }
}

bool ak0991x_dae_if_available(sns_sensor_instance *this)
{
  ak0991x_dae_if_info *dae_if = &((ak0991x_instance_state*)this->state->state)->dae_if;
  return stream_usable(&dae_if->mag);
}

void ak0991x_dae_if_process_sensor_events(sns_sensor *this)
{
  ak0991x_state *state = (ak0991x_state*)this->state->state;
  sns_data_stream *stream = state->dae_if.mag.stream;
  sns_sensor_event *event;

  if(NULL == stream || 0 == stream->api->get_input_cnt(stream))
  {
    return;
  }

  while(NULL != (event = stream->api->peek_input(stream)))
  {
    pb_istream_t pbstream = pb_istream_from_buffer((pb_byte_t*)event->event, event->event_len);

    if(SNS_DAE_MSGID_SNS_DAE_RESP == event->message_id)
    {
      sns_dae_resp resp = sns_dae_resp_init_default;
      if(pb_decode(&pbstream, sns_dae_resp_fields, &resp))
      {
        if(SNS_DAE_MSGID_SNS_DAE_SET_STATIC_CONFIG == resp.msg_id)
        {
          if(state->dae_if.mag.state == INIT_PENDING)
          {
            state->dae_if.mag.state =
              (SNS_STD_ERROR_NO_ERROR != resp.err) ? UNAVAILABLE : IDLE;
            if(UNAVAILABLE == state->dae_if.mag.state)
            {
              SNS_PRINTF(HIGH, this, "process_sensor_events:: dae unavailable");
            }
          }
        }
      }
    }
    else if(SNS_STD_MSGID_SNS_STD_ERROR_EVENT == event->message_id)
    {
      state->dae_if.mag.state = UNAVAILABLE;
    }

    event = stream->api->get_next_input(stream);
  }

  if(UNAVAILABLE == state->dae_if.mag.state || IDLE == state->dae_if.mag.state)
  {
    sns_sensor_util_remove_sensor_stream(this, &state->dae_if.mag.stream);
  }
}

/* ------------------------------------------------------------------------------------ */
bool ak0991x_dae_if_is_initializing(sns_sensor_instance *this)
{
  ak0991x_dae_if_info *dae_if = &((ak0991x_instance_state*)this->state->state)->dae_if;
  return (stream_usable(&dae_if->mag) && dae_if->mag.state == INIT_PENDING);
}

/* ------------------------------------------------------------------------------------ */
bool ak0991x_dae_if_is_streaming(sns_sensor_instance *this)
{
  ak0991x_dae_if_info *dae_if = &((ak0991x_instance_state*)this->state->state)->dae_if;
  return (stream_usable(&dae_if->mag) && dae_if->mag.state == STREAMING);
}

/* ------------------------------------------------------------------------------------ */
sns_rc ak0991x_dae_if_init(
  sns_sensor_instance     *const this,
  sns_stream_service      *stream_mgr,
  sns_sensor_uid          *dae_suid,
  ak0991x_state           *sensor_state)
{
  sns_rc rc = SNS_RC_NOT_AVAILABLE;
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  ak0991x_dae_if_info* dae_if = &state->dae_if;

  AK0991X_INST_PRINT(LOW, this, "dae_if_init set inst mag.state %d <= %d", (int)dae_if->mag.state, (int)sensor_state->dae_if.mag.state);
  dae_if->mag.state = sensor_state->dae_if.mag.state;

#ifdef AK0991X_DAE_FORCE_NOT_AVAILABLE
  sns_sensor_util_remove_sensor_instance_stream(this, &dae_if->mag.stream);
  dae_if->mag.stream_usable = false;
  return rc;
#endif

  if(IDLE == dae_if->mag.state)
  {
    stream_mgr->api->create_sensor_instance_stream(stream_mgr,
                                                   this,
                                                   *dae_suid,
                                                   &dae_if->mag.stream);

    if(NULL != dae_if->mag.stream)
    {
      sns_dae_set_static_config config_req = sns_dae_set_static_config_init_default;
      build_static_config_request(sensor_state, &config_req, sensor_state->hardware_id);
      rc = send_static_config_request(dae_if->mag.stream, &config_req);
    }
  }

  if(SNS_RC_SUCCESS != rc)
  {
    ak0991x_dae_if_deinit(this);
  }
  else
  {
    AK0991X_INST_PRINT(LOW, this, "dae ready");
    dae_if->mag.stream_usable   = true;
    dae_if->mag.status_bytes_per_fifo = 3; /* CNTL1, CNTL2, ST1 */
  }

  return rc;
}

/* ------------------------------------------------------------------------------------ */
void ak0991x_dae_if_deinit(sns_sensor_instance *this)
{
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  sns_sensor_util_remove_sensor_instance_stream(this, &state->dae_if.mag.stream);
  state->dae_if.mag.state = PRE_INIT;
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
    AK0991X_INST_PRINT(LOW, this,"stopping mag stream=0x%x", &dae_if->mag.stream);
    cmd_sent |= stop_streaming(&dae_if->mag);
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
bool ak0991x_dae_if_start_streaming(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;

  if(stream_usable(&state->dae_if.mag) && state->dae_if.mag.state > PRE_INIT &&
     (0 < state->mag_info.desired_odr))
  {
    cmd_sent |= send_mag_config(this);
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
bool ak0991x_dae_if_flush_hw(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  ak0991x_dae_if_info *dae_if = &((ak0991x_instance_state*)this->state->state)->dae_if;
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  state->irq_info.detect_irq_event = false;

  if(stream_usable(&dae_if->mag) && dae_if->mag.state >= IDLE)
  {
    if(!dae_if->mag.flushing_hw)
    {
      flush_hw(&dae_if->mag);
    }
    cmd_sent |= dae_if->mag.flushing_hw;
  }
  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
bool ak0991x_dae_if_flush_samples(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  ak0991x_dae_if_info *dae_if = &((ak0991x_instance_state*)this->state->state)->dae_if;

  if(stream_usable(&dae_if->mag) && dae_if->mag.state >= IDLE)
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

  if(!state->dae_if.mag.stream_usable)
  {
    ak0991x_dae_if_deinit(this);
  }
}

#else //defined(AK0991X_ENABLE_DAE)

bool ak0991x_dae_if_available(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
void ak0991x_dae_if_check_support(sns_sensor *this)
{
  UNUSED_VAR(this);
}
void ak0991x_dae_if_process_sensor_events(sns_sensor *this)
{
  UNUSED_VAR(this);
}
bool ak0991x_dae_if_is_initializing(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
bool ak0991x_dae_if_is_streaming(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
sns_rc ak0991x_dae_if_init(
  sns_sensor_instance     *const this,
  sns_stream_service      *stream_mgr,
  sns_sensor_uid          *dae_suid,
  ak0991x_state           *sensor_state)
{
  UNUSED_VAR(this);
  UNUSED_VAR(stream_mgr);
  UNUSED_VAR(dae_suid);
  UNUSED_VAR(sensor_state);
  return SNS_RC_NOT_AVAILABLE;
}
void ak0991x_dae_if_deinit(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
}
bool ak0991x_dae_if_stop_streaming(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
bool ak0991x_dae_if_start_streaming(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
bool ak0991x_dae_if_flush_hw(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
bool ak0991x_dae_if_flush_samples(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
void ak0991x_dae_if_process_events(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
}
#endif //defined(AK0991X_ENABLE_DAE)

