/**
 * @file sns_ak0991x_dae_if_island.c
 *
 * AK0991X - DAE sensor interface
 *
 * Copyright (c) 2017-2019 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2017-2020 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include "sns_ak0991x_dae_if.h"
#include "sns_types.h"

#if defined(AK0991X_ENABLE_DAE)
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_time.h"
#include "sns_sensor_event.h"

#include "sns_ak0991x_hal.h"
#include "sns_ak0991x_sensor_instance.h"
#include "sns_ak0991x_s4s.h"

#include "sns_dae.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"

#ifndef SNS_MAX
#define SNS_MAX(a,b) ({ __auto_type _a = (a);    \
                        __auto_type _b = (b);    \
                        _a > _b ? _a : _b; })
#endif /* SNS_MAX */

#ifdef AK0991X_PATCH_FOR_DAE_S4S_DT_EVENT_FIELDS_ON_704
const pb_field_t sns_dae_s4s_dt_event_fields[3] = {
    PB_FIELD(  1, FIXED64 , REQUIRED, STATIC  , FIRST, sns_dae_s4s_dt_event, timestamp, timestamp, 0),
    PB_FIELD(  2, INT32   , REQUIRED, STATIC  , OTHER, sns_dae_s4s_dt_event, dt_value, timestamp, 0),
    PB_LAST_FIELD
};
#endif

/*======================================================================================
  Helper Functions
  ======================================================================================*/
static bool stream_usable(ak0991x_dae_stream *dae_stream)
{
  return (NULL != dae_stream->stream && dae_stream->stream_usable);
}

/* ------------------------------------------------------------------------------------ */
static void ak0991x_send_mag_s4s_config(sns_sensor_instance *this, bool send_dt_event)
{
#ifdef AK0991X_ENABLE_S4S
  ak0991x_instance_state *state      = (ak0991x_instance_state*)this->state->state;
  ak0991x_dae_stream     *dae_stream = &state->dae_if.mag;

  sns_dae_s4s_dynamic_config s4s_config_req = sns_dae_s4s_dynamic_config_init_default;
  uint8_t s4s_encoded_msg[sns_dae_s4s_dynamic_config_size];
  sns_request s4s_req = {
    .message_id  = SNS_DAE_MSGID_SNS_DAE_S4S_DYNAMIC_CONFIG,
    .request     = s4s_encoded_msg,
    .request_len = 0
  };

  //This is T_Ph start moment at the first time
  s4s_config_req.ideal_sync_offset = state->polling_timer_start_time;
  s4s_config_req.sync_interval = sns_convert_ns_to_ticks(AK0991X_S4S_INTERVAL_MS * 1000 * 1000);
  s4s_config_req.resolution_ratio = AK0991X_S4S_RR;
  //st_delay is defined in the sns_dae.proto file
  //This is a hardware and sampling-rate dependent value which needs to be filled in by the vendor

  s4s_config_req.st_delay = s4s_config_req.sync_interval * 0.001f;

  // Ask DAE to send DT events until S4S is synchronized
  s4s_config_req.has_send_dt_event = true;
  s4s_config_req.send_dt_event = send_dt_event;


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
#else
  UNUSED_VAR(this);
  UNUSED_VAR(send_dt_event);
#endif // AK0991X_ENABLE_S4S
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
  uint16_t batch_num = 0;
  uint16_t wm;
  sns_request req = {
    .message_id   = SNS_DAE_MSGID_SNS_DAE_SET_STREAMING_CONFIG,
    .request      = encoded_msg
  };

  AK0991X_INST_PRINT(HIGH, this, "send_mag_config:: stream=0x%x, #clk_err_meas_count=%u/%u, in_clk_err_proc=%u, use_dri=%u",
                     dae_stream->stream,
                     state->mag_info.clock_error_meas_count,
                     AK0991X_IRQ_NUM_FOR_OSC_ERROR_CALC,
                     state->in_clock_error_procedure,
                     state->mag_info.int_mode);

  if( state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING ||
      state->mag_info.clock_error_meas_count >= AK0991X_IRQ_NUM_FOR_OSC_ERROR_CALC)
  {
    wm = !mag_info->use_fifo ? 1 : (((mag_info->device_select == AK09917) || (mag_info->device_select == AK09919)) ?
                                    mag_info->cur_cfg.fifo_wmk : mag_info->max_fifo_size);

    if(state->mag_info.flush_only || state->mag_info.max_batch)
    {
      config_req.dae_watermark = UINT32_MAX;
    }
    else
    {
      batch_num = SNS_MAX(mag_info->cur_cfg.dae_wmk, 1) / wm;
      config_req.dae_watermark = batch_num * wm;
    }

    AK0991X_INST_PRINT(LOW, this, "cur_wmk=%d wm=%d batch_num=%u dae_watermark=%u",
        mag_info->cur_cfg.fifo_wmk,
        wm,
        batch_num,
        config_req.dae_watermark);
  }
  else
  {
    config_req.dae_watermark = wm = 1;  // go into clock error procedure
  }

  config_req.has_data_age_limit_ticks = true;
  if(state->mag_info.max_batch)
  {
    config_req.data_age_limit_ticks = UINT64_MAX;
  }
  else
  {
    config_req.data_age_limit_ticks = mag_info->flush_period * 1.1f;
  }

  config_req.has_polling_config  = (state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING);
  if( config_req.has_polling_config )
  {
    config_req.polling_config.polling_interval_ticks =
      wm * ak0991x_get_sample_interval(state->mag_info.cur_cfg.odr);

    state->system_time = sns_get_system_time();

    //TODO: it looks like the polling offset will not be adjusted for S4S.
    //So it won't be synced with any other sensors
    config_req.polling_config.polling_offset = state->polling_timer_start_time;
  }
  config_req.has_accel_info = false;
  config_req.has_expected_get_data_bytes = true;
  config_req.expected_get_data_bytes =
      wm * AK0991X_NUM_DATA_HXL_TO_ST2 + dae_stream->status_bytes_per_fifo;

  if(mag_info->use_sync_stream)
  {
    ak0991x_send_mag_s4s_config( this, true );
    config_req.polling_config.polling_offset +=
      sns_convert_ns_to_ticks(AK0991X_S4S_INTERVAL_MS * 1000 * 1000 * 0.001f);
  }

  AK0991X_INST_PRINT(HIGH, this, "send_mag_config:: sys= %u, pre_orphan= %u, polling_offset= %u interval= %u",
      (uint32_t)state->system_time,
      (uint32_t)state->pre_timestamp_for_orphan,
      (uint32_t)config_req.polling_config.polling_offset,
      (uint32_t)config_req.polling_config.polling_interval_ticks);

  SNS_INST_PRINTF(HIGH, this, "send_mag_config:: polling_offset=%X%08X sys=%X%08X inteval_tick=%u",
      (uint32_t)(config_req.polling_config.polling_offset>>32),
      (uint32_t)(config_req.polling_config.polling_offset & 0xFFFFFFFF),
      (uint32_t)(state->system_time>>32),
      (uint32_t)(state->system_time & 0xFFFFFFFF),
      (uint32_t)config_req.polling_config.polling_interval_ticks);

  SNS_INST_PRINTF(HIGH, this, "send_mag_config:: dae_watermark=%u, data_age_limit_ticks=0x%X%08X, wm=%u,expected_get_data_bytes=%d ",
                       config_req.dae_watermark,
                       (uint32_t)(config_req.data_age_limit_ticks>>32),
                       (uint32_t)(config_req.data_age_limit_ticks & 0xFFFFFFFF),
                        wm,
                        config_req.expected_get_data_bytes);

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

  SNS_INST_PRINTF(HIGH, this, "send_mag_config:: stream=0x%x, dae_stream=%d request_len=%d cmd_sent=%d",
      dae_stream->stream,
      (uint8_t)dae_stream->state,
      (uint8_t)req.request_len,
      (uint8_t)cmd_sent);

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
static bool pause_s4s_streaming(ak0991x_dae_stream *dae_stream)
{
  bool cmd_sent = false;
  UNUSED_VAR(dae_stream);
#ifdef AK0991X_ENABLE_S4S
  sns_request req = {
    .message_id   = SNS_DAE_MSGID_SNS_DAE_PAUSE_S4S_SCHED,
    .request      = NULL,
    .request_len  = 0
  };

  if(SNS_RC_SUCCESS == dae_stream->stream->api->send_request(dae_stream->stream, &req))
  {
    cmd_sent = true;
//    dae_stream->state = PAUSE_S4S_SCHEDULE;
  }
#endif
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
  uint8_t dummy_buf[] = {0U,0U,0U,0U,0U,0U,0U,AK0991X_INV_FIFO_DATA}; // set INV bit
  uint8_t *mag_data_buf = buf + state->dae_if.mag.status_bytes_per_fifo;
  sns_time calculated_timestamp_from_previous;
  uint8_t ref_num_samples;
  bool dummy_state_0 = false;
  bool dummy_state = false;
  sns_time dummy_data_ts_of_batch;

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

  ak0991x_mag_odr odr = (ak0991x_mag_odr)(buf[1] & 0x1F);
  uint16_t fifo_wmk = (state->mag_info.use_fifo) ? (uint8_t)(buf[0] & 0x1F) + 1 : 1;  // read value from WM[4:0]
  state->reg_fifo_wmk = fifo_wmk;

  state->is_orphan = false;
  sampling_intvl = (ak0991x_get_sample_interval(odr) *
                    state->internal_clock_error) >> AK0991X_CALC_BIT_RESOLUTION;

  state->num_samples = (buf[2] & AK0991X_DRDY_BIT) ? 1 : 0;

  // calculate num_samples
  if(!state->in_clock_error_procedure)
  {
    state->is_orphan =
//        (state->dae_event_time < state->config_set_time); // use time
        ( odr != state->mag_info.cur_cfg.odr ) ||
        ( fifo_wmk != state->mag_info.cur_cfg.fifo_wmk );

    // calc num_samples
    {
      if(state->mag_info.use_fifo)
      {
        // num_samples update when FIFO enabled.
        if((state->mag_info.device_select == AK09917) || (state->mag_info.device_select == AK09919))
        {
          state->num_samples = buf[2] >> 2;
        }
        else if(state->mag_info.device_select == AK09915C || state->mag_info.device_select == AK09915D)
        {
          state->num_samples = fifo_wmk; // read value from WM[4:0]
        }
      }
      else
      {
        // num_samples update when Polling mode.
        if(state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING)  // polling mode: *** Doesn't care FIFO+Polling ***
        {
          if( !state->this_is_the_last_flush )
          {
            // only timer event. skip all flush requests.
            state->num_samples = (state->irq_info.detect_irq_event) ? 1 : 0;
          }
          else
          {
            if( !state->fifo_flush_in_progress ) // last batch but not last flush
            {
              // only timer event. skip all flush requests.
              state->num_samples = (state->irq_info.detect_irq_event) ? 1 : 0;
            }
            else  // last flush data
            {
              state->num_samples = (state->system_time > state->pre_timestamp_for_orphan + sampling_intvl/2) ? 1 : 0;
            }
          }

          if( state->num_samples > 0 && state->fifo_flush_in_progress )
          {
            state->flush_sample_count++;
          }
          else
          {
            state->flush_sample_count = 0;
          }
        }
      }
    }

    if( state->is_orphan )  // orphan
    {
      AK0991X_INST_PRINT(LOW, this, "orphan num_samples=%d, pre_orphan=%u, event=%u, intvl=%u sys=%u offset=%u",
          state->num_samples,
          (uint32_t)state->pre_timestamp_for_orphan,
          (uint32_t)state->dae_event_time,
          (uint32_t)sampling_intvl,
          (uint32_t)state->system_time,
          (uint32_t)state->polling_timer_start_time);

    }
  }

  if((state->num_samples*AK0991X_NUM_DATA_HXL_TO_ST2) > fifo_len)
  {
    // for polling mode, always FIFO enabled. No sample data from DAE
    if( fifo_len == 0 &&
        state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING &&
        !state->mag_info.use_sync_stream )
    {
      AK0991X_INST_PRINT(HIGH, this, "num_samples=0 But forced to set 1, fifo_len=8.");
      state->num_samples = 1;
      fifo_len = AK0991X_NUM_DATA_HXL_TO_ST2;
      mag_data_buf = dummy_buf; // switch pointer to the dummy buf data.
    }
    else
    {
      SNS_INST_PRINTF(
        ERROR, this, "fifo_samples:: #samples %u disagrees with fifo len %u",
        state->num_samples, fifo_len);
        state->num_samples = fifo_len/AK0991X_NUM_DATA_HXL_TO_ST2;
    }
  }

  if(((state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING) && (state->mag_info.use_fifo)) && (0 == state->num_samples) && !state->mag_info.use_sync_stream)
  {
    state->num_samples = 1;
    fifo_len = AK0991X_NUM_DATA_HXL_TO_ST2;
    mag_data_buf =  dummy_buf;
    dummy_state_0 = true ;
    AK0991X_INST_PRINT(MED, this, "num_samples=0 But forced to set 1, add dummy data");
  }

  if( !state->is_orphan )
  {
    state->mag_info.data_count_for_dri += state->num_samples;
  }

  if(state->num_samples >= 1)
  {
    if(!state->in_clock_error_procedure)
    {
      state->data_over_run = (buf[2] & AK0991X_DOR_BIT) ? true : false;
      state->data_is_ready = (buf[2] & AK0991X_DRDY_BIT) ? true : false;

      if( !state->is_orphan ) // regular sequence
      {
        if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING)
        {
          ak0991x_validate_timestamp_for_dri(this);
          sampling_intvl = state->averaged_interval; // update sampling_intvl
        }
        else
        {
          if(state->mag_info.use_fifo) // for Polling + FIFO
          {
            ref_num_samples = (state->num_samples <= state->mag_info.cur_cfg.fifo_wmk) ? state->num_samples : state->mag_info.cur_cfg.fifo_wmk;
            calculated_timestamp_from_previous = state->pre_timestamp + sampling_intvl * ref_num_samples;
            if(state->system_time >= calculated_timestamp_from_previous)
            {
              if((state->dae_event_time < calculated_timestamp_from_previous + 0.8*sampling_intvl) && (state->dae_event_time > (calculated_timestamp_from_previous - sampling_intvl)))
              {
                state->first_data_ts_of_batch = state->dae_event_time - sampling_intvl * (ref_num_samples - 1);
              }
              else
              {
                state->first_data_ts_of_batch = state->pre_timestamp + sampling_intvl;
              }

              if(state->fifo_flush_in_progress || state->this_is_the_last_flush || !state->irq_info.detect_irq_event)
              {
                state->first_data_ts_of_batch = state->pre_timestamp + sampling_intvl;
              }
            }
            else
            {
              if(ref_num_samples - 1 >= ((calculated_timestamp_from_previous - state->system_time)/sampling_intvl))
              {
                if(((calculated_timestamp_from_previous - state->system_time) % sampling_intvl) == 0)
                {
                  state->num_samples = ref_num_samples - ((calculated_timestamp_from_previous - state->system_time)/sampling_intvl);
                }
                else
                {
                  state->num_samples = ref_num_samples - (((calculated_timestamp_from_previous- state->system_time)/sampling_intvl) + 1);
                }
                fifo_len = state->num_samples * AK0991X_NUM_DATA_HXL_TO_ST2;
                if(0 != state->num_samples)
                {
                  if((state->dae_event_time - (state->pre_timestamp + sampling_intvl * state->num_samples) < 0.8*sampling_intvl) && (state->dae_event_time > state->pre_timestamp+ sampling_intvl * (state->num_samples-1)))
                  {
                    state->first_data_ts_of_batch = state->dae_event_time - sampling_intvl * (state->num_samples - 1);
                  }
                  else
                  {
                    state->first_data_ts_of_batch = state->pre_timestamp + sampling_intvl;
                  }

                  if(state->fifo_flush_in_progress || state->this_is_the_last_flush || !state->irq_info.detect_irq_event)
                  {
                    state->first_data_ts_of_batch = state->pre_timestamp + sampling_intvl;
                  }
                }
                else
                {
                  state->num_samples = 1;
                  fifo_len = state->num_samples * AK0991X_NUM_DATA_HXL_TO_ST2;
                  state->first_data_ts_of_batch = state->system_time;
                }
              }
              else
              {
                state->num_samples = 0;
                fifo_len = 0;
                if(state->fifo_flush_in_progress || state->this_is_the_last_flush || !state->irq_info.detect_irq_event)
                {
                  ak0991x_send_fifo_flush_done(this);
                }
              }
            }

            if((1 == (state->mag_info.cur_cfg.fifo_wmk - state->num_samples)) && !dummy_state_0 && !state->data_is_ready && (!(state->fifo_flush_in_progress || state->this_is_the_last_flush || !state->irq_info.detect_irq_event)))
            {
              if(state->dae_event_time <= calculated_timestamp_from_previous)
              {
                state->first_data_ts_of_batch = state->dae_event_time - sampling_intvl * ((state->num_samples - ((calculated_timestamp_from_previous - state->dae_event_time)/sampling_intvl))- 1);
              }
              else
              {
                dummy_state = true;
                if(((state->dae_event_time - calculated_timestamp_from_previous) % sampling_intvl) != 0)
                {
                  state->first_data_ts_of_batch = state->dae_event_time - sampling_intvl * (state->mag_info.cur_cfg.fifo_wmk + (state->dae_event_time - calculated_timestamp_from_previous)/sampling_intvl - 1);
                  dummy_data_ts_of_batch = state->dae_event_time - sampling_intvl * ((state->dae_event_time - calculated_timestamp_from_previous)/sampling_intvl);
                }
                else
                {
                  state->first_data_ts_of_batch = state->dae_event_time - sampling_intvl * (state->mag_info.cur_cfg.fifo_wmk + ((state->dae_event_time - calculated_timestamp_from_previous)/sampling_intvl -1)- 1);
                  dummy_data_ts_of_batch = state->dae_event_time - sampling_intvl * ((state->dae_event_time - calculated_timestamp_from_previous)/sampling_intvl -1);
                }
              }
            }
            state->flush_sample_count  = state->fifo_flush_in_progress ? state->num_samples : 0;
            state->averaged_interval = sampling_intvl;
          }
          else
          {
            state->interrupt_timestamp = state->dae_event_time;

#ifdef AK0991X_OPEN_SSC_711_PATCH_FOR_JITTER
            // use ideal interval for more than 50Hz ODR because of the timer jitter
            if( !state->this_is_first_data && (sampling_intvl < 384000 ) &&
                !(state->this_is_the_last_flush && state->fifo_flush_in_progress) )
            {
              state->interrupt_timestamp = state->pre_timestamp_for_orphan + sampling_intvl * state->num_samples;
            }
#endif

            if( state->this_is_the_last_flush && state->fifo_flush_in_progress )
            {
              AK0991X_INST_PRINT(LOW, this, "last flush total= %d pre= %u ts= %u sys= %u p_off=%u",
                  state->total_samples,
                  (uint32_t)state->pre_timestamp_for_orphan,
                  (uint32_t)state->interrupt_timestamp,
                  (uint32_t)state->system_time,
                  (uint32_t)state->polling_timer_start_time);
            }

            state->first_data_ts_of_batch = state->interrupt_timestamp;
            state->averaged_interval = sampling_intvl;
          }
        }

#ifdef AK0991X_ENABLE_TS_DEBUG
        AK0991X_INST_PRINT(
          MED, this, "fifo_samples:: odr=0x%X intvl=%u #samples=%u ts=%X-%X first_data=%d",
          odr, (uint32_t)sampling_intvl, state->num_samples,
          (uint32_t)state->first_data_ts_of_batch,
          (uint32_t)state->irq_event_time,
          state->this_is_first_data);
#endif
      }
      else  // orphan
      {
        // when the pre_timestamp_for_orphan is too old or newer than the dae_event_time, use dae_event_time instead.
        if(state->irq_info.detect_irq_event && // dri or timer event
           (state->dae_event_time > state->pre_timestamp_for_orphan + sampling_intvl * state->num_samples + sampling_intvl/5 ||
            state->dae_event_time < state->pre_timestamp_for_orphan + sampling_intvl * state->num_samples)  &&
           ((state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING) || !state->mag_info.use_fifo))
        {
          state->first_data_ts_of_batch = state->dae_event_time - sampling_intvl * (state->num_samples - 1);
        }
        else if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING)  // dri
        {
          if(state->irq_info.detect_irq_event ||  // interrupt event
             state->dae_event_time < state->pre_timestamp_for_orphan + sampling_intvl * state->num_samples)
          {
            state->first_data_ts_of_batch = state->dae_event_time - sampling_intvl * (state->num_samples - 1);
          }
          else
          {
            state->first_data_ts_of_batch = state->pre_timestamp_for_orphan + sampling_intvl;
          }
        }
        else  // polling
        {
          if(state->mag_info.use_fifo)
          {
            ref_num_samples = (state->num_samples <= fifo_wmk) ? state->num_samples : fifo_wmk;
            calculated_timestamp_from_previous = state->pre_timestamp_for_orphan + sampling_intvl * ref_num_samples;
            if(state->system_time >= calculated_timestamp_from_previous)
            {
              if((state->dae_event_time < calculated_timestamp_from_previous + 0.8*sampling_intvl) && (state->dae_event_time > (calculated_timestamp_from_previous - sampling_intvl)))
              {
                state->first_data_ts_of_batch = state->dae_event_time - sampling_intvl * (ref_num_samples - 1);
              }
              else
              {
                state->first_data_ts_of_batch = state->pre_timestamp_for_orphan + sampling_intvl;
              }

              if(state->fifo_flush_in_progress || state->this_is_the_last_flush || !state->irq_info.detect_irq_event)
              {
                state->first_data_ts_of_batch = state->dae_event_time - sampling_intvl * (ref_num_samples - 1);
                if(state->this_is_the_last_flush && (!state->data_is_ready || state->fifo_flush_in_progress))
                {
                  state->first_data_ts_of_batch = state->pre_timestamp_for_orphan + sampling_intvl;
                }
              }
            }
            else
            {
              if(ref_num_samples - 1 >= ((calculated_timestamp_from_previous - state->system_time)/sampling_intvl))
              {
                if(((calculated_timestamp_from_previous - state->system_time) % sampling_intvl) == 0)
                {
                  state->num_samples = ref_num_samples - ((calculated_timestamp_from_previous - state->system_time)/sampling_intvl);
                }
                else
                {
                  state->num_samples = ref_num_samples - (((calculated_timestamp_from_previous- state->system_time)/sampling_intvl) + 1);
                }
                fifo_len = state->num_samples * AK0991X_NUM_DATA_HXL_TO_ST2;
                if(0 != state->num_samples)
                {
                  if((state->dae_event_time - (state->pre_timestamp_for_orphan + sampling_intvl * state->num_samples) < 0.8*sampling_intvl) && (state->dae_event_time > state->pre_timestamp_for_orphan+ sampling_intvl * (state->num_samples-1)))
                  {
                    state->first_data_ts_of_batch = state->dae_event_time - sampling_intvl * (state->num_samples - 1);
                  }
                  else
                  {
                    state->first_data_ts_of_batch = state->pre_timestamp_for_orphan + sampling_intvl;
                  }

                  if(state->fifo_flush_in_progress || state->this_is_the_last_flush || !state->irq_info.detect_irq_event)
                  {
                    if(state->this_is_first_data && state->fifo_flush_in_progress && state->this_is_the_last_flush)
                    {
                      state->first_data_ts_of_batch = state->pre_timestamp_for_orphan + sampling_intvl;
                      fifo_len = state->num_samples * AK0991X_NUM_DATA_HXL_TO_ST2;
                    }
                    else
                    {
                      state->first_data_ts_of_batch = state->dae_event_time - sampling_intvl * (ref_num_samples - 1);
                      state->num_samples = ref_num_samples;
                      fifo_len = state->num_samples * AK0991X_NUM_DATA_HXL_TO_ST2;
                    }
                  }
                }
                else
                {
                  state->num_samples = 1;
                  fifo_len = state->num_samples * AK0991X_NUM_DATA_HXL_TO_ST2;
                  state->first_data_ts_of_batch = state->system_time;
                }
              }
              else
              {
                state->num_samples = 0;
                fifo_len = 0;
                if(state->fifo_flush_in_progress || state->this_is_the_last_flush || !state->irq_info.detect_irq_event)
                {
                  state->fifo_flush_in_progress = false;
                  state->flush_requested_in_dae = false;
                }
              }
            }
            state->flush_sample_count  = state->fifo_flush_in_progress ? state->num_samples : 0;
          }
          else
          {
            state->first_data_ts_of_batch = state->dae_event_time;
          }

#ifdef AK0991X_OPEN_SSC_711_PATCH_FOR_JITTER
          // use ideal interval for more than 50Hz ODR because of the timer jitter
          if( !state->this_is_first_data && (sampling_intvl < 384000 ) &&
              !(state->this_is_the_last_flush && state->fifo_flush_in_progress) )
          {
            state->first_data_ts_of_batch = state->pre_timestamp_for_orphan + sampling_intvl;
          }
#endif


        }
        AK0991X_INST_PRINT(MED, this, "fifo_samples:: orphan batch odr=(%d->%d) num_samples=%d event_time=%u",
            odr,
            state->mag_info.cur_cfg.odr,
            state->num_samples,
            (uint32_t)state->dae_event_time);
      }

      ak0991x_process_mag_data_buffer(this,
                                      state->first_data_ts_of_batch,
                                      sampling_intvl,
                                      mag_data_buf,
                                      fifo_len);
      if(dummy_state && (!(state->fifo_flush_in_progress || state->this_is_the_last_flush)))
      {
        ak0991x_process_mag_data_buffer(this,
                                        dummy_data_ts_of_batch,
                                        sampling_intvl,
                                        dummy_buf,
                                        AK0991X_NUM_DATA_HXL_TO_ST2);
        AK0991X_INST_PRINT(MED, this, "Add one dummy data");
      }
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

    // Set new timeout for heart beat
    ak0991x_register_heart_beat_timer(this);
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

  if( state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING ) // DRI
  {
    if( buf[2] & AK0991X_DRDY_BIT )
    {
      state->irq_info.detect_irq_event = true;  // regular DRI
    }
    else
    {
      state->fifo_flush_in_progress = true;  // Flush request
    }
  }
  else  // Polling
  {
    polling_timestamp = state->pre_timestamp + state->averaged_interval * wm;

    if( state->this_is_first_data )
    {
      state->irq_info.detect_irq_event = true;  // polling timer event
    }
    // there is a chance to get wrong result
    if( state->dae_event_time > polling_timestamp - state->averaged_interval/200 &&
        state->dae_event_time < polling_timestamp + state->averaged_interval/200 )
    {
      state->irq_info.detect_irq_event = true;  // polling timer event
    }
    else
    {
      state->fifo_flush_in_progress = true;  // Flush request
    }
  }
  AK0991X_INST_PRINT(LOW, this, "estimated result: detect_irq=%d flush=%d",
      (uint8_t)state->irq_info.detect_irq_event,
      (uint8_t)state->fifo_flush_in_progress);
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
    state->dae_event_time = data_event.timestamp;
    state->irq_info.detect_irq_event = false;
    state->fifo_flush_in_progress = false;

    if(data_event.has_timestamp_type)
    {
      if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING) // DRI
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

      if( data_event.timestamp_type == sns_dae_timestamp_type_SNS_DAE_TIMESTAMP_TYPE_SYSTEM_TIME )
      {
        state->fifo_flush_in_progress = true;  // Flush request
      }
    }
    else
    {
      estimate_event_type(this, (uint8_t*)decode_arg.buf);
    }

    AK0991X_INST_PRINT(HIGH, this, "process_data_event:%u. flush=(%d,%d) count=(%d,%d) ts_type=%d",
        (uint32_t)data_event.timestamp,
        (uint8_t)state->fifo_flush_in_progress,
        state->dae_if.mag.flushing_data,
        state->mag_info.data_count_for_dri,
        state->total_samples,
        data_event.timestamp_type);

    if(state->irq_info.detect_irq_event)
    {
      state->irq_event_time = state->dae_event_time;
    }

    process_fifo_samples(
      this, (uint8_t*)decode_arg.buf, decode_arg.buf_len);

    if(state->irq_info.detect_irq_event)
    {
      if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING)
      {
        // When DAE enable, validate timestamp can return false.
        if(state->mag_info.data_count_for_dri == 0)
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
        else
        {
          AK0991X_INST_PRINT(LOW, this, "ak0991x_dae_if_start_streaming return false - err=%u state=%u odr=%d dae_mag_state=%d",
                             resp.err, dae_stream->state, state->mag_info.cur_cfg.odr, state->dae_if.mag.state);
        }
      }
      else
      {
        /* DAE sensor does not have support for this driver */
        dae_stream->stream_usable = false;
        dae_stream->state = PRE_INIT;
        if(state->mag_info.cur_cfg.odr != AK0991X_MAG_ODR_OFF)
        {
          ak0991x_inst_exit_island(this);
          ak0991x_continue_client_config(this, true);
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
      AK0991X_INST_PRINT(LOW, this,"DAE_SET_STREAMING_CONFIG - err=%u state=%u",
                         resp.err, dae_stream->state);
      if(dae_stream->stream != NULL && dae_stream->state == STREAM_STARTING)
      {
        if(SNS_STD_ERROR_NO_ERROR == resp.err)
        {
          dae_stream->state = STREAMING;
          // No reset call.
          ak0991x_reconfig_hw(this, false);
          state->config_step = AK0991X_CONFIG_IDLE;
          if(state->do_flush_after_change_config)
          {
            state->do_flush_after_change_config = false;
            AK0991X_INST_PRINT(LOW, this,"Flush after change config.");
            if(!state->flush_requested_in_dae)
            {
              state->flush_requested_in_dae = true;
              if(state->mag_info.use_fifo && state->mag_info.cur_cfg.fifo_wmk > 1)
              {
                ak0991x_dae_if_flush_hw(this);
              }
              else if(state->mag_info.cur_cfg.dae_wmk > 1)
              {
                ak0991x_dae_if_flush_samples(this);
              }
              else
              {
                ak0991x_send_fifo_flush_done(this);
              }
            }
          }
        }
        else
        {
          if(SNS_STD_ERROR_INVALID_STATE == resp.err &&
              state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING)
          {
            AK0991X_INST_PRINT(LOW, this,"stop and restart dae streaming");
            ak0991x_dae_if_stop_streaming(this);
            state->config_step = AK0991X_CONFIG_UPDATING_HW;
          }
          else
          {
             dae_stream->state = IDLE;
          }
        }
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_FLUSH_HW:
      AK0991X_INST_PRINT(LOW, this, "DAE_FLUSH_HW - err=%u state=%u config_step=%d",
                         resp.err, dae_stream->state, state->config_step);
      dae_stream->flushing_hw = false;

      if(state->config_step != AK0991X_CONFIG_IDLE)
      {
        ak0991x_dae_if_flush_samples(this);
        ak0991x_dae_if_start_streaming(this);
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
      AK0991X_INST_PRINT(LOW, this, "DAE_FLUSH_DATA - err=%u state=%u config_step=%d num_samples=%d",
                         resp.err, dae_stream->state, state->config_step, state->num_samples);
      if(state->flush_requested_in_dae)
      {
        ak0991x_send_fifo_flush_done(this);
      }
      dae_stream->flushing_data = false;
      state->this_is_the_last_flush = false;
      state->wait_for_last_flush = false;

      if( !state->in_self_test &&
          state->mag_info.cur_cfg.num > state->mag_info.last_sent_cfg.num &&
          state->mag_info.cur_cfg.odr != AK0991X_MAG_ODR_OFF)
      {
        // check if handled same config.
        if(state->mag_info.cur_cfg.odr == state->mag_info.last_sent_cfg.odr &&
           state->mag_info.cur_cfg.fifo_wmk == state->mag_info.last_sent_cfg.fifo_wmk &&
           state->mag_info.cur_cfg.dae_wmk == state->mag_info.last_sent_cfg.dae_wmk)
        {
          AK0991X_INST_PRINT(HIGH, this, "Same config. send previous config.");
          ak0991x_send_config_event(this, false);  // send new config event
          ak0991x_send_cal_event(this, false);    // send previous cal event
        }
        else
        {
          AK0991X_INST_PRINT(HIGH, this, "Send new config #%d in DAE: odr=0x%02X fifo_wmk=%d, dae_wmk=%d",
              state->mag_info.cur_cfg.num,
              (uint32_t)state->mag_info.cur_cfg.odr,
              (uint32_t)state->mag_info.cur_cfg.fifo_wmk,
              (uint32_t)state->mag_info.cur_cfg.dae_wmk);

          ak0991x_send_config_event(this, true);  // send new config event
          ak0991x_send_cal_event(this, false);    // send previous cal event
        }
      }

      break;
    case SNS_DAE_MSGID_SNS_DAE_PAUSE_SAMPLING:
      AK0991X_INST_PRINT(LOW, this,
                         "DAE_PAUSE_SAMPLING stream_state=%u if_state=%u config_step=%u",
                         dae_stream->state,
                         state->dae_if.mag.state,
                         state->config_step);

      if(dae_stream->state == STREAM_STOPPING)
      {
        dae_stream->state = (SNS_STD_ERROR_NO_ERROR != resp.err) ? STREAMING : IDLE;
      }

      if(dae_stream->state != STREAM_STOPPING)
      {
        if(state->config_step == AK0991X_CONFIG_STOPPING_STREAM)
        {
          if(state->mag_info.use_fifo && (state->mag_info.cur_cfg.fifo_wmk > 1 || !(state->mag_info.int_mode == AK0991X_INT_OP_MODE_POLLING)))
          {
            state->this_is_the_last_flush = true;
            if(ak0991x_dae_if_flush_hw(this))
            {
              state->config_step = AK0991X_CONFIG_FLUSHING_HW;
              AK0991X_INST_PRINT(LOW, this,"Last flush before changing ODR.");
            }
          }
          else
          {
            ak0991x_dae_if_start_streaming(this);
            state->config_step = AK0991X_CONFIG_UPDATING_HW;
            ak0991x_dae_if_flush_samples(this);
          }
        }
        else if(state->config_step == AK0991X_CONFIG_UPDATING_HW)
        {
          if(state->do_flush_after_clock_error_procedure)
          {
            state->do_flush_after_clock_error_procedure = false;
            AK0991X_INST_PRINT(LOW, this,"Flush after clock error procedure.");
            if(!state->flush_requested_in_dae)
            {
              state->flush_requested_in_dae = true;
              if(state->mag_info.use_fifo && state->mag_info.cur_cfg.fifo_wmk > 1)
              {
                ak0991x_dae_if_flush_hw(this);
              }
              else if(state->mag_info.cur_cfg.dae_wmk > 1)
              {
                ak0991x_dae_if_flush_samples(this);
              }
              else
              {
                ak0991x_send_fifo_flush_done(this);
              }
            }
          }
          else
          {
            if( !state->in_self_test &&
                state->mag_info.cur_cfg.num > state->mag_info.last_sent_cfg.num &&
                state->mag_info.cur_cfg.odr != AK0991X_MAG_ODR_OFF)
            {
              AK0991X_INST_PRINT(HIGH, this, "Send new config #%d in DAE: odr=0x%02X fifo_wmk=%d, dae_wmk=%d",
                  state->mag_info.cur_cfg.num,
                  (uint32_t)state->mag_info.cur_cfg.odr,
                  (uint32_t)state->mag_info.cur_cfg.fifo_wmk,
                  (uint32_t)state->mag_info.cur_cfg.dae_wmk);

              ak0991x_send_config_event(this, true);  // send new config event
              ak0991x_send_cal_event(this, false);    // send previous cal event
            }
            ak0991x_dae_if_start_streaming(this);
          }
        }
      }
      break;
    case SNS_DAE_MSGID_SNS_DAE_PAUSE_S4S_SCHED:
      // After DAE receivce PAUSE_S4S_SCHEDULE message,
      // Stop sending the ST/DT message to the HW automatically
      AK0991X_INST_PRINT(LOW, this,
                         "DAE_PAUSE_S4S_SCHED stream_state=%u if_state=%u config_step=%u",
                         dae_stream->state,
                         state->dae_if.mag.state,
                         state->config_step);
      //if(state->mag_info.use_sync_stream)
      //{
      //  state->mag_info.s4s_dt_abort = true;
      //  ak0991x_s4s_handle_timer_event(this);
      //  state->mag_info.s4s_dt_abort = false;
      //}
      break;

    case SNS_DAE_MSGID_SNS_DAE_RESP:
    case SNS_DAE_MSGID_SNS_DAE_DATA_EVENT:
      break; /* unexpected */
    }
  }
}

/* ------------------------------------------------------------------------------------ */
static void process_dt_event(
  sns_sensor_instance *this,
  pb_istream_t        *pbstream)
{
#ifdef AK0991X_ENABLE_S4S
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;

  sns_dae_s4s_dt_event dt_event = sns_dae_s4s_dt_event_init_default;
  if(pb_decode(pbstream, sns_dae_s4s_dt_event_fields, &dt_event))
  {
    if( dt_event.dt_value < 0x80 )
    {
      if( state->mag_info.s4s_sync_state < AK0991X_S4S_SYNCED )
      {
        state->mag_info.s4s_sync_state++;
      }
      if( state->mag_info.s4s_sync_state == AK0991X_S4S_1ST_SYNCED )
      {
        // Update config_set_time, since we need to send out phy config
        // event with new timestamp.
        state->config_set_time = state->pre_timestamp_for_orphan+1;

        // Send config event with stream_is_synchronous=true:
        ak0991x_send_config_event( this, true );

        // Turn off DT messages from DAE, since they're no longer needed:
        ak0991x_send_mag_s4s_config( this, false );

        // send previous cal event
        ak0991x_send_cal_event( this, false );
      }
    }
    else
    {
      if( state->mag_info.s4s_sync_state < AK0991X_S4S_1ST_SYNCED )
      {
        state->mag_info.s4s_sync_state = AK0991X_S4S_NOT_SYNCED;
      }
    }
  }
#else
  UNUSED_VAR(this);
  UNUSED_VAR(pbstream);
#endif // AK0991X_ENABLE_S4S
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
           state->mag_info.cur_cfg.odr != AK0991X_MAG_ODR_OFF)
        {
          ak0991x_inst_exit_island(this);
          ak0991x_continue_client_config(this, true);
        }
      }
      else if(SNS_DAE_MSGID_SNS_DAE_S4S_DT_EVENT == event->message_id)
      {
        process_dt_event(this, &pbstream);
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

bool ak0991x_dae_if_available(sns_sensor_instance *this)
{
  ak0991x_dae_if_info *dae_if = &((ak0991x_instance_state*)this->state->state)->dae_if;
  return stream_usable(&dae_if->mag);
}

/* ------------------------------------------------------------------------------------ */
bool ak0991x_dae_if_is_streaming(sns_sensor_instance *this)
{
  ak0991x_dae_if_info *dae_if = &((ak0991x_instance_state*)this->state->state)->dae_if;
  return (stream_usable(&dae_if->mag) && dae_if->mag.state == STREAMING);
}

/* ------------------------------------------------------------------------------------ */
/*
bool ak0991x_dae_if_pause_s4s_schedule(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;
  ak0991x_dae_if_info    *dae_if = &state->dae_if;

  if(stream_usable(&state->dae_if.mag) &&
     state->mag_info.use_sync_stream &&
     (dae_if->mag.state == STREAMING || dae_if->mag.state == STREAM_STARTING))
  {
    AK0991X_INST_PRINT(LOW, this,"pausing s4s schedule stream=0x%x", &dae_if->mag.stream);
    cmd_sent |= pause_s4s_streaming(&dae_if->mag);
  }
  return cmd_sent;
}
*/

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
    if( state->mag_info.use_sync_stream )
    {
      cmd_sent |= pause_s4s_streaming(&dae_if->mag);
    }
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
     (AK0991X_MAG_ODR_OFF != state->mag_info.cur_cfg.odr))
  {
    if(state->mag_info.int_mode != AK0991X_INT_OP_MODE_POLLING)
    {
      cmd_sent |= send_mag_config(this);
    }
    else
    {
      AK0991X_INST_PRINT(HIGH, this, "cur_cfg.num=%d, cur_cfg.odr=0x%02X, last_sent_cfg.num=%d, last_sent_cfg.odr=0x%02X",
          state->mag_info.cur_cfg.num,
          (uint32_t)state->mag_info.cur_cfg.odr,
          state->mag_info.last_sent_cfg.num,
          (uint32_t)state->mag_info.last_sent_cfg.odr);
      if(state->reg_event_for_dae_poll_sync)
      {
        cmd_sent |= send_mag_config(this);
        state->reg_event_for_dae_poll_sync = false;
      }
      else
      {
        // Register timer to synchronize the DAE polling timing with other sensors.
        ak0991x_register_timer(this);
        cmd_sent = true;
        AK0991X_INST_PRINT(LOW, this,"register timer with is_dry_run=ture");
      }
    }
  }

  return cmd_sent;
}

/* ------------------------------------------------------------------------------------ */
bool ak0991x_dae_if_flush_hw(sns_sensor_instance *this)
{
  bool cmd_sent = false;
  ak0991x_dae_if_info *dae_if = &((ak0991x_instance_state*)this->state->state)->dae_if;
  ak0991x_instance_state *state = (ak0991x_instance_state*)this->state->state;

  if( state->mag_info.use_fifo || state->this_is_the_last_flush )
  {
    if(stream_usable(&dae_if->mag) && dae_if->mag.state >= IDLE)
    {
      if(!dae_if->mag.flushing_hw)
      {
        flush_hw(&dae_if->mag);
      }
      else
      {
        AK0991X_INST_PRINT( HIGH, this,"Already flushing_hw=%d", (uint8_t)dae_if->mag.flushing_hw );
      }
      cmd_sent |= dae_if->mag.flushing_hw;
    }

    if(!cmd_sent)
    {
      AK0991X_INST_PRINT( HIGH, this,"Failed to flush_hw state=%d", (uint8_t)dae_if->mag.state );
    }
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
    ak0991x_inst_exit_island(this);
    ak0991x_dae_if_deinit(this);
  }
}

#else //defined(AK0991X_ENABLE_DAE)

bool ak0991x_dae_if_available(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
}
bool ak0991x_dae_if_is_streaming(sns_sensor_instance *this)
{
  UNUSED_VAR(this);
  return false;
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

