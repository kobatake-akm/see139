/**
 * @file sns_ak0991x_dae_if.c
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
#include "sns_ak0991x_sensor.h"
#include "sns_types.h"

#if defined(AK0991X_ENABLE_DAE)
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_rc.h"
#include "sns_request.h"
#include "sns_time.h"
#include "sns_sensor_event.h"
#include "sns_sensor_util.h"

#include "sns_ak0991x_hal.h"
#include "sns_ak0991x_sensor_instance.h"

#include "sns_dae.pb.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"

//#define AK0991X_DAE_FORCE_NOT_AVAILABLE

/*======================================================================================
  Helper Functions
  ======================================================================================*/
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
  config_req->interrupt      = SNS_DAE_INT_OP_MODE_POLLING;
  config_req->has_irq_config = false;
  config_req->has_ibi_config = false;
#else
  config_req->interrupt =
      (sensor_state->int_mode == AK0991X_INT_OP_MODE_IBI) ?
          SNS_DAE_INT_OP_MODE_IBI :
      (sensor_state->int_mode == AK0991X_INT_OP_MODE_IRQ) ?
          SNS_DAE_INT_OP_MODE_IRQ : SNS_DAE_INT_OP_MODE_POLLING;
  config_req->has_irq_config = sensor_state->int_mode == AK0991X_INT_OP_MODE_IRQ;
  config_req->has_ibi_config = sensor_state->int_mode == AK0991X_INT_OP_MODE_IBI;
#endif /* AK0991X_DAE_FORCE_POLLING */
  if(sensor_state->int_mode == AK0991X_INT_OP_MODE_IRQ)
  {
    config_req->irq_config = sensor_state->irq_config;
  }
  else if(sensor_state->int_mode == AK0991X_INT_OP_MODE_IBI)
  {
    config_req->ibi_config             =
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
  return(dae_if->mag.stream != NULL &&
         dae_if->mag.stream_usable  &&
         dae_if->mag.state == INIT_PENDING);
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

#else //defined(AK0991X_ENABLE_DAE)

void ak0991x_dae_if_check_support(sns_sensor *this)
{
  UNUSED_VAR(this);
}
bool ak0991x_dae_if_is_initializing(sns_sensor_instance *this)
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
void ak0991x_dae_if_process_sensor_events(sns_sensor *this)
{
  UNUSED_VAR(this);
}
#endif //defined(AK0991X_ENABLE_DAE)

