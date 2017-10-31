/**
 * @file sns_ak0991x_sensor.c
 *
 * Common implementation for AK0991X Sensors.
 *
 * Copyright (c) 2016-2017 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 **/

#include <string.h>
#include "sns_mem_util.h"
#include "sns_service_manager.h"
#include "sns_stream_service.h"
#include "sns_service.h"
#include "sns_sensor_util.h"
#include "sns_types.h"
#include "sns_attribute_util.h"

#include "sns_ak0991x_sensor.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"
#include "sns_printf.h"

/* device specific information */
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09911)
float ak09911_odr_table[] =
{AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};
static char *ak09911_ope_mode_table[] = {AK0991X_NORMAL};
#endif

#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09912)
float ak09912_odr_table[] =
{AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};
static char *ak09912_ope_mode_table[] = {AK0991X_NORMAL};
#endif

#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09913)
float ak09913_odr_table[] =
{AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};
static char *ak09913_ope_mode_table[] = {AK0991X_NORMAL};
#endif

#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09915C) || defined(AK0991X_TARGET_AK09915D)
float ak09915_odr_table[] =
{AK0991X_ODR_1, AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};
static char *ak09915_ope_mode_table[] = {AK0991X_LOW_POWER, AK0991X_LOW_NOISE};
#endif

#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09916C) || defined(AK0991X_TARGET_AK09916D)
float ak09916_odr_table[] =
{AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};
static char *ak09916_ope_mode_table[] = {AK0991X_NORMAL};
#endif

#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09917)
float ak09917_odr_table[] =
{AK0991X_ODR_1, AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};
static char *ak09917_ope_mode_table[] = {AK0991X_LOW_POWER, AK0991X_LOW_NOISE};
#endif

#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09918)
float ak09918_odr_table[] =
{AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};
static char *ak09918_ope_mode_table[] = {AK0991X_NORMAL};
#endif


typedef struct ak0991x_dev_info
{
  float      *odr;
  float      resolutions;
  uint32_t   max_fifo_depth;
  uint32_t   active_current;
  uint32_t   sleep_current;
  range_attr ranges;
  char       **operating_modes;
  bool       supports_dri;
  bool       supports_sync_stream;
} ak0991x_dev_info;

const struct ak0991x_dev_info ak0991x_dev_info_array[] = {
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09911)
  [AK09911] = {
    .odr                  = ak09911_odr_table,
    .resolutions          = AK09911_RESOLUTION,
    .max_fifo_depth       = AK09911_FIFO_SIZE,
    .active_current       = AK09911_HI_PWR,
    .sleep_current        = AK09911_LO_PWR,
    .ranges               = {AK09911_MIN_RANGE, AK09911_MAX_RANGE},
    .operating_modes      = ak09911_ope_mode_table,
    .supports_dri         = false,
    .supports_sync_stream = false,
  },
#endif
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09912)
  [AK09912] = {
    .odr                  = ak09912_odr_table,
    .resolutions          = AK09912_RESOLUTION,
    .max_fifo_depth       = AK09912_FIFO_SIZE,
    .active_current       = AK09912_HI_PWR,
    .sleep_current        = AK09912_LO_PWR,
    .ranges               = {AK09912_MIN_RANGE, AK09912_MAX_RANGE},
    .operating_modes      = ak09912_ope_mode_table,
    .supports_dri         = true,
    .supports_sync_stream = false,
  },
#endif
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09913)
  [AK09913] = {
    .odr                  = ak09913_odr_table,
    .resolutions          = AK09913_RESOLUTION,
    .max_fifo_depth       = AK09913_FIFO_SIZE,
    .active_current       = AK09913_HI_PWR,
    .sleep_current        = AK09913_LO_PWR,
    .ranges               = {AK09913_MIN_RANGE, AK09913_MAX_RANGE},
    .operating_modes      = ak09913_ope_mode_table,
    .supports_dri         = false,
    .supports_sync_stream = false,
  },
#endif
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09915C)
  [AK09915C] = {
    .odr                  = ak09915_odr_table,
    .resolutions          = AK09915_RESOLUTION,
    .max_fifo_depth       = AK09915_FIFO_SIZE,
    .active_current       = AK09915_HI_PWR,
    .sleep_current        = AK09915_LO_PWR,
    .ranges               = {AK09915_MIN_RANGE, AK09915_MAX_RANGE},
    .operating_modes      = ak09915_ope_mode_table,
    .supports_dri         = true,
    .supports_sync_stream = false,
  },
#endif
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09915D)
  [AK09915D] = {
    .odr                  = ak09915_odr_table,
    .resolutions          = AK09915_RESOLUTION,
    .max_fifo_depth       = AK09915_FIFO_SIZE,
    .active_current       = AK09915_HI_PWR,
    .sleep_current        = AK09915_LO_PWR,
    .ranges               = {AK09915_MIN_RANGE, AK09915_MAX_RANGE},
    .operating_modes      = ak09915_ope_mode_table,
    .supports_dri         = true,
    .supports_sync_stream = true,
  },
#endif
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09916C)
  [AK09916C] = {
    .odr                  = ak09916_odr_table,
    .resolutions          = AK09916_RESOLUTION,
    .max_fifo_depth       = AK09916_FIFO_SIZE,
    .active_current       = AK09916_HI_PWR,
    .sleep_current        = AK09916_LO_PWR,
    .ranges               = {AK09916_MIN_RANGE, AK09916_MAX_RANGE},
    .operating_modes      = ak09916_ope_mode_table,
    .supports_dri         = false,
    .supports_sync_stream = false,
  },
#endif
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09916D)
  [AK09916D] = {
    .odr                  = ak09916_odr_table,
    .resolutions          = AK09916_RESOLUTION,
    .max_fifo_depth       = AK09916_FIFO_SIZE,
    .active_current       = AK09916_HI_PWR,
    .sleep_current        = AK09916_LO_PWR,
    .ranges               = {AK09916_MIN_RANGE, AK09916_MAX_RANGE},
    .operating_modes      = ak09916_ope_mode_table,
    .supports_dri         = true,
    .supports_sync_stream = false,
  },
#endif
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09917)
  [AK09917] = {
    .odr                  = ak09917_odr_table,
    .resolutions          = AK09917_RESOLUTION,
    .max_fifo_depth       = AK09917_FIFO_SIZE,
    .active_current       = AK09917_HI_PWR,
    .sleep_current        = AK09917_LO_PWR,
    .ranges               = {AK09917_MIN_RANGE, AK09917_MAX_RANGE},
    .operating_modes      = ak09917_ope_mode_table,
    .supports_dri         = true,
    .supports_sync_stream = true,
  },
#endif
#if defined(AK0991X_ENABLE_ALL_DEVICES) || defined(AK0991X_TARGET_AK09918)
  [AK09918] = {
    .odr                  = ak09918_odr_table,
    .resolutions          = AK09918_RESOLUTION,
    .max_fifo_depth       = AK09918_FIFO_SIZE,
    .active_current       = AK09918_HI_PWR,
    .sleep_current        = AK09918_LO_PWR,
    .ranges               = {AK09918_MIN_RANGE, AK09918_MAX_RANGE},
    .operating_modes      = ak09918_ope_mode_table,
    .supports_dri         = false,
    .supports_sync_stream = false,
  },
#endif
};

/**
 * Returns decoded request message for type
 * sns_sensor_stream_config.
 *
 * @param[in] in_request   Request as stored in client_requests
 *                         list.
 * @param decoded_request  Standard decoded message.
 * @param decoded_payload  Decoded stream request payload.
 *
 * @return bool true if decode is successful else false
 */
static bool ak0991x_get_decoded_mag_request(
                                            sns_sensor const *this,
                                            sns_request const *in_request,
                                            sns_std_request *decoded_request,
                                            sns_std_sensor_config *decoded_payload)
{
  pb_istream_t stream;
  pb_simple_cb_arg arg =
  { .decoded_struct = decoded_payload,
    .fields = sns_std_sensor_config_fields };
  decoded_request->payload = (struct pb_callback_s)
  { .funcs.decode = &pb_decode_simple_cb, .arg = &arg };
  stream = pb_istream_from_buffer(in_request->request,
                                  in_request->request_len);

  if (!pb_decode(&stream, sns_std_request_fields, decoded_request))
  {
    AK0991X_PRINT(ERROR, this, "AK0991X decode error");
    return false;
  }

#ifndef AK0991X_ENABLE_DEBUG_MSG
  UNUSED_VAR(this);
#endif

  return true;
}

static void ak0991x_get_mag_config(
                                   sns_sensor const *this,
                                   sns_sensor_instance *instance,
                                   float *chosen_sample_rate,
                                   float *chosen_report_rate,
                                   uint32_t *chosen_flush_period,
                                   bool *is_flush_only,
                                   bool *sensor_client_present)
{
#ifdef AK0991X_ENABLE_DUAL_SENSOR
  ak0991x_state *state = (ak0991x_state *)this->state->state;
  sns_sensor_uid mag_suid = (state->hardware_id == 0)? (sns_sensor_uid)MAG_SUID1 : (sns_sensor_uid)MAG_SUID2;
#else
  sns_sensor_uid mag_suid = (sns_sensor_uid)MAG_SUID1;
#endif
  sns_request const *request;

  *chosen_report_rate = 0;
  *chosen_sample_rate = 0;
  *chosen_flush_period = 0;
  *is_flush_only = true;
  *sensor_client_present = false;

  /** Parse through existing requests and get fastest sample
   *  rate, report rate, and longest flush period requests. */
  for (request = instance->cb->get_client_request(instance, &mag_suid, true);
       NULL != request;
       request = instance->cb->get_client_request(instance, &mag_suid, false))
  {
    sns_std_request decoded_request;
    sns_std_sensor_config decoded_payload = sns_std_sensor_config_init_default;

    if(request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
    {
      if(ak0991x_get_decoded_mag_request(
          this, request, &decoded_request, &decoded_payload))
      {
        float report_rate;
        uint32_t flush_period;

        *is_flush_only &= decoded_request.batching.flush_only;

        *chosen_sample_rate = SNS_MAX(*chosen_sample_rate,
                                      decoded_payload.sample_rate);

        if (decoded_request.has_batching
            &&
            decoded_request.batching.batch_period > 0)
        {
          report_rate = 1000000.0f / (float)decoded_request.batching.batch_period;
          if( decoded_request.batching.has_flush_period )
          {
            flush_period = decoded_request.batching.flush_period;
          }
          else
          {
            flush_period = UINT32_MAX;
          }
        }
        else
        {
          report_rate = *chosen_sample_rate;
          flush_period = UINT32_MAX;
        }


        *chosen_report_rate = SNS_MAX(*chosen_report_rate,
                                      report_rate);
        *chosen_flush_period = SNS_MAX(*chosen_flush_period,
                                       flush_period);
        *sensor_client_present = true;
      }
    }
  }
}

static void ak0991x_set_mag_inst_config(sns_sensor *this,
                                        sns_sensor_instance *instance,
                                        float chosen_report_rate,
                                        float chosen_sample_rate,
                                        sns_ak0991x_registry_cfg registry_cfg,
                                        uint32_t chosen_flush_period,
                                        bool is_flush_only)
{
  sns_ak0991x_mag_req new_client_config;
  sns_request config;

  new_client_config.report_rate = chosen_report_rate;
  new_client_config.sample_rate = chosen_sample_rate;
  new_client_config.flush_period = chosen_flush_period;
  new_client_config.registry_cfg = registry_cfg;
  new_client_config.is_flush_only = is_flush_only;

  config.message_id = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG;
  config.request_len = sizeof(sns_ak0991x_mag_req);
  config.request = &new_client_config;

  this->instance_api->set_client_config(instance, &config);
}

/**
 * This function parse the client_request list per Sensor and
 * determines final config for the Sensor Instance.
 *
 * @param[i] this        Sensor reference
 * @param[i] instance    Sensor Instance of config
 *
 * @return none
 */
static void ak0991x_reval_instance_config(sns_sensor *this,
                                   sns_sensor_instance *instance)
{
  /**
   * 1. Get best Mag Config.
   * 2. Decide best Instance Config based on above outputs.
   */
  float chosen_sample_rate = 0;
  float chosen_report_rate = 0;
  uint32_t chosen_flush_period = 0;
  bool is_flush_only = false;
  bool m_sensor_client_present;
  UNUSED_VAR(instance);
  ak0991x_state *state = (ak0991x_state*)this->state->state;
  sns_ak0991x_registry_cfg registry_cfg;

  AK0991X_PRINT(LOW, this, "ak0991x_reval_instance_config");

  ak0991x_get_mag_config(
                         this,
                         instance,
                         &chosen_sample_rate,
                         &chosen_report_rate,
                         &chosen_flush_period,
                         &is_flush_only,
                         &m_sensor_client_present);

  sns_memscpy(registry_cfg.fac_cal_bias, sizeof(registry_cfg.fac_cal_bias),
      state->fac_cal_bias, sizeof(state->fac_cal_bias));

  sns_memscpy(&registry_cfg.fac_cal_corr_mat, sizeof(registry_cfg.fac_cal_corr_mat),
      &state->fac_cal_corr_mat, sizeof(state->fac_cal_corr_mat));

  ak0991x_set_mag_inst_config(this,
                              instance,
                              chosen_report_rate,
                              chosen_sample_rate,
                              registry_cfg,
                              chosen_flush_period,
                              is_flush_only);
}

static sns_rc ak0991x_register_com_port(sns_sensor *const this)
{
  sns_rc           rv = SNS_RC_SUCCESS;
  ak0991x_state *state = (ak0991x_state *)this->state->state;
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);

  AK0991X_PRINT(LOW, this, "ak0991x_register_com_port");

  if (NULL == state->com_port_info.port_handle)
  {
    state->scp_service =  (sns_sync_com_port_service *)
        service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);

    rv = state->scp_service->api->sns_scp_register_com_port(
      &state->com_port_info.com_config,
      &state->com_port_info.port_handle);

    if(rv == SNS_RC_SUCCESS)
    {
      rv = state->scp_service->api->sns_scp_open(state->com_port_info.port_handle);
      AK0991X_PRINT(LOW, this, "Open port: %u",rv);
    }
    else
    {
      AK0991X_PRINT(ERROR, this, "sns_scp_register_com_port fail rc:%u",rv);
    }
  }
  return rv;
}

static void ak0991x_register_power_rails(sns_sensor *const this)
{
  AK0991X_PRINT(LOW, this, "ak0991x_register_power_rails");

   ak0991x_state *state = (ak0991x_state *)this->state->state;
   sns_service_manager *service_mgr = this->cb->get_service_manager(this);

   state->rail_config.rail_vote = SNS_RAIL_OFF;

   state->pwr_rail_service =
     (sns_pwr_rail_service*)service_mgr->get_service(service_mgr,
                                                     SNS_POWER_RAIL_SERVICE);

   state->pwr_rail_service->api->sns_register_power_rails(state->pwr_rail_service,
                                                          &state->rail_config);
}

static void ak0991x_send_flush_config(sns_sensor *const this,
                                      sns_sensor_instance *instance)
{
  sns_request config;

  config.message_id = SNS_STD_MSGID_SNS_STD_FLUSH_REQ;
  config.request_len = 0;
  config.request = NULL;

  this->instance_api->set_client_config(instance, &config);
}

static void ak0991x_start_power_rail_timer(sns_sensor *const this,
                                           sns_time timeout_ticks,
                                           ak0991x_power_rail_pending_state pwr_rail_pend_state)
{
  ak0991x_state *state = (ak0991x_state *)this->state->state;

  sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
  size_t                  req_len;
  uint8_t                 buffer[20];
  req_payload.is_periodic = false;
  req_payload.start_time = sns_get_system_time();
  req_payload.timeout_period = timeout_ticks;
  sns_service_manager *smgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_svc =
     (sns_stream_service*)smgr->get_service(smgr, SNS_STREAM_SERVICE);

  AK0991X_PRINT(LOW, this, "start_power_rail_timer: timeout=%u state=%u",
                (uint32_t)timeout_ticks, pwr_rail_pend_state);

  if (state->timer_stream == NULL)
  {
    sns_sensor_uid suid;

    sns_suid_lookup_get(&state->suid_lookup_data, "timer", &suid);
    stream_svc->api->create_sensor_stream(stream_svc, this, suid,
                                                &state->timer_stream);
    state->remove_timer_stream = false;
  }

  req_len = pb_encode_request(buffer, sizeof(buffer), &req_payload,
                              sns_timer_sensor_config_fields, NULL);

  if (req_len > 0)
  {
    sns_request timer_req =
    {  .message_id = SNS_TIMER_MSGID_SNS_TIMER_SENSOR_CONFIG,
       .request = buffer, .request_len = req_len};
    state->timer_stream->api->send_request(state->timer_stream, &timer_req);
    state->power_rail_pend_state = pwr_rail_pend_state;
  }
  else
  {
    AK0991X_PRINT(ERROR, this, "AK0991x timer req encode error");
  }
}

#ifdef AK0991X_ENABLE_REGISTRY_ACCESS
static void ak0991x_sensor_send_registry_request(sns_sensor *const this,
                                                 char *reg_group_name)
{
  ak0991x_state *state = (ak0991x_state *)this->state->state;
  uint8_t buffer[100];
  int32_t encoded_len;
  sns_memset(buffer, 0, sizeof(buffer));
  sns_rc rc = SNS_RC_SUCCESS;

  sns_registry_read_req read_request;
  pb_buffer_arg data = (pb_buffer_arg){ .buf = reg_group_name,
    .buf_len = (strlen(reg_group_name) + 1) };

  read_request.name.arg = &data;
  read_request.name.funcs.encode = pb_encode_string_cb;

  encoded_len = pb_encode_request(buffer, sizeof(buffer),
      &read_request, sns_registry_read_req_fields, NULL);
  if(0 < encoded_len)
  {
    sns_request request = (sns_request){
      .request_len = encoded_len, .request = buffer,
      .message_id = SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_REQ };
    rc = state->reg_data_stream->api->send_request(state->reg_data_stream, &request);
  }

  //SENSOR_PRINTF_LOW_FULL(this, "Sending registry request for group name:%s", reg_group_name);
}

static void ak0991x_request_registry(sns_sensor *const this)
{
  ak0991x_state *state = (ak0991x_state*)this->state->state;
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_svc = (sns_stream_service*)
    service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

  // place a request to registry sensor

  if(state->reg_data_stream == NULL)
  {
    sns_sensor_uid reg_suid;
    sns_suid_lookup_get(&state->suid_lookup_data, "registry", &reg_suid);
    stream_svc->api->create_sensor_stream(stream_svc, this, reg_suid,
        &state->reg_data_stream);

#ifdef AK0991X_ENABLE_DUAL_SENSOR
    // Each driver registration shall be treated as a separate library within SEE.
    // the below case is selected by SEE for each sensor as a separate library.
    // hw_id = 0 is considered one library,
    // the case is "true" to register for ak0991x_dri_0.json and msm8996_ak9911x_0.json.
    // hw_id = 1 is considered another library,
    // the case is "false" to register for a0991x_dri_1.json and msm8996_ak9911x_1.json.
    //
    // TODO:
    if (0)
    {
      ak0991x_sensor_send_registry_request(this, AK0991X_REGISTRY_0_PF_CONFIG);
      ak0991x_sensor_send_registry_request(this, AK0991X_REGISTRY_0_PLACE);
      ak0991x_sensor_send_registry_request(this, AK0991X_REGISTRY_0_ORIENT);
      ak0991x_sensor_send_registry_request(this, AK0991X_REGISTRY_0_FACCAL);
      ak0991x_sensor_send_registry_request(this, AK0991X_REGISTRY_0_MAG_CONFIG);
      ak0991x_sensor_send_registry_request(this, AK0991X_REGISTRY_0_REG_CONFIG);
    }
    else
    {
      ak0991x_sensor_send_registry_request(this, AK0991X_REGISTRY_1_PF_CONFIG);
      ak0991x_sensor_send_registry_request(this, AK0991X_REGISTRY_1_PLACE);
      ak0991x_sensor_send_registry_request(this, AK0991X_REGISTRY_1_ORIENT);
      ak0991x_sensor_send_registry_request(this, AK0991X_REGISTRY_1_FACCAL);
      ak0991x_sensor_send_registry_request(this, AK0991X_REGISTRY_1_MAG_CONFIG);
      ak0991x_sensor_send_registry_request(this, AK0991X_REGISTRY_1_REG_CONFIG);
    }
#else
    ak0991x_sensor_send_registry_request(this, AK0991X_REGISTRY_0_PF_CONFIG);
    ak0991x_sensor_send_registry_request(this, AK0991X_REGISTRY_0_PLACE);
    ak0991x_sensor_send_registry_request(this, AK0991X_REGISTRY_0_ORIENT);
    ak0991x_sensor_send_registry_request(this, AK0991X_REGISTRY_0_FACCAL);
    ak0991x_sensor_send_registry_request(this, AK0991X_REGISTRY_0_MAG_CONFIG);
    ak0991x_sensor_send_registry_request(this, AK0991X_REGISTRY_0_REG_CONFIG);
#endif // AK0991X_ENABLE_DUAL_SENSOR
  }
}
#endif

#ifdef AK0991X_ENABLE_REGISTRY_ACCESS
static bool ak0991x_registry_parse_phy_sensor_cfg(sns_registry_data_item *reg_item,
                                                  pb_buffer_arg *item_name,
                                                  pb_buffer_arg *item_str_val,
                                                  void *parsed_buffer)
{
  bool rv = true;

  if(NULL == reg_item || NULL == item_name || NULL == item_str_val ||
     NULL == parsed_buffer)
  {
    rv = false;
  }
  else if(reg_item->has_sint)
  {
    ak0991x_registry_phy_sensor_cfg *cfg =
      (ak0991x_registry_phy_sensor_cfg *)parsed_buffer;

    if(0 == strncmp((char*)item_name->buf,
                    "use_fifo",
                    item_name->buf_len))
    {
      cfg->use_fifo = (reg_item->sint == 1) ? true : false;
    }
    else if(0 == strncmp((char*)item_name->buf,
                         "nsf",
                         item_name->buf_len))
    {
      cfg->nsf = reg_item->sint;
    }
    else if(0 == strncmp((char*)item_name->buf,
                         "sdr",
                         item_name->buf_len))
    {
      cfg->sdr = reg_item->sint;
    }
  }
  else
  {
    rv = false;
  }

  return rv;
}

static void ak0991x_sensor_process_registry_event(sns_sensor *const this,
                                                  sns_sensor_event *event)
{
  bool rv = true;
  sns_rc rc = SNS_RC_SUCCESS;
  ak0991x_state *state = (ak0991x_state *)this->state->state;

  pb_istream_t stream = pb_istream_from_buffer((void*)event->event,
      event->event_len);

  AK0991X_PRINT(LOW, this, "ak0991x_sensor_process_registry_event");

  if(SNS_REGISTRY_MSGID_SNS_REGISTRY_READ_EVENT == event->message_id)
  {
    sns_registry_read_event read_event = sns_registry_read_event_init_default;
    pb_buffer_arg group_name = {0,0};
    read_event.name.arg = &group_name;
    read_event.name.funcs.decode = pb_decode_string_cb;
 
    if(!pb_decode(&stream, sns_registry_read_event_fields, &read_event))
    {
      AK0991X_PRINT(ERROR, this, "Error decoding registry event");
    }
    else
    {
      stream = pb_istream_from_buffer((void*)event->event, event->event_len);
      bool mag_config;
      bool reg_config;
      bool pf_config;
      bool place;
      bool orient;
      bool faccal;
      mag_config = (0 == strncmp((char*)group_name.buf, AK0991X_REGISTRY_0_MAG_CONFIG,
                           group_name.buf_len));
      reg_config = (0 == strncmp((char*)group_name.buf, AK0991X_REGISTRY_0_REG_CONFIG,
                           group_name.buf_len));
      pf_config = (0 == strncmp((char*)group_name.buf, AK0991X_REGISTRY_0_PF_CONFIG,
                           group_name.buf_len));
      place = (0 == strncmp((char*)group_name.buf, AK0991X_REGISTRY_0_PLACE,
                           group_name.buf_len));
      orient = (0 == strncmp((char*)group_name.buf, AK0991X_REGISTRY_0_ORIENT,
                             group_name.buf_len));
      faccal = (0 == strncmp((char*)group_name.buf, AK0991X_REGISTRY_0_FACCAL,
                             group_name.buf_len));
#ifdef AK0991X_ENABLE_DUAL_SENSOR
      mag_config |= (0 == strncmp((char*)group_name.buf, AK0991X_REGISTRY_1_MAG_CONFIG,
                           group_name.buf_len));
      reg_config |= (0 == strncmp((char*)group_name.buf, AK0991X_REGISTRY_1_REG_CONFIG,
                           group_name.buf_len));
      pf_config |= (0 == strncmp((char*)group_name.buf, AK0991X_REGISTRY_1_PF_CONFIG,
                           group_name.buf_len));
      place |= (0 == strncmp((char*)group_name.buf, AK0991X_REGISTRY_1_PLACE,
                           group_name.buf_len));
      orient |= (0 == strncmp((char*)group_name.buf, AK0991X_REGISTRY_1_ORIENT,
                           group_name.buf_len));
      faccal |= (0 == strncmp((char*)group_name.buf, AK0991X_REGISTRY_1_FACCAL,
                            group_name.buf_len));
#endif
      if(mag_config)
      {
        {
          sns_registry_decode_arg arg = {
            .item_group_name = &group_name,
            .parse_info_len = 1,
            .parse_info[0] = {
            .group_name = "config",
            .parse_func = sns_registry_parse_phy_sensor_cfg,
            .parsed_buffer = &state->registry_cfg }
          };

          read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
          read_event.data.items.arg = &arg;

          rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
        }

        if(rv)
        {
          state->registry_cfg_received = true;
          state->is_dri = state->registry_cfg.is_dri;
          state->hardware_id = state->registry_cfg.hw_id;
          state->resolution_idx = state->registry_cfg.res_idx;
          state->supports_sync_stream = state->registry_cfg.sync_stream;

          AK0991X_PRINT(LOW, this, "is_dri:%d, hardware_id:%d ",
                                   state->is_dri,
                                   (uint32_t)state->hardware_id);

          AK0991X_PRINT(LOW, this, "resolution_idx:%d, supports_sync_stream:%d ",
                                   state->resolution_idx,
                                   state->supports_sync_stream);
        }
      }

      if(reg_config)
      {
        {
          sns_registry_decode_arg arg = {
            .item_group_name = &group_name,
            .parse_info_len = 1,
            .parse_info[0] = {
            .group_name = "config_2",
            .parse_func = ak0991x_registry_parse_phy_sensor_cfg,
            .parsed_buffer = &state->registry_reg_cfg }
          };

          read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
          read_event.data.items.arg = &arg;

          rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
        }

        if(rv)
        {
          state->registry_reg_cfg_received = true;
          state->use_fifo = state->registry_reg_cfg.use_fifo;
          state->nsf = state->registry_reg_cfg.nsf;
          state->sdr = state->registry_reg_cfg.sdr;


          AK0991X_PRINT(LOW, this, "use_fifo:%d, nsf:%d ,sdr:%d",
                                   state->use_fifo,
                                   state->nsf,
                                   state->sdr);
        }
      }
      else if (pf_config)
      {
        {
          sns_registry_decode_arg arg = {
            .item_group_name = &group_name,
            .parse_info_len = 1,
            .parse_info[0] = {
              .group_name = "config",
              .parse_func = sns_registry_parse_phy_sensor_pf_cfg,
              .parsed_buffer = &state->registry_pf_cfg }
          };

          read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
          read_event.data.items.arg = &arg;

          rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
        }

        if(rv)
        {
          state->registry_pf_cfg_received = true;

          state->com_port_info.com_config.bus_type = state->registry_pf_cfg.bus_type;
          state->com_port_info.com_config.bus_instance = state->registry_pf_cfg.bus_instance;
          state->com_port_info.com_config.slave_control = state->registry_pf_cfg.slave_config;
          state->com_port_info.com_config.min_bus_speed_KHz = state->registry_pf_cfg.min_bus_speed_khz;
          state->com_port_info.com_config.max_bus_speed_KHz = state->registry_pf_cfg.max_bus_speed_khz;
          state->com_port_info.com_config.reg_addr_type = state->registry_pf_cfg.reg_addr_type;
          state->irq_config.interrupt_num = state->registry_pf_cfg.dri_irq_num;
          state->irq_config.interrupt_pull_type = state->registry_pf_cfg.irq_pull_type;
          state->irq_config.is_chip_pin = state->registry_pf_cfg.irq_is_chip_pin;
          state->irq_config.interrupt_drive_strength = state->registry_pf_cfg.irq_drive_strength;
          state->irq_config.interrupt_trigger_type = state->registry_pf_cfg.irq_trigger_type;
          state->rail_config.num_of_rails = state->registry_pf_cfg.num_rail;
          state->registry_rail_on_state = state->registry_pf_cfg.rail_on_state;
          sns_strlcpy(state->rail_config.rails[0].name,
                      state->registry_pf_cfg.vddio_rail,
                      sizeof(state->rail_config.rails[0].name));
          sns_strlcpy(state->rail_config.rails[1].name,
                      state->registry_pf_cfg.vdd_rail,
                      sizeof(state->rail_config.rails[1].name));
          AK0991X_PRINT(LOW, this, "bus_type:%d bus_instance:%d slave_control:%d",
                     state->com_port_info.com_config.bus_type,
                     state->com_port_info.com_config.bus_instance,
                     state->com_port_info.com_config.slave_control);

          AK0991X_PRINT(LOW, this, "min_bus_speed_KHz :%d max_bus_speed_KHz:%d reg_addr_type:%d",
                     state->com_port_info.com_config.min_bus_speed_KHz,
                     state->com_port_info.com_config.max_bus_speed_KHz,
                     state->com_port_info.com_config.reg_addr_type);

          AK0991X_PRINT(LOW, this, "interrupt_num:%d interrupt_pull_type:%d is_chip_pin:%d",
                     state->irq_config.interrupt_num,
                     state->irq_config.interrupt_pull_type,
                     state->irq_config.is_chip_pin);

          AK0991X_PRINT(LOW, this, "interrupt_drive_strength:%d interrupt_trigger_type:%d"
                     " rigid body type:%d",
                     state->irq_config.interrupt_drive_strength,
                     state->irq_config.interrupt_trigger_type,
                     state->registry_pf_cfg.rigid_body_type);

          AK0991X_PRINT(LOW, this, "num_rail:%d, rail_on_state:%d",
                     state->rail_config.num_of_rails,
                     state->registry_rail_on_state);

          /**---------------------Register Com Ports --------------------------*/
          rc = ak0991x_register_com_port(this);

          /**---------------------Register Power Rails --------------------------*/
          if(NULL == state->pwr_rail_service && rc == SNS_RC_SUCCESS)
          {
            ak0991x_register_power_rails(this);
          }
        }
      }
      else if (place)
      {
        {
          uint8_t arr_index = 0;
          pb_float_arr_arg arr_arg = {
            .arr = state->placement,
            .arr_index = &arr_index,
            .arr_len = 12
          };

          sns_registry_decode_arg arg = {
            .item_group_name = &group_name,
            .parse_info_len = 1,
            .parse_info[0] = {
              .group_name = "placement",
              .parse_func = sns_registry_parse_float_arr,
              .parsed_buffer = &arr_arg
            }
          };

          read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
          read_event.data.items.arg = &arg;

          rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
        }

        if(rv)
        {
          state->registry_placement_received = true;
          AK0991X_PRINT(LOW, this, "p[0]:%d p[6]:%d p[11]:%d",
              (int)state->placement[0],(int)state->placement[6],
              (int)state->placement[11]);
        }
      }
      else if (orient)
      {
        {
          sns_registry_decode_arg arg = {
            .item_group_name = &group_name,
            .parse_info_len = 1,
            .parse_info[0] = {
              .group_name = "orient",
              .parse_func = sns_registry_parse_axis_orientation,
              .parsed_buffer = state->axis_map
            }
          };

          read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
          read_event.data.items.arg = &arg;

          rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
        }

        if(rv)
        {
          state->registry_orient_received = true;

          AK0991X_PRINT(LOW, this, "Input Axis:%d maps to Output Axis:%d with inversion %d",
                 state->axis_map[0].ipaxis,
                 state->axis_map[0].opaxis, state->axis_map[0].invert);

          AK0991X_PRINT(LOW, this, "Input Axis:%d maps to Output Axis:%d with inversion %d",
                 state->axis_map[1].ipaxis, state->axis_map[1].opaxis,
                 state->axis_map[1].invert);

          AK0991X_PRINT(LOW, this, "Input Axis:%d maps to Output Axis:%d with inversion %d",
                 state->axis_map[2].ipaxis, state->axis_map[2].opaxis,
                 state->axis_map[2].invert);
        }
      }
      else if (faccal)
      {
        {
          uint8_t bias_arr_index = 0, scale_arr_index = 0;
          pb_float_arr_arg bias_arr_arg = {
            .arr = state->fac_cal_bias,
            .arr_index = &bias_arr_index,
            .arr_len = TRIAXIS_NUM
          };

          pb_float_arr_arg scale_arr_arg = {
            .arr = state->fac_cal_scale,
            .arr_index = &scale_arr_index,
            .arr_len = TRIAXIS_NUM
          };

          sns_registry_decode_arg arg = {
            .item_group_name = &group_name,
            .parse_info_len = 3,
            .parse_info[0] = {
              .group_name = "bias",
              .parse_func = sns_registry_parse_float_arr,
              .parsed_buffer = &bias_arr_arg
            },
            .parse_info[1] = {
              .group_name = "scale",
              .parse_func = sns_registry_parse_float_arr,
              .parsed_buffer = &scale_arr_arg
            },
            .parse_info[2] = {
              .group_name = "corr_mat",
              .parse_func = sns_registry_parse_corr_matrix_3,
              .parsed_buffer = &state->fac_cal_corr_mat
            }
          };

          read_event.data.items.funcs.decode = &sns_registry_item_decode_cb;
          read_event.data.items.arg = &arg;

          rv = pb_decode(&stream, sns_registry_read_event_fields, &read_event);
        }

        if(rv)
        {
          state->registry_fac_cal_received = true;
          if(state->fac_cal_scale[0] != 0.0)
          {
            state->fac_cal_corr_mat.e00 = state->fac_cal_scale[0];
            state->fac_cal_corr_mat.e11 = state->fac_cal_scale[1];
            state->fac_cal_corr_mat.e22 = state->fac_cal_scale[2];
          }

          //AK0991X_PRINT(ERROR, this, "Fac Cal Corr Matrix e00:%f e01:%f e02:%f", state->fac_cal_corr_mat.e00,state->fac_cal_corr_mat.e01,
          //       state->fac_cal_corr_mat.e02);
          //AK0991X_PRINT(ERROR, this, "Fac Cal Corr Matrix e10:%f e11:%f e12:%f", state->fac_cal_corr_mat.e10,state->fac_cal_corr_mat.e11,
          //       state->fac_cal_corr_mat.e12);
          //AK0991X_PRINT(ERROR, this, "Fac Cal Corr Matrix e20:%f e21:%f e22:%f", state->fac_cal_corr_mat.e20,state->fac_cal_corr_mat.e21,
          //       state->fac_cal_corr_mat.e22);
          //AK0991X_PRINT(ERROR, this, "Fac Cal Bias x:%f y:%f z:%f", state->fac_cal_bias[0], state->fac_cal_bias[1],
          //       state->fac_cal_bias[2]);
        }
      }
      else
      {
        rv = false;
      }
      if(!rv)
      {
        //SENSOR_PRINTF_MED_FULL(this, "Error decoding registry group %s due to %s", (char*)group_name.buf,
        //                       PB_GET_ERROR(&stream));
      }
    }
  }
  else
  {
    AK0991X_PRINT(ERROR, this, "Received unsupported registry event msg id %u",
                             event->message_id);
  }
}
#endif

/**
 * Publish attributes read from registry
 *
 * @param[i] this    reference to this Sensor
 *
 * @return none
 */
#ifdef AK0991X_ENABLE_ALL_ATTRIBUTES
static void
ak0991x_publish_registry_attributes(sns_sensor *const this)
{
  ak0991x_state *state = (ak0991x_state*)this->state->state;
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = state->is_dri;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_DRI, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_boolean = true;
    value.boolean = state->supports_sync_stream;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_STREAM_SYNC, &value, 1, false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
    value.sint = state->hardware_id;
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_HW_ID, &value, 1, false);
  }
  {
    sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR,
      SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR};
    for(uint8_t i =0; i < 12; i++)
    {
      values[i].has_flt = true;
      values[i].flt = state->placement[i];
    }
    sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_PLACEMENT,
        values, ARR_SIZE(values), false);
  }
  {
    sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
    value.has_sint = true;
#ifdef AK0991X_ENABLE_REGISTRY_ACCESS
    value.sint = state->registry_pf_cfg.rigid_body_type;
#else
    value.sint = 0; //
#endif
    sns_publish_attribute(
        this, SNS_STD_SENSOR_ATTRID_RIGID_BODY, &value, 1, false);
  }
}
#endif

#ifdef AK0991X_ENABLE_REGISTRY_ACCESS
static sns_rc ak0991x_process_registry_events(sns_sensor *const this)
{
  AK0991X_PRINT(LOW, this, "ak0991x_process_registry_events");

  sns_rc rv = SNS_RC_SUCCESS;
  ak0991x_state *state = (ak0991x_state *)this->state->state;
  sns_sensor_event *event;

  if(NULL != state->reg_data_stream)
  {
    event = state->reg_data_stream->api->peek_input(state->reg_data_stream);
    while(NULL != event)
    {
      ak0991x_sensor_process_registry_event(this, event);

#ifdef  AK0991X_ENABLE_ALL_ATTRIBUTES
      if(state->registry_cfg_received && state->registry_placement_received)
      {
        ak0991x_publish_registry_attributes(this);
      }
#endif

      event = state->reg_data_stream->api->get_next_input(state->reg_data_stream);
    }
  }

  if(NULL != state->reg_data_stream
     && state->registry_cfg_received
     && state->registry_pf_cfg_received
     && state->registry_orient_received
     && state->registry_fac_cal_received
     && state->registry_placement_received)
  {
    // Done receiving all registry.
    // remove unused reg_data_stream.
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_mgr =
      (sns_stream_service *)service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

    stream_mgr->api->remove_stream(stream_mgr, state->reg_data_stream);
    state->reg_data_stream = NULL;
  }
  return rv;
}
#endif

static void
ak0991x_sensor_publish_available(sns_sensor *const this)
{
  AK0991X_PRINT(MED, this, "AK0991x HW Present. Publishing available");
  sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
  value.has_boolean = true;
  value.boolean = true;
  sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_AVAILABLE,
                        &value, 1, true);
}

/**
 * Initialize attributes to their default state.  They may/will be updated
 * within notify_event.
 */
static void ak0991x_publish_hw_attributes(sns_sensor *const this,
                                akm_device_type device_select)
{
#ifdef  AK0991X_ENABLE_ALL_ATTRIBUTES
 ak0991x_state *state = (ak0991x_state *)this->state->state;

 {
   sns_std_attr_value_data values[] = {SNS_ATTR};

   sns_std_attr_value_data range1[] = {SNS_ATTR, SNS_ATTR};
   range1[0].has_flt = true;
   range1[0].flt = ak0991x_dev_info_array[device_select].ranges.min;
   range1[1].has_flt = true;
   range1[1].flt = ak0991x_dev_info_array[device_select].ranges.max;
   values[0].has_subtype = true;
   values[0].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
   values[0].subtype.values.arg =
     &((pb_buffer_arg){ .buf = range1, .buf_len = ARR_SIZE(range1) });

   sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RANGES,
       values, ARR_SIZE(values), false);
 }
 {
   sns_std_attr_value_data values[] = {SNS_ATTR};
   values[0].has_sint = true;
   values[0].sint = ak0991x_dev_info_array[device_select].active_current;
   sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_ACTIVE_CURRENT,
       values, ARR_SIZE(values), false);
 }
 {
   uint32_t value_len = 0;
   sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR, SNS_ATTR, SNS_ATTR,
       SNS_ATTR};

#ifdef AK0991X_ENABLE_ALL_DEVICES
   if((state->device_select == AK09915C) || (state->device_select == AK09915D))
   {
     values[0].has_flt = true;
     values[0].flt = ak09915_odr_table[0];
     values[1].has_flt = true;
     values[1].flt = ak09915_odr_table[1];
     values[2].has_flt = true;
     values[2].flt = ak09915_odr_table[2];
     values[3].has_flt = true;
     values[3].flt = ak09915_odr_table[3];
     values[4].has_flt = true;
     values[4].flt = ak09915_odr_table[4];
     value_len = ARR_SIZE(ak09915_odr_table);
   }
   else if(state->device_select == AK09917)
   {
     values[0].has_flt = true;
     values[0].flt = ak09917_odr_table[0];
     values[1].has_flt = true;
     values[1].flt = ak09917_odr_table[1];
     values[2].has_flt = true;
     values[2].flt = ak09917_odr_table[2];
     values[3].has_flt = true;
     values[3].flt = ak09917_odr_table[3];
     values[4].has_flt = true;
     values[4].flt = ak09917_odr_table[4];
     value_len = ARR_SIZE(ak09917_odr_table);
   }
   else // Other parts use same ODR as ak09911
   {
     values[0].has_flt = true;
     values[0].flt = ak09911_odr_table[0];
     values[1].has_flt = true;
     values[1].flt = ak09911_odr_table[1];
     values[2].has_flt = true;
     values[2].flt = ak09911_odr_table[2];
     values[3].has_flt = true;
     values[3].flt = ak09911_odr_table[3];
     value_len = ARR_SIZE(ak09911_odr_table);
   }
#else

#if defined(AK0991X_TARGET_AK09911)
   {
      values[0].has_flt = true;
      values[0].flt = ak09911_odr_table[0];
      values[1].has_flt = true;
      values[1].flt = ak09911_odr_table[1];
      values[2].has_flt = true;
      values[2].flt = ak09911_odr_table[2];
      values[3].has_flt = true;
      values[3].flt = ak09911_odr_table[3];
      value_len = ARR_SIZE(ak09911_odr_table);
    }
#elif defined(AK0991X_TARGET_AK09912)
   {
      values[0].has_flt = true;
      values[0].flt = ak09912_odr_table[0];
      values[1].has_flt = true;
      values[1].flt = ak09912_odr_table[1];
      values[2].has_flt = true;
      values[2].flt = ak09912_odr_table[2];
      values[3].has_flt = true;
      values[3].flt = ak09912_odr_table[3];
      value_len = ARR_SIZE(ak09912_odr_table);
    }
#elif defined(AK0991X_TARGET_AK09913)
   {
      values[0].has_flt = true;
      values[0].flt = ak09913_odr_table[0];
      values[1].has_flt = true;
      values[1].flt = ak09913_odr_table[1];
      values[2].has_flt = true;
      values[2].flt = ak09913_odr_table[2];
      values[3].has_flt = true;
      values[3].flt = ak09913_odr_table[3];
      value_len = ARR_SIZE(ak09913_odr_table);
    }
#elif defined(AK0991X_TARGET_AK09915C) || defined(AK0991X_TARGET_AK09915D)
   {
     values[0].has_flt = true;
     values[0].flt = ak09915_odr_table[0];
     values[1].has_flt = true;
     values[1].flt = ak09915_odr_table[1];
     values[2].has_flt = true;
     values[2].flt = ak09915_odr_table[2];
     values[3].has_flt = true;
     values[3].flt = ak09915_odr_table[3];
     values[4].has_flt = true;
     values[4].flt = ak09915_odr_table[4];
     value_len = ARR_SIZE(ak09915_odr_table);
   }
#elif defined(AK0991X_TARGET_AK09916C) || defined(AK0991X_TARGET_AK09916D)
   {
      values[0].has_flt = true;
      values[0].flt = ak09916_odr_table[0];
      values[1].has_flt = true;
      values[1].flt = ak09916_odr_table[1];
      values[2].has_flt = true;
      values[2].flt = ak09916_odr_table[2];
      values[3].has_flt = true;
      values[3].flt = ak09916_odr_table[3];
      value_len = ARR_SIZE(ak09916_odr_table);
    }
#elif defined(AK0991X_TARGET_AK09917)
   {
     values[0].has_flt = true;
     values[0].flt = ak09917_odr_table[0];
     values[1].has_flt = true;
     values[1].flt = ak09917_odr_table[1];
     values[2].has_flt = true;
     values[2].flt = ak09917_odr_table[2];
     values[3].has_flt = true;
     values[3].flt = ak09917_odr_table[3];
     values[4].has_flt = true;
     values[4].flt = ak09917_odr_table[4];
     value_len = ARR_SIZE(ak09917_odr_table);
   }
#elif defined(AK0991X_TARGET_AK09918)
   {
      values[0].has_flt = true;
      values[0].flt = ak09918_odr_table[0];
      values[1].has_flt = true;
      values[1].flt = ak09918_odr_table[1];
      values[2].has_flt = true;
      values[2].flt = ak09918_odr_table[2];
      values[3].has_flt = true;
      values[3].flt = ak09918_odr_table[3];
      value_len = ARR_SIZE(ak09918_odr_table);
    }
#endif

#endif //  AK0991X_ENABLE_ALL_DEVICES

   sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RATES,
       values, value_len, false);
 }
 {
   sns_std_attr_value_data values[] = {SNS_ATTR};
   values[0].has_flt = true;
   values[0].flt = ak0991x_dev_info_array[device_select].resolutions;
   sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_RESOLUTIONS,
       values, ARR_SIZE(values), false);
 }
 {
   sns_std_attr_value_data values[] = {SNS_ATTR, SNS_ATTR};
   int i;
   for(i = 0; i < 2 && i < ARR_SIZE(ak0991x_dev_info_array[device_select].operating_modes);
       i++)
   {
     char const *op_mode = ak0991x_dev_info_array[device_select].operating_modes[i];
     values[0].str.funcs.encode = pb_encode_string_cb;
     values[0].str.arg = &((pb_buffer_arg)
         { .buf = op_mode, .buf_len = sizeof(op_mode) });
   }
   sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_OP_MODES,
       values, i, false);
 }
 {
   sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
   value.has_boolean = true;
   value.boolean = (state->is_dri ? ak0991x_dev_info_array[device_select].supports_dri : false);
   sns_publish_attribute(
       this, SNS_STD_SENSOR_ATTRID_DRI, &value, 1, false);
 }
 {
   sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
   value.has_boolean = true;
   value.boolean = (state->supports_sync_stream ? ak0991x_dev_info_array[device_select].supports_sync_stream : false);
   sns_publish_attribute(
       this, SNS_STD_SENSOR_ATTRID_STREAM_SYNC, &value, 1, false);
 }
 {
   sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
   value.has_sint = true;
   value.sint = ak0991x_dev_info_array[device_select].max_fifo_depth;
   sns_publish_attribute(
       this, SNS_STD_SENSOR_ATTRID_FIFO_SIZE, &value, 1, false);
 }
 {
   sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
   value.has_sint = true;
   value.sint = ak0991x_dev_info_array[device_select].sleep_current;
   sns_publish_attribute(
       this, SNS_STD_SENSOR_ATTRID_SLEEP_CURRENT, &value, 1, false);
 }
 {
   sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
   value.has_flt = true;
   value.flt = ak0991x_dev_info_array[device_select].resolutions;
   sns_publish_attribute(
       this, SNS_STD_SENSOR_ATTRID_SELECTED_RESOLUTION, &value, 1, false);
 }
 {
   sns_std_attr_value_data values[] = {SNS_ATTR};
   sns_std_attr_value_data range1[] = {SNS_ATTR, SNS_ATTR};
   range1[0].has_flt = true;
   range1[0].flt = ak0991x_dev_info_array[device_select].ranges.min;
   range1[1].has_flt = true;
   range1[1].flt = ak0991x_dev_info_array[device_select].ranges.max;
   values[0].has_subtype = true;
   values[0].subtype.values.funcs.encode = sns_pb_encode_attr_cb;
   values[0].subtype.values.arg =
     &((pb_buffer_arg){ .buf = range1, .buf_len = ARR_SIZE(range1) });
   sns_publish_attribute(
       this, SNS_STD_SENSOR_ATTRID_SELECTED_RANGE, &values[0], ARR_SIZE(values), true);
 }
#else
 UNUSED_VAR(this);
 UNUSED_VAR(device_select);
#endif
}

/**
 * Decodes self test requests.
 *
 * @param[i] request         Encoded input request
 * @param[o] decoded_request Decoded standard request
 * @param[o] test_config     Decoded self test request
 *
 * @return bool True if decoding is successful else false.
 */
static bool ak0991x_get_decoded_self_test_request(
                                                  sns_sensor const *this,
                                                  sns_request const *request,
                                                  sns_std_request *decoded_request,
                                                  sns_physical_sensor_test_config *test_config)
{
  pb_istream_t stream;
  pb_simple_cb_arg arg =
      { .decoded_struct = test_config,
        .fields = sns_physical_sensor_test_config_fields };
  decoded_request->payload = (struct pb_callback_s)
      { .funcs.decode = &pb_decode_simple_cb, .arg = &arg };
  stream = pb_istream_from_buffer(request->request,
                                  request->request_len);
  if(!pb_decode(&stream, sns_std_request_fields, decoded_request))
  {
    AK0991X_PRINT(ERROR, this, "AK0991X decode error");
    return false;
  }
#ifndef AK0991X_ENABLE_DEBUG_MSG
  UNUSED_VAR(this);
#endif
  return true;
}

/**
 * Updates instance state with request info.
 *
 * @param[i] this          Sensor reference
 * @param[i] instance      Sensor Instance reference
 * @param[i] new_request   Encoded request
 *
 * @return Ture if request is valid else false
 */
static bool ak0991x_extract_self_test_info(
                                   sns_sensor const *this,
                                   sns_sensor_instance *instance,
                                   struct sns_request const *new_request)
{
  sns_std_request decoded_request;
  sns_physical_sensor_test_config test_config = sns_physical_sensor_test_config_init_default;
  ak0991x_instance_state *inst_state = (ak0991x_instance_state*)instance->state->state;
  ak0991x_self_test_info *self_test_info;

#ifndef AK0991X_ENABLE_DEBUG_MSG
  UNUSED_VAR(this);
#endif

  self_test_info = &inst_state->mag_info.test_info;

  if(ak0991x_get_decoded_self_test_request(
      this,
  		new_request, &decoded_request, &test_config))
  {
    self_test_info->test_type = test_config.test_type;
    self_test_info->test_client_present = true;
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * Sets instance config to run a self test.
 *
 * @param[i] this      Sensor reference
 * @param[i] instance  Sensor Instance reference
 *
 * @return none
 */
static void ak0991x_set_self_test_inst_config(sns_sensor *this,
                                              sns_sensor_instance *instance)
{
  sns_request config;

  config.message_id = SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG;
  config.request_len = 0;
  config.request = NULL;

  this->instance_api->set_client_config(instance, &config);
}

static sns_rc ak0991x_process_timer_events(sns_sensor *const this)
{
  ak0991x_state    *state = (ak0991x_state *)this->state->state;
  uint8_t          buffer[AK0991X_NUM_READ_DEV_ID];
  sns_rc           rv = SNS_RC_SUCCESS;
  sns_sensor_event *event;

  if(NULL != state->timer_stream)
  {
    event = state->timer_stream->api->peek_input(state->timer_stream);

    while (NULL != event)
    {
      AK0991X_PRINT(LOW, this, "process_timer_events: msg=%u", event->message_id);
      if( event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT )
      {
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t *)event->event,
                                                     event->event_len);
        sns_timer_sensor_event timer_event;

        if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
        {
          AK0991X_PRINT(LOW, this, "process_timer_events: state=%u", 
                        state->power_rail_pend_state);

          if (state->power_rail_pend_state == AK0991X_POWER_RAIL_PENDING_INIT)
          {
            /**-------------------Read and Confirm WHO-AM-I------------------------*/
            rv = ak0991x_get_who_am_i(state->scp_service,
                                      state->com_port_info.port_handle, &buffer[0]);

            if (rv != SNS_RC_SUCCESS)
            {
              AK0991X_PRINT(LOW, this, "Read WHO-AM-I error");
              rv = SNS_RC_INVALID_LIBRARY_STATE;
            }
            else
            {
              state->who_am_i = buffer[1] << 8 | buffer[0];
              AK0991X_PRINT(LOW, this, "Read WHO-AM-I %d",state->who_am_i);

              //Check AKM device ID
              if (buffer[0] == AK0991X_WHOAMI_COMPANY_ID)
              {
#ifdef AK0991X_ENABLE_ALL_DEVICES
                if (buffer[1] == AK09911_WHOAMI_DEV_ID)
                {
                  state->device_select = AK09911;
                }
                else if (buffer[1] == AK09912_WHOAMI_DEV_ID)
                {
                  state->device_select = AK09912;
                }
                else if (buffer[1] == AK09913_WHOAMI_DEV_ID)
                {
                  state->device_select = AK09913;
                }
                else if ((buffer[1] == AK09915_WHOAMI_DEV_ID) && (buffer[3] == AK09915C_SUB_ID))
                {
                  state->device_select = AK09915C;
                }
                else if ((buffer[1] == AK09915_WHOAMI_DEV_ID) && (buffer[3] == AK09915D_SUB_ID))
                {
                  state->device_select = AK09915D;
                }
                else if (buffer[1] == AK09916C_WHOAMI_DEV_ID)
                {
                  state->device_select = AK09916C;
                }
                else if (buffer[1] == AK09917_WHOAMI_DEV_ID)
                {
                  state->device_select = AK09917;
                }
                else if (buffer[1] == AK09916D_WHOAMI_DEV_ID)
                {
                  state->device_select = AK09916D;
                }
                else if (buffer[1] == AK09918_WHOAMI_DEV_ID)
                {
                  state->device_select = AK09918;
                }
                else
                {
                  AK0991X_PRINT(ERROR, this, "Unsupported Sensor");
                  rv = SNS_RC_INVALID_STATE;
                }
#else
#ifdef AK0991X_TARGET_AK09911
                if(buffer[1] == AK09911_WHOAMI_DEV_ID){
                  state->device_select = AK09911;
                }
#endif
#ifdef AK0991X_TARGET_AK09912
                if(buffer[1] == AK09912_WHOAMI_DEV_ID){
                  state->device_select = AK09912;
                }
#endif
#ifdef AK0991X_TARGET_AK09913
                if(buffer[1] == AK09913_WHOAMI_DEV_ID){
                  state->device_select = AK09913;
                }
#endif
#ifdef AK0991X_TARGET_AK09915C
                if((buffer[1] == AK09915_WHOAMI_DEV_ID) && (buffer[3] == AK09915C_SUB_ID)){
                  state->device_select = AK09915C;
                }
#endif
#ifdef AK0991X_TARGET_AK09915D
                if((buffer[1] == AK09915_WHOAMI_DEV_ID) && (buffer[3] == AK09915D_SUB_ID)){
                  state->device_select = AK09915D;
                }
#endif
#ifdef AK0991X_TARGET_AK09916C
                if(buffer[1] == AK09916C_WHOAMI_DEV_ID){
                  state->device_select = AK09916C;
                }
#endif
#ifdef AK0991X_TARGET_AK09916D
                if(buffer[1] == AK09916D_WHOAMI_DEV_ID){
                  state->device_select = AK09916D;
                }
#endif
#ifdef AK0991X_TARGET_AK09917
                if(buffer[1] == AK09917_WHOAMI_DEV_ID){
                  state->device_select = AK09917;
                }
#endif
#ifdef AK0991X_TARGET_AK09918
                if(buffer[1] == AK09918_WHOAMI_DEV_ID){
                  state->device_select = AK09918;
                }
#endif
                else{
                  AK0991X_PRINT(ERROR, this, "Unsupported Sensor");
                  rv = SNS_RC_INVALID_STATE;
                }
#endif
              }
              else
              {
                AK0991X_PRINT(ERROR, this, "Unsupported Sensor");
                rv = SNS_RC_INVALID_STATE;
              }
            }
            if (rv == SNS_RC_SUCCESS)
            {               
              // Set sensitivity adjustment data
              rv = ak0991x_set_sstvt_adj(
                                         state->scp_service,
                                         state->com_port_info.port_handle,
                                         state->device_select,
                                         &state->sstvt_adj[0]);
            }
            if (rv == SNS_RC_SUCCESS)
            {
              // Reset Sensor
              rv = ak0991x_device_sw_reset(NULL,
                                           state->scp_service,
                                           state->com_port_info.port_handle);
            }

            if (rv == SNS_RC_SUCCESS)
            {
              state->hw_is_present = true;
              AK0991X_PRINT(MED, this, "Find Sensor. state->hw_is_present=%d state->device_select=%d",(int)state->hw_is_present, (int)state->device_select);
            }

            /**------------------Power Down and Close COM Port--------------------*/
            state->scp_service->api->
              sns_scp_update_bus_power(state->com_port_info.port_handle,
                                       false);
            state->scp_service->api->
              sns_scp_close(state->com_port_info.port_handle);
            state->scp_service->api->
              sns_scp_deregister_com_port(&state->com_port_info.port_handle);

#ifdef AK0991X_ENABLE_POWER_RAIL
            /**----------------------Turn Power Rail OFF--------------------------*/
            ak0991x_start_power_rail_timer(this,
                                           sns_convert_ns_to_ticks(AK0991X_POWER_RAIL_OFF_TIMEOUT_NS),
                                           AK0991X_POWER_RAIL_PENDING_OFF);
#endif

            if (state->hw_is_present)
            {
              ak0991x_publish_hw_attributes(this,state->device_select);
              ak0991x_sensor_publish_available(this);
              AK0991X_PRINT(MED, this, "AK0991X HW present. device_select: %u",
                                       state->device_select);
            }
            else
            {
              AK0991X_PRINT(MED, this, "AK0991X HW absent");
              #ifdef AK0991X_ENABLE_POWER_RAIL
              state->rail_config.rail_vote = SNS_RAIL_OFF;
              state->pwr_rail_service->api->
              sns_vote_power_rail_update(state->pwr_rail_service, this,
                            &state->rail_config,     NULL);
              state->power_rail_pend_state = AK0991X_POWER_RAIL_PENDING_NONE;
              state->remove_timer_stream = true;
              #endif
            }
          }
          else if (state->power_rail_pend_state == AK0991X_POWER_RAIL_PENDING_SET_CLIENT_REQ)
          {
            sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);

            AK0991X_PRINT(LOW, this, "state = SET_CLIENT_REQ");

            if (NULL != instance)
            {
              AK0991X_PRINT(LOW, this, "state = SET_CLIENT_REQ && instance is Not NULL");
              ak0991x_instance_state *inst_state =
                (ak0991x_instance_state*) instance->state->state;
              ak0991x_reval_instance_config(this, instance);
              if(inst_state->new_self_test_request)
              {
                AK0991X_PRINT(LOW, this, "new_self_test_request = true");
                ak0991x_set_self_test_inst_config(this, instance);
              }
            }
            state->power_rail_pend_state = AK0991X_POWER_RAIL_PENDING_NONE;
            state->remove_timer_stream = true;
          }
          else if (state->power_rail_pend_state == AK0991X_POWER_RAIL_PENDING_OFF)
          {
            AK0991X_PRINT(LOW, this, "state = POWER_RAIL_PENDING_OFF");
            #ifdef AK0991X_ENABLE_POWER_RAIL
            state->rail_config.rail_vote = SNS_RAIL_OFF;
            state->pwr_rail_service->api->
              sns_vote_power_rail_update(state->pwr_rail_service, this,
                                         &state->rail_config,     NULL);
            state->power_rail_pend_state = AK0991X_POWER_RAIL_PENDING_NONE;
            state->remove_timer_stream = true;
            #endif
          }
        }
        else
        {
          AK0991X_PRINT(ERROR, this, "pb_decode error");
          rv = SNS_RC_INVALID_STATE;
        }
      }
      else if( event->message_id != SNS_TIMER_MSGID_SNS_TIMER_SENSOR_REG_EVENT )
      {
        AK0991X_PRINT(HIGH, this, "unexpected timer message");
      }

      event = state->timer_stream->api->get_next_input(state->timer_stream);
    }
  }

  return rv;
}

/** See sns_ak0991x_sensor.h */
sns_sensor_instance *ak0991x_set_client_request(sns_sensor *const this,
                                                struct sns_request const *exist_request,
                                                struct sns_request const *new_request,
                                                bool remove)
{
  sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
  ak0991x_state *state = (ak0991x_state *)this->state->state;
  bool reval_config = false;
#ifdef AK0991X_ENABLE_DUAL_SENSOR
  sns_sensor_uid mag_suid = (state->hardware_id == 0)? (sns_sensor_uid)MAG_SUID1: (sns_sensor_uid)MAG_SUID2;
#else
  sns_sensor_uid mag_suid = (sns_sensor_uid)MAG_SUID1;
#endif

  AK0991X_PRINT(HIGH, this, "ak0991x_set_client_request");

  if (remove)
  {
    if (NULL != instance)
    {
      instance->cb->remove_client_request(instance, exist_request);
      /* Assumption: The FW will call deinit() on the instance before destroying it.
                   Putting all HW resources (sensor HW, COM port, power rail)in
                   low power state happens in Instance deinit().*/

      ak0991x_reval_instance_config(this, instance);
    }
  }
  else
  {
    // 1. If new request then:
    //     a. Power ON rails.
    //     b. Power ON COM port - Instance must handle COM port power.
    //     c. Create new instance.
    //     d. Re-evaluate existing requests and choose appropriate instance config.
    //     e. set_client_config for this instance.
    //     f. Add new_request to list of requests handled by the Instance.
    //     h. Power OFF COM port if not needed- Instance must handle COM port power.
    //     g. Return the Instance.
    // 2. If there is an Instance already present:
    //     a. Add new_request to list of requests handled by the Instance.
    //     b. Remove exist_request from list of requests handled by the Instance.
    //     c. Re-evaluate existing requests and choose appropriate instance config.
    //     d. set_client_config for the instance if not the same as current config.
    //     e. publish the updated config
    //     f. Return the Instance.
    // 3.  If "flush" request:
    //     a. Perform flush on the instance.
    //     b. Return NULL.

    if (NULL == instance)
    {
#ifdef AK0991X_ENABLE_POWER_RAIL
      sns_time on_timestamp;
      sns_time delta;
      state->rail_config.rail_vote = SNS_RAIL_ON_NPM;
      state->pwr_rail_service->api->sns_vote_power_rail_update(
        state->pwr_rail_service,
        this,
        &state->rail_config,
        &on_timestamp);

      delta = sns_get_system_time() - on_timestamp;

      // Use on_timestamp to determine correct Timer value.
      if (delta < sns_convert_ns_to_ticks(AK0991X_OFF_TO_IDLE_MS * 1000 * 1000))
      {
        ak0991x_start_power_rail_timer(this,
                                       sns_convert_ns_to_ticks(
                                       AK0991X_OFF_TO_IDLE_MS * 1000000LL) - delta,
                                       AK0991X_POWER_RAIL_PENDING_SET_CLIENT_REQ);
      }
      else
      {
        // rail is already ON
        AK0991X_PRINT(LOW, this, "rail is already ON");
        state->power_rail_pend_state = AK0991X_POWER_RAIL_PENDING_NONE;
        state->remove_timer_stream = true;
        reval_config = true;
      }
#else
      state->power_rail_pend_state = AK0991X_POWER_RAIL_PENDING_SET_CLIENT_REQ;
      // rail is already ON
      AK0991X_PRINT(LOW, this, "rail is already ON");
      reval_config = true;
#endif
      AK0991X_PRINT(LOW, this, "Creating instance");

      /** create_instance() calls init() for the Sensor Instance */
      instance = this->cb->create_instance(this,
                                           sizeof(ak0991x_instance_state));
    }
    else
    {
      ak0991x_instance_state *inst_state =
        (ak0991x_instance_state *)instance->state->state;

      if(NULL != exist_request
          &&
          NULL != new_request
          &&
          new_request->message_id == SNS_STD_MSGID_SNS_STD_FLUSH_REQ)
      {
        ak0991x_instance_state *inst_state =
          (ak0991x_instance_state *)instance->state->state;

        if(inst_state->mag_info.curr_odr != AK0991X_MAG_ODR_OFF)
        {
          ak0991x_send_flush_config(this, instance);
          return instance;
        }
        else
        {
          return NULL;
        }
      }
      else
      {
        // if the self-test is running,
        // Keep the exist_request and Reject the incoming stream request.
        if (inst_state->new_self_test_request)
        {
          AK0991X_PRINT(LOW, this, "self-test is still running. Keep the exist_request.");
          return NULL;
        }
        else
        {
          reval_config = true;

          /** An existing client is changing request*/
          if ((NULL != exist_request) && (NULL != new_request))
          {
            instance->cb->remove_client_request(instance, exist_request);
          }
          /** A new client sent new_request*/
          else if (NULL != new_request)
          {
            // No-op. new_request will be added to requests list below.
          }
        }
      }
    }

    /** Add the new request to list of client_requests.*/
    if (NULL != instance)
    {
      ak0991x_instance_state *inst_state =
        (ak0991x_instance_state *)instance->state->state;

      // if the self-test is running,
      // Keep the exist_request and Reject the incoming stream request.
      if (inst_state->new_self_test_request)
      {
        AK0991X_PRINT(LOW, this, "self-test is still running. Reject the incoming stream request.");
      }
      else
      {
        AK0991X_PRINT(LOW, this, "Add the new request to list");

        if (NULL != new_request)
        {
          instance->cb->add_client_request(instance, new_request);
          if(new_request->message_id ==
             SNS_PHYSICAL_SENSOR_TEST_MSGID_SNS_PHYSICAL_SENSOR_TEST_CONFIG)
          {
            if(ak0991x_extract_self_test_info(
                this,
            		instance, new_request))
            {
              AK0991X_PRINT(LOW, this, "new_self_test_request = true");
              inst_state->new_self_test_request = true;
              // if the sensor is streaming and there is a client request to run self-test,
              // avoid to reconfigure the settings by self-test.
              if (inst_state->mag_info.curr_odr != AK0991X_MAG_ODR_OFF)
              {
                ak0991x_set_self_test_inst_config(this, instance);
                reval_config = false;
              }
            }
          }
        }

        if (reval_config)
        {
          ak0991x_reval_instance_config(this, instance);

          if(inst_state->new_self_test_request)
          {
            ak0991x_set_self_test_inst_config(this, instance);
          }
        }
      }
    }
  }

  // Sensors are required to call remove_instance when clientless
  if(NULL != instance &&
     NULL == instance->cb->
     get_client_request(instance, &mag_suid, true))
  {
    sns_sensor *sensor;
    AK0991X_PRINT(LOW, this, "Removing instance");
    this->cb->remove_instance(instance);

    for (sensor = this->cb->get_library_sensor(this, true);
         NULL != sensor;
         sensor = this->cb->get_library_sensor(this, false))
    {
      ak0991x_state *sensor_state = (ak0991x_state *)sensor->state->state;

      if (sensor_state->rail_config.rail_vote != SNS_RAIL_OFF)
      {
#ifdef AK0991X_ENABLE_POWER_RAIL
        ak0991x_start_power_rail_timer(this,
                                       sns_convert_ns_to_ticks(AK0991X_POWER_RAIL_OFF_TIMEOUT_NS),
                                       AK0991X_POWER_RAIL_PENDING_OFF);
#endif
      }
    }
  }

  return instance;
}

/** See sns_ak0991x_sensor.h*/
sns_rc ak0991x_sensor_notify_event(sns_sensor *const this)
{
  ak0991x_state    *state = (ak0991x_state *)this->state->state;
  sns_rc           rv = SNS_RC_SUCCESS;

  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service *stream_svc = (sns_stream_service*)
    service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

  AK0991X_PRINT(LOW, this, "ak0991x_sensor_notify_event");

  if (!sns_suid_lookup_complete(&state->suid_lookup_data))
  {
    sns_suid_lookup_handle(this, &state->suid_lookup_data);
    if (state->timer_stream == NULL)
    {
      sns_sensor_uid timer_suid;
      if(sns_suid_lookup_get(&state->suid_lookup_data,"timer", &timer_suid))
      {
        stream_svc->api->create_sensor_stream(stream_svc, this, timer_suid,
                                              &state->timer_stream);
        state->remove_timer_stream = false;
        if(NULL == state->timer_stream)
        {
          AK0991X_PRINT(ERROR, this, __FILENAME__, __LINE__,
                    "Failed to create timer stream");
        }
      }
    }
#ifdef AK0991X_ENABLE_REGISTRY_ACCESS
    sns_sensor_uid reg_suid;
    if (sns_suid_lookup_get(&state->suid_lookup_data, "registry", &reg_suid))
    {
     if (state->reg_data_stream == NULL)
      {
        ak0991x_request_registry(this);
      }
    }
#endif
  }

#ifdef AK0991X_ENABLE_REGISTRY_ACCESS
  rv = ak0991x_process_registry_events(this);
#else
  rv = ak0991x_register_com_port(this);
  if(rv == SNS_RC_SUCCESS)
  {
    ak0991x_register_power_rails(this);
  }
#endif

  if(rv == SNS_RC_SUCCESS)
  {
    rv = ak0991x_process_timer_events(this);
  }

  if(rv == SNS_RC_SUCCESS &&
     !state->hw_is_present &&
     NULL != state->pwr_rail_service &&
     NULL != state->timer_stream &&
     state->power_rail_pend_state == AK0991X_POWER_RAIL_PENDING_NONE)
  {
    sns_time timeticks;
    state->rail_config.rail_vote = SNS_RAIL_ON_NPM;
    state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
                                                             this,
                                                             &state->rail_config,
                                                             &timeticks); /* ignored */

    ak0991x_start_power_rail_timer(this,
                                   sns_convert_ns_to_ticks(
                                   AK0991X_OFF_TO_IDLE_MS * 1000000LL),
                                   AK0991X_POWER_RAIL_PENDING_INIT);

  }

  if(rv == SNS_RC_SUCCESS &&
     NULL != state->timer_stream &&
     state->remove_timer_stream)
  {
    // remove unused timer_data_stream
    sns_service_manager *service_mgr = this->cb->get_service_manager(this);
    sns_stream_service *stream_mgr =
      (sns_stream_service *)service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);

    stream_mgr->api->remove_stream(stream_mgr, state->timer_stream);
    state->timer_stream = NULL;
    AK0991X_PRINT(LOW, this, "remove timer_stream");
  }

  return rv;
}
