/**
 * @file sns_ak0991x_sensor_island.c
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

/**
 * EDIT HISTORY FOR FILE
 *
 * This section contains comments describing changes made to the module.
 * Notice that changes are listed in reverse chronological order.
 *
 *
 * when         who     what, where, why
 * --------     ---     ------------------------------------------------
 * 04/04/17     AKM     Optimize code of MAG_SUID configuration.
 * 04/04/17     AKM     Optimize code of sample_rate and report_rate configuration.
 * 04/04/17     AKM     Fix IRQ configuration.
 * 04/04/17     AKM     Fix ODR attribute configuration.
 *
 **/

#include <string.h>
#include "sns_mem_util.h"
#include "sns_sensor_instance.h"
#include "sns_service_manager.h"
#include "sns_event_service.h"
#include "sns_stream_service.h"
#include "sns_service.h"
#include "sns_sensor_util.h"
#include "sns_math_util.h"
#include "sns_types.h"

#include "sns_ak0991x_sensor.h"
#include "sns_ak0991x_hal.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "sns_pb_util.h"
#include "sns_std.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_suid.pb.h"
#include "sns_timer.pb.h"
#include "sns_diag_service.h"
#include "sns_sync_com_port_service.h"
#include "sns_attribute_util.h"
#include "sns_printf.h"

#define SUID_IS_NULL(suid_ptr) ( sns_memcmp( (suid_ptr),                \
                                             &(sns_sensor_uid){{0}},    \
                                             sizeof(sns_sensor_uid) ) == 0 )

/* device specific information */
float ak09911_odr_table[] =
{AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};
float ak09912_odr_table[] =
{AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};
float ak09913_odr_table[] =
{AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};
float ak09915_odr_table[] =
{AK0991X_ODR_1, AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};
float ak09916_odr_table[] =
{AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};
float ak09917_odr_table[] =
{AK0991X_ODR_1, AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};
float ak09918_odr_table[] =
{AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};

static char *ak09911_ope_mode_table[] = {AK0991X_NORMAL};
static char *ak09912_ope_mode_table[] = {AK0991X_NORMAL};
static char *ak09913_ope_mode_table[] = {AK0991X_NORMAL};
static char *ak09915_ope_mode_table[] = {AK0991X_LOW_POWER, AK0991X_LOW_NOISE};
static char *ak09916_ope_mode_table[] = {AK0991X_NORMAL};
static char *ak09917_ope_mode_table[] = {AK0991X_LOW_POWER, AK0991X_LOW_NOISE};
static char *ak09918_ope_mode_table[] = {AK0991X_NORMAL};

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
};



static sns_sensor_uid const* ak0991x_mag_get_sensor_uid(sns_sensor const *const this)
{
  UNUSED_VAR(this);
  static const sns_sensor_uid sensor_uid = MAG_SUID;

  return &sensor_uid;
}

static void ak0991x_start_power_rail_timer(sns_sensor *const this,
                                           sns_time timeout_ticks,
                                           ak0991x_power_rail_pending_state pwr_rail_pend_state)
{
  ak0991x_state *state = (ak0991x_state *)this->state->state;

  sns_timer_sensor_config req_payload = sns_timer_sensor_config_init_default;
  size_t                  req_len;
  uint8_t                 buffer[20];
  sns_memset(buffer, 0, sizeof(buffer));
  req_payload.is_periodic = false;
  req_payload.start_time = sns_get_system_time();
  req_payload.timeout_period = timeout_ticks;

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
    SNS_PRINTF(ERROR, this, "AK0991x timer req encode error");
  }
}

static void
ak0991x_sensor_publish_available(sns_sensor *const this)
{
  ak0991x_state *state = (ak0991x_state *)this->state->state;

  if( !SUID_IS_NULL(&state->irq_suid) &&
      !SUID_IS_NULL(&state->acp_suid) &&
      !SUID_IS_NULL(&state->timer_suid) &&
      !SUID_IS_NULL(&state->dae_suid)
#if AK0991X_ENABLE_DEPENDENCY
      && !SUID_IS_NULL(&state->reg_suid)
#endif
      )
  {
    if( state->hw_is_present )
    {
      SNS_PRINTF(MED, this, "AK0991x HW Present. Publishing available");
      sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
      value.has_boolean = true;
      value.boolean = true;
      sns_publish_attribute(this, SNS_STD_SENSOR_ATTRID_AVAILABLE,
                            &value, 1, true);
    }
  }
  else
  {
    if( SUID_IS_NULL(&state->irq_suid) )
    {
      SNS_PRINTF(LOW, this, "AK0991x waiting for IRQ SUID" );
    }
    if( SUID_IS_NULL(&state->acp_suid) )
    {
      SNS_PRINTF(LOW, this, "AK0991x waiting for ACP SUID" );
    }
    if( SUID_IS_NULL(&state->timer_suid) )
    {
      SNS_PRINTF(LOW, this, "AK0991x waiting for Timer SUID" );
    }
    if( SUID_IS_NULL(&state->dae_suid) )
    {
      SNS_PRINTF(LOW, this, "AK0991x waiting for DAE SUID" );
    }
#if AK0991X_ENABLE_DEPENDENCY
    if( SUID_IS_NULL(&state->reg_suid) )
    {
      SNS_PRINTF(LOW, this, "AK0991x waiting for Reg SUID" );
    }
#endif
  }
}

static sns_rc ak0991x_process_registry_events(sns_sensor *const this)
{
  ak0991x_state       *state = (ak0991x_state *)this->state->state;
  sns_service_manager *service_mgr = this->cb->get_service_manager(this);
  sns_stream_service  *stream_svc = (sns_stream_service *)
    service_mgr->get_service(service_mgr, SNS_STREAM_SERVICE);
  sns_sync_com_port_service * scp_service = (sns_sync_com_port_service *)
    service_mgr->get_service(service_mgr, SNS_SYNC_COM_PORT_SERVICE);

  sns_time         on_timestamp;
  sns_rc           rv = SNS_RC_SUCCESS;

#if AK0991X_ENABLE_DEPENDENCY

  if (NULL != state->reg_data_stream)
  {
    event = state->reg_data_stream->api->peek_input(state->reg_data_stream);
    
    while (NULL != event)
    {
#endif //AK0991X_ENABLE_DEPENDENCY
#if AK0991X_USE_DEFAULTS

      if (AK0991X_BUS_TYPE == AK0991X_SPI)
      {
        state->com_port_info.com_config.bus_instance = SPI_BUS_INSTANCE;
        state->com_port_info.com_config.bus_type = SNS_BUS_SPI;
        state->com_port_info.com_config.max_bus_speed_KHz = SPI_BUS_MAX_FREQ_KHZ;
        state->com_port_info.com_config.min_bus_speed_KHz = SPI_BUS_MIN_FREQ_KHZ;
        state->com_port_info.com_config.reg_addr_type = SNS_REG_ADDR_8_BIT;
        state->com_port_info.com_config.slave_control = SPI_SLAVE_CONTROL;
      }
      else
      {
        state->com_port_info.com_config.bus_instance = I2C_BUS_INSTANCE;
        state->com_port_info.com_config.bus_type = SNS_BUS_I2C;
        state->com_port_info.com_config.max_bus_speed_KHz = I2C_BUS_FREQ;
        state->com_port_info.com_config.min_bus_speed_KHz = I2C_BUS_FREQ;
        state->com_port_info.com_config.reg_addr_type = SNS_REG_ADDR_8_BIT;
        state->com_port_info.com_config.slave_control = I2C_SLAVE_ADDRESS;
      }

#if AK0991X_ENABLE_DEPENDENCY
      // IRQ settings, it depends on the device.
      state->irq_info.irq_drive_strength = SNS_INTERRUPT_DRIVE_STRENGTH_2_MILLI_AMP;
      state->irq_info.irq_num = IRQ_NUM;
      state->irq_info.interrupt_pull_type = AK0991X_INTERRUPT_PULL_TYPE;
      state->irq_info.interrupt_trigger_type = AK0991X_INTERRUPT_TRIGGER_TYPE;
#endif //AK0991X_ENABLE_DEPENDENCY
#else   //AK0991X_USE_DEFAULTS
      //TODO update to use Registry Sensor data
#endif //AK0991X_USE_DEFAULTS

      /**-----------------Register and Open COM Port-------------------------*/
      if (NULL == state->com_port_info.port_handle)
      {
        scp_service->api->sns_scp_register_com_port(&state->com_port_info.com_config,
                                                    &state->com_port_info.port_handle);

        state->scp_service->api->sns_scp_open(state->com_port_info.port_handle);
      }

      /**---------------------Register Power Rails --------------------------*/
      if (0 != sns_memcmp(&state->timer_suid, &((sns_sensor_uid){{0}}), sizeof(state->timer_suid))
          && NULL == state->pwr_rail_service)
      {
        state->rail_config.rail_vote = SNS_RAIL_OFF;
        state->rail_config.num_of_rails = NUM_OF_RAILS;
        sns_strlcpy(state->rail_config.rails[0].name,
                RAIL_1,
                sizeof(state->rail_config.rails[0].name));
        sns_strlcpy(state->rail_config.rails[1].name,
                RAIL_2,
                sizeof(state->rail_config.rails[1].name));

        state->pwr_rail_service =
          (sns_pwr_rail_service *)service_mgr->get_service(service_mgr,
                                                           SNS_POWER_RAIL_SERVICE);

        state->pwr_rail_service->api->sns_register_power_rails(state->pwr_rail_service,
                                                               &state->rail_config);

        /**---------------------Turn Power Rails ON----------------------------*/
        state->rail_config.rail_vote = SNS_RAIL_ON_NPM;
        state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
                                                                 this,
                                                                 &state->rail_config,
                                                                 &on_timestamp);

        /**-------------Create a Timer stream for Power Rail ON timeout.---------*/
        if (NULL == state->timer_stream)
        {
          stream_svc->api->create_sensor_stream(stream_svc, this, state->timer_suid,
                                                &state->timer_stream);

          if (NULL != state->timer_stream)
          {
            ak0991x_start_power_rail_timer(this,
                                           sns_convert_ns_to_ticks(
                                                                   AK0991X_OFF_TO_IDLE_MS * 1000 * 1000),
                                           AK0991X_POWER_RAIL_PENDING_INIT);
          }
        }

        SNS_PRINTF(LOW, this, "power rail settings finished");
      }

#if AK0991X_ENABLE_DEPENDENCY
      event = state->reg_data_stream->api->get_next_input(state->reg_data_stream);
    }
  }
#endif //AK0991X_ENABLE_DEPENDENCY

  return rv;
}
/**
 * Initialize attributes to their default state.  They may/will be updated
 * within notify_event.
 */
void ak0991x_publish_hw_attributes(sns_sensor *const this,
                                akm_device_type device_select)
{
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
   value.boolean = ak0991x_dev_info_array[device_select].supports_dri;
   sns_publish_attribute(
       this, SNS_STD_SENSOR_ATTRID_DRI, &value, 1, false);
 }
 {
   sns_std_attr_value_data value = sns_std_attr_value_data_init_default;
   value.has_boolean = true;
   value.boolean = ak0991x_dev_info_array[device_select].supports_sync_stream;
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
}
/** See sns_ak0991x_sensor.h*/
sns_rc ak0991x_sensor_notify_event(sns_sensor *const this)
{
  ak0991x_state       *state = (ak0991x_state *)this->state->state;

  uint8_t          buffer[AK0991X_NUM_READ_DEV_ID];
  sns_rc           rv = SNS_RC_SUCCESS;
  sns_sensor_event *event;

  sns_diag_service *diag = state->diag_service;

  ak0991x_process_suid_events(this);
  rv = ak0991x_process_registry_events(this);

  /**----------------------Handle a Timer Sensor event.-------------------*/
  if (NULL != state->timer_stream)
  {
    event = state->timer_stream->api->peek_input(state->timer_stream);

    while (NULL != event)
    {
      if( event->message_id == SNS_TIMER_MSGID_SNS_TIMER_SENSOR_EVENT )
      {
        pb_istream_t stream = pb_istream_from_buffer((pb_byte_t *)event->event,
                                                     event->event_len);
        sns_timer_sensor_event timer_event;

        if (pb_decode(&stream, sns_timer_sensor_event_fields, &timer_event))
        {
          if (state->power_rail_pend_state == AK0991X_POWER_RAIL_PENDING_INIT)
          {
            /**-------------------Read and Confirm WHO-AM-I------------------------*/
            rv = ak0991x_get_who_am_i(state->scp_service,
                                      state->com_port_info.port_handle, &buffer[0]);

            if (rv != SNS_RC_SUCCESS)
            {
              SNS_PRINTF(ERROR, this, "Read WHO-AM-I error");
              return rv;
            }

            state->who_am_i = buffer[1] << 8 | buffer[0];

            //Check AKM device ID
            if (buffer[0] == AK0991X_WHOAMI_COMPANY_ID)
            {
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
                SNS_PRINTF(ERROR, this, "Unsupported Sensor");
                return SNS_RC_FAILED;
              }
            }
            else
            {
              SNS_PRINTF(ERROR, this, "Unsupported Sensor");
              return SNS_RC_FAILED;
            }

#if (!AK0991X_ENABLE_DEPENDENCY)
            // IRQ settings, it depends on the device.
            state->irq_config.interrupt_drive_strength = SNS_INTERRUPT_DRIVE_STRENGTH_2_MILLI_AMP;
            state->irq_config.interrupt_num = IRQ_NUM;

            switch (state->device_select)
            {
              case AK09911:
                state->irq_config.interrupt_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_RISING;
                state->irq_config.is_chip_pin = false;
                state->irq_config.interrupt_pull_type = SNS_INTERRUPT_PULL_TYPE_NO_PULL;
                break;

              case AK09912:
                state->irq_config.interrupt_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_RISING;
                state->irq_config.is_chip_pin = true;
                state->irq_config.interrupt_pull_type = SNS_INTERRUPT_PULL_TYPE_KEEPER;
                break;

              case AK09913:
                state->irq_config.interrupt_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_RISING;
                state->irq_config.is_chip_pin = false;
                state->irq_config.interrupt_pull_type = SNS_INTERRUPT_PULL_TYPE_NO_PULL;
                break;

              case AK09915C:
                state->irq_config.interrupt_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_RISING;
                state->irq_config.is_chip_pin = true;
                state->irq_config.interrupt_pull_type = SNS_INTERRUPT_PULL_TYPE_KEEPER;
                break;

              case AK09915D:
                state->irq_config.interrupt_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_FALLING;
                state->irq_config.is_chip_pin = true;
                state->irq_config.interrupt_pull_type = SNS_INTERRUPT_PULL_TYPE_PULL_UP;
                break;

              case AK09916C:
                state->irq_config.interrupt_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_RISING;
                state->irq_config.is_chip_pin = false;
                state->irq_config.interrupt_pull_type = SNS_INTERRUPT_PULL_TYPE_NO_PULL;
                break;

              case AK09916D:
                state->irq_config.interrupt_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_FALLING;
                state->irq_config.is_chip_pin = true;
                state->irq_config.interrupt_pull_type = SNS_INTERRUPT_PULL_TYPE_PULL_UP;
                break;

              case AK09917:
                state->irq_config.interrupt_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_FALLING;
                state->irq_config.is_chip_pin = true;
                state->irq_config.interrupt_pull_type = SNS_INTERRUPT_PULL_TYPE_PULL_UP;
                break;

              case AK09918:
                state->irq_config.interrupt_trigger_type = SNS_INTERRUPT_TRIGGER_TYPE_RISING;
                state->irq_config.is_chip_pin = false;
                state->irq_config.interrupt_pull_type = SNS_INTERRUPT_PULL_TYPE_NO_PULL;
                break;

              default:
                SNS_PRINTF(ERROR, this, "Unsupported Sensor");
                return SNS_RC_FAILED;
            }
#endif //AK0991X_ENABLE_DEPENDENCY

            // Set sensitivity adjustment data
            rv = ak0991x_set_sstvt_adj(state->scp_service,
                                       state->com_port_info.port_handle,
                                       diag,
                                       state->device_select,
                                       &state->sstvt_adj[0]);

            if (rv != SNS_RC_SUCCESS)
            {
              return rv;
            }

            // Reset Sensor
            rv = ak0991x_device_sw_reset(NULL,
                                         state->scp_service,
                                         state->com_port_info.port_handle,
                                         diag);

            if (rv == SNS_RC_SUCCESS)
            {
              state->hw_is_present = true;
            }

            /**------------------Power Down and Close COM Port--------------------*/
            state->scp_service->api->
              sns_scp_update_bus_power(state->com_port_info.port_handle,
                                       false);
            state->scp_service->api->
              sns_scp_close(state->com_port_info.port_handle);
            state->scp_service->api->
              sns_scp_deregister_com_port(state->com_port_info.port_handle);

            /**----------------------Turn Power Rail OFF--------------------------*/
            state->rail_config.rail_vote = SNS_RAIL_OFF;
            state->pwr_rail_service->api->
              sns_vote_power_rail_update(state->pwr_rail_service, this,
                                         &state->rail_config,     NULL);

            if (state->hw_is_present)
            {
              ak0991x_publish_hw_attributes(this,state->device_select);
              ak0991x_sensor_publish_available(this);
              SNS_PRINTF(HIGH, this, "AK0991X HW present. device_select: %u",
                                       state->device_select);
            }
            else
            {
              rv = SNS_RC_INVALID_STATE;
              SNS_PRINTF(MED, this, "AK0991X HW absent");
              state->rail_config.rail_vote = SNS_RAIL_OFF;
              state->pwr_rail_service->api->sns_vote_power_rail_update(state->pwr_rail_service,
                                                                       this,
                                                                       &state->rail_config,
                                                                       NULL);
            }

            state->power_rail_pend_state = AK0991X_POWER_RAIL_PENDING_NONE;
          }
          else if (state->power_rail_pend_state == AK0991X_POWER_RAIL_PENDING_SET_CLIENT_REQ)
          {
            sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);

            if (NULL != instance)
            {
              ak0991x_reval_instance_config(this, instance);
            }

            state->power_rail_pend_state = AK0991X_POWER_RAIL_PENDING_NONE;
          }
        }
        else
        {
          SNS_PRINTF(ERROR, this, "pb_decode error");
        }
      }
      else if( event->message_id != SNS_TIMER_MSGID_SNS_TIMER_SENSOR_REG_EVENT )
      {
        SNS_PRINTF(ERROR, this, "unexpected timer message");
      }

      event = state->timer_stream->api->get_next_input(state->timer_stream);
    }
  }

  return rv;
}

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
static bool ak0991x_get_decoded_mag_request(sns_sensor const *this,
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
    SNS_PRINTF(ERROR, this, "AK0991X decode error");
    return false;
  }

  return true;
}

static void ak0991x_get_mag_config(sns_sensor *this,
                                   sns_sensor_instance *instance,
                                   float *chosen_sample_rate,
                                   float *chosen_report_rate,
                                   uint32_t *chosen_flush_period,
                                   bool *sensor_client_present)
{
  ak0991x_state *state = (ak0991x_state *)this->state->state;
  sns_diag_service *diag = state->diag_service;

  sns_sensor_uid suid = MAG_SUID;
  sns_request const *request;

  *chosen_report_rate = 0;
  *chosen_sample_rate = 0;
  *chosen_flush_period = 0;
  *sensor_client_present = false;

  /** Parse through existing requests and get fastest sample
   *  rate, report rate, and longest flush period requests. */
  for (request = instance->cb->get_client_request(instance, &suid, true);
       NULL != request;
       request = instance->cb->get_client_request(instance, &suid, false))
  {
    sns_std_request decoded_request;
    sns_std_sensor_config decoded_payload;

    if(request->message_id == SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG)
    {
      if(ak0991x_get_decoded_mag_request(this, request, &decoded_request, &decoded_payload))
      {
        float report_rate;
        uint32_t flush_period;

        *chosen_sample_rate = SNS_MAX(*chosen_sample_rate,
                                      decoded_payload.sample_rate);

        if (decoded_request.has_batching
            &&
            decoded_request.batching.batch_period > 0)
        {
          report_rate = 1000000.f / (float)decoded_request.batching.batch_period;
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
      else // TODO handle self-test request
      {
        uint32_t err = 0;
        sns_rc rv;
        rv = ak0991x_self_test(instance,
                               state->scp_service,
                               state->com_port_info.port_handle,
                               diag,
                               state->device_select,
                               state->sstvt_adj,
                               &err);

        if (rv != SNS_RC_SUCCESS)
        {
          SNS_PRINTF(ERROR, this, "Test failed, err code = %ld", err);
        }
        else
        {
          SNS_PRINTF(HIGH, this, "Test passed");
        }
      }
    }
  }
}

static void ak0991x_set_mag_inst_config(sns_sensor *this,
                                        sns_sensor_instance *instance,
                                        float chosen_report_rate,
                                        float chosen_sample_rate,
                                        uint32_t chosen_flush_period)
{
  sns_ak0991x_mag_req new_client_config;
  sns_request config;

  new_client_config.report_rate = chosen_report_rate;
  new_client_config.sample_rate = chosen_sample_rate;
  new_client_config.flush_period = chosen_flush_period;

  config.message_id = SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG;
  config.request_len = sizeof(sns_ak0991x_mag_req);
  config.request = &new_client_config;

  this->instance_api->set_client_config(instance, &config);
}

void ak0991x_reval_instance_config(sns_sensor *this,
                                   sns_sensor_instance *instance)
{
  /**
   * 1. Get best Mag Config.
   * 2. Decide best Instance Config based on above outputs.
   */
  float chosen_sample_rate = 0;
  float chosen_report_rate = 0;
  uint32_t chosen_flush_period = 0;
  bool m_sensor_client_present;
  UNUSED_VAR(instance);

  ak0991x_get_mag_config(this,
                         instance,
                         &chosen_sample_rate,
                         &chosen_report_rate,
                         &chosen_flush_period,
                         &m_sensor_client_present);

  ak0991x_set_mag_inst_config(this,
                              instance,
                              chosen_report_rate,
                              chosen_sample_rate,
                              chosen_flush_period);
}

/** See sns_ak0991x_sensor.h */
sns_sensor_instance *ak0991x_set_client_request(sns_sensor *const this,
                                                struct sns_request const *exist_request,
                                                struct sns_request const *new_request,
                                                bool remove)
{
  sns_sensor_instance *instance = sns_sensor_util_get_shared_instance(this);
  ak0991x_state *state = (ak0991x_state *)this->state->state;
  sns_time on_timestamp;
  sns_time delta;
  bool reval_config = false;

  if (remove)
  {
    if (NULL != instance)
    {
      instance->cb->remove_client_request(instance, exist_request);
      /* Assumption: The FW will call deinit() on the instance before destroying it.
                   Putting all HW resources (sensor HW, COM port, power rail)in
                   low power state happens in Instance deinit().*/

      ak0991x_reval_instance_config(this, instance);

      sns_sensor *sensor;

      for (sensor = this->cb->get_library_sensor(this, true);
           NULL != sensor;
           sensor = this->cb->get_library_sensor(this, false))
      {
        ak0991x_state *sensor_state = (ak0991x_state *)sensor->state->state;

        if (sensor_state->rail_config.rail_vote != SNS_RAIL_OFF)
        {
          sensor_state->rail_config.rail_vote = SNS_RAIL_OFF;
          sensor_state->pwr_rail_service->api->sns_vote_power_rail_update(
            sensor_state->pwr_rail_service,
            sensor,
            &sensor_state->rail_config,
            NULL);
        }
      }
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
        reval_config = true;
      }

      /** create_instance() calls init() for the Sensor Instance */
      instance = this->cb->create_instance(this,
                                           sizeof(ak0991x_instance_state));
    }
    else
    {
      if (0) // flush_req
      {
        // TODO Flush FIFO samples.
        ak0991x_instance_state *inst_state =
          (ak0991x_instance_state *)instance->state->state;

        if (inst_state->mag_info.curr_odr != AK0991X_MAG_ODR_OFF)
        {
          ak0991x_flush_fifo(instance);
        }

        instance = NULL;
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

    /** Add the new request to list of client_requests.*/
    if (NULL != instance)
    {
      if (NULL != new_request)
      {
        instance->cb->add_client_request(instance, new_request);
      }

      if (reval_config)
      {
        ak0991x_reval_instance_config(this, instance);
      }
    }
  }

  // QC: Sensors are required to call remove_instance when clientless
  if(NULL != instance &&
     NULL == instance->cb->
     get_client_request(instance, &(sns_sensor_uid)MAG_SUID, true))
  {
    this->cb->remove_instance(instance);
  }

  return instance;
}

/** See sns_ak0991x_sensor.h */
void ak0991x_process_suid_events(sns_sensor *const this)
{
  ak0991x_state *state = (ak0991x_state *)this->state->state;

  for (;
       0 != state->fw_stream->api->get_input_cnt(state->fw_stream);
       state->fw_stream->api->get_next_input(state->fw_stream))
  {
    sns_sensor_event *event =
      state->fw_stream->api->peek_input(state->fw_stream);

    if (SNS_SUID_MSGID_SNS_SUID_EVENT == event->message_id)
    {
      pb_istream_t stream = pb_istream_from_buffer((void*)event->event, event->event_len);
      sns_suid_event suid_event = sns_suid_event_init_default;
      pb_buffer_arg data_type_arg = { .buf = NULL, .buf_len = 0 };
      sns_sensor_uid uid_list;
      sns_suid_search suid_search;
      suid_search.suid = &uid_list;
      suid_search.num_of_suids = 0;

      suid_event.data_type.funcs.decode = &pb_decode_string_cb;
      suid_event.data_type.arg = &data_type_arg;
      suid_event.suid.funcs.decode = &pb_decode_suid_event;
      suid_event.suid.arg = &suid_search;

      if(!pb_decode(&stream, sns_suid_event_fields, &suid_event)) {
         SNS_PRINTF(ERROR, this, "SUID Decode failed");
         continue;
       }

      /* if no suids found, ignore the event */
      if(suid_search.num_of_suids == 0)
      {
        continue;
      }

      /* save suid based on incoming data type name */
      if(0 == strncmp(data_type_arg.buf, "data_acquisition_engine", data_type_arg.buf_len))
      {
        state->dae_suid = uid_list;
      }
      else if(0 == strncmp(data_type_arg.buf, "interrupt", data_type_arg.buf_len))
      {
        state->irq_suid = uid_list;
      }
      else if(0 == strncmp(data_type_arg.buf, "timer", data_type_arg.buf_len))
      {
        state->timer_suid = uid_list;
      }
      else if (0 == strncmp(data_type_arg.buf, "async_com_port",
                            data_type_arg.buf_len))
      {
        state->acp_suid = uid_list;
      }
      else if (0 == strncmp(data_type_arg.buf, "registry", data_type_arg.buf_len))
      {
        state->reg_suid = uid_list;
      }
      else
      {
        SNS_PRINTF(ERROR, this, "unexpected datatype_name");
      }
      ak0991x_sensor_publish_available( this );
    }
  }
}

sns_sensor_api ak0991x_mag_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &ak0991x_mag_init,
  .deinit             = &ak0991x_mag_deinit,
  .get_sensor_uid     = &ak0991x_mag_get_sensor_uid,
  .set_client_request = &ak0991x_set_client_request,
  .notify_event       = &ak0991x_sensor_notify_event,
};
