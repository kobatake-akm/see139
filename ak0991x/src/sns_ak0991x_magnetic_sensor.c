/**
 * @file sns_ak0991x_magnetic_sensor.c
 *
 * AK0991X Magnetic virtual Sensor implementation.
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * Copyright (c) 2016-2017 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 **/

#include <string.h>
#include "sns_mem_util.h"
#include "sns_types.h"
#include "sns_service_manager.h"
#include "sns_ak0991x_sensor.h"
#include "sns_pb_util.h"

/* device specific information */
static float ak09911_odr_table[] =
{AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};
static float ak09912_odr_table[] =
{AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};
static float ak09913_odr_table[] =
{AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};
static float ak09915_odr_table[] =
{AK0991X_ODR_1, AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100, AK0991X_ODR_200};
static float ak09916_odr_table[] =
{AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};
static float ak09918_odr_table[] =
{AK0991X_ODR_10, AK0991X_ODR_20, AK0991X_ODR_50, AK0991X_ODR_100};

static char *ak09911_ope_mode_table[] = {AK0991X_NORMAL SNS_ATTRIBUTE_STRING_DL};
static char *ak09912_ope_mode_table[] = {AK0991X_NORMAL SNS_ATTRIBUTE_STRING_DL};
static char *ak09913_ope_mode_table[] = {AK0991X_NORMAL SNS_ATTRIBUTE_STRING_DL};
static char *ak09915_ope_mode_table[] =
{AK0991X_LOW_POWER SNS_ATTRIBUTE_STRING_DL, AK0991X_LOW_NOISE SNS_ATTRIBUTE_STRING_DL};
static char *ak09916_ope_mode_table[] = {AK0991X_NORMAL SNS_ATTRIBUTE_STRING_DL};
static char *ak09918_ope_mode_table[] = {AK0991X_NORMAL SNS_ATTRIBUTE_STRING_DL};

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

static const struct ak0991x_dev_info ak0991x_dev_info_array[] = {
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

/**
 * Initialize attributes to their default state.  They may/will be updated
 * within notify_event.
 */
void ak0991x_mag_init_attributes(sns_sensor *const this,
                                 akm_device_type device_select)
{
  ak0991x_state *state = (ak0991x_state *)this->state->state;
  int8_t        i = 0;

  static const char     name[] = "qc_ak0991x";
  static const char     type[] = "mag";
  static const char     vendor[] = "template";//"akm";
  static const uint32_t version = 0x00000100; // major[31:16].minor[15:0]

  static const char proto_files[] =
    "sns_physical_sensor_test.proto" SNS_ATTRIBUTE_STRING_DL "sns_std_sensor.proto";

  static const bool available = true;
  float             data[3] = {0};
  state->encoded_event_len =
    pb_get_encoded_size_sensor_stream_event(data, 3);
  static const sns_std_sensor_stream_type stream_type =
    SNS_STD_SENSOR_STREAM_TYPE_STREAMING;
  static const bool                           is_dynamic = false;
  static const sns_std_sensor_rigid_body_type rigid_body =
    SNS_STD_SENSOR_RIGID_BODY_TYPE_DISPLAY;
  static const float    placement[12] = {0};
  static const uint32_t hardware_id = 0;

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_available,
    .value = (uintptr_t)&available,
    .value_len = sizeof(available)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_name,
    .value = (uintptr_t)&name,
    .value_len = sizeof(name)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_data_type,
    .value = (uintptr_t)&type,
    .value_len = sizeof(type)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_vendor,
    .value = (uintptr_t)&vendor,
    .value_len = sizeof(vendor)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_version,
    .value = (uintptr_t)&version,
    .value_len = sizeof(version)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_rates,
    .value = (uintptr_t)&ak0991x_dev_info_array[device_select].odr,
    .value_len = sizeof(ak0991x_dev_info_array[device_select].odr)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_resolution,
    .value = (uintptr_t)&ak0991x_dev_info_array[device_select].resolutions,
    .value_len = sizeof(ak0991x_dev_info_array[device_select].resolutions)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_fifo_size,
    .value = (uintptr_t)&ak0991x_dev_info_array[device_select].max_fifo_depth,
    .value_len = sizeof(ak0991x_dev_info_array[device_select].max_fifo_depth)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_active_current,
    .value = (uintptr_t)&ak0991x_dev_info_array[device_select].active_current,
    .value_len = sizeof(ak0991x_dev_info_array[device_select].active_current)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_sleep_current,
    .value = (uintptr_t)&ak0991x_dev_info_array[device_select].sleep_current,
    .value_len = sizeof(ak0991x_dev_info_array[device_select].sleep_current)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_ranges,
    .value = (uintptr_t)&ak0991x_dev_info_array[device_select].ranges,
    .value_len = sizeof(ak0991x_dev_info_array[device_select].ranges)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_op_modes,
    .value = (uintptr_t)&ak0991x_dev_info_array[device_select].operating_modes,
    .value_len = ARR_SIZE(ak0991x_dev_info_array[device_select].operating_modes)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_api,
    .value = (uintptr_t)proto_files,
    .value_len = sizeof(proto_files)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_event_size,
    .value = (uintptr_t)&state->encoded_event_len,
    .value_len = sizeof(state->encoded_event_len)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_stream_type,
    .value = (uintptr_t)&stream_type,
    .value_len = sizeof(stream_type)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_is_dynamic,
    .value = (uintptr_t)&is_dynamic,
    .value_len = sizeof(is_dynamic)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_rigid_body,
    .value = (uintptr_t)&rigid_body,
    .value_len = sizeof(rigid_body)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_placement,
    .value = (uintptr_t)&placement,
    .value_len = sizeof(placement)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_hardware_id,
    .value = (uintptr_t)&hardware_id,
    .value_len = sizeof(hardware_id)
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_supports_dri,
    .value = (uintptr_t)&ak0991x_dev_info_array[device_select].supports_dri,
    .value_len = 1
  };

  state->attributes[i++] = (sns_sensor_attribute)
  {
    .name = sns_attr_supports_sync_stream,
    .value = (uintptr_t)&ak0991x_dev_info_array[device_select].supports_sync_stream,
    .value_len = 1
  };
}

/* See sns_sensor::get_sensor_uid */
static sns_sensor_uid const *ak0991x_mag_get_sensor_uid(sns_sensor const *const this)
{
  UNUSED_VAR(this);
  static const sns_sensor_uid sensor_uid = MAG_SUID;

  return &sensor_uid;
}

/* See sns_sensor::init */
static sns_rc ak0991x_mag_init(sns_sensor *const this)
{
  ak0991x_state *state = (ak0991x_state *)this->state->state;

  struct sns_service_manager *smgr = this->cb->get_service_manager(this);
  state->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);

  state->sensor_client_present = false;

  sns_memscpy(&state->my_suid,
              sizeof(state->my_suid),
              ak0991x_mag_get_sensor_uid(this),
              sizeof(sns_sensor_uid));

  ak0991x_send_suid_req(this, "interrupt", 10);
  ak0991x_send_suid_req(this, "async_com_port", 15);
  ak0991x_send_suid_req(this, "timer", 6);
#if AK0991X_ENABLE_DEPENDENCY
  ak0991x_send_suid_req(this, "registry", 9);
#endif //

  return SNS_RC_SUCCESS;
}

/** See sns_sensor.h */
static sns_rc ak0991x_mag_deinit(sns_sensor *const this)
{
  UNUSED_VAR(this);
  return SNS_RC_SUCCESS;
}

/*===========================================================================
  Public Data Definitions
  ===========================================================================*/

sns_sensor_api ak0991x_mag_sensor_api = {
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &ak0991x_mag_init,
  .deinit             = &ak0991x_mag_deinit,
  .get_sensor_uid     = &ak0991x_mag_get_sensor_uid,
  .get_attributes     = &ak0991x_get_attributes,
  .set_client_request = &ak0991x_set_client_request,
  .notify_event       = &ak0991x_sensor_notify_event,
};
