/**
 * @file sns_lsm6ds3_accel_sensor.c
 *
 * LSM6DS3 Accel virtual Sensor implementation.
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include <string.h>
#include "sns_mem_util.h"
#include "sns_types.h"
#include "sns_service_manager.h"
#include "sns_lsm6ds3_sensor.h"
#include "sns_pb_util.h"

/**
 * Initialize attributes to their default state. They may/will
 * be updated within notify_event.
 *
 * @param[i] this    reference to this Sensor
 *
 * @return none
 */
void lsm6ds3_accel_init_attributes(sns_sensor *const this)
{
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;
  int8_t i = 0;

  static const char name[] = "stm_lsm6ds3";
  static const char type[] = "accel";
  static const char vendor[] = "template";
  static const uint32_t version = 0x0100; // major[31:16].minor[15:0]
  static const float odrs[] =
    {
      LSM6DS3_ODR_13,
      LSM6DS3_ODR_26,
      LSM6DS3_ODR_52,
      LSM6DS3_ODR_104,
      LSM6DS3_ODR_208,
      LSM6DS3_ODR_416
    };
  static const float resolutions[] =
    {
      LSM6DS3_ACCEL_RESOLUTION_2G,
      LSM6DS3_ACCEL_RESOLUTION_4G,
      LSM6DS3_ACCEL_RESOLUTION_8G,
      LSM6DS3_ACCEL_RESOLUTION_16G
    }; //mg/LSB
  static const uint32_t max_fifo_depth = LSM6DS3_MAX_FIFO; // samples
  static const uint32_t active_current[3] = {24, 70, 240}; //uA
  static const uint32_t sleep_current = 6; //uA
  static const range_attr ranges[4] =
     {
       {LSM6DS3_ACCEL_RANGE_2G_MIN,  LSM6DS3_ACCEL_RANGE_2G_MAX},
       {LSM6DS3_ACCEL_RANGE_4G_MIN,  LSM6DS3_ACCEL_RANGE_4G_MAX},
       {LSM6DS3_ACCEL_RANGE_8G_MIN,  LSM6DS3_ACCEL_RANGE_8G_MIN},
       {LSM6DS3_ACCEL_RANGE_16G_MIN, LSM6DS3_ACCEL_RANGE_16G_MAX}
     };

  static const char operating_modes[] = LSM6DS3_LPM SNS_ATTRIBUTE_STRING_DL LSM6DS3_NORMAL SNS_ATTRIBUTE_STRING_DL LSM6DS3_HIGH_PERF;
  static const char proto_files[] = "sns_physical_sensor_test.proto" SNS_ATTRIBUTE_STRING_DL "sns_std_sensor.proto";
  static const bool available = true;
  float data[3] = {0};
  state->encoded_event_len =
     pb_get_encoded_size_sensor_stream_event(data, 3);
  static const sns_std_sensor_stream_type stream_type =
    SNS_STD_SENSOR_STREAM_TYPE_STREAMING;
  static const bool is_dynamic = false;
  static const sns_std_sensor_rigid_body_type rigid_body =
     SNS_STD_SENSOR_RIGID_BODY_TYPE_DISPLAY;
  static const float placement[12] = {0};
  static const uint32_t hardware_id = 0;
  static const bool supports_dri = true;
  static const bool supports_sync_stream = false;

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
                             .value = (uintptr_t)&odrs,
                             .value_len = sizeof(odrs)
                           };

  state->attributes[i++] = (sns_sensor_attribute)
                           {
                             .name = sns_attr_resolution,
                             .value = (uintptr_t)&resolutions,
                             .value_len = sizeof(resolutions)
                           };

  state->attributes[i++] = (sns_sensor_attribute)
                           {
                             .name = sns_attr_fifo_size,
                             .value = (uintptr_t)&max_fifo_depth,
                             .value_len = sizeof(max_fifo_depth)
                           };

  state->attributes[i++] = (sns_sensor_attribute)
                           {
                             .name = sns_attr_active_current,
                             .value = (uintptr_t)&active_current,
                             .value_len = sizeof(active_current)
                           };

  state->attributes[i++] = (sns_sensor_attribute)
                           {
                             .name = sns_attr_sleep_current,
                             .value = (uintptr_t)&sleep_current,
                             .value_len = sizeof(sleep_current)
                           };

  state->attributes[i++] = (sns_sensor_attribute)
                           {
                             .name = sns_attr_ranges,
                             .value = (uintptr_t)&ranges,
                             .value_len = sizeof(ranges)
                           };

  state->attributes[i++] = (sns_sensor_attribute)
                           {
                             .name = sns_attr_op_modes,
                             .value = (uintptr_t)operating_modes,
                             .value_len = sizeof(operating_modes)
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
                             .value = (uintptr_t)&supports_dri,
                             .value_len = 1
                           };

    state->attributes[i++] = (sns_sensor_attribute)
                           {
                             .name = sns_attr_supports_sync_stream,
                             .value = (uintptr_t)&supports_sync_stream,
                             .value_len = 1
                           };
}

/* See sns_sensor::get_sensor_uid */
static sns_sensor_uid const* lsm6ds3_accel_get_sensor_uid(sns_sensor const *const this)
{
  //UNUSED_VAR(this);
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;

  struct sns_service_manager *smgr= this->cb->get_service_manager(this);
  state->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);

  sns_diag_service* diag = state->diag_service;
  diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);


  static const sns_sensor_uid sensor_uid = ACCEL_SUID;

  return &sensor_uid;
}

/* See sns_sensor::init */
static sns_rc lsm6ds3_accel_init(sns_sensor *const this)
{
  lsm6ds3_state *state = (lsm6ds3_state*)this->state->state;

  struct sns_service_manager *smgr= this->cb->get_service_manager(this);
  state->diag_service = (sns_diag_service *)
    smgr->get_service(smgr, SNS_DIAG_SERVICE);

  sns_diag_service* diag = state->diag_service;
  diag->api->sensor_printf(diag, this, SNS_ERROR, __FILENAME__, __LINE__,__FUNCTION__);

  state->sensor = LSM6DS3_ACCEL;
  state->sensor_client_present = false;

  sns_memscpy(&state->my_suid,
              sizeof(state->my_suid),
              lsm6ds3_accel_get_sensor_uid(this),
              sizeof(sns_sensor_uid));

  lsm6ds3_send_suid_req(this, "interrupt", 10);
  lsm6ds3_send_suid_req(this, "async_com_port", 15);
  lsm6ds3_send_suid_req(this, "timer", 6);
#if LSM6DS3_ENABLE_DEPENDENCY
  lsm6ds3_send_suid_req(this, "registry", 9);
#endif //

  return SNS_RC_SUCCESS;
}

static  sns_rc lsm6ds3_accel_deinit(sns_sensor *const this)
{
  UNUSED_VAR(this);
  // Turn Sensor OFF.
  // Close COM port.
  // Turn Power Rails OFF.
  // No need to clear lsm6ds3_state because it will get freed anyway.

  return SNS_RC_SUCCESS;
}

/*===========================================================================
  Public Data Definitions
  ===========================================================================*/

sns_sensor_api lsm6ds3_accel_sensor_api =
{
  .struct_len         = sizeof(sns_sensor_api),
  .init               = &lsm6ds3_accel_init,
  .deinit             = &lsm6ds3_accel_deinit,
  .get_sensor_uid     = &lsm6ds3_accel_get_sensor_uid,
  .get_attributes     = &lsm6ds3_get_attributes,
  .set_client_request = &lsm6ds3_set_client_request,
  .notify_event       = &lsm6ds3_sensor_notify_event,
};

