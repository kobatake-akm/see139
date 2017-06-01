#pragma once
/**
 * @file sns_ak0991x_dae_if.h
 *
 * DAE sensor interface
 *
 * Copyright (c) 2016-2017 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

/**
 * Authors(, name)  : Masahiko Fukasawa, Tomoya Nakajima
 * Version          : v2017.06.01
 * Date(MM/DD/YYYY) : 06/01/2017
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
 * 05/11/17     AKM     Add DAE sensor support.
 *
 **/

#include <stdint.h>
#include "sns_sensor_instance.h"
#include "sns_data_stream.h"
#include "sns_stream_service.h"

struct sns_stream_service;
struct sns_data_stream;
struct ak0991x_instance_state;

typedef enum
{
  IDLE,
  STREAM_STARTING,
  STREAMING,
  STREAM_STOPPING,

} ak0991x_dae_if_state;

typedef struct
{
  struct sns_data_stream *stream;
  const char             *nano_hal_vtable_name;
  ak0991x_dae_if_state   state;
  bool                   flushing_hw;
  bool                   flushing_data;
} ak0991x_dae_stream;

typedef struct ak0991x_dae_if_info
{
  ak0991x_dae_stream   mag;
} ak0991x_dae_if_info;

bool ak0991x_dae_if_available(sns_sensor_instance *this);

sns_rc ak0991x_dae_if_init(
  sns_sensor_instance        *const this,
  struct sns_stream_service  *stream_mgr,
  sns_sensor_uid             *dae_suid);

void ak0991x_dae_if_deinit(
  struct ak0991x_instance_state *state,
  struct sns_stream_service     *stream_mgr);

bool ak0991x_dae_if_stop_streaming(sns_sensor_instance *this);

bool ak0991x_dae_if_start_streaming(sns_sensor_instance *this);

bool ak0991x_dae_if_flush_hw(sns_sensor_instance *this);

bool ak0991x_dae_if_flush_samples(sns_sensor_instance *this);

void ak0991x_dae_if_process_events(sns_sensor_instance *this);

