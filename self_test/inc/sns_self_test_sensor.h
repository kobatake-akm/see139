#pragma once
/**
 * @file sns_self_test_sensor.h
 *
 * The self test Sensor.
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * $Id:  $
 * $DateTime:  $
 * $Change:  $
 *
 **/

#include "sns_attribute_service.h"
#include "sns_data_stream.h"
#include "sns_diag_service.h"
#include "sns_sensor.h"
#include "sns_sensor_util.h"
#include "sns_std.pb.h"
#include "sns_std_sensor.pb.h"

/** Forward Declaration of Sensor API */
sns_sensor_api sns_self_test_sensor_api;

typedef struct sns_self_test_state
{
  sns_data_stream* sensor_stream;
  sns_data_stream* suid_stream;
  sns_diag_service *diag_service;
  sns_suid_search suid_search[1];
  sns_sensor_uid sensor_suid;
  uint8_t suid_search_count;
  bool test_in_progress;
} sns_self_test_state;


