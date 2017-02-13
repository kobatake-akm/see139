#pragma once
/**
 * @file sns_lsm6ds3_sensor_instance.h
 *
 * LSM6DS3 Accel virtual Sensor Instance implementation.
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

#include <stdint.h>
#include "sns_sensor_instance.h"
#include "sns_data_stream.h"
#include "sns_time.h"
#include "sns_com_port_types.h"
#include "sns_sync_com_port.h"
#include "sns_sensor_uid.h"

#include "sns_interrupt.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_motion_accel.pb.h"
#include "sns_physical_sensor_test.pb.h"
#include "sns_diag_service.h"

/** Forward Declaration of Instance API */
sns_sensor_instance_api lsm6ds3_sensor_instance_api;

/** Number of registers to read for debug */
#define LSM6DS3_DEBUG_REGISTERS          (32)

/** Number of entries in reg_map table. */
#define LSM6DS3_REG_MAP_TABLE_SIZE       (11)

/**
 * Accelerometer LSM6DS3_ACC Full Scales in register setting.
 */
typedef enum
{
  LSM6DS3_ACCEL_RANGE_2G   = 0x00,  /*corresponding value in register setting*/
  LSM6DS3_ACCEL_RANGE_4G   = 0x08,
  LSM6DS3_ACCEL_RANGE_8G   = 0x0C,
  LSM6DS3_ACCEL_RANGE_16G   = 0x04,
} lsm6ds3_accel_range;

/**
 * Accelerometer LSM6DS3_ACC sensitivity for each range.
 */
typedef enum
{
  LSM6DS3_ACCEL_SSTVT_2G  = 61,   /* in the unit of micro-g/digit */
  LSM6DS3_ACCEL_SSTVT_4G  = 122,
  LSM6DS3_ACCEL_SSTVT_8G  = 244,
  LSM6DS3_ACCEL_SSTVT_16G = 488,
} lsm6ds3_accel_sstvt;

/**
 * Accelerometer LSM6DS3 ACC filter bandwidth in register setting
 */
typedef enum
{
  LSM6DS3_ACCEL_BW50      = 0x03,  /* 50 Hz bandwidth */
  LSM6DS3_ACCEL_BW100     = 0x02,  /* 100 Hz bandwidth */
  LSM6DS3_ACCEL_BW200     = 0x01,  /* 200 Hz bandwidth */
  LSM6DS3_ACCEL_BW400     = 0x00   /* 400 Hz bandwidth */
} lsm6ds3_accel_bw;

/**
 * Accelerometer LSM6DS3_ACC output data rate in register setting
 */
typedef enum
{
  LSM6DS3_ACCEL_ODR_OFF   = 0x00,  /* power down output data rate */
  LSM6DS3_ACCEL_ODR13     = 0x10,  /* 13 Hz output data rate */
  LSM6DS3_ACCEL_ODR26     = 0x20,  /* 26 Hz output data rate */
  LSM6DS3_ACCEL_ODR52     = 0x30,  /* 52 Hz output data rate */
  LSM6DS3_ACCEL_ODR104    = 0x40,  /* 104 Hz output data rate */
  LSM6DS3_ACCEL_ODR208    = 0x50,  /* 208 Hz output data rate */
  LSM6DS3_ACCEL_ODR416    = 0x60,  /* 416 Hz output data rate */
  LSM6DS3_ACCEL_ODR833    = 0x70,  /* 833 Hz output data rate */
  LSM6DS3_ACCEL_ODR1660   = 0x80,  /* 1.66 kHz output data rate */
  LSM6DS3_ACCEL_ODR3330   = 0x90,  /* 3.33 kHz output data rate */
  LSM6DS3_ACCEL_ODR6660   = 0xA0,  /* 6.66 kHz output data rate */
} lsm6ds3_accel_odr;

/**
 * LSM6DS3 output data rate for gyro, Disabling LPF2,
 * so BW setting is not required
 */
typedef enum
{
  LSM6DS3_GYRO_ODR_OFF = 0x00,       /* power down output data rate */
  LSM6DS3_GYRO_ODR13   = 0x10,       /* 13 Hz output data rate */
  LSM6DS3_GYRO_ODR26   = 0x20,       /* 26 Hz output data rate */
  LSM6DS3_GYRO_ODR52   = 0x30,       /* 52 Hz output data rate */
  LSM6DS3_GYRO_ODR104  = 0x40,       /* 104 Hz output data rate */
  LSM6DS3_GYRO_ODR208  = 0x50,       /* 208 Hz output data rate */
  LSM6DS3_GYRO_ODR416  = 0x60,       /* 416 Hz output data rate */
  LSM6DS3_GYRO_ODR833  = 0x70,       /* 833 Hz output data rate */
  LSM6DS3_GYRO_ODR1660 = 0x80,       /* 1.66 kHz output data rate */
} lsm6ds3_gyro_odr;

/**
 * LSM6DS3 Full Scales in register setting for gyro
 */
typedef enum
{
  STM_LSM6DS3_GYRO_RANGE_245DPS   = 0x00,  /*corresponding value in register setting*/
  STM_LSM6DS3_GYRO_RANGE_500DPS   = 0x04,
  STM_LSM6DS3_GYRO_RANGE_1000DPS  = 0x08,
  STM_LSM6DS3_GYRO_RANGE_2000DPS  = 0x0C,
} lsm6ds3_gyro_range;

typedef float lsm6ds3_gyro_sstvt;

typedef struct lsm6ds3_com_port_info
{
  sns_com_port_config     com_config;
  sns_sync_com_port_handle     *port_handle;

} lsm6ds3_com_port_info;

/**
 * Range attribute.
 */
typedef struct range_attr {
  float min;
  float max;
} range_attr;

typedef enum
{
  LSM6DS3_ACCEL         = 0x1,
  LSM6DS3_GYRO          = 0x2,
  LSM6DS3_MOTION_ACCEL  = 0x4,
  LSM6DS3_SENSOR_TEMP   = 0x8
} lsm6ds3_sensor_type;

/** HW FIFO information */
typedef struct lsm6ds3_fifo_info
{
  /** FIFO enabled or not. Uses lsm6ds3_sensor_type as bit mask
   *  to determine which FIFO Sensors are enabled */
  uint8_t fifo_enabled;

  /** Determines which Sensor data to publish. Uses
   *  lsm6ds3_sensor_type as bit mask. */
  uint8_t publish_sensors;

  /** FIFO and INT enabled or not*/
  /** Uses lsm6ds3_sensor_type as bit mask to determine which
   *  Sensors are enabled */
  uint8_t fifo_int_enabled;

  /** fifo cur rate index */
  lsm6ds3_accel_odr fifo_rate;

  /** fifo desired rate index */
  lsm6ds3_accel_odr desired_fifo_rate;

  /** FIFO watermark levels for accel and gyro*/
  uint16_t cur_wmk;

} lsm6ds3_fifo_info;

typedef struct lsm6ds3_accel_info
{
  lsm6ds3_accel_odr       desired_odr;
  lsm6ds3_accel_odr       curr_odr;
  lsm6ds3_accel_sstvt     sstvt;
  lsm6ds3_accel_range     range;
  lsm6ds3_accel_bw        bw;
  bool                    lp_mode;
  sns_sensor_uid          suid;
  uint8_t                 num_samples_to_discard;
} lsm6ds3_accel_info;

typedef struct lsm6ds3_gyro_info
{
  lsm6ds3_gyro_odr        desired_odr;
  lsm6ds3_gyro_odr        curr_odr;
  lsm6ds3_gyro_sstvt      sstvt;
  lsm6ds3_gyro_range      range;
  bool                    is_in_sleep;
  sns_sensor_uid          suid;
  uint8_t                 num_samples_to_discard;
} lsm6ds3_gyro_info;

typedef struct lsm6ds3_motion_accel_info
{
  uint16_t                desired_wmk;
  lsm6ds3_accel_odr       desired_odr;
  lsm6ds3_accel_odr       curr_odr;
  lsm6ds3_accel_sstvt     sstvt;
  lsm6ds3_accel_range     range;
  lsm6ds3_accel_bw        bw;
  bool                    lp_mode;
  sns_sensor_uid          suid;
  bool                    enable_md_int;
  bool                    md_client_present;
  bool                    md_intr_fired;
  uint8_t                 num_samples_to_discard;
} lsm6ds3_motion_accel_info;

typedef struct lsm6ds3_sensor_temp_info
{
  sns_sensor_uid          suid;
  bool                    timer_is_active;
  uint32_t                report_timer_hz;
} lsm6ds3_sensor_temp_info;

typedef struct lsm6ds3_irq_info
{
  uint16_t                     irq_num;
  sns_interrupt_trigger_type   irq_trigger_type;
  sns_interrupt_drive_strength irq_drive_strength;
  sns_interrupt_pull_type      irq_pull;
  bool                         is_chip_pin;
  bool                         is_registered;
} lsm6ds3_irq_info;

typedef struct lsm6ds3_async_com_port_info
{
  uint32_t                port_handle;
}lsm6ds3_async_com_port_info;

/** Private state. */
typedef struct lsm6ds3_instance_state
{
  /** fifo details*/
  lsm6ds3_fifo_info       fifo_info;

  /** accel HW config details*/
  lsm6ds3_accel_info      accel_info;

  /** gyro HW config details*/
  lsm6ds3_gyro_info       gyro_info;

  /** motion accel HW config details*/
  lsm6ds3_motion_accel_info motion_accel_info;

  /** Sensor Temperature config details. */
  lsm6ds3_sensor_temp_info sensor_temp_info;

  /** Interrupt dependency info. */
  lsm6ds3_irq_info        irq_info;

  /** COM port info */
  lsm6ds3_com_port_info   com_port_info;

  /**--------Async Com Port--------*/
  lsm6ds3_async_com_port_info async_com_port_info;
  sns_time             interrupt_timestamp;

  /** Data streams from dependentcies. */
  sns_data_stream      *interrupt_data_stream;
  sns_data_stream      *timer_data_stream;
  sns_data_stream      *async_com_port_data_stream;

  uint32_t                     client_req_id;
  sns_std_sensor_config        imu_req;
  sns_motion_accel_config      ma_req;

  size_t               encoded_imu_event_len;
  size_t               encoded_sensor_temp_event_len;
  /**----------debug----------*/
  float     a_stream_event[3];
  float     g_stream_event[3];
  float     ma_stream_event[3];
  uint8_t   reg_status[LSM6DS3_DEBUG_REGISTERS];

  sns_diag_service *diag_service;
  bool instance_is_ready_to_configure;

} lsm6ds3_instance_state;

typedef struct odr_reg_map
{
  float              odr;
  lsm6ds3_accel_odr  accel_odr_reg_value;
  lsm6ds3_gyro_odr   gyro_odr_reg_value;
  uint8_t            discard_samples;
} odr_reg_map;

typedef struct sns_lsm6ds3_imu_req
{
  float sample_rate;
  float report_rate;
} sns_lsm6ds3_imu_req;

typedef struct sns_lsm6ds3_ma_req
{
  float sample_rate;
  float report_rate;
  bool enable_motion_detect;
} sns_lsm6ds3_ma_req;

