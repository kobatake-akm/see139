#pragma once
/**
 * @file sns_ak0991x_sensor_instance.h
 *
 * AK0991X Mag virtual Sensor Instance implementation.
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

#include "sns_com_port_types.h"
#include "sns_data_stream.h"
#include "sns_sensor_instance.h"
#include "sns_sensor_uid.h"
#include "sns_sync_com_port_service.h"
#include "sns_time.h"
#include <stdint.h>

#include "sns_diag_service.h"
#include "sns_interrupt.pb.h"
#include "sns_physical_sensor_test.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_ak0991x_dae_if.h"
#include "sns_async_com_port.pb.h"

/** Forward Declaration of Instance API */
sns_sensor_instance_api ak0991x_sensor_instance_api;

/** Number of entries in reg_map table. */
#define AK0991X_REG_MAP_TABLE_SIZE  (7)

/** AK0991X max number of settings */
#define AK0991X_MAX_NUM_REP_MODE    3
#define AK0991X_MAX_NUM_OPE_MODE    3
#define AK0991X_MAX_NUM_ODR         6

/** Supported AKM Devices */
typedef enum
{
  AK09911,
  AK09912,
  AK09913,
  AK09915C,
  AK09915D,
  AK09916C,
  AK09916D,
  AK09917,
  AK09918,
  SUPPORTED_DEVICES
} akm_device_type;

/**
 * AK0991X output data rate for mag
 */
typedef enum
{
  AK0991X_MAG_ODR_OFF = 0x00,      /* power down output data rate */
  AK0991X_MAG_ODR_SNG_MEAS = 0x01, /* single measurement mode */
  AK0991X_MAG_ODR10 = 0x02,        /* 10 Hz output data rate */
  AK0991X_MAG_ODR20 = 0x04,        /* 20 Hz output data rate */
  AK0991X_MAG_ODR50 = 0x06,        /* 50 Hz output data rate */
  AK0991X_MAG_ODR100 = 0x08,       /* 100 Hz output data rate */
  AK0991X_MAG_ODR200 = 0x0A,       /* 200 Hz output data rate */
  AK0991X_MAG_ODR1 = 0x0C,         /* 1 Hz output data rate */
  AK0991X_MAG_SELFTEST = 0x10,     /* selftest */
  AK0991X_MAG_FUSEROM = 0x1F,      /* FUSE ROM access mode */
} ak0991x_mag_odr;


typedef float ak0991x_mag_sstvt;

typedef struct ak0991x_com_port_info
{
  sns_com_port_config      com_config;
  sns_sync_com_port_handle *port_handle;
} ak0991x_com_port_info;

/**
 * Range attribute.
 */
typedef struct range_attr
{
  float min;
  float max;
} range_attr;

typedef enum
{
  AK0991X_CONFIG_IDLE,              /** not configuring */
  AK0991X_CONFIG_POWERING_DOWN,     /** cleaning up when no clients left */
  AK0991X_CONFIG_STOPPING_STREAM,   /** stream stop initiated, waiting for completion */
  AK0991X_CONFIG_FLUSHING_HW,       /** FIFO flush initiated, waiting for completion */
  AK0991X_CONFIG_UPDATING_HW        /** updating sensor HW, when done goes back to IDLE */
} ak0991x_config_step;

typedef struct ak0991x_mag_info
{
  ak0991x_mag_odr   desired_odr;
  ak0991x_mag_odr   curr_odr;
  ak0991x_mag_sstvt sstvt_adj[3];
  ak0991x_mag_sstvt resolution;
  akm_device_type   device_select;
  uint16_t       cur_wmk;
  uint16_t       max_fifo_size;
  bool           use_dri;
  bool           use_fifo;
  sns_sensor_uid suid;
} ak0991x_mag_info;

typedef struct ak0991x_irq_info
{
  sns_interrupt_req irq_config;
  bool is_registered;
  bool is_ready;
  bool detect_irq_event;
} ak0991x_irq_info;


typedef struct ak0991x_async_com_port_info
{
  uint32_t port_handle;
} ak0991x_async_com_port_info;


/** Private state. */
typedef struct ak0991x_instance_state
{
  /** mag HW config details*/
  ak0991x_mag_info mag_info;

  /** sampling info. */
  uint8_t  num_samples;
  sns_time pre_timestamp;
  bool     this_is_first_data;

  /** Timer info */
  sns_sensor_uid timer_suid;
  bool timer_stream_is_created;

  /** Interrupt dependency info. */
  ak0991x_irq_info irq_info;

  /** COM port info */
  ak0991x_com_port_info com_port_info;

  /**--------Async Com Port--------*/
  ak0991x_async_com_port_info async_com_port_info;
  sns_time interrupt_timestamp;

  sns_async_com_port_config ascp_config;

  /**--------DAE interface---------*/
  ak0991x_dae_if_info       dae_if;
  ak0991x_config_step       config_step;

  /** Data streams from dependentcies. */
  sns_data_stream       *interrupt_data_stream;
  sns_data_stream       *timer_data_stream;
  sns_data_stream       *async_com_port_data_stream;

  uint32_t              client_req_id;
  sns_std_sensor_config mag_req;

  size_t encoded_mag_event_len;
  /**----------debug----------*/
  float  m_stream_event[3];

  sns_diag_service *diag_service;
  sns_sync_com_port_service *scp_service;

  bool fifo_flush_in_progress;

  size_t           log_raw_encoded_size;
} ak0991x_instance_state;

typedef struct odr_reg_map
{
  float           odr;
  ak0991x_mag_odr mag_odr_reg_value;
  uint8_t         discard_samples;
} odr_reg_map;

typedef struct sns_ak0991x_mag_req
{
  float sample_rate;
  float report_rate;
} sns_ak0991x_mag_req;


sns_rc ak0991x_inst_init(sns_sensor_instance *const this,
    sns_sensor_state const *sstate);

sns_rc ak0991x_inst_deinit(sns_sensor_instance *const this);

