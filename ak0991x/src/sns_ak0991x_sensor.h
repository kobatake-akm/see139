#pragma once
/**
 * @file sns_ak0991x_sensor.h
 *
 * AK0991X Sensor implementation.
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

#include "sns_sensor.h"
#include "sns_data_stream.h"
#include "sns_sensor_uid.h"
#include "sns_pwr_rail_service.h"
#include "sns_ak0991x_hal.h"

#include "sns_ak0991x_sensor_instance.h"
#include "sns_math_util.h"
#include "sns_diag_service.h"

#define MAG_SUID \
  {  \
    .sensor_uid =  \
    {  \
      0x0f, 0x1e, 0x04, 0xec, 0x30, 0xcf, 0x4d, 0xa5,  \
      0xbc, 0x79, 0xd3, 0x09, 0x4d, 0x60, 0xd5, 0xeb  \
    }  \
  }

#if AK0991X_USE_DEFAULTS
/** TODO Using 8996 Platform config as defaults. This is for
 *  test purpose only. All platform specific information will
 *  be available to the Sensor driver via Registry. */
#define RAIL_1                     "/pmic/client/sensor_vddio"
#define RAIL_2                     "/pmic/client/sensor_vdd"
#define IRQ_NUM                    119
#define NUM_OF_RAILS               2
#define I2C_BUS_FREQ               400
#define I2C_SLAVE_ADDRESS          0x0C
#ifdef SSC_TARGET_HEXAGON_CORE_QDSP6_2_0
#define I2C_BUS_INSTANCE           0x06
#define SPI_BUS_INSTANCE           0x02
#else
#define I2C_BUS_INSTANCE           0x03
#define SPI_BUS_INSTANCE           0x01
#endif
#define SPI_BUS_MIN_FREQ_KHZ       0        // 0MHz
#define SPI_BUS_MAX_FREQ_KHZ       33 * 100 // 3MHz
#define SPI_SLAVE_CONTROL          0x0
#endif // AK0991X_USE_DEFAULTS

/** Forward Declaration of Magnetic Sensor API */
sns_sensor_api ak0991x_mag_sensor_api;

/**
 * AK0991X ODR definitions
 */
#define AK0991X_ODR_0              0.0
#define AK0991X_ODR_1              1.0
#define AK0991X_ODR_10             10.0
#define AK0991X_ODR_20             20.0
#define AK0991X_ODR_50             50.0
#define AK0991X_ODR_100            100.0
#define AK0991X_ODR_200            200.0

/**
 * Magnetometer ranges
 */
#define AK09918_MIN_RANGE          -4912 /* Minimum -4912uT */
#define AK09918_MAX_RANGE          4912  /* Maximum  4912uT */
#define AK09917_MIN_RANGE          -4912 /* Minimum -4912uT */
#define AK09917_MAX_RANGE          4912  /* Maximum  4912uT */
#define AK09916_MIN_RANGE          -4912 /* Minimum -4912uT */
#define AK09916_MAX_RANGE          4912  /* Maximum  4912uT */
#define AK09915_MIN_RANGE          -4912 /* Minimum -4912uT */
#define AK09915_MAX_RANGE          4912  /* Maximum  4912uT */
#define AK09913_MIN_RANGE          -4912 /* Minimum -4912uT */
#define AK09913_MAX_RANGE          4912  /* Maximum  4912uT */
#define AK09912_MIN_RANGE          -4912 /* Minimum -4912uT */
#define AK09912_MAX_RANGE          4912  /* Maximum  4912uT */
#define AK09911_MIN_RANGE          -4912 /* Minimum -4912uT */
#define AK09911_MAX_RANGE          4912  /* Maximum  4912uT */

/**
 * Magnetometer resolution
 */
#define AK09918_RESOLUTION         (0.15f) /* uT/LSB */
#define AK09917_RESOLUTION         (0.15f) /* uT/LSB */
#define AK09916_RESOLUTION         (0.15f) /* uT/LSB */
#define AK09915_RESOLUTION         (0.15f) /* uT/LSB */
#define AK09913_RESOLUTION         (0.15f) /* uT/LSB */
#define AK09912_RESOLUTION         (0.15f) /* uT/LSB */
#define AK09911_RESOLUTION         (0.6f)  /* uT/LSB */

/* Power consumption limits */
#define AK09918_LO_PWR             1    /* unit of uA */
#define AK09918_HI_PWR             1100 /* unit of uA @ 100Hz */
#define AK09917_LO_PWR             1    /* unit of uA */
#define AK09917_HI_PWR             900  /* unit of uA @ 100Hz low power (TBD copy from AK09915)*/
#define AK09916_LO_PWR             1    /* unit of uA */
#define AK09916_HI_PWR             1100 /* unit of uA @ 100Hz */
#define AK09915_LO_PWR             3    /* unit of uA */
#define AK09915_HI_PWR             900  /* unit of uA @ 100Hz low power */
#define AK09913_LO_PWR             3    /* unit of uA */
#define AK09913_HI_PWR             1500 /* unit of uA @ 100Hz */
#define AK09912_LO_PWR             3    /* unit of uA */
#define AK09912_HI_PWR             1000 /* unit of uA @ 100Hz */
#define AK09911_LO_PWR             3    /* unit of uA */
#define AK09911_HI_PWR             2400 /* unit of uA @ 100Hz */

/** Supported opertating modes */
#define AK0991X_NORMAL             "NORMAL"
#define AK0991X_LOW_POWER          "LOW_POWER"
#define AK0991X_LOW_NOISE          "LOW_NOISE"

#define AK0991X_NUM_OF_ATTRIBUTES  (21)

/** Power rail timeout States for the AK0991X Sensors.*/
typedef enum
{
  AK0991X_POWER_RAIL_PENDING_NONE,
  AK0991X_POWER_RAIL_PENDING_INIT,
  AK0991X_POWER_RAIL_PENDING_SET_CLIENT_REQ,
} ak0991x_power_rail_pending_state;

/** Interrupt Sensor State. */

typedef struct ak0991x_state
{
  sns_data_stream       *reg_data_stream;
  sns_data_stream       *fw_stream;
  sns_data_stream       *timer_stream;
  sns_sensor_uid        reg_suid;
  sns_sensor_uid        irq_suid;
  sns_sensor_uid        timer_suid;
  sns_sensor_uid        acp_suid; // Asynchronous COM Port
  sns_sensor_uid        my_suid;
  ak0991x_com_port_info com_port_info;
  //  ak0991x_irq_info      irq_info;
  sns_interrupt_req      irq_config;

  sns_pwr_rail_service  *pwr_rail_service;
  sns_rail_config       rail_config;

  bool hw_is_present;
  bool sensor_client_present;

  ak0991x_power_rail_pending_state power_rail_pend_state;

  // parameters which are determined when the connected device is specified.
  akm_device_type device_select; // store the current connected device
  float sstvt_adj[AK0991X_NUM_SENSITIVITY];

  // debug
  uint16_t who_am_i;
  sns_diag_service *diag_service;
  sns_sync_com_port_service *scp_service;
  size_t   encoded_event_len;
} ak0991x_state;

/** Functions shared by all AK0991X Sensors */
/**
 * This function parses the client_request list per Sensor and
 * determines final config for the Sensor Instance.
 *
 * @param[i] this          Sensor reference
 * @param[i] instance      Sensor Instance to config
 *
 * @return none
 */
void ak0991x_reval_instance_config(sns_sensor * this,
                                   sns_sensor_instance *instance);

/**
 * Sends a request to the SUID Sensor to get SUID of a dependent
 * Sensor.
 *
 * @param[i] this          Sensor reference
 * @param[i] data_type     data_type of dependent Sensor
 * @param[i] data_type_len Length of the data_type string
 */
void ak0991x_send_suid_req(sns_sensor * this, char *const data_type,
                           uint32_t data_type_len);

/**
 * Processes events from SUID Sensor.
 *
 * @param[i] this   Sensor reference
 *
 * @return none
 */
void ak0991x_process_suid_events(sns_sensor *const this
);

/**
 * Publishes Sensor attributes.
 *
 * @param[i] this    Sensor Reference
 *
 * @return none
 */
void ak0991x_publish_attributes(sns_sensor *const this,
                                akm_device_type device_select
);

/**
 * notify_event() Sensor API common between all AK0991X Sensors.
 *
 * @param this    Sensor reference
 *
 * @return sns_rc
 */
sns_rc ak0991x_sensor_notify_event(sns_sensor *const this
);

/**
 * set_client_request() Sensor API common between all AK0991X
 * Sensors.
 *
 * @param this            Sensor reference
 * @param exist_request   existing request
 * @param new_request     new request
 * @param remove          true to remove request
 *
 * @return sns_sensor_instance*
 */
sns_sensor_instance *ak0991x_set_client_request(sns_sensor *const this,
                                                struct sns_request const *exist_request,
                                                struct sns_request const *new_request,
                                                bool remove
);

/**
 * Initializes Magnetic Sensor attributes.
 *
 * @param this   Sensor reference
 * @param[i]     device ID
 *
 * @return none
 */
void ak0991x_mag_init_attributes(sns_sensor *const this,
                                 akm_device_type device_select
);



sns_rc ak0991x_mag_init(sns_sensor *const this);
sns_rc ak0991x_mag_deinit(sns_sensor *const this);
