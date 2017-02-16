#pragma once
/**
 * @file sns_ak0991x_hal.h
 *
 * Hardware Access Layer functions.
 *
 * Copyright (c) 2016 Qualcomm Technologies, Inc.
 * Copyright (c) 2016 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 **/

#include <stdint.h>
#include "sns_sensor.h"
#include "sns_sensor_uid.h"

#include "sns_ak0991x_sensor_instance.h"

// Enable for test code
#ifndef AK0991X_ENABLE_TEST_CODE
#define AK0991X_ENABLE_TEST_CODE      1
#endif

// Enable when Timer, Registry, ACP dependencies are available
#ifndef AK0991X_ENABLE_DEPENDENCY
#define AK0991X_ENABLE_DEPENDENCY     0
#endif

#ifndef AK0991X_USE_DEFAULTS
#define AK0991X_USE_DEFAULTS          1
#endif

// Set DRI or Polling
#ifndef AK0991X_USE_DRI
#define AK0991X_USE_DRI               1
#endif

typedef enum
{
  AK0991X_I2C = SNS_BUS_I2C,
  AK0991X_SPI = SNS_BUS_SPI,
} ak0991x_bus_type;
// Set Serial interface
#ifndef AK0991X_BUS_TYPE
#define AK0991X_BUS_TYPE              AK0991X_SPI
#endif

/**
 *  Address registers
 */
#define AKM_AK0991X_REG_WIA1              (0x00)
#define AKM_AK0991X_REG_WIA2              (0x01)
#define AKM_AK0991X_REG_INFO1             (0x02)
#define AKM_AK0991X_REG_INFO2             (0x03)
#define AKM_AK0991X_REG_ST1               (0x10)
#define AKM_AK0991X_REG_HXL               (0x11)
#define AKM_AK0991X_REG_HXH               (0x12)
#define AKM_AK0991X_REG_HYL               (0x13)
#define AKM_AK0991X_REG_HYH               (0x14)
#define AKM_AK0991X_REG_HZL               (0x15)
#define AKM_AK0991X_REG_HZH               (0x16)
#define AKM_AK0991X_REG_TMPS              (0x17)
#define AKM_AK0991X_REG_ST2               (0x18)
#define AKM_AK0991X_REG_CNTL1             (0x30)
#define AKM_AK0991X_REG_CNTL2             (0x31)
#define AKM_AK0991X_REG_CNTL3             (0x32)

#define AKM_AK0991X_FUSE_ASAX             (0x60)
#define AKM_AK0991X_FUSE_ASAY             (0x61)
#define AKM_AK0991X_FUSE_ASAZ             (0x62)

/** AK0991X number of data types*/
#define AK0991X_NUM_READ_DEV_ID     4
#define AK0991X_NUM_SENSITIVITY     3
#define AK0991X_NUM_DATA_ST1_TO_ST2 9
#define AK0991X_NUM_DATA_HXL_TO_ST2 8

/** DEVICE ID */
#define AK0991X_WHOAMI_COMPANY_ID         (0x48) /** Who Am I company ID */
#define AK09911_WHOAMI_DEV_ID             (0x5)  /** Who Am I device ID */
#define AK09912_WHOAMI_DEV_ID             (0x4)  /** Who Am I device ID */
#define AK09913_WHOAMI_DEV_ID             (0x8)  /** Who Am I device ID */
#define AK09915_WHOAMI_DEV_ID             (0x10) /** Who Am I device ID */
#define AK09916C_WHOAMI_DEV_ID            (0x9)  /** Who Am I device ID */
#define AK09916D_WHOAMI_DEV_ID            (0xB)  /** Who Am I device ID */
#define AK09918_WHOAMI_DEV_ID             (0xC)  /** Who Am I device ID */

/** DEVICE SUB ID to distinguish AK09915C and AK09915D */ 
#define AK09915_SUB_ID_IDX      0x3 /** RSV2 (03h) */
#define AK09915C_SUB_ID         0x0
#define AK09915D_SUB_ID         0x2

/** Magnetic sensor overflow bit */
#define AK0991X_HOFL_BIT        0x8

/** Invalid fifo data bit */
#define AK0991X_INV_FIFO_DATA   0x4

/** Soft reset */
#define AK0991X_SOFT_RESET      0x1

/** fifo paramters */
#define AK09911_FIFO_SIZE         0
#define AK09912_FIFO_SIZE         0
#define AK09913_FIFO_SIZE         0
#define AK09915_FIFO_SIZE         32
#define AK09916_FIFO_SIZE         0
#define AK09918_FIFO_SIZE         0
#define AK0991X_MAX_FIFO_SIZE     AK09915_FIFO_SIZE * AK0991X_NUM_DATA_HXL_TO_ST2 + 1

/** FIFO setting */
#define AK0991X_ENABLE_FIFO     1

/** NSF setting */
#define AK0991X_NSF             0

/** Sensor driver setting */
#define AK0991X_SDR             0 // 0: low power mode, 1: low noise mode

/** Off to idle time */
#define AK0991X_OFF_TO_IDLE_MS      100  //ms

/** masurement time */
#define AK09911_TIME_FOR_MEASURE_US                8500 //us
#define AK09912_TIME_FOR_MEASURE_US                8500 //us
#define AK09913_TIME_FOR_MEASURE_US                8200 //us
#define AK09915_TIME_FOR_LOW_POWER_MODE_MEASURE_US 4950 //us
#define AK09915_TIME_FOR_LOW_NOISE_MODE_MEASURE_US 8500 //us
#define AK09916_TIME_FOR_MEASURE_US                8200 //us
#define AK09918_TIME_FOR_MEASURE_US                8200 //us

/** Limit of factory shipment test */
#define TLIMIT_NO_READ_ID         0x001
#define TLIMIT_NO_INVALID_ID      0x002
#define TLIMIT_NO_RESET           0x003
#define TLIMIT_NO_READ_ASA        0x004
#define TLIMIT_NO_SET_SELFTEST    0x005
#define TLIMIT_NO_READ_ST1        0x006
#define TLIMIT_NO_READ_DATA       0x007
#define TLIMIT_NO_ASAX            0x101
#define TLIMIT_LO_ASAX            1
#define TLIMIT_HI_ASAX            254

#define TLIMIT_NO_ASAY            0x102
#define TLIMIT_LO_ASAY            1
#define TLIMIT_HI_ASAY            254

#define TLIMIT_NO_ASAZ            0x103
#define TLIMIT_LO_ASAZ            1
#define TLIMIT_HI_ASAZ            254

#define TLIMIT_NO_SLF_RVHX        0x201
#define TLIMIT_NO_SLF_RVHY        0x202
#define TLIMIT_NO_SLF_RVHZ        0x203
#define TLIMIT_NO_SLF_ST2         0x204
#define TLIMIT_LO_SLF_ST2         0
#define TLIMIT_HI_SLF_ST2         0
#define TLIMIT_ST2_MASK           (0x08)

/*******************************
* AK09918 dependent value
*/
#define TLIMIT_LO_SLF_RVHX_AK09918  -200
#define TLIMIT_HI_SLF_RVHX_AK09918  200
#define TLIMIT_LO_SLF_RVHY_AK09918  -200
#define TLIMIT_HI_SLF_RVHY_AK09918  200
#define TLIMIT_LO_SLF_RVHZ_AK09918  -1000
#define TLIMIT_HI_SLF_RVHZ_AK09918  -150

/*******************************
* AK09916 dependent value
*/
#define TLIMIT_LO_SLF_RVHX_AK09916  -200
#define TLIMIT_HI_SLF_RVHX_AK09916  200
#define TLIMIT_LO_SLF_RVHY_AK09916  -200
#define TLIMIT_HI_SLF_RVHY_AK09916  200
#define TLIMIT_LO_SLF_RVHZ_AK09916  -1000
#define TLIMIT_HI_SLF_RVHZ_AK09916  -200

/*******************************
* AK09915 dependent value
*/
#define TLIMIT_LO_SLF_RVHX_AK09915  -200
#define TLIMIT_HI_SLF_RVHX_AK09915  200
#define TLIMIT_LO_SLF_RVHY_AK09915  -200
#define TLIMIT_HI_SLF_RVHY_AK09915  200
#define TLIMIT_LO_SLF_RVHZ_AK09915  -800
#define TLIMIT_HI_SLF_RVHZ_AK09915  -200

/*******************************
 * AK09913 dependent value
 */
#define TLIMIT_LO_SLF_RVHX_AK09913  -200
#define TLIMIT_HI_SLF_RVHX_AK09913  200
#define TLIMIT_LO_SLF_RVHY_AK09913  -200
#define TLIMIT_HI_SLF_RVHY_AK09913  200
#define TLIMIT_LO_SLF_RVHZ_AK09913  -1000
#define TLIMIT_HI_SLF_RVHZ_AK09913  -200

/*******************************
 * AK09912 dependent value
 */
#define TLIMIT_LO_SLF_RVHX_AK09912  -200
#define TLIMIT_HI_SLF_RVHX_AK09912  200
#define TLIMIT_LO_SLF_RVHY_AK09912  -200
#define TLIMIT_HI_SLF_RVHY_AK09912  200
#define TLIMIT_LO_SLF_RVHZ_AK09912  -1600
#define TLIMIT_HI_SLF_RVHZ_AK09912  -400

/*******************************
 * AK09911 dependent value
 */
#define TLIMIT_LO_SLF_RVHX_AK09911  -30
#define TLIMIT_HI_SLF_RVHX_AK09911  30
#define TLIMIT_LO_SLF_RVHY_AK09911  -30
#define TLIMIT_HI_SLF_RVHY_AK09911  30
#define TLIMIT_LO_SLF_RVHZ_AK09911  -400
#define TLIMIT_HI_SLF_RVHZ_AK09911  -50


/******************* Function Declarations ***********************************/

/**
 * Resets the Sensor SW.
 *
 * @param[i] port_handle   handle to synch COM port
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc ak0991x_device_sw_reset(sns_sync_com_port_handle *port_handle);

/**
 * Enable Mag streaming. enables Mag sensor with
 * non-zero desired ODR.
 *
 * @param[i] state         Instance state
 *
 * @return none
 */
void ak0991x_start_mag_streaming(ak0991x_instance_state *state);

/**
 * Disable Mag streaming.
 *
 * @param[i] state         Instance state
 *
 * @return none
 */
void ak0991x_stop_mag_streaming(ak0991x_instance_state *state);

/**
 * Gets Who-Am-I register for the sensor.
 *
 * @param[i] state         Instance state
 * @param[o] buffer        who am I value read from HW
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc ak0991x_get_who_am_i(sns_sync_com_port_handle *port_handle,
                            uint8_t *buffer);

/**
 * Run a self-test.
 *
 * @param[i] state         Instance state
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc ak0991x_self_test(sns_sync_com_port_handle *port_handle,
                         akm_device_type device_select,
                         float *sstvt_adj,
                         uint32_t *err);

/**
 * Sets sensitivity adjustment for the sensor.
 *
 * @param[i] state         Instance state
 * @param[i] device_select device ID
 * @param[o] sstvt_adj     Set sensitivity adjustment
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc ak0991x_set_sstvt_adj(sns_sync_com_port_handle *port_handle,
                            uint8_t device_select,
                            float *sstvt_adj);

/**
 * Provides sample interval based on current ODR
 *
 * @param[i] curr_odr       Current ODR.
 *
 * @return sampling interval time in ticks
 */
sns_time ak0991x_get_sample_interval(ak0991x_mag_odr curr_odr);

/**
 * Sets Mag ODR, range and sensitivity.
 *
 * @param[i] port_handle     handle to synch COM port
 * @param[i] curr_odr        Mag ODR
 * @param[i] select_device   AKM device type
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc ak0991x_set_mag_config(sns_sync_com_port_handle *port_handle,
                               ak0991x_mag_odr      curr_odr,
                               akm_device_type      device_select,
                               uint16_t             cur_wmk);

/**
 * Extracts mag samples from the buffer
 * and generates event.
 *
 * @param[i] vector                Com port vector
 * @param[i] user_arg              Pointer to instance passed as user_arg
 */
void ak0991x_process_mag_data_buffer(sns_port_vector       *vector,
                                      void                  *user_arg)
;

/**
 * Flush mag samples from the buffer
 * and generates event.
 *
 * @param instance                 Sensor Instance
 */
void ak0991x_flush_fifo(sns_sensor_instance *const instance);


/**
 * Handle an interrupt by reading the Fifo status register and sending out
 * appropriate requests to the asynchronous com port sensor to read the fifo.
 *
 * @param instance                 Sensor Instance
 */
void ak0991x_handle_interrupt_event(sns_sensor_instance *const instance);

/**
 * Handle an timer by reading the register and sending out
 * appropriate requests to the asynchronous com port sensor to read the data.
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 * @param instance                 Sensor Instance
 */
sns_rc ak0991x_handle_timer_event(sns_sensor_instance *const instance);

/**
 * Sends config update event for the chosen sample_rate
 *
 * @param[i] instance    reference to this Instance
 */
void ak0991x_send_config_event(sns_sensor_instance *const instance);


