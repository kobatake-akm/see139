#pragma once
/**
 * @file sns_ak0991x_hal.h
 *
 * Hardware Access Layer functions.
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

#include <stdint.h>
#include "sns_sensor.h"
#include "sns_sensor_uid.h"
#include "sns_std.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_ak0991x_sensor_instance.h"

/* Referenced data sheet version
 * AK09911  data sheet version MS1626_E-01
 * AK09912  data sheet version MS1547-E-02
 * AK09913  data sheet version 015007259-E-00
 * AK09915C data sheet version 015006484-E-02
 * AK09915D data sheet version 016009278-E-00
 * AK09916C data sheet version 015007392-E-02
 * AK09916D data sheet version preliminary_E-00
 * AK09917D data sheet version preliminary_E-00-Q
 * AK09918  data sheet version 016014242_E_00
 */

// Enable for test code
#ifndef AK0991X_ENABLE_TEST_CODE
#define AK0991X_ENABLE_TEST_CODE         1
#endif

// Enable when Timer, Registry, ACP dependencies are available
#ifndef AK0991X_ENABLE_DEPENDENCY
#define AK0991X_ENABLE_DEPENDENCY        0
#endif

#ifndef AK0991X_USE_DEFAULTS
#define AK0991X_USE_DEFAULTS             1
#endif

// Set DRI(true) or Polling(false)
#ifndef AK0991X_USE_DRI
#define AK0991X_USE_DRI                  (false)
#endif

// Set Interrupt pull type
// AK09915D, AK09916D and AK09917D should set SNS_INTTERRUPT_PULL_TYPE_PULL_UP
// Other devices should set SNS_INTTERRUPT_PULL_TYPE_KEEPER
#ifndef AK0991X_INTERRUPT_PULL_TYPE
#define AK0991X_INTERRUPT_PULL_TYPE      SNS_INTTERRUPT_PULL_TYPE_PULL_UP
//#define AK0991X_INTERRUPT_PULL_TYPE      SNS_INTTERRUPT_PULL_TYPE_KEEPER
#endif


// Set Interrupt trigger type
// AK09912 and AK09915C should set SNS_INTTERRUPT_TRIGGER_TYPE_RISING
// AK09915D, AK09916D and AK09917D should set SNS_INTTERRUPT_TRIGGER_TYPE_FALLING
#ifndef AK0991X_INTERRUPT_TIRIGGER_TYPE
#define AK0991X_INTERRUPT_TIRIGGER_TYPE  SNS_INTTERRUPT_TRIGGER_TYPE_FALLING
//#define AK0991X_INTERRUPT_TIRIGGER_TYPE  SNS_INTTERRUPT_TRIGGER_TYPE_RISING
#endif


typedef enum
{
  AK0991X_I2C = SNS_BUS_I2C,
  AK0991X_SPI = SNS_BUS_SPI,
} ak0991x_bus_type;
// Set Serial interface
#ifndef AK0991X_BUS_TYPE
#define AK0991X_BUS_TYPE                            SNS_BUS_I2C
#endif

/**
 *  Address registers
 */
#define AKM_AK0991X_REG_WIA1                        (0x00)
#define AKM_AK0991X_REG_WIA2                        (0x01)
#define AKM_AK0991X_REG_INFO1                       (0x02)
#define AKM_AK0991X_REG_INFO2                       (0x03)
#define AKM_AK0991X_REG_ST1                         (0x10)
#define AKM_AK0991X_REG_HXL                         (0x11)
#define AKM_AK0991X_REG_HXH                         (0x12)
#define AKM_AK0991X_REG_HYL                         (0x13)
#define AKM_AK0991X_REG_HYH                         (0x14)
#define AKM_AK0991X_REG_HZL                         (0x15)
#define AKM_AK0991X_REG_HZH                         (0x16)
#define AKM_AK0991X_REG_TMPS                        (0x17)
#define AKM_AK0991X_REG_ST2                         (0x18)
#define AKM_AK0991X_REG_CNTL1                       (0x30)
#define AKM_AK0991X_REG_CNTL2                       (0x31)
#define AKM_AK0991X_REG_CNTL3                       (0x32)

#define AKM_AK0991X_FUSE_ASAX                       (0x60)
#define AKM_AK0991X_FUSE_ASAY                       (0x61)
#define AKM_AK0991X_FUSE_ASAZ                       (0x62)

/** AK0991X number of data types*/
#define AK0991X_NUM_READ_DEV_ID                     4
#define AK0991X_NUM_SENSITIVITY                     3
#define AK0991X_NUM_DATA_ST1_TO_ST2                 9
#define AK0991X_NUM_DATA_HXL_TO_ST2                 8

/** DEVICE ID */
#define AK0991X_WHOAMI_COMPANY_ID                   (0x48) /** Who Am I company ID */
#define AK09911_WHOAMI_DEV_ID                       (0x5)  /** Who Am I device ID */
#define AK09912_WHOAMI_DEV_ID                       (0x4)  /** Who Am I device ID */
#define AK09913_WHOAMI_DEV_ID                       (0x8)  /** Who Am I device ID */
#define AK09915_WHOAMI_DEV_ID                       (0x10) /** Who Am I device ID */
#define AK09916C_WHOAMI_DEV_ID                      (0x9)  /** Who Am I device ID */
#define AK09916D_WHOAMI_DEV_ID                      (0xB)  /** Who Am I device ID */
#define AK09917_WHOAMI_DEV_ID                       (0xD)  /** Who Am I device ID */
#define AK09918_WHOAMI_DEV_ID                       (0xC)  /** Who Am I device ID */

/** DEVICE SUB ID to distinguish AK09915C and AK09915D */
#define AK09915_SUB_ID_IDX                          0x3 /** RSV2 (03h) */
#define AK09915C_SUB_ID                             0x0
#define AK09915D_SUB_ID                             0x2

/** Magnetic sensor overflow bit */
#define AK0991X_HOFL_BIT                            0x8

/** Invalid fifo data bit */
#define AK0991X_INV_FIFO_DATA                       0x4

/** Soft reset */
#define AK0991X_SOFT_RESET                          0x1

/** fifo paramters */
#define AK09911_FIFO_SIZE                           0
#define AK09912_FIFO_SIZE                           0
#define AK09913_FIFO_SIZE                           0
#define AK09915_FIFO_SIZE                           32
#define AK09916_FIFO_SIZE                           0
#define AK09917_FIFO_SIZE                           32
#define AK09918_FIFO_SIZE                           0
#define AK0991X_MAX_FIFO_SIZE                       AK09915_FIFO_SIZE * \
                                                      AK0991X_NUM_DATA_HXL_TO_ST2 + 1

/** FIFO setting */
#define AK0991X_ENABLE_FIFO                         0

/** NSF setting */
#define AK0991X_NSF                                 0

/** Sensor driver setting */
#define AK0991X_SDR                                 0 // AK09915 case, 0: low power mode, 1: low noise mode
                                                      // AK09917 case, 0: low noise mode, 1: low power mode

/** Off to idle time */
#define AK0991X_OFF_TO_IDLE_MS                      100 //ms

/** masurement time */
#define AK09911_TIME_FOR_MEASURE_US                 8500 //us
#define AK09912_TIME_FOR_MEASURE_US                 8500 //us
#define AK09913_TIME_FOR_MEASURE_US                 8200 //us
#define AK09915_TIME_FOR_LOW_POWER_MODE_MEASURE_US  4950 //us
#define AK09915_TIME_FOR_LOW_NOISE_MODE_MEASURE_US  8500 //us
#define AK09916_TIME_FOR_MEASURE_US                 8200 //us
#define AK09917_TIME_FOR_LOW_POWER_MODE_MEASURE_US  4100 //us
#define AK09917_TIME_FOR_LOW_NOISE_MODE_MEASURE_US  8200 //us
#define AK09918_TIME_FOR_MEASURE_US                 8200 //us

/** Limit of factory shipment test */
#define TLIMIT_NO_READ_ID                           0x001
#define TLIMIT_NO_INVALID_ID                        0x002
#define TLIMIT_NO_RESET                             0x003
#define TLIMIT_NO_READ_ASA                          0x004
#define TLIMIT_NO_SET_SELFTEST                      0x005
#define TLIMIT_NO_READ_ST1                          0x006
#define TLIMIT_NO_READ_DATA                         0x007
#define TLIMIT_NO_ASAX                              0x101
#define TLIMIT_LO_ASAX                              1
#define TLIMIT_HI_ASAX                              254

#define TLIMIT_NO_ASAY                              0x102
#define TLIMIT_LO_ASAY                              1
#define TLIMIT_HI_ASAY                              254

#define TLIMIT_NO_ASAZ                              0x103
#define TLIMIT_LO_ASAZ                              1
#define TLIMIT_HI_ASAZ                              254

#define TLIMIT_NO_SLF_RVHX                          0x201
#define TLIMIT_NO_SLF_RVHY                          0x202
#define TLIMIT_NO_SLF_RVHZ                          0x203
#define TLIMIT_NO_SLF_ST2                           0x204
#define TLIMIT_LO_SLF_ST2                           0
#define TLIMIT_HI_SLF_ST2                           0
#define TLIMIT_ST2_MASK                             (0x08)

/*******************************
* AK09918 dependent value
*/
#define TLIMIT_LO_SLF_RVHX_AK09918                  -200
#define TLIMIT_HI_SLF_RVHX_AK09918                  200
#define TLIMIT_LO_SLF_RVHY_AK09918                  -200
#define TLIMIT_HI_SLF_RVHY_AK09918                  200
#define TLIMIT_LO_SLF_RVHZ_AK09918                  -1000
#define TLIMIT_HI_SLF_RVHZ_AK09918                  -150

/*******************************
* AK09917 dependent value
*/
#define TLIMIT_LO_SLF_RVHX_AK09917                  -200
#define TLIMIT_HI_SLF_RVHX_AK09917                  200
#define TLIMIT_LO_SLF_RVHY_AK09917                  -200
#define TLIMIT_HI_SLF_RVHY_AK09917                  200
#define TLIMIT_LO_SLF_RVHZ_AK09917                  -1000
#define TLIMIT_HI_SLF_RVHZ_AK09917                  -150

/*******************************
* AK09916 dependent value
*/
#define TLIMIT_LO_SLF_RVHX_AK09916                  -200
#define TLIMIT_HI_SLF_RVHX_AK09916                  200
#define TLIMIT_LO_SLF_RVHY_AK09916                  -200
#define TLIMIT_HI_SLF_RVHY_AK09916                  200
#define TLIMIT_LO_SLF_RVHZ_AK09916                  -1000
#define TLIMIT_HI_SLF_RVHZ_AK09916                  -200

/*******************************
* AK09915 dependent value
*/
#define TLIMIT_LO_SLF_RVHX_AK09915                  -200
#define TLIMIT_HI_SLF_RVHX_AK09915                  200
#define TLIMIT_LO_SLF_RVHY_AK09915                  -200
#define TLIMIT_HI_SLF_RVHY_AK09915                  200
#define TLIMIT_LO_SLF_RVHZ_AK09915                  -800
#define TLIMIT_HI_SLF_RVHZ_AK09915                  -200

/*******************************
 * AK09913 dependent value
 */
#define TLIMIT_LO_SLF_RVHX_AK09913                  -200
#define TLIMIT_HI_SLF_RVHX_AK09913                  200
#define TLIMIT_LO_SLF_RVHY_AK09913                  -200
#define TLIMIT_HI_SLF_RVHY_AK09913                  200
#define TLIMIT_LO_SLF_RVHZ_AK09913                  -1000
#define TLIMIT_HI_SLF_RVHZ_AK09913                  -200

/*******************************
 * AK09912 dependent value
 */
#define TLIMIT_LO_SLF_RVHX_AK09912                  -200
#define TLIMIT_HI_SLF_RVHX_AK09912                  200
#define TLIMIT_LO_SLF_RVHY_AK09912                  -200
#define TLIMIT_HI_SLF_RVHY_AK09912                  200
#define TLIMIT_LO_SLF_RVHZ_AK09912                  -1600
#define TLIMIT_HI_SLF_RVHZ_AK09912                  -400

/*******************************
 * AK09911 dependent value
 */
#define TLIMIT_LO_SLF_RVHX_AK09911                  -30
#define TLIMIT_HI_SLF_RVHX_AK09911                  30
#define TLIMIT_LO_SLF_RVHY_AK09911                  -30
#define TLIMIT_HI_SLF_RVHY_AK09911                  30
#define TLIMIT_LO_SLF_RVHZ_AK09911                  -400
#define TLIMIT_HI_SLF_RVHZ_AK09911                  -50


/*******************************
 * Log structure definition
 */

typedef struct log_sensor_state_raw_info
{
  /* Size of a single encoded sample */
  size_t encoded_sample_size;
  /* Pointer to log*/
  void *log;
  /* Size of allocated space for log*/
  uint32_t log_size;
  /* Number of actual bytes written*/
  uint32_t bytes_written;
  /* Number of batch samples written*/
  uint32_t sample_cnt;
} log_sensor_state_raw_info;


/******************* Function Declarations ***********************************/

/**
 * Resets the Sensor SW.
 * This function is used in ak0991x drivers flow only.
 * If call from other flow directly for HW reset,
 * should also reset the SW settings like a mag_info.curr_odr.
 *
 * @param[i] port_handle   handle to synch COM port
 * @param[i] scp_service   synch COM port service
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc ak0991x_device_sw_reset(sns_sync_com_port_service * scp_service,
		                       sns_sync_com_port_handle *port_handle);

/**
 * Enable Mag streaming. enables Mag sensor with
 * non-zero desired ODR.
 *
 * @param[i] state         Instance state
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc ak0991x_start_mag_streaming(ak0991x_instance_state *state
);

/**
 * Disable Mag streaming.
 *
 * @param[i] state         Instance state
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc ak0991x_stop_mag_streaming(ak0991x_instance_state *state
);

/**
 * Gets Who-Am-I register for the sensor.
 *
 * @param[i] port_handle   handle to synch COM port
 * @param[i] scp_service   handle to synch COM port service
 *
 * @param[o] buffer        who am I value read from HW
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc ak0991x_get_who_am_i(sns_sync_com_port_service * scp_service,
		                    sns_sync_com_port_handle *port_handle,
                            uint8_t *buffer
);

/**
 * Run a self-test.
 *
 * @param[i] scp_service   handle to synch COM port service
 * @param[i] port_handle   handle to synch COM port
 * @param[i] device_select device ID
 * @param[i] sstvt_adj     Set sensitivity adjustment
 * @param[o] err           error code
 *
 * @return sns_rc
 * SNS_RC_FAILED
 * SNS_RC_SUCCESS
 */
sns_rc ak0991x_self_test(sns_sync_com_port_service * scp_service,
		                 sns_sync_com_port_handle *port_handle,
                         akm_device_type device_select,
                         float *sstvt_adj,
                         uint32_t *err
);

/**
 * Sets sensitivity adjustment for the sensor.
 *
 * @param[i] scp_service   handle to synch COM port service
 * @param[i] port_handle   handle to synch COM port
 * @param[i] device_select device ID
 * @param[o] sstvt_adj     Set sensitivity adjustment
 *
 * @return sns_rc
 * SNS_RC_FAILED
 * SNS_RC_SUCCESS
 */
sns_rc ak0991x_set_sstvt_adj(sns_sync_com_port_service* scp_service,
		                     sns_sync_com_port_handle *port_handle,
                             uint8_t device_select,
                             float *sstvt_adj
);

/**
 * Gets current ODR.
 *
 * @param[i] curr_odr       Current ODR.
 *
 * @return current ODR
 */
float ak0991x_get_mag_odr(ak0991x_mag_odr curr_odr
);


/**
 * Provides sample interval based on current ODR
 *
 * @param[i] curr_odr       Current ODR.
 *
 * @return sampling interval time in ticks
 */
sns_time ak0991x_get_sample_interval(ak0991x_mag_odr curr_odr
);

/**
 * Sets Mag ODR, range and sensitivity.
 *
 * @param[i] scp_service     handle to synch COM port service
 * @param[i] port_handle     handle to synch COM port
 * @param[i] curr_odr        Mag ODR
 * @param[i] select_device   AKM device type
 * @param[i] cur_wmk         current FIFO water mark
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 */
sns_rc ak0991x_set_mag_config(sns_sync_com_port_service *scp_service,
		                      sns_sync_com_port_handle *port_handle,
                              ak0991x_mag_odr curr_odr,
                              akm_device_type device_select,
                              uint16_t cur_wmk
);

/**
 * Process a fifo buffer and extracts mag samples from the buffer
 * and generates event.
 *
 * @param[i] instance              Sensor instance
 * @param[i] first_ts              Timestamp of first sample in fifo
 * @param[i] interval              Sampling interval in time ticks
 * @param[i] fifo                  Buffer containing sample read from HW FIFO
 * @param[i] num_bytes             Number of bytes in fifo buffer
 *
 */
void ak0991x_process_fifo_data_buffer(sns_sensor_instance *instance,
                                      sns_time            first_ts,
                                      sns_time            interval,
                                      uint8_t             *fifo,
                                      size_t              num_bytes
);

/**
 * Sends a FIFO complete event.
 *
 * @param instance   Instance reference
 */
void ak0991x_send_fifo_flush_done(sns_sensor_instance *const instance);

/**
 * Extracts mag samples from the buffer
 * and generates event.
 *
 * @param[i] vector                Com port vector
 * @param[i] user_arg              Pointer to instance passed as user_arg
 */
void ak0991x_process_mag_data_buffer(sns_port_vector *vector,
                                     void *user_arg
)
;

/**
 * Flush mag samples from the buffer
 * and generates event.
 *
 * @param instance                 Sensor Instance
 */
void ak0991x_flush_fifo(sns_sensor_instance *const instance
);

/**
 * Handle an interrupt by reading the Fifo status register and sending out
 * appropriate requests to the asynchronous com port sensor to read the fifo.
 *
 * @param instance                 Sensor Instance
 */
void ak0991x_handle_interrupt_event(sns_sensor_instance *const instance
);

/**
 * Handle an timer by reading the register and sending out
 * appropriate requests to the asynchronous com port sensor to read the data.
 *
 * @return sns_rc
 * SNS_RC_FAILED - COM port failure
 * SNS_RC_SUCCESS
 * @param instance                 Sensor Instance
 */
sns_rc ak0991x_handle_timer_event(sns_sensor_instance *const instance
);

/**
 * Sends config update event for the chosen sample_rate
 *
 * @return sns_rc
 * SNS_RC_FAILED
 * SNS_RC_SUCCESS
 * @param[i] instance    reference to this Instance
 */
sns_rc ak0991x_send_config_event(sns_sensor_instance *const instance
);

/**
 * Submit the Sensor State Raw Log Packet
 *
 * @param[i] diag       Pointer to diag service
 * @param[i] instance   Pointer to sensor instance
 * @param[i] sensor_uid SUID of the sensor
 * @param[i] log_raw_info   Pointer to logging information
 *                      pertaining to the sensor
 */
void ak0991x_log_sensor_state_raw_submit(
  sns_diag_service *diag,
  sns_sensor_instance *const instance,
  struct sns_sensor_uid const *sensor_uid,
  log_sensor_state_raw_info *log_raw_info);

/**
 * Add raw uncalibrated sensor data to Sensor State Raw log
 * packet
 *
 * @param[i] log_raw_info Pointer to logging information
 *                        pertaining to the sensor
 * @param[i] raw_data     Uncalibrated sensor data to be logged
 * @param[i] timestamp    Timestamp of the sensor data
 * @param[i] status       Status of the sensor data
 *
 * * @return sns_rc,
 * SNS_RC_SUCCESS if encoding was succesful
 * SNS_RC_FAILED otherwise
 */
sns_rc ak0991x_log_sensor_state_raw_add(
  log_sensor_state_raw_info *log_raw_info,
  float *raw_data,
  sns_time timestamp,
  sns_std_sensor_sample_status status);

/**
 * Allocate Sensor State Raw Log Packet
 *
 * @param[i] diag       Pointer to diag service
 * @param[i] instance   Pointer to sensor instance
 * @param[i] sensor_uid SUID of the sensor
 * @param[i] log_raw_info   Pointer to raw sensor state logging
 *       information pertaining to the sensor
 */
void ak0991x_log_sensor_state_raw_alloc(
  sns_diag_service *diag,
  sns_sensor_instance *const instance,
  struct sns_sensor_uid const *sensor_uid,
  log_sensor_state_raw_info *log_raw_info);

/**
 * Encode log sensor state raw packet
 *
 * @param[i] log Pointer to log packet information
 * @param[i] log_size Size of log packet information
 * @param[i] encoded_log_size Maximum permitted encoded size of
 *                            the log
 * @param[o] encoded_log Pointer to location where encoded
 *                       log should be generated
 * @param[o] bytes_written Pointer to actual bytes written
 *                       during encode
 *
 * @return sns_rc
 * SNS_RC_SUCCESS if encoding was succesful
 * SNS_RC_FAILED otherwise
 */
sns_rc ak0991x_encode_log_sensor_state_raw(
  void *log, size_t log_size, size_t encoded_log_size, void *encoded_log,
  size_t *bytes_written);

