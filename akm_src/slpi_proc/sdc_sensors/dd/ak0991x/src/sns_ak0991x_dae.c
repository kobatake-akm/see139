/**
 * @file sns_ak0991x_dae.c
 *
 * Copyright (c) 2016-2018 Qualcomm Technologies, Inc.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 * All Rights Reserved.
 *
 * Copyright (c) 2016-2018 Asahi Kasei Microdevices
 * Confidential and Proprietary - Asahi Kasei Microdevices
 **/

/**
*****************************************************************************************
                               Includes
*****************************************************************************************
*/
#include <stdbool.h>
#include "sns_dd_if.h"
#include "sns_macros.h"

/**
*****************************************************************************************
                               Constants/Macros
*****************************************************************************************
*/
#define AKM_AK0991X_REG_WIA2                        (0x01)
#define AKM_AK0991X_REG_ST1                         (0x10)
#define AKM_AK0991X_REG_HXL                         (0x11)
#define AKM_AK0991X_REG_CNTL1                       (0x30)
#define AKM_AK0991X_REG_CNTL2                       (0x31)
#define AK09917_WHOAMI_DEV_ID                       (0x0D)  /** Who Am I device ID */

/**
*****************************************************************************************
                                  Static Functions
*****************************************************************************************
*/

static sns_com_port_status_e
ak0991x_get_data( sns_dd_handle_s*    dd_handle,
                  read_sensor_data    data_read_fptr,
                  notify_interrupt    notify_int_fptr,
                  int32_t             acc_delay,
                  int32_t*            delay_us,
                  bool*               call_again,
                  int32_t*            num_samples )
{
  sns_com_port_status_e status;
  uint8_t device_select;
  uint8_t fifo_status;
  struct akm_ak991x_reg_ctl_s
  {
    uint8_t water_mark;
    uint8_t fifo_mode;
  }  __attribute__((packed));
  struct akm_ak991x_reg_ctl_s reg_ctl;

  /* Status registers for use in this function */
  sns_com_port_vector_s state_vector_ptr[] =
    {
      /* QC: Assume high speed mode is not used, so no need to check for 9917 vs. 9915 */
      { .reg_addr = AKM_AK0991X_REG_WIA2,
        .buf_sz = 1,
        .buf = &device_select },
      { .reg_addr = AKM_AK0991X_REG_ST1,
        .buf_sz = 1,
        .buf = &fifo_status },
      { .reg_addr = AKM_AK0991X_REG_CNTL1,
        .buf_sz = 2,
        .buf = &reg_ctl },
    };

  /* Read the status registers */
  status = sns_com_port_read_reg_v( dd_handle, state_vector_ptr,
                                    ARR_SIZE(state_vector_ptr) );

  if( status == SNS_COM_PORT_STATUS_SUCCESS )
  {
    if((reg_ctl.fifo_mode & 0x80) != 0)
    {
      if (device_select == AK09917_WHOAMI_DEV_ID)
      {
        // AK09917D has FNUM bits
        // correspond to how many samples are currently in the FIFO buffer
        // QC: For non 9917D devices, this will be 0 (assuming high speed is disabled).
        *num_samples = ((fifo_status & 0xFC) >> 2);
      }
      else
      {
        *num_samples = (reg_ctl.water_mark & 0x1F) + 1;
      }
    }
    else
    {
      /* FIFO disabled, just read x/y/z */
      *num_samples = 1;
    }

    if( *num_samples > 0 )
    {
      sns_com_port_data_vector_s data_vectors[] =
      {
        { .mem_addr = &reg_ctl, .reg_addr = 0,
          .buf_sz = 2 },
        { .mem_addr = &fifo_status, .reg_addr = 0,
          .buf_sz = 1 },
        { .reg_addr = AKM_AK0991X_REG_HXL,
          .buf_sz = *num_samples * 8},
      };
      status = data_read_fptr(dd_handle, data_vectors, ARR_SIZE(data_vectors));
    }
  }
  *delay_us = 0;
  *call_again = false;
  return status;
}


/**
*****************************************************************************************
                            Global Function Pointer Table
*****************************************************************************************
*/

sns_dd_if_s ak0991x_hal_table =
  { .get_data = ak0991x_get_data,
    .parse_accel_data = NULL };

sns_dd_if_s ak0991x_hal_table2 =
  { .get_data = ak0991x_get_data,
    .parse_accel_data = NULL };

