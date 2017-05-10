/**
 * @file sns_ak0991x_dae.c
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * Copyright (c) 2016-2017 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
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
#define AK09917_WHOAMI_DEV_ID                       (0xD)  /** Who Am I device ID */

/**
*****************************************************************************************
                                  Static Functions
*****************************************************************************************
*/

static sns_com_port_status_e
ak0991x_get_data( sns_dd_handle_s*    dd_handle,
                  read_sensor_data    data_read_fptr,
                  int32_t             acc_delay,
                  int32_t*            delay_us,
                  bool*               call_again,
                  int32_t*            num_samples )
{
  sns_com_port_status_e status;
  uint8_t device_select;
  uint8_t fifo_status;
  uint8_t fifo_mode;
  uint8_t water_mark;

  /* Registers to read into Data Acquisition buffer */
  sns_com_port_data_vector_s data_vectors[2] =
    {
      { .reg_addr = AKM_AK0991X_REG_CNTL2,
        .buf_sz = 1 },
    };

  /* Status registers for use in this function */
  sns_com_port_vector_s state_vector_ptr[] =
    {
      { .reg_addr = AKM_AK0991X_REG_WIA2,
        .buf_sz = 1,
        .buf = &device_select },

      { .reg_addr = AKM_AK0991X_REG_ST1,
        .buf_sz = 1,
        .buf = &fifo_status },
      {
        .reg_addr = AKM_AK0991X_REG_CNTL1,
        .buf_sz = 1,
        .buf = &water_mark },
      {
        .reg_addr = AKM_AK0991X_REG_CNTL2,
        .buf_sz = 1,
        .buf = &fifo_mode },
    };

  /* Read the status registers */
  status = sns_com_port_read_reg_v( dd_handle, state_vector_ptr, 4 );

  if( status == SNS_COM_PORT_STATUS_SUCCESS )
  {
    if((fifo_mode & 0x80) != 0)
    {
      if (device_select == AK09917_WHOAMI_DEV_ID)
      {
        // AK09917D has FNUM bits
        // correspond to how many samples are currently in the FIFO buffer
        *num_samples = ((fifo_status & 0xFC) >> 2) + 1;
      }
      else
      {
        *num_samples = (water_mark & 0x1F) + 1;
      }

      data_vectors[1].reg_addr = AKM_AK0991X_REG_HXL;
      data_vectors[1].buf_sz = (*num_samples) * 8;
    }
    else
    {
      /* FIFO disabled, read from accel registers */
      data_vectors[1].reg_addr = AKM_AK0991X_REG_HXL;
      data_vectors[1].buf_sz = 8;
      *num_samples = 1;
    }
  }
  status = data_read_fptr(dd_handle, data_vectors, ARR_SIZE(data_vectors));
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

