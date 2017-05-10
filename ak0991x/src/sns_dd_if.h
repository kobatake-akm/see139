#pragma once
/** ======================================================================================
  @file sns_dd_if.h

  @brief Device Drivers API

  Copyright (c) 2016 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.

  $Id: $
  $DateTime: $
  $Change: $
====================================================================================== **/
 
/**
*****************************************************************************************
                               Includes
*****************************************************************************************
*/
#include "sns_com_port.h"

/**
*****************************************************************************************
                               Typedefs
*****************************************************************************************
*/
/**
   @brief Reads data into buffered memory

   Asynchronous call to read data into system buffers.
   Passed into the DD get_data() function.
*/
typedef sns_com_port_status_e (*read_sensor_data)
( sns_dd_handle_s*                  dd_handle,
  sns_com_port_data_vector_s const* data_vectors,
  uint32_t                          num_vectors );

/**
   @brief Sends one accel sample into the system

   Only used by Accel sensor DD. This will send one x/y/z Accel sample. The
   format is full range, 2's complement, 16-bit data. If the raw sensor
   data is more or less than 16-bits, the data should be right/left shifted
   to 16-bit resolution
*/
typedef sns_com_port_status_e (*notify_accel_data)
( sns_dd_handle_s*        dd_handle,
  int16_t                 x,
  int16_t                 y,
  int16_t                 z );


/**
   API to be implemented by DDs
*/

typedef struct
{
  /**
     @brief Gets data from hardware

     This function will start the process of reading data from the sensor
     hardware. This function may complete all read-related activities within
     this function, or it may start a process which is completed after some
     delay.
     If all operations are completed within this function, it should return
     "0" for the delay_read_us parameter. If a delay is needed, it should
     return the delay (in micro-seconds) upon which to complete the read.

     @param [i] data_read_fptr Pointer to function to read sensor data
                               into system buffers.
     @param [i] acc_delay      Accumulated delay so far for this sample read.
                               Will start at 0, and be incremented by 
                               delay_read_us each time the function is called.
     @param [o] delay_us       The sensor framework will delay by this long
                               before calling the function again.
     @param [o] call_again     Set to true if the driver is not yet done
                               reading sensor data. get_data() will be called
                               again after delay_us micro-seconds.
     @param [o] num_samples    Number of samples read in this call to
                               get_data().
  */

  sns_com_port_status_e (*get_data) (
      sns_dd_handle_s*    dd_handle,
      read_sensor_data    data_read_fptr,
      int32_t             acc_delay,
      int32_t*            delay_us,
      bool*               call_again,
      int32_t*            num_samples );
  
  /**
     @brief Parses accel data, and delivers data to the framework.

     @note Only implemented by Accel sensor DDs.

     @param [i] data_ptr    Pointer to the data read by read_sensor_data()
     @param [i] data_size   Size of data read by read_sensor_data()
     @param [i] notify_fptr Called to send each accel sample contained in
                            the data pointer.
  */
  sns_com_port_status_e (*parse_accel_data) (
      sns_dd_handle_s*    dd_handle,
      uint8_t const*      data_ptr,
      uint16_t            data_size,
      notify_accel_data   notify_fptr);

} sns_dd_if_s;
