/**
 * @file sns_test_hall.c
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * $Id:
 * //components/dev/ssc.slpi/3.0/kaushiks.ssc.slpi.3.0.napali_8996_19/sensors/test/src/sns_test_hall.c#1
 * $ $DateTime: 2017/07/24 07:13:15 $ $Change: 13914774 $
 *
 **/
#include "pb_decode.h"
#include "pb_encode.h"
#include "sns_hall.pb.h"
#include "sns_mem_util.h"
#include "sns_pb_util.h"
#include "sns_test_hall.h"
#include "sns_test_sensor.h"
#include "sns_types.h"
#include "sns_printf.h"

/**
 * See sns_test_hall.h
 */
void sns_test_create_hall_request(const sns_sensor *sensor,
                                     void *payload,
                                     const pb_field_t **payload_fields,
                                     uint32_t *message_id,
                                     sns_std_request *std_req)
{
  UNUSED_VAR(sensor);
  UNUSED_VAR(std_req);
  UNUSED_VAR(payload);
  UNUSED_VAR(payload_fields);

  //todo:this will be changed to a custom type
  *message_id = SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG;
}

/**
 * See sns_test_hall.h
 */
void sns_test_hall_process_event(const sns_sensor *sensor,
                                     void* event,
                                     uint32_t event_len,
                                     void* test_data,
                                     uint32_t message_id,
                                     sns_time timestamp)
{
  switch (message_id)
  {
    case SNS_HALL_MSGID_SNS_HALL_EVENT:
    {


      sns_hall_event hall_event = sns_hall_event_init_default;

      pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event,
          event_len);

      if (!pb_decode(&stream, sns_hall_event_fields, &hall_event))
      {
        SNS_PRINTF(ERROR, sensor, "pb_decode() failed for stream_event");
      }

      sns_test_std_sensor_data* tdata = (sns_test_std_sensor_data*)test_data;

      if (tdata->num_events > 0)
      {
        float delta = timestamp - tdata->last_ts;
        /* calculate running average of delta timestamps */
        tdata->avg_delta = (tdata->avg_delta * tdata->num_events + delta)
               / (tdata->num_events + 1);
      }
      tdata->last_ts = timestamp;
      tdata->num_events++;

      SNS_PRINTF(MED, sensor, "sensor sample #%04u: ts=%u ticks,"
          " avg_delta=%u ticks",
          tdata->num_events, (uint32_t)timestamp,
          (uint32_t)tdata->avg_delta);

    }
    break;
    case SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_PHYSICAL_CONFIG_EVENT:
    {
      SNS_PRINTF(MED, sensor, "test_hall: sns_sensor_stream_msgid_sns_std_sensor_config_event");

      sns_std_sensor_physical_config_event phy_sensor_config =
          sns_std_sensor_physical_config_event_init_default;

      pb_istream_t stream = pb_istream_from_buffer((pb_byte_t*)event,
                                                   event_len);

      if (!pb_decode(&stream, sns_std_sensor_physical_config_event_fields, &phy_sensor_config))
      {
        SNS_PRINTF(ERROR, sensor, "pb_decode() failed for stream_event");
      }
      else
      {

         SNS_PRINTF(MED, sensor, "min_range = [%d/1000] max_range = [%d/1000] active_current = %d",
                                 (int32_t)(phy_sensor_config.range[0]*1000), 
				 (int32_t)(phy_sensor_config.range[1]*1000),
                                 phy_sensor_config.active_current);
      }
    }

    break;
    default:
      SNS_PRINTF(ERROR, sensor, "incorrect message_id %u", message_id);
  }
}
