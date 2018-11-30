#pragma once
/**
 * @file sns_ak0991x_ver.h
 *
 * Driver version
 *
 * Copyright (c) 2017-2018 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2017-2018 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 **/

/**
 * EDIT HISTORY FOR FILE
 *
 * This section contains comments describing changes made to the module.
 * Notice that changes are listed in reverse chronological order.
 *
 *
 * when         version    who              what
 * --------     --------   ----------       ---------------------------------
 * 11/29/18                AKM              Use timestamp_type for detecting DRI or Flush in DAE mode.
 * 11/29/18     020033     AKM              Modified for flush requests on DAE.
 * 11/26/18     020032     AKM              Modified for AK09917 RevA/RevB bug and clock error procedure for DAE mode.
 *                                          Modified average interval calc for DRI on non DAE mode. Now it is same as DAE mode.
 * 11/19/18     020031     AKM              Regardless the DRDY status, set UNRELIABLE flag when XYZ data is all 0.
 * 11/16/18     020030     AKM              Modified for MAG039/MAG040 for DRI+FIFO mode
 * 11/15/18     020029     AKM              Modified not to use previous data. Added Re-check num_samples when DRDY when Polling ODR=0
 * 11/14/18     020028     AKM              Modified for current time < timestamp
 * 11/14/18     020027     AKM              Confirmed the 5Hz issue. Update the version.
 * 11/13/18                AKM              Test modification for AK09918C 5Hz issue
 * 11/08/18     020026     AKM/Qualcomm     AKM: Modified to use UNRELIABLE for the first data if data is not ready.
 *                                          Qualcomm fixed a handful of changes, in this version, made by AKM.
 * 10/29/18     020025     AKM              Debugged for very first data for Polling+DAE
 * 10/19/18     020024     AKM              Debugged for I2C mode at ak0991x_device_sw_reset
 * 10/19/18     020023     AKM              Merged Qualcomm 20021 and AKM 20023
 * 10/17/18                AKM              Implemented HB timer for Polling when DAE enabled.
 * 10/15/18     020022     AKM              Debugged for I2C+Polling+DAE mode.
 * 10/12/18     020021     AKM              HB timer perform while 100Hz dummy meas. Pause HB timer while self test.
 * 10/24/18     020023     Qualcomm         Do not add a cal reset request to the instance
 * 10/19/18     020022     Qualcomm         Retry enter i3c mode if fails
 * 10/16/18     020021     Qualcomm         Fixed setting of max batch, for max-batch = True and flush-only = True use case 
 * 10/12/18     020020     Qualcomm/AKM     Fixed setting of flush period for FlushOnly = True use case.
 * 09/26/18     020019     AKM              Merged Qualcomm 010017 and AKM 020018 and modified for WM
 * 09/25/18     020018     AKM              Modified for MAG221 with AK09917D RevA parts.
 * 09/17/18     020017     AKM              Modified for MAG023/MAG025/MAG027 timing error on DAE mode
 * 09/20/18     020017     Qualcomm         Fixed setting of FIFO watermark.
 *                                          Previous incorrect logic was causing garbage values to be reported
 *                                          in the extra data samples above the 25 max for AK09917
 * 09/10/18     020016     AKM              Merged Qualcomm's 020015 and AKM's 020015.
 * 09/06/18     020015     Qualcomm         Changed when to enter i3c
 * 09/03/18     020015     AKM              Modified for Dual Sensor on DAE
 * 08/03/18     020014     AKM              Debugged for the Klocwork P1 errors(#03603537)
 * 07/28/18     020013     Qualcomm         Send CFG Event for new request even no change 
 * 07/24/18     020013     AKM/Qualcomm     Enabled device mode as default and cleaned related code.
 * 07/12/18     020012     AKM/Qualcomm     Fixed compile error when DAE is enabled
 * 07/03/18     020011     AKM              Debugged when the registry access is disabled.
 * 06/28/18                Qualcomm         Retry 5 times if sw reset fails
 * 07/02/18     020010     AKM/Qualcomm     Modified for the upgrated LLVM
 * 06/27/18                AKM/Qualcomm     Merged Qualcomm's modification and AKM's 020009
 * 06/24/18     020009     AKM              Removed macros for SEE_LIET mode
 * 06/22/18                AKM              Refactor for the device_mode.
 * 06/20/18     020008     AKM/Qualcomm     Sometimes, mag is taking scp path with number of samples 32. 
 *                                          Changed local buffer size according to physical senosor
 *                                          Debugged mul-function when the AK0991X_FORCE_MAX_ODR_50HZ is set.
 *                                          Fixed watermark calculation for max batch
 *                                          Fixed flush only request handling
 *                                          Fixed wrong report rate calculation
 *                                          Removed odr < 100 limit for polling mode
 * 06/19/18     020007     AKM              Debugged compile error when dual sensor is enabled. Modified to ignore the irq time when WM!=num_samples.
 * 06/19/18     020006     AKM              Integrated the deltas between AKM's driver versions 77 and 80.
 *                                          AKM version 010080: Applied the device_mode modification from ver1.00.62F.
 *                                          AKM version 010079: Modified for device mode sensor and timestamp for DRI+FIFO mode.
 *                                          AKM version 010078: Modified contains mode check in HW self-test
 * 06/13/18     020005     Qualcomm         Integrated the deltas between AKM's driver versions 71 and 77.
 *                                          AKM version 010077:  Add continuous mode check in HW self-test
 *                                          AKM/QCOM version 010076: Added AK0991X_ENABLE_REG_FAC_CAL macro for reading 3x3 factory calibration parameter from registry
 *                                          AKM version 010075: Changed to read registry value for rail_vote when registry access is enabled
 *                                          AKM version 010074: Added sns_suid_lookup_deinit.
 *                                          AKM version 010073: Remove DC-Lib related code.
 *                                          AKM version 010072: Modified to set RAIL_ON_NPM by registry.
 * 05/16/18     020004     Qualcomm         Integrated versions 010063 to 010071
 * 05/10/18     020004     Qualcomm         Integrated versions 010059 to 010062
 * 04/20/18     020003     Qualcomm         Integrated versions 010057 and 010058
 * 04/20/18                Qualcomm         Fixed COM selftest; Fixed batching via DAE sensor
 * 04/20/18                Qualcomm         Added DAE WM to Config event
 * 03/16/18     020002     Qualcomm         DAE availability is discovered at boot
 * 03/07/18     020001     Qualcomm         Added I3C support
 * 03/02/18     020000     Qualcomm         Re-enabled streaming via DAE
 * 04/26/18     010071     AKM              Fixed error when ENABLE_DC is defined.
 * 04/20/18                AKM              Reduced parameters for SEE-Lite mode.
 * 04/20/18     010070     AKM              Modified for initialize use_dri,use_fifo,nsf and sdr.
 * 04/16/18     010069     AKM              Modified macro definition for non SEE_LITE mode. Removed AK0991X_ENABLE_ALL_DEVICES definition.
 * 04/13/18     010068     AKM              Fixed for SEE_LITE_MODE.
 * 04/12/18     010067     AKM              Added AK0991X_ENABLE_REG_WRITE_ACCESS macro
 * 04/12/18     010066     AKM              Modified for AK09918
 * 04/12/18                AKM              Modified for 0 gap detection on MAG-024/025/026 with S4S mode.
 * 04/12/18     010065     AKM              Modified for SEE_LITE_MODE again.
 * 04/12/18     010064     AKM              Modified for SEE_LITE_MODE.
 * 04/09/18     010063     AKM              Modified DAE settings.
 * 04/09/18                AKM              Fixed AK09915C/D read samples.
 * 04/11/18     010062     AKM              Protect several code for dual SI parameter by macro.
 * 04/03/18                AKM              Implement for handling dual SI parameter using device_mode_sensor.
 * 03/30/18     010061     AKM              Modified HB timer setting for crash when system is busy.
 * 03/21/18     010060     AKM              Fixed S4S to care timestamp.
 * 03/21/18                AKM              Added registry item version number. Case# 03380028
 * 03/12/18     010059     AKM              Fixed S4S settings.
 * 03/20/18                AKM              Debugged for HB timer on DRI mode.
 * 03/12/18                AKM              Fixed S4S settings.
 * 03/12/18                AKM              Change to read AKM's DC-parameter from registry file. Case# 03380124
 * 02/27/18     010058     AKM              Added unregister heart beat stream in ak0991x_stop_mag_streaming
 * 02/16/18     010057     AKM              Remove some comments after checked in.
 * 02/14/18     010056     Qualcomm/AKM     Qualcomm checked in version.
 * 02/14/18                Qualcomm/AKM     Fixed mag stops streams when system is busy and heartbeat timer expires
 * 02/14/18                AKM              Modified for Case# 03325581. Change to keep report rate equals max sample rate while streaming is running.
 * 02/14/18     010055     AKM              Change to remove timer_data_stream on each registering heart beat timer.
 * 01/24/18     010054     AKM              Modified for Case# 03314109. Always uses SNS_STD_SENSOR_SAMPLE_STATUS_ACCURACY_HIGH for polling mode.
 * 01/18/18                AKM              Modified for Case# 03314109. Added checking ST1 status on polling mode.
 * 01/16/18     010053     AKM              Merged the version 010052 and self test bug modification
 * 01/04/18                AKM              Modified a self test bug
 * 01/11/18     010052     AKM              Modify to avoid sending ASCP request by heart beat timer
 * 01/09/18     010051     Qualcomm         No longer sending Config event when deleting
 * 01/09/18                Qualcomm         Fixed DAE flush commands
 * 12/25/17     010050     AKM              Modified for DC-Lib.
 * 12/25/17                AKM              Fixed comparison of the timestamp for heart_beat_timer_event.
 * 12/20/17     010049     Qualcomm/AKM     Merged the version 010048 and dual sensor modification
 * 12/18/17                Qualcomm/AKM     Fixed dual sensor for simultaneous streaming.
 * 12/11/17                AKM              Debugged for sending SNS_STD_MSGID_SNS_STD_FLUSH_EVENT
 * 12/20/17     010048     AKM              Featurization fix for Flush
 * 12/20/17                Qualcomm         Fixed issue of unexpected stopping of streaming
 * 12/20/17                Qualcomm         Fixed to stream instead of batch when requested rate is lower than MIN ODR
 * 12/20/17                Qualcomm         Always print ERROR messages
 * 11/30/17     010047     AKM              Apply 50Hz limitation to AK09915C/D and AK09917. Fixed for DRI+FIFO mode.
 * 11/30/17                Qualcomm         When not streaming on DAE only flushes when FIFO is in use
 * 11/21/17     010046     AKM              Use requested_timeout_time for Polling system_time.
 * 11/20/17     010045     AKM              Fixed averaged interval for Polling.
 * 11/20/17                AKM              Debugged limited to select 50Hz ODR in ak0991x_mag_match_odr()
 * 11/17/17                AKM              Added AK0991X_FORCE_MAX_ODR_50HZ macro.
 * 11/17/17     010043     AKM              Refactor for clock error measurement.
 * 11/17/17                AKM              Debugged for SEE-Lite compile.
 * 11/17/17                AKM              Fixed delayed Flush response
 * 11/17/17                AKM              Fixed self test
 * 11/17/17                AKM              Modified to use 2 x 100Hz DRI measurements to calculate the clock error
 * 11/17/17                Qualcomm         Fixed race condition in DAE interface module
 * 11/09/17     010042     AKM              Fine tuned average interval calculation
 * 11/09/17                Qualcomm         Changed to disallow Flush requests without Config requests
 * 11/09/17                Qualcomm         When DAE is unavailable reconfig HW after interrupt is ready
 * 11/07/17     010041     Qualcomm         Remove 1Hz ODR. Don't delete timer stream while processing it.
 * 11/04/17     010040     Qualcomm         Added Calibration event. 
 * 11/03/17     010039     AKM              Removed AK09917_REV_A flag. Calculate averaged_interval.
 * 11/03/17                Qualcomm         Fixed flush request handling during power up
 * 10/31/17     010038     AKM              Refactor to use ASCP in flush. Added AK09917_REV_A flag. 
 * 10/31/17                AKM              Added dual sensor support
 * 10/25/17     010037     AKM              Removed averaging filter for DRI mode
 * 10/23/17     010036     Qualcomm         Sends config event to new clients immediately if already streaming
 * 10/20/17     010035     AKM              Modified for SEE-Lite. 
 * 10/20/17                AKM              Fixed negavite timestamp intervals
 * 10/19/17     010034     Qualcomm/AKM     Debugged timestamp issue. Added 1[sec] delay power rail when off. Removed GPIO check.
 * 10/18/17                AKM(M)           Modified for SEE-Lite except using new SUID handler utility
 * 10/18/17     010033     AKM(N)           Added heart beat timer function
 * 10/16/17     010032     AKM(M)           Supports flush_only.
 * 10/16/17     010031     AKM(M)           Debugged negative timestamp when DRI+FIFO/Polling+FIFO.
 * 10/13/17                AKM(M)           Changed S4S name and debugged for negative timestamp when irq->flush->acsp.
 * 10/12/17     010030     AKM(M)           Debugged of the ak0991x_get_adjusted_mag_data.
 * 10/12/17     010029     AKM(M)           Modified for the timestamp in the Polling and FIFO+Polling mode.
 * 10/12/17                AKM(N+M)         Removed duplicate functions/code paths.
 * 10/12/17                AKM(N)           Separated file for S4S.
 * 09/29/17     010028     Qualcomm/AKM     Debugged for negative timestamp issues.
 * 09/26/17     010027     Qualcomm/AKM     Merged Qualcomm's 010020 and AKM's 010026.
 * 09/24/17     010020     Qualcomm         Re-enable FIFO, ts fixes.
 * 09/22/17     010019     Qualcomm         Disable S4S, FIFO temporarily due to ts issues
 * 09/22/17     010026     Qualcomm/AKM     Added this_is_first_data=true when ak0991x_start_mag_streaming() called.
 * 09/20/17                AKM(M)           Modified debug messages(ERROR->LOW, uses %u for timestamp).
 * 09/19/17                AKM(M)           Modified debug messages.
 * 09/19/17     010025     AKM(M)           Added GPIO checking for the DRI interrupt.
 * 09/18/17     010024     AKM(M)           Modified for the timestamp in the FIFO mode
 * 09/14/17     010023     AKM(N+M)         Added compile switches for FIFO/DRI/FUSE and modified for issue#5/#23
 * 09/13/17     010022     AKM(M)           Modified check DRDY bit to ignore wrong notify event call
 * 09/13/17     010021     AKM(M)           Cont. Modified for SEE-Lite mode. Disabled registry. Not finished yet.
 * 09/09/17     010020     AKM(M)           Cont. Modified for SEE-Lite mode. Not finished yet.
 * 09/08/17     010019     AKM(M)           Modified for SEE-Lite mode. Not finished yet.
 * 09/08/17     010018     AKM(M)           Added to check DRDY bit to ignore wrong notify_event call
 * 09/07/17     010017     AKM(M)           enabled AK0991X_DAE_FORCE_NOT_AVAILABLE for second test run
 * 09/07/17     010016     AKM(M)           Merged 010014 and 010015
 * 09/07/17     010014     AKM              Update S4S in non-DAE and DAE.
 * 09/05/17     010015     AKM(M)           Modified filter for DRI/FIFO mode.
 * 09/05/17     010014     AKM(M)           Added filter for DRI mode(won't work with FIFO/Polling modes)
 * 09/05/17     010013     AKM              2nd Support S4S.
 * 09/01/17     010012     AKM              1st Support S4S.
 * 08/25/17     010011     Qualcomm         Work on OpenSSC v5.0.5.
 * 08/10/17     010010     AKM              Modify for prevention of duplicate interrupts using DRDY bit status and timestamp.
 * 08/07/17     010009     AKM              Modify for prevention of duplicate interrupts using DRDY bit status.
 * 08/04/17     010008     AKM              Modify for prevention of duplicate interrupts.
 * 07/10/17     010007     AKM              Modify for self-test which cares streaming.
 * 07/03/17     010006     AKM              Support FIFO+Polling mode.
 * 06/22/17     010005     Qualcomm/AKM     Support COM/HW self-test.
 * 06/22/17                Qualcomm/AKM     Re-do island refactoring.
 * 06/19/17     010004     Qualcomm/AKM     Fix to work on 845 platform.
 * 06/19/17                Qualcomm/AKM     Fix DAE sensor.
 * 06/19/17                Qualcomm/AKM     Fix mag streaming after flush.
 * 06/19/17                Qualcomm/AKM     Fix ODR sweep bugs.
 * 06/19/17                Qualcomm/AKM     Support registry.
 * 06/13/17     010003     Qualcomm         Work on OpenSSC v5.0.4.
 * 05/11/17     010002     AKM              Add AK09917D support.
 * 05/11/17                AKM              Add island mode support.
 * 05/11/17                AKM              Add DAE sensor support.
 * 04/04/17     010001     AKM              Fix IRQ configuration.
 * 04/04/17                AKM              Fix ODR attribute configuration.
 * 02/20/17     010000     AKM              First version.
 *
 **/

#define AK0991X_DRIVER_VERSION 20033  // major:02 minor:00 revision:33