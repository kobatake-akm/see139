#pragma once
/**
 * @file sns_ak0991x_ver.h
 *
 * Driver version
 *
 * Copyright (c) 2017 Asahi Kasei Microdevices
 * All Rights Reserved.
 * Confidential and Proprietary - Asahi Kasei Microdevices
 *
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
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
 * 10/20/17                AKM              Modified averaging paramter for batch/polling.
 * 10/19/17                AKM              Modified for SEE-Lite.
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

#define AK0991X_DRIVER_VERSION 10034  // major:01 minor:00 revision:34
