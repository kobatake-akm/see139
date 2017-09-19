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
 * 09/18/17     010025     AKM(M)           Added GPIO checking for the DRI interrupt.
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

#define AK0991X_DRIVER_VERSION 10025  // major:01 minor:00 revision:25
