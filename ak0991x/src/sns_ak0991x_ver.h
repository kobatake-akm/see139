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

#define AK0991X_DRIVER_VERSION 10017  // major:01 minor:00 revision:17
