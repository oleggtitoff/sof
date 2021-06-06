/* SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright(c) 2021 Intel Corporation. All rights reserved.
 */

/*
 * This file contains structures that are exact copies of an existing ABI used
 * by IOT middleware. They are Intel specific and will be used by one middleware.
 *
 * Some of the structures may contain programming implementations that makes them
 * unsuitable for generic use and general usage.
 */

/**
 * \file include/ipc4/notification.h
 * \brief IPC4 notification definitions
 */

#ifndef __IPC4_NOTIFICATION_H__
#define __IPC4_NOTIFICATION_H__

#include <stdint.h>
#include <ipc4/header.h>

/* ipc4 notification msg */

enum sof_ipc4_notification_type {
	SOF_IPC4_NOTIFY_PHRASE_DETECTED		= 4,
	SOF_IPC4_NOTIFY_RESOURCE_EVENT		= 5,
	SOF_IPC4_NOTIFY_LOG_BUFFER_STATUS	= 6,
	SOF_IPC4_NOTIFY_TIMESTAMP_CAPTURED	= 7,
	SOF_IPC4_NOTIFY_FW_READY		= 8,
	SOF_IPC4_FW_AUD_CLASS_RESULT		= 9,
	SOF_IPC4_EXCEPTION_CAUGHT		= 10,
	SOF_IPC4_MODULE_NOTIFICATION		= 12,
	SOF_IPC4_UAOL_RSVD_			= 13,
	SOF_IPC4_PROBE_DATA_AVAILABLE		= 14,
	SOF_IPC4_WATCHDOG_TIMEOUT		= 15,
	SOF_IPC4_MANAGEMENT_SERVICE		= 16,
};

#define SOF_IPC4_GLB_NOTIFY_DIR_MASK	BIT(29)
#define SOF_IPC4_REPLY_STATUS_MASK	0xFFFFFF
#define SOF_IPC4_GLB_NOTIFY_TYPE_SHIFT	16
#define SOF_IPC4_GLB_NOTIFY_MSG_TYPE_SHIFT	24


#define SOF_IPC4_FW_READY \
		(((SOF_IPC4_NOTIFY_FW_READY) << (SOF_IPC4_GLB_NOTIFY_TYPE_SHIFT)) |\
		((SOF_IPC4_GLB_NOTIFICATION) << (SOF_IPC4_GLB_NOTIFY_MSG_TYPE_SHIFT)))
#endif
