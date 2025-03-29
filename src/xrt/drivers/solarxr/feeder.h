// Copyright 2025, rcelyte
// SPDX-License-Identifier: BSL-1.0

#pragma once
#include "ipc_socket.h"
#include "os/os_threading.h"

struct feeder_device;
struct feeder
{
	struct os_mutex mutex;
	struct ipc_socket socket;
	_Atomic(int64_t) last_send;
	uint32_t next_id;
	uint32_t devices_len;
	struct feeder_device *devices;
};

bool
feeder_init(struct feeder *feeder, enum u_logging_level log_level);

void
feeder_destroy(struct feeder *feeder);

void
feeder_send_feedback(struct feeder *feeder, int64_t time); // thread safe

bool
feeder_set_devices(struct feeder *feeder,
                   struct xrt_device *const devices[],
                   uint32_t devices_len,
                   bool (*filter)(struct xrt_device *device)); // thread safe
