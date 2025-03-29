// Copyright 2025, rcelyte
// SPDX-License-Identifier: BSL-1.0

#pragma once
#include "util/u_logging.h"
#include <stdatomic.h>

struct ipc_socket
{
	_Atomic(int) sockfd;
	struct xrt_reference reference;
	enum u_logging_level log_level;
	int64_t timestamp;
	uint32_t head, buffer_len, buffer_cap;
	uint8_t *buffer;
};

void
ipc_socket_init(struct ipc_socket *state, enum u_logging_level log_level);

void
ipc_socket_destroy(struct ipc_socket *state); // thread safe

bool
ipc_socket_connect(
    struct ipc_socket *state, const char runtime_path[], const char fallback_path[], char path_out[], size_t path_cap);

bool
ipc_socket_wait(struct ipc_socket *state);

bool
ipc_socket_send(struct ipc_socket *state, const uint8_t message[], uint32_t message_len); // thread safe

bool
ipc_socket_send_raw(struct ipc_socket *state, const uint8_t packet[], uint32_t packet_len); // thread safe

uint32_t
ipc_socket_receive(struct ipc_socket *state);

static inline bool
ipc_socket_is_connected(struct ipc_socket *const state)
{
	return atomic_load(&state->sockfd) != -1;
}
