// Copyright 2025, rcelyte
// SPDX-License-Identifier: BSL-1.0

#include "ipc_socket.h"
#include "os/os_time.h"
#include "util/u_file.h"
#include <endian.h>
#include <errno.h>
#include <linux/un.h>
#include <netinet/in.h>
#include <poll.h>
#include <sched.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

void
ipc_socket_init(struct ipc_socket *const state, const enum u_logging_level log_level)
{
	*state = (struct ipc_socket){
	    .sockfd = -1,
	    .log_level = log_level,
	    .timestamp = (int64_t)os_monotonic_get_ns(),
	    .buffer = malloc(0x1000),
	};
	if (state->buffer != NULL) {
		state->buffer_cap = 0x1000;
	}
}

static void
ipc_socket_close(struct ipc_socket *const state)
{
	const int sockfd = atomic_exchange(&state->sockfd, -1);
	if (sockfd == -1) {
		return;
	}
	shutdown(sockfd, SHUT_RDWR); // unblock `ipc_socket_wait()`
	_Static_assert(sizeof(state->reference.count) == sizeof(volatile _Atomic(int)), "");
	while (atomic_load((volatile _Atomic(int) *)&state->reference.count) != 0) {
		sched_yield();
	}
	close(sockfd);
}

void
ipc_socket_destroy(struct ipc_socket *const state)
{
	ipc_socket_close(state);
	free(state->buffer);
	state->buffer = NULL;
	state->buffer_cap = 0;
}

static bool
path_is_socket(const char path[const])
{
	struct stat result = {0};
	return stat(path, &result) == 0 && S_ISSOCK(result.st_mode);
}

bool
ipc_socket_connect(struct ipc_socket *const state,
                   const char runtime_path[const],
                   const char fallback_path[const],
                   char path_out[const],
                   const size_t path_cap)
{
	ipc_socket_close(state);
	const int sockfd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (sockfd == -1) {
		U_LOG_IFL_E(state->log_level, "socket() failed");
		return false;
	}
	struct sockaddr_un addr = {
	    .sun_family = AF_UNIX,
	};
	ssize_t path_len = u_file_get_path_in_runtime_dir(runtime_path, addr.sun_path, sizeof(addr.sun_path));
	if (path_len <= 0 || (size_t)path_len >= sizeof(addr.sun_path)) {
		U_LOG_IFL_E(state->log_level, "u_file_get_path_in_runtime_dir() failed");
		goto fail;
	}
	if (!path_is_socket(addr.sun_path)) {
		U_LOG_IFL_W(state->log_level, "path not found: %s", addr.sun_path);
		const char *env;
		if ((env = getenv("XDG_DATA_HOME")) != NULL) {
			path_len = snprintf(addr.sun_path, sizeof(addr.sun_path), "%s/%s", env, fallback_path);
		} else if ((env = getenv("HOME")) != NULL) {
			path_len =
			    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s/.local/share/%s", env, fallback_path);
		} else {
			path_len = 0;
		}
		if (path_len <= 0 || (size_t)path_len >= sizeof(addr.sun_path)) {
			U_LOG_IFL_E(state->log_level, "failed to resolve SlimeVR socket path");
			goto fail;
		}
		if (!path_is_socket(addr.sun_path)) {
			U_LOG_IFL_E(state->log_level, "path not found: %s", addr.sun_path);
			goto fail;
		}
	}
	if (connect(sockfd, (const struct sockaddr *)&addr, sizeof(addr)) != 0) {
		U_LOG_IFL_E(state->log_level, "connect() failed: %s", strerror(errno));
		goto fail;
	}
	if (path_cap >= 1) {
		if ((size_t)path_len >= path_cap) {
			path_len = path_cap - 1;
		}
		memcpy(path_out, addr.sun_path, path_len + 1);
	}
	atomic_store(&state->sockfd, sockfd);
	return true;
fail:
	close(sockfd);
	return false;
}

bool
ipc_socket_wait(struct ipc_socket *const state)
{
	xrt_reference_inc(&state->reference);
	struct pollfd sockfd = {atomic_load(&state->sockfd), POLLIN, 0};
	bool result = false;
	if (sockfd.fd != -1) {
		result = poll(&sockfd, 1, -1) != -1 || errno == EINTR;
	}
	xrt_reference_dec(&state->reference);
	return result;
}

static size_t
ipc_socket_send_common(struct ipc_socket *const state, struct iovec parts[const], const uint32_t parts_len)
{
	xrt_reference_inc(&state->reference);
	const int sockfd = atomic_load(&state->sockfd);
	if (sockfd == -1) {
		xrt_reference_dec(&state->reference);
		return 0;
	}
	const ssize_t result = sendmsg(sockfd,
	                               &(const struct msghdr){
	                                   .msg_iov = parts,
	                                   .msg_iovlen = parts_len,
	                               },
	                               MSG_NOSIGNAL);
	xrt_reference_dec(&state->reference);
	return (size_t)result;
}

bool
ipc_socket_send(struct ipc_socket *const state, const uint8_t message[const], const uint32_t message_len)
{
	struct iovec parts[2] = {
	    {&(uint32_t){htole32(message_len + sizeof(uint32_t))}, sizeof(uint32_t)},
	    {*(uint8_t **)&message, message_len},
	};
	return ipc_socket_send_common(state, parts, ARRAY_SIZE(parts)) == sizeof(uint32_t) + message_len;
}

bool
ipc_socket_send_raw(struct ipc_socket *const state, const uint8_t packet[const], const uint32_t packet_len)
{
	struct iovec parts[1] = {
	    {*(uint8_t **)&packet, packet_len},
	};
	return ipc_socket_send_common(state, parts, ARRAY_SIZE(parts)) == packet_len;
}

uint32_t
ipc_socket_receive(struct ipc_socket *const state)
{
	xrt_reference_inc(&state->reference);
	const int sockfd = atomic_load(&state->sockfd);
	if (sockfd == -1) {
		xrt_reference_dec(&state->reference);
		return 0;
	}
	if (state->head == state->buffer_len) {
		uint32_t header = 0;
		ssize_t length = recv(sockfd, &header, sizeof(header), MSG_PEEK | MSG_DONTWAIT);
		if (length < 0 && errno != EAGAIN) {
			U_LOG_IFL_E(state->log_level, "recv() failed: %s", strerror(errno));
			goto fail;
		}
		if (length < (ssize_t)sizeof(header)) {
			xrt_reference_dec(&state->reference);
			return 0;
		}
		length = recv(sockfd, &header, sizeof(header), MSG_DONTWAIT);
		if (length != sizeof(header)) {
			U_LOG_IFL_E(state->log_level, "recv() failed: %s",
			            (length < 0) ? strerror(errno) : "bad length");
			goto fail;
		}
		const uint32_t packet_length = le32toh(header) - sizeof(header);
		if (packet_length > state->buffer_cap) {
			if (packet_length > 0x100000u) {
				U_LOG_IFL_E(state->log_level, "packet too large");
				goto fail;
			}
			uint8_t *const new_buffer = realloc(state->buffer, packet_length);
			if (new_buffer == NULL) {
				U_LOG_IFL_E(state->log_level, "realloc failed");
				goto fail;
			}
			state->buffer = new_buffer;
			state->buffer_cap = packet_length;
		}
		state->buffer_len = packet_length;
		state->head = 0;
		state->timestamp = (int64_t)os_monotonic_get_ns();
	}
	for (ssize_t length; state->head < state->buffer_len; state->head += (size_t)length) {
		length = recv(sockfd, &state->buffer[state->head], state->buffer_len - state->head, MSG_DONTWAIT);
		if (length < 0 && errno != EAGAIN) {
			U_LOG_IFL_E(state->log_level, "recv() failed: %s", strerror(errno));
			goto fail;
		}
		if (length <= 0) {
			xrt_reference_dec(&state->reference);
			return 0;
		}
		if (length > state->buffer_len - state->head) {
			U_LOG_IFL_E(state->log_level, "recv() returned invalid length");
			goto fail;
		}
	}
	xrt_reference_dec(&state->reference);
	return state->buffer_len;
fail:
	xrt_reference_dec(&state->reference);
	ipc_socket_destroy(state);
	return 0;
}
