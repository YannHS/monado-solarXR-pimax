// Copyright 2025, rcelyte
// SPDX-License-Identifier: BSL-1.0

#pragma once
#include <endian.h>

struct ipc_message
{
	uint32_t length;
	uint8_t body[];
};

static inline struct ipc_message *
ipc_message_start(uint8_t head[const], const uint8_t *const end)
{
	if ((size_t)(end - head) < sizeof(struct ipc_message)) {
		return NULL;
	}
	struct ipc_message *const message = (struct ipc_message *)head;
	message->length = 0;
	return message;
}

static inline bool
ipc_message_write(struct ipc_message *const message,
                  const uint8_t *const end,
                  const uint8_t data[const],
                  const uint32_t data_len)
{
	if (message == NULL) {
		return false;
	}
	if ((size_t)(end - message->body) < message->length || end - &message->body[message->length] < data_len) {
		message->length = UINT32_MAX;
		return false;
	}
	memcpy(&message->body[message->length], data, data_len);
	message->length += data_len;
	return true;
}

static inline uint32_t
ipc_message_end(struct ipc_message *const message, uint8_t **const end_out)
{
	if (message == NULL || message->length >= UINT32_MAX - sizeof(*message)) {
		return 0;
	}
	*end_out = &message->body[message->length];
	const uint32_t length = sizeof(*message) + message->length;
	message->length = htole32(length);
	return length;
}

static inline uint32_t
ipc_message_write_single(uint8_t **const head,
                         const uint8_t *const end,
                         const uint8_t data[const],
                         const uint32_t data_len)
{
	struct ipc_message *const message = ipc_message_start(*head, end);
	ipc_message_write(message, end, data, data_len);
	return ipc_message_end(message, head);
}
