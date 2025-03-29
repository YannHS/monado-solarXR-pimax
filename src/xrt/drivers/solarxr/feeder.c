// Copyright 2025, rcelyte
// SPDX-License-Identifier: BSL-1.0

#include "feeder.h"
#include "ipc_message.h"
#include "xrt/xrt_device.h"
#include <math.h>

// #define FEEDER_DESTROY_GUARD

#define PROTOBUF_FLOAT(v_) ((uint8_t *)&(v_))[0], ((uint8_t *)&(v_))[1], ((uint8_t *)&(v_))[2], ((uint8_t *)&(v_))[3]
#define PROTOBUF_INT32(v_)                                                                                             \
	(uint8_t)(0x80 | (uint32_t)(v_)), (uint8_t)(0x80 | (uint32_t)(v_) >> 7),                                       \
	    (uint8_t)(0x80 | (uint32_t)(v_) >> 14), (uint8_t)(0x80 | (uint32_t)(v_) >> 21), (uint32_t)(v_) >> 28

struct feeder_device
{
	struct xrt_device *xdev;
	enum xrt_input_name input_name;
	uint32_t id;
	uint8_t last_status;
	bool battery_charging;
	float battery_charge;
#ifdef FEEDER_DESTROY_GUARD
	void (*old_destroy)(struct xrt_device *xdev);
#endif
};

void
feeder_send_feedback(struct feeder *const feeder, const int64_t time)
{
	os_mutex_lock(&feeder->mutex);
	uint8_t packet[0x10000], *packet_end = packet;
	for (uint32_t i = 0, devices_len = feeder->devices_len; i < devices_len; ++i) {
		struct feeder_device *const device = &feeder->devices[i];
		if (device->xdev == NULL) {
			continue;
		}

		bool present = false, charging = false;
		float charge = 0;
		if (device->xdev->battery_status_supported &&
		    xrt_device_get_battery_status(device->xdev, &present, &charging, &charge) == XRT_SUCCESS &&
		    (charging != device->battery_charging || fabsf(charge - device->battery_charge) >= 1e-05)) {
			device->battery_charging = charging;
			device->battery_charge = charge;
			uint8_t message[] = {
			    (5 << 3) | 2, 0,                          // ProtobufMessage::battery
			    (1 << 3) | 0, PROTOBUF_INT32(device->id), // Battery::tracker_id
			    (2 << 3) | 5, PROTOBUF_FLOAT(charge),     // Battery::battery_level
			    (3 << 3) | 0, charging,                   // Battery::is_charging
			};
			message[1] = sizeof(message) - 2;
			ipc_message_write_single(&packet_end, &packet[ARRAY_SIZE(packet)], message, sizeof(message));
		}

		uint8_t status = 0; // Status::DISCONNECTED
		struct xrt_space_relation relation = {0};
		assert(device->xdev->get_tracked_pose != NULL);
		if (xrt_device_get_tracked_pose(device->xdev, device->input_name, time, &relation) == XRT_SUCCESS &&
		    (relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) != 0) {
			uint8_t message[30 + 15] = {
			    (1 << 3) | 2, 0,                                           // ProtobufMessage::position
			    (1 << 3) | 0, PROTOBUF_INT32(device->id),                  // Position::tracker_id
			    (5 << 3) | 5, PROTOBUF_FLOAT(relation.pose.orientation.x), // Position::qx
			    (6 << 3) | 5, PROTOBUF_FLOAT(relation.pose.orientation.y), // Position::qy
			    (7 << 3) | 5, PROTOBUF_FLOAT(relation.pose.orientation.z), // Position::qz
			    (8 << 3) | 5, PROTOBUF_FLOAT(relation.pose.orientation.w), // Position::qw
			    (9 << 3) | 0, 3, // Position::data_source = DataSource::FULL
			};
			uint32_t message_len = sizeof(message);
			if ((relation.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0) {
				const uint8_t position[15] = {
				    (2 << 3) | 5, PROTOBUF_FLOAT(relation.pose.position.x), // Position::x
				    (3 << 3) | 5, PROTOBUF_FLOAT(relation.pose.position.y), // Position::y
				    (4 << 3) | 5, PROTOBUF_FLOAT(relation.pose.position.z), // Position::z
				};
				memcpy(&message[sizeof(message) - sizeof(position)], position, sizeof(position));
			} else {
				message_len -= 15;
			}
			message[1] = message_len - 2;
			ipc_message_write_single(&packet_end, &packet[ARRAY_SIZE(packet)], message, message_len);
			status = 1; // Status::OK
		}

		if (status != device->last_status) {
			device->last_status = status;
			uint8_t message[] = {
			    (4 << 3) | 2, 0,                          // ProtobufMessage::tracker_status
			    (1 << 3) | 0, PROTOBUF_INT32(device->id), // TrackerStatus::tracker_id
			    (2 << 3) | 0, status,                     // TrackerStatus::status
			};
			message[1] = sizeof(message) - 2;
			ipc_message_write_single(&packet_end, &packet[ARRAY_SIZE(packet)], message, sizeof(message));
		}
	}
	if (packet_end != packet) {
		ipc_socket_send_raw(&feeder->socket, packet, packet_end - packet);
	}
	atomic_store(&feeder->last_send, time);
	os_mutex_unlock(&feeder->mutex);
}

static bool
filter_feeder_device(struct xrt_device *const device, bool (*const filter)(struct xrt_device *device))
{
	return device != NULL && device->orientation_tracking_supported && (filter == NULL || filter(device));
}

#ifdef FEEDER_DESTROY_GUARD
static void
halt_and_catch_fire(struct xrt_device *xdev)
{
	U_LOG_E("Device destroyed while still in use by feeder!");
	abort();
}
#endif

bool
feeder_set_devices(struct feeder *const feeder,
                   struct xrt_device *const devices[const],
                   const uint32_t devices_len,
                   bool (*const filter)(struct xrt_device *device))
{
	uint32_t device_count = 0;
	for (uint32_t i = 0; i < devices_len; ++i) {
		device_count += filter_feeder_device(devices[i], filter);
	}
	os_mutex_lock(&feeder->mutex);
	if (!ipc_socket_is_connected(&feeder->socket)) {
		if (!ipc_socket_connect(&feeder->socket, "SlimeVRInput", "slimevr/SlimeVRInput", NULL, 0)) {
			os_mutex_unlock(&feeder->mutex);
			return false;
		}
	}
	for (uint32_t i = 0, devices_len = feeder->devices_len; i < devices_len; ++i) {
		if (feeder->devices[i].xdev == NULL) {
			continue;
		}
		// The feeder protocol doesn't define a 'removed' message so this is the best we can do
		uint8_t message[] = {
		    (4 << 3) | 2, 0,                                     // ProtobufMessage::tracker_status
		    (1 << 3) | 0, PROTOBUF_INT32(feeder->devices[i].id), // TrackerStatus::tracker_id
		    (2 << 3) | 0, 0,                                     // TrackerStatus::status = Status::DISCONNECTED
		};
		message[1] = sizeof(message) - 2;
		ipc_socket_send(&feeder->socket, message, sizeof(message));

#ifdef FEEDER_DESTROY_GUARD
		feeder->devices[i].xdev->destroy = feeder->devices[i].old_destroy;
#endif
		feeder->devices[i] = (struct feeder_device){0};
	}
	struct feeder_device *new_devices = NULL;
	if (device_count != 0) {
		new_devices = realloc(feeder->devices, device_count * sizeof(*new_devices));
	}
	bool result;
	if (new_devices == NULL) {
		free(feeder->devices);
		feeder->devices = NULL;
		feeder->devices_len = 0;
		result = device_count == 0;
	} else {
		feeder->devices = new_devices;
		feeder->devices_len = device_count;
		for (uint32_t i = 0; i < devices_len; ++i) {
			if (!filter_feeder_device(devices[i], filter)) {
				continue;
			}
			enum xrt_input_name input_name = XRT_INPUT_GENERIC_TRACKER_POSE;
			for (size_t input = 0, input_count = devices[i]->input_count; input < input_count; ++input) {
				if (XRT_GET_INPUT_TYPE(devices[i]->inputs[input].name) == XRT_INPUT_TYPE_POSE) {
					input_name = devices[i]->inputs[input].name;
					break;
				}
			}
			uint8_t role = 0; // TrackerRole::NONE
			switch (devices[i]->device_type) {
			case XRT_DEVICE_TYPE_HMD: role = 19; break;                   // TrackerRole::HMD
			case XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER: role = 14; break; // TrackerRole::RIGHT_CONTROLLER
			case XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER: role = 13; break;  // TrackerRole::LEFT_CONTROLLER
			case XRT_DEVICE_TYPE_ANY_HAND_CONTROLLER: role = 21; break;   // TrackerRole::GENERIC_CONTROLLER
			default:;
			}
			assert(new_devices < &feeder->devices[feeder->devices_len]);
			const uint32_t id = feeder->next_id++;
			*new_devices++ = (struct feeder_device){
			    .xdev = devices[i],
			    .input_name = input_name,
			    .id = id,
			    .last_status = UINT8_MAX,
#ifdef FEEDER_DESTROY_GUARD
			    .old_destroy = devices[i]->destroy,
#endif
			};
#ifdef FEEDER_DESTROY_GUARD
			devices[i]->destroy = halt_and_catch_fire;
#endif

			const uint32_t serial_len = strlen(devices[i]->serial), name_len = strlen(devices[i]->str);
			const uint8_t message_1[] = {
			    (3 << 3) | 2, PROTOBUF_INT32(0),  // ProtobufMessage::tracker_added
			    (1 << 3) | 0, PROTOBUF_INT32(id), // TrackerAdded::tracker_id
			    (4 << 3) | 0, role,               // TrackerAdded::tracker_role
			    (2 << 3) | 2, PROTOBUF_INT32(serial_len),
			};
			const uint8_t message_2[] = {
			    (3 << 3) | 2,
			    PROTOBUF_INT32(name_len),
			};
			uint8_t packet[sizeof(struct ipc_message) + sizeof(message_1) + sizeof(devices[i]->serial) +
			               sizeof(message_2) + sizeof(devices[i]->str)];
			struct ipc_message *const message = ipc_message_start(packet, &packet[ARRAY_SIZE(packet)]);
			ipc_message_write(message, &packet[ARRAY_SIZE(packet)], message_1, sizeof(message_1));
			ipc_message_write(message, &packet[ARRAY_SIZE(packet)], (const uint8_t *)devices[i]->serial,
			                  serial_len); // TrackerAdded::tracker_serial
			ipc_message_write(message, &packet[ARRAY_SIZE(packet)], message_2, sizeof(message_2));
			ipc_message_write(message, &packet[ARRAY_SIZE(packet)], (const uint8_t *)devices[i]->str,
			                  name_len); // TrackerAdded::tracker_name
			const uint32_t packet_len = ipc_message_end(message, &(uint8_t *){packet});
			if (packet_len == 0) {
				assert(false);
				continue;
			}
			const uint8_t tracker_added[] = {
			    PROTOBUF_INT32(packet_len - sizeof(*message) - sizeof((uint8_t[]){0, PROTOBUF_INT32(0)})),
			};
			memcpy(&message->body[1], tracker_added, sizeof(tracker_added));
			ipc_socket_send_raw(&feeder->socket, packet, packet_len);
		}
		result = true;
	}
	os_mutex_unlock(&feeder->mutex);
	return result;
}

bool
feeder_init(struct feeder *const feeder, const enum u_logging_level log_level)
{
	ipc_socket_init(&feeder->socket, log_level);
	return os_mutex_init(&feeder->mutex) == 0;
}

void
feeder_destroy(struct feeder *const feeder)
{
	feeder_set_devices(feeder, NULL, 0, NULL);
	os_mutex_destroy(&feeder->mutex);
	ipc_socket_destroy(&feeder->socket);
}
