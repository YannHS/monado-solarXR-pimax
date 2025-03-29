// Copyright 2025, rcelyte
// SPDX-License-Identifier: BSL-1.0

#include "solarxr_interface.h"
#include "feeder.h"
#include "protocol.h"

#include "math/m_api.h"
#include "math/m_relation_history.h"
#include "math/m_vec3.h"
#include "os/os_time.h"
#include "util/u_debug.h"
#include "util/u_device.h"

#include "xrt/xrt_config_build.h" // TODO: drop `#ifdef XRT_FEATURE_OPENXR_BODY_TRACKING_FULL_BODY_META` guards once !2206 is merged

#include <endian.h>
#include <stdatomic.h>
#include <stdio.h>
#include <wchar.h>

#define MAX_GENERIC_TRACKERS 32

DEBUG_GET_ONCE_LOG_OPTION(solarxr_log, "SOLARXR_LOG", U_LOGGING_WARN)
DEBUG_GET_ONCE_BOOL_OPTION(solarxr_raw_trackers, "SOLARXR_RAW_TRACKERS", false)
DEBUG_GET_ONCE_NUM_OPTION(solarxr_sync_delay_ms, "SOLARXR_SYNC_DELAY_MS", 4)
DEBUG_GET_ONCE_NUM_OPTION(solarxr_sync_timeout_ms, "SOLARXR_SYNC_TIMEOUT_MS", 50)

struct solarxr_device;
struct solarxr_generic_tracker
{
	struct xrt_device base;
	struct m_relation_history *history;
	enum solarxr_body_part role;
	struct solarxr_device *parent;
	struct solarxr_generic_tracker **weak_ref;
};

struct solarxr_device
{
	struct xrt_device base;
	struct feeder feeder;
	struct os_thread thread;
	struct ipc_socket socket;
	_Atomic(int64_t) next_sync;
	struct os_mutex mutex;
	_Atomic(int64_t) timestamp;
	_Atomic(uint64_t) enabled_bones;
	static_assert(SOLARXR_BODY_PART_MAX_ENUM <= 64, "bitfield too small");
	struct solarxr_device_bone
	{
		struct xrt_pose pose;
		float length;
	} bones[SOLARXR_BODY_PART_MAX_ENUM];
	wchar_t tracker_ids[MAX_GENERIC_TRACKERS];
	struct solarxr_generic_tracker *trackers[MAX_GENERIC_TRACKERS];
	bool use_trackers;
	uint32_t generation;
	struct xrt_tracking_origin standalone_origin;
};

struct span
{
	size_t length;
	const uint8_t *data;
};

static void
solarxr_device_destroy(struct xrt_device *xdev);
static void
solarxr_generic_tracker_destroy(struct xrt_device *xdev);

static inline struct solarxr_device *
solarxr_device(struct xrt_device *const xdev)
{
	if (xdev == NULL || xdev->destroy != solarxr_device_destroy) {
		return NULL;
	}
	return (struct solarxr_device *)xdev;
}

static inline struct solarxr_generic_tracker *
solarxr_generic_tracker(struct xrt_device *const xdev)
{
	if (xdev == NULL || xdev->destroy != solarxr_generic_tracker_destroy) {
		return NULL;
	}
	return (struct solarxr_generic_tracker *)xdev;
}

// returns an arbitrary unique value to identify trackers by
static inline wchar_t
solarxr_tracker_id_to_wchar(struct solarxr_tracker_id id)
{
	if (!id.has_device_id) {
		id.device_id = 0;
	}
	wchar_t out = 0;
	static_assert(sizeof(id) <= sizeof(out), "");
	memcpy(&out, &id, sizeof(id));
	return le32toh(out);
}

static void
solarxr_device_sync(struct solarxr_device *const device)
{
	const int64_t time = os_monotonic_get_ns(), next_sync = atomic_exchange(&device->next_sync, INT64_MAX);
	if (time <= next_sync) {
		atomic_store(&device->next_sync, next_sync);
		return;
	}

	struct poll_packet
	{
		uint32_t _root;
		uint16_t _table_shared[3];
		struct
		{ // table MessageBundle
			int32_t _table;
			uint32_t data_feed_msgs; // vector*
		} bundle;
		struct
		{ // vector<table DataFeedMessageHeader>
			uint32_t length;
			uint32_t values[1]; // table*
		} data_feed_msgs;
		uint16_t _table_data_feed_msgs_0[4];
		struct
		{ // table DataFeedMessageHeader
			int32_t _table;
			uint32_t message;     // table*
			uint8_t message_type; // enum DataFeedMessage
			uint8_t _pad[3];
		} data_feed_msgs_0;
		struct
		{ // table PollDataFeed
			int32_t _table;
			uint32_t config; // table*
		} message;
		uint16_t _table_config[6];
		struct
		{ // table DataFeedConfig
			int32_t _table;
			uint32_t trackers_mask; // table*
			bool bone_mask;
			uint8_t _pad[3];
		} config;
		struct
		{ // table DeviceDataMask
			int32_t _table;
			uint32_t tracker_data; // table*
		} data_mask;
		uint16_t _table_synthetic_trackers_mask[10];
		struct
		{ // table TrackerDataMask
			int32_t _table;
			bool rotation, position, raw_angular_velocity, linear_acceleration;
		} synthetic_trackers_mask;
	} const poll_packet = {
	    ._root = htole32(offsetof(struct poll_packet, bundle)),
	    ._table_shared =
	        {
	            htole16(sizeof(poll_packet._table_shared)),
	            htole16(8),
	            htole16(4),
	        },
	    .bundle =
	        {
	            ._table =
	                htole32(offsetof(struct poll_packet, bundle) - offsetof(struct poll_packet, _table_shared)),
	            .data_feed_msgs = htole32(offsetof(struct poll_packet, data_feed_msgs) -
	                                      offsetof(struct poll_packet, bundle.data_feed_msgs)),
	        },
	    .data_feed_msgs =
	        {
	            .length = htole32(ARRAY_SIZE(poll_packet.data_feed_msgs.values)),
	            .values = {htole32(offsetof(struct poll_packet, data_feed_msgs_0) -
	                               offsetof(struct poll_packet, data_feed_msgs.values[0]))},
	        },
	    ._table_data_feed_msgs_0 =
	        {
	            htole16(sizeof(poll_packet._table_data_feed_msgs_0)),
	            htole16(sizeof(poll_packet.data_feed_msgs_0) - sizeof(poll_packet.data_feed_msgs_0._pad)),
	            htole16(offsetof(struct poll_packet, data_feed_msgs_0.message_type) -
	                    offsetof(struct poll_packet, data_feed_msgs_0)),
	            htole16(offsetof(struct poll_packet, data_feed_msgs_0.message) -
	                    offsetof(struct poll_packet, data_feed_msgs_0)),
	        },
	    .data_feed_msgs_0 =
	        {
	            ._table = htole32(offsetof(struct poll_packet, data_feed_msgs_0) -
	                              offsetof(struct poll_packet, _table_data_feed_msgs_0)),
	            .message = htole32(offsetof(struct poll_packet, message) -
	                               offsetof(struct poll_packet, data_feed_msgs_0.message)),
	            .message_type = 1, // DataFeedMessage::PollDataFeed
	        },
	    .message =
	        {
	            ._table =
	                htole32(offsetof(struct poll_packet, message) - offsetof(struct poll_packet, _table_shared)),
	            .config =
	                htole32(offsetof(struct poll_packet, config) - offsetof(struct poll_packet, message.config)),
	        },
	    ._table_config =
	        {
	            htole16(sizeof(poll_packet._table_config)),
	            htole16(sizeof(poll_packet.config) - sizeof(poll_packet.config._pad)),
	            0,
	            htole16(
	                (device->use_trackers && debug_get_bool_option_solarxr_raw_trackers()) *
	                (offsetof(struct poll_packet, config.trackers_mask) - offsetof(struct poll_packet, config))),
	            htole16(
	                (device->use_trackers && !debug_get_bool_option_solarxr_raw_trackers()) *
	                (offsetof(struct poll_packet, config.trackers_mask) - offsetof(struct poll_packet, config))),
	            htole16(offsetof(struct poll_packet, config.bone_mask) - offsetof(struct poll_packet, config)),
	        },
	    .config =
	        {
	            ._table =
	                htole32(offsetof(struct poll_packet, config) - offsetof(struct poll_packet, _table_config)),
	            .trackers_mask = htole32((debug_get_bool_option_solarxr_raw_trackers()
	                                          ? offsetof(struct poll_packet, data_mask)
	                                          : offsetof(struct poll_packet, synthetic_trackers_mask)) -
	                                     offsetof(struct poll_packet, config.trackers_mask)),
	            .bone_mask = true,
	        },
	    .data_mask =
	        {
	            ._table =
	                htole32(offsetof(struct poll_packet, data_mask) - offsetof(struct poll_packet, _table_shared)),
	            .tracker_data = htole32(offsetof(struct poll_packet, synthetic_trackers_mask) -
	                                    offsetof(struct poll_packet, data_mask.tracker_data)),
	        },
	    ._table_synthetic_trackers_mask =
	        {
	            htole16(sizeof(poll_packet._table_synthetic_trackers_mask)),
	            htole16(sizeof(poll_packet.synthetic_trackers_mask)),
	            0,
	            0,
	            htole16(offsetof(struct poll_packet, synthetic_trackers_mask.rotation) -
	                    offsetof(struct poll_packet, synthetic_trackers_mask)),
	            htole16(offsetof(struct poll_packet, synthetic_trackers_mask.position) -
	                    offsetof(struct poll_packet, synthetic_trackers_mask)),
	            htole16(offsetof(struct poll_packet, synthetic_trackers_mask.raw_angular_velocity) -
	                    offsetof(struct poll_packet, synthetic_trackers_mask)),
	            0,
	            0,
	            htole16(offsetof(struct poll_packet, synthetic_trackers_mask.linear_acceleration) -
	                    offsetof(struct poll_packet, synthetic_trackers_mask)),
	        },
	    .synthetic_trackers_mask =
	        {
	            ._table = htole32(offsetof(struct poll_packet, synthetic_trackers_mask) -
	                              offsetof(struct poll_packet, _table_synthetic_trackers_mask)),
	            .rotation = true,
	            .position = true,
	            .raw_angular_velocity = true,
	            .linear_acceleration = true,
	        },
	};

	feeder_send_feedback(&device->feeder, time);
	ipc_socket_send(&device->socket, (const uint8_t *)&poll_packet,
	                device->use_trackers ? sizeof(poll_packet) : offsetof(struct poll_packet, data_mask));
	atomic_store(&device->next_sync, time + debug_get_num_option_solarxr_sync_timeout_ms() * U_TIME_1MS_IN_NS);
}

static xrt_result_t
solarxr_device_update_inputs(struct xrt_device *const xdev)
{
	struct solarxr_device *const device = solarxr_device(xdev);
	assert(device != NULL);
	const int64_t time = os_monotonic_get_ns();
	if (time - atomic_load(&device->feeder.last_send) >
	    debug_get_num_option_solarxr_sync_timeout_ms() * U_TIME_1MS_IN_NS) {
		// ensures SlimeVR continues to receive head positions even if no XR app is running
		feeder_send_feedback(&device->feeder, time);
	}

	for (uint32_t i = 0; i < device->base.input_count; ++i) {
		device->base.inputs[i].timestamp = atomic_load(&device->timestamp);
	}
	return XRT_SUCCESS;
}

static xrt_result_t
solarxr_generic_tracker_update_inputs(struct xrt_device *const xdev)
{
	struct solarxr_generic_tracker *const device = solarxr_generic_tracker(xdev);
	assert(device != NULL);
	struct solarxr_device *const parent = device->parent;
	device->base.inputs[0].active = (atomic_load(&parent->enabled_bones) & (1llu << device->role)) != 0;
	device->base.inputs[0].timestamp = atomic_load(&parent->timestamp);
	return XRT_SUCCESS;
}

static inline struct xrt_body_skeleton_joint_fb
offset_joint(const struct xrt_body_skeleton_joint_fb parent, const int32_t name, const struct xrt_vec3 offset)
{
	return (struct xrt_body_skeleton_joint_fb){
	    .pose =
	        {
	            .orientation = parent.pose.orientation,
	            .position = m_vec3_add(parent.pose.position, offset),
	        },
	    .joint = name,
	    .parent_joint = parent.joint,
	};
}

// TODO: filter enabled bones
static xrt_result_t
solarxr_device_get_body_skeleton(struct xrt_device *const xdev,
                                 const enum xrt_input_name body_tracking_type,
                                 struct xrt_body_skeleton *const out_value)
{
	struct xrt_body_skeleton_joint_fb *joints;
	uint32_t joint_count;
	int32_t none;
	switch (body_tracking_type) {
	case XRT_INPUT_FB_BODY_TRACKING: {
		joints = out_value->body_skeleton_fb.joints;
		joint_count = ARRAY_SIZE(out_value->body_skeleton_fb.joints);
		none = XRT_BODY_JOINT_NONE_FB;
		break;
	}
#ifdef XRT_FEATURE_OPENXR_BODY_TRACKING_FULL_BODY_META
	case XRT_INPUT_META_FULL_BODY_TRACKING: {
		joints = out_value->full_body_skeleton_meta.joints;
		joint_count = ARRAY_SIZE(out_value->full_body_skeleton_meta.joints);
		none = XRT_FULL_BODY_JOINT_NONE_META;
		break;
	}
#endif
	default: return XRT_ERROR_NOT_IMPLEMENTED;
	}

	struct solarxr_device *const device = solarxr_device(xdev);
	assert(device != NULL);
	for (uint32_t i = 0; i < joint_count; ++i) {
		joints[i] = (struct xrt_body_skeleton_joint_fb){XRT_POSE_IDENTITY, none, none};
	}

	// The standard doesn't describe the layout for these joints more specifically than being "a T-pose"
	// clang-format off
	joints[0] = (struct xrt_body_skeleton_joint_fb){XRT_POSE_IDENTITY, XRT_BODY_JOINT_HEAD_FB, XRT_BODY_JOINT_ROOT_FB};
	joints[1] = offset_joint(joints[0], XRT_BODY_JOINT_NECK_FB, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_NECK].length, 0.f});
	joints[2] = offset_joint(joints[1], XRT_BODY_JOINT_CHEST_FB, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_UPPER_CHEST].length, 0.f});
	joints[3] = offset_joint(joints[2], XRT_BODY_JOINT_SPINE_UPPER_FB, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_CHEST].length, 0.f});
	joints[4] = offset_joint(joints[3], XRT_BODY_JOINT_SPINE_LOWER_FB, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_WAIST].length, 0.f});
	joints[5] = offset_joint(joints[4], XRT_BODY_JOINT_HIPS_FB, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_HIP].length, 0.f});
	joints[6] = offset_joint(joints[1], XRT_BODY_JOINT_LEFT_SHOULDER_FB, (struct xrt_vec3){-device->bones[SOLARXR_BODY_PART_LEFT_SHOULDER].length, 0.f, 0.f});
	joints[7] = offset_joint(joints[1], XRT_BODY_JOINT_RIGHT_SHOULDER_FB, (struct xrt_vec3){device->bones[SOLARXR_BODY_PART_RIGHT_SHOULDER].length, 0.f, 0.f});
	joints[8] = offset_joint(joints[6], XRT_BODY_JOINT_LEFT_ARM_UPPER_FB, (struct xrt_vec3){-device->bones[SOLARXR_BODY_PART_LEFT_UPPER_ARM].length, 0.f, 0.f});
	joints[9] = offset_joint(joints[7], XRT_BODY_JOINT_RIGHT_ARM_UPPER_FB, (struct xrt_vec3){device->bones[SOLARXR_BODY_PART_RIGHT_UPPER_ARM].length, 0.f, 0.f});
	joints[10] = offset_joint(joints[8], XRT_BODY_JOINT_LEFT_ARM_LOWER_FB, (struct xrt_vec3){-device->bones[SOLARXR_BODY_PART_LEFT_LOWER_ARM].length, 0.f, 0.f});
	joints[11] = offset_joint(joints[9], XRT_BODY_JOINT_RIGHT_ARM_LOWER_FB, (struct xrt_vec3){device->bones[SOLARXR_BODY_PART_RIGHT_LOWER_ARM].length, 0.f, 0.f});
	joints[12] = offset_joint(joints[10], XRT_BODY_JOINT_LEFT_HAND_WRIST_FB, (struct xrt_vec3){-device->bones[SOLARXR_BODY_PART_LEFT_HAND].length, 0.f, 0.f});
	joints[13] = offset_joint(joints[11], XRT_BODY_JOINT_RIGHT_HAND_WRIST_FB, (struct xrt_vec3){device->bones[SOLARXR_BODY_PART_RIGHT_HAND].length, 0.f, 0.f});
#ifdef XRT_FEATURE_OPENXR_BODY_TRACKING_FULL_BODY_META
	if (body_tracking_type != XRT_INPUT_META_FULL_BODY_TRACKING) {
		return XRT_SUCCESS;
	}
	joints[14] = offset_joint(joints[5], XRT_FULL_BODY_JOINT_LEFT_UPPER_LEG_META, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_LEFT_UPPER_LEG].length, 0.f});
	joints[15] = offset_joint(joints[5], XRT_FULL_BODY_JOINT_RIGHT_UPPER_LEG_META, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_RIGHT_UPPER_LEG].length, 0.f});
	joints[16] = offset_joint(joints[14], XRT_FULL_BODY_JOINT_LEFT_LOWER_LEG_META, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_LEFT_LOWER_LEG].length, 0.f});
	joints[17] = offset_joint(joints[15], XRT_FULL_BODY_JOINT_RIGHT_LOWER_LEG_META, (struct xrt_vec3){0.f, -device->bones[SOLARXR_BODY_PART_RIGHT_LOWER_LEG].length, 0.f});
	joints[18] = offset_joint(joints[16], XRT_FULL_BODY_JOINT_LEFT_FOOT_TRANSVERSE_META, (struct xrt_vec3){0.f, 0.f, -device->bones[SOLARXR_BODY_PART_LEFT_FOOT].length});
	joints[19] = offset_joint(joints[17], XRT_FULL_BODY_JOINT_RIGHT_FOOT_TRANSVERSE_META, (struct xrt_vec3){0.f, 0.f, -device->bones[SOLARXR_BODY_PART_RIGHT_FOOT].length});
#endif
	// clang-format on
	return XRT_SUCCESS;
}

// TODO: filter enabled bones
static xrt_result_t
solarxr_device_get_body_joints(struct xrt_device *const xdev,
                               const enum xrt_input_name body_tracking_type,
                               const int64_t desired_timestamp_ns,
                               struct xrt_body_joint_set *const out_value)
{
	static const uint32_t jointMap[SOLARXR_BODY_PART_MAX_ENUM] = {
	    [SOLARXR_BODY_PART_HEAD] = XRT_BODY_JOINT_HEAD_FB,
	    [SOLARXR_BODY_PART_NECK] = XRT_BODY_JOINT_NECK_FB,
	    [SOLARXR_BODY_PART_CHEST] = XRT_BODY_JOINT_SPINE_UPPER_FB,
	    [SOLARXR_BODY_PART_WAIST] = XRT_BODY_JOINT_SPINE_LOWER_FB,
	    [SOLARXR_BODY_PART_HIP] = XRT_BODY_JOINT_HIPS_FB,
#ifdef XRT_FEATURE_OPENXR_BODY_TRACKING_FULL_BODY_META
	    [SOLARXR_BODY_PART_LEFT_UPPER_LEG] = XRT_FULL_BODY_JOINT_LEFT_UPPER_LEG_META,
	    [SOLARXR_BODY_PART_RIGHT_UPPER_LEG] = XRT_FULL_BODY_JOINT_RIGHT_UPPER_LEG_META,
	    [SOLARXR_BODY_PART_LEFT_LOWER_LEG] = XRT_FULL_BODY_JOINT_LEFT_LOWER_LEG_META,
	    [SOLARXR_BODY_PART_RIGHT_LOWER_LEG] = XRT_FULL_BODY_JOINT_RIGHT_LOWER_LEG_META,
	    [SOLARXR_BODY_PART_LEFT_FOOT] = XRT_FULL_BODY_JOINT_LEFT_FOOT_TRANSVERSE_META,
	    [SOLARXR_BODY_PART_RIGHT_FOOT] = XRT_FULL_BODY_JOINT_RIGHT_FOOT_TRANSVERSE_META,
#endif
	    [SOLARXR_BODY_PART_LEFT_LOWER_ARM] = XRT_BODY_JOINT_LEFT_ARM_LOWER_FB,
	    [SOLARXR_BODY_PART_RIGHT_LOWER_ARM] = XRT_BODY_JOINT_RIGHT_ARM_LOWER_FB,
	    [SOLARXR_BODY_PART_LEFT_UPPER_ARM] = XRT_BODY_JOINT_LEFT_ARM_UPPER_FB,
	    [SOLARXR_BODY_PART_RIGHT_UPPER_ARM] = XRT_BODY_JOINT_RIGHT_ARM_UPPER_FB,
	    [SOLARXR_BODY_PART_LEFT_HAND] = XRT_BODY_JOINT_LEFT_HAND_WRIST_FB,
	    [SOLARXR_BODY_PART_RIGHT_HAND] = XRT_BODY_JOINT_RIGHT_HAND_WRIST_FB,
	    [SOLARXR_BODY_PART_LEFT_SHOULDER] = XRT_BODY_JOINT_LEFT_SHOULDER_FB,
	    [SOLARXR_BODY_PART_RIGHT_SHOULDER] = XRT_BODY_JOINT_RIGHT_SHOULDER_FB,
	    [SOLARXR_BODY_PART_UPPER_CHEST] = XRT_BODY_JOINT_CHEST_FB,
	    // LEFT_HIP
	    // RIGHT_HIP
	};
	struct xrt_body_joint_location_fb *joints;
	uint32_t joint_count;
	switch (body_tracking_type) {
	case XRT_INPUT_FB_BODY_TRACKING: {
		joints = out_value->body_joint_set_fb.joint_locations;
		joint_count = ARRAY_SIZE(out_value->body_joint_set_fb.joint_locations);
		break;
	}
#ifdef XRT_FEATURE_OPENXR_BODY_TRACKING_FULL_BODY_META
	case XRT_INPUT_META_FULL_BODY_TRACKING: {
		joints = out_value->full_body_joint_set_meta.joint_locations;
		joint_count = ARRAY_SIZE(out_value->full_body_joint_set_meta.joint_locations);
		break;
	}
#endif
	default: return XRT_ERROR_NOT_IMPLEMENTED;
	}

	struct solarxr_device *const device = solarxr_device(xdev);
	assert(device != NULL);
	solarxr_device_sync(device);

	os_mutex_lock(&device->mutex);
	out_value->base_body_joint_set_meta.sample_time_ns = device->timestamp;
	out_value->base_body_joint_set_meta.confidence = 1.f; // N/A
	out_value->base_body_joint_set_meta.skeleton_changed_count = device->generation;
	out_value->base_body_joint_set_meta.is_active = true;
	for (uint32_t i = 0; i < joint_count; ++i) {
		joints[i].relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
	}
	for (enum solarxr_body_part part = 0; part < ARRAY_SIZE(device->bones); ++part) {
		const struct xrt_pose pose = device->bones[part].pose;
		static_assert(ARRAY_SIZE(jointMap) == ARRAY_SIZE(device->bones), "");
		const uint32_t index = jointMap[part];
		if (index == 0 || index >= joint_count ||
		    memcmp(&pose.orientation, &(struct xrt_quat){0}, sizeof(struct xrt_quat)) == 0) {
			continue;
		}
		joints[index].relation = (struct xrt_space_relation){
		    .relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
		                      XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
		                      XRT_SPACE_RELATION_POSITION_TRACKED_BIT,
		    .pose = pose,
		};
	}
	out_value->body_pose = (struct xrt_space_relation){
	    .relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                      XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT,
	    .pose = XRT_POSE_IDENTITY,
	};
	os_mutex_unlock(&device->mutex);
	return XRT_SUCCESS;
}

static void
solarxr_device_handle_trackers(struct solarxr_device *const device,
                               const struct span buffer,
                               const solarxr_tracker_data_t trackers[const],
                               const uint32_t trackers_len)
{
	for (uint32_t i = 0; i < trackers_len; ++i) {
		struct solarxr_tracker_data data;
		if (!read_solarxr_tracker_data(&data, buffer.data, buffer.length, &trackers[i])) {
			U_LOG_IFL_W(device->socket.log_level, "read_solarxr_device_data() failed");
			continue;
		}
		// `wmemchr()` should be SIMD optimized, making it faster than a hash lookup in this case despite being
		// O(n^2)
		const wchar_t *const match = wmemchr(device->tracker_ids, solarxr_tracker_id_to_wchar(data.tracker_id),
		                                     ARRAY_SIZE(device->tracker_ids));
		if (match == NULL) {
			continue;
		}
		struct solarxr_generic_tracker *const tracker = device->trackers[match - device->tracker_ids];
		if (tracker == NULL) {
			continue;
		}
		struct xrt_space_relation relation = {.pose.orientation.w = 1};
		if (data.has_rotation) {
			relation.relation_flags |=
			    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT;
			relation.pose.orientation = data.rotation;
		}
		if (data.has_position) {
			relation.relation_flags |=
			    XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
			relation.pose.position = data.position;
		}
		if (data.has_raw_angular_velocity) {
			relation.relation_flags |= XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;
			relation.angular_velocity = data.raw_angular_velocity;
		}
		if (data.has_linear_acceleration) {
			relation.relation_flags |= XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT;
			relation.linear_velocity = data.linear_acceleration;
		}
		if (relation.relation_flags != 0) {
			m_relation_history_push(tracker->history, &relation, device->socket.timestamp);
		}
	}
}

static struct span
ipc_socket_receive_blocking(struct ipc_socket *const socket)
{
	do {
		const uint32_t buffer_len = ipc_socket_receive(socket);
		if (buffer_len != 0) {
			return (struct span){buffer_len, socket->buffer};
		}
	} while (ipc_socket_wait(socket));
	return (struct span){0};
}

static void *
solarxr_network_thread(void *const ptr)
{
	struct solarxr_device *const device = (struct solarxr_device *)ptr;
	for (struct span buffer; (buffer = ipc_socket_receive_blocking(&device->socket)).length != 0;) {
		struct solarxr_message_bundle bundle;
		if (!read_solarxr_message_bundle(&bundle, buffer.data, buffer.length,
		                                 (const solarxr_message_bundle_t *)buffer.data)) {
			U_LOG_IFL_W(device->socket.log_level, "read_solarxr_message_bundle() failed");
			continue;
		}
		struct solarxr_steamvr_trackers_setting toggles;
		bool toggles_changed = false;
		for (uint32_t i = 0; i < bundle.rpc_msgs.length; ++i) {
			struct solarxr_rpc_message_header header;
			if (!read_solarxr_rpc_message_header(&header, buffer.data, buffer.length,
			                                     &bundle.rpc_msgs.data[i])) {
				U_LOG_IFL_W(device->socket.log_level, "read_solarxr_rpc_message_header() failed");
				continue;
			}
			if (header.message_type != SOLARXR_RPC_MESSAGE_TYPE_SETTINGS_RESPONSE) {
				continue;
			}
			toggles = header.message.settings_response.steam_vr_trackers;
			toggles_changed = true;
		}
		if (toggles_changed) {
			// `SOLARXR_BODY_PART_HEAD` always disabled
			uint64_t bones = 1llu << SOLARXR_BODY_PART_NECK;
			bones |= (1llu << SOLARXR_BODY_PART_CHEST) * toggles.chest;
			bones |= (1llu << SOLARXR_BODY_PART_WAIST) * toggles.waist;
			bones |= (1llu << SOLARXR_BODY_PART_HIP) * toggles.waist;
			bones |= (1llu << SOLARXR_BODY_PART_LEFT_UPPER_LEG) * toggles.left_knee;
			bones |= (1llu << SOLARXR_BODY_PART_RIGHT_UPPER_LEG) * toggles.right_knee;
			bones |= (1llu << SOLARXR_BODY_PART_LEFT_LOWER_LEG) * (toggles.left_knee && toggles.left_foot);
			bones |=
			    (1llu << SOLARXR_BODY_PART_RIGHT_LOWER_LEG) * (toggles.right_knee && toggles.right_foot);
			bones |= (1llu << SOLARXR_BODY_PART_LEFT_FOOT) * toggles.left_foot;
			bones |= (1llu << SOLARXR_BODY_PART_RIGHT_FOOT) * toggles.right_foot;
			bones |= (1llu << SOLARXR_BODY_PART_LEFT_LOWER_ARM) * (toggles.left_elbow && toggles.left_hand);
			bones |=
			    (1llu << SOLARXR_BODY_PART_RIGHT_LOWER_ARM) * (toggles.right_elbow && toggles.right_hand);
			bones |= (1llu << SOLARXR_BODY_PART_LEFT_UPPER_ARM) * toggles.left_elbow;
			bones |= (1llu << SOLARXR_BODY_PART_RIGHT_UPPER_ARM) * toggles.right_elbow;
			bones |= (1llu << SOLARXR_BODY_PART_LEFT_HAND) * toggles.left_hand;
			bones |= (1llu << SOLARXR_BODY_PART_RIGHT_HAND) * toggles.right_hand;
			bones |= (1llu << SOLARXR_BODY_PART_LEFT_SHOULDER) * toggles.left_elbow;
			bones |= (1llu << SOLARXR_BODY_PART_RIGHT_SHOULDER) * toggles.right_elbow;
			bones |= (1llu << SOLARXR_BODY_PART_UPPER_CHEST) * toggles.chest;
			bones |= (1llu << SOLARXR_BODY_PART_LEFT_HIP) * toggles.waist;
			bones |= (1llu << SOLARXR_BODY_PART_RIGHT_HIP) * toggles.waist;
			bones |= ((2llu << SOLARXR_BODY_PART_LEFT_LITTLE_DISTAL) -
			          (1llu << SOLARXR_BODY_PART_LEFT_THUMB_PROXIMAL)) *
			         toggles.left_hand;
			bones |= ((2llu << SOLARXR_BODY_PART_RIGHT_LITTLE_DISTAL) -
			          (1llu << SOLARXR_BODY_PART_RIGHT_THUMB_PROXIMAL)) *
			         toggles.right_hand;
			atomic_store(&device->enabled_bones, bones);
		}
		if (bundle.data_feed_msgs.length == 0) {
			continue;
		}
#if 0 // for latency testing
		U_LOG_IFL_W(device->socket.log_level, "%.3fus", ((int64_t)os_monotonic_get_ns() - (atomic_load(&device->next_sync) - debug_get_num_option_solarxr_sync_timeout_ms() * U_TIME_1MS_IN_NS)) / 1000.);
#endif
		atomic_store(&device->next_sync,
		             device->timestamp + debug_get_num_option_solarxr_sync_delay_ms() * U_TIME_1MS_IN_NS);
		os_mutex_lock(&device->mutex);
		flatbuffers_vector(solarxr_bone_t) bones = {0};
		for (uint32_t i = 0; i < bundle.data_feed_msgs.length; ++i) {
			struct solarxr_data_feed_message_header header;
			if (!read_solarxr_data_feed_message_header(&header, buffer.data, buffer.length,
			                                           &bundle.data_feed_msgs.data[0])) {
				U_LOG_IFL_W(device->socket.log_level, "read_solarxr_data_feed_message_header() failed");
				continue;
			}
			if (header.message_type != SOLARXR_DATA_FEED_MESSAGE_DATA_FEED_UPDATE) {
				continue;
			}
			if (debug_get_bool_option_solarxr_raw_trackers()) {
				for (uint32_t j = 0; j < header.message.data_feed_update.devices.length; ++j) {
					struct solarxr_device_data device_data;
					if (!read_solarxr_device_data(
					        &device_data, buffer.data, buffer.length,
					        &header.message.data_feed_update.devices.data[j])) {
						U_LOG_IFL_W(device->socket.log_level,
						            "read_solarxr_device_data() failed");
						continue;
					}
					solarxr_device_handle_trackers(device, buffer, device_data.trackers.data,
					                               device_data.trackers.length);
				}
			} else {
				solarxr_device_handle_trackers(
				    device, buffer, header.message.data_feed_update.synthetic_trackers.data,
				    header.message.data_feed_update.synthetic_trackers.length);
			}
			if (header.message.data_feed_update.bones.length != 0) {
				bones.length = header.message.data_feed_update.bones.length;
				bones.data = header.message.data_feed_update.bones.data;
			}
		}
		if (bones.length != 0) {
			atomic_store(&device->timestamp, device->socket.timestamp);
			struct solarxr_device_bone newBones[ARRAY_SIZE(device->bones)] = {0};
			for (size_t i = 0; i < bones.length; ++i) {
				struct solarxr_bone bone;
				if (!read_solarxr_bone(&bone, buffer.data, buffer.length, &bones.data[i])) {
					U_LOG_IFL_W(device->socket.log_level, "read_solarxr_bone() failed");
					continue;
				}
				if (bone.body_part >= ARRAY_SIZE(device->bones)) {
					static bool _once = false;
					if (!_once) {
						_once = true;
						U_LOG_IFL_W(device->socket.log_level, "Unexpected SolarXR BodyPart %u",
						            (unsigned)bone.body_part);
					}
					continue;
				}
				newBones[bone.body_part].pose = (struct xrt_pose){
				    .orientation = bone.rotation_g,
				    .position = bone.head_position_g,
				};
				newBones[bone.body_part].length = bone.bone_length;
			}
			for (uint32_t i = 0; i < ARRAY_SIZE(device->bones); ++i) {
				if (memcmp(&newBones[i].length, &device->bones[i].length, sizeof(newBones[i].length)) !=
				    0) {
					++device->generation;
					break;
				}
			}
			memcpy(device->bones, newBones, sizeof(device->bones));
		}
		os_mutex_unlock(&device->mutex);
	}
	return NULL;
}

static void
solarxr_device_destroy(struct xrt_device *xdev)
{
	struct solarxr_device *const device = solarxr_device(xdev);
	assert(device != NULL);
	feeder_destroy(&device->feeder);
	ipc_socket_destroy(&device->socket);
	if (!pthread_equal(device->thread.thread, pthread_self())) {
		os_thread_join(&device->thread);
		os_thread_destroy(&device->thread);
	}
	for (uint32_t i = 0; i < ARRAY_SIZE(device->trackers); ++i) {
		if (device->trackers[i] == NULL) {
			continue;
		}
		device->trackers[i]->weak_ref = NULL;
		device->trackers[i]->parent = NULL;
	}
	os_mutex_destroy(&device->mutex);
	u_device_free(&device->base);
}

static xrt_result_t
solarxr_generic_tracker_get_tracked_pose(struct xrt_device *const xdev,
                                         const enum xrt_input_name name,
                                         const int64_t at_timestamp_ns,
                                         struct xrt_space_relation *const out_relation)
{
	struct solarxr_generic_tracker *const device = solarxr_generic_tracker(xdev);
	assert(device != NULL);
	struct solarxr_device *const parent = device->parent;
	solarxr_device_sync(parent);
	if ((atomic_load(&parent->enabled_bones) & (1llu << device->role)) == 0) {
		*out_relation = (struct xrt_space_relation){0};
		return XRT_ERROR_INPUT_UNSUPPORTED; // TODO: adding/removing devices at runtime
	}
	m_relation_history_get(device->history, at_timestamp_ns, out_relation);
	return XRT_SUCCESS;
}

static void
solarxr_generic_tracker_destroy(struct xrt_device *const xdev)
{
	struct solarxr_generic_tracker *const device = solarxr_generic_tracker(xdev);
	assert(device != NULL);
	if (device->weak_ref != NULL) {
		os_mutex_lock(&device->parent->mutex);
		*device->weak_ref = NULL;
		os_mutex_unlock(&device->parent->mutex);
	}
	m_relation_history_destroy(&device->history);
	u_device_free(&device->base);
}

uint32_t
solarxr_device_create_xdevs(struct xrt_tracking_origin *const tracking_origin,
                            struct xrt_device *out_xdevs[const],
                            uint32_t out_xdevs_cap)
{
	if (out_xdevs_cap == 0) {
		return 0;
	}
	if (out_xdevs_cap - 1 > MAX_GENERIC_TRACKERS) {
		out_xdevs_cap = 1 + MAX_GENERIC_TRACKERS;
	}

#ifdef XRT_FEATURE_OPENXR_BODY_TRACKING_FULL_BODY_META
	struct solarxr_device *const device = U_DEVICE_ALLOCATE(struct solarxr_device, U_DEVICE_ALLOC_NO_FLAGS, 2, 0);
#else
	struct solarxr_device *const device = U_DEVICE_ALLOCATE(struct solarxr_device, U_DEVICE_ALLOC_NO_FLAGS, 1, 0);
#endif
	device->base.name = XRT_DEVICE_FB_BODY_TRACKING;
	device->base.device_type = XRT_DEVICE_TYPE_BODY_TRACKER;
	strncpy(device->base.str, "SolarXR IPC Connection", sizeof(device->base.str) - 1);
	device->base.tracking_origin = tracking_origin;
	if (device->base.tracking_origin == NULL) {
		device->base.tracking_origin = &device->standalone_origin;
		device->standalone_origin = (struct xrt_tracking_origin){
		    .name = "SolarXR Bridge",
		    .type = XRT_TRACKING_TYPE_OTHER,
		    .initial_offset = XRT_POSE_IDENTITY,
		};
	}
	device->base.body_tracking_supported = true;
	device->base.update_inputs = solarxr_device_update_inputs;
	device->base.get_body_skeleton = solarxr_device_get_body_skeleton;
	device->base.get_body_joints = solarxr_device_get_body_joints;
	device->base.destroy = solarxr_device_destroy;
	device->base.inputs[0].name = XRT_INPUT_FB_BODY_TRACKING;
#ifdef XRT_FEATURE_OPENXR_BODY_TRACKING_FULL_BODY_META
	device->base.inputs[1].name = XRT_INPUT_META_FULL_BODY_TRACKING;
#endif
	device->thread.thread = pthread_self();
	const bool use_trackers = (out_xdevs_cap >= 2);
	device->use_trackers = use_trackers;
	ipc_socket_init(&device->socket, debug_get_log_option_solarxr_log());
	memset(device->tracker_ids, 0xff, sizeof(device->tracker_ids));

	// `solarxr_device_destroy()` asserts unless both have attempted initialization
	if (((int)!feeder_init(&device->feeder, debug_get_log_option_solarxr_log()) | os_mutex_init(&device->mutex)) !=
	    0) {
		goto fail;
	}

	if (!ipc_socket_connect(&device->socket, "SlimeVRRpc", "slimevr/SlimeVRRpc", device->base.serial,
	                        sizeof(device->base.serial))) {
		goto fail;
	}

	struct request_packet
	{
		uint32_t _root;
		uint16_t _table_bundle[4];
		struct
		{ // table MessageBundle
			int32_t _table;
			uint32_t data_feed_msgs; // vector*
			uint32_t rpc_msgs;       // vector*
		} bundle;
		struct
		{ // vector<table RpcMessageHeader>
			uint32_t length;
			uint32_t values[1]; // table*
		} rpc_msgs;
		uint16_t _table_rpc_msgs_0[5];
		struct
		{ // table RpcMessageHeader
			int32_t _table;
			uint32_t message;     // table*
			uint8_t message_type; // enum RpcMessage
			uint8_t _pad[3];
		} rpc_msgs_0;
		uint16_t _table_request[2];
		struct
		{ // table SettingsRequest
			int32_t _table;
		} request;
		struct
		{ // vector<table DataFeedMessageHeader>
			uint32_t length;
			uint32_t values[1]; // table*
		} data_feed_msgs;
		uint16_t _table_data_feed_msgs_0[4];
		struct
		{ // table DataFeedMessageHeader
			int32_t _table;
			uint32_t message;     // table*
			uint8_t message_type; // enum DataFeedMessage
			uint8_t _pad[3];
		} data_feed_msgs_0;
		uint16_t _table_shared[3];
		struct
		{ // table PollDataFeed
			int32_t _table;
			uint32_t config; // table*
		} message;
		uint16_t _table_config[6];
		struct
		{ // table DataFeedConfig
			int32_t _table;
			uint32_t trackers_mask; // table*
			bool bone_mask;
			uint8_t _pad[3];
		} config;
		struct
		{ // table DeviceDataMask
			int32_t _table;
			uint32_t tracker_data; // table*
		} data_mask;
		uint16_t _table_synthetic_trackers_mask[3];
		struct
		{ // table TrackerDataMask
			int32_t _table;
			bool info;
		} synthetic_trackers_mask;
	} const request_packet = {
	    ._root = htole32(offsetof(struct request_packet, bundle)),
	    ._table_bundle =
	        {
	            htole16(sizeof(request_packet._table_bundle)),
	            htole16(sizeof(request_packet.bundle)),
	            htole16(use_trackers * (offsetof(struct request_packet, bundle.data_feed_msgs) -
	                                    offsetof(struct request_packet, bundle))),
	            htole16(offsetof(struct request_packet, bundle.rpc_msgs) - offsetof(struct request_packet, bundle)),
	        },
	    .bundle =
	        {
	            ._table = htole32(offsetof(struct request_packet, bundle) -
	                              offsetof(struct request_packet, _table_bundle)),
	            .data_feed_msgs = htole32(use_trackers * (offsetof(struct request_packet, data_feed_msgs) -
	                                                      offsetof(struct request_packet, bundle.data_feed_msgs))),
	            .rpc_msgs = htole32(offsetof(struct request_packet, rpc_msgs) -
	                                offsetof(struct request_packet, bundle.rpc_msgs)),
	        },
	    .rpc_msgs =
	        {
	            .length = htole32(ARRAY_SIZE(request_packet.rpc_msgs.values)),
	            .values = {htole32(offsetof(struct request_packet, rpc_msgs_0) -
	                               offsetof(struct request_packet, rpc_msgs.values[0]))},
	        },
	    ._table_rpc_msgs_0 =
	        {
	            htole16(sizeof(request_packet._table_rpc_msgs_0)),
	            htole16(sizeof(request_packet.rpc_msgs_0) - sizeof(request_packet.rpc_msgs_0._pad)),
	            0,
	            htole16(offsetof(struct request_packet, rpc_msgs_0.message_type) -
	                    offsetof(struct request_packet, rpc_msgs_0)),
	            htole16(offsetof(struct request_packet, rpc_msgs_0.message) -
	                    offsetof(struct request_packet, rpc_msgs_0)),
	        },
	    .rpc_msgs_0 =
	        {
	            ._table = htole32(offsetof(struct request_packet, rpc_msgs_0) -
	                              offsetof(struct request_packet, _table_rpc_msgs_0)),
	            .message = htole32(offsetof(struct request_packet, request) -
	                               offsetof(struct request_packet, rpc_msgs_0.message)),
	            .message_type = SOLARXR_RPC_MESSAGE_TYPE_SETTINGS_REQUEST,
	        },
	    ._table_request =
	        {
	            htole16(sizeof(request_packet._table_request)),
	            htole16(sizeof(request_packet.request)),
	        },
	    .request =
	        {
	            ._table = htole32(offsetof(struct request_packet, request) -
	                              offsetof(struct request_packet, _table_request)),
	        },
	    .data_feed_msgs =
	        {
	            .length = htole32(ARRAY_SIZE(request_packet.data_feed_msgs.values)),
	            .values = {htole32(offsetof(struct request_packet, data_feed_msgs_0) -
	                               offsetof(struct request_packet, data_feed_msgs.values[0]))},
	        },
	    ._table_data_feed_msgs_0 =
	        {
	            htole16(sizeof(request_packet._table_data_feed_msgs_0)),
	            htole16(sizeof(request_packet.data_feed_msgs_0) - sizeof(request_packet.data_feed_msgs_0._pad)),
	            htole16(offsetof(struct request_packet, data_feed_msgs_0.message_type) -
	                    offsetof(struct request_packet, data_feed_msgs_0)),
	            htole16(offsetof(struct request_packet, data_feed_msgs_0.message) -
	                    offsetof(struct request_packet, data_feed_msgs_0)),
	        },
	    .data_feed_msgs_0 =
	        {
	            ._table = htole32(offsetof(struct request_packet, data_feed_msgs_0) -
	                              offsetof(struct request_packet, _table_data_feed_msgs_0)),
	            .message = htole32(offsetof(struct request_packet, message) -
	                               offsetof(struct request_packet, data_feed_msgs_0.message)),
	            .message_type = SOLARXR_DATA_FEED_MESSAGE_POLL_DATA_FEED,
	        },
	    ._table_shared =
	        {
	            htole16(sizeof(request_packet._table_shared)),
	            htole16(8),
	            htole16(4),
	        },
	    .message =
	        {
	            ._table = htole32(offsetof(struct request_packet, message) -
	                              offsetof(struct request_packet, _table_shared)),
	            .config = htole32(offsetof(struct request_packet, config) -
	                              offsetof(struct request_packet, message.config)),
	        },
	    ._table_config =
	        {
	            htole16(sizeof(request_packet._table_config)),
	            htole16(sizeof(request_packet.config) - sizeof(request_packet.config._pad)),
	            0,
	            htole16((device->use_trackers && debug_get_bool_option_solarxr_raw_trackers()) *
	                    (offsetof(struct request_packet, config.trackers_mask) -
	                     offsetof(struct request_packet, config))),
	            htole16((device->use_trackers && !debug_get_bool_option_solarxr_raw_trackers()) *
	                    (offsetof(struct request_packet, config.trackers_mask) -
	                     offsetof(struct request_packet, config))),
	            htole16(offsetof(struct request_packet, config.bone_mask) -
	                    offsetof(struct request_packet, config)),
	        },
	    .config =
	        {
	            ._table = htole32(offsetof(struct request_packet, config) -
	                              offsetof(struct request_packet, _table_config)),
	            .trackers_mask = htole32((debug_get_bool_option_solarxr_raw_trackers()
	                                          ? offsetof(struct request_packet, data_mask)
	                                          : offsetof(struct request_packet, synthetic_trackers_mask)) -
	                                     offsetof(struct request_packet, config.trackers_mask)),
	            .bone_mask = true,
	        },
	    .data_mask =
	        {
	            ._table = htole32(offsetof(struct request_packet, data_mask) -
	                              offsetof(struct request_packet, _table_shared)),
	            .tracker_data = htole32(offsetof(struct request_packet, synthetic_trackers_mask) -
	                                    offsetof(struct request_packet, data_mask.tracker_data)),
	        },
	    ._table_synthetic_trackers_mask =
	        {
	            htole16(sizeof(request_packet._table_synthetic_trackers_mask)),
	            htole16(sizeof(request_packet.synthetic_trackers_mask)),
	            htole16(offsetof(struct request_packet, synthetic_trackers_mask.info) -
	                    offsetof(struct request_packet, synthetic_trackers_mask)),
	        },
	    .synthetic_trackers_mask =
	        {
	            ._table = htole32(offsetof(struct request_packet, synthetic_trackers_mask) -
	                              offsetof(struct request_packet, _table_synthetic_trackers_mask)),
	            .info = true,
	        },
	};

	if (!ipc_socket_send(&device->socket, (const uint8_t *)&request_packet,
	                     use_trackers ? sizeof(request_packet) : offsetof(struct request_packet, data_feed_msgs))) {
		U_LOG_IFL_E(device->socket.log_level, "ipc_socket_send() failed");
		goto fail;
	}

	if (use_trackers) {
		const struct span buffer = ipc_socket_receive_blocking(&device->socket);
		if (buffer.length == 0) {
			U_LOG_IFL_E(device->socket.log_level, "ipc_socket_receive() failed");
			goto fail;
		}
		struct solarxr_message_bundle bundle;
		if (!read_solarxr_message_bundle(&bundle, buffer.data, buffer.length,
		                                 (const solarxr_message_bundle_t *)buffer.data)) {
			U_LOG_IFL_E(device->socket.log_level, "read_solarxr_message_bundle() failed");
			goto fail;
		}
		if (bundle.data_feed_msgs.length != 1) {
			U_LOG_IFL_E(device->socket.log_level, "Unexpected data feed count");
			goto fail;
		}
		struct solarxr_data_feed_message_header header;
		if (!read_solarxr_data_feed_message_header(&header, buffer.data, buffer.length,
		                                           &bundle.data_feed_msgs.data[0])) {
			U_LOG_IFL_E(device->socket.log_level, "read_solarxr_data_feed_message_header() failed");
			goto fail;
		}
		if (header.message_type != SOLARXR_DATA_FEED_MESSAGE_DATA_FEED_UPDATE) {
			U_LOG_IFL_E(device->socket.log_level, "Unexpected data feed message type");
			goto fail;
		}
		uint32_t tracker_descs_len = 0;
		const solarxr_tracker_data_t *tracker_descs[ARRAY_SIZE(device->trackers)];
		if (debug_get_bool_option_solarxr_raw_trackers()) {
			for (uint32_t i = 0; i < header.message.data_feed_update.devices.length; ++i) {
				struct solarxr_device_data device_data;
				if (!read_solarxr_device_data(&device_data, buffer.data, buffer.length,
				                              &header.message.data_feed_update.devices.data[i])) {
					U_LOG_IFL_W(device->socket.log_level, "read_solarxr_device_data() failed");
					continue;
				}
				uint32_t length = device_data.trackers.length;
				if (length >= ARRAY_SIZE(tracker_descs) - tracker_descs_len) {
					length = ARRAY_SIZE(tracker_descs) - tracker_descs_len;
					header.message.data_feed_update.devices.length = i; // early exit
				}
				for (uint32_t j = 0; j < length; ++j) {
					tracker_descs[tracker_descs_len++] = &device_data.trackers.data[j];
				}
			}
		} else {
			tracker_descs_len =
			    MIN(header.message.data_feed_update.synthetic_trackers.length, ARRAY_SIZE(tracker_descs));
			for (uint32_t i = 0; i < tracker_descs_len; ++i) {
				tracker_descs[i] = &header.message.data_feed_update.synthetic_trackers.data[i];
			}
		}
		if (tracker_descs_len > out_xdevs_cap - 1) {
			tracker_descs_len = out_xdevs_cap - 1;
		}
		for (uint32_t i = 0; i < tracker_descs_len; ++i) {
			struct solarxr_tracker_data data;
			if (!read_solarxr_tracker_data(&data, buffer.data, buffer.length, tracker_descs[i])) {
				U_LOG_IFL_W(device->socket.log_level, "read_solarxr_device_data() failed");
				continue;
			}
			const wchar_t id = solarxr_tracker_id_to_wchar(data.tracker_id);

			struct solarxr_generic_tracker *const tracker =
			    U_DEVICE_ALLOCATE(struct solarxr_generic_tracker, U_DEVICE_ALLOC_NO_FLAGS, 1, 0);
			tracker->base.name = XRT_DEVICE_VIVE_TRACKER; // TODO: use different name here?
			tracker->base.device_type = XRT_DEVICE_TYPE_GENERIC_TRACKER;
			snprintf(tracker->base.str, sizeof(tracker->base.str), "SolarXR Tracker %06x", id);
			snprintf(tracker->base.serial, sizeof(tracker->base.serial), "SOLARXR-%06x", id);
			tracker->base.tracking_origin = device->base.tracking_origin;
			tracker->base.orientation_tracking_supported = true;
			tracker->base.position_tracking_supported = true;
			tracker->base.update_inputs = solarxr_generic_tracker_update_inputs;
			tracker->base.get_tracked_pose = solarxr_generic_tracker_get_tracked_pose;
			tracker->base.destroy = solarxr_generic_tracker_destroy;
			tracker->base.inputs[0].name = XRT_INPUT_GENERIC_TRACKER_POSE;
			m_relation_history_create(&tracker->history);
			tracker->role = SOLARXR_BODY_PART_NONE;
			tracker->parent = device;
			tracker->weak_ref = &device->trackers[i];
			device->trackers[i] = tracker;
			device->tracker_ids[i] = id;

			if (!data.has_info) {
				continue;
			}
			if (data.info.body_part < ARRAY_SIZE(device->bones)) {
				tracker->role = data.info.body_part;
			}
			if (data.info.display_name.length != 0) {
				snprintf(tracker->base.str, sizeof(tracker->base.str), "SolarXR Tracker \"%.*s\"",
				         (unsigned)data.info.display_name.length, data.info.display_name.data);
			}
		}
	}
	if (os_thread_start(&device->thread, solarxr_network_thread, device) != 0) {
		U_LOG_IFL_E(device->socket.log_level, "pthread_create() failed");
		for (uint32_t i = 0; i < ARRAY_SIZE(device->trackers); ++i) {
			if (device->trackers[i] != NULL) {
				solarxr_generic_tracker_destroy(&device->trackers[i]->base);
				device->trackers[i] = NULL;
			}
		}
		goto fail;
	}
	// early sync to initialize bone lengths needed by `solarxr_device_get_body_skeleton()`
	solarxr_device_sync(device);

	uint32_t trackerCount = 0;
	out_xdevs[trackerCount++] = &device->base;
	for (uint32_t i = 0; i < ARRAY_SIZE(device->trackers); ++i) {
		if (device->trackers[i] == NULL) {
			continue;
		}
		assert(trackerCount < out_xdevs_cap);
		out_xdevs[trackerCount++] = &device->trackers[i]->base;
	}
	return trackerCount;
fail:
	solarxr_device_destroy(&device->base);
	return 0;
}

static bool
filter_feeder_device(struct xrt_device *const device)
{
	return solarxr_generic_tracker(device) == NULL;
}

bool
solarxr_device_set_feeder_devices(struct xrt_device *const device,
                                  struct xrt_device *const xdevs[const],
                                  const uint32_t xdevs_len)
{
	struct solarxr_device *const solarxr = solarxr_device(device);
	return (solarxr != NULL) ? feeder_set_devices(&solarxr->feeder, xdevs, xdevs_len, filter_feeder_device) : false;
}
