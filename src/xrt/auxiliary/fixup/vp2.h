// Copyright 2024, Coreforge
// SPDX-License-Identifier: BSL-1.0

#pragma once

#include "fixup.h"

#define VP2_VID 0x0bb4
#define VP2_PID 0x0342

long init_vivepro2(struct fixup_context* ctx, struct fixup_func_list* funcs, struct hid_device_info* devinfo);