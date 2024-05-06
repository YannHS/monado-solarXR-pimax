add_library(
	aux_fixup STATIC
	fixup.c
	pimax.c
	fixup_list.c
	)
target_link_libraries(
	aux_fixup
	PRIVATE
		xrt-interfaces
		aux_util
		aux_math
		aux_tracking
		xrt-external-cjson
		HIDAPI::hidapi
	)
target_include_directories(aux_fixup PRIVATE ${HIDAPI_INCLUDE_DIRS})