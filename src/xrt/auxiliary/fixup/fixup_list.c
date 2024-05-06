#include "fixup.h"
#include "pimax.h"

struct fixup_definition fixups[] = {
    {{PIMAX_VID, PIMAX_8KX_PID}, {init_pimax8kx, patch_pimax8kx}},      // Pimax 8KX
    {{0},{0}}     // end
};