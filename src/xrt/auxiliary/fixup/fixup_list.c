#include "fixup.h"
#include "pimax.h"
#include "vp2.h"


struct fixup_definition fixups[] = {
    {{PIMAX_VID, PIMAX_8KX_PID}, {init_pimax8kx, patch_pimax8kx}},      // Pimax 8KX
    {{VP2_VID, VP2_PID}, {init_vivepro2, NULL}},      // Vive Pro 2
    {{0},{0}}     // end
};