#pragma once
struct pimax_mesh_info{
       float ipd;
       float* mesh;
       float fovs[2][4];
};


#define PIMAX_MESH_0_VERTEX_COUNT 8450
#define PIMAX_MESH_1_VERTEX_COUNT 8450
#define PIMAX_MESH_2_VERTEX_COUNT 8450
#define PIMAX_MESH_3_VERTEX_COUNT 8450
#define PIMAX_MESH_4_VERTEX_COUNT 8450
#define PIMAX_MESH_COUNT 5
extern struct pimax_mesh_info pimax_distortion_meshes[];
