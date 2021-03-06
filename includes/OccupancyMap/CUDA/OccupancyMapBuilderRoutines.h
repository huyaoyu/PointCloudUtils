//
// Created by yaoyu on 5/10/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_HOLEBOUNDARYDETECTION_CUDA_OCCUPANCYMAPBUILDERROUTINES_H
#define POINTCLOUDUTILS_INCLUDES_HOLEBOUNDARYDETECTION_CUDA_OCCUPANCYMAPBUILDERROUTINES_H

#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>

#include <thrust/host_vector.h>

namespace pcu {
enum {
    OCP_MAP_CAM_INVISIBLE = 0,
    OCP_MAP_CAM_VISIBLE
};

typedef enum {
    OCP_MAP_OCC_FREE = 0,
    OCP_MAP_OCC_OCCUPIED,
    OCP_MAP_OCC_UNKNOWN,
    OCP_MAP_OCC_FRONTIER,
    OCP_MAP_OCC_PADDING
} OCP_MAP_OCC_t;

typedef float CReal;
typedef std::uint8_t CMask;

class CR_VisMask {
public:
    CR_VisMask(int n);
    ~CR_VisMask();

    CReal *get_u_cam_proj();
    void set_cam_proj_size(int h, int w);
    void copy_point_cloud(const CReal *pPointCloud, int n);
    int cr_update_visibility_mask();
    CMask *get_vis_mask();
    CReal *get_pixels();

protected:
    const int size;
    CReal *uPointCloud;
    int nPCPoints;
    CReal *uCamProj;
    int height;
    int width;
    CMask *uVisMask;
    CReal *uPixels;
};

class CR_DenseGrid {
public:
    CR_DenseGrid() = default;
    ~CR_DenseGrid() = default;

    CMask* get_dense_grid();
    const CMask* get_dense_grid() const;

    void resize(int vx, int vy, int vz);
    void find_frontiers();
protected:
    thrust::host_vector<CMask> denseGrid;
    int nx;
    int ny;
    int nz;
};

}

#endif //POINTCLOUDUTILS_INCLUDES_HOLEBOUNDARYDETECTION_CUDA_OCCUPANCYMAPBUILDERROUTINES_H
