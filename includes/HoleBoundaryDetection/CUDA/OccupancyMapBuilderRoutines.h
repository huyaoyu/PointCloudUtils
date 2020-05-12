//
// Created by yaoyu on 5/10/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_HOLEBOUNDARYDETECTION_CUDA_OCCUPANCYMAPBUILDERROUTINES_H
#define POINTCLOUDUTILS_INCLUDES_HOLEBOUNDARYDETECTION_CUDA_OCCUPANCYMAPBUILDERROUTINES_H

#include <cstdint>
#include <iostream>
#include <sstream>

namespace pcu {
enum {
    OCP_MAP_CAM_INVISIBLE = 0,
    OCP_MAP_CAM_VISIBLE
};

typedef float CReal;

class CR_VisMask {
public:
    CR_VisMask(int n);

    ~CR_VisMask();

    CReal *get_u_cam_proj();

    void set_cam_proj_size(int h, int w);

    void copy_point_cloud(const CReal *pPointCloud, int n);

    int cr_update_visibility_mask();

    std::uint8_t *get_vis_mask();

    CReal *get_pixels();

protected:
    const int size;

    CReal *uPointCloud;
    int nPCPoints;
    CReal *uCamProj;
    int height;
    int width;
    std::uint8_t *uVisMask;
    CReal *uPixels;
};

}

#endif //POINTCLOUDUTILS_INCLUDES_HOLEBOUNDARYDETECTION_CUDA_OCCUPANCYMAPBUILDERROUTINES_H
