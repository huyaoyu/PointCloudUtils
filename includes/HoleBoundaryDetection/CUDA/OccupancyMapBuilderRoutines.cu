//
// Created by yaoyu on 5/11/20.
//

#include "HoleBoundaryDetection/CUDA/OccupancyMapBuilderRoutines.h"

const int N_THREADS_PER_BLOCK=256;

using namespace pcu;

__device__
void d_Rx_T(const CReal *xw, const CReal *R, const CReal *T, CReal *xs) {
    CReal xt[3];

    xt[0] = xw[0] - T[0];
    xt[1] = xw[1] - T[1];
    xt[2] = xw[2] - T[2];

    xs[0] = R[0] * xt[0] + R[3] * xt[1] + R[6] * xt[2];
    xs[1] = R[1] * xt[0] + R[4] * xt[1] + R[7] * xt[2];
    xs[2] = R[2] * xt[0] + R[5] * xt[1] + R[8] * xt[2];
}

__device__
void d_sp_2_pixel( const CReal* K, const CReal *xs, CReal *pixel ) {
    pixel[0] = ( K[0] * xs[0] ) / xs[2] + K[2];
    pixel[1] = ( K[4] * xs[1] ) / xs[2] + K[5];
    pixel[2] = 1.0;
}

__global__
void g_update_visibility_mask(
        CReal* pc, int nPc,
        CReal* camProj, int height, int width,
        std::uint8_t* visMask,
        CReal *pixels ) {
    const int index  = blockIdx.x * blockDim.x + threadIdx.x;
    const int stride = blockDim.x * gridDim.x;

    const int sizeMat = 9;
    const int sizeVec = 3;
    const int sizeCP  = 2*sizeMat + sizeVec;

    __shared__ CReal cp[sizeCP];

    // Thread 0 load the data into the shared memory.
    if ( 0 == threadIdx.x ) {
        for ( int i = 0; i < sizeCP; ++i ) {
            cp[i] = camProj[i];
        }
    }

    __syncthreads();

    // Re-map the variables in cp.
    CReal *camK  = cp;
    CReal *camR  = camK + sizeMat;
    CReal *camT  = camR + sizeMat;

    CReal xs[3];
    CReal xw[3];
    CReal pixel[3];

    // Check visibility
    for (int i = index; i < nPc; i+=stride) {
        xw[0] = pc[ i*3 ];
        xw[1] = pc[ i*3 + 1];
        xw[2] = pc[ i*3 + 2];

        d_Rx_T( xw, camR, camT, xs );

        if ( xs[2] <= 0 ) {
            visMask[i] = OCP_MAP_CAM_INVISIBLE; // Invisible.
            continue;
        }

        // Project to the image plane.
        d_sp_2_pixel(camK, xs, pixel);

        if ( pixel[0] < 0 || pixel[0] > width ) {
            visMask[i] = OCP_MAP_CAM_INVISIBLE; // Invisible.
            continue;
        }

        if ( pixel[1] < 0 || pixel[1] > height ) {
            visMask[i] = OCP_MAP_CAM_INVISIBLE; // Invisible.
            continue;
        }

        visMask[i] = OCP_MAP_CAM_VISIBLE; // Visible.

        pixels[ i*3 ]    = pixel[0];
        pixels[ i*3 + 1] = pixel[1];
        pixels[ i*3 + 2] = xs[2];
    }

    __syncthreads();
}

static void copy_2_device( const CReal *from, CReal *to, int n ) {
    for ( int i = 0; i < n; ++i ) {
        to[i] = from[i];
    }
}

CR_VisMask::CR_VisMask(int n)
: size(3*n), nPCPoints(n)
{
    cudaMallocManaged(&uPointCloud, size * sizeof(CReal));

    const int sizeMat = 9;
    const int sizeVec = 3;
    const int sizeCamProj = sizeMat * 2 + sizeVec;
    cudaMallocManaged(&uCamProj, sizeCamProj * sizeof(CReal));

    cudaMallocManaged(&uVisMask, nPCPoints * sizeof(std::uint8_t));
    cudaMallocManaged(&uPixels, size * sizeof(CReal));
}

CR_VisMask::~CR_VisMask() {
    cudaFree(uPixels);
    cudaFree(uVisMask);
    cudaFree(uCamProj);
    cudaFree(uPointCloud);
}

CReal* CR_VisMask::get_u_cam_proj() {
    return uCamProj;
}

void CR_VisMask::set_cam_proj_size(int h, int w) {
    height = h;
    width  = w;
}

void CR_VisMask::copy_point_cloud( const CReal *pPointCloud, int n ) {
    if ( 3*n > size ) {
        std::stringstream ss;
        ss << __FILE__ << ": "<< __LINE__ << ": Point cloud contains too many points. "
           << size << " is reserved but " << n << " is required. ";
        throw std::runtime_error(ss.str());
    } else {
        nPCPoints = n;
    }

    copy_2_device(pPointCloud, uPointCloud, 3*nPCPoints);
}

int CR_VisMask::cr_update_visibility_mask() {
    // CUDA context check.
    auto err = cudaGetLastError();
    if ( cudaSuccess != err )
    {
        std::stringstream ss;
        ss << __FILE__ << ": "<< __LINE__ << ": cudaGetLastError() returns " << err;
        throw std::runtime_error(ss.str());
    }

    // Launch size.
    const int blocks = ( nPCPoints + N_THREADS_PER_BLOCK - 1 ) / N_THREADS_PER_BLOCK;
    g_update_visibility_mask<<<blocks, N_THREADS_PER_BLOCK>>>(
            uPointCloud, nPCPoints,
            uCamProj, height, width,
            uVisMask, uPixels );

    // Wait for the GPU.
    cudaDeviceSynchronize();

    // CUDA context check.
    err = cudaGetLastError();
    if ( cudaSuccess != err )
    {
        std::stringstream ss;
        ss << __FILE__ << ": "<< __LINE__ << ": cudaGetLastError() returns " << err;
        throw std::runtime_error(ss.str());
    }

    return 0;
}

std::uint8_t* CR_VisMask::get_vis_mask() {
    return uVisMask;
}

CReal* CR_VisMask::get_pixels() {
    return uPixels;
}