//
// Created by yaoyu on 5/11/20.
//

#include <fstream>
#include <sstream>

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

#include "OccupancyMapBuilderRoutines.h"

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
        CMask* visMask,
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

    cudaMallocManaged(&uVisMask, nPCPoints * sizeof(CMask));
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

CMask* CR_VisMask::get_vis_mask() {
    return uVisMask;
}

CReal* CR_VisMask::get_pixels() {
    return uPixels;
}

// ============================================================

void CR_DenseGrid::resize(int vx, int vy, int vz) {
    nx = vx;
    ny = vy;
    nz = vz;
    denseGrid.resize(static_cast<std::size_t>(nz) * ny * nx );
    thrust::fill( denseGrid.begin(), denseGrid.end(), OCP_MAP_OCC_UNKNOWN );
    std::cout << "denseGrid.size() = " << denseGrid.size() << std::endl;
}

CMask* CR_DenseGrid::get_dense_grid() {
    return thrust::raw_pointer_cast( denseGrid.data() );
}

typedef struct  {
    int x;
    int y;
    int z;
} DenseGridDim_t;

__global__
void g_find_frontiers( CMask *cmask, DenseGridDim_t dgd ) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z;
    const int strideX = blockDim.x * gridDim.x;
    const int strideY = blockDim.y * gridDim.y;
    const int strideZ = blockDim.z * gridDim.z;

    const int dimXY = dgd.x * dgd.y;

    const int kDimXP  = blockDim.x + 2; // Kernel dimension x padded.
    const int kDimYP  = blockDim.y + 2;
//    const int kDimZP  = blockDim.z + 2;
    const int kDimXYP = kDimXP * kDimYP;
    const int kIdx = (threadIdx.z+1) * kDimXYP + (threadIdx.y+1) * kDimXP + (threadIdx.x+1);

    extern __shared__ CMask shared_mask_array[];

    const int blockXEnd = ( ( dgd.x + strideX - 1 ) / strideX ) * strideX;
    const int blockYEnd = ( ( dgd.y + strideY - 1 ) / strideY ) * strideY;
    const int blockZEnd = ( ( dgd.z + strideZ - 1 ) / strideZ ) * strideZ;

    const int nNeighbors = 26;
    int neighborShift[nNeighbors] = {
            /* Group -1, 9 neighbors. */
            -kDimXYP-kDimXP-1, -kDimXYP-kDimXP, -kDimXYP-kDimXP+1,
            -kDimXYP       -1, -kDimXYP       , -kDimXYP       +1,
            -kDimXYP+kDimXP-1, -kDimXYP+kDimXP, -kDimXYP+kDimXP+1,
            /* Group 0, 8 neighbors. */
            -kDimXP-1, -kDimXP, -kDimXP+1,
                   -1,                 +1,
            +kDimXP+1, +kDimXP, +kDimXP+1,
            /* Group +1, 9 neighbors. */
            +kDimXYP-kDimXP-1, +kDimXYP-kDimXP, +kDimXYP-kDimXP+1,
            +kDimXYP       -1, +kDimXYP       , +kDimXYP       +1,
            +kDimXYP+kDimXP-1, +kDimXYP+kDimXP, +kDimXYP+kDimXP+1
    };
    const int frontierLimit = 2;

    for ( int iz = z; iz < blockZEnd; iz += strideZ ) {
        for ( int iy = y; iy < blockYEnd; iy += strideY ) {
            for ( int ix = x; ix < blockXEnd; ix += strideX ) {
                if ( iz < dgd.z && iy < dgd.y && ix < dgd.x )
                {
                    int idx = iz * dimXY + iy * dgd.x + ix;

                    // Load data to the shared memory.
                    shared_mask_array[kIdx] = cmask[idx];

                    if ( threadIdx.x == 0 ) {
                        int kIdxShift = kIdx - 1;
                        if ( ix != 0 ) {
                            shared_mask_array[kIdxShift] = cmask[idx-1];
                        } else {
                            shared_mask_array[kIdxShift] = OCP_MAP_OCC_PADDING;
                        }
                    } else if ( threadIdx.x == blockDim.x - 1 ) {
                        int kIdxShift = kIdx + 1;
                        if ( ix != dgd.x - 1 ) {
                            shared_mask_array[kIdxShift] = cmask[idx+1];
                        } else {
                            shared_mask_array[kIdxShift] = OCP_MAP_OCC_PADDING;
                        }
                    } else if ( ix == dgd.x - 1 ) {
                        shared_mask_array[kIdx + 1] = OCP_MAP_OCC_PADDING;
                    }

                    if ( threadIdx.y == 0 ) {
                        int kIdxShift = kIdx - kDimXP;
                        if ( iy != 0 ) {
                            shared_mask_array[ kIdxShift ] = cmask[ idx - dgd.x ];
                        } else {
                            shared_mask_array[ kIdxShift ] = OCP_MAP_OCC_PADDING;
                        }
                    } else if ( threadIdx.y == blockDim.y - 1 ) {
                        int kIdxShift = kIdx + kDimXP;
                        if ( iy != dgd.y - 1 ) {
                            shared_mask_array[ kIdxShift ] = cmask[ idx + dgd.x ];
                        } else {
                            shared_mask_array[ kIdxShift ] = OCP_MAP_OCC_PADDING;
                        }
                    } else if ( iy == dgd.y - 1 ) {
                        shared_mask_array[ kIdx + kDimXP ] = OCP_MAP_OCC_PADDING;
                    }

                    if ( threadIdx.z == 0 ) {
                        int kIdxShift = kIdx - kDimXYP;
                        if ( iz != 0 ) {
                            shared_mask_array[ kIdxShift ] = cmask[ idx - dimXY ];
                        } else {
                            shared_mask_array[ kIdxShift ] = OCP_MAP_OCC_PADDING;
                        }
                    } else if ( threadIdx.z == blockDim.z - 1 ) {
                        int kIdxShift = kIdx + kDimXYP;
                        if ( iz != dgd.z - 1 ) {
                            shared_mask_array[ kIdxShift ] = cmask[ idx + dimXY ];
                        } else {
                            shared_mask_array[ kIdxShift ] = OCP_MAP_OCC_PADDING;
                        }
                    } else if ( iz == dgd.z - 1 ) {
                        shared_mask_array[ kIdx + kDimXYP ] = OCP_MAP_OCC_PADDING;
                    }

                    __syncthreads();

                    // Check frontier.
                    if ( shared_mask_array[kIdx] == OCP_MAP_OCC_FREE ) {
                        int count = 0;
                        for ( int k = 0; k < nNeighbors; ++k ) {
                            if ( shared_mask_array[ kIdx + neighborShift[k] ] == OCP_MAP_OCC_UNKNOWN ) {
                                count++;
                                if ( count == frontierLimit ) {
                                    cmask[idx] = OCP_MAP_OCC_FRONTIER;
                                    break;
                                }
                            }
                        }
                    }

                    __syncthreads();
                } else {
                    // Load data to the shared memory.
                    shared_mask_array[kIdx] = OCP_MAP_OCC_PADDING;

                    if ( threadIdx.x == 0 ) {
                        int kIdxShift = kIdx - 1;
                        shared_mask_array[kIdxShift] = OCP_MAP_OCC_PADDING;
                    } else if ( threadIdx.x == blockDim.x - 1 ) {
                        int kIdxShift = kIdx + 1;
                        shared_mask_array[kIdxShift] = OCP_MAP_OCC_PADDING;
                    } else if ( ix == dgd.x - 1 ) {
                        shared_mask_array[kIdx + 1] = OCP_MAP_OCC_PADDING;
                    }

                    if ( threadIdx.y == 0 ) {
                        int kIdxShift = kIdx - kDimXP;
                        shared_mask_array[ kIdxShift ] = OCP_MAP_OCC_PADDING;
                    } else if ( threadIdx.y == blockDim.y - 1 ) {
                        int kIdxShift = kIdx + kDimXP;
                        shared_mask_array[ kIdxShift ] = OCP_MAP_OCC_PADDING;
                    } else if ( iy == dgd.y - 1 ) {
                        shared_mask_array[ kIdx + kDimXP ] = OCP_MAP_OCC_PADDING;
                    }

                    if ( threadIdx.z == 0 ) {
                        int kIdxShift = kIdx - kDimXYP;
                        shared_mask_array[ kIdxShift ] = OCP_MAP_OCC_PADDING;
                    } else if ( threadIdx.z == blockDim.z - 1 ) {
                        int kIdxShift = kIdx + kDimXYP;
                        shared_mask_array[ kIdxShift ] = OCP_MAP_OCC_PADDING;
                    } else if ( iz == dgd.z - 1 ) {
                        shared_mask_array[ kIdx + kDimXYP ] = OCP_MAP_OCC_PADDING;
                    }

                    __syncthreads();
                    __syncthreads();
                }
            }
        }
    }
}

void CR_DenseGrid::find_frontiers() {
    // Copy host memory to device.
    thrust::device_vector<CMask> dvDenseGrid = denseGrid;

    DenseGridDim_t dgd;
    dgd.x = nx;
    dgd.y = ny;
    dgd.z = nz;

    // CUDA context check.
    auto err = cudaGetLastError();
    if ( cudaSuccess != err )
    {
        std::stringstream ss;
        ss << __FILE__ << ": "<< __LINE__ << ": cudaGetLastError() returns " << err;
        throw std::runtime_error(ss.str());
    }

    // Launch size.
    const int blockDimX = 8;
    const int blockDimY = 8;
    const int blockDimZ = 8;
    dim3 workingBlockDim(blockDimX, blockDimY, blockDimZ);
    dim3 workingGridDim(8, 8, 8);
    const std::size_t sharedMemSize = ( blockDimZ+2 ) * (blockDimY+2) * (blockDimX+2) * sizeof(CMask);
    g_find_frontiers<<<workingGridDim, workingBlockDim, sharedMemSize>>>(
            thrust::raw_pointer_cast( dvDenseGrid.data() ), dgd );

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

    // Copy back from the device.
    denseGrid = dvDenseGrid;
}
