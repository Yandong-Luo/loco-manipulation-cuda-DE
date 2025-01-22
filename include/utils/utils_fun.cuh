#ifndef CUDA_PROCESS_UTILS_FUNCTION_H
#define CUDA_PROCESS_UTILS_FUNCTION_H

#include "diff_evolution_solver/data_type.h"

namespace cudaprocess{

template <int T = CUDA_SOLVER_POP_SIZE, int PARA_SIZE>
__global__ void ParaFindMax2(CudaParamClusterData<PARA_SIZE> *a) {
  __shared__ int idx_list[4];
  __shared__ float value_list[4];
  // if (threadIdx.x > 64) return;
  float value = a->fitness[threadIdx.x];
  int idx = threadIdx.x;

  float tmp_f;
  int tmp_idx;
  tmp_f = __shfl_down_sync(0xffffffff, value, 16);
  tmp_idx = __shfl_down_sync(0xffffffff, idx, 16);
  if (tmp_f < value) {
    value = tmp_f;
    idx = tmp_idx;
  }
  tmp_f = __shfl_down_sync(0xffffffff, value, 8);
  tmp_idx = __shfl_down_sync(0xffffffff, idx, 8);
  if (tmp_f < value) {
    value = tmp_f;
    idx = tmp_idx;
  }
  tmp_f = __shfl_down_sync(0xffffffff, value, 4);
  tmp_idx = __shfl_down_sync(0xffffffff, idx, 4);
  if (tmp_f < value) {
    value = tmp_f;
    idx = tmp_idx;
  }
  tmp_f = __shfl_down_sync(0xffffffff, value, 2);
  tmp_idx = __shfl_down_sync(0xffffffff, idx, 2);
  if (tmp_f < value) {
    value = tmp_f;
    idx = tmp_idx;
  }
  tmp_f = __shfl_down_sync(0xffffffff, value, 1);
  tmp_idx = __shfl_down_sync(0xffffffff, idx, 1);
  if (tmp_f < value) {
    value = tmp_f;
    idx = tmp_idx;
  }

  if ((threadIdx.x & 31) == 0) {
    idx_list[threadIdx.x >> 5] = idx;
    value_list[threadIdx.x >> 5] = value;
  }
  __syncthreads();

  if (T == 128) {
    if (threadIdx.x < 4) {
      value = value_list[threadIdx.x];
      idx = idx_list[threadIdx.x];
      tmp_f = __shfl_down_sync(0x0000000f, value, 2);
      tmp_idx = __shfl_down_sync(0x0000000f, idx, 2);
      if (tmp_f < value) {
        value = tmp_f;
        idx = tmp_idx;
      }
      tmp_f = __shfl_down_sync(0x0000000f, value, 1);
      tmp_idx = __shfl_down_sync(0x0000000f, idx, 1);
      if (tmp_f < value) {
        value = tmp_f;
        idx = tmp_idx;
      }
    }
  } else if (T == 64) {
    if (threadIdx.x < 2) {
      value = value_list[threadIdx.x];
      idx = idx_list[threadIdx.x];
      tmp_f = __shfl_down_sync(0x00000003, value, 1);
      tmp_idx = __shfl_down_sync(0x00000003, idx, 1);
      if (tmp_f < value) {
        value = tmp_f;
        idx = tmp_idx;
      }
    }
  }

  idx = __shfl_sync(0x0000ffff, idx, 0);
  if (threadIdx.x < 16) {
    float para = a->all_param[idx * CUDA_PARAM_MAX_SIZE + threadIdx.x];
    a->all_param[idx * CUDA_PARAM_MAX_SIZE + threadIdx.x] = a->all_param[threadIdx.x];
    a->all_param[threadIdx.x] = para;
    if (threadIdx.x == 0) {
      float f = a->fitness[idx];
      a->fitness[idx] = a->fitness[0];
      a->fitness[0] = f;
    }
  }
}

/**
 * extractDiagonal
 */
__global__ void extractDiagonal(float* input, float* output, int size) {
    if (threadIdx.x < size) {
        output[threadIdx.x] = input[threadIdx.x * size + threadIdx.x];  // 提取对角线元素
    }
}


}

#endif