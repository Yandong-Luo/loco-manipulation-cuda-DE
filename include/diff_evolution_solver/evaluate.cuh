#ifndef CUDAPROCESS_DIFF_EVALUATE_H
#define CUDAPROCESS_DIFF_EVALUATE_H

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <math.h>
#include <cublas_v2.h>
// #ifdef __CUDACC__
// #include <cuda.h>
// #include <cuda_runtime_api.h>
// #endif

#include "diff_evolution_solver/data_type.h"

namespace cudaprocess{

/**
 * (Abandoned) Use for loop to evaluate 
 * One Block with 64 thread to evaluate different 
 * In the future we can use matrix to present the constraint
 */
// template<int T>
// __device__ __forceinline__ void DynamicEvaluation2(CudaEvolveData *evolve, CudaParamClusterData<T> *cluster_data, const float *lambda){
//     float score = 0.0;
//     int idx = threadIdx.x;
    
//     if (idx >= T) return;

//     float constraint_score[2] = {0.f};
    
//     for(int i = 0; i < evolve->num_constraint; ++i){
//         for(int j = 0; j < evolve->num_constraint_variable; ++j){
//             if (j == evolve->num_constraint_variable - 1)   constraint_score[i] += evolve->constraint_para[i][j];
//             else  constraint_score[i] += cluster_data->all_param[idx * CUDA_PARAM_MAX_SIZE + j] * evolve->constraint_para[i][j];
//         }
//     }

//     // objective function
//     for(int i = 0; i < evolve->num_objective_param; ++i){
//         score += evolve->objective_param[i] * cluster_data->all_param[idx * CUDA_PARAM_MAX_SIZE + i];
//     }

//     for (int i = 0; i < evolve->num_constraint; ++i){
//         score += (constraint_score[i] <= 0) ? 0 : fabs(constraint_score[i]) * lambda[i];
//     }

//     cluster_data->fitness[idx] = score;
//     // printf("thread id:%d setting fitness to %f\n", idx, score);
// }

// generate a 64 x dims+1 matrix
template<int T>
__global__ void ConvertClusterToMatrix(CudaEvolveData *evolve, CudaParamClusterData<T> *cluster_data, float *param_matrix){
    if (threadIdx.x > evolve->problem_param.dims) return;
    if (blockIdx.x >= T)    return;
    if(threadIdx.x == evolve->problem_param.dims){
        param_matrix[blockIdx.x * (evolve->problem_param.dims + 1) + threadIdx.x] = 1.0;
        // printf("finish the convert: param[%d] to matrix[%d], value:%f\n", blockIdx.x * CUDA_PARAM_MAX_SIZE + threadIdx.x, blockIdx.x * (evolve->dims + 1) + threadIdx.x, param_matrix[blockIdx.x * (evolve->dims + 1) + threadIdx.x]);
        return;
    }
    if(threadIdx.x >= evolve->problem_param.con_var_dims){
        param_matrix[blockIdx.x * (evolve->problem_param.dims + 1) + threadIdx.x] = floor(cluster_data->all_param[blockIdx.x * CUDA_PARAM_MAX_SIZE + threadIdx.x]);
    }
    else{
        param_matrix[blockIdx.x * (evolve->problem_param.dims + 1) + threadIdx.x] = cluster_data->all_param[blockIdx.x * CUDA_PARAM_MAX_SIZE + threadIdx.x];
    }
    
    // printf("finish the convert: param[%d] to matrix[%d], value:%f\n", blockIdx.x * CUDA_PARAM_MAX_SIZE + threadIdx.x, blockIdx.x * (evolve->dims + 1) + threadIdx.x, param_matrix[blockIdx.x * (evolve->dims + 1) + threadIdx.x]);
}

// generate a dims + 1 x 64 matrix
template<int T>
__global__ void ConvertClusterToMatrix2(CudaEvolveData *evolve, CudaParamClusterData<T> *cluster_data, float *param_matrix, int size){
    if (threadIdx.x > T) return;
    if (blockIdx.x > evolve->problem_param.dims)    return;
    if(blockIdx.x == evolve->problem_param.dims){
        param_matrix[blockIdx.x * size + threadIdx.x] = 1.0;
        // printf("finish the convert: param[%d] to matrix[%d], value:%f\n", blockIdx.x * CUDA_PARAM_MAX_SIZE + threadIdx.x, blockIdx.x * (evolve->dims + 1) + threadIdx.x, param_matrix[blockIdx.x * (evolve->dims + 1) + threadIdx.x]);
        return;
    }
    if(blockIdx.x >= evolve->problem_param.con_var_dims){
        param_matrix[blockIdx.x * size + threadIdx.x] = floor(cluster_data->all_param[blockIdx.x + CUDA_PARAM_MAX_SIZE * threadIdx.x]);
    }
    else{
        param_matrix[blockIdx.x * size + threadIdx.x] = cluster_data->all_param[blockIdx.x + CUDA_PARAM_MAX_SIZE * threadIdx.x];
    }
    
    // printf("finish the convert: param[%d] to matrix[%d], value:%f\n", blockIdx.x * CUDA_PARAM_MAX_SIZE + threadIdx.x, blockIdx.x * (evolve->dims + 1) + threadIdx.x, param_matrix[blockIdx.x * (evolve->dims + 1) + threadIdx.x]);
}


/**
 * Matrix Add operator
 */
__global__ void MatrixAdd(float* quadratic, float* evaluate_score, int size) {
    if (threadIdx.x < size) {
        evaluate_score[threadIdx.x] += quadratic[threadIdx.x];
    }
}

template<int T = CUDA_SOLVER_POP_SIZE>
__global__ void UpdateFitnessBasedMatrix(CudaParamClusterData<T> *cluster_data, float *evaluate_score){
    if (threadIdx.x >= T) return;
    cluster_data->fitness[threadIdx.x] = evaluate_score[threadIdx.x];
}

__device__ __forceinline__ float Interpolation(float x0, float x1, float x, float y0, float y1) {
  if (x <= x0) return y0;
  if (x >= x1) return y1;
  return y0 + (x - x0) / (x1 - x0) * (y1 - y0);
}

__global__ void UpdateLambdaBasedInterpolation(CudaEvolveData *evolve_data, float *lambda_matrix, int epoch){
    lambda_matrix[threadIdx.x] = Interpolation(0, evolve_data->problem_param.max_round, epoch, evolve_data->problem_param.init_lambda, evolve_data->problem_param.max_lambda);
}

__global__ void InequalityMask(float *tmp_score){
    if(tmp_score[threadIdx.x] <= 0.) tmp_score[threadIdx.x] = 0.;
}

}


#endif