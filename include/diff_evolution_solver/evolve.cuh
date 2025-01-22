#ifndef CUDAPROCESS_DIFF_EVOLVE_H
#define CUDAPROCESS_DIFF_EVOLVE_H

#include <cuda.h>
#include <cuda_runtime_api.h>

// #ifdef __CUDACC__
// #include <cuda.h>
// #include <cuda_runtime_api.h>
// #endif

#include "diff_evolution_solver/data_type.h"

namespace cudaprocess{
    template <int T = CUDA_SOLVER_POP_SIZE>
    __global__ void DuplicateBestAndReorganize(int epoch, CudaParamClusterData<CUDA_SOLVER_POP_SIZE*3> *old_param, int copy_num){
        // if(epoch > 0)   return;
        int size = 2 * T;

        if (epoch == 0){
            size = T;
        }

        int param_id = blockIdx.x;
        int sol_id = threadIdx.x;

        float param;
        float best_fitness;
        
        if (sol_id < size + copy_num){
            if (sol_id <= copy_num){
                // For the first copy_num solutions, use the parameters of the best solution (sol_id=0)
                param = old_param->all_param[param_id];
                if (param_id == 0)  best_fitness = old_param->fitness[0];   // copy the best fitness
                
            }
            else{
                // for the remaining solution, use the original parameters
                param = old_param->all_param[(sol_id - copy_num) * CUDA_PARAM_MAX_SIZE + param_id];
                if(param_id == 0){
                    best_fitness = old_param->fitness[sol_id - copy_num];
                }
            }
        }
        // wait for all thread finish above all copy step
        __syncthreads();
        // reorganize
        
        if(sol_id < size + copy_num){
            old_param->all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id] = param;
            if (param_id == 0){
                old_param->fitness[sol_id] = best_fitness;
                if(sol_id == 0){
                    old_param->len = size + copy_num;
                }
            }
        }
    }

    template <CudaEvolveType SearchType = CudaEvolveType::GLOBAL>
    __global__ void CudaEvolveProcess(int epoch, CudaParamClusterData<CUDA_SOLVER_POP_SIZE*3> *old_param, CudaParamClusterData<CUDA_SOLVER_POP_SIZE> *new_param, float *uniform_data,
                                      float *normal_data, CudaEvolveData *evolve_data, int pop_size, float eliteRatio){
        int dims = evolve_data->problem_param.dims, con_dims = evolve_data->problem_param.con_var_dims;
        // int int_dims = evolve_data->problem_param.int_var_dims;

        int sol_idx = blockIdx.x;
        // int param_idx = threadIdx.x;
        int UsingEliteStrategy = 0;
        int guideEliteIdx = 0;

        float crossover, scale_f, scale_f1;

        int mutationStartDim;

        float origin_param, best_origin_param, result_param;

        // Avoid using the same random number in the same place as other functions
        int normal_rnd_evolve_pos = epoch * CUDA_SOLVER_POP_SIZE * 3 + sol_idx * 3;
        int uniform_rnd_evolve_pos = epoch * CUDA_SOLVER_POP_SIZE * 38 + sol_idx * 38;

        int selected_parent_idx;
        int parent_param_idx[3], sorted_parent_param_idx[3];
        if (threadIdx.x < 3){
            int total_len = old_param->len;
            // Initialize 3 index that increases with threadIdx.x to ensure that subsequent sorting does not require too frequent operations
            selected_parent_idx = min((int)floor(uniform_data[uniform_rnd_evolve_pos + threadIdx.x] * (total_len - threadIdx.x - 1)), total_len - threadIdx.x - 2);
        }
        // Due to the selected_parent_idx was calculated in thread 0-2, we need let thread 0 know all result of selected_parent_idx for future sorting part
        // Use warp shuffle to share indices between threads
        parent_param_idx[0] = selected_parent_idx;
        parent_param_idx[1] = __shfl_sync(0x0000000f, selected_parent_idx, 1);
        parent_param_idx[2] = __shfl_sync(0x0000000f, selected_parent_idx, 2);

        if(threadIdx.x == 0){
            // sorting part
            // Ensure that the parent individuals selected in the differential evolution algorithm are different and non-repetitive

            for (int i = 0; i < 3; ++i) {
                sorted_parent_param_idx[i] = parent_param_idx[i];
                for (int j = 0; j < i; ++j) {
                    if (parent_param_idx[j] <= sorted_parent_param_idx[i]) sorted_parent_param_idx[i]++;
                }
                parent_param_idx[i] = sorted_parent_param_idx[i];
                // 插入排序
                for (int j = i; j > 0; j--) {
                    if (parent_param_idx[j] < parent_param_idx[j - 1]) {
                        int tmp = parent_param_idx[j];
                        parent_param_idx[j] = parent_param_idx[j - 1];
                        parent_param_idx[j - 1] = tmp;
                    }
                }
                if (sorted_parent_param_idx[i] >= blockIdx.x)   sorted_parent_param_idx[i]++;
            }

            scale_f = min(1.f, max(normal_data[normal_rnd_evolve_pos] * 0.01f + evolve_data->hist_lshade_param.scale_f, 0.1));
            scale_f1 = min(1.f, max(normal_data[normal_rnd_evolve_pos + 1] * 0.01f + evolve_data->hist_lshade_param.scale_f, 0.1));
            crossover = min(1.f, max(normal_data[normal_rnd_evolve_pos + 2] * 0.01f + evolve_data->hist_lshade_param.Cr, 0.1));

            mutationStartDim = min((int)floor(uniform_data[uniform_rnd_evolve_pos + 3] * dims), dims - 1);

            int num_top = int(pop_size * evolve_data->problem_param.top_ratio) + 1;
            // The index of an individual randomly selected from the top proportion of high-quality individuals in the population
            // Due to uniform_rnd_evolve_pos to uniform_rnd_evolve_pos + 3 have been used for parent_param_idx and mutationStartDim. So, starting from 4
            guideEliteIdx = max(0, min((num_top - 1), (int)floor(uniform_data[uniform_rnd_evolve_pos + 4] * num_top)));

            if (sol_idx * 1.f < pop_size * eliteRatio)  UsingEliteStrategy = 1;
        }
        // make sure all parameter have been calculated
        __syncwarp();

        // 使用 warp shuffle 广播这些值给其他线程
        scale_f = __shfl_sync(0x0000ffff, scale_f, 0);
        scale_f1 = __shfl_sync(0x0000ffff, scale_f1, 0);
        crossover = __shfl_sync(0x0000ffff, crossover, 0);
        UsingEliteStrategy = __shfl_sync(0x0000ffff, UsingEliteStrategy, 0);
        guideEliteIdx = __shfl_sync(0x0000ffff, guideEliteIdx, 0);

        int parent1_idx = sorted_parent_param_idx[0], parent2_idx = sorted_parent_param_idx[1];
        int mutant_idx = sorted_parent_param_idx[2];

        parent1_idx = __shfl_sync(0x0000ffff, parent1_idx, 0);
        parent2_idx = __shfl_sync(0x0000ffff, parent2_idx, 0);
        mutant_idx = __shfl_sync(0x0000ffff, mutant_idx, 0);

        // Other threads need to obtain the broadcasted data
        sorted_parent_param_idx[0] = parent1_idx;
        sorted_parent_param_idx[1] = parent2_idx;
        sorted_parent_param_idx[2] = mutant_idx;
        
        // int totalsize = old_param->len;
        // check the random_idx valid or not
        if (sorted_parent_param_idx[0] >= old_param->len || sorted_parent_param_idx[1] >= old_param->len || sorted_parent_param_idx[2] >= old_param->len || old_param->len >= pop_size * 2 + 10) {
            printf("wft: %d, %d, %d, total len: %d\n", sorted_parent_param_idx[0], sorted_parent_param_idx[1], sorted_parent_param_idx[2], old_param->len);
        }

        // record the parameter for mutant
        float mutant_param[3];
        mutant_param[0] = old_param->all_param[sorted_parent_param_idx[0] * CUDA_PARAM_MAX_SIZE + threadIdx.x];
        mutant_param[1] = old_param->all_param[sorted_parent_param_idx[1] * CUDA_PARAM_MAX_SIZE + threadIdx.x];
        mutant_param[2] = old_param->all_param[sorted_parent_param_idx[2] * CUDA_PARAM_MAX_SIZE + threadIdx.x];

        if(UsingEliteStrategy){
            // Use the parameters corresponding to the current individual idx as the basis
            origin_param = old_param->all_param[sol_idx * CUDA_PARAM_MAX_SIZE + threadIdx.x];
        } else{
            // Use the best parameter as the basis
            origin_param = old_param->all_param[threadIdx.x];
        }

        // initial result param
        result_param = origin_param;

        // load the random param from top level as best
        best_origin_param = old_param->all_param[guideEliteIdx * CUDA_PARAM_MAX_SIZE + threadIdx.x];

        float f = (threadIdx.x >= con_dims) ? scale_f1 : scale_f;
        if (SearchType == CudaEvolveType::GLOBAL){
            float mutant_prob = uniform_data[uniform_rnd_evolve_pos + 5 + threadIdx.x];

            // initial the firstMutationDimIdx by last one
            int firstMutationDimIdx = pop_size;

            // crossover select
            if(mutant_prob > (UsingEliteStrategy ? crossover : 0.9f) && threadIdx.x < dims){
                firstMutationDimIdx = threadIdx.x;
            }

            // parallel reduce
            int tmp_idx = __shfl_down_sync(0x0000ffff, firstMutationDimIdx, 8);
            firstMutationDimIdx = min(tmp_idx, firstMutationDimIdx);
            tmp_idx = __shfl_down_sync(0x0000ffff, firstMutationDimIdx, 4);
            firstMutationDimIdx = min(tmp_idx, firstMutationDimIdx);
            tmp_idx = __shfl_down_sync(0x0000ffff, firstMutationDimIdx, 2);
            firstMutationDimIdx = min(tmp_idx, firstMutationDimIdx);
            tmp_idx = __shfl_down_sync(0x0000ffff, firstMutationDimIdx, 1);
            firstMutationDimIdx = min(tmp_idx, firstMutationDimIdx);
            // let all thread know the firstMutationDimIdx
            firstMutationDimIdx = __shfl_sync(0x0000ffff, firstMutationDimIdx, 0);

            if (threadIdx.x < dims){
                bool isInMutationWindow = true;
                if (firstMutationDimIdx < dims){
                    int step = threadIdx.x - mutationStartDim;
                    if(step < 0)    step += dims;
                    if(step > firstMutationDimIdx){
                        isInMutationWindow = false;
                    }
                }

                if(isInMutationWindow){
                    if(UsingEliteStrategy){
                        result_param = origin_param + f * (best_origin_param + mutant_param[0] - origin_param - mutant_param[1]);
                    }
                    else{
                        result_param = origin_param + 0.8f * (mutant_param[0] - mutant_param[3]);
                    }
                }
            }
        }

        if (threadIdx.x < dims){
            float lower_bound = evolve_data->lower_bound[threadIdx.x];
            float upper_bound = evolve_data->upper_bound[threadIdx.x];

            if(result_param < lower_bound || result_param > upper_bound){
                result_param = uniform_data[uniform_rnd_evolve_pos + 24 + threadIdx.x] * (upper_bound - lower_bound) + lower_bound;
            }
        }
        new_param->all_param[sol_idx * CUDA_PARAM_MAX_SIZE + threadIdx.x] = result_param;

        if (threadIdx.x == 0) {
            reinterpret_cast<float3 *>(new_param->lshade_param)[sol_idx] = float3{scale_f, scale_f1, crossover};
        }

    }

    __device__ __forceinline__ void BitonicWarpCompare(float &param, float &fitness, int lane_mask){
        float mapping_param = __shfl_xor_sync(0xffffffff, param, lane_mask);
        float mapping_fitness = __shfl_xor_sync(0xffffffff, fitness, lane_mask);
        // determine current sort order is increase (1.0) or decrease (-1.0)
        float sortOrder = (threadIdx.x > (threadIdx.x ^ lane_mask)) ? -1.0 : 1.0;

        if(sortOrder * (mapping_fitness - fitness) < 0.f){
            param = mapping_param;
            fitness = mapping_fitness;
        }
    }

    /**
     * Sort the first 64 of old param (0 - 63)
     */
    template <int T=CUDA_SOLVER_POP_SIZE>
    __device__ __forceinline__ void SortCurrentParamBasedBitonic(float *all_param, float *all_fitness){
        // each block have a share memory
        __shared__ float sm_sorted_fitness[T];
        __shared__ float sm_sorted_param[T];
        int param_id = blockIdx.x;
        int sol_id = threadIdx.x;
        float current_param = all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id];
        float current_fitness = all_fitness[sol_id];

        int compare_idx;
        float mapping_param, mapping_fitness, sortOrder;

        if (threadIdx.x < T){
            // Sort the contents of 32 threads in a warp based on Bitonic merge sort. Implement detail is the alternative representation of https://en.wikipedia.org/wiki/Bitonic_sorter
            BitonicWarpCompare(current_param, current_fitness, 1);

            BitonicWarpCompare(current_param, current_fitness, 3);
            BitonicWarpCompare(current_param, current_fitness, 1);

            BitonicWarpCompare(current_param, current_fitness, 7);
            BitonicWarpCompare(current_param, current_fitness, 2);
            BitonicWarpCompare(current_param, current_fitness, 1);

            BitonicWarpCompare(current_param, current_fitness, 15);
            BitonicWarpCompare(current_param, current_fitness, 4);
            BitonicWarpCompare(current_param, current_fitness, 2);
            BitonicWarpCompare(current_param, current_fitness, 1);

            // above all finish the sorting 16 threads in Warp, continue to finish 2 group of 16 threads
            BitonicWarpCompare(current_param, current_fitness, 31);
            BitonicWarpCompare(current_param, current_fitness, 8);
            BitonicWarpCompare(current_param, current_fitness, 4);
            BitonicWarpCompare(current_param, current_fitness, 2);
            BitonicWarpCompare(current_param, current_fitness, 1);

            // above all finsh the sort for each warp, continue to finish the sort between different warp by share memory.
            // record the warp sorting result to share memory
            sm_sorted_param[sol_id] = current_param;
            sm_sorted_fitness[sol_id] = current_fitness;
        }
        
        // Wait for all thread finish above computation
        __syncthreads();

        if(threadIdx.x < T){
            // if T == 64 (we have 2 warp), we just need to compare these 2 warp by share memory.
            // Otherwise, we need to modify the following code
            compare_idx = sol_id ^ 63;
            mapping_param = sm_sorted_param[compare_idx];
            mapping_fitness = sm_sorted_fitness[compare_idx];

            sortOrder = (threadIdx.x > (threadIdx.x ^ 63)) ? -1.0 : 1.0;

            if(sortOrder * (mapping_fitness - current_fitness) < 0.f){
                current_param = mapping_param;
                current_fitness = mapping_fitness;
            }
        }
        
        // Wait for the sort between two warp finish
        __syncthreads();

        if(threadIdx.x < CUDA_SOLVER_POP_SIZE){
            // Now, we can come back to the sorting in the warp
            BitonicWarpCompare(current_param, current_fitness, 16);
            BitonicWarpCompare(current_param, current_fitness, 8);
            BitonicWarpCompare(current_param, current_fitness, 4);
            BitonicWarpCompare(current_param, current_fitness, 2);
            BitonicWarpCompare(current_param, current_fitness, 1);

            // above all finish all sorting for fitness and param
            if (blockIdx.x < CUDA_PARAM_MAX_SIZE){
                all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id] = current_param;
                // printf("======================== Update sorted param for solution id:%d\n", threadIdx.x);
            }
            if (blockIdx.x == 0)    all_fitness[threadIdx.x] = current_fitness;
        } 
    }

    /**
     * Sort the delete part of old param (64 - 127)
     */
    template <int T=CUDA_SOLVER_POP_SIZE> // T is the pop size
    __global__ void SortDeleteParamBasedBitonic(float *all_param, float *all_fitness){
        if (all_param == nullptr || all_fitness == nullptr) return;
        // each block have a share memory
        __shared__ float sm_sorted_fitness[T * 2];
        __shared__ float sm_sorted_param[T * 2];
        int param_id = blockIdx.x;
        int sol_id = threadIdx.x;
        float current_param = all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id + T];
        float current_fitness = all_fitness[sol_id + T];

        int compare_idx;
        float mapping_param, mapping_fitness, sortOrder;

        if (threadIdx.x >= T){
            // Sort the contents of 32 threads in a warp based on Bitonic merge sort. Implement detail is the alternative representation of https://en.wikipedia.org/wiki/Bitonic_sorter
            BitonicWarpCompare(current_param, current_fitness, 1);

            BitonicWarpCompare(current_param, current_fitness, 3);
            BitonicWarpCompare(current_param, current_fitness, 1);

            BitonicWarpCompare(current_param, current_fitness, 7);
            BitonicWarpCompare(current_param, current_fitness, 2);
            BitonicWarpCompare(current_param, current_fitness, 1);

            BitonicWarpCompare(current_param, current_fitness, 15);
            BitonicWarpCompare(current_param, current_fitness, 4);
            BitonicWarpCompare(current_param, current_fitness, 2);
            BitonicWarpCompare(current_param, current_fitness, 1);

            // above all finish the sorting 16 threads in Warp, continue to finish 2 group of 16 threads
            BitonicWarpCompare(current_param, current_fitness, 31);
            BitonicWarpCompare(current_param, current_fitness, 8);
            BitonicWarpCompare(current_param, current_fitness, 4);
            BitonicWarpCompare(current_param, current_fitness, 2);
            BitonicWarpCompare(current_param, current_fitness, 1);

            // above all finsh the sort for each warp, continue to finish the sort between different warp by share memory.
            // record the warp sorting result to share memory
            sm_sorted_param[sol_id ] = current_param;
            sm_sorted_fitness[sol_id] = current_fitness;
        }
        
        // Wait for all thread finish above computation
        __syncthreads();

        if (threadIdx.x >= T)
        {
            compare_idx = sol_id ^ 63;
            mapping_param = sm_sorted_param[compare_idx];
            mapping_fitness = sm_sorted_fitness[compare_idx];

            sortOrder = (threadIdx.x > (threadIdx.x ^ 63)) ? -1.0 : 1.0;

            if(sortOrder * (mapping_fitness - current_fitness) < 0.f){
                current_param = mapping_param;
                current_fitness = mapping_fitness;
            }
        }
        
        
        // Wait for the sort between two warp finish
        __syncthreads();
        if(threadIdx.x >= T){
            // Now, we can come back to the sorting in the warp
            BitonicWarpCompare(current_param, current_fitness, 16);
            BitonicWarpCompare(current_param, current_fitness, 8);
            BitonicWarpCompare(current_param, current_fitness, 4);
            BitonicWarpCompare(current_param, current_fitness, 2);
            BitonicWarpCompare(current_param, current_fitness, 1);

            sm_sorted_param[sol_id ] = current_param;
            sm_sorted_fitness[sol_id] = current_fitness;
        }
        
        __syncthreads();
        if(threadIdx.x >= T){
            compare_idx = threadIdx.x ^ 127;
            mapping_param = sm_sorted_param[compare_idx];
            mapping_fitness = sm_sorted_fitness[compare_idx];
            sortOrder = (threadIdx.x > compare_idx) ? -1.f : 1.f;

            if (sortOrder * (mapping_fitness - current_fitness) < 0.f) {
                current_fitness = mapping_fitness;
                current_param = mapping_param;
                sm_sorted_fitness[threadIdx.x] = current_fitness;
                sm_sorted_param[threadIdx.x] = current_param;
            }
        }
        
        __syncthreads();
        if(threadIdx.x >= T){
            compare_idx = threadIdx.x ^ 32;
            mapping_fitness = sm_sorted_fitness[compare_idx];
            mapping_param = sm_sorted_param[compare_idx];
            sortOrder = (threadIdx.x > compare_idx) ? -1.f : 1.f;
            if (sortOrder * (mapping_fitness - current_fitness) < 0.f) {
                current_fitness = mapping_fitness;
                current_param = mapping_param;
            }
            BitonicWarpCompare(current_param, current_fitness, 16);
            BitonicWarpCompare(current_param, current_fitness, 8);
            BitonicWarpCompare(current_param, current_fitness, 4);
            BitonicWarpCompare(current_param, current_fitness, 2);
            BitonicWarpCompare(current_param, current_fitness, 1);

            // above all finish all sorting for fitness and param
            if (blockIdx.x < CUDA_PARAM_MAX_SIZE){
                all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id + T] = current_param;
                // printf("======================== Update sorted param for solution id:%d\n", threadIdx.x);
            }
            if (blockIdx.x == 0)    all_fitness[threadIdx.x + T] = current_fitness;
        }
    }

    /**
     * Sort the current parameter and delete part of old param (pop_size:64 0 - 127) or (pop_size:128 0 - 256)
     */
    template<int T = CUDA_SOLVER_POP_SIZE>// T is the pop size
    __device__ __forceinline__ void SortOldParamBasedBitonic(float *all_param, float *all_fitness){
        if (all_param == nullptr || all_fitness == nullptr) return;
        // if (threadIdx.x >= T)   return;
        // each block have a share memory
        __shared__ float sm_sorted_fitness[2*T];
        __shared__ float sm_sorted_param[2*T];
        int param_id = blockIdx.x;
        int sol_id = threadIdx.x;
        float current_param;
        float current_fitness;

        current_param = all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id];
        current_fitness = all_fitness[sol_id];
        
        int compare_idx;
        float mapping_param, mapping_fitness, sortOrder;

        // Sort the contents of 32 threads in a warp based on Bitonic merge sort. Implement detail is the alternative representation of https://en.wikipedia.org/wiki/Bitonic_sorter
        BitonicWarpCompare(current_param, current_fitness, 1);

        BitonicWarpCompare(current_param, current_fitness, 3);
        BitonicWarpCompare(current_param, current_fitness, 1);

        BitonicWarpCompare(current_param, current_fitness, 7);
        BitonicWarpCompare(current_param, current_fitness, 2);
        BitonicWarpCompare(current_param, current_fitness, 1);

        BitonicWarpCompare(current_param, current_fitness, 15);
        BitonicWarpCompare(current_param, current_fitness, 4);
        BitonicWarpCompare(current_param, current_fitness, 2);
        BitonicWarpCompare(current_param, current_fitness, 1);

        // above all finish the sorting 16 threads in Warp, continue to finish 2 group of 16 threads
        BitonicWarpCompare(current_param, current_fitness, 31);
        BitonicWarpCompare(current_param, current_fitness, 8);
        BitonicWarpCompare(current_param, current_fitness, 4);
        BitonicWarpCompare(current_param, current_fitness, 2);
        BitonicWarpCompare(current_param, current_fitness, 1);

        // above all finsh the sort for each warp, continue to finish the sort between different warp by share memory.
        // record the warp sorting result to share memory
        sm_sorted_param[sol_id ] = current_param;
        sm_sorted_fitness[sol_id] = current_fitness;
        
        // Wait for all thread finish above computation
        __syncthreads();

        if (T == 64){
            compare_idx = sol_id ^ 63;
            mapping_param = sm_sorted_param[compare_idx];
            mapping_fitness = sm_sorted_fitness[compare_idx];

            sortOrder = (threadIdx.x > (threadIdx.x ^ 63)) ? -1.0 : 1.0;

            if(sortOrder * (mapping_fitness - current_fitness) < 0.f){
                current_param = mapping_param;
                current_fitness = mapping_fitness;
            }
            
            // Wait for the sort between two warp finish
            __syncthreads();
            // Now, we can come back to the sorting in the warp
            BitonicWarpCompare(current_param, current_fitness, 16);
            BitonicWarpCompare(current_param, current_fitness, 8);
            BitonicWarpCompare(current_param, current_fitness, 4);
            BitonicWarpCompare(current_param, current_fitness, 2);
            BitonicWarpCompare(current_param, current_fitness, 1);
        }
        else if(T == 128){
            sm_sorted_param[sol_id ] = current_param;
            sm_sorted_fitness[sol_id] = current_fitness;
            compare_idx = threadIdx.x ^ 127;
            mapping_param = sm_sorted_param[compare_idx];
            mapping_fitness = sm_sorted_fitness[compare_idx];
            sortOrder = (threadIdx.x > compare_idx) ? -1.f : 1.f;

            if (sortOrder * (mapping_fitness - current_fitness) < 0.f) {
                current_fitness = mapping_fitness;
                current_param = mapping_param;
                sm_sorted_fitness[threadIdx.x] = current_fitness;
                sm_sorted_param[threadIdx.x] = current_param;
            }
        
            __syncthreads();
            
            compare_idx = threadIdx.x ^ 32;
            mapping_fitness = sm_sorted_fitness[compare_idx];
            mapping_param = sm_sorted_param[compare_idx];
            sortOrder = (threadIdx.x > compare_idx) ? -1.f : 1.f;
            if (sortOrder * (mapping_fitness - current_fitness) < 0.f) {
                current_fitness = mapping_fitness;
                current_param = mapping_param;
            }
            BitonicWarpCompare(current_param, current_fitness, 16);
            BitonicWarpCompare(current_param, current_fitness, 8);
            BitonicWarpCompare(current_param, current_fitness, 4);
            BitonicWarpCompare(current_param, current_fitness, 2);
            BitonicWarpCompare(current_param, current_fitness, 1);
        }
        else if(T == 256){
            // 记录当前warp排序结果到共享内存
            sm_sorted_param[sol_id] = current_param;
            sm_sorted_fitness[sol_id] = current_fitness;
            
            // 第一次大规模比较：与255异或（整体排序）
            compare_idx = threadIdx.x ^ 255;
            mapping_param = sm_sorted_param[compare_idx];
            mapping_fitness = sm_sorted_fitness[compare_idx];
            sortOrder = (threadIdx.x > compare_idx) ? -1.f : 1.f;

            if (sortOrder * (mapping_fitness - current_fitness) < 0.f) {
                current_fitness = mapping_fitness;
                current_param = mapping_param;
                sm_sorted_fitness[threadIdx.x] = current_fitness;
                sm_sorted_param[threadIdx.x] = current_param;
            }
            
            __syncthreads();
            
            // 第二次大规模比较：与64异或（128个元素分组排序）
            compare_idx = threadIdx.x ^ 64;
            mapping_fitness = sm_sorted_fitness[compare_idx];
            mapping_param = sm_sorted_param[compare_idx];
            sortOrder = (threadIdx.x > compare_idx) ? -1.f : 1.f;
            
            if (sortOrder * (mapping_fitness - current_fitness) < 0.f) {
                current_fitness = mapping_fitness;
                current_param = mapping_param;
                sm_sorted_fitness[threadIdx.x] = current_fitness;
                sm_sorted_param[threadIdx.x] = current_param;
            }
            
            __syncthreads();
            
            // 第三次大规模比较：与32异或（64个元素分组排序）
            compare_idx = threadIdx.x ^ 32;
            mapping_fitness = sm_sorted_fitness[compare_idx];
            mapping_param = sm_sorted_param[compare_idx];
            sortOrder = (threadIdx.x > compare_idx) ? -1.f : 1.f;
            
            if (sortOrder * (mapping_fitness - current_fitness) < 0.f) {
                current_fitness = mapping_fitness;
                current_param = mapping_param;
            }
            
            // 继续进行warp内部的比较排序
            BitonicWarpCompare(current_param, current_fitness, 16);
            BitonicWarpCompare(current_param, current_fitness, 8);
            BitonicWarpCompare(current_param, current_fitness, 4);
            BitonicWarpCompare(current_param, current_fitness, 2);
            BitonicWarpCompare(current_param, current_fitness, 1);
        }


        if (blockIdx.x < CUDA_PARAM_MAX_SIZE){
            all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id] = current_param;
        }
        if (blockIdx.x == 0)    all_fitness[threadIdx.x] = current_fitness;
    }
    
    /**
     * I feel something wrong for the following implement of SortOldParamBasedBitonic
     */
    // template <int T=64> // T is the pop size
    // __device__ __forceinline__ void SortOldParamBasedBitonic(float *all_param, float *all_fitness){
    //     if (all_param == nullptr || all_fitness == nullptr) return;
    //     // each block have a share memory
    //     __shared__ float sm_sorted_fitness[T * 2];
    //     __shared__ float sm_sorted_param[T * 2];
    //     int param_id = blockIdx.x;
    //     int sol_id = threadIdx.x;
    //     float current_param;
    //     float current_fitness;

    //     if(threadIdx.x < T){
    //         current_param = all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id];
    //         current_fitness = all_fitness[sol_id];
    //     }
    //     else{
    //         current_param = all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id + T];
    //         current_fitness = all_fitness[sol_id + T];
    //     }
        

    //     int compare_idx;
    //     float mapping_param, mapping_fitness, sortOrder;

    //     // if (threadIdx.x >= 64){
    //     // Sort the contents of 32 threads in a warp based on Bitonic merge sort. Implement detail is the alternative representation of https://en.wikipedia.org/wiki/Bitonic_sorter
    //     BitonicWarpCompare(current_param, current_fitness, 1);

    //     BitonicWarpCompare(current_param, current_fitness, 3);
    //     BitonicWarpCompare(current_param, current_fitness, 1);

    //     BitonicWarpCompare(current_param, current_fitness, 7);
    //     BitonicWarpCompare(current_param, current_fitness, 2);
    //     BitonicWarpCompare(current_param, current_fitness, 1);

    //     BitonicWarpCompare(current_param, current_fitness, 15);
    //     BitonicWarpCompare(current_param, current_fitness, 4);
    //     BitonicWarpCompare(current_param, current_fitness, 2);
    //     BitonicWarpCompare(current_param, current_fitness, 1);

    //     // above all finish the sorting 16 threads in Warp, continue to finish 2 group of 16 threads
    //     BitonicWarpCompare(current_param, current_fitness, 31);
    //     BitonicWarpCompare(current_param, current_fitness, 8);
    //     BitonicWarpCompare(current_param, current_fitness, 4);
    //     BitonicWarpCompare(current_param, current_fitness, 2);
    //     BitonicWarpCompare(current_param, current_fitness, 1);

    //     // above all finsh the sort for each warp, continue to finish the sort between different warp by share memory.
    //     // record the warp sorting result to share memory
    //     sm_sorted_param[sol_id ] = current_param;
    //     sm_sorted_fitness[sol_id] = current_fitness;
    //     // }
        
    //     // Wait for all thread finish above computation
    //     __syncthreads();

    //     // if (threadIdx.x >= 64)
    //     // {
    //     compare_idx = sol_id ^ 63;
    //     mapping_param = sm_sorted_param[compare_idx];
    //     mapping_fitness = sm_sorted_fitness[compare_idx];

    //     sortOrder = (threadIdx.x > (threadIdx.x ^ 63)) ? -1.0 : 1.0;

    //     if(sortOrder * (mapping_fitness - current_fitness) < 0.f){
    //         current_param = mapping_param;
    //         current_fitness = mapping_fitness;
    //     }        
    //     // Wait for the sort between two warp finish
    //     __syncthreads();
    //     // Now, we can come back to the sorting in the warp
    //     BitonicWarpCompare(current_param, current_fitness, 16);
    //     BitonicWarpCompare(current_param, current_fitness, 8);
    //     BitonicWarpCompare(current_param, current_fitness, 4);
    //     BitonicWarpCompare(current_param, current_fitness, 2);
    //     BitonicWarpCompare(current_param, current_fitness, 1);

    //     sm_sorted_param[sol_id ] = current_param;
    //     sm_sorted_fitness[sol_id] = current_fitness;
        
    //     __syncthreads();
    //     if(threadIdx.x >= T){
    //         compare_idx = threadIdx.x ^ 127;
    //         mapping_param = sm_sorted_param[compare_idx];
    //         mapping_fitness = sm_sorted_fitness[compare_idx];
    //         sortOrder = (threadIdx.x > compare_idx) ? -1.f : 1.f;

    //         if (sortOrder * (mapping_fitness - current_fitness) < 0.f) {
    //             current_fitness = mapping_fitness;
    //             current_param = mapping_param;
    //             sm_sorted_fitness[threadIdx.x] = current_fitness;
    //             sm_sorted_param[threadIdx.x] = current_param;
    //         }
    //     }
        
    //     __syncthreads();
    //     if(threadIdx.x >= T){
    //         compare_idx = threadIdx.x ^ 32;
    //         mapping_fitness = sm_sorted_fitness[compare_idx];
    //         mapping_param = sm_sorted_param[compare_idx];
    //         sortOrder = (threadIdx.x > compare_idx) ? -1.f : 1.f;
    //         if (sortOrder * (mapping_fitness - current_fitness) < 0.f) {
    //             current_fitness = mapping_fitness;
    //             current_param = mapping_param;
    //         }
    //         BitonicWarpCompare(current_param, current_fitness, 16);
    //         BitonicWarpCompare(current_param, current_fitness, 8);
    //         BitonicWarpCompare(current_param, current_fitness, 4);
    //         BitonicWarpCompare(current_param, current_fitness, 2);
    //         BitonicWarpCompare(current_param, current_fitness, 1);
    //     }

    //     if(threadIdx.x >= T){
    //         // above all finish all sorting for fitness and param
    //         if (blockIdx.x < CUDA_PARAM_MAX_SIZE){
    //             all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id + T] = current_param;
    //             // printf("======================== Update sorted param for solution id:%d\n", threadIdx.x);
    //         }
    //         if (blockIdx.x == 0)    all_fitness[threadIdx.x + T] = current_fitness;
    //     }
    //     else{
    //         // above all finish all sorting for fitness and param
    //         if (blockIdx.x < CUDA_PARAM_MAX_SIZE){
    //             all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id] = current_param;
    //             // printf("======================== Update sorted param for solution id:%d\n", threadIdx.x);
    //         }
    //         if (blockIdx.x == 0)    all_fitness[threadIdx.x] = current_fitness;
    //     }
    // }

    // __device__ __forceinline__ void EvolveTerminate(CudaEvolveData *evolve, float *last_best_fitness, float *all_fitness, int *terminate_flag){
    //     if(threadIdx.x > 0)    return;
    //     if(*terminate_flag != 0)    return;
    //     if(blockIdx.x > evolve->problem_param.elite_eval_count)   return;
        
    //     float elite_eval_sum = all_fitness[blockIdx.x];

    //     if (evolve->problem_param.elite_eval_count == 8){
    //         elite_eval_sum += __shfl_down_sync(0x000000ff, elite_eval_sum, 8);
    //         elite_eval_sum += __shfl_down_sync(0x000000ff, elite_eval_sum, 4);
    //         elite_eval_sum += __shfl_down_sync(0x000000ff, elite_eval_sum, 2);
    //         elite_eval_sum += __shfl_down_sync(0x000000ff, elite_eval_sum, 1);
    //     }
    //     printf("elite_eval_sum:%f\n",elite_eval_sum);
    //     if(abs((elite_eval_sum - evolve->problem_param.elite_eval_count * *last_best_fitness)/evolve->problem_param.elite_eval_count) < evolve->problem_param.accuracy_rng){
    //         *terminate_flag = 1;
    //     }
    // }

    __device__ __forceinline__ void EvolveTerminate(CudaEvolveData *evolve, float *last_best_fitness, float *all_fitness, int *terminate_flag){
        if(threadIdx.x > 0)    return;
        if(blockIdx.x > 0)  return;
        if(*terminate_flag != 0)    return;
                
        // 第0个block负责收集和累加所有block的结果
        if(blockIdx.x == 0) {
            float sum = 0.0;
            // 累加其他block的值
            for(int i = 1; i < evolve->problem_param.elite_eval_count; ++i) {
                sum += all_fitness[i];
                // printf("individual %d fitness:%f\n",i, sum );
            }
            
            // printf("elite_eval_sum:%f bias:%f\n", sum, abs((sum - evolve->problem_param.elite_eval_count * *last_best_fitness)/evolve->problem_param.elite_eval_count));
            if(abs((sum - evolve->problem_param.elite_eval_count * *last_best_fitness)/evolve->problem_param.elite_eval_count) < evolve->problem_param.accuracy_rng){
                *terminate_flag = 1;
            }
        }
    }

    template <int T = CUDA_SOLVER_POP_SIZE>
    __global__ void UpdateParameter(int epoch, CudaEvolveData *evolve, CudaParamClusterData<CUDA_SOLVER_POP_SIZE> *new_param, CudaParamClusterData<CUDA_SOLVER_POP_SIZE*3> *old_param, int* terminate_flag, float *last_f){
        if ((*terminate_flag & 1) > 0) return;
        // for old_param (current sol, delete sol, replaced sol), we select the fitness of current sol for all old param
        // so threadIdx.x & (T-1) equal to threadIdx.x % (T-1) which can help us to mapping all old_param to current sol
        float old_fitness = old_param->fitness[threadIdx.x & (T-1)], new_fitness = new_param->fitness[threadIdx.x & (T-1)];
        
        int sol_id = threadIdx.x;
        int param_id = blockIdx.x;
        float current_fitness = CUDA_MAX_FLOAT;

        // Update parameter
        if (sol_id < T){
            if (param_id < CUDA_PARAM_MAX_SIZE){
                float old_param_value = old_param->all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id];
                float new_param_value = new_param->all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id];

                // compare old_fitness and new_fitness to determine which solution should be replaced.
                if (new_fitness < old_fitness){
                    current_fitness = new_fitness;
                    // select better solution as current sol, and move previous solution to replaced part
                    old_param->all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id] = new_param_value;
                    old_param->all_param[(sol_id + 2 * T) * CUDA_PARAM_MAX_SIZE + param_id] = old_param_value;
                    old_param->fitness[sol_id] = current_fitness;
                }
                else{
                    current_fitness = old_fitness;
                    old_param->all_param[(sol_id + 2 * T) * CUDA_PARAM_MAX_SIZE + param_id] = new_param_value;
                }
            }
        }
        else{
            // old_param->fitness[sol_id ] = (new_fitness < old_fitness) ? old_fitness : new_fitness;
        }

        // wait for all thread finish above all computation
        __syncthreads();

        /**
         * Based on the rule of L shade to update hyperparameter
         */
        if (blockIdx.x == 0){
            // calculate 8 float data for a warp.
            float ALIGN(64) adaptiveParamSums[8];
            // each warp will runing parallel reduction for sum. for 64 pop_size cluster, we need 2 warp. And then, all result are storaged in a share memory array.
            __shared__ ALIGN(64) float share_sum[4 * 8];

            float3 lshade_param;
            float scale_f, scale_f1, cr, w;
            if (threadIdx.x < T){
                // float3 is a built-in vector type of CUDA. And their memory addresses are continuous.
                // It is more efficient to read parameters by this conversion method.
                lshade_param = reinterpret_cast<float3 *>(new_param->lshade_param)[sol_id];
                scale_f = lshade_param.x;
                scale_f1 = lshade_param.y;
                cr = lshade_param.z;
                // formula (8) and (9) in https://ieeexplore.ieee.org/document/6900380
                w  = (new_fitness - old_fitness) / max(1e-4f, new_fitness);

                // calculate w*cr, w*cr*cr, w*scale_f, w*scale_f*scale_f for equation (7) in https://ieeexplore.ieee.org/document/6900380
                adaptiveParamSums[0] = w;
                adaptiveParamSums[1] = w * scale_f;
                adaptiveParamSums[2] = w * scale_f * scale_f;
                adaptiveParamSums[3] = w * scale_f1;
                adaptiveParamSums[4] = w * scale_f1 * scale_f1;
                adaptiveParamSums[5] = w * cr;
                adaptiveParamSums[6] = w * cr * cr;
                adaptiveParamSums[7] = 0;

                // Warp parallel reduction sum (finish the sum part of equal (7) (8) (9))
                for (int i = 0; i < 7; ++i) {
                    adaptiveParamSums[i] += __shfl_down_sync(0xffffffff, adaptiveParamSums[i], 16);
                    adaptiveParamSums[i] += __shfl_down_sync(0xffffffff, adaptiveParamSums[i], 8);
                    adaptiveParamSums[i] += __shfl_down_sync(0xffffffff, adaptiveParamSums[i], 4);
                    adaptiveParamSums[i] += __shfl_down_sync(0xffffffff, adaptiveParamSums[i], 2);
                    adaptiveParamSums[i] += __shfl_down_sync(0xffffffff, adaptiveParamSums[i], 1);
                }
                // The recursive results for each warp are recorded at the shared memoery
                if ((threadIdx.x & 31) == 0){
                    reinterpret_cast<float4 *>(share_sum)[(threadIdx.x >> 5) * 2] = reinterpret_cast<float4 *>(adaptiveParamSums)[0];
                    reinterpret_cast<float4 *>(share_sum)[(threadIdx.x >> 5) * 2 + 1] = reinterpret_cast<float4 *>(adaptiveParamSums)[1];
                }
            }
            __syncthreads();
            // continue to use parallel reduction for different results of above data that have been storaged at share memory
            if (threadIdx.x < T){
                if (threadIdx.x < (T >> 5)){
                    // loading parameter from share memory to adaptiveParamSums so that each thread responsible for the from different warp
                    reinterpret_cast<float4 *>(adaptiveParamSums)[0] = reinterpret_cast<float4 *>(share_sum)[threadIdx.x * 2];
                    reinterpret_cast<float4 *>(adaptiveParamSums)[1] = reinterpret_cast<float4 *>(share_sum)[threadIdx.x * 2 + 1];
                    
                    // parallel reduction for different warp result in share memory
                    for(int i = 0; i < 7; ++i){
                        // !!!!!!!!!!!!!!! If the T or pop_size is not 64. This part should be modified. !!!!!!!!!!!!!!!!!!
                        if (T == 128) {
                            adaptiveParamSums[i] += __shfl_down_sync(0x0000000f, adaptiveParamSums[i], 2);
                        }
                        adaptiveParamSums[i] += __shfl_down_sync(0x00000003, adaptiveParamSums[i], 1);
                    }

                    // update the evolve data
                    if(threadIdx.x == 0){
                        if (adaptiveParamSums[2] > 1e-4f && adaptiveParamSums[4] > 1e-4f && adaptiveParamSums[6] > 1e-4f){
                            evolve->hist_lshade_param.scale_f = adaptiveParamSums[2] * adaptiveParamSums[0] / max(1e-4f, adaptiveParamSums[1] * adaptiveParamSums[0]);
                            evolve->hist_lshade_param.scale_f1 = adaptiveParamSums[4] * adaptiveParamSums[0] / max(1e-4f, adaptiveParamSums[3] * adaptiveParamSums[0]);
                            evolve->hist_lshade_param.Cr = adaptiveParamSums[6] * adaptiveParamSums[0] / max(1e-4f, adaptiveParamSums[5] * adaptiveParamSums[0]);
                        }
                    }
                }
            }
        }
        __syncthreads();

        SortOldParamBasedBitonic(old_param->all_param, old_param->fitness);

        // The following this method is only compare current fitness and last fitness. When the bias of these two fitness lower than accuracy_rng then stop evaluation
        // __syncthreads();
        // if(threadIdx.x == 0 && blockIdx.x == 0 && *terminate_flag == 0){
        //     // printf("evolve->problem_param.accuracy_rng:%f\n",evolve->problem_param.accuracy_rng);
        //     if(abs(old_param->fitness[0] - *last_f) < evolve->problem_param.accuracy_rng){
        //         *terminate_flag = 1;
        //         // printf("stop stop stop");
        //     }
        //     *last_f = old_param->fitness[threadIdx.x];
        // }

        // This method consider the average fitness of top 8 individual. when the avg lower than accuracy_rng then stop evaluation
        // EvolveTerminate(evolve, last_f, old_param->fitness, terminate_flag);
        *last_f = old_param->fitness[threadIdx.x];
    }
}


#endif