#ifndef CUDAPROCESS_DIFF_EVOLUTION_DECODER_H
#define CUDAPROCESS_DIFF_EVOLUTION_DECODER_H

#include <iostream>
#include <cuda_runtime.h>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <sys/time.h>
#include <curand_kernel.h>
#include "data_type.h"

namespace cudaprocess{
    template<int T = CUDA_SOLVER_POP_SIZE>
    __global__ void InitParameter(CudaEvolveData* evolve_data, int size, CudaParamClusterData<T>* new_cluster_data, CudaParamClusterData<T*3>* old_cluster_data, float *uniform_data){
        int idx = threadIdx.x;
        if (idx >= size)    return;

        // initial evolve data
        // evolve_data->new_cluster_vec->data[idx].con_var_dims = evolve_data->problem_param.con_var_dims;
        // evolve_data->new_cluster_vec->data[idx].int_var_dims = evolve_data->problem_param.int_var_dims;
        // evolve_data->new_cluster_vec->data[idx].dims = evolve_data->dims;
        // evolve_data->new_cluster_vec->data[idx].fitness = 0.f;
        // evolve_data->new_cluster_vec->data[idx].cur_scale_f1 = 0.5f;
        // evolve_data->new_cluster_vec->data[idx].cur_scale_f = 0.5f;
        // evolve_data->new_cluster_vec->data[idx].cur_Cr = 0.5f;

        // initial new_cluster_data
        for (int i = 0; i < evolve_data->problem_param.dims; ++i) {
            // evolve_data->new_cluster_vec->data[idx].param[i] = 0;
            if (i < evolve_data->problem_param.con_var_dims){
                // each parameters were decode as a vector with the length of CUDA_PARAM_MAX_SIZE
                // printf("%f\n", uniform_data[idx * CUDA_PARAM_MAX_SIZE + i]);
                new_cluster_data->all_param[idx * CUDA_PARAM_MAX_SIZE + i] = evolve_data->lower_bound[i] + uniform_data[idx * CUDA_PARAM_MAX_SIZE + i] * (evolve_data->upper_bound[i] - evolve_data->lower_bound[i]);
            }
            else{
                int generate_int = evolve_data->lower_bound[i] + uniform_data[idx * CUDA_PARAM_MAX_SIZE + i] * (evolve_data->upper_bound[i] + 1 - evolve_data->lower_bound[i]);
                if (generate_int == evolve_data->upper_bound[i] + 1 )   generate_int = evolve_data->upper_bound[i];
                new_cluster_data->all_param[idx * CUDA_PARAM_MAX_SIZE + i] = generate_int;
            }
            
            // printf("index:%d lower bound:%f, upper bound:%f, value:%f\n",i, evolve_data->lower_bound[i], evolve_data->upper_bound[i], new_cluster_data->all_param[idx * CUDA_PARAM_MAX_SIZE + i]);
        }
        // printf("\n");
        if(idx == 0){
            old_cluster_data->con_var_dims = new_cluster_data->con_var_dims = evolve_data->problem_param.con_var_dims;
            old_cluster_data->int_var_dims = new_cluster_data->int_var_dims = evolve_data->problem_param.int_var_dims;
            old_cluster_data->dims = new_cluster_data->dims = evolve_data->problem_param.dims;
            // printf("Thread 0: first few params = [%f, %f, %f]\n",
            // new_cluster_data->all_param[0],
            // new_cluster_data->all_param[1],
            // new_cluster_data->all_param[2]);
        }

        new_cluster_data->fitness[idx] = 0.;
        new_cluster_data->cost[idx] = 0.;
        new_cluster_data->lshade_param[idx * 3 + 0] = 0.5f;                        // scale_f
        new_cluster_data->lshade_param[idx * 3 + 1] = 0.5f;                        // scale_f1
        new_cluster_data->lshade_param[idx * 3 + 2] = 0.5f;                        // crossover

        // initial old_cluster_data
        old_cluster_data->fitness[idx] = 0.;
        old_cluster_data->cost[idx] = 0.;

        // printf("Finish the initialization of thread id:%d\n", idx);
    }
}
#endif