#ifndef CUDAPROCESS_CONVERTER_DECODER_H
#define CUDAPROCESS_CONVERTER_DECODER_H

#include <iostream>
#include <cuda_runtime.h>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <sys/time.h>
#include <assert.h>
#include "diff_evolution_solver/data_type.h"

namespace cudaprocess{
    template <int T>
    HOST_DEVICE void ConvertCudaParam(CudaParamClusterData<T> *output, CudaParamIndividual *input, int sol_id, int param_id){
        assert(sol_id < T);

        output->all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id] = input->param[param_id];
        
        if(param_id == 0){
            output->fitness[sol_id] = input->fitness;
            output->lshade_param[sol_id * 3] = input->cur_scale_f;
            output->lshade_param[sol_id * 3 + 1] = input->cur_scale_f1;
            output->lshade_param[sol_id * 3 + 2] = input->cur_Cr;
        }
    }

    template <int T>
    HOST_DEVICE void ConvertCudaParamRevert(CudaParamClusterData<T> *output, CudaParamIndividual *input, int sol_id, int param_id){
        assert(sol_id < T);

        input->param[param_id] = output->all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id];
        // printf("getting param_id: %d, value: %f\n", param_id, output->all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id]);
        if(param_id == 0){
            input->fitness = output->fitness[sol_id];
            // printf("aaaaaaaaaaaaaaa fitness:%f\n", input->fitness);
            input->cur_scale_f = output->lshade_param[sol_id * 3];
            input->cur_scale_f1 = output->lshade_param[sol_id * 3 + 1];
            input->cur_Cr = output->lshade_param[sol_id * 3 + 2];
        }
    }

    template <int SOURCE, int DESTINATION>
    HOST_DEVICE void ConvertCudaParamBetweenClusters(CudaParamClusterData<SOURCE> *input, CudaParamClusterData<DESTINATION> *output, int src_id, int dst_id, int param_id){
        if (src_id >= SOURCE || dst_id >= DESTINATION) {
            assert(false);
        }
        output->all_param[dst_id * CUDA_PARAM_MAX_SIZE + param_id] = input->all_param[src_id * CUDA_PARAM_MAX_SIZE + param_id];
        // printf("save to old:%f\n", output->all_param[dst_id * CUDA_PARAM_MAX_SIZE + param_id]);
        if (param_id == 0){
            output->fitness[dst_id] = input->fitness[dst_id];
            output->lshade_param[dst_id * 3] = input->lshade_param[src_id * 3];
            output->lshade_param[dst_id * 3 + 1] = input->lshade_param[src_id * 3 + 1];
            output->lshade_param[dst_id * 3 + 2] = input->lshade_param[src_id * 3 + 2];
        }
    }
}

#endif