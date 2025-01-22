#ifndef CUDAPROCESS_RANDOM_MANAGER_H
#define CUDAPROCESS_RANDOM_MANAGER_H


#include <curand.h>
#include <curand_kernel.h>
#include "utils/utils.cuh"

namespace cudaprocess{
    class CudaRandomManager {
        private:
            
        public:
            CudaRandomManager(int gpu_device);
            ~CudaRandomManager();
            void Generate();
            float* GetUniformData() { return uniform_data_; }
            float* GetNormalData() { return normal_data_; }


            curandGenerator_t gen;
            float* uniform_data_;
            float* normal_data_;
            const int uniform_size_ = 512000;
            const int normal_size_ = CUDA_SOLVER_POP_SIZE * 3 * CUDA_MAX_ROUND_NUM;
            cudaStream_t stream;
    };
}

#endif