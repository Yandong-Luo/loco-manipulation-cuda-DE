#include "diff_evolution_solver/random_manager.cuh"

namespace cudaprocess{
    CudaRandomManager::CudaRandomManager(int gpu_device) {
        CHECK_CUDA(cudaSetDevice(gpu_device));
        CHECK_CUDA(cudaStreamCreate(&stream));
        
        CHECK_CUDA(cudaMalloc(&uniform_data_, sizeof(float) * uniform_size_));
        CHECK_CUDA(cudaMalloc(&normal_data_, sizeof(float) * normal_size_));
        
        CURAND_CHECK(curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_XORWOW));
        CURAND_CHECK(curandSetStream(gen, stream));
        CURAND_CHECK(curandSetPseudoRandomGeneratorSeed(gen, 0));
        
        Generate();
    }

    CudaRandomManager::~CudaRandomManager() {
        cudaStreamSynchronize(stream);
        CURAND_CHECK(curandDestroyGenerator(gen));
        CHECK_CUDA(cudaFree(uniform_data_));
        CHECK_CUDA(cudaFree(normal_data_));
        CHECK_CUDA(cudaStreamDestroy(stream));
    }

    void CudaRandomManager::Generate() {
        CURAND_CHECK(curandGenerateUniform(gen, uniform_data_, uniform_size_));
        CURAND_CHECK(curandGenerateNormal(gen, normal_data_, normal_size_, 0.0f, 1.0f));
        cudaStreamSynchronize(stream);
    }
}