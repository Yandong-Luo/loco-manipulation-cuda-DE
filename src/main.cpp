#include <iostream>
#include <cuda_runtime.h>
#include "solver_center/solver_center.h"
#include <chrono>
// #include "diff_evolution_solver/solver.cuh"
// #include "diff_evolution_solver/data_type.h"

int main(int argc,char** argv){
    // CPU 计时
    auto cpu_start = std::chrono::high_resolution_clock::now();

    // Set the number of tasks and GPU device ID
    // each task use a differential evolution
    int gpu_device = 0;

    // Create an instance of the CudaDiffEvolveSolver class
    cudaprocess::CudaSolverCenter solver_center(gpu_device);

    // Initialize the solver
    solver_center.Init("test_config.yaml");

    solver_center.ParallelGenerateMultiTaskSol();

    // CPU 计时结束
    auto cpu_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float, std::milli> cpu_duration = cpu_end - cpu_start;

    std::cout << "CPU time: " << cpu_duration.count() << " ms" << std::endl;

    return 0;
}