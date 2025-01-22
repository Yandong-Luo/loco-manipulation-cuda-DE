#ifndef CUDAPROCESS_SOLVER_CENTER_H
#define CUDAPROCESS_SOLVER_CENTER_H
#include <cassert>
#include "diff_evolution_solver/data_type.h"
#include "diff_evolution_solver/solver.cuh"
// #include "diff_evolution_solver/random_center.cuh"
#include "diff_evolution_solver/random_manager.cuh"
#include "yaml-cpp/yaml.h"

namespace cudaprocess{
    class CudaSolverCenter
    {
    private:
        YAML::Node config;
        int gpu_device_, num_tasks_;
        CudaParamIndividual *tasks_best_sol_;
        CudaVector<CudaParamIndividual, CUDA_MAX_POTENTIAL_SOLUTION> *tasks_potential_sol_;
        CudaDiffEvolveSolver diff_evolve_solvers_[CUDA_MAX_TASKS];
        Problem *tasks_problem_;
        // std::shared_ptr<CudaRandomCenter> rnd_center_;
        std::shared_ptr<CudaRandomManager> rnd_manager_;
        bool cudamalloc_flag{false};
        cublasHandle_t cublas_handle_; 
    public:
        CudaSolverCenter(int gpu_device) : cudamalloc_flag(false) {gpu_device_ = gpu_device;};
        ~CudaSolverCenter();
        void Init(std::string filename);
        void GenerateSolution(int task_id);
        void ParallelGenerateMultiTaskSol();
        Problem LoadProblemFromYaml(const YAML::Node& node);

        Problem *Getproblem(int task_id){
            assert(task_id < task_id);
            return &tasks_problem_[task_id];
        };
        
        CudaVector<CudaParamIndividual, CUDA_MAX_POTENTIAL_SOLUTION> *GetPotentialSol(int task_id) {
            assert(task_id < num_tasks_);
            return &tasks_potential_sol_[task_id];
        };
        
        CudaParamIndividual *GetBestSol(int task_id) {
            assert(task_id < num_tasks_);
            return &tasks_best_sol_[task_id];
        };
    };

}

#endif