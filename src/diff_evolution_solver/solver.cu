#include "diff_evolution_solver/solver.cuh"
#include "diff_evolution_solver/decoder.cuh"
#include "diff_evolution_solver/debug.cuh"
#include "diff_evolution_solver/evolve.cuh"
#include "diff_evolution_solver/evaluate.cuh"
#include "cart_pole/cart_pole_utils.cuh"
#include "cart_pole/model.cuh"
#include "cart_pole/evaluate.cuh"
#include "utils/utils_fun.cuh"
#include <math.h>

namespace cudaprocess{

void CudaDiffEvolveSolver::MallocSetup(){
    CHECK_CUDA(cudaSetDevice(gpu_device_));

    // GPU Device
    // CHECK_CUDA(cudaMalloc(&decoder_, sizeof(CudaProblemDecoder)));
    CHECK_CUDA(cudaMalloc(&evolve_data_, sizeof(CudaEvolveData)));
    CHECK_CUDA(cudaMalloc(&new_cluster_data_, sizeof(CudaParamClusterData<CUDA_SOLVER_POP_SIZE>)));
    CHECK_CUDA(cudaMalloc(&old_cluster_data_, sizeof(CudaParamClusterData<CUDA_SOLVER_POP_SIZE * 3>)));
    // CHECK_CUDA(cudaMalloc(&new_cluster_vec_, sizeof(CudaVector<CudaParamIndividual, CUDA_SOLVER_POP_SIZE>)));
    // CHECK_CUDA(cudaMalloc(&problem_, sizeof(Problem)));
    CHECK_CUDA(cudaMalloc(&evaluate_score_, CUDA_SOLVER_POP_SIZE * sizeof(float)));
    CHECK_CUDA(cudaMalloc(&last_fitness, sizeof(float)));
    CHECK_CUDA(cudaMalloc(&terminate_flag, sizeof(int)));
    CHECK_CUDA(cudaMalloc(&result, sizeof(CudaParamIndividual)));

    // objective, constraint, constraint_score, lambda, parameter matrix
    // CHECK_CUDA(cudaMalloc(&constraint_matrix, row_constraint * col_constraint * sizeof(float)));
    // CHECK_CUDA(cudaMalloc(&objective_matrix, row_obj * col_obj * sizeof(float)));
    // CHECK_CUDA(cudaMalloc(&constraint_score, CUDA_SOLVER_POP_SIZE * row_constraint * sizeof(float)));
    // CHECK_CUDA(cudaMalloc(&lambda_matrix, row_lambda * col_lambda * sizeof(float)));
    // CHECK_CUDA(cudaMalloc(&param_matrix, (dims_ + 1) * CUDA_SOLVER_POP_SIZE * sizeof(float)));
    // CHECK_CUDA(cudaMalloc(&objective_Q_matrix, row_obj_Q * col_obj_Q * sizeof(float)));
    // CHECK_CUDA(cudaMalloc(&quad_matrix, CUDA_SOLVER_POP_SIZE * CUDA_SOLVER_POP_SIZE * sizeof(float)));
    // CHECK_CUDA(cudaMalloc(&quad_transform, row_obj_Q * CUDA_SOLVER_POP_SIZE * sizeof(float)));
    // CHECK_CUDA(cudaMalloc(&quadratic_score, 1 * CUDA_SOLVER_POP_SIZE * sizeof(float)));
    

    // CPU Host
    CHECK_CUDA(cudaHostAlloc(&h_terminate_flag, sizeof(int), cudaHostAllocDefault));
    CHECK_CUDA(cudaHostAlloc(&host_result, sizeof(CudaParamIndividual), cudaHostAllocDefault));
    CHECK_CUDA(cudaHostAlloc(&host_evolve_data_, sizeof(CudaEvolveData), cudaHostAllocDefault));

    if (DEBUG_PRINT_FLAG || DEBUG_PRINT_SOLVER_FLAG){
        CHECK_CUDA(cudaHostAlloc(&host_new_cluster_data_, sizeof(CudaParamClusterData<CUDA_SOLVER_POP_SIZE>), cudaHostAllocDefault));
        CHECK_CUDA(cudaHostAlloc(&host_old_cluster_data_, sizeof(CudaParamClusterData<CUDA_SOLVER_POP_SIZE*3>), cudaHostAllocDefault));
    }
    
    if (DEBUG_PRINT_FLAG || DEBUG_PRINT_EVALUATE_FLAG){
        // objective, constraint, constraint_score, lambda, parameter, score matrix
        // CHECK_CUDA(cudaHostAlloc(&h_constraint_matrix, row_constraint * col_constraint * sizeof(float), cudaHostAllocDefault));
        // CHECK_CUDA(cudaHostAlloc(&h_objective_matrix, row_obj * col_obj * sizeof(float), cudaHostAllocDefault));
        // CHECK_CUDA(cudaHostAlloc(&host_constraint_score, CUDA_SOLVER_POP_SIZE * row_constraint * sizeof(float), cudaHostAllocDefault));
        // CHECK_CUDA(cudaHostAlloc(&h_lambda_matrix, row_lambda * col_lambda * sizeof(float), cudaHostAllocDefault));
        // CHECK_CUDA(cudaHostAlloc(&host_param_matrix, (dims_ + 1) * CUDA_SOLVER_POP_SIZE * sizeof(float), cudaHostAllocDefault));
        // CHECK_CUDA(cudaHostAlloc(&h_objective_Q_matrix, row_obj_Q * col_obj_Q * sizeof(float), cudaHostAllocDefault));
        // CHECK_CUDA(cudaHostAlloc(&host_evaluate_score_, CUDA_SOLVER_POP_SIZE * sizeof(float), cudaHostAllocDefault));

        // CHECK_CUDA(cudaHostAlloc(&host_quad_matrix, CUDA_SOLVER_POP_SIZE * CUDA_SOLVER_POP_SIZE * sizeof(float), cudaHostAllocDefault));
        // CHECK_CUDA(cudaHostAlloc(&h_quad_transform, row_obj_Q * CUDA_SOLVER_POP_SIZE * sizeof(float), cudaHostAllocDefault));
        // CHECK_CUDA(cudaHostAlloc(&h_quadratic_score, 1 * CUDA_SOLVER_POP_SIZE * sizeof(float), cudaHostAllocDefault));
    }

    // !--------------- CART POLE ---------------!
    CHECK_CUDA(cudaMalloc(&env_constraint, cart_pole::num_constraints * sizeof(float)));
    CHECK_CUDA(cudaMalloc(&C_matrix, cart_pole::row_C * cart_pole::col_C * sizeof(float)));

    CHECK_CUDA(cudaMalloc(&A_matrix, cart_pole::row_A * cart_pole::col_A * sizeof(float)));
    
    CHECK_CUDA(cudaMalloc(&cluster_state, sizeof(CartStateList)));

    // evaluation of state
    CHECK_CUDA(cudaMalloc(&state_weight_matrix, cart_pole::row_state_weight * cart_pole::col_state_weight * sizeof(float)));

    CHECK_CUDA(cudaMalloc(&state_matrix, cart_pole::row_state_matrix * cart_pole::col_state_matrix * sizeof(float)));

    CHECK_CUDA(cudaMalloc(&temp_state_score, cart_pole::row_temp_state_score * cart_pole::col_temp_state_score * sizeof(float)));

    CHECK_CUDA(cudaMalloc(&quadratic_state, cart_pole::row_quadratic_state * cart_pole::col_quadratic_state * sizeof(float)));

    CHECK_CUDA(cudaMalloc(&state_score, cart_pole::row_state_score * cart_pole::col_state_score * sizeof(float)));

    // evaluation of control input
    CHECK_CUDA(cudaMalloc(&control_matrix, cart_pole::row_control_matrix * cart_pole::col_control_matrix * sizeof(float)));

    CHECK_CUDA(cudaMalloc(&control_weight_matrix, cart_pole::row_control_weight * cart_pole::col_control_weight * sizeof(float)));

    CHECK_CUDA(cudaMalloc(&temp_control_score, cart_pole::row_temp_control_score * cart_pole::col_temp_control_score * sizeof(float)));

    CHECK_CUDA(cudaMalloc(&quadratic_control, cart_pole::row_quadratic_control * cart_pole::col_quadratic_control * sizeof(float)));

    CHECK_CUDA(cudaMalloc(&control_score, cart_pole::row_control_score * cart_pole::col_control_score * sizeof(float)));

    CHECK_CUDA(cudaMalloc(&score, cart_pole::row_score * cart_pole::col_score * sizeof(float)));
    
    if (DEBUG_CART_POLE){
        CHECK_CUDA(cudaHostAlloc(&h_state, cart_pole::state_dims * sizeof(float), cudaHostAllocDefault));
        CHECK_CUDA(cudaHostAlloc(&h_env_constraint, cart_pole::num_constraints * sizeof(float), cudaHostAllocDefault));
        CHECK_CUDA(cudaHostAlloc(&h_C_matrix, cart_pole::row_C * cart_pole::col_C * sizeof(float), cudaHostAllocDefault));
        CHECK_CUDA(cudaHostAlloc(&h_A_matrix, cart_pole::row_A * cart_pole::col_A * sizeof(float), cudaHostAllocDefault));

        CHECK_CUDA(cudaHostAlloc(&h_cluster_state, sizeof(CartStateList), cudaHostAllocDefault));

        CHECK_CUDA(cudaHostAlloc(&h_state_matrix, cart_pole::row_state_matrix * cart_pole::col_state_matrix * sizeof(float), cudaHostAllocDefault));
        CHECK_CUDA(cudaHostAlloc(&h_control_matrix, cart_pole::row_control_matrix * cart_pole::col_control_matrix * sizeof(float), cudaHostAllocDefault));

        // the weight of state and control
        CHECK_CUDA(cudaHostAlloc(&h_state_weight_matrix, cart_pole::row_state_weight * cart_pole::col_state_weight * sizeof(float), cudaHostAllocDefault));
        CHECK_CUDA(cudaHostAlloc(&h_control_weight_matrix, cart_pole::row_control_weight * cart_pole::col_control_weight * sizeof(float), cudaHostAllocDefault));

        // the evaluation process of state
        CHECK_CUDA(cudaHostAlloc(&h_temp_state_score, cart_pole::row_temp_state_score * cart_pole::col_temp_state_score * sizeof(float), cudaHostAllocDefault));
        CHECK_CUDA(cudaHostAlloc(&h_quadratic_state, cart_pole::row_quadratic_state * cart_pole::col_quadratic_state * sizeof(float), cudaHostAllocDefault));

        // the evaluation process of control
        CHECK_CUDA(cudaHostAlloc(&h_temp_control_score, cart_pole::row_temp_control_score * cart_pole::col_temp_control_score * sizeof(float), cudaHostAllocDefault));
        CHECK_CUDA(cudaHostAlloc(&h_quadratic_control, cart_pole::row_quadratic_control * cart_pole::col_quadratic_control * sizeof(float), cudaHostAllocDefault));

        // score
        CHECK_CUDA(cudaHostAlloc(&h_score, cart_pole::row_score * cart_pole::col_score * sizeof(float), cudaHostAllocDefault));
        CHECK_CUDA(cudaHostAlloc(&h_state_score, cart_pole::row_state_score * cart_pole::col_state_score * sizeof(float), cudaHostAllocDefault));
        CHECK_CUDA(cudaHostAlloc(&h_control_score, cart_pole::row_control_score * cart_pole::col_control_score * sizeof(float), cudaHostAllocDefault));
    }

    cuda_utils_ = std::make_shared<CudaUtil>();

    cudamalloc_flag = true;
}

void CudaDiffEvolveSolver::InitDiffEvolveParam(float top, float d_top, float min_top, float diff, float d_diff, float min_diff, float scale_f, float prob_crossover){
    top_ = top;
    d_top_ = d_top;
    min_top_ = min_top;
    diff_ = diff;
    d_diff_ = d_diff;
    min_diff_ = min_diff;
    
    lshade_param_.scale_f = lshade_param_.scale_f1 = scale_f;
    lshade_param_.Cr = prob_crossover;

}

__global__ void InitCudaEvolveData(CudaEvolveData* evolve, CudaParamClusterData<CUDA_SOLVER_POP_SIZE*3>* old_cluster_data, int pop_size){
    int idx = threadIdx.x;
    if (idx == 0) {
        evolve->problem_param.top_ratio = 0.;
        evolve->hist_lshade_param.scale_f = evolve->hist_lshade_param.scale_f1 = 0.6;
        evolve->hist_lshade_param.Cr = 0.9;
        // evolve->new_cluster_vec->len = pop_size;
        old_cluster_data->len = pop_size;
    }
    if (idx < pop_size){
        // initial the each parameter in old_cluster 
        for (int i = 0; i < CUDA_PARAM_MAX_SIZE; ++i){
            old_cluster_data->all_param[(idx + pop_size) * CUDA_PARAM_MAX_SIZE + i] = 0.f;
        }
        old_cluster_data->fitness[idx + pop_size] = CUDA_MAX_FLOAT;
        old_cluster_data->cost[idx + pop_size] = CUDA_MAX_FLOAT;
    }
}

void CudaDiffEvolveSolver::SetBoundary(){
    for(int i = 0; i < dims_; ++i){
        host_evolve_data_->lower_bound[i] = cart_pole::u_lb;
        host_evolve_data_->upper_bound[i] = cart_pole::u_ub;
    }
}

/**
 * CudaEvolveData* ----> CudaParamClusterData<T> *
 */
__global__ void UpdateClusterDataBasedEvolve(CudaEvolveData* evolve_data, CudaParamClusterData<CUDA_SOLVER_POP_SIZE>* new_cluster_data, int num_last_potential_sol){
    int idx = blockIdx.x;
    if (idx >= num_last_potential_sol)   return;
    ConvertCudaParam<CUDA_SOLVER_POP_SIZE>(new_cluster_data, &evolve_data->last_potential_sol.data[idx], idx, threadIdx.x);
}

/**
 * CudaParamClusterData ----> CudaParamIndividual * as output
 */
template <int T>
__global__ void UpdateVecParamBasedClusterData(CudaParamIndividual *output, CudaParamClusterData<T> *cluster_data){
    ConvertCudaParamRevert<T>(cluster_data, &output[blockIdx.x], blockIdx.x, threadIdx.x);
}

/**
 * CudaParamClusterData<T> * ---->  CudaEvolveData* 
 */
__global__ void UpdateEvolveWarmStartBasedClusterData(CudaEvolveData *evolve_data, CudaParamClusterData<CUDA_SOLVER_POP_SIZE*3> *old_param){
    ConvertCudaParamRevert<CUDA_SOLVER_POP_SIZE*3>(old_param, &evolve_data->warm_start, 0, threadIdx.x);
}

__global__ void SaveNewParamAsOldParam(CudaParamClusterData<CUDA_SOLVER_POP_SIZE> *new_param, CudaParamClusterData<CUDA_SOLVER_POP_SIZE*3> *old_param, int left_bound, int right_bound, int bias){
    int sol_id = blockIdx.x;
    if (sol_id < left_bound || sol_id >= right_bound)   return;
    ConvertCudaParamBetweenClusters<CUDA_SOLVER_POP_SIZE, CUDA_SOLVER_POP_SIZE*3>(new_param, old_param, sol_id, sol_id + bias, threadIdx.x);
}

__global__ void GenerativeRandSolNearBest(CudaEvolveData *evolve, CudaParamClusterData<CUDA_SOLVER_POP_SIZE> *new_param, float *uniform_data, int rand_idx, float delta_con, float delta_int, int size){
    int sol_id = blockIdx.x;
    int param_id = threadIdx.x;

    if (sol_id == 0 || sol_id >= size)  return;
    float upper_bound = evolve->upper_bound[param_id];
    float lower_bound = evolve->lower_bound[param_id];

    if (param_id < evolve->problem_param.con_var_dims){
        float rand_range = (upper_bound - lower_bound) * delta_con;

        // based on rand_range update the boundary
        upper_bound = min(upper_bound, new_param->all_param[param_id] + rand_range);
        lower_bound = max(lower_bound, new_param->all_param[param_id] - rand_range);
        
        // based on new boundary near parameter to generate the new parameter
        new_param->all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id] = lower_bound + uniform_data[CUDA_SOLVER_POP_SIZE * 100 * CUDA_PARAM_MAX_SIZE + sol_id * CUDA_SOLVER_POP_SIZE + rand_idx + param_id] * (upper_bound - lower_bound);
    }
    else if(param_id < evolve->problem_param.int_var_dims){
        float rand_range = (upper_bound - lower_bound) * delta_int;

        // based on rand_range update the boundary
        upper_bound = min(upper_bound, new_param->all_param[param_id] + rand_range);
        lower_bound = max(lower_bound, new_param->all_param[param_id] - rand_range);
        
        // based on new boundary near parameter to generate the new parameter
        new_param->all_param[sol_id * CUDA_PARAM_MAX_SIZE + param_id] = lower_bound + uniform_data[CUDA_SOLVER_POP_SIZE * 100 * CUDA_PARAM_MAX_SIZE + sol_id * CUDA_SOLVER_POP_SIZE + rand_idx + param_id] * (upper_bound - lower_bound);
    }
}


void CudaDiffEvolveSolver::WarmStart(Problem* problem, CudaParamIndividual* output_sol){
    InitParameter<<<1, CUDA_SOLVER_POP_SIZE, 0, cuda_utils_->streams_[0]>>>(evolve_data_, CUDA_SOLVER_POP_SIZE, new_cluster_data_, old_cluster_data_, random_center_->uniform_data_);
    // CHECK_CUDA(cudaStreamSynchronize(cuda_utils_->streams_[0]));
    if(last_potential_sol_.len > 0){
        if (DEBUG_PRINT_FLAG || DEBUG_PRINT_INIT_SOLVER_FLAG) printf("USING LAST POTENTIAL SOL\n");
        // int half_pop_size = CUDA_SOLVER_POP_SIZE >> 1;
        int quad_pop_size = CUDA_SOLVER_POP_SIZE >> 2;
        // one cluster generate one solution, each cluster works on one block. 
        // We need to generate quad_pop_size new solutions based on last potential solution, so init the new cluster in quad_pop_size grid.
        UpdateClusterDataBasedEvolve<<<quad_pop_size, CUDA_PARAM_MAX_SIZE, 0, cuda_utils_->streams_[0]>>>(evolve_data_, new_cluster_data_, last_potential_sol_.len);
    }
    UpdateVecParamBasedClusterData<CUDA_SOLVER_POP_SIZE><<<CUDA_SOLVER_POP_SIZE, 16, 0, cuda_utils_->streams_[0]>>>(new_cluster_vec_->data, new_cluster_data_);

    // int cet = 10;
    // Update the output param based on warm start.
    // CHECK_CUDA(cudaMemcpyAsync(output_sol, &new_cluster_vec_->data[cet], sizeof(CudaParamIndividual), cudaMemcpyDeviceToHost, cuda_utils_->streams_[0]));

    // Evaluate random solutions or potential solutions in warmstart
    Evaluation(CUDA_SOLVER_POP_SIZE, 0);

    // SortParamBasedBitonic<64><<<16, 64, 0, cuda_utils_->streams_[0]>>>(new_cluster_data_->all_param, new_cluster_data_->fitness);

    // Find the best solution among the random solutions or potential solutions in warmstart and put it in the first place
    ParaFindMax2<CUDA_SOLVER_POP_SIZE, CUDA_SOLVER_POP_SIZE><<<1, CUDA_SOLVER_POP_SIZE, 0, cuda_utils_->streams_[0]>>>(new_cluster_data_);

    // based on warm start result to generate random solution. Further improve the quality of the initial population
    GenerativeRandSolNearBest<<<CUDA_SOLVER_POP_SIZE, CUDA_PARAM_MAX_SIZE, 0, cuda_utils_->streams_[0]>>>(evolve_data_, new_cluster_data_, random_center_->uniform_data_, 16, 0.1, 0.1, CUDA_SOLVER_POP_SIZE);

    // convert the parameter from warm start to old parameter
    SaveNewParamAsOldParam<<<CUDA_SOLVER_POP_SIZE, CUDA_PARAM_MAX_SIZE, 0, cuda_utils_->streams_[0]>>>(new_cluster_data_, old_cluster_data_, 0, CUDA_SOLVER_POP_SIZE, 0);

    // Based on all old parameter to update the warm start of evolve data
    // 将 old_cluster_data_<CUDA_SOLVER_POP_SIZE*3> 中索引为0的数据提取出来,填充到evolve data单个CudaParamIndividual结构中,记为warm start。索引为0的解是warm start过程中最优的
    UpdateEvolveWarmStartBasedClusterData<<<1, CUDA_PARAM_MAX_SIZE, 0, cuda_utils_->streams_[0]>>>(evolve_data_, old_cluster_data_);

    // CHECK_CUDA(cudaStreamSynchronize(cuda_utils_->streams_[0]));
}

// (Abandoned) Use for loop to evaluate 
// template<int T>
// __global__ void MainEvaluation(CudaEvolveData *evolve, CudaParamClusterData<T> *cluster_data){
//     DynamicEvaluation2(evolve, cluster_data, evolve->lambda);
// }

void CudaDiffEvolveSolver::Evaluation(int size, int epoch){
    cart_pole::Compute_NonlinearDynamics<CUDA_SOLVER_POP_SIZE><<<size, 1, 0, cuda_utils_->streams_[0]>>>(new_cluster_data_, score);

    // cart_pole::Compute_linearDynamics<CUDA_SOLVER_POP_SIZE><<<size, 1, 0, cuda_utils_->streams_[0]>>>(new_cluster_data_, cluster_state, state_matrix, control_matrix, score);

    // cart_pole::Evaluate_State(cublas_handle_, state_weight_matrix, state_matrix, state_score, temp_state_score, quadratic_state, cuda_utils_->streams_[0]);

    // cart_pole::Evaluate_Control(cublas_handle_, control_weight_matrix, control_matrix, control_score, temp_control_score, quadratic_control, cuda_utils_->streams_[0]);

    // float alpha = 1.0f;
    // cudaMemset(score, 0, size * sizeof(float));
    // cublasSaxpy(cublas_handle_, size, &alpha, state_score, 1, score, 1);
    // cublasSaxpy(cublas_handle_, size, &alpha, control_score, 1, score, 1);

    if(DEBUG_PRINT_FLAG || DEBUG_CART_POLE){
        // state and control
        CHECK_CUDA(cudaMemcpy(h_state_matrix, state_matrix, cart_pole::row_state_matrix * cart_pole::col_state_matrix * sizeof(float), cudaMemcpyDeviceToHost));
        CHECK_CUDA(cudaMemcpy(h_control_matrix, control_matrix, cart_pole::row_control_matrix * cart_pole::col_control_matrix * sizeof(float), cudaMemcpyDeviceToHost));

        // // state evaluation process
        // CHECK_CUDA(cudaMemcpy(h_temp_state_score, temp_state_score, cart_pole::row_temp_state_score * cart_pole::col_temp_state_score * sizeof(float), cudaMemcpyDeviceToHost));
        // CHECK_CUDA(cudaMemcpy(h_quadratic_state, quadratic_state, cart_pole::row_quadratic_state * cart_pole::col_quadratic_state * sizeof(float), cudaMemcpyDeviceToHost));

        // // control evaluation process
        // CHECK_CUDA(cudaMemcpy(h_temp_control_score, temp_control_score, cart_pole::row_temp_control_score * cart_pole::col_temp_control_score * sizeof(float), cudaMemcpyDeviceToHost));
        // CHECK_CUDA(cudaMemcpy(h_quadratic_control, quadratic_control, cart_pole::row_quadratic_control * cart_pole::col_quadratic_control * sizeof(float), cudaMemcpyDeviceToHost));

        // CHECK_CUDA(cudaMemcpy(h_state_score, state_score, size * sizeof(float), cudaMemcpyDeviceToHost));
        // CHECK_CUDA(cudaMemcpy(h_control_score, control_score, size * sizeof(float), cudaMemcpyDeviceToHost));
        CHECK_CUDA(cudaMemcpy(h_score, score, size * sizeof(float), cudaMemcpyDeviceToHost));

        CHECK_CUDA(cudaStreamSynchronize(cuda_utils_->streams_[0]));
        
        // int row_state_matrix = cart_pole::row_state_matrix, col_state_matrix = cart_pole::col_state_matrix;
        // int row_control_matrix = cart_pole::row_control_matrix, col_control_matrix = cart_pole::col_control_matrix;
        PrintMatrixByRow(h_state_matrix, cart_pole::row_state_matrix, cart_pole::col_state_matrix, "state matrix");
        // PrintMatrixByRow(h_temp_state_score, cart_pole::row_temp_state_score, cart_pole::col_temp_state_score, "Qx^T result");
        // PrintMatrixByRow(h_state_score, size, 1, "state score");
        // PrintMatrixByRow(h_control_matrix, cart_pole::row_control_matrix, cart_pole::col_control_matrix, "control matrix");
        PrintMatrixByRow(h_score, size, 1, "score");
    }
    
    UpdateFitnessBasedMatrix<CUDA_SOLVER_POP_SIZE><<<1, size, 0, cuda_utils_->streams_[0]>>>(new_cluster_data_, score);
    // CHECK_CUDA(cudaStreamSynchronize(cuda_utils_->streams_[0]));
    // // Row-major arrangement (size x dims+1 matrix)
    // // ConvertClusterToMatrix<64><<<size, dims_ + 1, 0, cuda_utils_->streams_[0]>>>(evolve_data_, new_cluster_data_, param_matrix);

    // // row-major arrangement (dims+1 x size matrix)
    // ConvertClusterToMatrix2<64><<<dims_ + 1, size, 0, cuda_utils_->streams_[0]>>>(evolve_data_, new_cluster_data_, param_matrix, size);

    // // printf("device obj_constant_matrix\n");
    // // printMatrix<<<1, row_obj_constant*col_obj_constant, 0, cuda_utils_->streams_[0]>>>(obj_constant_matrix);

    // // printf("device objective_matrix\n");
    // // printMatrix<<<1, row_obj*col_obj, 0, cuda_utils_->streams_[0]>>>(objective_matrix);
    
    // float alpha = 1.;
    // float beta = 1.;

    // // reset the evaluate score and tmp score
    // cudaMemset(evaluate_score_, 0, size * sizeof(float));

    // cudaMemset(constraint_score, 0, size * row_constraint * sizeof(float));

    // // Based on current epoch and interpolation to update lambda
    // UpdateLambdaBasedInterpolation<<<1, row_lambda * col_lambda, 0, cuda_utils_->streams_[0]>>>(evolve_data_, lambda_matrix, epoch);

    // // checking before matrix multiplication
    // if(DEBUG_PRINT_FLAG || DEBUG_PRINT_EVALUATE_FLAG){
        
    //     CHECK_CUDA(cudaMemcpy(host_param_matrix, param_matrix, (dims_+1) * size * sizeof(float), cudaMemcpyDeviceToHost));
    //     CHECK_CUDA(cudaMemcpy(h_objective_matrix, objective_matrix, row_obj * col_obj * sizeof(float), cudaMemcpyDeviceToHost));
    //     CHECK_CUDA(cudaMemcpy(h_constraint_matrix, constraint_matrix, row_constraint * col_constraint * sizeof(float), cudaMemcpyDeviceToHost));
    //     CHECK_CUDA(cudaMemcpy(h_lambda_matrix, lambda_matrix, row_lambda * col_lambda * sizeof(float), cudaMemcpyDeviceToHost));

    //     CHECK_CUDA(cudaStreamSynchronize(cuda_utils_->streams_[0]));

    //     // check parameter matrix before matrix multiplication
    //     PrintMatrixByRow(host_param_matrix,  dims_+1, size, "CHECK PARAMETER MATRIX BEFORE MATRIX MULTIPLICATION");

    //     // check obj matrix before matrix multiplication
    //     PrintMatrixByRow(h_objective_matrix, row_obj, col_obj, "CHECK OBJ MATRIX BEFORE MATRIX MULTIPLICATION");

    //     // check constraint matrix before matrix multiplication
    //     PrintMatrixByRow(h_constraint_matrix, row_constraint, col_constraint, "CHECK CONSTRAINT MATRIX BEFORE MATRIX MULTIPLICATION");

    //     // check lambda matrix before matrix multiplication
    //     PrintMatrixByRow(h_lambda_matrix, row_lambda, col_lambda, "CHECK LAMBDA MATRIX BEFORE MATRIX MULTIPLICATION");
    // }

    // // example:
    // // param_matrix: pop_size x dims (64 x 3)
    // // objective_matrix: 3 x 1
    // // obj_constant_matrix: 64 x 1
    // //     cublasSgemm(
    // //     handle,
    // //     CUBLAS_OP_T,  // A is stored row-first, so it is considered transposed, otherwise CUBLAS_OP_N
    // //     CUBLAS_OP_T,  // B is stored in row-major order, so it is considered transposed, otherwise CUBLAS_OP_N
    // //     m, n, k,     // m: the number of rows in C; n: the number of columns in C; k: the number of columns in A; 
    // //     &alpha,
    // //     A, lda,  // leading dimension of A. If the matrix is ​​stored row-major, lda should be the number of columns in the matrix. Otherwise, lda should be the number of rows.
    // //     B, ldb,  // leading dimension of B
    // //     &beta,
    // //     C, ldc   // C 的 leading dimension
    // // );
    // // Strongly recommend reading this blog: https://blog.csdn.net/HaoBBNuanMM/article/details/103054357
    // // c^Tx
    // cublasSgemm(cublas_handle_, CUBLAS_OP_N, CUBLAS_OP_N, size, 1,  dims_ + 1, &alpha, param_matrix, size, objective_matrix, col_obj, &beta, evaluate_score_, size);

    // // x^TQx
    // if (objective_Q_matrix != nullptr){
    //     cudaMemset(quad_matrix, 0, size * size * sizeof(float));
    //     cudaMemset(quad_transform, 0, row_obj_Q * size * sizeof(float));
    //     cudaMemset(quadratic_score, 0, 1 * size * sizeof(float));
    //     cublasSgemm(cublas_handle_, CUBLAS_OP_N, CUBLAS_OP_N, size, row_obj_Q,  dims_ + 1, &alpha, param_matrix, size, objective_Q_matrix, col_obj_Q, &beta, quad_transform, size);
    //     // quad_matrix size x size
    //     cublasSgemm(cublas_handle_, CUBLAS_OP_N, CUBLAS_OP_T, size, size,  dims_ + 1, &alpha, param_matrix, size, quad_transform, size, &beta, quad_matrix, size);

    //     extractDiagonal<<<1, size, 0, cuda_utils_->streams_[0]>>>(quad_matrix, quadratic_score, size);
    // }

    // // printf("CHECK THE PARAM OF cublasSgemm: %d %d %d %d %d %d\n", col_constraint, size, dims_+1, dims_+1, col_constraint , size);
    // cublasSgemm(cublas_handle_, CUBLAS_OP_N, CUBLAS_OP_N, size, row_constraint, dims_ + 1, &alpha, param_matrix, size, constraint_matrix, col_constraint, &beta, constraint_score, size);

    // InequalityMask<<<1, size * row_constraint, 0, cuda_utils_->streams_[0]>>>(constraint_score);

    // cublasSgemm(cublas_handle_, CUBLAS_OP_N, CUBLAS_OP_N, size, 1, col_lambda, &alpha, constraint_score, size, lambda_matrix, col_lambda, &beta, evaluate_score_, size);

    // MatrixAdd<<<1, size, 0, cuda_utils_->streams_[0]>>>(quadratic_score, evaluate_score_, size);

    // if(DEBUG_PRINT_FLAG || DEBUG_PRINT_EVALUATE_FLAG){
    //     // GPU DEBUG
    //     // printf("device obj_constant_matrix\n");
    //     // printMatrix<<<1, row_obj_constant*col_obj_constant, 0, cuda_utils_->streams_[0]>>>(obj_constant_matrix);

    //     CHECK_CUDA(cudaMemcpy(host_constraint_score, constraint_score, size * row_constraint * sizeof(float), cudaMemcpyDeviceToHost));
    //     CHECK_CUDA(cudaMemcpy(host_evaluate_score_, evaluate_score_, size * sizeof(float), cudaMemcpyDeviceToHost));
    //     CHECK_CUDA(cudaMemcpy(h_quad_transform, quad_transform, row_obj_Q * size * sizeof(float), cudaMemcpyDeviceToHost));
    //     CHECK_CUDA(cudaMemcpy(host_quad_matrix, quad_matrix, size * size * sizeof(float), cudaMemcpyDeviceToHost));
    //     CHECK_CUDA(cudaMemcpy(h_quadratic_score, quadratic_score, 1 * size * sizeof(float), cudaMemcpyDeviceToHost));
    //     // CHECK_CUDA(cudaMemcpy(host_param_matrix, param_matrix, (dims_ + 1) * size * sizeof(float), cudaMemcpyDeviceToHost));
    //     CHECK_CUDA(cudaStreamSynchronize(cuda_utils_->streams_[0]));

    //     // check tmp Q score
    //     PrintQScore(h_quad_transform, row_obj_Q, size, "CHECK TEMP Q SCORE");

    //     // check Q score
    //     PrintQScore(host_quad_matrix, size, size, "CHECK Q SCORE");

    //     // check Q score
    //     PrintQScore(h_quadratic_score, 1, size, "CHECK Q SCORE");

    //     // check tmp score with individual after matrix multiplication
    //     PrintConstraintScoreWithParam(host_constraint_score, host_param_matrix, row_constraint, size, dims_ + 1, "CHECK TMP SCORE");
    //     // check fitness with individual after matrix multiplication
    //     PrintFitnesssWithParam(host_evaluate_score_, host_param_matrix, 1, size, dims_ + 1, "CHECK FITNESS WITH PARAM");
    // }
    // UpdateFitnessBasedMatrix<64><<<1, size, 0, cuda_utils_->streams_[0]>>>(new_cluster_data_, evaluate_score_);

    // (Abandoned) Use for loop to evaluate 
    // MainEvaluation<64><<<1, size, 0, cuda_utils_->streams_[0]>>>(evolve_data_, new_cluster_data_);
}

void CudaDiffEvolveSolver::Evolution(int epoch, CudaEvolveType search_type){
    DuplicateBestAndReorganize<<<CUDA_PARAM_MAX_SIZE, CUDA_SOLVER_POP_SIZE*3, 0, cuda_utils_->streams_[0]>>>(epoch, old_cluster_data_, 2);
    CudaEvolveProcess<<<CUDA_SOLVER_POP_SIZE, CUDA_PARAM_MAX_SIZE, 0, cuda_utils_->streams_[0]>>>(epoch, old_cluster_data_, new_cluster_data_, random_center_->uniform_data_, random_center_->normal_data_, evolve_data_, CUDA_SOLVER_POP_SIZE, true);
    Evaluation(CUDA_SOLVER_POP_SIZE, epoch);

    // CHECK_CUDA(cudaStreamSynchronize(cuda_utils_->streams_[0]));
    UpdateParameter<CUDA_SOLVER_POP_SIZE><<<CUDA_PARAM_MAX_SIZE, CUDA_SOLVER_POP_SIZE*2, 0, cuda_utils_->streams_[0]>>>(epoch, evolve_data_, new_cluster_data_, old_cluster_data_, terminate_flag, last_fitness);

    CHECK_CUDA(cudaMemcpyAsync(h_terminate_flag, terminate_flag, sizeof(int), cudaMemcpyDeviceToHost, cuda_utils_->streams_[0]));
    CHECK_CUDA(cudaStreamSynchronize(cuda_utils_->streams_[0]));
}

__global__ void ConstructMatrix_A(float *A_matrix){
    if (threadIdx.x >= cart_pole::state_dims * (cart_pole::col_E + cart_pole::col_F + cart_pole::col_Inx))   return;
    if(blockIdx.x == 0){
        if (threadIdx.x >= cart_pole::row_Inx * cart_pole::col_Inx) return;
        int row_idx = threadIdx.x / cart_pole::col_Inx;
        int col_idx = threadIdx.x % cart_pole::col_Inx;
        if (row_idx == col_idx) {            
            A_matrix[row_idx * cart_pole::col_A + col_idx] = cart_pole::Inx[row_idx * cart_pole::col_Inx + col_idx];
        }
    }
    else{
        int local_row_idx = threadIdx.x / (cart_pole::col_E + cart_pole::col_F + cart_pole::col_Inx);
        int local_col_idx = threadIdx.x % (cart_pole::col_E + cart_pole::col_F + cart_pole::col_Inx);

        int global_row_idx = (blockIdx.x - 1) * cart_pole::state_dims + local_row_idx + cart_pole::row_Inx;
        int global_col_idx = (blockIdx.x - 1) * (cart_pole::col_E + cart_pole::col_F) + local_col_idx;

        int matrix_idx = global_row_idx * cart_pole::col_A + global_col_idx;

        if (local_col_idx < cart_pole::col_E) {
            A_matrix[matrix_idx] = -cart_pole::E[local_row_idx * cart_pole::col_E + local_col_idx];
        } 
        else if (local_col_idx < cart_pole::col_E + cart_pole::col_F) {
            A_matrix[matrix_idx] = -cart_pole::F[local_row_idx * cart_pole::col_F + (local_col_idx - cart_pole::col_E)];
        }
        else if (local_col_idx < cart_pole::col_E + cart_pole::col_F + cart_pole::col_Inx) {
            A_matrix[matrix_idx] = cart_pole::Inx[local_row_idx * cart_pole::col_Inx + (local_col_idx - (cart_pole::col_E + cart_pole::col_F))];
        }
    }
}

__global__ void ConstructMatrix_C(float *C_matrix){
    if (threadIdx.x >= cart_pole::num_constraints * (cart_pole::col_H1 + cart_pole::col_H2))   return;

    int local_row_idx = threadIdx.x / (cart_pole::col_H1 + cart_pole::col_H2);
    int local_col_idx = threadIdx.x % (cart_pole::col_H1 + cart_pole::col_H2);

    int global_row_id = blockIdx.x * cart_pole::num_constraints + local_row_idx;
    int global_col_id = blockIdx.x * (cart_pole::col_H1 + cart_pole::col_H2) + local_col_idx;

    int matrix_idx = global_row_id * cart_pole::col_C + global_col_id;

    if (local_col_idx < cart_pole::col_H1){
        C_matrix[matrix_idx] = cart_pole::H1[local_row_idx * cart_pole::col_H1 + local_col_idx];
    }
    else{
        C_matrix[matrix_idx] = cart_pole::H2[local_row_idx * cart_pole::col_H2 + local_col_idx];
    }
}

__global__ void ConstructStateWeight(float *state_weight_matrix){
    if (threadIdx.x >= cart_pole::row_state_weight) return;
    int idx = threadIdx.x % cart_pole::row_Q;
    state_weight_matrix[threadIdx.x * cart_pole::col_state_weight + threadIdx.x] = cart_pole::Q[idx * cart_pole::col_Q + idx];
}

__global__ void ConstructControlWeight(float *control_weight_matrix){
    if (threadIdx.x >= cart_pole::row_control_weight) return;
    control_weight_matrix[threadIdx.x * cart_pole::col_control_weight + threadIdx.x] = cart_pole::control_weight;
}

void CudaDiffEvolveSolver::InitSolver(int gpu_device){
    if(DEBUG_ENABLE_NVTX)   init_range = nvtxRangeStart("Init Different Evolution Solver");

    gpu_device_ = gpu_device;
    random_center_ =std::make_shared<CudaRandomManager>(gpu_device_);

    CHECK_CUDA(cudaSetDevice(gpu_device_));
    if (DEBUG_PRINT_FLAG || DEBUG_PRINT_INIT_SOLVER_FLAG) printf("CUDA SET DEVICE\n");

    dims_ = cart_pole::u_dims;

    con_var_dims_ = dims_;
    int_var_dims_ = 0;

    MallocSetup();

    // Initialize cuBLAS handle
    cublasStatus_t status = cublasCreate(&cublas_handle_);

    InitDiffEvolveParam();

    // initialize the C matrix as zero
    // CHECK_CUDA(cudaMemset(C_matrix, 0, cart_pole::row_C * cart_pole::col_C * sizeof(float)));

    // CHECK_CUDA(cudaMemset(A_matrix, 0, cart_pole::row_A * cart_pole::col_A * sizeof(float)));

    // CHECK_CUDA(cudaMemset(state_weight_matrix, 0, cart_pole::row_state_weight * cart_pole::col_state_weight * sizeof(float)));

    // ConstructMatrix_A<<<cart_pole::N + 1, cart_pole::state_dims * (cart_pole::col_E + cart_pole::col_F + cart_pole::col_Inx), 0, cuda_utils_->streams_[0]>>>(A_matrix);

    // ConstructMatrix_C<<<cart_pole::N, cart_pole::num_constraints * (cart_pole::state_dims + cart_pole::control_input_dims), 0, cuda_utils_->streams_[0]>>>(C_matrix);

    // ConstructStateWeight<<<1, cart_pole::row_state_weight, 0, cuda_utils_->streams_[0]>>>(state_weight_matrix);

    // CHECK_CUDA(cudaMemset(control_weight_matrix, 0, cart_pole::row_control_weight * sizeof(float)));

    // construct the weight matrix of control
    // ConstructControlWeight<<<1, cart_pole::row_control_weight * cart_pole::col_control_weight, 0, cuda_utils_->streams_[0]>>>(control_weight_matrix);
    // cublasSaxpy(cublas_handle_, cart_pole::row_control_weight, &alpha, cart_pole::control_weight, 1, control_weight_matrix, cart_pole::col_control_weight+1);

    // if(DEBUG_PRINT_FLAG || DEBUG_CART_POLE){
    //     // CHECK_CUDA(cudaMemcpyAsync(h_A_matrix, A_matrix, cart_pole::row_A * cart_pole::col_A * sizeof(float), cudaMemcpyDeviceToHost, cuda_utils_->streams_[0]));
    //     // CHECK_CUDA(cudaMemcpyAsync(h_C_matrix, C_matrix, cart_pole::row_C * cart_pole::col_C * sizeof(float), cudaMemcpyDeviceToHost, cuda_utils_->streams_[0]));
    //     CHECK_CUDA(cudaMemcpyAsync(h_state_weight_matrix, state_weight_matrix, cart_pole::row_state_weight * cart_pole::col_state_weight * sizeof(float), cudaMemcpyDeviceToHost, cuda_utils_->streams_[0]));
    //     CHECK_CUDA(cudaMemcpyAsync(h_control_weight_matrix, control_weight_matrix, cart_pole::row_control_weight * cart_pole::col_control_weight * sizeof(float), cudaMemcpyDeviceToHost, cuda_utils_->streams_[0]));
    //     CHECK_CUDA(cudaStreamSynchronize(cuda_utils_->streams_[0]));

    //     PrintMatrixByRow(h_state_weight_matrix, cart_pole::row_state_weight, cart_pole::col_state_weight, "state weight:");
    //     PrintMatrixByRow(h_control_weight_matrix, cart_pole::row_control_weight, cart_pole::col_control_weight, "control weight:");
    // }
    
    if (DEBUG_PRINT_FLAG || DEBUG_PRINT_INIT_SOLVER_FLAG) printf("INIT PARAM FOR DE\n");
    
    // *h_terminate_flag = 0;
    // cudaMemset(terminate_flag, 0, sizeof(int));
    // float init_last_f = CUDA_MAX_FLOAT;
    // CHECK_CUDA(cudaMemcpy(last_fitness, &init_last_f, sizeof(float), cudaMemcpyHostToDevice));

    if(DEBUG_ENABLE_NVTX)   setting_boundary_range = nvtxRangeStart("Init_Solver Setting Boundary");

    SetBoundary();

    host_evolve_data_->problem_param.con_var_dims = con_var_dims_;
    host_evolve_data_->problem_param.dims = dims_;
    host_evolve_data_->problem_param.int_var_dims = int_var_dims_;

    host_evolve_data_->problem_param.max_round = 60;
    
    if (DEBUG_PRINT_FLAG || DEBUG_PRINT_INIT_SOLVER_FLAG) printf("START MEMORY ASYNC\n");

    CHECK_CUDA(cudaMemcpyAsync(evolve_data_, host_evolve_data_, sizeof(CudaEvolveData), cudaMemcpyHostToDevice, cuda_utils_->streams_[0]));

    if (DEBUG_PRINT_FLAG || DEBUG_PRINT_INIT_SOLVER_FLAG) printf("MEMORY ASYNC SUBMIT\n");

    InitCudaEvolveData<<<1, CUDA_SOLVER_POP_SIZE, 0, cuda_utils_->streams_[0]>>>(evolve_data_, old_cluster_data_, CUDA_SOLVER_POP_SIZE);

    printf("FINISH INIT SOLVER\n");
}

void CudaDiffEvolveSolver::UpdateCartPoleSystem(float sys_state[4], float wall_pos[2]){
    float4 new_state = {sys_state[0], sys_state[1], sys_state[2], sys_state[3]};
    float2 new_wall_pos = {wall_pos[0], wall_pos[1]};
    
    CHECK_CUDA(cudaMemcpyToSymbol(cart_pole::current_state, &new_state, sizeof(float4)));
    CHECK_CUDA(cudaMemcpyToSymbol(cart_pole::current_wall_pos, &new_wall_pos, sizeof(float2)));
    cudaDeviceSynchronize();
}

__global__ void LoadWarmStartResultForSolver(CudaEvolveData *evolve, CudaParamClusterData<CUDA_SOLVER_POP_SIZE> *new_param){
    ConvertCudaParam<CUDA_SOLVER_POP_SIZE>(new_param, &evolve->warm_start, blockIdx.x, threadIdx.x);
}

template <int T=CUDA_SOLVER_POP_SIZE*3>
__global__ void GetSolFromOldParam(CudaParamClusterData<CUDA_SOLVER_POP_SIZE*3> *old_param, CudaParamIndividual *solution){
    ConvertCudaParamRevert<CUDA_SOLVER_POP_SIZE*3>(old_param, solution, blockIdx.x, threadIdx.x);
}

CudaParamIndividual CudaDiffEvolveSolver::Solver(bool enable_warmstart){
    // nvtx3::mark("Different Evolvution Solver!");
    if(DEBUG_ENABLE_NVTX)   solver_range = nvtxRangeStart("Different Evolvution Solver");

    init_pop_size_ = CUDA_SOLVER_POP_SIZE;
    pop_size_ = CUDA_SOLVER_POP_SIZE;

    InitCudaEvolveData<<<1, CUDA_SOLVER_POP_SIZE, 0, cuda_utils_->streams_[0]>>>(evolve_data_, old_cluster_data_, CUDA_SOLVER_POP_SIZE);

    InitParameter<<<1, CUDA_SOLVER_POP_SIZE, 0, cuda_utils_->streams_[0]>>>(evolve_data_, CUDA_SOLVER_POP_SIZE, new_cluster_data_, old_cluster_data_, random_center_->uniform_data_);

    if(enable_warmstart)    LoadWarmStartResultForSolver<<<1, CUDA_PARAM_MAX_SIZE, 0, cuda_utils_->streams_[0]>>>(evolve_data_, new_cluster_data_);

    // based on warm start result to generate 
    GenerativeRandSolNearBest<<<CUDA_SOLVER_POP_SIZE, CUDA_PARAM_MAX_SIZE, 0, cuda_utils_->streams_[0]>>>(evolve_data_, new_cluster_data_, random_center_->uniform_data_, 16, 0.1, 0.1, CUDA_SOLVER_POP_SIZE);

    Evaluation(CUDA_SOLVER_POP_SIZE, 0);

    ParaFindMax2<CUDA_SOLVER_POP_SIZE, CUDA_SOLVER_POP_SIZE><<<1, CUDA_SOLVER_POP_SIZE, 0, cuda_utils_->streams_[0]>>>(new_cluster_data_);

    SaveNewParamAsOldParam<<<CUDA_SOLVER_POP_SIZE, CUDA_PARAM_MAX_SIZE, 0, cuda_utils_->streams_[0]>>>(new_cluster_data_, old_cluster_data_, 0, CUDA_SOLVER_POP_SIZE, 0);

    for (int i = 0; i < host_evolve_data_->problem_param.max_round && !*h_terminate_flag; ++i) {
        // printf("generation i:%d\n", i);
        Evolution(i, CudaEvolveType::GLOBAL);
    }

    if (DEBUG_PRINT_FLAG || DEBUG_PRINT_SOLVER_FLAG){
        // CHECK_CUDA(cudaMemcpyAsync(host_new_cluster_data_, new_cluster_data_, sizeof(CudaParamClusterData<64>), cudaMemcpyDeviceToHost, cuda_utils_->streams_[0]));
        // PrintClusterData(host_new_cluster_data_);
        CHECK_CUDA(cudaMemcpyAsync(host_old_cluster_data_, old_cluster_data_, sizeof(CudaParamClusterData<CUDA_SOLVER_POP_SIZE*3>), cudaMemcpyDeviceToHost, cuda_utils_->streams_[0]));
        CHECK_CUDA(cudaStreamSynchronize(cuda_utils_->streams_[0]));
        PrintClusterData<CUDA_SOLVER_POP_SIZE*3>(host_old_cluster_data_);

        // CHECK_CUDA(cudaMemcpyAsync(host_evolve_data_, evolve_data_, sizeof(CudaEvolveData), cudaMemcpyDeviceToHost, cuda_utils_->streams_[0]));
        // printf("CUDA_MAX_FLOAT %f\n", CUDA_MAX_FLOAT);
    }
    // Get the first individual from old param (after sorting, the first one is the best one)
    GetSolFromOldParam<CUDA_SOLVER_POP_SIZE*3><<<1, CUDA_PARAM_MAX_SIZE, 0, cuda_utils_->streams_[0]>>>(old_cluster_data_, result);
    // 将 old_cluster_data_<CUDA_SOLVER_POP_SIZE*3> 中索引为0的数据提取出来,填充到evolve data单个CudaParamIndividual结构中,记为warm start。索引为0的解是warm start过程中最优的
    UpdateEvolveWarmStartBasedClusterData<<<1, CUDA_PARAM_MAX_SIZE, 0, cuda_utils_->streams_[0]>>>(evolve_data_, old_cluster_data_);
    CHECK_CUDA(cudaMemcpyAsync(host_result, result, sizeof(CudaParamIndividual), cudaMemcpyDeviceToHost, cuda_utils_->streams_[0]));
    CHECK_CUDA(cudaStreamSynchronize(cuda_utils_->streams_[0]));
    cudaDeviceSynchronize();

    // for(int i = con_var_dims_; i < dims_; ++i){
    //     host_result->param[i] = floor(host_result->param[i]);
    // }

    // if (DEBUG_PRINT_FLAG || DEBUG_PRINT_SOLVER_FLAG)   printFinalResult(host_result->fitness, host_result->param, dims_);
    // printFinalResult(host_result->fitness, host_result->param, dims_);

    if(DEBUG_ENABLE_NVTX)   nvtxRangeEnd(solver_range);

    return *host_result;
}

CudaDiffEvolveSolver::~CudaDiffEvolveSolver(){
    if (cudamalloc_flag){
        // GPU device
        CHECK_CUDA(cudaFree(evolve_data_));
        CHECK_CUDA(cudaFree(new_cluster_data_));
        CHECK_CUDA(cudaFree(old_cluster_data_));
        // CHECK_CUDA(cudaFree(new_cluster_vec_));
        CHECK_CUDA(cudaFree(constraint_matrix));
        CHECK_CUDA(cudaFree(objective_matrix));
        CHECK_CUDA(cudaFree(param_matrix));
        CHECK_CUDA(cudaFree(evaluate_score_));
        CHECK_CUDA(cudaFree(constraint_score));
        CHECK_CUDA(cudaFree(quad_matrix));
        CHECK_CUDA(cudaFree(quad_transform));
        CHECK_CUDA(cudaFree(quadratic_score));
        CHECK_CUDA(cudaFree(lambda_matrix));
        CHECK_CUDA(cudaFree(result));

        // CPU host
        if (DEBUG_PRINT_FLAG || DEBUG_PRINT_SOLVER_FLAG){
            CHECK_CUDA(cudaFreeHost(host_new_cluster_data_));
            CHECK_CUDA(cudaFreeHost(host_old_cluster_data_));
        }

        if (DEBUG_PRINT_FLAG || DEBUG_PRINT_EVALUATE_FLAG){
            CHECK_CUDA(cudaFreeHost(host_evaluate_score_));
            CHECK_CUDA(cudaFreeHost(host_param_matrix));
            CHECK_CUDA(cudaFreeHost(host_constraint_score));
            CHECK_CUDA(cudaFreeHost(h_lambda_matrix));
            CHECK_CUDA(cudaFreeHost(h_constraint_matrix));
            CHECK_CUDA(cudaFreeHost(h_objective_matrix));
            CHECK_CUDA(cudaFreeHost(host_quad_matrix));
            CHECK_CUDA(cudaFreeHost(h_quad_transform));
            CHECK_CUDA(cudaFreeHost(h_quadratic_score));
        }
        
        CHECK_CUDA(cudaFreeHost(host_evolve_data_));
        CHECK_CUDA(cudaFreeHost(host_result));
        
    }
}
}