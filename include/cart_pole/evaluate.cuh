#ifndef CUDAPROCESS_CART_POLE_EVALUATE_H
#define CUDAPROCESS_CART_POLE_EVALUATE_H

#include <cublas_v2.h>
#include "diff_evolution_solver/data_type.h"
#include "utils/utils_fun.cuh"

namespace cart_pole{
    void Evaluate_State(cublasHandle_t cublas_handle_, const float *weight, const float *state_matrix, float *state_score, float *temp_state_score, float *quadratic_state, cudaStream_t stream){
        float alpha = 1.;
        float beta = 1.;

        cudaMemset(state_score, 0, row_state_score * col_state_score * sizeof(float));

        cudaMemset(temp_state_score, 0, row_temp_state_score * col_temp_state_score * sizeof(float));

        cudaMemset(quadratic_state, 0, row_quadratic_state * col_quadratic_state * sizeof(float));
        
        // tmp_state_score = weight * state_matrix^T (40X40 x 40XPOP_SIZE)
        cublasSgemm(cublas_handle_, CUBLAS_OP_T, CUBLAS_OP_N, col_temp_state_score, row_temp_state_score,  row_state_weight,      // the row of original state_matrix, the col of weight, the row of weight
                    &alpha, 
                    state_matrix, col_state_matrix,
                    weight, col_state_weight,
                    &beta, 
                    temp_state_score, col_temp_state_score);
        
        // state_matrix * tmp_state_score
        cublasSgemm(cublas_handle_, CUBLAS_OP_N, CUBLAS_OP_N, row_quadratic_state, col_quadratic_state,  row_temp_state_score,    // m,n are the row and col of result matrix, 
                    &alpha, 
                    temp_state_score, col_temp_state_score,                 // temp_state_score is (POP_SIZE x cart_pole::N * cart_pole::state_dims) in cublas, the original is (cart_pole::N * cart_pole::state_dims x POP_SIZE)
                    state_matrix, col_state_matrix,     // state_matrix is (cart_pole::N * cart_pole::state_dims x POP_SIZE) matrix in cublas
                    &beta,
                    quadratic_state, col_quadratic_state);
        
        cudaprocess::extractDiagonal<<<1, CUDA_SOLVER_POP_SIZE, 0, stream>>>(quadratic_state, state_score, CUDA_SOLVER_POP_SIZE);
    }

    void Evaluate_Control(cublasHandle_t cublas_handle_, const float *weight, const float *control_matrix, float *control_score, float *temp_control_score, float *quadratic_control, cudaStream_t stream){
        float alpha = 1.;
        float beta = 1.;

        cudaMemset(control_score, 0, row_control_score * col_control_score * sizeof(float));

        cudaMemset(temp_control_score, 0, row_temp_control_score * col_temp_control_score * sizeof(float));

        cudaMemset(quadratic_control, 0, row_quadratic_control * col_quadratic_control * sizeof(float));
        
        // tmp_state_score = weight * state_matrix^T (40X40 x 40XPOP_SIZE)
        cublasSgemm(cublas_handle_, CUBLAS_OP_T, CUBLAS_OP_N, col_temp_control_score, row_temp_control_score,  row_control_weight,      // the row of original state_matrix, the col of weight, the row of weight
                    &alpha,
                    control_matrix, col_control_matrix,
                    weight, col_control_weight,
                    &beta, 
                    temp_control_score, col_temp_control_score);
        
        // state_matrix * tmp_state_score
        cublasSgemm(cublas_handle_, CUBLAS_OP_N, CUBLAS_OP_N, row_quadratic_control, col_quadratic_control,  row_temp_control_score,    // THE ROW of temp_state_score, the col of state_matrix, the col of temp
                    &alpha, 
                    temp_control_score, col_temp_control_score,                 // temp_state_score is (POP_SIZE x cart_pole::N * cart_pole::state_dims) in cublas, the original is (cart_pole::N * cart_pole::state_dims x POP_SIZE)
                    control_matrix, col_control_matrix,     // state_matrix is (cart_pole::N * cart_pole::state_dims x POP_SIZE) matrix in cublas
                    &beta,
                    quadratic_control, col_quadratic_control);
        
        cudaprocess::extractDiagonal<<<1, CUDA_SOLVER_POP_SIZE, 0, stream>>>(quadratic_control, control_score, CUDA_SOLVER_POP_SIZE);
    }
}

#endif