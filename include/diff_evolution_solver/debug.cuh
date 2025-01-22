#ifndef CUDAPROCESS_DEBUG_OUTPUT_H
#define CUDAPROCESS_DEBUG_OUTPUT_H
#include<string>
#include "diff_evolution_solver/data_type.h"

// template <int T>
// void PrintClusterData(cudaprocess::CudaParamClusterData<T> *cluster_data){
//     printf("cluster_data len:%d\n", T);
//     for(int i = 0; i < CUDA_PARAM_MAX_SIZE * T; ++i){
        
//         if (i % CUDA_PARAM_MAX_SIZE == 0 || i == 0){
//             printf("\n");
//             // if(cluster_data->fitness[i % CUDA_PARAM_MAX_SIZE] < 10)  printf("=================================\n");
//             printf("Individual fitness: %f, and its param: ", cluster_data->fitness[i % CUDA_PARAM_MAX_SIZE]);
//         }
//         printf("%f ", cluster_data->all_param[i]);
//     }
// }

template <int T>
void PrintClusterData(cudaprocess::CudaParamClusterData<T> *cluster_data){
    printf("cluster_data len:%d\n", T);
    
    // 遍历每个个体
    for(int i = 0; i < T; ++i){
        printf("\nIndividual %d fitness: %f, lshade params(f,f1,cr): %f %f %f\nParams: ", 
            i, 
            cluster_data->fitness[i],
            cluster_data->lshade_param[i * 3],     // scale_f
            cluster_data->lshade_param[i * 3 + 1], // scale_f1
            cluster_data->lshade_param[i * 3 + 2]  // crossover
        );
        
        // 打印该个体的所有参数
        for(int j = 0; j < cluster_data->dims; ++j){  // 只打印实际维度的参数
            printf("%f ", cluster_data->all_param[i * CUDA_PARAM_MAX_SIZE + j]);
        }
    }
    printf("\n\nProblem dimensions: con_var=%d, int_var=%d, total=%d\n", 
        cluster_data->con_var_dims, 
        cluster_data->int_var_dims, 
        cluster_data->dims);
}

void PrintEvolveData(cudaprocess::CudaEvolveData *evolve){
    for(int i=0; i < CUDA_PARAM_MAX_SIZE; ++i){
        printf("warmstart:%f ", evolve->warm_start.param[i]);
    }
}

/**
 * EVALUATE OUTPUT
 */
void PrintMatrixByRow(float *mat, int row, int col, std::string output_msg){
    // printf("%s\n",output_msg);
    std::cout<<output_msg<<std::endl;
    printf("[");
    for(int i = 0; i < row; ++i){
        for(int j = 0; j < col; ++j){
            // printf("matrix[%d,%d]=%f ", i, j, mat[i * col + j]);
            printf("%f ", mat[i * col + j]);
        }
        printf("\n");
    }
    printf("]\n");
}

void PrintMatrixByCol(float *obj_mat, int row, int size, std::string output_msg) {
    std::cout << output_msg << std::endl;
    
    for(int i = 0; i < row; ++i) {
        for(int j = 0; j < size; ++j) {
            printf("matrix[%d,%d]=%f ", i, j, obj_mat[j * row + i]);
        }
        printf("\n");
    }
}

void PrintFitnesssWithParam(float *obj_mat, float *param_mat, int row, int obj_mat_col, int dims, std::string output_msg){
    // printf("%s\n",output_msg);
    std::cout<<output_msg<<std::endl;

    // for(int i = 0; i < row; ++i){
    //     for (int j = 0; j < obj_mat_col; ++j){
    //         printf("individual:%d fitness:%f param", i, obj_mat[i * obj_mat_col +j]);
    //     }
        
    //     for (int j = 0; j < dims; ++j) {
    //         printf("[%d, %d]=%f ", i, j, param_mat[i * dims + j]);
    //     }
    //     printf("\n");
    // }
    for(int i = 0; i < obj_mat_col; ++i){
        printf("individual:%d fitness:%f param", i, obj_mat[i]);
        for (int j = 0; j < dims; ++j) {
            printf("[%d, %d]=%f ", i, j, param_mat[j * obj_mat_col + i]);
        }
        printf("\n");
    }
}

void PrintQScore(float *Q_score, int row, int col, std::string output_msg){
    // printf("%s\n",output_msg);
    std::cout<<output_msg<<std::endl;

    // for(int i = 0; i < row; ++i){
    //     for (int j = 0; j < obj_mat_col; ++j){
    //         printf("individual:%d fitness:%f param", i, obj_mat[i * obj_mat_col +j]);
    //     }
        
    //     for (int j = 0; j < dims; ++j) {
    //         printf("[%d, %d]=%f ", i, j, param_mat[i * dims + j]);
    //     }
    //     printf("\n");
    // }
    for(int i = 0; i < row; ++i){
        for (int j = 0; j < col; ++j) {
            printf("[%d, %d]=%f ", i, j, Q_score[i * col + j]);
        }
        printf("\n");
    }
}

void PrintConstraintScoreWithParam(float *tmp_score_matrix, float *param_mat, int row, int tmp_mat_col, int dims, std::string output_msg){
    // printf("%s\n",output_msg);
    std::cout<<output_msg<<std::endl;
    // 64
    for(int i = 0; i < tmp_mat_col; ++i){
        
        printf("%d param: ", i);
        for (int j = 0; j < dims; ++j) {
            printf("%f ", param_mat[j * tmp_mat_col + i]);
        }
        // 2
        for (int j = 0; j < row; ++j){
            printf("tmp score[%d, %d]:%f ", i, j, tmp_score_matrix[j * tmp_mat_col +i]);
        }
        printf("\n");
    }
}

void printMatrix2(float* matrix, int row, int col) {
    printf("%d %d", row, col);
    for(int i=0;i<row;i++)
    {
        std::cout << std::endl;
        std::cout << " [ ";
        for (int j=0; j<col; j++) {
         std::cout << matrix[i * col + j] << " ";
        }
        std::cout << " ] ";
    }
    std::cout << std::endl;
}

__global__ void printDeviceMatrix(float *matrix){
    // int sol_id = blockIdx.x;
    // int param_id = threadIdx.x;
    // param_matrix[blockIdx.x * evolve->dims + threadIdx.x] = cluster_data->all_param[blockIdx.x * CUDA_PARAM_MAX_SIZE + threadIdx.x];
    // param_matrix[blockIdx.x * evolve->dims + threadIdx.x] =
    // cluster_data->all_param[blockIdx.x * CUDA_PARAM_MAX_SIZE + threadIdx.x];

    printf("matrix parameter[%d] is %f\n", threadIdx.x, matrix[threadIdx.x]);
    // printf("finish the convert: param[%d] to matrix[%d], value:%f\n", blockIdx.x * CUDA_PARAM_MAX_SIZE + threadIdx.x, blockIdx.x * evolve->dims + threadIdx.x, param_matrix[blockIdx.x * evolve->dims + threadIdx.x]);
}

void printFinalResult(float fitness, float *param, float dims){
    printf("Fitness:%f param: ", fitness);
    for(int i = 0; i < dims; ++i){
        printf("%f ", param[i]);
    }
    printf("\n");
}


#endif