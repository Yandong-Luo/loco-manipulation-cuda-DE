#include <stdio.h>
#include <assert.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <sys/stat.h>
#include <cmath>
#include <time.h>
#include <cuda_runtime_api.h>
#include <cublas_v2.h>
#include <memory>
#include <string.h>
#include <cstdint>

#define N 4    // 向量维度
#define COL 2  // 列数

__constant__ float Q[16] = {
    1.0f,  0.0f,  0.0f,  0.0f,
    0.0f, 50.0f,  0.0f,  0.0f,
    0.0f,  0.0f,  1.0f,  0.0f,
    0.0f,  0.0f,  0.0f, 50.0f
};

void printMatrix(float (*matrix)[4], int row) {
    for(int i = 0; i < row; i++) {
        std::cout << "[ ";
        for(int j = 0; j < 4; j++) {
            std::cout << matrix[i][j] << " ";
        }
        std::cout << "]" << std::endl;
    }
}

int main(void) {
    float alpha = 1.0;
    float beta = 0.0;
    
    // 初始化数据
    // float h_x[N][COL] = {{1,1},{2,2},{3,3},{4,4}};
    float h_x[2][4] = {{-0.023370, 0.1657, -0.618915, 5.1048},{1,2,3,4}};
    float h_Q[N][N] = {           // 示例对称矩阵
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
    float h_temp[4][2] = {0};   // 临时存储 Qx 的结果
    float h_result[COL][COL] = {0}; // 2x2 结果矩阵

    // 分配设备内存
    float *d_x, *d_Q, *d_temp, *d_result;
    cudaMalloc((void**)&d_x, N * COL * sizeof(float));
    cudaMalloc((void**)&d_Q, N * N * sizeof(float));
    cudaMalloc((void**)&d_temp, N * COL * sizeof(float));
    cudaMalloc((void**)&d_result, COL * COL * sizeof(float));

    // 拷贝数据到设备
    cudaMemcpy(d_x, h_x, N * COL * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_Q, h_Q, N * N * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemset(d_temp, 0, N * COL * sizeof(float));

    // 创建 CUBLAS 句柄
    cublasHandle_t handle;
    cublasCreate(&handle);

    // // 步骤1: Qx
    // cublasSgemm(handle,
    //     CUBLAS_OP_N, CUBLAS_OP_N,  // Q * x
    //     2, 4, 4,
    //     &alpha,
    //     d_x, 2,
    //     d_Q, 4,   // Q: N×N
    //     &beta,
    //     d_temp, 2 // Qx: N×COL
    // );

    // // 步骤2: x^T * (Qx)
    // cublasSgemm(handle,
    //     CUBLAS_OP_N, CUBLAS_OP_T,  // 注意这里的转置
    //     2, 2, 4,
    //     &alpha,
    //     d_x, 2,      
    //     d_temp, 2,   
    //     &beta,
    //     d_result, 2 
    // );
    // 第一个cublasSgemm: 计算 Q×x^T
    // 步骤1: Qx^T （注意这里h_x在cublas看来是4×2的矩阵）
    cublasSgemm(handle,
        CUBLAS_OP_T, CUBLAS_OP_N,  // x^T * Q 
        2, 4, 4,                   // m=2(x的列数), n=4(Q的列数), k=4(Q的行数)
        &alpha,
        d_x, 4,                    // x在cublas看来是4×2的，所以lda=4
        d_Q, 4,                    // Q是4×4矩阵
        &beta,
        d_temp, 2                  // 结果是2×4矩阵
    );

    // 步骤2: (x^TQ)x
    cublasSgemm(handle,
        CUBLAS_OP_N, CUBLAS_OP_N,  // temp * x
        2, 2, 4,                   // m=2(temp的行数), n=2(x的列数), k=4(temp的列数)
        &alpha,
        d_temp, 2,                 // temp是2×4矩阵
        d_x, 4,                    // x在cublas看来是4×2的矩阵
        &beta,
        d_result, 2                // 结果是2×2矩阵
    );
    // 拷贝结果回主机
    cudaMemcpy(h_temp, d_temp, 4 * 2 * sizeof(float), cudaMemcpyDeviceToHost);
    // 拷贝结果回主机
    cudaMemcpy(h_result, d_result, COL * COL * sizeof(float), cudaMemcpyDeviceToHost);

    // 打印结果
    std::cout << "X matrix:" << std::endl;
    printMatrix(h_x, 2);
    
    std::cout << "\nQ matrix:" << std::endl;
    for(int i = 0; i < N; i++) {
        std::cout << "[ ";
        for(int j = 0; j < N; j++) {
            std::cout << h_Q[i][j] << " ";
        }
        std::cout << "]" << std::endl;
    }
    
    std::cout << "\nResult matrix (2x2):" << std::endl;
    for(int i = 0; i < COL; i++) {
        std::cout << "[ ";
        for(int j = 0; j < COL; j++) {
            std::cout << h_result[i][j] << " ";
        }
        std::cout << "]" << std::endl;
    }

    std::cout << "\temp matrix (2x2):" << std::endl;
    for(int i = 0; i < 4; i++) {
        std::cout << "[ ";
        for(int j = 0; j < 2; j++) {
            std::cout << h_temp[i][j] << " ";
        }
        std::cout << "]" << std::endl;
    }

    // 清理资源
    cudaFree(d_x);
    cudaFree(d_Q);
    cudaFree(d_temp);
    cudaFree(d_result);
    cublasDestroy(handle);

    return 0;
}