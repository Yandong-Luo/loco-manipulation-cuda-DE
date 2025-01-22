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

#define MATRIX_SIZE 40  // 矩阵大小

__constant__ float Q[16] = {
    1.0f,  0.0f,  0.0f,  0.0f,
    0.0f, 50.0f,  0.0f,  0.0f,
    0.0f,  0.0f,  1.0f,  0.0f,
    0.0f,  0.0f,  0.0f, 50.0f
};

// 用于打印矩阵的辅助函数
void printMatrix(float* matrix, int size) {
    for(int i = 0; i < size; i++) {
        std::cout << "[ ";
        for(int j = 0; j < size; j++) {
            std::cout << matrix[i * size + j] << " ";
        }
        std::cout << "]" << std::endl;
    }
}

int main(void) {
    // 创建 CUBLAS 句柄
    cublasHandle_t handle;
    cublasCreate(&handle);

    // 在主机和设备上分配内存
    float* h_matrix = new float[MATRIX_SIZE * MATRIX_SIZE]();  // 初始化为0
    float* d_matrix;
    float* d_ones;
    
    // 为设备矩阵和单位向量分配内存
    cudaMalloc((void**)&d_matrix, MATRIX_SIZE * MATRIX_SIZE * sizeof(float));
    cudaMalloc((void**)&d_ones, MATRIX_SIZE * sizeof(float));
    
    // 将矩阵初始化为0
    cudaMemset(d_matrix, 0, MATRIX_SIZE * MATRIX_SIZE * sizeof(float));
    
    // 在CPU上创建一个全1向量
    float* h_ones = new float[MATRIX_SIZE];
    for(int i = 0; i < MATRIX_SIZE; i++) {
        h_ones[i] = 1.0f;
    }
    
    // 将全1向量复制到GPU
    cudaMemcpy(d_ones, h_ones, MATRIX_SIZE * sizeof(float), cudaMemcpyHostToDevice);

    // 构建单位矩阵
    float alpha = 1.0f;
    // 使用cublasSaxpy设置对角线元素为1
    // incx为0表示使用相同的源值，incy为MATRIX_SIZE+1以跳到下一个对角线元素
    cublasStatus_t status = cublasSaxpy(handle, MATRIX_SIZE, &alpha, 
                                      d_ones, 1,  // 源向量是全1向量
                                      d_matrix, MATRIX_SIZE + 1);  // 步长为MATRIX_SIZE + 1
    
    if (status != CUBLAS_STATUS_SUCCESS) {
        printf("cublasSaxpy failed with error %d\n", status);
        return -1;
    }

    // 将结果拷贝回主机
    cudaMemcpy(h_matrix, d_matrix, MATRIX_SIZE * MATRIX_SIZE * sizeof(float), 
               cudaMemcpyDeviceToHost);

    // 打印结果矩阵
    std::cout << "\nIdentity matrix (" << MATRIX_SIZE << "x" << MATRIX_SIZE << "):" << std::endl;
    printMatrix(h_matrix, MATRIX_SIZE);

    // 验证结果
    bool isCorrect = true;
    for(int i = 0; i < MATRIX_SIZE; i++) {
        for(int j = 0; j < MATRIX_SIZE; j++) {
            float expected = (i == j) ? 1.0f : 0.0f;
            if (std::abs(h_matrix[i * MATRIX_SIZE + j] - expected) > 1e-6) {
                isCorrect = false;
                printf("Error at position (%d,%d): Expected %f, got %f\n", 
                       i, j, expected, h_matrix[i * MATRIX_SIZE + j]);
            }
        }
    }
    
    if (isCorrect) {
        std::cout << "\nIdentity matrix created successfully!" << std::endl;
    } else {
        std::cout << "\nError in creating identity matrix!" << std::endl;
    }

    // 清理资源
    delete[] h_matrix;
    delete[] h_ones;
    cudaFree(d_matrix);
    cudaFree(d_ones);
    cublasDestroy(handle);

    return 0;
}