# Set the minimum required CMake version
cmake_minimum_required(VERSION 3.16) # CUDA support requires CMake 3.16 or later

# Set the project name and languages
project(Parallel_DiffEvolutionSolver LANGUAGES CXX CUDA)

# Find the CUDA Toolkit
find_package(CUDA REQUIRED)

# Find the pybind11
# find_package(pybind11 REQUIRED)

# Find CUBLAS
find_library(CUBLAS_LIBRARY cublas HINTS ${CUDA_TOOLKIT_ROOT_DIR}/lib64)

# Find CURAND
find_library(CURAND_LIBRARY curand HINTS ${CUDA_TOOLKIT_ROOT_DIR}/lib64)

# # Find nlohmann_json
# find_package(nlohmann_json 3.2.0 REQUIRED)

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)

# Create NVTX interface target
add_library(nvtx3-cpp INTERFACE)
target_include_directories(nvtx3-cpp INTERFACE 
    ${PROJECT_SOURCE_DIR}/thirdparty/nvtx/c/include
)
if(UNIX)
    target_link_libraries(nvtx3-cpp INTERFACE ${NVTOOLSEXT_LIBRARY})
endif()

add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/pybind11)

# Include CUDA headers and libraries
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARIES})

# Enable debug symbols and CUDA debug info
set(CMAKE_BUILD_TYPE Debug)

# IF YOU ENABLE THE FOLLOWING THIS LINE, RANDOM CENTER WILL BECOME WEIRD!!!!!!
# set(CMAKE_CUDA_FLAGS_DEBUG "${CMAKE_CUDA_FLAGS_DEBUG} -G -g")
# set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -lineinfo")

# Set CUDA compilation options (optional)
# set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++17")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_CUDA_FLAGS_DEBUG "-g")
set(CMAKE_CUDA_FLAGS_RELEASE "-O3 --use_fast_math -Xcompiler -fPIC -lineinfo")
# set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++17 -lineinfo")


# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

# Include the directory for header files
include_directories(${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/thirdparty/nvtx/c/include)

# # Set the source files (add main.cpp here)
# set(SOURCE_FILES
#     ${PROJECT_SOURCE_DIR}/src/diff_evolution_solver/solver.cu
#     ${PROJECT_SOURCE_DIR}/src/diff_evolution_solver/random_center.cu
#     ${PROJECT_SOURCE_DIR}/src/diff_evolution_solver/random_manager.cu
#     ${PROJECT_SOURCE_DIR}/src/solver_center/solver_center.cu
#     ${PROJECT_SOURCE_DIR}/src/main.cpp
# )

# Recursively search all .cu files under src
file(GLOB_RECURSE ALL_CUDA "${PROJECT_SOURCE_DIR}/src/*.cu")

# find main.cpp
# file(GLOB_RECURSE MAIN_SOURCE "${PROJECT_SOURCE_DIR}/src/diff_evolution_solver/solver.cu")

pybind11_add_module(cuda_solver 
    ${PROJECT_SOURCE_DIR}/src/solver_center/solver_center.cpp
)

# Specify the search path for header files
# target_include_directories(diff_evolution_solver PRIVATE ${PROJECT_SOURCE_DIR}/include)

# Create a static library cuda_DE, using all found .cu files
add_library(cuda_DE STATIC ${ALL_CUDA})

# Set the CUDA architecture (adjust based on your hardware)
set_property(TARGET cuda_DE PROPERTY
    CUDA_ARCHITECTURES 86
)

# Enable separate compilation, allowing __device__ functions to be used in different compilation units
set_target_properties(cuda_DE PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    POSITION_INDEPENDENT_CODE ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
)

# Use the static version of the CUDA runtime library
set(CMAKE_CUDA_RUNTIME_LIBRARY STATIC)

# # Set the C++ standard
# set_target_properties(diff_evolution_solver PROPERTIES
#     CXX_STANDARD 17
#     CXX_STANDARD_REQUIRED YES
# )

# Add the executable target
# add_executable(${PROJECT_NAME} ${MAIN_SOURCE})

set_target_properties(cuda_solver PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib
    OUTPUT_NAME "DE_cuda_solver"  # 设置输出名称
    PREFIX ""  # 删除 lib 前缀
    SUFFIX ".so"  # 直接使用 .so 后缀
)


# Link the executable with CUBLAS
target_link_libraries(cuda_solver PRIVATE
    cuda_DE
    # ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARY}
    ${CURAND_LIBRARY}
    nvtx3-cpp
    # nlohmann_json::nlohmann_json
    yaml-cpp::yaml-cpp
)

# Enable device symbol resolution to ensure device code is linked correctly
set_target_properties(cuda_solver PROPERTIES CUDA_RESOLVE_DEVICE_SYMBOLS ON)


# # Set the minimum required CMake version
# cmake_minimum_required(VERSION 3.16) # CUDA support requires CMake 3.16 or later

# # Set the project name and languages
# project(Parallel_DiffEvolutionSolver LANGUAGES CXX CUDA)

# # Find the CUDA Toolkit
# find_package(CUDA REQUIRED)

# # Find CUBLAS
# find_library(CUBLAS_LIBRARY cublas HINTS ${CUDA_TOOLKIT_ROOT_DIR}/lib64)

# # Find CURAND
# find_library(CURAND_LIBRARY curand HINTS ${CUDA_TOOLKIT_ROOT_DIR}/lib64)

# # # Find nlohmann_json
# # find_package(nlohmann_json 3.2.0 REQUIRED)

# # Find yaml-cpp
# find_package(yaml-cpp REQUIRED)

# # Create NVTX interface target
# add_library(nvtx3-cpp INTERFACE)
# target_include_directories(nvtx3-cpp INTERFACE 
#     ${PROJECT_SOURCE_DIR}/thirdparty/nvtx/c/include
# )
# if(UNIX)
#     target_link_libraries(nvtx3-cpp INTERFACE ${NVTOOLSEXT_LIBRARY})
# endif()

# # Include CUDA headers and libraries
# include_directories(${CUDA_INCLUDE_DIRS})
# link_directories(${CUDA_LIBRARIES})

# # Enable debug symbols and CUDA debug info
# set(CMAKE_BUILD_TYPE Debug)

# # IF YOU ENABLE THE FOLLOWING THIS LINE, RANDOM CENTER WILL BECOME WEIRD!!!!!!
# # set(CMAKE_CUDA_FLAGS_DEBUG "${CMAKE_CUDA_FLAGS_DEBUG} -G -g")
# # set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -lineinfo")

# # Set CUDA compilation options (optional)
# # set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++17")
# # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
# # set(CMAKE_CUDA_FLAGS_DEBUG "-g")
# set(CMAKE_CUDA_FLAGS_RELEASE "-O3 --use_fast_math -Xcompiler -fPIC -lineinfo")
# # set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++17 -lineinfo")


# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

# # Include the directory for header files
# include_directories(${PROJECT_SOURCE_DIR}/include)

# # Set the source files (add main.cpp here)
# set(SOURCE_FILES
#     ${PROJECT_SOURCE_DIR}/src/diff_evolution_solver/solver.cu
#     ${PROJECT_SOURCE_DIR}/src/diff_evolution_solver/random_center.cu
#     ${PROJECT_SOURCE_DIR}/src/diff_evolution_solver/random_manager.cu
#     ${PROJECT_SOURCE_DIR}/src/solver_center/solver_center.cu
#     ${PROJECT_SOURCE_DIR}/src/main.cpp
# )

# # Add the executable target
# cuda_add_executable(diff_evolution_solver ${SOURCE_FILES})

# # Set the CUDA architecture (adjust based on your hardware)
# set_target_properties(diff_evolution_solver PROPERTIES
#     CUDA_ARCHITECTURES "86"
# )

# set_target_properties(diff_evolution_solver PROPERTIES
#     CUDA_SEPARABLE_COMPILATION ON
# )

# # Set the C++ standard
# set_target_properties(diff_evolution_solver PROPERTIES
#     CXX_STANDARD 17
#     CXX_STANDARD_REQUIRED YES
# )

# # Specify the search path for header files
# target_include_directories(diff_evolution_solver PRIVATE ${PROJECT_SOURCE_DIR}/include)

# # Link the executable with CUBLAS
# target_link_libraries(diff_evolution_solver
#     # ${CUDA_LIBRARIES}
#     ${CUBLAS_LIBRARY}
#     ${CURAND_LIBRARY}
#     nvtx3-cpp
#     # nlohmann_json::nlohmann_json
#     yaml-cpp::yaml-cpp
# )