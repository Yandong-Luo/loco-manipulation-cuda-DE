#ifndef CUDAPROCESS_CART_POLE_MODEL_H
#define CUDAPROCESS_CART_POLE_MODEL_H

#include "diff_evolution_solver/data_type.h"
#include "cart_pole/cart_pole_utils.cuh"
#include <cublas_v2.h>
#include <math.h>
namespace cart_pole{
    // Based on the control input (u) and model of cart pole calculate the next state
    // cart pole model is from https://courses.ece.ucsb.edu/ECE594/594D_W10Byl/hw/cartpole_eom.pdf
    template <int T = CUDA_SOLVER_POP_SIZE>
    __global__ void Compute_NonlinearDynamics(cudaprocess::CudaParamClusterData<T> *cluster_data, float *score_matrix){
        if (threadIdx.x > 0)    return;
        
        // int idx = blockIdx.x * cart_pole::col_state_matrix;
        float pos_weight = Q[0], speed_weight = Q[5], theta_weight = Q[10], dtheta_weight = Q[15];
        float score = 0.0f;
        float4 traj[N] = {0.0f};
        // float history_score[N] = {0.0f};
        // printf("blockid:%d\n",blockIdx.x);
        for (int i = 0; i < N; ++i){
            float pos = 0.0f, speed = 0.0f, theta = 0.0f, dtheta = 0.0f;
            if(i == 0){
                pos = current_state.x, speed = current_state.y, theta = current_state.z, dtheta = current_state.w;
            }
            else{
                // pos = cluster_state->state[(i - 1) + idx].x, speed = cluster_state->state[(i - 1) + idx].y, theta = cluster_state->state[(i - 1) + idx].z, dtheta = cluster_state->state[(i - 1) + idx].w;
                pos = traj[i-1].x, speed = traj[i-1].y, theta = traj[i-1].z, dtheta = traj[i-1].w;
            }    
            float force = cluster_data->all_param[blockIdx.x * CUDA_PARAM_MAX_SIZE];

            float pole_pos1 = -(ll * __sinf(theta) - pos), pole_pos2 = (ll * __sinf(theta) - pos);
            // float pole_pos = (pos - ll * __sinf(theta));
            // printf("pole pos:%f, right wall pos:%f, left wall pos:%f\n", pole_pos1, current_wall_pos.x, current_wall_pos.y);
            float lam1 = 0.0f, lam2 = 0.0f;
            if (pole_pos1 >= current_wall_pos.x){
                lam1 = k1 * (pole_pos1 - current_wall_pos.x);
                // printf("contact right wall, generate the force:%f, right wall pos:%f\n", lam1, current_wall_pos.x);
            }
            else if(-pole_pos2 <= -current_wall_pos.y){
                lam2 = k2 * (pole_pos2 - current_wall_pos.y);
                // printf("contact left wall, generate the force:%f left wall pos:%f\n", lam2, -current_wall_pos.y);
            }

            // control_matrix[blockIdx.x * col_control_matrix + i] = force;

            // float acc = (-mp * ll * __sinf(theta) * dtheta * dtheta + mp * g * __cosf(theta) * __sinf(theta) + force) / (mc + mp - mp * __cosf(theta) * __cosf(theta));

            // float angular_acc = (-mp * ll * __sinf(theta) * __cosf(theta) * dtheta * dtheta + (mc + mp) * g * __sinf(theta) + force * __cosf(theta)) / ((mc + mp * (1 - __cosf(theta) * __cosf(theta))) * ll);
            
            float acc = (-dtheta*dtheta*ll*mp*__sinf(theta) + g*mp*__sinf(2*theta)/2 + lam1*__cosf(theta)*__cosf(theta) - lam1 - lam2*__cosf(theta)*__cosf(theta) + lam2 + force)/(mc + mp*__sinf(theta)*__sinf(theta));

                    // (dtheta*dtheta*ll*mp*__sinf(theta) - g*mp*__sinf(2*theta)/2 - lam1*__cosf(theta)*__cosf(theta) - lam1 + lam2*__cosf(theta)*__cosf(theta) + lam2 + force)/(mc + mp*__sinf(theta)*__sinf(theta));

            float angular_acc = (-dtheta*dtheta*ll*mp*mp*__sinf(2*theta)/2 + g*mc*mp*__sinf(theta) + g*mp*mp*__sinf(theta) + lam1*mc*__cosf(theta) - lam2*mc*__cosf(theta) + mp*force*__cosf(theta))/(ll*mp*(mc + mp*__sinf(theta)*__sinf(theta)));

            // float acc =  (force - ll*dtheta*dtheta*mp*__sinf(theta) + g*mp*__sinf(2*theta)/2)/(mc + mp*__sinf(theta)*__sinf(theta));
            // float angular_acc =  (force*__cosf(theta) - ll*dtheta*dtheta*mp*__sinf(2*theta)/2 + mc*g*__sinf(theta) + g*mp*__sinf(theta))/(ll*(mc + mp*sin(theta)*sin(theta)));

            // float next_pos = pos + acc * runtime_step * runtime_step;
            // float next_speed = speed + acc * runtime_step;
            // float next_theta = theta + angular_acc * runtime_step * runtime_step;
            // float next_dtheta =  dtheta + angular_acc * runtime_step;

            float next_speed = speed + acc * runtime_step;              // v = v0 + at
            float next_pos = pos + speed * runtime_step + 0.5f * acc * runtime_step * runtime_step;  // x = x0 + v0t + 1/2at²

            float next_dtheta = dtheta + angular_acc * runtime_step;    // ω = ω0 + αt
            float next_theta = theta + dtheta * runtime_step + 0.5f * angular_acc * runtime_step * runtime_step;  // θ = θ0 + ω0t + 1/2αt²

            // calculate next position
            // state_matrix[i * state_dims + idx + 0] = cluster_state->state[i + idx].x = next_pos;

            // // calculate next speed
            // state_matrix[i * state_dims + idx + 1] = cluster_state->state[i + idx].y = next_speed;

            // // calculate next velocity
            // state_matrix[i * state_dims + idx + 2] = cluster_state->state[i + idx].z = next_theta;

            // // calculate next dtheta
            // state_matrix[i * state_dims + idx + 3] = cluster_state->state[i + idx].w = next_dtheta;

            traj[i].x = next_pos;
            traj[i].y = next_speed;
            traj[i].z = next_theta;
            traj[i].w = next_dtheta;

            float pole_pos = next_pos - ll * sin(next_theta);
            // pole_pos*pole_pos*50 +
            float current_score = pos_weight * next_pos * next_pos + speed_weight * next_speed * next_speed + theta_weight * next_theta * next_theta + dtheta_weight * next_dtheta * next_dtheta + control_weight * force * force;
            // printf("score:%f, pos_score:%f, speed_score:%f, theta_score:%f, dtheta_score:%f, u_score:%f, angular_acc:%f\n",
            //         current_score,
            //         pos_weight * next_pos * next_pos,
            //         speed_weight * next_speed * next_speed,
            //         theta_weight * next_theta * next_theta,
            //         dtheta_weight * next_dtheta * next_dtheta,
            //         control_weight * force * force,
            //         next_dtheta);

            // float current_score = pos_weight * next_pos * next_pos + speed_weight * next_speed * next_speed + theta_weight * next_theta * next_theta + dtheta_weight * next_dtheta * next_dtheta;
            // printf("block id:%d, time step:%d, score:%f, pos:%f, speed:%f, theta:%f, dtheta:%f, control:%f init_pos:%f, init_speed:%f, init_theta:%f, init_dtheta:%f\n",
            //          blockIdx.x, i, current_score, next_pos, next_speed, next_theta, next_dtheta, force, current_state.x, current_state.y, current_state.z, current_state.w);
            score += current_score;

            // history_score[i] = current_score;

            // constraint for cart pole
            if(fabs(pole_pos) > pos_ub)        score += penalty;
            if(fabs(pos) > pos_ub)            score += penalty;
            if(fabs(theta) > theta_ub)        score += penalty;
            if(fabs(speed) > speed_ub)        score += penalty;
            if(fabs(dtheta) > dtheta_ub)      score += penalty;
        }
        // for(int i = 0; i < N; ++i){
        //     printf("i:%d, hist_score:%f\n",i, history_score[i]);
        // }
        // printf("blockidx:%d, score:%f\n",blockIdx.x, score);
        score_matrix[blockIdx.x] = score;
    }

    template <int T>
    __global__ void Compute_linearDynamics(cudaprocess::CudaParamClusterData<T> *cluster_data, cudaprocess::CartStateList *cluster_state, float *state_matrix, float *control_matrix, float *score_matrix){
        if (threadIdx.x > 0)    return;
        
        int idx = blockIdx.x * cart_pole::col_state_matrix;
        float pos_weight = Q[0], speed_weight = Q[5], theta_weight = Q[10], dtheta_weight = Q[15];
        float score = 0.0f; 
        for (int i = 0; i < N; ++i){
            float pos = 0.0f, speed = 0.0f, theta = 0.0f, dtheta = 0.0f;
            if(i == 0){
                pos = current_state.x, speed = current_state.y, theta = current_state.z, dtheta = current_state.w;
            }
            else{
                pos = cluster_state->state[(i - 1) + idx].x, speed = cluster_state->state[(i - 1) + idx].y, theta = cluster_state->state[(i - 1) + idx].z, dtheta = cluster_state->state[(i - 1) + idx].w;
            }    
            float force = cluster_data->all_param[blockIdx.x * CUDA_PARAM_MAX_SIZE];

            // control_matrix[blockIdx.x * col_control_matrix + i] = force;

            float pole_pos1 = -(ll * theta - pos), pole_pos2 = (ll * theta - pos);
            // float pole_pos = (pos - ll * __sinf(theta));
            // printf("pole pos:%f, right wall pos:%f, left wall pos:%f\n", pole_pos1, current_wall_pos.x, current_wall_pos.y);
            float lam1 = 0.0f, lam2 = 0.0f;
            if (pole_pos1 >= current_wall_pos.x){
                lam1 = k1 * (pole_pos1 - current_wall_pos.x);
                // printf("contact right wall, generate the force:%f, right wall pos:%f\n", lam1, current_wall_pos.x);
            }
            else if(-pole_pos2 <= -current_wall_pos.y){
                lam2 = k2 * (pole_pos2 - current_wall_pos.y);
                // printf("contact left wall, generate the force:%f left wall pos:%f\n", lam2, -current_wall_pos.y);
            }

            float angular_acc = ((mc + mp) * g * theta) / (mc * ll) + force / (mc * ll);

            float acc = (mp *g * theta) / mc + force / mc;

            if(lam1 != 0 || lam2 != 0){
                angular_acc += lam1 / (ll * mp) - lam2 / (ll * mp);
            }

            float next_pos = pos + acc * runtime_step * runtime_step;
            float next_speed = speed + acc * runtime_step;
            float next_theta = theta + angular_acc * runtime_step * runtime_step;
            float next_dtheta =  dtheta + angular_acc * runtime_step;

            // calculate next position
            state_matrix[i * state_dims + idx + 0] = cluster_state->state[i + idx].x = next_pos;

            // calculate next speed
            state_matrix[i * state_dims + idx + 1] = cluster_state->state[i + idx].y = next_speed;

            // calculate next velocity
            state_matrix[i * state_dims + idx + 2] = cluster_state->state[i + idx].z = next_theta;

            // calculate next dtheta
            state_matrix[i * state_dims + idx + 3] = cluster_state->state[i + idx].w = next_dtheta;

            float current_score = pos_weight * next_pos * next_pos + speed_weight * next_speed * next_speed + theta_weight * next_theta * next_theta + dtheta_weight * next_dtheta * next_dtheta + control_weight * force * force;
            // float current_score = pos_weight * next_pos * next_pos + speed_weight * next_speed * next_speed + theta_weight * next_theta * next_theta + dtheta_weight * next_dtheta * next_dtheta;
            // printf("block id:%d, time step:%d, score:%f, pos:%f, speed:%f, theta:%f, dtheta:%f, control:%f init_pos:%f, init_speed:%f, init_theta:%f, init_dtheta:%f\n",
            //          blockIdx.x, i, current_score, next_pos, next_speed, next_theta, next_dtheta, force, current_state.x, current_state.y, current_state.z, current_state.w);
            score += current_score;

            // constraint for cart pole
            if(fabs(pos) > pos_ub)            score += penalty;
            if(fabs(theta) > theta_ub)        score += penalty;
            if(fabs(speed) > speed_ub)        score += penalty;
            if(fabs(dtheta) > dtheta_ub)      score += penalty;
        }
        score_matrix[blockIdx.x] = score;
    }
}

#endif
