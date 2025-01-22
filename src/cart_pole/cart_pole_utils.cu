#include "cart_pole/cart_pole_utils.cuh"

namespace cart_pole{
    __constant__ float E[16] = {
        1.0f + 0.0f*runtime_step, 0.0f*runtime_step,        runtime_step,      0.0f*runtime_step,
        0.0f*runtime_step,        1.0f + 0.0f*runtime_step, 0.0f,    runtime_step,
        0.0f*runtime_step,        g*mp/mc*runtime_step,      1.0f,    0.0f*runtime_step,
        0.0f*runtime_step,        g*(mc+mp)/(ll*mc)*runtime_step, 0.0f, 1.0f
    };

    __constant__ float F[4] = {
        0.0f,
        0.0f,
        runtime_step/mc,
        runtime_step/(ll*mc)
    };

    __constant__ float G[8] = {
        0.0f,                   0.0f,
        0.0f,                   0.0f,
        0.0f,                   0.0f,
        runtime_step/(ll*mp),  runtime_step/(ll*mp)
    };

    __constant__ float Q[16] = {
        1.0f,  0.0f,  0.0f,  0.0f,
        0.0f, 1.0f,  0.0f,  0.0f,
        0.0f,  0.0f,  50.0f,  0.0f,
        0.0f,  0.0f,  0.0f, 50.0f
    };

    __constant__ float R[9] = {
        0.1f, 0.0f, 0.0f,
        0.0f, 0.1f, 0.0f,
        0.0f, 0.0f, 0.1f
    };

    __constant__ float H1[80] = {
        0.0f,  0.0f,  0.0f,  0.0f,
        0.0f,  0.0f,  0.0f,  0.0f,
        -1.0f,  ll,   0.0f,  0.0f,
        1.0f,  -ll,   0.0f,  0.0f,
        1.0f,  -ll,   0.0f,  0.0f,
        -1.0f,  ll,   0.0f,  0.0f,
        1.0f,  0.0f,  0.0f,  0.0f,
        -1.0f, 0.0f,  0.0f,  0.0f,
        0.0f,  1.0f,  0.0f,  0.0f,
        0.0f, -1.0f,  0.0f,  0.0f,
        0.0f,  0.0f,  1.0f,  0.0f,
        0.0f,  0.0f, -1.0f,  0.0f,
        0.0f,  0.0f,  0.0f,  1.0f,
        0.0f,  0.0f,  0.0f, -1.0f,
        0.0f,  0.0f,  0.0f,  0.0f,
        0.0f,  0.0f,  0.0f,  0.0f,
        0.0f,  0.0f,  0.0f,  0.0f,
        0.0f,  0.0f,  0.0f,  0.0f,
        0.0f,  0.0f,  0.0f,  0.0f,
        0.0f,  0.0f,  0.0f,  0.0f
    };

    __constant__ float H2[60] = {
        0.0f,     1.0f,     0.0f,
        0.0f,     0.0f,     1.0f,
        0.0f,  1.0f/k1,     0.0f,
        0.0f, -1.0f/k1,     0.0f,
        0.0f,     0.0f,  1.0f/k2,
        0.0f,     0.0f, -1.0f/k2,
        0.0f,     0.0f,     0.0f,
        0.0f,     0.0f,     0.0f,
        0.0f,     0.0f,     0.0f,
        0.0f,     0.0f,     0.0f,
        0.0f,     0.0f,     0.0f,
        0.0f,     0.0f,     0.0f,
        0.0f,     0.0f,     0.0f,
        0.0f,     0.0f,     0.0f,
        1.0f,     0.0f,     0.0f,
        -1.0f,    0.0f,     0.0f,
        0.0f,     1.0f,     0.0f,
        0.0f,    -1.0f,     0.0f,
        0.0f,     0.0f,     1.0f,
        0.0f,     0.0f,    -1.0f
    };

    __constant__ float H3[40] = {
        -lam_max,      0.0f,
            0.0f, -lam_max,
        d_max,      0.0f,
            0.0f,      0.0f,
            0.0f,    d_max,
            0.0f,      0.0f,
            0.0f,      0.0f,
            0.0f,      0.0f,
            0.0f,      0.0f,
            0.0f,      0.0f,
            0.0f,      0.0f,
            0.0f,      0.0f,
            0.0f,      0.0f,
            0.0f,      0.0f,
            0.0f,      0.0f,
            0.0f,      0.0f,
            0.0f,      0.0f,
            0.0f,      0.0f,
            0.0f,      0.0f,
            0.0f,      0.0f
    };

    __constant__ float Inx[16] = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };

    __constant__ float4 current_state = {0,0,0,0};
    __constant__ float2 current_wall_pos = {0,0};
}
