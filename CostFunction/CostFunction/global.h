#pragma once

//#define N_OF_A (360)
//#define N_OF_A (10)


//#define N_OF_A_SQ (N_OF_A*N_OF_A)
#define WS_BORDER (1.0)

#define CAM_ITE 4
#define PAR_KERNEL_LAUNCHS 1
#define DIST_PIXEL (0.0025f)


#define MAX_ITE (PAR_KERNEL_LAUNCHS*CAM_ITE)

#define WEIGHT_W (1.0f)
#define WEIGHT_P (99.0f)
#define WEIGHT_SUM (WEIGHT_W+WEIGHT_P)

#define shared_mem_workspace 16384
#define shared_loop_ite 256
#define shared_mem_loop_ite 128
#define shared_mem_loop_ite_pos 8
#define thread_size 128
#define n_block 25
#define INT_FACTOR 4194304
#define ACCUM_N_WS 1024
#define IS_VALID_POS_N 80938
#define MAX_NUM_CAMS 20

#define VERTEX_BUFFER_SIZE (1364)

#define AVG_BUFFER_SIZE (512)

#define THREADS_MODEL_FITTING (512)
#define MAX_BUFFER_SIZE THREADS_MODEL_FITTING
#define POINT_BUFFER_SIZE (2218)

#define WS_NX 64
#define WS_NY 64
#define WS_NZ 32
#define WS_X_DIM 6.0f
#define WS_Y_DIM 6.0f
#define WS_Z_DIM 3.0f

#define V_DX (WS_X_DIM/(WS_NX-1))
#define V_DY (WS_Y_DIM/(WS_NY-1))
#define V_DZ (WS_Z_DIM/(WS_NZ-1))

#define WS_X_MIN (-3.0f)
#define WS_X_MAX (3.0f)
#define WS_Y_MIN (-3.0f)
#define WS_Y_MAX (3.0f)
#define WS_Z_MIN (0.0f)
#define WS_Z_MAX (3.0f)

#define MIN_DEPTH (0.159375f)
#define MIN_DEPTH_INT (2)
#define MAX_DEPTH_INT (44)
#define IMG_SIZE 3200
#define RES_X 50
#define RES_Y 64
#define WS_ID_SIZE (WS_NX*WS_NY*WS_NZ)
#define MAX_PCL_SIZE 2622
#define SHARED_MEM_WS_INT 4096

#define human_rx (0.1582f)
#define human_ry (0.2488f)
#define human_rz (0.8001f)

#define human_rx_2 (0.02502724f)
#define human_ry_2 (0.06190144f)
#define human_rz_2 (0.64016001f)


#define CONSTANT_MEM_ALLOC_SIZE N_OF_A*N_OF_A*N_OF_A
#define MATH_PI 3.14159265359f
#define DA  (2*MATH_PI/(float)N_OF_A)
#define MINIMUM_POINTS_DETECTION (1)




//
//#define d_nx 64
//#define d_ny 64
//#define d_nz 32
#define d_xdim 6.0f
#define d_ydim 6.0f
#define d_zdim 3.0f

#define N_COEFFS 4
#define N_SAMPLE_TRANSFORMATION 100

#define v_dx (d_xdim/d_nx)
#define v_dy (d_ydim/d_ny)
#define v_dz (d_zdim/d_nz)
#define MIN_DEPTH2 (0.159375f)
#define MAX_DEPTH 7.0f
#define NUMELEM_H 16
#define NUMELEM_C 12
#define NUMELEM_Mi 9
#define NUMELEM_Cp 3