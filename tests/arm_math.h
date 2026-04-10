/**
 * @file arm_math.h (test stub)
 * @brief Minimal CMSIS-DSP stub for host-side unit testing.
 */
#ifndef _ARM_MATH_H
#define _ARM_MATH_H

#include <stdint.h>
#include <string.h>
#include <math.h>

typedef struct {
    uint16_t numRows;
    uint16_t numCols;
    float *pData;
} arm_matrix_instance_f32;

typedef enum {
    ARM_MATH_SUCCESS = 0
} arm_status;

static inline void arm_mat_init_f32(arm_matrix_instance_f32 *S, uint16_t nRows, uint16_t nCols, float *pData)
{
    S->numRows = nRows;
    S->numCols = nCols;
    S->pData = pData;
}

static inline arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *A, const arm_matrix_instance_f32 *B, arm_matrix_instance_f32 *C)
{
    for (int i = 0; i < A->numRows * A->numCols; i++)
        C->pData[i] = A->pData[i] + B->pData[i];
    return ARM_MATH_SUCCESS;
}

static inline arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *A, const arm_matrix_instance_f32 *B, arm_matrix_instance_f32 *C)
{
    for (int i = 0; i < A->numRows * A->numCols; i++)
        C->pData[i] = A->pData[i] - B->pData[i];
    return ARM_MATH_SUCCESS;
}

static inline arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *A, const arm_matrix_instance_f32 *B, arm_matrix_instance_f32 *C)
{
    for (int i = 0; i < A->numRows; i++)
        for (int j = 0; j < B->numCols; j++) {
            C->pData[i * B->numCols + j] = 0;
            for (int k = 0; k < A->numCols; k++)
                C->pData[i * B->numCols + j] += A->pData[i * A->numCols + k] * B->pData[k * B->numCols + j];
        }
    return ARM_MATH_SUCCESS;
}

static inline arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *C)
{
    for (int i = 0; i < A->numRows; i++)
        for (int j = 0; j < A->numCols; j++)
            C->pData[j * A->numRows + i] = A->pData[i * A->numCols + j];
    return ARM_MATH_SUCCESS;
}

static inline arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *C)
{
    if (A->numRows == 1 && A->numCols == 1) {
        C->pData[0] = 1.0f / A->pData[0];
        return ARM_MATH_SUCCESS;
    }
    memcpy(C->pData, A->pData, sizeof(float) * A->numRows * A->numCols);
    return ARM_MATH_SUCCESS;
}

#endif /* _ARM_MATH_H */
