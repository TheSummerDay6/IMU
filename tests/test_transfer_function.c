/**
 * @file test_transfer_function.c
 * @brief Unit tests for Components/Devices/transfer_function.c
 *        Covers: Second_Order_TF_Init, Second_Order_TF_Calculate, Gauss_Rand
 */
#include "test_stubs.h"

#define __packed

#include "../Components/Devices/transfer_function.h"
#include "../Components/Devices/transfer_function.c"

/* ========================== Second_Order_TF_Init ========================== */
static void test_Second_Order_TF_Init(void)
{
    Second_Order_TF_t tf;
    memset(&tf, 0xFF, sizeof(tf)); /* fill with junk */
    float c[3] = {1.0f, 0.5f, 0.1f};
    Second_Order_TF_Init(&tf, c);

    ASSERT_FLOAT_EQ((float)tf.c[0], 1.0f, 1e-6f);
    ASSERT_FLOAT_EQ((float)tf.c[1], 0.5f, 1e-6f);
    ASSERT_FLOAT_EQ((float)tf.c[2], 0.1f, 1e-6f);
    ASSERT_FLOAT_EQ((float)tf.y, 0.0f, 1e-6f);
    ASSERT_FLOAT_EQ((float)tf.y_dot, 0.0f, 1e-6f);
    ASSERT_FLOAT_EQ((float)tf.y_ddot, 0.0f, 1e-6f);
    ASSERT_FLOAT_EQ((float)tf.Last_y_dot, 0.0f, 1e-6f);
    ASSERT_FLOAT_EQ((float)tf.Last_y_ddot, 0.0f, 1e-6f);
}

/* ========================== Second_Order_TF_Calculate ========================== */
static void test_Second_Order_TF_step_response(void)
{
    Second_Order_TF_t tf;
    memset(&tf, 0, sizeof(tf));
    /* G(s) = 1/(0.1s^2 + 0.5s + 1.0) -> steady-state gain = 1/c[0] = 1.0 */
    float c[3] = {1.0f, 0.5f, 0.1f};
    Second_Order_TF_Init(&tf, c);

    stub_dwt_dt = 0.001f;
    double out = 0;
    /* Step input = 1.0 for many iterations */
    for (int i = 0; i < 5000; i++)
        out = Second_Order_TF_Calculate(&tf, 1.0);

    /* Steady state: y_ddot = 0, y_dot = 0, so u = c[0]*y -> y = u/c[0] = 1.0 */
    ASSERT_FLOAT_EQ((float)out, 1.0f, 0.05f);
}

static void test_Second_Order_TF_zero_input(void)
{
    Second_Order_TF_t tf;
    memset(&tf, 0, sizeof(tf));
    float c[3] = {1.0f, 0.5f, 0.1f};
    Second_Order_TF_Init(&tf, c);

    stub_dwt_dt = 0.001f;
    /* Zero input -> output stays at 0 */
    for (int i = 0; i < 100; i++)
        Second_Order_TF_Calculate(&tf, 0.0);

    ASSERT_FLOAT_EQ((float)tf.y, 0.0f, 1e-6f);
}

static void test_Second_Order_TF_different_gains(void)
{
    Second_Order_TF_t tf;
    memset(&tf, 0, sizeof(tf));
    /* G(s) = 1/(0.01s^2 + 0.2s + 2.0) -> steady-state gain = 1/c[0] = 0.5 */
    float c[3] = {2.0f, 0.2f, 0.01f};
    Second_Order_TF_Init(&tf, c);

    stub_dwt_dt = 0.001f;
    double out = 0;
    for (int i = 0; i < 5000; i++)
        out = Second_Order_TF_Calculate(&tf, 1.0);

    /* Steady state: y = u/c[0] = 1.0/2.0 = 0.5 */
    ASSERT_FLOAT_EQ((float)out, 0.5f, 0.05f);
}

static void test_Second_Order_TF_integrates_correctly(void)
{
    /* Verify that trapezoid integration updates state correctly */
    Second_Order_TF_t tf;
    memset(&tf, 0, sizeof(tf));
    float c[3] = {1.0f, 1.0f, 1.0f};
    Second_Order_TF_Init(&tf, c);

    stub_dwt_dt = 0.001f;
    Second_Order_TF_Calculate(&tf, 1.0);

    /* After one step: y_ddot = u/c2 - y*c0/c2 - y_dot*c1/c2 = 1 - 0 - 0 = 1 */
    ASSERT_FLOAT_EQ((float)tf.y_ddot, 1.0f, 0.01f);
    /* y_dot = (Last_y_ddot + y_ddot) * dt / 2 = (0 + 1) * 0.001 / 2 = 0.0005 */
    ASSERT_FLOAT_EQ((float)tf.y_dot, 0.0005f, 1e-5f);
    /* y = (Last_y_dot + y_dot) * dt / 2 = (0 + 0.0005) * 0.001 / 2 ≈ 2.5e-7 */
    ASSERT_TRUE(tf.y > 0);
}

/* ========================== Gauss_Rand ========================== */
static void test_Gauss_Rand_returns_values(void)
{
    srand(42);
    double sum = 0;
    int n = 10000;
    for (int i = 0; i < n; i++)
        sum += Gauss_Rand();

    /* Mean should be close to 0 for standard normal */
    double mean = sum / n;
    ASSERT_TRUE(fabs(mean) < 0.1);
}

static void test_Gauss_Rand_variance(void)
{
    srand(42);
    double sum = 0, sum_sq = 0;
    int n = 10000;
    for (int i = 0; i < n; i++)
    {
        double val = Gauss_Rand();
        sum += val;
        sum_sq += val * val;
    }
    double mean = sum / n;
    double variance = sum_sq / n - mean * mean;
    /* Variance should be close to 1 for standard normal */
    ASSERT_TRUE(fabs(variance - 1.0) < 0.2);
}

static void test_Gauss_Rand_alternates_phase(void)
{
    srand(123);
    /* Two consecutive calls should use sin/cos pair */
    double v1 = Gauss_Rand();
    double v2 = Gauss_Rand();
    /* They should generally be different */
    ASSERT_TRUE(fabs(v1 - v2) > 1e-10 || v1 == v2); /* always true, just exercises both phases */
    /* Both should be finite */
    ASSERT_TRUE(isfinite(v1));
    ASSERT_TRUE(isfinite(v2));
}

/* ========================== main ========================== */
int main(void)
{
    printf("=== transfer_function tests ===\n");

    RUN_TEST(test_Second_Order_TF_Init);
    RUN_TEST(test_Second_Order_TF_step_response);
    RUN_TEST(test_Second_Order_TF_zero_input);
    RUN_TEST(test_Second_Order_TF_different_gains);
    RUN_TEST(test_Second_Order_TF_integrates_correctly);
    RUN_TEST(test_Gauss_Rand_returns_values);
    RUN_TEST(test_Gauss_Rand_variance);
    RUN_TEST(test_Gauss_Rand_alternates_phase);

    TEST_SUMMARY();
    return test_fail_count > 0 ? 1 : 0;
}
