/**
 * @file test_user_lib.c
 * @brief Unit tests for Components/user_lib.c
 *        Covers: Sqrt, ramp_init/calc, abs_limit, sign, float_deadband,
 *        int16_deadline, float_constrain, int16_constrain,
 *        loop_float_constrain, theta_format, float_rounding,
 *        OLS_Init, OLS_Update, OLS_Derivative, OLS_Smooth,
 *        Get_OLS_Derivative, Get_OLS_Smooth
 */
#include "test_stubs.h"

/* Stub headers (main.h, bsp_dwt.h) resolved via -I tests/ path.
   __packed is defined to empty via -D__packed= compiler flag. */
#include "../Components/user_lib.h"
#include "../Components/user_lib.c"

/* ========================== Sqrt ========================== */
static void test_Sqrt_positive(void)
{
    ASSERT_FLOAT_EQ(Sqrt(4.0f), 2.0f, 0.01f);
    ASSERT_FLOAT_EQ(Sqrt(9.0f), 3.0f, 0.01f);
    ASSERT_FLOAT_EQ(Sqrt(2.0f), 1.4142f, 0.01f);
    ASSERT_FLOAT_EQ(Sqrt(1.0f), 1.0f, 0.01f);
    ASSERT_FLOAT_EQ(Sqrt(100.0f), 10.0f, 0.01f);
}

static void test_Sqrt_zero_and_negative(void)
{
    ASSERT_FLOAT_EQ(Sqrt(0.0f), 0.0f, 0.001f);
    ASSERT_FLOAT_EQ(Sqrt(-1.0f), 0.0f, 0.001f);
    ASSERT_FLOAT_EQ(Sqrt(-100.0f), 0.0f, 0.001f);
}

static void test_Sqrt_small_values(void)
{
    ASSERT_FLOAT_EQ(Sqrt(0.01f), 0.1f, 0.01f);
    ASSERT_FLOAT_EQ(Sqrt(0.25f), 0.5f, 0.01f);
}

/* ========================== ramp ========================== */
static void test_ramp_init(void)
{
    ramp_function_source_t ramp;
    ramp_init(&ramp, 0.01f, 10.0f, -5.0f);
    ASSERT_FLOAT_EQ(ramp.frame_period, 0.01f, 1e-6f);
    ASSERT_FLOAT_EQ(ramp.max_value, 10.0f, 1e-6f);
    ASSERT_FLOAT_EQ(ramp.min_value, -5.0f, 1e-6f);
    ASSERT_FLOAT_EQ(ramp.input, 0.0f, 1e-6f);
    ASSERT_FLOAT_EQ(ramp.out, 0.0f, 1e-6f);
}

static void test_ramp_calc_ramps_up(void)
{
    ramp_function_source_t ramp;
    ramp_init(&ramp, 0.01f, 1.0f, -1.0f);
    /* input=100 means out += 100*0.01 = 1.0 per step */
    float result = ramp_calc(&ramp, 100.0f);
    ASSERT_FLOAT_EQ(result, 1.0f, 1e-4f); /* clamped to max */
}

static void test_ramp_calc_ramps_down(void)
{
    ramp_function_source_t ramp;
    ramp_init(&ramp, 0.01f, 1.0f, -1.0f);
    float result = ramp_calc(&ramp, -200.0f);
    ASSERT_FLOAT_EQ(result, -1.0f, 1e-4f); /* clamped to min */
}

static void test_ramp_calc_accumulates(void)
{
    ramp_function_source_t ramp;
    ramp_init(&ramp, 0.01f, 10.0f, -10.0f);
    ramp_calc(&ramp, 10.0f); /* out = 0.1 */
    ramp_calc(&ramp, 10.0f); /* out = 0.2 */
    float result = ramp_calc(&ramp, 10.0f); /* out = 0.3 */
    ASSERT_FLOAT_EQ(result, 0.3f, 1e-4f);
}

/* ========================== abs_limit ========================== */
static void test_abs_limit(void)
{
    ASSERT_FLOAT_EQ(abs_limit(5.0f, 3.0f), 3.0f, 1e-6f);
    ASSERT_FLOAT_EQ(abs_limit(-5.0f, 3.0f), -3.0f, 1e-6f);
    ASSERT_FLOAT_EQ(abs_limit(2.0f, 3.0f), 2.0f, 1e-6f);
    ASSERT_FLOAT_EQ(abs_limit(-2.0f, 3.0f), -2.0f, 1e-6f);
    ASSERT_FLOAT_EQ(abs_limit(0.0f, 3.0f), 0.0f, 1e-6f);
}

/* ========================== sign ========================== */
static void test_sign(void)
{
    ASSERT_FLOAT_EQ(sign(5.0f), 1.0f, 1e-6f);
    ASSERT_FLOAT_EQ(sign(-5.0f), -1.0f, 1e-6f);
    ASSERT_FLOAT_EQ(sign(0.0f), 1.0f, 1e-6f);  /* >= 0 returns 1 */
}

/* ========================== float_deadband ========================== */
static void test_float_deadband(void)
{
    /* Value inside deadband -> 0 */
    ASSERT_FLOAT_EQ(float_deadband(0.5f, -1.0f, 1.0f), 0.0f, 1e-6f);
    /* Value outside deadband -> unchanged */
    ASSERT_FLOAT_EQ(float_deadband(2.0f, -1.0f, 1.0f), 2.0f, 1e-6f);
    ASSERT_FLOAT_EQ(float_deadband(-2.0f, -1.0f, 1.0f), -2.0f, 1e-6f);
    /* Value at boundary -> unchanged (not strictly inside) */
    ASSERT_FLOAT_EQ(float_deadband(1.0f, -1.0f, 1.0f), 1.0f, 1e-6f);
    ASSERT_FLOAT_EQ(float_deadband(-1.0f, -1.0f, 1.0f), -1.0f, 1e-6f);
}

/* ========================== int16_deadline ========================== */
static void test_int16_deadline(void)
{
    ASSERT_INT_EQ(int16_deadline(0, -10, 10), 0);
    ASSERT_INT_EQ(int16_deadline(5, -10, 10), 0);
    ASSERT_INT_EQ(int16_deadline(15, -10, 10), 15);
    ASSERT_INT_EQ(int16_deadline(-15, -10, 10), -15);
    ASSERT_INT_EQ(int16_deadline(10, -10, 10), 10); /* at boundary */
}

/* ========================== float_constrain ========================== */
static void test_float_constrain(void)
{
    ASSERT_FLOAT_EQ(float_constrain(5.0f, 0.0f, 10.0f), 5.0f, 1e-6f);
    ASSERT_FLOAT_EQ(float_constrain(-5.0f, 0.0f, 10.0f), 0.0f, 1e-6f);
    ASSERT_FLOAT_EQ(float_constrain(15.0f, 0.0f, 10.0f), 10.0f, 1e-6f);
    ASSERT_FLOAT_EQ(float_constrain(0.0f, 0.0f, 10.0f), 0.0f, 1e-6f);
    ASSERT_FLOAT_EQ(float_constrain(10.0f, 0.0f, 10.0f), 10.0f, 1e-6f);
}

/* ========================== int16_constrain ========================== */
static void test_int16_constrain(void)
{
    ASSERT_INT_EQ(int16_constrain(5, 0, 10), 5);
    ASSERT_INT_EQ(int16_constrain(-5, 0, 10), 0);
    ASSERT_INT_EQ(int16_constrain(15, 0, 10), 10);
}

/* ========================== loop_float_constrain ========================== */
static void test_loop_float_constrain(void)
{
    /* Value within range */
    ASSERT_FLOAT_EQ(loop_float_constrain(90.0f, -180.0f, 180.0f), 90.0f, 1e-4f);
    /* Value above range -> wraps */
    ASSERT_FLOAT_EQ(loop_float_constrain(270.0f, -180.0f, 180.0f), -90.0f, 1e-4f);
    /* Value below range -> wraps */
    ASSERT_FLOAT_EQ(loop_float_constrain(-270.0f, -180.0f, 180.0f), 90.0f, 1e-4f);
    /* max < min -> returns input unchanged */
    ASSERT_FLOAT_EQ(loop_float_constrain(5.0f, 10.0f, 0.0f), 5.0f, 1e-4f);
}

/* ========================== theta_format ========================== */
static void test_theta_format(void)
{
    ASSERT_FLOAT_EQ(theta_format(90.0f), 90.0f, 1e-4f);
    ASSERT_FLOAT_EQ(theta_format(270.0f), -90.0f, 1e-4f);
    ASSERT_FLOAT_EQ(theta_format(-270.0f), 90.0f, 1e-4f);
    ASSERT_FLOAT_EQ(theta_format(0.0f), 0.0f, 1e-4f);
    ASSERT_FLOAT_EQ(theta_format(180.0f), 180.0f, 1e-4f);
}

/* ========================== float_rounding ========================== */
static void test_float_rounding(void)
{
    ASSERT_INT_EQ(float_rounding(2.3f), 2);
    ASSERT_INT_EQ(float_rounding(2.6f), 3);
    ASSERT_INT_EQ(float_rounding(2.5f), 2); /* <= 0.5 does not round up */
    ASSERT_INT_EQ(float_rounding(0.0f), 0);
    ASSERT_INT_EQ(float_rounding(-0.3f), 0); /* (int)(-0.3) = 0, decimal = -0.3, not > 0.5 */
}

/* ========================== OLS ========================== */
static void test_OLS_Init(void)
{
    Ordinary_Least_Squares_t ols;
    OLS_Init(&ols, 5);
    ASSERT_INT_EQ(ols.Order, 5);
    ASSERT_INT_EQ(ols.Count, 0);
    ASSERT_FLOAT_EQ(ols.k, 0.0f, 1e-6f);
    ASSERT_FLOAT_EQ(ols.b, 0.0f, 1e-6f);
    ASSERT_TRUE(ols.x != NULL);
    ASSERT_TRUE(ols.y != NULL);
    free(ols.x);
    free(ols.y);
}

static void test_OLS_Derivative_linear_signal(void)
{
    /* Feed a linear signal y = 2*x. OLS derivative should converge to ~2.0 */
    Ordinary_Least_Squares_t ols;
    OLS_Init(&ols, 5);

    float k = 0;
    for (int i = 0; i < 10; i++)
    {
        k = OLS_Derivative(&ols, 1.0f, 2.0f * i);
    }
    ASSERT_FLOAT_EQ(k, 2.0f, 0.1f);
    ASSERT_FLOAT_EQ(Get_OLS_Derivative(&ols), k, 1e-6f);
    free(ols.x);
    free(ols.y);
}

static void test_OLS_Smooth_linear_signal(void)
{
    /* Feed a linear signal. OLS smooth should return close to the line value */
    Ordinary_Least_Squares_t ols;
    OLS_Init(&ols, 5);

    float smooth = 0;
    for (int i = 0; i < 10; i++)
    {
        smooth = OLS_Smooth(&ols, 1.0f, 3.0f * i + 1.0f);
    }
    /* At the last point x=9, y should be ~28 = 3*9+1 */
    ASSERT_FLOAT_EQ(smooth, 28.0f, 1.0f);
    ASSERT_FLOAT_EQ(Get_OLS_Smooth(&ols), smooth, 1e-4f);
    free(ols.x);
    free(ols.y);
}

static void test_OLS_Update(void)
{
    Ordinary_Least_Squares_t ols;
    OLS_Init(&ols, 4);

    for (int i = 0; i < 6; i++)
    {
        OLS_Update(&ols, 1.0f, 5.0f * i + 2.0f);
    }
    /* Slope should be ~5.0, intercept ~2.0 */
    ASSERT_FLOAT_EQ(ols.k, 5.0f, 0.5f);
    free(ols.x);
    free(ols.y);
}

/* ========================== VAL_LIMIT macro ========================== */
static void test_VAL_LIMIT_macro(void)
{
    float val = 5.0f;
    VAL_LIMIT(val, 0.0f, 10.0f);
    ASSERT_FLOAT_EQ(val, 5.0f, 1e-6f);

    val = -5.0f;
    VAL_LIMIT(val, 0.0f, 10.0f);
    ASSERT_FLOAT_EQ(val, 0.0f, 1e-6f);

    val = 15.0f;
    VAL_LIMIT(val, 0.0f, 10.0f);
    ASSERT_FLOAT_EQ(val, 10.0f, 1e-6f);
}

/* ========================== rad_format macro ========================== */
static void test_rad_format_macro(void)
{
    float result = rad_format(0.0f);
    ASSERT_FLOAT_EQ(result, 0.0f, 1e-4f);

    result = rad_format(PI);
    ASSERT_FLOAT_EQ(result, PI, 1e-4f);

    /* 2*PI should wrap to ~0 */
    result = rad_format(2.0f * PI);
    ASSERT_FLOAT_EQ(result, 0.0f, 0.01f);
}

/* ========================== main ========================== */
int main(void)
{
    printf("=== user_lib tests ===\n");

    RUN_TEST(test_Sqrt_positive);
    RUN_TEST(test_Sqrt_zero_and_negative);
    RUN_TEST(test_Sqrt_small_values);
    RUN_TEST(test_ramp_init);
    RUN_TEST(test_ramp_calc_ramps_up);
    RUN_TEST(test_ramp_calc_ramps_down);
    RUN_TEST(test_ramp_calc_accumulates);
    RUN_TEST(test_abs_limit);
    RUN_TEST(test_sign);
    RUN_TEST(test_float_deadband);
    RUN_TEST(test_int16_deadline);
    RUN_TEST(test_float_constrain);
    RUN_TEST(test_int16_constrain);
    RUN_TEST(test_loop_float_constrain);
    RUN_TEST(test_theta_format);
    RUN_TEST(test_float_rounding);
    RUN_TEST(test_OLS_Init);
    RUN_TEST(test_OLS_Derivative_linear_signal);
    RUN_TEST(test_OLS_Smooth_linear_signal);
    RUN_TEST(test_OLS_Update);
    RUN_TEST(test_VAL_LIMIT_macro);
    RUN_TEST(test_rad_format_macro);

    TEST_SUMMARY();
    return test_fail_count > 0 ? 1 : 0;
}
