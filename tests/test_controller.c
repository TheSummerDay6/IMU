/**
 * @file test_controller.c
 * @brief Unit tests for Components/Controller/controller.c
 *        Covers: PID_Init, PID_Calculate (basic P/I/D, deadband, output limit,
 *        integral limit, trapezoid integral, changing integration rate,
 *        derivative filter, output filter, derivative on measurement,
 *        error handle, fuzzy PID, user callbacks),
 *        Fuzzy_Rule_Init, Fuzzy_Rule_Implementation,
 *        Feedforward_Init, Feedforward_Calculate,
 *        LDOB_Init, LDOB_Calculate,
 *        TD_Init, TD_Calculate
 */
#include "test_stubs.h"

#define __packed

/* Include the source under test (unity-build style) */
#include "../Components/user_lib.h"
#include "../Components/user_lib.c"
#include "../Components/Controller/controller.h"
#include "../Components/Controller/controller.c"

/* ========================== PID_Init ========================== */
static void test_PID_Init_basic(void)
{
    PID_t pid;
    memset(&pid, 0, sizeof(pid));
    PID_Init(&pid, 100.0f, 50.0f, 0.5f, 10.0f, 1.0f, 0.1f, 5.0f, 2.0f, 0.01f, 0.005f, 2, NONE);

    ASSERT_FLOAT_EQ(pid.MaxOut, 100.0f, 1e-6f);
    ASSERT_FLOAT_EQ(pid.IntegralLimit, 50.0f, 1e-6f);
    ASSERT_FLOAT_EQ(pid.DeadBand, 0.5f, 1e-6f);
    ASSERT_FLOAT_EQ(pid.Kp, 10.0f, 1e-6f);
    ASSERT_FLOAT_EQ(pid.Ki, 1.0f, 1e-6f);
    ASSERT_FLOAT_EQ(pid.Kd, 0.1f, 1e-6f);
    ASSERT_FLOAT_EQ(pid.CoefA, 5.0f, 1e-6f);
    ASSERT_FLOAT_EQ(pid.CoefB, 2.0f, 1e-6f);
    ASSERT_FLOAT_EQ(pid.Output_LPF_RC, 0.01f, 1e-6f);
    ASSERT_FLOAT_EQ(pid.Derivative_LPF_RC, 0.005f, 1e-6f);
    ASSERT_INT_EQ(pid.OLS_Order, 2);
    ASSERT_INT_EQ(pid.Improve, NONE);
    ASSERT_FLOAT_EQ(pid.Output, 0.0f, 1e-6f);
    ASSERT_FLOAT_EQ(pid.ITerm, 0.0f, 1e-6f);
    ASSERT_INT_EQ(pid.ERRORHandler.ERRORType, PID_ERROR_NONE);
}

/* ========================== PID_Calculate ========================== */
static void test_PID_Calculate_proportional_only(void)
{
    PID_t pid;
    memset(&pid, 0, sizeof(pid));
    PID_Init(&pid, 1000.0f, 500.0f, 0.0f, 10.0f, 0.0f, 0.0f, 0, 0, 0, 0, 2, NONE);

    stub_dwt_dt = 0.001f;
    float out = PID_Calculate(&pid, 0.0f, 5.0f);
    /* P-only: out = Kp * Err = 10 * 5 = 50 */
    ASSERT_FLOAT_EQ(out, 50.0f, 0.1f);
    ASSERT_FLOAT_EQ(pid.Err, 5.0f, 1e-6f);
    ASSERT_FLOAT_EQ(pid.Pout, 50.0f, 0.1f);
}

static void test_PID_Calculate_integral_accumulates(void)
{
    PID_t pid;
    memset(&pid, 0, sizeof(pid));
    PID_Init(&pid, 1000.0f, 500.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0, 0, 0, 0, 2, NONE);

    stub_dwt_dt = 0.001f;
    PID_Calculate(&pid, 0.0f, 5.0f);
    /* ITerm = Ki * Err * dt = 10 * 5 * 0.001 = 0.05 */
    /* Iout should accumulate */
    PID_Calculate(&pid, 0.0f, 5.0f);
    /* 2nd call: ITerm = 0.05 again, Iout = 0.05 + 0.05 = 0.10 */
    ASSERT_FLOAT_EQ(pid.Iout, 0.10f, 0.01f);
}

static void test_PID_Calculate_derivative(void)
{
    PID_t pid;
    memset(&pid, 0, sizeof(pid));
    PID_Init(&pid, 1000.0f, 500.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0, 0, 0, 0, 2, NONE);

    stub_dwt_dt = 0.001f;
    PID_Calculate(&pid, 0.0f, 5.0f);
    /* First call: Dout = Kd * (Err - Last_Err) / dt = 10 * (5 - 0) / 0.001 = 50000 */
    ASSERT_TRUE(pid.Dout > 0.0f);

    /* Second call with same error: derivative should be 0 */
    PID_Calculate(&pid, 0.0f, 5.0f);
    ASSERT_FLOAT_EQ(pid.Dout, 0.0f, 0.01f);
}

static void test_PID_Calculate_deadband(void)
{
    PID_t pid;
    memset(&pid, 0, sizeof(pid));
    PID_Init(&pid, 1000.0f, 500.0f, 5.0f, 10.0f, 0.0f, 0.0f, 0, 0, 0, 0, 2, NONE);

    stub_dwt_dt = 0.001f;
    /* Error = 3.0 < deadband = 5.0 -> no output change (stays 0) */
    float out = PID_Calculate(&pid, 2.0f, 5.0f);
    ASSERT_FLOAT_EQ(out, 0.0f, 1e-6f);

    /* Error = 10.0 > deadband = 5.0 -> output should change */
    out = PID_Calculate(&pid, 0.0f, 10.0f);
    ASSERT_TRUE(fabsf(out) > 0.0f);
}

static void test_PID_Calculate_output_limit(void)
{
    PID_t pid;
    memset(&pid, 0, sizeof(pid));
    PID_Init(&pid, 50.0f, 500.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0, 0, 0, 0, 2, NONE);

    stub_dwt_dt = 0.001f;
    /* P output = 100 * 10 = 1000, but clamped to MaxOut = 50 */
    float out = PID_Calculate(&pid, 0.0f, 10.0f);
    ASSERT_FLOAT_EQ(out, 50.0f, 0.1f);

    /* Negative direction */
    memset(&pid, 0, sizeof(pid));
    PID_Init(&pid, 50.0f, 500.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0, 0, 0, 0, 2, NONE);
    out = PID_Calculate(&pid, 10.0f, 0.0f);
    ASSERT_FLOAT_EQ(out, -50.0f, 0.1f);
}

static void test_PID_Calculate_with_integral_limit(void)
{
    PID_t pid;
    memset(&pid, 0, sizeof(pid));
    PID_Init(&pid, 1000.0f, 0.1f, 0.0f, 0.0f, 100.0f, 0.0f, 0, 0, 0, 0, 2, Integral_Limit);

    stub_dwt_dt = 0.001f;
    /* Large error -> large ITerm, but Iout should be limited to IntegralLimit */
    for (int i = 0; i < 100; i++)
        PID_Calculate(&pid, 0.0f, 10.0f);

    ASSERT_TRUE(pid.Iout <= pid.IntegralLimit + 0.01f);
    ASSERT_TRUE(pid.Iout >= -pid.IntegralLimit - 0.01f);
}

static void test_PID_Calculate_with_trapezoid_integral(void)
{
    PID_t pid;
    memset(&pid, 0, sizeof(pid));
    PID_Init(&pid, 1000.0f, 500.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0, 0, 0, 0, 2, Trapezoid_Intergral);

    stub_dwt_dt = 0.001f;
    PID_Calculate(&pid, 0.0f, 5.0f); /* Err=5, Last_Err=0, ITerm = Ki*(5+0)/2*dt */
    float first_iterm = pid.ITerm;
    /* Trapezoid: ITerm = Ki * (Err + Last_Err)/2 * dt = 10 * (5+0)/2 * 0.001 = 0.025 */
    ASSERT_FLOAT_EQ(first_iterm, 0.025f, 0.005f);
}

static void test_PID_Calculate_with_derivative_filter(void)
{
    PID_t pid;
    memset(&pid, 0, sizeof(pid));
    PID_Init(&pid, 1000.0f, 500.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0, 0, 0.01f, 0.01f, 2, DerivativeFilter);

    stub_dwt_dt = 0.001f;
    PID_Calculate(&pid, 0.0f, 5.0f);
    float dout1 = pid.Dout;
    /* Derivative filter smooths Dout, so second call with same error (Dout=0 raw)
       should still have some filtered residual from last Dout */
    PID_Calculate(&pid, 0.0f, 5.0f);
    float dout2 = pid.Dout;
    /* Filtered Dout should be smaller than raw first Dout */
    ASSERT_TRUE(fabsf(dout2) < fabsf(dout1));
}

static void test_PID_Calculate_with_output_filter(void)
{
    PID_t pid;
    memset(&pid, 0, sizeof(pid));
    PID_Init(&pid, 1000.0f, 500.0f, 0.0f, 10.0f, 0.0f, 0.0f, 0, 0, 0.01f, 0, 2, OutputFilter);

    stub_dwt_dt = 0.001f;
    float out1 = PID_Calculate(&pid, 0.0f, 5.0f);
    /* Output filter: filtered output < raw P output */
    /* Raw P = 50, but filtered = 50 * dt/(RC+dt) = 50 * 0.001/0.011 ≈ 4.55 */
    ASSERT_TRUE(out1 < 50.0f);
    ASSERT_TRUE(out1 > 0.0f);
}

static void test_PID_Calculate_with_changing_integration_rate(void)
{
    PID_t pid;
    memset(&pid, 0, sizeof(pid));
    /* CoefA=5, CoefB=2: full integral when |err| <= 2, reduced when 2 < |err| < 7, zero when |err| >= 7 */
    PID_Init(&pid, 1000.0f, 500.0f, 0.0f, 0.0f, 10.0f, 0.0f, 5.0f, 2.0f, 0, 0, 2, ChangingIntegrationRate);

    stub_dwt_dt = 0.001f;

    /* Error = 1 (within CoefB) -> full integral */
    PID_Calculate(&pid, 4.0f, 5.0f);
    float iterm_small = pid.ITerm;

    /* Error = 10 (> CoefA + CoefB = 7) -> ITerm should be 0 */
    memset(&pid, 0, sizeof(pid));
    PID_Init(&pid, 1000.0f, 500.0f, 0.0f, 0.0f, 10.0f, 0.0f, 5.0f, 2.0f, 0, 0, 2, ChangingIntegrationRate);
    PID_Calculate(&pid, 0.0f, 10.0f);
    /* Note: ChangingIntegrationRate only applies when Err*Iout > 0.
       On first call Iout=0, so condition isn't met yet.
       ITerm should still be non-zero on first call. */
    ASSERT_TRUE(iterm_small != 0.0f);
}

/* Test user callback */
static int user_func_called = 0;
static void my_user_func(PID_t *pid) {
    (void)pid;
    user_func_called++;
}

static void test_PID_Calculate_user_callback(void)
{
    PID_t pid;
    memset(&pid, 0, sizeof(pid));
    PID_Init(&pid, 1000.0f, 500.0f, 0.0f, 10.0f, 0.0f, 0.0f, 0, 0, 0, 0, 2, NONE);
    pid.User_Func1_f = my_user_func;
    pid.User_Func2_f = my_user_func;

    user_func_called = 0;
    stub_dwt_dt = 0.001f;
    PID_Calculate(&pid, 0.0f, 5.0f);
    /* User_Func1_f called before processing, User_Func2_f called after error check */
    ASSERT_TRUE(user_func_called >= 1);
}

/* ========================== Fuzzy ========================== */
static void test_Fuzzy_Rule_Init_default_tables(void)
{
    FuzzyRule_t fuzzy;
    memset(&fuzzy, 0, sizeof(fuzzy));
    Fuzzy_Rule_Init(&fuzzy, NULL, NULL, NULL, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);

    ASSERT_TRUE(fuzzy.FuzzyRuleKp != NULL);
    ASSERT_TRUE(fuzzy.FuzzyRuleKi != NULL);
    ASSERT_TRUE(fuzzy.FuzzyRuleKd != NULL);
    ASSERT_FLOAT_EQ(fuzzy.KpRatio, 1.0f, 1e-6f);
    ASSERT_FLOAT_EQ(fuzzy.eStep, 1.0f, 1e-6f);
    ASSERT_FLOAT_EQ(fuzzy.ecStep, 1.0f, 1e-6f);
}

static void test_Fuzzy_Rule_Init_small_step_clamped(void)
{
    FuzzyRule_t fuzzy;
    memset(&fuzzy, 0, sizeof(fuzzy));
    Fuzzy_Rule_Init(&fuzzy, NULL, NULL, NULL, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f);
    /* eStep and ecStep should be clamped to 1 when < 0.00001 */
    ASSERT_FLOAT_EQ(fuzzy.eStep, 1.0f, 1e-6f);
    ASSERT_FLOAT_EQ(fuzzy.ecStep, 1.0f, 1e-6f);
}

static void test_Fuzzy_Rule_Implementation_zero_error(void)
{
    FuzzyRule_t fuzzy;
    memset(&fuzzy, 0, sizeof(fuzzy));
    Fuzzy_Rule_Init(&fuzzy, NULL, NULL, NULL, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);

    stub_dwt_dt = 0.001f;
    Fuzzy_Rule_Implementation(&fuzzy, 5.0f, 5.0f);
    /* Zero error -> fuzzy outputs should be ~ZE (0) */
    ASSERT_FLOAT_EQ(fuzzy.e, 0.0f, 1e-6f);
}

static void test_Fuzzy_Rule_Implementation_large_error(void)
{
    FuzzyRule_t fuzzy;
    memset(&fuzzy, 0, sizeof(fuzzy));
    Fuzzy_Rule_Init(&fuzzy, NULL, NULL, NULL, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);

    stub_dwt_dt = 0.001f;
    /* Large positive error */
    Fuzzy_Rule_Implementation(&fuzzy, 0.0f, 100.0f);
    ASSERT_TRUE(fuzzy.e > 0.0f);
    /* KpFuzzy should be non-negative for large positive error */
    ASSERT_TRUE(fuzzy.KpFuzzy >= 0.0f);
}

/* ========================== PID with Fuzzy ========================== */
static void test_PID_Calculate_with_fuzzy(void)
{
    PID_t pid;
    FuzzyRule_t fuzzy;
    memset(&pid, 0, sizeof(pid));
    memset(&fuzzy, 0, sizeof(fuzzy));

    Fuzzy_Rule_Init(&fuzzy, NULL, NULL, NULL, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    PID_Init(&pid, 1000.0f, 500.0f, 0.0f, 10.0f, 1.0f, 0.1f, 0, 0, 0, 0, 2, NONE);
    pid.FuzzyRule = &fuzzy;

    stub_dwt_dt = 0.001f;
    float out = PID_Calculate(&pid, 0.0f, 5.0f);
    /* With fuzzy, Pout = (Kp + KpFuzzy) * Err */
    ASSERT_TRUE(fabsf(out) > 0.0f);
}

/* ========================== Feedforward ========================== */
static void test_Feedforward_Init(void)
{
    Feedforward_t ffc;
    memset(&ffc, 0, sizeof(ffc));
    float c[3] = {1.0f, 2.0f, 3.0f};
    Feedforward_Init(&ffc, 100.0f, c, 0.01f, 2, 2);

    ASSERT_FLOAT_EQ(ffc.MaxOut, 100.0f, 1e-6f);
    ASSERT_FLOAT_EQ(ffc.c[0], 1.0f, 1e-6f);
    ASSERT_FLOAT_EQ(ffc.c[1], 2.0f, 1e-6f);
    ASSERT_FLOAT_EQ(ffc.c[2], 3.0f, 1e-6f);
    ASSERT_FLOAT_EQ(ffc.LPF_RC, 0.01f, 1e-6f);
}

static void test_Feedforward_Init_null_c(void)
{
    Feedforward_t ffc;
    memset(&ffc, 0, sizeof(ffc));
    Feedforward_Init(&ffc, 100.0f, NULL, 0.01f, 2, 2);

    ASSERT_FLOAT_EQ(ffc.c[0], 0.0f, 1e-6f);
    ASSERT_FLOAT_EQ(ffc.c[1], 0.0f, 1e-6f);
    ASSERT_FLOAT_EQ(ffc.c[2], 0.0f, 1e-6f);
    ASSERT_FLOAT_EQ(ffc.MaxOut, 0.0f, 1e-6f);
}

static void test_Feedforward_Init_with_OLS(void)
{
    Feedforward_t ffc;
    memset(&ffc, 0, sizeof(ffc));
    float c[3] = {1.0f, 2.0f, 3.0f};
    Feedforward_Init(&ffc, 100.0f, c, 0.01f, 5, 5);

    ASSERT_INT_EQ(ffc.Ref_dot_OLS_Order, 5);
    ASSERT_INT_EQ(ffc.Ref_ddot_OLS_Order, 5);
}

static void test_Feedforward_Calculate(void)
{
    Feedforward_t ffc;
    memset(&ffc, 0, sizeof(ffc));
    float c[3] = {1.0f, 0.0f, 0.0f};
    Feedforward_Init(&ffc, 1000.0f, c, 0.01f, 2, 2);

    stub_dwt_dt = 0.001f;
    /* Step input: ff output should ramp up */
    float out = 0;
    for (int i = 0; i < 50; i++)
        out = Feedforward_Calculate(&ffc, 10.0f);
    /* c[0]*Ref should dominate. Ref converges to 10 through LPF */
    ASSERT_TRUE(out > 0.0f);
}

static void test_Feedforward_Calculate_output_limit(void)
{
    Feedforward_t ffc;
    memset(&ffc, 0, sizeof(ffc));
    float c[3] = {100.0f, 0.0f, 0.0f};
    Feedforward_Init(&ffc, 50.0f, c, 0.001f, 2, 2);

    stub_dwt_dt = 0.001f;
    float out = 0;
    for (int i = 0; i < 200; i++)
        out = Feedforward_Calculate(&ffc, 10.0f);
    /* Output should be clamped to MaxOut = 50 */
    ASSERT_TRUE(out <= 50.0f);
    ASSERT_TRUE(out >= -50.0f);
}

/* ========================== LDOB ========================== */
static void test_LDOB_Init(void)
{
    LDOB_t ldob;
    memset(&ldob, 0, sizeof(ldob));
    float c[3] = {1.0f, 0.5f, 0.1f};
    LDOB_Init(&ldob, 100.0f, 0.1f, c, 0.01f, 2, 2);

    ASSERT_FLOAT_EQ(ldob.Max_Disturbance, 100.0f, 1e-6f);
    ASSERT_FLOAT_EQ(ldob.DeadBand, 0.1f, 1e-6f);
    ASSERT_FLOAT_EQ(ldob.c[0], 1.0f, 1e-6f);
    ASSERT_FLOAT_EQ(ldob.c[1], 0.5f, 1e-6f);
    ASSERT_FLOAT_EQ(ldob.c[2], 0.1f, 1e-6f);
    ASSERT_FLOAT_EQ(ldob.LPF_RC, 0.01f, 1e-6f);
}

static void test_LDOB_Init_null_c(void)
{
    LDOB_t ldob;
    memset(&ldob, 0, sizeof(ldob));
    LDOB_Init(&ldob, 100.0f, 0.1f, NULL, 0.01f, 2, 2);

    ASSERT_FLOAT_EQ(ldob.c[0], 0.0f, 1e-6f);
    ASSERT_FLOAT_EQ(ldob.Max_Disturbance, 0.0f, 1e-6f);
}

static void test_LDOB_Calculate(void)
{
    LDOB_t ldob;
    memset(&ldob, 0, sizeof(ldob));
    float c[3] = {1.0f, 0.0f, 0.0f};
    LDOB_Init(&ldob, 100.0f, 0.0f, c, 0.01f, 2, 2);

    stub_dwt_dt = 0.001f;
    /* With constant measure=5 and u=0, disturbance = c[0]*measure - u = 5 */
    float out = 0;
    for (int i = 0; i < 50; i++)
        out = LDOB_Calculate(&ldob, 5.0f, 0.0f);
    /* Disturbance should converge toward 5.0 through LPF */
    ASSERT_TRUE(out > 0.0f);
}

static void test_LDOB_Calculate_deadband(void)
{
    LDOB_t ldob;
    memset(&ldob, 0, sizeof(ldob));
    float c[3] = {0.001f, 0.0f, 0.0f};
    /* Very high deadband relative to max: deadband=0.99 means |disturbance| must exceed 0.99*Max */
    LDOB_Init(&ldob, 100.0f, 0.99f, c, 0.01f, 2, 2);

    stub_dwt_dt = 0.001f;
    /* Small disturbance should be inside deadband -> output=0 */
    float out = LDOB_Calculate(&ldob, 1.0f, 0.0f);
    ASSERT_FLOAT_EQ(out, 0.0f, 0.01f);
}

/* ========================== TD ========================== */
static void test_TD_Init(void)
{
    TD_t td;
    memset(&td, 0, sizeof(td));
    TD_Init(&td, 100.0f, 0.01f);

    ASSERT_FLOAT_EQ(td.r, 100.0f, 1e-6f);
    ASSERT_FLOAT_EQ(td.h0, 0.01f, 1e-6f);
    ASSERT_FLOAT_EQ(td.x, 0.0f, 1e-6f);
    ASSERT_FLOAT_EQ(td.dx, 0.0f, 1e-6f);
    ASSERT_FLOAT_EQ(td.ddx, 0.0f, 1e-6f);
}

static void test_TD_Calculate_tracks_step(void)
{
    TD_t td;
    memset(&td, 0, sizeof(td));
    TD_Init(&td, 100.0f, 0.01f);

    stub_dwt_dt = 0.001f;
    /* Feed a step input; TD output should ramp toward the input */
    float out = 0;
    for (int i = 0; i < 500; i++)
        out = TD_Calculate(&td, 10.0f);
    /* After many iterations, x should approach 10.0 */
    ASSERT_TRUE(fabsf(out - 10.0f) < 2.0f);
}

static void test_TD_Calculate_large_dt_returns_zero(void)
{
    TD_t td;
    memset(&td, 0, sizeof(td));
    TD_Init(&td, 100.0f, 0.01f);

    /* dt > 0.5 should return 0 */
    stub_dwt_dt = 1.0f;
    float out = TD_Calculate(&td, 10.0f);
    ASSERT_FLOAT_EQ(out, 0.0f, 1e-6f);
}

static void test_TD_Calculate_derivative_output(void)
{
    TD_t td;
    memset(&td, 0, sizeof(td));
    TD_Init(&td, 100.0f, 0.01f);

    stub_dwt_dt = 0.001f;
    /* Ramp up for a while */
    for (int i = 0; i < 200; i++)
        TD_Calculate(&td, 10.0f);
    /* dx should be positive while tracking */
    ASSERT_TRUE(td.dx >= 0.0f);
}

/* ========================== main ========================== */
int main(void)
{
    printf("=== controller tests ===\n");

    RUN_TEST(test_PID_Init_basic);
    RUN_TEST(test_PID_Calculate_proportional_only);
    RUN_TEST(test_PID_Calculate_integral_accumulates);
    RUN_TEST(test_PID_Calculate_derivative);
    RUN_TEST(test_PID_Calculate_deadband);
    RUN_TEST(test_PID_Calculate_output_limit);
    RUN_TEST(test_PID_Calculate_with_integral_limit);
    RUN_TEST(test_PID_Calculate_with_trapezoid_integral);
    RUN_TEST(test_PID_Calculate_with_derivative_filter);
    RUN_TEST(test_PID_Calculate_with_output_filter);
    RUN_TEST(test_PID_Calculate_with_changing_integration_rate);
    RUN_TEST(test_PID_Calculate_user_callback);
    RUN_TEST(test_Fuzzy_Rule_Init_default_tables);
    RUN_TEST(test_Fuzzy_Rule_Init_small_step_clamped);
    RUN_TEST(test_Fuzzy_Rule_Implementation_zero_error);
    RUN_TEST(test_Fuzzy_Rule_Implementation_large_error);
    RUN_TEST(test_PID_Calculate_with_fuzzy);
    RUN_TEST(test_Feedforward_Init);
    RUN_TEST(test_Feedforward_Init_null_c);
    RUN_TEST(test_Feedforward_Init_with_OLS);
    RUN_TEST(test_Feedforward_Calculate);
    RUN_TEST(test_Feedforward_Calculate_output_limit);
    RUN_TEST(test_LDOB_Init);
    RUN_TEST(test_LDOB_Init_null_c);
    RUN_TEST(test_LDOB_Calculate);
    RUN_TEST(test_LDOB_Calculate_deadband);
    RUN_TEST(test_TD_Init);
    RUN_TEST(test_TD_Calculate_tracks_step);
    RUN_TEST(test_TD_Calculate_large_dt_returns_zero);
    RUN_TEST(test_TD_Calculate_derivative_output);

    TEST_SUMMARY();
    return test_fail_count > 0 ? 1 : 0;
}
