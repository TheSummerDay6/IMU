/**
 * @file test_stubs.h
 * @brief Test assertion macros and minimal setup for host-side unit testing.
 *        Hardware stubs (DWT, arm_math, main.h) are in separate headers
 *        in the tests/ directory and are resolved via -I include path.
 */
#ifndef TEST_STUBS_H
#define TEST_STUBS_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

/* Simple test assertion macros */
static int test_pass_count = 0;
static int test_fail_count = 0;

#define ASSERT_TRUE(cond) do { \
    if (cond) { test_pass_count++; } \
    else { test_fail_count++; printf("  FAIL: %s:%d: %s\n", __FILE__, __LINE__, #cond); } \
} while(0)

#define ASSERT_FLOAT_EQ(a, b, eps) do { \
    float _a = (a), _b = (b); \
    if (fabsf(_a - _b) <= (eps)) { test_pass_count++; } \
    else { test_fail_count++; printf("  FAIL: %s:%d: %s == %.6f, expected %.6f (eps=%.6f)\n", __FILE__, __LINE__, #a, _a, _b, (float)(eps)); } \
} while(0)

#define ASSERT_INT_EQ(a, b) do { \
    int _a = (a), _b = (b); \
    if (_a == _b) { test_pass_count++; } \
    else { test_fail_count++; printf("  FAIL: %s:%d: %s == %d, expected %d\n", __FILE__, __LINE__, #a, _a, _b); } \
} while(0)

#define RUN_TEST(fn) do { \
    printf("  Running %s...\n", #fn); \
    fn(); \
} while(0)

#define TEST_SUMMARY() do { \
    printf("\n========================================\n"); \
    printf("Results: %d passed, %d failed\n", test_pass_count, test_fail_count); \
    printf("========================================\n"); \
} while(0)

#endif /* TEST_STUBS_H */
