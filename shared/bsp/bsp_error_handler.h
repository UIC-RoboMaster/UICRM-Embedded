#pragma once

#include "bsp_print.h"

#define BSP_DEBUG print("[DEBUG] %s:%d ", __FUNCTION__, __LINE__)
// non-fatal assertions (does not hang)
#define RM_EXPECT_TRUE(cond, msg)                                \
  do {                                                           \
    if (!(cond)) bsp_error_handler(__FUNCTION__, __LINE__, msg); \
  } while (0)
#define RM_EXPECT_FALSE(cond, msg) RM_EXPECT_TRUE(!(cond), msg)
#define RM_EXPECT_EQ(expr, ref, msg) RM_EXPECT_TRUE((expr) == (ref), msg)
#define RM_EXPECT_NE(expr, ref, msg) RM_EXPECT_TRUE((expr) != (ref), msg)
#define RM_EXPECT_GT(expr, ref, msg) RM_EXPECT_TRUE((expr) > (ref), msg)
#define RM_EXPECT_GE(expr, ref, msg) RM_EXPECT_TRUE((expr) >= (ref), msg)
#define RM_EXPECT_LT(expr, ref, msg) RM_EXPECT_TRUE((expr) < (ref), msg)
#define RM_EXPECT_LE(expr, ref, msg) RM_EXPECT_TRUE((expr) <= (ref), msg)
#define RM_EXPECT_HAL_OK(expr, msg) RM_EXPECT_TRUE((expr) == HAL_OK, msg)
// fatal assertions (hangs program)
#define RM_ASSERT_TRUE(cond, msg)                     \
  do {                                                \
    if (!(cond)) {                                    \
      bsp_error_handler(__FUNCTION__, __LINE__, msg); \
      while (1)                                       \
        ;                                             \
    }                                                 \
  } while (0)
#define RM_ASSERT_FALSE(cond, msg) RM_ASSERT_TRUE(!(cond), msg)
#define RM_ASSERT_EQ(expr, ref, msg) RM_ASSERT_TRUE((expr) == (ref), msg)
#define RM_ASSERT_NE(expr, ref, msg) RM_ASSERT_TRUE((expr) != (ref), msg)
#define RM_ASSERT_GT(expr, ref, msg) RM_ASSERT_TRUE((expr) > (ref), msg)
#define RM_ASSERT_GE(expr, ref, msg) RM_ASSERT_TRUE((expr) >= (ref), msg)
#define RM_ASSERT_LT(expr, ref, msg) RM_ASSERT_TRUE((expr) < (ref), msg)
#define RM_ASSERT_LE(expr, ref, msg) RM_ASSERT_TRUE((expr) <= (ref), msg)
#define RM_ASSERT_HAL_OK(expr, msg) RM_ASSERT_TRUE((expr) == HAL_OK, msg)

/**
 * Handle error condition printf etc.
 *
 * @param  file       Which file the error occured
 * @param  line       Which line the error occured
 * @param  msg        Message want to print
 * @author Nickel_Liang
 * @date   2018-04-15
 */
void bsp_error_handler(const char* func, int line, const char* msg);
