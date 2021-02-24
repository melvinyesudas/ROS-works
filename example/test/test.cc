

#include "gtest/gtest.h"
#include <cstdint>
#include <unistd.h>

TEST(test_example, intro)
{
  // Test example

  // Non-fatal assertions, these expect a certain result, but will continue to
  // execute statements even if they fail to evaluate.

  EXPECT_TRUE(true);   // condition is true
  EXPECT_FALSE(false); // condition is false
  EXPECT_EQ(0, 0);     // val1 == val2
  EXPECT_NE(0, 1);     // val1 != val2
  EXPECT_LT(0, 1);     // val1 < val2
  EXPECT_LE(0, 1);     // val1 <= val2
  EXPECT_GT(1, 0);     // val1 > val2
  EXPECT_GE(1, 0);     // val1 >= val2

  // Fatal assertions, these expect a certain result. On failure these will
  // cause the test to terminate instead of continuing. Use these when future
  // tests require associated values

  ASSERT_TRUE(true);   // condition is true
  ASSERT_FALSE(false); // condition is false
  ASSERT_EQ(0, 0);     // val1 == val2
  ASSERT_NE(0, 1);     // val1 != val2
  ASSERT_LT(0, 1);     // val1 < val2
  ASSERT_LE(0, 1);     // val1 <= val2
  ASSERT_GT(1, 0);     // val1 > val2
  ASSERT_GE(1, 0);     // val1 >= val2

  // For more details on test assertions, see the following link
  // https://github.com/google/googletest/blob/master/googletest/docs/primer.md
}

// Complete Test Example Below

/** This function returns the src with the given bit set.
 * @param src the input value to set the bit of
 */
uint32_t bitset(uint32_t src, uint8_t index)
{
  return src | (1 << index);
}

// This tests ensures that setting all single bits function
TEST(test_bitset, single)
{
  ASSERT_EQ(bitset(0, 0), 0x1u);
  ASSERT_EQ(bitset(0, 1), 0x2u);
  ASSERT_EQ(bitset(0, 2), 0x4u);
  ASSERT_EQ(bitset(0, 3), 0x8u);

  ASSERT_EQ(bitset(0, 4), 0x10u);
  ASSERT_EQ(bitset(0, 5), 0x20u);
  ASSERT_EQ(bitset(0, 6), 0x40u);
  ASSERT_EQ(bitset(0, 7), 0x80u);

  ASSERT_EQ(bitset(0, 8), 0x100u);
  ASSERT_EQ(bitset(0, 9), 0x200u);
  ASSERT_EQ(bitset(0, 10), 0x400u);
  ASSERT_EQ(bitset(0, 11), 0x800u);

  ASSERT_EQ(bitset(0, 12), 0x1000u);
  ASSERT_EQ(bitset(0, 13), 0x2000u);
  ASSERT_EQ(bitset(0, 14), 0x4000u);
  ASSERT_EQ(bitset(0, 15), 0x8000u);

  ASSERT_EQ(bitset(0, 16), 0x10000u);
  ASSERT_EQ(bitset(0, 17), 0x20000u);
  ASSERT_EQ(bitset(0, 18), 0x40000u);
  ASSERT_EQ(bitset(0, 19), 0x80000u);

  ASSERT_EQ(bitset(0, 20), 0x100000u);
  ASSERT_EQ(bitset(0, 21), 0x200000u);
  ASSERT_EQ(bitset(0, 22), 0x400000u);
  ASSERT_EQ(bitset(0, 23), 0x800000u);

  ASSERT_EQ(bitset(0, 24), 0x1000000u);
  ASSERT_EQ(bitset(0, 25), 0x2000000u);
  ASSERT_EQ(bitset(0, 26), 0x4000000u);
  ASSERT_EQ(bitset(0, 27), 0x8000000u);

  ASSERT_EQ(bitset(0, 28), 0x10000000u);
  ASSERT_EQ(bitset(0, 29), 0x20000000u);
  ASSERT_EQ(bitset(0, 30), 0x40000000u);
  ASSERT_EQ(bitset(0, 31), 0x80000000u);
}
