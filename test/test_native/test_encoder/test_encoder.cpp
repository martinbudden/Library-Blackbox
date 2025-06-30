#include "BlackboxEncoder.h"
#include "BlackboxSerialDeviceNull.h"

#include <array>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-pro-bounds-pointer-arithmetic,readability-magic-numbers)
void test_blackbox_encoder_write()
{
    static BlackboxSerialDeviceNull sd; // NOLINT(misc-const-correctness) false positive
    static BlackboxEncoder bb(sd);
    bb.write(5);
    TEST_ASSERT_EQUAL(5, sd[0]);
    bb.write(7);
    TEST_ASSERT_EQUAL(7, sd[1]);
}

void test_blackbox_encoder_printf()
{
    static BlackboxSerialDeviceNull sd; // NOLINT(misc-const-correctness) false positive
    static BlackboxEncoder bb(sd);

    sd.fill(0xa5);
    bb.printf("hello"); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
    TEST_ASSERT_EQUAL('h', sd[0]);
    TEST_ASSERT_EQUAL('e', sd[1]);
    TEST_ASSERT_EQUAL('l', sd[2]);
    TEST_ASSERT_EQUAL('l', sd[3]);
    TEST_ASSERT_EQUAL('o', sd[4]);
    TEST_ASSERT_EQUAL(0xa5, sd[5]);

    sd.resetIndex();
    sd.fill(0xa5);
    bb.printf("%s, %d", "world", 3); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
    TEST_ASSERT_EQUAL('w', sd[0]);
    TEST_ASSERT_EQUAL('o', sd[1]);
    TEST_ASSERT_EQUAL('r', sd[2]);
    TEST_ASSERT_EQUAL('l', sd[3]);
    TEST_ASSERT_EQUAL('d', sd[4]);
    TEST_ASSERT_EQUAL(',', sd[5]);
    TEST_ASSERT_EQUAL(' ', sd[6]);
    TEST_ASSERT_EQUAL('3', sd[7]);
    TEST_ASSERT_EQUAL(0xa5, sd[8]);

    sd.resetIndex();
    sd.fill(0xa5);
    bb.printf("S:%s, C:%c", "Ab", 'I'); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
    TEST_ASSERT_EQUAL('S', sd[0]);
    TEST_ASSERT_EQUAL(':', sd[1]);
    TEST_ASSERT_EQUAL('A', sd[2]);
    TEST_ASSERT_EQUAL('b', sd[3]);
    TEST_ASSERT_EQUAL(',', sd[4]);
    TEST_ASSERT_EQUAL(' ', sd[5]);
    TEST_ASSERT_EQUAL('C', sd[6]);
    TEST_ASSERT_EQUAL(':', sd[7]);
    TEST_ASSERT_EQUAL('I', sd[8]);
    TEST_ASSERT_EQUAL(0xa5, sd[9]);

}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-pro-bounds-pointer-arithmetic,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_blackbox_encoder_write);
    RUN_TEST(test_blackbox_encoder_printf);

    UNITY_END();
}
