#include "blackbox_encoder.h"
#include "blackbox_serial_device_null.h"

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
    static BlackboxSerialDeviceNull serial_device; // NOLINT(misc-const-correctness) false positive
    static BlackboxEncoder encoder(serial_device);
    encoder.write(5);
    TEST_ASSERT_EQUAL(5, serial_device[0]);
    encoder.write(7);
    TEST_ASSERT_EQUAL(7, serial_device[1]);
}

void test_write_tag8_8Svb()
{
    static BlackboxSerialDeviceNull serial_device; // NOLINT(misc-const-correctness) false positive
    static BlackboxEncoder encoder(serial_device);

    std::array<int32_t, 8> array {};

    serial_device.resetIndex();
    serial_device.fill(0xa5);
    array[0] = 17;
    encoder.writeTag8_8SVB(&array[0], 1);
    TEST_ASSERT_EQUAL(BlackboxEncoder::zigzagEncode(17), serial_device[0]);
    TEST_ASSERT_EQUAL(0xa5, serial_device[1]);


    serial_device.resetIndex();
    serial_device.fill(0xa5);
    array[0] = 19;
    array[1] = 23;
    encoder.writeTag8_8SVB(&array[0], 2);

    TEST_ASSERT_EQUAL(0x03, serial_device[0]);
    TEST_ASSERT_EQUAL(BlackboxEncoder::zigzagEncode(19), serial_device[1]);
    TEST_ASSERT_EQUAL(BlackboxEncoder::zigzagEncode(23), serial_device[2]);
    TEST_ASSERT_EQUAL(0xa5, serial_device[3]);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-pro-bounds-pointer-arithmetic,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_blackbox_encoder_write);
    RUN_TEST(test_write_tag8_8Svb);

    UNITY_END();
}
