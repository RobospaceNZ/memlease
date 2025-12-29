#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include "memlease.h"

ZTEST(memlease, test_memlease_alloc)
{
    uint8_t *buf;
    uint32_t handle = memlease_alloc(10, &buf, 1000);
    zassert_not_equal(handle, 0, "Handle must not be zero");
    zassert_not_null(buf, "Buffer pointer should not be NULL");
}

ZTEST_SUITE(memlease, NULL, NULL, NULL, NULL, NULL);
