#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_heap.h>
#include <zephyr/irq_offload.h>
#include <string.h>
#include "memlease.h"

#define N CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF

// System heap used by k_malloc(), used to check that memlease does not leak
extern struct k_heap _system_heap;

static size_t heap_used(void)
{
    struct sys_memory_stats stats;

    sys_heap_runtime_stats_get(&_system_heap.heap, &stats);
    return stats.allocated_bytes;
}

static void after_each(void *fixture)
{
    ARG_UNUSED(fixture);
    zassert_equal(memlease_get_num_allocated(), 0, "Test left allocated buffers behind");
}

ZTEST(memlease, test_alloc_and_free)
{
    size_t baseline = heap_used();
    uint8_t *buf;
    uint32_t size = 0;
    uint32_t handle = memlease_alloc(10, (void **)&buf, 1000);

    zassert_not_equal(handle, 0, "Handle must not be zero");
    zassert_not_null(buf, "Buffer pointer should not be NULL");
    zassert_equal(memlease_get_num_allocated(), 1);
    memset(buf, 0xAA, 10);
    zassert_equal_ptr(memlease_get_buf(handle, &size, false), buf);
    zassert_equal(size, 10);
    zassert_equal(memlease_free(handle), 0);
    zassert_is_null(memlease_get_buf(handle, NULL, false), "Freed buffer must not be returned");
    zassert_equal(memlease_free(handle), MEMLEASE_ERROR_NOT_ALLOCATED);
    zassert_equal(heap_used(), baseline, "Heap leaked");
}

ZTEST(memlease, test_alloc_without_buf_out)
{
    uint32_t handle = memlease_alloc(16, NULL, 0);

    zassert_not_equal(handle, 0);
    zassert_not_null(memlease_get_buf(handle, NULL, false));
    zassert_equal(memlease_free(handle), 0);
}

ZTEST(memlease, test_alloc_failure)
{
    void *buf = (void *)1;

    zassert_equal(memlease_alloc(CONFIG_HEAP_MEM_POOL_SIZE * 2, &buf, 0), 0, "Oversized alloc must fail");
    zassert_is_null(buf, "buf_out must be NULL on failure");
}

ZTEST(memlease, test_invalid_handles)
{
    uint32_t handle;
    uint32_t handle2;

    zassert_equal(memlease_free(0), MEMLEASE_ERROR_ENTRY_OUT_OF_BOUNDS);
    zassert_equal(memlease_free(0xFFFF), MEMLEASE_ERROR_ENTRY_OUT_OF_BOUNDS);
    zassert_is_null(memlease_get_buf(0, NULL, false));

    // A stale handle must not access a new allocation that reuses the same entry
    handle = memlease_alloc(8, NULL, 0);
    zassert_equal(memlease_free(handle), 0);
    handle2 = memlease_alloc(8, NULL, 0);
    zassert_equal(handle & 0xFFFF, handle2 & 0xFFFF, "Expected the entry to be reused");
    zassert_not_equal(handle, handle2, "Handles must differ");
    zassert_equal(memlease_free(handle), MEMLEASE_ERROR_ALLOCATION_NUM_MISMATCH);
    zassert_is_null(memlease_get_buf(handle, NULL, false));
    zassert_not_null(memlease_get_buf(handle2, NULL, false));
    zassert_equal(memlease_set_timeout(handle, 10), MEMLEASE_ERROR_ALLOCATION_NUM_MISMATCH);
    zassert_equal(memlease_set_lock(handle, true), MEMLEASE_ERROR_ALLOCATION_NUM_MISMATCH);
    zassert_equal(memlease_set_release_count(handle, 1), MEMLEASE_ERROR_ALLOCATION_NUM_MISMATCH);
    zassert_equal(memlease_set_error_on_timeout(handle, false), MEMLEASE_ERROR_ALLOCATION_NUM_MISMATCH);
    zassert_equal(memlease_free(handle2), 0);
}

ZTEST(memlease, test_release_count)
{
    uint32_t handle = memlease_alloc(32, NULL, 0);

    zassert_equal(memlease_set_release_count(handle, 2), 0);
    zassert_equal(memlease_free(handle), 0);
    zassert_not_null(memlease_get_buf(handle, NULL, false), "Freed after 1 of 3 releases");
    zassert_equal(memlease_free(handle), 0);
    zassert_not_null(memlease_get_buf(handle, NULL, false), "Freed after 2 of 3 releases");
    zassert_equal(memlease_free(handle), 0);
    zassert_is_null(memlease_get_buf(handle, NULL, false), "Not freed after 3 releases");
}

ZTEST(memlease, test_timeout_frees_buffer)
{
    size_t baseline = heap_used();
    uint32_t handle = memlease_alloc(64, NULL, 100);

    zassert_equal(memlease_set_error_on_timeout(handle, false), 0);
    k_msleep(50);
    zassert_not_null(memlease_get_buf(handle, NULL, false), "Freed before timeout");
    k_msleep(100);
    zassert_is_null(memlease_get_buf(handle, NULL, false), "Not freed after timeout");
    zassert_equal(memlease_free(handle), MEMLEASE_ERROR_NOT_ALLOCATED);
    zassert_equal(heap_used(), baseline, "Heap leaked");
}

ZTEST(memlease, test_infinite_timeout)
{
    uint32_t handle = memlease_alloc(64, NULL, 0);

    k_msleep(200);
    zassert_not_null(memlease_get_buf(handle, NULL, false));
    zassert_equal(memlease_free(handle), 0);
}

ZTEST(memlease, test_earlier_timeout_wakes_thread)
{
    uint32_t late = memlease_alloc(8, NULL, 1000);
    uint32_t early = memlease_alloc(8, NULL, 100);

    k_msleep(200);
    zassert_is_null(memlease_get_buf(early, NULL, false), "Earlier timeout not handled");
    zassert_not_null(memlease_get_buf(late, NULL, false), "Later buffer freed too early");
    zassert_equal(memlease_free(late), 0);
}

ZTEST(memlease, test_renew_timeout)
{
    uint32_t handle = memlease_alloc(8, NULL, 100);

    k_msleep(60);
    zassert_equal(memlease_set_timeout(handle, 100), 0);
    k_msleep(60);
    zassert_not_null(memlease_get_buf(handle, NULL, false), "Renewed buffer freed");
    k_msleep(100);
    zassert_is_null(memlease_get_buf(handle, NULL, false), "Renewed buffer not freed");

    // Shortening a timeout must also be honoured
    handle = memlease_alloc(8, NULL, 0);
    zassert_equal(memlease_set_timeout(handle, 50), 0);
    k_msleep(100);
    zassert_is_null(memlease_get_buf(handle, NULL, false), "Shortened timeout not handled");

    // Making a timeout infinite
    handle = memlease_alloc(8, NULL, 50);
    zassert_equal(memlease_set_timeout(handle, 0), 0);
    k_msleep(100);
    zassert_not_null(memlease_get_buf(handle, NULL, false), "Infinite buffer freed");
    zassert_equal(memlease_free(handle), 0);
}

ZTEST(memlease, test_lock_prevents_timeout)
{
    uint32_t handle = memlease_alloc(8, NULL, 50);

    zassert_not_null(memlease_get_buf(handle, NULL, true));
    k_msleep(100);
    zassert_not_null(memlease_get_buf(handle, NULL, false), "Locked buffer freed");
    // get_buf with lock = false must not unlock
    k_msleep(10);
    zassert_not_null(memlease_get_buf(handle, NULL, false), "get_buf(false) unlocked the buffer");
    // Unlocking an expired buffer frees it
    zassert_equal(memlease_set_lock(handle, false), 0);
    k_msleep(10);
    zassert_is_null(memlease_get_buf(handle, NULL, false), "Expired buffer not freed after unlock");

    // Lock with set_lock and release with memlease_free
    handle = memlease_alloc(8, NULL, 50);
    zassert_equal(memlease_set_lock(handle, true), 0);
    k_msleep(100);
    zassert_not_null(memlease_get_buf(handle, NULL, false), "Locked buffer freed");
    zassert_equal(memlease_free(handle), 0);
}

ZTEST(memlease, test_release_unlocks)
{
    uint32_t handle = memlease_alloc(8, NULL, 50);

    zassert_equal(memlease_set_release_count(handle, 1), 0);
    zassert_not_null(memlease_get_buf(handle, NULL, true));
    k_msleep(100);
    zassert_not_null(memlease_get_buf(handle, NULL, false), "Locked buffer freed");
    // First release unlocks; the lease already expired, so the buffer is reclaimed
    zassert_equal(memlease_free(handle), 0);
    k_msleep(10);
    zassert_is_null(memlease_get_buf(handle, NULL, false), "Expired buffer not reclaimed after release");
}

ZTEST(memlease, test_multiple_blocks)
{
    size_t baseline = heap_used();
    uint32_t handles[3 * N + 1];
    uint8_t *bufs[ARRAY_SIZE(handles)];

    for (int i = 0; i < ARRAY_SIZE(handles); i++) {
        handles[i] = memlease_alloc(4, (void **)&bufs[i], 0);
        zassert_not_equal(handles[i], 0, "Alloc %d failed", i);
        memset(bufs[i], i, 4);
        for (int j = 0; j < i; j++) {
            zassert_not_equal(handles[i], handles[j], "Duplicate handle");
        }
    }
    zassert_equal(memlease_get_num_allocated(), ARRAY_SIZE(handles));
    for (int i = 0; i < ARRAY_SIZE(handles); i++) {
        uint8_t *buf = memlease_get_buf(handles[i], NULL, false);

        zassert_equal_ptr(buf, bufs[i], "Wrong buffer for handle %d", i);
        zassert_equal(buf[3], (uint8_t)i, "Buffer %d corrupted", i);
    }

    // Empty a block in the middle, the handles after it must stay valid
    for (int i = N; i < 2 * N; i++) {
        zassert_equal(memlease_free(handles[i]), 0);
    }
    for (int i = 2 * N; i < ARRAY_SIZE(handles); i++) {
        zassert_equal_ptr(memlease_get_buf(handles[i], NULL, false), bufs[i], "Handle %d broken", i);
    }
    // Freed middle entries get reused
    handles[N] = memlease_alloc(4, NULL, 0);
    zassert_equal(handles[N] & 0xFFFF, N + 1, "Expected first free entry to be reused");
    zassert_equal(memlease_free(handles[N]), 0);

    for (int i = 0; i < N; i++) {
        zassert_equal(memlease_free(handles[i]), 0);
    }
    for (int i = 2 * N; i < ARRAY_SIZE(handles); i++) {
        zassert_equal(memlease_free(handles[i]), 0);
    }
    zassert_equal(heap_used(), baseline, "Heap leaked (entry blocks not freed?)");
}

ZTEST(memlease, test_timeout_across_blocks)
{
    size_t baseline = heap_used();
    uint32_t handles[2 * N + 5];

    for (int i = 0; i < ARRAY_SIZE(handles); i++) {
        handles[i] = memlease_alloc(4, NULL, 50 + i);
        zassert_not_equal(handles[i], 0);
        memlease_set_error_on_timeout(handles[i], false);
    }
    k_msleep(50 + ARRAY_SIZE(handles) + 50);
    for (int i = 0; i < ARRAY_SIZE(handles); i++) {
        zassert_is_null(memlease_get_buf(handles[i], NULL, false), "Handle %d not timed out", i);
    }
    zassert_equal(heap_used(), baseline, "Heap leaked (entry blocks not freed?)");
}

static void isr_fn(const void *param)
{
    int32_t *ret = (int32_t *)param;

    ret[0] = memlease_alloc(8, NULL, 0);
    ret[1] = memlease_free(1);
    ret[2] = (memlease_get_buf(1, NULL, false) == NULL) ? 0 : 1;
}

ZTEST(memlease, test_isr_rejected)
{
    int32_t ret[3] = { -1, 0, -1 };

    irq_offload(isr_fn, ret);
    zassert_equal(ret[0], 0, "alloc must fail in ISR");
    zassert_equal(ret[1], MEMLEASE_ERROR_CANNOT_CALL_FROM_ISR);
    zassert_equal(ret[2], 0, "get_buf must fail in ISR");
}

ZTEST_SUITE(memlease, NULL, NULL, NULL, after_each, NULL);
