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
    k_msleep(10);   // Empty blocks are freed by the memlease thread
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


ZTEST(memlease, test_lock_count)
{
    uint32_t handle = memlease_alloc(8, NULL, 50);

    zassert_equal(memlease_set_lock(handle, false), MEMLEASE_ERROR_NOT_LOCKED);
    zassert_not_null(memlease_get_buf(handle, NULL, true));
    zassert_equal(memlease_set_lock(handle, true), 0);
    k_msleep(100);
    // One of two locks removed, must still be protected
    zassert_equal(memlease_set_lock(handle, false), 0);
    k_msleep(10);
    zassert_not_null(memlease_get_buf(handle, NULL, false), "Freed while still locked once");
    // Last lock removed, expired lease is reclaimed
    zassert_equal(memlease_set_lock(handle, false), 0);
    k_msleep(10);
    zassert_is_null(memlease_get_buf(handle, NULL, false), "Not reclaimed after last unlock");
}

ZTEST(memlease, test_shared_consumers_keep_their_locks)
{
    uint32_t handle = memlease_alloc(8, NULL, 50);

    zassert_equal(memlease_set_release_count(handle, 1), 0);
    // Two consumers lock the buffer
    zassert_not_null(memlease_get_buf(handle, NULL, true));
    zassert_not_null(memlease_get_buf(handle, NULL, true));
    // Consumer A is done, consumer B is still busy after the lease expired
    zassert_equal(memlease_free(handle), 0);
    k_msleep(100);
    zassert_not_null(memlease_get_buf(handle, NULL, false), "Freed while consumer B holds a lock");
    zassert_equal(memlease_free(handle), 0);
    zassert_is_null(memlease_get_buf(handle, NULL, false));
}

ZTEST(memlease, test_retain)
{
    uint32_t handle = memlease_alloc(8, NULL, 0);

    zassert_equal(memlease_retain(handle), 0);
    zassert_equal(memlease_retain(handle), 0);
    zassert_equal(memlease_free(handle), 0);
    zassert_equal(memlease_free(handle), 0);
    zassert_not_null(memlease_get_buf(handle, NULL, false), "Freed before final release");
    zassert_equal(memlease_free(handle), 0);
    zassert_is_null(memlease_get_buf(handle, NULL, false), "Not freed after final release");

    handle = memlease_alloc(8, NULL, 0);
    zassert_equal(memlease_set_release_count(handle, UINT8_MAX), 0);
    zassert_equal(memlease_retain(handle), MEMLEASE_ERROR_COUNT_OVERFLOW);
    zassert_equal(memlease_set_release_count(handle, 0), 0);
    zassert_equal(memlease_free(handle), 0);
}

struct expiry_record {
    struct k_sem sem;
    uint32_t handle;
    uint32_t size;
    uint8_t first_byte;
    void *user_data;
    int calls;
    bool handle_valid_in_cb;
    uint32_t nested_handle;
};

static void expiry_cb(uint32_t handle, void *buf, uint32_t size, void *user_data)
{
    struct expiry_record *rec = user_data;

    rec->handle = handle;
    rec->size = size;
    rec->first_byte = ((uint8_t *)buf)[0];
    rec->user_data = user_data;
    rec->calls++;
    // The handle is already invalid inside the callback
    rec->handle_valid_in_cb = memlease_get_buf(handle, NULL, false) != NULL;
    // Other memlease functions can be used from the callback
    rec->nested_handle = memlease_alloc(4, NULL, 0);
    k_sem_give(&rec->sem);
}

ZTEST(memlease, test_expiry_callback)
{
    static struct expiry_record rec;
    uint8_t *buf;
    uint32_t handle;

    memset(&rec, 0, sizeof(rec));
    k_sem_init(&rec.sem, 0, 1);
    handle = memlease_alloc(16, (void **)&buf, 50);
    buf[0] = 0x5A;
    zassert_equal(memlease_set_error_on_timeout(handle, false), 0);
    zassert_equal(memlease_set_expiry_cb(handle, expiry_cb, &rec), 0);
    zassert_equal(k_sem_take(&rec.sem, K_MSEC(500)), 0, "Expiry callback not called");
    zassert_equal(rec.calls, 1);
    zassert_equal(rec.handle, handle);
    zassert_equal(rec.size, 16);
    zassert_equal(rec.first_byte, 0x5A);
    zassert_equal_ptr(rec.user_data, &rec);
    zassert_false(rec.handle_valid_in_cb, "Handle still valid inside the callback");
    zassert_not_equal(rec.nested_handle, 0, "Alloc from callback failed");
    zassert_equal(memlease_free(rec.nested_handle), 0);

    // Not called when the buffer is freed explicitly
    rec.calls = 0;
    handle = memlease_alloc(16, NULL, 50);
    zassert_equal(memlease_set_expiry_cb(handle, expiry_cb, &rec), 0);
    zassert_equal(memlease_free(handle), 0);
    k_msleep(100);
    zassert_equal(rec.calls, 0, "Callback called for explicit free");

    // Removing the callback
    handle = memlease_alloc(16, NULL, 50);
    zassert_equal(memlease_set_error_on_timeout(handle, false), 0);
    zassert_equal(memlease_set_expiry_cb(handle, expiry_cb, &rec), 0);
    zassert_equal(memlease_set_expiry_cb(handle, NULL, NULL), 0);
    k_msleep(100);
    zassert_equal(rec.calls, 0, "Removed callback was called");
    zassert_is_null(memlease_get_buf(handle, NULL, false));
}

ZTEST(memlease, test_realloc)
{
    size_t baseline = heap_used();
    struct memlease_stats before, after;
    uint8_t *buf;
    uint8_t *new_buf;
    uint32_t size;
    uint32_t handle = memlease_alloc(8, (void **)&buf, 0);

    memlease_get_stats(&before);
    for (int i = 0; i < 8; i++) {
        buf[i] = i;
    }
    zassert_equal(memlease_realloc(handle, 256, (void **)&new_buf), 0);
    zassert_equal_ptr(memlease_get_buf(handle, &size, false), new_buf);
    zassert_equal(size, 256);
    for (int i = 0; i < 8; i++) {
        zassert_equal(new_buf[i], i, "Content lost");
    }
    memlease_get_stats(&after);
    zassert_equal(after.bytes_allocated, before.bytes_allocated + 248);

    // Failure keeps the original buffer
    zassert_equal(memlease_realloc(handle, 0, NULL), MEMLEASE_ERROR_INVALID_ARG);
    zassert_equal(memlease_realloc(handle, CONFIG_HEAP_MEM_POOL_SIZE * 2, NULL), MEMLEASE_ERROR_NO_MEMORY);
    zassert_equal_ptr(memlease_get_buf(handle, &size, false), new_buf);
    zassert_equal(size, 256);

    zassert_equal(memlease_realloc(0, 16, NULL), MEMLEASE_ERROR_ENTRY_OUT_OF_BOUNDS);
    zassert_equal(memlease_free(handle), 0);
    zassert_equal(heap_used(), baseline, "Heap leaked");

    // The lease still expires after a realloc
    handle = memlease_alloc(8, NULL, 50);
    zassert_equal(memlease_set_error_on_timeout(handle, false), 0);
    zassert_equal(memlease_realloc(handle, 64, NULL), 0);
    k_msleep(100);
    zassert_is_null(memlease_get_buf(handle, NULL, false), "Realloc'd lease did not expire");
}

ZTEST(memlease, test_stats)
{
    struct memlease_stats before, during, after;
    uint32_t h1, h2;

    memlease_get_stats(&before);
    zassert_equal(before.num_allocated, 0);
    zassert_equal(before.bytes_allocated, 0);
    zassert_equal(before.num_blocks, 1, "Dynamic blocks left over");
    h1 = memlease_alloc(100, NULL, 0);
    h2 = memlease_alloc(20, NULL, 30);
    memlease_set_error_on_timeout(h2, false);
    zassert_equal(memlease_alloc(CONFIG_HEAP_MEM_POOL_SIZE * 2, NULL, 0), 0);
    memlease_get_stats(&during);
    zassert_equal(during.num_allocated, 2);
    zassert_equal(during.bytes_allocated, 120);
    zassert_true(during.peak_allocated >= 2);
    zassert_true(during.peak_bytes >= 120);
    zassert_equal(during.total_allocs, before.total_allocs + 2);
    zassert_equal(during.alloc_failures, before.alloc_failures + 1);
    k_msleep(80);
    zassert_equal(memlease_free(h1), 0);
    memlease_get_stats(&after);
    zassert_equal(after.num_allocated, 0);
    zassert_equal(after.bytes_allocated, 0);
    zassert_equal(after.total_timeouts, before.total_timeouts + 1);
}

struct foreach_ctx {
    uint32_t handles[3];
    int found[3];
    int visited;
    int stop_after;
    bool info_ok;
};

static bool foreach_cb(const struct memlease_info *info, void *user_data)
{
    struct foreach_ctx *ctx = user_data;

    ctx->visited++;
    for (int i = 0; i < 3; i++) {
        if (info->handle != ctx->handles[i]) {
            continue;
        }
        ctx->found[i]++;
        if ((info->size != 10 + i) || (info->lock_count != (i == 1 ? 1 : 0)) ||
            ((info->expiry_time != 0) != (i == 2))) {
            ctx->info_ok = false;
        }
        if (IS_ENABLED(CONFIG_MEMLEASE_DEBUG_INFO)) {
            if (!info->file || !strstr(info->file, "main.c") || (info->line == 0) || (info->alloc_time == 0)) {
                ctx->info_ok = false;
            }
        } else if (info->file) {
            ctx->info_ok = false;
        }
    }
    return ctx->visited < ctx->stop_after;
}

static void foreach_isr_fn(const void *param)
{
    *(int32_t *)param = memlease_foreach(foreach_cb, NULL);
}

ZTEST(memlease, test_foreach_and_dump)
{
    struct foreach_ctx ctx = { .stop_after = 100, .info_ok = true };
    int32_t isr_ret = 0;

    k_msleep(1);    // Make sure alloc_time is not 0
    for (int i = 0; i < 3; i++) {
        ctx.handles[i] = memlease_alloc(10 + i, NULL, i == 2 ? 10000 : 0);
    }
    memlease_get_buf(ctx.handles[1], NULL, true);
    zassert_equal(memlease_foreach(foreach_cb, &ctx), 3);
    zassert_true(ctx.info_ok, "Lease info is wrong");
    for (int i = 0; i < 3; i++) {
        zassert_equal(ctx.found[i], 1, "Lease %d not visited once", i);
    }
    // Stop early
    ctx.visited = 0;
    ctx.stop_after = 1;
    zassert_equal(memlease_foreach(foreach_cb, &ctx), 1);

    memlease_dump();

    irq_offload(foreach_isr_fn, &isr_ret);
    zassert_equal(isr_ret, MEMLEASE_ERROR_CANNOT_CALL_FROM_ISR);

    for (int i = 0; i < 3; i++) {
        zassert_equal(memlease_free(ctx.handles[i]), 0);
    }
}

struct isr_ctx {
    uint32_t handle;
    uint8_t *buf;
    int32_t ret[6];
    void *got_buf;
    bool in_isr;
};

static void isr_alloc_fn(const void *param)
{
    struct isr_ctx *ctx = (struct isr_ctx *)param;

    ctx->in_isr = k_is_in_isr();
    ctx->handle = memlease_alloc(32, (void **)&ctx->buf, 0);
    if (ctx->handle) {
        memset(ctx->buf, 0xC3, 32);
        ctx->ret[0] = memlease_set_release_count(ctx->handle, 1);
        ctx->ret[1] = memlease_retain(ctx->handle);
        ctx->ret[2] = memlease_set_lock(ctx->handle, true);
        ctx->ret[3] = memlease_set_lock(ctx->handle, false);
        ctx->ret[4] = memlease_set_timeout(ctx->handle, 0);
        ctx->ret[5] = memlease_free(ctx->handle);
        ctx->got_buf = memlease_get_buf(ctx->handle, NULL, false);
    }
}

static void isr_free_fn(const void *param)
{
    struct isr_ctx *ctx = (struct isr_ctx *)param;

    ctx->ret[0] = memlease_free(ctx->handle);
}

static void isr_realloc_fn(const void *param)
{
    struct isr_ctx *ctx = (struct isr_ctx *)param;

    ctx->ret[0] = memlease_realloc(ctx->handle, 128, (void **)&ctx->buf);
}

ZTEST(memlease, test_isr_usage)
{
    size_t baseline = heap_used();
    struct isr_ctx ctx = { 0 };
    uint32_t size;

    // Allocate and use a buffer from an ISR, finish with it in a thread
    irq_offload(isr_alloc_fn, &ctx);
    zassert_true(ctx.in_isr, "irq_offload did not run in ISR context");
    zassert_not_equal(ctx.handle, 0, "Alloc from ISR failed");
    for (int i = 0; i < ARRAY_SIZE(ctx.ret); i++) {
        zassert_equal(ctx.ret[i], 0, "ISR call %d failed", i);
    }
    zassert_equal_ptr(ctx.got_buf, ctx.buf);
    zassert_equal_ptr(memlease_get_buf(ctx.handle, &size, false), ctx.buf);
    zassert_equal(size, 32);
    zassert_equal(ctx.buf[31], 0xC3);
    // Release count 1 + 1 retain - 1 free in ISR leaves 2 releases
    zassert_equal(memlease_free(ctx.handle), 0);
    zassert_not_null(memlease_get_buf(ctx.handle, NULL, false), "Freed too early");

    // Resize and free it from an ISR
    irq_offload(isr_realloc_fn, &ctx);
    zassert_equal(ctx.ret[0], 0, "Realloc from ISR failed");
    zassert_equal(ctx.buf[31], 0xC3, "Content lost in ISR realloc");
    irq_offload(isr_free_fn, &ctx);
    zassert_equal(ctx.ret[0], 0, "Free from ISR failed");
    zassert_is_null(memlease_get_buf(ctx.handle, NULL, false));
    zassert_equal(heap_used(), baseline, "Heap leaked");
}

ZTEST(memlease, test_isr_alloc_new_block)
{
    size_t baseline = heap_used();
    uint32_t handles[N];
    struct isr_ctx ctx = { 0 };

    // Fill the static block so the ISR has to allocate a new entry block
    for (int i = 0; i < N; i++) {
        handles[i] = memlease_alloc(4, NULL, 0);
        zassert_not_equal(handles[i], 0);
    }
    irq_offload(isr_alloc_fn, &ctx);
    zassert_not_equal(ctx.handle, 0, "Alloc of new block from ISR failed");
    zassert_equal(ctx.handle & 0xFFFF, N + 1, "Expected first entry of the second block");
    // The ISR left two releases outstanding
    zassert_equal(memlease_free(ctx.handle), 0);
    zassert_equal(memlease_free(ctx.handle), 0);
    for (int i = 0; i < N; i++) {
        zassert_equal(memlease_free(handles[i]), 0);
    }
    k_msleep(10);   // Empty blocks are freed by the memlease thread
    zassert_equal(heap_used(), baseline, "Heap leaked (entry block not freed?)");
}

static void isr_timeout_fn(const void *param)
{
    struct isr_ctx *ctx = (struct isr_ctx *)param;

    ctx->handle = memlease_alloc(8, NULL, 50);
    ctx->ret[0] = memlease_set_error_on_timeout(ctx->handle, false);
}

ZTEST(memlease, test_isr_alloc_with_timeout)
{
    struct isr_ctx ctx = { 0 };

    irq_offload(isr_timeout_fn, &ctx);
    zassert_not_equal(ctx.handle, 0);
    zassert_equal(ctx.ret[0], 0);
    k_msleep(100);
    zassert_is_null(memlease_get_buf(ctx.handle, NULL, false), "Lease from ISR did not expire");
}

ZTEST_SUITE(memlease, NULL, NULL, NULL, after_each, NULL);
