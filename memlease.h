/*
 * memlease.h
 *
 * Copyright (c) 2025 Hubert Jetschko
 *
 * This file is licensed under the MIT License.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef MEMLEASE_H
#define MEMLEASE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Error codes returned by the memlease functions (always negative)
#define MEMLEASE_ERROR_ENTRY_OUT_OF_BOUNDS      -1  // Handle index does not point to an existing entry
#define MEMLEASE_ERROR_ALLOCATION_NUM_MISMATCH  -2  // Handle is stale (entry was freed and reused)
#define MEMLEASE_ERROR_CORRUPT_BLOCK_POINTER    -3  // Internal block list is inconsistent
#define MEMLEASE_ERROR_NOT_ALLOCATED            -4  // Handle points to an entry that is not allocated (already freed or timed out)
#define MEMLEASE_ERROR_CANNOT_CALL_FROM_ISR     -5  // Function was called from an ISR but is thread-only
#define MEMLEASE_ERROR_COUNT_OVERFLOW           -6  // Release count or lock count would exceed 255
#define MEMLEASE_ERROR_NOT_LOCKED               -7  // Unlock requested but the buffer is not locked
#define MEMLEASE_ERROR_NO_MEMORY                -8  // Heap allocation failed
#define MEMLEASE_ERROR_INVALID_ARG              -9  // Invalid argument

// Called from the memlease thread when a lease expires, just before the buffer is freed.
// The handle is already invalid when this is called, and buf must not be used after returning.
// Other memlease functions may be called from the callback.
typedef void (*memlease_expiry_cb_t)(uint32_t handle, void *buf, uint32_t size, void *user_data);

// Snapshot of one active lease, see memlease_foreach()
struct memlease_info {
    uint32_t handle;
    void *buf;
    uint32_t size;
    int64_t expiry_time;        // Uptime in ms when the lease expires, 0 means infinite
    uint8_t release_count;      // Releases left before the buffer is freed, minus one
    uint8_t lock_count;
    bool error_on_timeout;
    memlease_expiry_cb_t expiry_cb;
    // Only filled in with CONFIG_MEMLEASE_DEBUG_INFO, otherwise NULL / 0
    const char *file;
    uint32_t line;
    int64_t alloc_time;         // Uptime in ms when the buffer was allocated
};

struct memlease_stats {
    uint32_t num_allocated;     // Buffers currently allocated
    uint32_t peak_allocated;    // Highest number of buffers allocated at the same time
    uint32_t bytes_allocated;   // Bytes currently allocated in buffers (excluding overhead)
    uint32_t peak_bytes;        // Highest number of bytes allocated at the same time
    uint32_t total_allocs;      // Successful allocations since boot
    uint32_t total_timeouts;    // Buffers freed because their lease expired
    uint32_t alloc_failures;    // Failed allocations
    uint32_t num_blocks;        // Entry blocks in use, including the static one
};

// Allocate a leased memory buffer.
// size     - Size of the buffer in bytes
// buf_out  - If not NULL, the pointer to the allocated buffer is written to this pointer (NULL on failure)
// timeout  - Time in milliseconds the buffer is leased for, 0 (or negative) means infinite
// Returns a handle to the leased memory buffer, 0 on failure. The handle is not a pointer.
// With CONFIG_MEMLEASE_DEBUG_INFO the caller's file and line are recorded.
#define memlease_alloc(size, buf_out, timeout) \
    memlease_alloc_at((size), (buf_out), (timeout), MEMLEASE_CALLER_FILE, __LINE__)

#ifdef CONFIG_MEMLEASE_DEBUG_INFO
#define MEMLEASE_CALLER_FILE __FILE__
#else
#define MEMLEASE_CALLER_FILE NULL
#endif

// Same as memlease_alloc(), with an explicit caller location for the debug info (file may be NULL).
// Useful for wrappers that want to report their own caller.
uint32_t memlease_alloc_at(uint32_t size, void **buf_out, int64_t timeout, const char *file, uint32_t line);

// Return a pointer to the buffer, NULL if the handle is invalid (freed, timed out or stale).
// out_size - If not NULL, the size of the buffer is written to this pointer
// lock     - If true, the lock count is incremented so the buffer is not freed on timeout while in use.
//            Every lock must be matched by memlease_set_lock(handle, false) or memlease_free().
//            If false, the lock count is left unchanged.
void *memlease_get_buf(uint32_t handle, uint32_t *out_size, bool lock);

// Release the leased buffer. If the release count is greater than zero it is decremented,
// otherwise the buffer is freed. Each release also removes one lock, if the buffer is locked.
// Returns 0 on success, negative MEMLEASE_ERROR_* on failure.
int32_t memlease_free(uint32_t handle);

// Add (true) or remove (false) a lock. A buffer with a lock count above zero is never freed on timeout.
// If the lease expired while locked, the buffer is freed shortly after the last lock is removed.
int32_t memlease_set_lock(uint32_t handle, bool lock);

// Renew the lease: the buffer now expires timeout milliseconds from now. 0 (or negative) means infinite.
int32_t memlease_set_timeout(uint32_t handle, int64_t timeout);

// If set (default), an error message is logged when the buffer times out.
int32_t memlease_set_error_on_timeout(uint32_t handle, bool error_on_timeout);

// Set the release count for the buffer. This is one less than the number of times the buffer
// needs to be released before it is freed. For example, if the buffer is sent to 3 consumers
// that each call memlease_free(), set release_count to 2.
int32_t memlease_set_release_count(uint32_t handle, uint8_t release_count);

// Add one more required release, for example when handing the buffer to another consumer.
// Like incrementing a reference count, each memlease_retain() needs a matching memlease_free().
int32_t memlease_retain(uint32_t handle);

// Call cb from the memlease thread when the lease expires (not when the buffer is freed with memlease_free).
// Pass NULL to remove the callback.
int32_t memlease_set_expiry_cb(uint32_t handle, memlease_expiry_cb_t cb, void *user_data);

// Resize the buffer, keeping its content and handle. The buffer may move, so pointers obtained
// before this call become invalid. Only call this while no other code is using the buffer.
// buf_out - If not NULL, the new buffer pointer is written to this pointer
// On failure the original buffer is left untouched.
int32_t memlease_realloc(uint32_t handle, uint32_t new_size, void **buf_out);

// Return the number of currently allocated (leased) buffers.
uint32_t memlease_get_num_allocated(void);

// Copy the current statistics into stats.
void memlease_get_stats(struct memlease_stats *stats);

// Call cb for every active lease. Stops early if cb returns false.
// The info is a snapshot; the lease may change or expire while cb runs. Thread context only.
// Returns the number of leases visited, or MEMLEASE_ERROR_CANNOT_CALL_FROM_ISR.
typedef bool (*memlease_foreach_cb_t)(const struct memlease_info *info, void *user_data);
int32_t memlease_foreach(memlease_foreach_cb_t cb, void *user_data);

// Log all active leases at info level. Thread context only.
void memlease_dump(void);

#ifdef __cplusplus
}
#endif

#endif // MEMLEASE_H
