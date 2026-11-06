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
#define MEMLEASE_ERROR_CANNOT_CALL_FROM_ISR     -5  // Function was called from an ISR

// Allocate a leased memory buffer.
// size     - Size of the buffer in bytes
// buf_out  - If not NULL, the pointer to the allocated buffer is written to this pointer
// timeout  - Time in milliseconds the buffer is leased for, 0 (or negative) means infinite
// Returns a handle to the leased memory buffer, 0 on failure. The handle is not a pointer.
uint32_t memlease_alloc(uint32_t size, void **buf_out, int64_t timeout);

// Return a pointer to the buffer, NULL if the handle is invalid (freed, timed out or stale).
// out_size - If not NULL, the size of the buffer is written to this pointer
// lock     - If true, the buffer is locked and will not be freed on timeout while in use.
//            A locked buffer is unlocked by memlease_set_lock(handle, false) or by memlease_free().
//            If false, the lock state is left unchanged.
void *memlease_get_buf(uint32_t handle, uint32_t *out_size, bool lock);

// Release the leased buffer. If the release count is greater than zero it is decremented
// (and the buffer is unlocked), otherwise the buffer is freed.
// Returns 0 on success, negative MEMLEASE_ERROR_* on failure.
int32_t memlease_free(uint32_t handle);

// Lock (true) or unlock (false) the buffer. Locked buffers are never freed on timeout.
// If the lease expired while locked, the buffer is freed shortly after it is unlocked.
int32_t memlease_set_lock(uint32_t handle, bool lock);

// Renew the lease: the buffer now expires timeout milliseconds from now. 0 (or negative) means infinite.
int32_t memlease_set_timeout(uint32_t handle, int64_t timeout);

// If set (default), an error message is logged when the buffer times out.
int32_t memlease_set_error_on_timeout(uint32_t handle, bool error_on_timeout);

// Set the release count for the buffer. This is one less than the number of times the buffer
// needs to be released before it is freed. For example, if the buffer is sent to 3 consumers
// that each call memlease_free(), set release_count to 2.
int32_t memlease_set_release_count(uint32_t handle, uint8_t release_count);

// Return the number of currently allocated (leased) buffers.
uint32_t memlease_get_num_allocated(void);

#ifdef __cplusplus
}
#endif

#endif // MEMLEASE_H
