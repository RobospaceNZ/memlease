/*
 * memlease.c
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

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "memlease.h"

LOG_MODULE_REGISTER(memlease, CONFIG_MEMLEASE_LOG_LEVEL);

#define MEMLEASE_STATUS_ALLOCATED               (1<<0)  // This entry is allocated
#define MEMLEASE_STATUS_LOCKED                  (1<<1)  // Don't free, even after timeout. A thread can lock it when it is busy with the buffer.
#define MEMLEASE_STATUS_ERROR_ON_TIMEOUT        (1<<2)  // When timeout occurs, send and error log message.

// The lower 16 bits of a handle hold the entry index + 1, so the highest usable index is 0xFFFE
#define MEMLEASE_MAX_ENTRY_INDEX                0xFFFE

typedef struct {
    void *buf;
    uint32_t size;
    int64_t expiry_time;        // Uptime in milliseconds, 0 means infinite
    uint16_t allocate_num;      // Unique number assigned for this allocation
    uint8_t status;
    uint8_t release_count;      // Number of times - 1 this buffer need to be released before it is freed
} memlease_entry_t;

typedef struct memlease_entry_block {
    memlease_entry_t entries[CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF];
    struct memlease_entry_block *next;
    struct memlease_entry_block *prev;
    uint32_t allocated_count;
} memlease_entry_block_t;

typedef struct {
    int64_t next_timeout;           // Earliest expiry time of all unlocked entries, 0 means none
    memlease_entry_block_t entries; // First block, statically allocated
    memlease_entry_block_t *last_block;
    uint32_t num_entries_allocated; // Number of entries in all blocks (allocated or not)
    uint32_t num_allocated;         // Number of entries currently in use
    uint16_t allocate_count;        // Counter to assign to allocated entries
    bool recalculate_timeout;
} memlease_data_t;

static memlease_data_t memlease_data = {
    .last_block = &memlease_data.entries,
    .num_entries_allocated = CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF,
};
static K_MUTEX_DEFINE(memlease_lock);
static K_SEM_DEFINE(timeout_changed_sem, 0, 1);

static inline uint32_t memlease_make_handle(uint32_t index, uint16_t allocate_num) {
    return (index + 1) | ((uint32_t)allocate_num << 16); // Handles are 1-based, so 0 is never valid
}

// Check if this entry affects the next timeout. Must be called with memlease_lock held.
static void memlease_check_timeout(memlease_entry_t *entry) {
    if (entry->expiry_time == 0) {
        // Infinite timeout
        return;
    }
    if (entry->status & MEMLEASE_STATUS_LOCKED) {
        // Locked entries do not affect timeout calculation
        return;
    }
    if (entry->expiry_time <= k_uptime_get()) {
        // Already expired
        memlease_data.recalculate_timeout = true;
        k_sem_give(&timeout_changed_sem);
        return;
    }
    if (!memlease_data.next_timeout || (entry->expiry_time < memlease_data.next_timeout)) {
        // New timeout is earlier than any other timeout
        memlease_data.next_timeout = entry->expiry_time;
        k_sem_give(&timeout_changed_sem);
    }
}

static void memlease_set_expiry(memlease_entry_t *entry, int64_t timeout) {
    if (timeout > 0) {
        entry->expiry_time = k_uptime_get() + timeout;
        memlease_check_timeout(entry);
    } else {
        entry->expiry_time = 0; // Infinite
    }
}

// Initialize a memlease entry and return its handle, 0 on failure
static uint32_t memlease_init_entry(uint32_t index, memlease_entry_t *entry, uint32_t size, int64_t timeout, void **buf_out) {
    entry->buf = k_malloc(size);
    if (!entry->buf) {
        return 0;
    }
    if (buf_out) {
        *buf_out = entry->buf;
    }
    entry->size = size;
    entry->status = MEMLEASE_STATUS_ALLOCATED | MEMLEASE_STATUS_ERROR_ON_TIMEOUT;
    entry->release_count = 0;   // Default release count
    entry->allocate_num = memlease_data.allocate_count++;   // Wraps around at 0xFFFF
    memlease_set_expiry(entry, timeout);
    return memlease_make_handle(index, entry->allocate_num);
}

// Free the buffer of an entry and mark it unused. Must be called with memlease_lock held.
// Does not free the block, call memlease_trim_blocks() afterwards.
static void memlease_free_entry(memlease_entry_block_t *block, memlease_entry_t *entry) {
    k_free(entry->buf);
    memset(entry, 0, sizeof(memlease_entry_t));
    if (block->allocated_count) {
        block->allocated_count--;
    }
    if (memlease_data.num_allocated) {
        memlease_data.num_allocated--;
    }
}

// Free empty dynamically allocated blocks at the end of the list. Must be called with memlease_lock held.
// Only trailing blocks can be freed, because the entry index in a handle depends on the block position.
static void memlease_trim_blocks(void) {
    memlease_entry_block_t *block = memlease_data.last_block;

    while ((block != &memlease_data.entries) && (block->allocated_count == 0)) {
        memlease_entry_block_t *prev_block = block->prev;
        prev_block->next = NULL;
        k_free(block);
        memlease_data.num_entries_allocated -= CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF;
        block = prev_block;
    }
    memlease_data.last_block = block;
}

uint32_t memlease_alloc(uint32_t size, void **buf_out, int64_t timeout) {
    uint32_t handle = 0;
    uint32_t index = 0;
    memlease_entry_block_t *block = &memlease_data.entries;
    memlease_entry_t *entry = NULL;

    if (buf_out) {
        *buf_out = NULL;
    }
    if (k_is_in_isr()) {
        LOG_ERR("memlease_alloc called from ISR");
        return 0;
    }
    k_mutex_lock(&memlease_lock, K_FOREVER);
    // Find a free entry, skipping blocks that are full
    while (block) {
        if (block->allocated_count < CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF) {
            for (uint32_t i = 0; i < CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF; i++) {
                if (!(block->entries[i].status & MEMLEASE_STATUS_ALLOCATED)) {
                    entry = &block->entries[i];
                    index += i;
                    break;
                }
            }
            if (entry) {
                break;
            }
            LOG_ERR("Block allocated count is %d but no free entry was found", block->allocated_count);
        }
        index += CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF;
        block = block->next;
    }
    do {
        if (!entry) {
            // No free entry found, need to allocate a new block
            if (index > MEMLEASE_MAX_ENTRY_INDEX) {
                LOG_ERR("Maximum number of memlease entries reached");
                break;
            }
            block = k_calloc(1, sizeof(memlease_entry_block_t));
            if (!block) {
                LOG_ERR("Failed to allocate new memlease entry block");
                break;
            }
            block->prev = memlease_data.last_block;
            memlease_data.last_block->next = block;
            memlease_data.last_block = block;
            memlease_data.num_entries_allocated += CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF;
            entry = &block->entries[0];
        } else if (index > MEMLEASE_MAX_ENTRY_INDEX) {
            LOG_ERR("Maximum number of memlease entries reached");
            break;
        }
        handle = memlease_init_entry(index, entry, size, timeout, buf_out);
        if (handle == 0) {
            LOG_ERR("Failed to allocate buffer of size %d", size);
            break;
        }
        block->allocated_count++;
        memlease_data.num_allocated++;
    } while (0);
    if (handle == 0) {
        // Free the new block again if it ended up unused
        memlease_trim_blocks();
    }
    k_mutex_unlock(&memlease_lock);
    return handle;
}

// Check if the handle is valid. Must be called with memlease_lock held.
// On success, out_entry and out_block are set to the corresponding entry and block
static int32_t memlease_check_handle(uint32_t handle, memlease_entry_t **out_entry, memlease_entry_block_t **out_block) {
    uint16_t allocate_num = (handle >> 16) & 0xFFFF;
    uint32_t entry_index = (handle & 0xFFFF) - 1; // Convert to 0-based index
    memlease_entry_block_t *block = &memlease_data.entries;
    memlease_entry_t *entry = NULL;

    // Check if entry index is within bounds
    if (entry_index >= memlease_data.num_entries_allocated) {
        LOG_WRN("Handle 0x%08X entry index out of bounds (max %d)", handle, memlease_data.num_entries_allocated - 1);
        return MEMLEASE_ERROR_ENTRY_OUT_OF_BOUNDS;
    }
    // Traverse to the correct block
    while (entry_index >= CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF) {
        if (!block->next) {
            LOG_ERR("Next should point to a valid block");
            return MEMLEASE_ERROR_CORRUPT_BLOCK_POINTER;
        }
        block = block->next;
        entry_index -= CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF;
    }
    entry = &block->entries[entry_index];
    if (!(entry->status & MEMLEASE_STATUS_ALLOCATED)) {
        LOG_WRN("Handle 0x%08X points to unallocated entry", handle);
        return MEMLEASE_ERROR_NOT_ALLOCATED;
    }
    if (entry->allocate_num != allocate_num) {
        LOG_WRN("Handle 0x%08X allocation number mismatch (expected %d, got %d)", handle, entry->allocate_num, allocate_num);
        return MEMLEASE_ERROR_ALLOCATION_NUM_MISMATCH;
    }
    // Valid handle
    *out_entry = entry;
    *out_block = block;
    return 0;
}

void *memlease_get_buf(uint32_t handle, uint32_t *out_size, bool lock) {
    memlease_entry_t *entry = NULL;
    memlease_entry_block_t *block = NULL;
    void *buf = NULL;

    if (k_is_in_isr()) {
        LOG_ERR("memlease_get_buf called from ISR");
        return NULL;
    }
    k_mutex_lock(&memlease_lock, K_FOREVER);
    if (memlease_check_handle(handle, &entry, &block) == 0) {
        if (lock) {
            entry->status |= MEMLEASE_STATUS_LOCKED;
        }
        if (out_size) {
            *out_size = entry->size;
        }
        buf = entry->buf;
    }
    k_mutex_unlock(&memlease_lock);
    return buf;
}

int32_t memlease_free(uint32_t handle) {
    memlease_entry_t *entry = NULL;
    memlease_entry_block_t *block = NULL;
    int32_t ret;

    if (k_is_in_isr()) {
        LOG_ERR("memlease_free called from ISR");
        return MEMLEASE_ERROR_CANNOT_CALL_FROM_ISR;
    }
    k_mutex_lock(&memlease_lock, K_FOREVER);
    ret = memlease_check_handle(handle, &entry, &block);
    if (ret == 0) {
        if (entry->release_count > 0) {
            entry->release_count--;
            entry->status &= ~MEMLEASE_STATUS_LOCKED;
            memlease_check_timeout(entry);
        } else {
            // No need to wake the timeout thread. If this entry held the next timeout,
            // the thread wakes at that time, finds nothing to do and recalculates.
            memlease_free_entry(block, entry);
            memlease_trim_blocks();
        }
    }
    k_mutex_unlock(&memlease_lock);
    return ret;
}

int32_t memlease_set_lock(uint32_t handle, bool lock) {
    memlease_entry_t *entry = NULL;
    memlease_entry_block_t *block = NULL;
    int32_t ret;

    if (k_is_in_isr()) {
        LOG_ERR("memlease_set_lock called from ISR");
        return MEMLEASE_ERROR_CANNOT_CALL_FROM_ISR;
    }
    k_mutex_lock(&memlease_lock, K_FOREVER);
    ret = memlease_check_handle(handle, &entry, &block);
    if (ret == 0) {
        if (lock) {
            entry->status |= MEMLEASE_STATUS_LOCKED;
        } else {
            entry->status &= ~MEMLEASE_STATUS_LOCKED;
            memlease_check_timeout(entry);
        }
    }
    k_mutex_unlock(&memlease_lock);
    return ret;
}

int32_t memlease_set_timeout(uint32_t handle, int64_t timeout) {
    memlease_entry_t *entry = NULL;
    memlease_entry_block_t *block = NULL;
    int32_t ret;

    if (k_is_in_isr()) {
        LOG_ERR("memlease_set_timeout called from ISR");
        return MEMLEASE_ERROR_CANNOT_CALL_FROM_ISR;
    }
    k_mutex_lock(&memlease_lock, K_FOREVER);
    ret = memlease_check_handle(handle, &entry, &block);
    if (ret == 0) {
        // If the timeout moved later, the thread wakes at the old time and recalculates
        memlease_set_expiry(entry, timeout);
    }
    k_mutex_unlock(&memlease_lock);
    return ret;
}

int32_t memlease_set_error_on_timeout(uint32_t handle, bool error_on_timeout) {
    memlease_entry_t *entry = NULL;
    memlease_entry_block_t *block = NULL;
    int32_t ret;

    if (k_is_in_isr()) {
        LOG_ERR("memlease_set_error_on_timeout called from ISR");
        return MEMLEASE_ERROR_CANNOT_CALL_FROM_ISR;
    }
    k_mutex_lock(&memlease_lock, K_FOREVER);
    ret = memlease_check_handle(handle, &entry, &block);
    if (ret == 0) {
        if (error_on_timeout) {
            entry->status |= MEMLEASE_STATUS_ERROR_ON_TIMEOUT;
        } else {
            entry->status &= ~MEMLEASE_STATUS_ERROR_ON_TIMEOUT;
        }
    }
    k_mutex_unlock(&memlease_lock);
    return ret;
}

int32_t memlease_set_release_count(uint32_t handle, uint8_t release_count) {
    memlease_entry_t *entry = NULL;
    memlease_entry_block_t *block = NULL;
    int32_t ret;

    if (k_is_in_isr()) {
        LOG_ERR("memlease_set_release_count called from ISR");
        return MEMLEASE_ERROR_CANNOT_CALL_FROM_ISR;
    }
    k_mutex_lock(&memlease_lock, K_FOREVER);
    ret = memlease_check_handle(handle, &entry, &block);
    if (ret == 0) {
        entry->release_count = release_count;
    }
    k_mutex_unlock(&memlease_lock);
    return ret;
}

uint32_t memlease_get_num_allocated(void) {
    uint32_t num;

    k_mutex_lock(&memlease_lock, K_FOREVER);
    num = memlease_data.num_allocated;
    k_mutex_unlock(&memlease_lock);
    return num;
}

// Free all expired entries and calculate the next timeout. Must be called with memlease_lock held.
static void memlease_process_timeouts(int64_t now) {
    uint32_t index = 0;

    memlease_data.next_timeout = 0;
    memlease_data.recalculate_timeout = false;
    for (memlease_entry_block_t *block = &memlease_data.entries; block; block = block->next) {
        for (uint32_t i = 0; i < CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF; i++, index++) {
            memlease_entry_t *entry = &block->entries[i];

            if (!(entry->status & MEMLEASE_STATUS_ALLOCATED) || (entry->expiry_time == 0) ||
                (entry->status & MEMLEASE_STATUS_LOCKED)) {
                // Unused, infinite or locked entries never time out
                continue;
            }
            if (now >= entry->expiry_time) {
                // This entry has timed out
                if (entry->status & MEMLEASE_STATUS_ERROR_ON_TIMEOUT) {
                    LOG_ERR("Memlease entry timed out (handle: 0x%08X)", memlease_make_handle(index, entry->allocate_num));
                }
                memlease_free_entry(block, entry);
            } else if (!memlease_data.next_timeout || (entry->expiry_time < memlease_data.next_timeout)) {
                memlease_data.next_timeout = entry->expiry_time;
            }
        }
    }
    memlease_trim_blocks();
}

// Memlease timeout management thread
static void memlease_thread_fn(void *p1, void *p2, void *p3) {
    k_timeout_t sleep_time = K_FOREVER;
    int64_t now;

    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
        k_sem_take(&timeout_changed_sem, sleep_time);
        k_mutex_lock(&memlease_lock, K_FOREVER);
        now = k_uptime_get();
        if (memlease_data.recalculate_timeout ||
            (memlease_data.next_timeout && (now >= memlease_data.next_timeout))) {
            // At least one of the entries has timed out or we need to recalculate the next timeout
            memlease_process_timeouts(now);
        }
        if (!memlease_data.next_timeout) {
            // No timeouts scheduled
            sleep_time = K_FOREVER;
        } else if (memlease_data.next_timeout > now) {
            sleep_time = K_MSEC(memlease_data.next_timeout - now);
        } else {
            sleep_time = K_NO_WAIT;
        }
        k_mutex_unlock(&memlease_lock);
    }
}

K_THREAD_DEFINE(memlease_thread, CONFIG_MEMLEASE_THREAD_STACK_SIZE, memlease_thread_fn, NULL, NULL, NULL, CONFIG_MEMLEASE_THREAD_PRIORITY, 0, 0);
