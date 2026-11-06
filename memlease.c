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

/*
 * Locking
 *
 * All entry and block bookkeeping is protected by memlease_lock, a spinlock, so every function
 * except memlease_foreach() / memlease_dump() can be called from an ISR. Heap calls (k_malloc,
 * k_free, ...) are ISR-safe in Zephyr, but they are kept outside the spinlock to keep interrupt
 * latency low.
 *
 * Entry blocks are only ever freed by the memlease thread, while holding memlease_block_mutex.
 * Code that walks the block list with memlease_lock released (the timeout scan, memlease_foreach)
 * holds that mutex, so the block it is looking at cannot disappear. Blocks are only added at the
 * end of the list, which is safe while such a walk is in progress.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include "memlease.h"

LOG_MODULE_REGISTER(memlease, CONFIG_MEMLEASE_LOG_LEVEL);

#define MEMLEASE_STATUS_ALLOCATED               (1<<0)  // This entry is allocated
#define MEMLEASE_STATUS_ERROR_ON_TIMEOUT        (1<<1)  // When timeout occurs, send and error log message.

// The lower 16 bits of a handle hold the entry index + 1, so the highest usable index is 0xFFFE
#define MEMLEASE_MAX_ENTRY_INDEX                0xFFFE
#define MEMLEASE_MAX_COUNT                      UINT8_MAX

typedef struct {
    void *buf;
    uint32_t size;
    int64_t expiry_time;        // Uptime in milliseconds, 0 means infinite
    memlease_expiry_cb_t expiry_cb;
    void *user_data;
#ifdef CONFIG_MEMLEASE_DEBUG_INFO
    const char *file;
    int64_t alloc_time;
    uint32_t line;
#endif
    uint16_t allocate_num;      // Unique number assigned for this allocation
    uint8_t status;
    uint8_t release_count;      // Number of times - 1 this buffer need to be released before it is freed
    uint8_t lock_count;         // Buffer does not time out while this is above zero
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
    struct memlease_stats stats;
    uint16_t allocate_count;        // Counter to assign to allocated entries
    bool recalculate_timeout;       // An entry expired, the thread must scan
    bool trim_pending;              // A dynamic block became empty, the thread should try to free blocks
} memlease_data_t;

static memlease_data_t memlease_data = {
    .last_block = &memlease_data.entries,
    .num_entries_allocated = CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF,
    .stats.num_blocks = 1,
};
static struct k_spinlock memlease_lock;
static K_MUTEX_DEFINE(memlease_block_mutex);
static K_SEM_DEFINE(memlease_wake_sem, 0, 1);

static inline uint32_t memlease_make_handle(uint32_t index, uint16_t allocate_num) {
    return (index + 1) | ((uint32_t)allocate_num << 16); // Handles are 1-based, so 0 is never valid
}

// Wake the timeout thread. Called after releasing memlease_lock, so the thread can be scheduled right away.
static inline void memlease_wake(bool wake) {
    if (wake) {
        k_sem_give(&memlease_wake_sem);
    }
}

// Check if this entry affects the next timeout. Must be called with memlease_lock held.
// Returns true if the thread needs to be woken.
static bool memlease_check_timeout(memlease_entry_t *entry) {
    if (entry->expiry_time == 0) {
        // Infinite timeout
        return false;
    }
    if (entry->lock_count) {
        // Locked entries do not affect timeout calculation
        return false;
    }
    if (entry->expiry_time <= k_uptime_get()) {
        // Already expired
        memlease_data.recalculate_timeout = true;
        return true;
    }
    if (!memlease_data.next_timeout || (entry->expiry_time < memlease_data.next_timeout)) {
        // New timeout is earlier than any other timeout
        memlease_data.next_timeout = entry->expiry_time;
        return true;
    }
    return false;
}

// Must be called with memlease_lock held. Returns true if the thread needs to be woken.
static bool memlease_set_expiry(memlease_entry_t *entry, int64_t timeout) {
    if (timeout > 0) {
        entry->expiry_time = k_uptime_get() + timeout;
        return memlease_check_timeout(entry);
    }
    // Infinite. If this entry held the next timeout, the thread wakes then, finds nothing and recalculates.
    entry->expiry_time = 0;
    return false;
}

// Find a free entry, skipping blocks that are full. Must be called with memlease_lock held.
// If none is free, returns NULL and out_index is set to the index the first entry of a new block would get.
static memlease_entry_t *memlease_find_free_entry(memlease_entry_block_t **out_block, uint32_t *out_index) {
    uint32_t index = 0;

    for (memlease_entry_block_t *block = &memlease_data.entries; block; block = block->next) {
        if (block->allocated_count < CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF) {
            for (uint32_t i = 0; i < CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF; i++) {
                if (!(block->entries[i].status & MEMLEASE_STATUS_ALLOCATED)) {
                    *out_block = block;
                    *out_index = index + i;
                    return &block->entries[i];
                }
            }
            LOG_ERR("Block allocated count is %d but no free entry was found", block->allocated_count);
        }
        index += CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF;
    }
    *out_index = index;
    return NULL;
}

// Initialize a free entry and return its handle. Must be called with memlease_lock held.
// Returns true in wake if the thread needs to be woken.
static uint32_t memlease_init_entry(uint32_t index, memlease_entry_block_t *block, memlease_entry_t *entry,
                                    void *buf, uint32_t size, int64_t timeout, const char *file, uint32_t line,
                                    bool *wake) {
    struct memlease_stats *stats = &memlease_data.stats;

    memset(entry, 0, sizeof(memlease_entry_t));
    entry->buf = buf;
    entry->size = size;
    entry->status = MEMLEASE_STATUS_ALLOCATED | MEMLEASE_STATUS_ERROR_ON_TIMEOUT;
    entry->allocate_num = memlease_data.allocate_count++;   // Wraps around at 0xFFFF
#ifdef CONFIG_MEMLEASE_DEBUG_INFO
    entry->file = file;
    entry->line = line;
    entry->alloc_time = k_uptime_get();
#else
    ARG_UNUSED(file);
    ARG_UNUSED(line);
#endif
    *wake = memlease_set_expiry(entry, timeout);
    block->allocated_count++;
    stats->num_allocated++;
    stats->bytes_allocated += size;
    stats->total_allocs++;
    stats->peak_allocated = MAX(stats->peak_allocated, stats->num_allocated);
    stats->peak_bytes = MAX(stats->peak_bytes, stats->bytes_allocated);
    return memlease_make_handle(index, entry->allocate_num);
}

// Mark an entry unused. Must be called with memlease_lock held.
// Returns the buffer, which the caller must k_free() after releasing memlease_lock.
// Returns true in wake if the thread needs to be woken to free an empty block.
static void *memlease_release_entry(memlease_entry_block_t *block, memlease_entry_t *entry, bool *wake) {
    void *buf = entry->buf;

    if (memlease_data.stats.num_allocated) {
        memlease_data.stats.num_allocated--;
    }
    memlease_data.stats.bytes_allocated -= MIN(entry->size, memlease_data.stats.bytes_allocated);
    memset(entry, 0, sizeof(memlease_entry_t));
    if (block->allocated_count) {
        block->allocated_count--;
    }
    if ((block->allocated_count == 0) && (block != &memlease_data.entries)) {
        memlease_data.trim_pending = true;
        *wake = true;
    }
    return buf;
}

uint32_t memlease_alloc_at(uint32_t size, void **buf_out, int64_t timeout, const char *file, uint32_t line) {
    memlease_entry_block_t *new_block = NULL;
    memlease_entry_block_t *block = NULL;
    memlease_entry_t *entry = NULL;
    k_spinlock_key_t key;
    uint32_t handle = 0;
    uint32_t index = 0;
    bool wake = false;
    void *buf;

    if (buf_out) {
        *buf_out = NULL;
    }
    buf = (size > 0) ? k_malloc(size) : NULL;
    while (buf) {
        key = k_spin_lock(&memlease_lock);
        entry = memlease_find_free_entry(&block, &index);
        if (!entry && new_block && (index + CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF - 1 <= MEMLEASE_MAX_ENTRY_INDEX)) {
            // Append the block allocated in the previous iteration
            new_block->prev = memlease_data.last_block;
            memlease_data.last_block->next = new_block;
            memlease_data.last_block = new_block;
            memlease_data.num_entries_allocated += CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF;
            memlease_data.stats.num_blocks++;
            block = new_block;
            entry = &new_block->entries[0];
            new_block = NULL;
        }
        if (entry) {
            handle = memlease_init_entry(index, block, entry, buf, size, timeout, file, line, &wake);
            k_spin_unlock(&memlease_lock, key);
            break;
        }
        k_spin_unlock(&memlease_lock, key);
        if (new_block) {
            // Already tried with a new block, so the maximum number of entries is reached
            LOG_ERR("Maximum number of memlease entries reached");
            break;
        }
        // No free entry, allocate a new block outside the spinlock and try again
        new_block = k_calloc(1, sizeof(memlease_entry_block_t));
        if (!new_block) {
            LOG_ERR("Failed to allocate new memlease entry block");
            break;
        }
    }
    if (new_block) {
        // Not needed after all, another context freed an entry in the meantime
        k_free(new_block);
    }
    if (handle == 0) {
        if (buf) {
            k_free(buf);
        } else {
            LOG_ERR("Failed to allocate buffer of size %u", size);
        }
        key = k_spin_lock(&memlease_lock);
        memlease_data.stats.alloc_failures++;
        k_spin_unlock(&memlease_lock, key);
    } else if (buf_out) {
        *buf_out = buf;
    }
    memlease_wake(wake);
    return handle;
}

// Check if the handle is valid. Must be called with memlease_lock held.
// On success, out_entry and (if not NULL) out_block are set to the corresponding entry and block
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
    if (out_block) {
        *out_block = block;
    }
    return 0;
}

void *memlease_get_buf(uint32_t handle, uint32_t *out_size, bool lock) {
    memlease_entry_t *entry = NULL;
    void *buf = NULL;
    k_spinlock_key_t key = k_spin_lock(&memlease_lock);

    if (memlease_check_handle(handle, &entry, NULL) == 0) {
        if (lock && (entry->lock_count >= MEMLEASE_MAX_COUNT)) {
            LOG_ERR("Handle 0x%08X lock count overflow", handle);
        } else {
            if (lock) {
                entry->lock_count++;
            }
            if (out_size) {
                *out_size = entry->size;
            }
            buf = entry->buf;
        }
    }
    k_spin_unlock(&memlease_lock, key);
    return buf;
}

int32_t memlease_free(uint32_t handle) {
    memlease_entry_t *entry = NULL;
    memlease_entry_block_t *block = NULL;
    void *buf = NULL;
    bool wake = false;
    k_spinlock_key_t key = k_spin_lock(&memlease_lock);
    int32_t ret = memlease_check_handle(handle, &entry, &block);

    if (ret == 0) {
        if (entry->lock_count) {
            entry->lock_count--;
        }
        if (entry->release_count > 0) {
            entry->release_count--;
            wake = memlease_check_timeout(entry);
        } else {
            // No need to wake the thread for the timeout. If this entry held the next timeout,
            // the thread wakes at that time, finds nothing to do and recalculates.
            buf = memlease_release_entry(block, entry, &wake);
        }
    }
    k_spin_unlock(&memlease_lock, key);
    if (buf) {
        k_free(buf);
    }
    memlease_wake(wake);
    return ret;
}

int32_t memlease_set_lock(uint32_t handle, bool lock) {
    memlease_entry_t *entry = NULL;
    bool wake = false;
    k_spinlock_key_t key = k_spin_lock(&memlease_lock);
    int32_t ret = memlease_check_handle(handle, &entry, NULL);

    if (ret == 0) {
        if (lock) {
            if (entry->lock_count >= MEMLEASE_MAX_COUNT) {
                ret = MEMLEASE_ERROR_COUNT_OVERFLOW;
            } else {
                entry->lock_count++;
            }
        } else if (entry->lock_count == 0) {
            ret = MEMLEASE_ERROR_NOT_LOCKED;
        } else {
            entry->lock_count--;
            wake = memlease_check_timeout(entry);
        }
    }
    k_spin_unlock(&memlease_lock, key);
    memlease_wake(wake);
    return ret;
}

int32_t memlease_set_timeout(uint32_t handle, int64_t timeout) {
    memlease_entry_t *entry = NULL;
    bool wake = false;
    k_spinlock_key_t key = k_spin_lock(&memlease_lock);
    int32_t ret = memlease_check_handle(handle, &entry, NULL);

    if (ret == 0) {
        // If the timeout moved later, the thread wakes at the old time and recalculates
        wake = memlease_set_expiry(entry, timeout);
    }
    k_spin_unlock(&memlease_lock, key);
    memlease_wake(wake);
    return ret;
}

int32_t memlease_set_error_on_timeout(uint32_t handle, bool error_on_timeout) {
    memlease_entry_t *entry = NULL;
    k_spinlock_key_t key = k_spin_lock(&memlease_lock);
    int32_t ret = memlease_check_handle(handle, &entry, NULL);

    if (ret == 0) {
        if (error_on_timeout) {
            entry->status |= MEMLEASE_STATUS_ERROR_ON_TIMEOUT;
        } else {
            entry->status &= ~MEMLEASE_STATUS_ERROR_ON_TIMEOUT;
        }
    }
    k_spin_unlock(&memlease_lock, key);
    return ret;
}

int32_t memlease_set_release_count(uint32_t handle, uint8_t release_count) {
    memlease_entry_t *entry = NULL;
    k_spinlock_key_t key = k_spin_lock(&memlease_lock);
    int32_t ret = memlease_check_handle(handle, &entry, NULL);

    if (ret == 0) {
        entry->release_count = release_count;
    }
    k_spin_unlock(&memlease_lock, key);
    return ret;
}

int32_t memlease_retain(uint32_t handle) {
    memlease_entry_t *entry = NULL;
    k_spinlock_key_t key = k_spin_lock(&memlease_lock);
    int32_t ret = memlease_check_handle(handle, &entry, NULL);

    if (ret == 0) {
        if (entry->release_count >= MEMLEASE_MAX_COUNT) {
            ret = MEMLEASE_ERROR_COUNT_OVERFLOW;
        } else {
            entry->release_count++;
        }
    }
    k_spin_unlock(&memlease_lock, key);
    return ret;
}

int32_t memlease_set_expiry_cb(uint32_t handle, memlease_expiry_cb_t cb, void *user_data) {
    memlease_entry_t *entry = NULL;
    k_spinlock_key_t key = k_spin_lock(&memlease_lock);
    int32_t ret = memlease_check_handle(handle, &entry, NULL);

    if (ret == 0) {
        entry->expiry_cb = cb;
        entry->user_data = user_data;
    }
    k_spin_unlock(&memlease_lock, key);
    return ret;
}

int32_t memlease_realloc(uint32_t handle, uint32_t new_size, void **buf_out) {
    memlease_entry_t *entry = NULL;
    void *old_buf = NULL;
    void *new_buf;
    bool wake = false;
    k_spinlock_key_t key;
    int32_t ret;

    if (new_size == 0) {
        return MEMLEASE_ERROR_INVALID_ARG;
    }
    // Lock the entry so it cannot time out while the heap is resized outside the spinlock
    key = k_spin_lock(&memlease_lock);
    ret = memlease_check_handle(handle, &entry, NULL);
    if ((ret == 0) && (entry->lock_count >= MEMLEASE_MAX_COUNT)) {
        ret = MEMLEASE_ERROR_COUNT_OVERFLOW;
    }
    if (ret == 0) {
        entry->lock_count++;
        old_buf = entry->buf;
    }
    k_spin_unlock(&memlease_lock, key);
    if (ret != 0) {
        return ret;
    }

    new_buf = k_realloc(old_buf, new_size);

    key = k_spin_lock(&memlease_lock);
    ret = memlease_check_handle(handle, &entry, NULL);
    if (ret != 0) {
        // Freed by someone else during the realloc, which is not allowed
        LOG_ERR("Handle 0x%08X freed during memlease_realloc", handle);
    } else {
        entry->lock_count--;
        if (new_buf) {
            struct memlease_stats *stats = &memlease_data.stats;

            stats->bytes_allocated = stats->bytes_allocated - MIN(entry->size, stats->bytes_allocated) + new_size;
            stats->peak_bytes = MAX(stats->peak_bytes, stats->bytes_allocated);
            entry->buf = new_buf;
            entry->size = new_size;
        } else {
            ret = MEMLEASE_ERROR_NO_MEMORY;
        }
        wake = memlease_check_timeout(entry);
    }
    k_spin_unlock(&memlease_lock, key);
    if ((ret == 0) && buf_out) {
        *buf_out = new_buf;
    }
    memlease_wake(wake);
    return ret;
}

uint32_t memlease_get_num_allocated(void) {
    k_spinlock_key_t key = k_spin_lock(&memlease_lock);
    uint32_t num = memlease_data.stats.num_allocated;

    k_spin_unlock(&memlease_lock, key);
    return num;
}

void memlease_get_stats(struct memlease_stats *stats) {
    k_spinlock_key_t key = k_spin_lock(&memlease_lock);

    *stats = memlease_data.stats;
    k_spin_unlock(&memlease_lock, key);
}

int32_t memlease_foreach(memlease_foreach_cb_t cb, void *user_data) {
    int32_t count = 0;
    bool keep_going = true;

    if (k_is_in_isr()) {
        LOG_ERR("memlease_foreach called from ISR");
        return MEMLEASE_ERROR_CANNOT_CALL_FROM_ISR;
    }
    k_mutex_lock(&memlease_block_mutex, K_FOREVER);
    uint32_t index = 0;
    for (memlease_entry_block_t *block = &memlease_data.entries; block && keep_going; ) {
        for (uint32_t i = 0; (i < CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF) && keep_going; i++) {
            struct memlease_info info = { 0 };
            bool found = false;
            k_spinlock_key_t key = k_spin_lock(&memlease_lock);
            memlease_entry_t *entry = &block->entries[i];

            if (entry->status & MEMLEASE_STATUS_ALLOCATED) {
                found = true;
                info.handle = memlease_make_handle(index + i, entry->allocate_num);
                info.buf = entry->buf;
                info.size = entry->size;
                info.expiry_time = entry->expiry_time;
                info.release_count = entry->release_count;
                info.lock_count = entry->lock_count;
                info.error_on_timeout = (entry->status & MEMLEASE_STATUS_ERROR_ON_TIMEOUT) != 0;
                info.expiry_cb = entry->expiry_cb;
#ifdef CONFIG_MEMLEASE_DEBUG_INFO
                info.file = entry->file;
                info.line = entry->line;
                info.alloc_time = entry->alloc_time;
#endif
            }
            k_spin_unlock(&memlease_lock, key);
            if (found) {
                count++;
                keep_going = cb(&info, user_data);
            }
        }
        k_spinlock_key_t key = k_spin_lock(&memlease_lock);
        block = block->next;
        k_spin_unlock(&memlease_lock, key);
        index += CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF;
    }
    k_mutex_unlock(&memlease_block_mutex);
    return count;
}

static bool memlease_dump_cb(const struct memlease_info *info, void *user_data) {
    int64_t now = k_uptime_get();

    ARG_UNUSED(user_data);
    LOG_INF("0x%08X size %u expires %d ms locks %u releases %u %s:%u age %d ms",
            info->handle, info->size,
            info->expiry_time ? (int32_t)(info->expiry_time - now) : -1,
            info->lock_count, info->release_count + 1,
            info->file ? info->file : "?", info->line,
            info->alloc_time ? (int32_t)(now - info->alloc_time) : -1);
    return true;
}

void memlease_dump(void) {
    struct memlease_stats stats;

    memlease_get_stats(&stats);
    LOG_INF("%u leases, %u bytes (peak %u leases, %u bytes), %u timeouts, %u failures, %u blocks",
            stats.num_allocated, stats.bytes_allocated, stats.peak_allocated, stats.peak_bytes,
            stats.total_timeouts, stats.alloc_failures, stats.num_blocks);
    memlease_foreach(memlease_dump_cb, NULL);
}

// Free all expired entries and calculate the next timeout. Must be called with memlease_block_mutex held.
// memlease_lock is released between blocks and around expiry callbacks to keep interrupt latency low.
static void memlease_process_timeouts(void) {
    uint32_t index = 0;
    k_spinlock_key_t key = k_spin_lock(&memlease_lock);
    int64_t now = k_uptime_get();
    memlease_entry_block_t *block = &memlease_data.entries;

    memlease_data.next_timeout = 0;
    memlease_data.recalculate_timeout = false;
    while (block) {
        for (uint32_t i = 0; i < CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF; i++) {
            memlease_entry_t *entry = &block->entries[i];

            if (!(entry->status & MEMLEASE_STATUS_ALLOCATED) || (entry->expiry_time == 0) || entry->lock_count) {
                // Unused, infinite or locked entries never time out
                continue;
            }
            if (now < entry->expiry_time) {
                if (!memlease_data.next_timeout || (entry->expiry_time < memlease_data.next_timeout)) {
                    memlease_data.next_timeout = entry->expiry_time;
                }
                continue;
            }
            // This entry has timed out
            uint32_t handle = memlease_make_handle(index + i, entry->allocate_num);
            uint32_t size = entry->size;
            bool log_error = (entry->status & MEMLEASE_STATUS_ERROR_ON_TIMEOUT) != 0;
            memlease_expiry_cb_t cb = entry->expiry_cb;
            void *user_data = entry->user_data;
            bool wake = false;  // The thread is already running, trimming happens after the scan
            void *buf = memlease_release_entry(block, entry, &wake);

            memlease_data.stats.total_timeouts++;
            k_spin_unlock(&memlease_lock, key);
            if (log_error) {
                LOG_ERR("Memlease entry timed out (handle: 0x%08X, size %u)", handle, size);
            }
            if (cb) {
                cb(handle, buf, size, user_data);
            }
            k_free(buf);
            key = k_spin_lock(&memlease_lock);
        }
        // Give interrupts a chance between blocks
        k_spin_unlock(&memlease_lock, key);
        key = k_spin_lock(&memlease_lock);
        block = block->next;
        index += CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF;
    }
    k_spin_unlock(&memlease_lock, key);
}

// Free empty dynamically allocated blocks at the end of the list. Must be called with memlease_block_mutex held.
// Only trailing blocks can be freed, because the entry index in a handle depends on the block position.
static void memlease_trim_blocks(void) {
    while (true) {
        k_spinlock_key_t key = k_spin_lock(&memlease_lock);
        memlease_entry_block_t *block = memlease_data.last_block;

        if ((block == &memlease_data.entries) || block->allocated_count) {
            memlease_data.trim_pending = false;
            k_spin_unlock(&memlease_lock, key);
            return;
        }
        memlease_data.last_block = block->prev;
        memlease_data.last_block->next = NULL;
        memlease_data.num_entries_allocated -= CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF;
        memlease_data.stats.num_blocks--;
        k_spin_unlock(&memlease_lock, key);
        k_free(block);
    }
}

// Memlease timeout management thread
static void memlease_thread_fn(void *p1, void *p2, void *p3) {
    k_timeout_t sleep_time = K_FOREVER;
    k_spinlock_key_t key;
    bool scan;
    bool trim;

    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
        k_sem_take(&memlease_wake_sem, sleep_time);
        k_mutex_lock(&memlease_block_mutex, K_FOREVER);
        key = k_spin_lock(&memlease_lock);
        int64_t now = k_uptime_get();
        // Scan if at least one entry has timed out or the next timeout needs to be recalculated
        scan = memlease_data.recalculate_timeout ||
               (memlease_data.next_timeout && (now >= memlease_data.next_timeout));
        k_spin_unlock(&memlease_lock, key);
        if (scan) {
            memlease_process_timeouts();
        }
        key = k_spin_lock(&memlease_lock);
        trim = memlease_data.trim_pending;
        k_spin_unlock(&memlease_lock, key);
        if (trim) {
            memlease_trim_blocks();
        }
        k_mutex_unlock(&memlease_block_mutex);

        key = k_spin_lock(&memlease_lock);
        now = k_uptime_get();
        if (memlease_data.recalculate_timeout) {
            sleep_time = K_NO_WAIT;
        } else if (!memlease_data.next_timeout) {
            // No timeouts scheduled
            sleep_time = K_FOREVER;
        } else if (memlease_data.next_timeout > now) {
            sleep_time = K_MSEC(memlease_data.next_timeout - now);
        } else {
            sleep_time = K_NO_WAIT;
        }
        k_spin_unlock(&memlease_lock, key);
    }
}

K_THREAD_DEFINE(memlease_thread, CONFIG_MEMLEASE_THREAD_STACK_SIZE, memlease_thread_fn, NULL, NULL, NULL, CONFIG_MEMLEASE_THREAD_PRIORITY, 0, 0);

#ifdef CONFIG_MEMLEASE_SHELL
#include <zephyr/shell/shell.h>

static bool memlease_shell_list_cb(const struct memlease_info *info, void *user_data) {
    const struct shell *sh = user_data;
    int64_t now = k_uptime_get();
    char expires[12] = "inf";

    if (info->expiry_time) {
        snprintk(expires, sizeof(expires), "%d", (int32_t)(info->expiry_time - now));
    }
    shell_print(sh, "0x%08X %8u %10s %5u %8u  %s:%u", info->handle, info->size, expires,
                info->lock_count, info->release_count + 1, info->file ? info->file : "?", info->line);
    return true;
}

static int memlease_cmd_list(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    shell_print(sh, "Handle         Size  Expires ms Locks Releases  Allocated at");
    int32_t count = memlease_foreach(memlease_shell_list_cb, (void *)sh);
    shell_print(sh, "%d leases", count);
    return 0;
}

static int memlease_cmd_stats(const struct shell *sh, size_t argc, char **argv) {
    struct memlease_stats stats;

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    memlease_get_stats(&stats);
    shell_print(sh, "Allocated:   %u leases, %u bytes", stats.num_allocated, stats.bytes_allocated);
    shell_print(sh, "Peak:        %u leases, %u bytes", stats.peak_allocated, stats.peak_bytes);
    shell_print(sh, "Total:       %u allocs, %u timeouts, %u failures", stats.total_allocs,
                stats.total_timeouts, stats.alloc_failures);
    shell_print(sh, "Blocks:      %u x %u entries", stats.num_blocks, CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(memlease_cmds,
    SHELL_CMD(list, NULL, "List active leases", memlease_cmd_list),
    SHELL_CMD(stats, NULL, "Show memlease statistics", memlease_cmd_stats),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(memlease, &memlease_cmds, "Memlease commands", NULL);
#endif // CONFIG_MEMLEASE_SHELL
