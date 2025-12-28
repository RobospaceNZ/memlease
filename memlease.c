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

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(memlease);

#define MEMLEASE_STATUS_ALLOCATED               (1<<0)  // This entry is allocated
#define MEMLEASE_STATUS_LOCKED                  (1<<1)  // Don't free, even after timeout. A thread can lock it when it is busy with the buffer.
#define MEMLEASE_STATUS_ERROR_ON_TIMEOUT        (1<<2)  // When timeout occurs, send and error log message.

#define MEMLEASE_ERROR_ENTRY_OUT_OF_BOUNDS      -1
#define MEMLEASE_ERROR_ALLOCATION_NUM_MISMATCH  -2
#define MEMLEASE_ERROR_CORRUPT_BLOCK_POINTER    -3
#define MEMLEASE_ERROR_NOT_ALLOCATED            -4
#define MEMLEASE_ERROR_CANNOT_CALL_FROM_ISR     -5

typedef struct {
    void *buf;
    uint32_t size;
    int64_t expiry_time;        // in milliseconds, 0 means infinite
    uint16_t allocate_num;      // Unique number assigend for this allocation
    uint8_t status;
    uint8_t release_count;      // Number of times - 1 this buffer need to be released before it is freed
} memlease_entry_t;

typedef struct {
    memlease_entry_t entries[CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF];
    void *next;
    void *prev;
    uint32_t allocated_count;
} memlease_entry_block_t;

typedef struct {
    int64_t next_timeout;
    memlease_entry_block_t entries;
    uint32_t num_entries_allocated;
    uint16_t allocate_count;        // Counter to assign to allocated entries
    bool recalculate_timeout;
} memlease_data_t;

static memlease_data_t memlease_data = {
    .num_entries_allocated = CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF,
};
static K_MUTEX_DEFINE(memlease_lock);
K_SEM_DEFINE(timeout_changed_sem, 0, 1);

// Check if this entry affects the next timeout
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

// Initialize a memlease entry and return its handle
static uint32_t memlease_init_entry(uint32_t index, memlease_entry_t *entry, uint32_t size, int64_t timeout) {
    uint32_t handle = 0;

    if (index == 0xFFFE) {
        // Maximum number of entries reached
        return 0;
    }
    entry->buf = k_malloc(size);
    if (!entry->buf) {
        return 0;
    }
    entry->size = size;
    entry->status = STATUS_ALLOCATED | STATUS_ERROR_ON_TIMEOUT;
    if (timeout > 0) {
        entry->expiry_time = k_uptime_get() + timeout;
        memlease_check_timeout(entry);
    } else {
        entry->expiry_time = 0; // Infinite
    }
    entry->release_count = 0;   // Default release count
    entry->allocate_num = memlease_data.allocate_count++;
    memlease_data.allocate_count &= 0xFFFF; // Wrap around
    handle = index + 1; // Handles are 1-based
    handle |= (entry->allocate_num << 16);
    return handle;
}

// size - Size of the buffer in bytes
// timeout - Time in milliseconds the buffer is leased for, 0 means infinite
// Returns a handle to the leased memory buffer, 0 on failure. This number cannot be used a pointer.
uint32_t memlease_alloc(uint32_t size, int64_t timeout)
{
    uint32_t handle = 0;
    bool error = false;

    if (k_is_in_isr()) {
        LOG_ERR("memlease_alloc called from ISR");
        return 0;
    }
    k_mutex_lock(&memlease_lock, K_FOREVER);
    do {
        // Find a free entry
        uint32_t i = 0;
        uint32_t entry_num = 0;
        memlease_entry_block_t *block = &memlease_data.entries;

        while (i < memlease_data.num_entries_allocated) {
            if (block->allocated_count >= CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF) {
                // This block is full, skip to next block
                i += CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF;
                if (!block->next) {
                    break;
                }
                block = (memlease_entry_block_t *)block->next;
                entry_num = 0;
                continue;
            }
            memlease_entry_t *entry = &block->entries[i];
            if (!(entry->status & STATUS_ALLOCATED)) {
                // Found a free entry
                handle = memlease_init_entry(i, entry, size, timeout);
                if (handle == 0) {
                    // Memory allocation failed
                    LOG_ERR("Failed to allocate buffer of size %d", size);
                    error = true;
                } else {
                    block->allocated_count++;
                }
                break;
            }
            entry_num++;
            if (entry_num >= CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF) {
                if (!block->next) {
                    LOG_ERR("Next block is NULL while number of allocated entries says there should be more");
                    break;
                }
                block = (memlease_entry_block_t *)block->next;
                entry_num = 0;
            }
            i++;
        }
        if ((handle != 0) || error) {
            // Successfully allocated or error occurred
            break;
        }
        // No free entry found, need to allocate a new block
        memlease_entry_block_t *new_block = k_calloc(sizeof(memlease_entry_block_t));
        if (!new_block) {
            // Memory allocation failed
            LOG_ERR("Failed to allocate new memlease entry block");
            break;
        }
        block->next = new_block;
        new_block->prev = block;
        memlease_data.num_entries_allocated += CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF;
        handle = memlease_init_entry(i, &new_block->entries[0], size, timeout);
        if (handle == 0) {
            // Memory allocation failed
            LOG_ERR("Failed to allocate buffer of size %d", size);
            k_free(new_block);
            block->next = NULL;
        } else {
            new_block->allocated_count++;
        }
    } while (0);
    k_mutex_unlock(&memlease_lock);
    return handle;
}

// Check if the handle is valid
// On success, out_entry and out_block are set to the corresponding entry and block
static int32_t memlease_check_handle(uint32_t handle, memlease_entry_t **out_entry, memlease_entry_block_t **out_block) {
    uint16_t allocate_num = (handle >> 16) & 0xFFFF;
    uint32_t entry_index = (handle & 0xFFFF) - 1; // Convert to 0-based index
    uint32_t current_index = 0;
    memlease_entry_block_t *block = &memlease_data.entries;
    memlease_entry_t *entry = NULL;

    // Check if entry index is within bounds
    if (entry_index >= memlease_data.num_entries_allocated) {
        LOG_ERR("Handle entry index %d out of bounds (max %d)", entry_index, memlease_data.num_entries_allocated - 1);
        return MEMLEASE_ERROR_ENTRY_OUT_OF_BOUNDS;      // Invalid handle
    }
    // Traverse to the correct block
    while(entry_index >= CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF) {
        if (!block->next) {
            LOG_ERR("Next should point to a valid block");
            return MEMLEASE_ERROR_CORRUPT_BLOCK_POINTER;
        }
        block = (memlease_entry_block_t *)block->next;
        entry_index -= CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF;
    }
    entry = &block->entries[entry_index];
    if (!(entry->status & STATUS_ALLOCATED)) {
        LOG_ERR("Handle points to unallocated entry");
        return MEMLEASE_ERROR_NOT_ALLOCATED;
    }
    if (entry->allocate_num != allocate_num) {
        LOG_ERR("Handle allocation number mismatch (expected %d, got %d)", entry->allocate_num, allocate_num);
        return MEMLEASE_ERROR_ALLOCATION_NUM_MISMATCH;  // Invalid handle
    }
    // Valid handle
    *out_entry = entry;
    *out_block = block;
    return 0;
}

// Return a pointer to the buffer, NULL if the handle is invalid
// handle - Handle returned by memlease_alloc
// out_size - If not NULL, the size of the buffer is written to this pointer
// lock - If true, the buffer is marked as locked and will not be freed on timeout.
//        If calling code is going to use this buffer and wants to prevent it from being freed while in use, 
//        it should set this to true. Caller must make sure to free the buffer when finished. 
void *memlease_get_buf(uint32_t handle, uint32_t *out_size, bool lock) {
    memlease_entry_t *entry = NULL;
    memlease_entry_block_t *block = NULL;
    int32_t ret;

    if (k_is_in_isr()) {
        LOG_ERR("memlease_get_buf called from ISR");
        return NULL;
    }
    k_mutex_lock(&memlease_lock, K_FOREVER);
    ret = memlease_check_handle(handle, &entry, &block);
    if (ret != 0) {
        k_mutex_unlock(&memlease_lock);
        return NULL;
    }
    if (lock) {
        entry->status |= MEMLEASE_STATUS_LOCKED;
    } else if (entry->status & MEMLEASE_STATUS_LOCKED) {
        entry->status &= ~MEMLEASE_STATUS_LOCKED;
        memlease_check_timeout(entry);
    }
    if (out_size) {
        *out_size = entry->size;
    }
    k_mutex_unlock(&memlease_lock);
    return entry->buf;
}

// Free the leased memory buffer
// handle - Handle returned by memlease_alloc
uint32_t memlease_free(uint32_t handle) {
    memlease_entry_t *entry = NULL;
    memlease_entry_block_t *block = NULL;
    int32_t ret;

    if (k_is_in_isr()) {
        LOG_ERR("memlease_free called from ISR");
        return MEMLEASE_ERROR_CANNOT_CALL_FROM_ISR;
    }
    k_mutex_lock(&memlease_lock, K_FOREVER);
    ret = memlease_check_handle(handle, &entry, &block);
    if (ret != 0) {
        k_mutex_unlock(&memlease_lock);
        return ret;
    }
    if (entry->release_count > 0) {
        entry->release_count--;
        entry->status &= ~MEMLEASE_STATUS_LOCKED;
        memlease_check_timeout(entry);
    } else {
        // Free the entry
        k_free(entry->buf);
        memset(entry, 0, sizeof(memlease_entry_t));
        block->allocated_count--;
        memlease_data.recalculate_timeout = true;
        k_sem_give(&timeout_changed_sem);
    }
    k_mutex_unlock(&memlease_lock);
    return 0;
}

// Set or clear the lock status of the leased memory buffer
uint32_t memlease_set_lock(uint32_t handle, bool lock) {
    memlease_entry_t *entry = NULL;
    memlease_entry_block_t *block = NULL;
    int32_t ret;

    if (k_is_in_isr()) {
        LOG_ERR("memlease_set_lock called from ISR");
        return MEMLEASE_ERROR_CANNOT_CALL_FROM_ISR;
    }
    k_mutex_lock(&memlease_lock, K_FOREVER);
    ret = memlease_check_handle(handle, &entry, &block);
    if (ret != 0) {
        k_mutex_unlock(&memlease_lock);
        return ret;
    }
    if (lock) {
        entry->status |= MEMLEASE_STATUS_LOCKED;
    } else {
        entry->status &= ~MEMLEASE_STATUS_LOCKED;
        memlease_check_timeout(entry);
    }
    k_mutex_unlock(&memlease_lock);
    return 0;
}

// Set the timeout for the leased memory buffer
uint32_t memlease_set_timeout(uint32_t handle, int64_t timeout) {
    memlease_entry_t *entry = NULL;
    memlease_entry_block_t *block = NULL;
    int32_t ret;

    if (k_is_in_isr()) {
        LOG_ERR("memlease_set_timeout called from ISR");
        return MEMLEASE_ERROR_CANNOT_CALL_FROM_ISR;
    }
    k_mutex_lock(&memlease_lock, K_FOREVER);
    ret = memlease_check_handle(handle, &entry, &block);
    if (ret != 0) {
        k_mutex_unlock(&memlease_lock);
        return ret;
    }
    if (timeout > 0) {
        entry->expiry_time = k_uptime_get() + timeout;
        memlease_check_timeout(entry);
    } else {
        entry->expiry_time = 0; // Infinite
    }
    k_mutex_unlock(&memlease_lock);
    return 0;
}

// Set or clear the error on timeout flag for the leased memory buffer.
// If set, an error message is logged when the buffer times out.
uint32_t memlease_set_error_on_timeout(uint32_t handle, bool error_on_timeout) {
    memlease_entry_t *entry = NULL;
    memlease_entry_block_t *block = NULL;
    int32_t ret;

    if (k_is_in_isr()) {
        LOG_ERR("memlease_set_error_on_timeout called from ISR");
        return MEMLEASE_ERROR_CANNOT_CALL_FROM_ISR;
    }
    k_mutex_lock(&memlease_lock, K_FOREVER);
    ret = memlease_check_handle(handle, &entry, &block);
    if (ret != 0) {
        k_mutex_unlock(&memlease_lock);
        return ret;
    }
    if (error_on_timeout) {
        entry->status |= MEMLEASE_STATUS_ERROR_ON_TIMEOUT;
    } else {
        entry->status &= ~MEMLEASE_STATUS_ERROR_ON_TIMEOUT;
    }
    k_mutex_unlock(&memlease_lock);
    return 0;
}

// Set the release count for the leased memory buffer.
// Need to be one less than the number of times the buffer needs to be released before it is freed.
// For example, if the buffer needs to be released 3 times before it is freed, set release_count to 2.
uint32_t memlease_set_release_count(uint32_t handle, uint8_t release_count) {
    memlease_entry_t *entry = NULL;
    memlease_entry_block_t *block = NULL;
    int32_t ret;

    if (k_is_in_isr()) {
        LOG_ERR("memlease_set_release_count called from ISR");
        return MEMLEASE_ERROR_CANNOT_CALL_FROM_ISR;
    }
    k_mutex_lock(&memlease_lock, K_FOREVER);
    ret = memlease_check_handle(handle, &entry, &block);
    if (ret != 0) {
        k_mutex_unlock(&memlease_lock);
        return ret;
    }
    entry->release_count = release_count;
    k_mutex_unlock(&memlease_lock);
    return 0;
}

// Memlease timeout management thread
static void memlease_thread_fn(void) {
    k_timeout_t sleep_time = K_FOREVER;
    int64_t now;

    while(true) {
        k_sem_take(&timeout_changed_sem, sleep_time);
        now = k_uptime_get();
        k_mutex_lock(&memlease_lock, K_FOREVER);
        if ((k_uptime_get() > memlease_data.next_timeout) || memlease_data.recalculate_timeout) {
            // At least one of the entries has timed out or we need to recalculate the next timeout
            memlease_data.next_timeout = 0;
            memlease_data.recalculate_timeout = false;
            uint32_t i = 0;
            memlease_entry_block_t *block = &memlease_data.entries;
            bool not_last_entry = true;
            while (not_last_entry) {
                memlease_entry_t *entry = &block->entries[i];
                if ((entry->status & STATUS_ALLOCATED) && (entry->expiry_time > 0)){
                    // This entry has timed out
                    if ((!(entry->status & STATUS_LOCKED)) && (now >= entry->expiry_time)) {
                        // Free the entry
                        if (entry->status & STATUS_ERROR_ON_TIMEOUT) {
                            LOG_ERR("Memlease entry timed out (handle: 0x%08X)", (i + 1) | (entry->allocate_num << 16));
                        }
                        k_free(entry->buf);
                        memset(entry, 0, sizeof(memlease_entry_t));
                        if (block->allocated_count) {
                            block->allocated_count--;
                        }
                    } else if (!memlease_data.next_timeout || (entry->expiry_time < memlease_data.next_timeout)) {
                        memlease_data.next_timeout = entry->expiry_time;
                    }
                }
                i++;
                if (i >= CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF) {
                    if (!block->allocated_count) {
                        // Current block is empty, free it if it's not the first block
                        if (block != &memlease_data.entries) {
                            memlease_entry_block_t *prev_block = (memlease_entry_block_t *)block->prev;
                            prev_block->next = block->next;
                            if (block->next) {
                                ((memlease_entry_block_t *)block->next)->prev = prev_block;
                            }
                            k_free(block);
                            memlease_data.num_entries_allocated -= CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF;
                            block = prev_block;
                        }
                    }
                    if (block->next) {
                        block = (memlease_entry_block_t *)block->next;
                        i = 0;
                    } else {
                        not_last_entry = false;
                    }
                }
            }
        }
        if (!memlease_data.next_timeout) {
            // No timeouts scheduled
            sleep_time = K_FOREVER;
        } else {
            sleep_time = K_MSEC(memlease_data.next_timeout - now);
        }
        k_mutex_unlock(&memlease_lock);
    }
}

K_THREAD_DEFINE(memlease_thread, CONFIG_MEMLEASE_THREAD_STACK_SIZE, memlease_thread_fn, NULL, NULL, NULL, CONFIG_MEMLEASE_THREAD_PRIORITY, 0, 0);
