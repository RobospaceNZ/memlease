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

#define MEMLEASE_STATUS_ALLOCATED               (1<<0)  // This entry is allocated
#define MEMLEASE_STATUS_LOCKED                  (1<<1)  // Don't free, even after timeout. A thread can lock it when it is busy with the buffer.
#define MEMLEASE_STATUS_ERROR_ON_TIMEOUT        (1<<2)  // When timeout occurs, send and error log message.

#define MEMLEASE_ERROR_ENTRY_OUT_OF_BOUNDS      -1
#define MEMLEASE_ERROR_ALLOCATION_NUM_MISMATCH  -2
#define MEMLEASE_ERROR_CORRUPT_BLOCK_POINTER    -3
#define MEMLEASE_ERROR_NOT_ALLOCATED            -4

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
    uint32_t allocated_count;
} memlease_entry_block_t;

typedef struct {
    memlease_entry_block_t entries;
    uint32_t num_entries_allocated;
    uint16_t allocate_count;        // Counter to assign to allocated entries
} memlease_data_t;

static memlease_data_t memlease_data = {
    .num_entries_allocated = CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF,
};
static K_MUTEX_DEFINE(memlease_lock);

static uint32_t memlease_init_entry(memlease_entry_t *entry, uint32_t size, int64_t timeout) {
    uint32_t handle = 0;

    entry->buf = k_malloc(size);
    if (!entry->buf) {
        return 0;
    }
    entry->size = size;
    entry->status = STATUS_ALLOCATED | STATUS_ERROR_ON_TIMEOUT;
    if (timeout > 0) {
        entry->expiry_time = k_uptime_get() + timeout;
    } else {
        entry->expiry_time = 0; // Infinite
    }
    entry->release_count = 0;   // Default release count
    entry->allocate_num = memlease_data.allocate_count++;
    memlease_data.allocate_count &= 0xFFFF; // Wrap around
    handle = i + 1; // Handles are 1-based
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
                handle = memlease_init_entry(entry, size, timeout);
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
        memlease_data.num_entries_allocated += CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF;
        handle = memlease_init_entry(&new_block->entries[0], size, timeout);
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
