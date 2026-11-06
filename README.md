# Memlease

## Introduction
It is easy to create a memory leak in C. Sometimes this happens accidentally by simply forgetting to free a buffer. In other cases, memory is passed through a queue, event system, or asynchronous workflow, and something goes wrong along the way. If the buffer never reaches its destination, there is often no clear place left in the code to safely free it — resulting in a memory leak that is difficult to detect and recover from.

memlease addresses this problem by introducing lease-based memory management. Instead of relying solely on perfect control flow and strict ownership, memory can be allocated with a defined lifetime. If a buffer is not explicitly released or renewed within that time, it is automatically reclaimed.

This approach makes memory management more resilient in event-driven, message-based, and fault-prone systems, where failure paths are hard to predict and traditional manual cleanup is fragile.

## Use cases
### 1. Shared buffer ownership (reference-style freeing)

memlease allows you to specify how many times a buffer must be released before it is actually freed. This is useful when the same buffer needs to be sent to multiple consumers.

Without memlease, the buffer typically has to be passed sequentially: one consumer processes it and then forwards it to the next, which becomes responsible for freeing it. This creates unnecessary coupling between components.

With memlease, the same buffer can be dispatched to multiple queues or event handlers independently. Each consumer simply calls the release function when it is done, and the library ensures the buffer is freed only when the final release occurs.

### 2. Timeout-based cleanup for optional or unreliable consumers

The timeout feature is optional and does not need to be used when you are certain a buffer will always be released. However, it is particularly useful in cases where memory may never be explicitly freed.

A common example is log buffering in BLE-based systems. After a device reset, it may take some time before a BLE connection is re-established, causing early startup log messages to be missed. To avoid this, you can either:

- Allocate a large static log buffer (wasting RAM), or
- Dynamically allocate log buffers and risk leaking memory if the logs are never requested.

Using memlease, log buffers can be dynamically allocated with a timeout. If the logs are not retrieved within the specified time, the memory is automatically freed, preventing long-term memory waste while still allowing logs to be captured when needed.

## Config

CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF

Specifies the number of allocation tracking entries stored in a single memlease entry block.

The first entry block is statically allocated at build time. Each entry in the block tracks one active allocation. When this initial block becomes full, additional entry blocks are dynamically allocated on the heap as needed.

As allocations are released, entry blocks are reclaimed automatically. When all entries in a dynamically allocated block are free and it is the most recently allocated block, that block is freed.

### Trade-offs
Larger values:
- Fewer dynamic allocations of entry blocks
- Increased static memory usage (the initial block can never be freed)

Smaller values:
- Reduced static memory footprint
- More frequent allocation and deallocation of entry blocks at runtime

The default value of 32 provides a balanced trade-off for most systems. With this setting, up to 32 active heap allocations can be tracked before an additional entry block of 32 entries is allocated.


### Other options

| Option | Default | Description |
|---|---|---|
| `CONFIG_MEMLEASE_THREAD_STACK_SIZE` | 1024 | Stack of the memlease thread. Expiry callbacks run on this thread, so increase it if your callbacks need more. |
| `CONFIG_MEMLEASE_THREAD_PRIORITY` | 7 | Priority of the memlease thread. |
| `CONFIG_MEMLEASE_DEBUG_INFO` | n | Record the file, line and time of every allocation (16 bytes per entry). Shown by `memlease_foreach()`, `memlease_dump()` and the shell. |
| `CONFIG_MEMLEASE_SHELL` | n | Adds the `memlease list` and `memlease stats` shell commands. Needs `CONFIG_SHELL`. |
| `CONFIG_MEMLEASE_LOG_LEVEL` | default | Log level of the module. |

## Usage

Add the library to a Zephyr / nRF Connect SDK application (`add_subdirectory()` the repo from your app's `CMakeLists.txt` and `rsource "path/to/Kconfig.memlease"` from your app's `Kconfig`). Buffers come from the system heap, so `CONFIG_HEAP_MEM_POOL_SIZE` must be greater than 0.

Buffers are referenced by a `uint32_t` handle instead of a pointer. The handle includes a per-allocation sequence number, so a stale handle (for example one whose buffer already timed out) is detected and rejected instead of touching memory that may have been reused.

### API

| Function | Description |
|---|---|
| `memlease_alloc(size, &buf, timeout_ms)` | Allocate a buffer. Returns a handle, or 0 on failure. `timeout_ms <= 0` means no timeout. |
| `memlease_get_buf(handle, &size, lock)` | Get the buffer pointer, or NULL if the handle is no longer valid. `lock = true` adds a lock so the buffer can't time out while you use it. |
| `memlease_free(handle)` | Release the buffer. It is freed once it has been released `release_count + 1` times. Each release also removes one lock. |
| `memlease_set_release_count(handle, n)` | The buffer needs `n + 1` releases before it is freed (for example 2 for three consumers). |
| `memlease_retain(handle)` | Add one required release at runtime, like a reference count increment. |
| `memlease_set_lock(handle, lock)` | Add or remove a lock. Locks are counted, so every consumer can hold its own. An expired buffer is reclaimed shortly after its last lock is removed. |
| `memlease_set_timeout(handle, timeout_ms)` | Renew or change the lease, counting from now. |
| `memlease_set_expiry_cb(handle, cb, user_data)` | Call `cb(handle, buf, size, user_data)` from the memlease thread when the lease expires, just before the buffer is freed. Not called on `memlease_free()`. |
| `memlease_set_error_on_timeout(handle, en)` | Log an error when the buffer times out (enabled by default). |
| `memlease_realloc(handle, new_size, &buf)` | Resize the buffer, keeping the content and the handle. The buffer may move. On failure the original is kept. |
| `memlease_get_num_allocated()` | Number of buffers currently allocated. |
| `memlease_get_stats(&stats)` | Current, peak and total counters: leases, bytes, timeouts, allocation failures and entry blocks. |
| `memlease_foreach(cb, user_data)` | Call `cb` with a snapshot of every active lease. |
| `memlease_dump()` | Log the statistics and every active lease. |

Functions that take a handle return 0 on success or a negative `MEMLEASE_ERROR_*` code.

### Calling from an ISR

Every function except `memlease_foreach()` and `memlease_dump()` can be called from an ISR, including `memlease_alloc()`, `memlease_free()` and `memlease_realloc()`. The bookkeeping is protected by a spinlock rather than a mutex, and all heap calls happen outside it (Zephyr's `k_malloc()`/`k_free()` are ISR-safe), so interrupts are only blocked while entries are searched or updated. Searching scales with the number of entry blocks, so keep `CONFIG_MEMLEASE_NUM_ENTRIES_PER_BUF` large enough that you rarely have more than a few blocks.

Expiry callbacks always run on the memlease thread, never in an ISR.

### Example

```c
// Send one buffer to three consumers, and reclaim it after 5 s if any of them never releases it
void *buf;
uint32_t handle = memlease_alloc(128, &buf, 5000);
memlease_set_release_count(handle, 2);
k_msgq_put(&consumer_a_q, &handle, K_NO_WAIT);
k_msgq_put(&consumer_b_q, &handle, K_NO_WAIT);
k_msgq_put(&consumer_c_q, &handle, K_NO_WAIT);

// In each consumer
uint32_t size;
uint8_t *data = memlease_get_buf(handle, &size, true);   // lock while in use
if (data) {
    process(data, size);
    memlease_free(handle);                              // release and remove this consumer's lock
}
```

```c
// Keep startup logs for 60 s in case a BLE client connects, and count what was dropped
static void log_expired(uint32_t handle, void *buf, uint32_t size, void *user_data)
{
    atomic_add((atomic_t *)user_data, size);
}

uint32_t handle = memlease_alloc(len, &buf, 60000);
memlease_set_error_on_timeout(handle, false);
memlease_set_expiry_cb(handle, log_expired, &dropped_log_bytes);
```

### Finding leaks

Enable `CONFIG_MEMLEASE_DEBUG_INFO` and `CONFIG_MEMLEASE_SHELL`, then:

```
uart:~$ memlease list
Handle         Size  Expires ms Locks Releases  Allocated at
0x00070001      128        4211     1        2  src/ble_log.c:88
0x00090002       64         inf     0        1  src/sensor.c:41
2 leases
uart:~$ memlease stats
```

## Tests

`tests/memlease_test` is a ztest suite with three twister scenarios:
- `memlease.test`: 4 entries per block (so dynamic blocks get exercised) and debug info enabled.
- `memlease.test.default_config`: the default of 32 entries per block, without debug info.
- `memlease.test.shell`: a build-only check of the shell commands.

Run them on an nRF52840 DK with `tests/twister_ncs_3.1.0.bat <COM port>`.
