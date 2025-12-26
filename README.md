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