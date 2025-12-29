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

int32_t memlease_alloc(uint32_t size, void **buf_out, int64_t timeout);
void *memlease_get_buf(uint32_t handle, uint32_t *out_size, bool lock);
int32_t memlease_free(uint32_t handle);
int32_t memlease_set_lock(uint32_t handle, bool lock);
int32_t memlease_set_timeout(uint32_t handle, int64_t timeout);
int32_t memlease_set_error_on_timeout(uint32_t handle, bool error_on_timeout);
int32_t memlease_set_release_count(uint32_t handle, uint8_t release_count);

#endif // MEMLEASE_H