/*
;    Project:       Open Vehicle Monitor System
;    Date:          14th March 2017
;
;    Changes:
;    1.0  Initial release
;
;    (C) 2011       Michael Stegen / Stegen Electronics
;    (C) 2011-2017  Mark Webb-Johnson
;    (C) 2011        Sonny Chen @ EPRO/DX
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#include <strings.h>
#include "ovms_malloc.h"
#include "esp_heap_caps.h"

void* ExternalRamMalloc(size_t sz)
  {
  void* ret = heap_caps_malloc(sz, MALLOC_CAP_SPIRAM);
  if (ret)
    return ret;
  if (sz > 0)
    OvmsDiagRecordAllocationFailure(
      OVMS_DIAG_ALLOC_EXTERNAL_MALLOC_SPIRAM, sz);

  ret = malloc(sz);
  if (!ret && sz > 0)
    OvmsDiagRecordAllocationFailure(
      OVMS_DIAG_ALLOC_EXTERNAL_MALLOC_FALLBACK, sz);
  return ret;
  }

void* ExternalRamCalloc(size_t count, size_t size)
  {
  size_t requested = count * size;
  void* ret = heap_caps_malloc(requested, MALLOC_CAP_SPIRAM);
  if (ret)
    {
    bzero(ret, requested);
    return ret;
    }
  if (requested > 0)
    OvmsDiagRecordAllocationFailure(
      OVMS_DIAG_ALLOC_EXTERNAL_CALLOC_SPIRAM, requested);

  ret = calloc(count, size);
  if (!ret && requested > 0)
    OvmsDiagRecordAllocationFailure(
      OVMS_DIAG_ALLOC_EXTERNAL_CALLOC_FALLBACK, requested);
  return ret;
  }

void* ExternalRamRealloc(void *ptr, size_t size)
  {
  if (!ptr)
    return ExternalRamMalloc(size);
  if (size == 0)
    {
    heap_caps_free(ptr);
    return NULL;
    }
  void* ret = heap_caps_realloc(ptr, size, MALLOC_CAP_SPIRAM);
  if (ret)
    return ret;
  OvmsDiagRecordAllocationFailure(
    OVMS_DIAG_ALLOC_EXTERNAL_REALLOC_SPIRAM, size);

  ret = realloc(ptr, size);
  if (!ret)
    OvmsDiagRecordAllocationFailure(
      OVMS_DIAG_ALLOC_EXTERNAL_REALLOC_FALLBACK, size);
  return ret;
  }

void* OvmsMongooseMalloc(size_t sz)
  {
  void* ret = ExternalRamMalloc(sz);
  if (!ret && sz > 0)
    OvmsDiagRecordAllocationFailure(OVMS_DIAG_ALLOC_MONGOOSE_MALLOC, sz);
  return ret;
  }

void* OvmsMongooseCalloc(size_t count, size_t size)
  {
  size_t requested = count * size;
  void* ret = ExternalRamCalloc(count, size);
  if (!ret && requested > 0)
    OvmsDiagRecordAllocationFailure(OVMS_DIAG_ALLOC_MONGOOSE_CALLOC,
      requested);
  return ret;
  }

void* OvmsMongooseRealloc(void *ptr, size_t size)
  {
  void* ret = ExternalRamRealloc(ptr, size);
  if (!ret && size > 0)
    OvmsDiagRecordAllocationFailure(OVMS_DIAG_ALLOC_MONGOOSE_REALLOC, size);
  return ret;
  }

void* InternalRamMalloc(size_t sz)
  {
  void* ret = heap_caps_malloc(sz, MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
  if (ret)
    return ret;
  else
    return malloc(sz);
  }

void* InternalRamCalloc(size_t count, size_t size)
  {
  void* ret = heap_caps_malloc(count*size, MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
  if (ret)
    {
    bzero(ret, count*size);
    return ret;
    }
  else
    return calloc(count, size);
  }

void* InternalRamRealloc(void *ptr, size_t size)
  {
  if (!ptr)
    return InternalRamMalloc(size);
  if (size == 0)
    {
    heap_caps_free(ptr);
    return NULL;
    }
  void* ret = heap_caps_realloc(ptr, size, MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
  if (ret)
    return ret;
  else
    return realloc(ptr, size);
  }
