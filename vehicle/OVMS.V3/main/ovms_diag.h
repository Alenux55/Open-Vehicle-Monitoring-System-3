/*
;    Project:       Open Vehicle Monitor System
;    Date:          23rd August 2026
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

#ifndef __OVMS_DIAG_H__
#define __OVMS_DIAG_H__

#include <stddef.h>

// Fixed numeric source identifiers keep allocation-failure recording usable
// from both C and C++ without allocating diagnostic strings.
typedef enum
  {
  OVMS_DIAG_ALLOC_NONE = 0,
  OVMS_DIAG_ALLOC_EXTERNAL_MALLOC_SPIRAM = 1,
  OVMS_DIAG_ALLOC_EXTERNAL_MALLOC_FALLBACK = 2,
  OVMS_DIAG_ALLOC_EXTERNAL_CALLOC_SPIRAM = 3,
  OVMS_DIAG_ALLOC_EXTERNAL_CALLOC_FALLBACK = 4,
  OVMS_DIAG_ALLOC_EXTERNAL_REALLOC_SPIRAM = 5,
  OVMS_DIAG_ALLOC_EXTERNAL_REALLOC_FALLBACK = 6,
  OVMS_DIAG_ALLOC_MONGOOSE_MALLOC = 7,
  OVMS_DIAG_ALLOC_MONGOOSE_CALLOC = 8,
  OVMS_DIAG_ALLOC_MONGOOSE_REALLOC = 9,
  OVMS_DIAG_ALLOC_MONGOOSE_SEND = 10,
  OVMS_DIAG_ALLOC_SSH_CONSOLE = 11,
  OVMS_DIAG_ALLOC_SSH_EVENT_QUEUE = 12,
  OVMS_DIAG_ALLOC_SSH_CONTEXT = 13,
  OVMS_DIAG_ALLOC_SSH_SESSION = 14,
  OVMS_DIAG_ALLOC_WOLFSSL_MALLOC = 15,
  OVMS_DIAG_ALLOC_WOLFSSL_REALLOC = 16,
  OVMS_DIAG_ALLOC_VFS_OWNER_BATCH = 17,
  } ovms_diag_alloc_source_t;

#ifdef __cplusplus
extern "C" {
#endif

void OvmsDiagRecordAllocationFailure(ovms_diag_alloc_source_t source,
  size_t requested);

#ifdef __cplusplus
}
#endif

#endif //#ifndef __OVMS_DIAG_H__
