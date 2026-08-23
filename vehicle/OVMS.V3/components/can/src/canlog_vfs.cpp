/*
;    Project:       Open Vehicle Monitor System
;    Module:        CAN logging framework
;    Date:          18th January 2018
;
;    (C) 2018       Michael Balzer
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

#include "ovms_log.h"
static const char *TAG = "canlog-vfs";

#include "can.h"
#include "canformat.h"
#include "canlog_vfs.h"
#include "ovms_utils.h"
#include "ovms_config.h"
#include "ovms_boot.h"
#include "ovms_peripherals.h"
#include "ovms_vfs.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "soc/soc_memory_layout.h"
#include <algorithm>
#include <errno.h>
#include <inttypes.h>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static const char *CAN_PARAM = "can";
static const char *CANLOG_VFS_PRIMARY_SIZE_CONFIG = "log.vfs.primarysize";
static const char *CANLOG_VFS_OVERFLOW_SIZE_CONFIG = "log.vfs.overflowsize";
static const char *CANLOG_VFS_BATCH_SIZE_CONFIG = "log.vfs.batchsize";
static const UBaseType_t CANLOG_VFS_TASK_PRIORITY = 5;
static const size_t CANLOG_VFS_BATCH_SIZE = 8192;
static const size_t CANLOG_VFS_BATCH_SIZE_MIN = 4096;
static const size_t CANLOG_VFS_BATCH_SIZE_MAX = 65536;
static const size_t CANLOG_VFS_BATCH_SIZE_ALIGNMENT = 4096;
static const uint32_t CANLOG_VFS_SLOW_WRITE_THRESHOLD_US = 5000;
static const size_t CANLOG_VFS_PRIMARY_SIZE_DEFAULT = 100;
static const size_t CANLOG_VFS_PRIMARY_SIZE_MIN = 32;
static const size_t CANLOG_VFS_PRIMARY_SIZE_MAX = 128;
static const size_t CANLOG_VFS_OVERFLOW_SIZE_DEFAULT = 1280;
static const size_t CANLOG_VFS_OVERFLOW_SIZE_MIN = 128;
static const size_t CANLOG_VFS_OVERFLOW_SIZE_MAX = 8192;
static const size_t CANLOG_VFS_OVERFLOW_SCRATCH_SIZE = 8;

static uint32_t VfsDiagNowMs()
  {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000);
  }

static uint32_t VfsDiagToUint32(uint64_t value)
  {
  return value > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(value);
  }

static void VfsDiagUpdateMax(uint32_t* field, uint32_t value)
  {
  uint32_t current = OvmsDiagLoad(field);
  while (value > current
      && !__atomic_compare_exchange_n(field, &current, value, true,
        __ATOMIC_RELAXED, __ATOMIC_RELAXED)) {}
  }

static void VfsDiagCaptureLifecycle(ovms_diag_vfs_lifecycle_stage_t stage)
  {
  if (stage >= OVMS_DIAG_VFS_LIFECYCLE_COUNT)
    return;

  ovms_diag_vfs_heap_stage_t& snapshot = ovms_diag_live.vfs_lifecycle[stage];
  const uint32_t caps = MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
  OvmsDiagStore(&snapshot.monotonic_ms, VfsDiagNowMs());
  OvmsDiagStore(&snapshot.dma_free,
    static_cast<uint32_t>(heap_caps_get_free_size(caps)));
  OvmsDiagStore(&snapshot.dma_largest,
    static_cast<uint32_t>(heap_caps_get_largest_free_block(caps)));
  __atomic_store_n(&snapshot.sequence,
    OvmsDiagLoad(&ovms_diag_live.vfs_lifecycle_sequence), __ATOMIC_RELEASE);
  OvmsDiagStore(&ovms_diag_live.vfs_lifecycle_stage,
    static_cast<uint32_t>(stage));
  }

static uint32_t VfsDiagBegin(ovms_diag_vfs_op_t operation)
  {
  uint32_t parent = OvmsDiagLoad(&ovms_diag_live.vfs_op);
  OvmsDiagStore(&ovms_diag_live.vfs_parent_op, parent);
  OvmsDiagIncrement(&ovms_diag_live.vfs_op_sequence);
  OvmsDiagStore(&ovms_diag_live.vfs_op_started_ms, VfsDiagNowMs());
  __atomic_store_n(&ovms_diag_live.vfs_op, static_cast<uint32_t>(operation),
    __ATOMIC_RELEASE);
  return parent;
  }

static void VfsDiagEnd(uint32_t parent)
  {
  OvmsDiagStore(&ovms_diag_live.vfs_op_completed_ms, VfsDiagNowMs());
  __atomic_store_n(&ovms_diag_live.vfs_op, parent, __ATOMIC_RELEASE);
  OvmsDiagStore(&ovms_diag_live.vfs_parent_op, OVMS_DIAG_VFS_IDLE);
  }

static void VfsDiagSampleHeap()
  {
  uint32_t now = VfsDiagNowMs();
  uint32_t last = OvmsDiagLoad(&ovms_diag_live.heap_sample_ms);
  if (now - last < 1000)
    return;

  OvmsDiagStore(&ovms_diag_live.heap_internal_free,
    static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
  OvmsDiagStore(&ovms_diag_live.heap_internal_largest,
    static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
  OvmsDiagStore(&ovms_diag_live.heap_internal_minimum,
    static_cast<uint32_t>(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
  OvmsDiagStore(&ovms_diag_live.heap_dma_free,
    static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_8BIT)));
  OvmsDiagStore(&ovms_diag_live.heap_dma_largest,
    static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_8BIT)));
  OvmsDiagStore(&ovms_diag_live.heap_dma_minimum,
    static_cast<uint32_t>(heap_caps_get_minimum_free_size(MALLOC_CAP_DMA | MALLOC_CAP_8BIT)));
  OvmsDiagStore(&ovms_diag_live.heap_sample_ms, now);
  }

static void VfsDiagReset()
  {
  OvmsDiagStore(&ovms_diag_live.vfs_active, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_heartbeat, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_primary_enqueue, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_overflow_enqueue, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_overflow_drop, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_dequeue, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_format_complete, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_batch_start, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_batch_end, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_fwrite_start, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_fwrite_end, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_fflush_start, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_fflush_end, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_fsync_start, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_fsync_end, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_op, OVMS_DIAG_VFS_IDLE);
  OvmsDiagStore(&ovms_diag_live.vfs_parent_op, OVMS_DIAG_VFS_IDLE);
  OvmsDiagStore(&ovms_diag_live.vfs_op_sequence, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_op_started_ms, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_op_completed_ms, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_storage_error, 0);
  OvmsDiagStore(&ovms_diag_live.heap_sample_ms, 0);
  OvmsDiagStore(&ovms_diag_live.heap_internal_free, 0);
  OvmsDiagStore(&ovms_diag_live.heap_internal_largest, 0);
  OvmsDiagStore(&ovms_diag_live.heap_internal_minimum, 0);
  OvmsDiagStore(&ovms_diag_live.heap_dma_free, 0);
  OvmsDiagStore(&ovms_diag_live.heap_dma_largest, 0);
  OvmsDiagStore(&ovms_diag_live.heap_dma_minimum, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_queues_ready_free, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_queues_ready_largest, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_task_ready_free, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_task_ready_largest, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_owner_requested, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_owner_caps, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_owner_attempt_ms, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_owner_before_free, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_owner_before_largest, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_owner_after_free, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_owner_after_largest, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_owner_result, OVMS_DIAG_VFS_OWNER_NONE);
  OvmsDiagStore(&ovms_diag_live.vfs_lifecycle_stage,
    OVMS_DIAG_VFS_LIFECYCLE_OPEN_BASELINE);
  for (size_t i = 0; i < OVMS_DIAG_VFS_LIFECYCLE_COUNT; ++i)
    {
    OvmsDiagStore(&ovms_diag_live.vfs_lifecycle[i].sequence, 0);
    OvmsDiagStore(&ovms_diag_live.vfs_lifecycle[i].monotonic_ms, 0);
    OvmsDiagStore(&ovms_diag_live.vfs_lifecycle[i].dma_free, 0);
    OvmsDiagStore(&ovms_diag_live.vfs_lifecycle[i].dma_largest, 0);
    }
  OvmsDiagStore(&ovms_diag_live.vfs_slow_write_threshold_us,
    CANLOG_VFS_SLOW_WRITE_THRESHOLD_US);
  OvmsDiagStore(&ovms_diag_live.vfs_slow_write_sequence, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_slow_write_requested, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_slow_write_accepted, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_slow_write_file_offset, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_slow_write_cluster_offset, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_slow_write_batch_used, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_slow_write_batch_capacity, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_slow_write_elapsed_us, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_slow_write_sync_due_ms, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_slow_write_sync_state, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_slow_write_primary_queued, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_slow_write_overflow_queued, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_slow_write_error_state, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_slow_write_errno, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_slow_write_ferror, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_fflush_last_us, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_fflush_max_us, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_fflush_result, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_fsync_last_us, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_fsync_max_us, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_fsync_result, 0);
  VfsDiagSampleHeap();
  }

static_assert(sizeof(canlog_vfs_queue_msg_t) == 72,
  "VFS CAN log queue item size changed; review queue and overflow memory budgets");

static bool GetVfsQueueSizeConfig(const char* key, size_t default_value,
  size_t minimum, size_t maximum, size_t& result)
  {
  if (!MyConfig.IsDefined(CAN_PARAM, key))
    {
    result = default_value;
    return true;
    }

  std::string configured = MyConfig.GetParamValue(CAN_PARAM, key);
  if (configured.empty()
      || configured.find_first_not_of("0123456789") != std::string::npos)
    {
    ESP_LOGE(TAG,
      "Invalid %s/%s configured value '%s': expected decimal integer in range %u..%u",
      CAN_PARAM, key, configured.c_str(), (unsigned)minimum, (unsigned)maximum);
    return false;
    }

  errno = 0;
  char* end = NULL;
  unsigned long parsed = strtoul(configured.c_str(), &end, 10);
  if (errno == ERANGE || !end || *end != '\0'
      || parsed < minimum || parsed > maximum)
    {
    ESP_LOGE(TAG,
      "Invalid %s/%s configured value '%s': valid range is %u..%u records",
      CAN_PARAM, key, configured.c_str(), (unsigned)minimum, (unsigned)maximum);
    return false;
    }

  result = static_cast<size_t>(parsed);
  return true;
  }

static bool GetVfsBatchSizeConfig(size_t& result)
  {
  if (!MyConfig.IsDefined(CAN_PARAM, CANLOG_VFS_BATCH_SIZE_CONFIG))
    {
    result = 0;
    return true;
    }

  std::string configured =
    MyConfig.GetParamValue(CAN_PARAM, CANLOG_VFS_BATCH_SIZE_CONFIG);
  if (configured.empty()
      || configured.find_first_not_of("0123456789") != std::string::npos)
    {
    ESP_LOGE(TAG,
      "Invalid %s/%s configured value '%s': expected 0 (auto) or aligned bytes",
      CAN_PARAM, CANLOG_VFS_BATCH_SIZE_CONFIG, configured.c_str());
    return false;
    }

  errno = 0;
  char* end = NULL;
  unsigned long parsed = strtoul(configured.c_str(), &end, 10);
  if (errno == ERANGE || !end || *end != '\0'
      || (parsed != 0
        && (parsed < CANLOG_VFS_BATCH_SIZE_MIN
          || parsed > CANLOG_VFS_BATCH_SIZE_MAX
          || parsed % CANLOG_VFS_BATCH_SIZE_ALIGNMENT != 0)))
    {
    ESP_LOGE(TAG,
      "Invalid %s/%s configured value '%s': use 0 (auto) or %u..%u bytes aligned to %u",
      CAN_PARAM, CANLOG_VFS_BATCH_SIZE_CONFIG, configured.c_str(),
      (unsigned)CANLOG_VFS_BATCH_SIZE_MIN, (unsigned)CANLOG_VFS_BATCH_SIZE_MAX,
      (unsigned)CANLOG_VFS_BATCH_SIZE_ALIGNMENT);
    return false;
    }

  result = static_cast<size_t>(parsed);
  return true;
  }

void can_log_vfs_start(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  std::string format(cmd->GetName());
  canlog_vfs* logger = new canlog_vfs(argv[0],format);
  logger->Open();

  if (logger->IsOpen())
    {
    if (argc>1)
      { MyCan.AddLogger(logger, argc-1, &argv[1]); }
    else
      { MyCan.AddLogger(logger); }
    writer->printf("CAN logging to VFS active: %s\n", logger->GetInfo().c_str());
    MyCan.LogInfo(NULL, CAN_LogInfo_Config, logger->GetInfo().c_str());
    }
  else
    {
    writer->printf("Error: Could not start CAN logging to: %s\n", logger->GetInfo().c_str());
    delete logger;
    }
  }

class OvmsCanLogVFSInit
  {
  public: OvmsCanLogVFSInit();
} MyOvmsCanLogVFSInit  __attribute__ ((init_priority (4560)));

OvmsCanLogVFSInit::OvmsCanLogVFSInit()
  {
  ESP_LOGI(TAG, "Initialising CAN logging to VFS (4560)");

  OvmsCommand* cmd_can = MyCommandApp.FindCommand("can");
  if (cmd_can)
    {
    OvmsCommand* cmd_can_log = cmd_can->FindCommand("log");
    if (cmd_can_log)
      {
      OvmsCommand* cmd_can_log_start = cmd_can_log->FindCommand("start");
      if (cmd_can_log_start)
        {
        // We have a place to put our command tree..
        OvmsCommand* start = cmd_can_log_start->RegisterCommand("vfs", "CAN logging to VFS");
        MyCanFormatFactory.RegisterCommandSet(start, "Start CAN logging to VFS",
          can_log_vfs_start,
          "<path> [filter1] ... [filterN]\n"
          "Filter: <bus> | <id>[-<id>] | <bus>:<id>[-<id>]\n"
          "Example: 2:2a0-37f",
          1, _COMMAND_TOKEN_NMB, true, vfs_file_validate);
        }
      }
    }
  }


canlog_vfs_conn::canlog_vfs_conn(canlog* logger, std::string format, canformat::canformat_serve_mode_t mode)
  : canlogconnection(logger, format, mode), m_file(NULL), m_stdio_buffer_size(0),
    m_stdio_buffer_set(false), m_batch_buffer(NULL),
    m_batch_capacity(0), m_cluster_size(0), m_batch_limit(0), m_batch_used(0),
    m_format_buffer(NULL), m_format_buffer_size(0),
    m_write_blocked(false),
    m_write_errors(0), m_write_short(0), m_write_zero(0),
    m_write_first_recorded(false), m_write_first_errno(0), m_write_first_ferror(0),
    m_write_first_requested(0), m_write_first_returned(0),
    m_write_first_duration_ms(0),
    m_write_in_progress(false), m_write_in_progress_started_ms(0),
    m_write_current_requested(0), m_file_size(0),
    m_vfs_msgcount(0), m_vfs_dropcount(0), m_vfs_discardcount(0), m_vfs_filtercount(0),
    m_sync_count(0), m_sync_errors(0), m_sync_time(0), m_dirty(false),
    m_sync_in_progress(false)
  {
  }

canlog_vfs_conn::~canlog_vfs_conn()
  {
  if (m_file)
    {
    CloseFile();
    }
  if (m_batch_buffer)
    {
    heap_caps_free(m_batch_buffer);
    m_batch_buffer = NULL;
    }
  if (m_format_buffer)
    {
    heap_caps_free(m_format_buffer);
    m_format_buffer = NULL;
    }
  }

void canlog_vfs_conn::OutputMsg(CAN_log_message_t& msg, std::string &result)
  {
  m_vfs_msgcount++;

  if ((m_filters != NULL) && (! m_filters->IsFiltered(&msg.frame)))
    {
    m_vfs_filtercount++;
    return;
    }

  if (m_file && result.length()>0 && !AppendBytes(result.data(), result.length()))
    m_vfs_dropcount++;
  }

bool canlog_vfs_conn::GetFormatBuffer(size_t required, char*& buffer, bool& in_batch)
  {
  buffer = NULL;
  in_batch = false;
  if (required == 0 || !m_format_buffer || required > m_format_buffer_size)
    return false;

  size_t batch_used = m_batch_used.load();
  size_t batch_limit = m_batch_limit.load();
  if (!m_write_blocked && m_batch_buffer && batch_used <= batch_limit
      && required <= batch_limit - batch_used)
    {
    buffer = m_batch_buffer + batch_used;
    in_batch = true;
    }
  else
    {
    buffer = m_format_buffer;
    }
  return true;
  }

void canlog_vfs_conn::OutputMsgDirect(CAN_log_message_t& msg, char* data, size_t length, bool in_batch)
  {
  m_vfs_msgcount++;

  if ((m_filters != NULL) && (! m_filters->IsFiltered(&msg.frame)))
    {
    m_vfs_filtercount++;
    return;
    }

  if (!m_file || length == 0)
    return;

  if (in_batch)
    {
    size_t batch_used = m_batch_used.load();
    size_t batch_limit = m_batch_limit.load();
    if (m_write_blocked || !m_batch_buffer || batch_used > batch_limit
        || data != m_batch_buffer + batch_used
        || length > batch_limit - batch_used)
      {
      ESP_LOGE(TAG, "Invalid direct-format batch state for '%s'", m_peer.c_str());
      m_vfs_dropcount++;
      return;
      }

    batch_used += length;
    m_batch_used.store(batch_used);
    m_dirty = true;

    if (batch_used == batch_limit)
      FlushBatch();
    return;
    }

  if (!AppendBytes(data, length))
    m_vfs_dropcount++;
  }

bool canlog_vfs_conn::AppendBytes(const char* data, size_t record_length)
  {
  if (record_length == 0)
    return true;
  if (m_write_blocked || !m_batch_buffer || m_batch_capacity == 0)
    return false;

  size_t copied = 0;
  while (copied < record_length)
    {
    size_t batch_used = m_batch_used.load();
    size_t batch_limit = m_batch_limit.load();
    if (batch_limit == 0 || batch_limit > m_batch_capacity || batch_used > batch_limit)
      {
      ESP_LOGE(TAG, "Invalid cluster-aligned batch state for '%s'", m_peer.c_str());
      m_write_blocked = true;
      return false;
      }

    if (batch_used == batch_limit)
      {
      if (!FlushBatch())
        {
        if (copied > 0)
          m_write_blocked = true;
        return copied == record_length;
        }
      continue;
      }

    size_t chunk = std::min(record_length - copied, batch_limit - batch_used);
    memcpy(m_batch_buffer + batch_used, data + copied, chunk);
    batch_used += chunk;
    copied += chunk;
    m_batch_used.store(batch_used);
    m_dirty = true;

    if (batch_used == batch_limit && !FlushBatch())
      {
      if (copied < record_length)
        m_write_blocked = true;
      return copied == record_length;
      }
    }
  return true;
  }

bool canlog_vfs_conn::WriteFileBytes(const char* data, size_t length, size_t& accepted)
  {
  accepted = 0;
  if (length == 0)
    return true;
  if (!m_file || !data)
    return false;
  if (static_cast<canlog_vfs*>(m_logger)->HasStorageError())
    return false;

  while (accepted < length)
    {
    size_t requested = length - accepted;
    size_t file_offset = m_file_size.load(std::memory_order_relaxed);
    size_t cluster_offset = m_cluster_size > 0
      ? file_offset % m_cluster_size : 0;
    size_t batch_used = m_batch_used.load(std::memory_order_relaxed);
    size_t batch_capacity = m_batch_capacity;
    errno = 0;
    int64_t started = esp_timer_get_time();
    m_write_current_requested.store(requested, std::memory_order_relaxed);
    m_write_in_progress_started_ms.store(
      static_cast<uint32_t>(started / 1000), std::memory_order_relaxed);
    m_write_in_progress.store(true, std::memory_order_release);
    uint32_t diag_parent = VfsDiagBegin(OVMS_DIAG_VFS_FWRITE);
    OvmsDiagIncrement(&ovms_diag_live.vfs_fwrite_start);
    size_t written = fwrite(data + accepted, 1, requested, m_file);
    OvmsDiagIncrement(&ovms_diag_live.vfs_fwrite_end);
    VfsDiagEnd(diag_parent);
    int write_errno = errno;
    int write_ferror = ferror(m_file);
    uint64_t elapsed = esp_timer_get_time() - started;
    bool fatal = written < requested || write_errno != 0 || write_ferror != 0;
    if (written < requested)
      {
      m_write_short.fetch_add(1, std::memory_order_relaxed);
      if (written == 0)
        m_write_zero.fetch_add(1, std::memory_order_relaxed);
      }
    if (fatal)
      m_write_errors.fetch_add(1, std::memory_order_relaxed);
    m_write_in_progress.store(false, std::memory_order_release);
    static_cast<canlog_vfs*>(m_logger)->RecordWriteTiming(elapsed, written);

    if (elapsed >= CANLOG_VFS_SLOW_WRITE_THRESHOLD_US)
      {
      canlog_vfs* logger = static_cast<canlog_vfs*>(m_logger);
      uint32_t now_ms = VfsDiagNowMs();
      uint32_t deadline_ms = logger->m_sync_deadline_ms.load(
        std::memory_order_relaxed);
      int32_t until_due = static_cast<int32_t>(deadline_ms - now_ms);
      uint32_t sync_state = 0;
      if (m_dirty)
        sync_state |= OVMS_DIAG_VFS_SYNC_DIRTY;
      if (logger->m_syncperiod.load(std::memory_order_relaxed) > 0)
        sync_state |= OVMS_DIAG_VFS_SYNC_PERIODIC;
      if (deadline_ms != 0 && until_due <= 0)
        sync_state |= OVMS_DIAG_VFS_SYNC_DUE;
      if (m_sync_in_progress.load(std::memory_order_relaxed))
        sync_state |= OVMS_DIAG_VFS_SYNC_RUNNING;

      uint32_t error_state = 0;
      if (written < requested)
        error_state |= OVMS_DIAG_VFS_WRITE_SHORT;
      if (write_errno != 0)
        error_state |= OVMS_DIAG_VFS_WRITE_ERRNO;
      if (write_ferror != 0)
        error_state |= OVMS_DIAG_VFS_WRITE_FERROR;
      if (logger->HasStorageError())
        error_state |= OVMS_DIAG_VFS_WRITE_STORAGE;

      OvmsDiagStore(&ovms_diag_live.vfs_slow_write_requested,
        VfsDiagToUint32(requested));
      OvmsDiagStore(&ovms_diag_live.vfs_slow_write_accepted,
        VfsDiagToUint32(written));
      OvmsDiagStore(&ovms_diag_live.vfs_slow_write_file_offset,
        VfsDiagToUint32(file_offset));
      OvmsDiagStore(&ovms_diag_live.vfs_slow_write_cluster_offset,
        VfsDiagToUint32(cluster_offset));
      OvmsDiagStore(&ovms_diag_live.vfs_slow_write_batch_used,
        VfsDiagToUint32(batch_used));
      OvmsDiagStore(&ovms_diag_live.vfs_slow_write_batch_capacity,
        VfsDiagToUint32(batch_capacity));
      OvmsDiagStore(&ovms_diag_live.vfs_slow_write_elapsed_us,
        VfsDiagToUint32(elapsed));
      OvmsDiagStore(&ovms_diag_live.vfs_slow_write_sync_due_ms,
        deadline_ms != 0 && until_due > 0 ? static_cast<uint32_t>(until_due) : 0);
      OvmsDiagStore(&ovms_diag_live.vfs_slow_write_sync_state, sync_state);
      OvmsDiagStore(&ovms_diag_live.vfs_slow_write_primary_queued,
        logger->m_queue ? uxQueueMessagesWaiting(logger->m_queue) : 0);
      OvmsDiagStore(&ovms_diag_live.vfs_slow_write_overflow_queued,
        VfsDiagToUint32(logger->m_overflow_occupancy.load(
          std::memory_order_relaxed)));
      OvmsDiagStore(&ovms_diag_live.vfs_slow_write_error_state, error_state);
      OvmsDiagStore(&ovms_diag_live.vfs_slow_write_errno, write_errno);
      OvmsDiagStore(&ovms_diag_live.vfs_slow_write_ferror, write_ferror);
      __atomic_add_fetch(&ovms_diag_live.vfs_slow_write_sequence, 1,
        __ATOMIC_RELEASE);
      }

    if (written > 0)
      {
      accepted += written;
      m_file_size += written;
      }

    if (fatal)
      {
      bool expected = false;
      if (m_write_first_recorded.compare_exchange_strong(expected, true,
            std::memory_order_acq_rel))
        {
        m_write_first_requested.store(requested, std::memory_order_relaxed);
        m_write_first_returned.store(written, std::memory_order_relaxed);
        m_write_first_errno.store(write_errno, std::memory_order_relaxed);
        m_write_first_ferror.store(write_ferror, std::memory_order_relaxed);
        m_write_first_duration_ms.store(static_cast<uint32_t>(elapsed / 1000),
          std::memory_order_relaxed);
        static_cast<canlog_vfs*>(m_logger)->EnterStorageError(
          CANLOG_VFS_STORAGE_ERROR_WRITE);
        }
      ESP_LOGE(TAG,
        "Terminal CAN log storage error for '%s': requested=%u returned=%u ferror=%d errno=%d duration=%.3fms",
        m_peer.c_str(), (unsigned)requested, (unsigned)written,
        write_ferror, write_errno, (double)elapsed / 1000.0);
      return false;
      }
    }

  return true;
  }

void canlog_vfs_conn::UpdateBatchLimit()
  {
  size_t batch_limit = m_batch_capacity;
  if (m_cluster_size > 0)
    {
    size_t cluster_offset = m_file_size.load(std::memory_order_relaxed) % m_cluster_size;
    size_t cluster_remaining = cluster_offset == 0
      ? m_cluster_size : m_cluster_size - cluster_offset;
    batch_limit = std::min(m_batch_capacity, cluster_remaining);
    }
  m_batch_limit.store(batch_limit);
  }

bool canlog_vfs_conn::FlushBatch()
  {
  size_t batch_used = m_batch_used.load();
  if (batch_used == 0)
    return true;
  if (!m_file || !m_batch_buffer)
    return false;

  uint32_t diag_parent = VfsDiagBegin(OVMS_DIAG_VFS_BATCH_FLUSH);
  OvmsDiagIncrement(&ovms_diag_live.vfs_batch_start);
  size_t accepted = 0;
  if (WriteFileBytes(m_batch_buffer, batch_used, accepted))
    {
    m_batch_used.store(0);
    UpdateBatchLimit();
    OvmsDiagIncrement(&ovms_diag_live.vfs_batch_end);
    VfsDiagEnd(diag_parent);
    return true;
    }

  size_t remaining = batch_used - accepted;
  if (accepted > 0)
    memmove(m_batch_buffer, m_batch_buffer + accepted, remaining);
  m_batch_used.store(remaining);
  UpdateBatchLimit();
  m_dirty = true;
  ESP_LOGE(TAG, "Error flushing CAN log batch for '%s': retained %u unwritten bytes",
    m_peer.c_str(), (unsigned)remaining);
  OvmsDiagIncrement(&ovms_diag_live.vfs_batch_end);
  VfsDiagEnd(diag_parent);
  return false;
  }

bool canlog_vfs_conn::Sync(bool force)
  {
  if (!m_file || (!force && !m_dirty))
    return true;
  if (static_cast<canlog_vfs*>(m_logger)->HasStorageError())
    return false;

  m_sync_in_progress.store(true, std::memory_order_release);
  int64_t started = esp_timer_get_time();
  bool batch_result = FlushBatch();
  if (!batch_result)
    {
    int64_t elapsed = esp_timer_get_time() - started;
    {
    OvmsMutexLock lock(&m_stats_mutex);
    m_sync_time += elapsed;
    m_sync_count++;
    m_sync_errors++;
    }
    ESP_LOGE(TAG, "Error syncing CAN log '%s': batch=-1 pending=%u",
      m_peer.c_str(), (unsigned)m_batch_used.load());
    m_sync_in_progress.store(false, std::memory_order_release);
    return false;
    }
  uint32_t diag_parent = VfsDiagBegin(OVMS_DIAG_VFS_FFLUSH);
  OvmsDiagIncrement(&ovms_diag_live.vfs_fflush_start);
  int64_t fflush_started = esp_timer_get_time();
  int flush_result = fflush(m_file);
  uint32_t fflush_elapsed = VfsDiagToUint32(
    esp_timer_get_time() - fflush_started);
  OvmsDiagIncrement(&ovms_diag_live.vfs_fflush_end);
  OvmsDiagStore(&ovms_diag_live.vfs_fflush_last_us, fflush_elapsed);
  VfsDiagUpdateMax(&ovms_diag_live.vfs_fflush_max_us, fflush_elapsed);
  OvmsDiagStore(&ovms_diag_live.vfs_fflush_result, flush_result);
  VfsDiagEnd(diag_parent);

  diag_parent = VfsDiagBegin(OVMS_DIAG_VFS_FSYNC);
  OvmsDiagIncrement(&ovms_diag_live.vfs_fsync_start);
  int64_t fsync_started = esp_timer_get_time();
  int sync_result = fsync(fileno(m_file));
  uint32_t fsync_elapsed = VfsDiagToUint32(
    esp_timer_get_time() - fsync_started);
  OvmsDiagIncrement(&ovms_diag_live.vfs_fsync_end);
  OvmsDiagStore(&ovms_diag_live.vfs_fsync_last_us, fsync_elapsed);
  VfsDiagUpdateMax(&ovms_diag_live.vfs_fsync_max_us, fsync_elapsed);
  OvmsDiagStore(&ovms_diag_live.vfs_fsync_result, sync_result);
  VfsDiagEnd(diag_parent);
  int64_t elapsed = esp_timer_get_time() - started;

  {
  OvmsMutexLock lock(&m_stats_mutex);
  m_sync_time += elapsed;
  m_sync_count++;

  if (!batch_result || flush_result != 0 || sync_result != 0)
    {
    m_sync_errors++;
    }
  }

  if (!batch_result || flush_result != 0 || sync_result != 0)
    {
    ESP_LOGE(TAG, "Error syncing CAN log '%s': batch=%d pending=%u fflush=%d fsync=%d",
      m_peer.c_str(), batch_result ? 0 : -1, (unsigned)m_batch_used.load(),
      flush_result, sync_result);
    m_sync_in_progress.store(false, std::memory_order_release);
    return false;
    }

  m_dirty = false;
  m_sync_in_progress.store(false, std::memory_order_release);
  return true;
  }

bool canlog_vfs_conn::CloseFile()
  {
  if (!m_file)
    return true;

  if (static_cast<canlog_vfs*>(m_logger)->HasStorageError())
    return CloseFileStorageError();

  bool result = Sync(true);
  if (static_cast<canlog_vfs*>(m_logger)->HasStorageError())
    return CloseFileStorageError() && result;
  size_t pending_batch = m_batch_used.load();
  if (pending_batch > 0)
    {
    ESP_LOGE(TAG, "Closing CAN log '%s' with %u owner-batch bytes still unwritten",
      m_peer.c_str(), (unsigned)pending_batch);
    result = false;
    }
  if (fclose(m_file) != 0)
    {
    result = false;
    OvmsMutexLock lock(&m_stats_mutex);
    m_sync_errors++;
    }
  m_file = NULL;
  if (m_batch_buffer)
    {
    heap_caps_free(m_batch_buffer);
    m_batch_buffer = NULL;
    }
  if (m_format_buffer)
    {
    heap_caps_free(m_format_buffer);
    m_format_buffer = NULL;
    }
  m_stdio_buffer_size = 0;
  m_stdio_buffer_set = false;
  m_batch_capacity = 0;
  m_batch_used.store(0);
  m_format_buffer_size = 0;
  m_write_blocked = false;
  m_dirty = false;
  return result;
  }

bool canlog_vfs_conn::CloseFileStorageError()
  {
  if (!m_file)
    return true;

  size_t pending_batch = m_batch_used.load();
  if (pending_batch > 0)
    ESP_LOGE(TAG,
      "Discarding %u pending CAN log batch bytes after storage error for '%s'",
      (unsigned)pending_batch, m_peer.c_str());
  m_batch_used.store(0);
  m_write_blocked = true;
  m_dirty = false;

  // Do not retry the poisoned FIL through FlushBatch/fflush/fsync. FatFS f_close
  // still calls f_sync and may perform one final metadata operation, but fclose
  // is required to release the newlib stream, VFS descriptor and FatFS object.
  int close_result = fclose(m_file);
  m_file = NULL;
  if (close_result != 0)
    ESP_LOGE(TAG, "Error closing failed CAN log '%s' (result=%d errno=%d)",
      m_peer.c_str(), close_result, errno);
  return close_result == 0;
  }


canlog_vfs::canlog_vfs(std::string path, std::string format)
  : canlog("vfs", format, canformat::Discard, false), m_vfs_conn(NULL),
    m_syncperiod(0), m_batch_capacity_config(0), m_sync_deadline_ms(0),
    m_storage_error_reason(CANLOG_VFS_STORAGE_ERROR_NONE),
    m_accepting(false), m_producers(0),
    m_queue_size(0), m_primary_highwater(0),
    m_overflow_ring(NULL), m_overflow_scratch(NULL), m_overflow_size(0),
    m_overflow_head(0), m_overflow_tail(0), m_overflow_count(0),
    m_overflow_occupancy(0),
    m_overflow_highwater(0), m_spill_active(false), m_spill_started(0),
    m_active_timing(NULL)
  {
  m_path = path;
  LoadConfig();
  using std::placeholders::_1;
  using std::placeholders::_2;
  MyEvents.RegisterEvent(IDTAG, "sd.mounted", std::bind(&canlog_vfs::MountListener, this, _1, _2));
  MyEvents.RegisterEvent(IDTAG, "sd.unmounting", std::bind(&canlog_vfs::MountListener, this, _1, _2));
  }

canlog_vfs::~canlog_vfs()
  {
  MyEvents.DeregisterEvent(IDTAG);
  Close();
  VfsDiagCaptureLifecycle(OVMS_DIAG_VFS_LIFECYCLE_DESTRUCTION);
  }

void canlog_vfs::VfsTaskEntry(void* context)
  {
  static_cast<canlog_vfs*>(context)->VfsTask();
  }

void canlog_vfs::VfsTask()
  {
  int syncperiod = m_syncperiod.load();
  int64_t sync_deadline = 0;
  m_sync_deadline_ms.store(0, std::memory_order_relaxed);
  m_task_ready.Give();
  OvmsDiagStore(&ovms_diag_live.vfs_active, 1);
  OvmsDiagStore(&ovms_diag_live.vfs_op, OVMS_DIAG_VFS_IDLE);
  VfsDiagSampleHeap();

  for (;;)
    {
    canlog_vfs_queue_msg_t item;

    // Preserve FIFO ordering by exhausting the internal primary queue before
    // copying any records from the overflow ring.
    if (xQueueReceive(m_queue, &item, 0) == pdTRUE)
      {
      if (!ProcessQueueItem(item, syncperiod, sync_deadline))
        break;
      VfsDiagSampleHeap();
      if (HasStorageError())
        {
        HandleStorageError(NULL, 0);
        break;
        }
      CheckSyncDeadline(syncperiod, sync_deadline);
      if (HasStorageError())
        {
        HandleStorageError(NULL, 0);
        break;
        }
      continue;
      }

    size_t overflow_count = CopyOverflowBatch(
      m_overflow_scratch, CANLOG_VFS_OVERFLOW_SCRATCH_SIZE);
    if (overflow_count > 0)
      {
      bool running = true;
      for (size_t i = 0; i < overflow_count; ++i)
        {
        if (!ProcessQueueItem(m_overflow_scratch[i], syncperiod, sync_deadline))
          {
          running = false;
          break;
          }
        VfsDiagSampleHeap();
        if (HasStorageError())
          {
          HandleStorageError(m_overflow_scratch + i + 1, overflow_count - i - 1);
          running = false;
          break;
          }
        CheckSyncDeadline(syncperiod, sync_deadline);
        if (HasStorageError())
          {
          HandleStorageError(m_overflow_scratch + i + 1, overflow_count - i - 1);
          running = false;
          break;
          }
        }
      if (!running)
        break;
      continue;
      }

    TickType_t timeout = portMAX_DELAY;
    if (m_vfs_conn && m_vfs_conn->m_dirty && syncperiod > 0)
      {
      int64_t remaining = sync_deadline - esp_timer_get_time();
      if (remaining <= 0)
        timeout = 0;
      else
        {
        timeout = pdMS_TO_TICKS((remaining + 999) / 1000);
        if (timeout == 0)
          timeout = 1;
        }
      }

    if (xQueueReceive(m_queue, &item, timeout) == pdTRUE)
      {
      if (!ProcessQueueItem(item, syncperiod, sync_deadline))
        break;
      VfsDiagSampleHeap();
      if (HasStorageError())
        {
        HandleStorageError(NULL, 0);
        break;
        }
      }
    CheckSyncDeadline(syncperiod, sync_deadline);
    if (HasStorageError())
      {
      HandleStorageError(NULL, 0);
      break;
      }
    }

  OvmsDiagStore(&ovms_diag_live.vfs_active, 0);
  OvmsDiagStore(&ovms_diag_live.vfs_op, OVMS_DIAG_VFS_IDLE);
  m_sync_deadline_ms.store(0, std::memory_order_relaxed);
  vTaskDelete(NULL);
  }

bool canlog_vfs::ProcessQueueItem(canlog_vfs_queue_msg_t& item,
  int& syncperiod, int64_t& sync_deadline)
  {
  switch (item.command)
    {
    case CANLOG_VFS_DATA:
      {
      OvmsDiagIncrement(&ovms_diag_live.vfs_heartbeat);
      OvmsDiagIncrement(&ovms_diag_live.vfs_dequeue);
      int64_t data_started = esp_timer_get_time();
      canlog_vfs_data_timing_t timing;
      m_active_timing = &timing;
      CAN_log_message_t& msg = item.data.message;
      if (m_formatter && m_vfs_conn)
        {
        bool was_dirty = m_vfs_conn->m_dirty;
        size_t direct_size = m_formatter->getbuffersize();
        char* format_buffer = NULL;
        bool in_batch = false;
        if (direct_size > 0
            && m_vfs_conn->GetFormatBuffer(direct_size, format_buffer, in_batch))
          {
          uint32_t diag_parent = VfsDiagBegin(OVMS_DIAG_VFS_FORMAT);
          int64_t format_started = esp_timer_get_time();
          size_t result_length = m_formatter->get(&msg, format_buffer, direct_size);
          OvmsDiagIncrement(&ovms_diag_live.vfs_format_complete);
          VfsDiagEnd(diag_parent);
          uint64_t format_elapsed = esp_timer_get_time() - format_started;
          RecordFormatTiming(timing, format_elapsed, result_length);
          if (result_length > 0)
            m_vfs_conn->OutputMsgDirect(msg, format_buffer, result_length, in_batch);
          }
        else
          {
          uint32_t diag_parent = VfsDiagBegin(OVMS_DIAG_VFS_FORMAT);
          int64_t format_started = esp_timer_get_time();
          std::string result = m_formatter->get(&msg);
          OvmsDiagIncrement(&ovms_diag_live.vfs_format_complete);
          VfsDiagEnd(diag_parent);
          uint64_t format_elapsed = esp_timer_get_time() - format_started;
          RecordFormatTiming(timing, format_elapsed, result.length());
          if (result.length() > 0)
            m_vfs_conn->OutputMsg(msg, result);
          }

        if (!was_dirty && m_vfs_conn->m_dirty && syncperiod > 0)
          {
          sync_deadline = esp_timer_get_time() + (int64_t)syncperiod * 1000000;
          m_sync_deadline_ms.store(
            static_cast<uint32_t>(sync_deadline / 1000),
            std::memory_order_relaxed);
          }

        if (syncperiod == -1 && m_vfs_conn->m_dirty)
          {
          m_vfs_conn->Sync();
          sync_deadline = 0;
          m_sync_deadline_ms.store(0, std::memory_order_relaxed);
          }
        }
      else
        {
        m_dropcount++;
        }

      FreeQueueItem(item);
      m_active_timing = NULL;
      RecordDataTiming(data_started, timing);
      break;
      }

    case CANLOG_VFS_OPEN:
      *item.data.control.result = OpenFile();
      if (m_vfs_conn && m_vfs_conn->m_dirty && syncperiod > 0)
        {
        sync_deadline = esp_timer_get_time() + (int64_t)syncperiod * 1000000;
        m_sync_deadline_ms.store(static_cast<uint32_t>(sync_deadline / 1000),
          std::memory_order_relaxed);
        }
      item.data.control.ack->Give();
      break;

    case CANLOG_VFS_CONFIG:
      syncperiod = item.data.control.syncperiod;
      if (m_vfs_conn && m_vfs_conn->m_dirty)
        {
        if (syncperiod == -1)
          {
          m_vfs_conn->Sync();
          sync_deadline = 0;
          m_sync_deadline_ms.store(0, std::memory_order_relaxed);
          }
        else if (syncperiod > 0)
          {
          sync_deadline = esp_timer_get_time() + (int64_t)syncperiod * 1000000;
          m_sync_deadline_ms.store(static_cast<uint32_t>(sync_deadline / 1000),
            std::memory_order_relaxed);
          }
        else
          {
          sync_deadline = 0;
          m_sync_deadline_ms.store(0, std::memory_order_relaxed);
          }
        }
      break;

    case CANLOG_VFS_CLOSE:
      CloseFile();
      m_task = NULL;
      item.data.control.ack->Give();
      return false;
    }

  return true;
  }

void canlog_vfs::CheckSyncDeadline(int syncperiod, int64_t& sync_deadline)
  {
  if (m_vfs_conn && m_vfs_conn->m_dirty && syncperiod > 0
      && esp_timer_get_time() >= sync_deadline)
    {
    m_vfs_conn->Sync();
    sync_deadline = m_vfs_conn->m_dirty
      ? esp_timer_get_time() + (int64_t)syncperiod * 1000000
      : 0;
    m_sync_deadline_ms.store(sync_deadline > 0
      ? static_cast<uint32_t>(sync_deadline / 1000) : 0,
      std::memory_order_relaxed);
    }
  }

bool canlog_vfs::StartVfsTask(size_t primary_size, size_t overflow_size)
  {
  if (overflow_size > std::numeric_limits<size_t>::max()
      / sizeof(canlog_vfs_queue_msg_t))
    {
    ESP_LOGE(TAG,
      "Invalid %s/%s configured value %u: overflow allocation size would overflow",
      CAN_PARAM, CANLOG_VFS_OVERFLOW_SIZE_CONFIG, (unsigned)overflow_size);
    return false;
    }
  size_t overflow_bytes = overflow_size * sizeof(canlog_vfs_queue_msg_t);
  ESP_LOGI(TAG,
    "Starting VFS CAN log queues: primary=%u internal, overflow=%u SPIRAM (%u bytes)",
    (unsigned)primary_size, (unsigned)overflow_size, (unsigned)overflow_bytes);

  m_queue = xQueueCreate(primary_size, sizeof(canlog_vfs_queue_msg_t));
  if (!m_queue)
    {
    ESP_LOGE(TAG,
      "Unable to create VFS CAN log primary queue: requested %u records (%u-byte items)",
      (unsigned)primary_size, (unsigned)sizeof(canlog_vfs_queue_msg_t));
    DestroyVfsQueue();
    return false;
    }
  if (!esp_ptr_internal(m_queue))
    {
    ESP_LOGE(TAG,
      "VFS CAN log primary queue is not in internal RAM: requested %u records",
      (unsigned)primary_size);
    DestroyVfsQueue();
    return false;
    }

  m_overflow_ring = static_cast<canlog_vfs_queue_msg_t*>(heap_caps_malloc(
    overflow_bytes,
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!m_overflow_ring)
    {
    ESP_LOGE(TAG,
      "Unable to allocate VFS CAN log SPIRAM overflow: requested %u records, %u bytes",
      (unsigned)overflow_size, (unsigned)overflow_bytes);
    DestroyVfsQueue();
    return false;
    }
  if (!esp_ptr_external_ram(m_overflow_ring))
    {
    ESP_LOGE(TAG,
      "VFS CAN log overflow is not in SPIRAM: requested %u records, %u bytes",
      (unsigned)overflow_size, (unsigned)overflow_bytes);
    DestroyVfsQueue();
    return false;
    }

  m_overflow_scratch = static_cast<canlog_vfs_queue_msg_t*>(heap_caps_malloc(
    CANLOG_VFS_OVERFLOW_SCRATCH_SIZE * sizeof(canlog_vfs_queue_msg_t),
    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (!m_overflow_scratch)
    {
    ESP_LOGE(TAG,
      "Unable to allocate internal VFS CAN log overflow scratch: requested %u records, %u bytes",
      (unsigned)CANLOG_VFS_OVERFLOW_SCRATCH_SIZE,
      (unsigned)(CANLOG_VFS_OVERFLOW_SCRATCH_SIZE * sizeof(canlog_vfs_queue_msg_t)));
    DestroyVfsQueue();
    return false;
    }
  if (!esp_ptr_internal(m_overflow_scratch))
    {
    ESP_LOGE(TAG,
      "VFS CAN log overflow scratch is not in internal RAM: requested %u records, %u bytes",
      (unsigned)CANLOG_VFS_OVERFLOW_SCRATCH_SIZE,
      (unsigned)(CANLOG_VFS_OVERFLOW_SCRATCH_SIZE * sizeof(canlog_vfs_queue_msg_t)));
    DestroyVfsQueue();
    return false;
    }

  m_queue_size = primary_size;
  m_primary_highwater.store(0);
  m_overflow_size = overflow_size;
  m_overflow_head = 0;
  m_overflow_tail = 0;
  m_overflow_count = 0;
  m_overflow_occupancy.store(0, std::memory_order_relaxed);
  m_overflow_highwater = 0;
  m_spill_active = false;
  m_spill_started = 0;
  m_overflow_stats = canlog_vfs_overflow_stats_t();

  while (m_task_ready.Take(0)) {}
  while (m_terminal_stop_ack.Take(0)) {}
  const uint32_t stage_caps = MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
  OvmsDiagStore(&ovms_diag_live.vfs_queues_ready_free,
    static_cast<uint32_t>(heap_caps_get_free_size(stage_caps)));
  OvmsDiagStore(&ovms_diag_live.vfs_queues_ready_largest,
    static_cast<uint32_t>(heap_caps_get_largest_free_block(stage_caps)));
  VfsDiagCaptureLifecycle(OVMS_DIAG_VFS_LIFECYCLE_QUEUES_READY);
  BaseType_t result = xTaskCreatePinnedToCore(VfsTaskEntry, "OVMS CanLogVFS", 4096, (void*)this,
    CANLOG_VFS_TASK_PRIORITY, &m_task, CORE(1));
  if (result != pdPASS)
    {
    ESP_LOGE(TAG, "Unable to create VFS CAN log task, error code=%d", result);
    DestroyVfsQueue();
    m_task = NULL;
    return false;
    }

  if (!m_task_ready.Take(pdMS_TO_TICKS(5000)))
    {
    ESP_LOGE(TAG, "VFS CAN log task did not become ready within 5 seconds");
    vTaskDelete(m_task);
    m_task = NULL;
    DestroyVfsQueue();
    return false;
    }

  return true;
  }

void canlog_vfs::StopVfsTask()
  {
  if (!m_task || !m_queue)
    return;

  if (HasStorageError())
    {
    xTaskNotifyGive(m_task);
    m_terminal_stop_ack.Take();
    return;
    }

  OvmsSemaphore ack;
  canlog_vfs_queue_msg_t item = {};
  item.command = CANLOG_VFS_CLOSE;
  item.data.control.ack = &ack;
  if (QueueControl(item))
    ack.Take();
  else if (m_task && HasStorageError())
    {
    xTaskNotifyGive(m_task);
    m_terminal_stop_ack.Take();
    }
  }

bool canlog_vfs::OpenFile()
  {
  canlog_vfs_conn* conn = new canlog_vfs_conn(this, m_format, m_mode);
  conn->m_peer = m_path;

  size_t batch_size = CANLOG_VFS_BATCH_SIZE;
#ifdef CONFIG_OVMS_COMP_SDCARD
  if (startsWith(m_path, "/sd"))
    {
    conn->m_cluster_size = MyPeripherals && MyPeripherals->m_sdcard
      ? MyPeripherals->m_sdcard->GetFatClusterSize() : 0;
    if (conn->m_cluster_size == 0)
      {
      ESP_LOGE(TAG, "Error: Can't determine FAT cluster size for '%s'", m_path.c_str());
      delete conn;
      return false;
      }
    batch_size = conn->m_cluster_size;
    }
#endif // CONFIG_OVMS_COMP_SDCARD
  size_t configured_batch = m_batch_capacity_config.load(
    std::memory_order_relaxed);
  if (configured_batch != 0)
    batch_size = configured_batch;

  conn->m_file = fopen(m_path.c_str(), "w");
  if (!conn->m_file)
    {
    ESP_LOGE(TAG, "Error: Can't write to '%s'", m_path.c_str());
    delete conn;
    return false;
    }

  int setvbuf_result = setvbuf(conn->m_file, NULL, _IONBF, 0);
  if (setvbuf_result != 0)
    {
    ESP_LOGE(TAG, "Error: Can't set unbuffered stdio mode for '%s' (result=%d)",
      m_path.c_str(), setvbuf_result);
    fclose(conn->m_file);
    conn->m_file = NULL;
    delete conn;
    return false;
    }
  conn->m_stdio_buffer_size = 0;
  conn->m_stdio_buffer_set = true;
  VfsDiagCaptureLifecycle(OVMS_DIAG_VFS_LIFECYCLE_FOPEN_READY);

  const uint32_t owner_caps = MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
  OvmsDiagIncrement(&ovms_diag_live.vfs_owner_sequence);
  OvmsDiagStore(&ovms_diag_live.vfs_owner_requested,
    static_cast<uint32_t>(batch_size));
  OvmsDiagStore(&ovms_diag_live.vfs_owner_caps, owner_caps);
  OvmsDiagStore(&ovms_diag_live.vfs_owner_attempt_ms, VfsDiagNowMs());
  OvmsDiagStore(&ovms_diag_live.vfs_owner_before_free,
    static_cast<uint32_t>(heap_caps_get_free_size(owner_caps)));
  OvmsDiagStore(&ovms_diag_live.vfs_owner_before_largest,
    static_cast<uint32_t>(heap_caps_get_largest_free_block(owner_caps)));
  __atomic_store_n(&ovms_diag_live.vfs_owner_result,
    static_cast<uint32_t>(OVMS_DIAG_VFS_OWNER_ATTEMPTING), __ATOMIC_RELEASE);

  conn->m_batch_buffer = static_cast<char*>(heap_caps_malloc(batch_size,
    owner_caps));
  OvmsDiagStore(&ovms_diag_live.vfs_owner_after_free,
    static_cast<uint32_t>(heap_caps_get_free_size(owner_caps)));
  OvmsDiagStore(&ovms_diag_live.vfs_owner_after_largest,
    static_cast<uint32_t>(heap_caps_get_largest_free_block(owner_caps)));
  if (!conn->m_batch_buffer)
    {
    __atomic_store_n(&ovms_diag_live.vfs_owner_result,
      static_cast<uint32_t>(OVMS_DIAG_VFS_OWNER_FAILED), __ATOMIC_RELEASE);
    OvmsDiagRecordAllocationFailure(OVMS_DIAG_ALLOC_VFS_OWNER_BATCH,
      batch_size);
    ESP_LOGE(TAG, "Error: Can't allocate %u byte owner batch for '%s'",
      (unsigned)batch_size, m_path.c_str());
    fclose(conn->m_file);
    conn->m_file = NULL;
    delete conn;
    return false;
    }
  __atomic_store_n(&ovms_diag_live.vfs_owner_result,
    static_cast<uint32_t>(OVMS_DIAG_VFS_OWNER_SUCCESS), __ATOMIC_RELEASE);
  conn->m_batch_capacity = batch_size;
  conn->m_batch_limit.store(batch_size);
  conn->m_batch_used.store(0);
  VfsDiagCaptureLifecycle(OVMS_DIAG_VFS_LIFECYCLE_OWNER_READY);

  size_t direct_size = m_formatter->getbuffersize();
  if (direct_size > 0)
    {
    conn->m_format_buffer = static_cast<char*>(heap_caps_malloc(direct_size, MALLOC_CAP_8BIT));
    if (conn->m_format_buffer)
      conn->m_format_buffer_size = direct_size;
    else
      ESP_LOGW(TAG, "Can't allocate %u byte direct-format boundary buffer for '%s'; using string fallback",
        (unsigned)direct_size, m_path.c_str());
    }

  std::string header = m_formatter->getheader();
  if (header.length() > 0)
    {
    conn->m_dirty = true;
    size_t accepted = 0;
    if (!conn->WriteFileBytes(header.data(), header.length(), accepted))
      {
      conn->m_write_blocked = true;
      conn->m_vfs_dropcount++;
      conn->CloseFileStorageError();
      delete conn;
      return false;
      }
    }
  conn->UpdateBatchLimit();

  {
  OvmsRecMutexLock lock(&m_cmmutex);
  m_vfs_conn = conn;
  m_connmap[NULL] = conn;
  }
  return true;
  }

void canlog_vfs::CloseFile()
  {
  if (m_vfs_conn)
    m_vfs_conn->CloseFile();
  }

bool canlog_vfs::BeginProducer()
  {
  if (!m_accepting.load(std::memory_order_acquire))
    return false;

  m_producers.fetch_add(1, std::memory_order_acq_rel);
  if (!m_accepting.load(std::memory_order_acquire))
    {
    m_producers.fetch_sub(1, std::memory_order_acq_rel);
    return false;
    }
  return true;
  }

void canlog_vfs::EndProducer()
  {
  m_producers.fetch_sub(1, std::memory_order_acq_rel);
  }

bool canlog_vfs::QueueMessage(CAN_log_message_t& msg, bool has_text)
  {
  canlog_vfs_queue_msg_t item = {};
  item.command = CANLOG_VFS_DATA;
  item.data.message = msg;
  if (!RouteQueueItem(item, true))
    {
    if (has_text)
      free(msg.text);
    return false;
    }
  return true;
  }

bool canlog_vfs::QueueControl(canlog_vfs_queue_msg_t& item)
  {
  while (m_queue && m_task)
    {
    if (HasStorageError())
      return false;
    if (RouteQueueItem(item, false))
      return true;
    vTaskDelay(1);
    }
  return false;
  }

void canlog_vfs::BeginSpillLocked()
  {
  if (m_spill_active)
    return;

  m_spill_active = true;
  m_spill_started = esp_timer_get_time();
  m_overflow_stats.transitions++;
  }

bool canlog_vfs::RouteQueueItem(const canlog_vfs_queue_msg_t& item, bool is_data)
  {
  OvmsMutexLock route(&m_route_mutex);
  if (HasStorageError())
    {
    if (is_data)
      m_dropcount++;
    return false;
    }
  if (!m_queue || !m_overflow_ring)
    {
    if (is_data)
      m_dropcount++;
    return false;
    }

  if (!m_spill_active)
    {
    if (xQueueSend(m_queue, &item, 0) == pdTRUE)
      {
      if (is_data)
        OvmsDiagIncrement(&ovms_diag_live.vfs_primary_enqueue);
      uint32_t waiting = uxQueueMessagesWaiting(m_queue);
      uint32_t highwater = m_primary_highwater.load(std::memory_order_relaxed);
      while (waiting > highwater
          && !m_primary_highwater.compare_exchange_weak(highwater, waiting,
            std::memory_order_relaxed)) {}
      return true;
      }
    BeginSpillLocked();
    }

  if (m_overflow_count == m_overflow_size)
    {
    if (is_data)
      {
      m_dropcount++;
      m_overflow_stats.drops++;
      OvmsDiagIncrement(&ovms_diag_live.vfs_overflow_drop);
      }
    return false;
    }

  m_overflow_ring[m_overflow_tail] = item;
  m_overflow_tail = (m_overflow_tail + 1) % m_overflow_size;
  m_overflow_count++;
  m_overflow_occupancy.store(m_overflow_count, std::memory_order_relaxed);
  if (m_overflow_count > m_overflow_highwater)
    m_overflow_highwater = m_overflow_count;
  m_overflow_stats.entries++;
  if (is_data)
    OvmsDiagIncrement(&ovms_diag_live.vfs_overflow_enqueue);
  return true;
  }

bool canlog_vfs::EnterStorageError(canlog_vfs_storage_error_t reason)
  {
  int expected = CANLOG_VFS_STORAGE_ERROR_NONE;
  if (!m_storage_error_reason.compare_exchange_strong(expected, reason,
        std::memory_order_acq_rel))
    return false;

  m_accepting.store(false, std::memory_order_release);
  OvmsDiagStore(&ovms_diag_live.vfs_storage_error, static_cast<uint32_t>(reason));
  return true;
  }

bool canlog_vfs::HasStorageError() const
  {
  return m_storage_error_reason.load(std::memory_order_acquire)
    != CANLOG_VFS_STORAGE_ERROR_NONE;
  }

void canlog_vfs::ReleaseStorageErrorItem(canlog_vfs_queue_msg_t& item,
  OvmsSemaphore*& close_ack)
  {
  switch (item.command)
    {
    case CANLOG_VFS_DATA:
      m_dropcount++;
      if (m_vfs_conn)
        m_vfs_conn->m_vfs_discardcount.fetch_add(1, std::memory_order_relaxed);
      FreeQueueItem(item);
      break;

    case CANLOG_VFS_CLOSE:
      if (!close_ack)
        close_ack = item.data.control.ack;
      else
        ESP_LOGE(TAG, "Multiple CLOSE requests found during terminal CAN log cleanup");
      break;

    case CANLOG_VFS_OPEN:
      *item.data.control.result = false;
      item.data.control.ack->Give();
      break;

    case CANLOG_VFS_CONFIG:
      break;
    }
  }

void canlog_vfs::HandleStorageError(canlog_vfs_queue_msg_t* pending,
  size_t pending_count)
  {
  // No producer may retain or enqueue a text pointer before terminal draining.
  while (m_producers.load(std::memory_order_acquire) != 0)
    vTaskDelay(1);

  OvmsSemaphore* close_ack = NULL;
  for (size_t i = 0; i < pending_count; ++i)
    ReleaseStorageErrorItem(pending[i], close_ack);

  {
  OvmsMutexLock route(&m_route_mutex);
  canlog_vfs_queue_msg_t item;
  while (m_queue && xQueueReceive(m_queue, &item, 0) == pdTRUE)
    ReleaseStorageErrorItem(item, close_ack);
  while (m_overflow_ring && m_overflow_count > 0)
    {
    item = m_overflow_ring[m_overflow_head];
    m_overflow_head = (m_overflow_head + 1) % m_overflow_size;
    m_overflow_count--;
    m_overflow_occupancy.store(m_overflow_count, std::memory_order_relaxed);
    ReleaseStorageErrorItem(item, close_ack);
    }
  if (m_spill_active && m_spill_started > 0)
    {
    uint64_t active_elapsed = esp_timer_get_time() - m_spill_started;
    m_overflow_stats.active_time += active_elapsed;
    UpdateTimingMax(m_overflow_stats.active_max, active_elapsed);
    }
  m_overflow_head = 0;
  m_overflow_tail = 0;
  m_spill_active = false;
  m_spill_started = 0;
  }

  if (m_vfs_conn)
    m_vfs_conn->CloseFileStorageError();

  // A CLOSE queued before the error keeps its normal acknowledgement. Otherwise
  // block at priority 5 until StopVfsTask uses the terminal out-of-band path.
  if (close_ack)
    {
    m_task = NULL;
    close_ack->Give();
    return;
    }

  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  m_task = NULL;
  m_terminal_stop_ack.Give();
  }

size_t canlog_vfs::CopyOverflowBatch(canlog_vfs_queue_msg_t* destination, size_t capacity)
  {
  if (!destination || capacity == 0)
    return 0;

  size_t copied = 0;
  size_t head = 0;
  {
  OvmsMutexLock route(&m_route_mutex);
  if (!m_spill_active || m_overflow_count == 0)
    return 0;

  head = m_overflow_head;
  copied = std::min(capacity, m_overflow_count);
  copied = std::min(copied, m_overflow_size - head);
  }

  int64_t copy_started = esp_timer_get_time();
  memcpy(destination, m_overflow_ring + head,
    copied * sizeof(canlog_vfs_queue_msg_t));
  uint64_t copy_elapsed = esp_timer_get_time() - copy_started;

  {
  OvmsMutexLock route(&m_route_mutex);
  // Only the owner advances the head. The count remains unchanged during the
  // unlocked copy, so producers cannot wrap onto the reserved source slots.
  if (m_overflow_head != head || m_overflow_count < copied)
    {
    ESP_LOGE(TAG, "VFS CAN log overflow state changed during owner copy");
    return 0;
    }

  m_overflow_head = (m_overflow_head + copied) % m_overflow_size;
  m_overflow_count -= copied;
  m_overflow_occupancy.store(m_overflow_count, std::memory_order_relaxed);
  m_overflow_stats.drain_batches++;
  m_overflow_stats.copy_time += copy_elapsed;
  UpdateTimingMax(m_overflow_stats.copy_max, copy_elapsed);

  if (m_overflow_count == 0)
    {
    int64_t now = esp_timer_get_time();
    uint64_t active_elapsed = now - m_spill_started;
    m_overflow_stats.active_time += active_elapsed;
    UpdateTimingMax(m_overflow_stats.active_max, active_elapsed);
    m_spill_active = false;
    m_spill_started = 0;
    }
  }

  return copied;
  }

void canlog_vfs::UpdateTimingMax(uint64_t& maximum, uint64_t elapsed)
  {
  if (maximum < elapsed)
    maximum = elapsed;
  }

void canlog_vfs::RecordDataTiming(int64_t started, const canlog_vfs_data_timing_t& timing)
  {
  OvmsMutexLock lock(&m_timing_mutex);
  m_timing.format_calls += timing.format_calls;
  m_timing.format_time += timing.format_time;
  UpdateTimingMax(m_timing.format_max, timing.format_max);
  m_timing.format_bytes += timing.format_bytes;
  m_timing.write_calls += timing.write_calls;
  m_timing.write_time += timing.write_time;
  UpdateTimingMax(m_timing.write_max, timing.write_max);
  m_timing.write_bytes += timing.write_bytes;
  m_timing.write_slow_1ms += timing.write_slow_1ms;
  m_timing.write_slow_5ms += timing.write_slow_5ms;
  m_timing.write_slow_10ms += timing.write_slow_10ms;

  uint64_t elapsed = esp_timer_get_time() - started;
  m_timing.data_calls++;
  m_timing.data_time += elapsed;
  UpdateTimingMax(m_timing.data_max, elapsed);
  }

void canlog_vfs::RecordFormatTiming(canlog_vfs_data_timing_t& timing, uint64_t elapsed, size_t bytes)
  {
  timing.format_calls++;
  timing.format_time += elapsed;
  UpdateTimingMax(timing.format_max, elapsed);
  if (bytes > 0)
    timing.format_bytes += bytes;
  }

void canlog_vfs::RecordWriteTiming(uint64_t elapsed, size_t bytes)
  {
  if (m_active_timing)
    {
    m_active_timing->write_calls++;
    m_active_timing->write_time += elapsed;
    UpdateTimingMax(m_active_timing->write_max, elapsed);
    m_active_timing->write_bytes += bytes;
    if (elapsed >= 1000)
      m_active_timing->write_slow_1ms++;
    if (elapsed >= 5000)
      m_active_timing->write_slow_5ms++;
    if (elapsed >= 10000)
      m_active_timing->write_slow_10ms++;
    return;
    }

  OvmsMutexLock lock(&m_timing_mutex);
  m_timing.write_calls++;
  m_timing.write_time += elapsed;
  UpdateTimingMax(m_timing.write_max, elapsed);
  m_timing.write_bytes += bytes;
  if (elapsed >= 1000)
    m_timing.write_slow_1ms++;
  if (elapsed >= 5000)
    m_timing.write_slow_5ms++;
  if (elapsed >= 10000)
    m_timing.write_slow_10ms++;
  }

void canlog_vfs::FreeQueueItem(canlog_vfs_queue_msg_t& item)
  {
  if (item.command != CANLOG_VFS_DATA)
    return;

  switch (item.data.message.type)
    {
    case CAN_LogInfo_Comment:
    case CAN_LogInfo_Config:
    case CAN_LogInfo_Event:
    case CAN_LogInfo_Metric:
      free(item.data.message.text);
      item.data.message.text = NULL;
      break;
    default:
      break;
    }
  }

void canlog_vfs::DrainQueue()
  {
  canlog_vfs_queue_msg_t item;
  if (m_queue)
    {
    while (xQueueReceive(m_queue, &item, 0) == pdTRUE)
      FreeQueueItem(item);
    }

  OvmsMutexLock route(&m_route_mutex);
  while (m_overflow_ring && m_overflow_count > 0)
    {
    item = m_overflow_ring[m_overflow_head];
    m_overflow_head = (m_overflow_head + 1) % m_overflow_size;
    m_overflow_count--;
    m_overflow_occupancy.store(m_overflow_count, std::memory_order_relaxed);
    FreeQueueItem(item);
    }
  m_overflow_head = 0;
  m_overflow_tail = 0;
  m_spill_active = false;
  m_spill_started = 0;
  }

void canlog_vfs::DestroyVfsQueue()
  {
  if (m_queue)
    {
    vQueueDelete(m_queue);
    m_queue = NULL;
    }
  if (m_overflow_ring)
    {
    heap_caps_free(m_overflow_ring);
    m_overflow_ring = NULL;
    }
  if (m_overflow_scratch)
    {
    heap_caps_free(m_overflow_scratch);
    m_overflow_scratch = NULL;
    }
  m_queue_size = 0;
  m_primary_highwater.store(0);
  m_overflow_size = 0;
  m_overflow_head = 0;
  m_overflow_tail = 0;
  m_overflow_count = 0;
  m_overflow_occupancy.store(0, std::memory_order_relaxed);
  m_overflow_highwater = 0;
  m_spill_active = false;
  m_spill_started = 0;
  }

bool canlog_vfs::Open()
  {
  // Snapshot startup-only queue capacities before taking the lifecycle mutex.
  // config.changed callbacks take the config lock before m_lifecycle_mutex, so
  // this preserves that lock order and avoids resizing an active instance.
  size_t primary_size;
  size_t overflow_size;
  size_t batch_capacity_config;
  {
  auto config_lock = MyConfig.Lock();
  if (!GetVfsQueueSizeConfig(CANLOG_VFS_PRIMARY_SIZE_CONFIG,
        CANLOG_VFS_PRIMARY_SIZE_DEFAULT, CANLOG_VFS_PRIMARY_SIZE_MIN,
        CANLOG_VFS_PRIMARY_SIZE_MAX, primary_size)
      || !GetVfsQueueSizeConfig(CANLOG_VFS_OVERFLOW_SIZE_CONFIG,
        CANLOG_VFS_OVERFLOW_SIZE_DEFAULT, CANLOG_VFS_OVERFLOW_SIZE_MIN,
        CANLOG_VFS_OVERFLOW_SIZE_MAX, overflow_size)
      || !GetVfsBatchSizeConfig(batch_capacity_config))
    return false;
  }
  m_batch_capacity_config.store(batch_capacity_config,
    std::memory_order_relaxed);

  OvmsMutexLock lifecycle(&m_lifecycle_mutex);

  if (m_queue || m_task || m_vfs_conn)
    CloseLocked();

  VfsDiagReset();
  OvmsDiagIncrement(&ovms_diag_live.vfs_lifecycle_sequence);
  VfsDiagCaptureLifecycle(OVMS_DIAG_VFS_LIFECYCLE_OPEN_BASELINE);
  m_storage_error_reason.store(CANLOG_VFS_STORAGE_ERROR_NONE,
    std::memory_order_release);

  if (MyConfig.ProtectedPath(m_path))
    {
    ESP_LOGE(TAG, "Error: Path '%s' is protected and cannot be opened", m_path.c_str());
    return false;
    }

#ifdef CONFIG_OVMS_COMP_SDCARD
  if (startsWith(m_path, "/sd") && (!MyPeripherals || !MyPeripherals->m_sdcard || !MyPeripherals->m_sdcard->isavailable()))
    {
    ESP_LOGE(TAG, "Error: Cannot open '%s' as SD filesystem not available", m_path.c_str());
    return false;
    }
#endif // #ifdef CONFIG_OVMS_COMP_SDCARD

  if (!StartVfsTask(primary_size, overflow_size))
    return false;

  bool opened = false;
  OvmsSemaphore ack;
  canlog_vfs_queue_msg_t item = {};
  item.command = CANLOG_VFS_OPEN;
  item.data.control.ack = &ack;
  item.data.control.result = &opened;
  const uint32_t stage_caps = MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
  OvmsDiagStore(&ovms_diag_live.vfs_task_ready_free,
    static_cast<uint32_t>(heap_caps_get_free_size(stage_caps)));
  OvmsDiagStore(&ovms_diag_live.vfs_task_ready_largest,
    static_cast<uint32_t>(heap_caps_get_largest_free_block(stage_caps)));
  VfsDiagCaptureLifecycle(OVMS_DIAG_VFS_LIFECYCLE_TASK_READY);
  if (!QueueControl(item))
    {
    StopVfsTask();
    DrainQueue();
    DestroyVfsQueue();
    return false;
    }
  ack.Take();

  if (!opened)
    {
    StopVfsTask();
    DrainQueue();
    DestroyVfsQueue();
    return false;
    }

  m_isopen = true;
  m_accepting.store(true, std::memory_order_release);
  VfsDiagCaptureLifecycle(OVMS_DIAG_VFS_LIFECYCLE_FULLY_OPEN);
  ESP_LOGI(TAG, "Now logging CAN messages to '%s'", m_path.c_str());
  return true;
  }

void canlog_vfs::Close()
  {
  OvmsMutexLock lifecycle(&m_lifecycle_mutex);
  CloseLocked();
  }

void canlog_vfs::CloseLocked()
  {
  bool had_resources = m_queue || m_task || m_vfs_conn;
  if (had_resources)
    VfsDiagCaptureLifecycle(OVMS_DIAG_VFS_LIFECYCLE_PRE_CLOSE);
  m_accepting.store(false, std::memory_order_release);
  m_isopen = false;

  while (m_producers.load(std::memory_order_acquire) != 0)
    vTaskDelay(1);

  StopVfsTask();

  if (m_vfs_conn)
    ESP_LOGI(TAG, "Closed vfs log '%s': %s", m_path.c_str(), GetStats().c_str());

  {
  OvmsRecMutexLock lock(&m_cmmutex);
  for (conn_map_t::iterator it=m_connmap.begin(); it!=m_connmap.end(); ++it)
    delete it->second;
  m_connmap.clear();
  m_vfs_conn = NULL;
  }
  if (had_resources)
    VfsDiagCaptureLifecycle(OVMS_DIAG_VFS_LIFECYCLE_POST_CLOSE_FREE);

  DrainQueue();
  DestroyVfsQueue();
  m_task = NULL;
  m_sync_deadline_ms.store(0, std::memory_order_relaxed);
  if (had_resources)
    VfsDiagCaptureLifecycle(OVMS_DIAG_VFS_LIFECYCLE_POST_STOP);
  }

void canlog_vfs::LogFrame(canbus* bus, CAN_log_type_t type, const CAN_frame_t* frame)
  {
  if (!bus || !frame || !BeginProducer())
    return;

  if (m_filter && !m_filter->IsFiltered(frame))
    {
    m_filtercount++;
    EndProducer();
    return;
    }

  CAN_log_message_t msg;
  msg.type = type;
  gettimeofday(&msg.timestamp, NULL);
  memcpy(&msg.frame, frame, sizeof(CAN_frame_t));
  msg.frame.origin = bus;
  m_msgcount++;
  QueueMessage(msg, false);
  EndProducer();
  }

void canlog_vfs::LogStatus(canbus* bus, CAN_log_type_t type, const CAN_status_t* status)
  {
  if (!bus || !status || !BeginProducer())
    return;

  if (m_filter && !m_filter->IsFiltered(bus))
    {
    m_filtercount++;
    EndProducer();
    return;
    }

  CAN_log_message_t msg;
  msg.type = type;
  gettimeofday(&msg.timestamp, NULL);
  msg.origin = bus;
  memcpy(&msg.status, status, sizeof(CAN_status_t));
  m_msgcount++;
  QueueMessage(msg, false);
  EndProducer();
  }

void canlog_vfs::LogInfo(canbus* bus, CAN_log_type_t type, const char* text)
  {
  if (!text || !BeginProducer())
    return;

  if (m_filter && !m_filter->IsFiltered(bus))
    {
    m_filtercount++;
    EndProducer();
    return;
    }

  CAN_log_message_t msg;
  msg.type = type;
  gettimeofday(&msg.timestamp, NULL);
  msg.origin = bus;
  msg.text = strdup(text);
  m_msgcount++;
  if (msg.text)
    QueueMessage(msg, true);
  else
    m_dropcount++;
  EndProducer();
  }

size_t canlog_vfs::GetFileSize()
  {
  OvmsRecMutexLock lock(&m_cmmutex);
  return m_vfs_conn ? m_vfs_conn->m_file_size.load() : 0;
  }

std::string canlog_vfs_conn::GetStats()
  {
  uint32_t msgcount = m_vfs_msgcount.load();
  uint32_t dropcount = m_vfs_dropcount.load();
  uint32_t discardcount = m_vfs_discardcount.load();
  uint32_t filtercount = m_vfs_filtercount.load();
  float droprate = (msgcount > 0) ? ((float)dropcount / msgcount * 100) : 0;

  char bufsize[15];
  format_file_size(bufsize, sizeof(bufsize), m_file_size.load());

  std::ostringstream result;
  result << "Size:" << bufsize
    << " Messages:" << msgcount
    << " Discarded:" << discardcount
    << " Dropped:" << dropcount
    << " Filtered:" << filtercount
    << " Rate:" << std::fixed << std::setprecision(1) << droprate << "%";

  {
  OvmsMutexLock lock(&m_stats_mutex);
  result << " Syncs:" << m_sync_count
    << " SyncErrors:" << m_sync_errors
    << " SyncTime:" << std::fixed << std::setprecision(3)
    << (double)m_sync_time / 1000000.0 << "s";
  }

  return result.str();
  }

std::string canlog_vfs::GetStats()
  {
  OvmsRecMutexLock lock(&m_cmmutex);

  char bufsize[15];
  size_t size = GetFileSize();
  format_file_size(bufsize, sizeof(bufsize), size);

  std::string result = "Size:";
  result.append(bufsize);
  result.append(" ");

  std::ostringstream stats;
  uint32_t primary_queued = 0;
  size_t overflow_queued = 0;
  size_t overflow_highwater = 0;
  size_t overflow_size = 0;
  bool spill_active = false;
  int64_t spill_started = 0;
  canlog_vfs_overflow_stats_t overflow_stats;
  {
  OvmsMutexLock route(&m_route_mutex);
  primary_queued = m_queue ? uxQueueMessagesWaiting(m_queue) : 0;
  overflow_queued = m_overflow_count;
  overflow_highwater = m_overflow_highwater;
  overflow_size = m_overflow_size;
  spill_active = m_spill_active;
  spill_started = m_spill_started;
  overflow_stats = m_overflow_stats;
  }
  if (spill_active && spill_started > 0)
    {
    uint64_t active_elapsed = esp_timer_get_time() - spill_started;
    overflow_stats.active_time += active_elapsed;
    UpdateTimingMax(overflow_stats.active_max, active_elapsed);
    }

  float droprate = (m_msgcount > 0) ? ((float)m_dropcount / m_msgcount * 100) : 0;
  stats << "Messages:" << m_msgcount
    << " Dropped:" << m_dropcount
    << " Filtered:" << m_filtercount
    << " Rate:" << std::fixed << std::setprecision(1) << droprate << "%"
    << " PrimarySize:" << m_queue_size
    << " PrimaryQueued:" << primary_queued
    << " PrimaryHighWater:" << m_primary_highwater.load()
    << " PrimaryStorage:" << (m_queue && esp_ptr_internal(m_queue) ? "internal" : "none")
    << " OverflowSize:" << overflow_size
    << " OverflowQueued:" << overflow_queued
    << " OverflowHighWater:" << overflow_highwater
    << " OverflowStorage:" << (m_overflow_ring && esp_ptr_external_ram(m_overflow_ring) ? "spiram" : "none")
    << " SpillActive:" << (spill_active ? 1 : 0)
    << " StorageError:" << (HasStorageError() ? 1 : 0)
    << " StorageErrorReason:" << m_storage_error_reason.load(std::memory_order_acquire)
    << " VfsBuffer:" << (m_vfs_conn ? m_vfs_conn->m_stdio_buffer_size : 0)
    << " BufferSet:" << (m_vfs_conn && m_vfs_conn->m_stdio_buffer_set ? 1 : 0)
    << " StdioMode:unbuffered"
    << " BatchConfigured:" << m_batch_capacity_config.load()
    << " BatchSize:" << (m_vfs_conn ? m_vfs_conn->m_batch_capacity : 0)
    << " BatchLimit:" << (m_vfs_conn ? m_vfs_conn->m_batch_limit.load() : 0)
    << " ClusterAlign:" << (m_vfs_conn ? m_vfs_conn->m_cluster_size : 0)
    << " BatchUsed:" << (m_vfs_conn ? m_vfs_conn->m_batch_used.load() : 0)
    << " DirectFormat:" << (m_vfs_conn && m_vfs_conn->m_format_buffer ? 1 : 0);
  result.append(stats.str());

  std::ostringstream overflowstats;
  overflowstats << "\n  OverflowEntries:" << overflow_stats.entries
    << " OverflowDrops:" << overflow_stats.drops
    << " OverflowTransitions:" << overflow_stats.transitions
    << " OverflowDrainBatches:" << overflow_stats.drain_batches
    << " OverflowCopyTime:" << std::fixed << std::setprecision(3)
    << (double)overflow_stats.copy_time / 1000000.0 << "s"
    << " OverflowCopyMax:" << std::fixed << std::setprecision(3)
    << (double)overflow_stats.copy_max / 1000.0 << "ms"
    << "\n  OverflowActiveTime:" << std::fixed << std::setprecision(3)
    << (double)overflow_stats.active_time / 1000000.0 << "s"
    << " OverflowActiveMax:" << std::fixed << std::setprecision(3)
    << (double)overflow_stats.active_max / 1000.0 << "ms";
  result.append(overflowstats.str());

  char syncstats[96];
  if (m_vfs_conn)
    {
    OvmsMutexLock stats_lock(&m_vfs_conn->m_stats_mutex);
    snprintf(syncstats, sizeof(syncstats),
      " SyncPeriod:%ds Syncs:%" PRIu32 " SyncErrors:%" PRIu32 " SyncTime:%.3fs",
      m_syncperiod.load(), m_vfs_conn->m_sync_count, m_vfs_conn->m_sync_errors,
      (double)m_vfs_conn->m_sync_time / 1000000.0);
    }
  else
    snprintf(syncstats, sizeof(syncstats), " SyncPeriod:%ds Syncs:0 SyncErrors:0 SyncTime:0.000s",
      m_syncperiod.load());
  result.append(syncstats);

  canlog_vfs_timing_stats_t timing;
  {
  OvmsMutexLock timing_lock(&m_timing_mutex);
  timing = m_timing;
  }

  std::ostringstream timingstats;
  timingstats << "\n  DataCalls:" << timing.data_calls
    << " DataTime:" << std::fixed << std::setprecision(3)
    << (double)timing.data_time / 1000000.0 << "s"
    << " DataMax:" << std::fixed << std::setprecision(3)
    << (double)timing.data_max / 1000.0 << "ms"
    << "\n  FormatCalls:" << timing.format_calls
    << " FormatTime:" << std::fixed << std::setprecision(3)
    << (double)timing.format_time / 1000000.0 << "s"
    << " FormatMax:" << std::fixed << std::setprecision(3)
    << (double)timing.format_max / 1000.0 << "ms"
    << " FormatBytes:" << timing.format_bytes
    << "\n  WriteCalls:" << timing.write_calls
    << " WriteTime:" << std::fixed << std::setprecision(3)
    << (double)timing.write_time / 1000000.0 << "s"
    << " WriteMax:" << std::fixed << std::setprecision(3)
    << (double)timing.write_max / 1000.0 << "ms"
    << " WriteBytes:" << timing.write_bytes
    << " WriteSlow1ms:" << timing.write_slow_1ms
    << " WriteSlow5ms:" << timing.write_slow_5ms
    << " WriteSlow10ms:" << timing.write_slow_10ms;
  result.append(timingstats.str());

  bool write_in_progress = m_vfs_conn
    && m_vfs_conn->m_write_in_progress.load(std::memory_order_acquire);
  uint32_t write_in_progress_ms = 0;
  size_t write_current_requested = 0;
  if (write_in_progress)
    {
    uint32_t started_ms = m_vfs_conn->m_write_in_progress_started_ms.load(
      std::memory_order_relaxed);
    uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    write_in_progress_ms = now_ms - started_ms;
    write_current_requested = m_vfs_conn->m_write_current_requested.load(
      std::memory_order_relaxed);
    }

  std::ostringstream writestats;
  writestats << "\n  WriteErrors:"
    << (m_vfs_conn ? m_vfs_conn->m_write_errors.load() : 0)
    << " WriteShort:" << (m_vfs_conn ? m_vfs_conn->m_write_short.load() : 0)
    << " WriteZero:" << (m_vfs_conn ? m_vfs_conn->m_write_zero.load() : 0)
    << " WriteFirstErrno:" << (m_vfs_conn ? m_vfs_conn->m_write_first_errno.load() : 0)
    << " WriteFirstFerror:" << (m_vfs_conn ? m_vfs_conn->m_write_first_ferror.load() : 0)
    << " WriteFirstRequested:" << (m_vfs_conn ? m_vfs_conn->m_write_first_requested.load() : 0)
    << " WriteFirstReturned:" << (m_vfs_conn ? m_vfs_conn->m_write_first_returned.load() : 0)
    << " WriteFirstDurationMs:" << (m_vfs_conn ? m_vfs_conn->m_write_first_duration_ms.load() : 0)
    << " WriteInProgress:" << (write_in_progress ? 1 : 0)
    << " WriteInProgressMs:" << write_in_progress_ms
    << " WriteCurrentRequested:" << write_current_requested;
  result.append(writestats.str());

  std::ostringstream diagstats;
  diagstats << "\n  SlowWriteThresholdUs:"
    << OvmsDiagLoad(&ovms_diag_live.vfs_slow_write_threshold_us)
    << " SlowWriteSequence:"
    << OvmsDiagLoad(&ovms_diag_live.vfs_slow_write_sequence)
    << " SlowWriteRequested:"
    << OvmsDiagLoad(&ovms_diag_live.vfs_slow_write_requested)
    << " SlowWriteAccepted:"
    << OvmsDiagLoad(&ovms_diag_live.vfs_slow_write_accepted)
    << " SlowWriteOffset:"
    << OvmsDiagLoad(&ovms_diag_live.vfs_slow_write_file_offset)
    << " SlowWriteClusterOffset:"
    << OvmsDiagLoad(&ovms_diag_live.vfs_slow_write_cluster_offset)
    << " SlowWriteBatch:"
    << OvmsDiagLoad(&ovms_diag_live.vfs_slow_write_batch_used) << "/"
    << OvmsDiagLoad(&ovms_diag_live.vfs_slow_write_batch_capacity)
    << " SlowWriteElapsedUs:"
    << OvmsDiagLoad(&ovms_diag_live.vfs_slow_write_elapsed_us)
    << " SlowWriteSyncDueMs:"
    << OvmsDiagLoad(&ovms_diag_live.vfs_slow_write_sync_due_ms)
    << " SlowWriteSyncState:"
    << OvmsDiagLoad(&ovms_diag_live.vfs_slow_write_sync_state)
    << " SlowWriteQueues:"
    << OvmsDiagLoad(&ovms_diag_live.vfs_slow_write_primary_queued) << "/"
    << OvmsDiagLoad(&ovms_diag_live.vfs_slow_write_overflow_queued)
    << " SlowWriteError:"
    << OvmsDiagLoad(&ovms_diag_live.vfs_slow_write_error_state)
    << "\n  FFlushLastUs:" << OvmsDiagLoad(&ovms_diag_live.vfs_fflush_last_us)
    << " FFlushMaxUs:" << OvmsDiagLoad(&ovms_diag_live.vfs_fflush_max_us)
    << " FFlushResult:" << OvmsDiagLoad(&ovms_diag_live.vfs_fflush_result)
    << " FSyncLastUs:" << OvmsDiagLoad(&ovms_diag_live.vfs_fsync_last_us)
    << " FSyncMaxUs:" << OvmsDiagLoad(&ovms_diag_live.vfs_fsync_max_us)
    << " FSyncResult:" << OvmsDiagLoad(&ovms_diag_live.vfs_fsync_result);
  result.append(diagstats.str());

  return result;
  }

std::string canlog_vfs::GetInfo()
  {
  std::string result = canlog::GetInfo();
  result.append(" Path:");
  result.append(m_path);
  return result;
  }

void canlog_vfs::MountListener(std::string event, void* data)
  {
  if (event == "sd.unmounting" && startsWith(m_path, "/sd"))
    Close();
  else if (event == "sd.mounted" && startsWith(m_path, "/sd"))
    Open();
  }

void canlog_vfs::LoadConfig()
  {
  canlog::LoadConfig();

  int syncperiod = MyConfig.GetParamValueInt(CAN_PARAM, "log.vfs.syncperiod", 0);
  if (syncperiod < -1)
    {
    ESP_LOGW(TAG, "Invalid can/log.vfs.syncperiod value %d; disabling periodic sync", syncperiod);
    syncperiod = 0;
    }

  m_syncperiod.store(syncperiod);

  size_t batch_capacity_config;
  if (GetVfsBatchSizeConfig(batch_capacity_config))
    m_batch_capacity_config.store(batch_capacity_config,
      std::memory_order_relaxed);

  OvmsMutexLock lifecycle(&m_lifecycle_mutex);
  if (m_task && m_queue)
    {
    canlog_vfs_queue_msg_t item = {};
    item.command = CANLOG_VFS_CONFIG;
    item.data.control.syncperiod = syncperiod;
    QueueControl(item);
    }
  }
