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
static const size_t CANLOG_VFS_PRIMARY_SIZE_DEFAULT = 100;
static const size_t CANLOG_VFS_PRIMARY_SIZE_MIN = 32;
static const size_t CANLOG_VFS_PRIMARY_SIZE_MAX = 128;
static const size_t CANLOG_VFS_OVERFLOW_SIZE_DEFAULT = 1280;
static const size_t CANLOG_VFS_OVERFLOW_SIZE_MIN = 128;
static const size_t CANLOG_VFS_OVERFLOW_SIZE_MAX = 8192;
static const size_t CANLOG_VFS_OVERFLOW_SCRATCH_SIZE = 8;

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
  : canlogconnection(logger, format, mode), m_file(NULL), m_batch_buffer(NULL),
    m_batch_capacity(0), m_cluster_size(0), m_batch_limit(0), m_batch_used(0),
    m_format_buffer(NULL), m_format_buffer_size(0),
    m_write_blocked(false),
    m_write_errors(0), m_write_short(0), m_write_zero(0),
    m_file_size(0),
    m_vfs_msgcount(0), m_vfs_dropcount(0), m_vfs_discardcount(0), m_vfs_filtercount(0),
    m_sync_count(0), m_sync_errors(0), m_sync_time(0), m_dirty(false)
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
    errno = 0;
    int64_t started = esp_timer_get_time();
    size_t written = fwrite(data + accepted, 1, requested, m_file);
    int write_errno = errno;
    int write_ferror = ferror(m_file);
    int64_t elapsed = esp_timer_get_time() - started;
    bool fatal = written < requested || write_errno != 0 || write_ferror != 0;
    if (written < requested)
      {
      m_write_short.fetch_add(1, std::memory_order_relaxed);
      if (written == 0)
        m_write_zero.fetch_add(1, std::memory_order_relaxed);
      }
    if (fatal)
      m_write_errors.fetch_add(1, std::memory_order_relaxed);

    if (written > 0)
      {
      accepted += written;
      m_file_size += written;
      }

    if (fatal)
      {
      static_cast<canlog_vfs*>(m_logger)->EnterStorageError(
        CANLOG_VFS_STORAGE_ERROR_WRITE);
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

  size_t accepted = 0;
  if (WriteFileBytes(m_batch_buffer, batch_used, accepted))
    {
    m_batch_used.store(0);
    UpdateBatchLimit();
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
  return false;
  }

bool canlog_vfs_conn::Sync(bool force)
  {
  if (!m_file || (!force && !m_dirty))
    return true;
  if (static_cast<canlog_vfs*>(m_logger)->HasStorageError())
    return false;

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
    return false;
    }

  int flush_result = fflush(m_file);
  int sync_result = fsync(fileno(m_file));
  int64_t elapsed = esp_timer_get_time() - started;

  {
  OvmsMutexLock lock(&m_stats_mutex);
  m_sync_time += elapsed;
  m_sync_count++;

  if (flush_result != 0 || sync_result != 0)
    {
    m_sync_errors++;
    }
  }

  if (flush_result != 0 || sync_result != 0)
    {
    ESP_LOGE(TAG, "Error syncing CAN log '%s': pending=%u fflush=%d fsync=%d",
      m_peer.c_str(), (unsigned)m_batch_used.load(), flush_result, sync_result);
    return false;
    }

  m_dirty = false;
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
    m_syncperiod(0), m_batch_capacity_config(0),
    m_storage_error_reason(CANLOG_VFS_STORAGE_ERROR_NONE),
    m_accepting(false), m_producers(0),
    m_queue_size(0), m_primary_highwater(0),
    m_overflow_ring(NULL), m_overflow_scratch(NULL), m_overflow_size(0),
    m_overflow_head(0), m_overflow_tail(0), m_overflow_count(0),
    m_overflow_occupancy(0),
    m_overflow_highwater(0), m_spill_active(false)
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
  }

void canlog_vfs::VfsTaskEntry(void* context)
  {
  static_cast<canlog_vfs*>(context)->VfsTask();
  }

void canlog_vfs::VfsTask()
  {
  int syncperiod = m_syncperiod.load();
  int64_t sync_deadline = 0;
  m_task_ready.Give();

  for (;;)
    {
    canlog_vfs_queue_msg_t item;

    // Preserve FIFO ordering by exhausting the internal primary queue before
    // copying any records from the overflow ring.
    if (xQueueReceive(m_queue, &item, 0) == pdTRUE)
      {
      if (!ProcessQueueItem(item, syncperiod, sync_deadline))
        break;
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

  vTaskDelete(NULL);
  }

bool canlog_vfs::ProcessQueueItem(canlog_vfs_queue_msg_t& item,
  int& syncperiod, int64_t& sync_deadline)
  {
  switch (item.command)
    {
    case CANLOG_VFS_DATA:
      {
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
          size_t result_length = m_formatter->get(&msg, format_buffer, direct_size);
          if (result_length > 0)
            m_vfs_conn->OutputMsgDirect(msg, format_buffer, result_length, in_batch);
          }
        else
          {
          std::string result = m_formatter->get(&msg);
          if (result.length() > 0)
            m_vfs_conn->OutputMsg(msg, result);
          }

        if (!was_dirty && m_vfs_conn->m_dirty && syncperiod > 0)
          sync_deadline = esp_timer_get_time() + (int64_t)syncperiod * 1000000;

        if (syncperiod == -1 && m_vfs_conn->m_dirty)
          {
          m_vfs_conn->Sync();
          sync_deadline = 0;
          }
        }
      else
        {
        m_dropcount++;
        }

      FreeQueueItem(item);
      break;
      }

    case CANLOG_VFS_OPEN:
      *item.data.control.result = OpenFile();
      if (m_vfs_conn && m_vfs_conn->m_dirty && syncperiod > 0)
        sync_deadline = esp_timer_get_time() + (int64_t)syncperiod * 1000000;
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
          }
        else if (syncperiod > 0)
          sync_deadline = esp_timer_get_time() + (int64_t)syncperiod * 1000000;
        else
          sync_deadline = 0;
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
  m_overflow_stats = canlog_vfs_overflow_stats_t();

  while (m_task_ready.Take(0)) {}
  while (m_terminal_stop_ack.Take(0)) {}
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

  // Batching is owned by this logger, so leave stdio unbuffered and let every
  // fwrite go straight through to the cluster-aligned owner batch.
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

  conn->m_batch_buffer = static_cast<char*>(heap_caps_malloc(batch_size,
    MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
  if (!conn->m_batch_buffer)
    {
    ESP_LOGE(TAG, "Error: Can't allocate %u byte owner batch for '%s'",
      (unsigned)batch_size, m_path.c_str());
    fclose(conn->m_file);
    conn->m_file = NULL;
    delete conn;
    return false;
    }
  conn->m_batch_capacity = batch_size;
  conn->m_batch_limit.store(batch_size);
  conn->m_batch_used.store(0);

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
  return true;
  }

bool canlog_vfs::EnterStorageError(canlog_vfs_storage_error_t reason)
  {
  int expected = CANLOG_VFS_STORAGE_ERROR_NONE;
  if (!m_storage_error_reason.compare_exchange_strong(expected, reason,
        std::memory_order_acq_rel))
    return false;

  m_accepting.store(false, std::memory_order_release);
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
  m_overflow_head = 0;
  m_overflow_tail = 0;
  m_spill_active = false;
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

  memcpy(destination, m_overflow_ring + head,
    copied * sizeof(canlog_vfs_queue_msg_t));

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

  if (m_overflow_count == 0)
    m_spill_active = false;
  }

  return copied;
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

  DrainQueue();
  DestroyVfsQueue();
  m_task = NULL;
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

  uint32_t primary_queued = 0;
  size_t overflow_queued = 0;
  size_t overflow_highwater = 0;
  size_t overflow_size = 0;
  bool spill_active = false;
  canlog_vfs_overflow_stats_t overflow_stats;
  {
  OvmsMutexLock route(&m_route_mutex);
  primary_queued = m_queue ? uxQueueMessagesWaiting(m_queue) : 0;
  overflow_queued = m_overflow_count;
  overflow_highwater = m_overflow_highwater;
  overflow_size = m_overflow_size;
  spill_active = m_spill_active;
  overflow_stats = m_overflow_stats;
  }

  std::ostringstream stats;
  float droprate = (m_msgcount > 0) ? ((float)m_dropcount / m_msgcount * 100) : 0;
  stats << "Messages:" << m_msgcount
    << " Dropped:" << m_dropcount
    << " Filtered:" << m_filtercount
    << " Rate:" << std::fixed << std::setprecision(1) << droprate << "%"
    << " PrimarySize:" << m_queue_size
    << " PrimaryQueued:" << primary_queued
    << " PrimaryHighWater:" << m_primary_highwater.load()
    << " OverflowSize:" << overflow_size
    << " OverflowQueued:" << overflow_queued
    << " OverflowHighWater:" << overflow_highwater
    << " SpillActive:" << (spill_active ? 1 : 0)
    << " StorageError:" << (HasStorageError() ? 1 : 0)
    << " BatchSize:" << (m_vfs_conn ? m_vfs_conn->m_batch_capacity : 0)
    << " BatchLimit:" << (m_vfs_conn ? m_vfs_conn->m_batch_limit.load() : 0)
    << " ClusterAlign:" << (m_vfs_conn ? m_vfs_conn->m_cluster_size : 0)
    << " DirectFormat:" << (m_vfs_conn && m_vfs_conn->m_format_buffer ? 1 : 0)
    << "\n  OverflowEntries:" << overflow_stats.entries
    << " OverflowDrops:" << overflow_stats.drops
    << " OverflowTransitions:" << overflow_stats.transitions
    << " OverflowDrainBatches:" << overflow_stats.drain_batches
    << " WriteErrors:" << (m_vfs_conn ? m_vfs_conn->m_write_errors.load() : 0)
    << " WriteShort:" << (m_vfs_conn ? m_vfs_conn->m_write_short.load() : 0)
    << " WriteZero:" << (m_vfs_conn ? m_vfs_conn->m_write_zero.load() : 0);
  result.append(stats.str());

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
