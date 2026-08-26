/*
;    Project:       Open Vehicle Monitor System
;    Module:        CAN logging framework
;    Date:          18th January 2018
;
;    (C) 2018       Michael Balzer
;    (C) 2019       Mark Webb-Johnson
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

#ifndef __CANLOG_VFS_H__
#define __CANLOG_VFS_H__

#include <atomic>
#include "canlog.h"
#include "ovms_mutex.h"
#include "ovms_semaphore.h"


typedef enum
  {
  CANLOG_VFS_DATA,
  CANLOG_VFS_OPEN,
  CANLOG_VFS_CLOSE,
  CANLOG_VFS_CONFIG,
  } canlog_vfs_command_t;

typedef enum
  {
  CANLOG_VFS_STORAGE_ERROR_NONE = 0,
  CANLOG_VFS_STORAGE_ERROR_WRITE = 1,
  } canlog_vfs_storage_error_t;

typedef struct
  {
  canlog_vfs_command_t command;
  union
    {
    CAN_log_message_t message;
    struct
      {
      OvmsSemaphore* ack;
      bool* result;
      int syncperiod;
      } control;
    } data;
  } canlog_vfs_queue_msg_t;


struct canlog_vfs_timing_stats_t
  {
  canlog_vfs_timing_stats_t()
    : data_calls(0), data_time(0), data_max(0),
      format_calls(0), format_time(0), format_max(0), format_bytes(0),
      write_calls(0), write_time(0), write_max(0), write_bytes(0),
      write_slow_1ms(0), write_slow_5ms(0), write_slow_10ms(0)
    {}

  uint32_t data_calls;
  uint64_t data_time;
  uint64_t data_max;
  uint32_t format_calls;
  uint64_t format_time;
  uint64_t format_max;
  uint64_t format_bytes;
  uint32_t write_calls;
  uint64_t write_time;
  uint64_t write_max;
  uint64_t write_bytes;
  uint32_t write_slow_1ms;
  uint32_t write_slow_5ms;
  uint32_t write_slow_10ms;
  };


struct canlog_vfs_data_timing_t
  {
  canlog_vfs_data_timing_t()
    : format_calls(0), format_time(0), format_max(0), format_bytes(0),
      write_calls(0), write_time(0), write_max(0), write_bytes(0),
      write_slow_1ms(0), write_slow_5ms(0), write_slow_10ms(0)
    {}

  uint32_t format_calls;
  uint64_t format_time;
  uint64_t format_max;
  uint64_t format_bytes;
  uint32_t write_calls;
  uint64_t write_time;
  uint64_t write_max;
  uint64_t write_bytes;
  uint32_t write_slow_1ms;
  uint32_t write_slow_5ms;
  uint32_t write_slow_10ms;
  };


struct canlog_vfs_overflow_stats_t
  {
  canlog_vfs_overflow_stats_t()
    : entries(0), drops(0), transitions(0), drain_batches(0),
      copy_time(0), copy_max(0), active_time(0), active_max(0)
    {}

  uint32_t entries;
  uint32_t drops;
  uint32_t transitions;
  uint32_t drain_batches;
  uint64_t copy_time;
  uint64_t copy_max;
  uint64_t active_time;
  uint64_t active_max;
  };


class canlog_vfs_conn: public canlogconnection
  {
  public:
    canlog_vfs_conn(canlog* logger, std::string format, canformat::canformat_serve_mode_t mode);
    virtual ~canlog_vfs_conn();

  public:
    virtual void OutputMsg(CAN_log_message_t& msg, std::string &result);
    void OutputMsgDirect(CAN_log_message_t& msg, char* data, size_t length, bool in_batch);
    virtual std::string GetStats();
    bool GetFormatBuffer(size_t required, char*& buffer, bool& in_batch);
    bool AppendBytes(const char* data, size_t length);
    bool WriteFileBytes(const char* data, size_t length, size_t& accepted);
    void UpdateBatchLimit();
    bool FlushBatch();
    bool Sync(bool force=false);
    bool CloseFile();
    bool CloseFileStorageError();

  public:
    FILE*               m_file;
    size_t              m_stdio_buffer_size;
    bool                m_stdio_buffer_set;
    char*               m_batch_buffer;
    size_t              m_batch_capacity;
    size_t              m_cluster_size;
    std::atomic<size_t> m_batch_limit;
    std::atomic<size_t> m_batch_used;
    char*               m_format_buffer;
    size_t              m_format_buffer_size;
    bool                m_write_blocked;
    std::atomic<uint32_t> m_write_errors;
    std::atomic<uint32_t> m_write_short;
    std::atomic<uint32_t> m_write_zero;
    std::atomic<bool>    m_write_first_recorded;
    std::atomic<int>     m_write_first_errno;
    std::atomic<int>     m_write_first_ferror;
    std::atomic<size_t>  m_write_first_requested;
    std::atomic<size_t>  m_write_first_returned;
    std::atomic<uint32_t> m_write_first_duration_ms;
    std::atomic<bool>    m_write_in_progress;
    std::atomic<uint32_t> m_write_in_progress_started_ms;
    std::atomic<size_t>  m_write_current_requested;
    std::atomic<size_t> m_file_size;
    std::atomic<uint32_t> m_vfs_msgcount;
    std::atomic<uint32_t> m_vfs_dropcount;
    std::atomic<uint32_t> m_vfs_discardcount;
    std::atomic<uint32_t> m_vfs_filtercount;
    uint32_t            m_sync_count;
    uint32_t            m_sync_errors;
    uint64_t            m_sync_time;
    bool                m_dirty;
    std::atomic<bool>    m_sync_in_progress;
    OvmsMutex           m_stats_mutex;
  };


class canlog_vfs : public canlog
  {
  friend class canlog_vfs_conn;

  public:
    canlog_vfs(std::string path, std::string format);
    virtual ~canlog_vfs();

  protected:
    static void VfsTaskEntry(void* context);
    void VfsTask();
    bool StartVfsTask(size_t primary_size, size_t overflow_size);
    void StopVfsTask();
    bool OpenFile();
    void CloseFile();
    void CloseLocked();
    bool QueueMessage(CAN_log_message_t& msg, bool has_text);
    bool QueueControl(canlog_vfs_queue_msg_t& item);
    bool RouteQueueItem(const canlog_vfs_queue_msg_t& item, bool is_data);
    bool EnterStorageError(canlog_vfs_storage_error_t reason);
    bool HasStorageError() const;
    void HandleStorageError(canlog_vfs_queue_msg_t* pending, size_t pending_count);
    void ReleaseStorageErrorItem(canlog_vfs_queue_msg_t& item,
      OvmsSemaphore*& close_ack);
    void BeginSpillLocked();
    size_t CopyOverflowBatch(canlog_vfs_queue_msg_t* destination, size_t capacity);
    bool ProcessQueueItem(canlog_vfs_queue_msg_t& item, int& syncperiod, int64_t& sync_deadline);
    void CheckSyncDeadline(int syncperiod, int64_t& sync_deadline);
    static void FreeQueueItem(canlog_vfs_queue_msg_t& item);
    bool BeginProducer();
    void EndProducer();
    void DrainQueue();
    void DestroyVfsQueue();
    static void UpdateTimingMax(uint64_t& maximum, uint64_t elapsed);
    void RecordDataTiming(int64_t started, const canlog_vfs_data_timing_t& timing);
    void RecordFormatTiming(canlog_vfs_data_timing_t& timing, uint64_t elapsed, size_t bytes);
    void RecordWriteTiming(uint64_t elapsed, size_t bytes);

  public:
    virtual bool Open();
    virtual void Close();
    virtual std::string GetInfo();
    virtual size_t GetFileSize();
    virtual void LogFrame(canbus* bus, CAN_log_type_t type, const CAN_frame_t* p_frame);
    virtual void LogStatus(canbus* bus, CAN_log_type_t type, const CAN_status_t* status);
    virtual void LogInfo(canbus* bus, CAN_log_type_t type, const char* text);

  public:
    virtual void MountListener(std::string event, void* data);
    virtual std::string GetStats();

  public:
    std::string         m_path;
    canlog_vfs_conn*    m_vfs_conn;
    std::atomic<int>    m_syncperiod;
    std::atomic<size_t> m_batch_capacity_config;
    int64_t             m_sync_deadline;
    uint32_t            m_diag_lifecycle_sequence;
    std::atomic<int>    m_storage_error_reason;
    std::atomic<bool>   m_accepting;
    std::atomic<uint32_t> m_producers;
    size_t              m_queue_size;
    std::atomic<uint32_t> m_primary_highwater;
    canlog_vfs_queue_msg_t* m_overflow_ring;
    canlog_vfs_queue_msg_t* m_overflow_scratch;
    size_t              m_overflow_size;
    size_t              m_overflow_head;
    size_t              m_overflow_tail;
    size_t              m_overflow_count;
    std::atomic<size_t> m_overflow_occupancy;
    size_t              m_overflow_highwater;
    bool                m_spill_active;
    int64_t             m_spill_started;
    canlog_vfs_overflow_stats_t m_overflow_stats;
    OvmsMutex           m_route_mutex;
    OvmsMutex           m_lifecycle_mutex;
    OvmsSemaphore       m_task_ready;
    OvmsSemaphore       m_terminal_stop_ack;
    OvmsMutex           m_timing_mutex;
    canlog_vfs_timing_stats_t m_timing;
    canlog_vfs_data_timing_t* m_active_timing;

  protected:
    virtual void LoadConfig();
  };

#endif // __CANLOG_VFS_H__
