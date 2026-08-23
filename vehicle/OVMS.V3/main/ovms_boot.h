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

#ifndef __OVMS_BOOT_H__
#define __OVMS_BOOT_H__

#include <string>

#include "rom/rtc.h"
#include "rom/crc.h"
#include "esp_system.h"
#include "ovms_events.h"
#include "ovms_mutex.h"
#include "ovms_diag.h"

typedef enum
  {
    BR_PowerOn = 0,                 // standard power on / hard reset
    BR_Wakeup,                      // reboot from deep sleep
    BR_SoftReset,                   // user requested reset ("module reset")
    BR_FirmwareUpdate,              // Firmware update reset
    BR_EarlyCrash,                  // crash during boot/init phase
    BR_Crash,                       // crash after reaching stable state
    BR_PartitionUpdate,             // Partition update reset
  } bootreason_t;

#if (ESP_IDF_VERSION_MAJOR < 4) || CONFIG_IDF_TARGET_ARCH_XTENSA
  #define __ARCH_NB_REGS 24
  #define __ARCH_REG_OFFSET_IN_FRAME 1
#elif CONFIG_IDF_TARGET_ARCH_RISCV
  #define __ARCH_NB_REGS 37
  #define __ARCH_REG_OFFSET_IN_FRAME 0
#else
  #error "Unknown architecture, please fix ovms_boot.h"
#endif

#define OVMS_BT_LEVELS 32
typedef struct
  {
  int core_id;
  bool is_abort;
  uint32_t reg[__ARCH_NB_REGS];
  struct
    {
    uint32_t pc;
    // possibly add stack info later on
    } bt[OVMS_BT_LEVELS];
  } crash_data_t;

typedef struct
  {
  char name[16];
  uint32_t stackfree;
  } task_info_t;

// Small, bounded diagnostic breadcrumbs for the CanSynth/CanLogVFS failure
// investigation. The live copy stays in normal DRAM; the panic handler copies
// it to boot_data once, so hot paths never write RTC slow memory or update the
// boot-data CRC.
enum ovms_diag_synth_state_t : uint32_t
  {
  OVMS_DIAG_SYNTH_NEVER = 0,
  OVMS_DIAG_SYNTH_RUNNING,
  OVMS_DIAG_SYNTH_DURATION,
  OVMS_DIAG_SYNTH_MANUAL,
  OVMS_DIAG_SYNTH_START_FAILED,
  };

enum ovms_diag_vfs_op_t : uint32_t
  {
  OVMS_DIAG_VFS_IDLE = 0,
  OVMS_DIAG_VFS_FORMAT,
  OVMS_DIAG_VFS_BATCH_FLUSH,
  OVMS_DIAG_VFS_FWRITE,
  OVMS_DIAG_VFS_FFLUSH,
  OVMS_DIAG_VFS_FSYNC,
  };

enum ovms_diag_vfs_owner_result_t : uint32_t
  {
  OVMS_DIAG_VFS_OWNER_NONE = 0,
  OVMS_DIAG_VFS_OWNER_ATTEMPTING,
  OVMS_DIAG_VFS_OWNER_SUCCESS,
  OVMS_DIAG_VFS_OWNER_FAILED,
  };

enum ovms_diag_vfs_lifecycle_stage_t : uint32_t
  {
  OVMS_DIAG_VFS_LIFECYCLE_OPEN_BASELINE = 0,
  OVMS_DIAG_VFS_LIFECYCLE_QUEUES_READY,
  OVMS_DIAG_VFS_LIFECYCLE_TASK_READY,
  OVMS_DIAG_VFS_LIFECYCLE_FOPEN_READY,
  OVMS_DIAG_VFS_LIFECYCLE_OWNER_READY,
  OVMS_DIAG_VFS_LIFECYCLE_FULLY_OPEN,
  OVMS_DIAG_VFS_LIFECYCLE_PRE_CLOSE,
  OVMS_DIAG_VFS_LIFECYCLE_POST_CLOSE_FREE,
  OVMS_DIAG_VFS_LIFECYCLE_POST_STOP,
  OVMS_DIAG_VFS_LIFECYCLE_DESTRUCTION,
  OVMS_DIAG_VFS_LIFECYCLE_COUNT,
  };

enum ovms_diag_vfs_sync_state_t : uint32_t
  {
  OVMS_DIAG_VFS_SYNC_DIRTY = 1 << 0,
  OVMS_DIAG_VFS_SYNC_PERIODIC = 1 << 1,
  OVMS_DIAG_VFS_SYNC_DUE = 1 << 2,
  OVMS_DIAG_VFS_SYNC_RUNNING = 1 << 3,
  };

enum ovms_diag_vfs_write_error_t : uint32_t
  {
  OVMS_DIAG_VFS_WRITE_SHORT = 1 << 0,
  OVMS_DIAG_VFS_WRITE_ERRNO = 1 << 1,
  OVMS_DIAG_VFS_WRITE_FERROR = 1 << 2,
  OVMS_DIAG_VFS_WRITE_STORAGE = 1 << 3,
  };

typedef struct
  {
  // sequence is published last and identifies the open/close cycle.
  uint32_t sequence;
  uint32_t monotonic_ms;
  uint32_t dma_free;
  uint32_t dma_largest;
  } ovms_diag_vfs_heap_stage_t;

typedef struct
  {
  // Even guard values identify stable snapshots; odd means a writer owns the
  // entry. All remaining fields are fixed-width and are written atomically.
  uint32_t guard;
  uint32_t sequence;
  uint32_t source;
  uint32_t requested;
  uint32_t monotonic_ms;
  uint32_t internal_free;
  uint32_t internal_largest;
  uint32_t dma_free;
  uint32_t dma_largest;
  uint32_t spiram_free;
  uint32_t spiram_largest;
  } ovms_diag_alloc_entry_t;

typedef struct
  {
  uint32_t version;
  uint32_t panic_snapshot;

  uint32_t synth_state;
  uint32_t synth_heartbeat;
  uint32_t synth_generated;
  uint32_t synth_injected;
  uint32_t synth_rejected;
  uint32_t synth_last_ms;
  uint32_t synth_stopped_ms;

  uint32_t vfs_active;
  uint32_t vfs_heartbeat;
  uint32_t vfs_primary_enqueue;
  uint32_t vfs_overflow_enqueue;
  uint32_t vfs_overflow_drop;
  uint32_t vfs_dequeue;
  uint32_t vfs_format_complete;
  uint32_t vfs_batch_start;
  uint32_t vfs_batch_end;
  uint32_t vfs_fwrite_start;
  uint32_t vfs_fwrite_end;
  uint32_t vfs_fflush_start;
  uint32_t vfs_fflush_end;
  uint32_t vfs_fsync_start;
  uint32_t vfs_fsync_end;
  uint32_t vfs_op;
  uint32_t vfs_parent_op;
  uint32_t vfs_op_sequence;
  uint32_t vfs_op_started_ms;
  uint32_t vfs_op_completed_ms;
  uint32_t vfs_storage_error;

  uint32_t heap_sample_ms;
  uint32_t heap_internal_free;
  uint32_t heap_internal_largest;
  uint32_t heap_internal_minimum;
  uint32_t heap_dma_free;
  uint32_t heap_dma_largest;
  uint32_t heap_dma_minimum;

  uint32_t vfs_queues_ready_free;
  uint32_t vfs_queues_ready_largest;
  uint32_t vfs_task_ready_free;
  uint32_t vfs_task_ready_largest;

  uint32_t vfs_owner_sequence;
  uint32_t vfs_owner_requested;
  uint32_t vfs_owner_caps;
  uint32_t vfs_owner_attempt_ms;
  uint32_t vfs_owner_before_free;
  uint32_t vfs_owner_before_largest;
  uint32_t vfs_owner_after_free;
  uint32_t vfs_owner_after_largest;
  uint32_t vfs_owner_result;

  uint32_t vfs_lifecycle_sequence;
  uint32_t vfs_lifecycle_stage;
  ovms_diag_vfs_heap_stage_t vfs_lifecycle[OVMS_DIAG_VFS_LIFECYCLE_COUNT];

  uint32_t vfs_slow_write_threshold_us;
  uint32_t vfs_slow_write_sequence;
  uint32_t vfs_slow_write_requested;
  uint32_t vfs_slow_write_accepted;
  uint32_t vfs_slow_write_file_offset;
  uint32_t vfs_slow_write_cluster_offset;
  uint32_t vfs_slow_write_batch_used;
  uint32_t vfs_slow_write_batch_capacity;
  uint32_t vfs_slow_write_elapsed_us;
  uint32_t vfs_slow_write_sync_due_ms;
  uint32_t vfs_slow_write_sync_state;
  uint32_t vfs_slow_write_primary_queued;
  uint32_t vfs_slow_write_overflow_queued;
  uint32_t vfs_slow_write_error_state;
  int32_t vfs_slow_write_errno;
  int32_t vfs_slow_write_ferror;

  uint32_t vfs_fflush_last_us;
  uint32_t vfs_fflush_max_us;
  int32_t vfs_fflush_result;
  uint32_t vfs_fsync_last_us;
  uint32_t vfs_fsync_max_us;
  int32_t vfs_fsync_result;

  uint32_t alloc_failure_count;
  ovms_diag_alloc_entry_t alloc_failure_first;
  ovms_diag_alloc_entry_t alloc_failure_latest;

  uint32_t netman_heartbeat;
  uint32_t netman_last_ms;
  uint32_t net_restart_requests;
  uint32_t net_restart_executes;
  uint32_t net_restart_requested_ms;
  uint32_t net_restart_executed_ms;
  uint32_t net_restart_text_sequence;
  char net_restart_source[16];
  char net_restart_reason[32];
  int32_t wifi_storage_result;
  uint32_t wifi_storage_ms;
  } ovms_diag_state_t;

static_assert(sizeof(ovms_diag_alloc_entry_t) == 44,
  "allocation diagnostic entry footprint changed");
static_assert(sizeof(ovms_diag_vfs_heap_stage_t) == 16,
  "VFS lifecycle heap stage footprint changed");
static_assert(sizeof(ovms_diag_state_t) == 632,
  "diagnostic state footprint changed; review permanent DRAM/RTC cost");

extern ovms_diag_state_t ovms_diag_live;

inline uint32_t OvmsDiagLoad(const uint32_t* field)
  { return __atomic_load_n(field, __ATOMIC_RELAXED); }
inline int32_t OvmsDiagLoad(const int32_t* field)
  { return __atomic_load_n(field, __ATOMIC_RELAXED); }
inline void OvmsDiagStore(uint32_t* field, uint32_t value)
  { __atomic_store_n(field, value, __ATOMIC_RELAXED); }
inline void OvmsDiagStore(int32_t* field, int32_t value)
  { __atomic_store_n(field, value, __ATOMIC_RELAXED); }
inline uint32_t OvmsDiagIncrement(uint32_t* field)
  { return __atomic_add_fetch(field, 1, __ATOMIC_RELAXED); }

typedef struct
  {
  // data consistency:
  uint32_t crc;
  uint32_t calc_crc() { return crc32_le(0, (uint8_t*)this+sizeof(crc), sizeof(*this)-sizeof(crc)); }

  // payload:
  unsigned int boot_count;          // Number of times system has rebooted (not power on)
  RESET_REASON bootreason_cpu0;     // Reason for last boot on CPU#0
  RESET_REASON bootreason_cpu1;     // Reason for last boot on CPU#1
  float adc1_factor;                // 12V battery ADC calibration factor
  float min_12v_level;              // 12V battery minimum voltage level to allow boot
  int wakeup_interval;              // Wakeup interval in seconds for 12V restoration check
  bool soft_reset;                  // true = user requested reset ("module reset")
  bool firmware_update;             // true = firmware update restart
  bool partition_update;            // true = partition update restart
  bool stable_reached;              // true = system has reached stable state (see housekeeping)
  unsigned int crash_count_early;   // Number of times system has crashed before reaching stable state
  unsigned int crash_count_total;   // Total number of times system has crashed since power on
  crash_data_t crash_data;          // Register dump & backtrace info
  esp_reset_reason_t reset_hint;    // Copy of RTC_RESET_CAUSE_REG
  char curr_event_name[32];         // Copy of MyEvents.m_current_event
  char curr_event_handler[16];      // … MyEvents.m_current_callback->m_caller
  uint16_t curr_event_runtime;      // … monotonictime-MyEvents.m_current_started
  char wdt_tasknames[32];           // Pipe (|) separated list of the tasks that triggered the TWDT
  bool stack_overflow;
  char stack_overflow_taskname[16];
  task_info_t curr_task[portNUM_PROCESSORS];
  bool heap_corruption;
  ovms_diag_state_t diag;
  } boot_data_t;

extern boot_data_t boot_data;

class Boot
  {
  public:
    Boot();
    virtual ~Boot();
    void Init();

  public:
    bootreason_t GetBootReason() { return m_bootreason; }
    const char* GetBootReasonName();
    esp_reset_reason_t GetResetReason() { return m_resetreason; }
    const char* GetResetReasonName();
    unsigned int GetCrashCount() { return boot_data.crash_count_total; }
    unsigned int GetEarlyCrashCount() { return m_crash_count_early; }
    bool GetStable() { return boot_data.stable_reached; }
    void SetStable();
    bool GetHeapCorruption() { return boot_data.heap_corruption; }
    void SetHeapCorruption();

  public:
    void SetSoftReset();
    void SetFirmwareUpdate();
    void SetPartitionUpdate();
    void SetMin12VLevel(float min_12v_level);
    float GetMin12VLevel() { return boot_data.min_12v_level; }
    void Restart(bool hard=false);
    void DeepSleep(unsigned int seconds = 60);
    void DeepSleep(time_t waketime);
    void ShutdownPending(const char* tag);
    void ShutdownReady(const char* tag);
    bool IsShuttingDown();
    void Ticker1(std::string event, void* data);
    void UpdateConfig(std::string event, void* data);

  public:
    OvmsMutex m_shutdown_mutex;
    unsigned int m_shutdown_timer;
    unsigned int m_shutdown_pending;
    bool m_shutdown_deepsleep;
    unsigned int m_shutdown_deepsleep_seconds;
    time_t m_shutdown_deepsleep_waketime;
    bool m_shutting_down;
    bool m_min_12v_level_override;

  public:
#if ESP_IDF_VERSION_MAJOR < 4
    static void ErrorCallback(XtExcFrame *f, int core_id, bool is_abort);
#else
    static void ErrorCallback(const void *f, int core_id, bool is_abort, esp_reset_reason_t reset_hint);
#endif
    void NotifyDebugCrash();

  protected:
    bootreason_t m_bootreason;
    esp_reset_reason_t m_resetreason;
    unsigned int m_crash_count_early;
    bool m_stack_overflow;
    bool m_heap_corruption;
  };

extern Boot MyBoot;

void OnBoot();

#endif //#ifndef __OVMS_BOOT_H__
