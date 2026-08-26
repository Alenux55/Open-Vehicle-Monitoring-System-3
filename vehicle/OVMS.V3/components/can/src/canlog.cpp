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
static const char *TAG = "canlog";

#include "can.h"
#include "canlog.h"
#include <sys/param.h>
#include <algorithm>
#include <ctype.h>
#include <stdarg.h>
#include <string.h>
#include <sys/time.h>
#include <string>
#include <sstream>
#include <iomanip>
#include "ovms_utils.h"
#include "ovms_config.h"
#include "ovms_command.h"
#include "ovms_events.h"
#include "ovms_boot.h"
#include "ovms_peripherals.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "metrics_standard.h"

static const char *CAN_PARAM = "can";

////////////////////////////////////////////////////////////////////////
// Command Processing
////////////////////////////////////////////////////////////////////////

void can_log_stop(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  if (!MyCan.HasLogger())
    {
    writer->puts("Error: No loggers running");
    return;
    }

  if (argc==0)
    {
    // Stop all loggers
    writer->puts("Stopping all loggers");
    MyCan.RemoveLoggers();
    return;
    }

  if (MyCan.RemoveLogger(atoi(argv[0])))
    {
    writer->puts("Stopped logger");
    }
  else
    {
    writer->puts("Error: Cannot find specified logger");
    }
  }

void can_log_status(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  if (!MyCan.HasLogger())
    {
    writer->puts("CAN logging inactive");
    return;
    }

  if (argc>0)
    {
    canlog* cl = MyCan.GetLogger(atoi(argv[0]));
    if (cl)
      {
      writer->printf("#%d: %s %s %s",
        atoi(argv[0]),
        (cl->m_isopen)?"open":"closed",
        cl->GetInfo().c_str(), cl->GetStats().c_str());
      #ifdef CONFIG_OVMS_SC_GPL_MONGOOSE
      if (cl->m_connmap.size() > 0)
        {
        OvmsRecMutexLock lock(&cl->m_cmmutex);
        for (canlog::conn_map_t::iterator it=cl->m_connmap.begin(); it!=cl->m_connmap.end(); ++it)
          {
          canlogconnection* clc = it->second;
          writer->printf("  %s: %s %s%s%s\n",
            clc->GetSummary().c_str(),
            (clc->m_ispaused)?"paused":"running",
            (clc->m_filters != NULL)?" filter:":"",
            (clc->m_filters != NULL)?clc->m_filters->Info().c_str():"",
            clc->GetStats().c_str());
          }
        }
      #endif //#ifdef CONFIG_OVMS_SC_GPL_MONGOOSE
      }
    else
      {
      writer->puts("Error: Cannot find specified can logger");
      }
    return;
    }
  else
    {
    // Show the status of all loggers
    OvmsRecMutexLock lock(&MyCan.m_loggermap_mutex);
    for (can::canlog_map_t::iterator it=MyCan.m_loggermap.begin(); it!=MyCan.m_loggermap.end(); ++it)
      {
      canlog* cl = it->second;
      writer->printf("#%" PRId32 ": %s %s %s\n",
        it->first,
        (cl->m_isopen)?"open":"closed",
        cl->GetInfo().c_str(), cl->GetStats().c_str());
      #ifdef CONFIG_OVMS_SC_GPL_MONGOOSE
      if (cl->m_connmap.size() > 0)
        {
        OvmsRecMutexLock lock(&cl->m_cmmutex);
        for (canlog::conn_map_t::iterator it=cl->m_connmap.begin(); it!=cl->m_connmap.end(); ++it)
          {
          canlogconnection* clc = it->second;
          writer->printf("  %s: %s %s%s%s\n",
            clc->GetSummary().c_str(),
            (clc->m_ispaused)?"paused":"running",
            (clc->m_filters != NULL)?" filter:":"",
            (clc->m_filters != NULL)?clc->m_filters->Info().c_str():"",
            clc->GetStats().c_str());
          }
        }
      #endif //#ifdef CONFIG_OVMS_SC_GPL_MONGOOSE
      }
    }
  }

static void can_log_health_line(OvmsWriter* writer, const char* format, ...)
  {
  char line[384];
  va_list args;
  va_start(args, format);
  int length = vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  if (length < 0)
    return;
  size_t output = std::min(static_cast<size_t>(length), sizeof(line) - 1);
  writer->write(line, output);
  }

static const char* const can_log_vfs_lifecycle_names[
  OVMS_DIAG_VFS_LIFECYCLE_COUNT] =
  {
  "open_baseline",
  "queues_ready",
  "task_ready",
  "fopen_ready",
  "owner_ready",
  "fully_open",
  "pre_close",
  "post_close_free",
  "post_stop",
  "destruction",
  };

void can_log_heap(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  // Sample everything before writing so console output cannot affect the
  // values being reported. Formatting uses only the fixed stack buffer in
  // can_log_health_line().
  const uint32_t internal_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
  const uint32_t dma_caps = MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
  const uint32_t spiram_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
  uint32_t internal_free = static_cast<uint32_t>(
    heap_caps_get_free_size(internal_caps));
  uint32_t internal_largest = static_cast<uint32_t>(
    heap_caps_get_largest_free_block(internal_caps));
  uint32_t internal_minimum = static_cast<uint32_t>(
    heap_caps_get_minimum_free_size(internal_caps));
  uint32_t dma_free = static_cast<uint32_t>(heap_caps_get_free_size(dma_caps));
  uint32_t dma_largest = static_cast<uint32_t>(
    heap_caps_get_largest_free_block(dma_caps));
  uint32_t dma_minimum = static_cast<uint32_t>(
    heap_caps_get_minimum_free_size(dma_caps));
  uint32_t spiram_free = static_cast<uint32_t>(
    heap_caps_get_free_size(spiram_caps));
  uint32_t spiram_largest = static_cast<uint32_t>(
    heap_caps_get_largest_free_block(spiram_caps));

  can_log_health_line(writer,
    "heap_now free:%u largest:%u minimum:%u dma_free:%u dma_largest:%u dma_minimum:%u spiram_free:%u spiram_largest:%u\n",
    internal_free, internal_largest, internal_minimum,
    dma_free, dma_largest, dma_minimum, spiram_free, spiram_largest);
  }

static bool can_log_health_alloc_entry(const ovms_diag_alloc_entry_t& source,
  ovms_diag_alloc_entry_t& snapshot)
  {
  memset(&snapshot, 0, sizeof(snapshot));
  for (int attempt = 0; attempt < 3; ++attempt)
    {
    uint32_t before = __atomic_load_n(&source.guard, __ATOMIC_ACQUIRE);
    if (before & 1)
      continue;
    snapshot.guard = before;
    snapshot.sequence = OvmsDiagLoad(&source.sequence);
    snapshot.source = OvmsDiagLoad(&source.source);
    snapshot.requested = OvmsDiagLoad(&source.requested);
    snapshot.monotonic_ms = OvmsDiagLoad(&source.monotonic_ms);
    snapshot.internal_free = OvmsDiagLoad(&source.internal_free);
    snapshot.internal_largest = OvmsDiagLoad(&source.internal_largest);
    snapshot.dma_free = OvmsDiagLoad(&source.dma_free);
    snapshot.dma_largest = OvmsDiagLoad(&source.dma_largest);
    snapshot.spiram_free = OvmsDiagLoad(&source.spiram_free);
    snapshot.spiram_largest = OvmsDiagLoad(&source.spiram_largest);
    uint32_t after = __atomic_load_n(&source.guard, __ATOMIC_ACQUIRE);
    if (before == after && !(after & 1))
      return snapshot.source != OVMS_DIAG_ALLOC_NONE;
    }
  memset(&snapshot, 0, sizeof(snapshot));
  return false;
  }

void can_log_health(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  // Intentionally no logger-map, CanSynth, VFS, route, formatter or stats
  // locks, and no dynamic strings. These are relaxed atomic snapshots; fields
  // can advance between loads, which is preferable to blocking a hot path.
  struct timeval wall;
  gettimeofday(&wall, NULL);
  int64_t mono_us = esp_timer_get_time();

  can_log_health_line(writer,
    "clock wall_sec:%lld wall_usec:%ld mono_us:%lld\n",
    static_cast<long long>(wall.tv_sec), static_cast<long>(wall.tv_usec),
    static_cast<long long>(mono_us));
  can_log_health_line(writer,
    "synth state:%u hb:%u gen:%u inj:%u rej:%u last_ms:%u stop_ms:%u\n",
    OvmsDiagLoad(&ovms_diag_live.synth_state),
    OvmsDiagLoad(&ovms_diag_live.synth_heartbeat),
    OvmsDiagLoad(&ovms_diag_live.synth_generated),
    OvmsDiagLoad(&ovms_diag_live.synth_injected),
    OvmsDiagLoad(&ovms_diag_live.synth_rejected),
    OvmsDiagLoad(&ovms_diag_live.synth_last_ms),
    OvmsDiagLoad(&ovms_diag_live.synth_stopped_ms));
  can_log_health_line(writer,
    "vfs active:%u hb:%u primary:%u overflow:%u odrop:%u dequeue:%u format:%u op:%u parent:%u seq:%u start_ms:%u end_ms:%u storage:%u\n",
    OvmsDiagLoad(&ovms_diag_live.vfs_active),
    OvmsDiagLoad(&ovms_diag_live.vfs_heartbeat),
    OvmsDiagLoad(&ovms_diag_live.vfs_primary_enqueue),
    OvmsDiagLoad(&ovms_diag_live.vfs_overflow_enqueue),
    OvmsDiagLoad(&ovms_diag_live.vfs_overflow_drop),
    OvmsDiagLoad(&ovms_diag_live.vfs_dequeue),
    OvmsDiagLoad(&ovms_diag_live.vfs_format_complete),
    OvmsDiagLoad(&ovms_diag_live.vfs_op),
    OvmsDiagLoad(&ovms_diag_live.vfs_parent_op),
    OvmsDiagLoad(&ovms_diag_live.vfs_op_sequence),
    OvmsDiagLoad(&ovms_diag_live.vfs_op_started_ms),
    OvmsDiagLoad(&ovms_diag_live.vfs_op_completed_ms),
    OvmsDiagLoad(&ovms_diag_live.vfs_storage_error));
  can_log_health_line(writer,
    "io batch:%u/%u fwrite:%u/%u fflush:%u/%u fsync:%u/%u\n",
    OvmsDiagLoad(&ovms_diag_live.vfs_batch_start),
    OvmsDiagLoad(&ovms_diag_live.vfs_batch_end),
    OvmsDiagLoad(&ovms_diag_live.vfs_fwrite_start),
    OvmsDiagLoad(&ovms_diag_live.vfs_fwrite_end),
    OvmsDiagLoad(&ovms_diag_live.vfs_fflush_start),
    OvmsDiagLoad(&ovms_diag_live.vfs_fflush_end),
    OvmsDiagLoad(&ovms_diag_live.vfs_fsync_start),
    OvmsDiagLoad(&ovms_diag_live.vfs_fsync_end));
  can_log_health_line(writer,
    "heap sample_ms:%u free:%u largest:%u minimum:%u dma_free:%u dma_largest:%u dma_minimum:%u\n",
    OvmsDiagLoad(&ovms_diag_live.heap_sample_ms),
    OvmsDiagLoad(&ovms_diag_live.heap_internal_free),
    OvmsDiagLoad(&ovms_diag_live.heap_internal_largest),
    OvmsDiagLoad(&ovms_diag_live.heap_internal_minimum),
    OvmsDiagLoad(&ovms_diag_live.heap_dma_free),
    OvmsDiagLoad(&ovms_diag_live.heap_dma_largest),
    OvmsDiagLoad(&ovms_diag_live.heap_dma_minimum));
  can_log_health_line(writer,
    "vfs_stage queues_free:%u queues_largest:%u task_free:%u task_largest:%u\n",
    OvmsDiagLoad(&ovms_diag_live.vfs_queues_ready_free),
    OvmsDiagLoad(&ovms_diag_live.vfs_queues_ready_largest),
    OvmsDiagLoad(&ovms_diag_live.vfs_task_ready_free),
    OvmsDiagLoad(&ovms_diag_live.vfs_task_ready_largest));
  can_log_health_line(writer,
    "vfs_owner seq:%u result:%u requested:%u caps:%u attempt_ms:%u before_free:%u before_largest:%u after_free:%u after_largest:%u\n",
    OvmsDiagLoad(&ovms_diag_live.vfs_owner_sequence),
    __atomic_load_n(&ovms_diag_live.vfs_owner_result, __ATOMIC_ACQUIRE),
    OvmsDiagLoad(&ovms_diag_live.vfs_owner_requested),
    OvmsDiagLoad(&ovms_diag_live.vfs_owner_caps),
    OvmsDiagLoad(&ovms_diag_live.vfs_owner_attempt_ms),
    OvmsDiagLoad(&ovms_diag_live.vfs_owner_before_free),
    OvmsDiagLoad(&ovms_diag_live.vfs_owner_before_largest),
    OvmsDiagLoad(&ovms_diag_live.vfs_owner_after_free),
    OvmsDiagLoad(&ovms_diag_live.vfs_owner_after_largest));
  can_log_health_line(writer,
    "vfs_lifecycle open_seq:%u stage:%u\n",
    OvmsDiagLoad(&ovms_diag_live.vfs_lifecycle_sequence),
    OvmsDiagLoad(&ovms_diag_live.vfs_lifecycle_stage));
  for (size_t i = 0; i < OVMS_DIAG_VFS_LIFECYCLE_COUNT; ++i)
    {
    ovms_diag_vfs_heap_stage_t stage;
    bool stage_valid = OvmsDiagReadVfsHeapStage(
      ovms_diag_live.vfs_lifecycle[i], stage);
    can_log_health_line(writer,
      "vfs_heap stage:%s seq:%u ms:%u dma_free:%u dma_largest:%u\n",
      can_log_vfs_lifecycle_names[i],
      stage_valid ? stage.sequence : 0U,
      stage_valid ? stage.monotonic_ms : 0U,
      stage_valid ? stage.dma_free : 0U,
      stage_valid ? stage.dma_largest : 0U);
    }
  ovms_diag_vfs_slow_write_snapshot_t slow_write;
  bool slow_write_valid = OvmsDiagReadVfsSlowWrite(ovms_diag_live, slow_write);
  can_log_health_line(writer,
    "slow_write threshold_us:%u seq:%u requested:%u accepted:%u file_offset:%u cluster_offset:%u batch_used:%u batch_capacity:%u elapsed_us:%u sync_due_ms:%u sync_state:%u primary:%u overflow:%u error:%u errno:%d ferror:%d\n",
    OvmsDiagLoad(&ovms_diag_live.vfs_slow_write_threshold_us),
    slow_write_valid ? slow_write.sequence : 0U,
    slow_write_valid ? slow_write.requested : 0U,
    slow_write_valid ? slow_write.accepted : 0U,
    slow_write_valid ? slow_write.file_offset : 0U,
    slow_write_valid ? slow_write.cluster_offset : 0U,
    slow_write_valid ? slow_write.batch_used : 0U,
    slow_write_valid ? slow_write.batch_capacity : 0U,
    slow_write_valid ? slow_write.elapsed_us : 0U,
    slow_write_valid ? slow_write.sync_due_ms : 0U,
    slow_write_valid ? slow_write.sync_state : 0U,
    slow_write_valid ? slow_write.primary_queued : 0U,
    slow_write_valid ? slow_write.overflow_queued : 0U,
    slow_write_valid ? slow_write.error_state : 0U,
    slow_write_valid ? slow_write.error_no : 0,
    slow_write_valid ? slow_write.file_error : 0);
  can_log_health_line(writer,
    "sync_io fflush_last_us:%u fflush_max_us:%u fflush_result:%d fsync_last_us:%u fsync_max_us:%u fsync_result:%d\n",
    OvmsDiagLoad(&ovms_diag_live.vfs_fflush_last_us),
    OvmsDiagLoad(&ovms_diag_live.vfs_fflush_max_us),
    OvmsDiagLoad(&ovms_diag_live.vfs_fflush_result),
    OvmsDiagLoad(&ovms_diag_live.vfs_fsync_last_us),
    OvmsDiagLoad(&ovms_diag_live.vfs_fsync_max_us),
    OvmsDiagLoad(&ovms_diag_live.vfs_fsync_result));

  ovms_diag_alloc_entry_t alloc_first;
  ovms_diag_alloc_entry_t alloc_latest;
  bool alloc_first_valid = can_log_health_alloc_entry(
    ovms_diag_live.alloc_failure_first, alloc_first);
  bool alloc_latest_valid = can_log_health_alloc_entry(
    ovms_diag_live.alloc_failure_latest, alloc_latest);
  uint32_t alloc_total = OvmsDiagLoad(&ovms_diag_live.alloc_failure_count);
  can_log_health_line(writer,
    "alloc_first valid:%u total:%u seq:%u source:%u requested:%u ms:%u ifree:%u ilargest:%u dfree:%u dlargest:%u sfree:%u slargest:%u\n",
    alloc_first_valid ? 1U : 0U, alloc_total, alloc_first.sequence,
    alloc_first.source, alloc_first.requested, alloc_first.monotonic_ms,
    alloc_first.internal_free, alloc_first.internal_largest,
    alloc_first.dma_free, alloc_first.dma_largest, alloc_first.spiram_free,
    alloc_first.spiram_largest);
  can_log_health_line(writer,
    "alloc_latest valid:%u total:%u seq:%u source:%u requested:%u ms:%u ifree:%u ilargest:%u dfree:%u dlargest:%u sfree:%u slargest:%u\n",
    alloc_latest_valid ? 1U : 0U, alloc_total, alloc_latest.sequence,
    alloc_latest.source, alloc_latest.requested, alloc_latest.monotonic_ms,
    alloc_latest.internal_free, alloc_latest.internal_largest,
    alloc_latest.dma_free, alloc_latest.dma_largest, alloc_latest.spiram_free,
    alloc_latest.spiram_largest);

  char source[sizeof(ovms_diag_live.net_restart_source)];
  char reason[sizeof(ovms_diag_live.net_restart_reason)];
  uint32_t before = 0, after = 0;
  for (int attempt = 0; attempt < 3; ++attempt)
    {
    before = __atomic_load_n(&ovms_diag_live.net_restart_text_sequence, __ATOMIC_ACQUIRE);
    if (before & 1)
      continue;
    memcpy(source, ovms_diag_live.net_restart_source, sizeof(source));
    memcpy(reason, ovms_diag_live.net_restart_reason, sizeof(reason));
    after = __atomic_load_n(&ovms_diag_live.net_restart_text_sequence, __ATOMIC_ACQUIRE);
    if (before == after)
      break;
    }
  source[sizeof(source) - 1] = 0;
  reason[sizeof(reason) - 1] = 0;
  if (before != after || (after & 1))
    {
    strlcpy(source, "updating", sizeof(source));
    strlcpy(reason, "updating", sizeof(reason));
    }
  else
    {
    if (!source[0])
      strlcpy(source, "-", sizeof(source));
    if (!reason[0])
      strlcpy(reason, "-", sizeof(reason));
    }
  can_log_health_line(writer,
    "net hb:%u last_ms:%u requests:%u executes:%u request_ms:%u execute_ms:%u source:%s reason:%s wifi_storage:%d wifi_ms:%u\n",
    OvmsDiagLoad(&ovms_diag_live.netman_heartbeat),
    OvmsDiagLoad(&ovms_diag_live.netman_last_ms),
    OvmsDiagLoad(&ovms_diag_live.net_restart_requests),
    OvmsDiagLoad(&ovms_diag_live.net_restart_executes),
    OvmsDiagLoad(&ovms_diag_live.net_restart_requested_ms),
    OvmsDiagLoad(&ovms_diag_live.net_restart_executed_ms), source, reason,
    OvmsDiagLoad(&ovms_diag_live.wifi_storage_result),
    OvmsDiagLoad(&ovms_diag_live.wifi_storage_ms));

  const ovms_diag_state_t& prior = boot_data.diag;
  uint32_t previous_valid =
    prior.version == 4 && prior.panic_snapshot == 1 ? 1U : 0U;
  can_log_health_line(writer,
    "previous valid:%u synth:%u/%u/%u/%u vfs:%u/%u/%u op:%u parent:%u seq:%u io:%u/%u,%u/%u,%u/%u,%u/%u heap:%u/%u/%u net:%u/%u wifi:%d\n",
    previous_valid,
    prior.synth_state, prior.synth_generated, prior.synth_injected,
    prior.synth_rejected, prior.vfs_heartbeat, prior.vfs_dequeue,
    prior.vfs_format_complete, prior.vfs_op, prior.vfs_parent_op,
    prior.vfs_op_sequence, prior.vfs_batch_start, prior.vfs_batch_end,
    prior.vfs_fwrite_start, prior.vfs_fwrite_end, prior.vfs_fflush_start,
    prior.vfs_fflush_end, prior.vfs_fsync_start, prior.vfs_fsync_end,
    prior.heap_internal_free, prior.heap_internal_largest,
    prior.heap_internal_minimum, prior.net_restart_requests,
    prior.net_restart_executes, prior.wifi_storage_result);
  can_log_health_line(writer,
    "previous_heap valid:%u sample_ms:%u free:%u largest:%u minimum:%u dma_free:%u dma_largest:%u dma_minimum:%u\n",
    previous_valid, prior.heap_sample_ms, prior.heap_internal_free,
    prior.heap_internal_largest, prior.heap_internal_minimum,
    prior.heap_dma_free, prior.heap_dma_largest, prior.heap_dma_minimum);
  can_log_health_line(writer,
    "previous_stage valid:%u queues_free:%u queues_largest:%u task_free:%u task_largest:%u\n",
    previous_valid, prior.vfs_queues_ready_free,
    prior.vfs_queues_ready_largest, prior.vfs_task_ready_free,
    prior.vfs_task_ready_largest);
  can_log_health_line(writer,
    "previous_owner valid:%u seq:%u result:%u requested:%u caps:%u attempt_ms:%u before_free:%u before_largest:%u after_free:%u after_largest:%u\n",
    previous_valid, prior.vfs_owner_sequence, prior.vfs_owner_result,
    prior.vfs_owner_requested, prior.vfs_owner_caps,
    prior.vfs_owner_attempt_ms, prior.vfs_owner_before_free,
    prior.vfs_owner_before_largest, prior.vfs_owner_after_free,
    prior.vfs_owner_after_largest);
  can_log_health_line(writer,
    "previous_lifecycle valid:%u open_seq:%u stage:%u\n",
    previous_valid, prior.vfs_lifecycle_sequence, prior.vfs_lifecycle_stage);
  for (size_t i = 0; i < OVMS_DIAG_VFS_LIFECYCLE_COUNT; ++i)
    {
    const ovms_diag_vfs_heap_stage_t& stage = prior.vfs_lifecycle[i];
    uint32_t encoded = stage.sequence;
    uint32_t decoded = (encoded & 1) ? 0 : encoded / 2;
    can_log_health_line(writer,
      "previous_vfs_heap valid:%u stage:%s seq:%u ms:%u dma_free:%u dma_largest:%u\n",
      previous_valid, can_log_vfs_lifecycle_names[i],
      previous_valid ? decoded : 0U,
      previous_valid ? stage.monotonic_ms : 0U,
      previous_valid ? stage.dma_free : 0U,
      previous_valid ? stage.dma_largest : 0U);
    }
  uint32_t previous_slow_encoded = prior.vfs_slow_write_sequence;
  uint32_t previous_slow_seq =
    (previous_slow_encoded & 1) ? 0 : previous_slow_encoded / 2;
  can_log_health_line(writer,
    "previous_slow_write valid:%u threshold_us:%u seq:%u requested:%u accepted:%u file_offset:%u cluster_offset:%u batch_used:%u batch_capacity:%u elapsed_us:%u sync_due_ms:%u sync_state:%u primary:%u overflow:%u error:%u errno:%d ferror:%d\n",
    previous_valid, prior.vfs_slow_write_threshold_us,
    previous_valid ? previous_slow_seq : 0U,
    previous_valid ? prior.vfs_slow_write_requested : 0U,
    previous_valid ? prior.vfs_slow_write_accepted : 0U,
    previous_valid ? prior.vfs_slow_write_file_offset : 0U,
    previous_valid ? prior.vfs_slow_write_cluster_offset : 0U,
    previous_valid ? prior.vfs_slow_write_batch_used : 0U,
    previous_valid ? prior.vfs_slow_write_batch_capacity : 0U,
    previous_valid ? prior.vfs_slow_write_elapsed_us : 0U,
    previous_valid ? prior.vfs_slow_write_sync_due_ms : 0U,
    previous_valid ? prior.vfs_slow_write_sync_state : 0U,
    previous_valid ? prior.vfs_slow_write_primary_queued : 0U,
    previous_valid ? prior.vfs_slow_write_overflow_queued : 0U,
    previous_valid ? prior.vfs_slow_write_error_state : 0U,
    previous_valid ? prior.vfs_slow_write_errno : 0,
    previous_valid ? prior.vfs_slow_write_ferror : 0);
  can_log_health_line(writer,
    "previous_sync_io valid:%u fflush_last_us:%u fflush_max_us:%u fflush_result:%d fsync_last_us:%u fsync_max_us:%u fsync_result:%d\n",
    previous_valid, prior.vfs_fflush_last_us, prior.vfs_fflush_max_us,
    prior.vfs_fflush_result, prior.vfs_fsync_last_us,
    prior.vfs_fsync_max_us, prior.vfs_fsync_result);
  can_log_health_line(writer,
    "previous_alloc_first valid:%u total:%u seq:%u source:%u requested:%u ms:%u ifree:%u ilargest:%u dfree:%u dlargest:%u sfree:%u slargest:%u\n",
    previous_valid && prior.alloc_failure_first.source != OVMS_DIAG_ALLOC_NONE
      ? 1U : 0U,
    prior.alloc_failure_count, prior.alloc_failure_first.sequence,
    prior.alloc_failure_first.source, prior.alloc_failure_first.requested,
    prior.alloc_failure_first.monotonic_ms,
    prior.alloc_failure_first.internal_free,
    prior.alloc_failure_first.internal_largest,
    prior.alloc_failure_first.dma_free, prior.alloc_failure_first.dma_largest,
    prior.alloc_failure_first.spiram_free,
    prior.alloc_failure_first.spiram_largest);
  can_log_health_line(writer,
    "previous_alloc_latest valid:%u total:%u seq:%u source:%u requested:%u ms:%u ifree:%u ilargest:%u dfree:%u dlargest:%u sfree:%u slargest:%u\n",
    previous_valid && prior.alloc_failure_latest.source != OVMS_DIAG_ALLOC_NONE
      ? 1U : 0U,
    prior.alloc_failure_count, prior.alloc_failure_latest.sequence,
    prior.alloc_failure_latest.source, prior.alloc_failure_latest.requested,
    prior.alloc_failure_latest.monotonic_ms,
    prior.alloc_failure_latest.internal_free,
    prior.alloc_failure_latest.internal_largest,
    prior.alloc_failure_latest.dma_free,
    prior.alloc_failure_latest.dma_largest,
    prior.alloc_failure_latest.spiram_free,
    prior.alloc_failure_latest.spiram_largest);
  }

void can_log_list(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  if (!MyCan.HasLogger())
    {
    writer->puts("CAN logging inactive");
    return;
    }
  else
    {
    // Show the list of all loggers
    OvmsRecMutexLock lock(&MyCan.m_loggermap_mutex);
    for (can::canlog_map_t::iterator it=MyCan.m_loggermap.begin(); it!=MyCan.m_loggermap.end(); ++it)
      {
      canlog* cl = it->second;
      writer->printf("#%" PRId32 ": %s\n", it->first, cl->GetInfo().c_str());
      }
    }
  }

////////////////////////////////////////////////////////////////////////
// CAN Logging System initialisation
////////////////////////////////////////////////////////////////////////

class OvmsCanLogInit
  {
  public: OvmsCanLogInit();
} MyOvmsCanLogInit  __attribute__ ((init_priority (4550)));

OvmsCanLogInit::OvmsCanLogInit()
  {
  ESP_LOGI(TAG, "Initialising CAN logging (4550)");

  OvmsCommand* cmd_can = MyCommandApp.FindCommand("can");
  if (cmd_can == NULL)
    {
    ESP_LOGE(TAG,"Cannot find CAN command - aborting log command registration");
    return;
    }

  OvmsCommand* cmd_canlog = cmd_can->RegisterCommand("log", "CAN logging framework");
  cmd_canlog->RegisterCommand("stop", "Stop logging", can_log_stop,"[<id>]",0,1);
  cmd_canlog->RegisterCommand("status", "Logging status", can_log_status,"[<id>]",0,1);
  cmd_canlog->RegisterCommand("heap", "Current CAN/VFS heap topology", can_log_heap);
  cmd_canlog->RegisterCommand("health", "Compact lock-free CAN/VFS health", can_log_health);
  cmd_canlog->RegisterCommand("list", "Logging list", can_log_list);
  cmd_canlog->RegisterCommand("start", "CAN logging start framework");
  }

////////////////////////////////////////////////////////////////////////
// CAN Logger Connection class
////////////////////////////////////////////////////////////////////////


canlogconnection::canlogconnection(canlog* logger, std::string format, canformat::canformat_serve_mode_t mode)
  {
  m_logger = logger;
  m_formatter = MyCanFormatFactory.NewFormat(format.c_str());
  m_formatter->SetServeMode(mode);
  m_nc = NULL;
  m_ispaused = false;
  m_filters = NULL;
  m_msgcount = 0;
  m_dropcount = 0;
  m_discardcount = 0;
  m_filtercount = 0;
  }

canlogconnection::~canlogconnection()
  {
  if (m_filters != NULL)
    {
    delete m_filters;
    m_filters = NULL;
    }
  if (m_formatter != NULL)
    {
    delete m_formatter;
    m_formatter = NULL;
    }
  }

void canlogconnection::OutputMsg(CAN_log_message_t& msg, std::string &result)
  {
  m_msgcount++;

  if ((m_filters != NULL) && (! m_filters->IsFiltered(&msg.frame)))
    {
    m_filtercount++;
    return;
    }

#ifdef CONFIG_OVMS_SC_GPL_MONGOOSE
  // The standard base implemention here is for mongoose network connections
  auto mglock = MongooseLock();
  if (m_nc != NULL)
    {
    if (result.length()>0)
      {
      if (m_nc->send_mbuf.len < 32768)
        {
        mg_send(m_nc, (const char*)result.c_str(), result.length());
        }
      else
        {
        m_dropcount++;
        }
      }
    }
  else
#endif // CONFIG_OVMS_SC_GPL_MONGOOSE
    {
    m_dropcount++;
    }
  }

void canlogconnection::TransmitCallback(uint8_t *buffer, size_t len)
  {
  ESP_LOGD(TAG,"TransmitCallback on %s (%d bytes)",m_peer.c_str(),len);

  m_msgcount++;
#ifdef CONFIG_OVMS_SC_GPL_MONGOOSE
  auto mglock = MongooseLock();
  if ((m_nc != NULL)&&(m_nc->send_mbuf.len < 32768))
    {
    mg_send(m_nc, buffer, len);
    }
  else
#endif // CONFIG_OVMS_SC_GPL_MONGOOSE
    {
    m_dropcount++;
    }
  }

void canlogconnection::ControlBusConfigure(canbus* bus, CAN_mode_t mode, CAN_speed_t speed)
  {
  ESP_LOGI(TAG,"Remote CAN bus configure: %s %s %dKbps",
    bus->GetName(),
    (mode == CAN_MODE_LISTEN)?"listen":"active",
    (int)speed);
  bus->Start(mode, speed);
  }

void canlogconnection::PauseTransmission()
  {
  ESP_LOGI(TAG,"Remote CAN bus pause transmission");
  m_ispaused = true;
  }

void canlogconnection::ResumeTransmission()
  {
  ESP_LOGI(TAG,"Remote CAN bus resume transmission");
  m_ispaused = false;
  }

void canlogconnection::ClearFilters()
  {
  ESP_LOGI(TAG,"Remote CAN bus clear filters");
  if (m_filters)
    {
    delete m_filters;
    m_filters = NULL;
    }
  }

void canlogconnection::AddFilter(std::string& filter)
  {
  ESP_LOGI(TAG,"Remote CAN bus add filter: %s", filter.c_str());
  if (!m_filters) m_filters = new canfilter();
  m_filters->AddFilter(filter.c_str());
  }

std::string canlogconnection::GetSummary()
  {
  return m_peer;
  }

std::string canlogconnection::GetStats()
  {
  std::ostringstream buf;

  float droprate = (m_msgcount > 0) ? ((float) m_dropcount/m_msgcount*100) : 0;

  buf << "Messages:" << m_msgcount
    << " Discarded:" << m_discardcount
    << " Dropped:" << m_dropcount
    << " Filtered:" << m_filtercount
    << " Rate:" << std::fixed << std::setprecision(1) << droprate << "%";

  return buf.str();
  }


////////////////////////////////////////////////////////////////////////
// CAN Logger class
////////////////////////////////////////////////////////////////////////

canlog::canlog(const char* type, std::string format, canformat::canformat_serve_mode_t mode, bool start_task)
  : m_events_filters(TAG), m_metrics_filters(TAG)
  {
  m_type = type;
  m_format = format;
  m_mode = mode;
  m_formatter = MyCanFormatFactory.NewFormat(format.c_str());
  m_formatter->SetServeMode(mode);
  m_filter = NULL;
  m_isopen = false;

  m_msgcount = 0;
  m_dropcount = 0;
  m_filtercount = 0;

  using std::placeholders::_1;
  using std::placeholders::_2;
  MyEvents.RegisterEvent(IDTAG, "*", std::bind(&canlog::EventListener, this, _1, _2));
  MyEvents.RegisterEvent(IDTAG,"config.mounted", std::bind(&canlog::UpdatedConfig, this, _1, _2));
  MyEvents.RegisterEvent(IDTAG,"config.changed", std::bind(&canlog::UpdatedConfig, this, _1, _2));
  MyMetrics.RegisterListener(IDTAG, "*", std::bind(&canlog::MetricListener, this, _1));

  m_task = NULL;
  m_queue = NULL;

  LoadConfig();
  if (start_task)
    {
    int queuesize = MyConfig.GetParamValueInt(CAN_PARAM, "log.queuesize",100);
    m_queue = xQueueCreate(queuesize, sizeof(CAN_log_message_t));
    xTaskCreatePinnedToCore(RxTask, "OVMS CanLog", 4096, (void*)this, 10, &m_task, CORE(1));
    }
  }

canlog::~canlog()
  {
  MyEvents.DeregisterEvent(IDTAG);
  MyMetrics.DeregisterListener(IDTAG);

  if (m_task)
    {
    TaskHandle_t t = m_task;
    m_task = NULL;

    vTaskDelete(t);
    }

  if (m_queue)
    {
    QueueHandle_t q = m_queue;
    m_queue = NULL;

    CAN_log_message_t msg;
    while (xQueueReceive(q, &msg, 0) == pdTRUE)
      {
      switch (msg.type)
        {
        case CAN_LogInfo_Comment:
        case CAN_LogInfo_Config:
        case CAN_LogInfo_Event:
        case CAN_LogInfo_Metric:
          free(msg.text);
          break;
        default:
          break;
        }
      }
    vQueueDelete(q);
    }

  if (m_formatter)
    {
    delete m_formatter;
    m_formatter = NULL;
    }

  if (m_filter)
    {
    delete m_filter;
    m_filter = NULL;
    }
  }

void canlog::RxTask(void *context)
  {
  canlog* me = (canlog*) context;
  CAN_log_message_t msg;
  while (1)
    {
    if (xQueueReceive(me->m_queue, &msg, (portTickType)portMAX_DELAY) == pdTRUE)
      {
      switch (msg.type)
        {
        case CAN_LogInfo_Comment:
        case CAN_LogInfo_Config:
        case CAN_LogInfo_Event:
        case CAN_LogInfo_Metric:
          me->OutputMsg(msg);
          free(msg.text);
          break;
        default:
          me->OutputMsg(msg);
          break;
        }
      }
    }
  }

/**
 * Load, or reload, the configuration of events and metrics filters.
 *
 * The configuration item is a string containing a comma-separated list of filters.
 */
void canlog::LoadConfig()
  {
  std::string list_of_events_filters = MyConfig.GetParamValue(CAN_PARAM, "log.events_filters", "x*,vehicle*");
  if (m_events_filters.LoadFilters(list_of_events_filters))
    MyCan.LogInfo(NULL, CAN_LogInfo_Config, ("Events filters: " + list_of_events_filters).c_str());

  std::string list_of_metrics_filters = MyConfig.GetParamValue(CAN_PARAM, "log.metrics_filters");
  if (m_metrics_filters.LoadFilters(list_of_metrics_filters))
    MyCan.LogInfo(NULL, CAN_LogInfo_Config, ("Metrics filters: " + list_of_metrics_filters).c_str());
  }

/**
 * Load, or reload, the configuration if a config event occurred.
 */
void canlog::UpdatedConfig(std::string event, void* data)
  {
  if (event == "config.changed")
    {
    // Only reload if our parameter has changed
    OvmsConfigParam*p = (OvmsConfigParam*)data;
    if (p->GetName() != CAN_PARAM)
      {
      return;
      }
    }
  LoadConfig();
  }

void canlog::EventListener(std::string event, void* data)
  {
  // Log vehicle custom (x…) & framework events:
  if (m_events_filters.CheckFilter(event))
    LogInfo(NULL, CAN_LogInfo_Event, event.c_str());
  }

void canlog::MetricListener(OvmsMetric* metric)
  {
    std::string name = metric->m_name;
  // Log metrics (in JSON for later parsing):
  if (m_metrics_filters.CheckFilter(name))
    {
    std::string metric_text = "{ ";
    metric_text += "\"name\": \"" + json_encode(name) + "\", ";
    metric_text += "\"value\": " + metric->AsJSON() + ", ";
    metric_text += "\"unit\": \"" + json_encode(std::string(OvmsMetricUnitLabel(metric->GetUnits()))) + "\" }";
    LogInfo(NULL, CAN_LogInfo_Metric, metric_text.c_str());
    }
  }

const char* canlog::GetType()
  {
  return m_type;
  }

const char* canlog::GetFormat()
  {
  return m_format.c_str();
  }

void canlog::OutputMsg(CAN_log_message_t& msg)
  {
  if (m_formatter == NULL)
    {
    m_dropcount++;
    return;
    }

  if (!m_isopen)
    {
    m_dropcount++;
    return;
    }

  std::string result = m_formatter->get(&msg);
  if (result.length()>0)
    {
    OvmsRecMutexLock lock(&m_cmmutex);
    for (conn_map_t::iterator it=m_connmap.begin(); it!=m_connmap.end(); ++it)
      {
      if (it->second->m_ispaused)
        {
        it->second->m_msgcount++;
        it->second->m_discardcount++;
        }
      else
        {
        it->second->OutputMsg(msg, result);
        }
      }
    }
  }

std::string canlog::GetInfo()
  {
  std::ostringstream buf;

  buf << "Type:" << m_type;

  if (m_formatter)
    {
    buf << " Format:" << m_format << "(" << m_formatter->GetServeModeName() << ")";
    }

  if (m_filter)
    {
    buf << " Filter:" << m_filter->Info();
    }

  if (StdMetrics.ms_v_type->IsDefined())
    {
    buf << " Vehicle:" << StdMetrics.ms_v_type->AsString();
    }

  return buf.str();
  }

bool canlog::IsOpen()
  {
  return m_isopen;
  }

std::string canlog::GetStats()
  {
  std::ostringstream buf;

  float droprate = (m_msgcount > 0) ? ((float) m_dropcount/m_msgcount*100) : 0;
  uint32_t waiting = uxQueueMessagesWaiting(m_queue);

  buf << "Messages:" << m_msgcount
    << " Dropped:" << m_dropcount
    << " Filtered:" << m_filtercount
    << " Rate:" << std::fixed << std::setprecision(1) << droprate << "%";

  if (waiting > 0)
    buf << " Queued:" << waiting;

  return buf.str();
  }

void canlog::SetFilter(canfilter* filter)
  {
  if (m_filter)
    {
    delete m_filter;
    }
  m_filter = filter;
  }

void canlog::ClearFilter()
  {
  if (m_filter)
    {
    delete m_filter;
    m_filter = NULL;
    }
  }

void canlog::LogFrame(canbus* bus, CAN_log_type_t type, const CAN_frame_t* frame)
  {
  if (!IsOpen() || !bus || !frame) return;

  if (((m_filter == NULL)||(m_filter->IsFiltered(frame)))&&(m_queue))
    {
    CAN_log_message_t msg;
    msg.type = type;
    gettimeofday(&msg.timestamp,NULL);
    memcpy(&msg.frame,frame,sizeof(CAN_frame_t));
    msg.frame.origin = bus;
    m_msgcount++;
    if (xQueueSend(m_queue, &msg, 0) != pdTRUE) m_dropcount++;
    }
  else
    {
    m_filtercount++;
    }
  }

void canlog::LogStatus(canbus* bus, CAN_log_type_t type, const CAN_status_t* status)
  {
  if (!IsOpen() || !bus) return;

  if (((m_filter == NULL)||(m_filter->IsFiltered(bus)))&&(m_queue))
    {
    CAN_log_message_t msg;
    msg.type = type;
    gettimeofday(&msg.timestamp,NULL);
    msg.origin = bus;
    memcpy(&msg.status,status,sizeof(CAN_status_t));
    m_msgcount++;
    if (xQueueSend(m_queue, &msg, 0) != pdTRUE) m_dropcount++;
    }
  else
    {
    m_filtercount++;
    }
  }

void canlog::LogInfo(canbus* bus, CAN_log_type_t type, const char* text)
  {
  if (!IsOpen() || !text) return;

  if (((m_filter == NULL)||(m_filter->IsFiltered(bus)))&&(m_queue))
    {
    CAN_log_message_t msg;
    msg.type = type;
    gettimeofday(&msg.timestamp,NULL);
    msg.origin = bus;
    msg.text = strdup(text);
    m_msgcount++;
    if (xQueueSend(m_queue, &msg, 0) != pdTRUE)
      {
      m_dropcount++;
      free(msg.text);
      }
    }
  else
    {
    m_filtercount++;
    }
  }
