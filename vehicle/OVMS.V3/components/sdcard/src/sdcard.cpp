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

#include "ovms_log.h"
static const char *TAG = "sdcard";

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <atomic>
#include <string>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_intr_alloc.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "sdcard.h"
#include "ovms_config.h"
#include "ovms_command.h"
#include "ovms_mutex.h"
#include "ovms_peripherals.h"
#include "ovms_events.h"
#include "ovms_boot.h"

static int insertcount = 0;
static int mountcount = 0;

void sdcard::Ticker1(std::string event, void* data)
  {
  if (insertcount > 0)
    {
    insertcount--;
    if (insertcount == 0)
      {
      CheckCardState();
      }
    }
  if (mountcount > 0)
    {
    mountcount--;
    if (mountcount == 0)
      {
      if (m_mounted)
        MyEvents.SignalEvent("sd.mounted", NULL);
      else
        {
        MyEvents.SignalEvent("sd.unmounted", NULL);
        if (MyBoot.IsShuttingDown())
          MyBoot.ShutdownReady(TAG);
        }
      }
    }
  }

void sdcard::EventSystemShutDown(std::string event, void* data)
  {
  if (m_mounted && event == "system.shuttingdown")
    {
    MyBoot.ShutdownPending(TAG);
    ESP_LOGI(TAG,"Unmounting SDCARD for reset");
    unmount();
    }
  else if (m_mounted && event == "system.shutdown")
    {
    ESP_LOGI(TAG,"Hard SD unmount for reset");
    unmount(true);
    }
  }

static void IRAM_ATTR sdcard_isr_handler(void* arg)
  {
  insertcount = 2;
  }

sdcard::sdcard(const char* name, bool mode1bit, bool autoformat, int cdpin)
  : pcp(name)
  {
  m_host = sdmmc_host_t SDMMC_HOST_DEFAULT();
  if (mode1bit)
    {
    m_host.flags = SDMMC_HOST_FLAG_1BIT;
    }

  m_slot = sdmmc_slot_config_t SDMMC_SLOT_CONFIG_DEFAULT();
// Disable driver-level CD pin, as we do this ourselves
//  if (cdpin)
//    {
//    m_slot.gpio_cd = (gpio_num_t)cdpin;
//    }
  m_slot.width = 1;

  memset(&m_mount,0,sizeof(esp_vfs_fat_sdmmc_mount_config_t));
  m_mount.format_if_mount_failed = autoformat;
  m_mount.max_files = 5;

  m_mounted = false;
  m_unmounting = false;
  m_cd = false;
  m_cdpin = cdpin;
  insertcount = 5;

  // Register our events
  #undef bind  // Kludgy, but works
  using std::placeholders::_1;
  using std::placeholders::_2;
  MyEvents.RegisterEvent(TAG,"ticker.1", std::bind(&sdcard::Ticker1, this, _1, _2));
  MyEvents.RegisterEvent(TAG,"system.shuttingdown", std::bind(&sdcard::EventSystemShutDown, this, _1, _2));
  MyEvents.RegisterEvent(TAG,"system.shutdown", std::bind(&sdcard::EventSystemShutDown, this, _1, _2));

  gpio_pullup_en((gpio_num_t)cdpin);
  gpio_set_intr_type((gpio_num_t)cdpin, GPIO_INTR_ANYEDGE);
  gpio_isr_handler_add((gpio_num_t)cdpin, sdcard_isr_handler, (void*) this);
  }

sdcard::~sdcard()
  {
  if (m_mounted)
    {
    unmount(true);
    }
  }

esp_err_t sdcard::mount()
  {
  if (m_unmounting)
    {
    ESP_LOGE(TAG, "mount failed: unmount in progress");
    return ESP_FAIL;
    }
  else if (m_mounted)
    {
    ESP_LOGE(TAG, "mount failed: already mounted");
    return ESP_FAIL;
    }
  else if (MyBoot.GetBootReason() == BR_PartitionUpdate)
    {
    ESP_LOGE(TAG, "mount failed: partition update detected");
    return ESP_FAIL;
    }

  m_host.max_freq_khz = MyConfig.GetParamValueInt("sdcard", "maxfreq.khz", 16000);
  esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sd", &m_host, &m_slot, &m_mount, &m_card);
  if (ret == ESP_OK)
    {
    ESP_LOGI(TAG, "mount done");
    m_mounted = true;
    m_unmounting = false;
    mountcount = 3;
    }
  else
    {
    ESP_LOGE(TAG, "mount failed: %s", esp_err_to_name(ret));
    }

  return ret;
  }

static void sdcard_unmounting_done(const char* event, void* data)
  {
  ESP_LOGD(TAG, "unmount: preparation done");
  MyPeripherals->m_sdcard->unmount(true);
  }

esp_err_t sdcard::unmount(bool hard /*=false*/)
  {
  if (!m_mounted)
    return ESP_OK;

  if (!hard)
    {
    if (!m_unmounting)
      {
      m_unmounting = true;
      ESP_LOGD(TAG, "unmount: preparing");
      MyEvents.SignalEvent("sd.unmounting", NULL, sdcard_unmounting_done);
      }
    return ESP_FAIL;
    }
  else
    {
#if ESP_IDF_VERSION_MAJOR >= 5
    esp_err_t ret = esp_vfs_fat_sdcard_unmount("/sd", m_card);
#else
    esp_err_t ret = esp_vfs_fat_sdmmc_unmount();
#endif
    if (ret == ESP_OK)
      {
      ESP_LOGI(TAG, "unmount done");
      m_mounted = false;
      m_unmounting = false;
      mountcount = 3;
      }
    else
      {
      ESP_LOGE(TAG, "unmount failed: %s", esp_err_to_name(ret));
      }
    return ret;
    }
  }

bool sdcard::isavailable()
  {
  return m_mounted && !m_unmounting;
  }

size_t sdcard::GetFatClusterSize()
  {
  if (!isavailable() || !m_card)
    return 0;

  FATFS* fs = NULL;
  DWORD free_clusters = 0;
  if (f_getfree("1:", &free_clusters, &fs) != FR_OK || !fs)
    return 0;

  return (size_t)fs->csize * m_card->csd.sector_size;
  }

bool sdcard::isunmounting()
  {
  return m_unmounting;
  }

bool sdcard::ismounted()
  {
  return m_mounted;
  }

bool sdcard::isinserted()
  {
  return m_cd;
  }

void sdcard::CheckCardState()
  {
  bool cd = (gpio_get_level((gpio_num_t)m_cdpin)==0)?true:false;

  if (cd != m_cd)
    {
    m_cd = cd;
    if (m_cd)
      {
      // SD CARD has been inserted. Let's auto-mount
      ESP_LOGI(TAG, "SD CARD has been inserted");
      MyEvents.SignalEvent("sd.insert", NULL);
      if (MyConfig.GetParamValueBool("sdcard", "automount", true))
        {
        mount();
        }
      }
    else
      {
      // SD CARD has been removed. A bit late, but let's dismount
      ESP_LOGI(TAG, "SD CARD has been removed");
      if (m_mounted) unmount(true);
      MyEvents.SignalEvent("sd.remove", NULL);
      }
    }
  }

void sdcard_printinfo(int verbosity, OvmsWriter* writer)
  {
  sdmmc_card_t* card = MyPeripherals->m_sdcard->m_card;
  FATFS *fs;
  DWORD fre_clust, fre_sect, tot_sect;

  writer->printf("Name: %s\n", card->cid.name);

  // Get volume information and free clusters
  // Note: assuming drive "1:" = /sd ("0:" = /store)
  if (f_getfree("1:", &fre_clust, &fs) == FR_OK)
    {
    tot_sect = (fs->n_fatent - 2) * fs->csize;
    fre_sect = fre_clust * fs->csize;
    writer->printf("Size: %6llu MB\nFree: %6llu MB\n",
      ((uint64_t) tot_sect) * card->csd.sector_size / (1024 * 1024),
      ((uint64_t) fre_sect) * card->csd.sector_size / (1024 * 1024));
    }

  if (verbosity > COMMAND_RESULT_MINIMAL)
    {
    writer->printf("\nCard type: %s\n", (card->ocr & SD_OCR_SDHC_CAP)?"SDHC/SDXC":"SDSC");
    writer->printf("Max speed: %d kHz\n", card->csd.tr_speed/1000);
    writer->printf("Capacity: %llu MB\n", ((uint64_t) card->csd.capacity) * card->csd.sector_size / (1024 * 1024));
    writer->printf("CSD: ver=%d, sector_size=%d, capacity=%d read_bl_len=%d\n",
                    card->csd.csd_ver,
                    card->csd.sector_size, card->csd.capacity, card->csd.read_block_len);
    writer->printf("SCR: sd_spec=%d, bus_width=%d\n", card->scr.sd_spec, card->scr.bus_width);
    }
  }

void sdcard_mount(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  if (MyPeripherals->m_sdcard->isunmounting())
    {
    writer->puts("Error: SD CARD is unmounting");
    return;
    }
  if (MyPeripherals->m_sdcard->ismounted())
    {
    writer->puts("SD CARD is already mounted");
    return;
    }

  esp_err_t res = MyPeripherals->m_sdcard->mount();
  if (res == ESP_OK)
    writer->puts("Mounted SD CARD");
  else
    writer->printf("Error: SD mount failed: %s\n", esp_err_to_name(res));
  }

void sdcard_unmount(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  if (!MyPeripherals->m_sdcard->ismounted())
    {
    writer->puts("SD CARD not mounted");
    return;
    }

  esp_err_t res = ESP_OK;
  int maxwait_seconds = 5;
  if (argc >= 1)
    maxwait_seconds = atoi(argv[0]);

  if (maxwait_seconds == 0)
    {
    MyPeripherals->m_sdcard->unmount();
    writer->puts("Unmounting SD CARD");
    return;
    }

  for (int i=maxwait_seconds; i; i--)
    {
    res = MyPeripherals->m_sdcard->unmount();
    if (res == ESP_OK) break;
    if (i > 1)
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  if (res == ESP_OK)
    writer->puts("Unmounted SD CARD");
  else
    writer->printf("Error: SD unmount failed: %s\n", esp_err_to_name(res));
  }

void sdcard_status(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  if (MyPeripherals->m_sdcard->isinserted())
    {
    writer->puts("SD CARD is inserted");
    }
  else
    {
    writer->puts("SD CARD is not inserted");
    }
  if (MyPeripherals->m_sdcard->ismounted())
    {
    if (MyPeripherals->m_sdcard->isavailable())
      writer->puts("Status: available");
    else
      writer->puts("Status: unmounting");
    sdcard_printinfo(verbosity, writer);
    }
  }

static const size_t SDCARD_BENCH_BLOCK_MIN = 512;
static const size_t SDCARD_BENCH_BLOCK_MAX = 64 * 1024;
static const uint32_t SDCARD_BENCH_TOTAL_MIB_MIN = 1;
static const uint32_t SDCARD_BENCH_TOTAL_MIB_MAX = 64;

struct sdcard_bench_stats
  {
  uint64_t write_calls;
  uint64_t write_bytes;
  uint64_t write_time_us;
  uint64_t write_max_us;
  uint64_t write_slow_1ms;
  uint64_t write_slow_5ms;
  uint64_t write_slow_10ms;
  uint64_t write_slow_50ms;
  uint64_t write_slow_100ms;
  uint64_t write_slow_250ms;
  };

static bool sdcard_bench_parse_uint32(const char* text, uint32_t* value)
  {
  if (!text || !*text)
    return false;

  errno = 0;
  char* end = NULL;
  unsigned long parsed = strtoul(text, &end, 10);
  if (errno == ERANGE || end == text || *end != '\0' || parsed > UINT32_MAX)
    return false;

  *value = (uint32_t)parsed;
  return true;
  }

static bool sdcard_bench_path_valid(const char* path)
  {
  size_t length = path ? strlen(path) : 0;
  if (length <= 4 || strncmp(path, "/sd/", 4) != 0)
    return false;
  if (strstr(path, "/../") || (length >= 3 && strcmp(path + length - 3, "/..") == 0))
    return false;
  return true;
  }

static void sdcard_bench_add_latency(sdcard_bench_stats* stats, uint64_t elapsed_us)
  {
  stats->write_calls++;
  stats->write_time_us += elapsed_us;
  if (elapsed_us > stats->write_max_us)
    stats->write_max_us = elapsed_us;
  if (elapsed_us >= 1000)
    stats->write_slow_1ms++;
  if (elapsed_us >= 5000)
    stats->write_slow_5ms++;
  if (elapsed_us >= 10000)
    stats->write_slow_10ms++;
  if (elapsed_us >= 50000)
    stats->write_slow_50ms++;
  if (elapsed_us >= 100000)
    stats->write_slow_100ms++;
  if (elapsed_us >= 250000)
    stats->write_slow_250ms++;
  }

void sdcard_bench_write(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  bool stdio_mode;
  if (strcmp(argv[0], "stdio") == 0)
    stdio_mode = true;
  else if (strcmp(argv[0], "posix") == 0)
    stdio_mode = false;
  else
    {
    writer->puts("Error: mode must be 'stdio' or 'posix'");
    return;
    }

  const char* path = argv[1];
  if (!sdcard_bench_path_valid(path))
    {
    writer->puts("Error: path must name a file below /sd and must not contain '..'");
    return;
    }

  uint32_t block_size_arg;
  if (!sdcard_bench_parse_uint32(argv[2], &block_size_arg) ||
      block_size_arg < SDCARD_BENCH_BLOCK_MIN ||
      block_size_arg > SDCARD_BENCH_BLOCK_MAX ||
      (block_size_arg % 512) != 0)
    {
    writer->printf("Error: blocksize must be a multiple of 512 from %u through %u bytes\n",
      (unsigned)SDCARD_BENCH_BLOCK_MIN, (unsigned)SDCARD_BENCH_BLOCK_MAX);
    return;
    }
  size_t block_size = block_size_arg;

  uint32_t total_mib;
  if (!sdcard_bench_parse_uint32(argv[3], &total_mib) ||
      total_mib < SDCARD_BENCH_TOTAL_MIB_MIN || total_mib > SDCARD_BENCH_TOTAL_MIB_MAX)
    {
    writer->printf("Error: total_mib must be from %u through %u\n",
      (unsigned)SDCARD_BENCH_TOTAL_MIB_MIN, (unsigned)SDCARD_BENCH_TOTAL_MIB_MAX);
    return;
    }

  bool sync_end = true;
  if (argc >= 5)
    {
    if (strcmp(argv[4], "end") == 0)
      sync_end = true;
    else if (strcmp(argv[4], "none") == 0)
      sync_end = false;
    else
      {
      writer->puts("Error: sync mode must be 'end' or 'none'");
      return;
      }
    }

  uint64_t total_bytes = ((uint64_t)total_mib) * 1024 * 1024;
  if ((total_bytes % block_size) != 0)
    {
    writer->puts("Error: total byte count must be an exact multiple of blocksize");
    return;
    }

  if (!MyPeripherals || !MyPeripherals->m_sdcard || !MyPeripherals->m_sdcard->isavailable())
    {
    writer->puts("Error: SD CARD is not available");
    return;
    }

  struct stat file_stat;
  errno = 0;
  if (stat(path, &file_stat) == 0)
    {
    writer->printf("Error: benchmark file already exists: %s\n", path);
    return;
    }
  if (errno != ENOENT)
    {
    writer->printf("Error: cannot check benchmark path %s: %s\n", path, strerror(errno));
    return;
    }

  sdmmc_card_t* card = MyPeripherals->m_sdcard->m_card;
  FATFS* fs = NULL;
  DWORD free_clusters = 0;
  FRESULT fat_result = f_getfree("1:", &free_clusters, &fs);
  if (fat_result != FR_OK || !fs || !card)
    {
    writer->printf("Error: cannot read SD filesystem information (FatFS result %d)\n", (int)fat_result);
    return;
    }

  uint32_t sector_size = card->csd.sector_size;
  uint64_t cluster_bytes = ((uint64_t)fs->csize) * sector_size;
  uint64_t free_bytes = ((uint64_t)free_clusters) * cluster_bytes;
  if (total_bytes > free_bytes)
    {
    writer->printf("Error: requested %" PRIu64 " bytes, only %" PRIu64 " bytes are free\n",
      total_bytes, free_bytes);
    return;
    }

  uint8_t* buffer = static_cast<uint8_t*>(heap_caps_malloc(
    block_size, MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
  if (!buffer)
    {
    writer->printf("Error: cannot allocate %u bytes of DMA-capable internal memory\n",
      (unsigned)block_size);
    return;
    }
  for (size_t i = 0; i < block_size; ++i)
    buffer[i] = (uint8_t)((i % 251) + 1);

  FILE* file = NULL;
  int fd = -1;
  if (stdio_mode)
    {
    file = fopen(path, "wb");
    if (!file)
      {
      int open_errno = errno;
      heap_caps_free(buffer);
      writer->printf("Error: fopen(%s) failed: %s\n", path, strerror(open_errno));
      return;
      }
    }
  else
    {
    fd = open(path, O_WRONLY | O_CREAT | O_EXCL, 0666);
    if (fd < 0)
      {
      int open_errno = errno;
      heap_caps_free(buffer);
      writer->printf("Error: open(%s) failed: %s\n", path, strerror(open_errno));
      return;
      }
    }

  writer->printf("SD write benchmark starting; do not remove or unmount the card\n"
    "  Mode:%s Path:%s Sync:%s\n"
    "  BlockSize:%u RequestedBytes:%" PRIu64 " SectorSize:%u ClusterSize:%" PRIu64 "\n"
    "  BusWidth:%d HostFrequency:%dkHz Buffer:DMA+8BIT\n",
    stdio_mode ? "stdio" : "posix", path, sync_end ? "end" : "none",
    (unsigned)block_size, total_bytes, (unsigned)sector_size, cluster_bytes,
    MyPeripherals->m_sdcard->m_slot.width, MyPeripherals->m_sdcard->m_host.max_freq_khz);

  sdcard_bench_stats stats = {};
  const char* failure_stage = NULL;
  int failure_errno = 0;
  uint64_t failure_expected = 0;
  uint64_t failure_actual = 0;

  int64_t write_start_us = esp_timer_get_time();
  while (stats.write_bytes < total_bytes)
    {
    errno = 0;
    int64_t call_start_us = esp_timer_get_time();
    ssize_t written;
    if (stdio_mode)
      written = (ssize_t)fwrite(buffer, 1, block_size, file);
    else
      written = write(fd, buffer, block_size);
    int call_errno = errno;
    uint64_t call_time_us = (uint64_t)(esp_timer_get_time() - call_start_us);
    sdcard_bench_add_latency(&stats, call_time_us);

    if (written > 0)
      stats.write_bytes += written;
    if (written != (ssize_t)block_size)
      {
      failure_stage = stdio_mode ? "fwrite" : "write";
      failure_errno = call_errno;
      failure_expected = block_size;
      failure_actual = written > 0 ? written : 0;
      break;
      }
    }
  int64_t write_end_us = esp_timer_get_time();

  uint64_t flush_time_us = 0;
  uint64_t sync_time_us = 0;
  uint64_t close_time_us = 0;
  bool flush_performed = false;
  bool sync_performed = false;

  if (sync_end && stdio_mode)
    {
    flush_performed = true;
    errno = 0;
    int64_t operation_start_us = esp_timer_get_time();
    int result = fflush(file);
    int operation_errno = errno;
    flush_time_us = (uint64_t)(esp_timer_get_time() - operation_start_us);
    if (result != 0 && !failure_stage)
      {
      failure_stage = "fflush";
      failure_errno = operation_errno;
      }

    if (result == 0)
      {
      int stream_fd = fileno(file);
      if (stream_fd < 0)
        {
        if (!failure_stage)
          {
          failure_stage = "fileno";
          failure_errno = errno;
          }
        }
      else
        {
        sync_performed = true;
        errno = 0;
        operation_start_us = esp_timer_get_time();
        result = fsync(stream_fd);
        operation_errno = errno;
        sync_time_us = (uint64_t)(esp_timer_get_time() - operation_start_us);
        if (result != 0 && !failure_stage)
          {
          failure_stage = "fsync";
          failure_errno = operation_errno;
          }
        }
      }
    }
  else if (sync_end)
    {
    sync_performed = true;
    errno = 0;
    int64_t operation_start_us = esp_timer_get_time();
    int result = fsync(fd);
    int operation_errno = errno;
    sync_time_us = (uint64_t)(esp_timer_get_time() - operation_start_us);
    if (result != 0 && !failure_stage)
      {
      failure_stage = "fsync";
      failure_errno = operation_errno;
      }
    }

  errno = 0;
  int64_t close_start_us = esp_timer_get_time();
  int close_result = stdio_mode ? fclose(file) : close(fd);
  int close_errno = errno;
  close_time_us = (uint64_t)(esp_timer_get_time() - close_start_us);
  int64_t benchmark_end_us = esp_timer_get_time();
  if (close_result != 0 && !failure_stage)
    {
    failure_stage = stdio_mode ? "fclose" : "close";
    failure_errno = close_errno;
    }

  heap_caps_free(buffer);

  uint64_t elapsed_time_us = (uint64_t)(write_end_us - write_start_us);
  uint64_t total_time_us = (uint64_t)(benchmark_end_us - write_start_us);
  double average_write_us = stats.write_calls
    ? (double)stats.write_time_us / stats.write_calls : 0;
  double throughput_mib_s = elapsed_time_us
    ? ((double)stats.write_bytes * 1000000.0) / elapsed_time_us / (1024.0 * 1024.0) : 0;
  double end_throughput_mib_s = total_time_us
    ? ((double)stats.write_bytes * 1000000.0) / total_time_us / (1024.0 * 1024.0) : 0;

  writer->printf("SD write benchmark results\n"
    "  Mode:%s Path:%s Sync:%s\n"
    "  BlockSize:%u RequestedBytes:%" PRIu64 " TotalBytes:%" PRIu64 " WriteCalls:%" PRIu64 "\n"
    "  WriteTime:%.6fs ElapsedTime:%.6fs AverageWrite:%.1fus WriteMax:%.3fms\n"
    "  WriteSlow1ms:%" PRIu64 " WriteSlow5ms:%" PRIu64 " WriteSlow10ms:%" PRIu64
    " WriteSlow50ms:%" PRIu64 " WriteSlow100ms:%" PRIu64 " WriteSlow250ms:%" PRIu64 "\n"
    "  FlushPerformed:%d FlushTime:%.6fs SyncPerformed:%d SyncTime:%.6fs CloseTime:%.6fs\n"
    "  TotalTime:%.6fs Throughput:%.3fMiB/s EndThroughput:%.3fMiB/s\n",
    stdio_mode ? "stdio" : "posix", path, sync_end ? "end" : "none",
    (unsigned)block_size, total_bytes, stats.write_bytes, stats.write_calls,
    stats.write_time_us / 1000000.0, elapsed_time_us / 1000000.0,
    average_write_us, stats.write_max_us / 1000.0,
    stats.write_slow_1ms, stats.write_slow_5ms, stats.write_slow_10ms,
    stats.write_slow_50ms, stats.write_slow_100ms, stats.write_slow_250ms,
    flush_performed ? 1 : 0, flush_time_us / 1000000.0,
    sync_performed ? 1 : 0, sync_time_us / 1000000.0,
    close_time_us / 1000000.0, total_time_us / 1000000.0,
    throughput_mib_s, end_throughput_mib_s);

  if (failure_stage)
    {
    if (failure_expected)
      writer->printf("Result: ERROR at %s: expected %" PRIu64 " bytes, got %" PRIu64,
        failure_stage, failure_expected, failure_actual);
    else
      writer->printf("Result: ERROR at %s", failure_stage);
    if (failure_errno)
      writer->printf(": %s", strerror(failure_errno));
    writer->puts("");
    }
  else
    writer->puts("Result: OK");
  }

static const size_t SDCARD_LONG_BLOCK_SIZE = 32768;
static const uint32_t SDCARD_LONG_TOTAL_MIB_MAX = 2048;
static const uint32_t SDCARD_LONG_RATE_KIB_MAX = 4096;
static const uint32_t SDCARD_LONG_SYNC_PERIOD_MAX = 3600;
static const uint32_t SDCARD_LONG_SLOW_MS_MAX = 60000;
static const size_t SDCARD_LONG_RECENT_COUNT = 64;
static const uint32_t SDCARD_LONG_FAIRNESS_WRITES = 8;

enum sdcard_long_state_t
  {
  SDCARD_LONG_IDLE,
  SDCARD_LONG_STARTING,
  SDCARD_LONG_RUNNING,
  SDCARD_LONG_STOPPING,
  SDCARD_LONG_COMPLETE,
  SDCARD_LONG_STOPPED,
  SDCARD_LONG_ERROR,
  };

enum sdcard_long_op_type_t
  {
  SDCARD_LONG_OP_WRITE,
  SDCARD_LONG_OP_SYNC,
  SDCARD_LONG_OP_CLOSE,
  };

struct sdcard_long_op_t
  {
  uint64_t sequence;
  uint64_t offset;
  uint64_t requested;
  int64_t returned;
  uint64_t duration_us;
  int error_number;
  int file_error;
  sdcard_long_op_type_t type;
  };

struct sdcard_long_snapshot_t
  {
  sdcard_long_state_t state;
  std::string path;
  uint64_t target_bytes;
  uint32_t rate_kib;
  uint32_t sync_period_s;
  uint32_t slow_ms;
  uint32_t sector_size;
  uint64_t cluster_size;
  int bus_width;
  int host_frequency_khz;
  bool stop_requested;
  uint64_t started_us;
  uint64_t ended_us;
  uint64_t write_calls;
  uint64_t write_bytes;
  uint64_t write_time_us;
  uint64_t write_max_us;
  uint64_t write_slow_50ms;
  uint64_t write_slow_100ms;
  uint64_t write_slow_250ms;
  uint64_t write_slow_1000ms;
  uint64_t sync_calls;
  uint64_t sync_errors;
  uint64_t sync_time_us;
  uint64_t sync_max_us;
  uint64_t close_time_us;
  char error_stage[24];
  int error_number;
  int error_ferror;
  uint64_t error_requested;
  int64_t error_returned;
  uint64_t error_duration_us;
  uint64_t operation_sequence;
  size_t recent_count;
  size_t recent_next;
  sdcard_long_op_t recent[SDCARD_LONG_RECENT_COUNT];
  };

static const char* sdcard_long_state_name(sdcard_long_state_t state)
  {
  switch (state)
    {
    case SDCARD_LONG_IDLE: return "idle";
    case SDCARD_LONG_STARTING: return "starting";
    case SDCARD_LONG_RUNNING: return "running";
    case SDCARD_LONG_STOPPING: return "stopping";
    case SDCARD_LONG_COMPLETE: return "complete";
    case SDCARD_LONG_STOPPED: return "stopped";
    case SDCARD_LONG_ERROR: return "error";
    }
  return "unknown";
  }

static const char* sdcard_long_result_name(sdcard_long_state_t state)
  {
  switch (state)
    {
    case SDCARD_LONG_COMPLETE: return "OK";
    case SDCARD_LONG_ERROR: return "ERROR";
    case SDCARD_LONG_STOPPED: return "STOPPED";
    case SDCARD_LONG_IDLE: return "IDLE";
    default: return "RUNNING";
    }
  }

static const char* sdcard_long_op_name(sdcard_long_op_type_t type)
  {
  switch (type)
    {
    case SDCARD_LONG_OP_WRITE: return "write";
    case SDCARD_LONG_OP_SYNC: return "sync";
    case SDCARD_LONG_OP_CLOSE: return "close";
    }
  return "unknown";
  }

class sdcard_long_benchmark
  {
  public:
    sdcard_long_benchmark();
    bool Start(OvmsWriter* writer, const char* path, uint32_t total_mib,
      uint32_t rate_kib, uint32_t sync_period_s, uint32_t slow_ms);
    void Stop(OvmsWriter* writer);
    void Status(OvmsWriter* writer, bool show_recent);

  private:
    static void TaskEntry(void* context);
    void Run();
    void ResetLocked();
    void RecordOperationLocked(sdcard_long_op_type_t type, uint64_t offset,
      uint64_t requested, int64_t returned, uint64_t duration_us,
      int error_number, int file_error);
    void RecordWrite(uint64_t offset, size_t requested, size_t returned,
      uint64_t duration_us, int error_number, int file_error);
    void RecordSync(uint64_t offset, uint64_t duration_us, bool success,
      int error_number, int file_error);
    void RecordClose(uint64_t offset, uint64_t duration_us, int returned,
      int error_number);
    void SetError(const char* stage, int error_number, int file_error,
      uint64_t requested, int64_t returned, uint64_t duration_us);
    bool SyncFile(FILE* file, int fd, uint64_t offset);
    bool WaitUntil(int64_t deadline_us);
    void Finish(sdcard_long_state_t state);
    void GetSnapshot(sdcard_long_snapshot_t* snapshot);

  private:
    OvmsMutex m_mutex;
    TaskHandle_t m_task;
    std::atomic<bool> m_stop_requested;
    sdcard_long_state_t m_state;
    std::string m_path;
    uint64_t m_target_bytes;
    uint32_t m_rate_kib;
    uint32_t m_sync_period_s;
    uint32_t m_slow_ms;
    uint32_t m_sector_size;
    uint64_t m_cluster_size;
    int m_bus_width;
    int m_host_frequency_khz;
    uint64_t m_started_us;
    uint64_t m_ended_us;
    uint64_t m_write_calls;
    uint64_t m_write_bytes;
    uint64_t m_write_time_us;
    uint64_t m_write_max_us;
    uint64_t m_write_slow_50ms;
    uint64_t m_write_slow_100ms;
    uint64_t m_write_slow_250ms;
    uint64_t m_write_slow_1000ms;
    uint64_t m_sync_calls;
    uint64_t m_sync_errors;
    uint64_t m_sync_time_us;
    uint64_t m_sync_max_us;
    uint64_t m_close_time_us;
    char m_error_stage[24];
    int m_error_number;
    int m_error_ferror;
    uint64_t m_error_requested;
    int64_t m_error_returned;
    uint64_t m_error_duration_us;
    uint64_t m_operation_sequence;
    size_t m_recent_count;
    size_t m_recent_next;
    sdcard_long_op_t m_recent[SDCARD_LONG_RECENT_COUNT];
  };

sdcard_long_benchmark::sdcard_long_benchmark()
  : m_task(NULL), m_stop_requested(false), m_state(SDCARD_LONG_IDLE)
  {
  OvmsMutexLock lock(&m_mutex);
  ResetLocked();
  }

void sdcard_long_benchmark::ResetLocked()
  {
  m_stop_requested.store(false);
  m_state = SDCARD_LONG_IDLE;
  m_path.clear();
  m_target_bytes = 0;
  m_rate_kib = 0;
  m_sync_period_s = 0;
  m_slow_ms = 0;
  m_sector_size = 0;
  m_cluster_size = 0;
  m_bus_width = 0;
  m_host_frequency_khz = 0;
  m_started_us = 0;
  m_ended_us = 0;
  m_write_calls = 0;
  m_write_bytes = 0;
  m_write_time_us = 0;
  m_write_max_us = 0;
  m_write_slow_50ms = 0;
  m_write_slow_100ms = 0;
  m_write_slow_250ms = 0;
  m_write_slow_1000ms = 0;
  m_sync_calls = 0;
  m_sync_errors = 0;
  m_sync_time_us = 0;
  m_sync_max_us = 0;
  m_close_time_us = 0;
  m_error_stage[0] = '\0';
  m_error_number = 0;
  m_error_ferror = 0;
  m_error_requested = 0;
  m_error_returned = 0;
  m_error_duration_us = 0;
  m_operation_sequence = 0;
  m_recent_count = 0;
  m_recent_next = 0;
  memset(m_recent, 0, sizeof(m_recent));
  }

bool sdcard_long_benchmark::Start(OvmsWriter* writer, const char* path,
  uint32_t total_mib, uint32_t rate_kib, uint32_t sync_period_s, uint32_t slow_ms)
  {
  OvmsMutexLock lock(&m_mutex);
  if (m_task || m_state == SDCARD_LONG_STARTING || m_state == SDCARD_LONG_RUNNING ||
      m_state == SDCARD_LONG_STOPPING)
    {
    writer->puts("Error: SD long benchmark is already active");
    return false;
    }

  ResetLocked();
  m_path = path;
  m_target_bytes = ((uint64_t)total_mib) * 1024 * 1024;
  m_rate_kib = rate_kib;
  m_sync_period_s = sync_period_s;
  m_slow_ms = slow_ms;
  m_state = SDCARD_LONG_STARTING;

  BaseType_t result = xTaskCreatePinnedToCore(TaskEntry, "OVMS SDBenchLong", 4096,
    this, 2, &m_task, CORE(1));
  if (result != pdPASS)
    {
    m_task = NULL;
    m_state = SDCARD_LONG_ERROR;
    strlcpy(m_error_stage, "task-create", sizeof(m_error_stage));
    writer->puts("Error: cannot create SD long benchmark task");
    return false;
    }

  writer->printf("Started SD long benchmark: Path:%s TotalMiB:%u RateKiB:%u SyncPeriod:%us SlowMs:%u\n",
    path, (unsigned)total_mib, (unsigned)rate_kib,
    (unsigned)sync_period_s, (unsigned)slow_ms);
  return true;
  }

void sdcard_long_benchmark::Stop(OvmsWriter* writer)
  {
  OvmsMutexLock lock(&m_mutex);
  if (!m_task || (m_state != SDCARD_LONG_STARTING &&
      m_state != SDCARD_LONG_RUNNING && m_state != SDCARD_LONG_STOPPING))
    {
    writer->printf("SD long benchmark is not active (State:%s)\n",
      sdcard_long_state_name(m_state));
    return;
    }

  m_stop_requested.store(true);
  m_state = SDCARD_LONG_STOPPING;
  xTaskNotifyGive(m_task);
  writer->puts("SD long benchmark stop requested");
  }

void sdcard_long_benchmark::RecordOperationLocked(sdcard_long_op_type_t type,
  uint64_t offset, uint64_t requested, int64_t returned, uint64_t duration_us,
  int error_number, int file_error)
  {
  sdcard_long_op_t* operation = &m_recent[m_recent_next];
  operation->sequence = ++m_operation_sequence;
  operation->offset = offset;
  operation->requested = requested;
  operation->returned = returned;
  operation->duration_us = duration_us;
  operation->error_number = error_number;
  operation->file_error = file_error;
  operation->type = type;
  m_recent_next = (m_recent_next + 1) % SDCARD_LONG_RECENT_COUNT;
  if (m_recent_count < SDCARD_LONG_RECENT_COUNT)
    m_recent_count++;
  }

void sdcard_long_benchmark::RecordWrite(uint64_t offset, size_t requested,
  size_t returned, uint64_t duration_us, int error_number, int file_error)
  {
  uint64_t sequence;
  bool slow;
  {
  OvmsMutexLock lock(&m_mutex);
  m_write_calls++;
  m_write_bytes += returned;
  m_write_time_us += duration_us;
  if (duration_us > m_write_max_us)
    m_write_max_us = duration_us;
  if (duration_us >= 50000)
    m_write_slow_50ms++;
  if (duration_us >= 100000)
    m_write_slow_100ms++;
  if (duration_us >= 250000)
    m_write_slow_250ms++;
  if (duration_us >= 1000000)
    m_write_slow_1000ms++;
  RecordOperationLocked(SDCARD_LONG_OP_WRITE, offset, requested, returned,
    duration_us, error_number, file_error);
  sequence = m_operation_sequence;
  slow = duration_us >= ((uint64_t)m_slow_ms * 1000);
  }
  if (slow)
    ESP_LOGW(TAG,
      "SD long benchmark slow write: seq=%" PRIu64 " offset=%" PRIu64
      " requested=%u returned=%u duration=%.3fms errno=%d ferror=%d",
      sequence, offset, (unsigned)requested, (unsigned)returned,
      (double)duration_us / 1000.0, error_number, file_error);
  }

void sdcard_long_benchmark::RecordSync(uint64_t offset, uint64_t duration_us,
  bool success, int error_number, int file_error)
  {
  OvmsMutexLock lock(&m_mutex);
  m_sync_calls++;
  if (!success)
    m_sync_errors++;
  m_sync_time_us += duration_us;
  if (duration_us > m_sync_max_us)
    m_sync_max_us = duration_us;
  RecordOperationLocked(SDCARD_LONG_OP_SYNC, offset, 0, success ? 0 : -1,
    duration_us, error_number, file_error);
  }

void sdcard_long_benchmark::RecordClose(uint64_t offset, uint64_t duration_us,
  int returned, int error_number)
  {
  OvmsMutexLock lock(&m_mutex);
  m_close_time_us = duration_us;
  RecordOperationLocked(SDCARD_LONG_OP_CLOSE, offset, 0, returned,
    duration_us, error_number, 0);
  }

void sdcard_long_benchmark::SetError(const char* stage, int error_number,
  int file_error, uint64_t requested, int64_t returned, uint64_t duration_us)
  {
  OvmsMutexLock lock(&m_mutex);
  if (m_error_stage[0] == '\0')
    {
    strlcpy(m_error_stage, stage, sizeof(m_error_stage));
    m_error_number = error_number;
    m_error_ferror = file_error;
    m_error_requested = requested;
    m_error_returned = returned;
    m_error_duration_us = duration_us;
    }
  m_state = SDCARD_LONG_ERROR;
  }

bool sdcard_long_benchmark::SyncFile(FILE* file, int fd, uint64_t offset)
  {
  errno = 0;
  int64_t started = esp_timer_get_time();
  int result = fflush(file);
  int operation_errno = errno;
  const char* failure_stage = result == 0 ? NULL : "fflush";
  if (result == 0)
    {
    errno = 0;
    result = fsync(fd);
    operation_errno = errno;
    if (result != 0)
      failure_stage = "fsync";
    }
  uint64_t elapsed_us = (uint64_t)(esp_timer_get_time() - started);
  int file_error = ferror(file);
  bool success = failure_stage == NULL && file_error == 0;
  RecordSync(offset, elapsed_us, success, operation_errno, file_error);
  if (!success)
    {
    SetError(failure_stage ? failure_stage : "sync-ferror", operation_errno,
      file_error, 0, result, elapsed_us);
    ESP_LOGE(TAG,
      "Terminal SD long benchmark sync error: stage=%s offset=%" PRIu64
      " result=%d errno=%d ferror=%d duration=%.3fms",
      failure_stage ? failure_stage : "sync-ferror", offset, result,
      operation_errno, file_error, (double)elapsed_us / 1000.0);
    }
  return success;
  }

bool sdcard_long_benchmark::WaitUntil(int64_t deadline_us)
  {
  while (!m_stop_requested.load())
    {
    int64_t remaining_us = deadline_us - esp_timer_get_time();
    if (remaining_us <= 0)
      return true;
    uint64_t remaining_ms = ((uint64_t)remaining_us + 999) / 1000;
    TickType_t ticks = pdMS_TO_TICKS(remaining_ms);
    if (ticks == 0)
      ticks = 1;
    ulTaskNotifyTake(pdTRUE, ticks);
    }
  return false;
  }

void sdcard_long_benchmark::Finish(sdcard_long_state_t state)
  {
  OvmsMutexLock lock(&m_mutex);
  m_ended_us = esp_timer_get_time();
  if (m_state != SDCARD_LONG_ERROR)
    m_state = state;
  m_task = NULL;
  }

void sdcard_long_benchmark::TaskEntry(void* context)
  {
  sdcard_long_benchmark* benchmark = static_cast<sdcard_long_benchmark*>(context);
  benchmark->Run();
  vTaskDelete(NULL);
  }

void sdcard_long_benchmark::Run()
  {
  std::string path;
  uint64_t target_bytes;
  uint32_t rate_kib;
  uint32_t sync_period_s;
  {
  OvmsMutexLock lock(&m_mutex);
  path = m_path;
  target_bytes = m_target_bytes;
  rate_kib = m_rate_kib;
  sync_period_s = m_sync_period_s;
  }

  FILE* file = NULL;
  uint8_t* buffer = NULL;
  int fd = -1;
  bool dirty = false;
  bool completed = false;

  if (!MyPeripherals || !MyPeripherals->m_sdcard ||
      !MyPeripherals->m_sdcard->isavailable())
    {
    SetError("sd-unavailable", ENODEV, 0, 0, -1, 0);
    goto cleanup;
    }

  {
  struct stat file_stat;
  errno = 0;
  if (stat(path.c_str(), &file_stat) == 0)
    {
    SetError("file-exists", EEXIST, 0, 0, -1, 0);
    goto cleanup;
    }
  if (errno != ENOENT)
    {
    SetError("stat", errno, 0, 0, -1, 0);
    goto cleanup;
    }
  }

  {
  sdmmc_card_t* card = MyPeripherals->m_sdcard->m_card;
  FATFS* fs = NULL;
  DWORD free_clusters = 0;
  FRESULT result = f_getfree("1:", &free_clusters, &fs);
  if (result != FR_OK || !fs || !card)
    {
    SetError("f_getfree", EIO, 0, 0, result, 0);
    goto cleanup;
    }
  uint64_t free_bytes = (uint64_t)free_clusters * fs->csize * card->csd.sector_size;
  uint64_t cluster_size = (uint64_t)fs->csize * card->csd.sector_size;
  if (cluster_size != SDCARD_LONG_BLOCK_SIZE)
    {
    SetError("cluster-size", EINVAL, 0, SDCARD_LONG_BLOCK_SIZE, cluster_size, 0);
    goto cleanup;
    }
  if (target_bytes > free_bytes)
    {
    SetError("no-space", ENOSPC, 0, target_bytes, free_bytes, 0);
    goto cleanup;
    }
  {
  OvmsMutexLock lock(&m_mutex);
  m_sector_size = card->csd.sector_size;
  m_cluster_size = cluster_size;
  m_bus_width = MyPeripherals->m_sdcard->m_slot.width;
  m_host_frequency_khz = MyPeripherals->m_sdcard->m_host.max_freq_khz;
  }
  }

  buffer = static_cast<uint8_t*>(heap_caps_malloc(
    SDCARD_LONG_BLOCK_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
  if (!buffer)
    {
    SetError("alloc", ENOMEM, 0, SDCARD_LONG_BLOCK_SIZE, 0, 0);
    goto cleanup;
    }
  for (size_t i = 0; i < SDCARD_LONG_BLOCK_SIZE; ++i)
    buffer[i] = (uint8_t)((i % 251) + 1);

  errno = 0;
  file = fopen(path.c_str(), "w");
  if (!file)
    {
    SetError("fopen", errno, 0, 0, -1, 0);
    goto cleanup;
    }
  if (setvbuf(file, NULL, _IONBF, 0) != 0)
    {
    SetError("setvbuf", errno, ferror(file), 0, -1, 0);
    goto cleanup;
    }
  fd = fileno(file);
  if (fd < 0)
    {
    SetError("fileno", errno, ferror(file), 0, fd, 0);
    goto cleanup;
    }

  {
  int64_t run_start_us = esp_timer_get_time();
  int64_t next_sync_us = run_start_us + ((int64_t)sync_period_s * 1000000);
  {
  OvmsMutexLock lock(&m_mutex);
  m_started_us = run_start_us;
  m_state = SDCARD_LONG_RUNNING;
  }
  ESP_LOGI(TAG,
    "SD long benchmark running: path=%s bytes=%" PRIu64
    " block=%u rate=%uKiB/s sync=%us stdio=_IONBF core=1 priority=2",
    path.c_str(), target_bytes, (unsigned)SDCARD_LONG_BLOCK_SIZE,
    (unsigned)rate_kib, (unsigned)sync_period_s);

  while (!m_stop_requested.load())
    {
    uint64_t write_bytes;
    uint64_t write_calls;
    {
    OvmsMutexLock lock(&m_mutex);
    write_bytes = m_write_bytes;
    write_calls = m_write_calls;
    }
    if (write_bytes >= target_bytes)
      {
      completed = true;
      break;
      }

    int64_t now_us = esp_timer_get_time();
    if (now_us >= next_sync_us)
      {
      if (dirty)
        {
        if (!SyncFile(file, fd, write_bytes))
          break;
        dirty = false;
        now_us = esp_timer_get_time();
        }
      do
        next_sync_us += (int64_t)sync_period_s * 1000000;
      while (next_sync_us <= now_us);
      continue;
      }

    if (rate_kib > 0 && write_calls > 0)
      {
      int64_t next_write_us = run_start_us +
        (int64_t)((write_bytes * 1000000ULL) / ((uint64_t)rate_kib * 1024));
      int64_t wake_us = next_write_us < next_sync_us ? next_write_us : next_sync_us;
      if (now_us < wake_us && !WaitUntil(wake_us))
        break;
      if (esp_timer_get_time() < next_write_us)
        continue;
      }

    if (!MyPeripherals->m_sdcard->isavailable())
      {
      SetError("sd-unavailable", ENODEV, 0, SDCARD_LONG_BLOCK_SIZE, -1, 0);
      break;
      }

    size_t requested = SDCARD_LONG_BLOCK_SIZE;
    uint64_t remaining = target_bytes - write_bytes;
    if (remaining < requested)
      requested = remaining;
    errno = 0;
    int64_t operation_start_us = esp_timer_get_time();
    size_t returned = fwrite(buffer, 1, requested, file);
    int operation_errno = errno;
    int file_error = ferror(file);
    uint64_t elapsed_us = (uint64_t)(esp_timer_get_time() - operation_start_us);
    RecordWrite(write_bytes, requested, returned, elapsed_us,
      operation_errno, file_error);
    if (returned != requested || operation_errno != 0 || file_error != 0)
      {
      SetError("fwrite", operation_errno, file_error, requested, returned, elapsed_us);
      ESP_LOGE(TAG,
        "Terminal SD long benchmark write error: offset=%" PRIu64
        " requested=%u returned=%u errno=%d ferror=%d duration=%.3fms",
        write_bytes, (unsigned)requested, (unsigned)returned,
        operation_errno, file_error, (double)elapsed_us / 1000.0);
      break;
      }
    dirty = true;

    if (rate_kib == 0 && ((write_calls + 1) % SDCARD_LONG_FAIRNESS_WRITES) == 0)
      vTaskDelay(1);
    }

  if (m_error_stage[0] == '\0')
    {
    uint64_t write_bytes;
    {
    OvmsMutexLock lock(&m_mutex);
    write_bytes = m_write_bytes;
    }
    if (!SyncFile(file, fd, write_bytes))
      completed = false;
    }
  }

cleanup:
  if (file)
    {
    uint64_t write_bytes;
    {
    OvmsMutexLock lock(&m_mutex);
    write_bytes = m_write_bytes;
    }
    errno = 0;
    int64_t close_started_us = esp_timer_get_time();
    int close_result = fclose(file);
    int close_errno = errno;
    uint64_t close_elapsed_us = (uint64_t)(esp_timer_get_time() - close_started_us);
    RecordClose(write_bytes, close_elapsed_us, close_result, close_errno);
    file = NULL;
    if (close_result != 0 && m_error_stage[0] == '\0')
      SetError("fclose", close_errno, 0, 0, close_result, close_elapsed_us);
    }
  if (buffer)
    heap_caps_free(buffer);

  sdcard_long_state_t final_state;
  {
  OvmsMutexLock lock(&m_mutex);
  if (m_state == SDCARD_LONG_ERROR)
    final_state = SDCARD_LONG_ERROR;
  else if (m_stop_requested.load())
    final_state = SDCARD_LONG_STOPPED;
  else if (completed)
    final_state = SDCARD_LONG_COMPLETE;
  else
    final_state = SDCARD_LONG_ERROR;
  }
  Finish(final_state);
  ESP_LOGI(TAG, "SD long benchmark finished: state=%s path=%s",
    sdcard_long_state_name(final_state), path.c_str());
  }

void sdcard_long_benchmark::GetSnapshot(sdcard_long_snapshot_t* snapshot)
  {
  OvmsMutexLock lock(&m_mutex);
  snapshot->state = m_state;
  snapshot->path = m_path;
  snapshot->target_bytes = m_target_bytes;
  snapshot->rate_kib = m_rate_kib;
  snapshot->sync_period_s = m_sync_period_s;
  snapshot->slow_ms = m_slow_ms;
  snapshot->sector_size = m_sector_size;
  snapshot->cluster_size = m_cluster_size;
  snapshot->bus_width = m_bus_width;
  snapshot->host_frequency_khz = m_host_frequency_khz;
  snapshot->stop_requested = m_stop_requested.load();
  snapshot->started_us = m_started_us;
  snapshot->ended_us = m_ended_us;
  snapshot->write_calls = m_write_calls;
  snapshot->write_bytes = m_write_bytes;
  snapshot->write_time_us = m_write_time_us;
  snapshot->write_max_us = m_write_max_us;
  snapshot->write_slow_50ms = m_write_slow_50ms;
  snapshot->write_slow_100ms = m_write_slow_100ms;
  snapshot->write_slow_250ms = m_write_slow_250ms;
  snapshot->write_slow_1000ms = m_write_slow_1000ms;
  snapshot->sync_calls = m_sync_calls;
  snapshot->sync_errors = m_sync_errors;
  snapshot->sync_time_us = m_sync_time_us;
  snapshot->sync_max_us = m_sync_max_us;
  snapshot->close_time_us = m_close_time_us;
  strlcpy(snapshot->error_stage, m_error_stage, sizeof(snapshot->error_stage));
  snapshot->error_number = m_error_number;
  snapshot->error_ferror = m_error_ferror;
  snapshot->error_requested = m_error_requested;
  snapshot->error_returned = m_error_returned;
  snapshot->error_duration_us = m_error_duration_us;
  snapshot->operation_sequence = m_operation_sequence;
  snapshot->recent_count = m_recent_count;
  snapshot->recent_next = m_recent_next;
  memcpy(snapshot->recent, m_recent, sizeof(snapshot->recent));
  }

void sdcard_long_benchmark::Status(OvmsWriter* writer, bool show_recent)
  {
  sdcard_long_snapshot_t snapshot;
  GetSnapshot(&snapshot);
  uint64_t now_us = snapshot.ended_us ? snapshot.ended_us : esp_timer_get_time();
  uint64_t elapsed_us = snapshot.started_us && now_us > snapshot.started_us
    ? now_us - snapshot.started_us : 0;
  double progress = snapshot.target_bytes
    ? (double)snapshot.write_bytes * 100.0 / snapshot.target_bytes : 0;
  double rate_kib = elapsed_us
    ? (double)snapshot.write_bytes * 1000000.0 / elapsed_us / 1024.0 : 0;

  writer->printf("SD long benchmark status\n"
    "  State:%s Result:%s StopRequested:%d\n"
    "  Path:%s Mode:stdio-unbuffered BlockSize:%u TargetBytes:%" PRIu64
    " RateKiB:%u SyncPeriod:%us SlowMs:%u\n"
    "  SectorSize:%u ClusterSize:%" PRIu64 " BusWidth:%d HostFrequency:%dkHz"
    " Buffer:DMA+8BIT Core:1 Priority:2\n"
    "  WriteCalls:%" PRIu64 " WriteBytes:%" PRIu64
    " Progress:%.3f%% Elapsed:%.3fs ActualRate:%.3fKiB/s\n"
    "  WriteTime:%.6fs WriteMax:%.3fms WriteSlow50ms:%" PRIu64
    " WriteSlow100ms:%" PRIu64 " WriteSlow250ms:%" PRIu64
    " WriteSlow1000ms:%" PRIu64 "\n"
    "  SyncCalls:%" PRIu64 " SyncErrors:%" PRIu64
    " SyncTime:%.6fs SyncMax:%.3fms CloseTime:%.6fs\n"
    "  ErrorStage:%s ErrorErrno:%d ErrorFerror:%d ErrorRequested:%" PRIu64
    " ErrorReturned:%" PRId64 " ErrorDuration:%.3fms\n"
    "  OperationSequence:%" PRIu64 " RecentCount:%u\n",
    sdcard_long_state_name(snapshot.state), sdcard_long_result_name(snapshot.state),
    snapshot.stop_requested ? 1 : 0, snapshot.path.c_str(),
    (unsigned)SDCARD_LONG_BLOCK_SIZE, snapshot.target_bytes,
    (unsigned)snapshot.rate_kib, (unsigned)snapshot.sync_period_s,
    (unsigned)snapshot.slow_ms, (unsigned)snapshot.sector_size,
    snapshot.cluster_size, snapshot.bus_width, snapshot.host_frequency_khz,
    snapshot.write_calls, snapshot.write_bytes,
    progress, elapsed_us / 1000000.0, rate_kib,
    snapshot.write_time_us / 1000000.0, snapshot.write_max_us / 1000.0,
    snapshot.write_slow_50ms, snapshot.write_slow_100ms,
    snapshot.write_slow_250ms, snapshot.write_slow_1000ms,
    snapshot.sync_calls, snapshot.sync_errors,
    snapshot.sync_time_us / 1000000.0, snapshot.sync_max_us / 1000.0,
    snapshot.close_time_us / 1000000.0,
    snapshot.error_stage[0] ? snapshot.error_stage : "none",
    snapshot.error_number, snapshot.error_ferror, snapshot.error_requested,
    snapshot.error_returned, snapshot.error_duration_us / 1000.0,
    snapshot.operation_sequence, (unsigned)snapshot.recent_count);

  if (show_recent && snapshot.recent_count > 0)
    {
    size_t index = snapshot.recent_count == SDCARD_LONG_RECENT_COUNT
      ? snapshot.recent_next : 0;
    for (size_t count = 0; count < snapshot.recent_count; ++count)
      {
      const sdcard_long_op_t* operation = &snapshot.recent[index];
      writer->printf("  Recent Seq:%" PRIu64 " Offset:%" PRIu64
        " Type:%s Requested:%" PRIu64 " Returned:%" PRId64
        " Duration:%.3fms Errno:%d Ferror:%d\n",
        operation->sequence, operation->offset,
        sdcard_long_op_name(operation->type), operation->requested,
        operation->returned, operation->duration_us / 1000.0,
        operation->error_number, operation->file_error);
      index = (index + 1) % SDCARD_LONG_RECENT_COUNT;
      }
    }
  }

static sdcard_long_benchmark MySDCardLongBenchmark;

void sdcard_bench_long_start(int verbosity, OvmsWriter* writer, OvmsCommand* cmd,
  int argc, const char* const* argv)
  {
  const char* path = argv[0];
  if (!sdcard_bench_path_valid(path))
    {
    writer->puts("Error: path must name a file below /sd and must not contain '..'");
    return;
    }

  uint32_t total_mib;
  if (!sdcard_bench_parse_uint32(argv[1], &total_mib) || total_mib < 1 ||
      total_mib > SDCARD_LONG_TOTAL_MIB_MAX)
    {
    writer->printf("Error: total_mib must be from 1 through %u\n",
      (unsigned)SDCARD_LONG_TOTAL_MIB_MAX);
    return;
    }

  uint32_t rate_kib = 115;
  uint32_t sync_period_s = 5;
  uint32_t slow_ms = 50;
  if (argc >= 3 && (!sdcard_bench_parse_uint32(argv[2], &rate_kib) ||
      rate_kib > SDCARD_LONG_RATE_KIB_MAX))
    {
    writer->printf("Error: rate_kib must be 0 through %u (0 = unpaced)\n",
      (unsigned)SDCARD_LONG_RATE_KIB_MAX);
    return;
    }
  if (argc >= 4 && (!sdcard_bench_parse_uint32(argv[3], &sync_period_s) ||
      sync_period_s < 1 || sync_period_s > SDCARD_LONG_SYNC_PERIOD_MAX))
    {
    writer->printf("Error: sync_period must be 1 through %u seconds\n",
      (unsigned)SDCARD_LONG_SYNC_PERIOD_MAX);
    return;
    }
  if (argc >= 5 && (!sdcard_bench_parse_uint32(argv[4], &slow_ms) ||
      slow_ms < 1 || slow_ms > SDCARD_LONG_SLOW_MS_MAX))
    {
    writer->printf("Error: slow_ms must be 1 through %u\n",
      (unsigned)SDCARD_LONG_SLOW_MS_MAX);
    return;
    }

  MySDCardLongBenchmark.Start(writer, path, total_mib, rate_kib,
    sync_period_s, slow_ms);
  }

void sdcard_bench_long_status(int verbosity, OvmsWriter* writer, OvmsCommand* cmd,
  int argc, const char* const* argv)
  {
  bool show_recent = argc == 1 && strcmp(argv[0], "recent") == 0;
  if (argc == 1 && !show_recent)
    {
    writer->puts("Error: optional status argument must be 'recent'");
    return;
    }
  MySDCardLongBenchmark.Status(writer, show_recent);
  }

void sdcard_bench_long_stop(int verbosity, OvmsWriter* writer, OvmsCommand* cmd,
  int argc, const char* const* argv)
  {
  MySDCardLongBenchmark.Stop(writer);
  }

class SDCardInit
  {
  public: SDCardInit();
} MySDCardInit  __attribute__ ((init_priority (4400)));

SDCardInit::SDCardInit()
  {
  ESP_LOGI(TAG, "Initialising SD CARD (4400)");

  MyConfig.RegisterParam("sdcard", "SD CARD configuration", true, true);

  OvmsCommand* cmd_sd = MyCommandApp.RegisterCommand("sd","SD CARD framework", sdcard_status, "", 0, 0, false);
  cmd_sd->RegisterCommand("mount","Mount SD CARD",sdcard_mount);
  cmd_sd->RegisterCommand("unmount","Unmount SD CARD",sdcard_unmount,"[<maxwait_seconds>]",0,1);
  cmd_sd->RegisterCommand("status","Show SD CARD status",sdcard_status);
  OvmsCommand* cmd_bench = cmd_sd->RegisterCommand("bench", "SD CARD diagnostic benchmarks");
  cmd_bench->RegisterCommand("write", "Benchmark sequential SD writes", sdcard_bench_write,
    "<stdio|posix> </sd/path> <blocksize_bytes> <total_mib> [end|none]", 4, 5);
  OvmsCommand* cmd_long = cmd_bench->RegisterCommand("long", "Long-file SD write benchmark");
  cmd_long->RegisterCommand("start", "Start asynchronous long-file benchmark",
    sdcard_bench_long_start,
    "</sd/path> <total_mib> [rate_kib=115] [sync_period=5] [slow_ms=50]", 2, 5);
  cmd_long->RegisterCommand("status", "Show long-file benchmark status",
    sdcard_bench_long_status, "[recent]", 0, 1);
  cmd_long->RegisterCommand("stop", "Request long-file benchmark stop",
    sdcard_bench_long_stop);
  }
