#include "dallas.hh"
#include "common.hh"
#include "time.hh"

// delay between two temperature measurements
static const int delayTemp = secToTicks(CONFIG_TEMPERATURE_PERIOD_SECONDS);

namespace ds {

void TempSensor::start(Queue<Measurement> &msQueue) {
  queue = &msQueue;
  char buf[24];
  snprintf(buf, sizeof(buf), "ms_temp_%d", pin);
  xTaskCreate(collectionTask, buf, KiB(2), this, 2, nullptr);
}

[[noreturn]] void TempSensor::collectionTask(void *const arg) {
  TempSensor &sensor = *reinterpret_cast<TempSensor *>(arg);

  ESP_LOGI(logTag, "starting temp collection task for %s", sensor.name);

  while (true) {
    vTaskDelay(secToTicks(2));

    owb_rmt_driver_info driver_info;
    OneWireBus *owb = owb_rmt_initialize(&driver_info, sensor.pin,
                                         sensor.txChan, sensor.rxChan);
    owb_use_crc(owb, true);

    DS18B20_Info *device = searchTempSensor(owb);

    if (device) {
      runTempMeasurements(device, &sensor);
      ds18b20_free(&device);
    }

    owb_uninitialize(owb);

    ESP_LOGE(logTag, "sensor %s failed, restarting", sensor.name);
  }
}

DS18B20_Info *TempSensor::searchTempSensor(const OneWireBus *const owb) {
  for (bool found = false; !found;) {
    OneWireBus_SearchState search_state;
    const owb_status status = owb_search_first(owb, &search_state, &found);

    if (status != OWB_STATUS_OK) {
      ESP_LOGE(logTag, "owb search failed: %d", status);
      return nullptr;
    }
    if (!found) {
      ESP_LOGD(logTag, "temp sensor not found, retrying");
      vTaskDelay(msToTicks(500));
    }
  }

  OneWireBus_ROMCode rom_code;
  const owb_status status = owb_read_rom(owb, &rom_code);

  if (status != OWB_STATUS_OK) {
    ESP_LOGE(logTag, "could not read ROM code: %d", status);
    return nullptr;
  }

  char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
  owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
  ESP_LOGI(logTag, "found device 0x%s", rom_code_s);

  // Create DS18B20 devices on the 1-Wire bus
  DS18B20_Info *const device = ds18b20_malloc(); // heap allocation
  ds18b20_init_solo(device, owb);                // only one device on bus
  ds18b20_use_crc(device, true); // enable CRC check on all reads
  ds18b20_set_resolution(device, DS18B20_RESOLUTION_12_BIT);

  return device;
}

void TempSensor::runTempMeasurements(const DS18B20_Info *device,
                                     const TempSensor *const config) {
  int errCount = 0;
  Measurement ms{.type = MeasurementType::Temperature,
                 .sensor = config->name};
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (errCount < 4) {
    const DS18B20_ERROR err = ds18b20_convert_and_read_temp(device, &ms.temp);

    if (err != DS18B20_OK) {
      ++errCount;
      ESP_LOGW(logTag, "measurement failed in %s, err %d", config->name, err);
    } else {
      errCount = 0;
      ms.time = getTimestamp();
      if (!config->queue->putRetrying(ms)) {
        ESP_LOGE(logTag, "could not put temp measurement into queue");
      }
    }

    vTaskDelayUntil(&lastWakeTime, delayTemp);
  }
}

} // namespace ds
