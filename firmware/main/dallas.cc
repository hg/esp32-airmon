#include "dallas.hh"
#include "common.hh"
#include "settings.hh"
#include "time.hh"

namespace ds {

static std::unique_ptr<DS18B20_Info> findSensor(const OneWireBus &owb) {
  for (bool found = false; !found;) {
    OneWireBus_SearchState searchState;
    const owb_status status = owb_search_first(&owb, &searchState, &found);

    if (status != OWB_STATUS_OK) {
      ESP_LOGE(logTag, "owb search failed: %d", status);
      return nullptr;
    }
    if (!found) {
      ESP_LOGD(logTag, "temp sensor not found, retrying");
      vTaskDelay(msToTicks(500));
    }
  }

  OneWireBus_ROMCode romCode;
  const owb_status status = owb_read_rom(&owb, &romCode);

  if (status != OWB_STATUS_OK) {
    ESP_LOGE(logTag, "could not read ROM code: %d", status);
    return nullptr;
  }

  char romCodeStr[OWB_ROM_CODE_STRING_LENGTH];
  owb_string_from_rom_code(romCode, romCodeStr, sizeof(romCodeStr));
  ESP_LOGI(logTag, "found device 0x%s", romCodeStr);

  std::unique_ptr<DS18B20_Info> device{ds18b20_malloc()};

  ds18b20_init_solo(device.get(), &owb);
  ds18b20_use_crc(device.get(), true);
  ds18b20_set_resolution(device.get(), DS18B20_RESOLUTION_12_BIT);

  return device;
}

void TempSensor::runMeasurements(const DS18B20_Info &device) {
  Measurement ms{.type = MeasurementType::TEMPERATURE, .sensor = name};
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (int errCount = 0; errCount < 4;) {
    const DS18B20_ERROR err = ds18b20_convert_and_read_temp(&device, &ms.temp);

    if (err == DS18B20_OK) {
      errCount = 0;
      ms.time = getTimestamp();
      if (!queue->putRetrying(ms)) {
        ESP_LOGE(logTag, "could not put temp measurement into queue");
      }
    } else {
      ++errCount;
      ESP_LOGW(logTag, "measurement failed in %s, err 0x%x", name, err);
    }

    vTaskDelayUntil(&lastWakeTime, appSettings.period.temp);
  }
}

[[noreturn]] void TempSensor::collectionTask(void *const arg) {
  TempSensor &sensor = *reinterpret_cast<TempSensor *>(arg);

  ESP_LOGI(logTag, "starting temp collection task for %s", sensor.name);

  while (true) {
    vTaskDelay(secToTicks(2));

    owb_rmt_driver_info driver_info{};
    std::unique_ptr<OneWireBus, decltype(&owb_uninitialize)> owb{
        owb_rmt_initialize(&driver_info, sensor.pin, sensor.txChan,
                           sensor.rxChan),
        owb_uninitialize};

    owb_use_crc(owb.get(), true);

    const std::unique_ptr<DS18B20_Info> device = findSensor(*owb);
    if (device) {
      sensor.runMeasurements(*device);
    }

    ESP_LOGE(logTag, "sensor %s failed, restarting", sensor.name);
  }
}

void TempSensor::start(Queue<Measurement> &msQueue) {
  queue = &msQueue;
  char buf[24];
  snprintf(buf, sizeof(buf), "temp_%s", name);
  xTaskCreate(collectionTask, buf, KiB(2), this, 2, nullptr);
}

} // namespace ds
