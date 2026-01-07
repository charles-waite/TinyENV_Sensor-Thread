// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Standard Matter Temperature Sensor Endpoint but modified to report 
// battery level to Matter servers.

#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include <Matter.h>
#include "MatterEndpoints/MatterTemperatureSensorBattery.h"
#include <esp_matter_cluster.h>
#include <cstring>

using namespace esp_matter;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

bool MatterTemperatureSensorBattery::attributeChangeCB(uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val) {
  bool ret = true;
  if (!started) {
    log_e("Matter Temperature Sensor device has not begun.");
    return false;
  }

  log_d("Temperature Sensor Attr update callback: endpoint: %u, cluster: %lu, attribute: %lu, val: %u", endpoint_id, cluster_id, attribute_id, val->val.u32);
  return ret;
}

MatterTemperatureSensorBattery::MatterTemperatureSensorBattery() {}

MatterTemperatureSensorBattery::~MatterTemperatureSensorBattery() {
  end();
}

bool MatterTemperatureSensorBattery::begin(int16_t _rawTemperature) {
  struct ArduinoMatterInitShim : public ArduinoMatter {
    using ArduinoMatter::_init;   // expose protected as public within this derived type
  };
  ArduinoMatterInitShim::_init();
  if (getEndPointId() != 0) {
    log_e("Temperature Sensor with Endpoint Id %d device has already been created.", getEndPointId());
    return false;
  }

  temperature_sensor::config_t temperature_sensor_config;
  temperature_sensor_config.temperature_measurement.measured_value = _rawTemperature;
  temperature_sensor_config.temperature_measurement.min_measured_value = nullptr;
  temperature_sensor_config.temperature_measurement.max_measured_value = nullptr;

    // endpoint handles can be used to add/modify clusters
  endpoint_t *ep =
      temperature_sensor::create(node::get(), &temperature_sensor_config, ENDPOINT_FLAG_NONE, (void *)this);

  if (ep == nullptr) {
    log_e("Failed to create Temperature Sensor endpoint");
    return false;
  }

// ---- Add Power Source cluster (battery reporting) ----
{
  esp_matter::cluster::power_source::config_t ps_cfg{};
  ps_cfg.status = 1; // Active
  ps_cfg.order  = 0;
  strncpy(ps_cfg.description, "Battery", sizeof(ps_cfg.description) - 1);
  ps_cfg.description[sizeof(ps_cfg.description) - 1] = '\0';

  uint32_t features = 0;
  features |= (uint32_t)PowerSource::Feature::kBattery;
  features |= (uint32_t)PowerSource::Feature::kRechargeable;

  log_i("Creating PowerSource cluster, features=0x%08lx",
        (unsigned long)features);

  auto *ps_cluster =
      esp_matter::cluster::power_source::create(
          ep,
          &ps_cfg,
          CLUSTER_FLAG_SERVER,
          features
      );

  if (!ps_cluster) {
    log_e("PowerSource cluster create FAILED");
  } else {
    log_i("PowerSource cluster created");

    namespace ps_attr = esp_matter::cluster::power_source::attribute;

    ps_attr::create_bat_present(ps_cluster, true);

    ps_attr::create_bat_percent_remaining(
        ps_cluster,
        ::nullable((uint8_t)200),
        ::nullable((uint8_t)0),
        ::nullable((uint8_t)200)
    );

    ps_attr::create_bat_voltage(
        ps_cluster,
        ::nullable((uint32_t)3854),
        ::nullable((uint32_t)2500),
        ::nullable((uint32_t)5000)
    );

    ps_attr::create_bat_charge_state(ps_cluster, 0);
  }
}

  rawTemperature = _rawTemperature;
  setEndPointId(esp_matter::endpoint::get_id(ep));
  log_i("Temperature Sensor created with endpoint_id %d", getEndPointId());

  started = true;
  return true;
}

void MatterTemperatureSensorBattery::end() {
  started = false;
}

bool MatterTemperatureSensorBattery::setRawTemperature(int16_t _rawTemperature) {
  if (!started) {
    log_e("Matter Temperature Sensor device has not begun.");
    return false;
  }

  // avoid processing if there was no change
  if (rawTemperature == _rawTemperature) {
    return true;
  }

  esp_matter_attr_val_t temperatureVal = esp_matter_invalid(NULL);

  if (!getAttributeVal(TemperatureMeasurement::Id, TemperatureMeasurement::Attributes::MeasuredValue::Id, &temperatureVal)) {
    log_e("Failed to get Temperature Sensor Attribute.");
    return false;
  }
  if (temperatureVal.val.i16 != _rawTemperature) {
    temperatureVal.val.i16 = _rawTemperature;
    bool ret;
    ret = updateAttributeVal(TemperatureMeasurement::Id, TemperatureMeasurement::Attributes::MeasuredValue::Id, &temperatureVal);
    if (!ret) {
      log_e("Failed to update Temperature Sensor Attribute.");
      return false;
    }
    rawTemperature = _rawTemperature;
  }
  log_v("Temperature Sensor set to %.02fC", (float)_rawTemperature / 100.00);

  return true;
}

bool MatterTemperatureSensorBattery::setBatteryPercent(uint8_t percent) {
  if (!started) {
    log_e("Matter Temperature Sensor device has not begun.");
    return false;
  }

  // Matter Power Source BatPercentRemaining is commonly 0..200 (0.5% units)
  uint8_t halfPct = (percent >= 100) ? 200 : (uint8_t)(percent * 2);

  esp_matter_attr_val_t v = esp_matter_invalid(NULL);

  if (!getAttributeVal(PowerSource::Id, PowerSource::Attributes::BatPercentRemaining::Id, &v)) {
    log_w("BatPercentRemaining attribute not found (Power Source cluster missing?)");
    return false;
  }

  v.val.u8 = halfPct;
  return updateAttributeVal(PowerSource::Id, PowerSource::Attributes::BatPercentRemaining::Id, &v);
}

bool MatterTemperatureSensorBattery::setBatteryVoltageMv(uint32_t mv) {
  if (!started) {
    log_e("Matter Temperature Sensor device has not begun.");
    return false;
  }

  esp_matter_attr_val_t v = esp_matter_invalid(NULL);

  if (!getAttributeVal(PowerSource::Id, PowerSource::Attributes::BatVoltage::Id, &v)) {
    log_w("BatVoltage attribute not found (Power Source cluster missing?)");
    return false;
  }

  v.val.u32 = mv;
  return updateAttributeVal(PowerSource::Id, PowerSource::Attributes::BatVoltage::Id, &v);
}
#endif /* CONFIG_ESP_MATTER_ENABLE_DATA_MODEL */
