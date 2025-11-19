// based on: https://github.com/DomiStyle/esphome-panasonic-ac
#include "esppac.h"

#include "esphome/core/log.h"

namespace esphome {
namespace sinclair_ac {

static const char *const TAG = "sinclair_ac";

climate::ClimateTraits SinclairAC::traits()
{
    auto traits = climate::ClimateTraits();

    traits.set_supports_action(false);

    traits.set_supports_current_temperature(true);
    traits.set_supports_two_point_target_temperature(false);
    traits.set_visual_min_temperature(MIN_TEMPERATURE);
    traits.set_visual_max_temperature(MAX_TEMPERATURE);
    traits.set_visual_temperature_step(TEMPERATURE_STEP);

    traits.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_AUTO, climate::CLIMATE_MODE_COOL,
                                climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_FAN_ONLY, climate::CLIMATE_MODE_DRY});

    traits.add_supported_custom_fan_mode(fan_modes::FAN_AUTO);
    traits.add_supported_custom_fan_mode(fan_modes::FAN_QUIET);
    traits.add_supported_custom_fan_mode(fan_modes::FAN_LOW);
    traits.add_supported_custom_fan_mode(fan_modes::FAN_MEDL);
    traits.add_supported_custom_fan_mode(fan_modes::FAN_MED);
    traits.add_supported_custom_fan_mode(fan_modes::FAN_MEDH);
    traits.add_supported_custom_fan_mode(fan_modes::FAN_HIGH);
    traits.add_supported_custom_fan_mode(fan_modes::FAN_TURBO);

    traits.set_supported_swing_modes({climate::CLIMATE_SWING_OFF, climate::CLIMATE_SWING_BOTH,
                                      climate::CLIMATE_SWING_VERTICAL, climate::CLIMATE_SWING_HORIZONTAL});

    return traits;
}

void SinclairAC::setup()
{
  // Initialize times
    this->init_time_ = millis();
    this->last_packet_sent_ = millis();
    this->last_atc_sensor_update_ = 0;

    // Initialize temperature source to AC own sensor by default
    this->temp_source_state_ = temp_source_options::AC_OWN;
    ESP_LOGD(TAG, "Temperature source initialized to: %s", this->temp_source_state_.c_str());

    ESP_LOGI(TAG, "Sinclair AC component v%s starting...", VERSION);

#ifdef USE_ESP32_BLE_TRACKER
    // Register BLE device listener for ATC sensor
    if (esp32_ble_tracker::global_esp32_ble_tracker != nullptr) {
        esp32_ble_tracker::global_esp32_ble_tracker->register_listener(this);
        ESP_LOGI(TAG, "BLE tracker listener registered for dynamic ATC sensor support");
    } else {
        ESP_LOGW(TAG, "BLE tracker not available - ATC sensor support disabled");
    }
#else
    ESP_LOGD(TAG, "BLE tracker not compiled - ATC sensor support not available");
#endif
}

void SinclairAC::loop()
{
    read_data();  // Read data from UART (if there is any)
    check_atc_sensor_timeout();  // Check if ATC sensor has timed out
}

void SinclairAC::read_data()
{
    while (available())  // Read while data is available
    {
        /* If we had a packet or a packet had not been decoded yet - do not recieve more data */
        if (this->serialProcess_.state == STATE_COMPLETE)
        {
            break;
        }
        uint8_t c;
        this->read_byte(&c);  // Store in receive buffer

        if (this->serialProcess_.state == STATE_RESTART)
        {
            this->serialProcess_.data.clear();
            this->serialProcess_.state = STATE_WAIT_SYNC;
        }
        
        this->serialProcess_.data.push_back(c);
        if (this->serialProcess_.data.size() >= DATA_MAX)
        {
            this->serialProcess_.data.clear();
            continue;
        }
        switch (this->serialProcess_.state)
        {
            case STATE_WAIT_SYNC:
                /* Frame begins with 0x7E 0x7E LEN CMD
                   LEN - frame length in bytes
                   CMD - command
                 */
                if (c != 0x7E && 
                    this->serialProcess_.data.size() > 2 && 
                    this->serialProcess_.data[this->serialProcess_.data.size()-2] == 0x7E && 
                    this->serialProcess_.data[this->serialProcess_.data.size()-3] == 0x7E)
                {
                    this->serialProcess_.data.clear();

                    this->serialProcess_.data.push_back(0x7E);
                    this->serialProcess_.data.push_back(0x7E);
                    this->serialProcess_.data.push_back(c);

                    this->serialProcess_.frame_size = c;
                    this->serialProcess_.state = STATE_RECIEVE;
                }
                break;
            case STATE_RECIEVE:
                this->serialProcess_.frame_size--;
                if (this->serialProcess_.frame_size == 0)
                {
                    /* WE HAVE A FRAME FROM AC */
                    this->serialProcess_.state = STATE_COMPLETE;
                }
                break;
            case STATE_RESTART:
            case STATE_COMPLETE:
                break;
            default:
                this->serialProcess_.state = STATE_WAIT_SYNC;
                this->serialProcess_.data.clear();
                break;
        }

    }
}

void SinclairAC::update_current_temperature(float temperature)
{
    if (temperature > TEMPERATURE_THRESHOLD) {
        ESP_LOGW(TAG, "Received out of range inside temperature: %f", temperature);
        return;
    }

    this->current_temperature = temperature;
}

void SinclairAC::update_target_temperature(float temperature)
{
    if (temperature > TEMPERATURE_THRESHOLD) {
        ESP_LOGW(TAG, "Received out of range target temperature %.2f", temperature);
        return;
    }

    this->target_temperature = temperature;
}

void SinclairAC::update_swing_horizontal(const std::string &swing)
{
    this->horizontal_swing_state_ = swing;

    if (this->horizontal_swing_select_ != nullptr &&
        this->horizontal_swing_select_->state != this->horizontal_swing_state_)
    {
        this->horizontal_swing_select_->publish_state(this->horizontal_swing_state_);
    }
}

void SinclairAC::update_swing_vertical(const std::string &swing)
{
    this->vertical_swing_state_ = swing;

    if (this->vertical_swing_select_ != nullptr && 
        this->vertical_swing_select_->state != this->vertical_swing_state_)
    {
        this->vertical_swing_select_->publish_state(this->vertical_swing_state_);
    }
}

void SinclairAC::update_display(const std::string &display)
{
    this->display_state_ = display;

    if (this->display_select_ != nullptr && 
        this->display_select_->state != this->display_state_)
    {
        this->display_select_->publish_state(this->display_state_);
    }
}

void SinclairAC::update_display_unit(const std::string &display_unit)
{
    this->display_unit_state_ = display_unit;

    if (this->display_unit_select_ != nullptr && 
        this->display_unit_select_->state != this->display_unit_state_)
    {
        this->display_unit_select_->publish_state(this->display_unit_state_);
    }
}

void SinclairAC::update_temp_source(const std::string &temp_source)
{
    ESP_LOGD(TAG, "Temperature source changing from '%s' to '%s'", 
             this->temp_source_state_.c_str(), temp_source.c_str());
    
    this->temp_source_state_ = temp_source;

    if (this->temp_source_select_ != nullptr && 
        this->temp_source_select_->state != this->temp_source_state_)
    {
        this->temp_source_select_->publish_state(this->temp_source_state_);
    }
}

void SinclairAC::update_plasma(bool plasma)
{
    this->plasma_state_ = plasma;

    if (this->plasma_switch_ != nullptr)
    {
        this->plasma_switch_->publish_state(this->plasma_state_);
    }
}

void SinclairAC::update_beeper(bool beeper)
{
    this->beeper_state_ = beeper;

    if (this->beeper_switch_ != nullptr)
    {
        this->beeper_switch_->publish_state(this->beeper_state_);
    }
}

void SinclairAC::update_sleep(bool sleep)
{
    this->sleep_state_ = sleep;

    if (this->sleep_switch_ != nullptr)
    {
        this->sleep_switch_->publish_state(this->sleep_state_);
    }
}

void SinclairAC::update_xfan(bool xfan)
{
    this->xfan_state_ = xfan;

    if (this->xfan_switch_ != nullptr)
    {
        this->xfan_switch_->publish_state(this->xfan_state_);
    }
}

void SinclairAC::update_save(bool save)
{
    this->save_state_ = save;

    if (this->save_switch_ != nullptr)
    {
        this->save_switch_->publish_state(this->save_state_);
    }
}

climate::ClimateAction SinclairAC::determine_action()
{
    if (this->mode == climate::CLIMATE_MODE_OFF) {
        return climate::CLIMATE_ACTION_OFF;
    } else if (this->mode == climate::CLIMATE_MODE_FAN_ONLY) {
        return climate::CLIMATE_ACTION_FAN;
    } else if (this->mode == climate::CLIMATE_MODE_DRY) {
        return climate::CLIMATE_ACTION_DRYING;
    } else if ((this->mode == climate::CLIMATE_MODE_COOL || this->mode == climate::CLIMATE_MODE_HEAT_COOL) &&
                this->current_temperature + TEMPERATURE_TOLERANCE >= this->target_temperature) {
        return climate::CLIMATE_ACTION_COOLING;
    } else if ((this->mode == climate::CLIMATE_MODE_HEAT || this->mode == climate::CLIMATE_MODE_HEAT_COOL) &&
                this->current_temperature - TEMPERATURE_TOLERANCE <= this->target_temperature) {
        return climate::CLIMATE_ACTION_HEATING;
    } else {
        return climate::CLIMATE_ACTION_IDLE;
    }
}

/*
 * Sensor handling
 */

void SinclairAC::set_current_temperature_sensor(sensor::Sensor *current_temperature_sensor)
{
    this->current_temperature_sensor_ = current_temperature_sensor;
    this->current_temperature_sensor_->add_on_state_callback([this](float state)
        {
            this->current_temperature = state;
            this->publish_state();
        });
}

void SinclairAC::set_vertical_swing_select(select::Select *vertical_swing_select)
{
    this->vertical_swing_select_ = vertical_swing_select;
    this->vertical_swing_select_->add_on_state_callback([this](const std::string &value, size_t index) {
        if (value == this->vertical_swing_state_)
            return;
        this->on_vertical_swing_change(value);
    });
}

void SinclairAC::set_horizontal_swing_select(select::Select *horizontal_swing_select)
{
    this->horizontal_swing_select_ = horizontal_swing_select;
    this->horizontal_swing_select_->add_on_state_callback([this](const std::string &value, size_t index) {
        if (value == this->horizontal_swing_state_)
            return;
        this->on_horizontal_swing_change(value);
    });
}

void SinclairAC::set_display_select(select::Select *display_select)
{
    this->display_select_ = display_select;
    this->display_select_->add_on_state_callback([this](const std::string &value, size_t index) {
        if (value == this->display_state_)
            return;
        this->on_display_change(value);
    });
}

void SinclairAC::set_display_unit_select(select::Select *display_unit_select)
{
    this->display_unit_select_ = display_unit_select;
    this->display_unit_select_->add_on_state_callback([this](const std::string &value, size_t index) {
        if (value == this->display_unit_state_)
            return;
        this->on_display_unit_change(value);
    });
}

void SinclairAC::set_temp_source_select(select::Select *temp_source_select)
{
    this->temp_source_select_ = temp_source_select;
    this->temp_source_select_->add_on_state_callback([this](const std::string &value, size_t index) {
        if (value == this->temp_source_state_)
            return;
        ESP_LOGD(TAG, "User changed temperature source via UI to: %s", value.c_str());
        this->on_temp_source_change(value);
    });
}

void SinclairAC::set_atc_mac_address_text(text::Text *atc_mac_address_text)
{
    this->atc_mac_address_text_ = atc_mac_address_text;
    ESP_LOGD(TAG, "ATC MAC address text input registered");
}

void SinclairAC::set_ac_indoor_temp_sensor(sensor::Sensor *ac_indoor_temp_sensor)
{
    this->ac_indoor_temp_sensor_ = ac_indoor_temp_sensor;
}

void SinclairAC::set_atc_room_temp_sensor(sensor::Sensor *atc_room_temp_sensor)
{
    this->atc_room_temp_sensor_ = atc_room_temp_sensor;
}

void SinclairAC::set_atc_room_humidity_sensor(sensor::Sensor *atc_room_humidity_sensor)
{
    this->atc_room_humidity_sensor_ = atc_room_humidity_sensor;
}

void SinclairAC::set_atc_battery_sensor(sensor::Sensor *atc_battery_sensor)
{
    this->atc_battery_sensor_ = atc_battery_sensor;
}

void SinclairAC::set_plasma_switch(switch_::Switch *plasma_switch)
{
    this->plasma_switch_ = plasma_switch;
    this->plasma_switch_->add_on_state_callback([this](bool state) {
        if (state == this->plasma_state_)
            return;
        this->on_plasma_change(state);
    });
}

void SinclairAC::set_beeper_switch(switch_::Switch *beeper_switch)
{
    this->beeper_switch_ = beeper_switch;
    this->beeper_switch_->add_on_state_callback([this](bool state) {
        if (state == this->beeper_state_)
            return;
        this->on_beeper_change(state);
    });
}

void SinclairAC::set_sleep_switch(switch_::Switch *sleep_switch)
{
    this->sleep_switch_ = sleep_switch;
    this->sleep_switch_->add_on_state_callback([this](bool state) {
        if (state == this->sleep_state_)
            return;
        this->on_sleep_change(state);
    });
}

void SinclairAC::set_xfan_switch(switch_::Switch *xfan_switch)
{
    this->xfan_switch_ = xfan_switch;
    this->xfan_switch_->add_on_state_callback([this](bool state) {
        if (state == this->xfan_state_)
            return;
        this->on_xfan_change(state);
    });
}

void SinclairAC::set_save_switch(switch_::Switch *save_switch)
{
    this->save_switch_ = save_switch;
    this->save_switch_->add_on_state_callback([this](bool state) {
        if (state == this->save_state_)
            return;
        this->on_save_change(state);
    });
}

/*
 * ATC Sensor timeout check and fallback logic
 */

void SinclairAC::check_atc_sensor_timeout()
{
    // Only check if we're using external ATC sensor
    if (!is_using_atc_sensor()) {
        return;
    }

    // Check if MAC address is valid (not empty)
    if (this->atc_mac_address_text_ == nullptr || this->atc_mac_address_text_->state.empty()) {
        if (this->atc_sensor_valid_) {
            ESP_LOGW(TAG, "ATC MAC address is empty, falling back to AC own sensor");
            ESP_LOGD(TAG, "Fallback trigger: Empty MAC address");
            this->temp_source_state_ = temp_source_options::AC_OWN;
            this->update_temp_source(this->temp_source_state_);
            this->atc_sensor_valid_ = false;
        }
        return;
    }

    // Check if sensor has timed out (15 minutes)
    if (this->atc_sensor_valid_ && this->last_atc_sensor_update_ > 0) {
        uint32_t time_since_update = millis() - this->last_atc_sensor_update_;
        if (time_since_update > ATC_SENSOR_TIMEOUT_MS) {
            ESP_LOGW(TAG, "ATC sensor timeout (no data for 15 minutes), falling back to AC own sensor");
            ESP_LOGD(TAG, "Fallback trigger: Timeout after %lu ms", time_since_update);
            this->temp_source_state_ = temp_source_options::AC_OWN;
            this->update_temp_source(this->temp_source_state_);
            this->atc_sensor_valid_ = false;
        }
    }
}

void SinclairAC::update_atc_sensor(float temperature, float humidity)
{
    this->last_atc_sensor_update_ = millis();
    this->last_atc_temperature_ = temperature;
    this->last_atc_humidity_ = humidity;
    this->atc_sensor_valid_ = true;

    ESP_LOGD(TAG, "ATC sensor data updated: Temp=%.2f°C, Humidity=%.1f%%", temperature, humidity);
#ifdef SINCLAIR_AC_VERBOSE_LOG
    ESP_LOGV(TAG, "ATC sensor update timestamp: %lu ms", this->last_atc_sensor_update_);
#endif

    // Publish to sensors if they exist
    if (this->atc_room_temp_sensor_ != nullptr) {
        this->atc_room_temp_sensor_->publish_state(temperature);
#ifdef SINCLAIR_AC_VERBOSE_LOG
        ESP_LOGV(TAG, "Published ATC temperature to sensor: %.2f°C", temperature);
#endif
    }

    if (this->atc_room_humidity_sensor_ != nullptr) {
        this->atc_room_humidity_sensor_->publish_state(humidity);
#ifdef SINCLAIR_AC_VERBOSE_LOG
        ESP_LOGV(TAG, "Published ATC humidity to sensor: %.1f%%", humidity);
#endif
    }

    // Update current temperature if using ATC sensor
    if (is_using_atc_sensor()) {
        ESP_LOGD(TAG, "Updating climate current temperature from ATC sensor: %.2f°C", temperature);
        this->current_temperature = temperature;
        this->publish_state();
    }
}

bool SinclairAC::is_using_atc_sensor()
{
    return this->temp_source_state_ == temp_source_options::EXTERNAL_ATC;
}

void SinclairAC::update_atc_battery(float battery_percent)
{
    this->last_atc_battery_ = battery_percent;
    
    ESP_LOGD(TAG, "ATC battery updated: %.0f%%", battery_percent);
    
    // Publish to battery sensor if it exists
    if (this->atc_battery_sensor_ != nullptr) {
        this->atc_battery_sensor_->publish_state(battery_percent);
    }
}

#ifdef USE_ESP32_BLE_TRACKER
/*
 * BLE Advertisement parsing for ATC (Xiaomi ATC1441 custom firmware)
 */

std::string SinclairAC::normalize_mac_(const std::string &mac)
{
    std::string normalized;
    for (char c : mac) {
        if (c != ':' && c != '-' && c != ' ') {
            normalized += std::toupper(c);
        }
    }
    return normalized;
}

bool SinclairAC::macs_equal_(const std::string &mac1, const std::string &mac2)
{
    return normalize_mac_(mac1) == normalize_mac_(mac2);
}

bool SinclairAC::parse_device(const esp32_ble_tracker::ESPBTDevice &device)
{
    // Only process if we have a MAC address configured
    if (this->atc_mac_address_text_ == nullptr || this->atc_mac_address_text_->state.empty()) {
#ifdef SINCLAIR_AC_VERBOSE_LOG
        ESP_LOGV(TAG, "No ATC MAC address configured, skipping BLE device");
#endif
        return false;
    }

    std::string configured_mac = this->atc_mac_address_text_->state;
    
    // Check if advertiser address matches
    std::string device_mac = device.address_str();
    bool mac_matches = macs_equal_(device_mac, configured_mac);
    
#ifdef SINCLAIR_AC_VERBOSE_LOG
    if (!mac_matches) {
        ESP_LOGV(TAG, "BLE device MAC %s doesn't match configured %s", 
                 device_mac.c_str(), configured_mac.c_str());
    }
#endif
    
    // Look for ATC custom firmware service data (UUID 0x181A - Environmental Sensing)
    for (auto &service_data : device.get_service_datas()) {
        if (service_data.uuid.get_uuid().len != ESP_UUID_LEN_16) {
            continue;
        }
        
        uint16_t uuid = service_data.uuid.get_uuid().uuid.uuid16;
        if (uuid != 0x181A) {
            continue;
        }
        
        const auto &data = service_data.data;
        
        // ATC format: minimum 13 bytes
        // Bytes 0-5: MAC (reversed)
        // Bytes 6-7: Temperature in centi-degrees C (int16, big-endian)
        // Bytes 8-9: Humidity in centi-% (uint16, big-endian)
        // Byte 10: Battery %
        // Bytes 11-12: Battery mV (optional)
        // Byte 13: Packet counter (optional)
        
        if (data.size() < 11) {
#ifdef SINCLAIR_AC_VERBOSE_LOG
            ESP_LOGV(TAG, "ATC service data too short: %d bytes (need 11)", data.size());
#endif
            continue;
        }
        
        // Check embedded MAC if present (first 6 bytes, reversed order)
        if (data.size() >= 6) {
            char embedded_mac[18];
            snprintf(embedded_mac, sizeof(embedded_mac), "%02X:%02X:%02X:%02X:%02X:%02X",
                     data[5], data[4], data[3], data[2], data[1], data[0]);
            
            if (macs_equal_(embedded_mac, configured_mac)) {
                mac_matches = true;
                ESP_LOGD(TAG, "ATC MAC match found (embedded): %s", embedded_mac);
            }
#ifdef SINCLAIR_AC_VERBOSE_LOG
            else {
                ESP_LOGV(TAG, "Embedded MAC %s doesn't match configured %s", 
                         embedded_mac, configured_mac.c_str());
            }
#endif
        }
        
        if (!mac_matches) {
            continue;
        }
        
        // Parse temperature (int16, big-endian, in centi-degrees C)
        int16_t temp_raw = (int16_t)((data[6] << 8) | data[7]);
        float temperature = temp_raw / 100.0f;
        
        // Parse humidity (uint16, big-endian, in centi-%)
        uint16_t hum_raw = (uint16_t)((data[8] << 8) | data[9]);
        float humidity = hum_raw / 100.0f;
        
        // Parse battery percentage
        uint8_t battery = data[10];
        
        ESP_LOGD(TAG, "ATC BLE data received from %s: Temp=%.2f°C, Hum=%.1f%%, Batt=%d%%",
                 device_mac.c_str(), temperature, humidity, battery);
#ifdef SINCLAIR_AC_VERBOSE_LOG
        ESP_LOGV(TAG, "Raw ATC data: temp_raw=%d, hum_raw=%d, battery=%d", temp_raw, hum_raw, battery);
#endif
        
        // Update sensors
        update_atc_sensor(temperature, humidity);
        update_atc_battery((float)battery);
        
        return true;
    }
    
    return false;
}
#endif

/*
 * Debugging
 */

void SinclairAC::log_packet(std::vector<uint8_t> data, bool outgoing)
{
    if (outgoing) {
        ESP_LOGV(TAG, "TX: %s", format_hex_pretty(data).c_str());
    } else {
        ESP_LOGV(TAG, "RX: %s", format_hex_pretty(data).c_str());
    }
}

}  // namespace sinclair_ac
}  // namespace esphome
