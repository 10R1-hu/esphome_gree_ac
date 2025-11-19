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

    // Initialize preference objects
    this->pref_temp_source_ = global_preferences->make_preference<uint8_t>(fnv1_hash("sinclair_temp_src"));
    this->pref_atc_mac_ = global_preferences->make_preference<std::string>(fnv1_hash("sinclair_atc_mac"));
    this->pref_display_ = global_preferences->make_preference<uint8_t>(fnv1_hash("sinclair_display"));
    this->pref_display_unit_ = global_preferences->make_preference<uint8_t>(fnv1_hash("sinclair_disp_unit"));
    this->pref_vertical_swing_ = global_preferences->make_preference<uint8_t>(fnv1_hash("sinclair_v_swing"));
    this->pref_horizontal_swing_ = global_preferences->make_preference<uint8_t>(fnv1_hash("sinclair_h_swing"));
    this->pref_plasma_ = global_preferences->make_preference<bool>(fnv1_hash("sinclair_plasma"));
    this->pref_beeper_ = global_preferences->make_preference<bool>(fnv1_hash("sinclair_beeper"));
    this->pref_sleep_ = global_preferences->make_preference<bool>(fnv1_hash("sinclair_sleep"));
    this->pref_xfan_ = global_preferences->make_preference<bool>(fnv1_hash("sinclair_xfan"));
    this->pref_save_ = global_preferences->make_preference<bool>(fnv1_hash("sinclair_save"));

    // Load saved preferences
    load_preferences();

    ESP_LOGI(TAG, "Sinclair AC component v%s starting...", VERSION);

#ifdef USE_ESP32_BLE_TRACKER
    // Register BLE device listener for ATC sensor
    if (esp32_ble_tracker::global_esp32_ble_tracker != nullptr) {
        esp32_ble_tracker::global_esp32_ble_tracker->register_listener(this);
        ESP_LOGI(TAG, "BLE tracker listener registered for dynamic ATC sensor support");
    } else {
        ESP_LOGW(TAG, "BLE tracker not available - ATC sensor support disabled");
    }
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
        this->save_vertical_swing_preference();
    });
}

void SinclairAC::set_horizontal_swing_select(select::Select *horizontal_swing_select)
{
    this->horizontal_swing_select_ = horizontal_swing_select;
    this->horizontal_swing_select_->add_on_state_callback([this](const std::string &value, size_t index) {
        if (value == this->horizontal_swing_state_)
            return;
        this->on_horizontal_swing_change(value);
        this->save_horizontal_swing_preference();
    });
}

void SinclairAC::set_display_select(select::Select *display_select)
{
    this->display_select_ = display_select;
    this->display_select_->add_on_state_callback([this](const std::string &value, size_t index) {
        if (value == this->display_state_)
            return;
        this->on_display_change(value);
        this->save_display_preference();
    });
}

void SinclairAC::set_display_unit_select(select::Select *display_unit_select)
{
    this->display_unit_select_ = display_unit_select;
    this->display_unit_select_->add_on_state_callback([this](const std::string &value, size_t index) {
        if (value == this->display_unit_state_)
            return;
        this->on_display_unit_change(value);
        this->save_display_unit_preference();
    });
}

void SinclairAC::set_temp_source_select(select::Select *temp_source_select)
{
    this->temp_source_select_ = temp_source_select;
    this->temp_source_select_->add_on_state_callback([this](const std::string &value, size_t index) {
        if (value == this->temp_source_state_)
            return;
        this->on_temp_source_change(value);
        this->save_temp_source_preference();
    });
}

void SinclairAC::set_atc_mac_address_text(text::Text *atc_mac_address_text)
{
    this->atc_mac_address_text_ = atc_mac_address_text;
    // Add callback to save when MAC address changes
    this->atc_mac_address_text_->add_on_state_callback([this](const std::string &value) {
        this->save_atc_mac_preference();
    });
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
        this->save_plasma_preference();
    });
}

void SinclairAC::set_beeper_switch(switch_::Switch *beeper_switch)
{
    this->beeper_switch_ = beeper_switch;
    this->beeper_switch_->add_on_state_callback([this](bool state) {
        if (state == this->beeper_state_)
            return;
        this->on_beeper_change(state);
        this->save_beeper_preference();
    });
}

void SinclairAC::set_sleep_switch(switch_::Switch *sleep_switch)
{
    this->sleep_switch_ = sleep_switch;
    this->sleep_switch_->add_on_state_callback([this](bool state) {
        if (state == this->sleep_state_)
            return;
        this->on_sleep_change(state);
        this->save_sleep_preference();
    });
}

void SinclairAC::set_xfan_switch(switch_::Switch *xfan_switch)
{
    this->xfan_switch_ = xfan_switch;
    this->xfan_switch_->add_on_state_callback([this](bool state) {
        if (state == this->xfan_state_)
            return;
        this->on_xfan_change(state);
        this->save_xfan_preference();
    });
}

void SinclairAC::set_save_switch(switch_::Switch *save_switch)
{
    this->save_switch_ = save_switch;
    this->save_switch_->add_on_state_callback([this](bool state) {
        if (state == this->save_state_)
            return;
        this->on_save_change(state);
        this->save_save_preference();
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

    // Publish to sensors if they exist
    if (this->atc_room_temp_sensor_ != nullptr) {
        this->atc_room_temp_sensor_->publish_state(temperature);
    }

    if (this->atc_room_humidity_sensor_ != nullptr) {
        this->atc_room_humidity_sensor_->publish_state(humidity);
    }

    // Update current temperature if using ATC sensor
    if (is_using_atc_sensor()) {
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
        return false;
    }

    std::string configured_mac = this->atc_mac_address_text_->state;
    
    // Check if advertiser address matches
    std::string device_mac = device.address_str();
    bool mac_matches = macs_equal_(device_mac, configured_mac);
    
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
            continue;
        }
        
        // Check embedded MAC if present (first 6 bytes, reversed order)
        if (data.size() >= 6) {
            char embedded_mac[18];
            snprintf(embedded_mac, sizeof(embedded_mac), "%02X:%02X:%02X:%02X:%02X:%02X",
                     data[5], data[4], data[3], data[2], data[1], data[0]);
            
            if (macs_equal_(embedded_mac, configured_mac)) {
                mac_matches = true;
            }
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
        
        // Update sensors
        update_atc_sensor(temperature, humidity);
        update_atc_battery((float)battery);
        
        return true;
    }
    
    return false;
}
#endif

/*
 * Preference management for persistent storage
 */

void SinclairAC::load_preferences()
{
    ESP_LOGD(TAG, "Loading preferences from flash...");

    // Load temperature source
    uint8_t temp_source_val = 0;
    if (this->pref_temp_source_.load(&temp_source_val)) {
        if (temp_source_val == 0) {
            this->temp_source_state_ = temp_source_options::AC_OWN;
        } else {
            this->temp_source_state_ = temp_source_options::EXTERNAL_ATC;
        }
        ESP_LOGD(TAG, "Restored temp source: %s", this->temp_source_state_.c_str());
        if (this->temp_source_select_ != nullptr) {
            this->temp_source_select_->publish_state(this->temp_source_state_);
        }
    }

    // Load ATC MAC address
    std::string atc_mac_val;
    if (this->pref_atc_mac_.load(&atc_mac_val)) {
        ESP_LOGD(TAG, "Restored ATC MAC: %s", atc_mac_val.c_str());
        if (this->atc_mac_address_text_ != nullptr) {
            this->atc_mac_address_text_->state = atc_mac_val;
            this->atc_mac_address_text_->publish_state(atc_mac_val);
        }
    }

    // Load display mode
    uint8_t display_val = 0;
    if (this->pref_display_.load(&display_val)) {
        const char* display_options[] = {
            display_options::OFF.c_str(),
            display_options::AUTO.c_str(),
            display_options::SET.c_str(),
            display_options::ACT.c_str(),
            display_options::OUT.c_str()
        };
        if (display_val < 5) {
            this->display_state_ = display_options[display_val];
            ESP_LOGD(TAG, "Restored display: %s", this->display_state_.c_str());
            if (this->display_select_ != nullptr) {
                this->display_select_->publish_state(this->display_state_);
            }
        }
    }

    // Load display unit
    uint8_t display_unit_val = 0;
    if (this->pref_display_unit_.load(&display_unit_val)) {
        if (display_unit_val == 0) {
            this->display_unit_state_ = display_unit_options::DEGC;
        } else {
            this->display_unit_state_ = display_unit_options::DEGF;
        }
        ESP_LOGD(TAG, "Restored display unit: %s", this->display_unit_state_.c_str());
        if (this->display_unit_select_ != nullptr) {
            this->display_unit_select_->publish_state(this->display_unit_state_);
        }
    }

    // Load vertical swing
    uint8_t vswing_val = 0;
    if (this->pref_vertical_swing_.load(&vswing_val)) {
        const char* vswing_options[] = {
            vertical_swing_options::OFF.c_str(),
            vertical_swing_options::FULL.c_str(),
            vertical_swing_options::DOWN.c_str(),
            vertical_swing_options::MIDD.c_str(),
            vertical_swing_options::MID.c_str(),
            vertical_swing_options::MIDU.c_str(),
            vertical_swing_options::UP.c_str(),
            vertical_swing_options::CDOWN.c_str(),
            vertical_swing_options::CMIDD.c_str(),
            vertical_swing_options::CMID.c_str(),
            vertical_swing_options::CMIDU.c_str(),
            vertical_swing_options::CUP.c_str()
        };
        if (vswing_val < 12) {
            this->vertical_swing_state_ = vswing_options[vswing_val];
            ESP_LOGD(TAG, "Restored vertical swing: %s", this->vertical_swing_state_.c_str());
            if (this->vertical_swing_select_ != nullptr) {
                this->vertical_swing_select_->publish_state(this->vertical_swing_state_);
            }
        }
    }

    // Load horizontal swing
    uint8_t hswing_val = 0;
    if (this->pref_horizontal_swing_.load(&hswing_val)) {
        const char* hswing_options[] = {
            horizontal_swing_options::OFF.c_str(),
            horizontal_swing_options::FULL.c_str(),
            horizontal_swing_options::CLEFT.c_str(),
            horizontal_swing_options::CMIDL.c_str(),
            horizontal_swing_options::CMID.c_str(),
            horizontal_swing_options::CMIDR.c_str(),
            horizontal_swing_options::CRIGHT.c_str()
        };
        if (hswing_val < 7) {
            this->horizontal_swing_state_ = hswing_options[hswing_val];
            ESP_LOGD(TAG, "Restored horizontal swing: %s", this->horizontal_swing_state_.c_str());
            if (this->horizontal_swing_select_ != nullptr) {
                this->horizontal_swing_select_->publish_state(this->horizontal_swing_state_);
            }
        }
    }

    // Load switch states
    bool plasma_val = false;
    if (this->pref_plasma_.load(&plasma_val)) {
        this->plasma_state_ = plasma_val;
        ESP_LOGD(TAG, "Restored plasma: %s", plasma_val ? "ON" : "OFF");
        if (this->plasma_switch_ != nullptr) {
            this->plasma_switch_->publish_state(plasma_val);
        }
    }

    bool beeper_val = false;
    if (this->pref_beeper_.load(&beeper_val)) {
        this->beeper_state_ = beeper_val;
        ESP_LOGD(TAG, "Restored beeper: %s", beeper_val ? "ON" : "OFF");
        if (this->beeper_switch_ != nullptr) {
            this->beeper_switch_->publish_state(beeper_val);
        }
    }

    bool sleep_val = false;
    if (this->pref_sleep_.load(&sleep_val)) {
        this->sleep_state_ = sleep_val;
        ESP_LOGD(TAG, "Restored sleep: %s", sleep_val ? "ON" : "OFF");
        if (this->sleep_switch_ != nullptr) {
            this->sleep_switch_->publish_state(sleep_val);
        }
    }

    bool xfan_val = false;
    if (this->pref_xfan_.load(&xfan_val)) {
        this->xfan_state_ = xfan_val;
        ESP_LOGD(TAG, "Restored xfan: %s", xfan_val ? "ON" : "OFF");
        if (this->xfan_switch_ != nullptr) {
            this->xfan_switch_->publish_state(xfan_val);
        }
    }

    bool save_val = false;
    if (this->pref_save_.load(&save_val)) {
        this->save_state_ = save_val;
        ESP_LOGD(TAG, "Restored save: %s", save_val ? "ON" : "OFF");
        if (this->save_switch_ != nullptr) {
            this->save_switch_->publish_state(save_val);
        }
    }

    ESP_LOGI(TAG, "Preferences loaded successfully");
}

void SinclairAC::save_temp_source_preference()
{
    uint8_t val = (this->temp_source_state_ == temp_source_options::EXTERNAL_ATC) ? 1 : 0;
    this->pref_temp_source_.save(&val);
    ESP_LOGD(TAG, "Saved temp source preference: %d", val);
}

void SinclairAC::save_atc_mac_preference()
{
    if (this->atc_mac_address_text_ != nullptr) {
        std::string mac = this->atc_mac_address_text_->state;
        this->pref_atc_mac_.save(&mac);
        ESP_LOGD(TAG, "Saved ATC MAC preference: %s", mac.c_str());
    }
}

void SinclairAC::save_display_preference()
{
    uint8_t val = 0;
    if (this->display_state_ == display_options::OFF) val = 0;
    else if (this->display_state_ == display_options::AUTO) val = 1;
    else if (this->display_state_ == display_options::SET) val = 2;
    else if (this->display_state_ == display_options::ACT) val = 3;
    else if (this->display_state_ == display_options::OUT) val = 4;
    
    this->pref_display_.save(&val);
    ESP_LOGD(TAG, "Saved display preference: %d", val);
}

void SinclairAC::save_display_unit_preference()
{
    uint8_t val = (this->display_unit_state_ == display_unit_options::DEGF) ? 1 : 0;
    this->pref_display_unit_.save(&val);
    ESP_LOGD(TAG, "Saved display unit preference: %d", val);
}

void SinclairAC::save_vertical_swing_preference()
{
    uint8_t val = 0;
    if (this->vertical_swing_state_ == vertical_swing_options::OFF) val = 0;
    else if (this->vertical_swing_state_ == vertical_swing_options::FULL) val = 1;
    else if (this->vertical_swing_state_ == vertical_swing_options::DOWN) val = 2;
    else if (this->vertical_swing_state_ == vertical_swing_options::MIDD) val = 3;
    else if (this->vertical_swing_state_ == vertical_swing_options::MID) val = 4;
    else if (this->vertical_swing_state_ == vertical_swing_options::MIDU) val = 5;
    else if (this->vertical_swing_state_ == vertical_swing_options::UP) val = 6;
    else if (this->vertical_swing_state_ == vertical_swing_options::CDOWN) val = 7;
    else if (this->vertical_swing_state_ == vertical_swing_options::CMIDD) val = 8;
    else if (this->vertical_swing_state_ == vertical_swing_options::CMID) val = 9;
    else if (this->vertical_swing_state_ == vertical_swing_options::CMIDU) val = 10;
    else if (this->vertical_swing_state_ == vertical_swing_options::CUP) val = 11;
    
    this->pref_vertical_swing_.save(&val);
    ESP_LOGD(TAG, "Saved vertical swing preference: %d", val);
}

void SinclairAC::save_horizontal_swing_preference()
{
    uint8_t val = 0;
    if (this->horizontal_swing_state_ == horizontal_swing_options::OFF) val = 0;
    else if (this->horizontal_swing_state_ == horizontal_swing_options::FULL) val = 1;
    else if (this->horizontal_swing_state_ == horizontal_swing_options::CLEFT) val = 2;
    else if (this->horizontal_swing_state_ == horizontal_swing_options::CMIDL) val = 3;
    else if (this->horizontal_swing_state_ == horizontal_swing_options::CMID) val = 4;
    else if (this->horizontal_swing_state_ == horizontal_swing_options::CMIDR) val = 5;
    else if (this->horizontal_swing_state_ == horizontal_swing_options::CRIGHT) val = 6;
    
    this->pref_horizontal_swing_.save(&val);
    ESP_LOGD(TAG, "Saved horizontal swing preference: %d", val);
}

void SinclairAC::save_plasma_preference()
{
    this->pref_plasma_.save(&this->plasma_state_);
    ESP_LOGD(TAG, "Saved plasma preference: %s", this->plasma_state_ ? "ON" : "OFF");
}

void SinclairAC::save_beeper_preference()
{
    this->pref_beeper_.save(&this->beeper_state_);
    ESP_LOGD(TAG, "Saved beeper preference: %s", this->beeper_state_ ? "ON" : "OFF");
}

void SinclairAC::save_sleep_preference()
{
    this->pref_sleep_.save(&this->sleep_state_);
    ESP_LOGD(TAG, "Saved sleep preference: %s", this->sleep_state_ ? "ON" : "OFF");
}

void SinclairAC::save_xfan_preference()
{
    this->pref_xfan_.save(&this->xfan_state_);
    ESP_LOGD(TAG, "Saved xfan preference: %s", this->xfan_state_ ? "ON" : "OFF");
}

void SinclairAC::save_save_preference()
{
    this->pref_save_.save(&this->save_state_);
    ESP_LOGD(TAG, "Saved save preference: %s", this->save_state_ ? "ON" : "OFF");
}

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
