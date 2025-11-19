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

    ESP_LOGI(TAG, "Sinclair AC component v%s starting...", VERSION);

    // Initialize preference objects
    this->pref_display_ = global_preferences->make_preference<std::string>(fnv1_hash("sinclair_ac/display"));
    this->pref_display_unit_ = global_preferences->make_preference<std::string>(fnv1_hash("sinclair_ac/display_unit"));
    this->pref_vertical_swing_ = global_preferences->make_preference<std::string>(fnv1_hash("sinclair_ac/vswing"));
    this->pref_horizontal_swing_ = global_preferences->make_preference<std::string>(fnv1_hash("sinclair_ac/hswing"));
    this->pref_temp_source_ = global_preferences->make_preference<std::string>(fnv1_hash("sinclair_ac/temp_source"));
    this->pref_plasma_ = global_preferences->make_preference<bool>(fnv1_hash("sinclair_ac/plasma"));
    this->pref_beeper_ = global_preferences->make_preference<bool>(fnv1_hash("sinclair_ac/beeper"));
    this->pref_sleep_ = global_preferences->make_preference<bool>(fnv1_hash("sinclair_ac/sleep"));
    this->pref_xfan_ = global_preferences->make_preference<bool>(fnv1_hash("sinclair_ac/xfan"));
    this->pref_save_ = global_preferences->make_preference<bool>(fnv1_hash("sinclair_ac/save"));
    this->pref_atc_mac_ = global_preferences->make_preference<std::string>(fnv1_hash("sinclair_ac/atc_mac"));

    // Load persisted preferences
    load_preferences_();

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
    
    // Save preference
    this->pref_horizontal_swing_.save(&this->horizontal_swing_state_);
}

void SinclairAC::update_swing_vertical(const std::string &swing)
{
    this->vertical_swing_state_ = swing;

    if (this->vertical_swing_select_ != nullptr && 
        this->vertical_swing_select_->state != this->vertical_swing_state_)
    {
        this->vertical_swing_select_->publish_state(this->vertical_swing_state_);
    }
    
    // Save preference
    this->pref_vertical_swing_.save(&this->vertical_swing_state_);
}

void SinclairAC::update_display(const std::string &display)
{
    this->display_state_ = display;

    if (this->display_select_ != nullptr && 
        this->display_select_->state != this->display_state_)
    {
        this->display_select_->publish_state(this->display_state_);
    }
    
    // Save preference
    this->pref_display_.save(&this->display_state_);
}

void SinclairAC::update_display_unit(const std::string &display_unit)
{
    this->display_unit_state_ = display_unit;

    if (this->display_unit_select_ != nullptr && 
        this->display_unit_select_->state != this->display_unit_state_)
    {
        this->display_unit_select_->publish_state(this->display_unit_state_);
    }
    
    // Save preference
    this->pref_display_unit_.save(&this->display_unit_state_);
}

void SinclairAC::update_temp_source(const std::string &temp_source)
{
    this->temp_source_state_ = temp_source;

    if (this->temp_source_select_ != nullptr && 
        this->temp_source_select_->state != this->temp_source_state_)
    {
        this->temp_source_select_->publish_state(this->temp_source_state_);
    }
    
    // Save preference
    this->pref_temp_source_.save(&this->temp_source_state_);
}

void SinclairAC::update_plasma(bool plasma)
{
    this->plasma_state_ = plasma;

    if (this->plasma_switch_ != nullptr)
    {
        this->plasma_switch_->publish_state(this->plasma_state_);
    }
    
    // Save preference
    this->pref_plasma_.save(&this->plasma_state_);
}

void SinclairAC::update_beeper(bool beeper)
{
    this->beeper_state_ = beeper;

    if (this->beeper_switch_ != nullptr)
    {
        this->beeper_switch_->publish_state(this->beeper_state_);
    }
    
    // Save preference
    this->pref_beeper_.save(&this->beeper_state_);
}

void SinclairAC::update_sleep(bool sleep)
{
    this->sleep_state_ = sleep;

    if (this->sleep_switch_ != nullptr)
    {
        this->sleep_switch_->publish_state(this->sleep_state_);
    }
    
    // Save preference
    this->pref_sleep_.save(&this->sleep_state_);
}

void SinclairAC::update_xfan(bool xfan)
{
    this->xfan_state_ = xfan;

    if (this->xfan_switch_ != nullptr)
    {
        this->xfan_switch_->publish_state(this->xfan_state_);
    }
    
    // Save preference
    this->pref_xfan_.save(&this->xfan_state_);
}

void SinclairAC::update_save(bool save)
{
    this->save_state_ = save;

    if (this->save_switch_ != nullptr)
    {
        this->save_switch_->publish_state(this->save_state_);
    }
    
    // Save preference
    this->pref_save_.save(&this->save_state_);
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
        this->on_temp_source_change(value);
    });
}

void SinclairAC::set_atc_mac_address_text(text::Text *atc_mac_address_text)
{
    this->atc_mac_address_text_ = atc_mac_address_text;
    
    // Add callback to save MAC when it changes
    this->atc_mac_address_text_->add_on_state_callback([this](const std::string &value) {
        if (validate_mac_format_(value)) {
            this->pref_atc_mac_.save(&value);
            ESP_LOGD(TAG, "ATC MAC address saved: %s", value.c_str());
        } else if (!value.empty()) {
            ESP_LOGW(TAG, "Invalid MAC address format: %s (expected AA:BB:CC:DD:EE:FF)", value.c_str());
        }
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

/*
 * MAC address validation
 */

bool SinclairAC::validate_mac_format_(const std::string &mac)
{
    if (mac.size() != 17) return false;
    for (int i = 0; i < 17; i++) {
        if ((i + 1) % 3 == 0) {
            if (mac[i] != ':') return false;
        } else {
            char c = mac[i];
            if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'))) return false;
        }
    }
    return true;
}

/*
 * Preference loading
 */

void SinclairAC::load_preferences_()
{
    std::string loaded_display;
    std::string loaded_display_unit;
    std::string loaded_vswing;
    std::string loaded_hswing;
    std::string loaded_temp_source;
    std::string loaded_atc_mac;
    bool loaded_plasma = false;
    bool loaded_beeper = false;
    bool loaded_sleep = false;
    bool loaded_xfan = false;
    bool loaded_save = false;
    
    // Load string preferences
    if (this->pref_display_.load(&loaded_display)) {
        if (this->display_select_ != nullptr && loaded_display != this->display_state_) {
            this->display_state_ = loaded_display;
            this->display_select_->publish_state(loaded_display);
        }
    }
    
    if (this->pref_display_unit_.load(&loaded_display_unit)) {
        if (this->display_unit_select_ != nullptr && loaded_display_unit != this->display_unit_state_) {
            this->display_unit_state_ = loaded_display_unit;
            this->display_unit_select_->publish_state(loaded_display_unit);
        }
    }
    
    if (this->pref_vertical_swing_.load(&loaded_vswing)) {
        if (this->vertical_swing_select_ != nullptr && loaded_vswing != this->vertical_swing_state_) {
            this->vertical_swing_state_ = loaded_vswing;
            this->vertical_swing_select_->publish_state(loaded_vswing);
        }
    }
    
    if (this->pref_horizontal_swing_.load(&loaded_hswing)) {
        if (this->horizontal_swing_select_ != nullptr && loaded_hswing != this->horizontal_swing_state_) {
            this->horizontal_swing_state_ = loaded_hswing;
            this->horizontal_swing_select_->publish_state(loaded_hswing);
        }
    }
    
    // Load ATC MAC address
    if (this->pref_atc_mac_.load(&loaded_atc_mac)) {
        if (this->atc_mac_address_text_ != nullptr && !loaded_atc_mac.empty()) {
            if (validate_mac_format_(loaded_atc_mac)) {
                this->atc_mac_address_text_->publish_state(loaded_atc_mac);
            } else {
                ESP_LOGW(TAG, "Persisted ATC MAC has invalid format: %s", loaded_atc_mac.c_str());
            }
        }
    }
    
    // Load temperature source
    if (this->pref_temp_source_.load(&loaded_temp_source)) {
        // If temp source is External ATC but MAC is invalid/empty, fallback to AC Own
        if (loaded_temp_source == temp_source_options::EXTERNAL_ATC) {
            if (this->atc_mac_address_text_ == nullptr || 
                this->atc_mac_address_text_->state.empty() || 
                !validate_mac_format_(this->atc_mac_address_text_->state)) {
                ESP_LOGW(TAG, "Fallback to AC Own Sensor due to invalid or missing ATC MAC");
                loaded_temp_source = temp_source_options::AC_OWN;
            }
        }
        
        if (this->temp_source_select_ != nullptr && loaded_temp_source != this->temp_source_state_) {
            this->temp_source_state_ = loaded_temp_source;
            this->temp_source_select_->publish_state(loaded_temp_source);
        }
    }
    
    // Load boolean preferences
    if (this->pref_plasma_.load(&loaded_plasma)) {
        if (this->plasma_switch_ != nullptr && loaded_plasma != this->plasma_state_) {
            this->plasma_state_ = loaded_plasma;
            this->plasma_switch_->publish_state(loaded_plasma);
        }
    }
    
    if (this->pref_beeper_.load(&loaded_beeper)) {
        if (this->beeper_switch_ != nullptr && loaded_beeper != this->beeper_state_) {
            this->beeper_state_ = loaded_beeper;
            this->beeper_switch_->publish_state(loaded_beeper);
        }
    }
    
    if (this->pref_sleep_.load(&loaded_sleep)) {
        if (this->sleep_switch_ != nullptr && loaded_sleep != this->sleep_state_) {
            this->sleep_state_ = loaded_sleep;
            this->sleep_switch_->publish_state(loaded_sleep);
        }
    }
    
    if (this->pref_xfan_.load(&loaded_xfan)) {
        if (this->xfan_switch_ != nullptr && loaded_xfan != this->xfan_state_) {
            this->xfan_state_ = loaded_xfan;
            this->xfan_switch_->publish_state(loaded_xfan);
        }
    }
    
    if (this->pref_save_.load(&loaded_save)) {
        if (this->save_switch_ != nullptr && loaded_save != this->save_state_) {
            this->save_state_ = loaded_save;
            this->save_switch_->publish_state(loaded_save);
        }
    }
    
    ESP_LOGD(TAG, "Loaded persisted display=%s unit=%s hswing=%s vswing=%s temp_source=%s",
             this->display_state_.c_str(), this->display_unit_state_.c_str(),
             this->horizontal_swing_state_.c_str(), this->vertical_swing_state_.c_str(),
             this->temp_source_state_.c_str());
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
