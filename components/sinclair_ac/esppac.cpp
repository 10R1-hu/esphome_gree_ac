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
