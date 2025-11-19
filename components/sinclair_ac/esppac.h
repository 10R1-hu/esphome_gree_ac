// based on: https://github.com/DomiStyle/esphome-panasonic-ac
#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/select/select.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text/text.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

#ifdef USE_ESP32_BLE_TRACKER
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#endif

namespace esphome {

namespace sinclair_ac {

static const char *const VERSION = "0.0.3";

static const uint8_t READ_TIMEOUT = 20;  // The maximum time to wait before considering a packet complete

static const uint8_t MIN_TEMPERATURE = 16;   // Minimum temperature as reported by EWPE SMART APP
static const uint8_t MAX_TEMPERATURE = 30;   // Maximum temperature as supported by EWPE SMART APP
static const float TEMPERATURE_STEP = 0.5;   // Steps the temperature can be set in
static const float TEMPERATURE_TOLERANCE = 1;  // The tolerance to allow when checking the climate state
static const uint8_t TEMPERATURE_THRESHOLD = 100;  // Maximum temperature the AC can report (formally 119.5 for sinclair protocol, but 100 is impossible, soo...)

static const uint32_t ATC_SENSOR_TIMEOUT_MS = 900000;  // 15 minutes in milliseconds

namespace fan_modes{
    const std::string FAN_AUTO  = "0 - Auto";
    const std::string FAN_QUIET = "1 - Quiet";
    const std::string FAN_LOW   = "2 - Low";
    const std::string FAN_MEDL  = "3 - Medium-Low";
    const std::string FAN_MED   = "4 - Medium";
    const std::string FAN_MEDH  = "5 - Medium-High";
    const std::string FAN_HIGH  = "6 - High";
    const std::string FAN_TURBO = "7 - Turbo";
}

/* this must be same as HORIZONTAL_SWING_OPTIONS in climate.py */
namespace horizontal_swing_options{
    const std::string OFF    = "0 - OFF";
    const std::string FULL   = "1 - Swing - Full";
    const std::string CLEFT  = "2 - Constant - Left";
    const std::string CMIDL  = "3 - Constant - Mid-Left";
    const std::string CMID   = "4 - Constant - Middle";
    const std::string CMIDR  = "5 - Constant - Mid-Right";
    const std::string CRIGHT = "6 - Constant - Right";
}

/* this must be same as VERTICAL_SWING_OPTIONS in climate.py */
namespace vertical_swing_options{
    const std::string OFF   = "00 - OFF";
    const std::string FULL  = "01 - Swing - Full";
    const std::string DOWN  = "02 - Swing - Down";
    const std::string MIDD  = "03 - Swing - Mid-Down";
    const std::string MID   = "04 - Swing - Middle";
    const std::string MIDU  = "05 - Swing - Mid-Up";
    const std::string UP    = "06 - Swing - Up";
    const std::string CDOWN = "07 - Constant - Down";
    const std::string CMIDD = "08 - Constant - Mid-Down";
    const std::string CMID  = "09 - Constant - Middle";
    const std::string CMIDU = "10 - Constant - Mid-Up";
    const std::string CUP   = "11 - Constant - Up";
}

/* this must be same as DISPLAY_OPTIONS in climate.py */
namespace display_options{
    const std::string OFF  = "0 - OFF";
    const std::string AUTO = "1 - Auto";
    const std::string SET  = "2 - Set temperature";
    const std::string ACT  = "3 - Actual temperature";
    const std::string OUT  = "4 - Outside temperature";
}

/* this must be same as DISPLAY_UNIT_OPTIONS in climate.py */
namespace display_unit_options{
    const std::string DEGC = "C";
    const std::string DEGF = "F";
}

/* this must be same as TEMP_SOURCE_OPTIONS in climate.py */
namespace temp_source_options{
    const std::string AC_OWN = "AC Own Sensor";
    const std::string EXTERNAL_ATC = "External ATC Sensor";
}

typedef enum {
        STATE_WAIT_SYNC,
        STATE_RECIEVE,
        STATE_COMPLETE,
        STATE_RESTART
} SerialProcessState_t;

static const uint8_t DATA_MAX = 200;

typedef struct {
        std::vector<uint8_t> data;
        uint8_t data_cnt;
        uint8_t frame_size;
        SerialProcessState_t state;
} SerialProcess_t;

class SinclairAC : public Component, public uart::UARTDevice, public climate::Climate
#ifdef USE_ESP32_BLE_TRACKER
    , public esp32_ble_tracker::ESPBTDeviceListener
#endif
{
    public:
        void set_vertical_swing_select(select::Select *vertical_swing_select);
        void set_horizontal_swing_select(select::Select *horizontal_swing_select);

        void set_display_select(select::Select *display_select);
        void set_display_unit_select(select::Select *display_unit_select);
        void set_temp_source_select(select::Select *temp_source_select);

        void set_plasma_switch(switch_::Switch *plasma_switch);
        void set_beeper_switch(switch_::Switch *beeper_switch);
        void set_sleep_switch(switch_::Switch *sleep_switch);
        void set_xfan_switch(switch_::Switch *plasma_switch);
        void set_save_switch(switch_::Switch *plasma_switch);

        void set_current_temperature_sensor(sensor::Sensor *current_temperature_sensor);
        void set_atc_mac_address_text(text::Text *atc_mac_address_text);
        void set_ac_indoor_temp_sensor(sensor::Sensor *ac_indoor_temp_sensor);
        void set_atc_room_temp_sensor(sensor::Sensor *atc_room_temp_sensor);
        void set_atc_room_humidity_sensor(sensor::Sensor *atc_room_humidity_sensor);
        void set_atc_battery_sensor(sensor::Sensor *atc_battery_sensor);

        void setup() override;
        void loop() override;

    protected:
        select::Select *vertical_swing_select_   = nullptr; /* Advanced vertical swing select */
        select::Select *horizontal_swing_select_ = nullptr; /* Advanced horizontal swing select */

        select::Select *display_select_          = nullptr; /* Select for setting display mode */
        select::Select *display_unit_select_     = nullptr; /* Select for setting display temperature unit */
        select::Select *temp_source_select_      = nullptr; /* Select for temperature source (AC own or external ATC) */

        switch_::Switch *plasma_switch_          = nullptr; /* Switch for plasma */
        switch_::Switch *beeper_switch_          = nullptr; /* Switch for beeper */
        switch_::Switch *sleep_switch_           = nullptr; /* Switch for sleep */
        switch_::Switch *xfan_switch_            = nullptr; /* Switch for X-fan */
        switch_::Switch *save_switch_            = nullptr; /* Switch for save */

        sensor::Sensor *current_temperature_sensor_ = nullptr; /* If user wants to replace reported temperature by an external sensor readout */
        text::Text *atc_mac_address_text_        = nullptr; /* Text input for ATC MAC address */
        sensor::Sensor *ac_indoor_temp_sensor_   = nullptr; /* AC indoor temperature sensor for HA display */
        sensor::Sensor *atc_room_temp_sensor_    = nullptr; /* ATC room temperature sensor */
        sensor::Sensor *atc_room_humidity_sensor_ = nullptr; /* ATC room humidity sensor */
        sensor::Sensor *atc_battery_sensor_      = nullptr; /* ATC battery sensor */

        std::string vertical_swing_state_;
        std::string horizontal_swing_state_;

        std::string display_state_;
        std::string display_unit_state_;
        std::string temp_source_state_;

        bool plasma_state_;
        bool beeper_state_;
        bool sleep_state_;
        bool xfan_state_;
        bool save_state_;

        uint32_t last_atc_sensor_update_ = 0;  /* Timestamp of last ATC sensor update */
        bool atc_sensor_valid_ = false;         /* Flag indicating if ATC sensor data is valid */
        float last_atc_temperature_ = 0;        /* Last received ATC temperature */
        float last_atc_humidity_ = 0;           /* Last received ATC humidity */
        float last_atc_battery_ = 0;            /* Last received ATC battery percentage */

        SerialProcess_t serialProcess_;

        float Temrec0 [16];
        float Temrec1 [16];

        uint32_t init_time_;   // Stores the current time
        // uint32_t last_read_;   // Stores the time at which the last read was done
        uint32_t last_packet_sent_;  // Stores the time at which the last packet was sent
        uint32_t last_03packet_sent_;  // Stores the time at which the last packet was sent
        uint32_t last_packet_received_;  // Stores the time at which the last packet was received
        bool wait_response_;

        climate::ClimateTraits traits() override;

        void read_data();

        void update_current_temperature(float temperature);
        void update_target_temperature(float temperature);

        void update_swing_horizontal(const std::string &swing);
        void update_swing_vertical(const std::string &swing);

        void update_display(const std::string &display);
        void update_display_unit(const std::string &display_unit);
        void update_temp_source(const std::string &temp_source);

        void update_plasma(bool plasma);
        void update_beeper(bool beeper);
        void update_sleep(bool sleep);
        void update_xfan(bool xfan);
        void update_save(bool save);

        void check_atc_sensor_timeout();
        void update_atc_sensor(float temperature, float humidity);
        void update_atc_battery(float battery_percent);
        bool is_using_atc_sensor();

#ifdef USE_ESP32_BLE_TRACKER
        bool parse_device(const esp32_ble_tracker::ESPBTDevice &device) override;
        std::string normalize_mac_(const std::string &mac);
        bool macs_equal_(const std::string &mac1, const std::string &mac2);
#endif

        virtual void on_horizontal_swing_change(const std::string &swing) = 0;
        virtual void on_vertical_swing_change(const std::string &swing) = 0;

        virtual void on_display_change(const std::string &display) = 0;
        virtual void on_display_unit_change(const std::string &display_unit) = 0;
        virtual void on_temp_source_change(const std::string &temp_source) = 0;

        virtual void on_plasma_change(bool plasma) = 0;
        virtual void on_beeper_change(bool beeper) = 0;
        virtual void on_sleep_change(bool sleep) = 0;
        virtual void on_xfan_change(bool xfan) = 0;
        virtual void on_save_change(bool save) = 0;

        climate::ClimateAction determine_action();

        void log_packet(std::vector<uint8_t> data, bool outgoing = false);
};

}  // namespace sinclair_ac
}  // namespace esphome
