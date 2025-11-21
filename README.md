# Open source WIFI module replacement for Gree protocol based AC's for Home Assistant.

**Version: v0.0.5**

This repository adds support for ESP-based WiFi modules to interface with Gree/Sinclair AC units.
It's forked from https://github.com/piotrva/esphome_gree_ac, big thanks to @piotrva for his work!

My fork currently differs from the original code in the following ways. What I did:

1) Fixed the fan mode, tested on Gree/Daizuki/TGM AC's.
2) Fixed the dropping of commands
3) Fixed the rejection of commands
4) Fixed reporting of current temp
5) Fixed the Fahrenheit mode
6) Implemented an optional silent mode (no beeping), only works for module sent commands (not for
   remote control sent commands)
7) **Added 8 fan speed levels** (Auto, Quiet, Low, Medium-Low, Medium, Medium-High, High, Turbo) for granular control
   
It's now compatible with GRJWB04-J / Cs532ae wifi modules

# Current state:
No known problems! if you run into an issue though, please let me know.

## Features:
- **8 Fan Speed Levels**: Auto, Quiet, Low, Medium-Low, Medium, Medium-High, High, and Turbo modes for precise control
- Temperature control (16°C - 30°C)
- Multiple operating modes: Auto, Cool, Heat, Dry, Fan Only
- Vertical and horizontal swing control
- Plasma, X-fan, Sleep, and Energy Saving modes
- Optional silent operation (no beeping)
- Display control and temperature unit selection (°C/°F)
- **External Temperature Sensor Support**: Use any external temperature sensor (like pvvx_mithermometer, BLE sensors, or other ESPHome sensors) for more accurate climate control
- **Smart Temperature Source Selection**: Three-state mode with "AC Own Sensor", "External ATC Sensor", and "ATC Fail"
- **Automatic Timeout & Recovery**: Automatically switches to "ATC Fail" mode after 15 minutes without external sensor data, and automatically recovers when data resumes
- **Persistent User Settings**: All user preferences (display mode, swing positions, temperature source, switches) are automatically saved and restored across reboots without requiring YAML `restore_value` or `restore_mode` configuration

See [FAN_LEVELS.md](FAN_LEVELS.md) for detailed information about fan speed levels.

## External Temperature Sensor Support

The component supports using external temperature sensors as an alternative to the AC's built-in temperature sensor for more accurate climate control. Simply configure any ESPHome sensor (e.g., `pvvx_mithermometer`, `dht22`, or other sensors) and link it via `current_temperature_sensor`.

### Temperature Source Modes:
1. **AC Own Sensor** - Uses the AC unit's internal temperature sensor (default)
2. **External ATC Sensor** - Uses the external sensor configured via `current_temperature_sensor`
3. **ATC Fail** - Automatic fallback state when external sensor times out (15 minutes without data)

### Key Features:
- **Simplified Configuration**: No MAC address management or BLE parsing - just link any ESPHome sensor in YAML
- **Automatic Timeout**: If the external sensor doesn't send data for 15 minutes, automatically switches to "ATC Fail" mode and uses AC's sensor
- **Automatic Recovery**: When external sensor data resumes, automatically recovers from "ATC Fail" back to "External ATC Sensor"
- **Manual Control**: Use the Temperature Source select entity to manually switch between modes
- **State Persistence**: Current mode (including Fail state) persists across reboots

### Configuration:
Add an external sensor to your YAML configuration. Here's an example using a BLE temperature sensor with pvvx_mithermometer:

```yaml
# Enable BLE tracker (ESP32 only - for BLE sensors like pvvx_mithermometer)
esp32_ble_tracker:
  scan_parameters:
    active: false

# Configure your external temperature sensor
# This example uses pvvx_mithermometer, but any ESPHome sensor works
sensor:
  - platform: pvvx_mithermometer
    mac_address: "A4:C1:38:0D:10:15"
    temperature:
      name: "External Room Temperature"
      id: external_room_temp

climate:
  - platform: sinclair_ac
    name: "Living Room AC"
    # Link the external sensor to the climate component
    current_temperature_sensor: external_room_temp
    
    # Temperature source selector
    temp_source_select:
      name: "AC Temperature Source"
    
    # AC's indoor temperature sensor (optional, for monitoring)
    ac_indoor_temp_sensor:
      name: "AC Indoor Temperature"
      unit_of_measurement: "°C"
      accuracy_decimals: 1
      device_class: temperature
      state_class: measurement
```

### Alternative Sensors:
You can use **any** ESPHome sensor, not just BLE sensors:

```yaml
# DHT22 temperature sensor example
sensor:
  - platform: dht
    pin: GPIO4
    temperature:
      name: "Room Temperature"
      id: room_temp
    humidity:
      name: "Room Humidity"
    update_interval: 60s

climate:
  - platform: sinclair_ac
    name: "Bedroom AC"
    current_temperature_sensor: room_temp
    temp_source_select:
      name: "Temperature Source"
```

### Usage:
1. Configure any ESPHome temperature sensor in your YAML (examples above)
2. Link it to the climate component using `current_temperature_sensor`
3. In Home Assistant, use the "AC Temperature Source" select entity to choose:
   - **AC Own Sensor** - Use the AC unit's built-in sensor
   - **External ATC Sensor** - Use your configured external sensor
   - **ATC Fail** - Automatic fallback when external sensor times out (or can be manually selected)
4. The system will automatically:
   - Switch to "ATC Fail" if no external data for 15 minutes (when using External ATC Sensor mode)
   - Automatically recover to "External ATC Sensor" when data resumes
   - Persist your selection across reboots

### How It Works:
- **When "AC Own Sensor" is selected**: Climate entity uses the AC unit's internal temperature sensor
- **When "External ATC Sensor" is selected**: Climate entity uses your configured external sensor for control
  - If external sensor stops sending data for 15 minutes → automatically switches to "ATC Fail" mode
  - When external data resumes → automatically recovers back to "External ATC Sensor"
- **When "ATC Fail" is selected**: Uses AC's internal sensor (manual override or automatic timeout state)

### Notes:
- To change external sensors, simply edit your YAML configuration - no runtime MAC changes needed
- The temperature source mode and fail state persist across reboots
- ESP8266 users: Works fine! Just use non-BLE sensors (DHT, Dallas, etc.)
- ESP32 users: Can use BLE sensors (pvvx_mithermometer, etc.) or any other sensor type

## Persistent Settings (v0.0.5+)

From version 0.0.5 onwards, all user preferences are automatically persisted across reboots:

### Persisted Settings:
- Display mode (OFF, Auto, Set, Actual, Outside temperature)
- Display unit (°C / °F)
- Vertical swing position (12 positions)
- Horizontal swing position (7 positions)
- Temperature source selection (AC Own Sensor / External ATC Sensor / ATC Fail)
- All switch states (Plasma, Beeper, Sleep, X-fan, Save)

### Features:
- **No YAML configuration needed**: You do NOT need to add `restore_value: true` or `restore_mode` to your entity configurations
- **Automatic saving**: Settings are saved immediately when changed via Home Assistant
- **Smart validation**: Invalid settings are detected and corrected on load with fallback to safe defaults
- **Cross-reboot persistence**: All settings survive ESP reboots, power cycles, and firmware updates
- **Fail state persistence**: If system is in "ATC Fail" mode during reboot, it restores to that state

This means your AC will maintain its configuration exactly as you left it, without any additional YAML configuration!

# HOW TO 
You can flash this to an ESP module. I used an ESP01-M module, like this one:
https://nl.aliexpress.com/item/1005008528226032.html
So that’s both an ESP01 and the ‘adapter board’ for 3.3V ↔ 5V conversion (since the ESP01 uses 3.3V and the AC uses 5V).


Create a new project in the ESPbuilder from Home assistant and use my YAML (from the examples directory), copy or modify the info from the generated YAML to mine, where it says '[insert yours]'.
Then you should be able to compile it. I think you can flash directly from Home Assistant,
but I downloaded the compiled binary and flashed with: https://github.com/esphome/esphome-flasher/releases

See the 4 module cable photos for wiring (for flashing and the wiring for connecting it to your AC). The connector is a 4 pins “JST XARP-04V”, you can for example order them here: https://es.aliexpress.com/item/1005009830663057.html. Alternatively it's possible to just use dupont cables and then put tape around the 4 ends to simulate the form of a connector (so that it makes it thicker), so that it will sit still in the connector socket, but make sure all 4 cables make connection, I tried that first and you might need to make adjustments because of 1 cable not making solid connection (this might result in errors in the log). So best to use the real connector. 


After you've connected the module to your AC, it should pop under settings/integrations/esphome as a 'new device' and then you can add it to HA. If not, check if it started a WIFI access point, which it will do if it can't connect to your home wifi. You can then connect to that and configure it from there (via 192.168.4.1)

**USE AT YOUR OWN RISK!**
