# Open source WIFI module replacement for Gree protocol based AC's for Home Assistant.

**Version: v0.0.4**

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
- **External Sensor Support**: Use any ESPHome sensor as an external temperature source
- **Temperature Source Selection**: Choose between AC's own sensor or external sensor
- **Persistent User Settings**: All user preferences (display mode, swing positions, temperature source, switches) are automatically saved and restored across reboots without requiring YAML `restore_value` or `restore_mode` configuration

See [FAN_LEVELS.md](FAN_LEVELS.md) for detailed information about fan speed levels.

## External Temperature Sensor Support

The component supports using any ESPHome sensor as an external temperature source instead of the AC's built-in temperature sensor. This feature provides:

### Features:
- **Universal Sensor Support**: Works with any ESPHome sensor (BLE, I2C, SPI, UART, etc.)
- **Simple Configuration**: Reference any sensor by its `id` using `current_temperature_sensor:`
- **Temperature Source Selection**: Choose between "AC Own Sensor" or "External Sensor" via a select entity
- **Smart Switching**: External sensor updates only affect climate when "External Sensor" is selected
- **No Additional Complexity**: No internal BLE parsing or MAC address management

### Configuration:
Add the following to your YAML configuration:

```yaml
# Example with pvvx_mithermometer BLE sensor
sensor:
  - platform: pvvx_mithermometer
    mac_address: "A4:C1:38:0D:10:15"
    temperature:
      name: "External Room Temperature"
      id: external_room_temp

climate:
  - platform: sinclair_ac
    name: "Living Room AC"
    # ... other configuration ...
    
    # Reference external temperature sensor
    current_temperature_sensor: external_room_temp
    
    # Temperature source selector
    temp_source_select:
      name: "AC Temperature Source"
    
    # AC's indoor temperature sensor (optional, for display in HA)
    ac_indoor_temp_sensor:
      name: "AC Indoor Temperature"
      unit_of_measurement: "°C"
      accuracy_decimals: 1
      device_class: temperature
      state_class: measurement
```

### Other Sensor Examples:

#### DHT22 Sensor
```yaml
sensor:
  - platform: dht
    pin: GPIO4
    temperature:
      name: "Room Temperature"
      id: room_temp
    humidity:
      name: "Room Humidity"
    model: DHT22
    update_interval: 60s

climate:
  - platform: sinclair_ac
    name: "Bedroom AC"
    current_temperature_sensor: room_temp
    temp_source_select:
      name: "AC Temperature Source"
```

#### Home Assistant Sensor
```yaml
sensor:
  - platform: homeassistant
    name: "Living Room Temperature from HA"
    id: ha_room_temp
    entity_id: sensor.living_room_temperature

climate:
  - platform: sinclair_ac
    name: "Living Room AC"
    current_temperature_sensor: ha_room_temp
    temp_source_select:
      name: "AC Temperature Source"
```

### Usage:
1. Define your external sensor in YAML with an `id:`
2. Reference the sensor in the climate configuration using `current_temperature_sensor:`
3. Use the "AC Temperature Source" select entity in Home Assistant to switch between:
   - **"AC Own Sensor"**: Uses the AC's built-in UART-reported indoor temperature
   - **"External Sensor"**: Uses the external sensor you configured
4. When switched to "AC Own Sensor", external sensor updates are ignored
5. When switched to "External Sensor", the climate tracks the external sensor value

### Notes:
- The component does **not** require BLE tracker or any specific sensor type
- You can use **any** ESPHome sensor platform (BLE, I2C, WiFi, Home Assistant, etc.)
- Works on both ESP8266 and ESP32
- Temperature source preference is saved and restored across reboots
- External sensor is optional - the AC works normally without it using its own sensor

## Persistent Settings (v0.0.3+)

From version 0.0.3 onwards, all user preferences are automatically persisted across reboots:

### Persisted Settings:
- Display mode (OFF, Auto, Set, Actual, Outside temperature)
- Display unit (°C / °F)
- Vertical swing position (12 positions)
- Horizontal swing position (7 positions)
- Temperature source selection (AC Own Sensor / External Sensor)
- All switch states (Plasma, Beeper, Sleep, X-fan, Save)

### Features:
- **No YAML configuration needed**: You do NOT need to add `restore_value: true` or `restore_mode` to your entity configurations
- **Automatic saving**: Settings are saved immediately when changed via Home Assistant
- **Smart validation**: Invalid settings are detected and corrected on load with fallback to safe defaults
- **Cross-reboot persistence**: All settings survive ESP reboots, power cycles, and firmware updates

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
