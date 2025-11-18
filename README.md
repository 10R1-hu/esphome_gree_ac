# Open source WIFI module replacement for Gree protocol based AC's for Home Assistant.
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
- **Universal External Sensor Support**: Use external ATC BLE temperature/humidity sensors
- **Robust Temperature Source Selection**: Choose between AC's own sensor or external ATC sensor
- **Automatic Fallback Logic**: Automatically falls back to AC sensor if external sensor fails or times out

See [FAN_LEVELS.md](FAN_LEVELS.md) for detailed information about fan speed levels.

## External Temperature Sensor Support

The component now supports using external ATC (Xiaomi) BLE temperature/humidity sensors as an alternative to the AC's built-in temperature sensor. This feature provides:

### Features:
- **Dynamic MAC Configuration**: Configure the ATC sensor's MAC address from Home Assistant without firmware reload
- **Temperature Source Selection**: Choose between "AC Own Sensor" or "External ATC Sensor" via a select entity
- **Automatic Fallback**: If the external sensor becomes unavailable or doesn't send data for 15 minutes, the system automatically falls back to the AC's own sensor
- **High Precision**: ATC temperature readings are displayed with 2 decimal places for greater accuracy
- **Additional Sensors**: When using an ATC sensor, both temperature and humidity are available in Home Assistant

### Configuration:
Add the following to your YAML configuration:

```yaml
climate:
  - platform: sinclair_ac
    name: "Living Room AC"
    # ... other configuration ...
    
    # Temperature source selector
    temp_source_select:
      name: "AC Temperature Source"
    
    # ATC MAC address input (format: AA:BB:CC:DD:EE:FF)
    atc_mac_address_text:
      name: "ATC MAC Address"
      mode: text
    
    # AC's indoor temperature sensor
    ac_indoor_temp_sensor:
      name: "AC Indoor Temperature"
      unit_of_measurement: "°C"
      accuracy_decimals: 1
    
    # ATC sensor readings (shown when ATC sensor is configured)
    atc_room_temp_sensor:
      name: "ATC Room Temperature"
      unit_of_measurement: "°C"
      accuracy_decimals: 2
    
    atc_room_humidity_sensor:
      name: "ATC Room Humidity"
      unit_of_measurement: "%"
      accuracy_decimals: 1
```

### Usage:
1. In Home Assistant, enter the MAC address of your ATC sensor in the "ATC MAC Address" text input (format: AA:BB:CC:DD:EE:FF)
2. Set up your ATC sensor to send temperature/humidity data (typically via ESPHome's BLE tracking or Home Assistant's bluetooth integration)
3. Use the "AC Temperature Source" select entity to switch between "AC Own Sensor" and "External ATC Sensor"
4. If the external sensor fails or doesn't send data for 15 minutes, the system automatically falls back to the AC's sensor and logs the event

### Notes:
- The fallback mechanism ensures your AC continues to work even if the external sensor fails
- All sensor names appear clearly in Home Assistant for easy identification
- This feature works universally across all rooms and sensors in your setup

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
