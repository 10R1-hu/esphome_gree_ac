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

The component now supports using external ATC (Xiaomi ATC1441 custom firmware) BLE temperature/humidity sensors as an alternative to the AC's built-in temperature sensor. This feature provides:

### Features:
- **Built-in BLE Listener**: Direct BLE advertisement parsing - no separate `atc_mithermometer` component needed!
- **Dynamic MAC Configuration**: Configure the ATC sensor's MAC address from Home Assistant without firmware reload
- **Runtime MAC Switching**: Change the ATC sensor MAC address on-the-fly; the listener immediately switches to the new device
- **Temperature Source Selection**: Choose between "AC Own Sensor" or "External ATC Sensor" via a select entity
- **Automatic Fallback**: If the external sensor becomes unavailable or doesn't send data for 15 minutes, the system automatically falls back to the AC's own sensor
- **High Precision**: ATC temperature readings are displayed with 2 decimal places for greater accuracy
- **Additional Sensors**: Temperature, humidity, and battery percentage are all exposed in Home Assistant
- **ESP32 Required**: BLE functionality requires ESP32 (ESP8266 does not support BLE)

### Configuration:
Add the following to your YAML configuration:

```yaml
# Enable BLE tracker (ESP32 only - required for ATC sensor support)
esp32_ble_tracker:
  scan_parameters:
    active: false

climate:
  - platform: sinclair_ac
    name: "Living Room AC"
    # ... other configuration ...
    
    # Temperature source selector
    temp_source_select:
      name: "AC Temperature Source"
    
    # ATC MAC address input (format: AA:BB:CC:DD:EE:FF)
    # Can be changed at runtime from Home Assistant
    atc_mac_address_text:
      name: "ATC MAC Address"
      mode: text
      # Optional: restore_value: true to persist across reboots
    
    # AC's indoor temperature sensor
    ac_indoor_temp_sensor:
      name: "AC Indoor Temperature"
      unit_of_measurement: "°C"
      accuracy_decimals: 1
      device_class: temperature
      state_class: measurement
    
    # ATC sensor readings (exposed when ATC BLE advertisements are received)
    atc_room_temp_sensor:
      name: "ATC Room Temperature"
      unit_of_measurement: "°C"
      accuracy_decimals: 2
      device_class: temperature
      state_class: measurement
    
    atc_room_humidity_sensor:
      name: "ATC Room Humidity"
      unit_of_measurement: "%"
      accuracy_decimals: 1
      device_class: humidity
      state_class: measurement
    
    atc_battery_sensor:
      name: "ATC Battery"
      unit_of_measurement: "%"
      accuracy_decimals: 0
      device_class: battery
      state_class: measurement
```

### Usage:
1. Flash your Xiaomi thermometer with [ATC1441 custom firmware](https://github.com/atc1441/ATC_MiThermometer)
2. Enable `esp32_ble_tracker` in your ESPHome configuration (see above)
3. In Home Assistant, enter the MAC address of your ATC sensor in the "ATC MAC Address" text input (format: AA:BB:CC:DD:EE:FF)
4. The component will automatically start receiving temperature, humidity, and battery data from the ATC sensor
5. Use the "AC Temperature Source" select entity to switch between "AC Own Sensor" and "External ATC Sensor"
6. Change the MAC address at any time - the listener will immediately switch to the new sensor (no reboot required!)
7. If the external sensor fails or doesn't send data for 15 minutes, the system automatically falls back to the AC's sensor and logs the event

### Notes:
- **No separate component needed**: Unlike previous versions, you don't need to configure `atc_mithermometer` separately
- The built-in listener parses ATC custom firmware BLE advertisements directly (Service Data UUID 0x181A)
- MAC address matching supports both the advertiser address and the embedded MAC in the ATC payload
- The fallback mechanism ensures your AC continues to work even if the external sensor fails
- All sensor names appear clearly in Home Assistant for easy identification
- This feature works universally across all rooms and sensors in your setup
- **ESP8266 compatibility**: If using ESP8266, the BLE listener is automatically disabled, and the component continues to work without ATC support

## BLE Requirements

The external ATC sensor feature requires **ESP32** hardware. ESP8266 does not support Bluetooth Low Energy (BLE) and cannot use this feature.

### Hardware Requirements:
- **ESP32**: Required for BLE functionality (e.g., ESP32-DevKit, ESP32-WROOM, ESP01-M with ESP32)
- **ESP8266**: Does NOT support BLE - the component will work but without external ATC sensor support

### Configuration Requirements:
In your ESPHome YAML configuration, you must:

1. **Use ESP32 platform**:
```yaml
esp32:
  board: esp32dev  # or your specific ESP32 board
  framework:
    type: esp-idf  # or arduino
```

2. **Enable BLE tracker**:
```yaml
esp32_ble_tracker:
  scan_parameters:
    active: false
```

Without the `esp32_ble_tracker` component enabled, the ATC sensor listener will not function, and you'll see a warning in the logs at startup.

## Debug Logging & Verbose Mode

The component includes comprehensive logging to help troubleshoot issues and understand system behavior.

### Log Levels

| Level | Description | Use Case |
|-------|-------------|----------|
| `ERROR` | Critical errors only | Production - minimal logging |
| `WARN` | Warnings and errors | Production - recommended default |
| `INFO` | General information | Normal operation visibility |
| `DEBUG` | Detailed debugging info | Troubleshooting and development |
| `VERBOSE` | Very detailed logs | Deep debugging (not recommended for production) |

### Configuring Log Level

Set the logger level in your ESPHome YAML:

```yaml
logger:
  baud_rate: 0  # disable UART logging (UART used for AC communication)
  level: DEBUG  # Set to DEBUG for troubleshooting
```

**Recommendation**: Use `DEBUG` level during initial setup and troubleshooting. Once stable, change to `INFO` or `WARN` for production use to reduce log clutter.

### Verbose Mode (Advanced)

For even more detailed logging, you can enable **verbose mode** by defining the `SINCLAIR_AC_VERBOSE_LOG` macro. This adds extra ESP_LOGV statements that show:

- Raw BLE packet data and parsing details
- Timestamp information for sensor updates
- MAC address matching attempts
- Internal state transitions

#### Enabling Verbose Mode

Add the following to your ESPHome YAML configuration:

```yaml
esphome:
  name: your_ac_name
  platformio_options:
    build_flags:
      - -DSINCLAIR_AC_VERBOSE_LOG
```

**Note**: This significantly increases log output and should only be used for deep debugging. The verbose logs require logger level to be set to `VERBOSE`:

```yaml
logger:
  level: VERBOSE
```

### What Gets Logged at DEBUG Level

With `level: DEBUG`, you'll see:
- Component startup and version information
- BLE tracker registration status
- Temperature source changes (AC own sensor ↔ External ATC sensor)
- ATC sensor data updates (temperature, humidity, battery)
- Automatic fallback events when ATC sensor times out
- MAC address validation results
- User-initiated changes via Home Assistant UI

### What Gets Logged at VERBOSE Level (with macro)

With `SINCLAIR_AC_VERBOSE_LOG` defined and `level: VERBOSE`:
- Raw BLE advertisement data and parsing
- Detailed MAC address comparison logic
- Sensor update timestamps
- Individual sensor publication events
- All DEBUG level logs plus extra verbose details

### Example Debug Output

With `level: DEBUG`:
```
[D][sinclair_ac:51] Sinclair AC component v0.0.3 starting...
[I][sinclair_ac:57] BLE tracker listener registered for dynamic ATC sensor support
[D][sinclair_ac:48] Temperature source initialized to: AC Own Sensor
[D][sinclair_ac:576] ATC BLE data received from AA:BB:CC:DD:EE:FF: Temp=22.45°C, Hum=55.2%, Batt=87%
[D][sinclair_ac:454] ATC sensor data updated: Temp=22.45°C, Humidity=55.2%
[D][sinclair_ac:340] User changed temperature source via UI to: External ATC Sensor
[D][sinclair_ac:201] Temperature source changing from 'AC Own Sensor' to 'External ATC Sensor'
```



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
