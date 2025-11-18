# Fan Speed Levels in ESPHome Gree AC

## Question / Kérdés
**Hungarian:** "Most hány ventilátor fokozat van engedélyezve?"  
**English:** "How many fan speed levels are currently enabled?"

## Answer / Válasz

### After Recent Changes:
**Currently, there are 8 fan speed levels enabled in the system.**

## Fan Speed Level Details / Ventilátor fokozat részletei

The component now supports the following fan speed settings:

| Number | Name | Description | Protocol Values |
|--------|------|-------------|-----------------|
| 0 | Auto | Automatic fan speed control | fanSpeed1=0, fanSpeed2=0 |
| 1 | Quiet | Silent operation - minimal noise | fanSpeed1=1, fanSpeed2=1, fanQuiet=true |
| 2 | Low | Low fan speed | fanSpeed1=1, fanSpeed2=1 |
| 3 | Medium-Low | Between low and medium | fanSpeed1=2, fanSpeed2=2 |
| 4 | Medium | Medium fan speed | fanSpeed1=3, fanSpeed2=2 |
| 5 | Medium-High | Between medium and high | fanSpeed1=4, fanSpeed2=3 |
| 6 | High | High fan speed | fanSpeed1=5, fanSpeed2=3 |
| 7 | Turbo | Maximum fan speed | fanSpeed1=5, fanSpeed2=3, fanTurbo=true |

## History / Történet

### Before Changes (Previous State):
Previously, only **5 fan speed levels** were enabled:
- 0 - Auto
- 1 - Low
- 2 - Medium
- 3 - High
- 4 - Turbo

The following modes were commented out and not accessible:
- Quiet mode
- Medium-Low mode
- Medium-High mode

### Change Motivation:
The additional fan speed levels were enabled to provide users with more granular control over their air conditioning unit's fan speed. This is particularly useful for:
- Fine-tuning comfort levels
- Optimizing energy efficiency
- Reducing noise in quiet environments (Quiet mode)
- Better adapting to different room sizes and cooling needs

## Technical Implementation

The fan modes are implemented using a combination of protocol fields:
- `fanSpeed1` and `fanSpeed2`: Primary fan speed indicators (0-5)
- `fanQuiet`: Boolean flag for quiet mode operation
- `fanTurbo`: Boolean flag for turbo mode operation

These fields are encoded in the Sinclair AC protocol packets and allow the ESP module to communicate the desired fan speed to the air conditioning unit.

## Compatibility / Kompatibilitás

This implementation has been tested with:
- Gree AC units
- Daizuki AC units
- TGM AC units
- Sinclair AC units using the Sinclair protocol

Units must support the GRJWB04-J / Cs532ae WiFi module interface.
