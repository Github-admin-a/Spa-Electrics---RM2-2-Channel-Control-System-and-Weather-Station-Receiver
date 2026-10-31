# Pool Weather Bridge – 433 MHz + 915 MHz Transceiver

One ESP8266 (NodeMCU) board with two RFM69 radios and a built-in web UI. It does three jobs:

- **Pool light controller (915 MHz):** sends and receives the packets of a Spa Electrics pool light remote at 916.02 MHz. It tracks whether the light is on and which colour it's showing, and lets you control it from Home Assistant or the web page.
- **HP1000 weather station receiver (433 MHz):** receives the HP1000 / Fine Offset outdoor sensor array (WH24) and indoor sensor (WH25) and publishes the readings to Home Assistant over MQTT.
- **Wake on LAN:** wakes a PC or NAS from a button on the web page or an HTTP call. This works without MQTT or Home Assistant, so it can wake the machine they run on.

All three share one WiFi connection and one MQTT broker connection. Home Assistant MQTT discovery creates all the entities automatically.

## Features

### Pool light
- Home Assistant light entity with on/off and the light's colours as effects. A colour is selected by stepping the light through power OFF/ON cycles, the same way you would by hand.
- **Sync** button that sends the remote's own SYNC command, then restores the colour the light was showing.
- Listens for the real remote. Pressing it updates the tracked state and cancels any action the bridge is running.
- "Tracked colour" and "Tracked power" controls to correct the bridge's model if it gets out of step with the light.
- "Last remote press" sensor with the raw packet as attributes. This is handy for finding your remote's ID.
- TX power, packets per press, and all light timings can be changed in the web UI.
- RF test mode: 10 s of dummy packets that the light ignores, for SDR or range checks.

Colours, in the order the light steps through them: Electric Blue, Midnight Magenta, Ruby Red, Spring Lime, Lagoon Green, Tropical Aqua, Daylight White, Warm White, Slow Colour Blend, Fast Colour Change.

### Weather
- **Outdoor (WH24):** temperature, humidity, wind direction and bearing, wind speed, gust, rainfall total, UV index, solar radiation, battery, signal strength.
- **Rain totals:** today, this week (weeks start on Sunday), this month and this year. They are kept on the board, can be set by hand like on the console, and are published to Home Assistant.
- **Indoor (WH25):** temperature, humidity, pressure, battery, signal strength.
- **Receiver diagnostics:** IP, WiFi signal, uptime, restart reason, valid frame count, WiFi drops.
- **Calibration** to match the HP1000 console's Calibration screen:
  - indoor and outdoor temperature and humidity offsets
  - ABS and REL barometer offsets
  - wind direction offset and wind gain
  - rain gain
  - solar lux-per-W/m² factor (default 126.7)
  - solar and UV gain
- Optional list of allowed sensor IDs, so a neighbour's station is ignored.
- All weather entities appear under one Home Assistant device (`hp1000_station`) that isn't tied to the board, so you can swap boards without reconfiguring Home Assistant.

### Wake on LAN
- Saves a name, MAC address, broadcast address and UDP port. A blank broadcast address is worked out from the board's own IP and subnet mask.
- Wake button on the `/wol` page, plus `POST /api/wol`, which suits a phone shortcut.
- Optional "is it up?" check against a target IP and TCP port, e.g. a NAS web port.

### General
- One shared page layout (header and tab bar) across all pages, with a light, dark or follow-device theme.
- WiFiManager setup portal on first boot.
- Firmware upload page (`/update`) and Arduino IDE network OTA.
- Self-recovery: WiFi reconnect every 30 s while offline, restart after 10 min offline, restart if the MQTT broker is unreachable for 15 min. It never restarts in the middle of a pool light action.

## Hardware

- NodeMCU 1.0 (ESP-12E) or a similar ESP8266 board
- RFM69H**W/HCW** 915 MHz module (the high-power version, for the pool light)
- RFM69 433 MHz module (for the weather sensors)
- 10–100 µF capacitor
- Wire antennas: 8.2 cm for 915 MHz and 17.3 cm for 433 MHz. Keep them a few cm apart.

### Wiring (3.3 V only – never 5 V on an RFM69)

| Signal | NodeMCU pin | Notes |
|---|---|---|
| SCK  | D5 (GPIO14) | both radios |
| MISO | D6 (GPIO12) | both radios |
| MOSI | D7 (GPIO13) | both radios |
| RESET | D2 (GPIO4) | both radios' RESET pins joined |
| 3.3 V / GND | 3V3 / GND | both radios |
| 915 MHz NSS | D8 (GPIO15) | |
| 915 MHz DIO0 | D1 (GPIO5) | interrupt |
| 433 MHz NSS | D3 (GPIO0) | |
| 433 MHz DIO0 | D0 (GPIO16) | polled |

Fit the capacitor across 3.3 V/GND next to the 915 MHz module, because it draws about 130 mA while transmitting at 20 dBm.

Why these pins: D3, D4 and D8 are the ESP8266's boot-strap pins. Only the chip selects are on them, because they are inputs on the RFM69 and can't stop the board booting. The 433 MHz DIO0 is on D0, which has no interrupt, so it is polled. The RFM69 holds a received frame in its FIFO until it is read.

## Building

**Board:** "NodeMCU 1.0 (ESP-12E Module)", ESP8266 Arduino core 3.x, Flash Size "4MB (FS:2MB OTA:~1019KB)"

**Libraries:**
- RadioLib 7.6.0 or 7.7.x
- WiFiManager (tzapu) 2.0.x
- PubSubClient 2.8
- ArduinoJson 7.x

## Setup

1. Flash the sketch over USB.
2. Connect to the WiFi network **`PoolWeather-Setup`** and choose your WiFi. You can also enter the MQTT broker details there.
3. Open `http://poollight.local/` or the board's IP address.
4. On **Settings**:
   - Set your **timezone** as a POSIX string, e.g. `AEST-10`, `EST5EDT`, `CET-1CEST,M3.5.0,M10.5.0/3`. The default is `UTC0`. The timezone is used for log times and for when the rain totals roll over.
   - Set your **remote ID** (4 hex digits) and **light number** (decimal). They default to `0000` / `0`, which the light ignores. To find yours, press a button on your real remote and read the ID and light number from the event log or the "Last remote press" entity.
   - Fill in the MQTT broker details if you didn't enter them in the portal. Leave the broker blank to run without MQTT.
5. On **Weather → Setup**, optionally list your sensor IDs (e.g. `42,117`). Leave it blank to accept every sensor. The IDs appear on the Weather page once the sensors are heard. They change when you replace the batteries.
6. On **Weather → Calibration**, copy the values from your console's Calibration screen, and set the current rain totals if you want them to carry on from the console.
7. On **Wake on LAN**, enter the target's MAC address. Use its wired port, because WoL rarely works over WiFi.

## Web pages

| Path | What |
|---|---|
| `/` | Pool light control |
| `/weather` | Weather sensors and receiver status |
| `/weather/setup` | 433 MHz radio, allowed sensor IDs, weather topic |
| `/weather/calibration` | Console-style calibration and rain totals |
| `/wol` | Wake on LAN |
| `/settings` | Device, theme, WiFi, MQTT, pool remote and timing |
| `/update` | Firmware upload (OTA) |

HTTP API: `POST /api/on`, `/api/off`, `/api/sync`, `/api/wol`; `GET /api/status`, `/api/wol/status`, `/weather/data.json`.

## MQTT

Pool light topics are under `<base>/` (default `poollight`):

| Topic | Direction | Payload |
|---|---|---|
| `poollight/status` | out | `online` / `offline` (LWT, also used by the weather entities) |
| `poollight/light/set` | in | `ON` / `OFF` |
| `poollight/light/state` | out | `ON` / `OFF` |
| `poollight/light/effect/set` | in | colour name or number 1–10 |
| `poollight/light/effect` | out | current colour name |
| `poollight/sync` | in | `sync` |
| `poollight/command` | in | `power` (one raw power toggle) |
| `poollight/track/colour/set` | in | correct the tracked colour without transmitting |
| `poollight/track/power/set` | in | correct the tracked power without transmitting |
| `poollight/state` | out | JSON status (status text, RSSI, IP…) |
| `poollight/rx` | out | JSON of the last packet heard from a remote |

Weather topics are under the weather prefix (default `homeassistant/sensor/hp1000`):

- `<prefix>/outdoor_<id>/state` – JSON readings from the outdoor array, including rain totals
- `<prefix>/indoor_<id>/state` – JSON readings from the indoor sensor
- `<prefix>/bridge/state` – receiver diagnostics

Discovery is published under `homeassistant/` (configurable).

## Pool light protocol notes

The remote sends 6-byte packets at 916.02 MHz, 2-FSK, 9.6 kbps, about 155 kHz deviation, sync word `0x2D 0xD4`:

```
<remote id hi> <remote id lo> <light #> <command> <byte 4> <checksum>
```

- Commands: `0x11` = power toggle, `0x13` = SYNC (byte 4 = `0x03`)
- Checksum: `0x100 - ((0x2D + 0xD4 + bytes 0..4) % 0x100)`
- Each press is sent as 4 identical packets about 17 ms apart.

The light has no colour command. Colour is changed by power cycling. An OFF of about 2 s followed by ON steps to the next colour. A rapid OFF/ON (under about 1 s) resets it (it flashes blue). An OFF longer than about 60 s keeps the current colour. The bridge models this to know which colour is showing. All timings can be adjusted in Settings.

## Weather protocol notes

433.906 MHz, 2-FSK, 17.24 kbps, sync `0x2DD4`. The WH24 decoding (including the UV index table) follows [rtl_433](https://github.com/merbanan/rtl_433)'s Fine Offset decoder. Solar radiation is lux ÷ 126.7 by default, the same conversion the HP1000 console uses, and the factor can be changed on the calibration page.

## ⚠️ Security

The web pages, the `/update` firmware page, the HTTP API and Arduino IDE OTA have **no password**. Anyone on the same network can control the light, send Wake on LAN, change settings or flash new firmware. Only run this on a network you trust, such as an isolated IoT VLAN. To reach it from outside, use a VPN rather than port forwarding.
