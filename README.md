# Overhead Water Tank Monitor

Monitors an overhead water tank with an ESP32-S3 and an ultrasonic sensor. Water
level, fill percentage, stored volume and net flow are printed to the Serial
Monitor and shown on a live web page served by the board itself. Any device on
the same WiFi network can open the page. Tank dimensions are set from the web
page and stored in flash, so they survive a power cut.

## Demo video

[demo.mp4](demo.mp4) — the Serial Monitor and the web dashboard updating together
as water is poured into the tank.

## Hardware

| Part | Notes |
|------|-------|
| ESP32-S3 development board | Any S3 module with USB |
| HC-SR04 ultrasonic sensor | HC-SR04P or JSN-SR04T also work |
| 2 resistors | 1 kΩ and 2 kΩ, for the Echo divider |

### Wiring

| Sensor pin | ESP32-S3 | Notes |
|------------|----------|-------|
| VCC  | 5V   | HC-SR04 needs 5 V to transmit reliably |
| GND  | GND  | common ground |
| TRIG | GPIO 5  | direct connection |
| ECHO | GPIO 18 | **through a voltage divider** |

The Echo pin idles at 5 V but ESP32 GPIOs are 3.3 V tolerant only. Drop it with a
divider, or the pin will be damaged over time:

```
ECHO ---[ 1k ]---+--- GPIO 18
                 |
               [ 2k ]
                 |
                GND
```

Mount the sensor at the top of the tank, pointing straight down at the water,
clear of the inlet stream and the walls.

## Build and upload

Arduino IDE with the **esp32 by Espressif Systems** core (tested on 3.3.12).

1. **Tools → Board → esp32 → ESP32S3 Dev Module**
2. **Tools → USB CDC On Boot → Enabled**
3. **Tools → Port →** your COM port
4. Upload, then open the Serial Monitor at **115200 baud**

> **USB CDC On Boot must be Enabled.** With the default *Disabled*, `Serial`
> output is routed to the UART0 pins instead of the USB cable and the Serial
> Monitor stays empty even though the firmware is running correctly.

### WiFi

Set your network at the top of `WaterTankMonitor.ino`:

```cpp
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
```

The ESP32 has a 2.4 GHz radio only; it cannot join a 5 GHz network. On boot the
firmware scans, finds the network's channel and joins on it, retrying every 20
seconds if the network is not yet available.

The Serial Monitor prints the assigned address:

```
Connected to 'YOUR_WIFI_NAME'. Dashboard: http://192.168.1.50 or http://watertank.local
```

> Some guest and corporate networks enable client isolation, which blocks
> device-to-device traffic and makes the page unreachable even though the board
> is connected. A phone hotspot or a home router avoids this.

## Calibration

Readings stay blank until the real tank dimensions are confirmed, so the default
values are never mistaken for a calibrated tank. On the web page, fill in:

| Field | Meaning |
|-------|---------|
| Tank height | Tank floor to the maximum fill line |
| Sensor-to-bottom distance | Sensor face straight down to the tank floor |
| Shape and dimensions | Diameter, or length and width |

These are two different measurements. The sensor usually sits above the tank
rim, so the sensor-to-bottom distance is the larger of the two. Press **Save**
and the values are written to flash (NVS keys `h`, `empty`, `shape`, `d1`, `d2`,
`verified`) and reloaded on every boot.

## How it works

### Distance

The sensor is triggered with a 10 µs pulse and the Echo pin stays high for the
round-trip time of the sound:

```
distance (cm) = echo duration (µs) × 0.01715
```

Sound travels at about 343 m/s, which is 0.0343 cm/µs. The pulse travels to the
water and back, so the constant is halved to 0.01715.

### Level, volume and percentage

```
level    = sensor-to-bottom distance − measured distance
area     = π r²            (cylinder)
area     = length × width  (rectangle)
volume_L = area × level / 1000        (1 litre = 1000 cm³)
percent  = level / tank height × 100
```

### Flow

```
flow (L/min) = level change × area × 60 / elapsed ms
```

The 1000 that converts cm³ to litres and the 1000 that converts milliseconds to
seconds cancel, leaving only the factor of 60.

Flow is reported from a 5 second rolling window of accepted levels, and resets
after 2.5 seconds without movement so an old pour does not linger.

### Stability

An ultrasonic sensor is noisy and quantises to roughly 0.4 cm, so raw readings
are filtered in stages:

1. One reading every 100 ms into a 9-slot buffer; the **median** is taken, which
   discards spikes from wall echoes and failed pings.
2. A second median of the last three values, then an exponential moving average.
3. The displayed level only moves when a change larger than 0.15 cm is confirmed
   on two consecutive samples, which stops the number flickering.
4. Fill and drain are detected from the **least-squares gradient** over a two
   second window rather than from the difference between samples. Differences
   are dominated by the sensor's quantisation step and their sign is meaningless;
   the gradient averages it out. Movement slower than 0.05 cm/s reads as still.

## HTTP API

| Route | Method | Purpose |
|-------|--------|---------|
| `/` | GET | dashboard page |
| `/api/status` | GET | readings as JSON |
| `/api/config` | POST | save tank dimensions |
| `/api/wifi` | GET | connection diagnostics |

```json
{
  "ok": true, "calibrated": true, "distance": 12.02,
  "level": 1.98, "percent": 15.2, "volume": 0.099,
  "capacity": 0.653, "flow": 0.000, "flowReady": true, "trend": 0
}
```

`trend` is `1` filling, `-1` draining, `0` settled, and drives the pouring
animation. It reacts in about 1.25 s, well before `flow` has enough history.

## Files

| File | Contents |
|------|----------|
| `WaterTankMonitor.ino` | sensor, WiFi, web server, flash storage |
| `LevelFilter.h` | median, smoothing and trend detection |
| `index_html.h` | dashboard page, stored in flash as `PROGMEM` |

## Limitations

- **This is not a pipe flow meter.** An ultrasonic sensor only sees the water
  surface, so the reported figure is the *net* change in stored volume: inflow
  minus outflow. If a pump and a tap run at the same rate it reads zero while
  water is clearly moving. True pipe flow needs an inline sensor such as a
  YF-S201.
- The speed of sound varies by about 0.6 m/s per °C. The fixed 343 m/s constant
  gives roughly 1.7 % error over a 10 °C swing.
- `pulseIn()` blocks for up to 30 ms when an echo is lost. An interrupt or RMT
  based reading would avoid this.
- Foam, steam and a sloped sensor mounting all scatter the echo and increase the
  number of discarded readings.
