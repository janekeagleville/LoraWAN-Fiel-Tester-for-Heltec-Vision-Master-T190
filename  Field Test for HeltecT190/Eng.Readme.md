# LoRa Field Test — Heltec Vision Master T190

Standalone LoRa range tester for the **Heltec Vision Master T190** (ESP32-S3, SX1262, 1.9" 170×320 TFT, two buttons, no touch, no GPS).
This is **custom firmware, not Meshtastic**: the board drives the radio itself. It does not talk to the CYD or the Tracker and has no Meshtastic map, chat or node list.

> Note: the firmware was written and tested on a host (simulated radio and screen, ASan/UBSan). **It has not been compiled for ESP32 or run on the board**, so the first flash may need small fixes.

## Features

| Screen | Description |
|---|---|
| HOME | tiles: link test, signal, noise, channel busy, radio parameters, battery |
| LINK | ping-pong between two T190s: RSSI and SNR in **both directions**, GOOD / MARGINAL / WEAK rating against the demodulation limit for the SF, chart, packet loss, RTT |
| NOISE | **real noise measurement** (instantaneous radio RSSI, 10×/s): now, floor, peak, busy, chart with 1 min / 10 min / 1 h windows |
| SCAN | sweep of 863–870 MHz (100 kHz step) with peak hold; the test and Meshtastic frequencies are marked |
| MESH | Meshtastic listener (headers are in clear): sender, RSSI, SNR, hops, age; node names only with the default channel key; **ping of a selected node** with ACK measurement |
| SETUP | role, preset, frequency, power, interval, Meshtastic preset, colour inversion, screen flip, TFT power polarity; stored in NVS |

### Link test (two boards)
1. Both boards need the same preset and frequency.
2. Set one to **INIT** and the other to **ECHO** (SETUP → Role).
3. INIT sends a PROBE every 3/5/10/30 s; ECHO answers with a REPLY carrying the RSSI/SNR it heard the PROBE with, so INIT knows both directions.
4. Defaults: MediumFast (SF9, BW 250 kHz, CR 4/5), 869.525 MHz, 14 dBm, every 5 s. The test frame is 12 bytes with sync word 0x12, so Meshtastic does not decode it.
5. Rating: SNR margin over the limit (SF7 −7.5 dB, 2.5 dB lower per SF step): ≥10 dB GOOD, ≥3 dB MARGINAL, below that WEAK.

### Meshtastic node ping (MESH screen)
Works with any Meshtastic node on the list, no second T190 needed.
1. KEY (short) selects a node, KEY (long) sends a ping.
2. The T190 sends the node a packet requesting acknowledgement (want_ack), using its own random node ID (stored permanently, shown in the serial log), hop_limit 0 (direct test, no relaying) and the PRIVATE_APP port. The target does not display such a message but sends back an ACK.
3. The bottom panel shows ACK with RSSI, SNR, response time and the hop count of the reply, or NAK, or no ACK after 12 s. Sent and acknowledged counters are kept per node.
4. Limits: only works for a channel with the **default key** (AQ==) and a preset name matching "Mesh preset". With another key the node cannot read the packet and no ACK comes back. A ping is a transmission into the network (power and duty limit as in SETUP), so use it sparingly. RSSI and SNR describe the ACK heard here.

### EU868 limits (enforced by the firmware)
- 869.4–869.65 MHz: 10 % duty cycle; 868.0–868.6 MHz: 1 % and max 14 dBm; other sub-bands 0.1 %.
- The firmware tracks airtime over the last hour and stops transmitting above the limit (HOME shows DUTY LIMIT). Power is clamped to the band limit.

### Noise
Noise is the instantaneous RSSI in receive mode, aggregated into one-second buckets (average, min, max, busy) for one hour. "Floor" is the 10th percentile of the minima in the window; "busy" is the share of readings above floor + 8 dB. It is not sampled while transmitting or on SCAN.

## Controls (two buttons)
| | PRG (GPIO0) | KEY (GPIO21) |
|---|---|---|
| normal screens | short: next screen, long: previous | short: screen action (pause / time window / scan pause / next node), long: reset, clear or ping (MESH) |
| SETUP | short: next item, long: exit | short: change value, long: previous item |

Holding **KEY at power-up** flips the TFT power polarity (GPIO7). Use it if the screen stays black.

## Building
```
pio run -e t190 -t upload       # RGB colours
pio run -e t190_bgr -t upload   # if red and blue are swapped
pio device monitor              # CSV log every 5 s
```
Dependencies (RadioLib 7.1.2, TFT_eSPI 2.5.43, espressif32 6.9.0) are pinned in `platformio.ini`. The board is built as `esp32-s3-devkitc-1` with 8 MB flash and OPI PSRAM. You may need bootloader mode to flash (hold PRG, press RST).

## Pins (from the Meshtastic variant)
TFT ST7789: CS 39, DC 47, MOSI 48, SCK 38, RST 40, backlight 17, TFT power 7, VEXT 5. LoRa SX1262: CS 8, SCK 9, MOSI 10, MISO 11, RST 12, BUSY 13, DIO1 14, TCXO 1.8 V, DIO2 as RF switch. Battery: ADC 6, divider control 46. Buttons: 0 and 21.

## To verify on hardware
- TFT power polarity (GPIO7) and a possible BGR variant / colour inversion (SETUP).
- The battery multiplier (4.9 × 1.03) comes from Meshtastic; the percentage is approximate.
- The 170-pixel ST7789 offset (`CGRAM_OFFSET`): if the picture is shifted, change the rotation (Flip screen).
- Node names in MESH appear only for a channel with the default key, after a NODEINFO packet.

## Host tests
```
g++ -std=c++17 -fsanitize=address,undefined -Istubs -Iinclude test/units.cpp -o units && ./units
g++ -std=c++17 -fsanitize=address,undefined -Istubs -Iinclude test/sim.cpp -o sim && mkdir -p shots && ./sim && python3 tools/render.py shots
```
`units` checks AES (FIPS-197; CTR matches Python's library), Meshtastic headers, the link protocol, noise and duty limits. `sim` runs the whole firmware against a virtual SX1262, buttons and clock; `render.py` builds screen previews (substitute fonts).
