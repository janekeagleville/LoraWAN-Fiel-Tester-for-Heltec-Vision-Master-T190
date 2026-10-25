# LoRa Field Test — Heltec Vision Master T190

Standalone LoRa range tester for the **Heltec Vision Master T190** (ESP32-S3, SX1262, 1.9" 170×320 TFT, two buttons, no touch, no GPS).
This is **custom firmware, not Meshtastic**: the board drives the radio itself. It has no map or chat. It passively listens to the Meshtastic network, shows a list of the nodes it hears, lets you select one to observe and ping it. It works with a single board, no second T190 needed.

> Note: the firmware was written and tested on a host (simulated radio and screen, ASan/UBSan). **It has not been compiled for ESP32 or run on the board**, so the first flash may need small fixes.

## Features

| Screen | Description |
|---|---|
| HOME | tiles: nodes heard, last packet (RSSI, SNR, GOOD / MARGINAL / WEAK rating against the demodulation limit for the SF), noise, channel busy, radio parameters, battery; your node name in the header |
| NOISE | **real noise measurement** (instantaneous radio RSSI, 10×/s): now, floor, peak, busy, chart with 1 min / 10 min / 1 h windows |
| SCAN | sweep of 863–870 MHz (100 kHz step) with peak hold; the Meshtastic channel frequency is marked |
| MESH | Meshtastic listener (headers are in clear): sender, RSSI, SNR, hops, age; node names only with the default channel key; **ping of a selected node** with ACK measurement |
| NODE | details of the selected Meshtastic node: name, ID, RSSI, SNR, hops, average RSSI, min/max, chart of the last 40 readings, packet count, age, ping result |
| SETUP | ping power, Meshtastic preset, colour inversion, screen flip, TFT power polarity, beacon interval, auto ping, Bluetooth (fixed PIN 123456); stored in NVS |

### Boot splash
At power-up (6 s): Matrix-style character rain, a mesh graph that draws itself with pulsing rings, the "MESH FIELD" title and a progress bar.

### Battery and charging
The BATTERY tile on HOME shows the voltage and an approximate percentage. When charging is detected it shows "CHARGING" instead of the percentage (also in the configurator and in the BLE status, field `chg`). Detection is a heuristic because the T190 has no known charge-status pin (see "To verify on hardware").

Critical-battery shutdown: when the level stays below 5% for 3 consecutive reads (about 30 s) and no charging is detected, the board shows "BATTERY CRITICAL" for 4 s, turns the radio and display off and enters deep sleep. Every 30 s it wakes briefly, measures the voltage and boots again only when the voltage has clearly risen (>=60 mV above the minimum, i.e. power was connected) or reaches >=3.75 V. A button reset also boots normally (if the battery is still critical it shuts down again after about 30 s). Without a charge-status pin the restart is a heuristic; not verified on hardware.

### Node name
You type your node's name (up to 12 ASCII characters, no Polish letters) in the Bluetooth configurator ("Node name" field, after entering the PIN). The board stores it permanently, shows it in the header instead of "LoRa Field Test", reports it in the device status and in the BLE name (`T190-<name>`, after Bluetooth is switched on again). It is not a name advertised on the Meshtastic network.

### Visibility on the Meshtastic network (beacon) and GPS position
Listening and pinging alone do not make other devices see this node: a ping is a private-port packet to one node, and the board used to advertise nothing about itself. Now a **beacon** does: every 15 / 30 (default) / 60 / 180 min (SETUP → Node beacon, or the configurator) it broadcasts:
- **NODEINFO** with your name (long name = node name, short name = first 4 characters; without a name: `T190 xxxx`, xxxx being the end of the node ID),
- **POSITION** with the position entered in the configurator (latitude, longitude, altitude; empty fields = no position; "Use phone location" takes it from the phone's GPS).
The first beacon goes out about 30 s after boot, and about 3 s after you change the name, position or interval. On HOME, holding KEY sends a beacon at once (the sent counter is in the footer).
Packets are encrypted with the default channel key (AQ==), hop_limit 3, role CLIENT_MUTE (the node relays nothing). Other nodes will list it if they use the **same preset** (name and frequency as in "Mesh preset") on a default-key channel, are in direct range or reachable through relays, and the beacon arrived. The position is public to anyone receiving that channel. Limitations: no GPS receiver (the position is manual and fixed), the board does not answer NODEINFO requests or relay other packets, and transmissions count against the duty-cycle limit (a beacon is about 0.2–0.4 s of airtime on LongFast).

### Node list and observing one node
1. The **MESH** screen lists the nodes it hears (freshest first): name or ID, RSSI, SNR, hops, age.
2. **KEY** (short) moves the selection to the next node. The choice is remembered by node ID, so it does not jump when the list reorders.
3. **PRG** goes to the **NODE** screen, which observes the selected node: current RSSI and SNR, hops, averages, min/max, chart of the last 40 readings, packet count, age. On NODE, KEY keeps cycling nodes.
4. Once a node is **selected** (the first KEY press selects the highlighted one, later presses move on) the board **pings it automatically every 10 s** until you pick another, change the preset or turn "Auto ping" off in SETUP. Pings also continue on other screens (except SCAN). A long press of KEY on MESH or NODE pings immediately.

### Meshtastic node ping (MESH screen)
Works with any Meshtastic node on the list, no second T190 needed.
1. Select a node (KEY), hold KEY: ping.
2. The T190 sends the node a packet requesting acknowledgement (want_ack), using its own random node ID (stored permanently, shown in the serial log), hop_limit 0 (direct test, no relaying) and the PRIVATE_APP port. The target does not display such a message but sends back an ACK.
3. The bottom panel shows ACK with RSSI, SNR, response time and the hop count of the reply, or NAK, or no ACK after 12 s. Sent and acknowledged counters are kept per node.
4. Limits: only works for a channel with the **default key** (AQ==) and a preset name matching "Mesh preset". With another key the node cannot read the packet and no ACK comes back. A ping is a transmission into the network (power and duty limit as in SETUP), so use it sparingly. RSSI and SNR describe the ACK heard here.

### Presets and regions
By default (fresh flash, no stored settings) the board runs **MediumFast** in the EU 868 band (869.525 MHz, SF9, BW250). Change it in SETUP or in the configurator; the choice is stored.
The preset list matches the current Meshtastic firmware for Europe: **EU_868** (LongFast, LongSlow, MediumSlow, MediumFast, ShortSlow, ShortFast, LongMod; 869.4–869.65 MHz), **EU_866** (LiteFast, LiteSlow; 865.6–867.6 MHz, 4 slots 600 kHz apart) and **EU_N_868** (NarrowFast, NarrowSlow; 62.5 kHz, 869.4–869.65 MHz). The channel frequency shown on MESH is computed as in the firmware: slot = hash(preset name) % slot count. VeryLongSlow and ShortTurbo are not available in EU regions.

### Bluetooth configurator
The Meshtastic app only talks to Meshtastic firmware, so the board exposes its own BLE service and you change settings with the page `tools/configurator.html` (Web Bluetooth).
1. On the board: SETUP → Bluetooth → KEY. The screen shows `ON  PIN 123456` (the PIN is fixed: 123456).
2. Open `tools/configurator.html` in Chrome or Edge (Mac, Windows, Linux, Android). Safari has no Web Bluetooth; on iPhone use the Bluefy browser. If the browser blocks a local file, run `python3 -m http.server` in `tools` and open `http://localhost:8000/configurator.html`.
3. "Connect to T190", pick the `T190-…` device, enter the PIN and "Authorize". Then type the node name and GPS position (coordinates by hand or from the phone), set the beacon and auto ping, change the same settings as in SETUP and press "Apply"; the board applies and stores them at once. The page also shows live status: last packet, noise, battery.
4. Without the PIN you can only read settings and status. Three wrong PINs disconnect. The PIN travels without BLE encryption, so this protects against accidental changes, not against a determined attacker. The PIN is fixed (123456). Turning Bluetooth off in SETUP stops advertising and blocks authorization (the BLE stack stays in memory until reboot).
5. The LoRa settings of the Meshtastic app (region, preset, channel, power, hop limit, etc.) belong to Meshtastic firmware. Here the equivalents are: the Meshtastic preset (listening, noise, ping) and the ping power.

### EU868 limits (enforced by the firmware)
- 869.4–869.65 MHz: 10 % duty cycle; 868.0–868.6 MHz: 1 % and max 14 dBm; other sub-bands 0.1 %.
- The firmware tracks airtime over the last hour and stops transmitting above the limit (HOME shows DUTY LIMIT). Power is clamped to the band limit.

### Noise
Noise is the instantaneous RSSI in receive mode, aggregated into one-second buckets (average, min, max, busy) for one hour. "Floor" is the 10th percentile of the minima in the window; "busy" is the share of readings above floor + 8 dB that are also louder than −110 dBm. The floor is the 25th percentile of one-second averages over the last 5 minutes; for the first 20 s of measurement busy is 0 because the floor is not known yet. Steady high noise is not "busy": its level shows in FLOOR and NOW. It is not sampled while transmitting or on SCAN.

## Controls (two buttons)
| | PRG (GPIO0) | KEY (GPIO21) |
|---|---|---|
| normal screens | short: next screen, long: previous | short: screen action (time window / scan pause / next node on MESH and NODE), long: clear (NOISE, SCAN) or ping (MESH, NODE) |
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
- Charging: instead of the percentage the BATTERY tile shows "CHARGING". No charge-status pin was found for the T190, so detection is a heuristic (voltage rises >=20 mV between 10 s reads, or >=4.18 V; cleared on a fall or after 60 s without a rise below 4.15 V). Not verified on hardware.
- The 170-pixel ST7789 offset (`CGRAM_OFFSET`): if the picture is shifted, change the rotation (Flip screen).
- Node names in MESH appear only for a channel with the default key, after a NODEINFO packet.

## Host tests
```
g++ -std=c++17 -fsanitize=address,undefined -Istubs -Iinclude test/units.cpp -o units && ./units
g++ -std=c++17 -fsanitize=address,undefined -Istubs -Iinclude test/sim.cpp -o sim && mkdir -p shots && ./sim && python3 tools/render.py shots
```
Configurator: `node test/configurator_test.js json_options.txt json_status.txt` (the JSON files are written by `sim`) and `node test/configurator_dom_test.js tools/configurator.html json_options.txt json_status.txt`.
`units` checks AES (FIPS-197; CTR matches Python's library), Meshtastic headers, reception quality rating, noise and duty limits. `sim` runs the whole firmware against a virtual SX1262, buttons and clock; `render.py` builds screen previews (substitute fonts).
