# HeishaMon OTA Firmware Upload & Webserver Behavior

## 1. Incident Summary & Root Cause Analysis

### What Happened
During an automated attempt to upload a newly compiled firmware binary (`HeishaMon_ESP32-Meisha-8062c6c5351f06878687afc9c0bbbed6.bin`, ~1.5 MB) via `curl` to `http://192.168.10.12/firmware`, the connection timed out after 60 seconds. Following this attempt, HTTP connections to the HeishaMon web server failed with:
```text
curl: (56) Recv failure: Connection reset by peer
```

### Technical Root Cause
1. **HTTP 1.1 `Expect: 100-continue` Stall**:
   - `curl` automatically injects the header `Expect: 100-continue` when issuing POST requests with payloads larger than 1 MB.
   - HeishaMon uses a minimalist, custom embedded web server (`src/common/webserver.cpp`) designed for microcontrollers. It does not implement support for the HTTP `Expect: 100-continue` negotiation mechanism.
   - Consequently, `curl` paused waiting for a `100 Continue` response before transmitting the multipart payload bytes, while HeishaMon waited for incoming request data. After 60 seconds, `curl` aborted the connection.

2. **Webserver Client Slot Starvation & OTA State**:
   - In `HeishaMon.ino`, receiving the initial URI `/firmware` immediately calls `Update.begin(...)` to initialize the flash partition update.
   - When `curl` abruptly closed the TCP connection mid-handshake, `Update.end()` was never called.
   - Furthermore, HeishaMon's web server manages connections using a fixed pool of 5 client descriptors (`#define WEBSERVER_MAX_CLIENTS 5` in `src/common/webserver.h`). The ungraceful TCP termination left the lwIP TCP protocol control blocks (PCBs) in an unresolved state without clean callback teardown.
   - When new HTTP requests arrive at port 80, the web server finds no available slot in `clients[WEBSERVER_MAX_CLIENTS]`, leaving incoming connections without a receive callback. lwIP immediately resets the connection (`TCP RST`).

---

## 2. Current Device Health & Operational Status

Verification via live MQTT query (`docker exec mosquitto mosquitto_sub`) confirms:
- **Heat Pump Core Serial Loop**: **Fully operational**. The ESP32 FreeRTOS serial task (`serialTXTask`), serial RX, and query cycles are running normally.
- **MQTT Telemetry**: **Publishing in real time**. Live topics (`panasonic_heat_pump/main/Heatpump_State`, `panasonic_heat_pump/main/Pump_Flow`, `panasonic_heat_pump/1wire/...`) continue updating Home Assistant without interruption.
- **Heating & Domestic Hot Water (DHW)**: **Completely unaffected**. The heat pump continues running its programmed schedules and heating curves.
- **Affected Subsystem**: Only the local HTTP daemon on TCP port 80 is non-responsive.

---

## 3. Reset Assessment

### Should You Reset the Device?
**Yes.** A physical reset or power cycle of the HeishaMon board is required to restore the HTTP web interface.

- **Why a manual reset is needed**:
  - The HTTP `/reboot` endpoint cannot be served because the web server client pool is wedged.
  - HeishaMon does not implement an MQTT-based reboot command topic.
- **Safety guarantee**:
  - Because `Update.end(true)` was never executed, the ESP32 OTA boot partition was never marked active.
  - Resetting the device will boot cleanly and safely back into the existing working firmware (`3.9-meisha-1.0`).
  - No settings, rules, or calibrations will be lost.

---

## 4. Recommendations for OTA Updates

### Option A: Using `curl` CLI
When uploading firmware via `curl`, **always** suppress the `Expect: 100-continue` header by adding `-H "Expect:"`:
```bash
curl -v -H "Expect:" \
  -F "md5=<MD5_CHECKSUM>" \
  -F "firmware=@<PATH_TO_BINARY>" \
  http://192.168.10.12/firmware
```

### Option B: Using the Web Browser UI
Navigate to `http://192.168.10.12/firmware` in a web browser. Standard browser form submissions do not attach `Expect: 100-continue` and will upload the file cleanly through HeishaMon's native web form handler.

---

## 5. Implemented Firmware & Webserver Safeguards

To prevent this issue from recurring, the following safeguards were implemented:

1. **Firmware Update Inactivity Timeout (`FIRMWARE_UPDATE_TIMEOUT = 30000`)**:
   - `lastFirmwareUpdateActivity` tracks the last time a firmware chunk was received or initialized.
   - If `Update.isRunning()` is active and 30 seconds elapse without receiving any data, `loop()` automatically calls `Update.end(false)` to release the update partition lock and log the timeout.

2. **Client Teardown Abort Handler (`WEBSERVER_CLIENT_CLOSE` for Route 150)**:
   - Added `case 150:` in `HeishaMon.ino` under `WEBSERVER_CLIENT_CLOSE`. If the remote client disconnects or times out mid-transfer, `Update.end(false)` is immediately called.
   - Fixed `webserver_client_close()` in `webserver.cpp` to set `client->step = WEBSERVER_CLIENT_CLOSE` before invoking callbacks, ensuring cleanup handlers run on unexpected disconnects.

3. **HTTP `Expect: 100-continue` Negotiation**:
   - In `webserver.cpp`, added detection for the `Expect: 100-continue` header during request parsing.
   - When the client's HTTP header ends, the web server immediately returns `HTTP/1.1 100 Continue\r\n\r\n`, prompting standard `curl` or automated upload tools to send payload data immediately without waiting for a 60s timeout.

4. **lwIP PCB Leak Protection**:
   - In `webserver_client_close()` and `webserver_reset_client()`, if `tcp_close()` returns an error (e.g., `ERR_MEM`), `tcp_abort()` is called to guarantee the connection control block is never leaked.

5. **Stale Session Takeover**:
   - If a new `/firmware` POST request arrives while `Update.isRunning()` is still true from a previously abandoned session, the server gracefully aborts the stale session with `Update.end(false)` and accepts the new upload instead of immediately rejecting it.
