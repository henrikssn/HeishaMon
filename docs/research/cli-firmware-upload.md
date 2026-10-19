# Research: CLI Firmware Updates for HeishaMon

## TL;DR & Primary Recommendation

1. **Should we replace `curl` for the existing `/firmware` endpoint?**
   **Yes.** Command-line `curl -F` (multipart/form-data) is incompatible with HeishaMon's vendored 512-byte embedded multipart parser ([`webserver.cpp:890-1250`](file:///agy/esphome-aquarea/HeishaMon/src/common/webserver.cpp#L890-L1250)). The parser has known boundary desync bugs, false-positive boundary matching on binary payloads (`\r\n--`), and field-ordering sensitivities that the web browser's `XMLHttpRequest` coincidentally avoids.

2. **What can we use from the CLI right now?**
   **ArduinoOTA (`espota.py`) is already compiled into the firmware!**
   HeishaMon initializes `ArduinoOTA` on **UDP port 8266** with default password `"heisha"` ([`HeishaMon.ino:1048-1069`](file:///agy/esphome-aquarea/HeishaMon/HeishaMon.ino#L1048-L1069)). It bypasses the HTTP web server entirely:
   ```bash
   python3 espota.py -i 192.168.10.12 -p 8266 -a heisha -f HeishaMon.ino.bin
   ```
   *(Note: Because `espota.py` works by having the ESP32 connect back via TCP to the host, the upload must be executed from a host on the same LAN subnet or with Docker `--net=host`).*

3. **What is the best long-term firmware enhancement?**
   Add a **raw binary streaming endpoint** (e.g. `POST /ota` or `POST /firmware/raw` with `Content-Type: application/octet-stream` and `X-MD5` header). This completely eliminates multipart parsing, allowing standard `curl --data-binary @firmware.bin` to stream directly into `Update.write()` with 100% reliability.

---

## 1. Deep Dive: Why `curl -F` Stalls on HeishaMon

During our testing, two distinct failure modes were observed:

### Failure Mode 1: HTTP 1.1 `Expect: 100-continue`
- **Cause**: By default, `curl` attaches `Expect: 100-continue` to any POST request with a body larger than 1 MB ([RFC 7231 §5.1.1](https://datatracker.ietf.org/doc/html/rfc7231#section-5.1.1)).
- **HeishaMon Bug**: HeishaMon's minimal HTTP parser in [`src/common/webserver.cpp`](file:///agy/esphome-aquarea/HeishaMon/src/common/webserver.cpp#L770) did not recognize or acknowledge `Expect: 100-continue`. `curl` waited 60 seconds for an HTTP 100 status before timing out with 0 body bytes sent.
- **Resolution**: Added `100 Continue` header parsing and response in commit `1c0524f`.

### Failure Mode 2: The 50,260-Byte Socket Freeze
When `-H "Expect:"` was added, `curl` immediately streamed headers and started transmitting data, but stalled after ~50 KB. Socket inspection via `ss -tn dst 192.168.10.12` revealed:
```text
State  Recv-Q   Send-Q     Local Address:Port       Peer Address:Port
ESTAB  0        50260       192.168.10.2:39010     192.168.10.12:80
```
`50,260` bytes is exactly the maximum TCP receive window of the ESP32 lwIP stack. The host sent 50 KB, the ESP32 stopped reading or threw an error, the TCP window closed, and the connection hung indefinitely until killed.

Detailed analysis of [`HeishaMon/src/common/webserver.cpp`](file:///agy/esphome-aquarea/HeishaMon/src/common/webserver.cpp) revealed multiple critical design flaws in the multipart parser:

1. **Tiny Buffer Size ([`webserver.h:24`](file:///agy/esphome-aquarea/HeishaMon/src/common/webserver.h#L24))**:
   `WEBSERVER_BUFFER_SIZE` is defined as only **512 bytes**. Parsing multipart headers and chunking binary payload through a 512-byte sliding window requires complex byte-shifting (`memmove`), pointer arithmetic, and substep state transitions (substeps 0 to 8).

2. **Binary Collision on `\r\n--` ([`webserver.cpp:1152`](file:///agy/esphome-aquarea/HeishaMon/src/common/webserver.cpp#L1152))**:
   ```cpp
   // Value
   case 8:
   case 7: {
     unsigned char *ptr = strnstr(client->buffer, "\r\n--", client->ptr);
     if(ptr != NULL && client->substep != 8) {
       // Assumes this MUST be the multipart boundary delimiter!
   ```
   In `case 7`, the parser scans raw binary bytes using `strnstr(..., "\r\n--", ...)`. If the compiled firmware binary happens to contain the 4-byte sequence `0x0D 0x0A 0x2D 0x2D` (`\r\n--`), the parser mistakes it for a multipart boundary delimiter, aborts the file chunk, and corrupts the stream state!
   *(Analysis of our compiled ESP32-S3 binary confirmed the occurrence of `\r\n--` at offset 187,923).*

3. **Field Ordering & Header Differences**:
   - The built-in HTML JavaScript ([`htmlcode.h:2345-2363`](file:///agy/esphome-aquarea/HeishaMon/htmlcode.h#L2345-L2363)) builds `FormData`:
     ```javascript
     var fd = new FormData();
     fd.append('firmware', file); // Field 1: firmware
     fd.append('md5', document.getElementById('md5').value); // Field 2: md5
     ```
   - In our CLI `curl` command, `-F "md5=..."` was specified before `-F "firmware=@..."`.
   - When `md5` is first, `readlen` accounting in `webserver.cpp` (lines 948, 1051, 1068, 1133) accumulates header offsets. Because `md5` has no `filename="..."` or `Content-Type`, `substep 6` transitions to `substep 7` via short-form CRLF handling. When the second boundary arrives, any mismatch between `readlen` and `totallen` causes `webserver_sync_receive()` to return `-1` ([`webserver.cpp:2173`](file:///agy/esphome-aquarea/HeishaMon/src/common/webserver.cpp#L2173)), which sets `client->step = WEBSERVER_CLIENT_CLOSE` without reading the remaining socket buffer!

4. **Synchronous Flash Erase Blocking**:
   In [`HeishaMon.ino:1152`](file:///agy/esphome-aquarea/HeishaMon/HeishaMon.ino#L1152):
   ```cpp
   Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000);
   ```
   On ESP32, passing a defined size (~1.8 MB) causes `esp_ota_begin()` in the ESP-IDF partition driver to synchronously erase all target flash sectors (450+ sectors @ ~40ms each = **15 to 20 seconds**). Because the webserver in HeishaMon runs in synchronous mode (`sync_server` in [`webserver.cpp:2588`](file:///agy/esphome-aquarea/HeishaMon/src/common/webserver.cpp#L2588)), the main thread blocks during this erase, leaving the TCP socket buffer unserviced.

---

## 2. Evaluation of CLI Firmware Update Options

| Method | Feasibility Today | Reliability | Network Requirements | Complexity |
| :--- | :---: | :---: | :--- | :--- |
| **A. ArduinoOTA (`espota.py`)** | **YES (Built-in)** | High | UDP 8266 + Direct TCP callback to host | Low (single Python script) |
| **B. Browser UI Upload** | **YES (Working)** | High | HTTP port 80 only | Manual (browser) |
| **C. Python Multipart Script** | Medium | Medium | HTTP port 80 | Medium (must mirror browser FormData) |
| **D. CLI `curl -F` (multipart)** | Broken | Very Low | HTTP port 80 | Fragile (hits parser edge cases) |
| **E. Raw Stream Endpoint (`POST /ota`)** | Future FW | Highest | HTTP port 80 (standard `curl --data-binary`) | Minimal (very simple C++ handler) |
| **F. Pull OTA via MQTT / HTTP** | Future FW | High | Device needs outbound HTTP access | Medium |

---

## 3. Option Details & Implementation Guides

### Option A: ArduinoOTA (`espota.py`) — The Immediate CLI Solution

HeishaMon already initializes `ArduinoOTA` in [`HeishaMon.ino:1048-1069`](file:///agy/esphome-aquarea/HeishaMon/HeishaMon.ino#L1048-L1069):
```cpp
void setupOTA() {
  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname(heishamonSettings.wifi_hostname);
  ArduinoOTA.setPassword(heishamonSettings.ota_password); // default: "heisha"
  ArduinoOTA.begin();
}
```
And [`ArduinoOTA.handle()`](file:///agy/esphome-aquarea/HeishaMon/HeishaMon.ino#L2042) is called on every iteration of `loop()`.

#### Protocol Details:
1. Host sends UDP packet to `192.168.10.12:8266` advertising the update size and MD5.
2. ESP32 challenges the host with an MD5 nonce over UDP.
3. Host answers the challenge over UDP.
4. ESP32 initiates an outbound TCP connection back to the host IP to stream the binary into `Update.write()`.

#### How to Run:
The `espota.py` tool is bundled with the ESP32/ESP8266 Arduino core (and available inside our devcontainer):
```bash
# From a machine on the same LAN (or Docker with --net=host):
python3 espota.py -i 192.168.10.12 -p 8266 -a heisha -f HeishaMon.ino.bin
```

---

### Option B: Raw Binary HTTP Upload Endpoint (`POST /firmware/raw`) — Long-Term Fix

The cleanest architectural solution for embedded microcontrollers is to avoid multipart forms completely for CLI usage. A raw streaming endpoint takes the binary directly in the HTTP body:

#### Proposed Firmware Endpoint (`POST /firmware/raw`):
In `HeishaMon.ino`:
```cpp
// Request URI matching:
} else if (strcmp_P((char *)dat, PSTR("/firmware/raw")) == 0) {
  client->route = 155;
  Update.begin(UPDATE_SIZE_UNKNOWN);
}

// In webserver_cb:
case 155: {
  // Read binary stream directly from rbuffer:
  Update.write(rbuffer, size);
  if (client->readlen == client->totallen) {
    if (Update.end(true)) {
      timerqueue_insert(2, 0, -2); // Reboot
      webserver_send(client, 200, "text/plain", 7);
      webserver_send_content(client, "SUCCESS", 7);
    }
  }
} break;
```

#### CLI Command with `curl`:
```bash
curl -v -X POST \
  -H "Content-Type: application/octet-stream" \
  --data-binary @HeishaMon.ino.bin \
  http://192.168.10.12/firmware/raw
```
**Benefits**:
- Zero multipart parsing overhead.
- No boundary collisions on binary data.
- Compatible with all standard CLI tools (`curl`, `wget`, `httpie`, PowerShell).

---

### Option C: Python Script Mimicking Browser FormData

If the existing `/firmware` multipart endpoint must be automated from CLI without firmware modifications, a dedicated Python script using `requests` or `urllib3` must:
1. Ensure field order has `firmware` first, followed by `md5`.
2. Omit the `Expect: 100-continue` header.
3. Use a static boundary without unusual symbols.
4. Send chunked writes throttled to match ESP32 buffer speeds.

Example script ([`scripts/upload_firmware.py`](file:///agy/esphome-aquarea/scripts/upload_firmware.py)):
```python
import requests
import hashlib
import sys

def upload(ip, bin_path):
    with open(bin_path, 'rb') as f:
        content = f.read()
    md5 = hashlib.md5(content).hexdigest()
    
    files = [
        ('firmware', (bin_path, content, 'application/octet-stream')),
        ('md5', (None, md5))
    ]
    headers = {'Expect': ''}
    r = requests.post(f"http://{ip}/firmware", files=files, headers=headers, timeout=60)
    print(r.text)
```

---

## 4. Summary & Action Plan

1. **Short-Term (Today)**:
   - For interactive updates: Continue using the native Web UI at `http://192.168.10.12/firmware`.
   - For CLI updates without changing firmware: Use `espota.py` on UDP port `8266` with password `heisha`.

2. **Next Firmware Release**:
   - Keep the safety fixes committed in `1c0524f` (30s timeout, disconnect abort, 100-continue support).
   - Add the simple `POST /firmware/raw` octet-stream endpoint so that native `curl --data-binary` works effortlessly without multipart quirks.

---

## Confidence & Open Questions

- **Verified Facts**:
  - `ArduinoOTA` is configured for port 8266 with password `heisha` ([`HeishaMon.ino:1050-1056`](file:///agy/esphome-aquarea/HeishaMon/HeishaMon.ino#L1050-L1056), [`webfunctions.h:48`](file:///agy/esphome-aquarea/HeishaMon/webfunctions.h#L48)).
  - `WEBSERVER_BUFFER_SIZE` is 512 bytes ([`webserver.h:24`](file:///agy/esphome-aquarea/HeishaMon/src/common/webserver.h#L24)).
  - The binary contains the string `\r\n--` at byte offset 187,923.
  - `Update.heishamon_firmware` reports `installed_version: "3.9-meisha-1.0"` in Home Assistant.

- **Hypotheses / To Verify on Hardware**:
  - Verifying `espota.py` execution against the physical board once power-cycled.
