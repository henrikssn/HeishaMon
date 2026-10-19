# HeishaMon Rules Engine vs. ESPHome for Local Panasonic Aquarea M-Series Control

> **Research Scope & Primary Sources**
> This document evaluates the **HeishaMon Rules Engine** (`heishamon/HeishaMon`) against **ESPHome** (`ElVit/esphome_components`, `IoTLabs-pl/ESPHome-Components`, and custom ESPHome C++/YAML architectures) for running autonomous, fault-tolerant local heating and DHW control on an ESP32-S3 (HeishaMon Large) connected to a **Panasonic Aquarea M-series** heat pump. Every technical claim is verified against primary source code and evaluated against the concrete control specification in [`/config/docs/meisha-heat-pump-control.md`](file:///config/docs/meisha-heat-pump-control.md).

---

## Executive Summary & Architectural Verdict

| Evaluation Dimension | HeishaMon Rules Engine (`heishamon/HeishaMon`) | ESPHome (`ElVit/esphome_components` / `IoTLabs-pl`) |
| :--- | :--- | :--- |
| **Autonomous Local Execution** | Runs on-device in a custom bytecode VM (`32 KB` PSRAM pool on ESP32-S3). | Runs natively on-device via compiled C++ state machines, `interval` components, and lambdas. |
| **Dynamic Setpoints from HA (No Rule Re-upload)** | **Not supported.** No MQTT command topic or HTTP endpoint can write to `#` rule globals. Only 2 OpenTherm registers (`?dhwSetpoint`, `?maxTSet`) are both MQTT-writable and rule-readable if OpenTherm is enabled. | **Native.** `number`, `switch`, `select` entities (or `homeassistant` sensors with local NVS-backed mirrors) update on-device variables in real time via the native ESPHome API or MQTT. |
| **External HA Sensors (e.g., `garage_dew_point`)** | **Not supported.** Cannot subscribe to arbitrary MQTT topics or HA entities (`HeishaMon.ino:491-500`). | **Native.** `platform: homeassistant` sensors import any HA entity directly into ESPHome with `.has_state()` and `isnan()` validity checks. |
| **State & Setpoint Persistence Across Reboots** | **None.** All `#` globals live solely in volatile RAM (`rules.cpp:79, 969-974`). Reboots wipe rolling averages, degree-hour integrals, PID state, and any dynamic setpoints. | **Native.** `globals` with `restore_value: true` and `number`/`switch`/`select` with `restore_value: true` persist state to ESP32 NVS flash across power cycles. |
| **Crash / Brownout Recovery** | **Hazardous default:** If `esp_reset_reason()` is in `4..11` (including `ESP_RST_BROWNOUT = 9` or watchdog resets) and `force_rules` is `false`, HeishaMon **refuses to load `/rules.txt` on boot** (`HeishaMon.ino:1925-1942`), leaving the heat pump uncontrolled. | **Deterministic:** Compiled into firmware; boots unconditionally with `restore_value: true` NVS state and safe fallback defaults. |
| **Network / HA Disconnect Detection** | **None.** Neither `WiFi.status()`, `ETH.connected()`, nor `mqtt_client.connected()` is exposed to the rules VM (`rules.cpp:121-246`). | **Native.** `api.connected` and `wifi.connected` / `mqtt.connected` conditions allow automatic fallback when Home Assistant is unreachable. |
| **Floating-Point Math & I-Only PID** | VM truncates all heap floats from 32-bit IEEE-754 to **27-bit floats** (18 explicit mantissa bits, ~5.7 decimal digits) via Veltkamp-Dekker splitting (`src/rules/rules.cpp:222-227`) and uses `EPSILON = 1e-6f` (`src/rules/rules.cpp:41`). | Full hardware 32-bit (`float`) or 64-bit (`double`) IEEE-754 arithmetic in C++ lambdas. |
| **HA Entity & History Preservation** | Native MQTT discovery (`panasonic_heat_pump/main/...`, `1wire/<addr>`). | Full history continuity achievable by matching entity IDs in ESPHome or publishing identical MQTT topics via ESPHome's `mqtt` component. |

### Bottom-Line Verdict
1. **HeishaMon Rules Engine cannot satisfy the [`meisha-heat-pump-control.md`](file:///config/docs/meisha-heat-pump-control.md) specification without major architectural compromises.** Specifically, HeishaMon Rules has **no mechanism to receive the 8 user-tunable HA helpers or the external `sensor.garage_dew_point` from Home Assistant** without regenerating and HTTP-POSTing `/rules.txt` on every change (which in turn wipes all `#` global variables in RAM, destroying the 24h/6h/1h rolling averages, the daily degree-hour integral, and the I-only PID accumulator). Furthermore, any ESP32 reboot or power brownout resets all `#` globals to `NULL` (or disables `/rules.txt` entirely on brownout/WDT reset if `force_rules` is off).
2. **ESPHome (using `ElVit/esphome_components` on the HeishaMon Large ESP32-S3 hardware) satisfies 100% of the requirements natively.** It runs the exact same 9600 8E1 CN-CNT (`0x10` + `0x21` extra) protocol and onboard 1-Wire (`GPIO3`) bus, exposes user-tunable `number`/`switch` entities with NVS flash persistence (`restore_value: true`), persists rolling-window ring buffers or EWMA state and `degree_hours_day` in NVS globals, executes wall-clock schedules via SNTP (`platform: sntp`), and can preserve existing Home Assistant long-term statistics and sensor history.

---

## 1. HeishaMon Rules Engine Capabilities & Limitations

### 1.1 Language Constructs, Variables, Operators, Functions, Timers & Triggers

The HeishaMon rules engine is a custom lexer, bytecode compiler, and virtual machine located in [`HeishaMon/src/rules/rules.cpp`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.cpp) and integrated into HeishaMon via [`HeishaMon/rules.cpp`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp). An official reference summary is also maintained in [`Examples/Rules/SKILL.md`](https://github.com/heishamon/HeishaMon/blob/main/Examples/Rules/SKILL.md).

#### Control Flow & Syntax Constraints
- **Event blocks (`on <trigger> then ... end`):** All logic lives inside event blocks ([`src/rules/rules.h:68-98`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.h#L68-L98)). Custom functions are simply named event blocks (`on myFunc($arg1, $arg2) then ... end`) invoked by name.
- **Conditionals (`if ... then ... elseif ... then ... else ... end`):** Only `if`/`elseif`/`else` branching is supported ([`src/rules/rules.h:70-74`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.h#L70-L74)).
- **No Loops or Arrays:** There are **no `for` or `while` loops** and **no arrays or structs** in the language. Any ring buffer (such as a 24-sample hourly window) would require 24 individual scalar global variables (`#buf0` .. `#buf23`) shifted manually via 24 assignment statements.

#### Variable Scopes & Sigils
Variable and event prefixes are parsed in [`HeishaMon/rules.cpp:121-364`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L121-L364):

| Prefix / Syntax | Scope & Storage | Read/Write | Event Trigger (`on ... then`)? | Source Citation |
| :--- | :--- | :--- | :--- | :--- |
| `$var` | **Local variable** (`obj->userdata` per rule). Freed immediately when the rule/timer finishes (`rules_free_stack()`). | R/W | No (used as formal parameters in custom functions) | [`HeishaMon/rules.cpp:240-241`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L240-L241), [`808-818`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L808-L818), [`902`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L902), [`1063`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L1063) |
| `#var` | **Global variable** (`static struct varstack_t global_varstack` in RAM). Initialized to `NULL` (`VNULL`) until assigned. | R/W | No | [`HeishaMon/rules.cpp:79`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L79), [`243-244`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L243-L244), [`517-579`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L517-L579) |
| `@<TopicName>` | **Heat pump telemetry** (`TOP0..TOP143` in `topics[]`, `OPT0..OPT6` in `optTopics[]`, `XTOP0..XTOP5` in `xtopics[]`). | Read-only | **Yes** — fires whenever a decoded frame changes that topic's value (`decode.cpp:365, 421`). | [`HeishaMon/rules.cpp:159-203`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L159-L203), [`251-298`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L251-L298), [`decode.cpp:365`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/decode.cpp#L365) |
| `@Set<Command>` | **Heat pump write command** (`commands[]` and `optionalCommands[]`, e.g., `@SetHeatpump`, `@SetZ1HeatRequestTemperature`, `@SetOperationMode`, `@SetCurves`). | Write-only (reads return `NULL`) | **Yes** (parsed in `is_event()`, though `commands.cpp` does not emit `rules_event_cb` on MQTT command receipt). | [`HeishaMon/rules.cpp:204-225`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L204-L225), [`299-321`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L299-L321), [`687-736`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L687-L736) |
| `%hour`, `%minute`, `%month`, `%day` | **System time globals** via C `localtime()` (requires NTP sync). `%hour` = `0..23`, `%minute` = `0..59`, `%day` = `1..7` (Sun=1..Sat=7). **Bug note:** [`rules.cpp:456`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L456) pushes `tm_struct->tm_mon` (`0..11`), contradicting [`README.md:65`](https://github.com/heishamon/HeishaMon/blob/main/README.md#L65) which documents `1..12`! | Read-only | **No** (`is_event()` rejects `%`; cannot write `on %hour then`). | [`HeishaMon/rules.cpp:144-157`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L144-L157), [`446-461`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L446-L461) |
| `ds18b20#<16-hex>` | **1-Wire DS18B20 temperature** by 64-bit ROM address (e.g., `ds18b20#2800000000000000`). | Read-only | **Yes** — fires in `dallas.cpp:177` when sensor value changes or on `updateAllDallasTime`. | [`HeishaMon/rules.cpp:124-125`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L124-L125), [`340-342`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L340-L342), [`462-471`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L462-L471), [`dallas.cpp:177`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/dallas.cpp#L177) |
| `s0#watt_1`, etc. | **S0 pulse meter** values (`watt_1/2`, `watthour_1/2`, `watthourtotal_1/2`). | Read-only | **Yes** — fires in `s0.cpp` when pulse interval updates. | [`HeishaMon/rules.cpp:126-138`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L126-L138), [`344-350`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L344-L350), [`472-495`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L472-L495) |
| `?<OTParam>` | **OpenTherm bridge parameters** (`heishaOTDataStruct[]` in `HeishaOT.cpp`). | R/W depends on `rw` flag | **Yes** | [`HeishaMon/rules.cpp:226-238`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L226-L238), [`422-444`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L422-L444), [`603-642`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L603-L642) |
| `System#Boot` | **Single-shot boot trigger** fired in `HeishaMon.ino:2204` (`rules_boot()`) after initial heat pump query (`waitForFirstQuery`). | Event only | **Yes** (`on System#Boot then`) | [`HeishaMon/rules.cpp:336-338`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L336-L338), [`898-905`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L898-L905), [`HeishaMon.ino:2202-2205`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L2202-L2205) |
| `timer=<N>` | **One-shot timer trigger** armed via `setTimer(<N>, <seconds>)`. | Event only | **Yes** (`on timer=1 then`) | [`HeishaMon/rules.cpp:352-354`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L352-L354), [`1026-1091`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L1026-L1091) |

#### Operators & Built-in Functions
- **Operators ([`src/rules/operator.cpp:27-42`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/operator.cpp#L27-L42)):**
  - Logical: `&&` (precedence 1), `||` (precedence 1)
  - Relational: `==`, `!=` (precedence 2); `>=`, `<=`, `<`, `>` (precedence 3)
  - Additive: `+`, `-` (precedence 4)
  - Multiplicative: `*`, `/`, `%` (`fmodf`, precedence 5)
  - Exponential: `^` (`powf`, precedence 6, right-associative)
- **Built-in Functions ([`src/rules/functions/`](https://github.com/heishamon/HeishaMon/tree/main/HeishaMon/src/rules/functions)):**
  - Math: `round(x)`, `floor(x)`, `ceil(x)`, `min(a, b, ...)`, `max(a, b, ...)`
  - Null-handling: `isset(x)` (returns `1` if non-`NULL`, `0` if `NULL`), `coalesce(a, b, ...)` (returns first non-`NULL` argument)
  - Strings & Logging: `concat(a, b, ...)`, `print(a, b, ...)` (writes to MQTT `<base>/log` and WebSocket log)
  - Hardware & Timers: `gpio(pin)` / `gpio(pin, state)`, `setTimer(nr, seconds)` (`seconds = 0` cancels timer `nr`, see [`src/rules/functions/settimer.cpp`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/functions/settimer.cpp) and [`HeishaMon/rules.cpp:1004-1024`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L1004-L1024)).

#### Critical VM Quirks & Precision Hazards
1. **27-bit Float Truncation (`float32to27`):**
   To pack a 5-bit type tag (`VFLOAT = 3`) inside a 32-bit aligned VM memory slot (`struct vm_vfloat_t` in [`src/rules/rules.h:132-135`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.h#L132-L135)), the VM strips the lowest 5 mantissa bits from every floating-point literal and intermediate calculation using the Veltkamp-Dekker splitting algorithm ([`src/rules/rules.cpp:222-227`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.cpp#L222-L227), called at lines `1526`, `1784`, and `4646`):
   ```cpp
   // HeishaMon/src/rules/rules.cpp:221-227
   static float float32to27(float f) {
     uint8_t bits = 5; // remove 8 bits [sic: zeroes lowest 5 mantissa bits]
     float factor = (float)((1u << bits) + 1u);
     float c = factor * f;
     return (c-(c-f));
   }
   ```
   This reduces IEEE-754 single precision from 24 significant bits (~7.2 decimal digits) to **19 significant bits** (~5.7 decimal digits), and relational operators compare floats using `EPSILON = 0.000001f` ([`src/rules/rules.cpp:41`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.cpp#L41)).
2. **Documented Engine Bugs ([`Examples/Rules/SKILL.md:39-47`](https://github.com/heishamon/HeishaMon/blob/main/Examples/Rules/SKILL.md#L39-L47)):**
   - **`NULL` in `&&` chains evaluates truthy:** `if @X == 0 && 1 == 1 then` executes its body when `@X` is `NULL` (`VNULL`). Every topic or uninitialized `#` variable must be wrapped in `coalesce(@X, 0)` or guarded by `isset(@X)`.
   - **Dropped `@Set...` before built-in call:** Inside a custom function block with `$` parameters, placing a top-level `@Set...` write immediately before a built-in function call like `setTimer(...)` causes the VM to execute them out of order and **silently drop the `@Set...` command** (`SKILL.md:40`).

---

### 1.2 Dynamic Setpoint Injection from Home Assistant (Without Re-uploading `/rules.txt`)

**Finding: HeishaMon Rules has no native mechanism to receive custom setpoints, parameters, or external sensor values from Home Assistant via MQTT or HTTP.**

#### Exhaustive Audit of HeishaMon MQTT Subscriptions & HTTP Endpoints
In [`HeishaMon/HeishaMon.ino:491-500`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L491-L500) and `mqtt_callback()` ([`HeishaMon.ino:971-1040`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L971-L1040)), HeishaMon subscribes exclusively to:
1. `<mqtt_topic_base>/commands/#` (`mqtt_topic_commands`): Dispatches directly to `send_heatpump_command()` ([`commands.cpp:1288-1325`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/commands.cpp#L1288-L1325)), which formats a 110-byte UART command and pushes it to `cmdqueue` for the heat pump. **It never calls `rules_event_cb()` and cannot write to `#` rule variables.**
2. `<mqtt_topic_base>/gpio/#`: Writes directly to digital GPIO pins (`gpio.cpp`).
3. `<mqtt_topic_base>/SendRawValue`: Sends raw hex bytes over UART to the heat pump.
4. `<mqtt_topic_base>/s0/WatthourTotal/1` & `/2`: Restores total S0 watt-hours once at boot, then **immediately unsubscribes** ([`HeishaMon.ino:1007-1012`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1007-L1012)).
5. `<mqtt_topic_base>/1wire/+`: Subscribed only during the first 3 seconds after boot (`dallasTimer <= 3`) to pre-populate `actDallasData` for already-discovered local 1-Wire ROM addresses, then **immediately unsubscribes** ([`HeishaMon.ino:2052-2057`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L2052-L2057)).
6. `<mqtt_topic_base>/opentherm/read/#`: Subscribed **only if `heishamonSettings.opentherm` is enabled** ([`HeishaMon.ino:491-494`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L491-L494)).

Similarly, the HTTP web server routes in [`HeishaMon.ino:1399-1511`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1399-L1511) (`/command`, `/ saverules`, `/settings`, etc.) contain no endpoint to read or mutate `#` rule variables.

#### Why the OpenTherm (`?`) Backdoor Workaround Fails for `meisha-heat-pump-control.md`
One might wonder whether enabling `heishamonSettings.opentherm` without an OpenTherm thermostat attached could allow Home Assistant to pass floats via `<base>/opentherm/read/<param>` into `?<param>` rule variables. Inspection of [`HeishaMon/HeishaOT.cpp:17-47`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaOT.cpp#L17-L47), [`436-556`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaOT.cpp#L436-L556), and [`HeishaMon/rules.cpp:422-444`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L422-L444) proves this cannot scale:
- `mqttOTCallback()` ([`HeishaOT.cpp:447`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaOT.cpp#L447)) only accepts writes to entries in `heishaOTDataStruct[]` where `rw == 1` or `rw == 2`.
- However, `vm_value_get()` in [`HeishaMon/rules.cpp:431`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L431) **only allows rules to read entries where `rw >= 2`**:
  ```cpp
  // HeishaMon/rules.cpp:431
  if (heishaOTDataStruct[x].rw >= 2 && heishaOTDataStruct[x].type == TFLOAT) {
    vm_push_float(heishaOTDataStruct[x].value.f);
  ```
- In `heishaOTDataStruct[]` ([`HeishaOT.cpp:17-47`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaOT.cpp#L17-L47)), **only two float entries have `rw == 2`**:
  1. `?dhwSetpoint`
  2. `?maxTSet` (which `mqttOTCallback` at [`HeishaOT.cpp:486-490`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaOT.cpp#L486-L490) hard-clamps to `20.0 <= val <= 65.0`!).
- All other MQTT-writable OpenTherm floats (`outsideTemp`, `inletTemp`, `outletTemp`, `dhwTemp`, `chPressure`, etc.) have `rw == 1` and return `NULL` if read by a rule!

#### Why Abusing Unused Heat Pump `@TOP` Registers or Re-uploading `/rules.txt` Is Unacceptable
- **Abusing Unused Heat Pump Settings (e.g., Z2 curve bytes `TOP34..TOP37`):** Having HA write `SetCurves` to store arbitrary numbers inside the heat pump's unused Zone 2 parameters writes directly to the Panasonic mainboard's non-volatile EEPROM on every adjustment, risking **EEPROM wear-out** on the heat pump PCB, and is limited to integer temperature ranges validated by the heat pump firmware.
- **Re-uploading `/rules.txt` via HTTP POST `/saverules`:** Whenever `/saverules` is called ([`HeishaMon.ino:1495-1505`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1495-L1505) -> `rules_parse("/rules.txt")` at [`rules.cpp:922-1002`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L922-L1002)), lines `969-974` execute:
  ```cpp
  // HeishaMon/rules.cpp:969-974
  for (int i = 0; i < global_varstack.nrbytes; i++) {
    free(global_varstack.array[i].key);
  }
  free(global_varstack.array);
  global_varstack.array = NULL;
  global_varstack.nrbytes = 0;
  ```
  **Every `/saverules` upload destroys all `#` global variables in RAM**, wiping out the 24h ambient rolling average, 6h room rolling average, 1h ambient rolling average, daily degree-hour accumulation, PID state, and defrost cooldown timer.

---

### 1.3 Persistence Across Reboots

**Finding: `#` global variables in HeishaMon Rules do not persist across reboots.**

- In [`HeishaMon/rules.cpp:79`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L79), `#` variables are stored in `static struct varstack_t global_varstack`, allocated in volatile heap via `realloc()` ([`rules.cpp:755`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L755)).
- Commented-out helper stubs `readRuleFromFS()` and `writeRuleToFS()` exist at [`HeishaMon/rules.cpp:88-120`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L88-L120), confirming filesystem persistence for rules state was never implemented.
- As documented in [`Examples/Rules/SKILL.md:19`](https://github.com/heishamon/HeishaMon/blob/main/Examples/Rules/SKILL.md#L19):
  > *"All `#` globals start as `NULL` after boot or ruleset reload, and `@` topics are `NULL` until the first heat-pump frame populates them."*

---

### 1.4 Network / Wi-Fi / MQTT Disconnect Detection

**Finding: Rules cannot detect whether Wi-Fi, Ethernet, MQTT, or Home Assistant is connected or disconnected.**

- `is_variable()` in [`HeishaMon/rules.cpp:121-246`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L121-L246) only recognizes `$`, `#`, `@`, `%`, `?`, `ds18b20#`, and `s0#`.
- Neither `WiFi.status()`, `ETH.connected()`, nor `mqtt_client.connected()` is exposed as a variable, event, or built-in function.
- Consequently, a hybrid architecture ("let HA control the heat pump when online, and only run HeishaMon Rules when HA/MQTT disconnects") is **impossible** in HeishaMon Rules without external heartbeat workarounds (and since HA cannot write to `#` globals or timers over MQTT, HA cannot even send a non-destructive heartbeat to HeishaMon Rules!).

---

### 1.5 ESP32-S3 & M-Series Specific Behavior in HeishaMon

1. **PSRAM Rule Pool Allocation (`ESP32-S3`):**
   - On ESP32 (`#if defined(ESP32)` in [`HeishaMon/rules.cpp:913-920`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L913-L920)), `rules_setup()` allocates `MEMPOOL_SIZE` (`32768` bytes, defined in [`src/rules/rules.h:51`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.h#L51)) in PSRAM via `ps_malloc(MEMPOOL_SIZE)`.
   - All compiled rule bytecode and active VM heap values (`$`, literals, temporary stack frames) share this 32 KB pool, while `#` globals live in standard heap (`global_varstack`).
2. **Crash & Brownout Rule-Disable Hazard (`HeishaMon.ino:1924-1942`):**
   - On boot, HeishaMon checks `esp_reset_reason()` ([`HeishaMon.ino:1925-1931`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1925-L1931)):
     ```cpp
     // HeishaMon/HeishaMon.ino:1925-1942
     #if defined(ESP32)
     const int reset_reason = esp_reset_reason();
     if (reset_reason < 4 || reset_reason > 11 || heishamonSettings.force_rules) {
     #endif
       if (LittleFS.begin()) {
         if (LittleFS.exists("/rules.txt")) {
           rules_parse("/rules.txt");
         }
       }
     } else {
        log_message(_F("Not loading rules due to crash reboot!"));
     }
     ```
   - In ESP-IDF (`esp_system.h`), reset reasons `4..11` include `ESP_RST_PANIC (4)`, `ESP_RST_INT_WDT (5)`, `ESP_RST_TASK_WDT (6)`, `ESP_RST_WDT (7)`, `ESP_RST_DEEPSLEEP (8)`, **`ESP_RST_BROWNOUT (9)`**, and `ESP_RST_SDIO (10)`.
   - Unless `heishamonSettings.force_rules` (`force_rules` checkbox in settings) is explicitly enabled, **a power flicker/brownout (`reset_reason == 9`) or transient watchdog timeout permanently skips loading `/rules.txt` on reboot**, leaving the heat pump completely uncontrolled until manual intervention!
3. **M-Series Telemetry (`XTOP0..XTOP5`):**
   - For K/L/M-series heat pumps, HeishaMon detects `actData[0xc7] >= 3` (`byte[199] >= 3`) in the standard `0x10` response ([`HeishaMon.ino:768`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L768)) and polls the extra `0x21` packet (`panasonicQuery` with byte 3 set to `0x21`), decoding `XTOP0..XTOP5` (`Heat_Power_Consumption`, `Cool_Power_Consumption`, `DHW_Power_Consumption`, `Heat_Power_Production`, `Cool_Power_Production`, `DHW_Power_Production`) in [`decode.cpp:378-426`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/decode.cpp#L378-L426). All 6 `XTOP` topics are accessible in rules via `@Heat_Power_Consumption`, etc. ([`rules.cpp:193-202`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L193-L202)).

---

## 2. ESPHome for Panasonic Aquarea (M-Series & HeishaMon Large)

### 2.1 Open-Source ESPHome CN-CNT Components Comparison

Two mature open-source ESPHome external components implement the Panasonic Aquarea CN-CNT serial protocol:

| Feature / Aspect | [`ElVit/esphome_components`](https://github.com/ElVit/esphome_components/tree/main/components/panasonic_heatpump) (`panasonic_heatpump`) | [`IoTLabs-pl/ESPHome-Components`](https://github.com/IoTLabs-pl/ESPHome-Components/tree/master/components/panasonic_aquarea) (`panasonic_aquarea`) |
| :--- | :--- | :--- |
| **Repository & Status** | Active successor to `ElVit/esphome_panasonic_heatpump`. Supports modern ESPHome & ESP-IDF (`esp-idf` framework with FreeRTOS UART tasks on ESP32). | Active component maintained by IoTLabs-pl (used in PanaProxy hardware). |
| **HeishaMon Large (ESP32-S3) Reference Config** | **Yes** — includes [`example_heishamon_large.yaml`](https://github.com/ElVit/esphome_components/blob/main/prototypes/panasonic_heatpump/example_heishamon_large.yaml) with exact GPIO pin mappings (`GPIO5` level shifter, `GPIO17/18` UART, `GPIO8/9` CZ-TAW1 proxy). | Configured primarily for PanaProxy pins (`GPIO16/17`, `GPIO25/26`), though pins are user-configurable in YAML. |
| **M-Series Extra Packet (`0x21` / `XTOP0..5`)** | **Yes** — automatically enables `send_extra_request_` when `tuple[199] > 0x02` ([`panasonic_heatpump.cpp:321-323`](https://github.com/ElVit/esphome_components/blob/main/components/panasonic_heatpump/panasonic_heatpump.cpp#L321-L323)) and decodes `XTOP0..XTOP5` ([`decode.cpp:477-498`](https://github.com/ElVit/esphome_components/blob/main/components/panasonic_heatpump/decode.cpp#L477-L498)). | **Yes** — polls `0x21` extra query when `data[199] >= 3` (`EXTRA_SUPPORT_BYTE_INDEX = 199` in [`protocol.cpp:11, 232-238`](https://github.com/IoTLabs-pl/ESPHome-Components/blob/master/components/panasonic_aquarea/protocol.cpp#L232-L238)) and decodes `XTOP0..XTOP5`. |
| **Standard Telemetry & Commands** | `TOP0..TOP143` sensors/binary_sensors/text_sensors and `SET1..SET48` selects/numbers/switches/climates/water_heaters ([`README.md`](https://github.com/ElVit/esphome_components/blob/main/components/panasonic_heatpump/README.md)). | `TOP0..TOP141` (`XTOP0..5`) and `SET1..SET39` ([`README.md`](https://github.com/IoTLabs-pl/ESPHome-Components/blob/master/components/panasonic_aquarea/README.md)). |
| **CZ-TAW1 / Cloud Adapter Proxy** | **Yes** (`uart_client_id`), allowing parallel operation with CZ-TAW1/CZ-TAW1B. | **Yes** (`external_controller_uart_id`), supporting listen-only, standalone, or proxy mode. |
| **Optional PCB (`0xF1 0x11 0x01 0x50`) Emulation** | Not polled in `panasonic_heatpump.cpp` loop (though byte encoding helpers `setDemandControl` / `setByte6` exist in [`commands.cpp:307-363`](https://github.com/ElVit/esphome_components/blob/main/components/panasonic_heatpump/commands.cpp#L307-L363)). Not needed unless emulating an Optional PCB (`OPT0..6`). | Not implemented (`panasonic_aquarea` focuses on mainboard `0x10`/`0x21` packets). |
| **Raw Byte Access in ESPHome Lambdas** | **Yes** — `id(my_heatpump).get_response_byte(index)` exposes any byte of the 203-byte response directly to YAML lambdas. | Requires modifying C++ component to expose additional internal arrays. |

**Recommendation:** **`ElVit/esphome_components` (`panasonic_heatpump`)** is the strongest foundation for HeishaMon Large on an M-series Aquarea: it provides an official HeishaMon Large hardware YAML, native ESP-IDF FreeRTOS UART handling on ESP32-S3, full `TOP0..143` + `XTOP0..5` + `SET1..48` coverage, and direct C++ method calls (`set_command_byte()`, `set_command_curve()`, etc.) usable from local ESPHome automation lambdas.

---

### 2.2 Panasonic CN-CNT Protocol Complexity & Anatomy

The Panasonic CN-CNT protocol is straightforward and fully documented in [`HeishaMon/commands.cpp`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/commands.cpp), [`HeishaMon/decode.cpp`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/decode.cpp), and [`ElVit/esphome_components/components/panasonic_heatpump/`](https://github.com/ElVit/esphome_components/tree/main/components/panasonic_heatpump):

1. **Physical & UART Layer:**
   - **5V TTL UART** at **9600 baud, 8 data bits, Even parity, 1 stop bit (`9600, 8E1`)** ([`HeishaMon.ino:223`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L223), [`ElVit_example_heishamon_large.yaml:36-42`](https://github.com/ElVit/esphome_components/blob/main/prototypes/panasonic_heatpump/example_heishamon_large.yaml#L36-L42)).
2. **Framing & Checksum ([`HeishaMon.ino:649-656`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L649-L656), [`ElVit_commands.cpp:17-25`](https://github.com/ElVit/esphome_components/blob/main/components/panasonic_heatpump/commands.cpp#L17-L25)):**
   - Every outgoing frame (query or command) is **110 payload bytes + 1 checksum byte = 111 bytes**:
     - Byte `0`: `0x71` (read poll) or `0xF1` (write command)
     - Byte `1`: `0x6C` (`108` = remaining payload length after byte `1`)
     - Byte `2`: `0x01` (source address)
     - Byte `3`: `0x10` (standard packet) or `0x21` (extra power packet for K/L/M series)
     - Bytes `4..109`: `0x00` for polls; modified byte(s) for commands (template in `panasonicSendQuery`, [`commands.cpp:12-23`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/commands.cpp#L12-L23))
     - Byte `110` (Checksum): Two's complement of the sum of bytes `0..109`: `chk = ((sum & 0xFF) ^ 0xFF) + 1` (so the sum of all 111 bytes `== 0x00 mod 256`).
   - Every incoming response from the heat pump is **203 bytes**:
     - Byte `0`: `0x71`
     - Byte `1`: `0xC8` (`200` = remaining payload bytes before checksum)
     - Byte `2`: `0x01`
     - Byte `3`: `0x10` (standard telemetry: `TOP0..TOP143`) or `0x21` (extra telemetry: `XTOP0..XTOP5` 16-bit unsigned watts at bytes `14..25`)
     - Byte `202`: Checksum (`sum of bytes 0..202 == 0x00 mod 256`).
3. **Key Command Offsets Used by `meisha-heat-pump-control.md`:**
   - `SET1` (`SetHeatpump` / Power On-Off): Byte `4` (`1` = Off, `2` = On; [`commands.cpp:251-265`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/commands.cpp#L251-L265))
   - `SET2` (`SetPump` / Circulation Pump): Byte `4` bits `4..5` (`16` = Off, `32` = On; [`commands.cpp:267-281`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/commands.cpp#L267-L281))
   - `SET3` (`SetQuietMode` / `0..3`): Byte `7` (`(val + 1) * 8`; [`commands.cpp:323-339`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/commands.cpp#L323-L339))
   - `SET4` (`SetZ1HeatRequestTemperature` / Shift or Direct Temp): Byte `38` (`val + 128`; [`commands.cpp:341-357`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/commands.cpp#L341-L357))
   - `SET5` (`SetZ1CoolRequestTemperature`): Byte `39` (`val + 128`; [`commands.cpp:359-375`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/commands.cpp#L359-L375))
   - `SET9` (`SetOperationMode` / `0..6`): Byte `6` ([`commands.cpp:446-484`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/commands.cpp#L446-L484))
   - `SET11` (`SetDHWTemp`): Byte `42` (`val + 128`; [`commands.cpp:504-520`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/commands.cpp#L504-L520))
   - `SET16` (`SetCurves` / Z1 & Z2 heating/cooling curve bytes): Bytes `75..82` + `86..89` ([`commands.cpp:522-562`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/commands.cpp#L522-L562); in `ElVit`, exposed directly as individual `number` entities `SET16_1..SET16_8`: `z1_heat_curve_target_high_temp` at byte `75`, `z1_heat_curve_target_low_temp` at byte `76`, `z1_heat_curve_outside_high_temp` at byte `78`, `z1_heat_curve_outside_low_temp` at byte `77`).
   - `SET17` (`SetZones` / `0=Zone1, 1=Zone2, 2=Zone1+2`): Byte `6` bits `6..7` (`64`, `128`, `192`; [`commands.cpp:564-580`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/commands.cpp#L564-L580)).

---

### 2.3 HeishaMon Large (ESP32-S3-MINI-1-N4R2) Hardware Pinout

Verified directly against [`HeishaMon/HeishaMon.ino:11-27, 168-193`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L11-L27), [`HeishaMon/dallas.h:20-22`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/dallas.h#L20-L22), [`HeishaMon/s0.h:16-19`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/s0.h#L16-L19), [`HeishaMon/gpio.h:20-23`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/gpio.h#L20-L23), and [`ElVit_example_heishamon_large.yaml`](https://github.com/ElVit/esphome_components/blob/main/prototypes/panasonic_heatpump/example_heishamon_large.yaml):

| Peripheral / Function | ESP32-S3 GPIO | Direction / Notes | Source Citation |
| :--- | :--- | :--- | :--- |
| **Heat Pump UART RX** (`Serial1`) | `GPIO18` | Input (via onboard 5V↔3.3V level shifter from CN-CNT TX) | [`HeishaMon.ino:186`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L186), [`example_heishamon_large.yaml:40`](https://github.com/ElVit/esphome_components/blob/main/prototypes/panasonic_heatpump/example_heishamon_large.yaml#L40) |
| **Heat Pump UART TX** (`Serial1`) | `GPIO17` | Output (via onboard level shifter to CN-CNT RX) | [`HeishaMon.ino:186`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L186), [`example_heishamon_large.yaml:39`](https://github.com/ElVit/esphome_components/blob/main/prototypes/panasonic_heatpump/example_heishamon_large.yaml#L39) |
| **UART Level Shifter Enable (TX Gate)** | `GPIO5` | Output: **must be driven `HIGH`** to enable transmitting to heat pump / proxy (`LOW` = listen-only) | [`HeishaMon.ino:175, 193, 211`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L175), [`example_heishamon_large.yaml:22-27`](https://github.com/ElVit/esphome_components/blob/main/prototypes/panasonic_heatpump/example_heishamon_large.yaml#L22-L27) |
| **Proxy / CZ-TAW1 UART RX** (`Serial2`) | `GPIO9` | Input (from CZ-TAW1 TX) | [`HeishaMon.ino:238`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L238), [`example_heishamon_large.yaml:48`](https://github.com/ElVit/esphome_components/blob/main/prototypes/panasonic_heatpump/example_heishamon_large.yaml#L48) |
| **Proxy / CZ-TAW1 UART TX** (`Serial2`) | `GPIO8` | Output (to CZ-TAW1 RX) | [`HeishaMon.ino:238`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L238), [`example_heishamon_large.yaml:47`](https://github.com/ElVit/esphome_components/blob/main/prototypes/panasonic_heatpump/example_heishamon_large.yaml#L47) |
| **1-Wire Bus (DS18B20)** | `GPIO3` | Bidirectional with onboard 4.7 kΩ pull-up (ESPHome `one_wire:` / `dallas_temp:`) | [`HeishaMon/dallas.h:21`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/dallas.h#L21) (`ONE_WIRE_BUS = 3` on `ESP32`) |
| **Onboard Relay 1 & Relay 2** | `GPIO21`, `GPIO47` | Outputs | [`HeishaMon/gpio.h:22`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/gpio.h#L22) |
| **S0 Pulse Inputs 1 & 2** | `GPIO1`, `GPIO2` | Inputs (internal pull-up) | [`HeishaMon/s0.h:16-19`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/s0.h#L16-L19) |
| **OpenTherm Interface** | `GPIO4` (EN), `GPIO6` (IN), `GPIO7` (OUT) | OpenTherm daughterboard interface | [`HeishaMon.ino:170`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L170), [`HeishaOT.h:12-13`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaOT.h#L12-L13) |
| **Addressable RGB LED (WS2812)** | `GPIO42` | Status NeoPixel (`esp32_rmt_led_strip`) | [`HeishaMon.ino:173`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L173) |
| **W5500 Ethernet (Optional SPI)** | `GPIO10` (CS), `GPIO11` (MOSI), `GPIO12` (CLK), `GPIO13` (MISO), `GPIO14` (RST), `GPIO15` (IRQ) | Optional W5500 Ethernet module | [`HeishaMon.ino:17-22`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L17-L22) |
| **USB Native CDC/JTAG** | `GPIO19` (D-), `GPIO20` (D+) | USB-C programming & serial logs | ESP32-S3 hardware fixed |

---

## 3. Requirement-by-Requirement Evaluation Against `meisha-heat-pump-control.md`

Below is a direct mapping of every control-critical requirement in [`/config/docs/meisha-heat-pump-control.md`](file:///config/docs/meisha-heat-pump-control.md) against **HeishaMon Rules Engine** and **ESPHome**.

### 3.1 Summary Matrix

| Spec Section in `meisha-heat-pump-control.md` | HeishaMon Rules Engine | ESPHome (`ElVit/esphome_components` + Lambdas) |
| :--- | :--- | :--- |
| **§2.2: 8 User-Tunable Helpers** (`meisha_room_target`, `curve_start`, `curve_angle`, `fixed_target`, `start_hour`, `max_outlet_temp`, `cooling_mode`, `high_power_override`) | **FAIL.** Cannot receive MQTT/HA helper values without re-uploading `/rules.txt` (which wipes all `#` globals in RAM). | **PASS.** Exposed as native ESPHome `number:` and `switch:` components with `restore_value: true` (persisted in ESP32 NVS flash, editable from HA UI). |
| **§3.1–3.2: Rolling Time-Window Averages & Curve** (`ambient_24h`, `ambient_1h`, `room_temp_6h` 09:00 snapshot) | **POOR / FRAGILE.** No arrays or loops; requires EWMA approximations or manual scalar globals; **100% wiped on any reboot or `/saverules` edit**. | **PASS.** Implemented as C++ ring buffers (e.g., `float[96]` for 24h at 15-min intervals, `float[72]` for 6h at 5-min intervals, `float[60]` for 1h at 1-min intervals) persisted in NVS `globals` (`restore_value: true`) and seeded on first boot. |
| **§3.3: Daily Degree-Hour Budget** (`room_delta = outlet - room_target` when `pump_flow > 0` & valve=`Room`; `degree_hours_day` reset at `09:00`; `DHT = 24 × (curve_target - room_target)`) | **FRAGILE.** Can integrate `#dh_day += (#outlet - #room_target) * dt` on timer and reset when `%hour == 9 && %minute == 0`, but **wipes to `NULL` on any midday reboot**, causing full double-heating for the day. | **PASS.** Trapezoidal/step accumulator in an NVS-backed `globals:` float (`restore_value: true`), reset at `09:00:00` via `time: on_time:`, surviving midday reboots intact. |
| **§3.4: Power Target & I-Only PID** (`curve_error = curve_target - outlet` when flow>0 & valve=`Room`; `Ki = 5e-5`, 10s sample, clamped `-1..+1`; `power_target = max(27, curve_target - pid)` with ±0.1 hyst) | **FRAGILE.** Expressible via `#pid_i`, but subject to 27-bit float mantissa truncation (`float32to27`), `EPSILON = 1e-6f`, and resets to `0` on any reboot or rules save. | **PASS.** Native 32-bit/64-bit IEEE-754 float math in a 10s `interval:` lambda, with `restore_value: true` NVS persistence of the integral accumulator (“last known value” start mode). |
| **§3.5: Time-Based Helpers** (`dhw_time` hours valve=`Tank` since `09:00`, `hours_target`, `hours_time_left`) | **FRAGILE.** Can accumulate `#dhw_time` while `@ThreeWay_Valve_State == 1`, but resets to `0` on any reboot. | **PASS.** Accumulated in NVS-backed `globals:` float (`restore_value: true`) and reset at `09:00`. |
| **§4.1–4.6: Heating Automations & Change-Lock** (`09:00` Day mode #1, `bonusgrad curve` +30m lock, `malusgrad` +20m lock, `bonusgrad high-temp` +15m lock, `night mode` 25 °C when `DHD > DHT`, `turn off` after 4h at 25 °C) | **VERBOSE / FRAGILE.** Requires manual timer/timestamp management across ~8 `setTimer` slots and loses change-lock / 4h night-mode countdown across reboots or rule edits. | **PASS.** Clean state machine with `millis()` / epoch timestamps for change-lock (`+15m`/`+20m`/`+30m`) and 4h `setpoint == 25` duration tracking. |
| **§5, §6, §7: DHW (`14:00` / `<38 °C`), Cooling (`08:00`/`20:00`, `garage_dew_point < 15`), Quiet Mode (`FM off`/`FM on` with 2h defrost guard)** | **FAIL on `garage_dew_point`; VERBOSE on durations.** Cannot read HA's `sensor.garage_dew_point` for cooling condensation guard; requires manual timers for 1m/5m/10m/15m/1h/2h/4h `for:` conditions. | **PASS.** Native `on_time:` triggers (`08:00`, `09:00`, `14:00`, `20:00`), `platform: homeassistant` import for `sensor.garage_dew_point` (with safe offline fallback), and declarative `for:` durations / `millis()` timestamps. |
| **§2.1: 2× DS18B20 1-Wire Sensors** (`281010c6271901e7` outlet, `28a9deb227190150` inlet) | **PASS.** Native `ds18b20#281010c6271901e7` and `ds18b20#28a9deb227190150` on `GPIO3`. | **PASS.** Native ESPHome `one_wire:` (`pin: GPIO3`) + `dallas_temp:` (`address: 0xe7011927c6101028` / `0x50011927b2dea928`). |

---

### 3.2 Detailed Technical Breakdown by Requirement

#### 1. User-Tunable Helpers & External HA Sensors (§2.1, §2.2, §6)
The control spec requires 6 user-tunable `input_number` helpers (`meisha_room_target`, `meisha_heating_curve_start`, `meisha_heating_curve_angle`, `meisha_fixed_target`, `meisha_heating_start_hour`, `meisha_heating_power_max_outlet_temp`), 2 `input_boolean` switches (`meisha_cooling_mode`, `meisha_heating_high_power_override`), and external Home Assistant sensors (`sensor.garage_dew_point` for cooling condensation guard and optionally `weather_forecast_home_temperature_24h`).

- **In HeishaMon Rules:** As proven in Section 1.2, there is **no way** for Home Assistant to pass these values into HeishaMon Rules at runtime without regenerating `/rules.txt` and uploading it via `POST /saverules` — which wipes `global_varstack` and destroys all rolling averages, degree-hours, and PID state.
- **In ESPHome:** Every helper is defined directly on the ESP32 as a `number` or `switch` component with `optimistic: true` and `restore_value: true`:
  ```yaml
  number:
    - platform: template
      id: meisha_room_target
      name: "Meisha Room Target"
      min_value: 18.0
      max_value: 25.0
      step: 0.5
      initial_value: 23.0
      optimistic: true
      restore_value: true
  ```
  - When Home Assistant is online, these entities appear natively in Home Assistant and can be adjusted from the dashboard at any time without reflashing.
  - Every change is written to ESP32 NVS flash (`restore_value: true`, throttled by `preferences: flash_write_interval: 5min` to prevent flash wear).
  - If Home Assistant or Wi-Fi is completely offline across a power outage, the ESP32 boots with the last saved values from NVS and continues controlling the heat pump without interruption.
  - For `sensor.garage_dew_point`, ESPHome subscribes via `platform: homeassistant`. In the `08:00` cooling start check (§6), the lambda checks `if (id(garage_dew_point).has_state() && !isnan(id(garage_dew_point).state))` (or falls back to blocking automatic cooling start if HA is offline and dew point is unknown).

#### 2. Rolling Averages (`ambient_24h`, `ambient_1h`, `room_temp_6h`) & `degree_hours_day` (§3.1, §3.3)
- **In HeishaMon Rules:**
  - Because the VM lacks arrays and loops, a true sliding-window average over 24 hours (`ambient_24h`) or 6 hours (`room_temp_6h`) cannot be cleanly maintained. One would have to approximate with an Exponentially Weighted Moving Average (EWMA), which starts at `@Outside_Temp` on every reboot and takes 24+ hours to converge after any power blip or rule edit.
  - Worse, if the ESP32 reboots at `16:00` on a winter afternoon after most of the daily heat budget (`degree_hours_day`) has already been delivered, `#degree_hours_day` resets to `NULL` (`0`), causing the heat pump to deliver a second full day's heat budget overnight instead of entering `25 °C` night mode!
- **In ESPHome:**
  - An `interval: 1min` lambda updates fixed-size C++ ring buffers stored in `globals:` (`type: float[96]` for `ambient_24h` sampled every 15 minutes, `float[72]` for `room_temp_6h` sampled every 5 minutes, `float[60]` for `ambient_1h` sampled every 1 minute) with `restore_value: true`.
  - `degree_hours_day` integrates `room_delta = outlet - room_target` (when `pump_flow > 0` and `valve == Room`) in an NVS-backed `globals:` float (`restore_value: true`) and resets to `0.0f` at `09:00:00` inside the `Meisha day mode #1` routine.
  - With `preferences: flash_write_interval: 15min`, NVS flash wear is negligible (~96 writes/day across wear-leveled NVS pages) while ensuring a reboot loses at most 15 minutes of integrated degree-hours.

#### 3. Wall-Clock Triggers (`09:00` Day Mode #1, `14:00` DHW, `08:00`/`20:00` Cooling, `07:00–09:00` Window) (§4.1, §4.5, §5.1, §6)
- **In HeishaMon Rules:** `%hour` and `%minute` are read-only variables, not events ([`HeishaMon/rules.cpp:144-157, 249-364`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L144-L157)). A rule must run a perpetual `setTimer(1, 30)` loop checking `if %hour == 9 && %minute == 0 && #day_started != 1 then ... end`.
- **In ESPHome:** The `time:` component supports dual time sources (`platform: sntp` + `platform: homeassistant` with local timezone `CET-1CEST,M3.5.0,M10.5.0/3`). Fixed daily schedules (`08:00`, `09:00`, `14:00`, `20:00`) use declarative `on_time:` cron blocks, and time-of-day window checks (`not between 07:00–09:00` for night mode §4.5, and `hours_since(start_hour)` for `hours_time_left` §3.5) read `id(sntp_time).now()` directly.

#### 4. I-Only PID Controller & Floating-Point Precision (§3.4)
The spec defines an integral-only controller on `Heating Curve Error = curve_target - outlet` (when `pump_flow > 0` and `valve == Room`, else `0`):
- `simple_pid_controller` has `setpoint = 0`, `Kp = 0`, `Ki = 5e-5` `/s`, `Kd = 0`, `sample = 10 s`, output clamped to `[-1.0, +1.0]` with anti-windup and start mode `"last known value"`.
- `Heating Power Target = max(27, curve_target - pid_output)`, rounded to integer with `±0.1` hysteresis vs. its previous integer value.

- **In HeishaMon Rules:**
  - Every float literal (`0.00005`) and intermediate value in the VM heap passes through `float32to27()` ([`src/rules/rules.cpp:222-227`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.cpp#L222-L227)), truncating the 23-bit IEEE-754 mantissa to 18 bits. For small errors over `dt = 10s` (`dI = 0.0005 * error`), adding `dI` to `#pid_i` near `1.0` (`2^-18 = 0.00000381`) suffers noticeable mantissa quantization, and any reboot wipes `#pid_i` instead of preserving `"last known value"`.
- **In ESPHome:**
  - A 10-second `interval:` executes standard 32-bit `float` (or 64-bit `double`) arithmetic:
    ```cpp
    float error = (pump_flow > 0.0f && valve_is_room) ? (outlet - curve_target) : 0.0f;
    id(pid_integral) = std::clamp(id(pid_integral) + 0.00005f * 10.0f * error, -1.0f, 1.0f);
    ```
  - `id(pid_integral)` is stored in `globals:` with `restore_value: true`, matching `"last known value"` across reboots, and the `±0.1` hysteresis rounding for `power_target` is 4 lines of C++.

#### 5. State Duration (`for:`) Conditions & Change-Lock Timers (§4.2–4.6, §5.1–5.2, §6, §7.1–7.2)
The baseline relies heavily on state-duration (`for:`) guards and cooldown locks:
- `bonusgrad curve` (§4.2) & `malusgrad` (§4.3): valve = `Room` for `10 min`, compressor on (`> 10 Hz` hyst `2`) for `5 min`, plus `change_lock` (`+30 min` / `+20 min`).
- `bonusgrad high-temp` (§4.4): `z1_current >= int(setpoint) + 3` held for `1 min` and `2 min`, plus `change_lock` (`+15 min`).
- `turn off (heat mode only)` (§4.6): `setpoint == 25` (or `< 25.1`) held for `4 h`.
- `DHW start` (§5.1): tank temp `< 38 °C` for `1 min`.
- `DHW end` (§5.2): valve `Tank -> Room` held `15 min`, or compressor off for `15 min`.
- `Cool Turn Off` (§6): `ambient_24h < 22` for `1 h`.
- `FM off` / `FM on` (§7.1–7.2): compressor freq > 25 Hz for `1 min`, `pipe_ambient_delta > 4` (or `> 3`) for `5 min`, compressor 1–23.5 Hz for `1 min`, and **no defrost for `2 h`**.

- **In HeishaMon Rules:** Requires allocating and managing a dozen separate one-shot timer indices (`setTimer(N, secs)`) or manual elapsed-second counters in `#` variables, while working around the VM bug where `@Set...` before `setTimer(...)` is silently dropped ([`SKILL.md:40`](https://github.com/heishamon/HeishaMon/blob/main/Examples/Rules/SKILL.md#L40)).
- **In ESPHome:** Supported declaratively via ESPHome's built-in `for:` condition (`condition: for: time: 10min condition: ...`), `binary_sensor:` template filters (`delayed_on`, `delayed_off`), or `uint32_t` millisecond timestamps (`millis()`), making every duration guard explicit and bug-free.

---

### 3.3 Preserving Home Assistant Sensor History & `kamaradclimber/heishamon-homeassistant` Compatibility

A key requirement is **not losing existing Home Assistant sensor history, long-term statistics, or energy/COP utility meters (§8)** when migrating from stock HeishaMon firmware to ESPHome.

#### How Current Entities Are Mapped in `/config/docs/meisha-heat-pump-control.md`
1. **Current Integration (`custom_components/aquarea`, `kamaradclimber/heishamon-homeassistant`):**
   - Stock HeishaMon publishes to MQTT prefix `panasonic_heat_pump/` (`panasonic_heat_pump/main/<TopicName>` and `panasonic_heat_pump/1wire/<rom_hex>`), which `kamaradclimber/heishamon-homeassistant` consumes to create entities like `sensor.panasonic_heat_pump_main_outside_temp`, `sensor.panasonic_heat_pump_main_main_outlet_temp`, `climate.panasonic_heat_pump_main_z1_temp`, `sensor.panasonic_heat_pump_1wire_outlet_temp`, etc.
2. **Two Proven Ways to Preserve 100% of HA Sensor History in ESPHome:**
   - **Strategy A — Keep `kamaradclimber/heishamon-homeassistant` Zero-Touch via ESPHome MQTT Publishing:**
     If you want **zero changes** to existing HA dashboards, energy/COP templates (§8), and `kamaradclimber/heishamon-homeassistant` entities, ESPHome can enable the `mqtt:` component (alongside or instead of `api:`) and publish the exact `panasonic_heat_pump/main/<TopicName>` and `panasonic_heat_pump/1wire/<rom_hex>` topics (plus `panasonic_heat_pump/stats` LWT/stats if expected by the integration). To Home Assistant, the ESPHome board looks 100% identical to stock HeishaMon over MQTT, while all control automations (§3.1–3.5, §4–§7) run locally on the ESP32!
   - **Strategy B — Native ESPHome API with Matching Entity IDs:**
     In Home Assistant, long-term statistics (`statistics` / `statistics_short_term`) and state history (`states_meta`) are keyed by **`entity_id`**. If retiring the MQTT integration in favor of ESPHome's native API, naming the ESPHome entities to match the existing `entity_id`s (`sensor.panasonic_heat_pump_main_outside_temp`, etc.) with matching `unit_of_measurement`, `device_class`, and `state_class` makes Home Assistant seamlessly continue the existing historical graphs.
3. **1-Wire DS18B20 Address Mapping Warning (Byte Order):**
   - HeishaMon formats the 8-byte 1-Wire ROM address by printing `addr[0]..addr[7]` in sequential hex order (`281010c6271901e7` and `28a9deb227190150`, see [`HeishaMon/dallas.cpp:96-100`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/dallas.cpp#L96-L100)).
   - ESPHome's `dallas_temp` component displays/accepts the 64-bit ROM address as a little-endian `uint64_t` integer literal (where family code `0x28` is the least-significant byte `0x...28`):
     - HeishaMon `281010c6271901e7` (`28 10 10 c6 27 19 01 e7`) $\rightarrow$ ESPHome `address: 0xe7011927c6101028`
     - HeishaMon `28a9deb227190150` (`28 a9 de b2 27 19 01 50`) $\rightarrow$ ESPHome `address: 0x50011927b2dea928`

---

## 4. Primary Source References

1. **Baseline Control Specification:**
   - [`/config/docs/meisha-heat-pump-control.md`](file:///config/docs/meisha-heat-pump-control.md)
2. **HeishaMon Firmware & Rules Engine (`heishamon/HeishaMon`):**
   - [`README.md`](https://github.com/heishamon/HeishaMon/blob/main/README.md) (Rules overview, lines 38–165)
   - [`Examples/Rules/SKILL.md`](https://github.com/heishamon/HeishaMon/blob/main/Examples/Rules/SKILL.md) (Rules engine reference, VM bugs, and constraints)
   - [`HeishaMon/HeishaMon.ino`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino) (ESP32-S3 pinout lines 11–27 & 168–193; MQTT subscriptions lines 491–500 & 971–1040; extra `0x21` packet trigger line 768; crash/brownout rule-skip check lines 1924–1942)
   - [`HeishaMon/rules.cpp`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp) (`global_varstack` line 79; `is_variable()` lines 121–246; `is_event()` lines 249–364; `vm_value_get()` lines 366–515; `vm_value_set()` lines 517–738; PSRAM `MEMPOOL_SIZE` allocation lines 913–920; `global_varstack` wipe on parse lines 969–974)
   - [`HeishaMon/src/rules/rules.cpp`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.cpp) (`EPSILON` line 41; `float32to27` Veltkamp-Dekker 27-bit float truncation lines 221–227, 1526, 1784, 4646)
   - [`HeishaMon/src/rules/rules.h`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.h) (`MEMPOOL_SIZE = 32768` line 51; token definitions lines 68–98; `vm_vfloat_t` 5-bit type tag + 27-bit float layout lines 132–135)
   - [`HeishaMon/src/rules/operator.cpp`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/operator.cpp) (Supported operators and precedences lines 27–42)
   - [`HeishaMon/HeishaOT.cpp`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaOT.cpp) (`heishaOTDataStruct` `rw` flags lines 17–47; `mqttOTCallback` lines 436–556)
   - [`HeishaMon/decode.cpp`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/decode.cpp) & [`HeishaMon/commands.cpp`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/commands.cpp) (CN-CNT `0x10` and `0x21` protocol decoding and command byte offsets)
   - [`HeishaMon/dallas.h`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/dallas.h) & [`HeishaMon/dallas.cpp`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/dallas.cpp) (`ONE_WIRE_BUS = 3` on ESP32; ROM address hex formatting lines 96–100)
3. **ESPHome Panasonic Aquarea Components:**
   - [`ElVit/esphome_components` (`components/panasonic_heatpump`)](https://github.com/ElVit/esphome_components/tree/main/components/panasonic_heatpump):
     - [`README.md`](https://github.com/ElVit/esphome_components/blob/main/components/panasonic_heatpump/README.md) (`TOP0..TOP143`, `XTOP0..XTOP5`, `SET1..SET48`)
     - [`panasonic_heatpump.cpp`](https://github.com/ElVit/esphome_components/blob/main/components/panasonic_heatpump/panasonic_heatpump.cpp) (ESP-IDF UART handling, `0x21` extra query trigger at line 321)
     - [`prototypes/panasonic_heatpump/example_heishamon_large.yaml`](https://github.com/ElVit/esphome_components/blob/main/prototypes/panasonic_heatpump/example_heishamon_large.yaml) (HeishaMon Large ESP32-S3 hardware configuration)
   - [`IoTLabs-pl/ESPHome-Components` (`components/panasonic_aquarea`)](https://github.com/IoTLabs-pl/ESPHome-Components/tree/master/components/panasonic_aquarea):
     - [`README.md`](https://github.com/IoTLabs-pl/ESPHome-Components/blob/master/components/panasonic_aquarea/README.md) & [`protocol.cpp`](https://github.com/IoTLabs-pl/ESPHome-Components/blob/master/components/panasonic_aquarea/protocol.cpp) (`EXTRA_SUPPORT_BYTE_INDEX = 199`, `0x10`/`0x21` frame construction)
