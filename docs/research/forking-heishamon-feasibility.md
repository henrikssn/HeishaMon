# Feasibility, Architecture & Trade-Offs of Forking HeishaMon vs. Migrating to ESPHome

> **Research Scope & Primary Sources**
> This report investigates the feasibility, implementation scope, build/OTA workflow, and architectural trade-offs of **forking the official [`heishamon/HeishaMon`](https://github.com/heishamon/HeishaMon) firmware** (either by patching its Rules Engine or by embedding a native C++ control module) versus **migrating to ESPHome (`ElVit/esphome_components`)** to run the autonomous **Meisha** heat pump control baseline ([`/config/docs/meisha-heat-pump-control.md`](file:///config/docs/meisha-heat-pump-control.md)) on a **HeishaMon Large (ESP32-S3)** connected to a **Panasonic Aquarea M-series** heat pump.
>
> All findings are verified against the primary source files in [`heishamon/HeishaMon`](https://github.com/heishamon/HeishaMon) (`main` branch) and build upon [`/agy/esphome-aquarea/docs/research/heishamon-rules-vs-esphome.md`](file:///agy/esphome-aquarea/docs/research/heishamon-rules-vs-esphome.md).

---

## Executive Summary

| Dimension | Option 1: Fork HeishaMon + Patch Rules Engine | Option 2: Fork HeishaMon + Native C++ Module (`meisha.cpp`) | Option 3: Migrate to ESPHome (`ElVit/esphome_components`) |
| :--- | :--- | :--- | :--- |
| **Core Concept** | Patch `HeishaMon/rules.cpp` & `HeishaMon.ino` to add MQTT pub/sub and NVS/LittleFS persistence for `#` globals; write Meisha logic in `/rules.txt` DSL. | Add `meisha.cpp` / `meisha.h` directly into `HeishaMon/` (~10-line hook in `HeishaMon.ino`), bypassing the Rules VM entirely and running C++ state machines. | Replace HeishaMon firmware with ESPHome + `ElVit/esphome_components` (`panasonic_heatpump`), implementing Meisha logic in ESPHome YAML + C++ lambdas. |
| **Feasibility for Meisha Baseline** | **Low / High-Risk.** Even after fixing MQTT pub/sub and NVS persistence in C++, the Rules VM has a **hard 127-slot `varstack` limit** (`src/rules/rules.cpp:5413-5420`), no arrays/loops for 24h/6h/1h sliding windows, 27-bit float truncation (`float32to27`), and severe VM bugs. | **High.** Full IEEE-754 `float`/`double`, native C++ ring buffers, structs, `Preferences` NVS persistence, `millis()` timers for all 12+ `for:` guards, and HA MQTT Discovery. | **High.** Native ESPHome entities (`number`, `switch`, `sensor`, `time`, `interval`), `restore_value: true` NVS persistence, native HA entity import (`platform: homeassistant`), and declarative `for:` conditions. |
| **First-Time Flash Experience** | **Web UI OTA (`/firmware`)** — zero USB-C required; preserves `/config.json` (Wi-Fi/MQTT) and `/dallas.json` in LittleFS automatically. | **Web UI OTA (`/firmware`)** — zero USB-C required; preserves `/config.json` and `/dallas.json` in LittleFS automatically. | **Web UI OTA possible (`firmware.ota.bin`)**, but replaces firmware stack; if Wi-Fi/config has a typo, recovery requires opening enclosure for USB-C (`GPIO19/20`). |
| **`kamaradclimber/heishamon-homeassistant` Compatibility** | **100% Native (Zero-Touch).** Keeps exact same MQTT topics, Web UI, and HA integration. | **100% Native (Zero-Touch).** Keeps exact same MQTT topics, Web UI, CZ-TAW1 proxy, and HA integration, while adding MQTT Discovery for the 8 Meisha helpers. | **Requires Bridge or Entity Renaming.** Either mirror `panasonic_heat_pump/main/...` MQTT topics in ESPHome or rename ESPHome entity IDs to preserve HA history. |
| **External HA Inputs (`garage_dew_point`, `forecast_24h`)** | Requires HA automation (or `mqtt_statestream`) to publish HA state to an MQTT topic under `panasonic_heat_pump/...`. | Requires HA automation (or `mqtt_statestream`) to publish those 2 HA entities to MQTT (or subscribe directly to their source MQTT topics). | **Zero-Touch Native.** `platform: homeassistant` pulls any HA entity directly over ESPHome's native API without MQTT glue. |
| **Host-Side Unit Testing** | Can use `Examples/Rules/harness/` (`g++`), but requires porting any C++ patches in `HeishaMon/rules.cpp` into `Examples/Rules/harness/harness.cpp`. | **`meisha.cpp` can be compiled & unit-tested directly on Linux (`g++` / `pytest` / GoogleTest)** with a thin mock header for `getDataValue()` and `send_command()`. | Logic embedded in YAML lambdas is harder to unit-test standalone unless extracted into a separate C++ `.h` file included via `includes:`. |

### Key Takeaway
If you want to **fork HeishaMon** (to keep 100% zero-touch compatibility with `kamaradclimber/heishamon-homeassistant`, the HeishaMon Web UI, `/config.json`, and risk-free Web UI OTA flashing without touching USB-C), **do NOT use Approach A (patching the Rules Engine)**. Instead, **Approach B (embedding a native C++ `meisha.cpp` module inside the HeishaMon fork)** is dramatically simpler, safer, and cleaner (~500–650 lines of clean C++ vs. fighting a 5,672-line custom bytecode VM with a 127-variable hard ceiling and 27-bit floats).

The real architectural decision is therefore a two-way contest between:
1. **Option 2: Forking HeishaMon + Native C++ Module (`meisha.cpp`)**, and
2. **Option 3: Migrating to ESPHome (`ElVit/esphome_components`)**.

---

## 1. Two Architectural Ways to Fork HeishaMon

### 1.1 Approach A: Patching & Extending the HeishaMon Rules Engine

In stock HeishaMon, the rules subsystem is split across two layers:
1. **The Core VM (`HeishaMon/src/rules/`):** A 5,672-line custom lexer, AST/bytecode compiler, and stack machine in [`HeishaMon/src/rules/rules.cpp`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.cpp) and [`HeishaMon/src/rules/rules.h`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.h), originally written by CurlyMo for ESP8266 memory constraints.
2. **The Firmware Glue (`HeishaMon/rules.cpp`):** A 1,112-line integration layer in [`HeishaMon/rules.cpp`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp) that maps `@`, `#`, `$`, `%`, `?`, `ds18b20#`, and `s0#` sigils to HeishaMon's global C++ state.

Below is an exact code audit of what it would take to patch `HeishaMon/rules.cpp` and `HeishaMon.ino` to fix the missing MQTT pub/sub and persistence, followed by the VM limitations that **still remain** afterward.

#### A.1 Adding MQTT Pub/Sub for `#` Globals (`HeishaMon.ino` & `HeishaMon/rules.cpp`)
To allow Home Assistant to read and write `#` globals (such as the 8 user-tunable Meisha helpers and external HA sensors `garage_dew_point` / `forecast_24h`) without re-uploading `/rules.txt`:

1. **Subscribing in `mqtt_reconnect()` ([`HeishaMon/HeishaMon.ino:491-501`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L491-L501)):**
   Adding a subscription to `<mqtt_topic_base>/rules/set/#` (or `<mqtt_topic_base>/rules/#`) is 2 lines of C++:
   ```cpp
   sprintf(topic, "%s/rules/set/#", heishamonSettings.mqtt_topic_base);
   mqtt_client.subscribe(topic);
   ```
2. **Dispatching in `mqtt_callback()` ([`HeishaMon/HeishaMon.ino:971-1040`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L971-L1040)):**
   In `mqtt_callback()`, matching `strncmp(topic_command, "rules/set/", 10) == 0` extracts the variable name `varname` and string payload `msg`.
3. **Mutating `global_varstack` from C++ Outside the VM (`HeishaMon/rules.cpp:742-804`) — Hidden Pointer Hazard:**
   Look at how `vm_value_set()` stores `#` globals in `global_varstack` ([`HeishaMon/rules.cpp:742-795`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L742-L795)):
   ```cpp
   // HeishaMon/rules.cpp:754-765
   if(array == NULL) {
     if((table->array = (struct array_t *)REALLOC(table->array, sizeof(struct array_t)*(table->nr+1))) == NULL) {
       OUT_OF_MEMORY
     }
     array = &table->array[table->nr];
     memset(array, 0, sizeof(struct array_t));
     table->nr++;
     rules_ref(key);
   }
   array->key = key;
   ```
   - Notice that `array->key` is **not** a heap-allocated `strdup()` string! `key` comes from `rules_tostring(-2)` ([`HeishaMon/rules.cpp:638`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L638)), which returns a `char *` pointing into the VM's internal `varstack` table (`struct vm_vchar_t` in [`HeishaMon/src/rules/rules.cpp:598, 1564-1594`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.cpp#L1564-L1594)), and `rules_ref(key)` increments the VM's internal reference count on that `varstack` entry.
   - If `mqtt_callback()` writes a `#` variable from C++ (especially during initial MQTT connect at boot, or for a `#` variable that hasn't been assigned yet), passing a stack or `strdup()` pointer as `array->key` without decoupling `global_varstack` from `rules_ref()`/`rules_unref()` will mix heap pointers with VM `varstack` pointers.
   - Therefore, you must refactor `global_varstack` in `HeishaMon/rules.cpp` so `array->key` and `array->val.s` own their own `strdup()`/`free()` heap buffers instead of calling `rules_ref()` / `rules_unref()`.
4. **Firing `on #var then` Events (`HeishaMon/rules.cpp:249-364`):**
   Currently, `is_event()` ([`HeishaMon/rules.cpp:249-364`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L249-L364)) rejects `#` at `text[0]` (line `353` requires `isalpha(text[0]) || text[0] == '_'`). You must patch `is_event()` to accept `text[0] == '#'`, and call `rules_event_cb("#", varname)` from `mqtt_callback()`.
5. **Publishing `#` Global Changes to MQTT (`HeishaMon/rules.cpp:742-805`):**
   In `vm_value_set()`, when `key[0] == '#'` and **`parsing == 0`** (critical: during `rule_initialize()`, the parser dry-runs every rule with `parsing = 1` at [`HeishaMon/rules.cpp:963, 1008`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L963) and [`src/rules/rules.cpp:5618`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.cpp#L5618)!), you can publish `<base>/rules/state/<name>` if the value changed.

#### A.2 Implementing NVS/Filesystem Persistence & Preserving `global_varstack` Across `/saverules`
1. **What `readRuleFromFS()` / `writeRuleToFS()` (`HeishaMon/rules.cpp:88-120`) Actually Are:**
   Inspection of [`HeishaMon/rules.cpp:88-120`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L88-L120) reveals:
   ```cpp
   // HeishaMon/rules.cpp:99-111
   // sprintf_P((char *)&fname, PSTR("/rule%d.bc"), i);
   // File f = LittleFS.open(fname, "r");
   // ...
   // f.readBytes((char *)rules[i]->ast.buffer, rules[i]->ast.bufsize);
   ```
   Those commented-out stubs were **not** for saving `#` global variables — they were CurlyMo's abandoned stubs for paging compiled rule bytecode (`/rule%d.bc`) to flash on ESP8266! Persisting `global_varstack` must be written from scratch (e.g., serializing `global_varstack` via `ArduinoJson` to `/rules_vars.json` on LittleFS or ESP32 `Preferences` NVS).
2. **Why Preserving `global_varstack` Across `/saverules` (`rules_parse()` at `HeishaMon/rules.cpp:954-1041`) Is Tricky:**
   Look at [`HeishaMon/rules.cpp:965-1027`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L965-L1027):
   ```cpp
   // HeishaMon/rules.cpp:965-975
   if(nrrules > 0) {
     rules_free_stack();
     rules_gc(&rules, &nrrules);

     struct varstack_t *table = (struct varstack_t *)&global_varstack;
     if(table->array != NULL) {
       FREE(table->array);
     }
     table->nr = 0;
   }
   memset(mempool, 0, MEMPOOL_SIZE);
   // ... [lines 1008-1010: rule_initialize() runs dry-run validation rule_run(obj, 1)] ...
   // HeishaMon/rules.cpp:1022-1026
   struct varstack_t *table = (struct varstack_t *)&global_varstack;
   if(table->array != NULL) {
     FREE(table->array);
   }
   table->nr = 0;
   ```
   Notice three critical traps in `rules_parse()`:
   - **Trap 1:** `global_varstack` is wiped **twice** — once at lines `969-974` before parsing, and **a second time at lines `1022-1026`** after `rule_initialize()`!
   - **Trap 2:** Why does line `1022-1026` wipe `global_varstack` after `rule_initialize()`? Because `rule_initialize()` ([`HeishaMon/src/rules/rules.cpp:5618`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.cpp#L5618)) executes `rule_run(obj, 1)` on every `on ... then` block to validate bytecode! And inside `vm_value_set()` ([`HeishaMon/rules.cpp:670 vs 742`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L670)), only `@Set` commands check `if(parsing == 0)`; `#` global assignments (`key[0] == '#'`) **execute during the validation dry-run (`parsing == 1`)** and overwrite `global_varstack` with whatever dummy branches the validator walked!
   - **Trap 3:** `rules_gc(&rules, &nrrules)` ([`HeishaMon/rules.cpp:967`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L967) $\rightarrow$ [`HeishaMon/src/rules/rules.cpp:5292-5320`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.cpp#L5292-L5320)) frees `varstack->buffer` and all `node->value` strings that `global_varstack.array[x].key` points to!
   - To preserve `#` globals across `/saverules` (and across `rules_boot()`, which `timer_cb(-4)` calls right after `/saverules` at [`HeishaMon.ino:1770`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1770)), you must:
     1. Refactor `global_varstack` to own heap-allocated `strdup()` strings for `key` and `val.s`.
     2. Snapshot `global_varstack` before `rule_initialize()` (or make `vm_value_set()` a no-op on `global_varstack` when `parsing == 1`), and remove the two `FREE(table->array)` wipes.
     3. In `/rules.txt`, change `on System#Boot then` so it only seeds `#` globals when `!isset(#var)` (`#var = coalesce(#var, <default>)`), otherwise `rules_boot()` on `/saverules` or reboot will overwrite the restored NVS/RAM values!
3. **Patching the Crash/Brownout Rule-Disable Check (`HeishaMon.ino:1924-1942`):**
   In [`HeishaMon/HeishaMon.ino:1930-1938`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1930-L1938), any `esp_reset_reason()` in `4..11` (which includes `ESP_RST_BROWNOUT = 9` and watchdog resets) skips `rules_parse("/rules.txt")` if `force_rules` is `false`. You must either narrow that check to `ESP_RST_PANIC (4)` or force `force_rules = true`.

#### A.3 What Still Remains Painful or Broken in the Rules DSL Even After Patching MQTT & Persistence
Even if you write ~250 lines of C++ in `HeishaMon/rules.cpp` and `HeishaMon.ino` to add MQTT pub/sub and NVS persistence for `#` globals, **you still have to implement the Meisha baseline inside the Rules VM (`HeishaMon/src/rules/rules.cpp`)**, which suffers from five severe architectural blockers:

1. **BLOCKER: Hard 127-Slot `varstack` Limit (`HeishaMon/src/rules/rules.cpp:5413-5420`):**
   In [`HeishaMon/src/rules/rules.cpp:5412-5420`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.cpp#L5412-L5420), `rule_initialize()` enforces a hard ceiling of `INT8_MAX` (`127`) entries in `varstack`:
   ```cpp
   // HeishaMon/src/rules/rules.cpp:5412-5420
   if(rule_prepare((char **)&input->payload, &bcsize, &heapsize, &varsize, &memsize, &newlen) == -1 ||
     (varsize/sizeof(struct vm_vchar_t)) > INT8_MAX ||
     ((varstack->nrbytes + varsize) / sizeof(struct vm_vchar_t)) > INT8_MAX) {
     ...
     logprintf_P(F("ERROR: maximum number of 127 variables reached"));
   ```
   What consumes a `vm_vchar_t` slot in `varstack`? Inspection of `rule_prepare()` ([`src/rules/rules.cpp:674, 1009, 1228, 1329`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.cpp#L674)) shows that **every unique identifier and string literal** in `/rules.txt` takes a slot:
   - Every unique `#` global variable, `$` local variable, `@` read topic, `@Set` command topic, `%` time variable, and `ds18b20#` sensor (`lines 1228-1231`)
   - Every `on <event> then` block name and custom function call target (`lines 1009-1011, 1329-1331`)
   - Every unique quoted string literal in `print(...)` or `concat(...)` (`lines 674-676`)
   - Every unique runtime string pushed via `rules_pushstring()` (`line 1547`).
   Because bytecode instructions encode `varstack` indices in a signed 8-bit integer (`INT8_MAX = 127`), **a ruleset with >127 total unique variable names + event names + string literals refuses to compile**!
2. **No Arrays or Loops (`HeishaMon/src/rules/rules.h:68-98`):**
   Because the DSL has no arrays or loops, you cannot implement true sliding-window ring buffers for `ambient_24h` (24h window), `room_temp_6h` (6h window), and `ambient_1h` (1h window) without declaring dozens of scalar `#` globals — which immediately hits the 127-slot `varstack` limit above. You are forced to replace the true sliding windows from [`meisha-heat-pump-control.md`](file:///config/docs/meisha-heat-pump-control.md) §3.1 with first-order EWMA approximations.
3. **27-bit Float Mantissa Truncation (`float32to27` in `HeishaMon/src/rules/rules.cpp:222-227`):**
   Every float pushed to the VM stack or stored in a rule's constant heap is truncated from 32-bit IEEE-754 (23-bit mantissa) to 27 bits (18-bit mantissa, ~5.7 decimal digits) via `float32to27()` ([`src/rules/rules.cpp:222-227, 1526, 1784, 4646`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.cpp#L222-L227)). For the Meisha I-only PID (`Ki = 0.00005` `/s`, `dt = 10s` $\rightarrow$ step `0.0005 * error` added to `#pid_i` near `±1.0`), 18-bit mantissa quantization (`ulp = 2^-18 ≈ 3.8e-6`) introduces ~1% step quantization error on every 10-second tick unless you scale the internal accumulator by `10000x` in the DSL.
4. **Landmine VM Bugs (`Examples/Rules/SKILL.md:39-47`):**
   - **`NULL` in `&&` chains evaluates truthy (`SKILL.md:46`):** `if @X == 0 && 1 == 1 then` executes its `then` branch when `@X` is `NULL`! Every single condition across all 12 automations must wrap variables in `coalesce(#x, 0)` or nested `if isset(#x) then`.
   - **Silent drop of `@Set...` before built-in calls (`SKILL.md:40`):** In any function block with `$` parameters, a top-level `@Set...` assignment before `setTimer(...)` or `print(...)` is executed out of order and **silently dropped**.
5. **12+ State-Duration (`for:`) Timers Managed by Hand:**
   The Meisha spec uses 12+ distinct `for:` state-duration conditions (`valve == Room for 10m`, `compressor > 10Hz for 5m`, `z1_current >= int(setpoint)+3 for 1m / 2m`, `setpoint == 25 for 4h`, `DHW_Temp < 38 for 1m`, `valve Tank->Room for 15m`, `compressor off for 15m`, `ambient_24h < 22 for 1h`, `forecast < 0.1 for 5m`, `pipe_ambient_delta > 4 for 5m`, `compressor > 25Hz for 1m`, `compressor 1..23.5Hz for 1m`, `no defrost for 2h`) plus `change_lock` (`15m`/`20m`/`30m`). Expressing all of these in a flat timer-ID namespace (`setTimer(1..20, secs)`) without structs or helper classes makes `/rules.txt` brittle and sprawling (~600+ lines of untyped DSL).

---

### 1.2 Approach B: Embedding a Native C++ Controller Module in the HeishaMon Fork (`meisha.cpp` / `meisha.h`)

Instead of touching the Rules Engine at all, we can leave `HeishaMon/rules.cpp` untouched (with `/rules.txt` empty) and add a self-contained C++ module (`HeishaMon/meisha.h` and `HeishaMon/meisha.cpp`) directly into the `HeishaMon/` sketch directory.

#### B.1 How `HeishaMon.ino` Is Structured & Where `meisha.cpp` Hooks In
As documented in [`AGENTS.md:17-53`](https://github.com/heishamon/HeishaMon/blob/main/AGENTS.md#L17-L53), the entire firmware is a single Arduino sketch in `HeishaMon/`. `arduino-cli compile` automatically compiles every `.cpp` file in `HeishaMon/`.

On ESP32-S3 (`#ifdef ESP32`), `HeishaMon.ino` runs two execution contexts:
1. **`serialTXTask` (FreeRTOS task pinned to Core 1, [`HeishaMon/HeishaMon.ino:846-929, 1696-1704`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L846-L929)):**
   Handles outbound `Serial1` (`GPIO17/18`) polling every `heishamonSettings.waitTime` seconds (default `5s`: `0x10` standard query at line `890`, `0x21` extra query at line `904`, `0x50` optional PCB query at line `872`) and dequeues outbound user commands from `cmdQueue` (line `916`).
2. **Main Arduino `setup()` and `loop()` (Core 1, [`HeishaMon/HeishaMon.ino:1801-2246`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1801-L2246)):**
   Handles `webserver_loop()`, `check_wifi()`, `ArduinoOTA.handle()`, `mqtt_client.loop()`, `readHeatpump()` $\rightarrow$ `readSerial()` $\rightarrow$ `decode_heatpump_data()`, `readProxy()`, `dallasLoop()`, `s0Loop()`, periodic stats publishing, and `timerqueue_update()`.

Hooking `meisha.cpp` into `HeishaMon.ino` requires **only 4 lines of glue across `HeishaMon.ino`** ( keeping merge conflicts with upstream `heishamon/HeishaMon` near zero):
1. **`setup()` ([`HeishaMon.ino:1943`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1943)):**
   ```cpp
   meisha_setup();
   ```
2. **`loop()` ([`HeishaMon.ino:2051`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L2051), right after `readHeatpump()` and `dallasLoop()`):**
   ```cpp
   meisha_loop();
   ```
3. **`mqtt_reconnect()` ([`HeishaMon.ino:501`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L501)):**
   ```cpp
   meisha_mqtt_connected(); // subscribes to <base>/meisha/set/# + publishes HA MQTT discovery
   ```
4. **`mqtt_callback()` ([`HeishaMon.ino:1036`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1036)):**
   ```cpp
   } else if (meisha_mqtt_callback(topic, topic_command, msg, length)) {
     // handled by meisha module
   }
   ```

#### B.2 Accessing Decoded Telemetry, 1-Wire Sensors, Commands & MQTT from `meisha.cpp`
All HeishaMon state is already exposed via global symbols in `decode.h`, `commands.h`, `dallas.h`, and `webfunctions.h`:

1. **Heat Pump Telemetry (`actData[]` & `getDataValue()` in [`HeishaMon/decode.h:18, 86-145`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/decode.h#L18) and [`HeishaMon/decode.cpp:158-237`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/decode.cpp#L158-L237)):**
   - `extern char actData[DATASIZE];` (`203` bytes, defined at [`HeishaMon.ino:114`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L114)).
   - Before the first valid `0x10` frame arrives from the heat pump, `actData[0] == '\0'`. Once valid, `actData[0] == 0x71` ([`HeishaMon.ino:768`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L768)).
   - Any `TOP0..TOP143` value can be read in C++ via `getDataValue(actData, Topic_Number).toFloat()` / `.toInt()`:
     - `TOP0` (`Heatpump_State`): `0` or `1`
     - `TOP1` (`Pump_Flow`): `float` L/min (`decode.cpp:126-131`)
     - `TOP4` (`Operating_Mode_State`): `0`=Heat only, `1`=Cool only, `2`=Auto, `3`=DHW only, `4`=Heat+DHW, `5`=Cool+DHW, `6`=Auto+DHW (`decode.cpp:81-104`)
     - `TOP5` (`Main_Inlet_Temp`) & `TOP6` (`Main_Outlet_Temp`): `float` °C with `0.25 °C` fractional resolution (`decode.cpp:165-210`)
     - `TOP7` (`Main_Target_Temp`): `int` °C
     - `TOP8` (`Compressor_Freq`): `int` Hz
     - `TOP10` (`DHW_Temp`): `int` °C
     - `TOP14` (`Outside_Temp`): `int` °C
     - `TOP18` (`Quiet_Mode_Level`): `0..3`
     - `TOP20` (`ThreeWay_Valve_State`): `0` (`Room`) or `1` (`Tank`)
     - `TOP21` (`Outside_Pipe_Temp`): `int` °C
     - `TOP26` (`Defrosting_State`): `0` or `1`
     - `TOP27` (`Z1_Heat_Request_Temp`): `int` °C
     - `TOP33` (`Room_Thermostat_Temp`): `int` °C
     - `TOP36` (`Z1_Water_Temp`): `int` °C
   - Similarly, M-series extra power registers (`XTOP0..XTOP5`) live in `extern char actDataExtra[DATASIZE];` (`actDataExtra[0] == 0x71`) via `getDataValueExtra(actDataExtra, Topic_Number)` ([`decode.cpp:239-249`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/decode.cpp#L239-L249)).
2. **1-Wire DS18B20 Sensors (`actDallasData` in [`HeishaMon/dallas.h:25-32`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/dallas.h#L25-L32) and [`HeishaMon/dallas.cpp:21-22, 135-181`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/dallas.cpp#L21-L22)):**
   - `extern dallasDataStruct *actDallasData;` and `extern int dallasDevicecount;` (`MAX_DALLAS_SENSORS = 15`).
   - A 6-line helper in `meisha.cpp` looks up `"281010c6271901e7"` (outlet) and `"28a9deb227190150"` (inlet) in `actDallasData[i]`, checking `actDallasData[i].known && actDallasData[i].present && actDallasData[i].temperature > -50.0f`, and automatically falls back to `TOP6` (`Main_Outlet_Temp`) / `TOP5` (`Main_Inlet_Temp`) if a 1-Wire probe is temporarily offline.
3. **Sending Commands (`send_heatpump_command()` / `send_command()` in [`HeishaMon/commands.cpp:1288-1318`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/commands.cpp#L1288-L1318) & [`HeishaMon.ino:930-941`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L930-L941)):**
   - Calling `send_heatpump_command((char*)"SetZ1HeatRequestTemperature", buf, send_command, log_message, heishamonSettings.optionalPCB)` builds the 110-byte `panasonicSendQuery` frame, logs the action to MQTT `<base>/log` and the Web UI console, and enqueues it onto FreeRTOS `cmdQueue` (`MAXCOMMANDSINBUFFER = 10`, [`HeishaMon.ino:133, 939, 1693`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L939)).
   - Better yet, `meisha.cpp` can implement a **desired-state reconciler with command pacing** (e.g., minimum 10s spacing between multi-step commands like `SetOperationMode` + `SetHeatpump` in §4.6 and §5.1, and 30s retry verification if the heat pump drops a command during a busy cycle).
4. **MQTT Publishing, Subscriptions & Home Assistant Auto-Discovery (`mqtt_client` in [`HeishaMon.ino:156, 1666-1687`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1666-L1687)):**
   - `extern PubSubClient mqtt_client;` is already initialized with `mqtt_client.setBufferSize(1024)` ([`HeishaMon.ino:1667`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1667)).
   - On MQTT connect, `meisha.cpp` can publish **Home Assistant MQTT Discovery** config payloads (`homeassistant/number/meisha_<id>/config`, `homeassistant/switch/meisha_<id>/config`, `homeassistant/sensor/meisha_<id>/config`) so all 8 tunable helpers and all derived Meisha sensors (`curve_target`, `power_target`, `pid_output`, `degree_hours_day`, `degree_hours_target`, `hours_time_left`, `ambient_24h`, `room_temp_6h`) appear automatically in Home Assistant as native entities backed by the ESP32-S3!
   - **Handling the 2 External HA Inputs (`sensor.garage_dew_point` and `weather_forecast_home_temperature_24h`):**
     Because HeishaMon speaks MQTT (not ESPHome's native HA API), `meisha.cpp` subscribes to `<base>/meisha/in/garage_dew_point` and `<base>/meisha/in/forecast_24h` (or directly to the source sensor's MQTT topic if `garage_dew_point` already comes from MQTT). In Home Assistant, a tiny 10-line automation publishes those two states with `retain: true` whenever they change. If HA is offline at `08:00`, `meisha.cpp` checks the age/validity of `garage_dew_point` and safely inhibits automatic cooling start if unknown.

#### B.3 Non-Volatile Persistence (`<Preferences.h>` NVS vs. `LittleFS`) on ESP32-S3
On ESP32-S3 (`esp32:esp32@3.3.11`), `PartitionScheme=min_spiffs` provides **two separate flash storage subsystems**:
1. **ESP-IDF NVS via `<Preferences.h>` (Recommended):**
   - Built into the Arduino-ESP32 core (`#include <Preferences.h>`), backed by the dedicated `nvs` partition (`0x9000`, `20 KB`), which HeishaMon does not currently use (except implicitly for Wi-Fi PHY calibration).
   - Completely isolated from `LittleFS` (`0x3D0000`, `192 KB`), where HeishaMon stores `/config.json`, `/dallas.json`, `/heishamon`, and `/rules.txt`.
   - Wear-leveled at 32-byte entry granularity by ESP-IDF.
   - `meisha.cpp` can store:
     - The **8 user-tunable helpers** in NVS namespace `"meisha_cfg"` — written immediately whenever adjusted via MQTT.
     - The **runtime accumulator & ring-buffer state** (`pid_integral`, `degree_hours_day`, `dhw_time`, `room_temp_snapshot`, `forecast_snapshot`, `last_reset_day`, `ambient_24h` 96-sample ring buffer, `room_6h` 72-sample ring buffer) in a single `struct MeishaState` (`~720 bytes`) saved via `prefs.putBytes("state", &state, sizeof(state))` once every 15 minutes (and at the `09:00` daily transition).

---

## 2. Build Toolchain, Flash Layout & OTA Workflow for a HeishaMon Fork

### 2.1 Build System & Toolchain (`scripts/build_esp32s3.sh` & `.github/workflows/main.yml`)
HeishaMon does **not** use PlatformIO (`platformio.ini` does not exist in the repository). Instead, as verified in [`scripts/build_esp32s3.sh:1-5`](https://github.com/heishamon/HeishaMon/blob/main/scripts/build_esp32s3.sh#L1-L5), [`.github/workflows/main.yml:52-84`](https://github.com/heishamon/HeishaMon/blob/main/.github/workflows/main.yml#L52-L84), and [`AGENTS.md:19-30`](https://github.com/heishamon/HeishaMon/blob/main/AGENTS.md#L19-L30), it uses **`arduino-cli`**:

- **Board Core:** `esp32:esp32@3.3.11` in CI ([`.github/workflows/main.yml:59`](https://github.com/heishamon/HeishaMon/blob/main/.github/workflows/main.yml#L59); `AGENTS.md:20` mentions `3.0.7` in the prebuilt `ghcr.io/the78mole/heishamon-dev` devcontainer).
- **Exact Compile Command ([`scripts/build_esp32s3.sh:3-5`](https://github.com/heishamon/HeishaMon/blob/main/scripts/build_esp32s3.sh#L3-L5)):**
  ```bash
  cd HeishaMon
  arduino-cli compile --output-dir . \
    --fqbn=esp32:esp32:esp32s3:CDCOnBoot=cdc,PSRAM=enabled,PartitionScheme=min_spiffs \
    --warnings=none --verbose HeishaMon.ino
  ```
- **Required Libraries ([`.github/workflows/main.yml:62`](https://github.com/heishamon/HeishaMon/blob/main/.github/workflows/main.yml#L62)):**
  ```bash
  arduino-cli lib install ringbuffer pubsubclient arduinojson dallastemperature onewire "Adafruit NeoPixel"
  ```
- **Firmware Binary & MD5 Naming Convention ([`.github/workflows/main.yml:78-84`](https://github.com/heishamon/HeishaMon/blob/main/.github/workflows/main.yml#L78-L84) & [`HeishaMon/htmlcode.h:2339-2343`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/htmlcode.h#L2339-L2343)):**
  CI renames `HeishaMon.ino.bin` to `HeishaMon_ESP32-<BuildType>-<md5>.bin`. On the `/firmware` Web UI page (`htmlcode.h:2339-2343`), the browser JavaScript `getMD5()` splits the filename on `'-'` (`fn.split('-')[2].split('.')[0]`) and auto-fills the 32-character MD5 input box if the filename has at least two hyphens followed by the MD5 hash!

### 2.2 ESP32-S3 Flash Partition Layout (`PartitionScheme=min_spiffs`)
On the HeishaMon Large (`ESP32-S3-MINI-1-N4R2`, 4 MB Flash, 2 MB Quad PSRAM), `PartitionScheme=min_spiffs` uses the standard Arduino-ESP32 `min_spiffs.csv` partition table at offset `0x8000`:

| Partition Name | Type / Subtype | Flash Offset | Size | Usage in HeishaMon Large |
| :--- | :--- | :--- | :--- | :--- |
| `nvs` | `data` / `nvs` | `0x009000` | `0x005000` (20 KB) | ESP-IDF NVS (Wi-Fi PHY + available for `<Preferences.h>` in a fork or ESPHome `restore_value: true`) |
| `otadata` | `data` / `ota` | `0x00E000` | `0x002000` (8 KB) | Tracks active OTA boot slot (`ota_0` vs `ota_1`) |
| `app0` | `app` / `ota_0` | `0x010000` | `0x1E0000` (1,920 KB) | Firmware slot A (~1.875 MB max binary size) |
| `app1` | `app` / `ota_1` | `0x1F0000` | `0x1E0000` (1,920 KB) | Firmware slot B (~1.875 MB max binary size) |
| `spiffs` | `data` / `spiffs` | `0x3D0000` | `0x030000` (192 KB) | Mounted by `LittleFS.begin()` (`HeishaMon.ino:1822`); stores `/config.json`, `/dallas.json`, `/heishamon`, `/rules.txt`, `/optionalpcb.raw`, `/ca.pem` |
| `coredump` | `data` / `coredump` | `0x3F0000` | `0x010000` (64 KB) | Crash core dump region |

### 2.3 First-Time Flashing & Ongoing OTA Update Experience: HeishaMon Fork vs. ESPHome

#### How HeishaMon's Web UI `/firmware` OTA Works (`HeishaMon.ino:1143-1159, 1249-1276, 1422-1435`)
1. When `POST /firmware` is requested, `webserver_cb()` calls `Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)` ([`HeishaMon.ino:1148`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1148)).
2. As multipart form arguments stream in (`WEBSERVER_CLIENT_ARGS`, [`HeishaMon.ino:1249-1276`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1249-L1276)):
   - `args->name == "firmware"` writes raw chunks to the inactive OTA partition (`app0` or `app1`) via `Update.write((uint8_t *)args->value, args->len)`.
   - `args->name == "md5"` (if non-empty) sets the expected MD5 checksum via `Update.setMD5(md5)`.
3. On request completion (`WEBSERVER_CLIENT_WRITE`, [`HeishaMon.ino:1422-1435`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1422-L1435)), `Update.end(true)` verifies the `0xE9` magic byte, image checksum, and MD5, updates `otadata` (`0xE000`) to boot the new slot, and schedules a reboot in 2 seconds (`timerqueue_insert(2, 0, -2)`).
4. In addition to Web UI `/firmware`, HeishaMon also runs **`ArduinoOTA` on UDP/TCP port `8266`** ([`HeishaMon.ino:1042-1063`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1042-L1063)) authenticated with `heishamonSettings.ota_password` (default `"heisha"`).

#### Comparison of Flashing Scenarios (Without Opening the Enclosure / No USB-C)

| Flashing Aspect | Forked HeishaMon (`HeishaMon.ino.bin`) | ESPHome (`firmware.ota.bin`) |
| :--- | :--- | :--- |
| **Can First Flash Be Done Over Web UI `/firmware` (No USB-C)?** | **Yes — 100% native.** Upload `HeishaMon_ESP32-Meisha-<md5>.bin` via `http://<ip>/firmware` (or `curl -F "firmware=@..." -F "md5=..." http://<ip>/firmware`). | **Yes, with caveats.** ESPHome's `firmware.ota.bin` (starts with `0xE9`) is accepted by `Update.begin()` and written to the inactive `min_spiffs` OTA slot (`1,920 KB`) without touching the bootloader or partition table at `0x8000`. |
| **What Happens to Existing Wi-Fi, MQTT & 1-Wire Config on First Flash?** | **100% Preserved Automatically.** `Update.begin()` only writes `app0`/`app1`. The `LittleFS` partition (`0x3D0000`) is untouched, so `/config.json` (Wi-Fi, MQTT credentials, 1-Wire settings), `/dallas.json` (sensor aliases), and `/heishamon` work immediately on boot with zero reconfiguration. | **Ignored by ESPHome.** ESPHome does not read HeishaMon's `/config.json` on LittleFS; all Wi-Fi credentials, static IPs, MQTT/API keys, and 1-Wire addresses must be compiled into the ESPHome YAML before the first flash. |
| **Risk of Bricking / Needing Physical USB-C Recovery** | **Near Zero.** Uses the exact same Wi-Fi/Ethernet/HTTP stack and existing `/config.json` that is already working on the device. | **Moderate on First Cross-Flash.** If the ESPHome YAML has a Wi-Fi/network misconfiguration or boot-loops on a pin conflict, and cannot join Wi-Fi (or its fallback AP isn't reachable), recovery requires physically opening the heat pump / HeishaMon enclosure and connecting a USB-C cable (`GPIO19/20`). |
| **Rollback to Stock HeishaMon** | **Trivial (10 seconds via `/firmware`).** Upload stock `HeishaMon.ino.esp32-v3.9.bin` to `/firmware`; all `/config.json` settings are still intact. | Requires enabling `web_server:` (with OTA) in ESPHome to upload stock `HeishaMon.ino.esp32.bin` back over HTTP (since LittleFS at `0x3D0000` is unmounted by ESPHome, `/config.json` survives an OTA round-trip as long as ESPHome was flashed via OTA and not USB factory flash). |

---

## 3. Head-to-Head Trade-Off Comparison for the Grilling Session

| Evaluation Criterion | (1) Fork HeishaMon + Rules Engine Patches | (2) Fork HeishaMon + Native C++ Module (`meisha.cpp`) | (3) Migrate to ESPHome (`ElVit/esphome_components`) |
| :--- | :--- | :--- | :--- |
| **1. Implementation Effort & Complexity** | **High & Frustrating.** ~250 LoC C++ patches in `rules.cpp`/`HeishaMon.ino` (fixing `global_varstack` pointer ownership, `parsing == 1` dry-run clobbering, MQTT pub/sub, NVS save/load) **plus** ~600+ LoC of fragile `/rules.txt` DSL working around the 127-variable limit and VM bugs. | **Low–Moderate & Clean.** **4 lines** of hooks in `HeishaMon.ino` + ~550–650 LoC of standard C++ in `HeishaMon/meisha.cpp` & `meisha.h` implementing the exact equations, ring buffers, PID, `for:` timers, NVS persistence, and MQTT Discovery. | **Low–Moderate & Declarative.** ~500–700 lines of ESPHome YAML (with embedded C++ lambdas for the curve, ring buffers, degree-hour integral, PID, and state machine) on top of `ElVit/esphome_components`. |
| **2. First-Time Flashing & Ongoing OTA Workflow** | **Best.** Flash directly via HeishaMon Web UI `/firmware` or `curl`; keeps `/config.json` Wi-Fi/MQTT settings; ~25s incremental `arduino-cli` compile. | **Best.** Flash directly via HeishaMon Web UI `/firmware` or `curl`; keeps `/config.json` Wi-Fi/MQTT settings; ~25s incremental `arduino-cli` compile (or GitHub Actions fork build). | **Good once installed; higher first-flash risk.** Can cross-flash `firmware.ota.bin` via `/firmware`, then use `esphome run` OTA afterward (~30–60s incremental ESP-IDF compile). First cross-flash carries USB-C recovery risk if Wi-Fi config fails. |
| **3. `kamaradclimber/heishamon-homeassistant` & HA History Compatibility** | **100% Zero-Touch.** All existing `panasonic_heat_pump/main/...`, `1wire/...`, `stats`, and `LWT` MQTT topics and HA entities remain completely untouched. | **100% Zero-Touch.** All existing `panasonic_heat_pump/main/...` & `1wire/...` topics, `kamaradclimber/heishamon-homeassistant` entities, COP/energy utility meters (§8), and HeishaMon Web UI remain untouched. | **Requires Explicit Mapping.** Must either configure ESPHome `mqtt:` to republish all `panasonic_heat_pump/main/<Topic>` + `1wire/<rom>` + `stats` topics for `kamaradclimber`, OR migrate HA entities to native ESPHome API with matching `entity_id`s. |
| **4. External HA Inputs (`garage_dew_point`, `forecast_24h`)** | Needs a small HA automation to mirror `sensor.garage_dew_point` & `weather_forecast_home_temperature_24h` to MQTT (`panasonic_heat_pump/rules/set/...`). | Needs a small HA automation (or direct MQTT subscription if the sensor is already on MQTT) to publish `garage_dew_point` & `forecast_24h` to `panasonic_heat_pump/meisha/in/...`. | **Native Zero-Touch.** `platform: homeassistant` imports `sensor.garage_dew_point` and `sensor.weather_forecast_home_temperature_24h` directly over the native ESPHome API. |
| **5. Expressiveness & Maintainability of Control Logic** | **Poor.** 127-variable hard ceiling (`src/rules/rules.cpp:5414`), no arrays/loops, no structs, 27-bit float truncation (`float32to27`), `NULL` in `&&` bug, `@Set` drop bug, manual `setTimer` numbering. | **Excellent.** Full C++17 (`struct`, `enum class`, fixed-size ring buffers `float[96]`, standard IEEE-754 `float`/`double`, reusable `DurationCondition` helper class for `for:` guards, explicit command pacing queue). | **Very Good.** Native `on_time:` cron triggers (`08:00`, `09:00`, `14:00`, `20:00`), `interval:` components, `number`/`switch` entities with `restore_value: true`, and C++ lambdas. |
| **6. Off-Device Unit Testability on Host PC** | Partial — `Examples/Rules/harness/` compiles `src/rules/` on Linux, but `harness.cpp` mocks `HeishaMon/rules.cpp` and would have to be separately updated for any C++ patches. | **Best.** `meisha.cpp` is pure C++ logic that can be compiled natively on Linux with `g++` in `<1 second` and tested with deterministic unit/scenario tests covering all 24h heating, DHW, cooling, and FM transitions. | Harder to unit-test standalone unless all control logic is extracted out of YAML lambdas into a standalone C++ header (`meisha_controller.h`) included via `esphome: includes:`. |
| **7. Upstream Maintenance & Reliability** | High friction if upstream `HeishaMon/rules.cpp` changes. | **Minimal friction.** Only a 4-line hook in `HeishaMon.ino`; `meisha.cpp` and `meisha.h` are separate files that never conflict with `git pull --rebase upstream main`. | Managed via standard ESPHome updates and `ElVit/esphome_components` git ref pinning; note `ElVit` does not include HeishaMon's built-in CZ-TAW1 WebSocket hex/topic inspector UI unless configured via ESPHome `web_server`. |

---

## 4. Concrete Architecture Blueprint for Option 2 (`meisha.cpp` in a HeishaMon Fork)

For comparison against ESPHome in the upcoming design review, here is how clean Option 2 (`meisha.cpp` inside a HeishaMon fork) is in practice:

1. **Files Modified in Upstream `HeishaMon/`:**
   - [`HeishaMon/HeishaMon.ino`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino): 4 single-line hooks (`#include "meisha.h"`, `meisha_setup()`, `meisha_loop()`, `meisha_mqtt_connected()`, `meisha_mqtt_callback()`).
2. **Files Added (`HeishaMon/meisha.h` & `HeishaMon/meisha.cpp`):**
   - **Config & NVS (`<Preferences.h>`):**
     - `struct MeishaConfig`: holds the 8 tunable parameters (`room_target = 23.0f`, `curve_start = 14.0f`, `curve_angle = 0.8f`, `fixed_target = 25.0f`, `start_hour = 9`, `max_outlet_temp = 33.0f`, `cooling_mode = false`, `high_power_override = false`). Loaded from `Preferences` namespace `"meisha_cfg"` on boot, updated on MQTT `<base>/meisha/set/<param>`, and auto-discovered in Home Assistant via `homeassistant/number/meisha_.../config` and `homeassistant/switch/meisha_.../config`.
     - `struct MeishaState`: holds `pid_integral` (`-1.0..+1.0`), `degree_hours_day`, `dhw_time_hours`, `room_temp_snapshot`, `forecast_snapshot`, `ambient_24h_ring[96]` (15-min samples), `room_6h_ring[72]` (5-min samples), `ambient_1h_ring[60]` (1-min samples), and `last_day_reset_yday`. Saved to `Preferences` namespace `"meisha_st"` every 15 minutes and at `09:00`.
   - **Duration Tracker Helper (`struct HoldTimer`):**
     - A 10-line C++ struct `bool held_for(bool cond, uint32_t duration_ms, uint32_t now_ms)` that cleanly implements every `for:` condition (`valve == Room for 10m`, `compressor > 10Hz for 5m`, `setpoint == 25 for 4h`, `no defrost for 2h`, etc.) using `millis()`.
   - **Paced Command Reconciler:**
     - Queues desired heat pump changes (`SetHeatpump`, `SetOperationMode`, `SetZ1HeatRequestTemperature`, `SetQuietMode`) with a configurable inter-command delay (e.g., `10s` between `SetOperationMode` and `SetHeatpump` as required by §4.6 and §5.1) and verifies read-back state in `actData` (`TOP0`, `TOP4`, `TOP18`, `TOP27`).
   - **Host-Side Testability:**
     - Because `meisha.cpp` interacts with HeishaMon through a tiny interface (`read_top()`, `read_dallas()`, `issue_hp_command()`, `publish_mqtt()`, `now_ms()`, `local_time()`), the exact same `meisha.cpp` compiles on Linux with `g++` for instant unit and full-day simulation testing.

---

## 5. Primary Source Citations (`heishamon/HeishaMon`)

- **Build & CI Configuration:**
  - [`scripts/build_esp32s3.sh:1-5`](https://github.com/heishamon/HeishaMon/blob/main/scripts/build_esp32s3.sh#L1-L5) (`arduino-cli compile --fqbn=esp32:esp32:esp32s3:CDCOnBoot=cdc,PSRAM=enabled,PartitionScheme=min_spiffs`)
  - [`.github/workflows/main.yml:52-85`](https://github.com/heishamon/HeishaMon/blob/main/.github/workflows/main.yml#L52-L85) (`esp32:esp32@3.3.11`, library dependencies, MD5 filename suffix convention)
  - [`AGENTS.md:11-79`](https://github.com/heishamon/HeishaMon/blob/main/AGENTS.md#L11-L79) & [`LIBSUSED.md:10-16`](https://github.com/heishamon/HeishaMon/blob/main/LIBSUSED.md#L10-L16)
- **Main Sketch, FreeRTOS Tasks, MQTT & Web UI OTA (`HeishaMon/HeishaMon.ino`):**
  - [`HeishaMon/HeishaMon.ino:11-27, 101-116`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L11-L27) (ESP32-S3 pins, `actData`, `actDataExtra`, `actOptData`, FreeRTOS queues `pcbQueue`/`cmdQueue`/`logQueue`)
  - [`HeishaMon/HeishaMon.ino:488-537`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L488-L537) (`mqtt_reconnect()` subscriptions)
  - [`HeishaMon/HeishaMon.ino:846-941`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L846-L941) (`serialTXTask()` and `send_command()` via `cmdQueue`)
  - [`HeishaMon/HeishaMon.ino:971-1040`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L971-L1040) (`mqtt_callback()` topic dispatch)
  - [`HeishaMon/HeishaMon.ino:1143-1159, 1249-1276, 1422-1435`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1143-L1159) (`/firmware` HTTP POST OTA via `Update.begin()`, `Update.setMD5()`, `Update.write()`, `Update.end(true)`)
  - [`HeishaMon/HeishaMon.ino:1666-1705`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1666-L1705) (`setupMqtt()` 1024-byte buffer; `setupConditionals()` `serialTXTask` creation)
  - [`HeishaMon/HeishaMon.ino:1755-1771`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1755-L1771) (`timer_cb(-4)` `/rules.new` validation and `rules_boot()` invocation)
  - [`HeishaMon/HeishaMon.ino:1818-1957`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L1818-L1957) (`setup()`, LittleFS `/heishamon` & `/config.json` check, `esp_reset_reason()` crash/brownout rule-disable check at lines `1930-1938`)
  - [`HeishaMon/HeishaMon.ino:2013-2246`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/HeishaMon.ino#L2013-L2246) (`loop()` structure)
- **Decode, Commands, 1-Wire & Web Functions:**
  - [`HeishaMon/decode.h:18-23, 53-145`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/decode.h#L18-L23) & [`HeishaMon/decode.cpp:158-428`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/decode.cpp#L158-L428) (`getDataValue()`, `getDataValueExtra()`, `decode_heatpump_data()`)
  - [`HeishaMon/commands.h:32-208`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/commands.h#L32-L208) & [`HeishaMon/commands.cpp:1-300, 1288-1345`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/commands.cpp#L1288-L1345) (`commands[]`, `send_heatpump_command()`)
  - [`HeishaMon/dallas.h:1-35`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/dallas.h#L1-L35) & [`HeishaMon/dallas.cpp:20-199`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/dallas.cpp#L20-L199) (`actDallasData`, `dallasLoop()`)
  - [`HeishaMon/webfunctions.h:33-75`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/webfunctions.h#L33-L75), [`HeishaMon/webfunctions.cpp:210-304, 1346-1461`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/webfunctions.cpp#L210-L304), & [`HeishaMon/htmlcode.h:2326-2409`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/htmlcode.h#L2326-L2409) (`settingsStruct`, `/config.json`, `/firmware` UI and `getMD5()`)
- **Rules Engine Glue & VM Internals:**
  - [`HeishaMon/rules.cpp:63-120, 249-364, 623-806, 954-1041`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/rules.cpp#L63-L120) (`global_varstack`, commented-out `readRuleFromFS`/`writeRuleToFS` bytecode stubs, `is_event()`, `vm_value_set()`, `rules_ref()`, double wipe of `global_varstack` in `rules_parse()`)
  - [`HeishaMon/src/rules/rules.cpp:221-227, 521-625, 1525-1595, 5292-5672`](https://github.com/heishamon/HeishaMon/blob/main/HeishaMon/src/rules/rules.cpp#L221-L227) (`float32to27()`, `varstack_add()`, `rules_ref()`, `rule_initialize()` hard 127-variable `INT8_MAX` limit at lines `5413-5420`, and dry-run `rule_run(obj, 1)` at line `5618`)
  - [`Examples/Rules/SKILL.md:1-80`](https://github.com/heishamon/HeishaMon/blob/main/Examples/Rules/SKILL.md#L1-L80) & [`Examples/Rules/harness/README.md:1-59`](https://github.com/heishamon/HeishaMon/blob/main/Examples/Rules/harness/README.md#L1-L59)
