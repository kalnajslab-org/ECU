# LoRa TX Suspend — Design Plan

## Problem

At cold start, periodic LoRa transmissions from the ECU can desense/overwhelm
the onboard GPS receiver when it is using a passive antenna, delaying or
preventing GPS acquisition. LoRa TX needs to be held off during the window
when the GPS is most vulnerable (right after boot, before it has a fix), and
that quiet window needs to be commandable from the ground as well, not just a
fixed boot-time behavior.

## Requirements

1. On boot, LoRa **transmissions** are disabled until either 2 minutes have
   elapsed, or `gps_valid` becomes true — whichever happens first.
2. A LoRa command can be sent to the ECU to suspend transmissions for a
   specified number of seconds, or until `gps_valid` is true — whichever
   happens first.
3. LoRa **reception** is always active, and the ECU always processes and
   responds to commands. If a received command calls for a LoRa reply, that
   reply is sent even while transmissions are otherwise suspended.

## Current behavior (for reference)

- [ECU.cpp](../src/ECU.cpp) `loop()` is a flat, non-blocking cooperative loop
  (fed by the watchdog each iteration). There is no RTOS and no state
  machine.
- `process_lora()` ([ECU_Lib.cpp:111](../src/ECU_Lib.cpp#L111)) is called
  every loop iteration and unconditionally decodes any received LoRa command
  (`tempC`, `rs41Regen`, `rs41Enable`, `tsenPower`, `rs41Metadata`). This
  already satisfies "reception always active" — no change needed there.
- Two places call `ecu_lora_tx()`:
  - The periodic ECU report, gated by `sample_timer > SAMPLE_MILLIS &&
    lora_tx_timer > LORA_MIN_TX_MILLIS` ([ECU.cpp:174](../src/ECU.cpp#L174)).
  - The RS41 metadata report, sent as a direct reply when a `rs41Metadata`
    command was received, gated only by `lora_tx_timer > LORA_MIN_TX_MILLIS`
    ([ECU.cpp:188](../src/ECU.cpp#L188)).
- GPS validity is currently only visible as `ecu_report.gps_valid`, which is
  reset to `0` every loop by `ecu_report_init()` and only set when a new NMEA
  sentence is decoded that loop iteration. It is not a reliable persistent
  signal. `ecu_gps.location.isValid()` (TinyGPSPlus) is the persistent,
  authoritative source of GPS validity and should be used directly instead.
- `LORA_MIN_TX_MILLIS`/`SAMPLE_MILLIS`/`PRINT_MILLIS` follow a
  `#define <PURPOSE>_MILLIS` + `elapsedMillis` convention
  ([ECU.cpp:9-20](../src/ECU.cpp#L9-L20)); new timing constants should match.

## Design

### New state: `LoraTxSuspend_t`

The boot-time suspend and the commanded suspend have different end
conditions: boot-time suspend is cut short by GPS acquiring a fix (per
requirement 1); a commanded suspend is an unconditional quiet period for the
requested duration — it forces quiet time even if GPS already has a fix, and
is not cut short if GPS acquires one mid-suspend. A `gps_override_enabled`
flag distinguishes the two.

Add to [ECU_Lib.h](../src/ECU_Lib.h):

```cpp
// The ECU boots with LoRa TX suspended for this long, to protect GPS
// acquisition on a passive antenna. Ends early if GPS gets a fix first.
#define LORA_BOOT_SUSPEND_MS (2UL * 60UL * 1000UL)  // 2 minutes

struct LoraTxSuspend_t {
    bool active = true;                        // starts suspended at boot
    elapsedMillis timer;
    uint32_t duration_ms = LORA_BOOT_SUSPEND_MS;
    bool gps_override_enabled = true;          // boot suspend: GPS can end it early
};

// Clears `active` once the duration has elapsed, or (if gps_override_enabled)
// once GPS has a valid fix. Call once per loop iteration.
void update_lora_tx_suspend(LoraTxSuspend_t& suspend, bool gps_valid);
```

`active` is a latch, not a live recomputation: once cleared it stays cleared
until a new suspend command arrives. This avoids flapping if GPS briefly
loses its fix later in flight.

Declared as a global in [ECU.cpp](../src/ECU.cpp) alongside the other timers:

```cpp
LoraTxSuspend_t lora_tx_suspend;
```

Because `elapsedMillis` starts counting from construction, the boot-time
2-minute suspend requires no explicit setup code — it's active as soon as the
global is constructed.

### Updating suspend state each loop

In `ECU_Lib.cpp`:

```cpp
void update_lora_tx_suspend(LoraTxSuspend_t& suspend, bool gps_valid)
{
    bool duration_elapsed = suspend.timer >= suspend.duration_ms;
    bool gps_cleared = suspend.gps_override_enabled && gps_valid;
    if (suspend.active && (duration_elapsed || gps_cleared))
    {
        suspend.active = false;
    }
}
```

Called in `loop()` right after the GPS block, using the persistent
`ecu_gps.location.isValid()` rather than `ecu_report.gps_valid`:

```cpp
update_lora_tx_suspend(lora_tx_suspend, ecu_gps.location.isValid());
```

### Gating transmissions

Only the **periodic** ECU report is gated. The RS41-metadata reply is a
direct response to a received command and must go out regardless (per
requirement 3):

```cpp
if (!lora_tx_suspend.active && !rs41_metadata_requested &&
    sample_timer > SAMPLE_MILLIS && lora_tx_timer > LORA_MIN_TX_MILLIS)
{
    ...existing ECU report send...
}

if (rs41_metadata_requested && lora_tx_timer > LORA_MIN_TX_MILLIS)
{
    ...existing RS41 metadata reply send, unchanged...
}
```

If future commands add other direct replies, they should follow the
metadata-reply pattern (ungated by `lora_tx_suspend.active`), not the
periodic-report pattern.

### New command: `loraSuspendSec`

Extend `process_lora()`'s signature to take the suspend state by reference,
alongside the existing `tempC_setpoint`/`rs41_metadata_requested` out-params:

```cpp
void process_lora(float& tempC_setpoint, RS41& rs41,
                   bool& rs41_metadata_requested,
                   LoraTxSuspend_t& lora_tx_suspend);
```

Handle the new JSON key the same way the other command keys are handled
([ECU_Lib.cpp:138](../src/ECU_Lib.cpp#L138) area):

```cpp
if (ecu_json_doc.containsKey("loraSuspendSec")) {
    int suspend_sec = ecu_json_doc["loraSuspendSec"] | -1;
    if (suspend_sec >= 0) {
        lora_tx_suspend.active = true;
        lora_tx_suspend.timer = 0;
        lora_tx_suspend.duration_ms = (uint32_t)suspend_sec * 1000UL;
        lora_tx_suspend.gps_override_enabled = false;  // unconditional
        Serial.println("LoRa TX suspended for " + String(suspend_sec) + " s");
    }
}
```

Ground command example: `{"loraSuspendSec": 90}` unconditionally suspends
periodic reports for 90 seconds, regardless of GPS validity — even if GPS
already has a fix, and even if it acquires one mid-suspend.

**Cancelling a suspend**: `{"loraSuspendSec": 0}` sets `duration_ms = 0`,
so on the very next loop iteration `timer >= duration_ms` is immediately
true and `active` clears. This works out to a "resume transmissions now"
command — it cancels the boot-time suspend or a previous `loraSuspendSec`
suspend within about one loop iteration. This is intentional and should be
documented for ground-station operators as the way to cancel a suspend
early, rather than left as an undocumented side effect.

## Files touched

| File | Change |
|---|---|
| [ECU_Lib.h](../src/ECU_Lib.h) | Add `LORA_BOOT_SUSPEND_MS`, `LoraTxSuspend_t`, `update_lora_tx_suspend()` decl; extend `process_lora()` signature |
| [ECU_Lib.cpp](../src/ECU_Lib.cpp) | Implement `update_lora_tx_suspend()`; handle `loraSuspendSec` in `process_lora()` |
| [ECU.cpp](../src/ECU.cpp) | Declare `lora_tx_suspend` global; call `update_lora_tx_suspend()` each loop; gate the periodic-report TX block; pass `lora_tx_suspend` into `process_lora()` |

No changes are needed in the vendored `ECULoRa` library
(`.pio/libdeps/ecu/ECUComm`) — it's pulled from an external git repo
(`platformio.ini`), and this feature is purely an application-level TX gate
sitting in front of the existing `ecu_lora_tx()` calls. RX is already
unconditional, so requirement 3's "reception always active" needs no code
change beyond confirming nothing new gates the `process_lora()` call itself.

## Open questions

1. **Should there be a maximum commanded quiet length, and if so what should
   it be?** As designed, `loraSuspendSec` accepts any non-negative value
   with no upper bound, so a mistaken or malicious command (e.g. a typo'd
   extra digit) could silence periodic telemetry for an arbitrarily long
   time with no GPS-fix override to fall back on. Worth clamping
   `duration_ms` to a `LORA_MAX_SUSPEND_SEC` ceiling? If so, what's a
   reasonable max (minutes? one orbit/pass interval? something else tied to
   ground-contact cadence)?
2. **Command field name**: `loraSuspendSec` — confirm this matches (or should
   match) the ground-station command schema/naming convention.
3. **Acknowledgment**: proposed to only `Serial.println()` locally, matching
   how every other command in `process_lora()` behaves (no LoRa ack). Confirm
   that's sufficient, vs. wanting a LoRa reply confirming the command was
   received/applied.

## Test plan

- Cold boot with no GPS antenna connected (or in a location without a fix):
  confirm no periodic ECU-report LoRa transmissions occur for ~2 minutes,
  and that RX still works (e.g. send a `tempC` command and confirm via
  serial log that the setpoint is applied while TX is suspended).
- Cold boot with GPS fix acquired quickly: confirm periodic transmissions
  resume as soon as `ecu_gps.location.isValid()` becomes true, without
  waiting the full 2 minutes.
- Mid-flight, with GPS already valid: send `{"loraSuspendSec": 30}` and
  confirm periodic reports still stop unconditionally for the full 30
  seconds (i.e. an existing GPS fix does not short-circuit a commanded
  suspend).
- While suspended: send `{"rs41Metadata": true}` and confirm the metadata
  reply is still transmitted.
- While suspended (boot-time or commanded): send `{"loraSuspendSec": 0}` and
  confirm periodic transmissions resume within about one loop iteration.
- Confirm no `delay()`/blocking calls were introduced — the watchdog timing
  and cooperative loop structure must be unaffected.

**GPS dropout simulation**: a metal can (tin/steel) domed fully over the GPS
antenna, with no sky visible through gaps, acts as a rough Faraday cage and
is sufficient to force a fix loss for these tests — no need to physically
unplug the antenna. Note that some GPS modules hold their last fix for a few
seconds (dead-reckoning/estimated status) before the NMEA stream actually
reports invalid, so `ecu_gps.location.isValid()` may lag a few seconds behind
the can going on; account for that when timing the test rather than assuming
an instant transition.
