# Smart Temp Sensor

Software-only temperature monitor for the Arduino Uno (ATmega328P). The sketch samples a simulated indoor temperature every 2 seconds, rejects impossible readings, smooths noise, and reports comfort, a 1-minute trend, and sensor faults as JSON on `Serial`.

No physical sensor, no GPIO, and no third-party libraries. `ScenarioSimulator::readCelsius()` is the only temperature source. Replace that class with a real driver later and keep the same method signature.

## Requirements

- Arduino IDE or Arduino CLI
- Board package: **Arduino AVR Boards**
- Board: **Arduino Uno**
- Serial Monitor: **9600 baud**

## Build

1. Open `smart-temp-sensor/smart-temp-sensor.ino`.
2. Select **Tools → Board → Arduino Uno**.
3. Upload. The sketch also runs in the simulator view of the IDE; it does not need a sensor attached.

Measured size on Arduino AVR Boards 1.8.8:

| Resource | Used | Limit | Share |
|---|---:|---:|---:|
| Flash | 6954 B | 32256 B | 21% |
| SRAM (globals) | 534 B | 2048 B | 26% |

About 1514 bytes of SRAM remain for the stack and local variables. Allocation is static: the sketch does not use `String`, `new`, or `malloc`.

## Architecture

| Layer | Role |
|---|---|
| `SampleAcquisition` | Non-blocking 2 s clock (`millis()`). Accepts a sample only if it is finite and inside −20 °C to 60 °C. |
| `MovingAverageFilter` | Circular buffer of the last 5 valid samples. |
| `TrendAndEventDetector` | 60 s sliding window, `delta_1min`, stuck-sensor fault. |
| `ComfortClassifier` | `COLD` / `COMFORTABLE` / `WARM` with 0.5 °C hysteresis. |
| `ScenarioSimulator` | Deterministic room profile. |
| `TelemetryFormatter` | One JSON line per sample. Floats are printed with `dtostrf` because AVR `snprintf` has no `%f`. |

`loop()` never calls `delay()`. The sample period uses unsigned subtraction (`now - lastSampleMs >= interval`), so the `millis()` wrap at about 49.7 days does not stall the sampler. One JSON line takes roughly 200 ms to leave the UART at 9600 baud, which fits inside the 2 s period.

## Telemetry

```json
{"t_ms":120000,"raw_c":21.04,"filtered_c":21.02,"delta_1min_c":-1.46,"comfort":"COMFORTABLE","event":"NORMAL_TREND","fault_stuck":false,"fault_invalid_sample":false,"reject":"none"}
```

| Field | Meaning |
|---|---|
| `t_ms` | Milliseconds since boot (unsigned; wrap-safe). |
| `raw_c` | Simulated reading. `null` when the value is `NaN` or `Inf`. |
| `filtered_c` | Moving average. `null` until the first valid sample. |
| `delta_1min_c` | Filtered temperature minus the value from about 60 s earlier. `null` during warm-up or after a long gap. |
| `comfort` | `UNKNOWN`, `COLD`, `COMFORTABLE`, or `WARM`. |
| `event` | See the event table below. |
| `fault_stuck` | `true` after 8 consecutive raw samples within 0.002 °C. |
| `fault_invalid_sample` | `true` when this sample was rejected. |
| `reject` | `none`, `nan`, `inf`, or `range`. |

Rejected samples are printed and then discarded. They do not enter the moving average, the 60 s history, or the comfort state.

### Events

| Event | When |
|---|---|
| `WARMING_UP_60S` | No stored sample is 60 s old yet. The sketch does not invent a 1-minute delta. |
| `NORMAL_TREND` | Window is valid and no fault or rapid trend is active. |
| `WINDOW_OPEN_DETECTED` | Filtered temperature fell by at least 1.5 °C within 60 s. |
| `RAPID_HEATING_DETECTED` | Filtered temperature rose by at least 2.5 °C within 60 s. |
| `SENSOR_STUCK_FAULT` | Frozen raw value. This overrides the rate events once the 60 s window is valid, because the slope would be computed from a dead reading. |

### Comfort

| State | Enter | Leave |
|---|---|---|
| `COLD` | below 19.0 °C | at or above 19.5 °C |
| `COMFORTABLE` | between the release thresholds | — |
| `WARM` | above 25.0 °C | at or below 24.5 °C |

The first valid sample is classified on the plain thresholds. Hysteresis applies after that, so the label does not chatter on 19.0 °C or 25.0 °C.

The 5-sample average and the 60 s warm-up are different. The filter publishes a partial mean immediately. `delta_1min_c` stays `null` until a full minute of history exists.

## Simulation timeline

The profile is a function of the sample index, so a run is the same every time. Samples are 2 s apart. A full pass takes about 11 minutes.

| Time | Room | What to observe |
|---|---|---|
| 10–16 s | ~22.5 °C | Four rejected samples: `nan`, `inf`, 85 °C, −40 °C. Filter and comfort do not move. |
| 0–60 s | ~22.5 °C | `WARMING_UP_60S`, comfort `COMFORTABLE`. |
| 60–90 s | ~22.5 °C | `NORMAL_TREND`. |
| 90–150 s | 22.5 → 20.3 °C (−2.2 °C) | `WINDOW_OPEN_DETECTED`. Still comfortable; 20.3 °C is above 19 °C. |
| 150–180 s | 20.3 °C | Window event clears once the drop leaves the 60 s window. |
| 180–210 s | 20.3 °C, no noise | `SENSOR_STUCK_FAULT` after 8 identical samples. |
| 210–390 s | 20.3 → 18.4 °C | Slow cool (about −0.6 °C/min), so the window event stays off. Comfort becomes `COLD` below 19.0 °C. |
| 390–450 s | 18.4 → 26.2 °C | `RAPID_HEATING_DETECTED`. Cold releases only at 19.5 °C, then `WARM` above 25.0 °C. |
| 450–630 s | 26.2 → 22.5 °C | Slow recovery, `NORMAL_TREND`. Warm holds until 24.5 °C, then `COMFORTABLE`. |
| after 630 s | ~22.5 °C | Steady comfortable room. |

## Replacing the simulator

Implement a driver with the same contract:

```cpp
float readCelsius(uint32_t sampleIndex) const;
```

`SampleAcquisition` still validates the range. Everything downstream already assumes that invalid readings never reach it.
