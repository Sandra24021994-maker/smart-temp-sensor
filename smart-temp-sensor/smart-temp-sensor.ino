/*
 * Smart temperature sensor — software-only simulation for Arduino Uno.
 *
 * Layers (replace ScenarioSimulator with a real driver later; keep readCelsius()):
 *   SampleAcquisition       non-blocking period + range / NaN / Inf checks
 *   MovingAverageFilter     fixed circular buffer, valid samples only
 *   TrendAndEventDetector   60 s delta, warm-up, stuck fault, trend events
 *   ComfortClassifier       cold / comfortable / warm with hysteresis
 *   ScenarioSimulator       deterministic room profile (no pins, no RNG)
 *   TelemetryFormatter      one JSON object per sample on Serial
 *
 * SRAM stays static: no String, no new/malloc. Timing uses unsigned
 * millis() subtraction so the 49.7-day wrap does not stall the sampler.
 * loop() never calls delay(). Serial at 9600 baud can stall briefly while
 * the 64-byte UART buffer drains (~200 ms per line); the 2 s period absorbs that.
 */

#include <math.h>
#include <stdlib.h>

namespace cfg {

constexpr uint32_t kBaud = 9600;
constexpr uint32_t kSamplePeriodMs = 2000;
constexpr uint32_t kTrendWindowMs = 60000;
// Closest sample at or before now-60s. 32 slots cover 62 s at 2 s
// so that point is still in the buffer when the newest sample arrives.
constexpr uint8_t kTrendCapacity = 32;
constexpr uint32_t kTrendMaxAgeMs = 75000;

constexpr uint8_t kAverageWindow = 5;

constexpr float kMinValidC = -20.0f;
constexpr float kMaxValidC = 60.0f;

constexpr float kWindowOpenDropC = 1.5f;
constexpr float kRapidHeatRiseC = 2.5f;
// Frozen if the raw value does not move by more than the sensor quantum.
// Below the smallest scripted step (ripple 0.01 C, slowest ramp ~0.02 C).
constexpr float kStuckEpsilonC = 0.002f;
constexpr uint8_t kStuckSamples = 8;

constexpr float kColdEnterC = 19.0f;
constexpr float kWarmEnterC = 25.0f;
constexpr float kHysteresisC = 0.5f;

}  // namespace cfg

enum class ComfortState : uint8_t { Unknown, Cold, Comfortable, Warm };

enum class TrendEvent : uint8_t {
  WarmingUp60s,
  NormalTrend,
  WindowOpenDetected,
  RapidHeatingDetected,
  SensorStuckFault
};

enum class RejectReason : uint8_t { None, Nan, Inf, OutOfRange };

// ---------------------------------------------------------------------------
// Sensor stand-in. A real driver replaces this class and keeps readCelsius().
// The profile is a pure function of the sample index so a seminar run is
// repeatable and does not depend on how late loop() woke up.
// ---------------------------------------------------------------------------
class ScenarioSimulator {
 public:
  float readCelsius(uint32_t sampleIndex) const {
    // Local constants, not static members: AVR C++11 ODR-uses those and
    // then fails to link unless they are defined out of line.
    constexpr uint32_t kBaselineEnd = 45;   // 0–90 s at room temperature
    constexpr uint32_t kWindowEnd = 75;     // 90–150 s window drop
    constexpr uint32_t kStuckBegin = 90;    // 150–180 s hold, 180 s stuck
    constexpr uint32_t kStuckEnd = 105;     // 180–210 s frozen reading
    constexpr uint32_t kColdSoakEnd = 195;  // 210–390 s gentle cool
    constexpr uint32_t kHeatEnd = 225;      // 390–450 s heat source
    constexpr uint32_t kRecoverEnd = 315;   // 450–630 s settle

    constexpr float kRoomC = 22.5f;
    constexpr float kPostWindowC = 20.3f;  // 22.5 − 2.2
    constexpr float kColdSoakC = 18.4f;
    constexpr float kPeakHeatC = 26.2f;

    if (sampleIndex == 5) {
      return NAN;
    }
    if (sampleIndex == 6) {
      return static_cast<float>(INFINITY);
    }
    if (sampleIndex == 7) {
      return 85.0f;
    }
    if (sampleIndex == 8) {
      return -40.0f;
    }

    // Exact repeats, no ripple: this is the frozen-sensor case.
    if (sampleIndex >= kStuckBegin && sampleIndex < kStuckEnd) {
      return kPostWindowC;
    }

    float base = kRoomC;
    if (sampleIndex < kBaselineEnd) {
      base = kRoomC;
    } else if (sampleIndex < kWindowEnd) {
      base = lerp(kRoomC, kPostWindowC, sampleIndex, kBaselineEnd, kWindowEnd);
    } else if (sampleIndex < kStuckBegin) {
      base = kPostWindowC;
    } else if (sampleIndex < kColdSoakEnd) {
      base = lerp(kPostWindowC, kColdSoakC, sampleIndex, kStuckEnd, kColdSoakEnd);
    } else if (sampleIndex < kHeatEnd) {
      base = lerp(kColdSoakC, kPeakHeatC, sampleIndex, kColdSoakEnd, kHeatEnd);
    } else if (sampleIndex < kRecoverEnd) {
      base = lerp(kPeakHeatC, kRoomC, sampleIndex, kHeatEnd, kRecoverEnd);
    }

    return base + ripple(sampleIndex);
  }

 private:

  static float lerp(float fromC, float toC, uint32_t index, uint32_t start,
                    uint32_t end) {
    const float span = static_cast<float>(end - start);
    const float t = static_cast<float>(index - start) / span;
    return fromC + (toC - fromC) * t;
  }

  // Small deterministic wobble. Steps are >= 0.01 C so they do not look frozen.
  static float ripple(uint32_t sampleIndex) {
    static const float kRipple[8] = {0.00f,  0.04f, -0.03f, 0.02f,
                                     -0.05f, 0.03f, -0.02f, 0.01f};
    return kRipple[sampleIndex % 8U];
  }
};

// ---------------------------------------------------------------------------
// Clock + sanity gate. Invalid readings are reported and then dropped so
// they cannot enter the average or the 60 s history.
// ---------------------------------------------------------------------------
class SampleAcquisition {
 public:
  void start(uint32_t nowMs) {
    // Unsigned wrap makes the first loop() sample due immediately.
    lastSampleMs_ = nowMs - cfg::kSamplePeriodMs;
    started_ = true;
  }

  bool due(uint32_t nowMs) const {
    return started_ && (nowMs - lastSampleMs_ >= cfg::kSamplePeriodMs);
  }

  void markSampled(uint32_t nowMs) { lastSampleMs_ = nowMs; }

  uint32_t sampleIndex() const { return sampleIndex_; }

  void advance() { ++sampleIndex_; }

  RejectReason assess(float rawC) const {
    // NaN compares false against every range check, so it must be first.
    if (isnan(rawC)) {
      return RejectReason::Nan;
    }
    if (isinf(rawC)) {
      return RejectReason::Inf;
    }
    if (rawC < cfg::kMinValidC || rawC > cfg::kMaxValidC) {
      return RejectReason::OutOfRange;
    }
    return RejectReason::None;
  }

 private:
  uint32_t lastSampleMs_ = 0;
  uint32_t sampleIndex_ = 0;
  bool started_ = false;
};

// ---------------------------------------------------------------------------
// Mean of the last N accepted samples. Order does not matter, so the
// circular buffer is summed in slot order. A partial window is used until
// N valid samples exist; that is not the same as the 60 s trend warm-up.
// ---------------------------------------------------------------------------
class MovingAverageFilter {
 public:
  float push(float sampleC) {
    buffer_[head_] = sampleC;
    head_ = static_cast<uint8_t>((head_ + 1U) % cfg::kAverageWindow);
    if (count_ < cfg::kAverageWindow) {
      ++count_;
    }
    return average();
  }

  uint8_t count() const { return count_; }

 private:
  float average() const {
    float sum = 0.0f;
    for (uint8_t i = 0; i < count_; ++i) {
      sum += buffer_[i];
    }
    return sum / static_cast<float>(count_);
  }

  float buffer_[cfg::kAverageWindow] = {};
  uint8_t head_ = 0;
  uint8_t count_ = 0;
};

struct TrendSnapshot {
  TrendEvent event;
  float deltaC;
  bool deltaValid;
  bool stuckFault;
};

// ---------------------------------------------------------------------------
// 60 s rate-of-change plus a frozen-sensor counter.
// Warm-up wins over trend events until some stored sample is actually
// 60 s old — a short buffer must not invent delta_1min.
// A stuck sensor wins over window / heating once the window is valid,
// because the rate is then a property of a dead reading.
// ---------------------------------------------------------------------------
class TrendAndEventDetector {
 public:
  TrendSnapshot push(uint32_t nowMs, float filteredC, float rawC) {
    pushHistory(nowMs, filteredC);
    updateStuck(rawC);

    TrendSnapshot snap;
    snap.stuckFault = stuckCount_ >= cfg::kStuckSamples;
    snap.deltaValid = delta1Min(nowMs, filteredC, snap.deltaC);
    snap.event = classify(snap);
    return snap;
  }

 private:
  struct Point {
    uint32_t timeMs;
    float filteredC;
  };

  void pushHistory(uint32_t nowMs, float filteredC) {
    history_[head_].timeMs = nowMs;
    history_[head_].filteredC = filteredC;
    head_ = static_cast<uint8_t>((head_ + 1U) % cfg::kTrendCapacity);
    if (count_ < cfg::kTrendCapacity) {
      ++count_;
    }
  }

  void updateStuck(float rawC) {
    if (!haveRaw_) {
      lastRawC_ = rawC;
      stuckCount_ = 1;
      haveRaw_ = true;
      return;
    }
    // Compare with the previous accepted sample, then always advance.
    if (fabsf(rawC - lastRawC_) <= cfg::kStuckEpsilonC) {
      if (stuckCount_ < 255) {
        ++stuckCount_;
      }
    } else {
      stuckCount_ = 1;
    }
    lastRawC_ = rawC;
  }

  bool delta1Min(uint32_t nowMs, float currentC, float& deltaC) const {
    bool found = false;
    uint32_t bestAge = 0xFFFFFFFFUL;
    float pastC = 0.0f;

    for (uint8_t i = 0; i < count_; ++i) {
      const uint32_t age = nowMs - history_[i].timeMs;
      // Newest sample that is already at least one minute old.
      if (age >= cfg::kTrendWindowMs && age < bestAge) {
        bestAge = age;
        pastC = history_[i].filteredC;
        found = true;
      }
    }

    // A long gap (rejected burst, stalled loop) is not a 1-minute delta.
    if (!found || bestAge > cfg::kTrendMaxAgeMs) {
      return false;
    }
    deltaC = currentC - pastC;
    return true;
  }

  TrendEvent classify(const TrendSnapshot& snap) const {
    if (!snap.deltaValid) {
      return TrendEvent::WarmingUp60s;
    }
    if (snap.stuckFault) {
      return TrendEvent::SensorStuckFault;
    }
    if (snap.deltaC <= -cfg::kWindowOpenDropC) {
      return TrendEvent::WindowOpenDetected;
    }
    if (snap.deltaC >= cfg::kRapidHeatRiseC) {
      return TrendEvent::RapidHeatingDetected;
    }
    return TrendEvent::NormalTrend;
  }

  Point history_[cfg::kTrendCapacity] = {};
  uint8_t head_ = 0;
  uint8_t count_ = 0;
  float lastRawC_ = 0.0f;
  uint8_t stuckCount_ = 0;
  bool haveRaw_ = false;
};

// ---------------------------------------------------------------------------
// Comfort band with a dead-zone at each edge.
// Enter cold below 19.0 C, leave only at 19.5 C.
// Enter warm above 25.0 C, leave only at 24.5 C.
// The first valid sample is classified on the raw thresholds; hysteresis
// applies after that so the state cannot chatter on the boundary.
// ---------------------------------------------------------------------------
class ComfortClassifier {
 public:
  ComfortState update(float tempC) {
    if (!initialized_) {
      if (tempC < cfg::kColdEnterC) {
        state_ = ComfortState::Cold;
      } else if (tempC > cfg::kWarmEnterC) {
        state_ = ComfortState::Warm;
      } else {
        state_ = ComfortState::Comfortable;
      }
      initialized_ = true;
      return state_;
    }

    switch (state_) {
      case ComfortState::Cold:
        if (tempC >= cfg::kColdEnterC + cfg::kHysteresisC) {
          state_ = ComfortState::Comfortable;
        }
        break;
      case ComfortState::Warm:
        if (tempC <= cfg::kWarmEnterC - cfg::kHysteresisC) {
          state_ = ComfortState::Comfortable;
        }
        break;
      case ComfortState::Comfortable:
        if (tempC < cfg::kColdEnterC) {
          state_ = ComfortState::Cold;
        } else if (tempC > cfg::kWarmEnterC) {
          state_ = ComfortState::Warm;
        }
        break;
      case ComfortState::Unknown:
        break;
    }
    return state_;
  }

  ComfortState state() const { return state_; }

 private:
  ComfortState state_ = ComfortState::Unknown;
  bool initialized_ = false;
};

struct AnalysisState {
  float filteredC;
  float deltaC;
  bool hasFiltered;
  bool deltaValid;
  ComfortState comfort;
  TrendEvent event;
  bool stuckFault;
};

// ---------------------------------------------------------------------------
// Piecewise Serial prints keep format strings in flash (F) and avoid a
// large stack buffer. dtostrf is required because AVR snprintf has no %f.
// ---------------------------------------------------------------------------
class TelemetryFormatter {
 public:
  void publish(uint32_t elapsedMs, float rawC, bool rawFinite,
               RejectReason reject, const AnalysisState& state) const {
    Serial.print(F("{\"t_ms\":"));
    Serial.print(elapsedMs);

    Serial.print(F(",\"raw_c\":"));
    if (rawFinite) {
      printFixed(rawC);
    } else {
      Serial.print(F("null"));
    }

    Serial.print(F(",\"filtered_c\":"));
    if (state.hasFiltered) {
      printFixed(state.filteredC);
    } else {
      Serial.print(F("null"));
    }

    Serial.print(F(",\"delta_1min_c\":"));
    if (state.deltaValid) {
      printFixed(state.deltaC);
    } else {
      Serial.print(F("null"));
    }

    Serial.print(F(",\"comfort\":\""));
    printComfort(state.comfort);
    Serial.print(F("\",\"event\":\""));
    printEvent(state.event);
    Serial.print(F("\",\"fault_stuck\":"));
    printBool(state.stuckFault);
    Serial.print(F(",\"fault_invalid_sample\":"));
    printBool(reject != RejectReason::None);
    Serial.print(F(",\"reject\":\""));
    printReject(reject);
    Serial.println(F("\"}"));
  }

 private:
  static void printFixed(float value) {
    char buf[14];
    dtostrf(static_cast<double>(value), 1, 2, buf);
    Serial.print(buf);
  }

  static void printBool(bool value) {
    Serial.print(value ? F("true") : F("false"));
  }

  static void printComfort(ComfortState comfort) {
    switch (comfort) {
      case ComfortState::Cold:
        Serial.print(F("COLD"));
        break;
      case ComfortState::Comfortable:
        Serial.print(F("COMFORTABLE"));
        break;
      case ComfortState::Warm:
        Serial.print(F("WARM"));
        break;
      case ComfortState::Unknown:
        Serial.print(F("UNKNOWN"));
        break;
    }
  }

  static void printEvent(TrendEvent event) {
    switch (event) {
      case TrendEvent::WarmingUp60s:
        Serial.print(F("WARMING_UP_60S"));
        break;
      case TrendEvent::NormalTrend:
        Serial.print(F("NORMAL_TREND"));
        break;
      case TrendEvent::WindowOpenDetected:
        Serial.print(F("WINDOW_OPEN_DETECTED"));
        break;
      case TrendEvent::RapidHeatingDetected:
        Serial.print(F("RAPID_HEATING_DETECTED"));
        break;
      case TrendEvent::SensorStuckFault:
        Serial.print(F("SENSOR_STUCK_FAULT"));
        break;
    }
  }

  static void printReject(RejectReason reason) {
    switch (reason) {
      case RejectReason::None:
        Serial.print(F("none"));
        break;
      case RejectReason::Nan:
        Serial.print(F("nan"));
        break;
      case RejectReason::Inf:
        Serial.print(F("inf"));
        break;
      case RejectReason::OutOfRange:
        Serial.print(F("range"));
        break;
    }
  }
};

// Statically allocated. These objects are the whole application state.
ScenarioSimulator gSimulator;
SampleAcquisition gAcquisition;
MovingAverageFilter gFilter;
TrendAndEventDetector gTrend;
ComfortClassifier gComfort;
TelemetryFormatter gTelemetry;

AnalysisState gState = {0.0f, 0.0f, false, false, ComfortState::Unknown,
                        TrendEvent::WarmingUp60s, false};

uint32_t gBootMs = 0;

void setup() {
  Serial.begin(cfg::kBaud);
  gBootMs = millis();
  gAcquisition.start(gBootMs);
}

void loop() {
  const uint32_t now = millis();
  if (!gAcquisition.due(now)) {
    return;
  }
  gAcquisition.markSampled(now);

  const uint32_t index = gAcquisition.sampleIndex();
  gAcquisition.advance();

  const float rawC = gSimulator.readCelsius(index);
  const RejectReason reject = gAcquisition.assess(rawC);
  const bool rawFinite = reject != RejectReason::Nan && reject != RejectReason::Inf;
  const uint32_t elapsedMs = now - gBootMs;

  if (reject != RejectReason::None) {
    // Hold filter, trend, comfort, and stuck exactly as they were.
    gTelemetry.publish(elapsedMs, rawC, rawFinite, reject, gState);
    return;
  }

  const float filteredC = gFilter.push(rawC);
  const TrendSnapshot trend = gTrend.push(now, filteredC, rawC);
  const ComfortState comfort = gComfort.update(filteredC);

  gState.filteredC = filteredC;
  gState.deltaC = trend.deltaC;
  gState.hasFiltered = true;
  gState.deltaValid = trend.deltaValid;
  gState.comfort = comfort;
  gState.event = trend.event;
  gState.stuckFault = trend.stuckFault;

  gTelemetry.publish(elapsedMs, rawC, true, RejectReason::None, gState);
}
