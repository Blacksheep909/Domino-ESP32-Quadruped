#include "live_robot_endpoint.h"

#include <ArduinoJson.h>
#include <Preferences.h>
#include <math.h>

#include "crsf.h"
#include "imu.h"
#include "leg_controller.h"

namespace {
constexpr char kProtocol[] = "domino-robot-link-v1";
constexpr uint32_t kTelemetryIntervalMs = 50;
constexpr uint32_t kHelloIntervalMs = 1000;
constexpr uint32_t kWatchdogMs = 400;
constexpr uint32_t kLinkStatsFreshMs = 1000;
constexpr float kCalibrationMaxSpeedDegPerSec = 5.0f;
constexpr float kCalibrationJogLimitDeg = 10.0f;
constexpr uint32_t kCalibrationMagic = 0x4443414c;  // DCAL
constexpr char kCalibrationNamespace[] = "domino-cal";

LiveRobotState state = LiveRobotState::Disarmed;
LiveRobotPoseSnapshot expectedPose{};
uint32_t lastTelemetryMs = 0;
uint32_t lastHelloMs = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t lastHeartbeatSequence = 0;
bool haveHeartbeatSequence = false;
bool benchMode = false;
String inputLine;
bool calibrationJogActive = false;
bool calibrationChannelActive = false;
uint8_t calibrationChannel = 0;
float calibrationTargetDeg = 135.0f;
float calibrationCurrentDeg = 135.0f;
uint32_t calibrationUpdatedMs = 0;

struct StoredCalibrationProfile {
  uint32_t magic;
  ServoCalibrationProfile profile;
  uint32_t checksum;
};

uint32_t checksumBytes(const uint8_t *bytes, size_t length) {
  uint32_t hash = 2166136261u;
  for (size_t index = 0; index < length; ++index) {
    hash ^= bytes[index];
    hash *= 16777619u;
  }
  return hash;
}

StoredCalibrationProfile storedProfileFor(const ServoCalibrationProfile &profile) {
  StoredCalibrationProfile stored{};
  stored.magic = kCalibrationMagic;
  stored.profile = profile;
  stored.checksum = checksumBytes(
      reinterpret_cast<const uint8_t *>(&stored.profile), sizeof(stored.profile));
  return stored;
}

bool storedProfileValid(const StoredCalibrationProfile &stored) {
  return stored.magic == kCalibrationMagic &&
         stored.checksum == checksumBytes(
             reinterpret_cast<const uint8_t *>(&stored.profile), sizeof(stored.profile)) &&
         validateServoCalibrationProfile(stored.profile);
}

bool loadCalibrationProfile() {
  Preferences preferences;
  if (!preferences.begin(kCalibrationNamespace, true)) return false;
  StoredCalibrationProfile stored{};
  const size_t length = preferences.getBytesLength("active");
  const size_t read = length == sizeof(stored)
      ? preferences.getBytes("active", &stored, sizeof(stored)) : 0;
  preferences.end();
  return read == sizeof(stored) && storedProfileValid(stored) &&
         setServoCalibrationProfile(stored.profile);
}

bool persistCalibrationProfile(const ServoCalibrationProfile &profile) {
  if (!validateServoCalibrationProfile(profile)) return false;
  const StoredCalibrationProfile candidate = storedProfileFor(profile);
  Preferences preferences;
  if (!preferences.begin(kCalibrationNamespace, false)) return false;
  bool success = preferences.putBytes("candidate", &candidate, sizeof(candidate)) == sizeof(candidate);
  StoredCalibrationProfile verified{};
  success = success && preferences.getBytes("candidate", &verified, sizeof(verified)) == sizeof(verified) &&
            storedProfileValid(verified);
  success = success && preferences.putBytes("active", &verified, sizeof(verified)) == sizeof(verified);
  if (success) preferences.remove("candidate");
  preferences.end();
  return success && setServoCalibrationProfile(profile);
}

bool parseCalibrationProfile(JsonObjectConst source, ServoCalibrationProfile *profile) {
  if (!profile || source.isNull() || (source["schemaVersion"] | 0) != DOMINO_CALIBRATION_SCHEMA_VERSION ||
      strcmp(source["robot"] | "", "domino-esp32-quadruped")) return false;
  JsonArrayConst joints = source["joints"].as<JsonArrayConst>();
  if (joints.size() != DOMINO_DRIVEN_JOINT_COUNT) return false;
  ServoCalibrationProfile candidate{};
  candidate.schemaVersion = DOMINO_CALIBRATION_SCHEMA_VERSION;
  candidate.savedAt = source["savedAt"] | static_cast<uint64_t>(0);
  uint8_t index = 0;
  for (JsonObjectConst joint : joints) {
    candidate.joints[index++] = {
        static_cast<uint8_t>(joint["channel"] | 255),
        joint["offsetDeg"] | NAN,
        static_cast<int8_t>(joint["direction"] | 0),
        joint["minimumDeg"] | NAN,
        joint["maximumDeg"] | NAN,
    };
  }
  if (!validateServoCalibrationProfile(candidate)) return false;
  *profile = candidate;
  return true;
}

void addCalibrationProfile(JsonObject target, const ServoCalibrationProfile &profile) {
  target["schemaVersion"] = profile.schemaVersion;
  target["robot"] = "domino-esp32-quadruped";
  target["savedAt"] = profile.savedAt;
  JsonArray joints = target["joints"].to<JsonArray>();
  for (const ServoCalibrationJoint &joint : profile.joints) {
    JsonObject item = joints.add<JsonObject>();
    item["channel"] = joint.channel;
    item["offsetDeg"] = joint.offsetDeg;
    item["direction"] = joint.direction;
    item["minimumDeg"] = joint.minimumDeg;
    item["maximumDeg"] = joint.maximumDeg;
  }
}

const char* stateName() {
  switch (state) {
    case LiveRobotState::Disarmed: return "disarmed";
    case LiveRobotState::Armed: return "armed";
    case LiveRobotState::Estopped: return "estopped";
    case LiveRobotState::Watchdog: return "watchdog";
    default: return "fault";
  }
}

uint16_t txPowerMw(uint8_t code) {
  static constexpr uint16_t powers[] = {0, 10, 25, 100, 500, 1000, 2000, 250, 50};
  return code < sizeof(powers) / sizeof(powers[0]) ? powers[code] : 0;
}

void writeDocument(JsonDocument &document) {
  serializeJson(document, Serial);
  Serial.println();
}

void addCapabilities(JsonObject capabilities) {
  capabilities["telemetry"] = true;
  capabilities["calibration"] = true;
  capabilities["gaitProfiles"] = false;
  capabilities["persistentProfiles"] = false;
  capabilities["manualControl"] = false;
}

void sendHello() {
  JsonDocument document;
  document["protocol"] = kProtocol;
  document["type"] = "robot-hello";
  document["robotId"] = "domino-esp32-quadruped";
  document["robotName"] = "Domino";
  document["firmwareVersion"] = "0.3.0";
  document["robotState"] = stateName();
  addCapabilities(document["capabilities"].to<JsonObject>());
  writeDocument(document);
}

void addBody(JsonObject body, const LiveRobotPoseSnapshot &pose) {
  body["rollDeg"] = pose.rollDeg;
  body["pitchDeg"] = pose.pitchDeg;
  body["yawDeg"] = pose.yawDeg;
  body["heightMm"] = pose.heightMm;
}

void sendTelemetry(uint32_t now) {
  JsonDocument document;
  document["protocol"] = kProtocol;
  document["type"] = "robot-telemetry";
  document["robotState"] = stateName();
  document["robotTimeMs"] = now;
  addCapabilities(document["capabilities"].to<JsonObject>());

  JsonObject expected = document["expected"].to<JsonObject>();
  expected["timestampMs"] = now;
  JsonArray servoAngles = expected["servoAngleDeg"].to<JsonArray>();
  const float *angles = commandedServoAnglesDeg();
  for (uint8_t channel = 0; channel < 16; ++channel) servoAngles.add(angles[channel]);
  addBody(expected["body"].to<JsonObject>(), expectedPose);

  if (gImuState.online && gImuState.has_sample) {
    const float gx = gImuState.ay_g_filt;
    const float gy = gImuState.az_g_filt;
    const float gz = -gImuState.ax_g_filt;
    JsonObject measured = document["measured"].to<JsonObject>();
    measured["timestampMs"] = now;
    JsonObject body = measured["body"].to<JsonObject>();
    body["rollDeg"] = atan2f(gy, gz) * 180.0f / PI;
    body["pitchDeg"] = atan2f(-gx, sqrtf(gy * gy + gz * gz)) * 180.0f / PI;
  }

  // The IMU measures body attitude, not servo positions. Until joint encoders
  // exist this remains diagnostics rather than a fabricated measured skeleton.
  JsonObject diagnostics = document["diagnostics"].to<JsonObject>();
  diagnostics["robotState"] = stateName();
  diagnostics["outputsEnabled"] = servoOutputsEnabled();
  diagnostics["imuOnline"] = gImuState.online && gImuState.has_sample;
  diagnostics["imuAxG"] = gImuState.ax_g_filt;
  diagnostics["imuAyG"] = gImuState.ay_g_filt;
  diagnostics["imuAzG"] = gImuState.az_g_filt;
  diagnostics["servoLimitClipCount"] = servoSafetyClipCount();
  diagnostics["jointFeedbackAvailable"] = false;

  const CrsfLinkStatistics link = crsfLinkStatistics();
  JsonObject controller = document["controller"].to<JsonObject>();
  controller["source"] = "boxer-elrs";
  controller["frameTimestampMs"] = lastCrsfMs;
  controller["packetRateHz"] = crsfPacketRateHz();
  controller["frameLossCount"] = 0;
  controller["failsafe"] = !crsfLinkAlive(now);
  controller["failsafeCount"] = 0;
  controller["linkQualityPercent"] = link.valid ? link.linkQualityPercent : 0;
  controller["rssi1Dbm"] = link.valid ? link.rssi1Dbm : -127;
  controller["rssi2Dbm"] = link.valid ? link.rssi2Dbm : -127;
  controller["snrDb"] = link.valid ? link.snrDb : -128;
  controller["rfMode"] = link.valid ? String(link.rfMode) : "unknown";
  controller["txPowerMw"] = link.valid ? txPowerMw(link.txPowerCode) : 0;
  controller["activeAntenna"] = link.valid ? link.activeAntenna + 1 : 0;
  JsonArray channels = controller["channelsUs"].to<JsonArray>();
  for (uint8_t channel = 0; channel < 16; ++channel) channels.add(ch_us[channel]);
  writeDocument(document);
}

void acknowledge(const char *kind, const char *action, const char *requestId,
                 bool accepted, const char *reason = nullptr) {
  JsonDocument document;
  document["protocol"] = kProtocol;
  document["type"] = "robot-ack";
  document["kind"] = kind;
  document["action"] = action;
  document["requestId"] = requestId;
  document["accepted"] = accepted;
  document["robotState"] = stateName();
  if (reason) document["reason"] = reason;
  writeDocument(document);
}

void acknowledgeCalibration(const char *action, const char *requestId, bool accepted,
                            const char *reason = nullptr, bool persisted = false,
                            bool includeProfile = false) {
  JsonDocument document;
  document["protocol"] = kProtocol;
  document["type"] = "robot-ack";
  document["kind"] = "calibration";
  document["action"] = action;
  document["requestId"] = requestId;
  document["accepted"] = accepted;
  document["robotState"] = stateName();
  document["benchMode"] = benchMode;
  document["supportsSafeJog"] = true;
  document["maxSpeedDegPerSec"] = kCalibrationMaxSpeedDegPerSec;
  document["persisted"] = persisted;
  if (reason) document["reason"] = reason;
  if (includeProfile) addCalibrationProfile(document["profile"].to<JsonObject>(), servoCalibrationProfile());
  writeDocument(document);
}

void disableOutputs(Adafruit_PWMServoDriver &driver, LiveRobotState nextState) {
  benchMode = false;
  calibrationJogActive = false;
  calibrationChannelActive = false;
  setServoOutputsEnabled(driver, false);
  state = nextState;
}

bool controllerSafeToArm(uint32_t now) {
  const CrsfLinkStatistics link = crsfLinkStatistics();
  return crsfLinkAlive(now) && link.valid && now - link.timestampMs <= kLinkStatsFreshMs &&
         link.linkQualityPercent >= 50 && link.rssi1Dbm >= -105;
}

void handleSafety(JsonObjectConst command, JsonObjectConst payload,
                  Adafruit_PWMServoDriver &driver, uint32_t now) {
  const char *action = command["action"] | "";
  const char *requestId = command["requestId"] | "";
  if (!strcmp(action, "request-state")) {
    acknowledge("safety", action, requestId, true);
  } else if (!strcmp(action, "arm")) {
    if (state != LiveRobotState::Disarmed) {
      acknowledge("safety", action, requestId, false, "Robot must be disarmed.");
    } else if (!controllerSafeToArm(now) || ch_us[SA_CH_INDEX] > SA_OFF_THRESHOLD_US) {
      acknowledge("safety", action, requestId, false, "Fresh safe Boxer/ELRS link and SA-low are required.");
    } else {
      state = LiveRobotState::Armed;
      lastHeartbeatMs = now;
      haveHeartbeatSequence = false;
      setServoOutputsEnabled(driver, true);
      acknowledge("safety", action, requestId, true);
    }
  } else if (!strcmp(action, "disarm")) {
    if (state == LiveRobotState::Estopped) {
      acknowledge("safety", action, requestId, false, "E-stop is latched until the ESP32 is physically reset.");
    } else {
      disableOutputs(driver, LiveRobotState::Disarmed);
      acknowledge("safety", action, requestId, true);
    }
  } else if (!strcmp(action, "estop")) {
    disableOutputs(driver, LiveRobotState::Estopped);
    acknowledge("safety", action, requestId, true);
  } else if (!strcmp(action, "reset-estop")) {
    acknowledge("safety", action, requestId, false, "E-stop is latched until the ESP32 is physically reset.");
  } else {
    acknowledge("safety", action, requestId, false, "Unsupported safety action.");
  }
  (void)payload;
}

void handleHeartbeat(JsonObjectConst command, uint32_t now) {
  const uint32_t sequence = command["sequence"] | 0;
  if (state != LiveRobotState::Armed || (haveHeartbeatSequence && sequence <= lastHeartbeatSequence)) return;
  lastHeartbeatSequence = sequence;
  haveHeartbeatSequence = true;
  lastHeartbeatMs = now;
  JsonDocument document;
  document["protocol"] = kProtocol;
  document["type"] = "robot-safety-heartbeat-ack";
  document["sequence"] = sequence;
  document["robotState"] = stateName();
  document["watchdogRemainingMs"] = kWatchdogMs;
  writeDocument(document);
}

void handleCalibration(JsonObjectConst command, JsonObjectConst payload,
                       Adafruit_PWMServoDriver &driver, uint32_t now) {
  const char *action = command["action"] | "";
  const char *requestId = command["requestId"] | "";
  if (state != LiveRobotState::Disarmed) {
    acknowledgeCalibration(action, requestId, false, "Calibration requires disarmed state.");
    return;
  }
  if (!strcmp(action, "enter")) {
    benchMode = true;
    setServoOutputsEnabled(driver, true);
    acknowledgeCalibration(action, requestId, true);
    return;
  }
  if (!strcmp(action, "exit")) {
    disableOutputs(driver, LiveRobotState::Disarmed);
    acknowledgeCalibration(action, requestId, true);
    return;
  }
  if (!benchMode) {
    acknowledgeCalibration(action, requestId, false, "Bench mode has not been acknowledged.");
    return;
  }
  if (!strcmp(action, "jog")) {
    const int channel = payload["selectedChannel"] | -1;
    const float jog = payload["jogOffsetDeg"] | 99.0f;
    const float target = payload["targetServoDeg"] | NAN;
    const bool drivenChannel = channel >= 0 && channel < 16 &&
        findServoCalibrationJoint(servoCalibrationProfile(), static_cast<uint8_t>(channel)) != nullptr;
    const bool targetBounded = drivenChannel && isfinite(target) &&
        fabsf(target - servoCalibrationNeutralDeg(static_cast<uint8_t>(channel))) <= 40.0f;
    if (!drivenChannel || fabsf(jog) > kCalibrationJogLimitDeg || !targetBounded) {
      acknowledgeCalibration(action, requestId, false, "Jog exceeds channel or +/-10 degree safety bounds.");
      return;
    }
    if (calibrationChannelActive && calibrationChannel != static_cast<uint8_t>(channel)) {
      disableServoOutputChannel(driver, calibrationChannel);
    }
    calibrationChannel = static_cast<uint8_t>(channel);
    calibrationChannelActive = true;
    calibrationCurrentDeg = commandedServoAnglesDeg()[calibrationChannel];
    calibrationTargetDeg = target;
    calibrationUpdatedMs = now;
    calibrationJogActive = true;
    acknowledgeCalibration(action, requestId, true);
  } else if (!strcmp(action, "save-profile")) {
    ServoCalibrationProfile profile{};
    if (!parseCalibrationProfile(payload["profile"].as<JsonObjectConst>(), &profile)) {
      acknowledgeCalibration(action, requestId, false, "Profile must contain exactly 12 unique, bounded Domino joints.");
    } else if (!persistCalibrationProfile(profile)) {
      acknowledgeCalibration(action, requestId, false, "NVS verification failed; previous calibration remains active.");
    } else {
      acknowledgeCalibration(action, requestId, true, nullptr, true, true);
    }
  } else {
    acknowledgeCalibration(action, requestId, false, "Unsupported calibration action.");
  }
}

void handleCommand(const String &line, Adafruit_PWMServoDriver &driver, uint32_t now) {
  JsonDocument document;
  if (deserializeJson(document, line) != DeserializationError::Ok) return;
  JsonObjectConst command = document.as<JsonObjectConst>();
  if (strcmp(command["protocol"] | "", kProtocol) || strcmp(command["type"] | "", "companion-command")) return;
  const char *kind = command["kind"] | "";
  JsonObjectConst payload = command["payload"].as<JsonObjectConst>();
  if (!strcmp(kind, "safety")) handleSafety(command, payload, driver, now);
  else if (!strcmp(kind, "safety-heartbeat")) handleHeartbeat(command, now);
  else if (!strcmp(kind, "calibration")) handleCalibration(command, payload, driver, now);
  else if (command["requestId"].is<const char*>())
    acknowledge(kind, command["action"] | "", command["requestId"], false, "Capability is not implemented by this firmware.");
}

void readUsb(Adafruit_PWMServoDriver &driver, uint32_t now) {
  while (Serial.available()) {
    const char next = static_cast<char>(Serial.read());
    if (next == '\n') {
      if (inputLine.length()) handleCommand(inputLine, driver, now);
      inputLine = "";
    } else if (next != '\r') {
      if (inputLine.length() < 8192) inputLine += next;
      else inputLine = "";
    }
  }
}

void updateCalibrationJog(Adafruit_PWMServoDriver &driver, uint32_t now) {
  if (!calibrationJogActive || !benchMode || state != LiveRobotState::Disarmed) return;
  const float step = kCalibrationMaxSpeedDegPerSec * static_cast<float>(now - calibrationUpdatedMs) / 1000.0f;
  calibrationUpdatedMs = now;
  const float error = calibrationTargetDeg - calibrationCurrentDeg;
  if (fabsf(error) <= step) {
    calibrationCurrentDeg = calibrationTargetDeg;
    calibrationJogActive = false;
  } else {
    calibrationCurrentDeg += error > 0 ? step : -step;
  }
  commandCalibrationServoAngle(driver, calibrationChannel, calibrationCurrentDeg);
}
}  // namespace

void liveRobotEndpointBegin(Adafruit_PWMServoDriver &driver) {
  setServoOutputsEnabled(driver, false);
  if (!loadCalibrationProfile()) setServoCalibrationProfile(defaultServoCalibrationProfile());
  inputLine.reserve(1024);
  sendHello();
}

void liveRobotEndpointLoop(uint32_t now, Adafruit_PWMServoDriver &driver) {
  readUsb(driver, now);
  updateCalibrationJog(driver, now);
  if (state == LiveRobotState::Armed &&
      (now - lastHeartbeatMs > kWatchdogMs || !controllerSafeToArm(now))) {
    disableOutputs(driver, LiveRobotState::Watchdog);
  }
  if (now - lastTelemetryMs >= kTelemetryIntervalMs) {
    lastTelemetryMs = now;
    sendTelemetry(now);
  }
  if (now - lastHelloMs >= kHelloIntervalMs) {
    lastHelloMs = now;
    sendHello();
  }
}

void liveRobotEndpointSetExpectedPose(const LiveRobotPoseSnapshot &pose) { expectedPose = pose; }
LiveRobotState liveRobotEndpointState() { return state; }
bool liveRobotEndpointAllowsLocomotion() { return state == LiveRobotState::Armed; }
