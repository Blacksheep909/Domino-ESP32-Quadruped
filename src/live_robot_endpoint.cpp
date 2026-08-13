#include "live_robot_endpoint.h"

#include <ArduinoJson.h>
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
uint8_t calibrationChannel = 0;
float calibrationTargetDeg = 135.0f;
float calibrationCurrentDeg = 135.0f;
uint32_t calibrationUpdatedMs = 0;

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

void disableOutputs(Adafruit_PWMServoDriver &driver, LiveRobotState nextState) {
  benchMode = false;
  calibrationJogActive = false;
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
    acknowledge("calibration", action, requestId, false, "Calibration requires disarmed state.");
    return;
  }
  if (!strcmp(action, "enter")) {
    benchMode = true;
    setServoOutputsEnabled(driver, true);
    acknowledge("calibration", action, requestId, true);
    return;
  }
  if (!strcmp(action, "exit")) {
    disableOutputs(driver, LiveRobotState::Disarmed);
    acknowledge("calibration", action, requestId, true);
    return;
  }
  if (!benchMode) {
    acknowledge("calibration", action, requestId, false, "Bench mode has not been acknowledged.");
    return;
  }
  if (!strcmp(action, "jog")) {
    const int channel = payload["selectedChannel"] | -1;
    const float jog = payload["jogOffsetDeg"] | 99.0f;
    const float target = payload["targetServoDeg"] | NAN;
    if (channel < 0 || channel >= 16 || fabsf(jog) > kCalibrationJogLimitDeg || !isfinite(target)) {
      acknowledge("calibration", action, requestId, false, "Jog exceeds channel or +/-10 degree safety bounds.");
      return;
    }
    calibrationChannel = static_cast<uint8_t>(channel);
    calibrationCurrentDeg = commandedServoAnglesDeg()[calibrationChannel];
    calibrationTargetDeg = target;
    calibrationUpdatedMs = now;
    calibrationJogActive = true;
    acknowledge("calibration", action, requestId, true);
  } else if (!strcmp(action, "save-profile")) {
    acknowledge("calibration", action, requestId, false, "Persistent calibration is not supported by this firmware yet.");
  } else {
    acknowledge("calibration", action, requestId, false, "Unsupported calibration action.");
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
