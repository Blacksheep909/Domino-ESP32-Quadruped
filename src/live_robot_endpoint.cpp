#include "live_robot_endpoint.h"

#include <ArduinoJson.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>

#if __has_include("live_robot_secrets.h")
#include "live_robot_secrets.h"
#endif

#ifndef DOMINO_LIVE_WIFI_ENABLED
#define DOMINO_LIVE_WIFI_ENABLED 0
#endif
#ifndef DOMINO_LIVE_WIFI_SSID
#define DOMINO_LIVE_WIFI_SSID ""
#endif
#ifndef DOMINO_LIVE_WIFI_PASSWORD
#define DOMINO_LIVE_WIFI_PASSWORD ""
#endif
#ifndef DOMINO_LIVE_WIFI_HOSTNAME
#define DOMINO_LIVE_WIFI_HOSTNAME "domino-robot"
#endif
#ifndef DOMINO_LIVE_WIFI_PORT
#define DOMINO_LIVE_WIFI_PORT 8766
#endif
#ifndef DOMINO_LIVE_BLUETOOTH_ENABLED
#define DOMINO_LIVE_BLUETOOTH_ENABLED 0
#endif
#ifndef DOMINO_LIVE_BLUETOOTH_NAME
#define DOMINO_LIVE_BLUETOOTH_NAME "Domino-LIVE"
#endif
#ifndef DOMINO_LIVE_BLUETOOTH_PIN
#define DOMINO_LIVE_BLUETOOTH_PIN ""
#endif
#ifndef DOMINO_LIVE_LINK_KEY
#define DOMINO_LIVE_LINK_KEY ""
#endif
#ifndef DOMINO_POWER_CRITICAL_VOLTAGE_MV
#define DOMINO_POWER_CRITICAL_VOLTAGE_MV 12800
#endif
#ifndef DOMINO_POWER_FAULT_RECOVERY_VOLTAGE_MV
#define DOMINO_POWER_FAULT_RECOVERY_VOLTAGE_MV 13600
#endif

#if DOMINO_LIVE_WIFI_ENABLED
#include <WiFi.h>
#endif
#if DOMINO_LIVE_BLUETOOTH_ENABLED
#include <BluetoothSerial.h>
#endif

#include "crsf.h"
#include "gait_profile.h"
#include "imu.h"
#include "leg_controller.h"
#include "power_monitor.h"
#include "power_fault_guard.h"

namespace {
constexpr char kProtocol[] = "domino-robot-link-v1";
constexpr uint32_t kTelemetryIntervalMs = 100;
constexpr uint32_t kHelloIntervalMs = 2000;
constexpr uint32_t kWatchdogMs = 400;
constexpr uint32_t kLinkStatsFreshMs = 1000;
constexpr float kCalibrationMaxSpeedDegPerSec = 5.0f;
constexpr float kCalibrationJogLimitDeg = 10.0f;
constexpr uint32_t kCalibrationMagic = 0x4443414c;  // DCAL
constexpr char kCalibrationNamespace[] = "domino-cal";
constexpr uint32_t kGaitMagic = 0x44474149;  // DGAI
constexpr char kGaitNamespace[] = "domino-gait";
constexpr uint32_t kTransportOwnerIdleMs = 2000;
constexpr uint32_t kWirelessAuthenticationMs = 2000;
constexpr uint32_t kLowVoltageFaultHoldMs = 750;
constexpr float kPowerCriticalVoltageV = DOMINO_POWER_CRITICAL_VOLTAGE_MV / 1000.0f;
constexpr float kPowerFaultRecoveryVoltageV = DOMINO_POWER_FAULT_RECOVERY_VOLTAGE_MV / 1000.0f;
static_assert(DOMINO_POWER_FAULT_RECOVERY_VOLTAGE_MV > DOMINO_POWER_CRITICAL_VOLTAGE_MV,
              "Power fault recovery voltage must exceed the critical threshold");

#if DOMINO_LIVE_WIFI_ENABLED
static_assert(sizeof(DOMINO_LIVE_WIFI_SSID) > 1, "Wi-Fi SSID must not be empty");
static_assert(sizeof(DOMINO_LIVE_WIFI_PASSWORD) >= 9, "Wi-Fi password must contain at least 8 characters");
WiFiServer wifiServer(DOMINO_LIVE_WIFI_PORT);
WiFiClient wifiClient;
bool wifiServerStarted = false;
bool wifiAuthenticated = false;
uint32_t wifiAcceptedMs = 0;
#endif
#if DOMINO_LIVE_BLUETOOTH_ENABLED
static_assert(sizeof(DOMINO_LIVE_BLUETOOTH_PIN) >= 5 && sizeof(DOMINO_LIVE_BLUETOOTH_PIN) <= 17,
              "Bluetooth PIN must contain 4-16 characters");
BluetoothSerial bluetoothSerial;
bool bluetoothAuthenticated = false;
bool bluetoothHadClient = false;
uint32_t bluetoothAcceptedMs = 0;
#endif
#if DOMINO_LIVE_WIFI_ENABLED || DOMINO_LIVE_BLUETOOTH_ENABLED
static_assert(sizeof(DOMINO_LIVE_LINK_KEY) >= 17,
              "Wireless LIVE transports require a link key of at least 16 characters");
#endif

enum class LiveTransport : uint8_t { None, Usb, Wifi, Bluetooth };

LiveRobotState state = LiveRobotState::Disarmed;
// Standalone CRSF control is available after boot. A LIVE disarm, E-stop,
// watchdog, fault, or calibration session can latch it off; an accepted LIVE
// arm or a physical reboot restores it.
bool radioControlEnabled = true;
LiveRobotPoseSnapshot expectedPose{};
float expectedFootTargetsMm[4][3] = {};
uint32_t lastTelemetryMs = 0;
uint32_t lastHelloMs = 0;
uint32_t lastDetailedTelemetryMs = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t lastHeartbeatSequence = 0;
bool haveHeartbeatSequence = false;
uint32_t loopRateWindowStartedMs = 0;
uint32_t loopRateIterations = 0;
float measuredLoopRateHz = 0.0f;
bool benchMode = false;
String usbInputLine;
#if DOMINO_LIVE_WIFI_ENABLED
String wifiInputLine;
#endif
#if DOMINO_LIVE_BLUETOOTH_ENABLED
String bluetoothInputLine;
#endif
LiveTransport commandOwner = LiveTransport::None;
uint32_t lastOwnerCommandMs = 0;
bool calibrationJogActive = false;
bool calibrationChannelActive = false;
uint8_t calibrationChannel = 0;
uint8_t calibrationPhysicalChannel = 0;
float calibrationTargetDeg = 135.0f;
float calibrationCurrentDeg = 135.0f;
uint32_t calibrationUpdatedMs = 0;
ManualControlGuard manualGuard;
PowerFaultGuard powerFaultGuard(kPowerCriticalVoltageV, kPowerFaultRecoveryVoltageV,
                                kLowVoltageFaultHoldMs);
char faultReason[96] = {};

struct StoredCalibrationProfile {
  uint32_t magic;
  ServoCalibrationProfile profile;
  uint32_t checksum;
};

// Retained solely to migrate the two-slot NVS profile written by firmware 0.6.
// Keep this layout byte-for-byte compatible with gait schema v1.
struct GaitProfileSettingsV1 {
  bool enabled;
  char preset[12];
  float cadenceHz;
  float strideMm;
  float liftMm;
  float dutyFactor;
  float bodyHeightMm;
  float stanceWidthMm;
  float turnGain;
  float responseMs;
  float swingShape;
  float diagonalPhase;
};

struct GaitProfileV1 {
  uint16_t schemaVersion;
  uint64_t updatedAt;
  char name[33];
  GaitProfileSettingsV1 settings;
};

struct StoredGaitProfileV1 {
  uint32_t magic;
  GaitProfileV1 profile;
  uint32_t checksum;
};

struct StoredGaitProfile {
  uint32_t magic;
  GaitProfile profile;
  uint32_t checksum;
};

uint8_t activeGaitSlot = 0;
bool gaitRollbackAvailable = false;

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
  // NVS blob replacement is atomic at the key level. One committed write plus
  // a read-only verification avoids the former candidate/active/remove chain,
  // which could stall the USB command path through several flash commits.
  Preferences writer;
  if (!writer.begin(kCalibrationNamespace, false)) return false;
  const bool written = writer.putBytes("active", &candidate, sizeof(candidate)) == sizeof(candidate);
  writer.end();
  if (!written) return false;

  Preferences verifier;
  if (!verifier.begin(kCalibrationNamespace, true)) return false;
  StoredCalibrationProfile verified{};
  const bool verifiedRead = verifier.getBytes("active", &verified, sizeof(verified)) == sizeof(verified);
  verifier.end();
  const bool exactMatch = verifiedRead && storedProfileValid(verified) &&
      memcmp(&verified, &candidate, sizeof(candidate)) == 0;
  return exactMatch && setServoCalibrationProfile(profile);
}

StoredGaitProfile storedGaitFor(const GaitProfile &profile) {
  StoredGaitProfile stored{};
  stored.magic = kGaitMagic;
  stored.profile = profile;
  stored.checksum = checksumBytes(
      reinterpret_cast<const uint8_t *>(&stored.profile), sizeof(stored.profile));
  return stored;
}

bool storedGaitValid(const StoredGaitProfile &stored) {
  return stored.magic == kGaitMagic &&
      stored.checksum == checksumBytes(
          reinterpret_cast<const uint8_t *>(&stored.profile), sizeof(stored.profile)) &&
      validateGaitProfile(stored.profile);
}

bool migrateStoredGaitV1(const StoredGaitProfileV1 &stored, StoredGaitProfile *migrated) {
  if (!migrated || stored.magic != kGaitMagic || stored.profile.schemaVersion != 1 ||
      stored.checksum != checksumBytes(
          reinterpret_cast<const uint8_t *>(&stored.profile), sizeof(stored.profile))) return false;
  GaitProfile profile = defaultGaitProfile();
  profile.updatedAt = stored.profile.updatedAt;
  memcpy(profile.name, stored.profile.name, sizeof(profile.name));
  profile.name[sizeof(profile.name) - 1] = '\0';
  profile.settings.enabled = stored.profile.settings.enabled;
  memcpy(profile.settings.preset, stored.profile.settings.preset, sizeof(profile.settings.preset));
  profile.settings.preset[sizeof(profile.settings.preset) - 1] = '\0';
  profile.settings.cadenceHz = stored.profile.settings.cadenceHz;
  profile.settings.strideMm = stored.profile.settings.strideMm;
  profile.settings.liftMm = stored.profile.settings.liftMm;
  profile.settings.dutyFactor = stored.profile.settings.dutyFactor;
  profile.settings.bodyHeightMm = stored.profile.settings.bodyHeightMm;
  profile.settings.stanceWidthMm = stored.profile.settings.stanceWidthMm;
  profile.settings.turnGain = stored.profile.settings.turnGain;
  profile.settings.responseMs = stored.profile.settings.responseMs;
  profile.settings.swingShape = stored.profile.settings.swingShape;
  profile.settings.diagonalPhase = stored.profile.settings.diagonalPhase;
  if (!strcmp(profile.settings.preset, "balanced")) {
    profile.settings.maxForwardScale = 0.80f;
    profile.settings.maxTurnScale = 0.70f;
  } else if (!strcmp(profile.settings.preset, "fast")) {
    profile.settings.maxForwardScale = 1.0f;
    profile.settings.maxTurnScale = 0.90f;
  }
  if (!validateGaitProfile(profile)) return false;
  *migrated = storedGaitFor(profile);
  return true;
}

bool readGaitSlot(Preferences &preferences, uint8_t slot, StoredGaitProfile *stored) {
  const char *key = slot == 0 ? "slot0" : "slot1";
  if (!stored) return false;
  const size_t length = preferences.getBytesLength(key);
  if (length == sizeof(*stored)) {
    return preferences.getBytes(key, stored, sizeof(*stored)) == sizeof(*stored) &&
        storedGaitValid(*stored);
  }
  if (length == sizeof(StoredGaitProfileV1)) {
    StoredGaitProfileV1 legacy{};
    return preferences.getBytes(key, &legacy, sizeof(legacy)) == sizeof(legacy) &&
        migrateStoredGaitV1(legacy, stored);
  }
  return false;
}

bool loadGaitProfile() {
  Preferences preferences;
  if (!preferences.begin(kGaitNamespace, true)) return false;
  const uint8_t preferred = preferences.getUChar("active", 0) > 0 ? 1 : 0;
  StoredGaitProfile selected{};
  StoredGaitProfile alternate{};
  const bool selectedValid = readGaitSlot(preferences, preferred, &selected);
  const bool alternateValid = readGaitSlot(preferences, 1 - preferred, &alternate);
  preferences.end();
  if (selectedValid && setGaitProfile(selected.profile)) {
    activeGaitSlot = preferred;
    gaitRollbackAvailable = alternateValid;
    return true;
  }
  if (alternateValid && setGaitProfile(alternate.profile)) {
    activeGaitSlot = 1 - preferred;
    gaitRollbackAvailable = false;
    return true;
  }
  return false;
}

bool persistGaitProfile(const GaitProfile &profile) {
  if (!validateGaitProfile(profile)) return false;
  Preferences preferences;
  if (!preferences.begin(kGaitNamespace, false)) return false;
  // Seed the first slot with the currently running safe/default profile so
  // the very first browser apply is also recoverable.
  StoredGaitProfile current{};
  if (!readGaitSlot(preferences, activeGaitSlot, &current)) {
    current = storedGaitFor(gaitProfile());
    const char *currentKey = activeGaitSlot == 0 ? "slot0" : "slot1";
    if (preferences.putBytes(currentKey, &current, sizeof(current)) != sizeof(current)) {
      preferences.end();
      return false;
    }
  }
  const uint8_t candidateSlot = 1 - activeGaitSlot;
  const char *candidateKey = candidateSlot == 0 ? "slot0" : "slot1";
  const StoredGaitProfile candidate = storedGaitFor(profile);
  bool success = preferences.putBytes(candidateKey, &candidate, sizeof(candidate)) == sizeof(candidate);
  StoredGaitProfile verified{};
  success = success && readGaitSlot(preferences, candidateSlot, &verified);
  success = success && preferences.putUChar("active", candidateSlot) == 1;
  preferences.end();
  if (!success || !setGaitProfile(verified.profile)) return false;
  activeGaitSlot = candidateSlot;
  gaitRollbackAvailable = true;
  return true;
}

bool revertGaitProfile() {
  if (!gaitRollbackAvailable) return false;
  Preferences preferences;
  if (!preferences.begin(kGaitNamespace, false)) return false;
  const uint8_t previousSlot = 1 - activeGaitSlot;
  StoredGaitProfile previous{};
  const bool success = readGaitSlot(preferences, previousSlot, &previous) &&
      preferences.putUChar("active", previousSlot) == 1;
  preferences.end();
  if (!success || !setGaitProfile(previous.profile)) return false;
  activeGaitSlot = previousSlot;
  return true;
}

bool copyBoundedString(JsonVariantConst source, char *destination, size_t capacity) {
  const char *value = source.as<const char *>();
  if (!value || !destination || capacity == 0 || strlen(value) >= capacity) return false;
  strcpy(destination, value);
  return true;
}

bool parseGaitProfile(JsonObjectConst source, GaitProfile *profile) {
  if (!profile || source.isNull() ||
      (source["schemaVersion"] | 0) != DOMINO_GAIT_SCHEMA_VERSION ||
      strcmp(source["robot"] | "", "domino-esp32-quadruped")) return false;
  GaitProfile candidate{};
  candidate.schemaVersion = DOMINO_GAIT_SCHEMA_VERSION;
  candidate.updatedAt = source["updatedAt"] | static_cast<uint64_t>(0);
  JsonObjectConst settings = source["settings"].as<JsonObjectConst>();
  if (!copyBoundedString(source["name"], candidate.name, sizeof(candidate.name)) ||
      !copyBoundedString(settings["preset"], candidate.settings.preset,
                         sizeof(candidate.settings.preset))) return false;
  candidate.settings.enabled = settings["enabled"] | false;
  candidate.settings.cadenceHz = settings["cadenceHz"] | NAN;
  candidate.settings.strideMm = settings["strideMm"] | NAN;
  candidate.settings.liftMm = settings["liftMm"] | NAN;
  candidate.settings.dutyFactor = settings["dutyFactor"] | NAN;
  candidate.settings.bodyHeightMm = settings["bodyHeightMm"] | NAN;
  candidate.settings.stanceWidthMm = settings["stanceWidthMm"] | NAN;
  candidate.settings.turnGain = settings["turnGain"] | NAN;
  candidate.settings.responseMs = settings["responseMs"] | NAN;
  candidate.settings.swingShape = settings["swingShape"] | NAN;
  candidate.settings.diagonalPhase = settings["diagonalPhase"] | NAN;
  candidate.settings.touchdownXMm = settings["touchdownXMm"] | NAN;
  candidate.settings.maxForwardScale = settings["maxForwardScale"] | NAN;
  candidate.settings.maxTurnScale = settings["maxTurnScale"] | NAN;
  if (!validateGaitProfile(candidate)) return false;
  *profile = candidate;
  return true;
}

void addGaitProfile(JsonObject target, const GaitProfile &profile) {
  target["schemaVersion"] = profile.schemaVersion;
  target["robot"] = "domino-esp32-quadruped";
  target["name"] = profile.name;
  target["updatedAt"] = profile.updatedAt;
  target["source"] = "robot";
  JsonObject settings = target["settings"].to<JsonObject>();
  settings["enabled"] = profile.settings.enabled;
  settings["preset"] = profile.settings.preset;
  settings["cadenceHz"] = profile.settings.cadenceHz;
  settings["strideMm"] = profile.settings.strideMm;
  settings["liftMm"] = profile.settings.liftMm;
  settings["dutyFactor"] = profile.settings.dutyFactor;
  settings["bodyHeightMm"] = profile.settings.bodyHeightMm;
  settings["stanceWidthMm"] = profile.settings.stanceWidthMm;
  settings["turnGain"] = profile.settings.turnGain;
  settings["responseMs"] = profile.settings.responseMs;
  settings["swingShape"] = profile.settings.swingShape;
  settings["diagonalPhase"] = profile.settings.diagonalPhase;
  settings["touchdownXMm"] = profile.settings.touchdownXMm;
  settings["maxForwardScale"] = profile.settings.maxForwardScale;
  settings["maxTurnScale"] = profile.settings.maxTurnScale;
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
        static_cast<uint8_t>(joint["logicalChannel"] | 255),
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
    item["logicalChannel"] = joint.logicalChannel;
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
  // ArduinoJson's Stream writer calls write() for each byte. On ESP32 that
  // turns a ~1.3 KB telemetry document into hundreds of UART queue operations
  // and can stall the physical control loop. Serialize once, then send one
  // bounded block to every active transport.
  static constexpr size_t kDocumentBufferBytes = 8192;
  static char output[kDocumentBufferBytes];
  const size_t length = serializeJson(document, output, kDocumentBufferBytes - 1);
  if (length == 0 || length >= kDocumentBufferBytes - 1) return;
  output[length] = '\n';
  Serial.write(reinterpret_cast<const uint8_t *>(output), length + 1);
#if DOMINO_LIVE_WIFI_ENABLED
  if (wifiClient && wifiClient.connected()) {
    wifiClient.write(reinterpret_cast<const uint8_t *>(output), length + 1);
  }
#endif
#if DOMINO_LIVE_BLUETOOTH_ENABLED
  if (bluetoothSerial.hasClient()) {
    bluetoothSerial.write(reinterpret_cast<const uint8_t *>(output), length + 1);
  }
#endif
}

void addCapabilities(JsonObject capabilities) {
  capabilities["telemetry"] = true;
  capabilities["calibration"] = true;
  capabilities["gaitProfiles"] = true;
  capabilities["persistentProfiles"] = true;
  capabilities["persistentGaitProfiles"] = true;
  capabilities["manualControl"] = true;
}

bool boundedManualAxis(JsonVariantConst source, float *value) {
  if (!value || !source.is<float>()) return false;
  const float candidate = source.as<float>();
  if (!isfinite(candidate) || candidate < -1.0f || candidate > 1.0f) return false;
  *value = candidate;
  return true;
}

bool constantTimeLinkKeyMatches(const char *candidate) {
  if (!candidate) return false;
  const size_t expectedLength = strlen(DOMINO_LIVE_LINK_KEY);
  const size_t candidateLength = strlen(candidate);
  size_t difference = expectedLength ^ candidateLength;
  const size_t comparedLength = expectedLength > candidateLength ? expectedLength : candidateLength;
  for (size_t index = 0; index < comparedLength; ++index) {
    const uint8_t expected = index < expectedLength ? DOMINO_LIVE_LINK_KEY[index] : 0;
    const uint8_t supplied = index < candidateLength ? candidate[index] : 0;
    difference |= expected ^ supplied;
  }
  return difference == 0;
}

bool transportAuthenticated(JsonObjectConst command, LiveTransport source) {
  if (source == LiveTransport::Usb) return true;
#if DOMINO_LIVE_WIFI_ENABLED
  if (source == LiveTransport::Wifi && !wifiAuthenticated) return false;
#endif
#if DOMINO_LIVE_BLUETOOTH_ENABLED
  if (source == LiveTransport::Bluetooth && !bluetoothAuthenticated) return false;
#endif
  return constantTimeLinkKeyMatches(command["linkKey"] | static_cast<const char *>(nullptr));
}

void sendHello() {
  JsonDocument document;
  document["protocol"] = kProtocol;
  document["type"] = "robot-hello";
  document["robotId"] = "domino-esp32-quadruped";
  document["robotName"] = "Domino";
  document["firmwareVersion"] = "0.8.4";
  document["robotState"] = stateName();
  if (state == LiveRobotState::Fault && faultReason[0]) document["faultReason"] = faultReason;
  document["wirelessAuth"] = "psk-v1";
  addCapabilities(document["capabilities"].to<JsonObject>());
  addGaitProfile(document["gaitProfile"].to<JsonObject>(), gaitProfile());
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
  // Capabilities and the active gait profile are stable session metadata and
  // travel in robot-hello. Repeating them in every sample saturated 115200
  // baud and starved the physical control loop.
  const PowerMonitorSample power = powerMonitorSample(now);
  if (power.valid) {
    JsonObject powerJson = document["power"].to<JsonObject>();
    powerJson["timestampMs"] = power.timestampMs;
    if (power.voltageValid) powerJson["voltageV"] = power.voltageV;
    if (power.currentValid) powerJson["currentA"] = power.currentA;
    if (power.powerValid) powerJson["powerW"] = power.powerW;
  }

  JsonObject expected = document["expected"].to<JsonObject>();
  expected["timestampMs"] = now;
  JsonArray servoAngles = expected["servoAngleDeg"].to<JsonArray>();
  const float *angles = commandedServoAnglesDeg();
  for (uint8_t channel = 0; channel < 16; ++channel) servoAngles.add(angles[channel]);
  // Angles and body pose drive the 10 Hz digital twin. Pulse routing and foot
  // targets change much less often, so publish them once per second and let
  // the companion retain the last detailed snapshot. This keeps the control
  // loop responsive even on USB-UART bridges with small hardware FIFOs.
  const bool includeDetails = lastDetailedTelemetryMs == 0 ||
      now - lastDetailedTelemetryMs >= 1000;
  if (includeDetails) {
    lastDetailedTelemetryMs = now;
    JsonArray servoPulseUs = expected["servoPulseUs"].to<JsonArray>();
    JsonArray servoPhysicalChannel = expected["servoPhysicalChannel"].to<JsonArray>();
    const uint16_t *pulses = commandedServoPulseUs();
    const ServoCalibrationProfile &calibration = servoCalibrationProfile();
    for (uint8_t logicalChannel = 0; logicalChannel < 16; ++logicalChannel) {
      servoPulseUs.add(pulses[logicalChannel]);
      servoPhysicalChannel.add(
          servoCalibrationPhysicalChannel(calibration, logicalChannel));
    }
    JsonArray footTargets = expected["footTargetMm"].to<JsonArray>();
    for (uint8_t leg = 0; leg < 4; ++leg) {
      JsonArray target = footTargets.add<JsonArray>();
      for (uint8_t axis = 0; axis < 3; ++axis) target.add(expectedFootTargetsMm[leg][axis]);
    }
  }
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
    body["yawDeg"] = gImuState.yaw_deg;
  }

  // The IMU measures body attitude, not servo positions. Until joint encoders
  // exist this remains diagnostics rather than a fabricated measured skeleton.
  JsonObject diagnostics = document["diagnostics"].to<JsonObject>();
  diagnostics["robotState"] = stateName();
  if (state == LiveRobotState::Fault && faultReason[0]) diagnostics["faultReason"] = faultReason;
  diagnostics["outputsEnabled"] = servoOutputsEnabled();
  diagnostics["imuOnline"] = gImuState.online && gImuState.has_sample;
  diagnostics["imuAxG"] = gImuState.ax_g_filt;
  diagnostics["imuAyG"] = gImuState.ay_g_filt;
  diagnostics["imuAzG"] = gImuState.az_g_filt;
  diagnostics["imuYawRateDps"] = gImuState.yaw_rate_dps;
  diagnostics["servoLimitClipCount"] = servoSafetyClipCount();
  diagnostics["jointFeedbackAvailable"] = false;
  const LiveManualControlSnapshot &manual = manualGuard.snapshot();
  diagnostics["manualAuthorityActive"] = manualGuard.authorityActive();
  diagnostics["manualOverrideActive"] = manual.active;
  diagnostics["manualDeadman"] = manual.deadman;
  diagnostics["manualFrameAgeMs"] = manualGuard.frameAgeMs(now);
  diagnostics["manualLeaseRemainingMs"] = manualGuard.leaseRemainingMs(now);
  diagnostics["powerMonitorOnline"] = power.online;
  diagnostics["powerSampleValid"] = power.valid;
  diagnostics["uptimeMs"] = now;
  diagnostics["esp32LoopHz"] = measuredLoopRateHz;
  diagnostics["controllerHz"] = crsfPacketRateHz();
  if (crsfHasReceivedFrame() && now >= lastCrsfMs) {
    diagnostics["commandLatencyMs"] = now - lastCrsfMs;
  }

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
  else if (state == LiveRobotState::Fault && faultReason[0]) document["reason"] = faultReason;
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

void acknowledgeGait(const char *action, const char *requestId, bool accepted,
                     const char *reason = nullptr, bool includeProfile = true) {
  JsonDocument document;
  document["protocol"] = kProtocol;
  document["type"] = "robot-ack";
  document["kind"] = "gait";
  document["action"] = action;
  document["requestId"] = requestId;
  document["accepted"] = accepted;
  document["robotState"] = stateName();
  document["persisted"] = accepted;
  document["rollbackAvailable"] = gaitRollbackAvailable;
  if (reason) document["reason"] = reason;
  if (includeProfile) addGaitProfile(document["profile"].to<JsonObject>(), gaitProfile());
  writeDocument(document);
}

void disableOutputs(Adafruit_PWMServoDriver &driver, LiveRobotState nextState) {
  radioControlEnabled = false;
  benchMode = false;
  calibrationJogActive = false;
  calibrationChannelActive = false;
  setServoOutputsEnabled(driver, false);
  manualGuard.revoke();
  state = nextState;
}

void updatePowerFault(Adafruit_PWMServoDriver &driver, uint32_t now) {
  const PowerMonitorSample power = powerMonitorSample(now);
  if (!powerFaultGuard.observe(now, state == LiveRobotState::Armed,
                               power.voltageValid, power.voltageV)) return;
  snprintf(faultReason, sizeof(faultReason),
           "Battery %.2f V remained below %.2f V for at least %lu ms.",
           powerFaultGuard.tripVoltageV(), kPowerCriticalVoltageV,
           static_cast<unsigned long>(kLowVoltageFaultHoldMs));
  disableOutputs(driver, LiveRobotState::Fault);
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
      radioControlEnabled = true;
      lastHeartbeatMs = now;
      haveHeartbeatSequence = false;
      setServoOutputsEnabled(driver, true);
      acknowledge("safety", action, requestId, true);
    }
  } else if (!strcmp(action, "disarm")) {
    if (state == LiveRobotState::Estopped) {
      acknowledge("safety", action, requestId, false, "E-stop is latched until the ESP32 is physically reset.");
    } else if (state == LiveRobotState::Fault) {
      acknowledge("safety", action, requestId, false,
                  "Fault is latched. Correct the cause and use ACKNOWLEDGE FAULT.");
    } else {
      disableOutputs(driver, LiveRobotState::Disarmed);
      acknowledge("safety", action, requestId, true);
    }
  } else if (!strcmp(action, "estop")) {
    disableOutputs(driver, LiveRobotState::Estopped);
    acknowledge("safety", action, requestId, true);
  } else if (!strcmp(action, "reset-estop")) {
    acknowledge("safety", action, requestId, false, "E-stop is latched until the ESP32 is physically reset.");
  } else if (!strcmp(action, "acknowledge-fault")) {
    if (state != LiveRobotState::Fault) {
      acknowledge("safety", action, requestId, false, "Robot is not in the fault state.");
    } else if (powerFaultGuard.latched()) {
      const PowerMonitorSample power = powerMonitorSample(now);
      if (!powerFaultGuard.canAcknowledge(power.voltageValid, power.voltageV)) {
        if (!power.valid) {
          acknowledge("safety", action, requestId, false,
                      "Fresh power telemetry is required before clearing the low-voltage fault.");
        } else {
          char reason[96] = {};
          snprintf(reason, sizeof(reason),
                   "Battery is %.2f V; recover to at least %.2f V before acknowledging.",
                   power.voltageV, kPowerFaultRecoveryVoltageV);
          acknowledge("safety", action, requestId, false, reason);
        }
      } else {
        powerFaultGuard.acknowledge(power.voltageValid, power.voltageV);
        faultReason[0] = '\0';
        disableOutputs(driver, LiveRobotState::Disarmed);
        acknowledge("safety", action, requestId, true);
      }
    } else {
      acknowledge("safety", action, requestId, false,
                  "The active fault has no verified recovery condition.");
    }
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
    radioControlEnabled = false;
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
    const int physicalChannel = payload["physicalChannel"] | -1;
    const float jog = payload["jogOffsetDeg"] | 99.0f;
    const float target = payload["targetServoDeg"] | NAN;
    const bool drivenChannel = channel >= 0 && channel < 16 &&
        findServoCalibrationJoint(servoCalibrationProfile(), static_cast<uint8_t>(channel)) != nullptr;
    const bool validPhysicalChannel = physicalChannel >= 0 && physicalChannel < 16;
    const bool targetBounded = drivenChannel && isfinite(target) &&
        fabsf(target - servoCalibrationNeutralDeg(static_cast<uint8_t>(channel))) <= 40.0f;
    if (!drivenChannel || !validPhysicalChannel ||
        fabsf(jog) > kCalibrationJogLimitDeg || !targetBounded) {
      acknowledgeCalibration(action, requestId, false, "Jog exceeds channel or +/-10 degree safety bounds.");
      return;
    }
    if (calibrationChannelActive &&
        calibrationPhysicalChannel != static_cast<uint8_t>(physicalChannel)) {
      disableServoOutputPhysicalChannel(driver, calibrationPhysicalChannel);
    }
    calibrationChannel = static_cast<uint8_t>(channel);
    calibrationPhysicalChannel = static_cast<uint8_t>(physicalChannel);
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
    } else {
      // A channel-map replacement is applied only with every PCA9685 output
      // fully off. Re-entering bench mode is required before any further jog.
      disableOutputs(driver, LiveRobotState::Disarmed);
      if (!persistCalibrationProfile(profile)) {
        acknowledgeCalibration(action, requestId, false, "NVS verification failed; previous calibration remains active.");
      } else {
        // Keep the persistence acknowledgement compact and deterministic on
        // USB-UART links. The checksum-verified write above is the evidence;
        // echoing the whole profile here previously delayed or lost the ACK.
        acknowledgeCalibration(action, requestId, true, nullptr, true, false);
      }
    }
  } else {
    acknowledgeCalibration(action, requestId, false, "Unsupported calibration action.");
  }
}

void handleManualAuthority(JsonObjectConst command, JsonObjectConst payload, uint32_t now) {
  const char *action = command["action"] | "";
  const char *requestId = command["requestId"] | "";
  if (!strcmp(action, "request-authority")) {
    const char *token = command["authorityToken"] | "";
    const uint32_t leaseMs = command["leaseMs"] | 0;
    const bool safetyContract = payload["safety"]["requiresArmed"] == true &&
        payload["safety"]["requiresDeadman"] == true &&
        payload["safety"]["neutralOnRelease"] == true &&
        (payload["safety"]["commandTimeoutMs"] | 0) == DOMINO_MANUAL_TIMEOUT_MS;
    if (state != LiveRobotState::Armed || !controllerSafeToArm(now)) {
      acknowledge("manual-authority", action, requestId, false,
                  "Armed state and a fresh healthy Boxer/ELRS link are required.");
    } else if (manualGuard.authorityActive()) {
      acknowledge("manual-authority", action, requestId, false,
                  "Another manual-control lease is already active.");
    } else if (!safetyContract || !manualGuard.grant(token, leaseMs, now)) {
      acknowledge("manual-authority", action, requestId, false,
                  "Invalid authority token, lease, or safety contract.");
    } else {
      acknowledge("manual-authority", action, requestId, true);
    }
    return;
  }
  if (!strcmp(action, "release-authority")) {
    if (!manualGuard.release(payload["authorityToken"] | "")) {
      acknowledge("manual-authority", action, requestId, false,
                  "Authority token does not match the active lease.");
    } else {
      acknowledge("manual-authority", action, requestId, true);
    }
    return;
  }
  acknowledge("manual-authority", action, requestId, false,
              "Unsupported manual-authority action.");
}

void handleManualControl(JsonObjectConst command, uint32_t now) {
  // A companion-generated neutral command intentionally needs no token. It is
  // a one-way fail-safe used on browser disconnect, deadman release and lease
  // teardown; accepting it can only reduce motion.
  const bool neutral = command["neutral"] == true && command["deadman"] == false;
  if (neutral) {
    manualGuard.acceptNeutral(now);
    return;
  }
  const char *token = command["authorityToken"] | "";
  const uint32_t sequence = command["sequence"] | 0;
  const uint32_t timeoutMs = command["timeoutMs"] | 0;
  const char *mode = command["mode"] | "";
  if (command["deadman"] != true) return;
  LiveManualMode parsedMode = LiveManualMode::Stand;
  if (!strcmp(mode, "stand")) parsedMode = LiveManualMode::Stand;
  else if (!strcmp(mode, "careful")) parsedMode = LiveManualMode::Careful;
  else if (!strcmp(mode, "trot")) parsedMode = LiveManualMode::Trot;
  else return;
  JsonObjectConst axes = command["axes"].as<JsonObjectConst>();
  float forward = 0.0f;
  float turn = 0.0f;
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  float bodyX = 0.0f;
  float bodyY = 0.0f;
  float height = 0.0f;
  if (state != LiveRobotState::Armed || !controllerSafeToArm(now) ||
      !boundedManualAxis(axes["forward"], &forward) ||
      !boundedManualAxis(axes["turn"], &turn) ||
      !boundedManualAxis(axes["roll"], &roll) ||
      !boundedManualAxis(axes["pitch"], &pitch) ||
      !boundedManualAxis(axes["yaw"], &yaw) ||
      !boundedManualAxis(axes["bodyX"], &bodyX) ||
      !boundedManualAxis(axes["bodyY"], &bodyY) ||
      !boundedManualAxis(axes["height"], &height)) return;
  manualGuard.acceptFrame(token, sequence, now, parsedMode, forward, turn, roll,
                          pitch, yaw, bodyX, bodyY, height, timeoutMs);
}

void updateManualControl(uint32_t now) {
  manualGuard.tick(now, state == LiveRobotState::Armed && controllerSafeToArm(now));
}

void handleGait(JsonObjectConst command, JsonObjectConst payload) {
  const char *action = command["action"] | "";
  const char *requestId = command["requestId"] | "";
  if (!strcmp(action, "request-profile")) {
    acknowledgeGait(action, requestId, true);
    return;
  }
  if (state != LiveRobotState::Disarmed || benchMode || servoOutputsEnabled()) {
    acknowledgeGait(action, requestId, false,
                    "Gait persistence requires disarmed state with every servo output disabled.");
    return;
  }
  if (!strcmp(action, "apply-profile")) {
    if (payload["safety"]["requiresDisarmed"] != true ||
        payload["safety"]["twoStageApply"] != true) {
      acknowledgeGait(action, requestId, false, "Required two-stage safety acknowledgement is missing.");
      return;
    }
    GaitProfile profile{};
    if (!parseGaitProfile(payload["profile"].as<JsonObjectConst>(), &profile)) {
      acknowledgeGait(action, requestId, false,
                      "Profile schema or one of the thirteen bounded gait settings is invalid.");
    } else if (!persistGaitProfile(profile)) {
      acknowledgeGait(action, requestId, false,
                      "NVS verification failed; the previous gait remains active.");
    } else {
      acknowledgeGait(action, requestId, true);
    }
  } else if (!strcmp(action, "revert-profile")) {
    if (!revertGaitProfile()) {
      acknowledgeGait(action, requestId, false, "No verified previous gait profile is available.");
    } else {
      acknowledgeGait(action, requestId, true);
    }
  } else {
    acknowledgeGait(action, requestId, false, "Unsupported gait action.");
  }
}

void handleCommand(const String &line, LiveTransport source,
                   Adafruit_PWMServoDriver &driver, uint32_t now) {
  JsonDocument document;
  if (deserializeJson(document, line) != DeserializationError::Ok) return;
  JsonObjectConst command = document.as<JsonObjectConst>();
  if (strcmp(command["protocol"] | "", kProtocol)) return;
  if (!strcmp(command["type"] | "", "companion-auth")) {
    const bool authenticated = source != LiveTransport::Usb &&
        constantTimeLinkKeyMatches(command["linkKey"] | static_cast<const char *>(nullptr));
#if DOMINO_LIVE_WIFI_ENABLED
    if (source == LiveTransport::Wifi) wifiAuthenticated = authenticated;
#endif
#if DOMINO_LIVE_BLUETOOTH_ENABLED
    if (source == LiveTransport::Bluetooth) bluetoothAuthenticated = authenticated;
#endif
    if (authenticated) sendHello();
    return;
  }
  if (strcmp(command["type"] | "", "companion-command")) return;
  if (!transportAuthenticated(command, source)) return;
  const char *kind = command["kind"] | "";
  const char *action = command["action"] | "";
  const bool emergencyEstop = !strcmp(kind, "safety") && !strcmp(action, "estop");
  if (!emergencyEstop) {
    if (commandOwner == LiveTransport::None) commandOwner = source;
    if (commandOwner != source) return;
    lastOwnerCommandMs = now;
  }
  JsonObjectConst payload = command["payload"].as<JsonObjectConst>();
  if (!strcmp(kind, "safety")) handleSafety(command, payload, driver, now);
  else if (!strcmp(kind, "safety-heartbeat")) handleHeartbeat(command, now);
  else if (!strcmp(kind, "manual-authority")) handleManualAuthority(command, payload, now);
  else if (!strcmp(kind, "manual-control")) handleManualControl(command, now);
  else if (!strcmp(kind, "calibration")) handleCalibration(command, payload, driver, now);
  else if (!strcmp(kind, "gait")) handleGait(command, payload);
  else if (command["requestId"].is<const char*>())
    acknowledge(kind, command["action"] | "", command["requestId"], false, "Capability is not implemented by this firmware.");
}

void readTransport(Stream &stream, String &line, LiveTransport source,
                   Adafruit_PWMServoDriver &driver, uint32_t now) {
  while (stream.available()) {
    const char next = static_cast<char>(stream.read());
    if (next == '\n') {
      if (line.length()) handleCommand(line, source, driver, now);
      line = "";
    } else if (next != '\r') {
      if (line.length() < 8192) line += next;
      else line = "";
    }
  }
}

bool ownerConnected() {
  if (commandOwner == LiveTransport::Usb || commandOwner == LiveTransport::None) return true;
#if DOMINO_LIVE_WIFI_ENABLED
  if (commandOwner == LiveTransport::Wifi) return wifiClient && wifiClient.connected();
#endif
#if DOMINO_LIVE_BLUETOOTH_ENABLED
  if (commandOwner == LiveTransport::Bluetooth) return bluetoothSerial.hasClient();
#endif
  return false;
}

void updateWirelessTransports(Adafruit_PWMServoDriver &driver, uint32_t now) {
#if DOMINO_LIVE_WIFI_ENABLED
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiServerStarted) {
      wifiServer.begin();
      wifiServer.setNoDelay(true);
      wifiServerStarted = true;
      Serial.printf("LIVE Wi-Fi ready at %s:%u\n",
                    WiFi.localIP().toString().c_str(), DOMINO_LIVE_WIFI_PORT);
    }
    WiFiClient candidate = wifiServer.available();
    if (candidate) {
      if (!wifiClient || !wifiClient.connected()) {
        wifiClient = candidate;
        wifiClient.setNoDelay(true);
        wifiInputLine = "";
        wifiAuthenticated = false;
        wifiAcceptedMs = now;
        sendHello();
      } else {
        candidate.stop();
      }
    }
    if (wifiClient && wifiClient.connected()) {
      readTransport(wifiClient, wifiInputLine, LiveTransport::Wifi, driver, now);
      if (!wifiAuthenticated && now - wifiAcceptedMs > kWirelessAuthenticationMs) {
        wifiClient.stop();
      }
    }
  }
#endif
#if DOMINO_LIVE_BLUETOOTH_ENABLED
  const bool bluetoothHasClient = bluetoothSerial.hasClient();
  if (bluetoothHasClient && !bluetoothHadClient) {
    bluetoothAuthenticated = false;
    bluetoothAcceptedMs = now;
    sendHello();
  }
  if (bluetoothHasClient) {
    readTransport(bluetoothSerial, bluetoothInputLine, LiveTransport::Bluetooth, driver, now);
    if (!bluetoothAuthenticated && now - bluetoothAcceptedMs > kWirelessAuthenticationMs) {
      bluetoothSerial.disconnect();
    }
  }
  if (!bluetoothHasClient) bluetoothAuthenticated = false;
  bluetoothHadClient = bluetoothHasClient;
#endif

  if (!ownerConnected()) {
    if (state == LiveRobotState::Armed) disableOutputs(driver, LiveRobotState::Watchdog);
    else if (benchMode) disableOutputs(driver, LiveRobotState::Disarmed);
    commandOwner = LiveTransport::None;
  } else if (commandOwner != LiveTransport::None && state != LiveRobotState::Armed && !benchMode &&
             now - lastOwnerCommandMs > kTransportOwnerIdleMs) {
    commandOwner = LiveTransport::None;
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
  commandCalibrationServoAngle(
      driver, calibrationChannel, calibrationPhysicalChannel, calibrationCurrentDeg);
}
}  // namespace

void liveRobotEndpointBegin(Adafruit_PWMServoDriver &driver) {
  radioControlEnabled = true;
  setServoOutputsEnabled(driver, false);
  if (!loadCalibrationProfile()) setServoCalibrationProfile(defaultServoCalibrationProfile());
  if (!loadGaitProfile()) setGaitProfile(defaultGaitProfile());
  powerMonitorBegin();
  loopRateWindowStartedMs = millis();
  loopRateIterations = 0;
  measuredLoopRateHz = 0.0f;
  // Match the hardware UART RX ring. Calibration and gait profile commands
  // are intentionally bounded below 4 KB but exceed the String's old 1 KB
  // reservation, which made long commands vulnerable to truncation.
  usbInputLine.reserve(4096);
#if DOMINO_LIVE_WIFI_ENABLED
  wifiInputLine.reserve(1024);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DOMINO_LIVE_WIFI_HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.begin(DOMINO_LIVE_WIFI_SSID, DOMINO_LIVE_WIFI_PASSWORD);
#endif
#if DOMINO_LIVE_BLUETOOTH_ENABLED
  bluetoothInputLine.reserve(1024);
  if (bluetoothSerial.begin(DOMINO_LIVE_BLUETOOTH_NAME) &&
      bluetoothSerial.setPin(DOMINO_LIVE_BLUETOOTH_PIN)) {
    Serial.printf("LIVE Bluetooth SPP ready as %s (PIN enabled).\n", DOMINO_LIVE_BLUETOOTH_NAME);
  } else {
    Serial.println("LIVE Bluetooth failed to start securely; USB recovery remains available.");
  }
#endif
  sendHello();
}

void liveRobotEndpointLoop(uint32_t now, Adafruit_PWMServoDriver &driver) {
  loopRateIterations += 1;
  const uint32_t loopWindowElapsedMs = now - loopRateWindowStartedMs;
  if (loopWindowElapsedMs >= 1000) {
    measuredLoopRateHz = static_cast<float>(loopRateIterations) * 1000.0f /
                         static_cast<float>(loopWindowElapsedMs);
    loopRateIterations = 0;
    loopRateWindowStartedMs = now;
  }
  readTransport(Serial, usbInputLine, LiveTransport::Usb, driver, now);
  updateWirelessTransports(driver, now);
  updateCalibrationJog(driver, now);
  updateManualControl(now);
  powerMonitorUpdate(now);
  updatePowerFault(driver, now);
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

void liveRobotEndpointSetExpectedFootTarget(uint8_t legIndex, float xMm, float yMm, float zMm) {
  if (legIndex >= 4) return;
  expectedFootTargetsMm[legIndex][0] = xMm;
  expectedFootTargetsMm[legIndex][1] = yMm;
  expectedFootTargetsMm[legIndex][2] = zMm;
}
LiveRobotState liveRobotEndpointState() { return state; }
bool liveRobotEndpointAllowsLocomotion() { return state == LiveRobotState::Armed; }
bool liveRobotEndpointAllowsRadioControl() {
  return radioControlEnabled && !benchMode &&
      state != LiveRobotState::Estopped && state != LiveRobotState::Fault &&
      state != LiveRobotState::Watchdog;
}
bool liveRobotEndpointCalibrationOwnsOutputs() { return benchMode; }
LiveManualControlSnapshot liveRobotEndpointManualControl() { return manualGuard.snapshot(); }
