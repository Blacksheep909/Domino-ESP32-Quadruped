#include <Arduino.h>
#include <windows.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "crsf.h"
#include "Adafruit_PWMServoDriver.h"
#include "gait_profile.h"
#include "leg_controller.h"
#include "sim_pwm.h"
#include "servo_calibration.h"

void setup();
void loop();

extern "C" int dominoSilBodyMode();
extern "C" float dominoSilTargetZ();
extern "C" float dominoSilPoseZ();
extern "C" bool dominoSilTiltActive();
extern "C" float dominoSilRideHeightMm();
extern "C" bool dominoSilGaitActive();
extern "C" bool dominoSilMotionInputArmed();
extern "C" float dominoSilGaitPhaseRad();
extern "C" float dominoSilGaitForwardCommand();
extern "C" float dominoSilGaitTurnCommand();
extern "C" float dominoSilLegCommandX(int legIndex);
extern "C" float dominoSilLegCommandY(int legIndex);
extern "C" float dominoSilLegCommandZ(int legIndex);
extern "C" float dominoSilBodyRollDeg();
extern "C" float dominoSilBodyPitchDeg();
extern "C" float dominoSilBodyYawDeg();

namespace {
constexpr uint32_t kStepMs = 5;
constexpr uint32_t kFrameIntervalMs = 20;
// Match the production 20 ms control loop so the rendered CAD receives every
// commanded servo frame instead of visibly stepping at 20 Hz.
constexpr uint32_t kTelemetryIntervalMs = 20;
constexpr uint32_t kScenarioDurationMs = 11500;
constexpr uint32_t kLoopPeriodMs = 12000;
constexpr int kRideHeightChannelIndex = 2;  // Boxer left-stick vertical / CRSF CH3.
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
constexpr DWORD CREATE_WAITABLE_TIMER_HIGH_RESOLUTION = 0x00000002;
#endif
constexpr uint8_t kServoChannels[12] = {0, 1, 2, 3, 4, 15, 14, 7, 8, 9, 10, 11};

bool validateCalibrationMath() {
  ServoCalibrationProfile profile = defaultServoCalibrationProfile();
  if (!validateServoCalibrationProfile(profile)) {
    std::cerr << "FAIL: default servo calibration profile is invalid\n";
    return false;
  }
  ServoCalibrationJoint *flUpper = nullptr;
  for (ServoCalibrationJoint &joint : profile.joints) {
    if (joint.logicalChannel == 1) flUpper = &joint;
  }
  if (flUpper == nullptr) return false;
  flUpper->offsetDeg = 3.0f;
  flUpper->minimumDeg = -10.0f;
  flUpper->maximumDeg = 20.0f;
  const float neutral = servoCalibrationNeutralDeg(1);
  const float normal = applyServoCalibration(profile, 1, neutral - 12.0f);
  flUpper->direction = 1;
  const float inverted = applyServoCalibration(profile, 1, neutral - 12.0f);
  const bool transformOk = fabsf(normal - (neutral - 9.0f)) < 0.01f &&
                           fabsf(inverted - (neutral + 15.0f)) < 0.01f;
  profile.joints[0].channel = 15;
  profile.joints[11].channel = 0;
  const bool remapAccepted = validateServoCalibrationProfile(profile) &&
                             servoCalibrationPhysicalChannel(profile, 0) == 15 &&
                             servoCalibrationPhysicalChannel(profile, 15) == 0;
  Adafruit_PWMServoDriver calibrationDriver;
  simResetServoOutputs();
  setServoOutputsEnabled(calibrationDriver, true);
  const bool profileActivated = setServoCalibrationProfile(profile);
  moveLegFL(calibrationDriver, 0.0f, 60.0f, 200.0f);
  const bool runtimeRouted = profileActivated &&
                             simServoWriteCount(15) > 0 &&
                             simServoWriteCount(0) == 0;
  setServoCalibrationProfile(defaultServoCalibrationProfile());
  simResetServoOutputs();
  profile.joints[1].channel = profile.joints[0].channel;
  const bool duplicateRejected = !validateServoCalibrationProfile(profile);
  profile = defaultServoCalibrationProfile();
  profile.joints[1].logicalChannel = profile.joints[0].logicalChannel;
  const bool duplicateLogicalRejected = !validateServoCalibrationProfile(profile);
  if (!transformOk || !remapAccepted || !runtimeRouted ||
      !duplicateRejected || !duplicateLogicalRejected) {
    std::cerr << "FAIL: calibration transform or logical-to-physical channel validation failed\n";
  }
  return transformOk && remapAccepted && runtimeRouted &&
         duplicateRejected && duplicateLogicalRejected;
}

bool validateGaitProfiles() {
  const GaitProfile original = gaitProfile();
  GaitProfile candidate = defaultGaitProfile();
  strcpy(candidate.name, "SIL profile");
  strcpy(candidate.settings.preset, "custom");
  candidate.settings.cadenceHz = 1.25f;
  candidate.settings.strideMm = 72.0f;
  candidate.settings.stanceWidthMm = 44.0f;
  const bool accepted = validateGaitProfile(candidate) && setGaitProfile(candidate) &&
      fabsf(gaitProfile().settings.cadenceHz - 1.25f) < 0.001f &&
      fabsf(gaitProfile().settings.stanceWidthMm - 44.0f) < 0.001f;
  candidate.settings.strideMm = 121.0f;
  const bool outOfBoundsRejected = !validateGaitProfile(candidate) && !setGaitProfile(candidate);
  candidate = defaultGaitProfile();
  candidate.settings.cadenceHz = NAN;
  const bool nonFiniteRejected = !validateGaitProfile(candidate);
  const bool restored = setGaitProfile(original);
  if (!accepted || !outOfBoundsRejected || !nonFiniteRejected || !restored) {
    std::cerr << "FAIL: gait profile bounds or runtime activation failed\n";
  }
  return accepted && outOfBoundsRejected && nonFiniteRejected && restored;
}

struct Options {
  uint32_t durationMs = kScenarioDurationMs;
  bool realtime = false;
  bool loopForever = false;
  std::string controlFile;
  std::string stateFile;
  std::string telemetryFile;
};

struct RealtimePacer {
  HANDLE timer;
  LARGE_INTEGER frequency;
  LONGLONG nextDeadline;
};

RealtimePacer createRealtimePacer() {
  typedef HANDLE(WINAPI* CreateWaitableTimerExAFunction)(
      LPSECURITY_ATTRIBUTES, LPCSTR, DWORD, DWORD);
  HMODULE kernel32 = GetModuleHandleA("kernel32.dll");
  CreateWaitableTimerExAFunction createTimerEx =
      kernel32 == NULL
          ? NULL
          : reinterpret_cast<CreateWaitableTimerExAFunction>(
                GetProcAddress(kernel32, "CreateWaitableTimerExA"));
  HANDLE timer = createTimerEx == NULL
                     ? NULL
                     : createTimerEx(
                           NULL,
                           NULL,
                           CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
                           TIMER_ALL_ACCESS);
  if (timer == NULL) {
    timer = CreateWaitableTimerA(NULL, FALSE, NULL);
  }
  LARGE_INTEGER frequency;
  LARGE_INTEGER now;
  QueryPerformanceFrequency(&frequency);
  QueryPerformanceCounter(&now);
  return {timer, frequency, now.QuadPart};
}

void waitRealtimeStep(RealtimePacer* pacer) {
  if (pacer == NULL || pacer->timer == NULL) {
    Sleep(kStepMs);
    return;
  }
  pacer->nextDeadline +=
      (pacer->frequency.QuadPart * static_cast<LONGLONG>(kStepMs)) / 1000LL;
  LARGE_INTEGER now;
  QueryPerformanceCounter(&now);
  const LONGLONG remainingTicks = pacer->nextDeadline - now.QuadPart;
  if (remainingTicks <= 0) {
    return;
  }
  LARGE_INTEGER dueTime;
  dueTime.QuadPart = -std::max<LONGLONG>(
      1,
      (remainingTicks * 10000000LL) / pacer->frequency.QuadPart);
  if (!SetWaitableTimer(pacer->timer, &dueTime, 0, NULL, NULL, FALSE)) {
    Sleep(kStepMs);
    return;
  }
  WaitForSingleObject(pacer->timer, INFINITE);
}

uint8_t crc8DvbS2(const uint8_t* bytes, size_t length) {
  uint8_t crc = 0;
  for (size_t i = 0; i < length; ++i) {
    crc ^= bytes[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1U) ^ 0xD5U)
                          : static_cast<uint8_t>(crc << 1U);
    }
  }
  return crc;
}

uint16_t microsecondsToCrsf(int microseconds) {
  const int clamped = std::max(1000, std::min(2000, microseconds));
  return static_cast<uint16_t>(172 + ((clamped - 1000) * 1639 + 500) / 1000);
}

std::vector<uint8_t> makeRcFrame(const int channelsUs[16]) {
  uint8_t payload[22] = {0};
  for (int channel = 0; channel < 16; ++channel) {
    const uint16_t raw = microsecondsToCrsf(channelsUs[channel]);
    const uint32_t bitStart = static_cast<uint32_t>(channel) * 11U;
    for (uint8_t bit = 0; bit < 11; ++bit) {
      if ((raw & (1U << bit)) != 0U) {
        const uint32_t destinationBit = bitStart + bit;
        payload[destinationBit >> 3U] |= static_cast<uint8_t>(1U << (destinationBit & 7U));
      }
    }
  }

  std::vector<uint8_t> frame;
  frame.reserve(26);
  frame.push_back(CRSF_ADDR_FC);
  frame.push_back(24);
  frame.push_back(CRSF_TYPE_RC_CHANNELS);
  frame.insert(frame.end(), payload, payload + 22);
  frame.push_back(crc8DvbS2(&frame[2], 23));
  return frame;
}

bool applyScenario(uint32_t scenarioMs, int channelsUs[16]) {
  for (int channel = 0; channel < 16; ++channel) {
    channelsUs[channel] = 1500;
  }

  channelsUs[SA_CH_INDEX] = 1000;
  channelsUs[kRideHeightChannelIndex] = 2000;
  channelsUs[5] = 2000;  // Legacy SB must not control ride height anymore.
  channelsUs[SC_CH_INDEX] = 1000;
  channelsUs[7] = 1000;

  if (scenarioMs < 500U || scenarioMs >= 9000U) {
    return false;
  }

  if (scenarioMs >= 1500U) {
    channelsUs[SA_CH_INDEX] = 2000;
  }

  if (scenarioMs >= 4500U && scenarioMs < 6000U) {
    channelsUs[7] = 2000;
    channelsUs[SC_CH_INDEX] = 2000;  // Gait request must remain blocked by SD/tilt.
    if (scenarioMs < 4700U) {
      // Deliberately hold the sticks while switching. The mode-input
      // interlock must suppress this command until a centred handoff occurs.
      channelsUs[0] = 1900;
      channelsUs[1] = 1800;
      channelsUs[3] = 1750;
    } else if (scenarioMs >= 4950U) {
      const float phase = static_cast<float>(scenarioMs - 4950U) * 0.0062831853f / 1000.0f;
      channelsUs[0] = 1500 + static_cast<int>(430.0f * sinf(phase));
      channelsUs[1] = 1500 + static_cast<int>(250.0f * cosf(phase * 0.7f));
      channelsUs[3] = 1500 + static_cast<int>(320.0f * sinf(phase * 0.5f));
    }
  }

  if (scenarioMs >= 6000U && scenarioMs < 7500U) {
    channelsUs[SC_CH_INDEX] = 1500;
    channelsUs[7] = 1000;
    if (scenarioMs >= 7200U) {
      channelsUs[3] = 1600;
      channelsUs[1] = 1900;
    }
  }

  if (scenarioMs >= 7500U && scenarioMs < 9000U) {
    channelsUs[SC_CH_INDEX] = 2000;
    channelsUs[7] = 1000;
    channelsUs[3] = 1680;  // Yaw / right-stick horizontal: gentle turn.
    channelsUs[1] = 1900;  // Right-stick vertical: forward travel.
  }

  if (scenarioMs >= 7500U && scenarioMs < 9000U) {
    const uint32_t heightProgressMs = scenarioMs - 7500U;
    channelsUs[kRideHeightChannelIndex] =
        2000 - static_cast<int>((1000U * heightProgressMs) / 1500U);
  }
  return true;
}

bool readControlFile(const std::string& path, int channelsUs[16]) {
  std::ifstream stream(path.c_str());
  if (!stream) {
    return false;
  }

  for (int channel = 0; channel < 16; ++channel) {
    int value = 1500;
    if (!(stream >> value)) {
      return false;
    }
    channelsUs[channel] = std::max(1000, std::min(2000, value));
  }
  return true;
}

const char* modeName(int mode) {
  switch (mode) {
    case 0:
      return "STOW";
    case 1:
      return "STAND";
    case 2:
      return "TILT";
    case 3:
      return "BALANCE";
    case 4:
      return "GAIT";
    case 5:
      return "CAREFUL";
    default:
      return "UNKNOWN";
  }
}

std::string makeStateJson(uint32_t elapsedMs, uint32_t scenarioMs) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(2);
  output << "{\"elapsed_ms\":" << elapsedMs;
  output << ",\"scenario_ms\":" << scenarioMs;
  output << ",\"mode\":\"" << modeName(dominoSilBodyMode()) << "\"";
  output << ",\"link_alive\":" << (crsfLinkAlive(millis()) ? "true" : "false");
  output << ",\"tilt_active\":" << (dominoSilTiltActive() ? "true" : "false");
  output << ",\"gait_active\":" << (dominoSilGaitActive() ? "true" : "false");
  output << ",\"motion_input_armed\":"
         << (dominoSilMotionInputArmed() ? "true" : "false");
  output << ",\"gait_phase_rad\":" << dominoSilGaitPhaseRad();
  output << ",\"gait_command\":["
         << dominoSilGaitForwardCommand() << ','
         << dominoSilGaitTurnCommand() << ']';
  output << ",\"ride_height_mm\":" << dominoSilRideHeightMm();
  output << ",\"accepted_frames\":" << crsfAcceptedFrameCount();
  output << ",\"target_z_mm\":" << dominoSilTargetZ();
  output << ",\"pose_z_mm\":" << dominoSilPoseZ();
  output << ",\"leg_command_xyz_mm\":[";
  for (int leg = 0; leg < 4; ++leg) {
    if (leg != 0) {
      output << ',';
    }
    output << '['
           << dominoSilLegCommandX(leg) << ','
           << dominoSilLegCommandY(leg) << ','
           << dominoSilLegCommandZ(leg) << ']';
  }
  output << ']';
  output << ",\"body_pose_rpy_deg\":["
         << dominoSilBodyRollDeg() << ','
         << dominoSilBodyPitchDeg() << ','
         << dominoSilBodyYawDeg() << ']';

  output << ",\"channels_us\":[";
  for (int channel = 0; channel < 16; ++channel) {
    if (channel != 0) {
      output << ',';
    }
    output << ch_us[channel];
  }
  output << ']';

  output << ",\"servo_angle_deg\":[";
  for (int channel = 0; channel < 16; ++channel) {
    if (channel != 0) {
      output << ',';
    }
    output << simServoAngleDeg(static_cast<uint8_t>(channel));
  }
  output << ']';

  output << ",\"servo_pulse_us\":[";
  for (int channel = 0; channel < 16; ++channel) {
    if (channel != 0) {
      output << ',';
    }
    output << simServoPulseUs(static_cast<uint8_t>(channel));
  }
  output << "]}";
  return output.str();
}

bool writeTextFile(const std::string& path, const std::string& content, bool append) {
  if (path.empty()) {
    return true;
  }

  std::ofstream stream(
      path.c_str(),
      append ? (std::ios::out | std::ios::app) : std::ios::out);
  if (!stream) {
    std::cerr << "Unable to write " << path << "\n";
    return false;
  }
  stream << content << '\n';
  stream.close();
  return true;
}

bool writeStateSnapshot(
    const std::string& path,
    const std::string& content,
    uint64_t sequence) {
  // MoveFileEx can stall indefinitely when Windows filesystem filters inspect
  // a destination that Node is polling. Alternate two independently-written
  // slots instead. Readers parse both and retain the newest complete frame;
  // if they catch one slot mid-write, the other remains valid.
  const std::string slotPath = path + "." + ((sequence & 1ULL) == 0 ? "0" : "1");
  return writeTextFile(slotPath, content, false);
}

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--realtime") {
      options.realtime = true;
    } else if (argument == "--loop") {
      options.loopForever = true;
    } else if (argument == "--duration-ms" && index + 1 < argc) {
      options.durationMs = static_cast<uint32_t>(strtoul(argv[++index], NULL, 10));
    } else if (argument == "--control-file" && index + 1 < argc) {
      options.controlFile = argv[++index];
    } else if (argument == "--state-file" && index + 1 < argc) {
      options.stateFile = argv[++index];
    } else if (argument == "--telemetry-file" && index + 1 < argc) {
      options.telemetryFile = argv[++index];
    }
  }
  return options;
}

bool validateOutputs(bool sawStand, bool sawTilt, bool sawGait, bool sawCareful,
                     bool gaitTiltInterlockViolation,
                     bool motionInputInterlockViolation,
                     bool gaitHeightTransitionDropout,
                     bool gaitSupportViolation,
                     bool sawGaitSwing,
                     bool sawGaitSupportOverlap,
                     bool carefulSupportViolation,
                     bool sawCarefulSwing,
                     float gaitMinX, float gaitMaxX, float gaitMaxLiftMm,
                     bool sawFailsafeStow,
                     float minRideHeightMm, float maxRideHeightMm,
                     const float minAngle[16], const float maxAngle[16]) {
  bool passed = true;
  for (size_t index = 0; index < sizeof(kServoChannels); ++index) {
    const uint8_t channel = kServoChannels[index];
    const uint16_t pulse = simServoPulseUs(channel);
    if (simServoWriteCount(channel) == 0 || pulse < 500U || pulse > 2500U ||
        !std::isfinite(simServoAngleDeg(channel))) {
      std::cerr << "FAIL: invalid output on servo channel " << static_cast<int>(channel) << "\n";
      passed = false;
    }
  }
  if (!sawStand) {
    std::cerr << "FAIL: stand mode was never reached\n";
    passed = false;
  }
  if (!sawTilt) {
    std::cerr << "FAIL: tilt mode was never reached\n";
    passed = false;
  }
  if (!sawGait) {
    std::cerr << "FAIL: gait mode was never reached\n";
    passed = false;
  }
  if (!sawCareful) {
    std::cerr << "FAIL: careful walk mode was never reached\n";
    passed = false;
  }
  if (gaitTiltInterlockViolation) {
    std::cerr << "FAIL: gait ran while the SD tilt request was active\n";
    passed = false;
  }
  if (motionInputInterlockViolation) {
    std::cerr << "FAIL: held sticks actuated the newly selected motion mode before centering\n";
    passed = false;
  }
  if (gaitHeightTransitionDropout) {
    std::cerr << "FAIL: gait dropped out while ride height changed\n";
    passed = false;
  }
  if (gaitSupportViolation || !sawGaitSwing || !sawGaitSupportOverlap) {
    std::cerr << "FAIL: gait did not preserve two-foot swing and four-foot transition support\n";
    passed = false;
  }
  if (carefulSupportViolation || !sawCarefulSwing) {
    std::cerr << "FAIL: careful walk did not preserve at least three planted feet\n";
    passed = false;
  }
  if ((gaitMaxX - gaitMinX) < 15.0f || gaitMaxLiftMm < 8.0f) {
    std::cerr << "FAIL: gait scenario did not produce enough stride and foot lift\n";
    passed = false;
  }
  if (!sawFailsafeStow) {
    std::cerr << "FAIL: link-loss failsafe did not return to stow\n";
    passed = false;
  }
  if ((maxRideHeightMm - minRideHeightMm) < 40.0f) {
    std::cerr << "FAIL: continuous ride-height scenario did not traverse the validated range\n";
    passed = false;
  }

  float maximumTravel = 0.0f;
  for (size_t index = 0; index < sizeof(kServoChannels); ++index) {
    const uint8_t channel = kServoChannels[index];
    maximumTravel = std::max(maximumTravel, maxAngle[channel] - minAngle[channel]);
  }
  if (maximumTravel < 5.0f) {
    std::cerr << "FAIL: tilt scenario did not move the simulated servos\n";
    passed = false;
  }

  // Deep stow changes only the sagittal linkage drives. The shoulder pulses
  // must return to their calibrated, straight high-stand references rather
  // than acquiring a height-dependent right-side abduction angle.
  constexpr uint8_t kHipChannels[4] = {0, 3, 14, 9};
  constexpr float kExpectedStraightHipDeg[4] = {129.87f, 105.44f, 119.88f, 115.42f};
  for (size_t index = 0; index < 4; ++index) {
    const float angle = simServoAngleDeg(kHipChannels[index]);
    if (fabsf(angle - kExpectedStraightHipDeg[index]) > 0.35f) {
      std::cerr << "FAIL: stow shoulder channel "
                << static_cast<int>(kHipChannels[index])
                << " drifted to " << angle << " degrees\n";
      passed = false;
    }
  }
  return passed;
}
}  // namespace

int main(int argc, char** argv) {
  const Options options = parseOptions(argc, argv);
  // A high-resolution waitable timer avoids Sleep(5) rounding up to the
  // default ~15.6 ms Windows scheduler tick and slowing the whole robot down.
  RealtimePacer realtimePacer = options.realtime
                                    ? createRealtimePacer()
                                    : RealtimePacer{NULL, LARGE_INTEGER(), 0};
  const bool calibrationMathPassed = validateCalibrationMath();
  const bool gaitProfilesPassed = validateGaitProfiles();
  simSetTimeUs(0);
  simResetServoOutputs();
  setup();

  float minAngle[16];
  float maxAngle[16];
  for (int channel = 0; channel < 16; ++channel) {
    minAngle[channel] = 10000.0f;
    maxAngle[channel] = -10000.0f;
  }

  bool sawStand = false;
  bool sawTilt = false;
  bool sawGait = false;
  bool sawCareful = false;
  bool gaitTiltInterlockViolation = false;
  bool motionInputInterlockViolation = false;
  bool gaitHeightTransitionStarted = false;
  bool gaitHeightTransitionDropout = false;
  bool gaitSupportViolation = false;
  bool sawGaitSwing = false;
  bool sawGaitSupportOverlap = false;
  bool carefulSupportViolation = false;
  bool sawCarefulSwing = false;
  bool sawFailsafeStow = false;
  float gaitMinX = 10000.0f;
  float gaitMaxX = -10000.0f;
  float gaitMaxLiftMm = 0.0f;
  float minRideHeightMm = 10000.0f;
  float maxRideHeightMm = -10000.0f;
  uint32_t nextFrameMs = 0;
  uint32_t nextTelemetryMs = 0;
  uint64_t statePublishSequence = 0;
  uint32_t elapsedMs = millis();

  while (options.loopForever || elapsedMs <= options.durationMs) {
    const uint32_t scenarioMs = elapsedMs % kLoopPeriodMs;
    int channelsUs[16];
    const bool interactiveControl = !options.controlFile.empty();
    const bool linkEnabled = interactiveControl
                                 ? readControlFile(options.controlFile, channelsUs)
                                 : applyScenario(scenarioMs, channelsUs);

    if (!interactiveControl && scenarioMs < kStepMs) {
      nextFrameMs = elapsedMs;
    }
    if (linkEnabled && elapsedMs >= nextFrameMs) {
      Serial2.inject(makeRcFrame(channelsUs));
      nextFrameMs = elapsedMs + kFrameIntervalMs;
    }

    loop();
    const int mode = dominoSilBodyMode();
    sawStand = sawStand || mode == 1;
    sawTilt = sawTilt || mode == 2;
    sawGait = sawGait || mode == 4;
    sawCareful = sawCareful || mode == 5;
    gaitTiltInterlockViolation =
        gaitTiltInterlockViolation || ((mode == 4 || mode == 5) && ch_us[7] > 1600);
    if (!dominoSilMotionInputArmed()) {
      motionInputInterlockViolation = motionInputInterlockViolation ||
          fabsf(dominoSilGaitForwardCommand()) > 0.01f ||
          fabsf(dominoSilGaitTurnCommand()) > 0.01f ||
          fabsf(dominoSilBodyRollDeg()) > 0.1f ||
          fabsf(dominoSilBodyPitchDeg()) > 0.1f ||
          fabsf(dominoSilBodyYawDeg()) > 0.1f;
    }
    if (!interactiveControl && scenarioMs >= 7500U && scenarioMs < 9000U) {
      gaitHeightTransitionStarted = gaitHeightTransitionStarted || mode == 4;
      if (gaitHeightTransitionStarted) {
        gaitHeightTransitionDropout = gaitHeightTransitionDropout || mode != 4;
      }
    }
    sawFailsafeStow = sawFailsafeStow || (elapsedMs > 10000U && mode == 0);
    const float rideHeightMm = dominoSilRideHeightMm();
    minRideHeightMm = std::min(minRideHeightMm, rideHeightMm);
    maxRideHeightMm = std::max(maxRideHeightMm, rideHeightMm);

    if (mode == 4 && dominoSilMotionInputArmed()) {
      int stanceLegCount = 0;
      for (int leg = 0; leg < 4; ++leg) {
        const float x = dominoSilLegCommandX(leg);
        const float z = dominoSilLegCommandZ(leg);
        gaitMinX = std::min(gaitMinX, x);
        gaitMaxX = std::max(gaitMaxX, x);
        const float liftMm = dominoSilPoseZ() - z;
        gaitMaxLiftMm = std::max(gaitMaxLiftMm, liftMm);
        if (liftMm <= 0.75f) {
          ++stanceLegCount;
        }
      }
      gaitSupportViolation = gaitSupportViolation || stanceLegCount < 2;
      sawGaitSwing = sawGaitSwing || stanceLegCount == 2;
      sawGaitSupportOverlap = sawGaitSupportOverlap || stanceLegCount == 4;
    }

    if (mode == 5 && dominoSilMotionInputArmed()) {
      int stanceLegCount = 0;
      for (int leg = 0; leg < 4; ++leg) {
        const float liftMm = dominoSilPoseZ() - dominoSilLegCommandZ(leg);
        if (liftMm <= 0.75f) {
          ++stanceLegCount;
        }
      }
      carefulSupportViolation = carefulSupportViolation || stanceLegCount < 3;
      sawCarefulSwing = sawCarefulSwing || stanceLegCount == 3;
    }

    for (size_t index = 0; index < sizeof(kServoChannels); ++index) {
      const uint8_t channel = kServoChannels[index];
      const float angle = simServoAngleDeg(channel);
      minAngle[channel] = std::min(minAngle[channel], angle);
      maxAngle[channel] = std::max(maxAngle[channel], angle);
    }

    if (elapsedMs >= nextTelemetryMs) {
      const std::string state = makeStateJson(elapsedMs, scenarioMs);
      if (!options.stateFile.empty()) {
        writeStateSnapshot(options.stateFile, state, statePublishSequence++);
      }
      if (!options.telemetryFile.empty()) {
        writeTextFile(options.telemetryFile, state, true);
      }
      nextTelemetryMs = elapsedMs + kTelemetryIntervalMs;
    }

    simAdvanceTimeUs(static_cast<uint64_t>(kStepMs) * 1000ULL);
    if (options.realtime) {
      waitRealtimeStep(&realtimePacer);
    }
    elapsedMs = millis();
  }

  if (options.loopForever) {
    return 0;
  }

  const bool passed = calibrationMathPassed && gaitProfilesPassed &&
      validateOutputs(sawStand, sawTilt, sawGait, sawCareful,
                                      gaitTiltInterlockViolation,
                                      motionInputInterlockViolation,
                                      gaitHeightTransitionDropout,
                                      gaitSupportViolation,
                                      sawGaitSwing,
                                      sawGaitSupportOverlap,
                                      carefulSupportViolation,
                                      sawCarefulSwing,
                                      gaitMinX, gaitMaxX, gaitMaxLiftMm,
                                      sawFailsafeStow,
                                      minRideHeightMm, maxRideHeightMm,
                                      minAngle, maxAngle);
  std::cout << "SIL " << (passed ? "PASS" : "FAIL")
            << ": frames=" << crsfAcceptedFrameCount()
            << " final_mode=" << modeName(dominoSilBodyMode())
            << " final_pose_z_mm=" << dominoSilPoseZ() << "\n";
  if (realtimePacer.timer != NULL) {
    CloseHandle(realtimePacer.timer);
  }
  return passed ? 0 : 1;
}
