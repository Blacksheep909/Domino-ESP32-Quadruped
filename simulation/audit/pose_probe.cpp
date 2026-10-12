// Offline audit only: runs production kinematics against the SIL peripherals.
// This program has no serial transport and cannot command physical hardware.
#include "../../src/main.cpp"
#include <cstdio>

int main() {
  float height, roll, pitch, yaw, bodyX, bodyY;
  int profileIndex;
  const float offsets[12] = {-5, 5, -3, 7, -.5f, 1, 3, 8.5f, -9, 0, -6, -3};
  while (scanf("%d %f %f %f %f %f %f", &profileIndex, &height, &roll,
               &pitch, &yaw, &bodyX, &bodyY) == 7) {
    ServoCalibrationProfile profile = defaultServoCalibrationProfile();
    if (profileIndex >= 1) {
      for (int i = 0; i < 12; ++i) profile.joints[i].offsetDeg = offsets[i];
    }
    if (profileIndex == 2) {
      for (auto &joint : profile.joints) {
        if (joint.logicalChannel == 0 || joint.logicalChannel == 14) joint.direction = -1;
      }
    }
    setServoCalibrationProfile(profile);
    const uint32_t beforeClips = servoSafetyClipCount();
    moveLegsFromBodyPose(pca, bodyX, bodyY, height - 280, roll, pitch, yaw);
    printf("{\"angles\":[");
    for (int i = 0; i < 16; ++i) printf("%s%.7g", i ? "," : "", commandedServoAnglesDeg()[i]);
    printf("],\"model\":[");
    for (int i = 0; i < 16; ++i) printf("%s%.7g", i ? "," : "", commandedModelServoAnglesDeg()[i]);
    printf("],\"pulses\":[");
    for (int i = 0; i < 16; ++i) printf("%s%u", i ? "," : "", commandedServoPulseUs()[i]);
    printf("],\"targets\":[");
    for (int i = 0; i < 4; ++i) printf("%s[%.7g,%.7g,%.7g]", i ? "," : "", silLegCommandX[i], silLegCommandY[i], silLegCommandZ[i]);
    printf("],\"hardClips\":%lu}\n", static_cast<unsigned long>(servoSafetyClipCount() - beforeClips));
  }
}
