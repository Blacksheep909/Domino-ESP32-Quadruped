// Host test for the persistent calibration record. It builds without Arduino
// or ESP32 headers so a layout/regression failure is caught before flashing.
#include <math.h>
#include <string.h>

#include <iostream>

#include "calibration_storage.h"

namespace {
int failures = 0;

void expect(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << "\n";
    ++failures;
  }
}

ServoCalibrationProfile ownerProfile() {
  ServoCalibrationProfile profile = defaultServoCalibrationProfile();
  profile.savedAt = 1790639152123ull;  // exceeds 32 bits, like Date.now()
  const float offsets[DOMINO_DRIVEN_JOINT_COUNT] = {
      -5.0f, 5.0f, -3.0f, 7.0f, -0.5f, 1.0f, 3.0f, 8.5f, -9.0f, 0.0f, -6.0f, -3.0f};
  for (uint8_t i = 0; i < DOMINO_DRIVEN_JOINT_COUNT; ++i) profile.joints[i].offsetDeg = offsets[i];
  for (ServoCalibrationJoint &joint : profile.joints) {
    if (joint.logicalChannel == 0 || joint.logicalChannel == 14) joint.direction = -1;
  }
  profile.joints[2].minimumDeg = -12.5f;
  profile.joints[2].maximumDeg = 33.25f;
  // A remapped cable: logical 4 now drives physical 6 (unused), 6 stays free.
  profile.joints[4].channel = 6;
  return profile;
}

bool sameProfile(const ServoCalibrationProfile &a, const ServoCalibrationProfile &b) {
  if (a.schemaVersion != b.schemaVersion || a.savedAt != b.savedAt) return false;
  for (uint8_t i = 0; i < DOMINO_DRIVEN_JOINT_COUNT; ++i) {
    const ServoCalibrationJoint &x = a.joints[i];
    const ServoCalibrationJoint &y = b.joints[i];
    if (x.logicalChannel != y.logicalChannel || x.channel != y.channel ||
        x.direction != y.direction || x.offsetDeg != y.offsetDeg ||
        x.minimumDeg != y.minimumDeg || x.maximumDeg != y.maximumDeg) return false;
  }
  return true;
}
}  // namespace

int main() {
  const ServoCalibrationProfile profile = ownerProfile();
  expect(validateServoCalibrationProfile(profile), "owner fixture must be valid");

  uint8_t record[kCalibrationRecordSize];
  expect(encodeCalibrationRecord(profile, record, sizeof(record)) == kCalibrationRecordSize,
         "encode returns the record size");
  uint8_t tooSmall[kCalibrationRecordSize - 1];
  expect(encodeCalibrationRecord(profile, tooSmall, sizeof(tooSmall)) == 0,
         "encode refuses a short buffer");

  ServoCalibrationProfile decoded{};
  expect(decodeCalibrationRecord(record, sizeof(record), &decoded) == CalibrationDecodeResult::Ok,
         "round trip decodes");
  expect(sameProfile(profile, decoded), "round trip preserves every field incl. 64-bit savedAt");

  // Layout is pinned: a struct or compiler change must not alter stored bytes.
  expect(kCalibrationRecordSize == 212, "record size is pinned at 212 bytes");
  expect(record[0] == 0x44 && record[1] == 0x43 && record[2] == 0x41 && record[3] == 0x32,
         "record starts with DCA2 magic");
  expect(record[8] == 0xfb && record[9] == 0x1b && record[12] == 0xa0,
         "savedAt is little-endian");

  ServoCalibrationProfile invalid = profile;
  invalid.joints[0].direction = 0;
  expect(encodeCalibrationRecord(invalid, record, sizeof(record)) == 0, "invalid profile is not encoded");
  encodeCalibrationRecord(profile, record, sizeof(record));

  // Every single-bit flip anywhere in the record must be rejected.
  int accepted = 0;
  for (size_t byte = 0; byte < kCalibrationRecordSize; ++byte) {
    for (int bit = 0; bit < 8; ++bit) {
      uint8_t corrupt[kCalibrationRecordSize];
      memcpy(corrupt, record, sizeof(corrupt));
      corrupt[byte] ^= static_cast<uint8_t>(1u << bit);
      ServoCalibrationProfile ignored{};
      if (decodeCalibrationRecord(corrupt, sizeof(corrupt), &ignored) == CalibrationDecodeResult::Ok) {
        ++accepted;
      }
    }
  }
  expect(accepted == 0, "no single-bit corruption is accepted");

  ServoCalibrationProfile untouched = defaultServoCalibrationProfile();
  uint8_t bad[kCalibrationRecordSize];
  memcpy(bad, record, sizeof(bad));
  bad[0] ^= 1;
  expect(decodeCalibrationRecord(bad, sizeof(bad), &untouched) == CalibrationDecodeResult::BadMagic,
         "wrong magic is named");
  expect(sameProfile(untouched, defaultServoCalibrationProfile()),
         "failed decode leaves the output untouched");
  expect(decodeCalibrationRecord(record, sizeof(record) - 1, &untouched) ==
             CalibrationDecodeResult::WrongSize, "short record is a size error");

  // A future record version is reported as BadVersion, never as corruption.
  memcpy(bad, record, sizeof(bad));
  bad[4] = 2;
  const uint32_t checksum = calibrationChecksum(bad, sizeof(bad) - 4);
  for (int i = 0; i < 4; ++i) bad[sizeof(bad) - 4 + i] = static_cast<uint8_t>(checksum >> (8 * i));
  expect(decodeCalibrationRecord(bad, sizeof(bad), &untouched) == CalibrationDecodeResult::BadVersion,
         "unknown record version is reported");

  // Firmware <= 0.8.15 blob migrates without losing the owner's values.
  LegacyStoredCalibrationProfile legacy{};
  legacy.magic = kLegacyCalibrationMagic;
  legacy.profile = profile;
  legacy.checksum = calibrationChecksum(
      reinterpret_cast<const uint8_t *>(&legacy.profile), sizeof(legacy.profile));
  uint8_t legacyBytes[sizeof(legacy)];
  memcpy(legacyBytes, &legacy, sizeof(legacy));
  ServoCalibrationProfile migrated{};
  expect(decodeLegacyCalibrationRecord(legacyBytes, sizeof(legacyBytes), &migrated) ==
             CalibrationDecodeResult::Ok, "legacy blob decodes");
  expect(sameProfile(profile, migrated), "legacy blob preserves the owner's profile");
  legacyBytes[sizeof(legacyBytes) / 2] ^= 0x10;
  // Padding bytes are covered by the checksum but carry no data, so flips there
  // may legitimately still fail; the point is a flip never yields a wrong profile.
  ServoCalibrationProfile maybe{};
  if (decodeLegacyCalibrationRecord(legacyBytes, sizeof(legacyBytes), &maybe) ==
      CalibrationDecodeResult::Ok) {
    expect(sameProfile(profile, maybe), "legacy corruption never yields a different profile");
  }
  expect(decodeLegacyCalibrationRecord(record, sizeof(record), &migrated) ==
             CalibrationDecodeResult::WrongSize, "new record is not mistaken for a legacy blob");

  expect(calibrationSourceIsDefault(CalibrationSource::DefaultEmpty) &&
             calibrationSourceIsDefault(CalibrationSource::DefaultFallback) &&
             !calibrationSourceIsDefault(CalibrationSource::Nvs) &&
             !calibrationSourceIsDefault(CalibrationSource::NvsBackup) &&
             !calibrationSourceIsDefault(CalibrationSource::NvsLegacy),
         "only default sources are flagged as defaults");

  if (failures == 0) std::cout << "calibration storage: all checks passed\n";
  return failures == 0 ? 0 : 1;
}
