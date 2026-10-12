#include "calibration_storage.h"

#include <string.h>

namespace {
void putU16(uint8_t *out, uint16_t value) {
  out[0] = static_cast<uint8_t>(value);
  out[1] = static_cast<uint8_t>(value >> 8);
}
void putU32(uint8_t *out, uint32_t value) {
  for (uint8_t i = 0; i < 4; ++i) out[i] = static_cast<uint8_t>(value >> (8 * i));
}
void putU64(uint8_t *out, uint64_t value) {
  for (uint8_t i = 0; i < 8; ++i) out[i] = static_cast<uint8_t>(value >> (8 * i));
}
void putF32(uint8_t *out, float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  putU32(out, bits);
}
uint16_t getU16(const uint8_t *in) {
  return static_cast<uint16_t>(in[0] | (in[1] << 8));
}
uint32_t getU32(const uint8_t *in) {
  uint32_t value = 0;
  for (uint8_t i = 0; i < 4; ++i) value |= static_cast<uint32_t>(in[i]) << (8 * i);
  return value;
}
uint64_t getU64(const uint8_t *in) {
  uint64_t value = 0;
  for (uint8_t i = 0; i < 8; ++i) value |= static_cast<uint64_t>(in[i]) << (8 * i);
  return value;
}
float getF32(const uint8_t *in) {
  const uint32_t bits = getU32(in);
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}
}  // namespace

uint32_t calibrationChecksum(const uint8_t *bytes, size_t length) {
  uint32_t hash = 2166136261u;
  for (size_t index = 0; index < length; ++index) {
    hash ^= bytes[index];
    hash *= 16777619u;
  }
  return hash;
}

const char* calibrationSourceName(CalibrationSource source) {
  switch (source) {
    case CalibrationSource::DefaultEmpty: return "default-empty";
    case CalibrationSource::DefaultFallback: return "default-fallback";
    case CalibrationSource::Nvs: return "nvs";
    case CalibrationSource::NvsBackup: return "nvs-backup";
    case CalibrationSource::NvsLegacy: return "nvs-legacy";
  }
  return "unknown";
}

const char* calibrationDecodeResultName(CalibrationDecodeResult result) {
  switch (result) {
    case CalibrationDecodeResult::Ok: return "ok";
    case CalibrationDecodeResult::WrongSize: return "wrong-size";
    case CalibrationDecodeResult::BadMagic: return "bad-magic";
    case CalibrationDecodeResult::BadVersion: return "bad-version";
    case CalibrationDecodeResult::BadChecksum: return "bad-checksum";
    case CalibrationDecodeResult::InvalidProfile: return "invalid-profile";
  }
  return "unknown";
}

size_t encodeCalibrationRecord(const ServoCalibrationProfile &profile,
                               uint8_t *out, size_t capacity) {
  if (!out || capacity < kCalibrationRecordSize ||
      !validateServoCalibrationProfile(profile)) return 0;
  memset(out, 0, kCalibrationRecordSize);
  putU32(out, kCalibrationRecordMagic);
  putU16(out + 4, kCalibrationRecordVersion);
  putU16(out + 6, profile.schemaVersion);
  putU64(out + 8, profile.savedAt);
  uint8_t *cursor = out + kCalibrationRecordHeaderBytes;
  for (const ServoCalibrationJoint &joint : profile.joints) {
    cursor[0] = joint.logicalChannel;
    cursor[1] = joint.channel;
    cursor[2] = static_cast<uint8_t>(joint.direction);
    cursor[3] = 0;
    putF32(cursor + 4, joint.offsetDeg);
    putF32(cursor + 8, joint.minimumDeg);
    putF32(cursor + 12, joint.maximumDeg);
    cursor += kCalibrationRecordJointBytes;
  }
  putU32(cursor, calibrationChecksum(out, kCalibrationRecordSize - sizeof(uint32_t)));
  return kCalibrationRecordSize;
}

CalibrationDecodeResult decodeCalibrationRecord(const uint8_t *bytes, size_t length,
                                                ServoCalibrationProfile *profile) {
  if (!bytes || !profile || length != kCalibrationRecordSize) {
    return CalibrationDecodeResult::WrongSize;
  }
  if (getU32(bytes) != kCalibrationRecordMagic) return CalibrationDecodeResult::BadMagic;
  const size_t checksumOffset = kCalibrationRecordSize - sizeof(uint32_t);
  if (getU32(bytes + checksumOffset) != calibrationChecksum(bytes, checksumOffset)) {
    return CalibrationDecodeResult::BadChecksum;
  }
  if (getU16(bytes + 4) != kCalibrationRecordVersion) return CalibrationDecodeResult::BadVersion;
  ServoCalibrationProfile candidate{};
  candidate.schemaVersion = getU16(bytes + 6);
  candidate.savedAt = getU64(bytes + 8);
  const uint8_t *cursor = bytes + kCalibrationRecordHeaderBytes;
  for (ServoCalibrationJoint &joint : candidate.joints) {
    joint.logicalChannel = cursor[0];
    joint.channel = cursor[1];
    joint.direction = static_cast<int8_t>(cursor[2]);
    joint.offsetDeg = getF32(cursor + 4);
    joint.minimumDeg = getF32(cursor + 8);
    joint.maximumDeg = getF32(cursor + 12);
    cursor += kCalibrationRecordJointBytes;
  }
  if (!validateServoCalibrationProfile(candidate)) return CalibrationDecodeResult::InvalidProfile;
  *profile = candidate;
  return CalibrationDecodeResult::Ok;
}

CalibrationDecodeResult decodeLegacyCalibrationRecord(const uint8_t *bytes, size_t length,
                                                      ServoCalibrationProfile *profile) {
  if (!bytes || !profile || length != sizeof(LegacyStoredCalibrationProfile)) {
    return CalibrationDecodeResult::WrongSize;
  }
  LegacyStoredCalibrationProfile stored;
  memcpy(&stored, bytes, sizeof(stored));
  if (stored.magic != kLegacyCalibrationMagic) return CalibrationDecodeResult::BadMagic;
  if (stored.checksum != calibrationChecksum(
          reinterpret_cast<const uint8_t *>(&stored.profile), sizeof(stored.profile))) {
    return CalibrationDecodeResult::BadChecksum;
  }
  if (!validateServoCalibrationProfile(stored.profile)) return CalibrationDecodeResult::InvalidProfile;
  *profile = stored.profile;
  return CalibrationDecodeResult::Ok;
}
