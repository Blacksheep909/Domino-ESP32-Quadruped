#pragma once

#include <stddef.h>
#include <stdint.h>

#include "servo_calibration.h"

// Persistent calibration record.
//
// The v2 record is serialised field by field in little-endian order instead of
// dumping the C++ struct. Compiler padding, member order or a future struct
// change can therefore no longer make a saved profile unreadable. Unknown
// record versions are rejected explicitly and reported, never silently
// replaced by defaults.
constexpr uint32_t kCalibrationRecordMagic = 0x32414344u;  // "DCA2"
constexpr uint16_t kCalibrationRecordVersion = 1;
constexpr size_t kCalibrationRecordJointBytes = 16;
constexpr size_t kCalibrationRecordHeaderBytes = 16;
constexpr size_t kCalibrationRecordSize =
    kCalibrationRecordHeaderBytes +
    kCalibrationRecordJointBytes * DOMINO_DRIVEN_JOINT_COUNT + sizeof(uint32_t);

enum class CalibrationDecodeResult : uint8_t {
  Ok,
  WrongSize,
  BadMagic,
  BadVersion,
  BadChecksum,
  InvalidProfile,
};

// Where the active in-memory profile came from. Reported to Studio so a
// robot running compiled defaults is never mistaken for a saved profile.
enum class CalibrationSource : uint8_t {
  DefaultEmpty,     // nothing was ever stored (first boot or flash erased)
  DefaultFallback,  // something was stored but could not be used
  Nvs,              // current record
  NvsBackup,        // previous record, used because the current one failed
  NvsLegacy,        // firmware <= 0.8.15 raw struct blob
};

const char* calibrationSourceName(CalibrationSource source);
const char* calibrationDecodeResultName(CalibrationDecodeResult result);
inline bool calibrationSourceIsDefault(CalibrationSource source) {
  return source == CalibrationSource::DefaultEmpty ||
         source == CalibrationSource::DefaultFallback;
}

// Returns kCalibrationRecordSize, or 0 if the profile is invalid or the
// buffer is too small.
size_t encodeCalibrationRecord(const ServoCalibrationProfile &profile,
                               uint8_t *out, size_t capacity);
CalibrationDecodeResult decodeCalibrationRecord(const uint8_t *bytes, size_t length,
                                                ServoCalibrationProfile *profile);

// Firmware <= 0.8.15 stored {magic, ServoCalibrationProfile, checksum} as a
// raw struct. Kept byte-for-byte compatible so an upgrade retains the saved
// profile; it is never written any more.
constexpr uint32_t kLegacyCalibrationMagic = 0x4443414cu;  // "DCAL"
struct LegacyStoredCalibrationProfile {
  uint32_t magic;
  ServoCalibrationProfile profile;
  uint32_t checksum;
};
CalibrationDecodeResult decodeLegacyCalibrationRecord(const uint8_t *bytes, size_t length,
                                                      ServoCalibrationProfile *profile);
uint32_t calibrationChecksum(const uint8_t *bytes, size_t length);
