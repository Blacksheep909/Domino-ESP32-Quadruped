# Domino movement capture, 25 September 2026

The owner confirmed that the intermittent ESP32 disconnect / ELRS freeze **did not occur during this recording**. This is a movement and telemetry-quality investigation, not a reproduction or diagnosis of that fault.

Update, 26 September: **roll now works correctly after the owner inverted FL and BL hips in Calibration and wrote the profile to the dog. Pitch still shifts the body with planted feet, including while held steady.** Vertical left-stick motion remains straight and satisfactory. The earlier blanket hip-sign diagnosis is withdrawn below. See [the completed cross-layer audit](full-motion-audit-2026-09-26.md) for current findings and numerical checks.

## Evidence

Sources: `domino-live-engineering-session-2026-09-25T08-35-47-114Z.json` and `domino-live-session-2026-09-25T08-35-50-701Z.csv` from the owner's Downloads directory. Both contain the same 281 sample times. The recorded interval was 08:34:56.560–08:35:26.233 UTC, lasting 29.673 seconds. The final sample is at 29.626 seconds.

| Observation | Result | Interpretation |
| --- | --- | --- |
| Largest interval between recorded samples | 207 ms | No long telemetry outage in this capture |
| Largest interval between expected-pose timestamps | 216 ms | These are host-normalized timestamps, not raw ESP32 uptime |
| Dropped / rejected packet counters | Both remain 19 throughout | The totals predate this recording; they do not show 19 new losses during it |
| Main-loop rate | 43.74–74.85 Hz | This is loop iteration rate, not a direct measurement of servo update rate |
| Changes in commanded joint-angle vector | 93 | Every recorded body-command change also changes joint-angle commands |
| Expected pitch | −9.98° to +10.00° | Right-stick forward/back exercise |
| Measured pitch | −21.79° to +12.57° | Physical attitude shows larger excursions; transient acceleration and mechanical behavior remain relevant |
| Expected roll | −3.64° to +1.64° | This run contains limited side-to-side roll, not a full-range roll test |
| Measured roll | −7.71° to +11.68° | Initial body alignment, compliance and motion affect direct error comparisons |
| Pack voltage | 14.53–15.305 V | Pack-level, sampled voltage does not measure the ESP32 supply rail or rule out brief brownouts in other runs |

The old firmware sends body pose and commanded joint angles at about 10 Hz, but pulse widths and foot targets only once per second. Studio carries those details forward without retaining their original timestamp. Their apparent one-second plateaus are therefore **not motor-freeze evidence**. The joint angles and pulse values are commands, not encoder measurements.

Measured height, joint angles, current and electrical power are unavailable. The old summary incorrectly computed `abs(null)` as zero height error. The roughly 49° initial yaw is a relative integrated gyro heading, not an absolute heading aligned with the commanded zero. Neither yaw error nor accelerometer-derived pitch/roll identifies the position of a rotation pivot or proves whether feet slipped.

## Working hip calibration confirmed by the owner

The owner inverted front-left CH0 and back-left CH14 in the active calibration and verified correct, body-centred roll on the physical dog. That observation is stronger evidence than interpreting default direction signs alone.

The existing solver uses an outward-positive hip coordinate on each side. The active calibration then maps that coordinate to the installed servo shaft. Mirrored canonical coordinates do not require mirrored physical servo mountings. The owner's left inversions make the physical hip commands move together during roll while preserving the existing solver convention.

The earlier unflashed change removed the side sign in the solver and renderer. Applied together with the now-working saved profile, it would reverse the right pair relative to the left pair again. That change has been withdrawn; the original canonical basis and physical profile compatibility are preserved. No saved calibration has been rewritten.

The revised regression verifies all four physical commands under a left-inverted profile, through both CAD IK and the forced analytic fallback. Separate default-profile tests check canonical roll signs. The CAD endpoint checks still reconstruct planted feet for both signs of roll, pitch and yaw.

## Forward/back body pitch

Forward/back right-stick motion is pitch; sideways right-stick motion is roll. The owner additionally reports that left-stick height changes move the body vertically as intended. That supports the height path but does not validate the coupled fore/aft movement needed during pitch, or the hip directions needed during roll.

The current body transform keeps world-space foot targets fixed and uses the inverse body rotation to express each target in its leg frame. At the recorded +10° pitch and 279.94 mm height, its fore/aft leg target changes from −15.75 mm to +27.76 mm at the front and +32.85 mm at the rear: approximately **44–49 mm of fore/aft compensation**. The recorded upper/lower commands agree with this transform after removing the apparent calibration offsets. The compensation is present in the commands; a Z-only pitch algorithm is not supported as the cause in this run.

### Direct command replay, without physics assistance

The current reproducible audit is `simulation/audit/run_pose_audit.py` with `pose_probe.cpp`. It calls production body/leg IK, independently reconstructs the closed CAD linkage from commands, and fits chassis translation and rotation from planted feet without physics assistance. Earlier scratch replays have been superseded by this audit.

The original export does not contain the active calibration profile. The replay uses a **candidate profile inferred from command residuals**, consistent with the existing reviewed calibration fixture: offsets −5, +5, −3, +7, −0.5, +1, +3, +8.5, −9, 0, −6, −3 degrees in logical channel order 0, 1, 2, 3, 4, 7, 8, 9, 10, 11, 14, 15. This assumption is not a measurement of the horn alignment. The initial stand transition is excluded: before approximately 4.3 seconds, the reported body height does not describe the intermediate joint pose.

| Check | Result under that geometry/calibration assumption |
| --- | --- |
| Upper/lower recorded commands versus production replay, 239 post-transition rows | Maximum difference 0.021° |
| Isolated pitch, 39 rows with zero roll and absolute pitch above 1° | Maximum reconstructed planted-foot error 0.763 mm |
| Actual recorded isolated-pitch commands with inferred calibration removed | Maximum predicted centre displacement 0.116 mm |
| Full independent audit across default and candidate profiles | 6,267 envelope poses plus all 281 recorded poses; see the updated report |

These are **predicted geometric errors**, not measured foot travel. They show that pitch command geometry is consistent with the intended pivot, conditional on CAD and calibration being physically accurate. The earlier 34.6 mm hip calculation used an assumed common-axis interpretation and is withdrawn as evidence for a firmware sign change.

### Why a convincing simulator is insufficient

The Rapier model has direct body-attitude torque, body-position/height forces and foot-anchor assistance. Its sagittal proxy joints are also driven from desired foot targets, rather than independently validating the actual two servo commands through the physical closed linkage. The rendered TILT body attitude follows the requested pose. The existing contact-physics tests therefore check an assisted visualization; they cannot establish that the physical chassis rotates about the same point.

With planted feet confirmed, the observed low pivot means body rotation is accompanied by fore/aft translation. The owner confirms that this translation persists at a held pitch, so transient rocking alone cannot explain it. A centre-preserving pitch requires both differential front/rear extension and a common fore/aft leg adjustment. An incorrect ratio between those physical movements can preserve good height-only motion while producing the reported arc. This narrows investigation to the static realization of the coupled pitch commands: actual linkage geometry, joint neutral, angular gain, mapping and loaded deflection. It does not identify which is wrong.

The source body origin is at CAD Z = 0, with hip axes 10.5 mm above it. That known offset produces only 1.82 mm of hip-plane-centre fore/aft travel at 10° pitch; it must not be confused with a large arc about a point far below the chassis. Both firmware and CAD reconstruction use that same origin.

The next useful physical check is an isolated, small fore/aft body translation with roll and pitch held at zero, compared with the already-working vertical translation. Studio's existing manual STAND body-X control exercises this separately (full-scale is 15 mm); no new drive mode or guessed pivot correction is needed. If that motion is wrong, investigate the horizontal linkage/actuator response directly. If it is correct, compare the combined pitch motion, its timing and the actual stance geometry. A held small pitch with measured chassis-centre displacement would quantify the apparent pivot depth. The recorded IMU alone cannot supply that displacement.

For a simple pitch about a point directly below the level body centre, horizontal displacement is `depth * sin(actual pitch change)`. This is a geometric relation, not an estimate of this robot's depth: neither the displacement nor a settled physical pitch/height pair was measured in this capture. No speculative pitch-pivot offset or gain has been added.

## LIVE joint-display calibration correction

The LIVE renderer previously fed calibrated electrical servo angles directly into the fixed, untrimmed CAD reference. Hardware offsets and reversed calibration directions were consequently drawn as real changes in linkage pose. The replay predicts up to 51.5 mm of apparent foot error during isolated pitch from drawing the candidate electrical commands this way, even though calibration-aware reconstruction stays within 0.763 mm.

Firmware now publishes `expected.modelServoAngleDeg` alongside the original `servoAngleDeg`. It reverses the **active firmware calibration on the final limited command**, preserving both user-limit and hard-limit effects. Studio uses the model angles for its LIVE skeleton and retains electrical angles/pulses separately in recordings. Browser draft calibration cannot change the interpretation of a robot command. JSON/CSV exports retain both bases; older firmware keeps the legacy display because the missing active calibration cannot be reconstructed reliably. These values remain commands, not encoders, and do not prove the horn or linkage reached them.

## Disconnect investigation: preparation, not a claimed fix

No cause of the intermittent disconnect is established by this capture. Firmware 0.8.5 includes preventive changes and evidence for the next occurrence:

- Increase the CRSF UART receive ring from its platform default of 256 bytes to 4096 bytes. At 500 Hz, 26-byte channel frames alone fill 256 bytes in about 19.7 ms. This addresses limited buffering during other firmware work; this recording does not establish the actual RF packet rate or prove an overflow occurred.
- Bound each LIVE transport read to 512 bytes per loop, retaining partial commands across iterations.
- Skip whole periodic USB telemetry/hello messages if the transmit ring has insufficient space. Explicit replies retain the normal delivery path. This change does not make Wi-Fi or Bluetooth writes nonblocking.
- Report a boot identifier, raw uptime, ESP-IDF reset reason, maximum observed loop gap, skipped telemetry count, CRSF accepted-frame/CRC-error/UART-overflow counters, body mode, stick-centering interlock and radio/output state.
- Preserve diagnostics and all 16 received controller channels in JSON and CSV session exports. Preserve independent source timestamps for the slower pulse/foot snapshots. Old exports remain readable, and unknown feedback remains null.

On a subsequent freeze: a changed boot identifier or restarted uptime distinguishes a new boot from continued execution. Reset reason 9 identifies a brownout reset; reasons 5/6/7 identify watchdog resets. A boot reason describes the current boot, so it must be interpreted with a boot change or uptime reset, not as a repeated new event in every row. ESP-IDF definitions: [Espressif system API](https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/system/system.html).

With unchanged boot identity, inspect received channel values, frame age, CRSF counters, maximum loop gap, output enable state and the stick-centering interlock around the pause. These distinguish input loss, firmware delay and an intentionally inhibited output. Pulse commands still cannot prove that servo power remained present or that a joint actually moved.

## Delivery boundary

Validation completed:

- Studio's configured regression suite passed, including calibration-aware command selection, backward-compatible recordings and independent model/electrical exports. CAD, linkage and assisted-physics tests also passed; the assistance limits described above apply.
- The production-source SIL scenario passed, including the owner's left-inverted hip convention through CAD IK and forced fallback, calibration inversion with travel/hard clipping, CRC/burst recovery, existing mode interlocks and failsafe stow.
- PlatformIO built ESP32 firmware 0.8.5 successfully: `.pio/build/esp32dev/firmware.bin`.
- Vite built the updated Studio frontend successfully.
- Studio re-imported all 281 samples from the owner's original JSON and correctly returned unavailable height error as null.

Changes are in the local repository, with the updated SIL built separately as `simulation/sil/bin/domino_sil_0_8_5.exe`. No firmware upload, robot movement, saved-calibration rewrite or installed-app replacement is performed by this investigation. The owner has physically validated the left-hip calibration correction. Pitch remains unresolved on hardware, and the intermittent freeze still needs a capture of an actual occurrence. Updated Studio code and firmware are both required to collect the new diagnostic fields.
