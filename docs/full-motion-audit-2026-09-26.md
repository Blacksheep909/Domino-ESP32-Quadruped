# CAD, commands, simulation and physical motion audit

## New video and horn-reference evidence, 26 September

The owner confirms that the dog is upside down on a white support box in the
supplied 22.79-second video, with the four feet above the body and off the
floor throughout the visible sitting, standing, height and pitch motions.
The body remains near the support while
the leg assemblies move. This is useful evidence that commands reach the leg
drives, but it is not a loaded, planted-foot pitch trial and cannot locate the
rotation centre observed in the earlier ground-supported run. The owner's
earlier planted-feet observation remains a separate fact.

The horn-reference screenshot describes a nominal high STAND pose at 280 mm:
the forward servo horn is vertical, and the rear foot endpoint lies under its
hip pivot in side view. That matches the coded CAD neutral construction: its
forward upper-drive pin is 40 mm directly above the corresponding motor axis,
and its neutral foot is only about 2.54 mm fore/aft from the hip. In the video,
the visible forward horn is approximately vertical when the leg is near that
neutral pose, once the bench orientation is accounted for. This is a visual
consistency check, not a measured spline angle or proof that every real joint
has the expected gain, channel mapping or loaded position.

During the later stick movements the visible front and rear endpoints both
sweep in the fore/aft direction relative to their hip mounts while the body
stays supported. Thus the bench motion is more than a front/rear height-only
change. The clip lacks synchronized joint-command telemetry and a fixed
measurement scale, so it cannot tell whether **the endpoint travel has the
correct size under load**. The model needs roughly 44–49 mm at a 10° pitch
command. That remains the specific unverified part of the physical motion.
No servo sign or pivot constant is changed on the basis of this bench clip.

The owner's FL/BL hip inversion has physically corrected roll. **Pitch remains a static fore/aft displacement problem:** the feet stay planted, the body shifts during pitch, and the shift remains while pitch is held. Vertical motion is reported correct. The available evidence does not identify a physical pitch root cause yet. It does show that the production pitch equations and the recorded commands contain the compensation needed for a body-centred rotation.

The earlier proposed blanket hip sign change was withdrawn before deployment. Canonical outward-positive hip coordinates must remain compatible with the owner's now-working saved left-inverted calibration. No physical calibration, firmware upload, robot motion or installed-app replacement was performed in this audit.

## What was checked

| Layer | Check and result |
| --- | --- |
| Build CAD versus rendered CAD | All 29 STL files and the URDF are byte-identical across the build export and simulation asset directories. |
| CAD link pivots | Independent circle fits of 28 pin bores match the coded X/Z centres within 0.000593 mm. This is digital geometry agreement, not manufacturing accuracy. |
| Raw URDF | Several passive-joint axes do not coincide with the STL pin bores; one right lower connection differs by 40.0 mm in X/Z. Current firmware and Studio's closed-linkage renderer use the measured mesh pivots instead. A raw URDF import is not a trustworthy independent reference for this mechanism without correcting those joints. |
| Body frame | Firmware and renderer share CAD origin `[99, 62.375, 0]` mm. Hip axes lie at `[±167.5, ±62.375, 10.5]` relative to it. Fixed-foot IK uses the full inverse rotation, including fore/aft compensation. |
| Closed linkage | Independent forward reconstruction from production commands tested 6,267 envelope cases, plus 281 recorded poses, without Rapier, body forces or renderer command scaling. |
| Recorded pitch | All 239 post-transition upper/lower command vectors match production replay within 0.0206°. In 39 isolated-pitch rows, actual recorded commands reconstruct maximum foot error 0.763 mm and centre displacement 0.117 mm under the inferred calibration. |
| Working hip convention | Tests cover the owner's FL CH0 / BL CH14 inversion through both CAD IK and forced analytic fallback. Original canonical signs are preserved. |
| Studio rotations | Corrected LIVE rotation composition and attitude readout for simultaneous roll/pitch/yaw. Single-axis pitch is unaffected by this correction. |
| LIVE joint display | Firmware now supplies calibration-aware model commands separately from electrical commands. This prevents trim/inversion from being drawn as actual CAD motion. These remain commanded positions, not encoder feedback. |
| PWM output | The PCA9685 introduces approximately 4.889 μs steps, equivalent to 0.66° in the configured servo model. Existing SIL models whole microseconds but omits this additional tick quantization. |
| Deployed versions | The open packaged app bundles firmware 0.8.4. A passive relay inspection retained a last-known robot identity of 0.8.4 but had no fresh telemetry. Local changes/builds are 0.8.5; their deployment is not assumed. |

## Pitch and the apparent low pivot

At recorded +10° pitch near maximum height, the required fore/aft leg target moves approximately 44–49 mm from neutral. That motion is present in the recorded upper/lower servo commands. A missing horizontal term or a height-only tilt implementation does not explain this capture.

Independent reconstruction also estimates the chassis pose from its four planted feet. It does not supply the requested attitude to the fit. Across 50 isolated pitch cases at five ride heights, the estimated body centre stays within 0.127 mm of its requested position before pulse rounding. The large arc described on hardware is therefore not reproduced by the nominal CAD/command model.

The source origin is 10.5 mm below the hip-axis plane. At 10° pitch that offset moves the centre of the hip plane approximately 1.82 mm fore/aft. It is a real reference-plane distinction, but it is much smaller than a pivot halfway down the legs. The base STL's bounding-box centre and exported centre of mass are different points again; neither automatically defines the intended hip-rectangle/body-plane centre.

With good vertical motion and corrected roll, the outstanding check is the **static fore/aft response of the upper/lower linkage pair**. Candidate causes include actual driver-arm neutral angles, unequal angular response, physical channel/driver mapping, geometry differing from CAD, or deflection under load. These are hypotheses, not established faults. Persistence during a hold means transient lag alone is insufficient. Blindly inverting the upper/lower servos or adding a guessed pivot offset is not justified by these data.

The owner's apparent pivot depth has a plausible scale: at 10° pitch, an
unrealized 25 mm of fore/aft endpoint compensation would translate the body by
about 25 mm despite planted feet. Interpreted as rotation about a fixed point
below a level body centre, that is `25 / sin(10°) = 144 mm` below the centre,
roughly halfway toward the model feet. This is an illustrative sensitivity
calculation, **not** a measurement from the bench video or old IMU log. It
explains why a moderate horizontal linkage-response error can resemble a large
change in pivot location.

## Envelope and pulse limits

Each profile was exercised over 2,089 cases: five heights (220, 230, 250, 270, 280 mm), combinations inside the normalized roll ±20° / pitch ±10° / yaw ±25° envelope, and four level-body translations.

| Assumed profile | Hard-clipped poses | Maximum ideal endpoint error | Maximum endpoint error after predicted PCA ticks |
| --- | ---: | ---: | ---: |
| Defaults | 0 / 2,089 | 0.782 mm | 4.606 mm |
| Offsets inferred from old recording | 161 / 2,089 | 25.414 mm | 27.892 mm |
| Same offsets, FL/BL hips inverted | 161 / 2,089 | 25.414 mm | 27.892 mm |

The last profile assumes all other offsets, routes and limits stayed unchanged. The old recording does not contain an exported active calibration profile. The confirmed fact is the owner's two left hip inversions and successful roll, not that every inferred value equals today's profile.

The worst clipped example is a compound pose at height 220 mm, roll −15°, pitch −2°, yaw +15°. A leg cannot maintain its ideal endpoint when one drive reaches a hard limit. Isolated pitch ±10° at all five tested heights produces **no hard clipping** and stays within 0.764 mm before rounding. Replaying the post-transition recording also produces no hard clipping. This is a separate envelope limitation, not demonstrated evidence for the reported isolated-pitch fault. Hard limits have not been widened.

The pulse prediction assumes the configured 27 MHz PCA clock, 50 Hz, and linear 270° motion over 500–2500 μs. Actual oscillator rate and servo angular response are unmeasured. With the left-inverted candidate profile, isolated pitch reaches 3.711 mm predicted endpoint error after quantization. These predictions include neither backlash nor compliance; they cannot certify the real chassis trajectory.

## Why the simulator could conceal the mismatch

Rapier uses a simplified leg proxy. Its upper/lower joints follow ideal foot targets rather than independently reproducing the two actuator-driven closed linkage. Body-attitude torque, body-position/height forces and foot-position assistance stabilize the scene; the rendered TILT attitude follows the requested pose. LIVE also lacks measured chassis translation. A convincing body-centred animation therefore does not establish a body-centred physical motion.

The audit bypasses those aids. Python closes the CAD linkage using a separate triangle construction, then fits the chassis from planted feet. It validates the commanded geometry while leaving the hardware assumptions explicit. The mixed-axis display bug is separately covered by tests against independently rotated CAD basis vectors.

## Next decisive physical comparison

Use the existing manual STAND body-X control to compare a small level fore/aft shift with the already-working height movement. Start with about 5–10 mm, feet planted, roll/pitch/yaw at zero. Body-X has a ±15 mm full-scale command; it is separate from walking or pitch. This investigation did not activate it remotely.

Record the actual chassis-centre shift and any unintended change in pitch/height, then compare a held small positive and negative pitch at the same ride height. A side view with a fixed ruler and a marker at the centre between front and rear hip axes is sufficient to distinguish the motions. If level body-X already goes the wrong way, changes pitch, or has the wrong scale, the horizontal response can be diagnosed independently of pitch. If it behaves correctly, compare the combined pitch motion and the actual stance/neutral geometry under load.

The planar channel pairs to check against their corresponding CAD driver arms are FL upper/lower **1/2**, FR **4/15**, BL **7/8**, BR **10/11**. Preserve the working hip calibration while making this comparison. A neutral horn-angle observation and one known small command per drive can distinguish a wrong neutral, reversed route and angular-scale error; an IMU-only recording cannot.

For a level initial body rotating about a fixed point directly below its centre, pivot depth equals horizontal centre displacement divided by `sin(actual pitch change)`. The current recording contains no position measurement, so no physical pivot depth has been estimated. The original disconnect/freeze did not occur in that recording and remains a separate investigation.

## Reproduction and validation

Run [the offline audit](../simulation/audit/README.md) with the original engineering JSON. Results are in `tmp/full-pose-audit/summary.json` and `poses.json`. The former records input SHA-256, CAD hashes, assumptions, clipping counts and worst cases. The original JSON and CSV contain matching 281 sample times; initial stand-transition rows before 4.3 seconds are excluded from settled-pose comparisons.

- Studio's configured suite: **285 tests passed**, including combined-axis frame checks and calibration-aware telemetry handling.
- Production-source SIL scenario: passed, including calibrated hips, fallback, travel limits and failsafe stow.
- ESP32 0.8.5 firmware and Studio frontend builds: passed.
- Independent audit: passed CAD identity, pin-fit and nominal endpoint assertions; candidate-profile clipping remains explicitly reported.

The remaining uncertainty is physical: actual linkage pose versus calibrated command. The local display/diagnostic fixes improve that comparison, but they are not presented as a verified repair of the dog's persistent pitch displacement.
