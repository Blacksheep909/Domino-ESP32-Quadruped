# Pitch and yaw audit — 28 September 2026

## Observations and scope

The owner reports nearly matching simulation and hardware roll, a roughly 95 mm body translation during held pitch, and joint bending with little chassis turn during small yaw commands on planted feet. At maximum ride height, the upside-down hardware and simulator have similar neutral foot endpoints. Top-view photos at **yaw left / CH4 near 1000 µs** show a different foot arrangement relative to the body. The neutral match makes a large fixed offset unlikely; the commanded-to-physical *change* is the key discrepancy.

This audit traced the current source from CRSF channels through body targets, CAD four-bar IK, calibration and PWM, and the visual and physics simulator. It ran the production-source offline pose probe and tests. It did not read the robot's current NVS calibration, measure shaft angles, flash firmware, or move hardware.

## Command path

- `src/main.cpp`: TILT reads channel indices 0/1/3 as roll/pitch/yaw, normalizes 1000–2000 µs around 1500 µs, limits pitch to ±10° and yaw to ±25°, and sets body X/Y translation to zero. The simulator's `S` key sets channel index 1 to about 1000 µs, so the owner's LIVE reading confirms the compared poses have the same command sign.
- `bodyKinematicsSimple()` applies `Rz(yaw) * Ry(pitch) * Rx(roll)` to each CAD hip and transforms nominal fixed world feet back into leg coordinates with the inverse rotation. Hip axes are at X ±167.5 mm, Y ±62.375 mm and Z +10.5 mm. The neutral CAD foot is about X +2.54 mm and Y ±38.1 mm from its hip. The legacy leg command maps that CAD X to −15.75 mm; this is a coordinate conversion, not an additional movement command.
- `src/leg_controller.cpp`: X/Z targets go to the two-drive CAD four-bar inverse solver, Y/Z to the hip rotation. `applyServoCalibration()` applies the saved per-joint direction, offset and min/max travel before PWM output. There are no measured joint angles: telemetry reports commands and pulses.

## Direct CAD dimension check

`simulation/audit/verify_cad_ik_geometry.py` fits each cylindrical pin centre and both 12 mm foot-ball centres from the supplied STL vertices. It then parses the active C++ geometry and compares local pin coordinates, rigid lengths, mating pins and the four hip axes. The generated and build CAD mesh sets are byte-identical. This check uses the actual mesh vertices rather than the simulator's hand-entered point list.

| Dimension | CAD mesh / URDF base joints | Firmware |
| --- | ---: | ---: |
| Front to rear hip axes | 335.000 mm | 335.000 mm |
| Left to right hip axes | 124.750 mm | 124.750 mm |
| Hip axis above body centre plane | 10.500 mm | 10.500 mm |
| Upper drive crank | 40.000 mm | 40.000 mm |
| Long lower arm | 159.860–159.861 mm | 159.860–159.861 mm |
| Upper closure link | 31.900 mm | 31.900 mm |
| Coupler arm / lower closure link | 40.000 mm | 40.000 mm |
| Diagonal lower link | 159.860–159.861 mm | 159.860–159.861 mm |

The largest point or link-length discrepancy is **0.025 mm**, at the rounded left foot-ball centre. All drive and closure pin discrepancies are below **0.002 mm**, including checks against both mating parts. The right CAD foot-ball centre is **X +2.5399, Z −276.0089 mm** from its hip; firmware has **+2.545, −275.993 mm**. The as-exported left CAD centre is **X −5.4326, Z −267.4843 mm**; firmware has **−5.427, −267.460 mm** and uses the source assembly's **4.18°** lower-link offset to align its neutral pose. The body rotation uses the exact average of the solver's aligned neutral foot positions: **X +2.541356, Y ±38.1, Z −275.984374 mm** from each hip. No material IK dimension error was found, so altering those lengths is not a credible way to remove the observed travel.

The firmware's legacy neutral command **X −15.75 mm** corresponds to CAD **X +2.541356 mm** through a constant **−18.291356 mm** frame conversion. The corresponding Y conversion is **−0.1 mm** per side, and Z command 280 mm corresponds to a CAD hip-to-ball-centre height of about **276 mm**. These conversions cancel at neutral and preserve displacement signs. `solveCadEndpoint()` runs before `applyServoCalibration()` in `write270()`: a saved trim shifts the output angle after IK, while a saved direction or route changes its response to future commands.

At 10° pitch, 276 mm of leg height gives about **47.9 mm** of required fore/aft compensation. A reversed response about the matching neutral would displace the support relationship by about **2 × 47.9 = 95.8 mm**. Even a 5 mm error in the vertical CAD measurement changes the pitch X compensation by only **0.87 mm**. The neutral CAD drive Jacobian has condition number about **1.5**, so this pose is not near a linkage singularity where a tiny dimensional difference would flip the fore/aft direction. The two front legs receive the same pitch X target, so an identical drive-direction mismatch on both naturally makes them move together in the wrong direction. A single unpaired motor inversion would also disturb vertical motion and likely spoil the good roll; the paired upper/lower response hypothesis below preserves most vertical motion.

The generated URDF's closure-joint origins disagree with the cylindrical centres in their parent STL parts by approximately **10–40 mm** at the authored pose. Studio and the firmware use the mutually agreeing mating-hole centres instead. Those exported closure origins should not be copied into IK as replacement dimensions.

### Why good roll does not validate pitch or yaw

These are current production-source targets at 280 mm ride height. X is relative to each hip in the legacy leg command frame; neutral X is −15.75 mm on every leg.

| Isolated pose | FL X | FR X | BL X | BR X | Required X change |
| --- | ---: | ---: | ---: | ---: | --- |
| Roll +20° | −15.75 | −15.75 | −15.75 | −15.75 | **Zero** on every leg |
| Pitch −10° | −64.43 | −64.43 | −59.34 | −59.34 | About 44–49 mm backward on all legs |
| Yaw +6.25° | −5.82 | −27.70 | −3.83 | −25.71 | About 10–12 mm in opposite patterns |
| Yaw +25° | +10.78 | −74.14 | +42.17 | −42.76 | Up to about 58 mm |

Pure roll mathematically leaves X unchanged. Both failing modes require X motion from the planar linkage. Roll therefore supports the hip and lateral geometry, but does not exercise the part of the mechanism most implicated by the pitch/yaw symptoms. At quarter-stick yaw the largest lateral change is about 19 mm and no default hard limit is reached, so a severe response at that small command is not explained by the requested pose being inherently large.

Roll also does **not** validate front/rear channel routing. At +20° roll, the canonical hip changes are about **FL +18.6° and BL +18.6°**, with **FR −20.0° and BR −20.0°**. At −25° yaw they are **FL −12.6°, BL +16.3°, FR +16.7°, BR −12.2°**. A front/rear hip route swap on one side can therefore preserve the roll pattern while corrupting yaw. Pitch similarly commands different front/rear upper/lower motion, while roll's front/rear commands are alike. The active logical-to-physical route and actual plugged-in motor at each PCA channel have not been verified.

The owner clarified that both top views show the robot **upside down with its front to the right**. The front foot at the bottom right is behind its hip in Studio but ahead on the dog; the rear foot at the top left shows the opposite discrepancy. The Studio +X arrow points right and +Y points down, so these are the code's **FL and BR** legs. The other diagonal pair looks closer, but the photos are not calibrated orthographic measurements.

## Side-photo comparison at −10° pitch

The two side crops have the chassis extending in opposite image directions from the pictured hip. This **might** mean they show opposite ends, but the dog or camera could have been turned between shots; the real photographed leg has not yet been identified as front or rear. The photographed lower link appears roughly 28° from image vertical (perspective and overlapping legs limit precision). Until the leg identity is confirmed, comparing it with the nearly vertical simulated **front** link is inconclusive.

The current CAD visual linkage, driven with production-source neutral and −10° commands at 280 mm, gives approximately:

| Foot-link angle from body vertical | Neutral | Pitch −10° |
| --- | ---: | ---: |
| Front, normal commanded motion | +22.5° | **−2.2°** |
| Rear, normal commanded motion | +22.5° | **+16.5°** |

At −10°, the nominal front upper drive changes by about **+24.8° in canonical CAD joint coordinates**; the front lower changes by about −7.6°. With the *compiled default* electrical directions, this is approximately FL CH1 **−24.7°**, FR CH4 **+24.7°**, FL CH2 **+7.6°**, and FR CH15 **−7.6°** from their own neutral commands. These values remain useful when comparing the actual **front** horns, but the existing side photos do not establish that their direction is wrong without confirming the pictured end.

Reversing FL upper alone makes the simulated **front** link about 47° and reverses its yaw X sign, but also causes about **38 mm FL foot error during full roll**. Reversing both front uppers and both rear lowers makes full roll fit about **7.5° instead of 20°**. These direction-flip hypotheses conflict with the owner's nearly 1:1 roll and are not a justified calibration change.

## A fore-aft response inversion fits all three symptoms

I modeled the physical response as `actual foot X change = gain × commanded foot X change`, retaining the commanded Y/Z response and neutral location. This is a fault hypothesis, **not a measurement of the robot**.

| X response gain | Predicted roll for +20° | Predicted chassis X travel at −10° pitch | Predicted yaw at −25° | FL / BR foot X relative to hips at yaw |
| --- | ---: | ---: | ---: | ---: |
| +1 (nominal CAD) | +20.0° | ~0 mm | −25.0° | −55.6 / +60.2 mm |
| 0 (X motion stalled) | +20.0° | ~45 mm | −17.7° fit | +2.5 / +2.5 mm |
| **−1 (X response reversed)** | **+20.0°** | **~90 mm** | **−10.8° fit, high foot conflict** | **+60.7 / −55.1 mm** |

The reversed-X case agrees with the approximate **95 mm** pitch travel, preserves roll, and puts FL ahead and BR behind at yaw left as in the top views. It also predicts about 53 mm error on FR/BL and about 116 mm on FL/BR; this is consistent with the other diagonal looking closer without proving exact agreement. This result ranks a reversed fore-aft response above a fixed CAD origin offset or stopped servo. It does **not** identify whether the reversal is in flashed firmware, saved channel routing/directions, or the real linkage's shaft-to-foot motion.

Several concrete servo swaps were checked. Front/rear planar pairs swapped by side preserve +20° roll and create ~90 mm pitch travel, but predict FL still behind and BR still ahead at yaw left. A FL/BR diagonal pair swap flips those yaw feet but predicts only ~7.7° roll and essentially no pitch. No single tested channel swap reproduces all observations.

One **systematic two-drive mapping** does closely reproduce all three: on each leg, physical upper response behaves like the *negative commanded lower delta*, and physical lower response like the *negative commanded upper delta*, both about their neutral angles. The CAD simulation of that fault predicts **19.1° roll for +20°**, **92.2 mm chassis X travel at −10° pitch while still pitching −9.8°**, and **FL +53.0 / BR −55.3 mm** at yaw left, with a poorly fitting body yaw near −11°. It also predicts a rear foot-link angle near 39.5° at −10° pitch. Across the 2,089 default-profile motion-envelope cases, the inverse command for this candidate remains inside all eight affected electrical hard limits. This is a high-priority *testable fault hypothesis*, not proof of the physical wiring or four-bar convention; the saved profile and actual shafts are still unknown.

The measured CAD neutral Jacobian is the same on all four translated legs: upper/lower canonical drive changes produce X changes of **−2.47/−1.98 mm per degree** and Z changes of **−1.02/+1.97 mm per degree**. Applying a cross-reversal to the drives gives an approximate endpoint transfer matrix `[[−0.93, +0.32], [+0.41, +0.93]]` in X/Z. This explains mechanically why that mapping nearly reverses X while retaining Z and therefore preserves much of the good roll/height response.

The test correction changes the **drive deltas around each servo's existing neutral pulse**. A raw upper/lower PCA channel swap would shift neutral inputs by roughly 50–58° per leg and could move the dog abruptly; the saved profile's ±30° trim range cannot safely represent that swap. The mathematically inverse command for a cross-reversed physical response is upper command delta `−desired lower delta` and lower command delta `−desired upper delta`, leaving all neutral outputs unchanged. It is an involution: applying the same cross-reversal in the physical response recovers the desired CAD joint deltas. The physical firmware test build enables this mapping with `DOMINO_PLANAR_DRIVE_CROSS_REVERSED=1`; the SIL simulator keeps its ideal CAD mapping. LIVE electrical angles/pulses remain the actual servo commands, while LIVE model angles apply the assumed physical response after calibration and safety clamps.

| Leg | Upper logical channel | Lower logical channel | Hip channel left unchanged |
| --- | ---: | ---: | ---: |
| Front left | CH1 | CH2 | CH0 |
| Front right | CH4 | CH15 | CH3 |
| Back left | CH7 | CH8 | CH14 |
| Back right | CH10 | CH11 | CH9 |

The paired mapping applies to all four rows: `new upper delta = −old lower delta` and `new lower delta = −old upper delta`. These are logical channels; a saved physical-channel route, if present on the robot, applies afterward. A 160–280 mm isolated height sweep of the compiled test firmware retained identical neutral pulses at 280 mm, produced no electrical hard clips, and under the assumed cross-reversed physical response differed from the original predicted foot Z by at most **0.113 mm**. This is a conditional software prediction. If the real shaft-to-foot response is different, height motion can change; the robot has not yet physically confirmed the paired reversal.

## Software checks and limitations

- The independent offline CAD closure and planted-foot fit reproduces the present firmware commands at maximum ride height. Across isolated pitch cases at 220–280 mm, the default-profile maximum predicted endpoint error is **0.763 mm** and centre error **0.126 mm**. Across isolated yaw cases the maximum predicted endpoint error is **0.740 mm** and centre error **0.151 mm**. These are predictions conditional on CAD geometry, calibration and servo response, not physical measurements.
- In the 280 mm isolated yaw sweep, neither the default nor the assumed left-hip-inverted candidate profile reaches a hard output limit. The candidate profile does hard-clip three isolated full-yaw cases at 220–230 mm. Its offsets and limits are inferred from an older recording, not exported from today's robot.
- The saved per-joint calibration travel clamp and the inner four-bar/hip motion clamps do **not** increment `servoSafetyClipCount`; that telemetry counter only covers the later electrical hard limits. A zero counter cannot rule out a saved travel limit truncating the front upper command.
- The Studio CAD visual uses simulated firmware servo commands. Its Rapier leg proxy independently solves a simple two-link leg from requested foot targets, and the simulation applies attitude, body-position and foot-hold forces in TILT. Physics tests passing therefore do not prove the real closed linkage or planted-foot yaw response.
- The known 10.5 mm body-to-hip reference offset contributes only about 1.8 mm of horizontal hip-plane movement at 10° pitch. It cannot account for about 95 mm of body travel or the yaw foot arrangement.
- The 27 September calibration JSON supplied on 28 September has `savedAt: null`, identity physical channels, zero offsets and the compiled default directions, including FL CH0 and BL CH14 `+1`. The owner previously saved those two hips inverted and then verified roll. Studio's old Export JSON button serialized its **browser draft**, not an active ESP32 readback. Consequently the attached JSON cannot be treated as the robot's active calibration; it must not be sent back to the robot as a purported fix.

## Conclusion and next discriminating check

The current body rotation and fixed-foot equations include the required pitch and yaw X compensation; no missing-X, sign, unit or material CAD-dimension error was found in that source. A **reversed physical fore-aft response about neutral**, plausibly caused by a cross-reversed upper/lower drive convention, is the most coherent available model of the reported roll, pitch and yaw. The side-link photo comparison has unconfirmed leg identity and must not be used alone to choose an upper-servo direction. A neutral-preserving **test firmware** now applies the inverse drive-delta mapping. Its effectiveness still needs physical verification because the active robot profile, physical channel plugs and actual shaft directions remain unobserved.

The simplest physical discriminator is a small **+10 mm Body X** command in Expert STAND with the dog upside down, neutral tilt and front pointing right. With the present coordinate convention all four feet should move **left, toward the rear**, relative to the chassis. Right/toward front on all four would directly confirm the reversed-X response. Mixed directions identify specific legs. This test is much safer and clearer than a planted full-yaw trial. Afterward, compare a matching front-leg side view and a rear-leg side view at neutral and −10° pitch.

The workspace now includes a read-only **Export Robot Profile** path: firmware handles `calibration/request-profile` while disarmed without bench mode or servo output changes; the companion forwards the actual profile; Studio labels browser draft exports separately and downloads the verified robot response as `robot-active`. This code must be in both the updated app and flashed firmware before that button can read the robot. It does not alter existing calibration or pitch commands. Compare its directions, limits and routes against physical motors FL **0/1/2**, FR **3/4/15**, BL **14/7/8**, BR **9/10/11** (hip/upper/lower). Preserve the working CH0/CH14 inversions while diagnosing X.

Two separate local installers were built. Studio **0.2.22** at `simulation/standalone/out/calibration-readback/Domino-Quadruped-Studio-0.2.22-x64.exe` bundles the **readback-only 0.8.11** firmware. Studio **0.2.23** at `simulation/standalone/out/pitch-correction-candidate/Domino-Quadruped-Studio-0.2.23-x64.exe` bundles **0.8.12 test firmware** with the neutral-preserving drive-delta correction and the readback feature. The 0.2.23 package contains the exact edited firmware source and enabled build flag. Neither installer was run and the robot was not flashed during this audit.

## Opt-in rendered CAD response preview

Studio's local SIL build omits `DOMINO_PLANAR_DRIVE_CROSS_REVERSED`, so it supplies the original uncompensated servo commands. The simulator now has opt-in URL modes that transform each rendered leg's canonical upper/lower **deltas about neutral** to `physical upper = -commanded lower` and `physical lower = -commanded upper`. Only the CAD linkage renderer uses this transform. The simulator's body/ground physics continues to use ideal foot targets, so these modes compare leg shapes and foot locations relative to the chassis; they do not demonstrate actual planted-body translation or yaw.

| Input at 280 mm ride height | Ideal URL mode | Suspected response mode | Rendered foot X difference, FR / FL / BL / BR |
| --- | --- | --- | --- |
| CH2 1000, pitch −10° | `?demo=pitch-back` | `?demo=pitch-back-fault` | **+88 / +89 / +97 / +97 mm** |
| CH4 1000, yaw left | `?demo=yaw-left` | `?demo=yaw-left-fault` | **−47 / +102 / +47 / −101 mm** |
| CH1 2000, roll +20° | `?demo=roll` | `?demo=roll-fault` | **+7 / −6 / −6 / +7 mm** |

These are differences in the rendered world X direction after each mode reached its full command; the ideal body pose was almost identical across each pair. In the yaw comparison, the large forward FL and backward BR errors are the same diagonal identified in the upside-down photos. FR and BL still differ by about 47 mm, so this is a qualitative match, not a perfect reconstruction of the photographs. In the pitch comparison, planted feet with a roughly 90 mm forward error would require roughly 90 mm of opposite chassis translation, consistent with the separately measured approximate 95 mm. During roll the rendered vertical foot differences were at most about 3 mm. The correction in the test firmware remains a hypothesis until the physical shaft-to-foot response is measured.

## Verification

- `simulation/audit/run_pose_audit.py`: PASS, 6,267 total pose/profile cases. Results in `tmp/pitch-yaw-audit-2026-09-28/`.
- `simulation/audit/verify_cad_ik_geometry.py`: PASS; measured maximum CAD-to-firmware point, mating or rigid-length discrepancy **0.025 mm**.
- `simulation/sil/test.ps1`: PASS.
- `node simulation/standalone/physics.test.mjs`: 22/22 PASS. The `node --test` wrapper could not spawn in this sandbox (`EPERM`); direct execution ran the same tests.
- Active-profile readback tests: `live-calibration-state.test.mjs` 15/15 PASS and `live-companion-core.test.mjs` 15/15 PASS.
- PlatformIO ESP32 firmware build: PASS; Vite production UI build: PASS. The latter needed the Windows process-spawn sandbox exception.
- Local Electron NSIS installer build: PASS. Bundled firmware endpoint source SHA-256 matches the workspace source.
- 0.8.12 test firmware: PlatformIO build PASS. Its compiled neutral output is identical on all 12 driven channels. Across 2,089 default-profile pose cases the compiled mapping differs by at most 0.12° from the calculated cross-reversed deltas, the estimated physical model angles match the original intended angles within 0.12°, and there are zero electrical hard clips. The 160/180/200 mm level poses also report zero hard clips. Studio 0.2.23 local installer build PASS.
