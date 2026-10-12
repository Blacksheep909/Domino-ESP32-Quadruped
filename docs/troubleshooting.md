# Troubleshooting Notes

> Work in progress: this page covers common bring-up checks. It is not a complete fault tree for every hardware configuration.

## PlatformIO Does Not Build

Check:

- PlatformIO extension is installed.
- The repository root is open, not only the `src` folder.
- `platformio.ini` is visible in the VS Code Explorer.
- The active environment is `esp32dev`.

Command:

```powershell
pio run
```

## Upload Fails

Check:

- USB cable supports data.
- Correct COM port is selected.
- ESP32 is not held in reset.
- No other serial monitor is using the port.

Try unplugging and reconnecting the ESP32 before uploading again.

## No Serial Output

Check:

- Monitor speed is 460800 for current application logs. ESP32 ROM boot text
  uses its fixed startup baud and may look garbled before Domino starts.
- ESP32 is running the uploaded firmware.
- USB cable supports data.

Command:

```powershell
pio device monitor
```

## CRSF Link Not Alive

For the SpeedyBee Nano 2.4 GHz receiver, identify the receiver LED state first:
two red blinks followed by a pause mean bind mode; a slow single blink means
unlinked; fast blinking means Wi-Fi; solid red means an RF connection. These
patterns come from the receiver, before Domino can read a CRSF frame. Three
quick power interruptions enter bind mode. If this happened during repeated
robot resets, remove **all** receiver power (including USB backfeed), wait for
the LED to go dark, then power it once normally. ExpressLRS retains the old
binding when bind mode is cancelled by a complete power cycle.

If it still double-blinks, open the ExpressLRS Lua script on the Boxer and
select **Bind** while the receiver is double-blinking. A solid receiver LED
confirms RF binding. If this does not work, check that the Boxer is using its
ELRS model/module and that transmitter and receiver have compatible ELRS
major firmware versions. The receiver's Wi-Fi mode can be entered with its
button (hold for three seconds) or, when unlinked, after the configured auto
Wi-Fi interval (normally about 60 seconds). Merely entering Wi-Fi mode does
not change Domino's CRSF code or servo calibration.

References: [SpeedyBee Nano receiver manual](https://docs.speedybee.cn/en/fpv/receiver/nano-2.4ghz-rx/speedybee-nano-2.4ghz-elrs-receiver-manual.html),
[ExpressLRS binding guide](https://www.expresslrs.org/quick-start/binding/).

Check:

- Receiver TX is wired to ESP32 RX2 / GPIO 16.
- Receiver and ESP32 share ground.
- Receiver is powered.
- Radio is bound to the receiver.
- Receiver is configured for CRSF output.
- Firmware baud rate is 420000.

Use [ESP32_CRSF_Reader](https://github.com/Blacksheep909/ESP32_CRSF_Reader) to debug the receiver without servo code.

## Servos Do Not Move

Check:

- External servo power is on.
- Servo power ground is common with ESP32 ground.
- PCA9685 is powered.
- PCA9685 I2C wiring is correct.
- Servo plug orientation is correct.
- The robot is receiving a valid stand command.

## Servos Twitch Or Brown Out

Likely causes:

- Regulator cannot supply enough current.
- Wiring is too thin or too long.
- Grounding is poor.
- Battery voltage is sagging.
- Servos are fighting mechanical stops.

Test with one servo or one leg before powering all twelve.

## One Leg Moves Backwards

Check:

- Servo channel assignment.
- Servo horn orientation.
- `hipDir`, `upperDir`, and `lowerDir` in the leg configuration.
- Left/right mechanical mirroring.

Do not compensate for wiring or channel mistakes with random trim values.

## Tilt Or Balance Mode Behaves Incorrectly

Check:

- Robot is stable in stand mode first.
- IMU is detected.
- IMU mounting orientation matches the assumptions in `src/main.cpp`.
- RC switch channels match the transmitter setup.
- Sticks are centered when testing neutral pose.

Balance mode is experimental and should be treated as a tuning area, not a finished stabilizer.

### Body rolls around a point near the floor

If the chassis swings sideways in an arc instead of rotating around the center
of the hip rectangle, inspect LIVE expected foot targets during an isolated
roll. A correct fixed-foot command changes every leg's lateral target and moves
all four hip servos. A command that changes only left/right leg height creates
the floor-pivot behavior even though a simulator can hide it with body-pose and
foot-hold assistance.

After flashing a fixed-foot build, test on a support harness at a small roll
angle first. Confirm the four hip channels move in the same physical direction,
no stored calibration limit clips them, and the feet do not scrub before
increasing the angle.

If sit/stand and ride height remain correct but tilt alone fails, inspect the
active LIVE calibration profile before changing the IK. Height changes mostly
exercise the two planar linkage drives, whereas fixed-foot roll also depends on
all four hip drives. A reversed hip direction, stale channel route, or hip
travel limit can therefore leave height looking excellent while destroying the
roll geometry. The browser Rapier scene is explicitly labelled **ASSISTED**;
its body torque and foot-position holds are useful for visualization but are
not evidence that the physical calibration is correct.

On the owner's build, inverting FL CH0 and BL CH14 in Calibration corrected roll.
Keep the outward-positive canonical hip basis unchanged when using that profile;
applying an additional blanket firmware hip sign change would conflict with it.

For forward/back pitch, inspect the fore/aft leg target as well as front/rear
height. At 10 degrees near maximum ride height, each foot needs roughly 44–49 mm
of fore/aft compensation relative to its hip. Clean vertical ride-height motion
does not prove this coupled movement is correct. Observe both the chassis centre
and foot contact at a small, isolated pitch: the IMU alone cannot locate a pivot.
If the displacement persists while holding pitch, check the static fore/aft
response rather than treating it only as servo lag. A level manual STAND body-X
translation isolates this response from the front/rear height difference.
In LIVE, firmware 0.8.5 supplies calibration-aware model commands separately from
electrical servo commands so that saved horn offsets are not drawn as CAD motion.
