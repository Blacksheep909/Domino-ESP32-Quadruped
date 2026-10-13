# Domino Quadruped Studio 0.4.2 (hotfix)

Studio 0.4.2 ships with firmware **0.8.21**. It fixes **CONFIRM & SEND doing nothing** when saving calibration to the robot, plus other foundation bugs found in an app-wide audit.

## Hotfix: CONFIRM & SEND did nothing

The save dialog is modal, and when a safety gate refused the send (bench permission lost, the "robot is supported" checkbox, telemetry older than one second, or another request pending), the reason was written to a status line **behind the dialog**. The button just did nothing, however many times it was clicked.

- The reason is now shown **inside the dialog**, and the button stays disabled while a gate is closed.
- On firmware **0.8.21 and later**, saving no longer needs bench mode at all: it only writes flash, with every output off. A broken bench handshake can no longer stop a calibration being saved. Older firmware still requires bench mode and says so.

## Other fixes from the audit

- **Firmware gait scenario passes again.** The software-in-the-loop walking scenario (stand, tilt, careful crawl, trot, failsafe stow) had been failing: it switched from careful to trot without the landing handoff the firmware deliberately performs. Its timeline now matches the firmware. It is the only automated coverage of walking, and it now runs in CI.
- **Studio polled the firmware simulator about 43 times a second** with no backoff when the simulator was not running, flooding the console and network. It now backs off to once every two seconds and pauses while the window is hidden.
- **Three test files never ran** (a hand-maintained list skipped them), and one was stale. `npm test` now discovers every test file (48 files, 366 tests).
- **CI**: a new workflow runs the firmware build, the SIL scenario, the calibration storage test, the Studio tests and the Studio build on every push.
- **Security**: updated `electron-updater` (6.6.2 to 6.8.10) and pinned `js-yaml` to fix two high-severity advisories in the updater's dependencies.
- **Docs**: removed a design audit whose screenshots were links to a private computer, and fixed a link to a file that is not in the repository.

## Upgrading

Install over any previous version. To get bench-less saving, flash firmware 0.8.21 (hold BOOT while uploading); your saved calibration is kept.