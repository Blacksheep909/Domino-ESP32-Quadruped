## HOTFIX: Domino Quadruped Studio 0.5.3 (firmware 0.8.27)

Fixes three Studio display problems. The firmware is unchanged at 0.8.27, so there is no need to reflash.

### Fixes
- **Gait grid and green line showed up in the simulator.** The Gaits page's moving floor grid and path line stayed on after you left that page. They now appear only in the LIVE Gaits preview.
- **Gait library buttons were oversized.** IMPORT JSON and DELETE SELECTED stretched across the whole card. The library controls now sit on one compact row.
- **The update prompt could disappear.** An update waiting to install now keeps an **UPDATE READY** button in the top bar, even after you press LATER. Click it to bring back **RESTART & UPDATE** (on a Mac, **DOWNLOAD**). Studio also re-checks the update state whenever its window gets focus.
- The Gaits preview caption now says the path line shows the next 6 s, which matches what it draws.

Windows and Apple-silicon Mac builds are both attached.
