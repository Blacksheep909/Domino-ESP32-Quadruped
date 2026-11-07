## Domino Quadruped Studio 0.5.1 (firmware 0.8.27)

Walking starts as soon as you push the stick, and turning no longer scuffs the feet. Flash firmware 0.8.27 after installing.

### Walk starts immediately (firmware 0.8.27)
- Switching into walk mode no longer waits for the sticks to be centred first. If you are already holding forward, the walk ramps in smoothly from a standstill instead of ignoring the stick.
- Tilt mode still waits for centred sticks, so the body never jumps to a held stick angle.

### Turning without foot slip (firmware 0.8.27 and simulator)
- 0.5.0 moved every planted foot the same distance when turning. The front feet sit closer to the body centre than the back feet (about 182 mm against 208 mm), so the front pair tried to turn about 14% faster and the difference was scuffed off sideways.
- Each foot's turning motion is now proportional to its distance from the centre, so all four feet rotate at exactly the same rate and none has to slide. A new test checks this.
- The Gaits page preview uses the same maths, so the turn rate and path shown match the robot.

### Tips for spinning on the spot
The legs are weakest sideways. For on-the-spot turns, use a moderate turn limit, a slightly wider stance and a lower body height in the Gaits page.
