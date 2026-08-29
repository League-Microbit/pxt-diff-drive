# DiffDrive — closed-loop differential drive for micro:bit V2 + Nezha

A MakeCode extension for PXT/microbit that drives the ElecFreaks
**Nezha** brick's two-wheel differential drive **closed loop**. An
encoder-servoed wheel-speed controller runs at a 24 ms cadence, so
straight means straight, distances land on the encoder, turns land on
the angle, and the robot's pose (x, y, heading) is always available
from odometry.

Blocks appear under a **DiffDrive** category. Student-facing units are
**cm, cm/s, degrees and degrees/s**. Positive yaw is counter-clockwise.

**Hardware:** a micro:bit **V2** (the extension is V2-only — its control
loop is native code and does not build for V1) on an ElecFreaks Nezha
brick, with the left wheel motor on **M2** and the right on **M1** — the
standard two-motor chassis. Optionally, an OTOS optical tracking sensor
on the I²C bus for world-frame positioning (the **World** blocks).

## Use as Extension

In [MakeCode for micro:bit](https://makecode.microbit.org/), open a
project, click the gear menu → **Extensions**, and paste this repo's
URL:

```
https://github.com/League-Microbit/pxt-diff-drive
```

Releases are git tags (`v1.0.10`, …). MakeCode pins your project to the
tag you added it at; to pick up a newer release, re-add the extension
or edit the version in your project's `pxt.json`.

## Quick start

Drive a 30 cm square with the blocking `move` block:

```js
input.onButtonPressed(Button.A, function () {
    for (let i = 0; i < 4; i++) {
        diffDrive.move(30, 0)   // 30 cm straight
        diffDrive.move(0, 90)   // pivot 90° counter-clockwise
    }
})
```

Do something while it moves — the loop body gets the live pose:

```js
diffDrive.whileMoving(50, 0, function (x, y, heading) {
    led.plotBarGraph(diffDrive.moveProgress() * 100, 100)
    if (input.buttonIsPressed(Button.B)) diffDrive.stopMove()
})
```

Drive continuously (see "The tick contract" below for why the loop):

```js
diffDrive.driveTwist(15, 45)   // 15 cm/s forward, turning 45°/s CCW
while (diffDrive.driveTick()) {
    if (input.buttonIsPressed(Button.A)) diffDrive.stop()
}
```

[`test.ts`](test.ts) in this repository is a complete sample program.

## Coordinates

Pose is measured from wherever the robot was when the program started
(or the last `reset pose`): **x forward, y left, heading in degrees,
counter-clockwise positive**. `go to x y` takes a point in the robot's
*current* frame — x ahead of it, y to its left.

The **World** blocks use a fixed world frame instead, supplied by the
OTOS sensor and seeded with `set world pose`.

## Blocks

### Move — position moves

| block | what it does |
|---|---|
| `move %distance cm turning %yaw degrees` | Drive a distance while turning an angle, then stop. One alone is a straight or a pivot; both at once is an arc. Waits for the move to finish. |
| `start move …` | Same, but returns immediately. Something must still tick the control loop (`drive tick`) or the move does not progress — prefer `move` / `while moving` unless you are writing your own tick loop. |
| `while moving …` | Runs the body once per control cycle (about every 24 ms) with the live `x`, `y`, `heading`, until the move completes or `stop move` is called. |

### GoTo — drive to a point

| block | what it does |
|---|---|
| `go to x %x cm y %y cm` | Curved path to a point in the robot's current frame, then stop. Waits for the move to finish. |
| `start go to …` / `while going to …` | Async and loop forms, same contract as the Move group. |
| `go to world x %x cm y %y cm` | Drive to a point in **world** coordinates, reading the OTOS sensor to find out where the robot is before the leg. One pass: it drives at the point and stops; the next call re-measures. Pivots first if the target is more than 12° off the bow. Needs `start world tracking`. |

### Drive — continuous velocity

| block | what it does |
|---|---|
| `drive %speed cm/s turning %yawRate deg/s` | Set a body speed and yaw rate. Continuous: the robot only moves while something ticks the loop — follow it with `while drive tick`. |
| `start drive …` | Same, and ticks the loop in the background for you until the drive stops. Stop the drive before starting a position move. |
| `while driving …` | Loop form with the live pose in the body. A continuous drive has no finish line of its own, so give the body a way out (`stop`). |

### Wheels

| block | what it does |
|---|---|
| `set wheel speeds left %left right %right cm/s` | Command each wheel's speed directly (−50 … 50 cm/s). Continuous, same tick rule as `drive`. |

### Moving?

| block | what it does |
|---|---|
| `drive tick` | Advance one control cycle; true while the robot should keep driving. Self-paces to ~24 ms — do not add a `pause` in the loop. |
| `moving?` | Is a position move running? |
| `move progress` | Fraction of the current move completed, 0–1. |
| `is stalled` | The stall latch has tripped: the robot demanded motion but the wheels did not turn. Every Drive/Move block is ignored until `clear stall latch`. |

### Stop

| block | what it does |
|---|---|
| `stop` | Normal stop — ends any move or continuous drive. |
| `stop move` | The same full stop, offered next to the Move blocks. |
| `emergency stop` | Latch the drive off until `clear emergency stop`. |
| `clear emergency stop` | Release the emergency-stop latch. |
| `clear stall latch` | Release the stall latch (independent of the emergency-stop latch). |

### Pose (in the Pose subcategory)

`pose x (cm)`, `pose y (cm)`, `heading (deg)`, `reset pose` — the
odometry pose since program start or the last reset.

### World (in the Pose subcategory) — OTOS optical tracking

| block | what it does |
|---|---|
| `start world tracking` | Start the OTOS sensor; true if it answered. Call once at program start with the robot still. |
| `world tracking ready?` | Is the sensor present and answering? |
| `calibrate world sensor` | Recalibrate the gyro bias. Robot parked and still for about a second. |
| `set world sensor offset x y yaw` | Where the sensor sits relative to the robot's centre of rotation, and its mounting rotation. Set once at startup. |
| `set world pose to x y heading` | Declare where the robot is now, in world coordinates. Sets both the sensor and the odometry so they agree. |
| `read world position` | Take a fresh fix; false if the sensor did not answer (the last good values are kept). |
| `world x (cm)`, `world y (cm)`, `world heading (deg)` | The most recent fix. |

World reads are I²C transactions: call them from the same code that
ticks the drive, never from a second forever loop running alongside a
move.

### Setup (in the Setup subcategory)

| block | preset in the block |
|---|---|
| `set track width %width cm` | 11.5 — distance between the wheels |
| `set wheel calibration %calib mm/deg` | 0.7837 — wheel travel per shaft degree |
| `set default speed %speed cm/s` | 15 — used by move / go to |
| `set default turn rate %yawRate deg/s` | 90 — used by move / go to |
| `set arrival tolerance %tol cm` | 1 — used by `go to world` |
| `setup radio channel %channel group %group` | 4 / 10 — turn on the wire protocol over radio (see Remote). Off until called. |
| `set config %field to %value` | Advanced: set a controller value directly (PID gains, duty limits, stall thresholds, …). |

Defaults are tuned for the standard Nezha kit.

### Remote and Debug (in the Extra subcategory)

The extension speaks a line-oriented wire protocol over USB serial —
and over the radio once `setup radio` has been called — so a computer
can drive and observe the robot. Two blocks let a program answer to it:

- `on run %name` — run code when `RUN:<name>` or `RUN:<name>:<arg>`
  arrives; the first argument comes in as a number. Bind test routines
  to names so a bench host can trigger them.
- `on run command` — run code for *any* `RUN:` command.

And two send things back:

- `send string %text` — a `DBG:`-tagged line, shown in the console.
- `send value %name = %value` — `name:value`, which the MakeCode
  console plots as a graph.

`setup radio` takes the radio over: MakeCode's own `radio` blocks stop
working in the same program once it has been called, which is why it
is off by default.

## The tick contract

The robot only moves while something keeps ticking its control loop.
`move`, `go to`, `while moving`, `while going to` and `start drive`
tick internally — they keep driving on their own until they are done.

`drive` and `set wheel speeds` are different: they are continuous-mode
commands, and starting one is not enough to keep the robot moving.
Follow it with a `while (diffDrive.driveTick())` loop:

```js
diffDrive.setWheelSpeeds(15, 15)   // or driveTwist(...)
while (diffDrive.driveTick()) {
    // runs once per ~24 ms control cycle -- read a sensor, check a
    // button, whatever your program needs. Don't add your own
    // pause() here; driveTick() already paces itself.
    if (input.buttonIsPressed(Button.A)) diffDrive.stop()
}
```

If nothing ticks the loop — it exits, the program pauses, a button
handler returns — a starvation watchdog stops the robot within about
150 ms. It is a safety net, not an emergency stop: nothing latches,
and a fresh `move` or tick loop resumes driving right away.

## Stalls

If the controller demands motion for a while and the wheels do not
turn — the robot is against a wall, or lifted with a wheel jammed —
the stall latch trips and every Drive/Move block is silently ignored
until `clear stall latch`. Check `is stalled` in a program that might
run into things.

## Provenance

This repository is **generated**. Every push to
[League-Robotics/pxt-nezha-diffdrive](https://github.com/League-Robotics/pxt-nezha-diffdrive)
re-publishes the extension files here, and the release tags follow that
repo's `pxt.json` version. Do not edit files here — report issues and
send changes to the source repository.

The wheel controller is vendored from the
[radio-robot](https://github.com/League-Robotics/radio-robot) firmware
and is maintained there.

## Supported targets

* for PXT/microbit

(The metadata above is needed for package cataloging.)

## License

MIT — see [LICENSE](LICENSE).
