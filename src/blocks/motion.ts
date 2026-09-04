/**
 * DiffDrive — closed-loop differential drive for the Nezha brick.
 *
 * Student-facing units: cm, cm/s, degrees, degrees/s. Positive yaw is
 * counter-clockwise (right wheel forward). Pose is (x, y, heading) in
 * robot start coordinates: x forward, y left.
 *
 * There is no dedicated motor fiber: the robot only moves while
 * something keeps ticking the control loop (driveTick(), or an
 * internally-ticking block like move()/goTo()) -- see driveTick()'s
 * own doc comment. The functions exported below are the real
 * block-API bodies, calling into the hardware shim layer (the .cpp
 * files); the browser-simulator fallbacks live in sim.ts.
 */

enum ConfigField {
    //% block="max duty %"
    MaxDuty = 0,
    //% block="full-duty wheel speed"
    FullDutyVelocity = 1,
    //% block="PID kp"
    Kp = 2,
    //% block="PID ki"
    Ki = 3,
    //% block="PID integral limit"
    IMax = 4,
    //% block="accel feedforward"
    Kaff = 5,
    //% block="PID output limit"
    PidMax = 6,
    //% block="twist hold gain"
    TwistHoldGain = 7,
    //% block="speed floor mm/s"
    VFloor = 8,
    //% block="position error limit"
    PosErrMax = 9,
    //% block="stall speed"
    StallSpeed = 10,
    //% block="stall demand"
    StallDemand = 11,
    //% block="stall window ms"
    StallWindow = 12,
    //% block="lambda enabled"
    LambdaEnabled = 13,
    //% block="crawl pulse"
    CrawlPulse = 14,
    //% block="default cruise speed"
    DefaultCruise = 15,
    //% block="rotational slip"
    RotationalSlip = 16,
    //% block="clear stall latch"
    StallClear = 17,
    //% block="stop distance mm"
    StopDistance = 18,
    //% block="acceleration mm/s2"
    Accel = 19,
    //% block="deceleration mm/s2"
    Decel = 20,
    //% block="max speed mm/s"
    VMax = 21,
    // 22, 23, 24, 25, 26, 27, 29, 31 (BrakeFrac/DistTaper/YawTaper/
    // DistFloor/TurnFloor/RampMs/PlateauMinS/ProfileExit) are REMOVED
    // (design S4.7/S8): the
    // taper window, the floor fraction, the ramp time and the plateau/
    // profile-exit derate are all superseded by MotionLimits +
    // VelocityShaper -- see Accel/Decel/VMax/Jerk/OmegaMax/VFloor/
    // OmegaFloor above and below. No enum member exists for these
    // ordinals any more; `set config` can no longer be asked for one at
    // compile time, and the matching wire names answer `err 1` for one
    // release (wire_adapter.cpp's kFields).
    //% block="jerk"
    Jerk = 28,
    //% block="max turn rate deg/s"
    OmegaMax = 30,
    //% block="turn rate floor deg/s"
    OmegaFloor = 34,
    //% block="arrive distance mm"
    ArriveDist = 35,
    //% block="arrive yaw deg"
    ArriveYaw = 36,
    //% block="response lag (s)"
    Lag = 37
}

//% color=#0f9c5a icon="" block="DiffDrive"
//% groups='["Move", "Drive", "Wheels", "GoTo", "Moving?", "Stop", "Pose", "World", "Setup", "Remote", "Debug"]'
//% subcategories='["Pose", "Setup", "Extra"]'
namespace diffDrive {
    let defaultSpeed = 15      // [cm/s]
    let defaultYawRate = 90    // [deg/s]
    // Guards startDrive()'s background tick fiber so repeated calls
    // re-aim the running loop instead of stacking fibers. Cleared by
    // the loop itself when tickDrive() goes false.
    let driveLoopRunning = false

    // Start the wire-protocol loop (its own CODAL fiber -- see
    // protocol.h) as soon as this extension's code loads, independent
    // of whether any block below is ever placed in a user's program.
    // This is what makes the boot/HELLO identity banner go out
    // "without any host request" (sprint.md SUC-001). Idempotent (the
    // underlying Protocol object guards itself), and a no-op in the
    // simulator (see the shim body below) -- there's no serial link or
    // fiber scheduler to start there.
    _startProtocol()

    // ================= public API: velocity commands =================

    // Every block below carries an explicit weight= -- without one,
    // order ties break on file layout, which the module split (sprint
    // 012 ticket 007) changed. Weights are DESCENDING (higher renders
    // first) and are generated from reports/blocks-toolbox.csv as
    // (41 - new_order) * 10, so the CSV is the source of truth for
    // toolbox order; edit it and re-apply rather than hand-tuning here.
    // Group and subcategory assignment come from the same CSV. Note a
    // block carrying subcategory= is EXCLUDED from the parent flyout,
    // which is why the DiffDrive rows have none.

    /**
     * Set the two wheel speeds. Continuous-mode command: the robot only
     * moves while something keeps ticking the control loop -- run a
     * `while (diffDrive.driveTick())` loop after calling this to keep
     * the robot moving. If nothing ticks (e.g. your loop exits or the
     * program pauses), a safety watchdog stops the robot within about
     * 150 ms; a fresh move or tick loop resumes right away, no
     * clear-emergency-stop needed. (Position-mode blocks like move()
     * and whileMoving() tick internally, so this only matters for
     * setWheelSpeeds()/driveTwist().)
     * @param left left wheel speed, eg: 15
     * @param right right wheel speed, eg: 15
     */
    //% block="set wheel speeds left %left right %right cm/s"
    //% left.min=-50 left.max=50 right.min=-50 right.max=50
    //% group="Wheels" weight=380
    export function setWheelSpeeds(left: number, right: number): void {
        _setWheels(Math.round(left * 10), Math.round(right * 10))
    }

    /**
     * Drive with a body speed and yaw rate. Continuous-mode command:
     * the robot only moves while something keeps ticking the control
     * loop -- run a `while (diffDrive.driveTick())` loop after calling
     * this to keep the robot moving. If nothing ticks, a safety
     * watchdog stops the robot within about 150 ms; a fresh move or
     * tick loop resumes right away, no clear-emergency-stop needed.
     * @param speed forward speed, eg: 15
     * @param yawRate turn rate CCW+, eg: 0
     */
    //% block="drive %speed cm/s turning %yawRate deg/s"
    //% speed.min=-50 speed.max=50 yawRate.min=-180 yawRate.max=180
    //% group="Drive" weight=410
    export function driveTwist(speed: number, yawRate: number): void {
        _driveTwist(Math.round(speed * 10), Math.round(yawRate * 100))
    }

    /**
     * Start driving continuously AND keep the control loop ticking in
     * the background, so the robot actually moves without a
     * hand-written `while (diffDrive.driveTick())` loop. That
     * background ticking is the whole difference from drive(), which
     * only posts the command and leaves the ticking to you. Returns
     * immediately.
     *
     * The background loop ends by itself when the drive stops --
     * stop(), emergencyStop(), a stall, or the starvation watchdog --
     * because tickDrive() returns false there. Calling this again
     * while it is still running re-aims the existing loop rather than
     * stacking a second fiber.
     *
     * UNVERIFIED on hardware (added 2026-08-29): the background fiber
     * is new. tickDrive() is documented safe against a second fiber
     * calling it (shims.cpp's check-and-set guard is atomic on CODAL's
     * cooperative fibers), but a foreground move()/goTo() run WHILE
     * this loop is live means two fibers pacing the same kernel, which
     * has not been measured. Stop the drive before starting a
     * position-mode move.
     * @param speed forward speed, eg: 15
     * @param yawRate turn rate CCW+, eg: 0
     */
    //% block="start drive %speed cm/s turning %yawRate deg/s"
    //% group="Drive" weight=400
    export function startDrive(speed: number, yawRate: number): void {
        driveTwist(speed, yawRate)
        if (driveLoopRunning) return
        driveLoopRunning = true
        control.inBackground(() => {
            while (_tickDrive());
            driveLoopRunning = false
        })
    }

    /**
     * Run code while driving continuously. The body gets the live pose
     * each iteration. Unlike whileMoving(), a continuous drive has no
     * finish line of its own -- the loop runs until something stops the
     * drive (stop(), emergencyStop(), a stall, or driving to zero), so
     * give the body a way out.
     * @param speed forward speed, eg: 15
     * @param yawRate turn rate CCW+, eg: 0
     */
    //% block="while driving %speed cm/s turning %yawRate deg/s"
    //% draggableParameters="reporter" handlerStatement=1
    //% group="Drive" weight=390
    export function whileDriving(speed: number, yawRate: number,
        body: (x: number, y: number, heading: number) => void): void {
        driveTwist(speed, yawRate)
        while (_tickDrive()) {
            body(poseX(), poseY(), heading())
        }
        _endMove()
    }

    // ================= continuous-mode ticking ========================

    /**
     * Advance one control cycle. Run this in a loop --
     * `while (diffDrive.driveTick())` -- after setWheelSpeeds()/
     * driveTwist() to keep the robot moving; the robot only drives
     * while this (or an internally-ticking block like move()) keeps
     * getting called. Self-paces to the drive's control cadence
     * (about 24 ms), so don't add your own pause() in the loop.
     */
    //% block="drive tick"
    //% group="Moving?" weight=300
    export function driveTick(): boolean {
        return _tickDrive()
    }

    // ================= position-mode moves: blocking =================

    /**
     * Drive a distance while turning a yaw angle, then stop. Both at
     * once makes an arc. Waits until the move is done.
     * @param distance distance to travel, eg: 20
     * @param yaw angle to turn CCW+, eg: 0
     */
    //% block="move %distance cm turning %yaw degrees"
    //% group="Move" weight=440
    export function move(distance: number, yaw: number): void {
        startMove(distance, yaw)
        while (_tickDrive());
    }

    /**
     * Drive a curved path to a point in robot coordinates, then stop.
     * x is forward, y is left. Waits until the move is done.
     * @param x forward distance, eg: 20
     * @param y leftward distance, eg: 10
     */
    //% block="go to x %x cm y %y cm"
    //% group="GoTo" weight=370
    export function goTo(x: number, y: number): void {
        startGoTo(x, y)
        while (_tickDrive());
    }

    // ================= position-mode moves: async ====================

    /**
     * Start a move without waiting. Poll isMoving() / call stopMove().
     * KNOWN GAP under the tick model: this poll pattern does not, by
     * itself, advance the move -- something must still call
     * driveTick() (or otherwise tick the control loop) concurrently,
     * or the move never progresses and the safety watchdog stops it
     * within about 150 ms. Nothing supplies that tick automatically;
     * prefer move()/whileMoving() unless you are pairing this with your
     * own driveTick() loop.
     */
    //% block="start move %distance cm turning %yaw degrees"
    //% group="Move" weight=430
    export function startMove(distance: number, yaw: number): void {
        _startMove(Math.round(distance * 10), Math.round(yaw * 100),
            Math.round(defaultSpeed * 10),
            Math.round(defaultYawRate * 100))
    }

    /**
     * Start a go-to without waiting. Same tick-model gap as
     * startMove() -- see its doc comment: without a concurrent
     * driveTick() loop, this does not progress on its own.
     */
    //% block="start go to x %x cm y %y cm"
    //% group="GoTo" weight=350
    export function startGoTo(x: number, y: number): void {
        // Calls goToR() directly (the //%-exposed engineGoToRArmed/
        // engineSetGoToDeadline shim pair -- split in two, sprint 015
        // ticket 006, because a single five-parameter shim crashes the
        // PXT packager with TS9200; see sim.ts's _setGoToDeadline()/
        // _goToR() comments) instead of reducing to (distance, yaw) and
        // going through startMove()/moveX(): moveX()'s own >=50 deg
        // split reissues an arc-length/arc-angle pair as
        // pivot-then-straight, which lands at a DIFFERENT point than
        // the arc that pair was computed for. goToR() owns its own
        // bearing-then-chord split and short-arc wrap instead
        // (motion_engine.cpp), reaching (x, y) exactly.
        if (x == 0 && y == 0) return
        const goalX = Math.round(x * 10)  // [mm]
        const goalY = Math.round(y * 10)  // [mm]
        const goalSpeed = Math.round(defaultSpeed * 10)  // [mm/s]
        // arrive: 1 mm -- tight enough that "on target" means on
        // target, loose enough not to fight int-mm rounding.
        const goalArrive = 1  // [mm]
        // timeout: goToR() drives a <=180 deg pivot (its own short-arc
        // wrap) THEN the straight-line chord -- two SEQUENTIAL phases,
        // not one blended segment like startMove()'s reconciliation, so
        // their worst-case durations are summed, not maxed, using the
        // same defaultYawRate/defaultSpeed startMove() itself would use
        // for those two axes; +1500 ms mirrors startMove()'s own
        // end-of-move taper backstop (shims.cpp).
        const chordCm = Math.sqrt(x * x + y * y)
        const pivotS = 180 / defaultYawRate
        const straightS = chordCm / defaultSpeed
        const timeout = Math.round((pivotS + straightS) * 1000) + 1500  // [ms]
        // Must precede _goToR() immediately -- see
        // Rig::pendingGoToDeadline_'s comment (shims.cpp) for the
        // one-shot handoff contract this pair relies on.
        _setGoToDeadline(timeout)
        _goToR(goalX, goalY, goalSpeed, goalArrive)
    }

    /**
     * Is a move currently running? Calls _updateMove()
     * (MotionEngine::serviceMove() in the hardware shim), which
     * re-scales the taper/ramp, reissues the drive command, and can
     * end the move at its deadline backstop -- so this call DOES
     * advance the move as a side effect of checking it, not "state
     * only". It does not itself step the kernel (that is driveTick()'s
     * job): without a concurrent driveTick() loop, a move polled only
     * through isMoving() still stalls and the safety watchdog stops it
     * -- see startMove()'s doc comment for that gap.
     */
    //% block="moving?"
    //% group="Moving?" weight=330
    export function isMoving(): boolean {
        return _updateMove()
    }

    /**
     * Fraction of the current move completed, 0 to 1. Checks state
     * only -- same tick-model gap as isMoving(): it does not itself
     * advance the move (see startMove()'s doc comment).
     */
    //% block="move progress"
    //% group="Moving?" weight=320
    export function moveProgress(): number {
        return _progress() / 1000
    }

    /**
     * Stop the robot now -- including a continuous drive command in
     * progress (setWheelSpeeds()/driveTwist()), the same full-stop
     * contract stop() has (stop.ts). A no-op if the robot was already
     * idle. Note: under the tick model, a move started with
     * startMove()/startGoTo() and never paired with a driveTick() loop
     * will not have progressed anyway (see startMove()'s doc comment).
     */
    //% block="stop move"
    //% group="Stop" weight=290
    export function stopMove(): void {
        _endMove()
    }

    // ================= loop forms ====================================

    /**
     * Run code while moving. The body gets the live pose each
     * iteration; when the loop exits — move complete or stopMove() —
     * the move is over.
     */
    //% block="while moving %distance cm turning %yaw degrees"
    //% draggableParameters="reporter" handlerStatement=1
    //% group="Move" weight=420
    export function whileMoving(distance: number, yaw: number,
        body: (x: number, y: number, heading: number) => void): void {
        startMove(distance, yaw)
        while (_tickDrive()) {
            body(poseX(), poseY(), heading())
        }
        _endMove()
    }

    /**
     * Run code while going to a point. Same contract as whileMoving.
     */
    //% block="while going to x %x cm y %y cm"
    //% draggableParameters="reporter" handlerStatement=1
    //% group="GoTo" weight=340
    export function whileGoingTo(x: number, y: number,
        body: (x: number, y: number, heading: number) => void): void {
        startGoTo(x, y)
        while (_tickDrive()) {
            body(poseX(), poseY(), heading())
        }
        _endMove()
    }

    // ================= configuration =================================

    /**
     * Default speed for move/goTo blocks.
     * @param speed eg: 15
     */
    //% block="set default speed %speed cm/s"
    //% group="Setup" weight=50
    //% subcategory="Setup"
    export function setDefaultSpeed(speed: number): void {
        defaultSpeed = Math.max(1, speed)
    }

    /**
     * Default turn rate for move/goTo blocks.
     * @param yawRate eg: 90
     */
    //% block="set default turn rate %yawRate deg/s"
    //% group="Setup" weight=80
    //% subcategory="Setup"
    export function setDefaultYawRate(yawRate: number): void {
        defaultYawRate = Math.max(1, yawRate)
    }

    /**
     * Distance between the wheels, in cm.
     * @param width eg: 11.5
     */
    //% block="set track width %width cm"
    //% group="Setup" weight=110
    //% subcategory="Setup"
    export function setTrackWidth(width: number): void {
        _setGeometry(Math.round(width * 100), 0)
    }

    /**
     * Wheel travel per shaft degree, in mm/degree.
     * @param calib eg: 0.7837
     */
    //% block="set wheel calibration %calib mm/deg"
    //% group="Setup" weight=100
    //% subcategory="Setup"
    export function setWheelCalibration(calib: number): void {
        _setGeometry(0, Math.round(calib * 10000))
    }

    /**
     * Advanced: set a kernel configuration value directly.
     */
    //% block="set config %field to %value"
    //% group="Setup" weight=70
    //% subcategory="Setup"
    export function setConfigValue(field: ConfigField,
        value: number): void {
        _setKernelValue(field, Math.round(value * 1000))
    }

}
