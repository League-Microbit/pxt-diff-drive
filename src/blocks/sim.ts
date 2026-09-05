namespace diffDrive {
    // =========== internal shims (simulator fallback bodies) ==========
    // Hardware uses the C++ in shims.cpp; the bodies below are a
    // minimal kinematic stand-in so programs behave in the browser.

    let simX = 0            // [mm]
    let simY = 0            // [mm]
    let simHeading = 0      // [rad]
    let simVel = 0          // [mm/s]
    let simYawRate = 0      // [rad/s]
    let simLast = 0  // [ms]
    let simMoveRemainDist = 0  // [mm]
    let simMoveRemainYaw = 0  // [rad]
    let simMoveActive = false

    // Pivot-then-straight split bookkeeping, mirroring the real motion
    // engine's own queued-second-phase fields (a pivot segment that,
    // once finished, hands off to a queued straight segment). Needed
    // because simMoveRemainDist/simMoveRemainYaw alone can't express
    // "there is a second phase still to come" once the pivot phase has
    // legitimately driven simMoveRemainDist to 0 for its own duration
    // -- see _startMove()/simIntegrate() below.
    let simMoveHasPendingStraight = false
    let simMovePendingDistance = 0  // [mm] signed
    let simMovePendingSpeed = 0  // [mm/s]

    // E-stop latch (sprint 007 ticket 004, closes R-13/BLK-07): mirrors
    // hardware's estopLatch_ (diffdrive.h/.cpp). Set by _estopAll(),
    // cleared only by _estopClear(); gates _setWheels()/_driveTwist()/
    // _startMove() at INTAKE, mirroring checkCommandable()'s
    // Status::kRefusedEstopped gate -- not a per-tick override like
    // step()'s own `effective = kModeNeutral` (diffdrive.cpp), because
    // nothing else in this simulator can introduce velocity between
    // calls, so intake refusal alone is sufficient. _stopAll() (plain
    // "stop") deliberately never touches this latch, the same
    // stop-vs-latch distinction shims.cpp's deliverStopNow() documents
    // for hardware.
    let simEstopped = false

    // Tick-engine sim state: _tickDrive()'s simulator body mirrors
    // shims.cpp's absolute-deadline pacing (tickDrive(), 24 ms
    // cadence) so a simulator-run program is timing-observable the same
    // way hardware is -- an anchored deadline while ticks stay
    // consecutive, re-anchored to "now" after a gap.
    const kSimTickPeriod = 24  // [ms]
    let simTickDeadline = 0    // [ms] 0 = no tick has run yet
    let simCycleCount = 0
    let simTickOverrunCount = 0

    function simIntegrate(): void {
        const now = control.millis()
        let dt = (now - simLast) / 1000
        if (dt < 0 || dt > 0.5) dt = 0
        simLast = now
        if (dt == 0) return
        // Capture the velocity/yaw-rate in effect for THIS step before
        // any end-of-move zeroing below, and clip this step's own
        // contribution to the fraction of dt actually needed to reach
        // the target, so a move that finishes partway through a step
        // neither overshoots nor undershoots.
        const stepVel = simVel
        const stepYawRate = simYawRate
        let stepDt = dt
        if (simMoveActive) {
            const dDist = simVel * dt  // [mm]
            const dYaw = simYawRate * dt  // [rad]
            let frac = 1
            if (dDist != 0 && simMoveRemainDist < Math.abs(dDist)) {
                const f = simMoveRemainDist / Math.abs(dDist)
                if (f < frac) frac = f
            }
            if (dYaw != 0 && simMoveRemainYaw < Math.abs(dYaw)) {
                const f = simMoveRemainYaw / Math.abs(dYaw)
                if (f < frac) frac = f
            }
            if (frac < 0) frac = 0
            stepDt = dt * frac

            simMoveRemainDist -= Math.abs(dDist)
            simMoveRemainYaw -= Math.abs(dYaw)
            if (simMoveRemainDist <= 0 && simMoveRemainYaw <= 0) {
                if (simMoveHasPendingStraight) {
                    // The pivot phase just finished -- start the
                    // queued straight phase now, the same sequential
                    // hand-off the real motion engine performs once
                    // its own pivot segment ends.
                    simMoveHasPendingStraight = false
                    const straightDistance = simMovePendingDistance
                    const straightSpeed = simMovePendingSpeed
                    simYawRate = 0
                    const straightDuration =
                        straightDistance != 0 && straightSpeed > 0
                            ? Math.abs(straightDistance) / straightSpeed : 0
                    if (straightDuration > 0) {
                        simMoveRemainDist = Math.abs(straightDistance)
                        simMoveRemainYaw = 0
                        simVel = straightDistance / straightDuration
                        simMoveActive = true
                    } else {
                        simMoveActive = false
                        simVel = 0
                    }
                } else {
                    simMoveActive = false
                    simVel = 0
                    simYawRate = 0
                }
            }
        }
        const mid = simHeading + stepYawRate * stepDt / 2
        simX += stepVel * stepDt * Math.cos(mid)
        simY += stepVel * stepDt * Math.sin(mid)
        simHeading += stepYawRate * stepDt
    }

    // Live stand-ins for motion_engine.h's trackWidth_/rotationalSlip_.
    // Default values mirror that header's own compiled defaults exactly
    // (drift-tested, see tests/host) so an unconfigured simulator
    // matches an unconfigured robot. _setGeometry()/_setKernelValue()
    // below now actually update these instead of discarding their
    // arguments, so a calibrated robot's browser twin turns the same
    // way its own hardware does once a program pastes its calibration
    // block. Kept as two named variables (not one derived value) so a
    // future geometry/slip update can't silently reopen the gap between
    // this divisor and effectiveTrackWidth()'s.
    let simTrackWidth = 114.2  // [mm]
    let simRotationalSlip = 0.952

    //% shim=diffDrive::setWheels
    export function _setWheels(left: number, right: number): void {
        simIntegrate()
        if (simEstopped) return
        simVel = (left + right) / 2
        // Differential-drive kinematics: omega [rad/s] = (v_right -
        // v_left) [mm/s] / trackWidth [mm] -- the same relation
        // _driveTwist() below applies in reverse (its hardware shim
        // computes `twist = yaw * 0.5 * effectiveTrackWidth()`,
        // shims.cpp). Hardware's own divisor is NOT trackWidth_ alone:
        // it is effectiveTrackWidth() = trackWidth_ / rotationalSlip_
        // (motion_engine.h). _driveTwist() below needs no matching
        // update: it converts yawRate straight to rad/s with no
        // trackWidth divisor of its own, because hardware's own
        // multiply-by-effectiveTrackWidth()-then-divide-by-it round trip
        // (twist -> wheelsV -> omega) cancels algebraically for ANY
        // trackWidth/slip value -- the two paths' observable yaw rate
        // is exactly equal without _driveTwist() ever reading either
        // one. The simulator's contract is exact parity on *observable*
        // kinematic output, not a physical model of hardware's
        // calibration mechanism -- rotationalSlip_ corrects for a real
        // wheel imperfection this idealized simulator has no equivalent
        // of -- so this divides by the same two constituent values
        // (simTrackWidth / simRotationalSlip, just above -- now live
        // state, not fixed constants) rather than growing its own
        // "slip" concept. (Previously divided by trackWidth_ alone via a
        // bare 115 literal -- a 4.3% discrepancy against hardware and
        // against _driveTwist() below. Before that, divided by 10 first
        // as well, an erroneous effective 1150 mm track that turned 10x
        // too slowly -- R-12/BLK-06.)
        simYawRate = (right - left) / (simTrackWidth / simRotationalSlip)  // [rad/s]
        simMoveActive = false
    }

    //% shim=diffDrive::driveTwist
    export function _driveTwist(speed: number, yawRate: number): void {
        simIntegrate()
        if (simEstopped) return
        simVel = speed
        simYawRate = (yawRate / 100) * Math.PI / 180
        simMoveActive = false
    }

    // [rad] a nonzero distance combined with a rotation at/above this
    // is NOT one blended segment on the real motion engine -- pivot to
    // the new heading FIRST, then drive the distance straight, as two
    // SEQUENTIAL phases (see _startMove() below). Read, not re-typed:
    // drift-tested against the real firmware constant this mirrors so
    // the two copies can't diverge silently.
    const kSimTurnFirstAngle = 0.8726646  // [rad]

    //% shim=diffDrive::startMove
    export function _startMove(distance: number, yaw: number, speed: number,
        yawRate: number): void {
        simIntegrate()
        if (simEstopped) return
        simMoveHasPendingStraight = false
        const yawMagnitude = Math.abs(yaw / 100) * Math.PI / 180  // [rad]
        // Mirrors the real motion engine's own split condition exactly
        // (nonzero distance AND |rotation| at/above the shared
        // threshold): below the threshold, or a pure pivot/pure
        // straight, behavior is UNCHANGED -- one blended segment, the
        // `else` branch below.
        if (distance != 0 && yawMagnitude >= kSimTurnFirstAngle) {
            const yawDur = Math.abs(yaw) / yawRate
            if (yawDur <= 0) return
            simMoveRemainYaw = yawMagnitude
            simMoveRemainDist = 0
            simVel = 0
            simYawRate = ((yaw / 100) * Math.PI / 180) / yawDur
            simMoveHasPendingStraight = true
            simMovePendingDistance = distance
            simMovePendingSpeed = speed
            simMoveActive = true
            return
        }
        simMoveRemainDist = Math.abs(distance)
        simMoveRemainYaw = yawMagnitude
        let duration = 0
        if (distance != 0) duration = Math.abs(distance) / speed
        if (yaw != 0) {
            const yawDur = Math.abs(yaw) / yawRate
            if (yawDur > duration) duration = yawDur
        }
        if (duration <= 0) return
        simVel = distance / duration
        simYawRate = ((yaw / 100) * Math.PI / 180) / duration
        simMoveActive = true
    }

    //% shim=diffDrive::updateMove
    export function _updateMove(): boolean {
        simIntegrate()
        return simMoveActive
    }

    // [ms] -- pre-arms the NEXT _goToR() call's deadline. Split out of
    // what used to be a single five-parameter _goToR()/engineGoToR()
    // shim pair (sprint 015 ticket 006): a real PXT build of that
    // version reproduced "TS9200: Assertion failed" deterministically,
    // surviving make_deploy.py's own retry for the benign packaging-
    // abort shape tools/DESIGN.md documents under the same error code
    // -- so this was the arity, not a nondeterministic abort. See
    // shims.cpp's engineSetGoToDeadline()/engineGoToRArmed() for the
    // native side of this same split. `timeout` was always a
    // hardware-only deadline backstop the simulator never used (see
    // _goToR()'s own comment below), so this is a genuine no-op here
    // -- nothing to store, nothing for _goToR() below to read.
    //% shim=diffDrive::engineSetGoToDeadline
    export function _setGoToDeadline(timeout: number): void {  // [ms]
        // Simulator: no-op. `timeout` is a hardware-only deadline
        // backstop; nothing can strand a move in this simulator. The
        // explicit `return` below is load-bearing, not decorative: a
        // body with zero statements -- even one containing only this
        // comment -- is indistinguishable, to pxt, from a bare `{}`.
        // Both are emitted as a native-only shim call with no pxsim
        // implementation, crashing the simulator exactly like this
        // file's other previously-empty-bodied shims.
        return
    }

    // [cdeg/s] -- pre-arms the NEXT _goToR() call's yaw-rate ceiling,
    // the sim-side twin of shims.cpp's Rig::pendingGoToYawRate_/
    // engineSetGoToYawRate(). Same one-shot-handoff, <=4-param-per-shim
    // reason _setGoToDeadline() immediately above exists: this is
    // startGoTo()'s (blocks/motion.ts) "defaultYawRate, converted to
    // this shim's own centidegree-per-second convention" ceiling,
    // reconciled against `speed` inside _goToR() below the same way
    // _startMove() above reconciles its own two rate ceilings -- unlike
    // `timeout`, this one is NOT a no-op here.
    let simPendingGoToYawRate = 0  // [cdeg/s]

    //% shim=diffDrive::engineSetGoToYawRate
    export function _setGoToYawRate(yawRate: number): void {  // [cdeg/s]
        simPendingGoToYawRate = yawRate
    }

    // [mm] [mm] [mm/s] [mm] -- simulator stand-in for
    // MotionEngine::goToR()'s arc reduction (motion_engine.cpp): bearing
    // = atan2(y, x), turn angle theta = 2*bearing wrapped to the short
    // arc (|theta| <= pi, same wrap goToR() applies, KERN-03) so a
    // target behind the robot turns the short way, not almost a full
    // circle. Unlike _startMove() below (which now mirrors the real
    // motion engine's own >=50 deg pivot-then-straight split), this
    // reduction is always a single blended arc regardless of angle --
    // an arc-length/arc-angle pair reissued as pivot-then-straight
    // would land at a DIFFERENT point than the arc it was computed
    // for (arc length != chord length except in the limit), so goToR's
    // own reduction deliberately never splits; it reaches (x, y)
    // exactly by construction instead. FOUR params, not five: the
    // fifth (`timeout`, a hardware-only deadline backstop -- nothing
    // can strand a move in this simulator) moved to _setGoToDeadline()
    // immediately above, matching shims.cpp's native-side split.
    // Params typed `number`, matching the rest of this file's
    // shim-fallback functions now that none of them declares an int32
    // local/parameter -- int32 locals/params on a function with a TS
    // body fail the JS->Blocks decompiler with TS9256. The native shim
    // ABI is governed by the C++ signature, not this declaration, so
    // `number` here is hardware-safe.
    //
    // `speed` and the pre-armed `simPendingGoToYawRate` are two
    // INDEPENDENT rate ceilings (mirroring _startMove()'s own `speed`/
    // `yawRate` pair) reconciled into a single `duration` here --
    // whichever axis takes LONGER at its own ceiling governs, same
    // "whichever axis takes longer governs" reconciliation
    // _startMove()'s non-split (blended) branch above already applies,
    // not the pivot-then-straight budget its split branch uses: this
    // reduction never splits (see this function's own comment above),
    // so there is only ever one blended segment to budget for. Fixes
    // the pivot arc previously always running at whatever angular rate
    // `speed` alone implied, ignoring "set default turn rate" entirely.
    //% shim=diffDrive::engineGoToRArmed
    export function _goToR(x: number, y: number, speed: number,
        arrive: number): void {
        simIntegrate()
        if (simEstopped) return
        const chord = Math.sqrt(x * x + y * y)
        if (chord <= arrive) return
        const bearing = Math.atan2(y, x)   // [rad] already short-arc
        let theta = 2 * bearing            // [rad] |.| < 2*pi
        if (theta > Math.PI) theta -= 2 * Math.PI
        else if (theta <= -Math.PI) theta += 2 * Math.PI
        let s: number
        if (Math.abs(y) < 0.1) {           // ~0.1 mm: call it straight
            s = x
        } else {
            const radius = (x * x + y * y) / (2 * y)
            s = radius * theta
        }
        const spd = speed > 0 ? speed : 1
        const turnRate = (simPendingGoToYawRate > 0 ? simPendingGoToYawRate : 1)
            / 100 * Math.PI / 180  // [rad/s]
        let duration = 0
        if (s != 0) duration = Math.abs(s) / spd
        if (theta != 0) {
            const turnDuration = Math.abs(theta) / turnRate
            if (turnDuration > duration) duration = turnDuration
        }
        if (duration <= 0) return
        simMoveRemainDist = Math.abs(s)
        simMoveRemainYaw = Math.abs(theta)
        simVel = s / duration
        simYawRate = theta / duration
        simMoveActive = true
    }

    // Simulator body for the tick engine: integrate one step (kinematic
    // stand-in for kernel.step()+serviceMove()), then self-pace to the
    // next absolute 24 ms schedule with basic.pause(), same anchoring
    // rule as tickDrive() in shims.cpp -- so blocking/loop forms built
    // on `while (_tickDrive())` behave the same way in the browser as
    // on hardware. Always steps (simIntegrate()), even with no move
    // active, matching the hardware contract that continuous-mode
    // driving depends on.
    //
    // Returns `simMoveActive || simVel != 0 || simYawRate != 0` (sprint
    // 007 ticket 002, closes R-10/API-01) -- the simulator-state mirror
    // of shims.cpp's `commandLooksActive(r)`, not raw `simMoveActive`.
    // simIntegrate() (just above) already zeroed simVel/simYawRate
    // synchronously, in this same call, if a position-mode move
    // completed on this step -- there is no motor coast-down to model
    // in the browser, so no settle-loop equivalent is needed the way
    // shims.cpp's tickDrive() needs one on hardware -- so a move's final
    // tick still returns false here, same as before. A continuous-mode
    // command (setWheelSpeeds()/driveTwist(), which leave
    // simMoveActive == false but simVel/simYawRate nonzero) now keeps
    // this true instead of returning false on the very first tick, the
    // same fix as shims.cpp's own. See
    // tests/host/test_continuous_drive_command_looks_active.py for the
    // host-side proof of the equivalent hardware condition (this
    // simulator body itself is not host-testable -- no automated check
    // reaches this TypeScript layer; see this ticket's C++11 Gate Coverage).
    //% shim=diffDrive::tickDrive
    export function _tickDrive(): boolean {
        simIntegrate()
        simCycleCount += 1
        const stillCommanded = simMoveActive || simVel != 0 || simYawRate != 0

        const now = control.millis()
        const consecutive = simTickDeadline != 0 &&
            now < simTickDeadline + kSimTickPeriod
        const deadline = consecutive ?
            simTickDeadline + kSimTickPeriod : now + kSimTickPeriod
        simTickDeadline = deadline

        const wait = deadline - control.millis()
        if (wait > 0) {
            basic.pause(wait)
        } else {
            simTickOverrunCount += 1
        }
        return stillCommanded
    }

    //% shim=diffDrive::progress
    export function _progress(): int32 {
        simIntegrate()
        return simMoveActive ? 500 : 1000
    }

    //% shim=diffDrive::endMove
    export function _endMove(): void {
        simIntegrate()
        simMoveActive = false
        simVel = 0
        simYawRate = 0
    }

    //% shim=diffDrive::stopAll
    export function _stopAll(): void {
        simIntegrate()
        simMoveActive = false
        simVel = 0
        simYawRate = 0
    }

    //% shim=diffDrive::estopAll
    export function _estopAll(): void {
        _stopAll()
        simEstopped = true
    }

    //% shim=diffDrive::estopClear
    export function _estopClear(): void {
        simEstopped = false
    }

    // Stall latch clear/readback (sprint 007 ticket 001): no-ops in the
    // simulator -- there is no stall model in the browser, matching
    // this file's existing precedent for setGeometry/setKernelValue's
    // simulator fallbacks (specification.md §5). Real (if trivial) body
    // below, not a bare `{}`: pxt emits an empty-bodied shim as
    // native-only, and no pxsim implementation exists, so the simulator
    // crashes at the call site.
    //% shim=diffDrive::clearStall
    export function _clearStallLatch(): void {
        return
    }

    //% shim=diffDrive::isStalled
    export function _isStalled(): boolean { return false }

    //% shim=diffDrive::poseX
    export function _poseX(): int32 {
        simIntegrate()
        return Math.round(simX)
    }

    //% shim=diffDrive::poseY
    export function _poseY(): int32 {
        simIntegrate()
        return Math.round(simY)
    }

    //% shim=diffDrive::poseHeading
    export function _poseHeading(): int32 {
        simIntegrate()
        return Math.round(simHeading * 180 / Math.PI * 100)
    }

    //% shim=diffDrive::resetPose
    export function _resetPose(): void {
        simIntegrate()
        simX = 0
        simY = 0
        simHeading = 0
    }

    // Geometry/kernel-tuning setters: _setGeometry() below now updates
    // the live simTrackWidth state _setWheels() actually divides by
    // (see that function's own comment). Its `calib` argument (wheel
    // travel calibration, mm per shaft degree) has no simulator model
    // to route to -- this simulator integrates simVel directly rather
    // than converting a move from an encoder count, so there is nothing
    // for that value to correct -- and is validated the same way but
    // not stored. _setKernelValue() below forwards field 16
    // (ConfigField.RotationalSlip in blocks/motion.ts; wire name
    // "rotational_slip") to the paired simRotationalSlip state; every
    // other field still has no simulator model and stays a silent
    // no-op, same as before. Both keep real (if partial) bodies, not
    // bare `{}`: an empty body is emitted by pxt as native-only, and no
    // pxsim implementation exists for either, so the simulator crashes
    // at the call site.

    //% shim=diffDrive::setGeometry
    export function _setGeometry(trackWidth: number, calib: number): void {  // [0.1 mm] [1e-4 mm/deg]
        if (trackWidth > 0) simTrackWidth = trackWidth * 0.1
    }

    //% shim=diffDrive::setKernelValue
    export function _setKernelValue(field: number, value: number): void {  // [x1000 scaled]
        const v = value * 0.001
        if (field == 16 && v > 0) simRotationalSlip = v
    }

    // Recorded so a bare project's on-start sequence is observable in
    // the simulator; hardware's real startProtocol() bring-up has no
    // other in-sim effect to model.
    let simProtocolStarted = false

    //% shim=diffDrive::startProtocol
    export function _startProtocol(): void {
        simProtocolStarted = true
    }

    // ---- OTOS (zeguz bench bring-up) -- shim-only surface, no blocks.
    // Call only from the fiber that calls driveTick(): an OTOS I2C
    // transaction interposed in the Nezha encoder's select->read
    // window destroys the encoder sample. Sim fallbacks report a
    // sensor that is absent.

    /**
     * Read one diagnostic value. See diagValue() in shims.cpp for the
     * index list: 2 stall halted (see isStalled(), the named block for
     * this same bit), 10/11 encoder positions, 12/13 applied duty
     * percent x100 (10000 == full duty), 14/15 velocities, 6/7 wedge
     * suspicion.
     */
    //% shim=diffDrive::probe
    export function probe(what: number): number { return 0 }

    /**
     * RETIRED (design S4.7/S8): no-ops kept for one
     * release so a program saved before this sprint still compiles and
     * runs. Shaping is now `set config`'s own accel/decel/jerk/v_max/
     * omega_max/v_floor/omega_floor fields (ConfigField), or the
     * `setLimits` shim `test.ts`'s profile functions use. Simulator: no
     * taper/floor/ramp shaping model exists in the browser, so these
     * were already no-ops there -- real (if trivial) bodies below, not
     * bare `{}`, so pxt doesn't treat them as native-only shims (no
     * pxsim implementation exists for any of the three, so an empty
     * body would crash the simulator at the call site).
     */
    //% shim=diffDrive::setTaperWindows
    export function setTaperWindows(dist: number, yaw: number): void {
        return
    }

    //% shim=diffDrive::setTaperFloors
    export function setTaperFloors(dist: number, turn: number): void {
        return
    }

    //% shim=diffDrive::setRampMs
    export function setRampMs(ms: number): void {
        return
    }

    // The replacement for the three retired shims above -- see
    // shims.cpp's own setLimits() comment for the full rationale (four
    // plain-unit int params, no wire x1000 scaling). Simulator has no
    // shaping model to update; recorded into its own last-seen-args
    // variables below so a test can still observe the call landed (the
    // paired ConfigField.RotationalSlip case above has a live state
    // variable to route to instead, so it no longer needs this pattern).
    let simLastLimitsAccel = 0
    let simLastLimitsDecel = 0
    let simLastLimitsVMax = 0
    let simLastLimitsOmegaMax = 0

    //% shim=diffDrive::setLimits
    export function setLimits(accel: number, decel: number, vMax: number,
        omegaMax: number): void {
        simLastLimitsAccel = accel
        simLastLimitsDecel = decel
        simLastLimitsVMax = vMax
        simLastLimitsOmegaMax = omegaMax
    }

    //% shim=diffDrive::otosBegin
    export function otosBegin(): number { return 0 }

    //% shim=diffDrive::otosRead
    export function otosRead(): boolean { return false }

    /**
     * Cached OTOS value: 0=x [0.1mm] 1=y [0.1mm] 2=heading [cdeg]
     * 3=vx [mm/s] 4=vy [mm/s] 5=omega [cdeg/s] 6=product id
     * 7=connected 8=IMU-cal samples remaining
     */
    //% shim=diffDrive::otosGet
    export function otosGet(what: number): number { return 0 }

    // Sim fallbacks below report a sensor that is absent (see this
    // section's header comment); real (if trivial) bodies, not bare
    // `{}`, so pxt doesn't treat them as native-only shims (no pxsim
    // implementation exists for any of the three, so an empty body
    // would crash the simulator at the call site).
    //% shim=diffDrive::otosZero
    export function otosZero(): void {
        return
    }

    //% shim=diffDrive::otosCalibrate
    export function otosCalibrate(samples: number): void {
        return
    }

    //% shim=diffDrive::otosSetOffset
    export function otosSetOffset(x: number, y: number, yaw: number): void {
        return
    }

    /**
     * Write a line to BOTH transports -- USB serial and the wireless
     * link. Test programs use this instead of serial.writeLine, which
     * reaches the cable only, and the cable only reaches the bench
     * stand where the wheels are off the ground.
     *
     * Student code inside an event handler (a button press, a radio
     * receive) MUST call diffDrive.emitLine here, never PXT's own
     * serial.writeLine/serial.writeString: those go straight to the
     * device's serial port from whatever fiber calls them, and this
     * extension has no way to route that path through its own queue
     * from the inside.
     *
     * (Do not write the word r-a-d-i-o followed by a full stop in this
     * file: PXT scans the TypeScript for `<name>.` to auto-add package
     * dependencies, and a prose mention makes it demand a `radio`
     * package this project does not use -- it drives CODAL's radio
     * directly from radio_transport.cpp.)
     */
    //% shim=diffDrive::emitLine
    export function emitLine(text: string): void {
        serial.writeLine(text)
    }

    // Text of whichever RUN command is currently being dispatched. The
    // simulator has no wire, so nothing ever calls the dispatch callback
    // registered below and this body is never reached.
    //% shim=diffDrive::runCommandText
    export function runCommandText(): string {
        return ""
    }

    // Registers the callback protocol.cpp's dispatchJob() invokes
    // directly once per dequeued RUN command (replacing the old
    // control.onEvent()-based MessageBus registration). The simulator
    // has no wire and never dequeues a RUN command, so this has nothing
    // to model -- a real (if trivial) body, not a bare `{}`, for the
    // same reason every other shim-only stub in this file has one: an
    // empty body is emitted by pxt as native-only, and no pxsim
    // implementation exists, which crashes the simulator at the call
    // site.
    //% shim=diffDrive::registerRunDispatch
    export function _registerRunDispatch(cb: () => void): void {
        return
    }

    //% shim=diffDrive::seedPose
    export function _seedPose(x: number, y: number, heading: number): void {
        simIntegrate()
        simX = x
        simY = y
        simHeading = heading / 100 * Math.PI / 180
    }

    // Recorded so the "setup radio" block (blocks/run.ts) is observable
    // in the simulator; there is no radio in the browser, so this has no
    // other in-sim effect to model. Real (if trivial) body, not a bare
    // `{}` -- an empty body is emitted by pxt as native-only, and no
    // pxsim implementation exists, which crashes the simulator at the
    // call site (the exact defect fixed elsewhere in this file).
    let simRadioChannel = 4
    let simRadioGroup = 10
    let simRadioEnabled = false

    // Params typed `number`, not `int32` -- see _goToR()'s comment
    // above: an int32 param on a function with a TS body fails the
    // JS->Blocks decompiler with TS9256. The native ABI is governed by
    // the C++ signature, not this declaration.
    //% shim=diffDrive::setupRadio
    export function _setupRadio(channel: number, group: number): void {
        simRadioChannel = channel
        simRadioGroup = group
    }

    //% shim=diffDrive::enableRadioLink
    export function _enableRadioLink(): void {
        simRadioEnabled = true
    }

    //% shim=diffDrive::enableWifiLink
    export function _enableWifiLink(): void {
        // No simulator model of the WiFi module: a no-op here, exactly
        // as the radio link is a flag with no behaviour behind it.
    }
}
