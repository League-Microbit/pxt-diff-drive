namespace diffDrive {
    // =========== internal shims (simulator fallback bodies) ==========
    // Hardware uses the C++ in shims.cpp; the bodies below are a
    // minimal kinematic stand-in so programs behave in the browser.

    let simX = 0            // [mm]
    let simY = 0            // [mm]
    let simHeading = 0      // [rad]
    let simVel = 0          // [mm/s]
    let simYawRate = 0      // [rad/s]
    let simLastMs = 0
    let simMoveRemainMm = 0
    let simMoveRemainRad = 0
    let simMoveActive = false

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
    const kSimTickPeriodMs = 24
    let simTickDeadlineMs = 0    // 0 = no tick has run yet
    let simCycleCount = 0
    let simTickOverrunCount = 0

    function simIntegrate(): void {
        const now = control.millis()
        let dt = (now - simLastMs) / 1000
        if (dt < 0 || dt > 0.5) dt = 0
        simLastMs = now
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
            const dMm = simVel * dt
            const dRad = simYawRate * dt
            let frac = 1
            if (dMm != 0 && simMoveRemainMm < Math.abs(dMm)) {
                const f = simMoveRemainMm / Math.abs(dMm)
                if (f < frac) frac = f
            }
            if (dRad != 0 && simMoveRemainRad < Math.abs(dRad)) {
                const f = simMoveRemainRad / Math.abs(dRad)
                if (f < frac) frac = f
            }
            if (frac < 0) frac = 0
            stepDt = dt * frac

            simMoveRemainMm -= Math.abs(dMm)
            simMoveRemainRad -= Math.abs(dRad)
            if (simMoveRemainMm <= 0 && simMoveRemainRad <= 0) {
                simMoveActive = false
                simVel = 0
                simYawRate = 0
            }
        }
        const mid = simHeading + stepYawRate * stepDt / 2
        simX += stepVel * stepDt * Math.cos(mid)
        simY += stepVel * stepDt * Math.sin(mid)
        simHeading += stepYawRate * stepDt
    }

    // Fixed stand-ins for motion_engine.h's trackWidth_/rotationalSlip_
    // (caliper-measured 114.2 mm; camera-derived 0.952 slip correction).
    // setGeometry() is a no-op in the simulator (see _setGeometry below),
    // so there is no live value to read -- these are the simulator's own
    // copies, kept as two named constants (not one derived literal) so a
    // future geometry/slip bake update can't silently reopen the gap
    // between this divisor and effectiveTrackWidth()'s.
    const kSimTrackWidthMm = 114.2
    const kSimRotationalSlip = 0.952

    //% shim=diffDrive::setWheels
    export function _setWheels(left: number, right: number): void {
        simIntegrate()
        if (simEstopped) return
        simVel = (left + right) / 2
        // Differential-drive kinematics: omega [rad/s] = (v_right -
        // v_left) [mm/s] / trackWidth [mm] -- the same relation
        // _driveTwist() below applies in reverse (its hardware shim
        // computes `twistMmS = yawRad * 0.5 * effectiveTrackWidth()`,
        // shims.cpp). Hardware's own divisor is NOT trackWidth_ alone:
        // it is effectiveTrackWidth() = trackWidth_ / rotationalSlip_
        // (motion_engine.h), and _driveTwist() below already reproduces
        // that exactly. The simulator's contract is exact parity on
        // *observable* kinematic output, not a physical model of
        // hardware's calibration mechanism -- rotationalSlip_ corrects
        // for a real wheel imperfection this idealized simulator has no
        // equivalent of -- so this divides by the same two constituent
        // constants (kSimTrackWidthMm / kSimRotationalSlip, just above)
        // rather than growing its own "slip" concept. (Previously
        // divided by trackWidth_ alone via a bare 115 literal -- a 4.3%
        // discrepancy against hardware and against _driveTwist() below.
        // Before that, divided by 10 first as well, an erroneous
        // effective 1150 mm track that turned 10x too slowly --
        // R-12/BLK-06.)
        simYawRate = (right - left) / (kSimTrackWidthMm / kSimRotationalSlip)  // [rad/s]
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

    //% shim=diffDrive::startMove
    export function _startMove(distance: number, yaw: number, speed: number,
        yawRate: number): void {
        simIntegrate()
        if (simEstopped) return
        simMoveRemainMm = Math.abs(distance)
        simMoveRemainRad = Math.abs(yaw / 100) * Math.PI / 180
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
    export function _setGoToDeadline(timeoutMs: number): void {
        // Simulator: no-op. `timeoutMs` is a hardware-only deadline
        // backstop; nothing can strand a move in this simulator. The
        // explicit `return` below is load-bearing, not decorative: a
        // body with zero statements -- even one containing only this
        // comment -- is indistinguishable, to pxt, from a bare `{}`.
        // Both are emitted as a native-only shim call with no pxsim
        // implementation, crashing the simulator exactly like this
        // file's other previously-empty-bodied shims.
        return
    }

    // [mm] [mm] [mm/s] [mm] -- simulator stand-in for
    // MotionEngine::goToR()'s arc reduction (motion_engine.cpp): bearing
    // = atan2(y, x), turn angle theta = 2*bearing wrapped to the short
    // arc (|theta| <= pi, same wrap goToR() applies, KERN-03) so a
    // target behind the robot turns the short way, not almost a full
    // circle. Unlike hardware, this simulator has no wheel-duty
    // constraint forcing goToR()'s own >=50 deg pivot-then-straight
    // split (KERN-02) -- blending the whole arc as one segment, the same
    // way _startMove() already blends distance+yaw, reaches (x, y)
    // exactly, so no split is needed here. FOUR params, not five: the
    // fifth (`timeout`, a hardware-only deadline backstop -- nothing
    // can strand a move in this simulator) moved to _setGoToDeadline()
    // immediately above, matching shims.cpp's native-side split.
    // Params typed `number`, matching the rest of this file's
    // shim-fallback functions now that none of them declares an int32
    // local/parameter -- int32 locals/params on a function with a TS
    // body fail the JS->Blocks decompiler with TS9256. The native shim
    // ABI is governed by the C++ signature, not this declaration, so
    // `number` here is hardware-safe.
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
        const duration = Math.abs(s) / spd
        if (duration <= 0) return
        simMoveRemainMm = Math.abs(s)
        simMoveRemainRad = Math.abs(theta)
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
        const consecutive = simTickDeadlineMs != 0 &&
            now < simTickDeadlineMs + kSimTickPeriodMs
        const deadline = consecutive ?
            simTickDeadlineMs + kSimTickPeriodMs : now + kSimTickPeriodMs
        simTickDeadlineMs = deadline

        const wait = deadline - control.millis()
        if (wait > 0) {
            basic.pause(wait)
        } else {
            simTickOverrunCount += 1
        }
        return stillCommanded
    }

    // Minimal simulator stand-in for cycleStat() -- there is no real
    // fiber/cycle timing to observe in the browser simulator, so this
    // reports the nominal cadence plus the sim's own tick/overrun
    // counters (kept for parity with hardware's field layout) rather
    // than measured timing.
    //% shim=diffDrive::cycleStat
    function _cycleStat(which: number): int32 {
        switch (which) {
            case 0: return kSimTickPeriodMs * 1000  // nominal period [us]
            case 1: return 0                        // busy [us]: not modeled
            case 2: return simTickOverrunCount
            case 3: return simCycleCount
            default: return 0
        }
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

    // Simulator: geometry/kernel-tuning setters stay no-ops on purpose
    // (Architecture Design Rationale) -- _setWheels()'s divisor above is
    // a fixed stand-in, not a live-read value, so there is nothing for
    // either call to actually change. Recorded into otherwise-unread
    // module variables so each has a real body: an empty `{}` body is
    // emitted by pxt as native-only, and no pxsim implementation exists
    // for either, so the simulator crashes at the call site.
    let simLastGeometryTrackWidth = 0
    let simLastGeometryCalib = 0
    let simLastKernelField = 0
    let simLastKernelValue = 0

    //% shim=diffDrive::setGeometry
    export function _setGeometry(trackWidth: number, calib: number): void {
        simLastGeometryTrackWidth = trackWidth
        simLastGeometryCalib = calib
    }

    //% shim=diffDrive::setKernelValue
    export function _setKernelValue(field: number, value: number): void {
        simLastKernelField = field
        simLastKernelValue = value
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
     * End-of-move shaping. Bigger tapers and lower floors trade time
     * for accuracy; a closed-loop caller that takes a fresh fix before
     * every move should spend far less time on it. 0 leaves unchanged.
     * Simulator: no taper/floor/ramp shaping model exists in the
     * browser, so these are no-ops -- real (if trivial) bodies below,
     * not bare `{}`, so pxt doesn't treat them as native-only shims
     * (no pxsim implementation exists for any of the three, so an
     * empty body would crash the simulator at the call site).
     */
    //% shim=diffDrive::setTaperWindows
    export function setTaperWindows(distCounts: number, yawCounts: number): void {
        return
    }

    //% shim=diffDrive::setTaperFloors
    export function setTaperFloors(distPct: number, turnPct: number): void {
        return
    }

    //% shim=diffDrive::setRampMs
    export function setRampMs(ms: number): void {
        return
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
}
