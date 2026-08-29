namespace diffDrive {
    // ================= world pose (OTOS) =============================
    // The OTOS optical tracking sensor is the WORLD-POSE AUTHORITY, and
    // it is consulted at MOVE BOUNDARIES ONLY (stakeholder doctrine,
    // 2026-08-20): a move runs entirely on encoder odometry; the sensor
    // says where the robot actually ended up, and the NEXT move is
    // planned from that fix. It never steers a move in flight.
    //
    // Every read here is a live I2C burst, so these must be called from
    // the same fiber that calls driveTick() -- never concurrently with
    // one (an OTOS transaction landing inside the Nezha encoder's
    // select->read window destroys that encoder sample).

    /**
     * Start the OTOS sensor. Returns true if it answered.
     * Call once at program start, robot held still.
     */
    //% block="start world tracking"
    //% group="World" weight=170
    //% subcategory="Pose"
    export function startWorldTracking(): boolean {
        otosBegin()
        return worldTrackingReady()
    }

    /**
     * Is the world sensor present and answering?
     */
    //% block="world tracking ready?"
    //% group="World" weight=160
    //% subcategory="Pose"
    export function worldTrackingReady(): boolean {
        return otosGet(7) != 0
    }

    /**
     * Declare where the robot is right now, in world coordinates.
     * Sets BOTH pose sources -- the world sensor and the wheel
     * odometry -- so they start agreed.
     * @param x world x, eg: 0
     * @param y world y, eg: 0
     * @param heading world heading in degrees CCW, eg: 0
     */
    //% block="set world pose to x %x cm y %y cm heading %heading deg"
    //% group="World" weight=180
    //% subcategory="Pose"
    export function seedPose(x: number, y: number,
        heading: number): void {
        _seedPose(Math.round(x * 10), Math.round(y * 10),
            Math.round(heading * 100))
    }

    /**
     * Take a fresh fix from the world sensor. Returns false if the
     * sensor did not answer; the last good values are kept.
     */
    //% block="read world position"
    //% group="World" weight=150
    //% subcategory="Pose"
    export function readWorld(): boolean {
        return otosRead()
    }

    /**
     * World x from the most recent fix, in cm.
     */
    //% block="world x (cm)"
    //% group="World" weight=130
    //% subcategory="Pose"
    export function worldX(): number {
        return otosGet(0) / 100
    }

    /**
     * World y from the most recent fix, in cm.
     */
    //% block="world y (cm)"
    //% group="World" weight=120
    //% subcategory="Pose"
    export function worldY(): number {
        return otosGet(1) / 100
    }

    /**
     * World heading from the most recent fix, in degrees CCW.
     */
    //% block="world heading (deg)"
    //% group="World" weight=140
    //% subcategory="Pose"
    export function worldHeading(): number {
        return otosGet(2) / 100
    }

    /**
     * Recalibrate the sensor's gyro bias. The robot must be parked and
     * completely still for about a second.
     */
    //% block="calibrate world sensor"
    //% group="World" weight=200
    //% subcategory="Pose"
    export function calibrateWorldSensor(): void {
        otosCalibrate(0)
        basic.pause(800)
    }

    /**
     * Where the sensor sits relative to the robot's centre of rotation
     * (x forward, y left, cm) and its own mounting rotation (degrees).
     * Measured by the lever-arm calibration, then set once at startup.
     */
    //% block="set world sensor offset x %x cm y %y cm yaw %yaw deg"
    //% group="World" weight=190
    //% subcategory="Pose"
    export function setWorldSensorOffset(x: number, y: number,
        yaw: number): void {
        otosSetOffset(Math.round(x * 100), Math.round(y * 100),
            Math.round(yaw * 100))
    }

    // ---- goToWorld ---------------------------------------------------
    // Drive to a world point, planning from OTOS fixes taken BETWEEN
    // moves. Shaped after the v6 GOTO verb (protocol-v6-spec.md 5.3:
    // target, speed, arrival tolerance, required timeout) and after the
    // reference firmware's navigator, with one deliberate difference
    // the stakeholder specified: NO mid-move retargeting. Each move is
    // planned from a fresh fix and then runs to completion on encoder
    // odometry; a small bearing error is absorbed by yawing WHILE
    // driving (the arc), not by steering corrections in flight.

    let arriveTolCm = 1.0        // [cm] v6 GOTO `arrive`
    // Pivot first beyond this bearing error. 12 deg, NOT the 50 this
    // started at: a capped arc physically cannot reach a target that is
    // meaningfully off-bearing -- it drives a bounded curve and lands
    // short, which left the tour stopping partway to three of four
    // corners. Pointing at the target first makes every drive nearly
    // straight, and that is what actually hit the dots (0.7-12.7 cm
    // when it was done host-side; this moves it onto the robot).
    // Anything under 12 deg is left to curve out over the leg.
    let turnFirstDeg = 12.0

    /**
     * How close counts as "arrived", in cm.
     * @param tol eg: 1
     */
    //% block="set arrival tolerance %tol cm"
    //% group="Setup" weight=60
    //% subcategory="Setup"
    export function setArrivalTolerance(tol: number): void {
        arriveTolCm = Math.max(0.1, tol)
    }

    /**
     * Drive to a point in WORLD coordinates, using the world sensor to
     * decide where the robot is before the leg. Turns in place first
     * only if the target is far off to the side; otherwise curves to it
     * in one arc. ONE PASS: drives the leg and stops, whether or not it
     * lands inside the arrival tolerance -- it does not loop or creep
     * up on the target. Any remaining error is DISCARDED by the next
     * call/hop, which re-measures and plans its own absolute target
     * from wherever the robot actually is.
     * @param x world x, eg: 60
     * @param y world y, eg: 0
     */
    //% block="go to world x %x cm y %y cm"
    //% group="GoTo" weight=360
    export function goToWorld(x: number, y: number): void {
        // ONE PASS. Drive at the point, get as close as the leg gets,
        // stop. No arrival nudging, no creeping up on it, no squaring
        // up -- iterating at the destination is what turned a tour into
        // 44 s of lurch-and-sit, and it buys accuracy the next hop
        // gets for free.
        //
        // This leg's MISS IS NOT CARRIED FORWARD. The next hop re-reads
        // the OTOS and plans its own absolute target from the pose it
        // measures, so per-waypoint position error stays bounded by one
        // leg's execution error instead of accumulating. What IS
        // carried forward is OTOS error -- the gap between what the
        // sensor believes and where the body physically is. Re-planning
        // cannot see that, because it consults the same drifting
        // sensor; only a camera fix can.
        //
        // Correcting in place fixes neither. It chases the OTOS's own
        // belief, costs time, and looks awful.
        //
        // The OTOS is read here, on the robot. That is not the camera:
        // the overhead camera is a diagnostic and never drives a leg.
        readWorld()
        const px = worldX()
        const py = worldY()
        let ph = worldHeading() * Math.PI / 180

        let dx = x - px
        let dy = y - py
        if (Math.sqrt(dx * dx + dy * dy) <= arriveTolCm) return

        let bx = Math.cos(ph) * dx + Math.sin(ph) * dy
        let by = -Math.sin(ph) * dx + Math.cos(ph) * dy
        const bearing = Math.atan2(by, bx)

        // A target well off the bow needs a pivot first -- an arc to a
        // point abeam is a semicircle that leaves the field. This is
        // the ONE re-measure in the pass, and only because the pivot
        // itself changes the geometry the drive is planned from. This
        // stays even though startGoTo()/goToR() below can reach any
        // bearing on its own: it is a SENSING decision, not a geometry
        // one -- re-planning (bx, by) from a fresh OTOS fix after the
        // pivot moves the robot -- and goToR() cannot do that for
        // itself, since it reads its pose once, by contract.
        if (Math.abs(bearing) >= turnFirstDeg * Math.PI / 180) {
            tickedMove(0, bearing * 180 / Math.PI)
            readWorld()
            ph = worldHeading() * Math.PI / 180
            dx = x - worldX()
            dy = y - worldY()
            bx = Math.cos(ph) * dx + Math.sin(ph) * dy
            by = -Math.sin(ph) * dx + Math.cos(ph) * dy
        }

        // Drive the residual leg through startGoTo() (motion.ts), which
        // calls goToR() directly: it owns its own pivot-vs-blend split
        // and short-arc wrap, so it reaches (bx, by) exactly for ANY
        // residual bearing. No cap is needed here any more -- a capped
        // arc could only curve toward the straight-line distance and
        // leave a large residual short of the target; goToR has nothing
        // left for a cap to protect against.
        tickedGoTo(bx, by)
    }

    // Shared runner for goToWorld's legs: start the move, then tick it
    // to completion on THIS fiber (the same fiber that reads the OTOS,
    // so the two can never interleave on the I2C bus).
    function tickedMove(distance: number, yaw: number): void {
        if (distance == 0 && yaw == 0) return
        startMove(distance, yaw)
        while (_tickDrive());
    }

    // goTo-shaped sibling of tickedMove, above: startGoTo() (motion.ts)
    // only ARMS the move (see its own doc comment) -- something must
    // still tick it, and that has to be THIS fiber, same OTOS-fiber
    // constraint as tickedMove.
    function tickedGoTo(x: number, y: number): void {
        startGoTo(x, y)
        while (_tickDrive());
    }
}
