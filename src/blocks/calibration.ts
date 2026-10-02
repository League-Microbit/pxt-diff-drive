namespace diffDrive {
    // Flash keys. Each calibration also keeps count, sum, low and high
    // of every run saved, under its stats prefix.
    const wheelKey = "cal.whl"   // [mm/deg]
    const trackKey = "cal.trk"   // [cm]
    const slipKey = "cal.slp"
    const wheelStats = "cw"
    const turnStats = "ct"

    // 0 means "not stored": no calibration value is ever 0.
    function stored(key: string): number {
        if (!settings.exists(key)) return 0
        const value = settings.readNumber(key)
        return value > 0 ? value : 0
    }

    function addRun(prefix: string, value: number): void {
        const runs = stored(prefix + ".n")
        const low = stored(prefix + ".l")
        const high = stored(prefix + ".h")
        settings.writeNumber(prefix + ".n", runs + 1)
        settings.writeNumber(prefix + ".s", stored(prefix + ".s") + value)
        settings.writeNumber(prefix + ".l", runs < 1 || value < low ? value : low)
        settings.writeNumber(prefix + ".h", runs < 1 || value > high ? value : high)
    }

    function meanOf(prefix: string): number {
        const runs = stored(prefix + ".n")
        return runs > 0 ? stored(prefix + ".s") / runs : 0
    }

    function spreadOf(prefix: string): number {  // [%] of the mean
        const mean = meanOf(prefix)
        if (mean <= 0 || stored(prefix + ".n") < 2) return 0
        return (stored(prefix + ".h") - stored(prefix + ".l")) * 100 / mean
    }

    function clearRuns(prefix: string): void {
        settings.remove(prefix + ".n")
        settings.remove(prefix + ".s")
        settings.remove(prefix + ".l")
        settings.remove(prefix + ".h")
    }

    function hasWheel(): boolean {
        return stored(wheelKey) > 0
    }

    function hasTurn(): boolean {
        return stored(trackKey) > 0 && stored(slipKey) > 0
    }

    /**
     * Use a measured wheel calibration and keep it in flash.
     * @param calib wheel travel per shaft degree in mm/deg, eg: 0.7878
     */
    //% blockHidden=true
    export function saveWheelCalibration(calib: number): void {
        setWheelCalibration(calib)
        settings.writeNumber(wheelKey, calib)
        addRun(wheelStats, calib)
    }

    /**
     * Use a measured turn calibration and keep it in flash.
     * @param width track width in cm, eg: 11.4
     * @param slip rotational slip, eg: 0.95
     */
    //% blockHidden=true
    export function saveTurnCalibration(width: number, slip: number): void {
        setTrackWidth(width)
        setConfigValue(ConfigField.RotationalSlip, slip)
        settings.writeNumber(trackKey, width)
        settings.writeNumber(slipKey, slip)
        addRun(turnStats, slip)
    }

    /** Apply whatever calibration is stored in flash; none stored changes nothing. */
    //% blockHidden=true
    export function applyStoredCalibration(): void {
        if (hasWheel()) setWheelCalibration(stored(wheelKey))
        if (hasTurn()) {
            setTrackWidth(stored(trackKey))
            setConfigValue(ConfigField.RotationalSlip, stored(slipKey))
        }
    }

    /** The stored calibration as one `boot cal` line. */
    //% blockHidden=true
    export function storedCalibrationLine(): string {
        if (!hasWheel() && !hasTurn()) return "boot cal none stored"
        return "boot cal"
            + " wheel=" + (hasWheel() ? "" + roundTo(stored(wheelKey), 4) : "-")
            + " tw=" + (hasTurn() ? "" + roundTo(stored(trackKey), 2) : "-")
            + " slip=" + (hasTurn() ? "" + roundTo(stored(slipKey), 4) : "-")
            + " runs=" + stored(wheelStats + ".n") + "/" + stored(turnStats + ".n")
    }

    /** Report the stored calibration and its run statistics. */
    //% blockHidden=true
    export function reportStoredCalibration(): void {
        report("calstore.values")
            .num("wheel", stored(wheelKey), 4)
            .num("tw", stored(trackKey), 2)
            .num("slip", stored(slipKey), 4)
            .num("has_wheel", hasWheel() ? 1 : 0, 0)
            .num("has_turn", hasTurn() ? 1 : 0, 0)
            .num("live_tw", trackWidth(), 2)
            .num("live_slip", rotationalSlip(), 4)
            .send()
        report("calstore.runs")
            .num("wheel_runs", stored(wheelStats + ".n"), 0)
            .num("wheel_mean", meanOf(wheelStats), 4)
            .num("wheel_lo", stored(wheelStats + ".l"), 4)
            .num("wheel_hi", stored(wheelStats + ".h"), 4)
            .num("wheel_spread", spreadOf(wheelStats), 2)
            .num("turn_runs", stored(turnStats + ".n"), 0)
            .num("turn_mean", meanOf(turnStats), 4)
            .num("turn_lo", stored(turnStats + ".l"), 4)
            .num("turn_hi", stored(turnStats + ".h"), 4)
            .num("turn_spread", spreadOf(turnStats), 2)
            .send()
        flushReports()
    }

    /** Forget the stored calibration. The running geometry is unchanged. */
    //% blockHidden=true
    export function clearStoredCalibration(): void {
        settings.remove(wheelKey)
        settings.remove(trackKey)
        settings.remove(slipKey)
        clearRuns(wheelStats)
        clearRuns(turnStats)
        report("calstore.cleared").str("why", "asked").send()
        flushReports()
    }
}
