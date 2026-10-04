namespace diffDrive {
    let programNames: string[]
    let programPictures: Image[]
    let programRuns: (() => void)[]
    let menuIndex: number        // -1 until A picks a program
    let menuRequest: number      // the program B asked for, -1 when none
    let programBusy: boolean
    let programCancel: boolean
    let robotSetUp: boolean

    function beginProgram(): void {
        programBusy = true
        programCancel = false
    }

    function endProgram(): void {
        programBusy = false
        programCancel = false
    }

    // Stops the wheels now and flags the program so it does not start
    // another move. False when nothing was running.
    function cancelProgram(): boolean {
        if (!programBusy) return false
        programCancel = true
        stop()
        return true
    }

    // Programs run on a fiber of their own so the button handlers stay
    // free to cancel them.
    function startMenu(): void {
        menuIndex = -1
        menuRequest = -1
        input.onButtonPressed(Button.A, function () {
            if (cancelProgram()) return
            menuIndex = (menuIndex + 1) % programRuns.length
            programPictures[menuIndex].showImage(0, 0)
            emitLine("menu " + programNames[menuIndex])
        })
        input.onButtonPressed(Button.B, function () {
            if (cancelProgram()) return
            if (menuIndex < 0) return
            beginProgram()
            menuRequest = menuIndex
        })
        input.onButtonPressed(Button.AB, function () {
            cancelProgram()
        })
        control.inBackground(function () {
            while (true) {
                if (menuRequest >= 0) {
                    const i = menuRequest
                    menuRequest = -1
                    emitLine("run " + programNames[i])
                    programRuns[i]()
                    endProgram()
                }
                basic.pause(20)
            }
        })
    }

    /**
     * Add a program to the button menu and bind it to the run command
     * of the same name. A steps through the programs, B runs the one
     * shown, and any button stops a running program. Takes over the
     * A, B and A+B button handlers.
     * @param name the run command name, eg: "square"
     * @param picture what the menu shows for this program
     * @param run the program; test cancelled() between moves
     */
    //% blockHidden=true
    export function addProgram(name: string, picture: Image,
        run: () => void): void {
        if (!programNames) {
            programNames = []
            programPictures = []
            programRuns = []
            startMenu()
        }
        programNames.push(name)
        programPictures.push(picture)
        programRuns.push(run)
        onRun(name, function (arg: number) {
            beginProgram()
            run()
            endProgram()
        })
    }

    /** True while a program added with addProgram() is running. */
    //% blockHidden=true
    export function programRunning(): boolean {
        return !!programBusy
    }

    /** True once a button has stopped the running program. */
    //% blockHidden=true
    export function cancelled(): boolean {
        return !!programCancel
    }

    /**
     * move(), unless the program has been cancelled. False when the
     * program should stop: `if (!diffDrive.moveLeg(50, 0)) return`.
     * @param distance distance to travel, eg: 20
     * @param yaw angle to turn CCW+, eg: 0
     */
    //% blockHidden=true
    export function moveLeg(distance: number, yaw: number): boolean {
        if (cancelled()) return false
        move(distance, yaw)
        return !cancelled()
    }

    /**
     * Report that a button stopped the program and show an X.
     * @param name the program's name, eg: "square"
     */
    //% blockHidden=true
    export function reportStopped(name: string): void {
        stop()
        report(name + ".fail").str("why", "stopped by a button press").send()
        flushReports()
        basic.showIcon(IconNames.No)
    }

    // The WiFi credential source is latched at boot, so a reset is the
    // only way to pick up a newly stored credential.
    function reboot(): void {
        if (programRunning()) {
            report("reboot.fail").str("why",
                "a program is running -- press a button or send STOP first").send()
            flushReports()
            basic.showIcon(IconNames.No)
            return
        }
        report("reboot.ok").str("why", "asked").send()
        flushReports()
        stop()
        basic.pause(120)
        control.reset()
    }

    // The stored WiFi address as one number, a*2^24 + b*2^16 + c*2^8 + d.
    // A `settings` system key, like the calibration keys.
    const wifiAddressKey = "#net.ip"

    function storedWifiAddress(): number[] {
        if (!settings.exists(wifiAddressKey)) return []
        const packed = settings.readNumber(wifiAddressKey)
        if (!(packed > 0)) return []
        return [
            Math.floor(packed / 16777216) % 256,
            Math.floor(packed / 65536) % 256,
            Math.floor(packed / 256) % 256,
            packed % 256,
        ]
    }

    function dotted(address: number[]): string {
        return address.length == 4
            ? address[0] + "." + address[1] + "." + address[2] + "." + address[3]
            : "-"
    }

    /** Report the WiFi address this robot uses and where it comes from. */
    function reportWifiAddress(): void {
        const stored = storedWifiAddress()
        const byName = wifiAddressForName(control.deviceName())
        report("netstore.values")
            .str("ip", dotted(stored.length == 4 ? stored : byName))
            .num("stored", stored.length == 4 ? 1 : 0, 0)
            .str("default", dotted(byName))
            .str("gateway", "10.55.255.254")
            .str("netmask", "255.255.0.0")
            .send()
        flushReports()
    }

    // A stored address takes effect at the next reset, like a stored
    // WiFi credential.
    function saveWifiAddress(a: number, b: number, c: number, d: number): void {
        const parts = [a, b, c, d]
        for (const part of parts) {
            if (!(part >= 0 && part <= 255) || part != Math.floor(part)) {
                report("netstore.fail").str("why", "each part must be a whole number from 0 to 255").send()
                flushReports()
                return
            }
        }
        if (a == 0) {
            report("netstore.fail").str("why", "the first part cannot be 0").send()
            flushReports()
            return
        }
        settings.writeNumber(wifiAddressKey, a * 16777216 + b * 65536 + c * 256 + d)
        reportWifiAddress()
    }

    /**
     * Bring a fleet robot up: the radio link on the address this
     * micro:bit's name derives, WiFi from stored credentials, the
     * calibration stored in flash, and the console's own run commands
     * (`_calshow`, `_calclear`, `_calsave`, `_calscale`, `_calports`,
     * `_netshow`, `_netset`, `_netclear`, `_reboot`). The leading `_`
     * marks a command the console drives itself; the console keeps
     * those out of the student's function list. Call it once, after any setTrackWidth() or
     * setWheelCalibration() the stored calibration should override.
     *
     * Takes the radio over: MakeCode's own radio blocks stop working in
     * this program.
     */
    //% block="setup robot"
    //% group="Robot" weight=180
    //% subcategory="Setup"
    export function setupRobot(): void {
        if (robotSetUp) return
        robotSetUp = true
        const name = control.deviceName()
        const address = radioAddressForName(name)
        if (address.length == 2) setupRadio(address[0], address[1])
        const net = storedWifiAddress()
        if (net.length == 4) setWifiAddress(net[0], net[1], net[2], net[3])
        enableStoredWifiLink()
        applyStoredCalibration()

        onRun("_calshow", function (arg: number) { reportStoredCalibration() })
        runSignature("_calshow", "()")
        onRun("_calclear", function (arg: number) { clearStoredCalibration() })
        runSignature("_calclear", "()")
        onRun("_calsave", function (arg: number) {
            saveCalibration(runArg(0), runArg(1), runArg(2))
            reportStoredCalibration()
        })
        runSignature("_calsave", "(wheel:number=0, track:number=0, slip:number=0)")
        onRun("_calscale", function (arg: number) {
            saveWheelMultipliers(runArg(0), runArg(1))
            reportStoredCalibration()
        })
        runSignature("_calscale", "(left:number=1, right:number=1)")
        onRun("_calports", function (arg: number) {
            saveMotorPorts(runArg(0), runArg(1))
            reportStoredCalibration()
        })
        runSignature("_calports", "(left:number=1, right:number=2)")
        onRun("_netshow", function (arg: number) { reportWifiAddress() })
        runSignature("_netshow", "()")
        onRun("_netset", function (arg: number) {
            saveWifiAddress(runArg(0), runArg(1), runArg(2), runArg(3))
        })
        runSignature("_netset", "(a:number=10, b:number=55, c:number=0, d:number=0)")
        onRun("_netclear", function (arg: number) {
            settings.remove(wifiAddressKey)
            reportWifiAddress()
        })
        runSignature("_netclear", "()")
        onRun("_reboot", function (arg: number) { reboot() })
        runSignature("_reboot", "()")

        emitLine(address.length == 2
            ? "boot radio " + name + " ch " + address[0] + " grp " + address[1]
            : "boot radio off: " + name + " has no derived address")
        emitLine(storedCalibrationLine())
        emitLine("boot net " + dotted(net.length == 4 ? net : wifiAddressForName(name))
            + (net.length == 4 ? " stored" : " default"))
    }
}
