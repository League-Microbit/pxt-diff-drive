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

    /**
     * Bring a fleet robot up: the radio link on the address this
     * micro:bit's name derives, WiFi from stored credentials, the
     * calibration stored in flash, and the `calshow`, `calclear` and
     * `reboot` run commands. Call it once, after any setTrackWidth() or
     * setWheelCalibration() the stored calibration should override.
     *
     * Takes the radio over: MakeCode's own radio blocks stop working in
     * this program.
     */
    //% blockHidden=true
    export function setupRobot(): void {
        if (robotSetUp) return
        robotSetUp = true
        const name = control.deviceName()
        const address = radioAddressForName(name)
        if (address.length == 2) setupRadio(address[0], address[1])
        enableStoredWifiLink()
        applyStoredCalibration()

        onRun("calshow", function (arg: number) { reportStoredCalibration() })
        runSignature("calshow", "()")
        onRun("calclear", function (arg: number) { clearStoredCalibration() })
        runSignature("calclear", "()")
        onRun("reboot", function (arg: number) { reboot() })
        runSignature("reboot", "()")

        emitLine(address.length == 2
            ? "boot radio " + name + " ch " + address[0] + " grp " + address[1]
            : "boot radio off: " + name + " has no derived address")
        emitLine(storedCalibrationLine())
    }
}
