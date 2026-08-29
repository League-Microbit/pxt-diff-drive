namespace diffDrive {
    // Parts of the RUN command currently being dispatched: [0] is the
    // name, [1..] its arguments. Safe as shared state because
    // MessageBus delivers these events one at a time, each after the
    // previous handler returns.
    // Declared with NO INITIALISER, created on first use. This file's
    // namespace initialisers run AFTER a test file's top-level code, so
    // an initialiser here is doubly wrong: `runNames.push(...)` from a
    // top-level onRun() hits a null array and throws (on hardware a
    // SILENT boot death -- panic 980, unhandled exception, with no
    // serial output at all because the protocol fiber never gets
    // scheduled to print one), and any initialiser that DID run would
    // then wipe the handlers that were just registered. Measured both
    // ways on vevov 2026-08-21.
    let runParts: string[]
    let runNames: string[]
    let runHandlers: ((arg: number) => void)[]
    let runAnyHandlers: ((name: string, arg: number) => void)[]
    let runWired: boolean

    function ensureRunState(): void {
        if (!runParts) runParts = []
        if (!runNames) runNames = []
        if (!runHandlers) runHandlers = []
        if (!runAnyHandlers) runAnyHandlers = []
    }

    // ================= remote test trigger (RUN verb) =================

    // MessageBus source id for the wire protocol's RUN verb -- must
    // match kRunEventSource in protocol.cpp. An event value cannot
    // carry text, so the C++ handler parks the command's payload in a
    // slot and sends the SLOT as the event value; the dispatcher below
    // reads the text back through runCommandText() and routes it by
    // NAME. The wire therefore reads as what it does -- RUN:pivot:180,
    // not RUN:4 -- and arguments ride along as text instead of being
    // encoded into numeric offsets.
    const RUN_EVENT_SOURCE = 0x2001


    function wireRunDispatch(): void {
        if (runWired) return
        runWired = true
        control.onEvent(RUN_EVENT_SOURCE, 0, function () {
            const text = runCommandText(control.eventValue())
            if (text.length == 0) return
            ensureRunState()
            runParts = text.split(":")
            const name = runParts[0]
            for (let i = 0; i < runNames.length; i++) {
                if (runNames[i] == name) runHandlers[i](runArg(0))
            }
            for (let i = 0; i < runAnyHandlers.length; i++) {
                runAnyHandlers[i](name, runArg(0))
            }
        })
    }

    // Remote group: remote dispatch is not a move, so it gets its own
    // group rather than sharing Move's weight range. Weights on this
    // group's three blocks (onRun, onRunCommand, and setRadioGroup
    // below) are spaced 10 apart so a future fourth block can slot in
    // without renumbering the existing three.

    /**
     * Run code when the named command arrives over the wire protocol --
     * `RUN:<name>` or `RUN:<name>:<arg>`, e.g. RUN:pivot:180. Bind your
     * test functions to names so the bench host can trigger them
     * remotely, the same functions a button handler calls. The handler
     * receives the first argument as a number (0 when there is none);
     * further arguments are available from runArg(). Handlers run on
     * their own fiber, so a long test (a full tour) doesn't block the
     * protocol. Names are matched exactly, so keep them lower case.
     * @param name the command name to answer to, eg: "tour"
     */
    //% block="on run %name $arg"
    //% draggableParameters="reporter"
    //% group="Remote" weight=20
    //% subcategory="Extra"
    export function onRun(name: string, handler: (arg: number) => void): void {
        ensureRunState()
        wireRunDispatch()
        runNames.push(name)
        runHandlers.push(handler)
    }

    /**
     * Run code when ANY run command arrives, name-bound or not. Runs
     * after every matching onRun() handler, so it can log or reject
     * unknown names.
     */
    //% block="on run command $name $arg"
    //% draggableParameters="reporter"
    //% group="Remote" weight=10
    //% subcategory="Extra"
    export function onRunCommand(
        handler: (name: string, arg: number) => void): void {
        ensureRunState()
        wireRunDispatch()
        runAnyHandlers.push(handler)
    }

    /**
     * The robot listens for RUN commands from the radio relay on this
     * group. Safe to call any time -- before the radio has come up
     * (typically from on start) or after: applied immediately if the
     * radio is already up, picked up automatically the first time it
     * comes up otherwise. Does not affect the fleet radio channel,
     * which stays fixed per-robot.
     * @param group radio group, eg: 10
     */
    //% block="set radio group %group"
    //% group="Setup" weight=70
    //% subcategory="Setup"
    export function setRadioGroup(group: number = 10): void {
        _setRadioGroup(Math.round(group))
    }

    /**
     * The i-th argument of the run command being handled, as a number.
     * 0 when there is no such argument, or it isn't a number.
     * @param i argument index, 0 being the first after the name, eg: 0
     */
    //% blockHidden=true
    export function runArg(i: number): number {
        const text = runArgText(i)
        if (text.length == 0) return 0
        const value = parseFloat(text)
        return isNaN(value) ? 0 : value
    }

    /** The i-th argument of the run command, as text ("" if absent). */
    //% blockHidden=true
    export function runArgText(i: number): string {
        if (!runParts || i < 0 || i + 1 >= runParts.length) return ""
        return runParts[i + 1]
    }

    /** How many arguments the run command being handled carries. */
    //% blockHidden=true
    export function runArgCount(): number {
        if (!runParts) return 0
        return runParts.length - 1
    }
}
