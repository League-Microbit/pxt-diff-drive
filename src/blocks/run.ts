namespace diffDrive {
    // Argument-snapshot STACK for the RUN command(s) currently being
    // dispatched: each frame is one command's split parts ([0] the
    // name, [1..] its arguments), and the top of the stack (the last
    // element) is whichever command is innermost on this fiber right
    // now.
    //
    // A stack, not a bare variable, because RUN dispatch nests: the
    // abort/clearestop bypass (src/comms/protocol.h) can dispatch a
    // second command reentrantly, on this SAME fiber, while an outer
    // handler's job is still mid-tick -- RUN handling runs nested on
    // the protocol fiber itself rather than a second, forked fiber per
    // command (see onRun()'s own doc comment below). wireRunDispatch()
    // pushes a new frame before invoking any handler for it and pops
    // that frame in a `finally` once every handler/any-handler has
    // run, so an outer handler's runArg()/runArgText()/runArgCount()
    // calls made AFTER a nested dispatch returns still see the OUTER
    // command's own arguments, not whatever the nested dispatch left
    // behind. Replaces a bare module-level `runParts` a nested
    // dispatch could silently overwrite out from under the handler
    // that was still running -- nothing exercised that today (every
    // handler in this package reads its arguments only at entry,
    // before any reentrancy point), but nothing enforced it either.
    //
    // Declared with NO INITIALISER, created on first use. This file's
    // namespace initialisers run AFTER a test file's top-level code, so
    // an initialiser here is doubly wrong: `runNames.push(...)` from a
    // top-level onRun() hits a null array and throws (on hardware a
    // SILENT boot death -- panic 980, unhandled exception, with no
    // serial output at all because the protocol fiber never gets
    // scheduled to print one), and any initialiser that DID run would
    // then wipe the handlers that were just registered. Measured both
    // ways on vevov 2026-08-21.
    let runPartsStack: string[][]
    let runNames: string[]
    let runHandlers: ((arg: number) => void)[]
    let runAnyHandlers: ((name: string, arg: number) => void)[]
    let runWired: boolean

    function ensureRunState(): void {
        if (!runPartsStack) runPartsStack = []
        if (!runNames) runNames = []
        if (!runHandlers) runHandlers = []
        if (!runAnyHandlers) runAnyHandlers = []
    }

    // The parts array of the innermost RUN command currently
    // dispatching on this fiber (the top of runPartsStack), or
    // `undefined` when nothing is dispatching at all -- e.g. a call
    // to runArg()/runArgText()/runArgCount() from outside any onRun()
    // handler.
    function currentRunParts(): string[] {
        if (!runPartsStack || runPartsStack.length == 0) return undefined
        return runPartsStack[runPartsStack.length - 1]
    }

    // ================= remote test trigger (RUN verb) =================

    // The dispatcher below is invoked DIRECTLY by the firmware's own
    // protocol fiber (via _registerRunDispatch(), sim.ts) once per
    // dequeued RUN command -- not raised as a MessageBus event for a
    // second, forked fiber to pick up. That is what makes an abort sent
    // while a job is mid-tour land immediately rather than waiting
    // behind it: the fiber that would otherwise be forked to run this
    // callback IS the same fiber servicing the wire, so there is no
    // second fiber to wait for. runCommandText() (no argument -- the
    // firmware tracks "whichever command is current" itself, unlike the
    // old event-value-as-slot-number scheme) reads the text back; the
    // by-name lookup and dispatch logic below is unchanged. The wire
    // therefore still reads as what it does -- RUN:pivot:180, not a
    // magic number -- and arguments still ride along as text.
    function wireRunDispatch(): void {
        if (runWired) return
        runWired = true
        _registerRunDispatch(function () {
            const text = runCommandText()
            if (text.length == 0) return
            ensureRunState()
            const parts = text.split(":")
            const name = parts[0]
            // Push this dispatch's own frame -- see runPartsStack's
            // declaration comment above -- and pop it in a `finally`
            // so a handler that throws still unwinds the stack
            // correctly, leaving whatever dispatch is next-outermost
            // (if any) with its own frame back on top.
            runPartsStack.push(parts)
            try {
                for (let i = 0; i < runNames.length; i++) {
                    if (runNames[i] == name) runHandlers[i](runArg(0))
                }
                for (let i = 0; i < runAnyHandlers.length; i++) {
                    runAnyHandlers[i](name, runArg(0))
                }
            } finally {
                runPartsStack.pop()
            }
        })
    }

    // Remote dispatch is not a move, so it gets its own group rather
    // than sharing Move's weight range. Group, subcategory and weight on
    // every block below are GENERATED from
    // reports/blocks-toolbox.csv by tools/blocks_toolbox.py
    // (`just blocks-apply`) -- edit the CSV, not the annotations.

    /**
     * Run code when the named command arrives over the wire protocol --
     * `RUN:<name>` or `RUN:<name>:<arg>`, e.g. RUN:pivot:180. Bind your
     * test functions to names so the bench host can trigger them
     * remotely, the same functions a button handler calls. The handler
     * receives the first argument as a number (0 when there is none);
     * further arguments are available from runArg(). Handlers run
     * NESTED, directly on the wire's own (protocol) fiber -- not
     * forked to a fiber of their own -- so anything that sleeps or
     * blocks inside a handler body (or anything it calls) stalls
     * PING/STATUS/ESTOP and every other wire command for as long as it
     * runs. Keep a long test (a full tour) alive by ticking your own
     * wait loop instead of sleeping (see test.ts's tickWait()); every
     * tour handler in this package already does this. Names are
     * matched exactly, so keep them lower case.
     * @param name the command name to answer to, eg: "tour"
     */
    //% block="on run %name $arg"
    //% draggableParameters="reporter"
    //% group="Remote" weight=40
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
    //% group="Remote" weight=30
    //% subcategory="Extra"
    export function onRunCommand(
        handler: (name: string, arg: number) => void): void {
        ensureRunState()
        wireRunDispatch()
        runAnyHandlers.push(handler)
    }

    /**
     * Turn on the v6 wire protocol over the radio, on this channel and
     * group, so a bench host or relay can drive the robot remotely.
     *
     * OFF until you call this. That is deliberate: while it is off, the
     * radio belongs to MakeCode's own `radio` blocks, so a joystick
     * controller works normally. Calling this takes the radio over --
     * `radio send`/`on radio received` STOP WORKING in the same program.
     * It cannot be undone without restarting the robot.
     *
     * Call it from `on start`, before anything else touches the radio.
     *
     * The channel must match the relay you are talking to; each robot in
     * the fleet has its own, and changing it will take the robot off the
     * relay it is assigned to. The group defaults to 10, the relay's
     * listen group.
     * @param channel radio channel, eg: 4
     * @param group radio group, eg: 10
     */
    //% block="setup radio channel %channel group %group"
    //% group="Setup" weight=90
    //% subcategory="Setup"
    export function setupRadio(channel: number, group: number = 10): void {
        _setupRadio(Math.round(channel), Math.round(group))
    }

    /**
     * Bring the v6 radio link up on the channel this firmware was built
     * for -- the per-robot value tools/make_deploy.py injects at deploy
     * time -- and group 10.
     *
     * For the on-robot test program and advanced JavaScript users. It is
     * deliberately NOT a block and takes no channel: naming a channel
     * here would override the deploy injection and put every robot on
     * one channel. Students use `setup radio` instead.
     */
    //% blockHidden=true
    export function enableRadioLink(): void {
        _enableRadioLink()
    }

    /**
     * Bring the v6 wire up over the Planet X WiFi module (Ai-WB2-12F on
     * RJ11 jack J1), joining the network whose credentials
     * tools/make_deploy.py baked into this build. The robot then answers
     * the same protocol on UDP port 7654 that it answers on USB and
     * radio, learns the host from the first datagram it receives, and
     * advertises itself over mDNS as `<name> robot link` on
     * `_robotlink._udp.local`.
     *
     * OFF until you call this, so a program with no module fitted pays
     * nothing. A build with no credentials baked leaves it off even when
     * called. For the on-robot test program and advanced JavaScript
     * users -- not a block.
     */
    //% blockHidden=true
    export function enableWifiLink(): void {
        _enableWifiLink()
    }

    /**
     * Send a line of text back to the computer, tagged as debug output.
     * It shows up in the console as `DBG:` followed by your text.
     *
     * Use `send value` instead for a number you want to graph -- the
     * `DBG:` tag stops the console graphing it.
     *
     * Always goes out over the USB cable. It also goes out over the
     * radio once `setup radio` has been called, which is how an
     * untethered robot reports back.
     * @param text the text to send, eg: "hello"
     */
    //% block="send string %text"
    //% group="Debug" weight=20
    //% subcategory="Extra"
    export function sendString(text: string): void {
        emitLine("DBG:" + text)
    }

    /**
     * Send a named number back to the computer, in the form
     * `name:value` -- the format the MakeCode console plots on its
     * graph. Send the same name repeatedly to draw a line.
     *
     * Same wires as `send string`: always USB, plus radio once
     * `setup radio` has been called.
     * @param name what to call the value, eg: "x"
     * @param value the number to send, eg: 0
     */
    //% block="send value %name = %value"
    //% group="Debug" weight=10
    //% subcategory="Extra"
    export function sendValue(name: string, value: number): void {
        emitLine(name + ":" + value)
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

    /**
     * Typo-safe sibling of runArg(): a three-way contract that keeps
     * "no such argument" distinct from "argument present but garbage",
     * which runArg() collapses into the same 0. Unlike runArg(), this
     * is opt-in -- every existing runArg() call site (pivot, face, arc,
     * straight, seedxy, turnrate, etc.) is UNCHANGED by this function's
     * existence.
     *
     *   - ABSENT   (runArgText(i) == "")        -> fallback
     *   - PRESENT, valid, > minExclusive         -> the parsed value
     *   - PRESENT but unparseable, OR <= minExclusive -> NaN
     *
     * NaN is the sentinel because it never collides with a real
     * argument value a caller might legitimately pass (RUN:circle:0 is
     * a valid, if degenerate, radius under the bare two-argument form
     * -- parseFloat("0") is 0, not NaN, so it is returned as-is).
     * NaN is NEVER the same as fallback and NEVER 0: a caller must
     * check `isNaN()` itself rather than treating the return value as
     * always-usable the way runArg()'s is.
     *
     * minExclusive is the opt-in "this argument is geometrically a
     * radius (or similar quantity)" guard the Solution text asks for
     * ("rejects NaN and non-positive radii"): passing 0 makes any
     * parsed value <= 0 count as invalid too, folded into the same NaN
     * sentinel rather than a second return channel. Left at its
     * default (NaN, meaning "no bound") the function is pure typo
     * detection with no range check at all. A parameterized bound
     * lives on this one function rather than a wrapper per call site
     * because every current caller wants the exact same "not a real
     * radius" rule -- a wrapper would just be `runArgOr(i, fb, 0)`
     * spelled out three times.
     * @param i argument index, 0 being the first after the name, eg: 0
     * @param fallback value to use when the argument is absent, eg: 30
     * @param minExclusive reject a parsed value at or below this, eg: 0
     */
    //% blockHidden=true
    export function runArgOr(i: number, fallback: number,
        minExclusive: number = NaN): number {
        const text = runArgText(i)
        if (text.length == 0) return fallback
        const value = parseFloat(text)
        if (isNaN(value)) return NaN
        if (!isNaN(minExclusive) && value <= minExclusive) return NaN
        return value
    }

    /** The i-th argument of the run command, as text ("" if absent). */
    //% blockHidden=true
    export function runArgText(i: number): string {
        const parts = currentRunParts()
        if (!parts || i < 0 || i + 1 >= parts.length) return ""
        return parts[i + 1]
    }

    /** How many arguments the run command being handled carries. */
    //% blockHidden=true
    export function runArgCount(): number {
        const parts = currentRunParts()
        if (!parts) return 0
        return parts.length - 1
    }
}
