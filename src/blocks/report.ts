namespace diffDrive {
    let reportBuffer: string

    /** Round to a fixed number of decimal places. */
    //% blockHidden=true
    export function roundTo(value: number, places: number): number {
        const scale = Math.pow(10, places)
        return Math.round(value * scale) / scale
    }

    /**
     * One JSON object for the wire, built a field at a time:
     * `diffDrive.report("cal.result").num("b", 11.4, 3).send()`.
     * String values are not escaped.
     */
    export class Report {
        private text: string

        constructor(name: string) {
            this.text = "{\"ev\":\"" + name + "\""
        }

        num(key: string, value: number, places: number): Report {
            this.text += ",\"" + key + "\":" + roundTo(value, places)
            return this
        }

        str(key: string, value: string): Report {
            this.text += ",\"" + key + "\":\"" + value + "\""
            return this
        }

        /** Queue the report; flushReports() puts it on the wire. */
        send(): void {
            queueReport(this.text + "}")
        }
    }

    /**
     * Start a report named `name`.
     * @param name the event name, eg: "cal.result"
     */
    //% blockHidden=true
    export function report(name: string): Report {
        return new Report(name)
    }

    // A line emitted during a move is dropped when the link's outbound
    // ring is full, so reports are packed several to a frame.
    function queueReport(line: string): void {
        const frameLimit = 200  // [bytes] the transports clip at 240
        if (!reportBuffer) {
            reportBuffer = line
        } else if (reportBuffer.length + 1 + line.length > frameLimit) {
            flushReports()
            reportBuffer = line
        } else {
            reportBuffer = reportBuffer + "\n" + line
        }
    }

    /** Send every queued report. Call it once the last one is queued. */
    //% blockHidden=true
    export function flushReports(): void {
        if (!reportBuffer) return
        emitLine(reportBuffer)
        reportBuffer = ""
    }
}
