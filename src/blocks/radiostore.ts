namespace diffDrive {
    const radioChannelKey = "#radio.ch"
    const radioGroupKey = "#radio.grp"
    const radioChannelMax = 83
    const radioGroupMax = 255

    function isWholeNumberInRange(value: number, max: number): boolean {
        return value >= 0 && value <= max && value == Math.floor(value)
    }

    function isWholeDecimal(text: string): boolean {
        const digits = text.charAt(0) == "-" ? text.substr(1) : text
        if (digits.length == 0) return false
        for (let i = 0; i < digits.length; i++) {
            const c = digits.charCodeAt(i)
            if (c < 48 || c > 57) return false
        }
        return true
    }

    /** Why a _radioset cannot be stored: "args", "range", or "" if it can. */
    //% blockHidden=true
    export function radioSetError(count: number, channelText: string, groupText: string): string {
        if (count != 2 || !isWholeDecimal(channelText) || !isWholeDecimal(groupText)) return "args"
        return radioPairInRange(parseInt(channelText), parseInt(groupText)) ? "" : "range"
    }

    //% blockHidden=true
    export function radioPairInRange(channel: number, group: number): boolean {
        return isWholeNumberInRange(channel, radioChannelMax)
            && isWholeNumberInRange(group, radioGroupMax)
    }

    /** Where setupRobot() takes the radio address from this boot. */
    //% blockHidden=true
    export function radioSourceFor(programSet: boolean, stored: boolean, derivable: boolean): string {
        if (programSet) return "program"
        if (stored) return "stored"
        return derivable ? "name" : "off"
    }

    /** The [channel, group] kept in flash, or [] if none is. */
    //% blockHidden=true
    export function storedRadioAddress(): number[] {
        if (!settings.exists(radioChannelKey) || !settings.exists(radioGroupKey)) return []
        const channel = settings.readNumber(radioChannelKey)
        const group = settings.readNumber(radioGroupKey)
        return radioPairInRange(channel, group) ? [channel, group] : []
    }

    //% blockHidden=true
    export function writeRadioAddress(channel: number, group: number): void {
        settings.writeNumber(radioChannelKey, channel)
        settings.writeNumber(radioGroupKey, group)
    }

    //% blockHidden=true
    export function removeRadioAddress(): void {
        settings.remove(radioChannelKey)
        settings.remove(radioGroupKey)
    }
}
