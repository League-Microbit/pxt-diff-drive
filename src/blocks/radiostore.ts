namespace diffDrive {
    const radioChannelKey = "#radio.ch"
    const radioGroupKey = "#radio.grp"
    const radioChannelMax = 83
    const radioGroupMax = 255

    function isWholeNumberInRange(value: number, max: number): boolean {
        return value >= 0 && value <= max && value == Math.floor(value)
    }

    /** Why a channel and group cannot be stored, or "" if they can. */
    //% blockHidden=true
    export function radioPairError(channel: number, group: number): string {
        if (!isWholeNumberInRange(channel, radioChannelMax))
            return "channel must be a whole number from 0 to " + radioChannelMax
        if (!isWholeNumberInRange(group, radioGroupMax))
            return "group must be a whole number from 0 to " + radioGroupMax
        return ""
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
        return radioPairError(channel, group) == "" ? [channel, group] : []
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
