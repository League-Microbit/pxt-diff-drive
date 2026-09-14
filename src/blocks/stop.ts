namespace diffDrive {
    // ================= stopping ======================================

    /**
     * Stop driving (normal stop).
     */
    //% block="stop"
    //% group="Stop" weight=270
    export function stop(): void {
        _stopAll()
    }

    /**
     * Emergency stop: latch off until clearEmergencyStop().
     */
    //% block="emergency stop"
    //% group="Stop" weight=280
    export function emergencyStop(): void {
        _estopAll()
    }

    /**
     * Clear the emergency-stop latch.
     */
    //% block="clear emergency stop"
    //% group="Stop" weight=260
    export function clearEmergencyStop(): void {
        _estopClear()
    }

    /**
     * Whether a move stalled: the robot demanded motion for too long
     * with the wheels not turning, so that move was stopped. It stays
     * true until a later move gets the wheels turning again (or clear
     * stall latch). A stall only stops the move it happened in -- the
     * next Drive/Move block tries again on its own. Separate from the
     * emergency-stop latch. Always false in the simulator: there is no
     * stall model in the browser.
     */
    //% block="is stalled"
    //% group="Moving?" weight=310
    export function isStalled(): boolean {
        return _isStalled()
    }

    /**
     * Forget a stall, so is stalled reads false. Not needed to move
     * again: every new Drive/Move block re-arms stall detection by
     * itself. Does NOT clear the emergency-stop latch -- the two are
     * independent fault states (see clearEmergencyStop()). A no-op if
     * nothing stalled.
     */
    //% block="clear stall latch"
    //% group="Stop" weight=250
    export function clearStallLatch(): void {
        _clearStallLatch()
    }
}
