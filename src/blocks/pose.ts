namespace diffDrive {
    // ================= pose ==========================================

    /**
     * Robot x position (forward from start) in cm.
     */
    //% block="pose x (cm)"
    //% group="Pose" weight=310
    //% subcategory="Pose"
    export function poseX(): number {
        return _poseX() / 10
    }

    /**
     * Robot y position (left from start) in cm.
     */
    //% block="pose y (cm)"
    //% group="Pose" weight=300
    //% subcategory="Pose"
    export function poseY(): number {
        return _poseY() / 10
    }

    /**
     * Robot heading in degrees, CCW positive.
     */
    //% block="heading (deg)"
    //% group="Pose" weight=320
    //% subcategory="Pose"
    export function heading(): number {
        return _poseHeading() / 100
    }

    /**
     * Reset the pose to (0, 0, 0).
     */
    //% block="reset pose"
    //% group="Pose" weight=290
    //% subcategory="Pose"
    export function resetPose(): void {
        _resetPose()
    }
}
