// test.ts -- sample program for the DiffDrive extension.
//
// This is a MakeCode `testFiles` entry: it is compiled when you open or
// build THIS repository, and it is NOT included in projects that add
// the extension. Button A drives a square with the blocking `move`
// block; button B drives one leg with `while moving`, showing progress
// on the LEDs; A+B stops the robot.

basic.showIcon(IconNames.Happy)

input.onButtonPressed(Button.A, function () {
    diffDrive.resetPose()
    for (let i = 0; i < 4; i++) {
        diffDrive.move(30, 0)     // 30 cm straight
        diffDrive.move(0, 90)     // pivot 90 degrees counter-clockwise
    }
    basic.showNumber(Math.round(diffDrive.heading()))
})

input.onButtonPressed(Button.B, function () {
    diffDrive.whileMoving(30, 0, function (x, y, heading) {
        led.plotBarGraph(diffDrive.moveProgress() * 100, 100)
    })
    basic.clearScreen()
})

input.onButtonPressed(Button.AB, function () {
    diffDrive.stop()
    basic.showIcon(IconNames.No)
})
