package org.firstinspires.ftc.teamcode.kotlinTestCode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

/**
 * Created by caleb on 5/30/2017.
 * This is a basic teleop OpMode coded in kotlin.
 */
@TeleOp(name = "kotlin mecanum teleop", group = "kotlin")
class BasicTeleOp : OpMode() {

    /* Declare OpMode members. */
    var robot = BaseMecanumHardware()
    var fPS = FPS()

    var relativeMove = false
    var buttonPressed = false

    override fun init() {
        robot.init(hardwareMap, telemetry)
        fPS.init(hardwareMap, telemetry, robot)

        telemetry.addData("Status", "waiting to start")
        telemetry.update()
    }

    override fun loop() {
        // Run the wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot and the Right stick rotates it left and right
        var direction = Direction(gamepad1.left_stick_x as Double, -gamepad1.left_stick_y as Double)
        var rotation = gamepad1.right_stick_x as Double

        if (gamepad1.y && !buttonPressed) {
            buttonPressed = true
            relativeMove = !relativeMove
        }

        if (relativeMove)
            fPS.moveRobotRelative(direction, rotation)
        else
            robot.moveRobot(direction, rotation)
    }
}