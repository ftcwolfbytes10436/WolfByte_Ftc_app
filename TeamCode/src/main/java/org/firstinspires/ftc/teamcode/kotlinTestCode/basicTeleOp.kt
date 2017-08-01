package org.firstinspires.ftc.teamcode.kotlinTestCode

import com.cout970.vector.impl.Vector2d
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

    override fun init() {
        robot.init(hardwareMap, telemetry)

        telemetry.addData("Status", "waiting to start")
        telemetry.update()
    }

    override fun loop() {
        // Run the wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot and the Right stick rotates it left and right
        var direction = Vector2d(gamepad1.left_stick_x.toDouble(), -gamepad1.left_stick_y.toDouble())
        val rotation = gamepad1.right_stick_x.toDouble()

        robot.moveRobot(direction, rotation)
    }
}