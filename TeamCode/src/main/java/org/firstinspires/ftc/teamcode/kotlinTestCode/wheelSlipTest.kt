package org.firstinspires.ftc.teamcode.kotlinTestCode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

/**
 * Created by caleb on 6/5/2017.
 * This is an opmode to test how the mecanum wheels react to individually changing them
 * the first controller controls the front two wheels and the second controller controls the back two wheels
 */

@TeleOp(name = "wheel slip test", group = "kotlin")
class wheelSlipTest: OpMode() {

    /* Declare OpMode members. */
    var robot = BaseMecanumHardware()

    override fun init() {
        robot.init(hardwareMap, telemetry)

        telemetry.addData("Status", "waiting to start")
        telemetry.update()
    }

    override fun loop() {
        robot.frontLeftMotor?.power  = gamepad1.left_stick_y.toDouble()
        robot.frontRightMotor?.power = gamepad1.right_stick_y.toDouble()
        robot.backLeftMotor?.power   = gamepad2.left_stick_y.toDouble()
        robot.backRightMotor?.power  = gamepad2.right_stick_y.toDouble()
    }
}