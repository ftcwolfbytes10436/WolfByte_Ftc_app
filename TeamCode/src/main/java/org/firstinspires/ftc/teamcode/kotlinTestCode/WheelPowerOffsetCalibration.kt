package org.firstinspires.ftc.teamcode.kotlinTestCode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry

/**
 * Created by caleb on 6/20/2017.
 * an opmode to write a file to calibrate the wheels
 */
@TeleOp(name = "wheel slip test", group = "kotlin")
class WheelPowerOffsetCalibration: LinearOpMode(){

    var robot = BaseMecanumHardware()

    override fun runOpMode() {

        robot.init(hardwareMap, telemetry)

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()

        val fl = MotorOffset(0.0)
        val fr = MotorOffset(0.0)
        val bl = MotorOffset(0.0)
        val br = MotorOffset(0.0)

        robot.moveRobot(0.0, 0.1)

        var currentPower = 0.1
        var buttonPressed = false

        while (!gamepad1.y && opModeIsActive()) {

            if (gamepad1.right_bumper && !buttonPressed && currentPower < 1){
                buttonPressed = true
                currentPower += 0.1
            }

            if (gamepad1.left_bumper && !buttonPressed && currentPower > 0.1) {
                buttonPressed = true
                currentPower -= 0.1
            }

            if (gamepad1.atRest()) {
                buttonPressed = false
            }

            robot.moveRobot(0.0, currentPower)

            telemetry.addData("current Power", currentPower)
            addValuesToTelementry(telemetry, "fl", fl)
            addValuesToTelementry(telemetry, "fr", fr)
            addValuesToTelementry(telemetry, "bl", bl)
            addValuesToTelementry(telemetry, "br", br)
            telemetry.update()
        }
    }

    private fun addValuesToTelementry (telemetry: Telemetry, motorName: String, motorOffset: MotorOffset) {
        telemetry.addData("$motorName encoder speed", "" + motorOffset.encoderSpeed + " counts per second")
        telemetry.addData("$motorName average encoder speed", "" + motorOffset.averageEncoderSpeed + " counts per second")
    }
}

private data class MotorOffset(val currentPower: Double) {
    var encoderSpeed = 0; set(value) {numberOfCounts++; totalEncoderCounts+= value; field = value}
    val averageEncoderSpeed get() = totalEncoderCounts / numberOfCounts
    var numberOfCounts = 0
    var totalEncoderCounts = 0
}