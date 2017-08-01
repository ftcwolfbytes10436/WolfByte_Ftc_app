package org.firstinspires.ftc.teamcode.kotlinTestCode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor

class WheelPIDCalibrater: OpMode() {

    var robot = BaseMecanumHardware()
    var pidController = PIDController()

    var wheel: Int = 1
    var targetSpeed: Float? = 3000.0f
    var kp: Float? = 0.01f
    var maxError: Float = 0.0f
    var timeAtLastMaxError: Long = 0
    var oscillationPeriod: Float = 0.0f
    var variableNumber: Int = 1
    var variableIncrement: Float = 1.0f
    var lastTime: Long = 0
    var lastCount: Int = 0

    var dpadPressed: Boolean = false
    var leftStickPressed: Boolean = false
    var rightStickPressed: Boolean = false
    var letterButtonPressed: Boolean = false

    override fun init() {
        robot.init(hardwareMap, telemetry)

        pidController.SetMode(true)

        telemetry.addData("Status", "waiting to start")
        telemetry.update()
    }

    override fun loop() {
        var currentVariable: Float?
        var currentMotor: DcMotor?

        if ((gamepad1.dpad_left || gamepad1.dpad_right) && !dpadPressed) {
            if (gamepad1.dpad_right && variableNumber < 2) {
                variableNumber++
            } else if (gamepad1.dpad_left && variableNumber > 1) {
                variableNumber--
            }
        }
        dpadPressed = gamepad1.dpad_left || gamepad1.dpad_right

        currentVariable = when (variableNumber) {
            1 -> {telemetry.addData("Currently Selected Variable", "Target Speed"); targetSpeed}
            2 -> {telemetry.addData("Currently Selected Variable", "kp"); kp}
            else -> {telemetry.addData("Currently Selected Variable", "Target Speed"); targetSpeed}
        }

        if ((gamepad1.left_stick_y != 0.0f || gamepad1.left_stick_x != 0.0f)  && !leftStickPressed) {
            if (gamepad1.left_stick_y != 0.0f) {
                if (-gamepad1.left_stick_y > 0) {
                    variableIncrement *= 10
                } else {
                    variableIncrement /= 10
                }
            } else {
                if (gamepad1.left_stick_x > 0) {
                    currentVariable = currentVariable?.plus(variableIncrement)
                } else {
                    currentVariable = currentVariable?.minus(variableIncrement)
                }
            }
        }
        leftStickPressed = gamepad1.left_stick_y != 0.0f || gamepad1.left_stick_x != 0.0f

        telemetry.addData("Current Variable Increment", variableIncrement)
        telemetry.addData("Current Variable Value", currentVariable ?: 0)

        if (gamepad1.right_stick_x != 0.0f && !rightStickPressed) {
            if (gamepad1.right_stick_x > 0 && wheel < 4) {
                wheel++
            } else if (gamepad1.right_stick_x < 0 && wheel > 1) {
                wheel--
            }
            pidController = PIDController()
            maxError = 0.0f
        }
        rightStickPressed = gamepad1.right_stick_button

        currentMotor = when (wheel) {
            1 -> {telemetry.addData("Current Motor", "front left"); robot.frontLeftMotor}
            2 -> {telemetry.addData("Current Motor", "front right"); robot.frontRightMotor}
            3 -> {telemetry.addData("Current Motor", "back left"); robot.backLeftMotor}
            4 -> {telemetry.addData("Current Motor", "back right"); robot.backRightMotor}
            else -> {telemetry.addData("Current Motor", "front left");robot.frontLeftMotor}
        }

        pidController.SetTunings(kp ?: 0.0f,0.0f,0.0f)

        pidController.Setpoint = targetSpeed ?: 0.0f

        val currentSpeed: Float = ((currentMotor?.currentPosition ?: 0) - lastCount).toFloat()
        lastCount = currentMotor?.currentPosition ?: 0

        pidController.Input = currentSpeed

        pidController.Compute()

        currentMotor?.power = currentMotor?.power?.plus(pidController.Output) ?: 0.0

        var error =(pidController.Setpoint - pidController.Input)
        telemetry.addData("error", error)

        if (error > maxError){ maxError = error; timeAtLastMaxError = System.currentTimeMillis()}


    }
}