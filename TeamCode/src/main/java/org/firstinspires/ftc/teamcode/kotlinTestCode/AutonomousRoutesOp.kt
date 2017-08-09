package org.firstinspires.ftc.teamcode.kotlinTestCode

import com.cout970.vector.impl.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry

@Autonomous(name = "Auto Routes", group = "auto tests")
class AutonomousRoutesOp:  LinearOpMode(){

    var robot: AutonomousHardware = AutonomousHardware()
    var pathTelementry: Telemetry.Item? = null
    var targetTelementry: Telemetry.Item? = null

    override fun runOpMode() {

        robot.init(hardwareMap, telemetry)

        var selectedOption = 1
        var pressed = false

        while (!gamepad1.a && !isStopRequested) {
            if (!pressed && gamepad1.dpad_up && selectedOption < 3) {
                selectedOption++
            } else if (!pressed && gamepad1.dpad_down && selectedOption > 1) {
                selectedOption--
            }
            pressed = gamepad1.dpad_up || gamepad1.dpad_down

            telemetry.addData("Route", selectedOption)
            telemetry.update()
        }

        telemetry.addData("status", "ready to start")
        telemetry.update()

        waitForStart()

        telemetry.isAutoClear = false

        telemetry.addData("status", "running");

        pathTelementry = telemetry.addData("path","init")
        targetTelementry = telemetry.addData("target", "init")

        telemetry.addData("heading", {robot.currentHeading})

        robot.loopFunction = {autoLoop()}

        try {
            when (selectedOption) {
                1 -> route1()
                2 -> route2()
                3 -> route3()
            }
        } catch (exception: Exception) {}
    }

    fun autoLoop() {
        telemetry.update()
        if (!opModeIsActive()) {
            throw Exception();
        }
    }

    fun route1() {
        for (i in 1..4) {
            val target = Vector2d(0.5*(((i%2) xor 1 ) * (-(i/4) or 1)), 0.5*((i%2) * (-(i/3) or 1)))
            pathTelementry?.setValue("drive")
            targetTelementry?.setValue("("+target.x+", "+target.y+")")
            robot.driveForSecs(target, 1.0)
        }
        robot.moveRobot()
    }

    fun route2() {
        for (i in 1..2) {
            pathTelementry?.setValue("drive forward $i")
            targetTelementry?.setValue("(0, 0.5)")
            robot.driveForSecs(Vector2d(0.0, 0.5), 1.0)

            robot.moveRobot()

            pathTelementry?.setValue("turn $i")
            targetTelementry?.setValue(90*i)
            robot.turnToHeading(90.0*i, 0.2, Vector2d(0.0, 0.1))
        }

        for (i in 1..2) {
            pathTelementry?.setValue("drive forward $i")
            targetTelementry?.setValue("(0, 0.5)")
            robot.driveForSecs(Vector2d(0.0, 0.5), 1.0)

            robot.moveRobot()

            pathTelementry?.setValue("turn $i")
            targetTelementry?.setValue(-180 + 90*i)
            robot.turnToHeading(-180.0 + 90*i, 0.2, Vector2d(0.0, 0.1))
        }
        robot.moveRobot()
    }

    fun route3() {
        pathTelementry?.setValue("turn")
        targetTelementry?.setValue("0 (0.0, 1.0) 0.9")
        robot.moveRobot(0.0, 1.0, 0.9)
        robot.turnToHeading(0.0, 0.9, Vector2d(0.0, 1.0), 1)

        robot.moveRobot()
    }
}