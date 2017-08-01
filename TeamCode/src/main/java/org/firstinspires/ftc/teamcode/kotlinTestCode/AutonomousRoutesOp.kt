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

        while (!gamepad1.a) {
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

        when (selectedOption) {
            1 -> route1()
            2 -> route2()
            3 -> route3()
        }
    }

    fun autoLoop() {
        telemetry.update()
    }

    fun route1() {
            for (i in 1..4) {
                pathTelementry?.setValue("drive forward $i")
                targetTelementry?.setValue("(0, 0.5)")
                robot.driveForSecs(Vector2d(0.0, 0.5), 1.0)

                pathTelementry?.setValue("turn $i")
                targetTelementry?.setValue(robot.currentHeading + 90)
                robot.turnToHeading(robot.currentHeading.toDouble() + 90)
            }
            robot.moveRobot()
    }

    fun route2() {
        for (i in 1..4) {
            val target = Vector2d(0.5*(((i%2) xor 1 ) * -(i/4)), 0.5*((i%2) * -(i/3)))
            pathTelementry?.setValue("drive")
            targetTelementry?.setValue("("+target.x+", "+target.y+")")
            robot.driveForSecs(target, 1.0)
        }
        robot.moveRobot()
    }

    fun route3() {
        pathTelementry?.setValue("turn")
        targetTelementry?.setValue("180 (0.0, 0.5)")
        robot.turnToHeading(180.0, 0.1, Vector2d(0.0, 0.5))

        targetTelementry?.setValue("360 (0.0, 0.5)")
        robot.turnToHeading(360.0, 0.1, Vector2d(0.0, 0.5))

        robot.moveRobot()
    }
}