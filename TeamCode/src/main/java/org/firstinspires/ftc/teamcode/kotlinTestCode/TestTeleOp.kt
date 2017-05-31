package org.firstinspires.ftc.teamcode.kotlinTestCode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

/**
 * Created by caleb on 5/30/2017.
 * This is an OpMode to test making a OpMode in kotlin. It is based off of the linear pushbot teleop.
 */
@TeleOp(name = "kotlin mecanum teleop", group = "kotlin")
class TestTeleOp: LinearOpMode() {

    /* Declare OpMode members. */
    var robot = BaseMecanumHardware()

    override public fun runOpMode(){

        /* Initialize the hardwar variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap)

        // send telemetry to signify robot is waiting
        telemetry.addData("Status", "waiting to start")
        telemetry.update()

        // Wait for the game to start (driver presses PLAY)
        waitForStart()

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run the wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot and the Right stick rotates it left and right
            var direction = Direction(-gamepad1.left_stick_x as Double, -gamepad1.left_stick_y as Double)
            var rotation  = -gamepad1.right_stick_x as Double

            // send the values to the hardware class
            robot.moveRobot(direction, rotation, telemetry)

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40)
        }
    }

}