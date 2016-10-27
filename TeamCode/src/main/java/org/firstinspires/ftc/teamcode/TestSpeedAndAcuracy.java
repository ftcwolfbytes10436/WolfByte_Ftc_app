package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by rps on 10/27/16.
 */
@Autonomous(name="Test Speed and Acuracy", group="BetaLykos")  // @Autonomous(...) is the other common choice

public class TestSpeedAndAcuracy extends LinearOpMode {
    BetaLykosHardware robot           = new BetaLykosHardware();   // Use betaLykos' hardware
    ElapsedTime elapsedTime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        double secs = 0;
         /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Color", "test");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.moveRobotToPosition(2, 2, .1, false, this);
        robot.moveRobotForSeconds(0, 0, 0, this, 2);
        elapsedTime.reset();
        robot.moveRobotToPosition(2, 4, .2, true, this);
        robot.moveRobotToPosition(4, 4, .2, true, this);
        robot.moveRobotToPosition(4, 2, .2, true, this);
        robot.moveRobotToPosition(2, 2, .2, true, this);
        secs = elapsedTime.seconds();

        telemetry.addData("Time", secs);
        telemetry.update();

        while (!gamepad1.a) {
            idle();
        }

        robot.moveRobotToPosition(2, 2, .1, false, this);
        robot.moveRobotForSeconds(0, 0, 0, this, 2);
        elapsedTime.reset();
        robot.moveRobotToPosition(2, 4, .4, true, this);
        robot.moveRobotToPosition(4, 4, .4, true, this);
        robot.moveRobotToPosition(4, 2, .4, true, this);
        robot.moveRobotToPosition(2, 2, .4, true, this);
        secs = elapsedTime.seconds();

        telemetry.addData("Time", secs);
        telemetry.update();

        while (!gamepad1.a) {
            idle();
        }

        robot.moveRobotToPosition(2, 2, .1, false, this);
        robot.moveRobotForSeconds(0, 0, 0, this, 2);
        elapsedTime.reset();
        robot.moveRobotToPosition(2, 4, .6, true, this);
        robot.moveRobotToPosition(4, 4, .6, true, this);
        robot.moveRobotToPosition(4, 2, .6, true, this);
        robot.moveRobotToPosition(2, 2, .6, true, this);
        secs = elapsedTime.seconds();

        telemetry.addData("Time", secs);
        telemetry.update();

        while (!gamepad1.a) {
            idle();
        }

        robot.moveRobotToPosition(2, 2, .1, false, this);
        robot.moveRobotForSeconds(0, 0, 0, this, 2);
        elapsedTime.reset();
        robot.moveRobotToPosition(2, 4, .8, true, this);
        robot.moveRobotToPosition(4, 4, .8, true, this);
        robot.moveRobotToPosition(4, 2, .8, true, this);
        robot.moveRobotToPosition(2, 2, .8, true, this);
        secs = elapsedTime.seconds();

        telemetry.addData("Time", secs);
        telemetry.update();

        while (!gamepad1.a) {
            idle();
        }

        robot.moveRobotToPosition(2, 2, .1, false, this);
        robot.moveRobotForSeconds(0, 0, 0, this, 2);
        elapsedTime.reset();
        robot.moveRobotToPosition(2, 4, 1, true, this);
        robot.moveRobotToPosition(4, 4, 1, true, this);
        robot.moveRobotToPosition(4, 2, 1, true, this);
        robot.moveRobotToPosition(2, 2, 1, true, this);
        secs = elapsedTime.seconds();

        telemetry.addData("Time", secs);
        telemetry.update();

        while (!gamepad1.a) {
            idle();
        }
    }
}
