package org.firstinspires.ftc.teamcode.relicRecovery2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Disabled
@Autonomous(name="autonomous routes")
public class AutonomousRoutesOp extends LinearOpMode {

    AutonomousHardware robot = new AutonomousHardware();
    Telemetry.Item pathTelemetry = null;
    Telemetry.Item targetTelemetry = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);
        robot.setOpMode(this);

        int selectedOption = 1;
        boolean pressed = false;

        while (!gamepad1.a && !isStopRequested()) {
            if (!pressed && gamepad1.dpad_up && selectedOption < 3) {
                selectedOption++;
            } else if (!pressed && gamepad1.dpad_down && selectedOption > 1) {
                selectedOption--;
            }
            pressed = gamepad1.dpad_up || gamepad1.dpad_down;

            telemetry.addData("Route", selectedOption);
            telemetry.update();
        }

        telemetry.addData("status" , "ready to start");
        telemetry.update();

        waitForStart();

        telemetry.setAutoClear(false);

        telemetry.addData("status", "running");

        pathTelemetry = telemetry.addData("path", "init");
        targetTelemetry = telemetry.addData("target", "init");

        try {
            switch (selectedOption) {
                case 1: route1(); break;
                case 2: route2(); break;
                case 3: route3();break;
            }
        } catch (Exception exception) {}
    }

    void route1() throws Exception {
        for (int i=1; i <= 4; i++) {
            double xAxis = 0.5*(((i%2) ^ 1) * (-(i/4) | 1));
            double yAxis = 0.5*((i%2) * (-(i/3) | 1));
            pathTelemetry.setValue("drive");
            targetTelemetry.setValue("("+xAxis+", "+yAxis+")");
            robot.driveForSec(xAxis, yAxis, 1);
        }
        robot.moveRobot();
    }

    void route2() throws Exception {
        for (int i=1; i <= 2; i++) {
            pathTelemetry.setValue("drive forward" + i);
            targetTelemetry.setValue("(0, 0.5)");
            robot.driveForSec(0, 0.1, 1);

            robot.moveRobot();

            pathTelemetry.setValue("turn " + i);
            targetTelemetry.setValue(90*i);
            robot.turnToHeading(90.0*i, 0.2, 0, 0.1);
        }

        for (int i=3; i <= 4; i++) {
            pathTelemetry.setValue("drive forward " + i);
            targetTelemetry.setValue("(0, 0.5)");
            robot.driveForSec(0, 0.1, 1);

            robot.moveRobot();

            pathTelemetry.setValue("turn " + i);
            targetTelemetry.setValue(-180 + 90*i);
            robot.turnToHeading(-180.0 + 90*i, 0.2, 0, 0.1);
        }
        robot.moveRobot();
    }

    void route3() throws Exception {
        pathTelemetry.setValue("turn");
        targetTelemetry.setValue("0 (0.0, 1) 0.9");
        robot.moveRobot(0, 1, 0.9);
        robot.turnToHeading(0, 0.9, 0, 1, 1);

        robot.moveRobot();
    }
}
