package org.firstinspires.ftc.teamcode.relicRecovery2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by RPS on 10/6/17.
 */

@Autonomous(name="tank autonomous routes")
public class BaseTankAutoRoute extends LinearOpMode {

    BaseTankAuto robot = new BaseTankAuto();
    Telemetry.Item pathTelemetry = null;
    Telemetry.Item targetTelemetry = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry, this);

        waitForStart();
        try
        {
            //blue = 0
            //red  = 1
            int teamColor = 0;

            //blue = 0
            //red  = 1
            int rightColor = 0;

            robot.setGriperPos(1);


            //raise hit
            //lower arm
            robot.jewelHit.setPosition(.33);
            sleep(250);
            robot.jewelRaise.setPosition(.2);

            sleep(2000);

            //find color
            int red = robot.sensorRGB.red();
            int blue = robot.sensorRGB.blue();

            //use color
            if(red > blue)
            {
                rightColor = 1;
            }
            else
            {
                rightColor = 0;
            }

            if(rightColor == teamColor)
            {
                robot.hitLeft();
            }
            else
            {
                robot.hitRight();
            }


            sleep(500);
            robot.jewelRaise.setPosition(.85);
            sleep(500);
            robot.jewelHit.setPosition(1);
            sleep(500);

            robot.LifterMotor.setPower(0.5);
            sleep(1000);
            robot.LifterMotor.setPower(robot.backFeedPower);
            robot.moveForInches(30, .25);

            telemetry.addData("start heading", robot.getHeading());

            robot.turnToHeading(90, 0.25);
            telemetry.addData("end heading", robot.getHeading());


            telemetry.update();
            robot.moveForInches(2, .25);
            robot.setGriperPos(.85);
            while (opModeIsActive())sleep(1);

        }
        catch (Exception e)
        {
            telemetry.addData("error", e.toString());
            telemetry.update();
            while (opModeIsActive())sleep(1);
        }

    }

}
