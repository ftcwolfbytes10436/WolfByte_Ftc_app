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

            //straight on = 0
            //turning     = 1
            int position = 1;

            //left   cryptobox = 0
            //center cryptobox = 1
            //right  cryptobox = 2
            int placement = 1;

            robot.setGriperPos(1);

            hitJewel(teamColor);
            raiseArm();

            if (teamColor == 0) //blue
            {
                if (position == 0) //straight on
                {
                    robot.moveForInches(28, .25); //drive forward
                }

                if (position == 1) //turning
                {
                    if (placement == 0)
                    {
                        robot.moveForInches(26.5, .25); //drive forward
                        leftNinety();
                        placeGlyph();
                    }
                    if (placement == 1)
                    {
                        robot.moveForInches(35 , .25); //drive forward
                        leftNinety();
                        placeGlyph();
                    }
                    if (placement == 2)
                    {
                        robot.moveForInches(43.5, .25); //drive forward
                        leftNinety();
                        placeGlyph();
                    }
                }
            }
            if (teamColor == 1) //red
            {
                if (position == 0) //straight on
                {
                    if (placement == 0)
                    {
                        robot.moveForInches(23, .25, 0);
                        leftNinety();
                        robot.moveForInches(3, .25);
                    }
                    if (placement == 1)
                    {
                        robot.moveForInches(27, .25, 0);
                        leftNinety();
                        robot.moveForInches(3, .25);
                    }
                    if (placement == 2)
                    {
                        robot.moveForInches(32, .25, 0);
                        leftNinety();
                        robot.moveForInches(3, .25);
                    }
                }

                if (position == 1) //turning
                {
                    if (placement == 0)
                    {

                    }
                    if (placement == 1)
                    {

                    }
                    if (placement == 2)
                    {
//
                    }
                    if (placement == 3)
                    {
                        
                    }
                }
            }
            while (opModeIsActive())sleep(1);

        }
        catch (Exception e)
        {
            telemetry.addData("error", e.toString());
            telemetry.update();
            while (opModeIsActive())sleep(1);
        }

    }

    public void hitJewel(int teamColor)
    {
        //blue = 0
        //red  = 1
        int rightColor = 0;

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
    }

    public void raiseArm()
    {
        robot.LifterMotor.setPower(0.5);
        sleep(1000);
        robot.LifterMotor.setPower(robot.backFeedPower);
    }

    public void leftNinety()
    {
        if (opModeIsActive()) {
            while (robot.getHeading() < 91 && robot.getHeading() < 89) {
                robot.LeftMotor.setPower(-.3);
                robot.RightMotor.setPower(.2);
                telemetry.addData("Heading: ", robot.getHeading());
                telemetry.update();
            }
            robot.LeftMotor.setPower(0);
            robot.RightMotor.setPower(0);
            int loopCount = 0;
            while ((robot.getHeading() > 91 || robot.getHeading() < 89) && loopCount < 3) {

                if (robot.getHeading() > 91) {
                    robot.LeftMotor.setPower(.15);
                    robot.RightMotor.setPower(-.1);
                    sleep(300);
                    robot.LeftMotor.setPower(0);
                    robot.RightMotor.setPower(0);
                } else if (robot.getHeading() < 89) {
                    robot.RightMotor.setPower(.1);
                    robot.LeftMotor.setPower(-.15);
                    sleep(300);
                    robot.LeftMotor.setPower(0);
                    robot.RightMotor.setPower(0);
                }
                loopCount++;
                telemetry.addData("Heading: ", robot.getHeading());
                telemetry.update();
            }
        }
    }

    public void rightNinety()
    {
        if (opModeIsActive()) {
            while (robot.getHeading() > -92 && robot.getHeading() > -88) {
                robot.LeftMotor.setPower(.3);
                robot.RightMotor.setPower(-.2);
                telemetry.addData("Heading: ", robot.getHeading());
                telemetry.update();
            }
            robot.LeftMotor.setPower(0);
            robot.RightMotor.setPower(0);
        }
    }

    public void turnToDegrees(double degrees)
    {
        if (opModeIsActive()) {
            if (degrees > 0) {
                while (robot.getHeading() < degrees + 2 && robot.getHeading() < degrees - 2) {
                    robot.LeftMotor.setPower(-.3);
                    robot.RightMotor.setPower(.2);
                    telemetry.addData("Heading: ", robot.getHeading());
                    telemetry.update();
                }
                robot.LeftMotor.setPower(0);
                robot.RightMotor.setPower(0);
            }
            if (degrees < 0)
            {
                while (robot.getHeading() > degrees + 2 && robot.getHeading() > degrees - 2) {
                    robot.LeftMotor.setPower(.3);
                    robot.RightMotor.setPower(-.2);
                    telemetry.addData("Heading: ", robot.getHeading());
                    telemetry.update();
                }
                robot.LeftMotor.setPower(0);
                robot.RightMotor.setPower(0);
            }
        }
    }

    public void placeGlyph() throws Exception {
        robot.moveForInches(4, .25);
        robot.LifterMotor.setPower(-.3);
        //sleep(100);
        robot.LifterMotor.setPower(0);
        sleep(3000);
        robot.dropGlyph();
        robot.moveForInches(2);
        robot.moveForInches(2, .5, 0);
        raiseArm();
        robot.moveForInches(1, .5);
    }

}
