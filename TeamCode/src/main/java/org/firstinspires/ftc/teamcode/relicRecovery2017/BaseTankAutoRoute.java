package org.firstinspires.ftc.teamcode.relicRecovery2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by RPS on 10/6/17.
 */

@Autonomous(name="tank autonomous routes")
public class BaseTankAutoRoute extends LinearOpMode {

    BaseTankAuto robot = new BaseTankAuto();
    Telemetry.Item pathTelemetry = null;
    Telemetry.Item targetTelemetry = null;

    enum TeamColor
    {
        Red, Blue
    }

    enum FieldLocation
    {
        Straight, Turn
    }

    enum GlyphColumn
    {
        Left, Center, Right
    }

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry, this);
        Integer[] options = VariableMenu.loadSelectedOptions("AutoSelection", hardwareMap.appContext.getFilesDir());
        boolean optionsLoaded = options.length > 3;
        waitForStart();
        try
        {
            double offset = -2;

            TeamColor teamColor = TeamColor.Blue;

            FieldLocation fieldLocation = FieldLocation.Straight;

            GlyphColumn glyphColumn = GlyphColumn.Left;

            telemetry.addData("team Color", teamColor);
            telemetry.addData("position", fieldLocation);
            telemetry.addData("placement", glyphColumn);
            telemetry.update();

            robot.setGriperPos(1);
            sleep(2000);

            hitJewel(teamColor);

            raiseArm();

            if (teamColor == TeamColor.Blue)
            {
                if (fieldLocation == FieldLocation.Straight)
                {
                    if (true) {
                        robot.moveForInches(28 + offset, .25); //drive forward
                    }
                    /*
                    robot.moveForInches(28 + offset, .25); //drive forward
                    leftNinety();
                    leftNinety();
                    leftNinety();
                    */
                }

                if (fieldLocation == FieldLocation.Turn)
                {
                    if (glyphColumn == GlyphColumn.Left)
                    {
                        robot.moveForInches(26.5 + offset, .25); //drive forward
                        leftNinety();
                        placeGlyph();
                    }
                    else if (glyphColumn == GlyphColumn.Center)
                    {
                        robot.moveForInches(35 + offset, .25); //drive forward
                        leftNinety();
                        placeGlyph();
                    }
                    else if (glyphColumn == GlyphColumn.Right) {
                        robot.moveForInches(43.5 + offset, .25); //drive forward
                        leftNinety();
                        placeGlyph();
                    }
                }
            }
            if (teamColor == TeamColor.Red) //red
            {
                if (fieldLocation == FieldLocation.Straight) //straight on
                {
                    robot.moveForInches(28 + 4, .5, BaseTankAuto.Direction.Backward);
                }

                if (fieldLocation == FieldLocation.Turn) //turning
                {
                    robot.moveForInches(28, .5, BaseTankAuto.Direction.Backward);
                }
            }
        }
        catch (Exception e)
        {
            telemetry.addData("error", e.toString());
            telemetry.update();
            while (opModeIsActive())sleep(1);
        }
        while (opModeIsActive()){sleep(100);}

    }

    public void hitJewel(TeamColor teamColor)
    {
        TeamColor rightBallColor = TeamColor.Red;

        //straighten the hit arm
        robot.jewelHit.setPosition(.33);

        sleep(250);

        //lower the hit arm
        robot.jewelRaise.setPosition(.2);

        sleep(2000);

        //move the hit arm close to the ball
        robot.jewelHit.setPosition(.43);

        sleep(500);

        //find color
        int red = robot.sensorRGB.red();
        int blue = robot.sensorRGB.blue();

        //use color
        if(red > blue)
        {
            rightBallColor = TeamColor.Red;
        }
        else
        {
            rightBallColor = TeamColor.Blue;
        }

        if(rightBallColor == teamColor)
        {
            robot.hitLeft();
        }
        else
        {
            //move the hit arm back to center (wind up for the hit)
            robot.jewelHit.setPosition(.33);
            sleep(100);
            //and swing batter
            robot.hitRight();
        }


        //reset the arm back up
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

        //backup ( we need to work on this part)
        robot.moveForInches(9, .5, BaseTankAuto.Direction.Backward); //back up

        //drive forward
        robot.moveForInches(4, .25);

        //bump arm down (to keep from being stuck in up position)
        robot.LifterMotor.setPower(-.3);

        //sleep(100);

        //Turn off back drive (allow the arm to lower)
        robot.LifterMotor.setPower(0);

        sleep(3000);

        //open Glyph doors
        robot.dropGlyph();

        //push Glyph in
        robot.moveForInches(2);

        //backup up out of the way
        robot.moveForInches(2, .5, BaseTankAuto.Direction.Backward);

        raiseArm();

        //move back in a little to be in safe zone
        robot.moveForInches(1, .5);
    }

}
