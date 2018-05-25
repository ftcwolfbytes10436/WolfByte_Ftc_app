package org.firstinspires.ftc.teamcode.relicRecovery2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

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

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AUqb0l//////AAAAGevdjhGJj0rPhL7HPPOgXiMjObqiWrCOBJv2OvmyVIE1WTBpDt2ccEX7yWqDCZNRiMvT3ZeM/aA/Qx5Jpd1+8EraKY+8FD/uFVHJmMMwfkcYJkuIz3NzoVTdSc7c/3lwVmt7APWF/KAhoD6OPaoEjh1+gE17QlLkUoQNhlEEbbG3o2gjkyQf2xC+ZbXVehs0DF+ilZzliIa0NHNBSXutZaVmOzdbzJQNioxv9U+kf6P61pEy3aHvBPsqmRatPjzOeEN+/7NVyFJiDk2iakWxIrlTF0jUWl9zFBJcbXM+AwAaC57xY+txkzO8WFDR/ZQygDUajJKZQbfk+AbUj4yVMDCrX2bmrmZDAOFvrFtVFlqZ";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        robot.init(hardwareMap, telemetry, this);
        Integer[] options = VariableMenu.loadSelectedOptions("AutoSelection", hardwareMap.appContext.getFilesDir());
        boolean optionsLoaded = options.length > 3;
        waitForStart();
        try
        {
            relicTrackables.activate();

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            int ReadAttempts = 0;

            while (vuMark == RelicRecoveryVuMark.UNKNOWN && ReadAttempts < 15000 && opModeIsActive()) {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);

                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.addData("VuMark Read Attempt: ", "%s read(s)", ReadAttempts);
                telemetry.update();

                ReadAttempts++;
            }

            double offset = 0;

            TeamColor teamColor = TeamColor.Blue;

            FieldLocation fieldLocation = FieldLocation.Turn;

            GlyphColumn glyphColumn = GlyphColumn.Left;

            if (vuMark != RelicRecoveryVuMark.UNKNOWN)
            {
                if (vuMark == RelicRecoveryVuMark.LEFT)
                {
                    glyphColumn = GlyphColumn.Left;
                }
                else if (vuMark == RelicRecoveryVuMark.CENTER)
                {
                    glyphColumn = GlyphColumn.Center;
                }
                else
                {
                    glyphColumn = GlyphColumn.Right;
                }
            }

            telemetry.addData("VuMark", "%s visible", vuMark);

            telemetry.addData("team Color", teamColor);
            telemetry.addData("position", fieldLocation);
            telemetry.addData("placement", glyphColumn);
            telemetry.addData("VuMark Read Attempt: ", "%s read(s)", ReadAttempts);

            telemetry.update();

            robot.setGriperPos(1);

            hitJewel(teamColor);
            //sleep(500);
            
            raiseArm();

            if (teamColor == TeamColor.Blue)
            {
                if (fieldLocation == FieldLocation.Straight)
                {
                    robot.moveForInches(20 + offset, .25); //drive forward
                    placeGlyph(robot.getHeading());
                }

                if (fieldLocation == FieldLocation.Turn)
                {
                    if (glyphColumn == GlyphColumn.Left)
                    {
//                        robot.moveForInches(24.5 + offset, .25); //drive forward
//                        leftToDegrees(89);
//                        placeGlyph(89);
                        robot.moveForInches(17 + offset, .25); //drive forward
                        leftToDegrees(75);
                        placeGlyph(75);
                    }
                    else if (glyphColumn == GlyphColumn.Center)
                    {
//                        robot.moveForInches(30.5 + offset, .25); //drive forward
//                        leftToDegrees(89);
//                        placeGlyph(89);
                        robot.moveForInches(26 + offset, .25); //drive forward
                        leftToDegrees(75);
                        placeGlyph(73);
                    }
                    else if (glyphColumn == GlyphColumn.Right) {
//                        robot.moveForInches(38.5 + offset, .25); //drive forward
//                        leftToDegrees(89);
//                        placeGlyph(89);
                        robot.moveForInches(32.5 + offset, .25); //drive forward
                        leftToDegrees(75);
                        placeGlyph(73);
                    }
                }
            }
            if (teamColor == TeamColor.Red) //red
            {
                if (fieldLocation == FieldLocation.Straight) //straight on
                {
                    robot.moveForInches(30 + offset, .25, BaseTankAuto.Direction.Backward);
                }

                if (fieldLocation == FieldLocation.Turn) //turning
                {
//                    robot.moveForInches(28, .5, BaseTankAuto.Direction.Backward);
//                    rightNinety(250);
//                    placeGlyph(270);

                    if (glyphColumn == GlyphColumn.Left)
                    {
                        robot.moveForInches(47  + offset, .25, BaseTankAuto.Direction.Backward); //drive forward
                        robot.LeftMotor.setPower(-.3);
                        robot.RightMotor.setPower(.2);
                        sleep(1000);
                        leftToDegrees(75);
                        placeGlyph(75);
                    }
                    else if (glyphColumn == GlyphColumn.Center)
                    {
                        robot.moveForInches(38.5 + offset, .25, BaseTankAuto.Direction.Backward); //drive forward
                        robot.LeftMotor.setPower(-.3);
                        robot.RightMotor.setPower(.2);
                        sleep(1000);
                        leftToDegrees(75);
                        placeGlyph(72);
                    }
                    else if (glyphColumn == GlyphColumn.Right) {
                        robot.moveForInches(31.5 + offset, .25, BaseTankAuto.Direction.Backward); //drive forward
                        robot.LeftMotor.setPower(-.3);
                        robot.RightMotor.setPower(.2);
                        sleep(1000);
                        leftToDegrees(75);
                        placeGlyph(75);
                    }
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

    public double GetHeading()
    {
        double ABSHeadding = robot.getHeading();
        if ( ABSHeadding < 0)
            ABSHeadding =  ABSHeadding + 360;

//        telemetry.addData("Get Headding: ", ABSHeadding);
//        telemetry.update();

        return  ABSHeadding;
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
        sleep(50);
        robot.jewelHit.setPosition(1);
        sleep(500);
    }

    public void raiseArm()
    {
        robot.LifterMotor.setPower(0.5);
        //read the voltage and add up to 500 ms based on how low the voltage is
        VoltageSensor voltage = this.hardwareMap.voltageSensor.iterator().next();
        sleep(1000 + (350 * (14 - (long)voltage.getVoltage())));
        robot.LifterMotor.setPower(robot.backFeedPower);
    }

    public void courseCorrect(double degrees)
    {

            int loopCount = 10;
            while ((GetHeading() > (degrees + .5) || GetHeading() < (degrees - .5))
                    && loopCount > 0
                    && opModeIsActive()) {

                if (GetHeading() > degrees + .5) {
                    telemetry.addData("Turning: ", "Right");
                    //best we can tell we hit some kind of bug
                    //when we moved this code to a function we had to reverse the power
                    //I don't know maybe it was Andrew reversing the motors, changing it back
                    robot.LeftMotor.setPower(.2 * loopCount/10);
                    robot.RightMotor.setPower(-.15 * loopCount/10);
                    sleep(300);
                    robot.LeftMotor.setPower(0);
                    robot.RightMotor.setPower(0);
                } else if (GetHeading() < (degrees - .5)) {
                    telemetry.addData("Turning: ", "Left");
                    robot.RightMotor.setPower(.15 * loopCount/10);
                    robot.LeftMotor.setPower(-.2 * loopCount/10);
                    sleep(300);
                    robot.LeftMotor.setPower(0);
                    robot.RightMotor.setPower(0);
                }
                loopCount--;
                telemetry.addData("Heading: ", GetHeading());
                telemetry.update();
            }

    }

    public void leftToDegrees(double degrees)
    {
        if (opModeIsActive()) {
            while (robot.getHeading() < (degrees + 1) && robot.getHeading() < (degrees - 1) && opModeIsActive()) {
                robot.LeftMotor.setPower(-.3);
                robot.RightMotor.setPower(.2);
                telemetry.addData("Heading: ", robot.getHeading());
                telemetry.update();
            }
            robot.LeftMotor.setPower(0);
            robot.RightMotor.setPower(0);
//            int loopCount = 0;
//            while ((GetHeading() > (degrees + 1) || GetHeading() < (degrees - 1)) && loopCount < 3) {
//
//                if (GetHeading() > degrees + 1) {
//                    robot.LeftMotor.setPower(.2);
//                    robot.RightMotor.setPower(-.15);
//                    sleep(300);
//                    robot.LeftMotor.setPower(0);
//                    robot.RightMotor.setPower(0);
//                } else if (GetHeading() < (degrees - 1)) {
//                    robot.RightMotor.setPower(.15);
//                    robot.LeftMotor.setPower(-.2);
//                    sleep(300);
//                    robot.LeftMotor.setPower(0);
//                    robot.RightMotor.setPower(0);
//                }
//                loopCount++;
//                telemetry.addData("Heading: ", GetHeading());
//                telemetry.update();
//            }
        }
    }

    public void rightNinety(double degrees)
    {
        if (opModeIsActive()) {
            if (opModeIsActive()) {
                robot.LeftMotor.setPower(.3); //start before reading to make sure that we're past 0 in the positive direction
                robot.RightMotor.setPower(-.2);
            }
            sleep(1000); //give it a couple of seconds then slow down
            robot.LeftMotor.setPower(.2); //start before reading to make sure that we're past 0 in the positive direction
            robot.RightMotor.setPower(-.15);
            while ((GetHeading() < (degrees + 20)) && opModeIsActive()) {
                telemetry.addData("Heading: ", GetHeading());
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
                while (GetHeading() < degrees + 2 && GetHeading() < degrees - 2) {
                    robot.LeftMotor.setPower(-.3);
                    robot.RightMotor.setPower(.2);
                    telemetry.addData("Heading: ", GetHeading());
                    telemetry.update();
                }
                robot.LeftMotor.setPower(0);
                robot.RightMotor.setPower(0);
            }
            if (degrees < 0)
            {
                while (GetHeading() > degrees + 2 && GetHeading() > degrees - 2) {
                    robot.LeftMotor.setPower(.3);
                    robot.RightMotor.setPower(-.2);
                    telemetry.addData("Heading: ", GetHeading());
                    telemetry.update();
                }
                robot.LeftMotor.setPower(0);
                robot.RightMotor.setPower(0);
            }
        }
    }

    public void placeGlyph(double courseCorrectDegrees) throws Exception {

        //backup ( we need to work on this part)
        robot.moveForInches(4, .25, BaseTankAuto.Direction.Backward); //back up

        //bump arm down (to keep from being stuck in up position)
        robot.LifterMotor.setPower(-.3);

        sleep(100);

        //Turn off back drive (allow the arm to lower)
        robot.LifterMotor.setPower(0.05);

        sleep(3000);

        telemetry.addData("Before Heading: ", GetHeading());

        courseCorrect(courseCorrectDegrees);
        telemetry.addData("After Heading: ", GetHeading());

        robot.LifterMotor.setPower(0);

        //open Glyph doors
        robot.dropGlyph();

        robot.moveForInches(12, .25);

        //backup up out of the way
        robot.moveForInches(5, .5, BaseTankAuto.Direction.Backward);
    }

}
