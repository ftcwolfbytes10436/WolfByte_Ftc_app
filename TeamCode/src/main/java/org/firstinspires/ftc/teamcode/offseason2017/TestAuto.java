package org.firstinspires.ftc.teamcode.offseason2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Created by Andrew Brown on 4/16/18.
 */

public class TestAuto extends TestHardwareClass {

    private LinearOpMode opMode = null;

    enum Direction
    {
        Forward, Backward
    }

    public void init(HardwareMap ahwMap, Telemetry telemetry, LinearOpMode opMode) {
        this.opMode = opMode;
        super.init(ahwMap, telemetry);
    }

    void checkIfRunning() throws Exception {
        if (opMode != null && opMode.isStopRequested()) {
            throw new Exception("stop OpMode");
        }
    }
    
    public void moveForInches(double inches) throws Exception
    {
        moveForInches(inches, 1, TestAuto.Direction.Forward);
    }
    public void moveForInches(double inches, double power) throws Exception
    {
        moveForInches(inches, power, TestAuto.Direction.Forward);
    }
    public void moveForInches(double inches, double power, TestAuto.Direction direction) throws Exception //throws Exception {driveForInches(inches, power, brake);}
    {
        double pi = 3.14152653589793;

        if (direction == TestAuto.Direction.Forward) //forward
        {
            LeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            RightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        if (direction == TestAuto.Direction.Backward) //backward
        {
            LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        power = Range.clip(power, 0, 1);

        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int startingLeftPos = LeftMotor.getCurrentPosition();
        int startingRightPos = RightMotor.getCurrentPosition();

        int frontLeftOneRotation = 1440;
        int frontRightOneRotation = 1440;

        LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //(inches to move) * (number of clicks per rotation) / (2 * r * pi )
        int leftTargetPos = startingLeftPos + (int)(inches * (frontLeftOneRotation / (4 * pi)));
        int rightTargetPos = startingRightPos + (int)(inches * (frontRightOneRotation / (4 * pi)));

        LeftMotor.setTargetPosition(leftTargetPos);
        RightMotor.setTargetPosition(rightTargetPos);

        telemetry.addData("MoveInch", "Starting Motors");
        telemetry.update();

        LeftMotor.setPower(power);
        RightMotor.setPower(power);

        int currentLeftPos = LeftMotor.getCurrentPosition();;
        int currentRightPos = RightMotor.getCurrentPosition();

        while (( currentLeftPos < leftTargetPos) && ( currentRightPos < rightTargetPos))
        {
            checkIfRunning();
            if (currentLeftPos >= leftTargetPos)
            {
                LeftMotor.setPower(0);
            }
            if ( currentRightPos >= rightTargetPos)
            {
                RightMotor.setPower(0);
            }

            Thread.sleep(50);

            currentLeftPos = LeftMotor.getCurrentPosition();;
            currentRightPos = RightMotor.getCurrentPosition();

            telemetry.addData("MoveInch left", currentLeftPos + " " + leftTargetPos);
            telemetry.addData("MoveInch right", currentLeftPos + " " + leftTargetPos);


            telemetry.addData("MoveInch", "LeftPosition [" + currentLeftPos + "]");
            telemetry.addData("MoveInch", "RightPosition [" + currentRightPos + "]");
            telemetry.update();

            if ((currentLeftPos == startingLeftPos) || (currentRightPos == startingRightPos))
            {
                //throw new Exception("No Encoder Data");
            }
        }

        LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftMotor.setPower(0);
        RightMotor.setPower(0);

        LeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void moveToTick(int leftTickCount, int rightTickCount, double power) throws Exception
    {
        int startingLeftPos = LeftMotor.getCurrentPosition();
        int startingRightPos = RightMotor.getCurrentPosition();

        power = Range.clip(power, 0, 1);

        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftMotor.setTargetPosition(leftTickCount);
        RightMotor.setTargetPosition(rightTickCount);

        telemetry.addData("MoveInch", "Starting Motors");
        telemetry.update();

        LeftMotor.setPower(power);
        RightMotor.setPower(power);

        int currentLeftPos = LeftMotor.getCurrentPosition();;
        int currentRightPos = RightMotor.getCurrentPosition();

        while ((currentLeftPos < leftTickCount) && (currentRightPos < rightTickCount))
        {
            if (currentLeftPos >= leftTickCount)
            {
                LeftMotor.setPower(0);
            }
            if ( currentRightPos >= rightTickCount)
            {
                RightMotor.setPower(0);
            }

            Thread.sleep(50);

            currentLeftPos = LeftMotor.getCurrentPosition();;
            currentRightPos = RightMotor.getCurrentPosition();

            telemetry.addData("MoveInch", "LeftPosition [" + currentLeftPos + "]");
            telemetry.addData("MoveInch", "RightPosition [" + currentRightPos + "]");
            telemetry.update();

            if ((currentLeftPos == startingLeftPos) || (currentRightPos == startingRightPos))
            {
                //throw new Exception("No Encoder Data");
            }
        }

        LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftMotor.setPower(0);
        RightMotor.setPower(0);

    }
}
