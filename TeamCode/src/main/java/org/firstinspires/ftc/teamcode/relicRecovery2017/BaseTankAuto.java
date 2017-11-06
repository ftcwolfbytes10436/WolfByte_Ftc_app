package org.firstinspires.ftc.teamcode.relicRecovery2017;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class BaseTankAuto extends BaseTankHardware{

    public BNO055IMU imu = null;
    ColorSensor sensorRGB;


    private Orientation currentOrientation = null;

    private LinearOpMode opMode = null;

    public float getHeading() {
        if (currentOrientation == null || currentOrientation.acquisitionTime != System.nanoTime()) {
            currentOrientation = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        }
        return AngleUnit.DEGREES.fromUnit(currentOrientation.angleUnit, currentOrientation.firstAngle);
    }

    private ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap ahwMap, Telemetry telemetry, LinearOpMode opMode) {
        super.init(ahwMap, telemetry);

        this.opMode = opMode;

        LeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        sensorRGB = ahwMap.colorSensor.get("sensor_color");

        // Set up the parameters with which we will use our IMU.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2c port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    void checkIfRunning() throws Exception {
        if (opMode != null && opMode.isStopRequested()) {
            throw new Exception("stop OpMode");
        }
    }

    public void driveForSec(double xAxis, double yAxis, double time) throws Exception {driveForSecs(xAxis, yAxis, time, 0);}
    public void driveForSecs(double xAxis, double yAxis, double time, double rotation) throws Exception {
        moveRobot(xAxis, yAxis, rotation);
        double startTime = timer.seconds();
        double currentTime = timer.seconds();
        while (currentTime - startTime < time) {
            telemetry.update();
            checkIfRunning();
            currentTime = timer.seconds();
        }
    }


    public void moveForInches(double inches) throws Exception
    {
        moveForInches(inches, 1);
    }


    public void moveForInches(double inches, double power) throws Exception //throws Exception {driveForInches(inches, power, brake);}
    {
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
        int leftTargetPos = startingLeftPos + (int)(inches * (frontLeftOneRotation / (4 * 3.1415)));
        int rightTargetPos = startingRightPos + (int)(inches * (frontRightOneRotation / (4 * 3.1415)));

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

    }

    public void hitLeft()
    {
        jewelHit.setPosition(0);
    }

    public void hitRight()
    {
        jewelHit.setPosition(1);
    }

    public void displayColorTelementy() {
        telemetry.addData("Red  ", sensorRGB.red());
        telemetry.addData("Blue ", sensorRGB.blue());
        telemetry.update();
    }

    public void turnToHeading(double target) throws Exception {turnToHeading(target, 0.5);}
    public void turnToHeading(double target, double power) throws Exception  {
        double distance = target - getHeading();
        int hDirection = 0;
        int lastDirection;
        int numTargetPassed = 1;
        hDirection = (distance > 0)?1:0;
        if (Math.abs(distance) > 180) hDirection *= -1;

        moveRobot(-hDirection*power, hDirection * power);
        while (Math.abs(distance) > 0.5) {
            telemetry.update();
            checkIfRunning();
            distance = target - getHeading();
            if (Math.abs(distance) < 5) {
                lastDirection = hDirection;
                hDirection = (distance > 0)?1:0;
                if (Math.abs(distance) > 180) {hDirection *= -1;}
                if (lastDirection != hDirection && power / numTargetPassed > 0.2) {numTargetPassed++;}
                moveRobot(hDirection/numTargetPassed * -power, hDirection/numTargetPassed * power);
            }
        }
        moveRobot(0,0);
    }
}


