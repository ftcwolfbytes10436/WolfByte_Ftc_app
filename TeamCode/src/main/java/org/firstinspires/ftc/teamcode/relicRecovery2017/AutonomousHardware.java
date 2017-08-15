package org.firstinspires.ftc.teamcode.relicRecovery2017;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AutonomousHardware extends BaseMecanumHardware {

    public BNO055IMU imu = null;

    private Orientation currentOrientation = null;

    private LinearOpMode opMode = null;

    public float getHeading() {
        if (currentOrientation == null || currentOrientation.acquisitionTime != System.nanoTime()) {
            currentOrientation = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        }
        return AngleUnit.DEGREES.fromUnit(currentOrientation.angleUnit, currentOrientation.firstAngle);
    }

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init(HardwareMap ahwMap, Telemetry telemetry) {
        super.init(ahwMap, telemetry);

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
        imu.initialize();
    }

    public void setOpMode(LinearOpMode opMode) {this.opMode = opMode;}
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

    public void turnToHeading(double target) throws Exception {turnToHeading(target,0.5,0,0,0);}
    public void turnToHeading(double target, double power) throws Exception {turnToHeading(target,power, 0, 0, 0);}
    public void turnToHeading(double target, double power, double xAxis, double yAxis) throws Exception {turnToHeading(target, power, xAxis, yAxis, 0);}
    public void turnToHeading(double target, double power, double xAxis, double yAxis, int rotDirection) throws Exception {
        double distance = target - getHeading();
        int hDirection = rotDirection;
        int lastDirection;
        int numTargetPassed = 1;
        if (rotDirection == 0) {
            hDirection = (distance > 0)?1:0;
            if (Math.abs(distance) > 180) hDirection *= -1;
        }

        moveRobot(xAxis, yAxis, power * hDirection);
        while (Math.abs(distance) > 0.5) {
            telemetry.update();
            checkIfRunning();
            distance = target - getHeading();
            if (Math.abs(distance) < 5) {
                lastDirection = hDirection;
                hDirection = (distance > 0)?1:0;
                if (Math.abs(distance) > 180) {hDirection *= -1;}
                if (lastDirection != hDirection && power / numTargetPassed > 0.2) {numTargetPassed++;}
                moveRobot(xAxis, yAxis, power * hDirection/numTargetPassed);
            }
        }
    }
}
