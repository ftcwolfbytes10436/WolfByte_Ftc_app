package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  front Left  drive motor:  "front_left_drive"
 * Motor channel:  front Right drive motor:  "front_right_drive"
 * Motor channel:  back Left  drive motor:   "back_left_drive"
 * Motor channel:  back Right drive motor:   "back_right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class BetaLykosHardware
{
    /* Public OpMode members. */
    public DcMotor  frontLeftMotor  = null;
    public DcMotor  frontRightMotor = null;
    public DcMotor  backLeftMotor   = null;
    public DcMotor  backRightMotor  = null;
    public Servo    leftClaw        = null;
    public Servo    rightClaw       = null;

    // The IMU sensor object
    BNO055IMU imu;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public static final double powerPerDegree = .005;
    public static double heading;
    public static boolean updateHeading = false;
    public static boolean rotating = false;
    public static boolean red = false;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public BetaLykosHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftMotor = hwMap.dcMotor.get("front_left_drive");
        frontRightMotor = hwMap.dcMotor.get("front_right_drive");
        backLeftMotor = hwMap.dcMotor.get("back_left_drive");
        backRightMotor = hwMap.dcMotor.get("back_right_drive");

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Define and initialize ALL installed servos.
        //leftClaw = hwMap.servo.get("left_hand");
        //rightClaw = hwMap.servo.get("right_hand");
        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public double getHeading() {
        Orientation angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }
    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public void moveRobot(float xAxis, float yAxis, float rotation, Telemetry telemetry) {

        double currentHeading = getHeading();
        if (rotation != 0){
            rotating = true;
        } else if (rotating) {
            rotating = false;
            updateHeading = true;
        } else if (updateHeading) {
            heading = currentHeading;
            updateHeading = false;
        } else if (currentHeading != heading) {
            float dif = (float) (heading - currentHeading);
            rotation = (float) (dif * powerPerDegree);
        }

        double fLeft  = Range.clip(yAxis + xAxis + rotation,-1,1);
        double fRight = Range.clip(yAxis - xAxis - rotation,-1,1);
        double bLeft  = Range.clip(yAxis - xAxis + rotation,-1,1);
        double bRight = Range.clip(yAxis + xAxis - rotation,-1,1);

        frontLeftMotor.setPower(fLeft);
        frontRightMotor.setPower(fRight);
        backLeftMotor.setPower(bLeft);
        backRightMotor.setPower(bRight);

        // Send telemetry message to signify robot running;
        telemetry.addData("front left",  "%.2f", fLeft);
        telemetry.addData("front right", "%.2f", fRight);
        telemetry.addData("back left",  "%.2f", bLeft);
        telemetry.addData("back right", "%.2f", bRight);
    }

    public void moveRobotForSeconds(float xAxis, float yAxis, float rotation, LinearOpMode opMode, float secs) throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        moveRobot(xAxis, yAxis, rotation, opMode.telemetry);
        opMode.telemetry.update();
        while (opMode.opModeIsActive() && runtime.seconds() < secs) {
            opMode.idle();
        }
        moveRobot(0,0,0,opMode.telemetry);
        opMode.telemetry.update();
    }
}

