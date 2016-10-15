package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * This is NOT an opmode.
 *
 * This class is used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  front Left  drive motor:      "front_left_drive"
 * Motor channel:  front Right drive motor:      "front_right_drive"
 * Motor channel:  back Left  drive motor:       "back_left_drive"
 * Motor channel:  back Right drive motor:       "back_right_drive"
 * Motor channel:  particle collector:           "particle_motor"
 * Motor channel:  particle launcher:            "particle_launcher"
 * Servo channel:  Servo to hold the left rail:  "left_rail"
 * Servo channel:  Servo to hold the right rail: "right_rail"
 * I2C   channel:  IMU sensor for gyro:          "imu"
 */
public class BetaLykosHardware
{
    /* Public OpMode members. */
    public DcMotor  frontLeftMotor   = null;
    public DcMotor  frontRightMotor  = null;
    public DcMotor  backLeftMotor    = null;
    public DcMotor  backRightMotor   = null;
    public DcMotor  particleMotor    = null;
    public DcMotor  particleLauncher = null;
    public Servo    leftRail         = null;
    public Servo    rightRail        = null;

    // The IMU sensor object
    BNO055IMU imu;

    public static final double OPEN_SERVO_POSITION  =  0.5 ;
    public static final double CLOSED_SERVO_POSITION = 0;
    public static final double powerPerDegree = .005;

    public static double heading = 0;

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
        particleMotor = hwMap.dcMotor.get("particle_motor");
        particleLauncher = hwMap.dcMotor.get("particle_launcher");

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        particleMotor.setDirection(DcMotor.Direction.FORWARD);
        particleLauncher.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        particleMotor.setPower(0);
        particleLauncher.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        particleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        particleLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        leftRail = hwMap.servo.get("left_rail");
        rightRail = hwMap.servo.get("right_rail");

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
        while (!imu.isAccelerometerCalibrated()) {}
        imu.startAccelerationIntegration(null,imu.getVelocity(),100);
    }

    /**
     * Get the heading of the robot from the IMU
     *
     * @return Heading of the robot in degrees
     */

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    /**
     * Get the position of the robot using the IMU, specifically the accelerometer on the IMU
     *
     * @return The position of the robot as a position class
     */

    public Position getPosition() {
        return imu.getPosition();
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

    /**
     * Unlock or lock both rails
     *
     * @param unlocked whether to lock or unlock the rails
     */
    public void lockUnlockRails(boolean unlocked) {
        if (unlocked) {
            leftRail.setPosition(OPEN_SERVO_POSITION);
            rightRail.setPosition(OPEN_SERVO_POSITION);
        } else {
            leftRail.setPosition(CLOSED_SERVO_POSITION);
            rightRail.setPosition(CLOSED_SERVO_POSITION);
        }
    }

    /**
     * Moves the robot in the desired direction and power or rotates the robot with the desired power.
     * It will try to self correct its heading if it turns with out getting any rotation power.
     *
     * @param xAxis the power of the motors in the x axis
     * @param yAxis the power of the motors in the y axis
     * @param rotation the power of rotation of the robot
     * @param telemetry a reference to the telemetry class to report any data to the driver station
     */

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
            rotation =  (float) (dif * powerPerDegree);
        }

        double fLeft  = Range.clip(yAxis + xAxis + rotation,-1,1);
        double fRight = Range.clip(yAxis - xAxis - rotation,-1,1);
        double bLeft  = Range.clip(yAxis - xAxis + rotation,-1,1);
        double bRight = Range.clip(yAxis + xAxis - rotation,-1,1);

        frontLeftMotor.setPower (fLeft);
        frontRightMotor.setPower(fRight);
        backLeftMotor.setPower  (bLeft);
        backRightMotor.setPower (bRight);

        if (!rotating) {
            rotation = 0;
        }
        // Send telemetry message to signify robot running;
        telemetry.addData("MoveRobot input"  , "XPower: " + xAxis + "    YPower: " + yAxis + "Rotation: " + rotation);
        telemetry.addData("Front Wheel Power", "Left:   " + fLeft + "    Right:  " + fRight);
        telemetry.addData("Back  Wheel Power", "Left:   " + bLeft + "    Right:  " + bRight);
        telemetry.addData("Heading" , "%.2f" ,heading);
        telemetry.addData("Position", "("+ getPosition().x + "," + getPosition().y + ")");
    }

    /**
     * Moves the robot in the desired direction and power or rotates the robot with the desired power
     * for the specifed amount of time. It uses the moveRobot method.
     *
     * @param xAxis the power of the motors in the x axis
     * @param yAxis the power of the motors in the y axis
     * @param rotation the power of rotation of the robot
     * @param opMode a reference to the linear opMode
     * @param secs amount of seconds to drive for
     * @throws InterruptedException
     */

    public void moveRobotForSeconds(float xAxis, float yAxis, float rotation, LinearOpMode opMode, float secs) throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        opMode.telemetry.addData("Status", "Running");
        moveRobot(xAxis, yAxis, rotation, opMode.telemetry);
        opMode.telemetry.update();
        while (opMode.opModeIsActive() && runtime.seconds() < secs) {
            opMode.idle();
        }
        opMode.telemetry.addData("Status", "Running");
        moveRobot(0,0,0,opMode.telemetry);
        opMode.telemetry.update();
    }

    /**
     * Moves the robot to the specified location at the specified power. This method uses the IMU accelerometer to mesure
     * distance to go the location.
     *
     * @param x The target x position to go to
     * @param y The target y position to go to
     * @param power the max amount of power to apply to the motors
     * @param turnToDestination whether or not to turn to the destination
     * @param opMode a reference to the LinerOpMode
     * @throws InterruptedException
     */

    public void moveRobotToPosition(float x, float y, float power, boolean turnToDestination, LinearOpMode opMode) throws InterruptedException {

        Position position = getPosition();
        float distanceX = (float) (x - position.x);
        float distanceY = (float) (y - position.y);
        float powerX;
        float powerY;
//        if (turnToDestination) {
//            double targetHeading =
//        }
        while (distanceX > 0.01 && distanceY > 0.01 && opMode.opModeIsActive()) {
            opMode.telemetry.addData("Status", "Running");

            position = getPosition();
            distanceX = (float) (x - position.x);
            distanceY = (float) (y - position.y);

            if (distanceX >= distanceY) {
                powerX = power;
                powerY = distanceY / distanceX * power;
            } else {
                powerX = distanceX / distanceY * power;
                powerY = power;
            }

            moveRobot(powerX,powerY,0,opMode.telemetry);

            opMode.telemetry.addData("Target position","(" + x + "," + y + ")");
            opMode.telemetry.update();
            opMode.idle();
        }
        moveRobot(0,0,0,opMode.telemetry);
    }
}

