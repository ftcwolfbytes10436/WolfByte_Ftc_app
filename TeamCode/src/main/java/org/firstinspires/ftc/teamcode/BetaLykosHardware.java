package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.ServoConfiguration;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This is NOT an opmode.
 *
 * This class is used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor  channel:  front Left  drive motor:            "front_left_drive"
 * Motor  channel:  front Right drive motor:            "front_right_drive"
 * Motor  channel:  back Left  drive motor:             "back_left_drive"
 * Motor  channel:  back Right drive motor:             "back_right_drive"
 * Motor  channel:  particle collector:                 "particle_motor"
 * Motor  channel:  particle launcher:                  "particle_launcher"
 * Servo  channel:  Servo to hold the left rail:        "left_rail"
 * Servo  channel:  Servo to hold the right rail:       "right_rail"
 * Servo  channel:  Servo to push right beacon button:  "right_beacon_servo"
 * Servo  channel:  Servo to push left beacon button:   "left_beacon_servo"
 * I2C    channel:  IMU sensor for gyro:                "imu"
 * analog channel:  Front sonar range sensor:           "front_range_sensor"
 * analog channel:  Side sonar range sensor:            "side_range_sensor"
 */
public class BetaLykosHardware
{
    /* Public OpMode members. */
    public DcMotor      frontLeftMotor     = null;
    public DcMotor      frontRightMotor    = null;
    public DcMotor      backLeftMotor      = null;
    public DcMotor      backRightMotor     = null;
    public DcMotor      particleMotor      = null;
    public DcMotor      particleLauncher   = null;
    public Servo        leftRail           = null;
    public Servo        rightRail          = null;
    public Servo        rightBeaconServo   = null;
    public Servo        leftBeaconServo    = null;
    public CRServo      scoopServo         = null;

    DeviceInterfaceModule cdim;
    public AnalogInput frontRangeSensor   = null;
    public AnalogInput sideRangeSensor    = null;
    public OpticalDistanceSensor odsSensor = null;
    public ColorSensor sensorRGB           = null;
    public BNO055IMU imu;
    public TouchSensor touchSensor         = null;

    public boolean useDistanceSensorForInitialPosition = false;
    public boolean onRedAlliance = false;
    public Position currentPosition = new Position();


    static final int LED_CHANNEL = 5;
    public static final double OPEN_SERVO_POSITION  =  0.5 ;
    public static final double CLOSED_SERVO_POSITION = 0;
    public static final double SERVOPUSHEDPOSSITION = 1;
    public static final double SERVOUNPUSHEDPOSSITION = 0;
    public static final double rotationCorrectionPower = .5  ;
    public static final double distanceFromFrontSensorToCenter = 0;
    public static final double distanceFromSideSensorToCenter = 0;
    public static final double distanceFromBackToCenter = 0;
    public static final double LENGTHOFRAMP = 3;
    public static final double FIELDDIMENSION = 12;


    public static double heading = 0;

    public static boolean updateHeading = false;
    public static boolean rotating = false;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    Orientation currentHeading;
    Position lastPostion;

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
//        particleMotor = hwMap.dcMotor.get("particle_motor");
//        particleLauncher = hwMap.dcMotor.get("particle_launcher");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
//        particleMotor.setDirection(DcMotor.Direction.FORWARD);
//        particleLauncher.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
//        particleMotor.setPower(0);
//        particleLauncher.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        particleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        particleLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
//        leftRail = hwMap.servo.get("left_rail");
//        rightRail = hwMap.servo.get("right_rail");
        rightBeaconServo = hwMap.servo.get("right_beacon_servo");
        leftBeaconServo = hwMap.servo.get("left_beacon_servo");
        rightBeaconServo.setPosition(0);
        scoopServo = hwMap.crservo.get("scoop_servo");
        scoopServo.setPower(-0.05);

        cdim = hwMap.deviceInterfaceModule.get("dim");
        frontRangeSensor = hwMap.get(AnalogInput.class, "front_range_sensor");
        sideRangeSensor = hwMap.get(AnalogInput.class, "side_range_sensor");
        odsSensor = hwMap.opticalDistanceSensor.get("ods");
        sensorRGB = hwMap.colorSensor.get("sensor_color");
        touchSensor = hwMap.touchSensor.get("touch");

        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelState(LED_CHANNEL, false);

        // Set up the parameters with which we will use our IMU.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        Position initalPosition = new Position(DistanceUnit.METER, 0, 0, 0, 0);

        if (useDistanceSensorForInitialPosition) {
            double xPosition = 0;
            double yPosition = 0;

            if (onRedAlliance) {
                yPosition = 12 - distanceFromBackToCenter;
                xPosition = 12 - getSideRangeDistance() - distanceFromSideSensorToCenter;

            } else {
                xPosition = 12 - distanceFromBackToCenter;
                yPosition = 12 - getSideRangeDistance() - distanceFromSideSensorToCenter;

            }

            xPosition /= 3.28;
            yPosition /= 3.28;

            initalPosition = new Position(DistanceUnit.METER, xPosition, yPosition, 0, 0);
        }

        imu.startAccelerationIntegration(initalPosition, new Velocity(DistanceUnit.METER, 0, 0, 0, 0), 1000);
    }

    /**
     * Get the heading of the robot from the IMU
     *
     * @return Heading of the robot in degrees
     */

    public double getHeading() {
        if (currentHeading == null || currentHeading.acquisitionTime != System.nanoTime()) {
            currentHeading = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        }
        return AngleUnit.DEGREES.fromUnit(currentHeading.angleUnit, currentHeading.firstAngle);
    }

    /**
     * Get the position of the robot using the IMU, specifically the accelerometer on the IMU
     *
     * @return The position of the robot as a position class
     */

    public Position getPositionfromRangeSensor() {
//        if (imu != null) {
//            Position position = imu.getPosition();
//            position.x *= 3.28;
//            position.y *= 3.28;
//            return position;
//        }
        final double MAXCHANGEALLOWED = 10;
        double frontDistance = getFrontRangeDistance();
        double sideDistance = getSideRangeDistance();
        double heading = getHeading();
        double c;
        double f;
        double x;
        double y;

        if ((heading >= 0 && heading <= 90) || (heading <= -90 && heading >= -180)){
            c = sideDistance;
            f = frontDistance;
        } else {
            c = frontDistance;
            f = sideDistance;
        }

        double angleHeading = Math.abs(heading % 90);

        double B = 180 - (angleHeading + 90);
        double b = c / Math.sin(90) * Math.sin(B);
        double e = f / Math.sin(90) * Math.sin(B);
        double h = Math.sqrt(Math.pow(f,2) + Math.pow(e,2));
        double i = b - h;
        double k = LENGTHOFRAMP - i;

        if (heading >= 0 && heading <= 90) {
            if (k == 0) {
                x = b;
                y = e;
            } else {
                x = b;
                y = e + k;
            }
            y = FIELDDIMENSION - y;
        } else if (heading >= 90 && heading <= 180) {
            if (k == 0) {
                x = e;
                y = b;
            } else {
                x = e + k;
                y = b;
            }
        } else if (heading <= 0 && heading >= -90) {
            if (k == 0) {
                x = e;
                y = b;
            } else {
                x = e + k;
                y = b;
            }
            x = FIELDDIMENSION - x;
            y = FIELDDIMENSION - y;
        } else {
            if (k == 0) {
                x = b;
                y = e;
            } else {
                x = b;
                y = e + k;
            }
            x = FIELDDIMENSION - x;
        }
        Position position = new Position(DistanceUnit.METER,x,y,0, System.currentTimeMillis());
        if (lastPostion == null) {
            lastPostion = position;
        }
        double xDiff = Math.abs(currentPosition.x - position.x);
        double yDiff = Math.abs(currentPosition.y - position.y);
        if (xDiff < MAXCHANGEALLOWED) {
            lastPostion.x = position.x;
        }
        if (yDiff < MAXCHANGEALLOWED) {
            lastPostion.y = position.y;
        }
        return lastPostion;
    }

    public double getFrontRangeDistance() {
        if (frontRangeSensor != null) {
            return frontRangeSensor.getVoltage() / 9.8 * 1000 / 12;
        }
        return 0;
    }

    public double getSideRangeDistance() {
        if (sideRangeSensor != null) {
            return sideRangeSensor.getVoltage() / 9.8 * 1000 / 12;
        }
        return 0;
    }

    public double getODSLightLevel() {
        return odsSensor.getLightDetected();
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
        if (leftRail != null && rightRail != null) {
            if (unlocked) {
                leftRail.setPosition(OPEN_SERVO_POSITION);
                rightRail.setPosition(OPEN_SERVO_POSITION);
            } else {
                leftRail.setPosition(CLOSED_SERVO_POSITION);
                rightRail.setPosition(CLOSED_SERVO_POSITION);
            }
        }
    }

    public void pressLeftServo (){
        leftBeaconServo.setPosition(SERVOPUSHEDPOSSITION);
        leftBeaconServo.setPosition(SERVOUNPUSHEDPOSSITION);
    }

    public void pressRightServo (){
        rightBeaconServo.setPosition(SERVOPUSHEDPOSSITION);
        rightBeaconServo.setPosition(SERVOUNPUSHEDPOSSITION);
    }

    public double getAnglefromADirection(double x, double y) {
        Position position = getPositionfromRangeSensor();
        double xDistance = x - position.x;
        double yDistance = y - position.y;
        double heading = 0;
        if (xDistance > 0 && yDistance > 0) {
            heading = -Math.atan(xDistance / yDistance);

        } else if (xDistance < 0 && yDistance > 0) {
            heading = Math.atan(xDistance / yDistance);

        } else if (xDistance < 0 && yDistance < 0) {
            heading = Math.atan(yDistance / xDistance) + 90;

        } else if (xDistance > 0 && yDistance < 0) {
            heading = -Math.atan(yDistance / xDistance) - 90;
        }
        return heading;
    }

    public Position getDirectionFromXAndYDistance(double x, double y) {
        Position output = new Position();
        if (Math.abs(x) >= Math.abs(y)) {
            output.x = 1;
            output.y = y / x;
        } else {
            output.x = x / y;
            output.y = 1;
        }
        output.x = Math.copySign(output.x,x);
        output.y = Math.copySign(output.y,y);
        return output;
    }

    public void turnRobotToHeading(double heading, double power, LinearOpMode opMode) {
        double currentAngle = getHeading();

        while (currentAngle > heading && opMode.opModeIsActive()) {
            currentAngle = getHeading();
            moveRobot(0,0,power,opMode.telemetry);
            opMode.telemetry.addData("heading",heading);
            opMode.telemetry.addData("current heading", currentAngle);
            opMode.telemetry.addData("power",power);
            opMode.telemetry.update();
            opMode.idle();
        }
        while (currentAngle < heading && opMode.opModeIsActive()) {
            currentAngle = getHeading();
            moveRobot(0,0,-power/4,opMode.telemetry);
        }
        while (currentAngle > heading && opMode.opModeIsActive()) {
            currentAngle = getHeading();
            moveRobot(0,0,power/4/4,opMode.telemetry);
        }
        moveRobot(0,0,0,opMode.telemetry);
//        while (opMode.gamepad1.a && opMode.opModeIsActive()) {opMode.idle();}
//        while (!opMode.gamepad1.a && opMode.opModeIsActive()) {
//            opMode.telemetry.addData("if",currentAngle < heading);
//            opMode.telemetry.addData("heading",heading);
//            opMode.telemetry.addData("current heading", getHeading());
//            opMode.telemetry.addData("power",power);
//            opMode.telemetry.update();
//        }
    }

    public void turnRobotTowardsPoint(double x, double y, double power, LinearOpMode opMode) {
        turnRobotToHeading(getAnglefromADirection(x,y),power,opMode);
    }

    public void accelerateRobot(double xAxis, double yAxis, double power, double time, LinearOpMode opMode) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() < time && opMode.opModeIsActive()) {
            double finalpower = power * timer.seconds()/time;
            moveRobot(xAxis * finalpower, yAxis * finalpower,0,opMode.telemetry);
            opMode.idle();
        }
    }

    /**
     * Moves the robot in the desired direction and power or rotates the robot with the desired power.
     * It will try to self correct its heading if it turns with out getting any rotation power.
     *  @param xAxis the power of the motors in the x axis
     * @param yAxis the power of the motors in the y axis
     * @param rotation the power of rotation of the robot
     * @param telemetry a reference to the telemetry class to report any data to the driver station
     */

    public void moveRobot(double xAxis, double yAxis, double rotation, double separate, Telemetry telemetry) {

        double fLeft;
        double fRight;
        double bRight;
        double bLeft;
        long waittime = 200;
        final double tolorance = 4;
        double currentHeading = getHeading();
        float dif = (float) (heading - currentHeading);
        if (rotation != 0){
            rotating = true;
        } else if (currentHeading != heading && rotating) {
            heading = currentHeading;
            rotating = false;
        } else if (Math.abs(dif) > tolorance) {
//            rotation = Math.copySign(rotationCorrectionPower,dif);
        }

        fLeft = Range.clip(yAxis + xAxis + rotation + separate, -1, 1);
        fRight = Range.clip(yAxis - xAxis - rotation + separate, -1, 1);
        bLeft = Range.clip(yAxis - xAxis + rotation - separate, -1, 1);
        bRight = Range.clip(yAxis + xAxis - rotation - separate, -1, 1);

        if(xAxis  == 0) { // robot is moving forwards or backwards, start front motors first
            frontLeftMotor.setPower(fLeft);
            frontRightMotor.setPower(fRight);
            backRightMotor.setPower(bRight);
            backLeftMotor.setPower(bLeft);
        } else if (yAxis == 0 ) { // robot is moving sideways start the right side first
            frontRightMotor.setPower(fRight);
            backRightMotor.setPower(bRight*.8);  //the .8 is to compensate for the robot pulling forward
            backLeftMotor.setPower(bLeft*.8);
            frontLeftMotor.setPower(fLeft);
        } else if (xAxis == yAxis ) {  // robot is moving diagonal so start diagonal motors first
            frontLeftMotor.setPower(fLeft);
            backRightMotor.setPower(bRight);
            backLeftMotor.setPower(bLeft);
            frontRightMotor.setPower(fRight);
        } else {  // robot is moving the other diagonal so start its diagonal motors first
            frontRightMotor.setPower(fRight);
            backLeftMotor.setPower(bLeft);
            frontLeftMotor.setPower(fLeft);
            backRightMotor.setPower(bRight);
        }
        if (!rotating) {
            rotation = 0;
        }
        // Send telemetry message to signify robot running;
        //telemetry.addData("MoveRobot input"  , "XPower: %.2f    YPower: %.2f    Rotation: %.2f", xAxis, yAxis, rotation);
        telemetry.addData("Front Wheel Power", "Left:  %.2f     Right:  %.2f", fLeft, fRight);
        telemetry.addData("Back  Wheel Power", "Left:  %.2f     Right:  %.2f", bLeft, bRight);
//        telemetry.addData("Heading" , "%.2f" ,heading);
        telemetry.addData("Current heading", "%.2f", getHeading());
        /*telemetry.addData("Position", "( %.2f, %.2f)", getPositionfromRangeSensor().x, getPositionfromRangeSensor().y);
        telemetry.addData("front Range", getFrontRangeDistance());
        telemetry.addData("side Range", getSideRangeDistance());
        */
    }

    public void moveRobot(double xAxis, double yAxis, double rotation, Telemetry telementry) {
        moveRobot(xAxis,yAxis,rotation,0,telementry);
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

    public void moveRobotForSeconds(float xAxis, float yAxis, float rotation, LinearOpMode opMode, double secs) throws InterruptedException {

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
        opMode.telemetry.addData("Time", runtime.seconds());
        opMode.telemetry.update();
    }

    public void moveRobotToPositionUsingTime(double x, double y, double power, boolean turnToDestination, LinearOpMode opMode) throws InterruptedException {

        double distX = x - currentPosition.x;
        double distY = y - currentPosition.y;
        double distance = Math.sqrt(Math.pow(distX,2) + Math.pow(distY,2));
//        double time = Math.log10((distance * 12 + 189.5)/186.27)/Math.log10(1.1499);
        double time = (distance * 12 + 3.65) / 39.073;

        Position direction = getDirectionFromXAndYDistance(distX,distY);
        direction.x *= power;
        direction.y *= power;

//        accelerateRobot(direction.x,direction.y,power,1,opMode);
        moveRobotForSeconds((float)direction.x,(float)direction.y,0,opMode,time);

        currentPosition = new Position(DistanceUnit.METER,x,y,0,System.currentTimeMillis());
        opMode.telemetry.addData("Distance x and y", "( %.2f, %.2f)",distX,distY);
        opMode.telemetry.addData("Distance", distance);
        opMode.telemetry.addData("target",time);
    }

    /**
     * Moves the robot to the specified location at the specified power. This method uses the IMU accelerometer to mesure
     * distance to go the location.
     *
     * @return whether or not the the robot got to the destination
     * @param x The target x position to go to
     * @param y The target y position to go to
     * @param power the max amount of power to apply to the motors
     * @param turnToDestination whether or not to turn to the destination
     * @param opMode a reference to the LinerOpMode
     * @throws InterruptedException
     */

    public boolean moveRobotToPosition(double x, double y, double power, boolean turnToDestination, LinearOpMode opMode) throws InterruptedException {

        final double maxAmountOfSamePosition = 1000;
        final double amountOfToleranceForSame = 0.01;
        int amountOfSamePos = 0;
        Position position = getPositionfromRangeSensor();
        Position lastPosition;
        double distanceX =  x - position.x;
        double distanceY = y - position.y;
        double powerX;
        double powerY;

        if (turnToDestination) {
            //turnRobotTowardsPoint(x,y,power,opMode);
        }

        while ((Math.abs(distanceX) > 0.01 || Math.abs(distanceY) > 0.01) && opMode.opModeIsActive()) {
            opMode.telemetry.addData("Status", "Running");

            lastPosition = position;
            position = getPositionfromRangeSensor();
            distanceX = (float) (x - position.x);
            distanceY = (float) (y - position.y);

            Position direction = getDirectionFromXAndYDistance(distanceX,distanceY);
            powerX = direction.x * power;
            powerY = direction.y * power;

            moveRobot(powerX,powerY,0,opMode.telemetry);

            if (opMode.gamepad1.left_stick_button && opMode.gamepad1.right_stick_button) {
                moveRobot(0,0,0,opMode.telemetry);
                return false;
            }

            double xDiff = lastPosition.x - position.x;
            double yDiff = lastPosition.y - position.y;

            if ( xDiff < amountOfToleranceForSame * power && xDiff > -amountOfToleranceForSame * power &&
                    yDiff < amountOfToleranceForSame * power && yDiff > -amountOfToleranceForSame * power) {
                amountOfSamePos ++;
            }

            if (amountOfSamePos >= maxAmountOfSamePosition) {
                //moveRobot(0,0,0,opMode.telemetry);
                //return false;
            }

            if (!opMode.opModeIsActive()) {
                break;
            }

            opMode.telemetry.addData("Target position","( %.2f, %2f )",x,y);
            opMode.telemetry.update();
            opMode.idle();
        }
        moveRobot(0,0,0,opMode.telemetry);
        return true;
    }

    public boolean moveRobotFeetRelitive(double x, double y, double power, LinearOpMode opMode) throws InterruptedException {
        double targetXPos = getFrontRangeDistance() + y;
        double targetYPos = getSideRangeDistance() + x;
        double diffX = 1;
        double diffY = 1;
        double powerX;
        double powerY;
        while ((!(Math.abs(diffY) < 0.1) || !(Math.abs(diffX) < 0.1)) && opMode.opModeIsActive()) {
            diffY = targetYPos - getFrontRangeDistance();
            diffX = targetXPos - getSideRangeDistance();

            Position direction = getDirectionFromXAndYDistance(diffX,diffY);
            powerX = direction.x * power;
            powerY = direction.y * power;

            moveRobot(powerX,powerY,0,opMode.telemetry);
            opMode.telemetry.addData("distance to target","x: %.2f   y: %.2f",diffX,diffY);
            opMode.telemetry.update();
            opMode.idle();
        }
        moveRobot(0,0,0,opMode.telemetry);
        return true;
    }

    public void moveRobotCurrentNew(double currX, double currY, double newX, double newY, LinearOpMode opMode) throws InterruptedException {
        double xDiff = newX - currX;
        double yDiff = newY - currY;

        if ( yDiff == 0) {
            moveRobotForSeconds(0,0,0,opMode,0);
        } else if (yDiff != 0) {
            if (yDiff > 0) {
                moveRobotForSeconds(0,.5f,0,opMode,(float)(yDiff*12/39.1527));
            } else if (yDiff < 0) {
                moveRobotForSeconds(0,-.5f,0,opMode,(float)(+yDiff*12/39.1527));
            }
        }
        if (xDiff == 0) {
            moveRobotForSeconds(0,0,0,opMode,0);
        } else if (xDiff != 0) {
            if (xDiff > 0) {
                moveRobotForSeconds(0,.5f,0,opMode,(float)(xDiff*12/39.1527));
            } else if (xDiff < 0) {
                moveRobotForSeconds(0,-.5f,0,opMode,(float)(+xDiff*12/39.1527));
            }
        }
     }

    public void moveTwoRobotToPositionUsingTime(double x, double y, double power, boolean turnToDestination, LinearOpMode opMode) throws InterruptedException {

        double time = 0;
        double distX = x - currentPosition.x;
        double distY = y - currentPosition.y;
        double distance = Math.sqrt(Math.pow(distX,2) + Math.pow(distY,2));
//        double time = Math.log10((distance * 12 + 189.5)/186.27)/Math.log10(1.1499);
        if (distY != 0){
            time = (distance * 12 + 3.65) / 39.073;
        } else if (distY == 0){
            time = distance*12/32.008;
        }

        Position direction = getDirectionFromXAndYDistance(distX,distY);
        direction.x *= power;
        direction.y *= power;

//        accelerateRobot(direction.x,direction.y,power,1,opMode);
        moveRobotForSeconds((float)direction.x,(float)direction.y,0,opMode,time);

        currentPosition = new Position(DistanceUnit.METER,x,y,0,System.currentTimeMillis());
        /*opMode.telemetry.addData("Distance x and y", "( %.2f, %.2f)",distX,distY);
        opMode.telemetry.addData("Distance", distance);
        opMode.telemetry.addData("target",time);*/
    }
}

