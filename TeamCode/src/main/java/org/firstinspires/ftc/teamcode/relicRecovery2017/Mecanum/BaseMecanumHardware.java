package org.firstinspires.ftc.teamcode.relicRecovery2017.Mecanum;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is a hardware class to start testing if kotlin can be used to program as an alternative to java
 *
 * This is a base hardware class for the mecanum drive. Do not put robot specific code in this class.
 * Instead create a robot specific hardware class that inherits this class and put robot specific code in it.
 * This hardware class assumes that the robot only has 4 drive motors without encoders.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Front Left  drive motor:        "f_left_drive"
 * Motor channel:  Front Right drive motor:        "f_right_drive"
 * Motor channel:  Back  Left  drive motor:        "b_left_drive"
 * Motor channel:  Back  Right drive motor:        "b_right_drive"
 */

public class BaseMecanumHardware {

    /* public members */
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;

    /* local members */
    HardwareMap hwMap = null;
    Telemetry telemetry = null;

    public void init(HardwareMap ahwMap, Telemetry telemetry) {
        // Save a reference to Hardware map
        hwMap = ahwMap;

        // Save a reference to Telemetry instance
        this.telemetry = telemetry;

        // Get and initalize the motors
        frontLeftMotor = initMotor("f_left_drive", DcMotorSimple.Direction.REVERSE);
        frontRightMotor = initMotor("f_right_drive", DcMotorSimple.Direction.FORWARD);
        backLeftMotor = initMotor("b_left_drive", DcMotorSimple.Direction.REVERSE);
        backRightMotor = initMotor("b_right_drive", DcMotorSimple.Direction.FORWARD);
    }

    public DcMotor initMotor(String hardwareName, DcMotorSimple.Direction direction) {
        DcMotor motor = hwMap.dcMotor.get(hardwareName);
        if (motor == null) {telemetry.addData("error", hardwareName + "not found"); return null;}
        motor.setDirection(direction);
        motor.setPower(0.0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    public void moveRobot() {moveRobot(0,0,0);}
    public void moveRobot(double xAxis) {moveRobot(xAxis, 0, 0);}
    public void moveRobot(double xAxis, double yAxis) {moveRobot(xAxis, yAxis, 0);}
    public void moveRobot(double xAxis, double yAxis, double rotation) {
        frontLeftMotor.setPower(Range.clip(yAxis + xAxis + rotation, -1, 1));
        frontRightMotor.setPower(Range.clip(yAxis - xAxis - rotation, -1, 1));
        backLeftMotor.setPower(Range.clip(yAxis - xAxis + rotation, -1, 1));
        backRightMotor.setPower(Range.clip(yAxis + xAxis - rotation, -1, 1));
    }
}
