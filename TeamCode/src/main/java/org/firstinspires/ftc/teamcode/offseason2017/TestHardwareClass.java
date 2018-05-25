package org.firstinspires.ftc.teamcode.offseason2017;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Andrew Brown on 4/16/18.
 */

public class TestHardwareClass {

    //Public Members
    //Usage: public DcMotor MotorName = null;
    //Usage: public Servo ServoName = null;

    public DcMotor LeftMotor = null;
    public DcMotor RightMotor = null;

    HardwareMap hwMap = null;
    Telemetry telemetry = null;

    public void init(HardwareMap ahwMap, Telemetry telemetry) {
        // Save a reference to Hardware map
        hwMap = ahwMap;

        // Save a reference to Telemetry instance
        this.telemetry = telemetry;

        // Get and initalize the motors
        LeftMotor = initMotor("LeftMotor", DcMotorSimple.Direction.FORWARD);
        RightMotor = initMotor("RightMotor", DcMotorSimple.Direction.FORWARD);
    }

    public DcMotor initMotor(String hardwareName, DcMotorSimple.Direction direction) {
        DcMotor motor = hwMap.dcMotor.get(hardwareName);
        if (motor == null) {telemetry.addData("error", hardwareName + "not found"); return null;}
        motor.setDirection(direction);
        motor.setPower(0.0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    public Servo initServo(String hardwareName, double initalPos, Direction direction) {
        Servo servo = hwMap.servo.get(hardwareName);
        if (servo == null) {telemetry.addData("error", hardwareName + "not found"); return null;}
        servo.setDirection(direction);
        servo.setPosition(initalPos);
        return servo;
    }

    public void moveRobot(double leftMotor, double rightMotor)
    {
        LeftMotor.setPower(Range.clip(leftMotor, -1, 1));
        RightMotor.setPower(Range.clip(rightMotor, -1, 1));
    }

}
