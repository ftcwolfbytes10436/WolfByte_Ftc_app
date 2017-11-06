package org.firstinspires.ftc.teamcode.relicRecovery2017;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BaseTankHardware {

    /* public members */
    public DcMotor LeftMotor = null;
    public DcMotor RightMotor = null;

    public DcMotor LifterMotor = null;

    public Servo leftGripper = null;
    public Servo rightGripper = null;

    public Servo jewelRaise = null;
    public Servo jewelHit = null;

    /* local members */
    HardwareMap hwMap = null;
    Telemetry telemetry = null;
    double gripperPos = 0;
    public double backFeedPower = 0.1;

    public void init(HardwareMap ahwMap, Telemetry telemetry) {
        // Save a reference to Hardware map
        hwMap = ahwMap;

        // Save a reference to Telemetry instance
        this.telemetry = telemetry;

        // Get and initalize the motors
        LeftMotor = initMotor("LeftMotor", DcMotorSimple.Direction.REVERSE);
        RightMotor = initMotor("RightMotor", DcMotorSimple.Direction.FORWARD);

        LifterMotor = initMotor("LifterMotor", DcMotorSimple.Direction.FORWARD);

        leftGripper = initServo("LeftGripper", 0 /*gripperPos*/, Direction.FORWARD);
        rightGripper = initServo("RightGripper", 0 /*gripperPos*/, Direction.REVERSE);

        jewelRaise = initServo("jewelRaise", .85, Direction.FORWARD);
        jewelHit = initServo("jewelHit", 1, Direction.FORWARD);

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

    public void setGripperPower(double power) {
        gripperPos = Range.clip(power + gripperPos, 0, 1);
        leftGripper.setPosition(gripperPos);
        rightGripper.setPosition(gripperPos);
    }

    public void setGriperPos(double pos) {
        gripperPos = pos;
        leftGripper.setPosition(gripperPos);
        rightGripper.setPosition(gripperPos);
    }

    public void moveRobot() {moveRobot(0,0,0);}
    public void moveRobot(double xAxis) {moveRobot(xAxis, 0, 0);}
    public void moveRobot(double xAxis, double yAxis) {moveRobot(xAxis, yAxis, 0);}
    public void moveRobot(double xAxis, double yAxis, double rotation) {
        LeftMotor.setPower(Range.clip(xAxis, -1, 1));
        RightMotor.setPower(Range.clip(yAxis, -1, 1));
        //StrafeMotor.setPower(Range.clip(xAxis, -1, 1));
    }
}
