package org.firstinspires.ftc.teamcode.relicRecovery2017.Test_Scripts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BaseTankHardware {

    /* public members */
    public DcMotor LeftMotor = null;
    public DcMotor RightMotor = null;
    //public DcMotor StrafeMotor = null;
    public Servo Gripper = null;

    /* local members */
    HardwareMap hwMap = null;
    Telemetry telemetry = null;

    public void init(HardwareMap ahwMap, Telemetry telemetry) {
        // Save a reference to Hardware map
        hwMap = ahwMap;

        // Save a reference to Telemetry instance
        this.telemetry = telemetry;

        // Get and initalize the motors
        LeftMotor = initMotor("LeftMotor", DcMotorSimple.Direction.REVERSE);
        RightMotor = initMotor("RightMotor", DcMotorSimple.Direction.FORWARD);
        //StrafeMotor = initMotor("strafe_drive", DcMotorSimple.Direction.FORWARD);
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
        LeftMotor.setPower(Range.clip(yAxis, -1, 1));
        RightMotor.setPower(Range.clip(yAxis, -1, 1));
        //StrafeMotor.setPower(Range.clip(xAxis, -1, 1));
    }
}
