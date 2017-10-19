package org.firstinspires.ftc.teamcode.relicRecovery2017.Test_Scripts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name ="tank_teleop")

public class BaseTankTeleop extends OpMode{

    /* Declare OpMode members. */
    BaseTankHardware robot = new BaseTankHardware();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);

        telemetry.addData("Status", "waiting to start");
        telemetry.update();

        robot.LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        robot.LeftMotor.setPower(Range.clip(gamepad1.left_stick_y, -1, 1));
        robot.RightMotor.setPower(Range.clip(gamepad1.right_stick_y, -1, 1));

        float gamepad2RightX = -gamepad2.right_stick_x;
        double GripperPosition = Range.clip(robot.Gripper.getPosition() + gamepad2RightX * 0.05, -1, 1);

        robot.Gripper.setPosition(GripperPosition);
        telemetry.addData("Gripper Value: ", robot.Gripper.getPosition());

        telemetry.addData("count",robot.LeftMotor.getCurrentPosition());
        telemetry.addData("count",robot.RightMotor.getCurrentPosition());
    }
}

