package org.firstinspires.ftc.teamcode.relicRecovery2017;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name ="teleop")
public class basicTeleOp extends OpMode {

    /* Declare OpMode members. */
    AutonomousHardware robot = new AutonomousHardware();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);

        telemetry.addData("Status", "waiting to start");
        telemetry.update();

        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        double xAxis = gamepad1.left_stick_x/2;
        double yAxis = -gamepad1.left_stick_y;
        double rotation = gamepad1.right_stick_x;

        if (gamepad1.right_bumper)
        {
            xAxis /= 2;
            rotation /= 2;
        }

        robot.moveRobot(xAxis, yAxis, rotation);

        telemetry.addData("count",robot.frontLeftMotor.getCurrentPosition());
        telemetry.addData("count",robot.frontRightMotor.getCurrentPosition());
        telemetry.addData("count",robot.backLeftMotor.getCurrentPosition());
        telemetry.addData("count",robot.backRightMotor.getCurrentPosition());
    }
}
